"""Tests for the ROS 2 adapter module (robotic_harness_worker.ros).

Covers: rosbag2 inspection (repo fixture + tmp_path bags), the rosbag-backed
branches of ros-tf-audit / ros-diagnostics-snapshot, whitelisted action safety,
the C:-drive guard of rosbag-start, and the adapter pattern (live-ROS commands
report backend "unavailable" when the ros2 CLI is missing, or are exercised
with canned ros2 output through monkeypatched adapters).
"""

from __future__ import annotations

import json
import os
import sqlite3

import pytest

import robotic_harness_worker.ros as ros
from robotic_harness_worker.core import WorkerError

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEMO_BAG_DIR = os.path.join(_REPO_ROOT, "fixtures", "rosbags", "demo_rosbag")
DEMO_BAG_DB = os.path.join(DEMO_BAG_DIR, "demo_rosbag.db3")

START_NS = 1_000_000_000_000_000_000
STEP_NS = 100_000_000  # 100 ms


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _build_rosbag(bag_dir, topics, messages):
    """Create a minimal rosbag2 bag (directory with metadata.yaml + bag.db3).

    ``topics``: list of (name, type, serialization_format).
    ``messages``: dict topic-name -> list of bytes.
    """
    bag_dir.mkdir(parents=True, exist_ok=True)
    db_path = bag_dir / "bag.db3"
    conn = sqlite3.connect(db_path)
    conn.executescript(
        """
        CREATE TABLE schema(version INTEGER NOT NULL, rosbag2_version TEXT NOT NULL);
        CREATE TABLE topics(
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL
        );
        CREATE TABLE messages(
            id INTEGER PRIMARY KEY,
            topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            data BLOB NOT NULL
        );
        """
    )
    conn.execute("INSERT INTO schema(version, rosbag2_version) VALUES (4, '4.0.0')")
    topic_ids = {}
    for index, (name, type_, fmt) in enumerate(topics, start=1):
        conn.execute(
            "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?, ?)",
            (index, name, type_, fmt, "- history: 3\n  depth: 0\n"),
        )
        topic_ids[name] = index
    message_id = 1
    total = 0
    for name, blobs in messages.items():
        for index, blob in enumerate(blobs):
            conn.execute(
                "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?, ?, ?, ?)",
                (message_id, topic_ids[name], START_NS + index * STEP_NS, blob),
            )
            message_id += 1
            total += 1
    conn.commit()
    conn.close()
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 4,
            "storage_identifier": "sqlite3",
            "relative_file_paths": ["bag.db3"],
            "message_count": total,
        }
    }
    (bag_dir / "metadata.yaml").write_text(json.dumps(metadata), encoding="utf-8")
    return bag_dir


class _FakeProc:
    returncode = 0


# ---------------------------------------------------------------------------
# rosbag-inspect
# ---------------------------------------------------------------------------


def test_rosbag_inspect_fixture_directory():
    result = ros.inspect_rosbag2(DEMO_BAG_DIR)
    assert result["ok"] is True
    assert result["format"] == "rosbag2"
    assert result["version"] == 4
    assert result["messageCount"] == 40
    assert result["topicCount"] == 2
    assert result["durationS"] == pytest.approx(1.9, abs=0.01)
    by_name = {topic["name"]: topic for topic in result["topics"]}
    assert set(by_name) == {"/joint_states", "/signal"}

    joint_states = by_name["/joint_states"]
    assert joint_states["type"] == "sensor_msgs/msg/JointState"
    assert joint_states["serializationFormat"] == "cdr"
    assert joint_states["count"] == 20
    assert joint_states["decoded"] is False
    assert joint_states["decodeSummary"]["unsupported"] is True

    signal = by_name["/signal"]
    assert signal["type"] == "std_msgs/msg/Float64"
    assert signal["count"] == 20
    assert signal["decoded"] is True
    assert signal["decodeSummary"]["firstValue"] == pytest.approx(0.5)
    assert signal["decodeSummary"]["samples"] >= 1
    assert signal["minStampS"] == pytest.approx(1e9, rel=1e-9)
    assert signal["maxStampS"] == pytest.approx(1e9 + 1.9, rel=1e-9)
    assert signal["avgSizeBytes"] == pytest.approx(12.0)
    assert signal["minSizeBytes"] == 12

    # unsupported types must be listed explicitly, never silently dropped
    codes = {issue["code"] for issue in result["issues"]}
    assert "decode.unsupported" in codes
    unsupported = [i for i in result["issues"] if i["code"] == "decode.unsupported"]
    assert any("sensor_msgs/msg/JointState" in i["message"] for i in unsupported)


def test_rosbag_inspect_fixture_db3_file():
    result = ros.inspect_rosbag2(DEMO_BAG_DB)
    assert result["ok"] is True
    assert result["format"] == "rosbag2"
    assert result["version"] == 4
    assert result["messageCount"] == 40
    assert result["dbPath"].endswith("demo_rosbag.db3")


def test_rosbag_inspect_rejects_missing_path(tmp_path):
    with pytest.raises(WorkerError, match="does not exist"):
        ros.inspect_rosbag2(str(tmp_path / "nope"))


def test_rosbag_inspect_rejects_non_rosbag_file(tmp_path):
    text_file = tmp_path / "fake.db3"
    text_file.write_text("this is not a sqlite database", encoding="utf-8")
    with pytest.raises(WorkerError, match="not a valid rosbag2 sqlite database"):
        ros.inspect_rosbag2(str(text_file))


def test_rosbag_inspect_rejects_directory_without_metadata(tmp_path):
    empty_dir = tmp_path / "empty"
    empty_dir.mkdir()
    with pytest.raises(WorkerError, match="no metadata.yaml"):
        ros.inspect_rosbag2(str(empty_dir))


def test_rosbag_inspect_rejects_sqlite_without_rosbag_tables(tmp_path):
    db_path = tmp_path / "not_a_bag.db3"
    conn = sqlite3.connect(db_path)
    conn.execute("CREATE TABLE foo(bar TEXT)")
    conn.commit()
    conn.close()
    with pytest.raises(WorkerError, match="not a rosbag2 sqlite bag"):
        ros.inspect_rosbag2(str(db_path))


# ---------------------------------------------------------------------------
# ros-tf-audit and ros-diagnostics-snapshot (rosbag branches)
# ---------------------------------------------------------------------------


def test_ros_tf_audit_rosbag_branch(tmp_path):
    tf_message = ros.encode_cdr_tf_message(
        [
            {
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0, 1.0],
            },
            {
                "frame_id": "base_link",
                "child_frame_id": "tool0",
                "translation": [0.1, 0.2, 0.3],
                "rotation": [0.0, 0.0, 0.0, 1.0],
            },
        ]
    )
    tf_static_message = ros.encode_cdr_tf_message(
        [
            {
                "frame_id": "map",
                "child_frame_id": "odom",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0, 1.0],
            }
        ]
    )
    bag = _build_rosbag(
        tmp_path / "tf_bag",
        [
            ("/tf", "tf2_msgs/msg/TFMessage", "cdr"),
            ("/tf_static", "tf2_msgs/msg/TFMessage", "cdr"),
        ],
        {"/tf": [tf_message] * 10, "/tf_static": [tf_static_message] * 3},
    )
    result = ros.cmd_ros_tf_audit({"rosbagPath": str(bag)})
    assert result["ok"] is True
    assert result["backend"] == "rosbag"
    assert "tool0" in result["frames"]
    assert "odom" in result["frames"]
    assert "map" in result["frames"]
    assert result["tfMessageCount"] == 10
    assert result["tfStaticMessageCount"] == 3
    # 10 /tf messages 100 ms apart -> (10-1)/0.9 = 10 Hz
    assert result["tfRateHz"] == pytest.approx(10.0, abs=0.5)
    assert result["timeRangeS"]["start"] is not None
    assert result["timeRangeS"]["end"] is not None


def test_ros_diagnostics_snapshot_rosbag_branch(tmp_path):
    diag_message = ros.encode_cdr_diagnostic_array(
        [
            {
                "level": 0,
                "name": "motor",
                "message": "nominal",
                "hardware_id": "m1",
                "values": [{"key": "temperature", "value": "40.0"}],
            },
            {
                "level": 2,
                "name": "battery",
                "message": "low voltage",
                "hardware_id": "b1",
                "values": [],
            },
        ]
    )
    bag = _build_rosbag(
        tmp_path / "diag_bag",
        [("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", "cdr")],
        {"/diagnostics": [diag_message] * 3},
    )
    result = ros.cmd_ros_diagnostics_snapshot({"rosbagPath": str(bag)})
    assert result["ok"] is True
    assert result["backend"] == "rosbag"
    assert result["messageCount"] == 3
    assert result["errorCount"] == 3  # 3 messages x 1 ERROR status each
    assert result["warningCount"] == 0
    assert result["staleCount"] == 0
    levels = {status["level"] for status in result["statuses"]}
    assert levels == {0, 2}
    names = {status["name"] for status in result["statuses"]}
    assert names == {"motor", "battery"}
    motor = next(s for s in result["statuses"] if s["name"] == "motor")
    assert motor["levelName"] == "OK"
    assert motor["hardwareId"] == "m1"


# ---------------------------------------------------------------------------
# rosbag-start / rosbag-stop
# ---------------------------------------------------------------------------


@pytest.mark.skipif(os.name != "nt", reason="C:-drive guard is Windows-only")
def test_rosbag_start_rejects_c_drive():
    with pytest.raises(WorkerError, match="C:"):
        ros.cmd_rosbag_start({"bagPath": "C:\\tmp\\rosbag"})
    with pytest.raises(WorkerError, match="C:"):
        ros.cmd_rosbag_start({"bagPath": "c:/tmp/rosbag"})


def test_rosbag_start_requires_bag_path():
    with pytest.raises(WorkerError, match="bagPath"):
        ros.cmd_rosbag_start({})


def test_rosbag_start_reports_unavailable(monkeypatch, tmp_path):
    monkeypatch.setattr(ros, "_ros2_available", lambda: False)
    result = ros.cmd_rosbag_start({"bagPath": str(tmp_path / "out" / "bag")})
    assert result["ok"] is True
    assert result["backend"] == "unavailable"
    assert result["instructions"]


def test_rosbag_start_spawns_and_tracks_job(monkeypatch, tmp_path):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)

    class FakePopen:
        def __init__(self, argv, **kwargs):
            self.pid = 4242
            self.argv = argv

    monkeypatch.setattr(ros.subprocess, "Popen", FakePopen)
    bag = str(tmp_path / "rec" / "bag")
    result = ros.cmd_rosbag_start(
        {"bagPath": bag, "topics": ["/joint_states", "/signal"], "storeRoot": str(tmp_path)}
    )
    assert result["ok"] is True
    assert result["backend"] == "ros2"
    assert result["pid"] == 4242
    assert result["bagPath"] == bag
    assert result["topics"] == ["/joint_states", "/signal"]
    jobs = json.loads((tmp_path / ".rh" / "rosbag-jobs.json").read_text(encoding="utf-8"))
    assert len(jobs) == 1
    assert jobs[0]["jobId"] == result["jobId"]
    assert jobs[0]["pid"] == 4242


def test_rosbag_stop_unknown_job(tmp_path):
    with pytest.raises(WorkerError, match="unknown rosbag job"):
        ros.cmd_rosbag_stop({"jobId": "rosbag-unknown", "storeRoot": str(tmp_path)})


def test_rosbag_stop_removes_job(monkeypatch, tmp_path):
    state_file = tmp_path / ".rh" / "rosbag-jobs.json"
    state_file.parent.mkdir(parents=True)
    state_file.write_text(
        json.dumps([{"jobId": "rosbag-abc123", "pid": 99999, "bagPath": str(tmp_path / "bag")}]),
        encoding="utf-8",
    )
    monkeypatch.setattr(ros.subprocess, "run", lambda *a, **k: _FakeProc())
    result = ros.cmd_rosbag_stop({"jobId": "rosbag-abc123", "storeRoot": str(tmp_path)})
    assert result["ok"] is True
    assert result["stopped"] is True
    assert result["bagPath"] == str(tmp_path / "bag")
    remaining = json.loads(state_file.read_text(encoding="utf-8"))
    assert remaining == []


# ---------------------------------------------------------------------------
# ros-call-whitelisted-action
# ---------------------------------------------------------------------------


def test_ros_call_whitelisted_action_no_allowlist(tmp_path):
    with pytest.raises(WorkerError, match="no allowlist"):
        ros.cmd_ros_call_whitelisted_action(
            {"action": "nav/go_to_pose", "goal": {"x": 1.0}, "storeRoot": str(tmp_path)}
        )


def test_ros_call_whitelisted_action_not_in_allowlist():
    with pytest.raises(WorkerError, match="not in allowlist"):
        ros.cmd_ros_call_whitelisted_action(
            {
                "action": "evil/do_thing",
                "goal": {},
                "allowlist": [{"action": "nav/go_to_pose"}],
            }
        )


def test_ros_call_whitelisted_action_fields_guard():
    with pytest.raises(WorkerError, match="not allowed"):
        ros.cmd_ros_call_whitelisted_action(
            {
                "action": "nav/go_to_pose",
                "goal": {"x": 1.0, "unexpected": 2},
                "allowlist": [{"action": "nav/go_to_pose", "fields": ["x"]}],
            }
        )


def test_ros_call_whitelisted_action_backend_unavailable(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: False)
    result = ros.cmd_ros_call_whitelisted_action(
        {
            "action": "nav/go_to_pose",
            "goal": {"x": 1.0},
            "allowlist": [{"action": "nav/go_to_pose"}],
        }
    )
    assert result["ok"] is True
    assert result["backend"] == "unavailable"
    assert result["instructions"]


def test_ros_call_whitelisted_action_sends_goal(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    calls = []

    def fake_run(argv, timeout=10.0, ros_domain=None):
        calls.append(list(argv))
        return "Goal accepted with ID: abc123"

    monkeypatch.setattr(ros, "_run_ros2", fake_run)
    result = ros.cmd_ros_call_whitelisted_action(
        {
            "action": "nav/go_to_pose",
            "goal": {"x": 1.0},
            "allowlist": [{"action": "nav/go_to_pose", "type": "nav2_msgs/action/NavigateToPose"}],
        }
    )
    assert result["ok"] is True
    assert result["backend"] == "ros2"
    assert result["sent"] is True
    assert calls and calls[0][0] == "action" and calls[0][1] == "send_goal"


# ---------------------------------------------------------------------------
# live-ROS commands: adapter pattern
# ---------------------------------------------------------------------------


def test_live_commands_report_unavailable_when_no_ros2(monkeypatch, tmp_path):
    monkeypatch.setattr(ros, "_ros2_available", lambda: False)
    cases = {
        "ros_graph_snapshot": (ros.cmd_ros_graph_snapshot, {}),
        "ros_topic_profile": (ros.cmd_ros_topic_profile, {"topic": "/cmd_vel"}),
        "ros_qos_check": (ros.cmd_ros_qos_check, {"topic": "/cmd_vel"}),
        "ros_tf_audit": (ros.cmd_ros_tf_audit, {}),
        "ros_diagnostics_snapshot": (ros.cmd_ros_diagnostics_snapshot, {}),
        "ros_controller_status": (ros.cmd_ros_controller_status, {}),
        "ros_moveit_audit": (ros.cmd_ros_moveit_audit, {}),
        "rosbag_start": (ros.cmd_rosbag_start, {"bagPath": str(tmp_path / "bag")}),
        "ros_call_whitelisted_action": (
            ros.cmd_ros_call_whitelisted_action,
            {"action": "nav/go", "goal": {}, "allowlist": [{"action": "nav/go"}]},
        ),
    }
    for name, (command, args) in cases.items():
        result = command(args)
        assert result["ok"] is True, name
        assert result["backend"] == "unavailable", name
        assert result["reason"], name
        assert result["instructions"], name


def test_ros_graph_snapshot_environment_dependent_structure():
    result = ros.cmd_ros_graph_snapshot({})
    assert result["ok"] is True
    assert result["backend"] in ("unavailable", "ros2")
    if result["backend"] == "unavailable":
        assert "reason" in result and "instructions" in result
    else:
        for key in ("nodes", "topics", "services", "actions"):
            assert key in result and isinstance(result[key], list)


# ---------------------------------------------------------------------------
# live-ROS commands: parsers exercised with canned ros2 output
# ---------------------------------------------------------------------------

CANNED_TOPIC_INFO = """\
Type: std_msgs/msg/String

Publisher count: 1

Node name: /talker
Node namespace: /
Topic type: std_msgs/msg/String
QoS profile:
  Reliability: Reliable
  History: Keep Last
  Depth: 10
  Durability: Volatile

Subscription count: 1

Node name: /listener
Node namespace: /
Topic type: std_msgs/msg/String
QoS profile:
  Reliability: Best Effort
  History: Keep Last
  Depth: 10
  Durability: Volatile
"""

CANNED_TOPIC_INFO_COMPATIBLE = CANNED_TOPIC_INFO.replace(
    "Reliability: Best Effort", "Reliability: Reliable"
)

CANNED_GRAPH = {
    ("node", "list"): "/talker\n/listener\n",
    ("topic", "list"): "/chatter [std_msgs/msg/String]\n/tf [tf2_msgs/msg/TFMessage]\n",
    ("service", "list"): "/add_two_ints [example_interfaces/srv/AddTwoInts]\n",
    ("action", "list"): "/go_to_pose [nav2_msgs/action/NavigateToPose]\n",
}


def _fake_graph_run(argv, timeout=10.0, ros_domain=None):
    key = tuple(argv[:2])
    if key in CANNED_GRAPH:
        return CANNED_GRAPH[key]
    if argv[0] == "topic" and argv[1] == "info":
        return CANNED_TOPIC_INFO
    return ""


def test_ros_graph_snapshot_parses_ros2_output(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(ros, "_run_ros2", _fake_graph_run)
    result = ros.cmd_ros_graph_snapshot({})
    assert result["backend"] == "ros2"
    assert result["nodes"] == ["/talker", "/listener"]
    assert result["topics"][0]["name"] == "/chatter"
    assert result["topics"][0]["type"] == "std_msgs/msg/String"
    assert result["topics"][0]["publishers"] == 1
    assert result["topics"][0]["subscribers"] == 1
    assert result["services"][0]["type"] == "example_interfaces/srv/AddTwoInts"
    assert result["actions"][0]["name"] == "/go_to_pose"
    assert result["truncated"] is False


def test_ros_topic_profile_measures_rate(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros,
        "measure_topic_hz",
        lambda topic, duration_s, window=50, ros_domain=None: {
            "measuredRateHz": 10.0,
            "window": 50,
            "samples": 50,
            "minIntervalS": 0.09,
            "maxIntervalS": 0.11,
            "stdDevS": 0.002,
        },
    )
    result = ros.cmd_ros_topic_profile({"topic": "/cmd_vel"})
    assert result["backend"] == "ros2"
    assert result["measuredRateHz"] == pytest.approx(10.0)
    assert result["window"] == 50
    assert result["issues"] == []


def test_ros_topic_profile_flags_below_expected_rate(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros,
        "measure_topic_hz",
        lambda topic, duration_s, window=50, ros_domain=None: {
            "measuredRateHz": 1.0,
            "window": 50,
            "samples": 50,
        },
    )
    result = ros.cmd_ros_topic_profile({"topic": "/cmd_vel", "rate": 5.0})
    assert result["measuredRateHz"] == pytest.approx(1.0)
    codes = {issue["code"] for issue in result["issues"]}
    assert "rate.below_expected" in codes


def test_ros_topic_profile_flags_zero_rate(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros,
        "measure_topic_hz",
        lambda topic, duration_s, window=50, ros_domain=None: {
            "measuredRateHz": None,
            "window": 50,
            "samples": 0,
        },
    )
    result = ros.cmd_ros_topic_profile({"topic": "/cmd_vel"})
    assert result["measuredRateHz"] == 0.0
    codes = {issue["code"] for issue in result["issues"]}
    assert "rate.zero" in codes


def test_ros_qos_check_incompatible_reliability(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(ros, "_run_ros2", lambda argv, timeout=10.0, ros_domain=None: CANNED_TOPIC_INFO)
    result = ros.cmd_ros_qos_check({"topic": "/chatter"})
    assert result["backend"] == "ros2"
    assert result["qos"]["publisher"]["reliability"] == "Reliable"
    assert result["qos"]["subscriber"]["reliability"] == "Best Effort"
    assert result["compatible"] is False
    codes = {issue["code"] for issue in result["issues"]}
    assert "qos.reliability_mismatch" in codes


def test_ros_qos_check_compatible(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros, "_run_ros2", lambda argv, timeout=10.0, ros_domain=None: CANNED_TOPIC_INFO_COMPATIBLE
    )
    result = ros.cmd_ros_qos_check({"topic": "/chatter"})
    assert result["compatible"] is True
    assert result["issues"] == []


TF_STATIC_ECHO = """\
header:
  stamp:
    sec: 100
    nanosec: 0
  frame_id: odom
transforms:
- header:
    stamp:
      sec: 100
      nanosec: 0
    frame_id: odom
  child_frame_id: base_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"""


def test_ros_tf_audit_ros2_branch(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)

    def fake_run(argv, timeout=10.0, ros_domain=None):
        if argv[:2] == ["topic", "echo"]:
            return TF_STATIC_ECHO
        return ""

    monkeypatch.setattr(ros, "_run_ros2", fake_run)
    monkeypatch.setattr(
        ros,
        "measure_topic_hz",
        lambda topic, duration_s, window=20, ros_domain=None: {
            "measuredRateHz": 30.0,
            "window": 20,
            "samples": 20,
        },
    )
    result = ros.cmd_ros_tf_audit({})
    assert result["backend"] == "ros2"
    assert "base_link" in result["frames"]
    assert "odom" in result["frames"]
    assert result["tfRateHz"] == pytest.approx(30.0)
    assert result["issues"] == []


DIAGNOSTICS_ECHO = """\
header:
  stamp:
    sec: 100
    nanosec: 0
  frame_id: ""
status:
- level: 0
  name: motor
  message: nominal
  hardware_id: m1
  values: []
- level: 1
  name: battery
  message: low
  hardware_id: b1
  values: []
"""


def test_ros_diagnostics_snapshot_ros2_branch(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros,
        "_run_ros2",
        lambda argv, timeout=10.0, ros_domain=None: DIAGNOSTICS_ECHO,
    )
    result = ros.cmd_ros_diagnostics_snapshot({})
    assert result["backend"] == "ros2"
    assert result["warningCount"] == 1
    assert result["errorCount"] == 0
    levels = {status["level"] for status in result["statuses"]}
    assert levels == {0, 1}


def test_ros_controller_status_parses(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros,
        "_run_ros2",
        lambda argv, timeout=10.0, ros_domain=None: (
            "joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active\n"
            "  - joint1\n"
            "  - joint2\n"
            "arm_controller joint_trajectory_controller active\n"
        ),
    )
    result = ros.cmd_ros_controller_status({})
    assert result["backend"] == "ros2"
    assert result["controllers"][0]["name"] == "joint_state_broadcaster"
    assert result["controllers"][0]["claimedInterfaces"] == ["joint1", "joint2"]
    assert result["controllers"][1]["name"] == "arm_controller"
    assert result["controllers"][1]["state"] == "active"


def test_ros_controller_status_filters_by_name(monkeypatch):
    monkeypatch.setattr(ros, "_ros2_available", lambda: True)
    monkeypatch.setattr(
        ros,
        "_run_ros2",
        lambda argv, timeout=10.0, ros_domain=None: "a_controller diff_drive_controller active\nb_controller joint_trajectory_controller active\n",
    )
    result = ros.cmd_ros_controller_status({"controllerNames": ["b_controller"]})
    assert [c["name"] for c in result["controllers"]] == ["b_controller"]


def test_ros_moveit_audit_srdf(tmp_path):
    srdf = tmp_path / "robot.srdf"
    srdf.write_text(
        """<robot name="rh_arm">
  <group name="arm_group">
    <joint name="joint1"/>
    <joint name="joint2"/>
  </group>
  <group name="gripper_group">
    <joint name="gripper_joint"/>
  </group>
  <end_effector name="gripper_ee" parent_link="tool0" group="gripper_group"/>
</robot>""",
        encoding="utf-8",
    )
    result = ros.cmd_ros_moveit_audit({"configPath": str(srdf)})
    assert result["ok"] is True
    assert result["backend"] == "config"
    groups = {group["name"]: group for group in result["groups"]}
    assert groups["arm_group"]["joints"] == ["joint1", "joint2"]
    assert groups["gripper_group"]["joints"] == ["gripper_joint"]
    assert groups["gripper_group"]["endEffector"] == "gripper_ee"
    assert result["endEffectors"][0]["parentLink"] == "tool0"


def test_ros_moveit_audit_missing_config_file(tmp_path):
    with pytest.raises(WorkerError, match="not found"):
        ros.cmd_ros_moveit_audit({"configPath": str(tmp_path / "nope.srdf")})


def test_ros_moveit_audit_group_filter(tmp_path):
    srdf = tmp_path / "robot.srdf"
    srdf.write_text(
        '<robot name="r"><group name="a"><joint name="j1"/></group><group name="b"><joint name="j2"/></group></robot>',
        encoding="utf-8",
    )
    result = ros.cmd_ros_moveit_audit({"configPath": str(srdf), "group": "b"})
    assert [group["name"] for group in result["groups"]] == ["b"]
