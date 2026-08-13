"""Tests for the real-robot experiment state machine and preflight module
(robots.py), per docs/worker-module-contract.md.

Covers: honest preflight reporting (hardware skip / file pass-fail / verdict),
the full persisted state machine (prepare -> request-approval -> start ->
pause -> safe-cancel -> finalize), approvalRef enforcement, and illegal
transitions raising WorkerError.
"""

from __future__ import annotations

import json
import os

import pytest

from robotic_harness_worker import robots
from robotic_harness_worker.core import RunStore, Run, WorkerError


def _store(tmp_path):
    return str(tmp_path / "rh")


# ---------------------------------------------------------------------------
# robot-preflight
# ---------------------------------------------------------------------------


def test_preflight_generates_checklist_with_honest_hardware_status(tmp_path):
    result = robots.cmd_robot_preflight(
        {
            "robotModel": "RH-2000",
            "safetyLimits": {"maxVelocity": 0.5, "maxForce": 100.0},
            "storeRoot": _store(tmp_path),
        }
    )
    assert result["ok"] is True
    assert result["preflightId"].startswith("preflight-")
    checks = {c["id"]: c for c in result["checks"]}

    # real-hardware items are honestly skipped without an adapter
    assert checks["estop.released"]["status"] == "skip"
    assert "无真机适配器" in checks["estop.released"]["reason"]
    assert checks["controller.mode"]["status"] == "skip"
    assert checks["joint.state"]["status"] == "skip"

    # config / file items are really checked
    assert checks["robot.model"]["status"] == "pass"
    assert checks["limits.velocity_force"]["status"] == "pass"
    assert checks["recording.ready"]["status"] == "pass"
    # approval cannot be validated without an experimentId
    assert checks["approval.valid"]["status"] == "not-checked"

    assert result["passCount"] >= 2
    assert result["skipCount"] >= 5
    assert result["failCount"] == 0
    assert result["verdict"] == "incomplete"  # approval not-checked blocks "ready"
    assert "不构成功能安全证明" in result["note"]


def test_preflight_fails_when_limits_missing(tmp_path):
    result = robots.cmd_robot_preflight({"robotModel": "RH-2000", "storeRoot": _store(tmp_path)})
    checks = {c["id"]: c for c in result["checks"]}
    assert checks["limits.velocity_force"]["status"] == "fail"
    assert "速度/力限制" in checks["limits.velocity_force"]["reason"]
    assert result["failCount"] >= 1
    assert result["verdict"] == "not-ready"


def test_preflight_file_check_fails_for_missing_calibration(tmp_path):
    result = robots.cmd_robot_preflight(
        {
            "robotModel": "RH-2000",
            "safetyLimits": {"maxVelocity": 0.5},
            "cameraCalibrationPath": str(tmp_path / "missing-calib.yaml"),
            "storeRoot": _store(tmp_path),
        }
    )
    checks = {c["id"]: c for c in result["checks"]}
    assert checks["camera.calibration"]["status"] == "fail"
    assert checks["camera.calibration"]["evidence"]["exists"] is False
    assert result["verdict"] == "not-ready"


def test_preflight_auto_run_false_only_generates(tmp_path):
    result = robots.cmd_robot_preflight(
        {"robotModel": "RH-2000", "autoRun": False, "storeRoot": _store(tmp_path)}
    )
    assert result["ok"] is True
    assert result["autoRun"] is False
    assert all(c["status"] == "not-checked" for c in result["checks"])
    assert result["passCount"] == 0
    assert result["skipCount"] == 0
    assert result["failCount"] == 0
    assert result["verdict"] == "incomplete"


def test_preflight_ready_with_experiment(tmp_path):
    store_root = _store(tmp_path)
    calib = tmp_path / "calib.yaml"
    calib.write_text("version: 2.1\n", encoding="utf-8")
    prepared = robots.cmd_experiment_prepare(
        {
            "name": "pick-demo",
            "robotModel": "RH-2000",
            "safetyLimits": {"maxVelocity": 0.5},
            "storeRoot": store_root,
        }
    )
    exp_id = prepared["experimentId"]
    robots.cmd_experiment_request_approval({"experimentId": exp_id, "storeRoot": store_root})

    result = robots.cmd_robot_preflight(
        {"experimentId": exp_id, "cameraCalibrationPath": str(calib), "storeRoot": store_root}
    )
    checks = {c["id"]: c for c in result["checks"]}
    assert checks["robot.model"]["status"] == "pass"  # model from the record
    assert checks["limits.velocity_force"]["status"] == "pass"  # limits from the record
    assert checks["camera.calibration"]["status"] == "pass"
    assert checks["approval.valid"]["status"] == "pass"
    assert checks["recording.ready"]["status"] == "pass"
    assert result["failCount"] == 0
    assert result["verdict"] == "ready"

    # preflight summary is persisted on the experiment record
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": store_root})
    record = json.loads(open(status["recordPath"], encoding="utf-8").read())
    assert record["preflight"]["verdict"] == "ready"
    assert record["preflight"]["preflightId"] == result["preflightId"]


def test_preflight_requires_identity(tmp_path):
    with pytest.raises(WorkerError):
        robots.cmd_robot_preflight({"storeRoot": _store(tmp_path)})


# ---------------------------------------------------------------------------
# robot-state-snapshot
# ---------------------------------------------------------------------------


def test_robot_state_snapshot_from_experiment(tmp_path):
    store_root = _store(tmp_path)
    prepared = robots.cmd_experiment_prepare({"name": "snap", "robotModel": "RH-2000", "storeRoot": store_root})
    result = robots.cmd_robot_state_snapshot({"experimentId": prepared["experimentId"], "storeRoot": store_root})
    assert result["ok"] is True
    snap = result["snapshot"]
    assert snap["at"]
    assert snap["robotModel"] == "RH-2000"
    assert snap["lastRunId"] is None  # no runs in the store yet
    assert snap["source"] == "store"
    assert snap["issues"]


def test_robot_state_snapshot_no_data(tmp_path):
    result = robots.cmd_robot_state_snapshot({"storeRoot": _store(tmp_path)})
    snap = result["snapshot"]
    assert snap["lastRunId"] is None
    assert snap["robotModel"] is None
    assert snap["jointState"] is None
    assert "无可用数据" in result["note"]


def test_robot_state_snapshot_with_run(tmp_path):
    store_root = _store(tmp_path)
    store = RunStore(store_root)
    store.ensure()
    run = Run(
        id="run-1234abcd",
        project_id="p1",
        scenario="mujoco_pick_place",
        state="completed",
        metrics={"success": True, "steps": 10, "durationS": 1.0},
    )
    store.save_run(run)
    store.append_telemetry(
        run.id,
        [
            {"t": 0.0, "q": [0.0, 0.0, 0.0], "phase": "home"},
            {"t": 0.1, "q": [0.5, 0.3, 0.2], "phase": "reach"},
        ],
    )
    result = robots.cmd_robot_state_snapshot({"storeRoot": store_root})
    snap = result["snapshot"]
    assert snap["lastRunId"] == "run-1234abcd"
    assert snap["lastRunSuccess"] is True
    assert snap["jointState"]["q"] == [0.5, 0.3, 0.2]
    assert snap["jointState"]["phase"] == "reach"


def test_robot_state_snapshot_unknown_experiment(tmp_path):
    with pytest.raises(WorkerError, match="experiment not found"):
        robots.cmd_robot_state_snapshot({"experimentId": "nope", "storeRoot": _store(tmp_path)})


# ---------------------------------------------------------------------------
# experiment state machine
# ---------------------------------------------------------------------------


def _prepare(tmp_path, **overrides):
    args = {"name": "flow", "scenario": "mujoco_pick_place", "storeRoot": _store(tmp_path)}
    args.update(overrides)
    return robots.cmd_experiment_prepare(args)


def test_state_machine_full_flow(tmp_path):
    prepared = _prepare(tmp_path)
    exp_id = prepared["experimentId"]
    assert prepared["state"] == "VALIDATING"
    assert os.path.isfile(prepared["recordPath"])

    # DRAFT -> VALIDATING is recorded in the history
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": _store(tmp_path)})
    assert status["state"] == "VALIDATING"
    assert [h["state"] for h in status["history"]] == ["DRAFT", "VALIDATING"]

    # start without approvalRef -> refused, state unchanged
    with pytest.raises(WorkerError, match="approvalRef"):
        robots.cmd_experiment_start({"experimentId": exp_id, "storeRoot": _store(tmp_path)})
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": _store(tmp_path)})
    assert status["state"] == "VALIDATING"

    # request approval
    approved = robots.cmd_experiment_request_approval(
        {"experimentId": exp_id, "operator": "alice", "storeRoot": _store(tmp_path)}
    )
    assert approved["state"] == "READY_FOR_APPROVAL"
    assert approved["requiresHuman"] is True
    assert "人工" in approved["note"]

    # start with a human approvalRef
    started = robots.cmd_experiment_start(
        {"experimentId": exp_id, "approver": "alice", "approvalRef": "APPR-2025-001", "storeRoot": _store(tmp_path)}
    )
    assert started["state"] == "RUNNING"
    assert started["startedAt"]
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": _store(tmp_path)})
    states = [h["state"] for h in status["history"]]
    assert states[-1] == "RUNNING"
    assert "APPROVED" in states and "ARMED" in states

    # pause
    paused = robots.cmd_experiment_pause(
        {"experimentId": exp_id, "operator": "bob", "reason": "操作员休息", "storeRoot": _store(tmp_path)}
    )
    assert paused["state"] == "PAUSED"
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": _store(tmp_path)})
    assert status["history"][-1]["state"] == "PAUSED"
    assert status["history"][-1]["operator"] == "bob"
    assert status["history"][-1]["reason"] == "操作员休息"

    # safe cancel
    cancelled = robots.cmd_experiment_safe_cancel(
        {"experimentId": exp_id, "operator": "bob", "storeRoot": _store(tmp_path)}
    )
    assert cancelled["state"] == "ABORTED"
    assert "急停" in cancelled["note"]
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": _store(tmp_path)})
    assert status["state"] == "ABORTED"
    assert status["history"][-1]["state"] == "ABORTED"


def test_pause_with_recovery_keyword(tmp_path):
    exp_id = _prepare(tmp_path)["experimentId"]
    store_root = _store(tmp_path)
    robots.cmd_experiment_request_approval({"experimentId": exp_id, "storeRoot": store_root})
    robots.cmd_experiment_start({"experimentId": exp_id, "approvalRef": "A1", "storeRoot": store_root})
    recovered = robots.cmd_experiment_pause(
        {"experimentId": exp_id, "reason": "recovery: 关节过温，进入恢复流程", "storeRoot": store_root}
    )
    assert recovered["state"] == "RECOVERING"
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": store_root})
    assert status["state"] == "RECOVERING"


def test_state_machine_finalize_completed(tmp_path):
    exp_id = _prepare(tmp_path)["experimentId"]
    store_root = _store(tmp_path)
    robots.cmd_experiment_request_approval({"experimentId": exp_id, "storeRoot": store_root})
    robots.cmd_experiment_start({"experimentId": exp_id, "approvalRef": "SIG-alice", "storeRoot": store_root})
    fin = robots.cmd_experiment_finalize(
        {
            "experimentId": exp_id,
            "outcome": "completed",
            "summary": "任务完成",
            "humanConclusion": "确认无异常",
            "storeRoot": store_root,
        }
    )
    assert fin["state"] == "COMPLETED"
    status = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": store_root})
    assert status["state"] == "COMPLETED"
    assert "finalize" in status["history"][-1]["reason"]

    # double finalize on a terminal state is rejected
    with pytest.raises(WorkerError, match="终态"):
        robots.cmd_experiment_finalize({"experimentId": exp_id, "outcome": "failed", "storeRoot": store_root})


def test_finalize_invalid_outcome(tmp_path):
    exp_id = _prepare(tmp_path)["experimentId"]
    with pytest.raises(WorkerError, match="outcome"):
        robots.cmd_experiment_finalize({"experimentId": exp_id, "outcome": "exploded", "storeRoot": _store(tmp_path)})


def test_illegal_transitions_raise_worker_error(tmp_path):
    exp_id = _prepare(tmp_path)["experimentId"]
    store_root = _store(tmp_path)
    # force the record back to DRAFT to test illegal transitions
    record_path = robots.cmd_experiment_status({"experimentId": exp_id, "storeRoot": store_root})["recordPath"]
    record = json.loads(open(record_path, encoding="utf-8").read())
    record["state"] = "DRAFT"
    with open(record_path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, ensure_ascii=False, indent=2)

    with pytest.raises(WorkerError, match="非法状态转移"):
        robots.cmd_experiment_start({"experimentId": exp_id, "approvalRef": "X", "storeRoot": store_root})
    with pytest.raises(WorkerError, match="非法状态转移"):
        robots.cmd_experiment_pause({"experimentId": exp_id, "storeRoot": store_root})
    with pytest.raises(WorkerError, match="非法状态转移"):
        robots.cmd_experiment_safe_cancel({"experimentId": exp_id, "storeRoot": store_root})


def test_experiment_status_unknown_experiment(tmp_path):
    with pytest.raises(WorkerError, match="experiment not found"):
        robots.cmd_experiment_status({"experimentId": "ghost", "storeRoot": _store(tmp_path)})


# ---------------------------------------------------------------------------
# experiment-prepare validation
# ---------------------------------------------------------------------------


def test_prepare_requires_name(tmp_path):
    with pytest.raises(WorkerError, match="name"):
        robots.cmd_experiment_prepare({"storeRoot": _store(tmp_path)})


def test_prepare_rejects_unknown_scenario(tmp_path):
    with pytest.raises(WorkerError, match="scenario"):
        _prepare(tmp_path, scenario="no-such-scenario")


def test_prepare_rejects_missing_plan(tmp_path):
    with pytest.raises(WorkerError, match="plan"):
        _prepare(tmp_path, plan=str(tmp_path / "nope.md"))


def test_prepare_accepts_existing_plan_file(tmp_path):
    plan = tmp_path / "plan.md"
    plan.write_text("# experiment plan\n", encoding="utf-8")
    result = _prepare(tmp_path, plan=str(plan))
    assert result["ok"] is True
    assert result["state"] == "VALIDATING"


def test_prepare_validates_safety_limits(tmp_path):
    with pytest.raises(WorkerError, match="safetyLimits"):
        _prepare(tmp_path, safetyLimits={})
    with pytest.raises(WorkerError, match="safetyLimits"):
        _prepare(tmp_path, safetyLimits={"maxVelocity": -1.0})


def test_module_exports():
    expected = {
        "robot-preflight",
        "robot-state-snapshot",
        "experiment-prepare",
        "experiment-request-approval",
        "experiment-start",
        "experiment-pause",
        "experiment-safe-cancel",
        "experiment-status",
        "experiment-finalize",
    }
    assert expected <= set(robots.COMMANDS)
    assert 2 <= len(robots.CAPABILITIES) <= 3
