"""Tests for the telemetry analysis module.

Covers channel inventory, time windows, deterministic anomaly scanning,
failure evidence collection (with optional diagnostic case creation), run
comparison and timeline export. Synthetic telemetry is generated with numpy
and stored through ``RunStore`` under ``tmp_path``.
"""

from __future__ import annotations

import json
import os

import pytest

np = pytest.importorskip("numpy")

from robotic_harness_worker import telemetry  # noqa: E402
from robotic_harness_worker.core import Run, RunStore, WorkerError  # noqa: E402

N_SAMPLES = 201  # t = 0..10 s at 0.05 s -> 20 Hz


def _synthetic_telemetry(offset_after_s=None, offset_value=0.05):
    """Rows: normal sine (chanA), sine+spike at 5 s (chanB), 3-dim q vector,
    plus scalar/state/constant channels and perception scalar fields."""
    t = np.round(np.linspace(0.0, 10.0, N_SAMPLES), 6)
    freq = 0.2
    chan_a = np.sin(2.0 * np.pi * freq * t)
    chan_b = np.sin(2.0 * np.pi * freq * t).copy()
    spike_idx = int(np.argmin(np.abs(t - 5.0)))
    chan_b[spike_idx] += 6.0
    if offset_after_s is not None:
        mask = t >= offset_after_s
        chan_a = chan_a.copy()
        chan_a[mask] += offset_value
    q = np.column_stack([np.sin(2.0 * np.pi * freq * t + p) for p in (0.0, 1.0, 2.0)])
    rows = []
    for i in range(N_SAMPLES):
        rows.append(
            {
                "t": float(t[i]),
                "phase": "approach" if t[i] < 5.0 else "carry",
                "q": [round(float(v), 6) for v in q[i]],
                "qTarget": [round(float(v), 6) for v in q[i]],
                "trackingError": [0.01, 0.01, 0.01],
                "cupPos": [round(float(0.5 + 0.1 * np.sin(t[i])), 6), 0.0, 0.3],
                "objPos": [0.5, 0.0, 0.3],
                "attached": False,
                "suction": False,
                "chanA": round(float(chan_a[i]), 6),
                "chanB": round(float(chan_b[i]), 6),
                "perception": {"ok": True, "confidence": 0.9},
            }
        )
    return rows


def _make_run(store: RunStore, run_id: str, rows) -> Run:
    """Write a minimal run.json + telemetry.jsonl (+ artifact files) via Run.from_dict/to_dict."""
    artifact_dir = os.path.join(store.run_dir(run_id), "artifacts")
    os.makedirs(artifact_dir, exist_ok=True)
    artifacts = {}
    for name in ("joints.png", "tracking.png", "trajectory.png", "scene.png"):
        path = os.path.join(artifact_dir, name)
        with open(path, "wb") as handle:
            handle.write(b"placeholder")
        artifacts[name] = path
    run = Run.from_dict(
        {
            "id": run_id,
            "projectId": "proj-telemetry",
            "scenario": "synthetic-pick-place",
            "state": "failed",
            "metrics": {"success": False},
            "config": {},
            "phases": [],
            "anomalies": [],
            "artifacts": artifacts,
        }
    )
    store.save_run(run)
    with open(os.path.join(store.run_dir(run_id), "telemetry.jsonl"), "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
    return run


# ---------------------------------------------------------------------------
# telemetry-channels
# ---------------------------------------------------------------------------


def test_telemetry_channels_inventory(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-channels", _synthetic_telemetry())
    result = telemetry.cmd_telemetry_channels({"runPath": store.run_dir(run.id)})
    assert result["ok"] is True
    assert result["runId"] == run.id
    names = [c["name"] for c in result["channels"]]
    assert names == [
        "t",
        "phase",
        "q.0", "q.1", "q.2",
        "qTarget.0", "qTarget.1", "qTarget.2",
        "trackingError.0", "trackingError.1", "trackingError.2",
        "cupPos.0", "cupPos.1", "cupPos.2",
        "objPos.0", "objPos.1", "objPos.2",
        "attached",
        "suction",
        "chanA",
        "chanB",
        "perception.ok",
        "perception.confidence",
    ]
    by_name = {c["name"]: c for c in result["channels"]}
    assert by_name["q.0"]["kind"] == "vector"
    assert by_name["chanA"]["kind"] == "scalar"
    assert by_name["phase"]["kind"] == "state"
    for c in result["channels"]:
        assert c["length"] == N_SAMPLES
        assert c["missing"] == 0
        assert c["sampleRateHz"] is not None
        assert 19.5 <= c["sampleRateHz"] <= 20.5
    assert by_name["chanB"]["max"] > 5.0  # spike present in the range
    assert by_name["chanA"]["min"] < 0.0 < by_name["chanA"]["max"]
    assert "min" not in by_name["phase"] or by_name["phase"].get("min") is None


# ---------------------------------------------------------------------------
# telemetry-window
# ---------------------------------------------------------------------------


def test_telemetry_window_stats(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-window", _synthetic_telemetry())
    result = telemetry.cmd_telemetry_window({"runPath": store.run_dir(run.id), "startS": 2.0, "endS": 4.0})
    assert result["ok"] is True
    assert result["window"] == {"startS": 2.0, "endS": 4.0}
    assert result["samples"] == 41  # (4.0-2.0)/0.05 + 1
    by_name = {c["name"]: c for c in result["channels"]}
    t = np.round(np.linspace(0.0, 10.0, N_SAMPLES), 6)
    mask = (t >= 2.0) & (t <= 4.0)
    expected = np.sin(2.0 * np.pi * 0.2 * t[mask])
    stats = by_name["chanA"]["stats"]
    assert stats["count"] == 41
    assert abs(stats["mean"] - float(np.mean(expected))) < 1e-5
    assert abs(stats["std"] - float(np.std(expected))) < 1e-5
    assert abs(stats["min"] - float(np.min(expected))) < 1e-5
    assert abs(stats["max"] - float(np.max(expected))) < 1e-5
    # chanB spike is at 5 s (outside this window) so its max is the sine max
    assert abs(by_name["chanB"]["stats"]["max"] - float(np.max(expected))) < 1e-5
    # state channel: values present, stats null
    assert by_name["phase"]["stats"] is None
    assert set(by_name["phase"]["values"]) <= {"approach", "carry"}
    assert len(by_name["phase"]["values"]) == 41


def test_telemetry_window_default_and_filter(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-window2", _synthetic_telemetry())
    full = telemetry.cmd_telemetry_window({"runPath": store.run_dir(run.id)})
    assert full["samples"] == N_SAMPLES
    assert full["window"] == {"startS": 0.0, "endS": 10.0}
    filtered = telemetry.cmd_telemetry_window(
        {"runPath": store.run_dir(run.id), "startS": 0.0, "endS": 1.0, "channels": ["chanA", "q.0"]}
    )
    assert [c["name"] for c in filtered["channels"]] == ["chanA", "q.0"]
    assert filtered["samples"] == 21


# ---------------------------------------------------------------------------
# anomaly-scan
# ---------------------------------------------------------------------------


def test_anomaly_scan_detects_spike_threshold_and_constant(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-anomaly", _synthetic_telemetry())
    result = telemetry.cmd_anomaly_scan(
        {
            "runPath": store.run_dir(run.id),
            "method": "all",
            "windowS": 1.0,
            "thresholds": {"chanB": {"max": 2.0, "maxRate": 5.0, "spikeSigma": 6.0}},
        }
    )
    assert result["ok"] is True
    anomalies = result["anomalies"]
    assert anomalies == sorted(anomalies, key=lambda a: (a["t"], a["channel"]))
    assert result["summary"]["total"] == len(anomalies)
    assert result["summary"]["byMethod"]["spike"] == sum(1 for a in anomalies if a["method"] == "spike")
    spike = [a for a in anomalies if a["method"] == "spike"]
    assert any(4.9 <= a["t"] <= 5.1 for a in spike), spike
    threshold = [a for a in anomalies if a["method"] == "threshold"]
    assert any(4.9 <= a["t"] <= 5.1 for a in threshold), threshold
    assert any(a["channel"] == "chanB" and a["method"] == "threshold" and a["value"] > 2.0 for a in anomalies)
    # built-in constant-channel warning
    assert any(a["method"] == "constant" for a in anomalies)
    # rate detector also fires around the spike
    assert any(a["method"] == "rate" and 4.9 <= a["t"] <= 5.1 for a in anomalies)


def test_anomaly_scan_channel_filter(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-anomaly2", _synthetic_telemetry())
    result = telemetry.cmd_anomaly_scan(
        {"runPath": store.run_dir(run.id), "channels": ["chanB"], "thresholds": {"chanB": {"max": 2.0}}}
    )
    assert [s["name"] for s in result["scannedChannels"]] == ["chanB"]
    assert all(a["channel"] == "chanB" for a in result["anomalies"])
    assert any(a["method"] == "threshold" for a in result["anomalies"])


# ---------------------------------------------------------------------------
# failure-evidence-collect
# ---------------------------------------------------------------------------


def test_failure_evidence_collect_window_and_artifacts(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-evidence", _synthetic_telemetry())
    result = telemetry.cmd_failure_evidence_collect(
        {"runPath": store.run_dir(run.id), "anomalyKinds": ["threshold", "spike", "rate"]}
    )
    assert result["ok"] is True
    assert result["runId"] == run.id
    assert result["window"] == {"startS": 4.0, "endS": 6.0}
    assert result["window"]["startS"] <= 5.0 <= result["window"]["endS"]
    assert result["evidence"]["telemetryRows"] == 41
    assert any(a["t"] == 5.0 for a in result["anomalies"])
    names = {c["name"] for c in result["evidence"]["channels"]}
    assert "chanB" in names and "q.0" in names and "phase" in names
    artifacts = result["evidence"]["artifacts"]
    assert len(artifacts) == 4
    for path in artifacts:
        assert os.path.exists(path)
        assert os.path.basename(path) in ("joints.png", "tracking.png", "trajectory.png", "scene.png")
    assert result["evidence"]["diagnosticCaseRef"] is None
    assert result.get("caseId") is None


def test_failure_evidence_create_case(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-evidence-case", _synthetic_telemetry())
    result = telemetry.cmd_failure_evidence_collect(
        {"runPath": store.run_dir(run.id), "anomalyKinds": ["spike"], "createCase": True, "storeRoot": str(tmp_path / "store")}
    )
    assert result["ok"] is True
    case_path = result["casePath"]
    assert case_path and os.path.exists(case_path)
    with open(case_path, encoding="utf-8") as handle:
        case = json.load(handle)
    assert case["id"] == result["caseId"]
    assert case["runId"] == run.id
    assert result["evidence"]["diagnosticCaseRef"] == {"caseId": case["id"], "casePath": case_path}
    assert result["window"]["startS"] == 4.0 and result["window"]["endS"] == 6.0


# ---------------------------------------------------------------------------
# run-compare
# ---------------------------------------------------------------------------


def test_run_compare_identical(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    rows = _synthetic_telemetry()
    run_a = _make_run(store, "run-cmp-a", rows)
    run_b = _make_run(store, "run-cmp-b", json.loads(json.dumps(rows)))
    result = telemetry.cmd_run_compare(
        {"runA": store.run_dir(run_a.id), "runB": store.run_dir(run_b.id), "channels": ["chanA"]}
    )
    assert result["ok"] is True
    assert result["firstDivergence"] is None
    assert result["alignedSamples"] == N_SAMPLES
    stats = result["perChannel"]["chanA"]
    assert stats["maxDelta"] == 0.0
    assert stats["rmsDelta"] == 0.0
    assert stats["correlation"] == 1.0
    assert result["summary"]["anyDivergence"] is False


def test_run_compare_detects_divergence(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run_a = _make_run(store, "run-cmp-a2", _synthetic_telemetry())
    run_b = _make_run(store, "run-cmp-b2", _synthetic_telemetry(offset_after_s=3.0, offset_value=0.05))
    result = telemetry.cmd_run_compare(
        {"runA": store.run_dir(run_a.id), "runB": store.run_dir(run_b.id), "channels": ["chanA"]}
    )
    assert result["ok"] is True
    fd = result["firstDivergence"]
    assert fd is not None
    assert 2.95 <= fd["t"] <= 3.05, fd
    assert fd["channel"] == "chanA"
    assert abs(fd["delta"] - 0.05) < 1e-6, fd
    # valueA/valueB are rounded independently, so compare their fp difference
    # against the rounded delta with a small tolerance
    assert abs(abs(fd["valueA"] - fd["valueB"]) - fd["delta"]) < 1e-9, fd
    stats = result["perChannel"]["chanA"]
    assert stats["maxDelta"] is not None and abs(stats["maxDelta"] - 0.05) < 1e-6
    assert stats["correlation"] is not None and stats["correlation"] > 0.99
    assert result["summary"]["anyDivergence"] is True
    assert result["summary"]["worstChannel"] == "chanA"


def test_run_compare_default_channel_intersection(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run_a = _make_run(store, "run-cmp-a3", _synthetic_telemetry())
    run_b = _make_run(store, "run-cmp-b3", _synthetic_telemetry(offset_after_s=3.0))
    result = telemetry.cmd_run_compare({"runA": store.run_dir(run_a.id), "runB": store.run_dir(run_b.id)})
    assert result["ok"] is True
    assert "chanA" in result["perChannel"] and "q.0" in result["perChannel"] and "chanB" in result["perChannel"]
    # constant channels yield null correlation
    assert result["perChannel"]["attached"]["correlation"] is None


# ---------------------------------------------------------------------------
# timeline-export
# ---------------------------------------------------------------------------


def test_timeline_export_writes_file(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-timeline", _synthetic_telemetry())
    out = str(tmp_path / "timeline.html")
    result = telemetry.cmd_timeline_export({"runPath": store.run_dir(run.id), "outPath": out})
    assert result["ok"] is True
    assert result["runId"] == run.id
    assert os.path.exists(result["path"])
    assert os.path.isabs(result["path"])
    with open(result["path"], encoding="utf-8") as handle:
        content = handle.read()
    assert "Robotic Harness — run timeline" in content
    assert run.id in content


# ---------------------------------------------------------------------------
# failure paths
# ---------------------------------------------------------------------------


def test_failure_paths(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run = _make_run(store, "run-fail", _synthetic_telemetry())
    run_dir = store.run_dir(run.id)
    with pytest.raises(WorkerError):
        telemetry.cmd_telemetry_channels({})
    with pytest.raises(WorkerError):
        telemetry.cmd_telemetry_channels({"runPath": str(tmp_path / "missing")})
    with pytest.raises(WorkerError):
        telemetry.cmd_telemetry_window({"runPath": run_dir, "startS": 5.0, "endS": 2.0})
    with pytest.raises(WorkerError):
        telemetry.cmd_telemetry_window({"runPath": run_dir, "channels": ["does.not.exist"]})
    with pytest.raises(WorkerError):
        telemetry.cmd_anomaly_scan({"runPath": run_dir, "method": "bogus"})
    with pytest.raises(WorkerError):
        telemetry.cmd_anomaly_scan({"runPath": run_dir, "channels": ["does.not.exist"]})
    with pytest.raises(WorkerError):
        telemetry.cmd_failure_evidence_collect({"runPath": run_dir, "anomalyRef": {"channel": "does.not.exist"}})
    with pytest.raises(WorkerError):
        telemetry.cmd_run_compare({"runA": run_dir})
    with pytest.raises(WorkerError):
        telemetry.cmd_timeline_export({"runPath": run_dir})


def test_module_exports_contract():
    assert set(telemetry.COMMANDS) == {
        "telemetry-channels",
        "telemetry-window",
        "anomaly-scan",
        "failure-evidence-collect",
        "run-compare",
        "timeline-export",
    }
    assert isinstance(telemetry.CAPABILITIES, list) and len(telemetry.CAPABILITIES) == 6
    for capability in telemetry.CAPABILITIES:
        assert capability["id"] and capability["risk"]
