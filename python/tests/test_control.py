"""Tests for the control analysis module (``robotic_harness_worker.control``).

Covers: trace metrics & anomaly detection, trajectory validation, planned vs
actual comparison, PID experiment templates, controller-config diff, system
identification and the Markdown report. Input files are generated into pytest
``tmp_path``; all data is synthetic (no fixtures needed).
"""

from __future__ import annotations

import csv
import math
import os

import numpy as np
import pytest

from robotic_harness_worker import control
from robotic_harness_worker.core import WorkerError


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _write_csv(path, columns):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(list(columns))
        for row in zip(*columns.values()):
            writer.writerow(row)


def _first_order_step(t, tau, delay=0.0, gain=1.0):
    s = np.maximum(t - delay, 0.0)
    return gain * (1.0 - np.exp(-s / tau))


def _second_order_step(t, zeta, wn, gain=1.0):
    wd = wn * math.sqrt(1.0 - zeta**2)
    return gain * (
        1.0
        - np.exp(-zeta * wn * t)
        * (np.cos(wd * t) + zeta / math.sqrt(1.0 - zeta**2) * np.sin(wd * t))
    )


# ---------------------------------------------------------------------------
# control-trace-analyze
# ---------------------------------------------------------------------------


def test_trace_analyze_step_metrics(tmp_path):
    rng = np.random.default_rng(7)
    dt = 0.01
    t = np.arange(0.0, 6.0, dt)
    tau = 0.5
    step_t = 1.0
    setpoint = np.where(t >= step_t, 1.0, 0.0)
    response = _first_order_step(t, tau, delay=step_t)
    measurement = response + rng.normal(0.0, 0.003, size=len(t))
    effort = 2.0 * (setpoint - measurement)
    path = tmp_path / "trace.csv"
    _write_csv(path, {"t": t, "setpoint": setpoint, "measurement": measurement, "effort": effort})

    result = control.analyze_trace({"path": str(path), "stepStart": step_t})
    assert result["ok"] is True
    assert result["rows"] == len(t)
    m = result["metrics"]
    # first-order: riseTime = 2.198*tau ≈ 1.10 s, settling (2%) = 3.912*tau ≈ 1.96 s
    assert m["riseTimeS"] is not None and 0.9 <= m["riseTimeS"] <= 1.3
    assert m["settlingTimeS"] is not None and 1.5 <= m["settlingTimeS"] <= 2.5
    assert m["overshootPercent"] is not None and 0.0 <= m["overshootPercent"] < 5.0
    assert m["steadyStateError"] is not None and abs(m["steadyStateError"]) < 0.01
    assert 0.1 < m["trackingErrorRms"] < 0.35
    assert m["peakError"] > 0.8
    assert m["controlEffortRms"] is not None and m["controlEffortRms"] > 0
    # no false-positive anomalies on a clean damped step
    codes = {issue["code"] for issue in result["issues"]}
    assert "oscillation.high" not in codes
    assert "saturation.persistent" not in codes


def test_trace_analyze_saturation_issue(tmp_path):
    t = np.arange(0.0, 5.0, 0.01)
    setpoint = np.full_like(t, 1.0)
    measurement = np.full_like(t, 0.4)  # persistent error -> effort pinned at max
    effort = np.clip(3.0 * (setpoint - measurement), 0.0, 1.0)
    path = tmp_path / "sat.csv"
    _write_csv(path, {"t": t, "setpoint": setpoint, "measurement": measurement, "effort": effort})

    result = control.analyze_trace({"path": str(path), "effortMin": 0.0, "effortMax": 1.0})
    codes = {issue["code"] for issue in result["issues"]}
    assert "saturation.persistent" in codes
    saturation = next(i for i in result["issues"] if i["code"] == "saturation.persistent")
    assert saturation["evidence"]["fraction"] > 0.9


def test_trace_analyze_missing_time_column(tmp_path):
    path = tmp_path / "bad.csv"
    _write_csv(path, {"x": [1.0, 2.0], "y": [3.0, 4.0]})
    with pytest.raises(WorkerError):
        control.analyze_trace({"path": str(path)})


def test_trace_analyze_missing_file():
    with pytest.raises(WorkerError):
        control.analyze_trace({"path": "no/such/file.csv"})


def test_cmd_trace_wrapper_adds_input_args(tmp_path):
    t = np.arange(0.0, 2.0, 0.01)
    path = tmp_path / "w.csv"
    _write_csv(path, {"t": t, "setpoint": np.ones_like(t), "measurement": np.ones_like(t) * 0.9})
    result = control.COMMANDS["control-trace-analyze"]({"path": str(path)})
    assert result["ok"] is True
    assert result["inputArgs"]["path"] == str(path)


# ---------------------------------------------------------------------------
# trajectory-validate
# ---------------------------------------------------------------------------


def test_trajectory_validate_ok(tmp_path):
    t = np.arange(0.0, 2.0, 0.01)
    q0 = 0.5 * np.sin(2 * np.pi * 0.5 * t)
    path = tmp_path / "traj_ok.csv"
    _write_csv(path, {"t": t, "q0": q0, "dq0": np.gradient(q0, 0.01)})

    result = control.validate_trajectory({"path": str(path)})
    assert result["ok"] is True
    assert result["rows"] == len(t)
    assert result["issues"] == []
    assert result["stats"]["durationS"] == pytest.approx(2.0, abs=0.02)
    assert result["stats"]["meanDtS"] == pytest.approx(0.01, abs=1e-3)
    assert result["stats"]["jitterS"] == pytest.approx(0.0, abs=1e-6)


def test_trajectory_validate_detects_problems(tmp_path):
    t = [0.0, 0.1, 0.2, 0.15, 0.3, 0.4]  # time goes backwards at index 3
    q0 = [0.0, 0.05, 0.1, 0.2, 0.8, 0.9]  # jump 0.6 (idx 3->4), exceeds limit 0.5
    path = tmp_path / "traj_bad.csv"
    _write_csv(path, {"t": t, "q0": q0})

    result = control.validate_trajectory({"path": str(path), "limits": {"q0": [-0.5, 0.5]}})
    codes = {issue["code"] for issue in result["issues"]}
    assert "time.decreasing" in codes
    assert "joint.jump" in codes
    assert "joint.limit_exceeded" in codes
    assert result["ok"] is False


def test_trajectory_validate_nan_issue(tmp_path):
    path = tmp_path / "traj_nan.csv"
    _write_csv(path, {"t": [0.0, 0.1, 0.2], "q0": [0.0, np.nan, 0.2]})
    result = control.validate_trajectory({"path": str(path)})
    codes = {issue["code"] for issue in result["issues"]}
    assert "data.non_finite" in codes
    assert result["ok"] is False


def test_trajectory_validate_too_few_samples(tmp_path):
    path = tmp_path / "tiny.csv"
    _write_csv(path, {"t": [0.0], "q0": [0.1]})
    with pytest.raises(WorkerError):
        control.validate_trajectory({"path": str(path)})


# ---------------------------------------------------------------------------
# planned-actual-compare
# ---------------------------------------------------------------------------


def test_compare_almost_identical(tmp_path):
    rng = np.random.default_rng(11)
    t = np.arange(0.0, 3.0, 0.01)
    q0 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
    q1 = 0.2 * np.cos(2 * np.pi * 0.4 * t)
    planned = tmp_path / "planned.csv"
    actual = tmp_path / "actual.csv"
    _write_csv(planned, {"t": t, "q0": q0, "q1": q1})
    _write_csv(actual, {"t": t, "q0": q0 + rng.normal(0, 1e-3, len(t)), "q1": q1 + rng.normal(0, 1e-3, len(t))})

    result = control.compare_planned_actual({"plannedPath": str(planned), "actualPath": str(actual)})
    assert result["ok"] is True
    assert result["alignedSamples"] == len(t)
    assert result["firstDivergenceS"] is None
    for name in ("q0", "q1"):
        assert result["perJoint"][name]["rms"] < 0.005
        assert result["perJoint"][name]["max"] < 0.02


def test_compare_detects_offset_and_divergence(tmp_path):
    t = np.arange(0.0, 3.0, 0.01)
    delay = 0.1
    q0 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
    q1 = 0.2 * np.cos(2 * np.pi * 0.4 * t)
    planned = tmp_path / "planned2.csv"
    actual = tmp_path / "actual2.csv"
    _write_csv(planned, {"t": t, "q0": q0, "q1": q1})
    t_a = t + delay
    bump = np.where(t > 1.5, 0.05, 0.0)  # divergence on q1 after 1.5 s
    _write_csv(actual, {"t": t_a, "q0": q0, "q1": q1 + bump})

    result = control.compare_planned_actual({"plannedPath": str(planned), "actualPath": str(actual)})
    assert result["ok"] is True
    assert result["timeOffsetS"] > 0.05
    # the pure delay alone makes the signals diverge as soon as the planned
    # trajectory leaves its initial value, so a divergence must be reported
    assert result["firstDivergenceS"] is not None
    # delay-induced error on the 0.5 Hz sine: |dq/dt|*delay ~ 0.094 max, ~0.067 rms
    assert 0.03 < result["perJoint"]["q0"]["rms"] < 0.12
    assert result["perJoint"]["q0"]["max"] < 0.15
    assert result["perJoint"]["q1"]["rms"] > 0.02  # the added bump dominates q1
    assert result["perJoint"]["q1"]["max"] > 0.04


def test_compare_missing_files(tmp_path):
    with pytest.raises(WorkerError):
        control.compare_planned_actual(
            {"plannedPath": str(tmp_path / "a.csv"), "actualPath": str(tmp_path / "b.csv")}
        )


# ---------------------------------------------------------------------------
# pid-experiment-prepare
# ---------------------------------------------------------------------------


def test_pid_step_template():
    result = control.prepare_pid_experiment(
        {"controllerId": "arm-pid", "joints": ["q0", "q1"], "amplitude": 0.1, "stepTimeS": 1.0, "durationS": 4.0}
    )
    exp = result["experiment"]
    assert exp["kind"] == "step"
    assert exp["joints"] == ["q0", "q1"]
    assert exp["amplitude"] == 0.1
    assert exp["durationS"] == 4.0
    waypoints = exp["waypoints"]
    assert len(waypoints) == 401
    assert waypoints[0]["t"] == 0.0
    assert waypoints[0]["value"] == 0.1  # starts high
    values = {w["value"] for w in waypoints}
    assert values <= {0.0, 0.1}
    assert any(w["t"] == 1.0 and w["value"] == 0.0 for w in waypoints)  # first drop
    assert any(w["t"] == 2.0 and w["value"] == 0.1 for w in waypoints)  # next rise
    assert exp["safety"]["maxJump"] == pytest.approx(0.1, abs=1e-6)
    assert "模板不执行任何硬件操作" in result["note"]


def test_pid_sweep_template():
    result = control.prepare_pid_experiment(
        {
            "controllerId": "arm-pid",
            "joints": ["q0"],
            "amplitude": 0.2,
            "durationS": 2.0,
            "sweep": {"freqMinHz": 0.1, "freqMaxHz": 1.0},
        }
    )
    exp = result["experiment"]
    assert exp["kind"] == "sweep"
    waypoints = exp["waypoints"]
    assert len(waypoints) == 201
    values = np.array([w["value"] for w in waypoints])
    assert np.max(np.abs(values)) <= 0.2 + 1e-9
    assert np.min(values) < -0.1  # sine sweeps below zero
    assert waypoints[0]["value"] == 0.0  # sin(0) = 0
    times = np.array([w["t"] for w in waypoints])
    assert np.all(np.diff(times) > 0)


def test_pid_invalid_args():
    with pytest.raises(WorkerError):
        control.prepare_pid_experiment({"controllerId": "x", "joints": ["q0"], "amplitude": 0.0})
    with pytest.raises(WorkerError):
        control.prepare_pid_experiment({"controllerId": "x", "joints": []})
    with pytest.raises(WorkerError):
        control.prepare_pid_experiment(
            {"controllerId": "x", "joints": ["q0"], "sweep": {"freqMinHz": 2.0, "freqMaxHz": 1.0}}
        )


# ---------------------------------------------------------------------------
# controller-config-compare
# ---------------------------------------------------------------------------


def test_controller_config_compare():
    config_a = {"name": "cfgA", "joints": {"q0": {"kp": 80.0, "kv": 5.0}, "q1": {"kp": 60.0, "kv": 4.0}}}
    config_b = {"name": "cfgB", "joints": {"q0": {"kp": 120.0, "kv": 8.0, "ki": 2.0}, "q1": {"kp": 60.0, "kv": 4.0}}}
    result = control.compare_controller_configs({"configA": config_a, "configB": config_b})
    assert result["ok"] is True
    assert result["configA"]["name"] == "cfgA"
    assert result["configB"]["name"] == "cfgB"
    differences = result["differences"]
    assert len(differences) >= 3
    by_key = {(d["joint"], d["param"]): d for d in differences}
    assert by_key[("q0", "kp")]["valueA"] == 80.0
    assert by_key[("q0", "kp")]["valueB"] == 120.0
    assert ("q0", "ki") in by_key  # ki added in B only
    assert all(d["impact"] for d in differences)
    assert all(issue["severity"] != "error" for issue in result["issues"])


def test_controller_config_compare_bad_gains():
    config_a = {"name": "A", "joints": {"q0": {"kp": -5.0, "kv": 1.0}}}
    config_b = {"name": "B", "joints": {"q0": {"kp": 0.0, "kv": 1.0}}}
    result = control.compare_controller_configs({"configA": config_a, "configB": config_b})
    codes = {issue["code"] for issue in result["issues"]}
    assert "param.negative" in codes
    assert "param.zero_kp" in codes
    assert result["ok"] is False


def test_controller_config_compare_missing_configs():
    with pytest.raises(WorkerError):
        control.compare_controller_configs({"configA": {}, "configB": "not-a-dict"})


# ---------------------------------------------------------------------------
# system-identification
# ---------------------------------------------------------------------------


def test_system_id_first_order(tmp_path):
    rng = np.random.default_rng(5)
    t = np.arange(0.0, 4.0, 0.01)
    gain, tau = 2.0, 0.5
    y = _first_order_step(t, tau, gain=gain) + rng.normal(0.0, 0.005, len(t))
    path = tmp_path / "step1.csv"
    _write_csv(path, {"t": t, "measurement": y})

    result = control.identify_system({"path": str(path)})
    model = result["model"]
    assert model["kind"] == "first-order"
    assert model["gain"] == pytest.approx(gain, rel=0.1)
    assert model["timeConstantS"] == pytest.approx(tau, rel=0.1)
    assert model["delayS"] is not None and model["delayS"] < 0.1
    assert model["fitQuality"]["explainedVariance"] > 0.95
    assert result["method"]
    assert result["notes"]


def test_system_id_second_order(tmp_path):
    rng = np.random.default_rng(9)
    # duration long enough for the underdamped tail to settle (zeta=0.3, wn=4
    # -> 2% settling ~3.2 s) so the steady-state gain estimate is unbiased
    t = np.arange(0.0, 6.0, 0.01)
    zeta, wn = 0.3, 4.0
    y = _second_order_step(t, zeta, wn) + rng.normal(0.0, 0.005, len(t))
    path = tmp_path / "step2.csv"
    _write_csv(path, {"t": t, "measurement": y})

    result = control.identify_system({"path": str(path)})
    model = result["model"]
    assert model["kind"] == "second-order"
    assert model["gain"] == pytest.approx(1.0, rel=0.1)
    assert model["dampingRatio"] == pytest.approx(zeta, rel=0.1)
    assert model["naturalFrequencyHz"] == pytest.approx(wn / (2 * math.pi), rel=0.1)
    assert model["delayS"] is not None and model["delayS"] < 0.1
    assert model["fitQuality"]["explainedVariance"] > 0.9


def test_system_id_constant_response(tmp_path):
    path = tmp_path / "const.csv"
    _write_csv(path, {"t": [0.0, 0.1, 0.2], "measurement": [1.0, 1.0, 1.0]})
    with pytest.raises(WorkerError):
        control.identify_system({"path": str(path)})


def test_system_id_missing_measurement(tmp_path):
    path = tmp_path / "nocol.csv"
    _write_csv(path, {"t": [0.0, 0.1, 0.2], "q0": [0.0, 0.1, 0.2]})
    with pytest.raises(WorkerError):
        control.identify_system({"path": str(path)})


# ---------------------------------------------------------------------------
# control-report
# ---------------------------------------------------------------------------


def test_control_report(tmp_path):
    t = np.arange(0.0, 3.0, 0.01)
    q0 = 0.3 * np.sin(2 * np.pi * 0.5 * t)
    planned = tmp_path / "r_planned.csv"
    actual = tmp_path / "r_actual.csv"
    _write_csv(planned, {"t": t, "q0": q0})
    _write_csv(actual, {"t": t, "q0": q0 + 0.01})

    t2 = np.arange(0.0, 5.0, 0.01)
    setpoint = np.where(t2 >= 1.0, 1.0, 0.0)
    response = _first_order_step(t2, 0.5, delay=1.0)
    trace = tmp_path / "r_trace.csv"
    _write_csv(trace, {"t": t2, "setpoint": setpoint, "measurement": response})

    traj = tmp_path / "r_traj.csv"
    _write_csv(traj, {"t": t, "q0": q0})

    out = tmp_path / "report.md"
    result = control.generate_control_report({
        "outPath": str(out),
        "sections": [
            {"kind": "trace", "title": "阶跃响应跟踪分析", "path": str(trace), "stepStart": 1.0},
            {"kind": "trajectory", "title": "计划轨迹校验", "path": str(traj)},
            {"kind": "compare", "title": "计划 vs 实际", "plannedPath": str(planned), "actualPath": str(actual)},
        ],
    })
    assert result["ok"] is True
    assert result["path"] == os.path.abspath(str(out))
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    assert "结论需人工确认、不自动应用于真机" in text
    assert "阶跃响应跟踪分析" in text
    assert "计划轨迹校验" in text
    assert "计划 vs 实际" in text
    assert "上升时间" in text


def test_control_report_unknown_section(tmp_path):
    out = tmp_path / "bad.md"
    with pytest.raises(WorkerError):
        control.generate_control_report({"outPath": str(out), "sections": [{"kind": "nope"}]})


# ---------------------------------------------------------------------------
# module surface
# ---------------------------------------------------------------------------


def test_commands_registered():
    for name in (
        "control-trace-analyze",
        "trajectory-validate",
        "planned-actual-compare",
        "pid-experiment-prepare",
        "controller-config-compare",
        "system-identification",
        "control-report",
    ):
        assert name in control.COMMANDS


def test_capabilities_shape():
    capabilities = control.CAPABILITIES
    assert 3 <= len(capabilities) <= 5
    for capability in capabilities:
        assert capability["id"]
        assert capability["kind"]
        assert capability["risk"]
        assert capability["description"]
    ids = {c["id"] for c in capabilities}
    assert "control.trace_analyze" in ids
    assert "control.experiment_prepare" in ids
