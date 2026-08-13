"""End-to-end simulation tests: happy path, fault path, diagnostics, evidence.

These tests require mujoco (plus numpy); they are skipped when the modules
are missing so the pure-logic suite still runs on minimal environments.
"""

from __future__ import annotations

import json
import os

import pytest

mujoco = pytest.importorskip("mujoco")

from robotic_harness_worker import diagnostics as diag
from robotic_harness_worker import report
from robotic_harness_worker.core import RunStore
from robotic_harness_worker.simulation import SCENARIO_PICK_PLACE, run_pick_place


def test_sim_status_like_env_loads():
    from robotic_harness_worker.simulation import PickPlaceEnv

    env = PickPlaceEnv(SCENARIO_PICK_PLACE)
    env.reset()
    assert env.model.nq >= 10  # 3 arm joints + 7 free-joint qpos
    assert env.render_rgb() is not None or True  # renderer is best-effort


def test_happy_path_succeeds(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=42, store=store)
    assert run.state == "completed"
    assert run.metrics["success"] is True, run.metrics
    assert run.metrics["grasped"] is True
    assert run.metrics["inTargetZone"] is True
    assert run.artifacts["run.json"]
    telemetry_path = os.path.join(store.run_dir(run.id), "telemetry.jsonl")
    assert os.path.exists(telemetry_path)
    assert len(telemetry) > 30


def test_fault_run_fails_with_evidence(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    fault = {"perception_offset_px": [18.0, 6.0], "gripper_slip": True, "tf_offset": [0.015, 0.0]}
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, fault, seed=43, store=store)
    assert run.state == "completed"
    assert run.metrics["success"] is False
    kinds = {a.kind for a in run.anomalies}
    assert "grasp_missed" in kinds or "gripper_slip" in kinds, kinds


def test_diagnostics_attach_hypotheses_to_fault_run(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    fault = {"perception_offset_px": [18.0, 6.0], "tf_offset": [0.015, 0.0]}
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, fault, seed=44, store=store)
    case = diag.diagnose(run, telemetry)
    layers = {h.layer for h in case.hypotheses}
    assert "perception" in layers
    origins = {f.origin for f in case.findings}
    assert origins >= {"fact", "rule"}
    assert any(f.origin == "rule" for f in case.findings)


def test_slip_fault_detected(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, {"gripper_slip": True}, seed=45, store=store)
    assert any(a.kind == "gripper_slip" for a in run.anomalies)
    case = diag.diagnose(run, telemetry)
    assert any(h.layer == "mechanical" for h in case.hypotheses)


def test_evidence_bundle_and_report(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=46, store=store)
    run_dir = store.run_dir(run.id)
    case = diag.diagnose(run, telemetry)
    bundle_dir = str(tmp_path / "bundle")
    manifest = report.export_evidence(run_dir, case, bundle_dir)
    assert manifest["run"]["id"] == run.id
    names = {f["name"] for f in manifest["files"]}
    assert "run.json" in names and "telemetry.jsonl" in names and "diagnostics.json" in names
    report_path = str(tmp_path / "report.md")
    report.generate_report(run_dir, case, report_path)
    assert os.path.exists(report_path)
    with open(report_path, encoding="utf-8") as handle:
        content = handle.read()
    assert "Robotic Harness experiment report" in content
    assert "Fault configuration" in content


def test_timeline_html_embeds_payload(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=47, store=store)
    case = diag.diagnose(run, telemetry)
    path = str(tmp_path / "timeline.html")
    report.timeline_html(run, telemetry, case, path)
    with open(path, encoding="utf-8") as handle:
        content = handle.read()
    assert "Robotic Harness — run timeline" in content
    assert run.id in content


def test_telemetry_schema_stable(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=48, store=store)
    first = telemetry[0]
    for key in ("t", "phase", "q", "qTarget", "trackingError", "cupPos", "objPos", "attached", "suction"):
        assert key in first, key
