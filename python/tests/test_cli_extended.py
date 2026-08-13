"""Tests for the extended simulation/report CLI commands added by the core
integration (sdf-validate, sim-fault-inject, sim-replay, sim-real-gap-report,
sim-batch-benchmark, dashboard-generate).
"""

from __future__ import annotations

import os

import pytest

mujoco = pytest.importorskip("mujoco")

from robotic_harness_worker import cli
from robotic_harness_worker.core import RunStore
from robotic_harness_worker.simulation import SCENARIO_PICK_PLACE, run_pick_place

FIXTURES = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "fixtures",
    "robot_assets",
)


def _run_command(command: str, args: dict, store_root: str) -> dict:
    return cli.COMMANDS[command]({**args, "storeRoot": store_root})


def test_sdf_validate(tmp_path):
    good = os.path.join(FIXTURES, "rh_arm.sdf")
    result = _run_command("sdf-validate", {"path": good}, str(tmp_path))
    assert result["ok"] is True
    assert result["format"] == "sdf"
    assert result["summary"]["models"][0]["links"] == 2
    broken = tmp_path / "bad.sdf"
    broken.write_text("<sdf version='1.9'><model><link></model>", encoding="utf-8")
    result = _run_command("sdf-validate", {"path": str(broken)}, str(tmp_path))
    assert result["ok"] is False
    with pytest.raises(Exception):
        _run_command("sdf-validate", {"path": str(tmp_path / "missing.sdf")}, str(tmp_path))


def test_sim_fault_inject(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    result = _run_command(
        "sim-fault-inject",
        {"fault": {"gripper_slip": True}, "seed": 7},
        store.root,
    )
    assert result["ok"] is True
    assert result["injectedFault"]["gripper_slip"] is True
    assert any(a["kind"] == "gripper_slip" for a in result["anomalies"])


def test_sim_replay(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, _ = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=8, store=store)
    out_dir = tmp_path / "replay"
    result = _run_command("sim-replay", {"runPath": store.run_dir(run.id), "outDir": str(out_dir)}, store.root)
    assert result["ok"] is True
    assert os.path.exists(os.path.join(str(out_dir), "timeline.html"))
    assert os.path.exists(os.path.join(str(out_dir), "run.json"))
    assert "replay is read-only" in result["note"]


def test_sim_real_gap_report(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, telemetry = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=9, store=store)
    real_csv = tmp_path / "real.csv"
    import csv

    with open(real_csv, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "joint0"])
        for row in telemetry:
            writer.writerow([row["t"], row["q"][0] + 0.05])  # offset real data
    result = _run_command(
        "sim-real-gap-report",
        {"simRunPath": store.run_dir(run.id), "realCsvPath": str(real_csv), "channelMap": {"q.0": "joint0"}},
        store.root,
    )
    assert result["ok"] is True
    assert result["largestGap"] is not None
    assert result["largestGap"]["channel"] == "q.0"
    assert abs(result["largestGap"]["gap"] - 0.05) < 0.02
    assert "simulation is not real-robot evidence" in result["verdict"]


def test_sim_batch_benchmark(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    cells = [
        {"label": "clean", "seed": 42, "fault": {}},
        {"label": "slip", "seed": 43, "fault": {"gripper_slip": True}},
    ]
    result = _run_command("sim-batch-benchmark", {"cells": cells}, store.root)
    assert result["ok"] is True
    assert len(result["results"]) == 2
    assert result["summary"]["clean"]["successRate"] == 1.0
    assert "gripper_slip" in result["summary"]["slip"]["typicalAnomalies"]
    assert "not real-robot evidence" in result["note"]


def test_dashboard_generate(tmp_path):
    store = RunStore(str(tmp_path / "store"))
    run, _ = run_pick_place(SCENARIO_PICK_PLACE, {}, seed=10, store=store)
    out = tmp_path / "dashboard.html"
    result = _run_command("dashboard-generate", {"outPath": str(out)}, store.root)
    assert result["ok"] is True
    content = out.read_text(encoding="utf-8")
    assert "Robotic Harness — dashboard" in content
    assert run.id in content
