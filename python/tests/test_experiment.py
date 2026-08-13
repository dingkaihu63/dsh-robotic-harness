"""Tests for the experiment management module (plan chapter 15).

Covers spec-create validation/persistence, matrix expansion, a real small
matrix benchmark through the pick-place simulation, pure metrics aggregation,
ablation comparison and Markdown report generation.
"""

from __future__ import annotations

import json
import os

import pytest

from robotic_harness_worker import experiment as exp
from robotic_harness_worker.core import RunStore, WorkerError

# benchmark-start actually runs the MuJoCo pick-place simulation, so the
# whole suite is skipped when mujoco is unavailable (like test_simulation).
pytest.importorskip("mujoco")


def _spec(**overrides):
    """A fully-specified study definition for most tests."""
    spec = {
        "name": "grasp robustness",
        "researchQuestion": "perception offset 如何影响抓取成功率？",
        "hypothesis": "offset 增大会降低成功率",
        "independentVariables": [{"name": "perception_offset", "values": [0, 8, 16]}],
        "controlVariables": [{"name": "seed", "value": 42}],
        "baselines": [{"perception_offset": 0}],
        "seed": 42,
        "repetitions": 3,
        "statisticalMethod": "均值与成功率比较，无显著性检验",
    }
    spec.update(overrides)
    return spec


# ---------------------------------------------------------------------------
# experiment-spec-create
# ---------------------------------------------------------------------------


def test_spec_create_rejects_missing_independent_variables(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_experiment_spec_create({"name": "no-vars", "storeRoot": str(tmp_path)})


def test_spec_create_rejects_empty_values(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_experiment_spec_create(
            {
                "name": "bad-values",
                "independentVariables": [{"name": "x", "values": []}],
                "storeRoot": str(tmp_path),
            }
        )


def test_spec_create_rejects_bad_repetitions(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_experiment_spec_create({**_spec(repetitions=0), "storeRoot": str(tmp_path)})


def test_spec_create_saves_and_reports_open_questions(tmp_path):
    store_root = str(tmp_path / "rh")
    result = exp.cmd_experiment_spec_create(
        {
            "name": "grasp",
            "independentVariables": [{"name": "perception_offset", "values": [0, 8]}],
            "storeRoot": store_root,
        }
    )
    assert result["ok"] is True
    assert result["experimentId"].startswith("exp-")
    # persisted under <storeRoot>/.rh/experiments/<id>.json
    expected = os.path.join(store_root, ".rh", "experiments", f"{result['experimentId']}.json")
    assert result["path"] == expected
    assert os.path.exists(expected)
    with open(expected, encoding="utf-8") as handle:
        stored = json.load(handle)
    assert stored["name"] == "grasp"
    # defaults applied
    assert stored["seed"] == 42
    assert stored["repetitions"] == 3
    assert stored["primaryMetric"] == "success_rate"
    # incomplete study design surfaces in the agent checklist
    assert isinstance(result["openQuestions"], list)
    assert any("researchQuestion" in q for q in result["openQuestions"])
    assert result["agentChecklist"]["researchQuestion"] is False
    assert result["agentChecklist"]["control"] is False


def test_spec_create_full_spec_has_no_open_questions(tmp_path):
    result = exp.cmd_experiment_spec_create({**_spec(), "storeRoot": str(tmp_path)})
    assert result["ok"] is True
    assert result["openQuestions"] == []


# ---------------------------------------------------------------------------
# experiment-matrix-expand
# ---------------------------------------------------------------------------


def test_matrix_expand_cartesian_product_and_seeds(tmp_path):
    spec = _spec(
        independentVariables=[
            {"name": "perception_offset", "values": [0, 8]},
            {"name": "gripper_slip", "values": [False, True]},
        ],
        repetitions=2,
    )
    result = exp.cmd_experiment_matrix_expand({"spec": spec})
    assert result["ok"] is True
    assert result["total"] == 8  # 2 vars x 2 values x 2 repetitions
    assert len(result["cells"]) == 8
    # seeds increment: seed + flat index
    seeds = [c["seed"] for c in result["cells"]]
    assert seeds == list(range(42, 50))
    # 4 variable combinations, each repeated twice
    combos = [json.dumps(c["variables"], sort_keys=True) for c in result["cells"]]
    assert len(set(combos)) == 4
    assert result["cells"][0]["variables"] == {"perception_offset": 0, "gripper_slip": False}
    assert result["cells"][0]["repetition"] == 1
    assert result["cells"][1]["repetition"] == 2


def test_matrix_expand_truncates_with_max_cells(tmp_path):
    result = exp.cmd_experiment_matrix_expand({"spec": _spec(), "maxCells": 4})
    assert result["total"] == 4
    assert result["truncated"] is True
    assert result["requestedTotal"] == 9  # 3 values x 3 repetitions
    assert "截断" in result.get("note", "") or "truncated" in result.get("note", "")


def test_matrix_expand_requires_spec_or_experiment_id(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_experiment_matrix_expand({})


def test_matrix_expand_unknown_experiment(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_experiment_matrix_expand({"experimentId": "exp-00000000", "storeRoot": str(tmp_path)})


# ---------------------------------------------------------------------------
# benchmark-start (real simulation, small matrix)
# ---------------------------------------------------------------------------


def test_benchmark_start_runs_small_matrix(tmp_path):
    # storeRoot may be the workspace root or the .rh root; the module
    # normalizes both onto the RunStore root.
    store_root = str(tmp_path / "rh")
    rh_root = str(tmp_path / "rh" / ".rh")
    result = exp.cmd_benchmark_start(
        {
            "spec": {
                "name": "offset sweep",
                "independentVariables": [{"name": "perception_offset", "values": [0, 8]}],
                "repetitions": 1,
                "seed": 42,
            },
            "faultTemplates": {"perception_offset": {"perception_offset_px": ["__VALUE__", 0.0]}},
            "storeRoot": store_root,
        }
    )
    assert result["ok"] is True
    assert result["cells"] == 2
    assert len(result["runs"]) == 2
    for run in result["runs"]:
        assert run["cellId"].startswith("cell-")
        assert isinstance(run["success"], bool)
        assert run["runId"].startswith("run-")
        assert "success" in run["metrics"]
        # fault template substituted the cell value into the real fault key
        if run["cellId"] == "cell-0001":
            assert run["variables"] == {"perception_offset": 8}
            assert run["metrics"]["perceptionRoute"] in ("color", None)
    # runs persisted into the RunStore
    store = RunStore(rh_root)
    assert os.path.exists(store.run_dir(result["runs"][0]["runId"]))
    # summary aggregates the two runs
    assert result["summary"]["runs"] == 2
    assert 0.0 <= result["summary"]["successRate"] <= 1.0
    # inline spec -> no experiment JSON written
    assert result["path"] is None


def test_benchmark_start_updates_persisted_experiment(tmp_path):
    store_root = str(tmp_path / "rh")
    created = exp.cmd_experiment_spec_create({**_spec(repetitions=1), "storeRoot": store_root})
    experiment_id = created["experimentId"]
    result = exp.cmd_benchmark_start(
        {
            "experimentId": experiment_id,
            "faultTemplates": {"perception_offset": {"perception_offset_px": ["__VALUE__", 0.0]}},
            "storeRoot": store_root,
        }
    )
    assert result["ok"] is True
    assert result["path"] == created["path"]
    with open(created["path"], encoding="utf-8") as handle:
        stored = json.load(handle)
    assert stored["results"]["cells"] == 3
    assert stored["results"]["runs"][0]["runId"] == result["runs"][0]["runId"]


def test_benchmark_start_unknown_experiment(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_benchmark_start({"experimentId": "exp-00000000", "storeRoot": str(tmp_path)})


# ---------------------------------------------------------------------------
# metrics-compute (handcrafted runs)
# ---------------------------------------------------------------------------


def test_metrics_compute_handcrafted(tmp_path):
    runs = [
        {"cellId": "cell-0000", "variables": {"offset": 0}, "success": True,
         "metrics": {"success": True, "durationS": 1.0, "trackingErrorRms": 0.01}},
        {"cellId": "cell-0001", "variables": {"offset": 0}, "success": True,
         "metrics": {"success": True, "durationS": 1.2, "trackingErrorRms": 0.02}},
        {"cellId": "cell-0002", "variables": {"offset": 8}, "success": False,
         "metrics": {"success": False, "durationS": 1.5, "trackingErrorRms": 0.05}},
        {"cellId": "cell-0003", "variables": {"offset": 8}, "success": True,
         "metrics": {"success": True, "durationS": 1.1, "trackingErrorRms": 0.03}},
    ]
    result = exp.cmd_metrics_compute({"runs": runs})
    assert result["ok"] is True
    metrics = result["metrics"]
    assert metrics["successRate"] == 0.75  # 3/4
    assert metrics["runs"] == 4
    assert metrics["completionTimeMeanS"] == pytest.approx(1.2, abs=1e-3)
    assert metrics["trackingRmsMean"] == pytest.approx(0.0275, abs=1e-4)
    # per-variable grouping by value
    per_var = metrics["perVariable"]["offset"]
    assert per_var["0"]["successRate"] == 1.0 and per_var["0"]["runs"] == 2
    assert per_var["8"]["successRate"] == 0.5 and per_var["8"]["runs"] == 2
    # statistical-method declaration notes
    assert result["notes"] and any("numpy" in n for n in result["notes"])


def test_metrics_compute_from_persisted_experiment(tmp_path):
    store_root = str(tmp_path / "rh")
    created = exp.cmd_experiment_spec_create({**_spec(repetitions=1), "storeRoot": store_root})
    exp.cmd_benchmark_start(
        {
            "experimentId": created["experimentId"],
            "faultTemplates": {"perception_offset": {"perception_offset_px": ["__VALUE__", 0.0]}},
            "storeRoot": store_root,
        }
    )
    result = exp.cmd_metrics_compute({"experimentId": created["experimentId"], "storeRoot": store_root})
    assert result["ok"] is True
    assert result["runs"] == 3
    assert 0.0 <= result["metrics"]["successRate"] <= 1.0


def test_metrics_compute_requires_input(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_metrics_compute({})


# ---------------------------------------------------------------------------
# ablation-compare (handcrafted data)
# ---------------------------------------------------------------------------


def test_ablation_compare_direction(tmp_path):
    runs = []
    for offset, success in [(0, True), (0, True), (8, False), (8, False)]:
        runs.append(
            {
                "cellId": "cell-x",
                "variables": {"perception_offset": offset},
                "success": success,
                "metrics": {"success": success},
            }
        )
    result = exp.cmd_ablation_compare(
        {
            "runs": runs,
            "ablatedVariable": "perception_offset",
            "spec": {"independentVariables": [{"name": "perception_offset", "values": [0, 8]}]},
        }
    )
    assert result["ok"] is True
    # baseline = the first declared value (0)
    assert result["baseline"]["variables"] == {"perception_offset": 0}
    assert result["baseline"]["successRate"] == 1.0
    assert result["baseline"]["runs"] == 2
    assert len(result["groups"]) == 2
    assert result["effect"]["ablatedVariable"] == "perception_offset"
    assert result["effect"]["direction"] == "hurts"
    assert "成功率从" in result["effect"]["summary"]
    assert "相关性≠因果性" in result["note"]


def test_ablation_compare_improves_when_baseline_worst(tmp_path):
    runs = [
        {"variables": {"noise": 5}, "success": False, "metrics": {"success": False}},
        {"variables": {"noise": 5}, "success": False, "metrics": {"success": False}},
        {"variables": {"noise": 0}, "success": True, "metrics": {"success": True}},
        {"variables": {"noise": 0}, "success": True, "metrics": {"success": True}},
    ]
    result = exp.cmd_ablation_compare(
        {"runs": runs, "ablatedVariable": "noise", "baseline": {"variables": {"noise": 5}}}
    )
    assert result["effect"]["direction"] == "improves"


def test_ablation_compare_requires_ablated_variable(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_ablation_compare({"runs": [{"variables": {"x": 1}, "success": True, "metrics": {}}]})


# ---------------------------------------------------------------------------
# benchmark-report
# ---------------------------------------------------------------------------


def test_benchmark_report_generates_markdown(tmp_path):
    store_root = str(tmp_path / "rh")
    created = exp.cmd_experiment_spec_create({**_spec(repetitions=1), "storeRoot": store_root})
    exp.cmd_benchmark_start(
        {
            "experimentId": created["experimentId"],
            "faultTemplates": {"perception_offset": {"perception_offset_px": ["__VALUE__", 0.0]}},
            "storeRoot": store_root,
        }
    )
    out = str(tmp_path / "report.md")
    result = exp.cmd_benchmark_report(
        {"experimentId": created["experimentId"], "outPath": out, "storeRoot": store_root}
    )
    assert result["ok"] is True
    assert result["path"] == os.path.abspath(out)
    assert os.path.exists(out)
    with open(out, encoding="utf-8") as handle:
        content = handle.read()
    assert "实验报告" in content
    assert "需人工审阅" in content  # required human-review declaration
    assert "可复现性" in content
    assert "|" in content  # contains tables
    assert "成功率" in content
    assert "perception_offset" in content  # ablation section


def test_benchmark_report_from_inline_runs(tmp_path):
    runs = [
        {
            "cellId": "cell-0000",
            "variables": {"offset": 0},
            "seed": 42,
            "repetition": 1,
            "runId": "run-abc",
            "success": True,
            "metrics": {"success": True, "durationS": 1.0, "trackingErrorRms": 0.01},
        }
    ]
    out = str(tmp_path / "inline.md")
    result = exp.cmd_benchmark_report({"name": "inline study", "runs": runs, "outPath": out})
    assert result["ok"] is True
    with open(out, encoding="utf-8") as handle:
        content = handle.read()
    assert "inline study" in content
    assert "需人工审阅" in content


def test_benchmark_report_requires_out_path(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_benchmark_report({"name": "x", "runs": [{"success": True, "metrics": {"success": True}}]})


def test_benchmark_report_requires_runs_or_experiment_id(tmp_path):
    with pytest.raises(WorkerError):
        exp.cmd_benchmark_report({"name": "x", "outPath": str(tmp_path / "r.md")})


# ---------------------------------------------------------------------------
# module surface
# ---------------------------------------------------------------------------


def test_module_exports_commands_and_capabilities(tmp_path):
    assert set(exp.COMMANDS) == {
        "experiment-spec-create",
        "experiment-matrix-expand",
        "benchmark-start",
        "metrics-compute",
        "ablation-compare",
        "benchmark-report",
    }
    assert 2 <= len(exp.CAPABILITIES) <= 3
    for capability in exp.CAPABILITIES:
        assert capability["id"].startswith("experiment.")
        assert capability["provider"] == "robotic-harness-worker"
