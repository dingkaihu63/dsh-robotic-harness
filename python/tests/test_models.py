"""Tests for the embodied-model / VLM / VLA adapter module (models.py).

Covers: registry inventory, backend health detection, warmup, inference
(color segmentation + scripted IK policy + unreachable error), benchmarking,
rule-based capability routing, and the simulation policy rollout comparison.

Optional deps (cv2/PIL/mujoco) are skipped per-test so the pure-logic cases
still run on minimal environments (worker module contract, section 5).
"""

from __future__ import annotations

import numpy as np
import pytest

from robotic_harness_worker import models
from robotic_harness_worker.core import WorkerError


# ---------------------------------------------------------------------------
# model-inventory
# ---------------------------------------------------------------------------

def test_model_inventory_lists_builtins():
    result = models.COMMANDS["model-inventory"]({})
    assert result["ok"] is True
    ids = [m["id"] for m in result["models"]]
    assert {"demo.scripted_pick_place", "demo.color_segmentation", "demo.saliency_segmentation"} <= set(ids)
    by_kind = result["counts"]["byKind"]
    assert by_kind["policy"] >= 1
    assert by_kind["perception"] >= 2
    assert result["counts"]["total"] == len(result["models"])
    assert "demo.scripted_pick_place" in result["builtin"]
    # manifest carries the documented fields
    sample = next(m for m in result["models"] if m["id"] == "demo.scripted_pick_place")
    for key in ("id", "version", "kind", "provider", "description", "modalities", "output", "supportedEmbodiments", "risk", "backend"):
        assert key in sample, key
    assert sample["supportedEmbodiments"] == ["rh_planar_arm"]


def test_model_inventory_merges_registry_file(tmp_path):
    registry_path = tmp_path / "registry.json"
    registry_path.write_text(
        '{"models": [{"id": "ext.custom", "version": "2.0.0", "kind": "vlm", '
        '"module": "some_pkg", "entrypoint": "infer", "risk": "R3"}]}',
        encoding="utf-8",
    )
    result = models.COMMANDS["model-inventory"]({"registryPath": str(registry_path)})
    assert result["ok"] is True
    ids = {m["id"] for m in result["models"]}
    assert "ext.custom" in ids
    assert result["registryFileFound"] is True
    custom = next(m for m in result["models"] if m["id"] == "ext.custom")
    assert custom["kind"] == "vlm"
    assert custom["backend"] == "python-module"


# ---------------------------------------------------------------------------
# model-health
# ---------------------------------------------------------------------------

def test_model_health_builtin_ready():
    result = models.COMMANDS["model-health"]({"modelId": "demo.scripted_pick_place"})
    assert result["ok"] is True
    assert result["backend"] == "ready"
    assert result["details"]["builtin"] is True


def test_model_health_external_missing_module_unavailable():
    result = models.COMMANDS["model-health"](
        {
            "modelId": "ext.ghost",
            "models": [
                {"id": "ext.ghost", "kind": "vlm", "module": "nonexistent_module_xyz", "entrypoint": "infer"}
            ],
        }
    )
    assert result["ok"] is True
    assert result["backend"] == "unavailable"
    assert result["details"]["moduleImportable"] is False
    assert result["issues"], "an unavailable backend must explain itself"


def test_model_health_unknown_model_raises():
    with pytest.raises(WorkerError):
        models.COMMANDS["model-health"]({"modelId": "demo.does_not_exist"})


# ---------------------------------------------------------------------------
# model-warmup
# ---------------------------------------------------------------------------

def test_model_warmup_builtin():
    result = models.COMMANDS["model-warmup"]({"modelId": "demo.scripted_pick_place"})
    assert result["ok"] is True
    assert result["warmed"] is True
    assert result["latencyS"] is not None
    assert result["latencyS"] >= 0


def test_model_warmup_external_unavailable():
    result = models.COMMANDS["model-warmup"](
        {
            "modelId": "ext.ghost",
            "models": [{"id": "ext.ghost", "kind": "vlm", "module": "nonexistent_module_xyz"}],
        }
    )
    assert result["ok"] is True
    assert result["warmed"] is False
    assert result["backend"] == "unavailable"
    assert result["hint"]


# ---------------------------------------------------------------------------
# model-infer
# ---------------------------------------------------------------------------

def test_model_infer_color_segmentation(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("PIL")
    from PIL import Image

    image_path = tmp_path / "red.png"
    Image.fromarray(np.full((64, 64, 3), (255, 0, 0), dtype=np.uint8)).save(image_path)

    result = models.COMMANDS["model-infer"](
        {"modelId": "demo.color_segmentation", "input": {"imagePath": str(image_path), "color": "red"}}
    )
    assert result["ok"] is True
    assert result["backend"] == "builtin"
    perception = result["result"]
    assert perception["ok"] is True
    centroid_x, centroid_y = perception["centroidPx"]
    # a full-red 64x64 image puts the centroid at (31.5, 31.5)
    assert 24 <= centroid_x <= 40
    assert 24 <= centroid_y <= 40
    assert result["latencyMs"] >= 0
    assert result["trace"]["startedAt"] and result["trace"]["finishedAt"]
    assert "imagePath" in result["inputSummary"]
    assert result["outputSummary"]["ok"] is True


def test_model_infer_color_segmentation_missing_image_raises():
    with pytest.raises(WorkerError):
        models.COMMANDS["model-infer"]({"modelId": "demo.color_segmentation", "input": {}})


def test_model_infer_scripted_pick_place_reachable():
    result = models.COMMANDS["model-infer"](
        {
            "modelId": "demo.scripted_pick_place",
            "input": {"objectPose": [0.30, 0.19], "targetPose": [-0.16, 0.17]},
        }
    )
    assert result["ok"] is True
    joints = result["result"]["jointTargets"]
    assert len(joints) == 3
    assert all(isinstance(value, (int, float)) for value in joints)
    assert result["result"]["wristPose"]["x"] is not None
    assert result["result"]["placeJointTargets"]
    assert result["latencyMs"] >= 0


def test_model_infer_scripted_pick_place_unreachable():
    with pytest.raises(WorkerError) as excinfo:
        models.COMMANDS["model-infer"](
            {
                "modelId": "demo.scripted_pick_place",
                "input": {"objectPose": [10.0, 10.0], "targetPose": [-0.16, 0.17]},
            }
        )
    assert "unreachable" in str(excinfo.value)


def test_model_infer_external_unavailable_diagnostic():
    result = models.COMMANDS["model-infer"](
        {
            "modelId": "ext.ghost",
            "input": {"prompt": "hello"},
            "models": [{"id": "ext.ghost", "kind": "vlm", "module": "nonexistent_module_xyz", "entrypoint": "infer"}],
        }
    )
    assert result["ok"] is True
    assert result["backend"] == "unavailable"
    assert result["issues"]
    assert result["outputSummary"] is None


# ---------------------------------------------------------------------------
# model-benchmark
# ---------------------------------------------------------------------------

def test_model_benchmark_percentiles():
    result = models.COMMANDS["model-benchmark"](
        {
            "modelId": "demo.scripted_pick_place",
            "iterations": 15,
            "input": {"objectPose": [0.30, 0.19], "targetPose": [-0.16, 0.17]},
        }
    )
    assert result["ok"] is True
    assert result["iterations"] == 15
    stats = result["latencyMs"]
    for key in ("mean", "p50", "p90", "max", "min"):
        assert stats[key] is not None, key
        assert stats[key] >= 0, key
    assert stats["min"] <= stats["p50"] <= stats["max"]
    assert result["throughputHz"] > 0
    assert "environment" in result and "python" in result["environment"]


def test_model_benchmark_external_unavailable_skips():
    result = models.COMMANDS["model-benchmark"](
        {
            "modelId": "ext.ghost",
            "models": [{"id": "ext.ghost", "kind": "vlm", "module": "nonexistent_module_xyz"}],
        }
    )
    assert result["ok"] is True
    assert result["iterations"] == 0
    assert result["latencyMs"]["mean"] is None
    assert any("skipped" in note for note in result["notes"])


# ---------------------------------------------------------------------------
# capability-route-explain
# ---------------------------------------------------------------------------

def test_capability_route_pick_object_selects_policy():
    result = models.COMMANDS["capability-route-explain"]({"task": "pick_object"})
    assert result["ok"] is True
    kinds = {candidate["kind"] for candidate in result["candidates"]}
    assert "policy" in kinds
    assert result["selected"]["modelId"] == "demo.scripted_pick_place"
    assert result["selected"]["reasons"]
    for candidate in result["candidates"]:
        assert candidate["reasons"], candidate
        assert candidate["score"] is not None
    assert "规则路由" in result["note"]


def test_capability_route_modality_and_embodiment_filters():
    result = models.COMMANDS["capability-route-explain"](
        {"task": "pick_object", "modalities": ["rgb_image"], "embodiment": ["rh_planar_arm"]}
    )
    assert result["ok"] is True
    ids = {candidate["modelId"] for candidate in result["candidates"]}
    # rgb_image-only request excludes the robot_state policy and keeps perception models
    assert "demo.scripted_pick_place" not in ids
    assert {"demo.color_segmentation", "demo.saliency_segmentation"} <= ids


def test_capability_route_no_candidates_selected_null():
    result = models.COMMANDS["capability-route-explain"]({"task": "pick_object", "maxRisk": "R0"})
    assert result["ok"] is True
    assert result["candidates"] == []
    assert result["selected"] is None
    assert "无候选" in result["note"]


def test_capability_route_vqa_has_no_vlm():
    result = models.COMMANDS["capability-route-explain"]({"task": "vqa"})
    assert result["ok"] is True
    assert result["selected"] is None
    assert result["candidates"] == []


# ---------------------------------------------------------------------------
# policy-rollout-compare
# ---------------------------------------------------------------------------

def test_policy_rollout_compare_matrix_and_summary():
    pytest.importorskip("mujoco")
    result = models.COMMANDS["policy-rollout-compare"](
        {
            "policyA": {"modelId": "demo.scripted_pick_place"},
            "policyB": {"modelId": "demo.scripted_pick_place", "graspOffset": [18.0, 6.0]},
            "seeds": [42, 43],
        }
    )
    assert result["ok"] is True
    assert len(result["matrix"]) == 4  # 2 policies x 2 seeds x 1 default fault
    labels = {row["policy"] for row in result["matrix"]}
    assert labels == {"A", "B"}
    for row in result["matrix"]:
        assert row["success"] in (True, False)
        assert "trackingErrorRms" in row["metrics"]
        assert isinstance(row["anomalies"], list)
        assert row["fault"] is not None

    summary_a = result["summary"]["policyA"]
    summary_b = result["summary"]["policyB"]
    assert isinstance(summary_a["successRate"], float)
    assert isinstance(summary_b["successRate"], float)
    assert summary_a["successRate"] in (0.0, 0.5, 1.0)
    assert summary_b["successRate"] in (0.0, 0.5, 1.0)
    assert summary_a["avgTrackingRms"] is not None
    assert result["conclusion"]
    assert "仅仿真对比" in result["note"]
    # the offset-injected policy B must not beat the clean policy A here
    assert summary_a["successRate"] >= summary_b["successRate"]
