"""Pure-logic tests for Robotic Harness worker: assets, IK, camera, data quality.

Simulation tests live in ``test_simulation.py`` (they need mujoco, which is
present in the recommended environment but is skipped when missing).
"""

from __future__ import annotations

import json
import math
import os
import tempfile

import pytest

from robotic_harness_worker import assets, data_quality
from robotic_harness_worker.simulation import CameraModel, PlanarArm, SCENARIO_PICK_PLACE

FIXTURES = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "fixtures",
    "robot_assets",
)

GOOD_URDF = os.path.join(FIXTURES, "rh_arm.urdf")
BROKEN_URDF = os.path.join(FIXTURES, "rh_arm_broken.urdf")


def test_urdf_valid_asset_inspection_has_no_errors():
    inspection = assets.inspect_urdf(GOOD_URDF)
    summary = inspection.summary
    assert summary["linkCount"] == 5
    assert summary["jointCount"] == 4
    assert summary["rootLinks"] == ["base_link"]
    errors = [i for i in inspection.issues if i.severity == "error"]
    assert errors == [], [i.to_dict() for i in errors]
    assert summary["jointTypes"] == {"fixed": 1, "revolute": 3}


def test_urdf_inertial_checks_catch_bad_values():
    inspection = assets.inspect_urdf(BROKEN_URDF)
    codes = {i.code for i in inspection.issues}
    assert "inertial.mass_non_positive" in codes  # zero-mass link
    assert "inertial.zero" in codes  # zero inertia matrix
    assert "urdf.missing_limit" in codes  # revolute joint without limit
    assert "urdf.duplicate_link" in codes  # duplicated link name
    assert "urdf.missing_inertial" in codes  # link without inertial


def test_validate_urdf_verdict():
    result = assets.validate_urdf(GOOD_URDF)
    assert result["ok"] is True
    broken = assets.validate_urdf(BROKEN_URDF)
    assert broken["ok"] is False
    assert broken["issueCounts"]["error"] >= 1


def test_inspect_unsupported_format():
    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as handle:
        handle.write(b"x")
        path = handle.name
    try:
        with pytest.raises(ValueError, match="unsupported asset format"):
            assets.inspect_asset(path)
    finally:
        os.unlink(path)


def test_planar_arm_ik_fk_roundtrip():
    arm = PlanarArm([0.22, 0.19], [0.0, 0.0, 0.45], 0.065, SCENARIO_PICK_PLACE["arm"]["jointRanges"])
    for x, z, phi in [(0.24, 0.24, 2.2), (-0.16, 0.20, math.pi), (0.05, 0.62, math.pi), (0.243, 0.400, 2.285)]:
        solutions = arm.ik_solutions(x, z, phi)
        assert solutions, f"({x}, {z}, {phi}) should be reachable"
        for solution in solutions:
            fx, fz, fphi = arm.fk(solution)
            assert fx == pytest.approx(x, abs=1e-9)
            assert fz == pytest.approx(z, abs=1e-9)
            assert math.sin(fphi) == pytest.approx(math.sin(phi), abs=1e-9)
            assert math.cos(fphi) == pytest.approx(math.cos(phi), abs=1e-9)


def test_planar_arm_ik_out_of_reach():
    arm = PlanarArm([0.22, 0.19], [0.0, 0.0, 0.45], 0.065, SCENARIO_PICK_PLACE["arm"]["jointRanges"])
    assert arm.ik_solutions(0.0, 0.90, 0.0) == []  # beyond reach
    assert arm.ik_solutions(0.0, 0.451, 0.0) == []  # inside dead zone


def test_planar_arm_q2_wrapping():
    arm = PlanarArm([0.22, 0.19], [0.0, 0.0, 0.45], 0.065, SCENARIO_PICK_PLACE["arm"]["jointRanges"])
    for solution in arm.ik_solutions(-0.16, 0.20, math.pi):
        assert -math.pi <= solution[2] <= math.pi


def test_camera_roundtrip():
    camera = CameraModel(SCENARIO_PICK_PLACE)
    point = [0.30, 0.0, 0.19]
    px, py = camera.px_from_world(point)
    assert 0 <= px <= camera.width
    assert 0 <= py <= camera.height
    back = camera.world_from_px(px, py, 0.19)
    assert back[0] == pytest.approx(point[0], abs=1e-6)
    assert back[2] == pytest.approx(point[2], abs=1e-6)


def test_data_quality_audit_csv(tmp_path):
    path = tmp_path / "sample.csv"
    path.write_text(
        "t,pos,vel\n"
        "0.0,1.0,0.1\n"
        "0.1,2.0,0.2\n"
        "0.1,3.0,0.3\n"  # duplicate timestamp
        "0.05,4.0,0.4\n"  # out of order
        ",5.0,\n"  # missing t and vel
        "0.3,nan,0.6\n"  # non-finite value
        "0.4,7.0,0.7\n",
        encoding="utf-8",
    )
    result = data_quality.audit(str(path))
    assert result["ok"] is False
    assert result["timestamps"]["duplicates"] >= 1
    assert result["timestamps"]["outOfOrder"] >= 1
    assert result["timestamps"]["missing"] >= 1
    assert result["channels"]["pos"]["nonFinite"] >= 1
    codes = {i["code"] for i in result["issues"]}
    assert "ts.out_of_order" in codes
    assert "ts.duplicates" in codes


def test_data_quality_audit_jsonl(tmp_path):
    path = tmp_path / "sample.jsonl"
    path.write_text(
        '{"t": 0.0, "q0": 1.0}\n'
        '{"t": 0.1, "q0": 1.1}\n'
        '{"t": 0.2, "q0": 1.1}\n'
        'not json\n'
        '{"t": 0.3, "q0": 1.2}\n',
        encoding="utf-8",
    )
    result = data_quality.audit(str(path))
    assert result["parseErrors"] == 1
    assert result["channels"]["q0"]["constant"] is False


def test_data_quality_missing_time_column(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("x,y\n1,2\n", encoding="utf-8")
    result = data_quality.audit(str(path))
    assert result["ok"] is False
    assert result["issues"][0]["code"] == "format.invalid"


def test_capability_list_shape():
    from robotic_harness_worker import WORKER_CAPABILITIES

    assert len(WORKER_CAPABILITIES) >= 6
    for capability in WORKER_CAPABILITIES:
        assert capability["id"]
        assert capability["kind"]
        assert capability["risk"]
