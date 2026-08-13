"""Tests for the vision_extra worker module (plan chapter 9).

Covers camera health checks, calibration inspection, pose/transform
validation, the perception-run / perception-compare wrappers over
:mod:`robotic_harness_worker.vision`, dataset profiling and failure-frame
annotation.

Image-based cases need cv2 + PIL (both installed in the worker env); the
whole module is skipped when either is missing.
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest

pytest.importorskip("PIL")
pytest.importorskip("cv2")

from PIL import Image

from robotic_harness_worker import vision_extra as ve
from robotic_harness_worker.core import WorkerError


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _solid(size: int, rgb: tuple[int, int, int]) -> np.ndarray:
    image = np.zeros((size, size, 3), dtype=np.uint8)
    image[:, :] = rgb
    return image


def _save_png(path, array_rgb) -> str:
    Image.fromarray(array_rgb).save(str(path))
    return str(path)


# ---------------------------------------------------------------------------
# camera-health-check
# ---------------------------------------------------------------------------

def test_camera_health_check_brightness_and_blur_classification(tmp_path):
    _save_png(tmp_path / "bright.png", _solid(64, (240, 240, 240)))
    _save_png(tmp_path / "dark.png", _solid(64, (20, 20, 20)))
    _save_png(tmp_path / "solid.png", _solid(64, (128, 128, 128)))
    rng = np.random.default_rng(0)
    noisy = np.clip(127 + rng.normal(0.0, 25.0, (64, 64, 3)), 0, 255).astype(np.uint8)
    _save_png(tmp_path / "noisy.png", noisy)

    result = ve.cmd_camera_health_check({"imageDir": str(tmp_path)})
    assert result["ok"] is True
    assert result["summary"]["imagesChecked"] == 4
    assert result["summary"]["avgBrightness"] is not None
    assert result["summary"]["avgBlur"] is not None

    by_name = {os.path.basename(img["path"]): img for img in result["images"]}
    assert set(by_name) == {"bright.png", "dark.png", "solid.png", "noisy.png"}

    bright = by_name["bright.png"]
    assert bright["meanBrightness"] > 215
    assert any(i["code"] == "brightness.too_bright" for i in bright["issues"])

    dark = by_name["dark.png"]
    assert dark["meanBrightness"] < 40
    assert any(i["code"] == "brightness.too_dark" for i in dark["issues"])

    solid = by_name["solid.png"]
    assert any(i["code"] == "blur.detected" for i in solid["issues"])
    assert solid["blurScore"] < ve.BLUR_SCORE_THRESHOLD

    noisy_entry = by_name["noisy.png"]
    assert not any(i["code"] == "blur.detected" for i in noisy_entry["issues"])
    assert noisy_entry["blurScore"] >= ve.BLUR_SCORE_THRESHOLD
    assert noisy_entry["noiseEstimate"] > 5
    assert solid["noiseEstimate"] < 2


def test_camera_health_check_resolution_mismatch(tmp_path):
    _save_png(tmp_path / "img.png", _solid(64, (100, 100, 100)))
    result = ve.cmd_camera_health_check({"imagePath": str(tmp_path / "img.png"), "expectedWidth": 640, "expectedHeight": 480})
    assert result["ok"] is False
    assert result["summary"]["expectedResolution"] == {"width": 640, "height": 480}
    assert any(i["code"] == "resolution.mismatch" for i in result["images"][0]["issues"])
    assert any(i["code"] == "resolution.mismatch" for i in result["summary"]["issues"])


def test_camera_health_check_missing_path_raises():
    with pytest.raises(WorkerError):
        ve.cmd_camera_health_check({})
    # a nonexistent single image is reported as a read-failure issue, not raised
    result = ve.cmd_camera_health_check({"imagePath": os.path.join("nope", "x.png")})
    assert result["ok"] is False
    assert len(result["readFailures"]) == 1
    assert any(i["code"] == "read.failed" for i in result["summary"]["issues"])


# ---------------------------------------------------------------------------
# calibration-inspect
# ---------------------------------------------------------------------------

def test_calibration_inspect_plausible(tmp_path):
    path = tmp_path / "calib.json"
    data = {
        "cameraMatrix": [[600.0, 0.0, 320.0], [0.0, 600.0, 320.0], [0.0, 0.0, 1.0]],
        "distCoeffs": [0.1, -0.05, 0.001, 0.002, 0.01],
        "imageSize": [640, 480],
        "reprojectionError": 0.3,
        "calibrationDate": "2024-01-01",
        "source": "checkerboard 12x9",
    }
    path.write_text(json.dumps(data), encoding="utf-8")
    result = ve.cmd_calibration_inspect({"path": str(path)})
    assert result["ok"] is True
    assert result["format"] == "json"
    assert result["summary"]["fx"] == 600.0
    assert result["summary"]["fy"] == 600.0
    assert result["summary"]["cx"] == 320.0
    assert result["summary"]["cy"] == 320.0
    assert result["summary"]["imageSize"] == [640, 480]
    assert result["summary"]["distortionCount"] == 5
    assert result["summary"]["reprojectionError"] == pytest.approx(0.3, abs=1e-6)
    assert result["summary"]["calibrationDate"] == "2024-01-01"
    assert result["summary"]["source"] == "checkerboard 12x9"
    assert result["verdict"] == "plausible"
    assert not any(i["severity"] == "error" for i in result["issues"])


def test_calibration_inspect_yaml(tmp_path):
    path = tmp_path / "calib.yaml"
    path.write_text(
        "fx: 600.0\nfy: 600.0\ncx: 320.0\ncy: 320.0\nimageSize: [640, 480]\nreprojectionError: 0.5\n",
        encoding="utf-8",
    )
    result = ve.cmd_calibration_inspect({"path": str(path)})
    assert result["format"] == "yaml"
    assert result["verdict"] == "plausible"
    assert result["summary"]["fx"] == 600.0


def test_calibration_inspect_needs_recalibration(tmp_path):
    path = tmp_path / "calib_bad.json"
    path.write_text(
        json.dumps({"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 320.0, "imageSize": [640, 480], "reprojectionError": 3.0}),
        encoding="utf-8",
    )
    result = ve.cmd_calibration_inspect({"path": str(path)})
    assert result["verdict"] == "needs-recalibration"
    assert any(i["code"] == "calibration.high_reprojection_error" for i in result["issues"])
    assert result["ok"] is True  # warning-level only


def test_calibration_inspect_unusual_distortion_count(tmp_path):
    path = tmp_path / "calib_dist.json"
    path.write_text(
        json.dumps(
            {"cameraMatrix": [[600, 0, 320], [0, 600, 320], [0, 0, 1]], "imageSize": [640, 480], "distortion": [1, 2, 3, 4, 5, 6, 7]}
        ),
        encoding="utf-8",
    )
    result = ve.cmd_calibration_inspect({"path": str(path)})
    assert result["summary"]["distortionCount"] == 7
    assert any(i["code"] == "calibration.distortion_count_unusual" for i in result["issues"])


def test_calibration_inspect_missing_intrinsics_incomplete(tmp_path):
    path = tmp_path / "calib_incomplete.json"
    path.write_text(json.dumps({"distortion": [0.1]}), encoding="utf-8")
    result = ve.cmd_calibration_inspect({"path": str(path)})
    assert result["verdict"] == "incomplete"
    assert result["ok"] is False
    assert any(i["code"] == "calibration.intrinsics_missing" for i in result["issues"])


def test_calibration_inspect_extrinsic_orthonormality(tmp_path):
    path = tmp_path / "calib_stereo.json"
    # valid stereo baseline transform: translation along x
    good = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 320.0, "reprojectionError": 0.4}
    path.write_text(json.dumps(good), encoding="utf-8")
    result = ve.cmd_calibration_inspect({"path": str(path)})
    assert result["ok"] is True


def test_calibration_inspect_missing_file(tmp_path):
    with pytest.raises(WorkerError):
        ve.cmd_calibration_inspect({"path": str(tmp_path / "nope.json")})
    with pytest.raises(WorkerError):
        ve.cmd_calibration_inspect({})


# ---------------------------------------------------------------------------
# pose-transform-validate
# ---------------------------------------------------------------------------

def test_pose_transform_validate_valid_and_reflection():
    identity = {"name": "identity", "matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}
    reflection = {"name": "reflection", "matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]}
    result = ve.cmd_pose_transform_validate({"transforms": [identity, reflection]})
    assert result["count"] == 2
    assert result["ok"] is False
    by_name = {t["name"]: t for t in result["transforms"]}
    assert by_name["identity"]["valid"] is True
    assert by_name["identity"]["rotationOrthonormal"] is True
    assert by_name["identity"]["determinantOk"] is True
    assert by_name["reflection"]["valid"] is False
    assert by_name["reflection"]["rotationOrthonormal"] is True  # RR^T=I holds for reflections
    assert by_name["reflection"]["determinantOk"] is False
    assert by_name["reflection"]["determinant"] == pytest.approx(-1.0, abs=1e-6)
    assert any(i["code"] == "rotation.determinant_not_one" for i in result["issues"])


def test_pose_transform_validate_quaternion():
    result = ve.cmd_pose_transform_validate(
        {"transform": {"name": "q", "position": [0.1, 0.2, 0.3], "quaternion": [1.0, 0.0, 0.0, 0.0]}}
    )
    entry = result["transforms"][0]
    assert entry["valid"] is True
    assert entry["quaternionUnit"] == pytest.approx(1.0, abs=1e-4)
    assert entry["quaternionUnitOk"] is True
    assert result["ok"] is True


def test_pose_transform_validate_non_unit_quaternion():
    result = ve.cmd_pose_transform_validate(
        {"transform": {"position": [0, 0, 0], "quaternion": [2.0, 0.0, 0.0, 0.0]}}
    )
    entry = result["transforms"][0]
    assert entry["valid"] is False
    assert entry["quaternionUnit"] == pytest.approx(2.0, abs=1e-4)
    assert any(i["code"] == "quaternion.not_unit" for i in result["issues"])


def test_pose_transform_validate_rpy():
    result = ve.cmd_pose_transform_validate({"transform": {"position": [0, 0, 0], "rpy": [0.1, 0.2, 0.3]}})
    entry = result["transforms"][0]
    assert entry["rotationOrthonormal"] is True
    assert entry["determinantOk"] is True
    assert entry["valid"] is True
    assert result["ok"] is True


def test_pose_transform_validate_malformed():
    with pytest.raises(WorkerError):
        ve.cmd_pose_transform_validate({"transform": {"matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}})  # 3x3, not 4x4
    with pytest.raises(WorkerError):
        ve.cmd_pose_transform_validate({"transform": {"position": [0, 0, 0]}})  # rotation missing
    with pytest.raises(WorkerError):
        ve.cmd_pose_transform_validate({})


# ---------------------------------------------------------------------------
# perception-run
# ---------------------------------------------------------------------------

def test_perception_run_red_image_routes_color(tmp_path):
    path = _save_png(tmp_path / "red.png", _solid(100, (255, 0, 0)))
    result = ve.cmd_perception_run({"imagePath": str(path), "route": "auto"})
    assert result["ok"] is True
    assert result["route"] == "color"
    assert result["result"]["method"] == "color_segmentation"
    cx, cy = result["result"]["centroidPx"]
    assert abs(cx - 49.5) < 2 and abs(cy - 49.5) < 2
    assert result["latencyMs"] >= 0
    assert result["artifacts"]["input"] == os.path.abspath(str(path))
    # default outPath: <name>.annotated.png next to the input
    assert result["outPath"] == os.path.abspath(str(tmp_path / "red.annotated.png"))
    assert os.path.exists(result["outPath"])
    assert "outPath 未提供" in result["note"]


def test_perception_run_with_outpath(tmp_path):
    path = _save_png(tmp_path / "red2.png", _solid(80, (255, 0, 0)))
    out = tmp_path / "out.png"
    result = ve.cmd_perception_run({"imagePath": str(path), "route": "color", "color": "red", "outPath": str(out)})
    assert result["ok"] is True
    assert result["route"] == "color"
    assert os.path.exists(out)


def test_perception_run_unknown_color(tmp_path):
    path = _save_png(tmp_path / "red3.png", _solid(80, (255, 0, 0)))
    with pytest.raises(WorkerError):
        ve.cmd_perception_run({"imagePath": str(path), "route": "color", "color": "purple"})


def test_perception_run_missing_image(tmp_path):
    with pytest.raises(WorkerError):
        ve.cmd_perception_run({"imagePath": str(tmp_path / "nope.png")})
    with pytest.raises(WorkerError):
        ve.cmd_perception_run({})


# ---------------------------------------------------------------------------
# perception-compare
# ---------------------------------------------------------------------------

def test_perception_compare_same_image_delta_zero(tmp_path):
    path = _save_png(tmp_path / "cmp.png", _solid(100, (255, 0, 0)))
    result = ve.cmd_perception_compare({"imagePathA": str(path), "imagePathB": str(path), "method": "color"})
    assert result["ok"] is True
    assert result["resultA"]["ok"] is True and result["resultB"]["ok"] is True
    assert result["deltaPx"] == pytest.approx(0.0, abs=1e-6)
    assert result["iou"] == pytest.approx(1.0, abs=1e-3)
    assert result["agreement"] is True


def test_perception_compare_with_ground_truth(tmp_path):
    path = _save_png(tmp_path / "cmp2.png", _solid(100, (255, 0, 0)))
    result = ve.cmd_perception_compare(
        {"imagePathA": str(path), "imagePathB": str(path), "method": "color", "groundTruthCentroidPx": [49.5, 49.5]}
    )
    assert result["groundTruthErrorsPx"]["a"] == pytest.approx(0.0, abs=1e-3)
    assert result["groundTruthErrorsPx"]["b"] == pytest.approx(0.0, abs=1e-3)


def test_perception_compare_missing_args():
    with pytest.raises(WorkerError):
        ve.cmd_perception_compare({"imagePathA": "x.png"})
    with pytest.raises(WorkerError):
        ve.cmd_perception_compare({})


# ---------------------------------------------------------------------------
# image-dataset-profile
# ---------------------------------------------------------------------------

def test_image_dataset_profile_mixed_sizes(tmp_path):
    Image.new("RGB", (100, 50), (255, 0, 0)).save(tmp_path / "a.png")
    Image.new("RGB", (200, 100), (0, 255, 0)).save(tmp_path / "b.jpg")
    Image.new("RGB", (100, 50), (0, 0, 255)).save(tmp_path / "c.png")
    (tmp_path / "corrupt.png").write_bytes(b"this is not a real png file at all")

    result = ve.cmd_image_dataset_profile({"path": str(tmp_path)})
    assert result["ok"] is False  # corrupt file -> error issue
    stats = result["resolutionStats"]
    assert stats["minW"] == 100 and stats["minH"] == 50
    assert stats["maxW"] == 200 and stats["maxH"] == 100
    assert stats["commonW"] == 100 and stats["commonH"] == 50
    assert result["count"] == 3
    assert result["countByExt"][".png"] == 3  # a.png + c.png + corrupt.png
    assert result["countByExt"][".jpg"] == 1
    assert len(result["corruptFiles"]) == 1
    assert result["corruptFiles"][0]["path"].endswith("corrupt.png")
    codes = {i["code"] for i in result["issues"]}
    assert "corrupt.files" in codes
    assert "resolution.inconsistent" in codes
    assert result["totalSize"] > 0


def test_image_dataset_profile_not_a_directory(tmp_path):
    file_path = tmp_path / "file.txt"
    file_path.write_text("x", encoding="utf-8")
    with pytest.raises(WorkerError):
        ve.cmd_image_dataset_profile({"path": str(file_path)})
    with pytest.raises(WorkerError):
        ve.cmd_image_dataset_profile({})


# ---------------------------------------------------------------------------
# annotate-failure-frame
# ---------------------------------------------------------------------------

def test_annotate_failure_frame(tmp_path):
    src = _save_png(tmp_path / "frame.png", _solid(200, (100, 100, 100)))
    out = tmp_path / "annotated.png"
    detections = [
        {"bbox": [10, 10, 40, 30], "label": "obj1"},
        {"centroidPx": [120, 80], "label": "pt"},
        {"bbox": [150, 150, 20, 20], "color": [0, 255, 0]},
    ]
    result = ve.cmd_annotate_failure_frame({"imagePath": str(src), "detections": detections, "outPath": str(out)})
    assert result["ok"] is True
    assert result["annotationsDrawn"] == 3
    assert result["inputPath"] == os.path.abspath(str(src))
    assert result["outPath"] == os.path.abspath(str(out))
    assert os.path.exists(out)
    with Image.open(out) as handle:
        assert handle.size == (200, 200)


def test_annotate_failure_frame_no_detections(tmp_path):
    src = _save_png(tmp_path / "frame2.png", _solid(200, (50, 50, 50)))
    result = ve.cmd_annotate_failure_frame({"imagePath": str(src)})
    assert result["ok"] is True
    assert result["annotationsDrawn"] == 0
    assert result["detectionsRequested"] == 0
    assert os.path.exists(result["outPath"])
    assert result["outPath"].endswith("frame2.annotated.png")
    assert "outPath 未提供" in result["note"]


def test_annotate_failure_frame_missing_image(tmp_path):
    with pytest.raises(WorkerError):
        ve.cmd_annotate_failure_frame({"imagePath": str(tmp_path / "nope.png"), "outPath": str(tmp_path / "o.png")})
    with pytest.raises(WorkerError):
        ve.cmd_annotate_failure_frame({})


# ---------------------------------------------------------------------------
# module interface
# ---------------------------------------------------------------------------

def test_command_registry_and_capabilities():
    for name in (
        "camera-health-check",
        "calibration-inspect",
        "pose-transform-validate",
        "perception-run",
        "perception-compare",
        "image-dataset-profile",
        "annotate-failure-frame",
    ):
        assert name in ve.COMMANDS
        assert callable(ve.COMMANDS[name])
    assert len(ve.CAPABILITIES) == 7
    for capability in ve.CAPABILITIES:
        assert capability["id"] and capability["provider"] == "robotic-harness-worker"
