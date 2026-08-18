"""Vision perception health checks, calibration inspection and pose validation.

Implements the plan's chapter 9 worker tooling on top of the existing
:mod:`robotic_harness_worker.vision` perception routes:

- ``camera-health-check`` — per-image brightness / blur / noise / resolution
  quality metrics for one image or a sampled directory.
- ``calibration-inspect`` — structural validation of a JSON/YAML camera
  calibration file (intrinsics, distortion, reprojection error, extrinsics).
- ``pose-transform-validate`` — numeric validity of 4x4 homogeneous
  transforms / poses (orthonormality, determinant, finite translation,
  unit quaternion).
- ``perception-run`` — wrapper over :func:`vision.route_perception` with
  latency/artifact records and an annotated output image.
- ``perception-compare`` — run the same perception method on two images and
  compare centroids / masks (deltaPx, IoU, agreement).
- ``image-dataset-profile`` — directory scan: resolution stats, size by
  extension, corrupt files (PIL header-only reads).
- ``annotate-failure-frame`` — draw bbox / centroid / label overlays onto a
  failure frame, never modifying the source image.

Design notes:

- cv2 / PIL / PyYAML are optional imports, imported lazily with the same
  pattern as :mod:`robotic_harness_worker.vision`.  Missing optional backends
  raise :class:`WorkerError` with install instructions; numpy-only fallbacks
  are provided for the quality metrics so the health check still runs.
- PyYAML (``yaml``) is used to parse YAML calibration files; when it is
  missing, YAML files are rejected with an explicit message (JSON still
  works).  No new third-party dependency is introduced: the environment
  already ships PyYAML.
- All results are JSON-safe (no numpy types), numbers are rounded, and paths
  are absolute.  Annotated images are always written to ``outPath`` or, when
  omitted, to ``<name>.annotated.png`` next to the input image.
"""

from __future__ import annotations

import json
import math
import os
import time
from collections import Counter
from typing import Any, Optional

import numpy as np

from . import vision
from .core import WorkerError

try:
    import cv2  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    cv2 = None  # type: ignore[assignment]

try:
    from PIL import Image  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    Image = None  # type: ignore[assignment]

try:
    import yaml  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    yaml = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp", ".tif", ".tiff"}

#: mean brightness below this is treated as underexposed
TOO_DARK = 40.0
#: mean brightness above this is treated as overexposed
TOO_BRIGHT = 215.0
#: Laplacian variance below this flags blur / featureless content
BLUR_SCORE_THRESHOLD = 50.0

#: legal OpenCV distortion coefficient counts (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1..s4]]])
DISTORTION_COUNTS = {4, 5, 8, 12, 14}

#: HSV presets mirrored from vision.py so artifact masks can be recomputed
COLOR_PRESETS = {
    "red": (0, 10, 60, 255, 40, 255),
    "blue": (95, 130, 60, 255, 40, 255),
    "green": (35, 85, 60, 255, 40, 255),
    "yellow": (15, 35, 60, 255, 40, 255),
}

COLOR_NAMES = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
    "cyan": (255, 255, 0),
    "magenta": (255, 0, 255),
    "white": (255, 255, 255),
    "black": (0, 0, 0),
}  # values are BGR


# ---------------------------------------------------------------------------
# optional backend helpers
# ---------------------------------------------------------------------------

def _require_cv2() -> Any:
    if cv2 is None:
        raise WorkerError(
            "opencv-python is not installed in the worker environment; "
            "install it (pip install opencv-python-headless) to run this command"
        )
    return cv2


def _require_pil() -> Any:
    if Image is None:
        raise WorkerError("Pillow (PIL) is not installed; install it (pip install Pillow) to read image sizes")
    return Image


def _require_yaml() -> Any:
    if yaml is None:
        raise WorkerError("PyYAML is not installed; install it (pip install PyYAML) to parse YAML calibration files")
    return yaml


def _read_gray(path: str) -> np.ndarray:
    """Load an image as a uint8 grayscale array (cv2, else PIL fallback)."""
    if not os.path.exists(path):
        raise WorkerError(f"image not found: {path}")
    if cv2 is not None:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)  # type: ignore[attr-defined]
        if image is None:
            raise WorkerError(f"cannot decode image with OpenCV: {path}")
        return image
    pil = _require_pil()
    try:
        with pil.open(path) as handle:
            return np.asarray(handle.convert("L"))
    except Exception as error:  # noqa: BLE001 - report any decode failure
        raise WorkerError(f"cannot decode image with PIL: {path}: {error}") from error


def _read_rgb(path: str) -> np.ndarray:
    """Load an image as an RGB (H,W,3) uint8 array for the perception routes."""
    if not os.path.exists(path):
        raise WorkerError(f"image not found: {path}")
    if cv2 is not None:
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)  # type: ignore[attr-defined]
        if bgr is None:
            raise WorkerError(f"cannot decode image with OpenCV: {path}")
        if bgr.ndim == 2:
            bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)  # type: ignore[attr-defined]
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # type: ignore[attr-defined]
    pil = _require_pil()
    try:
        with pil.open(path) as handle:
            return np.asarray(handle.convert("RGB"))
    except Exception as error:  # noqa: BLE001 - report any decode failure
        raise WorkerError(f"cannot decode image with PIL: {path}: {error}") from error


def _read_bgr(path: str) -> np.ndarray:
    cv2 = _require_cv2()
    if not os.path.exists(path):
        raise WorkerError(f"image not found: {path}")
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)  # type: ignore[attr-defined]
    if bgr is None:
        raise WorkerError(f"cannot decode image with OpenCV: {path}")
    if bgr.ndim == 2:
        bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)  # type: ignore[attr-defined]
    return bgr


def _imwrite(path: str, image_bgr: np.ndarray) -> str:
    """Write a BGR image via cv2.imencode (handles non-ASCII paths)."""
    cv2 = _require_cv2()
    ext = os.path.splitext(path)[1] or ".png"
    ok_enc, buf = cv2.imencode(ext, image_bgr)  # type: ignore[attr-defined]
    if not ok_enc:
        raise WorkerError(f"cannot encode image for extension {ext!r}")
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(buf.tobytes())
    return os.path.abspath(path)


def _default_annotated_path(image_path: str) -> str:
    base, _ = os.path.splitext(image_path)
    return base + ".annotated.png"


# ---------------------------------------------------------------------------
# camera-health-check
# ---------------------------------------------------------------------------

def _blur_score(gray: np.ndarray) -> float:
    """Laplacian variance (cv2) or a mean-squared adjacent-diff fallback."""
    if cv2 is not None:
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())  # type: ignore[attr-defined]
    g = gray.astype(np.float64)
    dx = np.zeros_like(g)
    dy = np.zeros_like(g)
    dx[:, 1:] = g[:, 1:] - g[:, :-1]
    dy[1:, :] = g[1:, :] - g[:-1, :]
    return float(np.mean(dx * dx + dy * dy))


def _noise_estimate(gray: np.ndarray) -> float:
    """Std of the high-frequency residual (image minus 3x3 box-smoothed)."""
    g = gray.astype(np.float64)
    if cv2 is not None:
        smoothed = cv2.boxFilter(g, -1, (3, 3))  # type: ignore[attr-defined]
        return float(np.std(g - smoothed))
    padded = np.pad(g, 1, mode="edge")
    acc = np.zeros_like(g)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            acc += padded[1 + dy : 1 + dy + g.shape[0], 1 + dx : 1 + dx + g.shape[1]]
    return float(np.std(g - acc / 9.0))


def _motion_blur_score(gray: np.ndarray) -> Optional[float]:
    """Directional concentration of strong gradients (0..1, None if sparse).

    Strong-gradient pixels are binned by gradient angle (0..180 deg); a high
    peak fraction indicates a dominant gradient direction, which is what
    motion blur looks like.  Optional heuristic, numpy-only.
    """
    g = gray.astype(np.float64)
    gx = np.zeros_like(g)
    gy = np.zeros_like(g)
    gx[:, 1:] = g[:, 1:] - g[:, :-1]
    gy[1:, :] = g[1:, :] - g[:-1, :]
    mag = np.sqrt(gx * gx + gy * gy)
    strong = mag > 30.0
    count = int(np.count_nonzero(strong))
    if count < 100:
        return None
    angles = np.degrees(np.arctan2(gy[strong], gx[strong])) % 180.0
    hist, _ = np.histogram(angles, bins=np.linspace(0.0, 180.0, 19))
    return round(float(hist.max()) / float(hist.sum()), 4)


def _image_quality_issues(
    path: str,
    width: int,
    height: int,
    mean_brightness: float,
    blur: float,
    expected_w: Optional[int],
    expected_h: Optional[int],
) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    name = os.path.basename(path)
    if mean_brightness < TOO_DARK:
        issues.append(
            {
                "severity": "warning",
                "code": "brightness.too_dark",
                "message": f"{name}: mean brightness {mean_brightness:.1f} < {TOO_DARK:.0f} (underexposed?)",
            }
        )
    elif mean_brightness > TOO_BRIGHT:
        issues.append(
            {
                "severity": "warning",
                "code": "brightness.too_bright",
                "message": f"{name}: mean brightness {mean_brightness:.1f} > {TOO_BRIGHT:.0f} (overexposed?)",
            }
        )
    if blur < BLUR_SCORE_THRESHOLD:
        issues.append(
            {
                "severity": "warning",
                "code": "blur.detected",
                "message": f"{name}: Laplacian variance {blur:.1f} < {BLUR_SCORE_THRESHOLD:.0f} (blurry or featureless)",
            }
        )
    if expected_w is not None and width != expected_w:
        issues.append(
            {
                "severity": "error",
                "code": "resolution.mismatch",
                "message": f"{name}: width {width} != expected {expected_w}",
            }
        )
    if expected_h is not None and height != expected_h:
        issues.append(
            {
                "severity": "error",
                "code": "resolution.mismatch",
                "message": f"{name}: height {height} != expected {expected_h}",
            }
        )
    return issues


def cmd_camera_health_check(args: dict[str, Any]) -> dict[str, Any]:
    """Compute brightness/blur/noise quality metrics for image(s).

    Args (``imagePath`` XOR ``imageDir``)::

        {"imagePath": "...", "expectedWidth"?: 640, "expectedHeight"?: 480}
        {"imageDir": "...", "expectedWidth"?: 640, "expectedHeight"?: 480}
    """
    image_path = args.get("imagePath")
    image_dir = args.get("imageDir")
    if not image_path and not image_dir:
        raise WorkerError("missing required argument: provide 'imagePath' or 'imageDir'")
    if image_path and image_dir:
        raise WorkerError("provide only one of 'imagePath' or 'imageDir'")
    if image_path and os.path.isdir(image_path):
        image_dir, image_path = image_path, None

    def _to_int(value: Any) -> Optional[int]:
        return int(value) if value is not None else None

    expected_w = _to_int(args.get("expectedWidth"))
    expected_h = _to_int(args.get("expectedHeight"))
    max_sample = 20

    paths: list[str] = []
    sampled = False
    total = 0
    if image_path:
        paths = [image_path]
    else:
        if not os.path.isdir(image_dir):
            raise WorkerError(f"imageDir not found or not a directory: {image_dir}")
        candidates = sorted(
            os.path.join(image_dir, name)
            for name in os.listdir(image_dir)
            if os.path.isfile(os.path.join(image_dir, name)) and os.path.splitext(name)[1].lower() in IMAGE_EXTS
        )
        total = len(candidates)
        sampled = total > max_sample
        paths = candidates[:max_sample]

    images: list[dict[str, Any]] = []
    read_failures: list[dict[str, Any]] = []
    for path in paths:
        full = os.path.abspath(path)
        try:
            gray = _read_gray(full)
            height, width = int(gray.shape[0]), int(gray.shape[1])
            mean_b = float(gray.mean())
            std_b = float(gray.std())
            blur = _blur_score(gray)
            noise = _noise_estimate(gray)
            motion = _motion_blur_score(gray)
            issues = _image_quality_issues(full, width, height, mean_b, blur, expected_w, expected_h)
            images.append(
                {
                    "path": full,
                    "width": width,
                    "height": height,
                    "meanBrightness": round(mean_b, 2),
                    "stdBrightness": round(std_b, 2),
                    "blurScore": round(blur, 2),
                    "noiseEstimate": round(noise, 2),
                    "motionBlurScore": motion,
                    "issues": issues,
                }
            )
        except WorkerError as error:
            read_failures.append({"path": full, "message": str(error)})

    # aggregate per-code issues into the summary
    code_counts: Counter = Counter()
    code_severity: dict[str, str] = {}
    example: dict[str, str] = {}
    for entry in images:
        for issue in entry["issues"]:
            code_counts[issue["code"]] += 1
            code_severity[issue["code"]] = issue["severity"]
            example.setdefault(issue["code"], issue["message"])
    summary_issues = [
        {
            "severity": code_severity[code],
            "code": code,
            "message": example[code] if count == 1 else f"{example[code]} (+{count - 1} more)",
        }
        for code, count in code_counts.items()
    ]
    for failure in read_failures:
        summary_issues.append(
            {
                "severity": "error",
                "code": "read.failed",
                "message": f"{os.path.basename(failure['path'])}: {failure['message']}",
            }
        )
    if not images and not read_failures:
        summary_issues.append({"severity": "warning", "code": "images.none_found", "message": "no matching image files found"})

    summary: dict[str, Any] = {
        "imagesChecked": len(images),
        "readFailures": len(read_failures),
        "avgBlur": round(sum(float(i["blurScore"]) for i in images) / len(images), 2) if images else None,
        "avgBrightness": round(sum(float(i["meanBrightness"]) for i in images) / len(images), 2) if images else None,
        "minResolution": (
            {"width": min(int(i["width"]) for i in images), "height": min(int(i["height"]) for i in images)} if images else None
        ),
        "issues": summary_issues,
    }
    if expected_w is not None or expected_h is not None:
        summary["expectedResolution"] = {"width": expected_w, "height": expected_h}
    if sampled:
        summary["sampled"] = True
        summary["totalImages"] = total

    ok = not any(i["severity"] == "error" for i in summary_issues)
    return {
        "ok": ok,
        "images": images,
        "readFailures": read_failures,
        "summary": summary,
        "note": "启发式质量指标：blurScore 为拉普拉斯方差（阈值 50），noiseEstimate 为高频残差标准差，motionBlurScore 为梯度方向集中度（可选）",
        "inputArgs": {"imagePath": image_path, "imageDir": image_dir, "expectedWidth": expected_w, "expectedHeight": expected_h},
    }


# ---------------------------------------------------------------------------
# calibration-inspect
# ---------------------------------------------------------------------------

def _pick(mapping: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return mapping[key]
    return None


def _wrappers(data: dict[str, Any]) -> list[dict[str, Any]]:
    """Top-level dict plus common nested wrapper dicts."""
    result = [data]
    for key in ("intrinsic", "intrinsics", "camera", "calibration", "cam0", "camera0", "camera_0"):
        value = data.get(key)
        if isinstance(value, dict):
            result.append(value)
    return result


def _parse_image_size(value: Any) -> Optional[tuple[int, int]]:
    if value is None:
        return None
    if isinstance(value, dict):
        w = value.get("width") or value.get("w")
        h = value.get("height") or value.get("h")
        if w is not None and h is not None:
            try:
                return (int(w), int(h))
            except (TypeError, ValueError):
                return None
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return (int(value[0]), int(value[1]))
        except (TypeError, ValueError):
            return None
    if isinstance(value, str):
        parts = value.lower().replace("x", " ").replace(",", " ").split()
        if len(parts) >= 2:
            try:
                return (int(parts[0]), int(parts[1]))
            except ValueError:
                return None
    return None


def _as_matrix(value: Any, label: str) -> np.ndarray:
    """Coerce a calibration matrix value into a numeric ndarray.

    Accepts nested lists AND the OpenCV FileStorage ``{rows, cols, dt, data}``
    dict form; raises a structured WorkerError instead of a raw ValueError.
    """
    if isinstance(value, dict):
        data = value.get("data")
        if isinstance(data, list):
            value = data
        else:
            raise WorkerError(f"{label} must be a numeric array; got a dict without a 'data' list")
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise WorkerError(f"{label} must be a numeric array, got {type(value).__name__}") from error
    if arr.ndim == 0:
        raise WorkerError(f"{label} must be a numeric array, got a scalar")
    return arr


def _extract_intrinsics(data: dict[str, Any]) -> tuple[Optional[np.ndarray], dict[str, float]]:
    """Return (K or None, flat dict of fx/fy/cx/cy when found)."""
    for wrapper in _wrappers(data):
        matrix = _pick(wrapper, "cameraMatrix", "camera_matrix", "intrinsics", "K", "mtx", "cameraMatrixList")
        if matrix is not None:
            arr = _as_matrix(matrix, "cameraMatrix")
            if arr.ndim == 1 and arr.size == 9:
                arr = arr.reshape(3, 3)
            if arr.ndim == 1 and arr.size == 4:
                fx, fy, cx, cy = (float(v) for v in arr)
                return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]), {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
            if arr.shape == (3, 3):
                return arr, {"fx": float(arr[0, 0]), "fy": float(arr[1, 1]), "cx": float(arr[0, 2]), "cy": float(arr[1, 2])}
            raise WorkerError(f"cameraMatrix must be 3x3, 9 or 4 elements, got shape {arr.shape}")
        fx = _pick(wrapper, "fx", "focalLengthX", "focal_length_x", "focal_x")
        if fx is not None:
            fy = _pick(wrapper, "fy", "focalLengthY", "focal_length_y", "focal_y")
            cx = _pick(wrapper, "cx", "principalPointX", "principal_point_x", "ppx")
            cy = _pick(wrapper, "cy", "principalPointY", "principal_point_y", "ppy")
            if fy is not None and cx is not None and cy is not None:
                fx, fy, cx, cy = (float(fx), float(fy), float(cx), float(cy))
                return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]), {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
    return None, {}


def _extract_distortion(data: dict[str, Any]) -> tuple[Optional[list[float]], Optional[int]]:
    for wrapper in _wrappers(data):
        value = _pick(wrapper, "distortion", "distCoeffs", "dist_coeffs", "distortionCoefficients", "distortion_coefficients", "d")
        if value is None:
            continue
        if isinstance(value, dict):
            ordered = [value.get(k) for k in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4")]
            values = [float(v) for v in ordered if v is not None]
            return values, len(values)
        if isinstance(value, (list, tuple)):
            values = [float(v) for v in value]
            return values, len(values)
        return None, None
    return None, None


def _extract_number(data: dict[str, Any], *keys: str) -> Optional[float]:
    for wrapper in _wrappers(data):
        value = _pick(wrapper, *keys)
        if value is None:
            continue
        if isinstance(value, (list, tuple)) and value:
            try:
                return float(max(value))
            except (TypeError, ValueError):
                continue
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return None


def _quat_to_rotation_matrix(q: Any) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        raise WorkerError("quaternion norm is zero")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _parse_extrinsic(value: Any) -> tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
    """Parse an extrinsic into (R or None, t or None, parsed-as)."""
    if isinstance(value, dict):
        matrix = _pick(value, "matrix", "M", "transform", "homography")
        if matrix is not None:
            arr = np.asarray(matrix, dtype=float)
            if arr.shape == (4, 4):
                return arr[:3, :3], arr[:3, 3], "matrix4x4"
        rotation: Optional[np.ndarray] = None
        r = _pick(value, "rotation", "R", "rotationMatrix", "rotation_matrix")
        if r is not None:
            arr = np.asarray(r, dtype=float)
            if arr.shape == (3, 3):
                rotation = arr
            elif arr.size == 4:
                rotation = _quat_to_rotation_matrix(arr)
            elif arr.size == 3 and cv2 is not None:
                rotation = np.asarray(cv2.Rodrigues(arr)[0])  # type: ignore[attr-defined]
        q = _pick(value, "quaternion", "quat")
        if q is not None and rotation is None:
            qa = np.asarray(q, dtype=float)
            if qa.size == 4:
                rotation = _quat_to_rotation_matrix(qa)
        translation: Optional[np.ndarray] = None
        t = _pick(value, "translation", "t", "translationVector", "translation_vector")
        if t is not None:
            ta = np.asarray(t, dtype=float)
            if ta.size == 3:
                translation = ta.reshape(3)
        return rotation, translation, "dict"
    arr = np.asarray(value, dtype=float)
    if arr.shape == (4, 4):
        return arr[:3, :3], arr[:3, 3], "matrix4x4"
    if arr.shape == (3, 3):
        return arr, None, "rotation3x3"
    return None, None, "unknown"


def _check_extrinsic(prefix: str, rotation: Optional[np.ndarray], translation: Optional[np.ndarray], issues: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    if rotation is not None:
        err = float(np.max(np.abs(rotation @ rotation.T - np.eye(3))))
        det = float(np.linalg.det(rotation))
        summary["rotationOrthonormal"] = err < 1e-5
        summary["rotationOrthonormalError"] = round(err, 8)
        summary["rotationDet"] = round(det, 6)
        if err > 1e-5:
            issues.append(
                {
                    "severity": "warning",
                    "code": f"extrinsic.{prefix}.rotation_not_orthonormal",
                    "message": f"{prefix} 外参旋转矩阵不正交: max|RR^T-I| = {err:.2e}",
                }
            )
        if abs(det - 1.0) > 1e-5:
            issues.append(
                {
                    "severity": "warning",
                    "code": f"extrinsic.{prefix}.rotation_det_not_one",
                    "message": f"{prefix} 外参旋转矩阵 det = {det:.4f} != 1",
                }
            )
    if translation is not None:
        finite = bool(np.all(np.isfinite(translation)))
        summary["translationFinite"] = finite
        summary["translation"] = [round(float(v), 6) for v in translation]
        if not finite:
            issues.append(
                {
                    "severity": "error",
                    "code": f"extrinsic.{prefix}.translation_not_finite",
                    "message": f"{prefix} 平移向量包含 NaN/Inf",
                }
            )
        else:
            norm = float(np.linalg.norm(translation))
            summary["translationNorm"] = round(norm, 6)
            if norm < 1e-9 or norm > 1e6:
                issues.append(
                    {
                        "severity": "warning",
                        "code": f"extrinsic.{prefix}.translation_suspicious",
                        "message": f"{prefix} 平移尺度可疑: |t| = {norm:.3g} m",
                    }
                )
    return summary


def _yaml_loader() -> Any:
    """A PyYAML SafeLoader that understands OpenCV's ``!!opencv-matrix`` tag.

    ``cv2.FileStorage`` exports (the most common calibration files in the
    wild) use ``camera_matrix: !!opencv-matrix {rows, cols, dt, data: [...]}``;
    ``safe_load`` rejects the unknown tag with a ConstructorError.
    """
    ymod = _require_yaml()
    try:
        ymod.SafeLoader.add_constructor(
            "tag:yaml.org,2002:opencv-matrix",
            lambda loader, node: loader.construct_mapping(node, deep=True),
        )
    except Exception:  # noqa: BLE001 - best-effort registration
        pass
    return ymod


def _load_calibration(path: str) -> tuple[dict[str, Any], str]:
    if not os.path.exists(path):
        raise WorkerError(f"calibration file not found: {path}")
    ext = os.path.splitext(path)[1].lower()
    if ext in (".yaml", ".yml"):
        ymod = _yaml_loader()
        with open(path, encoding="utf-8") as handle:
            try:
                data = ymod.safe_load(handle)
            except Exception as error:  # noqa: BLE001 - report any parse failure
                raise WorkerError(f"invalid YAML calibration file {path}: {error}") from error
        if not isinstance(data, dict):
            raise WorkerError(f"calibration file {path} must contain a mapping, got {type(data).__name__}")
        return data, "yaml"
    with open(path, encoding="utf-8") as handle:
        try:
            data = json.load(handle)
        except json.JSONDecodeError as error:
            ymod = _yaml_loader()
            handle.seek(0)
            try:
                data = ymod.safe_load(handle)
            except Exception as inner:  # noqa: BLE001
                raise WorkerError(f"invalid JSON calibration file {path}: {error}") from inner
            if not isinstance(data, dict):
                raise WorkerError(f"calibration file {path} must contain a mapping, got {type(data).__name__}")
            return data, "yaml"
    if not isinstance(data, dict):
        raise WorkerError(f"calibration file {path} must contain a mapping, got {type(data).__name__}")
    return data, "json"


def cmd_calibration_inspect(args: dict[str, Any]) -> dict[str, Any]:
    """Structurally validate a JSON/YAML camera calibration file.

    Args::

        {"path": "calib.yaml"}
    """
    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    data, fmt = _load_calibration(path)

    issues: list[dict[str, Any]] = []
    K, flat = _extract_intrinsics(data)
    distortion, distortion_count = _extract_distortion(data)
    image_size = _parse_image_size(_pick(data, "imageSize", "image_size", "resolution", "widthHeight", "image_size_px"))
    reprojection_error = _extract_number(data, "reprojectionError", "reprojection_error", "rmsError", "rms", "totalAvgErr", "overallAvgErr")

    summary: dict[str, Any] = {}
    if K is not None:
        summary.update({key: round(flat[key], 4) for key in ("fx", "fy", "cx", "cy")})
        if float(flat["fx"]) <= 0 or float(flat["fy"]) <= 0:
            issues.append(
                {
                    "severity": "error",
                    "code": "calibration.intrinsics_invalid",
                    "message": f"内参无效: fx={flat['fx']}, fy={flat['fy']} 必须 > 0",
                }
            )
        if image_size is not None:
            width, height = image_size
            if not (0 <= flat["cx"] < width):
                issues.append(
                    {
                        "severity": "warning",
                        "code": "calibration.principal_point_out_of_bounds",
                        "message": f"cx={flat['cx']} 超出图像宽度 [0, {width})",
                    }
                )
            if not (0 <= flat["cy"] < height):
                issues.append(
                    {
                        "severity": "warning",
                        "code": "calibration.principal_point_out_of_bounds",
                        "message": f"cy={flat['cy']} 超出图像高度 [0, {height})",
                    }
                )
    else:
        issues.append(
            {
                "severity": "error",
                "code": "calibration.intrinsics_missing",
                "message": "缺少内参（cameraMatrix/K 或 fx/fy/cx/cy）",
            }
        )

    if image_size is not None:
        summary["imageSize"] = list(image_size)
    else:
        issues.append({"severity": "info", "code": "calibration.image_size_missing", "message": "未提供 imageSize，无法校验主点位置"})

    summary["distortionCount"] = distortion_count
    if distortion_count is None and distortion is None:
        issues.append({"severity": "info", "code": "calibration.distortion_missing", "message": "未提供畸变系数（无畸变模型可忽略）"})
    elif distortion_count is not None and distortion_count not in DISTORTION_COUNTS:
        issues.append(
            {
                "severity": "warning",
                "code": "calibration.distortion_count_unusual",
                "message": f"畸变系数数量 {distortion_count} 不在常见值 {sorted(DISTORTION_COUNTS)} 中",
            }
        )

    if reprojection_error is not None:
        summary["reprojectionError"] = round(reprojection_error, 4)
        if reprojection_error > 1.0:
            issues.append(
                {
                    "severity": "warning",
                    "code": "calibration.high_reprojection_error",
                    "message": f"重投影误差 {reprojection_error:.2f} px > 1.0 px：需要重新标定",
                }
            )
    else:
        issues.append({"severity": "info", "code": "calibration.reprojection_error_missing", "message": "未提供 reprojectionError，无法评估标定精度"})

    calib_date = _pick(data, "calibrationDate", "calibration_date", "date")
    source = _pick(data, "source", "origin", "software", "tool")
    if calib_date is not None:
        summary["calibrationDate"] = str(calib_date)
    if source is not None:
        summary["source"] = str(source)

    extrinsics: dict[str, Any] = {}
    for key, prefix in (("stereoTransform", "stereo"), ("handEyeTransform", "handeye")):
        value = None
        # per-prefix fallback keys: previously both loops accepted
        # "stereo_transform" AND "hand_eye_transform", so a file with only one
        # of them reported the same transform under BOTH labels
        fallbacks = ("stereo_transform",) if prefix == "stereo" else ("hand_eye_transform",)
        for wrapper in _wrappers(data):
            candidate = _pick(wrapper, key, *fallbacks)
            if candidate is not None:
                value = candidate
                break
        if value is not None:
            rotation, translation, parsed_as = _parse_extrinsic(value)
            if parsed_as == "unknown" or (rotation is None and translation is None):
                issues.append(
                    {
                        "severity": "warning",
                        "code": f"extrinsic.{prefix}.unparseable",
                        "message": f"{key} 无法解析为 4x4 矩阵或 rotation/translation 结构",
                    }
                )
                extrinsics[key] = {"parsedAs": "unknown"}
            else:
                detail = _check_extrinsic(prefix, rotation, translation, issues)
                detail["parsedAs"] = parsed_as
                extrinsics[key] = detail

    errors = [i for i in issues if i["severity"] == "error"]
    if errors:
        verdict = "incomplete"
    elif any(i["code"] == "calibration.high_reprojection_error" for i in issues):
        verdict = "needs-recalibration"
    else:
        verdict = "plausible"

    return {
        "ok": not errors,
        "path": os.path.abspath(path),
        "format": fmt,
        "summary": summary,
        "extrinsics": extrinsics,
        "issues": issues,
        "verdict": verdict,
        "note": "不承诺标定正确，仅结构检查",
        "inputArgs": {"path": path},
    }


# ---------------------------------------------------------------------------
# pose-transform-validate
# ---------------------------------------------------------------------------

def _rpy_to_rotation_matrix(rpy: Any) -> np.ndarray:
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _parse_pose(item: dict[str, Any]) -> tuple[Optional[str], np.ndarray, np.ndarray, np.ndarray, Optional[Any]]:
    name = item.get("name")
    label = name or "?"
    matrix = item.get("matrix")
    if matrix is not None:
        arr = np.asarray(matrix, dtype=float)
        if arr.shape != (4, 4):
            raise WorkerError(f"transform {label}: matrix must be 4x4, got shape {arr.shape}")
        return name, arr[:3, :3], arr[:3, 3], arr[3], item.get("quaternion", item.get("quat"))

    position = item.get("position")
    if position is None:
        raise WorkerError(f"transform {label}: must provide 'matrix' or 'position' plus a rotation")
    pos = np.asarray(position, dtype=float)
    if pos.shape != (3,):
        raise WorkerError(f"transform {label}: position must be [x, y, z]")

    quat = item.get("quaternion", item.get("quat"))
    rpy = item.get("rpy")
    if rpy is None and "rpyDeg" in item:
        rpy = [math.radians(float(v)) for v in item["rpyDeg"]]
    if quat is not None:
        qa = np.asarray(quat, dtype=float)
        if qa.shape != (4,):
            raise WorkerError(f"transform {label}: quaternion must be [w, x, y, z]")
        rotation = _quat_to_rotation_matrix(qa)
    elif rpy is not None:
        if len(rpy) != 3:
            raise WorkerError(f"transform {label}: rpy must be [roll, pitch, yaw] in radians")
        rotation = _rpy_to_rotation_matrix(rpy)
    else:
        raise WorkerError(f"transform {label}: rotation missing; provide 'quaternion' (w,x,y,z) or 'rpy' (radians)")
    return name, rotation, pos, np.array([0.0, 0.0, 0.0, 1.0]), quat


def _check_pose(
    name: Optional[str],
    rotation: np.ndarray,
    translation: np.ndarray,
    last_row: np.ndarray,
    quat: Optional[Any],
    issues: list[dict[str, Any]],
) -> dict[str, Any]:
    label = name or "?"
    ortho_err = float(np.max(np.abs(rotation @ rotation.T - np.eye(3))))
    det = float(np.linalg.det(rotation))
    translation_finite = bool(np.all(np.isfinite(translation)))
    last_row_ok = bool(np.allclose(last_row, [0.0, 0.0, 0.0, 1.0], atol=1e-5))
    rotation_orthonormal = ortho_err < 1e-5
    determinant_ok = abs(det - 1.0) < 1e-5

    quaternion_unit: Optional[float] = None
    quaternion_ok = True
    if quat is not None:
        norm = float(np.linalg.norm(np.asarray(quat, dtype=float)))
        quaternion_unit = round(norm, 6)
        quaternion_ok = abs(norm - 1.0) < 1e-3

    valid = rotation_orthonormal and determinant_ok and translation_finite and last_row_ok and quaternion_ok
    entry: dict[str, Any] = {
        "name": name,
        "rotationOrthonormal": rotation_orthonormal,
        "rotationOrthonormalError": round(ortho_err, 8),
        "determinant": round(det, 8),
        "determinantOk": determinant_ok,
        "translationFinite": translation_finite,
        "lastRowOk": last_row_ok,
        "valid": valid,
    }
    if quaternion_unit is not None:
        entry["quaternionUnit"] = quaternion_unit
        entry["quaternionUnitOk"] = quaternion_ok

    if not rotation_orthonormal:
        issues.append(
            {
                "transform": label,
                "code": "rotation.not_orthonormal",
                "message": f"max|RR^T - I| = {ortho_err:.2e} > 1e-5",
            }
        )
    if not determinant_ok:
        issues.append(
            {
                "transform": label,
                "code": "rotation.determinant_not_one",
                "message": f"det(R) = {det:.6f}, |det - 1| > 1e-5（反射或缩放）",
            }
        )
    if not translation_finite:
        issues.append({"transform": label, "code": "translation.not_finite", "message": "translation 包含 NaN/Inf"})
    if not last_row_ok:
        issues.append(
            {
                "transform": label,
                "code": "matrix.last_row_invalid",
                "message": f"最后一行 {[round(float(v), 6) for v in last_row]} != [0, 0, 0, 1]",
            }
        )
    if quat is not None and not quaternion_ok:
        issues.append(
            {
                "transform": label,
                "code": "quaternion.not_unit",
                "message": f"quaternion 范数 {quaternion_unit} != 1（允许误差 1e-3）",
            }
        )
    return entry


def cmd_pose_transform_validate(args: dict[str, Any]) -> dict[str, Any]:
    """Validate 4x4 homogeneous transforms / poses numerically.

    Args::

        {"transform": {"name"?, "matrix": [[...4x4]]}}
        {"transforms": [{"name"?, "position": [x,y,z], "quaternion": [w,x,y,z]}, ...]}
        {"transforms": [{"name"?, "position": [x,y,z], "rpy": [r,p,y]}, ...]}
    """
    single = args.get("transform")
    many = args.get("transforms")
    if single is None and many is None:
        raise WorkerError("missing required argument: provide 'transform' or 'transforms'")
    if single is not None and many is not None:
        raise WorkerError("provide only one of 'transform' or 'transforms'")
    items = many if many is not None else [single]
    if not isinstance(items, list):
        raise WorkerError("'transforms' must be a list")

    transforms: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise WorkerError(f"transforms[{index}] must be an object")
        name, rotation, translation, last_row, quat = _parse_pose(item)
        if name is None:
            name = f"transform-{index}"
        transforms.append(_check_pose(name, rotation, translation, last_row, quat, issues))

    return {
        "ok": all(t["valid"] for t in transforms),
        "count": len(transforms),
        "transforms": transforms,
        "issues": issues,
        "note": "仅数值有效性，不校验语义正确性",
        "notes": [
            "quaternion 约定 (w,x,y,z)；(w,x,y,z) 与 (-w,-x,-y,-z) 表示同一旋转，本检查不强制 w>=0",
            "rpy 按 ZYX 内旋顺序解释（R = Rz(yaw) @ Ry(pitch) @ Rx(roll)），单位弧度；rpyDeg 可用角度",
        ],
        "inputArgs": {"count": len(items)},
    }


# ---------------------------------------------------------------------------
# perception-run / perception-compare (wraps vision.route_perception)
# ---------------------------------------------------------------------------

def _require_known_color(color: str) -> None:
    if color not in COLOR_PRESETS:
        raise WorkerError(f"unknown color {color!r}; supported: {sorted(COLOR_PRESETS)}")


def _map_fault(fault: dict[str, Any]) -> dict[str, Any]:
    mapped: dict[str, Any] = {}
    if "perceptionOffsetPx" in fault:
        offset = fault["perceptionOffsetPx"]
        if not isinstance(offset, (list, tuple)) or len(offset) != 2:
            raise WorkerError("fault.perceptionOffsetPx must be [dx, dy]")
        mapped["perception_offset_px"] = [float(offset[0]), float(offset[1])]
    if "perception_offset_px" in fault:
        offset = fault["perception_offset_px"]
        if not isinstance(offset, (list, tuple)) or len(offset) != 2:
            raise WorkerError("fault.perception_offset_px must be [dx, dy]")
        mapped["perception_offset_px"] = [float(offset[0]), float(offset[1])]
    if "occlusion" in fault:
        mapped["occlusion"] = bool(fault["occlusion"])
    if "modelTimeoutS" in fault:
        mapped["model_timeout_s"] = float(fault["modelTimeoutS"])
    if "model_timeout_s" in fault:
        mapped["model_timeout_s"] = float(fault["model_timeout_s"])
    return mapped


def _segmentation_mask(image_rgb: np.ndarray, method: str, color: Optional[str]) -> np.ndarray:
    """Recompute the segmentation mask for artifact drawing / IoU.

    Mirrors the mask construction inside vision.color_segmentation /
    vision.saliency_segmentation so drawn contours match the reported blob.
    """
    cv2 = _require_cv2()
    bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)  # type: ignore[attr-defined]
    if method in ("color", "color_segmentation"):
        if color not in COLOR_PRESETS:
            raise WorkerError(f"unknown color {color!r}; supported: {sorted(COLOR_PRESETS)}")
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)  # type: ignore[attr-defined]
        lo = np.array([COLOR_PRESETS[color][0], COLOR_PRESETS[color][2], COLOR_PRESETS[color][4]], dtype=np.uint8)
        hi = np.array([COLOR_PRESETS[color][1], COLOR_PRESETS[color][3], COLOR_PRESETS[color][5]], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)  # type: ignore[attr-defined]
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # type: ignore[attr-defined]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)  # type: ignore[attr-defined]
    edges = cv2.Canny(gray, 50, 150)  # type: ignore[attr-defined]
    return cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=2)  # type: ignore[attr-defined]


def _write_annotation(image_rgb: np.ndarray, out_path: str, decision: dict[str, Any]) -> str:
    cv2 = _require_cv2()
    overlay = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR).copy()  # type: ignore[attr-defined]
    if decision.get("ok") and decision.get("result"):
        result = decision["result"]
        cx, cy = result["centroidPx"]
        route = decision.get("route", "color")
        color_hint = result.get("color")
        cv2.line(overlay, (int(cx) - 15, int(cy)), (int(cx) + 15, int(cy)), (0, 255, 0), 2)  # type: ignore[attr-defined]
        cv2.line(overlay, (int(cx), int(cy) - 15), (int(cx), int(cy) + 15), (0, 255, 0), 2)  # type: ignore[attr-defined]
        try:
            mask = _segmentation_mask(image_rgb, route, color_hint)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # type: ignore[attr-defined]
            if contours:
                best = max(contours, key=cv2.contourArea)  # type: ignore[attr-defined]
                cv2.drawContours(overlay, [best], -1, (0, 0, 255), 2)  # type: ignore[attr-defined]
            else:
                radius = max(int(math.sqrt(max(float(result.get("areaPx", 0.0)), 0.0) / math.pi)), 3)
                cv2.circle(overlay, (int(cx), int(cy)), radius, (0, 0, 255), 2)  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001 - fall back to an area-proportional circle
            radius = max(int(math.sqrt(max(float(result.get("areaPx", 0.0)), 0.0) / math.pi)), 3)
            cv2.circle(overlay, (int(cx), int(cy)), radius, (0, 0, 255), 2)  # type: ignore[attr-defined]
    else:
        cv2.putText(overlay, "NO_DETECTIONS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)  # type: ignore[attr-defined]
    return _imwrite(out_path, overlay)


def cmd_perception_run(args: dict[str, Any]) -> dict[str, Any]:
    """Run the perception router on one image (wraps vision.route_perception).

    Args::

        {"imagePath": "...", "route"?: "auto"|"color"|"saliency", "color"?: "red",
         "minArea"?: 40, "fault"?: {"perceptionOffsetPx"?: [dx,dy], "occlusion"?: bool},
         "outPath"?: "..."}
    """
    image_path = args.get("imagePath")
    if not image_path:
        raise WorkerError("missing required argument 'imagePath'")
    route = args.get("route", "auto")
    if route not in ("auto", "color", "saliency"):
        raise WorkerError(f"route must be 'auto', 'color' or 'saliency', got {route!r}")
    color = args.get("color", "red")
    if route in ("auto", "color"):
        _require_known_color(color)
    min_area = float(args["minArea"]) if args.get("minArea") is not None else None
    fault = _map_fault(dict(args.get("fault") or {}))

    image = _read_rgb(image_path)
    started = time.time()
    # forward minArea into the router: without this the segmentation defaults
    # (min_area=40) silently won — a documented minArea=10 still required 40px
    decision = vision.route_perception(
        image, {}, rng=None, perception=route, color=color, fault=fault, min_area=int(min_area) if min_area is not None else 40
    )
    latency_ms = round((time.time() - started) * 1000, 1)

    if decision.get("ok") and min_area is not None:
        area = (decision.get("result") or {}).get("areaPx")
        if area is not None and float(area) < min_area:
            decision = {
                "ok": False,
                "route": decision.get("route"),
                "result": None,
                "reason": f"blob area {float(area):.1f} px < minArea {min_area:.1f} px",
                "needsRecheck": False,
                "attempts": decision.get("attempts"),
            }

    out_path = args.get("outPath") or _default_annotated_path(image_path)
    note: Optional[str] = None
    try:
        written = _write_annotation(image, out_path, decision)
    except WorkerError as error:
        written = None
        note = f"标注图未生成：{error}"
    if written is None:
        written = out_path
    if args.get("outPath") is None:
        note = (note + "；" if note else "") + f"outPath 未提供，标注图输出到 {written}"

    return {
        "ok": bool(decision.get("ok")),
        "route": decision.get("route"),
        "result": decision.get("result"),
        "reason": decision.get("reason"),
        "needsRecheck": bool(decision.get("needsRecheck", False)),
        "attempts": decision.get("attempts"),
        "latencyMs": latency_ms,
        "imagePath": os.path.abspath(image_path),
        "artifacts": {"input": os.path.abspath(image_path), "output": written if os.path.exists(written) else None},
        "outPath": os.path.abspath(written) if os.path.exists(written) else None,
        "note": note or "标注输出：centroid 十字线 + 最大轮廓",
        "inputArgs": {"imagePath": image_path, "route": route, "color": color, "minArea": min_area},
    }


def cmd_perception_compare(args: dict[str, Any]) -> dict[str, Any]:
    """Run the same perception method on two images and compare results.

    Args::

        {"imagePathA": "...", "imagePathB": "...", "method"?: "color"|"saliency"|"auto",
         "color"?: "red", "groundTruthCentroidPx"?: [x, y]}
    """
    path_a = args.get("imagePathA")
    path_b = args.get("imagePathB")
    if not path_a or not path_b:
        raise WorkerError("missing required arguments 'imagePathA' and 'imagePathB'")
    method = args.get("method", "color")
    if method not in ("auto", "color", "saliency"):
        raise WorkerError(f"method must be 'auto', 'color' or 'saliency', got {method!r}")
    color = args.get("color", "red")
    if method in ("auto", "color"):
        _require_known_color(color)

    image_a = _read_rgb(path_a)
    image_b = _read_rgb(path_b)
    decision_a = vision.route_perception(image_a, {}, rng=None, perception=method, color=color)
    decision_b = vision.route_perception(image_b, {}, rng=None, perception=method, color=color)

    result_a = dict(decision_a)
    result_a["imagePath"] = os.path.abspath(path_a)
    result_b = dict(decision_b)
    result_b["imagePath"] = os.path.abspath(path_b)

    centroid_a = (decision_a.get("result") or {}).get("centroidPx") if decision_a.get("ok") else None
    centroid_b = (decision_b.get("result") or {}).get("centroidPx") if decision_b.get("ok") else None
    delta_px = round(math.dist(centroid_a, centroid_b), 2) if centroid_a and centroid_b else None

    iou: Optional[float] = None
    if centroid_a is not None and centroid_b is not None:
        try:
            mask_a = _segmentation_mask(image_a, decision_a.get("route", method), color if decision_a.get("route") == "color" else None)
            mask_b = _segmentation_mask(image_b, decision_b.get("route", method), color if decision_b.get("route") == "color" else None)
            inter = int(np.count_nonzero(mask_a & mask_b))
            union = int(np.count_nonzero(mask_a | mask_b))
            iou = round(inter / union, 4) if union else None
        except WorkerError:
            iou = None

    agreement_threshold = float(args.get("agreementThresholdPx", 20.0))
    agreement = delta_px is not None and delta_px <= agreement_threshold

    errors: Optional[dict[str, float]] = None
    gt = args.get("groundTruthCentroidPx")
    if gt is not None and len(gt) == 2 and centroid_a and centroid_b:
        errors = {
            "a": round(math.dist([float(gt[0]), float(gt[1])], centroid_a), 2),
            "b": round(math.dist([float(gt[0]), float(gt[1])], centroid_b), 2),
        }

    return {
        "ok": True,
        "resultA": result_a,
        "resultB": result_b,
        "deltaPx": delta_px,
        "iou": iou,
        "agreement": agreement,
        "agreementThresholdPx": agreement_threshold,
        "groundTruthErrorsPx": errors,
        "note": f"同一感知方法 '{method}'（color={color!r}）应用于两张图像；deltaPx 为两质心欧氏距离，iou 基于重算的分割掩码（掩码不可用时为 null）",
        "inputArgs": {"imagePathA": path_a, "imagePathB": path_b, "method": method, "color": color},
    }


# ---------------------------------------------------------------------------
# image-dataset-profile
# ---------------------------------------------------------------------------

def cmd_image_dataset_profile(args: dict[str, Any]) -> dict[str, Any]:
    """Profile an image dataset directory (resolution stats, corrupt files).

    Args::

        {"path": "...", "extensions"?: [".png", ".jpg"], "maxFiles"?: 200}
    """
    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.isdir(path):
        raise WorkerError(f"path is not a directory: {path}")
    pil = _require_pil()

    extensions = args.get("extensions") or sorted(IMAGE_EXTS)
    exts = {str(e).lower() if str(e).startswith(".") else "." + str(e).lower() for e in extensions}
    max_files = int(args.get("maxFiles", 200) or 200)

    discovered: list[str] = []
    for root, dirs, files in os.walk(path):
        dirs.sort()
        for name in sorted(files):
            if os.path.splitext(name)[1].lower() in exts:
                discovered.append(os.path.join(root, name))
    total = len(discovered)
    sampled_paths = discovered[:max_files]
    truncated = total > max_files

    files_out: list[dict[str, Any]] = []
    corrupt: list[dict[str, Any]] = []
    count_by_ext: dict[str, int] = {}
    widths: list[int] = []
    heights: list[int] = []
    total_size = 0

    for full in sampled_paths:
        size = os.path.getsize(full)
        total_size += size
        ext = os.path.splitext(full)[1].lower()
        count_by_ext[ext] = count_by_ext.get(ext, 0) + 1
        try:
            with pil.open(full) as handle:
                width, height = handle.size
            files_out.append({"path": os.path.abspath(full), "size": size, "width": width, "height": height, "ext": ext})
            widths.append(width)
            heights.append(height)
        except Exception as error:  # noqa: BLE001 - corrupt file, record and continue
            corrupt.append(
                {"path": os.path.abspath(full), "ext": ext, "size": size, "error": f"{type(error).__name__}: {error}"}
            )

    resolution_stats: Optional[dict[str, int]] = None
    if widths:
        resolution_stats = {
            "minW": min(widths),
            "minH": min(heights),
            "maxW": max(widths),
            "maxH": max(heights),
            "commonW": Counter(widths).most_common(1)[0][0],
            "commonH": Counter(heights).most_common(1)[0][0],
        }

    issues: list[dict[str, Any]] = []
    distinct = sorted({(w, h) for w, h in zip(widths, heights)})
    if len(distinct) > 1:
        issues.append(
            {
                "severity": "warning",
                "code": "resolution.inconsistent",
                "message": f"{len(distinct)} 种不同分辨率: {['{}x{}'.format(w, h) for w, h in distinct][:8]}",
            }
        )
    if corrupt:
        issues.append({"severity": "error", "code": "corrupt.files", "message": f"{len(corrupt)} 个文件损坏或无法读取"})

    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "path": os.path.abspath(path),
        "files": files_out,
        "corruptFiles": corrupt,
        "count": len(files_out),
        "totalSize": total_size,
        "countByExt": count_by_ext,
        "resolutionStats": resolution_stats,
        "issues": issues,
        "sampled": truncated,
        "totalDiscovered": total,
        "note": "用 PIL 读取尺寸（Image.open 后取 .size，不加载全图）；递归扫描目录",
        "inputArgs": {"path": path, "extensions": sorted(exts), "maxFiles": max_files},
    }


# ---------------------------------------------------------------------------
# annotate-failure-frame
# ---------------------------------------------------------------------------

def _parse_color(value: Any) -> tuple[int, int, int]:
    """Parse a color into BGR; accepts a name, '#rrggbb' or [r, g, b]."""
    if value is None:
        return (0, 0, 255)
    if isinstance(value, str):
        name = value.lower()
        if name in COLOR_NAMES:
            return COLOR_NAMES[name]
        if name.startswith("#") and len(name) == 7:
            try:
                r = int(name[1:3], 16)
                g = int(name[3:5], 16)
                b = int(name[5:7], 16)
                return (b, g, r)
            except ValueError as error:
                raise WorkerError(f"invalid hex color {value!r}") from error
        raise WorkerError(f"unknown color {value!r}; use a name, '#rrggbb' or [r, g, b]")
    if isinstance(value, (list, tuple)) and len(value) == 3:
        r, g, b = (int(v) for v in value)
        return (b, g, r)
    raise WorkerError(f"invalid color {value!r}")


def cmd_annotate_failure_frame(args: dict[str, Any]) -> dict[str, Any]:
    """Draw bbox / centroid / label overlays onto a failure frame.

    Args::

        {"imagePath": "...", "detections"?: [{"bbox"?: [x,y,w,h], "centroidPx"?: [x,y],
          "label"?: str, "color"?: str|[r,g,b]}, ...], "outPath"?: "..."}
    """
    image_path = args.get("imagePath")
    if not image_path:
        raise WorkerError("missing required argument 'imagePath'")
    detections = args.get("detections") or []
    if not isinstance(detections, list):
        raise WorkerError("'detections' must be a list")
    out_path = args.get("outPath") or _default_annotated_path(image_path)

    cv2 = _require_cv2()
    overlay = _read_bgr(image_path).copy()
    drawn = 0
    if detections:
        for index, det in enumerate(detections):
            if not isinstance(det, dict):
                raise WorkerError(f"detections[{index}] must be an object")
            color_bgr = _parse_color(det.get("color"))
            label = det.get("label")
            bbox = det.get("bbox")
            centroid = det.get("centroidPx")
            if bbox is None and centroid is None:
                raise WorkerError(f"detections[{index}] needs 'bbox' or 'centroidPx'")
            if bbox is not None:
                if len(bbox) != 4:
                    raise WorkerError(f"detections[{index}].bbox must be [x, y, w, h]")
                x, y, w, h = (int(v) for v in bbox)
                cv2.rectangle(overlay, (x, y), (x + w, y + h), color_bgr, 2)  # type: ignore[attr-defined]
                if label:
                    cv2.putText(overlay, str(label), (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)  # type: ignore[attr-defined]
                drawn += 1
            if centroid is not None:
                if len(centroid) != 2:
                    raise WorkerError(f"detections[{index}].centroidPx must be [x, y]")
                cx, cy = (int(v) for v in centroid)
                cv2.line(overlay, (cx - 12, cy), (cx + 12, cy), color_bgr, 2)  # type: ignore[attr-defined]
                cv2.line(overlay, (cx, cy - 12), (cx, cy + 12), color_bgr, 2)  # type: ignore[attr-defined]
                if label and bbox is None:
                    cv2.putText(overlay, str(label), (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)  # type: ignore[attr-defined]
                drawn += 1
    else:
        cv2.putText(overlay, "NO_DETECTIONS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)  # type: ignore[attr-defined]

    written = _imwrite(out_path, overlay)
    note = "标注仅供参考，不修改原图"
    if args.get("outPath") is None:
        note += f"；outPath 未提供，输出到 {written}"

    return {
        "ok": True,
        "inputPath": os.path.abspath(image_path),
        "outPath": written,
        "annotationsDrawn": drawn,
        "detectionsRequested": len(detections),
        "note": note,
        "inputArgs": {"imagePath": image_path, "detections": len(detections), "outPath": out_path},
    }


# ---------------------------------------------------------------------------
# module interface (worker module contract)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "camera-health-check": cmd_camera_health_check,
    "calibration-inspect": cmd_calibration_inspect,
    "pose-transform-validate": cmd_pose_transform_validate,
    "perception-run": cmd_perception_run,
    "perception-compare": cmd_perception_compare,
    "image-dataset-profile": cmd_image_dataset_profile,
    "annotate-failure-frame": cmd_annotate_failure_frame,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "vision.camera_health_check",
        "kind": "perception",
        "provider": "robotic-harness-worker",
        "input": {"imagePath": "string?", "imageDir": "string?", "expectedWidth": "integer?", "expectedHeight": "integer?"},
        "output": "per-image brightness/blur/noise metrics + aggregated issues",
        "risk": "R0-readonly",
        "description": "Camera health check: brightness, blur (Laplacian variance), noise and resolution checks for one image or a sampled directory.",
    },
    {
        "id": "vision.calibration_inspect",
        "kind": "calibration",
        "provider": "robotic-harness-worker",
        "input": {"path": "string"},
        "output": "calibration structure report with verdict",
        "risk": "R0-readonly",
        "description": "Structural validation of a JSON/YAML camera calibration file (intrinsics, distortion, reprojection error, extrinsics).",
    },
    {
        "id": "pose.transform_validate",
        "kind": "calibration",
        "provider": "robotic-harness-worker",
        "input": {"transform": "object?", "transforms": "array?"},
        "output": "per-transform numeric validity flags",
        "risk": "R0-readonly",
        "description": "Numeric validation of 4x4 homogeneous transforms / poses (orthonormality, determinant, finite translation, unit quaternion).",
    },
    {
        "id": "vision.perception_run",
        "kind": "perception",
        "provider": "robotic-harness-worker",
        "input": {"imagePath": "string", "route": "string?", "color": "string?", "minArea": "number?", "fault": "object?", "outPath": "string?"},
        "output": "perception decision + annotated image artifact",
        "risk": "R1-derive",
        "description": "Run the perception router (color/saliency/auto) on an image with latency and artifact records.",
    },
    {
        "id": "vision.perception_compare",
        "kind": "perception",
        "provider": "robotic-harness-worker",
        "input": {"imagePathA": "string", "imagePathB": "string", "method": "string?", "color": "string?", "groundTruthCentroidPx": "array?"},
        "output": "centroid delta, mask IoU and agreement between two images",
        "risk": "R1-derive",
        "description": "Run the same perception method on two images and compare centroids / masks.",
    },
    {
        "id": "vision.image_dataset_profile",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "extensions": "array?", "maxFiles": "integer?"},
        "output": "resolution stats, size by extension, corrupt files",
        "risk": "R0-readonly",
        "description": "Profile an image dataset directory without decoding pixels (PIL header reads).",
    },
    {
        "id": "vision.annotate_failure_frame",
        "kind": "perception",
        "provider": "robotic-harness-worker",
        "input": {"imagePath": "string", "detections": "array?", "outPath": "string?"},
        "output": "annotated overlay image",
        "risk": "R1-derive",
        "description": "Draw bbox / centroid / label overlays onto a failure frame; the source image is never modified.",
    },
]
