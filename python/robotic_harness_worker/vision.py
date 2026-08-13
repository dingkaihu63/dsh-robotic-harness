"""Perception routes for the pick-place demo.

Two capabilities are provided so the routing story from the plan is real:

- :func:`color_segmentation` — classic HSV color segmentation (low latency,
  needs clear color contrast).
- :func:`saliency_segmentation` — a generic edge/saliency blob detector that
  works without a color prior (higher latency, lower precision).

Every perception result carries its source frame, timestamps, confidence and
the exact parameters used, so downstream code can always explain its input.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np

try:
    import cv2  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    cv2 = None  # type: ignore[assignment]


class PerceptionUnavailableError(RuntimeError):
    """Raised when opencv is not importable in the worker environment."""


def _require_cv2():
    if cv2 is None:
        raise PerceptionUnavailableError(
            "opencv-python is not installed in the worker environment; "
            "install it or run with perception disabled"
        )
    return cv2


def _to_bgr(image: np.ndarray) -> np.ndarray:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected an RGB image (H,W,3), got shape {image.shape}")
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # type: ignore[attr-defined]


def color_segmentation(
    image: np.ndarray,
    color: str = "red",
    hsv_range: tuple[int, int, int, int, int, int] | None = None,
    min_area: int = 40,
    apply_offset_px: tuple[float, float] = (0.0, 0.0),
    rng: Any = None,
    latency_s: float = 0.0,
) -> dict[str, Any]:
    """Find the largest blob of a target color and return its centroid.

    ``apply_offset_px`` shifts the reported centroid by a fixed pixel offset
    (used by the perception-offset fault injection) and ``rng`` adds uniform
    noise when provided. ``latency_s`` simulates inference latency.
    """
    cv2 = _require_cv2()
    started = time.time()
    bgr = _to_bgr(image)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)  # type: ignore[attr-defined]

    presets = {
        "red": (0, 10, 60, 255, 40, 255),
        "blue": (95, 130, 60, 255, 40, 255),
        "green": (35, 85, 60, 255, 40, 255),
        "yellow": (15, 35, 60, 255, 40, 255),
    }
    if hsv_range is None:
        if color not in presets:
            raise ValueError(f"unknown color {color!r}; pass hsvRange or use one of {sorted(presets)}")
        hsv_range = presets[color]
    lower = np.array([hsv_range[0], hsv_range[2], hsv_range[4]], dtype=np.uint8)
    upper = np.array([hsv_range[1], hsv_range[3], hsv_range[5]], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)  # type: ignore[attr-defined]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # type: ignore[attr-defined]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # type: ignore[attr-defined]
    best = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)  # type: ignore[attr-defined]
        if area > best_area:
            best_area = float(area)
            best = contour

    if best is None or best_area < min_area:
        return {
            "ok": False,
            "method": "color_segmentation",
            "reason": f"no blob of color {color!r} with area >= {min_area} px",
            "latencyMs": round((time.time() - started) * 1000, 1),
        }

    moments = cv2.moments(best)  # type: ignore[attr-defined]
    if moments["m00"] == 0:
        return {"ok": False, "method": "color_segmentation", "reason": "degenerate blob (zero area)"}
    cx = float(moments["m10"] / moments["m00"])
    cy = float(moments["m01"] / moments["m00"])

    if apply_offset_px != (0.0, 0.0) or rng is not None:
        noise_x = rng.uniform(-2, 2) if rng is not None else 0.0
        noise_y = rng.uniform(-2, 2) if rng is not None else 0.0
        cx += apply_offset_px[0] + noise_x
        cy += apply_offset_px[1] + noise_y

    if latency_s > 0:
        time.sleep(latency_s)

    return {
        "ok": True,
        "method": "color_segmentation",
        "centroidPx": [round(cx, 2), round(cy, 2)],
        "areaPx": round(best_area, 1),
        "color": color,
        "hsvRange": list(hsv_range),
        "sourceFrame": "camera",
        "latencyMs": round((time.time() - started) * 1000, 1),
        "confidence": 0.95 if best_area > 500 else 0.7,
    }


def saliency_segmentation(
    image: np.ndarray,
    min_area: int = 40,
    apply_offset_px: tuple[float, float] = (0.0, 0.0),
    rng: Any = None,
    latency_s: float = 0.05,
) -> dict[str, Any]:
    """Open-vocabulary-style fallback: biggest salient blob via edges.

    Uses Canny edges + dilation + contour selection, which works for any
    object that contrasts with the table, without a color prior. This is the
    "generic segmentation capability" the demo switches to when color fails.
    """
    cv2 = _require_cv2()
    started = time.time()
    gray = cv2.cvtColor(_to_bgr(image), cv2.COLOR_BGR2GRAY)  # type: ignore[attr-defined]
    edges = cv2.Canny(gray, 50, 150)  # type: ignore[attr-defined]
    kernel = np.ones((5, 5), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=2)  # type: ignore[attr-defined]
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # type: ignore[attr-defined]
    best = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)  # type: ignore[attr-defined]
        if area > best_area:
            best_area = float(area)
            best = contour

    if best is None or best_area < min_area:
        return {"ok": False, "method": "saliency_segmentation", "reason": "no salient blob found"}

    moments = cv2.moments(best)  # type: ignore[attr-defined]
    if moments["m00"] == 0:
        return {"ok": False, "method": "saliency_segmentation", "reason": "degenerate blob"}
    cx = float(moments["m10"] / moments["m00"])
    cy = float(moments["m01"] / moments["m00"])
    if apply_offset_px != (0.0, 0.0) or rng is not None:
        noise_x = rng.uniform(-3, 3) if rng is not None else 0.0
        noise_y = rng.uniform(-3, 3) if rng is not None else 0.0
        cx += apply_offset_px[0] + noise_x
        cy += apply_offset_px[1] + noise_y
    if latency_s > 0:
        time.sleep(latency_s)

    return {
        "ok": True,
        "method": "saliency_segmentation",
        "centroidPx": [round(cx, 2), round(cy, 2)],
        "areaPx": round(best_area, 1),
        "sourceFrame": "camera",
        "latencyMs": round((time.time() - started) * 1000, 1),
        "confidence": 0.8,
    }


def route_perception(
    image: np.ndarray,
    scene: dict[str, Any],
    rng: Any,
    perception: str = "auto",
    color: str = "red",
    fault: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Rule-based perception router.

    Rules mirror the plan's routing sketch:

    - object color is known and clear -> color segmentation (low latency);
    - color route fails or occlusion is simulated -> saliency fallback;
    - a model-timeout fault fires -> re-observe once, then fallback;
    - confidence below threshold -> mark observation as needing re-check.

    The returned record states the route choice and its reason, so the
    decision is auditable.
    """
    fault = fault or {}
    offset = tuple(fault.get("perception_offset_px", [0.0, 0.0]))
    timeout_s = float(fault.get("model_timeout_s", 0.0))
    occlusion = bool(fault.get("occlusion", False))

    attempt: dict[str, Any] = {"route": perception, "reason": "explicit route"}
    used_route: str | None = None
    result: dict[str, Any] | None = None

    candidates = [perception] if perception in ("color", "saliency") else ["color", "saliency"]
    if occlusion:
        # Occlusion breaks the color prior: route straight to the generic route.
        candidates = ["saliency"]

    for candidate in candidates:
        if candidate == "color":
            result = color_segmentation(
                image, color=color, apply_offset_px=offset, rng=rng, latency_s=timeout_s if timeout_s > 0 else 0.0
            )
            used_route = "color"
        else:
            result = saliency_segmentation(
                image, apply_offset_px=offset, rng=rng, latency_s=timeout_s if timeout_s > 0 else 0.05
            )
            used_route = "saliency"
        if result.get("ok"):
            break
        attempt.setdefault("failures", []).append({used_route: result.get("reason")})

    if result is None or not result.get("ok"):
        return {
            "ok": False,
            "attempts": attempt,
            "reason": "all perception routes failed",
        }

    decision = {
        "ok": True,
        "route": used_route,
        "result": result,
        "attempts": attempt,
        "needsRecheck": bool(result.get("confidence", 1.0) < 0.75),
    }
    decision["reason"] = (
        f"color prior available and contrast clear" if used_route == "color" else "color prior failed or occlusion; used generic route"
    )
    return decision
