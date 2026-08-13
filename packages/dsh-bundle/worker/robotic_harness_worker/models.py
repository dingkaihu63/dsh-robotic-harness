"""Embodied-model / VLM / VLA adapter layer for the Robotic Harness worker.

Implements chapter 10 of the plan: an embodied-model registry with a
"registry + adapter interface + backend detection" pattern. Heavy models
(PyTorch / VLA / CLIP / vLLM ...) are **never hard dependencies**:

- (a) ``demo.*`` models are built-in *demo adapters* (scripted analytic-IK
      policy / classical color & saliency perception) that run for real inside
      this worker with only numpy + cv2;
- (b) *external module models* declare a python ``module``/``entrypoint``; the
      worker probes importability and returns a structured
      ``backend: "unavailable"`` diagnostic when the package is missing;
- (c) *endpoint models* declare an ``endpoint`` URL (vLLM / HTTP service);
      connectivity is probed with urllib (HEAD/GET) and inference POSTs JSON.

``capability-route-explain`` is a pure **rule router** (it never calls an
LLM): candidates are filtered by a task -> kind mapping, embodiment,
modalities and risk, then sorted by score (kind match dominates, latency/GPU
preferences adjust), and every choice is explained with human-readable
reasons. Selection still requires human confirmation.

Design notes:

- Determinism: routing and demo inference are fully deterministic; random
  flows only in simulation fault injection.
- JSON contract: every command result is plain-JSON (floats rounded, numpy
  types converted), see the worker module contract in docs/.
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import math
import os
import tempfile
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from typing import Any, Optional

from .core import WorkerError, snapshot_environment
from . import simulation
from . import vision

try:
    import numpy as np  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    np = None  # type: ignore[assignment]

try:
    from PIL import Image  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    Image = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# registry
# ---------------------------------------------------------------------------

ALL_KINDS = ["vlm", "vla", "perception", "grasp", "reward", "world-model", "policy", "traditional"]

RISK_ORDER = {"R0": 0, "R1": 1, "R2": 2, "R3": 3}

# Task -> acceptable model kinds (rule router's kind filter).
TASK_KIND_MAP: dict[str, list[str]] = {
    "pick_object": ["policy", "grasp", "perception"],
    "detect_object": ["perception"],
    "vqa": ["vlm"],
    "navigate": ["policy"],
}

BUILTIN_IDS = ("demo.scripted_pick_place", "demo.color_segmentation", "demo.saliency_segmentation")

DEMO_MODELS: list[dict[str, Any]] = [
    {
        "id": "demo.scripted_pick_place",
        "version": "1.0.0",
        "kind": "policy",
        "provider": "builtin",
        "description": "脚本化解析IK抓取策略（调用 simulation 的 PlanarArm IK）",
        "modalities": ["robot_state", "object_pose"],
        "output": "joint_targets",
        "supportedEmbodiments": ["rh_planar_arm"],
        "actionSpace": {"joints": ["shoulder", "elbow", "wrist_joint"], "dof": 3},
        "latencyS": 0.05,
        "expectedLatencyS": 0.05,
        "risk": "R2",
        "license": "MIT",
        "backend": "builtin",
    },
    {
        "id": "demo.color_segmentation",
        "version": "1.0.0",
        "kind": "perception",
        "provider": "builtin",
        "description": "经典颜色分割",
        "modalities": ["rgb_image"],
        "output": "object_centroid",
        "supportedEmbodiments": [],
        "latencyS": 0.01,
        "expectedLatencyS": 0.01,
        "risk": "R1",
        "license": "MIT",
        "backend": "builtin",
    },
    {
        "id": "demo.saliency_segmentation",
        "version": "1.0.0",
        "kind": "perception",
        "provider": "builtin",
        "description": "通用显著度分割",
        "modalities": ["rgb_image"],
        "output": "object_centroid",
        "supportedEmbodiments": [],
        "latencyS": 0.05,
        "expectedLatencyS": 0.05,
        "risk": "R1",
        "license": "MIT",
        "backend": "builtin",
    },
]

BUILTIN_IDS = tuple(m["id"] for m in DEMO_MODELS)


def _default_store_root(args: dict[str, Any]) -> str:
    return args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")


def _registry_path(args: dict[str, Any]) -> str:
    return args.get("registryPath") or os.path.join(_default_store_root(args), "model-registry.json")


def _read_registry_file(path: str) -> list[dict[str, Any]]:
    """Read a registry file (list or ``{"models": [...]}``); empty when absent."""
    if not os.path.exists(path):
        return []
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as error:
        raise WorkerError(f"model registry file {path} is not valid JSON: {error}") from error
    if isinstance(data, list):
        return data
    if isinstance(data, dict) and isinstance(data.get("models"), list):
        return data["models"]
    raise WorkerError(f"model registry file {path} must be a JSON list or an object with a 'models' list")


def _upsert_model(registry: dict[str, dict[str, Any]], entry: Any) -> None:
    if not isinstance(entry, dict) or not entry.get("id"):
        raise WorkerError(f"invalid model entry: expected an object with an 'id', got {entry!r}")
    registry[entry["id"]] = entry


def _merged_registry(args: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Builtin demos + args ``models`` + registry file (file wins on conflicts)."""
    registry: dict[str, dict[str, Any]] = {}
    for model in DEMO_MODELS:
        registry[model["id"]] = dict(model)
    models_arg = args.get("models")
    if models_arg is not None:
        if not isinstance(models_arg, list):
            raise WorkerError("'models' must be a list of model manifest objects")
        for entry in models_arg:
            _upsert_model(registry, entry)
    for entry in _read_registry_file(_registry_path(args)):
        _upsert_model(registry, entry)
    return registry


def _backend_from_decl(model: dict[str, Any]) -> str:
    if model.get("module"):
        return "python-module"
    if model.get("endpoint"):
        return "http-endpoint"
    return "unknown"


def _normalize_manifest(model: dict[str, Any]) -> dict[str, Any]:
    """Fill every manifest field with a default so the inventory is uniform."""
    out: dict[str, Any] = {
        "id": model["id"],
        "version": model.get("version", "0.1.0"),
        "kind": model.get("kind", "traditional"),
        "provider": model.get("provider", "external"),
        "description": model.get("description", ""),
        "modalities": list(model.get("modalities") or []),
        "output": model.get("output", ""),
        "supportedEmbodiments": list(model.get("supportedEmbodiments") or []),
        "risk": model.get("risk", "R1"),
        "backend": model.get("backend") or _backend_from_decl(model),
    }
    if model.get("actionSpace") is not None:
        out["actionSpace"] = model["actionSpace"]
    latency = model.get("expectedLatencyS")
    if latency is None:
        latency = model.get("latencyS")
    if latency is not None:
        out["expectedLatencyS"] = float(latency)
    if model.get("latencyS") is not None:
        out["latencyS"] = float(model["latencyS"])
    if model.get("license") is not None:
        out["license"] = model["license"]
    requirements = model.get("requirements")
    if requirements is not None:
        out["requirements"] = requirements if isinstance(requirements, list) else [requirements]
    if model.get("module"):
        out["module"] = model["module"]
    if model.get("entrypoint"):
        out["entrypoint"] = model["entrypoint"]
    if model.get("endpoint"):
        out["endpoint"] = model["endpoint"]
    return out


def _resolve_model(args: dict[str, Any], model_id: Optional[str] = None) -> dict[str, Any]:
    mid = model_id or args.get("modelId")
    if not mid:
        raise WorkerError("missing required argument 'modelId'")
    registry = _merged_registry(args)
    if mid not in registry:
        raise WorkerError(f"unknown model id {mid!r}; run model-inventory to list registered models")
    return registry[mid]


# ---------------------------------------------------------------------------
# backend detection (health)
# ---------------------------------------------------------------------------

def _module_importable(module: str) -> bool:
    try:
        return importlib.util.find_spec(module) is not None
    except Exception:
        return False


def _import_module(module: str) -> Any:
    try:
        return importlib.import_module(module)
    except Exception as error:  # pragma: no cover - defensive
        raise WorkerError(f"failed to import module {module!r}: {error}") from error


def _endpoint_reachable(url: str, timeout: float = 3.0) -> bool:
    """Probe connectivity with HEAD first, then GET (3s timeout)."""
    for method in ("HEAD", "GET"):
        try:
            request = urllib.request.Request(url, method=method)
            with urllib.request.urlopen(request, timeout=timeout) as _:
                return True
        except Exception:
            continue
    return False


def _detect_backend(model: dict[str, Any]) -> dict[str, Any]:
    """Detect runtime readiness of a model manifest.

    Returns ``{backend: "ready"|"unavailable", details: {...}, issues: [...]}``.
    Builtin demo models are always "ready".
    """
    mid = model["id"]
    if mid in BUILTIN_IDS:
        return {
            "backend": "ready",
            "details": {"builtin": True, "version": model.get("version")},
            "issues": [],
        }
    module = model.get("module")
    endpoint = model.get("endpoint")
    details: dict[str, Any] = {"version": model.get("version")}
    issues: list[str] = []
    backend = "unavailable"
    if module:
        importable = _module_importable(module)
        details["moduleImportable"] = importable
        entrypoint = model.get("entrypoint")
        entrypoint_ok: Optional[bool] = None
        if importable and entrypoint:
            module_obj = _import_module(module)
            entrypoint_ok = callable(getattr(module_obj, entrypoint, None))
            details["moduleEntrypointAvailable"] = entrypoint_ok
        if importable and (not entrypoint or entrypoint_ok):
            backend = "ready"
        else:
            if not importable:
                issues.append(f"module {module!r} is not importable in this environment")
            elif not entrypoint_ok:
                issues.append(f"module {module!r} imports but entrypoint {entrypoint!r} is not a callable")
    elif endpoint:
        reachable = _endpoint_reachable(endpoint)
        details["endpointReachable"] = reachable
        if reachable:
            backend = "ready"
        else:
            issues.append(f"endpoint {endpoint!r} not reachable within 3s (HEAD/GET)")
    else:
        issues.append("model declares neither a python 'module' nor an 'endpoint'; nothing to detect")
    return {"backend": backend, "details": details, "issues": issues}


# ---------------------------------------------------------------------------
# builtin demo adapters
# ---------------------------------------------------------------------------

def _require_cv2():
    if vision.cv2 is None:
        raise WorkerError(
            "opencv-python is not installed in the worker environment; "
            "install it (pip install opencv-python-headless) to run perception demo models"
        )
    return vision.cv2


def _load_rgb_image(inp: dict[str, Any]):
    """Read an image file with cv2 (BGR) and return an RGB array."""
    path = inp.get("imagePath")
    if not path:
        raise WorkerError("missing required input 'imagePath' for perception models")
    if not os.path.exists(path):
        raise WorkerError(f"image not found: {path}")
    cv2 = _require_cv2()
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise WorkerError(f"failed to read image at {path} (unsupported or corrupt file)")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _scripted_pick_place(inp: dict[str, Any]) -> dict[str, Any]:
    """Analytic-IK grasp policy: solve the 2R+XZ wrist for grasp & place poses."""
    object_pose = inp.get("objectPose")
    target_pose = inp.get("targetPose")
    if not isinstance(object_pose, (list, tuple)) or len(object_pose) < 2:
        raise WorkerError("missing required input 'objectPose' as [x, z]")
    if not isinstance(target_pose, (list, tuple)) or len(target_pose) < 2:
        raise WorkerError("missing required input 'targetPose' as [x, z]")
    shoulder = inp.get("shoulder") or [0.0, 0.0, 0.45]
    link_lengths = inp.get("linkLengths") or [0.22, 0.19]
    if len(link_lengths) != 2:
        raise WorkerError("linkLengths must have exactly 2 positive values")
    if len(shoulder) != 3:
        raise WorkerError("shoulder must be [x, y, z]")
    arm = simulation.PlanarArm(
        [float(v) for v in link_lengths],
        [float(v) for v in shoulder],
        cup_reach=simulation.SCENARIO_PICK_PLACE["arm"]["cupReach"],
        joint_ranges=simulation.SCENARIO_PICK_PLACE["arm"].get("jointRanges"),
    )
    grasp_q = arm.ik(float(object_pose[0]), float(object_pose[1]), math.pi, elbow="down")
    if grasp_q is None:
        raise WorkerError("pose unreachable")
    place_q = arm.ik(float(target_pose[0]), float(target_pose[1]), math.pi, elbow="down")
    if place_q is None:
        raise WorkerError("target pose unreachable")
    wrist_x, wrist_z, phi = arm.fk(grasp_q)
    return {
        "jointTargets": [round(float(q), 6) for q in grasp_q],
        "wristPose": {"x": round(wrist_x, 4), "z": round(wrist_z, 4), "phi": round(phi, 4)},
        "placeJointTargets": [round(float(q), 6) for q in place_q],
        "shoulder": [float(v) for v in shoulder],
        "linkLengths": [float(v) for v in link_lengths],
        "reach": {"max": round(arm.reach(), 4), "min": round(arm.min_reach(), 4)},
    }


def _infer_builtin(mid: str, inp: dict[str, Any]) -> dict[str, Any]:
    if mid in ("demo.color_segmentation", "demo.saliency_segmentation"):
        image = _load_rgb_image(inp)
        if mid == "demo.color_segmentation":
            return vision.color_segmentation(image, color=inp.get("color", "red"))
        return vision.saliency_segmentation(image)
    if mid == "demo.scripted_pick_place":
        return _scripted_pick_place(inp)
    raise WorkerError(f"no builtin infer implementation for model {mid!r}")


def _warm_minimal(mid: str) -> None:
    """One minimal real call used to simulate warmup of a builtin model."""
    if mid in ("demo.color_segmentation", "demo.saliency_segmentation"):
        _require_cv2()
        image = np.full((48, 48, 3), (255, 0, 0), dtype=np.uint8)
        if mid == "demo.color_segmentation":
            vision.color_segmentation(image, color="red")
        else:
            vision.saliency_segmentation(image)
        return
    if mid == "demo.scripted_pick_place":
        _scripted_pick_place({"objectPose": [0.30, 0.19], "targetPose": [-0.16, 0.17]})
        return
    raise WorkerError(f"no warmup path for builtin model {mid!r}")


def _default_input_for(mid: str) -> dict[str, Any]:
    if mid in ("demo.color_segmentation", "demo.saliency_segmentation"):
        return {"imagePath": _write_synthetic_red_image()}
    if mid == "demo.scripted_pick_place":
        return {"objectPose": [0.30, 0.19], "targetPose": [-0.16, 0.17]}
    return {}


def _write_synthetic_red_image(size: int = 96) -> str:
    """Write a solid red RGB PNG to a temp file (used for default benchmarks)."""
    if np is None:
        raise WorkerError("numpy is required to synthesize a benchmark image")
    image = np.full((size, size, 3), (255, 0, 0), dtype=np.uint8)
    fd, path = tempfile.mkstemp(suffix=".png")
    os.close(fd)
    if Image is not None:
        Image.fromarray(image).save(path)
        return path
    cv2 = _require_cv2()
    cv2.imwrite(path, image)
    return path


# ---------------------------------------------------------------------------
# external model inference
# ---------------------------------------------------------------------------

def _post_endpoint(url: str, payload: dict[str, Any], timeout_ms: Optional[float]) -> Any:
    timeout = max(0.5, float(timeout_ms) / 1000.0) if timeout_ms else 5.0
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.HTTPError as error:
        raise WorkerError(f"endpoint HTTP error {error.code}: {error.reason}") from error
    except urllib.error.URLError as error:
        raise WorkerError(f"endpoint request failed: {error.reason}") from error
    try:
        return json.loads(raw)
    except json.JSONDecodeError as error:
        raise WorkerError(f"endpoint returned a non-JSON response: {error}") from error


def _infer_external(model: dict[str, Any], inp: dict[str, Any], timeout_ms: Optional[float]) -> tuple[dict[str, Any], Optional[float]]:
    """Run an external model; returns ``(outcome, latencyMs)``.

    ``outcome`` is ``{"status": "ok", "result": ...}`` or a structured
    ``{"status": "unavailable", "details": {...}, "issues": [...]}`` diagnostic.
    A call failure on a nominally available backend raises :class:`WorkerError`.
    """
    module = model.get("module")
    endpoint = model.get("endpoint")
    if module:
        if not _module_importable(module):
            return (
                {"status": "unavailable", "details": {"moduleImportable": False}, "issues": [f"module {module!r} is not importable in this environment"]},
                None,
            )
        module_obj = _import_module(module)
        entrypoint = model.get("entrypoint", "infer")
        fn = getattr(module_obj, entrypoint, None)
        if not callable(fn):
            return (
                {"status": "unavailable", "details": {"moduleImportable": True, "moduleEntrypointAvailable": False}, "issues": [f"module {module!r} imports but entrypoint {entrypoint!r} is not a callable"]},
                None,
            )
        started = time.perf_counter()
        try:
            out = fn(inp)
        except WorkerError:
            raise
        except Exception as error:
            raise WorkerError(f"external model entrypoint raised {type(error).__name__}: {error}") from error
        return {"status": "ok", "result": out}, (time.perf_counter() - started) * 1000.0
    if endpoint:
        started = time.perf_counter()
        out = _post_endpoint(endpoint, inp, timeout_ms)
        return {"status": "ok", "result": out}, (time.perf_counter() - started) * 1000.0
    return (
        {"status": "unavailable", "details": {}, "issues": ["external model declares neither 'module' nor 'endpoint'"]},
        None,
    )


# ---------------------------------------------------------------------------
# JSON helpers
# ---------------------------------------------------------------------------

def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def _input_summary(inp: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(inp, dict):
        return {"type": type(inp).__name__}
    summary: dict[str, Any] = {}
    for key, value in inp.items():
        if key == "imagePath":
            summary[key] = os.path.abspath(str(value))
        elif isinstance(value, dict):
            summary[key] = {"keys": sorted(value.keys())}
        elif isinstance(value, (list, tuple)):
            summary[key] = _json_safe(list(value)) if len(value) <= 8 else {"len": len(value)}
        else:
            summary[key] = type(value).__name__
    return summary


def _output_summary(value: Any, depth: int = 0) -> Any:
    if depth > 3:
        return {"truncated": True}
    if isinstance(value, dict):
        out: dict[str, Any] = {}
        for key, item in list(value.items())[:10]:
            out[key] = _output_summary(item, depth + 1)
        return out
    if isinstance(value, (list, tuple)):
        items = [_output_summary(item, depth + 1) for item in list(value)[:8]]
        if len(value) > 8:
            items.append({"more": len(value) - 8})
        return items
    if isinstance(value, np.ndarray):
        return {"ndarray": list(value.shape), "dtype": str(value.dtype)}
    return _json_safe(value)


def _args_summary(args: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in args.items():
        if isinstance(value, dict):
            out[key] = {"keys": sorted(value.keys())}
        elif isinstance(value, list):
            out[key] = {"len": len(value)}
        else:
            out[key] = value
    return out


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _latency_stats(latencies: list[float]) -> dict[str, Optional[float]]:
    if not latencies:
        return {"mean": None, "p50": None, "p90": None, "max": None, "min": None}
    sorted_values = sorted(latencies)
    return {
        "mean": round(sum(sorted_values) / len(sorted_values), 2),
        "p50": round(_percentile(sorted_values, 50), 2),
        "p90": round(_percentile(sorted_values, 90), 2),
        "max": round(sorted_values[-1], 2),
        "min": round(sorted_values[0], 2),
    }


def _percentile(sorted_values: list[float], p: float) -> float:
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = (len(sorted_values) - 1) * (p / 100.0)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[int(position)]
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------

def cmd_model_inventory(args: dict[str, Any]) -> dict[str, Any]:
    registry = _merged_registry(args)
    models_out = [_normalize_manifest(model) for model in registry.values()]
    models_out.sort(key=lambda m: m["id"])
    by_kind: dict[str, int] = {}
    for model in models_out:
        by_kind[model["kind"]] = by_kind.get(model["kind"], 0) + 1
    path = _registry_path(args)
    return {
        "ok": True,
        "models": models_out,
        "counts": {"total": len(models_out), "byKind": by_kind},
        "builtin": [model["id"] for model in DEMO_MODELS],
        "registryPath": os.path.abspath(path),
        "registryFileFound": os.path.exists(path),
        "inputArgs": _args_summary(args),
    }


def cmd_model_health(args: dict[str, Any]) -> dict[str, Any]:
    model = _resolve_model(args)
    detection = _detect_backend(model)
    return {
        "ok": True,
        "modelId": model["id"],
        "backend": detection["backend"],
        "details": detection["details"],
        "issues": detection["issues"],
        "inputArgs": _args_summary(args),
    }


def cmd_model_warmup(args: dict[str, Any]) -> dict[str, Any]:
    model = _resolve_model(args)
    mid = model["id"]
    timeout_s = float(args.get("timeoutS", 30))
    if timeout_s <= 0:
        raise WorkerError("timeoutS must be positive")
    if mid in BUILTIN_IDS:
        started = time.perf_counter()
        try:
            _warm_minimal(mid)
        except WorkerError as error:
            return {
                "ok": True,
                "modelId": mid,
                "warmed": False,
                "latencyS": None,
                "backend": "builtin",
                "issues": [str(error)],
                "inputArgs": _args_summary(args),
            }
        return {
            "ok": True,
            "modelId": mid,
            "warmed": True,
            "latencyS": round(time.perf_counter() - started, 4),
            "backend": "builtin",
            "note": "builtin demo model warmed with one minimal call",
            "inputArgs": _args_summary(args),
        }
    detection = _detect_backend(model)
    if detection["backend"] == "ready":
        return {
            "ok": True,
            "modelId": mid,
            "warmed": True,
            "latencyS": None,
            "backend": "ready",
            "details": detection["details"],
            "inputArgs": _args_summary(args),
        }
    return {
        "ok": True,
        "modelId": mid,
        "warmed": False,
        "latencyS": None,
        "backend": "unavailable",
        "details": detection["details"],
        "issues": detection["issues"],
        "hint": "install the model's python requirements or start its endpoint service, then re-check with model-health",
        "inputArgs": _args_summary(args),
    }


def cmd_model_infer(args: dict[str, Any]) -> dict[str, Any]:
    model = _resolve_model(args)
    mid = model["id"]
    inp = args.get("input") or {}
    timeout_ms = args.get("timeoutMs")
    if mid in BUILTIN_IDS:
        started = time.perf_counter()
        result = _infer_builtin(mid, inp)
        latency_ms = (time.perf_counter() - started) * 1000.0
        return {
            "ok": True,
            "modelId": mid,
            "backend": "builtin",
            "result": _json_safe(result),
            "latencyMs": round(latency_ms, 2),
            "trace": {"startedAt": _now_iso(), "finishedAt": _now_iso()},
            "inputSummary": _input_summary(inp),
            "outputSummary": _output_summary(result),
            "inputArgs": _args_summary(args),
        }
    outcome, latency_ms = _infer_external(model, inp, timeout_ms)
    if outcome["status"] != "ok":
        return {
            "ok": True,
            "modelId": mid,
            "backend": "unavailable",
            "details": outcome.get("details", {}),
            "issues": outcome.get("issues", []),
            "hint": "install the model's python requirements or start its endpoint service, then re-check with model-health",
            "inputSummary": _input_summary(inp),
            "outputSummary": None,
            "inputArgs": _args_summary(args),
        }
    return {
        "ok": True,
        "modelId": mid,
        "backend": "python-module" if model.get("module") else "http-endpoint",
        "result": _json_safe(outcome["result"]),
        "latencyMs": round(latency_ms, 2) if latency_ms is not None else None,
        "inputSummary": _input_summary(inp),
        "outputSummary": _output_summary(outcome["result"]),
        "inputArgs": _args_summary(args),
    }


def cmd_model_benchmark(args: dict[str, Any]) -> dict[str, Any]:
    model = _resolve_model(args)
    mid = model["id"]
    iterations = int(args.get("iterations", 20))
    if iterations < 1:
        raise WorkerError("iterations must be >= 1")
    inp = args.get("input")
    environment = snapshot_environment()
    empty_stats = {"mean": None, "p50": None, "p90": None, "max": None, "min": None}

    if mid in BUILTIN_IDS:
        if inp is None:
            inp = _default_input_for(mid)
        latencies: list[float] = []
        for _ in range(iterations):
            started = time.perf_counter()
            _infer_builtin(mid, inp)
            latencies.append((time.perf_counter() - started) * 1000.0)
        stats = _latency_stats(latencies)
        total_s = sum(latencies) / 1000.0
        return {
            "ok": True,
            "modelId": mid,
            "iterations": iterations,
            "latencyMs": stats,
            "throughputHz": round(iterations / total_s, 2) if total_s > 0 else None,
            "environment": environment,
            "notes": ["builtin demo model benchmarked with real execution timing"],
            "inputSummary": _input_summary(inp),
            "inputArgs": _args_summary(args),
        }

    detection = _detect_backend(model)
    if detection["backend"] != "ready":
        return {
            "ok": True,
            "modelId": mid,
            "iterations": 0,
            "latencyMs": empty_stats,
            "throughputHz": None,
            "environment": environment,
            "notes": ["external model backend unavailable; benchmark skipped"] + detection["issues"],
            "inputArgs": _args_summary(args),
        }
    if inp is None:
        inp = {}
    latencies = []
    for _ in range(iterations):
        started = time.perf_counter()
        outcome, latency_ms = _infer_external(model, inp, args.get("timeoutMs"))
        if outcome["status"] != "ok":
            return {
                "ok": True,
                "modelId": mid,
                "iterations": 0,
                "latencyMs": empty_stats,
                "throughputHz": None,
                "environment": environment,
                "notes": ["external model became unavailable during benchmark"] + outcome.get("issues", []),
                "inputArgs": _args_summary(args),
            }
        latencies.append(latency_ms if latency_ms is not None else (time.perf_counter() - started) * 1000.0)
    stats = _latency_stats(latencies)
    total_s = sum(latencies) / 1000.0
    return {
        "ok": True,
        "modelId": mid,
        "iterations": iterations,
        "latencyMs": stats,
        "throughputHz": round(iterations / total_s, 2) if total_s > 0 else None,
        "environment": environment,
        "notes": ["external model benchmarked through its adapter"],
        "inputArgs": _args_summary(args),
    }


def _route_score(model: dict[str, Any], task: str, kinds: list[str], prefer_low: bool, gpu: bool) -> tuple[float, list[str]]:
    reasons: list[str] = []
    score = 0.0
    rank = kinds.index(model["kind"]) if model["kind"] in kinds else len(kinds)
    score += max(0, 3 - rank)
    latency = float(model.get("expectedLatencyS") if model.get("expectedLatencyS") is not None else model.get("latencyS") or 1.0)
    if prefer_low:
        bonus = round(max(0.0, 4.0 - latency * 20.0), 2)
        score += bonus
        reasons.append(f"low-latency preference: expectedLatencyS={latency:.3f}s -> +{bonus}")
    if gpu:
        requirements = str(model.get("requirements") or "")
        if model["kind"] in ("vlm", "vla") or "torch" in requirements or "cuda" in requirements.lower():
            score += 1.5
            reasons.append("GPU available: learned model / GPU-backed backend preferred")
    return round(score, 2), reasons


def cmd_capability_route_explain(args: dict[str, Any]) -> dict[str, Any]:
    task = args.get("task")
    if not task:
        raise WorkerError("missing required argument 'task'")
    modalities = args.get("modalities") or []
    embodiments = args.get("embodiment") or []
    prefer_low = bool(args.get("preferLowLatency"))
    max_risk = args.get("maxRisk")
    gpu = bool(args.get("gpuAvailable"))
    if max_risk is not None and max_risk not in RISK_ORDER:
        raise WorkerError(f"invalid maxRisk {max_risk!r}; use one of R0..R3")

    kinds = TASK_KIND_MAP.get(task, ALL_KINDS)
    registry = _merged_registry(args)
    candidates: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    for raw in registry.values():
        model = _normalize_manifest(raw)
        reasons: list[str] = []
        if model["kind"] not in kinds:
            rejected.append({"modelId": model["id"], "reason": f"kind {model['kind']!r} not in task kind set {kinds}"})
            continue
        reasons.append(f"kind {model['kind']!r} matches task {task!r}")
        if embodiments:
            supported = set(model.get("supportedEmbodiments") or [])
            if supported and not supported.intersection(embodiments):
                rejected.append({"modelId": model["id"], "reason": f"embodiment {sorted(supported)} incompatible with requested {embodiments}"})
                continue
            reasons.append(f"embodiment compatible with {embodiments}")
        if modalities:
            supported_mods = set(model.get("modalities") or [])
            if supported_mods and not supported_mods.intersection(modalities):
                rejected.append({"modelId": model["id"], "reason": f"modalities {sorted(supported_mods)} do not cover requested {modalities}"})
                continue
            reasons.append(f"modalities cover {modalities}")
        if max_risk is not None:
            model_risk = RISK_ORDER.get(model["risk"], 3)
            if model_risk > RISK_ORDER[max_risk]:
                rejected.append({"modelId": model["id"], "reason": f"risk {model['risk']} exceeds maxRisk {max_risk}"})
                continue
            reasons.append(f"risk {model['risk']} within maxRisk {max_risk}")
        score, score_reasons = _route_score(model, task, kinds, prefer_low, gpu)
        reasons.extend(score_reasons)
        candidates.append(
            {
                "modelId": model["id"],
                "kind": model["kind"],
                "score": score,
                "reasons": reasons,
                "risk": model["risk"],
                "expectedLatencyS": model.get("expectedLatencyS"),
            }
        )

    candidates.sort(key=lambda c: (-c["score"], c.get("expectedLatencyS") or 1e9, c["modelId"]))
    selected = None
    fallback = None
    if candidates:
        selected = {"modelId": candidates[0]["modelId"], "reasons": candidates[0]["reasons"]}
    if len(candidates) > 1:
        fallback = {"modelId": candidates[1]["modelId"], "reasons": candidates[1]["reasons"]}
    note = (
        "规则路由，不调用 LLM；无候选满足任务/模态/具身/风险过滤，selected 为 null"
        if not candidates
        else "规则路由，不调用 LLM；选择需人工确认"
    )
    return {
        "ok": True,
        "task": task,
        "candidates": candidates,
        "selected": selected,
        "fallback": fallback,
        "rejected": rejected,
        "note": note,
        "inputArgs": _args_summary(args),
    }


def _policy_base_fault(policy: dict[str, Any]) -> dict[str, Any]:
    offset = policy.get("graspOffset")
    if offset is None:
        return {}
    if isinstance(offset, (int, float)):
        value = float(offset)
        return {"perception_offset_px": [value, value]}
    if isinstance(offset, (list, tuple)) and len(offset) >= 2:
        return {"perception_offset_px": [float(offset[0]), float(offset[1])]}
    raise WorkerError(f"graspOffset must be a number or [dx, dy], got {offset!r}")


def _compare_conclusion(summary: dict[str, Any]) -> str:
    policy_a = summary.get("policyA", {})
    policy_b = summary.get("policyB", {})
    rate_a, rate_b = policy_a.get("successRate"), policy_b.get("successRate")
    if rate_a is None or rate_b is None:
        return "insufficient rollout data to compare policies"
    if rate_a > rate_b:
        return f"policyA 成功率 {rate_a:.0%} > policyB {rate_b:.0%}：策略 A（无抓取偏移）在仿真中表现更优，差异由 graspOffset 注入的感知偏移解释"
    if rate_b > rate_a:
        return f"policyB 成功率 {rate_b:.0%} > policyA {rate_a:.0%}：带 graspOffset 的策略在仿真中表现更优（通常由随机性与故障组合引起）"
    return f"两策略成功率相同（{rate_a:.0%}），无法区分；可增加 seed 或故障组合"


def cmd_policy_rollout_compare(args: dict[str, Any]) -> dict[str, Any]:
    scenario_name = args.get("scenario") or "mujoco_pick_place"
    try:
        scenario = simulation.load_scenario(scenario_name)
    except Exception as error:
        raise WorkerError(f"cannot load scenario {scenario_name!r}: {error}") from error
    if simulation.mujoco is None:
        raise WorkerError("mujoco is not importable in this environment; policy-rollout-compare requires MuJoCo")

    policy_a = args.get("policyA") or {"modelId": "demo.scripted_pick_place"}
    policy_b = args.get("policyB") or {"modelId": "demo.scripted_pick_place", "graspOffset": [18.0, 6.0]}
    seeds = args.get("seeds") or [42, 43]
    faults = args.get("faults") or [{}]
    if not isinstance(policy_a, dict) or not isinstance(policy_b, dict):
        raise WorkerError("policyA/policyB must be model manifest objects")
    if not isinstance(seeds, list) or not seeds:
        raise WorkerError("'seeds' must be a non-empty list of integers")
    if not isinstance(faults, list):
        raise WorkerError("'faults' must be a list of fault dicts")

    matrix: list[dict[str, Any]] = []
    for label, policy in (("A", policy_a), ("B", policy_b)):
        base = _policy_base_fault(policy)
        model_id = policy.get("modelId", "demo.scripted_pick_place")
        for seed in seeds:
            for fault in faults:
                merged = dict(base)
                merged.update(fault or {})
                try:
                    run, _ = simulation.run_pick_place(scenario, merged, int(seed))
                except Exception as error:
                    raise WorkerError(f"policy {label} seed {seed} run failed: {error}") from error
                metrics = {
                    key: run.metrics.get(key)
                    for key in ("success", "steps", "durationS", "trackingErrorRms", "inTargetZone", "grasped", "slipped", "perceptionRoute", "renderer")
                }
                matrix.append(
                    {
                        "policy": label,
                        "policyModelId": model_id,
                        "seed": int(seed),
                        "fault": merged,
                        "success": bool(run.metrics.get("success")),
                        "metrics": metrics,
                        "anomalies": [anomaly.kind for anomaly in run.anomalies],
                    }
                )

    summary: dict[str, Any] = {}
    for label in ("A", "B"):
        rows = [row for row in matrix if row["policy"] == label]
        successes = [row for row in rows if row["success"]]
        rate = len(successes) / len(rows) if rows else None
        rms_values = [row["metrics"].get("trackingErrorRms") for row in rows if row["metrics"].get("trackingErrorRms") is not None]
        policy = policy_a if label == "A" else policy_b
        summary[f"policy{label}"] = {
            "modelId": policy.get("modelId", "demo.scripted_pick_place"),
            "runs": len(rows),
            "successRate": round(rate, 3) if rate is not None else None,
            "avgTrackingRms": round(sum(rms_values) / len(rms_values), 5) if rms_values else None,
        }

    return {
        "ok": True,
        "matrix": matrix,
        "summary": summary,
        "conclusion": _compare_conclusion(summary),
        "scenario": scenario_name,
        "note": "仅仿真对比，不代表真机；策略差异由 graspOffset（映射到 simulation 的 perception_offset 故障注入）模拟",
        "inputArgs": _args_summary(args),
    }


# ---------------------------------------------------------------------------
# module interface (worker module contract)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "model-inventory": cmd_model_inventory,
    "model-health": cmd_model_health,
    "model-warmup": cmd_model_warmup,
    "model-infer": cmd_model_infer,
    "model-benchmark": cmd_model_benchmark,
    "capability-route-explain": cmd_capability_route_explain,
    "policy-rollout-compare": cmd_policy_rollout_compare,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "model.inventory",
        "kind": "model",
        "provider": "robotic-harness-worker",
        "input": {"registryPath": "string?"},
        "output": "merged model registry manifest + counts by kind",
        "risk": "R0-readonly",
        "description": "List the embodied-model registry (builtin demo adapters + external registrations).",
    },
    {
        "id": "model.health",
        "kind": "model",
        "provider": "robotic-harness-worker",
        "input": {"modelId": "string"},
        "output": "backend readiness: builtin | ready | unavailable",
        "risk": "R0-readonly",
        "description": "Detect a model's backend: module importability, endpoint reachability, version.",
    },
    {
        "id": "model.warmup",
        "kind": "model",
        "provider": "robotic-harness-worker",
        "input": {"modelId": "string", "timeoutS": "number?"},
        "output": "warmup status + latency",
        "risk": "R1-derive",
        "description": "Warm a builtin demo model with one minimal timed call; probe external readiness.",
    },
    {
        "id": "model.infer",
        "kind": "model",
        "provider": "robotic-harness-worker",
        "input": {"modelId": "string", "input": "object", "timeoutMs": "number?"},
        "output": "model result + latency + trace + summaries",
        "risk": "R1-derive",
        "description": "Run one inference through the demo adapter or an external module/endpoint backend.",
    },
    {
        "id": "model.benchmark",
        "kind": "model",
        "provider": "robotic-harness-worker",
        "input": {"modelId": "string", "iterations": "number?", "input": "object?"},
        "output": "latency percentiles + throughput + environment snapshot",
        "risk": "R0-readonly",
        "description": "Benchmark a model on a fixed input with real execution timing.",
    },
    {
        "id": "capability.route_explain",
        "kind": "model",
        "provider": "robotic-harness-worker",
        "input": {"task": "string", "modalities": "string[]?", "embodiment": "string[]?", "preferLowLatency": "bool?", "maxRisk": "string?", "gpuAvailable": "bool?"},
        "output": "ranked candidates + selection reasons",
        "risk": "R0-readonly",
        "description": "Rule-based capability router (no LLM): filter and rank models for a task with explanations.",
    },
    {
        "id": "policy.rollout_compare",
        "kind": "policy",
        "provider": "robotic-harness-worker",
        "input": {"policyA": "object?", "policyB": "object?", "scenario": "string?", "seeds": "number[]?", "faults": "object[]?"},
        "output": "comparison matrix + success rates + conclusion",
        "risk": "R2-simulation",
        "description": "Compare two policies in the MuJoCo pick-place simulation via graspOffset fault injection.",
    },
]
