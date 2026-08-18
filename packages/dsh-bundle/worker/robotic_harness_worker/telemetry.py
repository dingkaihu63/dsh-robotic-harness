"""Telemetry analysis, anomaly scanning, run comparison and timeline export.

Implements the plan's chapter 13 "telemetry / anomaly / multi-run comparison"
layer on top of the existing deterministic diagnostics (``diagnostics.py``):

- channel extraction from ``telemetry.jsonl`` rows (scalar / vector / state),
- time-window extraction with per-channel statistics,
- deterministic layer-2 anomaly scanning (threshold / rate / spike over MAD,
  plus constant and NaN/Inf channel checks) -- numpy only, no scipy,
- failure evidence collection around a focused window, with an optional
  ``diagnostics.diagnose`` case attached and persisted through ``RunStore``,
- nearest-neighbour aligned comparison of two runs' telemetry,
- standalone timeline export reusing ``report.timeline_html``.

This module deliberately does NOT re-implement the layer-1 diagnostic rules
(see ``diagnostics.py``); it operates on the raw channel level.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Optional

import numpy as np

from .core import RunStore, WorkerError
from .diagnostics import diagnose, load_run_data
from .report import timeline_html

DEFAULT_WINDOW_S = 1.0  # sliding window for the rate detector
DEFAULT_SPIKE_SIGMA = 6.0  # |value - median| > sigma * MAD counts as a spike
DIVERGENCE_EPSILON = 1e-6  # |a - b| above this counts as a divergence
_ARTIFACT_NAMES = ("joints.png", "tracking.png", "trajectory.png", "scene.png")


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _round(value: Any, n: int = 6) -> Any:
    """Round a numeric value; pass None/non-numeric through untouched."""
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return value
    if not math.isfinite(number):
        return None
    return round(number, n)


def _json_value(value: Any) -> Any:
    """Convert a raw row value into a strict-JSON-safe value."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        number = float(value)
        return None if not math.isfinite(number) else number
    return value


def _to_num(value: Any) -> Optional[float]:
    """Extract a float from a scalar value; None for missing/non-numeric."""
    if value is None:
        return None
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _extract(row: dict[str, Any], path: tuple[Any, ...]) -> Any:
    """Navigate a telemetry row along a path of keys/indices; None on any miss."""
    value: Any = row
    for part in path:
        if value is None or not isinstance(value, (dict, list, tuple)):
            return None
        try:
            value = value[part]
        except (KeyError, IndexError, TypeError):
            return None
    return value


def _load_run(run_path: str) -> tuple[Any, list[dict[str, Any]]]:
    """Load (run, telemetry) via diagnostics.load_run_data, wrapped in WorkerError."""
    if not run_path:
        raise WorkerError("missing required argument 'runPath'")
    try:
        return load_run_data(run_path)
    except (FileNotFoundError, json.JSONDecodeError, KeyError, OSError, ValueError) as error:
        raise WorkerError(f"cannot load run data from {run_path}: {error}") from error


def _store_for(args: dict[str, Any]) -> RunStore:
    root = args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")
    store = RunStore(root)
    store.ensure()
    return store


def _median_interval(t: np.ndarray) -> Optional[float]:
    if len(t) < 2:
        return None
    diffs = np.diff(t)
    diffs = diffs[diffs > 0]
    if len(diffs) == 0:
        return None
    return float(np.median(diffs))


# ---------------------------------------------------------------------------
# channel model
# ---------------------------------------------------------------------------


class _ChannelSpec:
    __slots__ = ("name", "kind", "path")

    def __init__(self, name: str, kind: str, path: tuple[Any, ...]) -> None:
        self.name = name
        self.kind = kind  # "scalar" | "vector" | "state"
        self.path = path


def _first_concrete(path: tuple[Any, ...], rows: list[dict[str, Any]]) -> Any:
    """First non-None value for a path (used when the first row holds None)."""
    for row in rows[1:]:
        value = _extract(row, path)
        if value is not None:
            return value
    return None


def _walk_channel(
    name: str,
    value: Any,
    path: tuple[Any, ...],
    rows: list[dict[str, Any]],
    specs: list[_ChannelSpec],
) -> None:
    """Recursively discover channels from the structure of the first row."""
    if value is None:
        value = _first_concrete(path, rows)
        if value is None:
            specs.append(_ChannelSpec(name, "scalar", path))
            return
    if isinstance(value, bool):
        specs.append(_ChannelSpec(name, "scalar", path))
    elif isinstance(value, (int, float)):
        specs.append(_ChannelSpec(name, "scalar", path))
    elif isinstance(value, str):
        specs.append(_ChannelSpec(name, "state", path))
    elif isinstance(value, (list, tuple)):
        if value and all(isinstance(item, (int, float)) for item in value):
            # numeric vector -> expand to name.0, name.1, ...
            for index in range(len(value)):
                specs.append(_ChannelSpec(f"{name}.{index}", "vector", path + (index,)))
        else:
            specs.append(_ChannelSpec(name, "state", path))
    elif isinstance(value, dict):
        for sub_key, sub_value in value.items():
            _walk_channel(f"{name}.{sub_key}", sub_value, path + (sub_key,), rows, specs)
    else:
        specs.append(_ChannelSpec(name, "state", path))


def _discover_channels(rows: list[dict[str, Any]]) -> list[_ChannelSpec]:
    if not rows:
        raise WorkerError("telemetry is empty")
    specs: list[_ChannelSpec] = []
    for key, value in rows[0].items():
        _walk_channel(key, value, (key,), rows, specs)
    return specs


class _Frame:
    """Columnar view over a telemetry row list."""

    def __init__(self, rows: list[dict[str, Any]]) -> None:
        if not rows:
            raise WorkerError("telemetry is empty")
        self.rows = rows
        self.specs = _discover_channels(rows)
        try:
            self.t = np.asarray([float(row["t"]) for row in rows], dtype=float)
        except (KeyError, TypeError, ValueError) as error:
            raise WorkerError(f"telemetry rows must contain a numeric 't': {error}") from error
        self.interval = _median_interval(self.t)
        self._numeric: dict[str, np.ndarray] = {}
        self._missing: dict[str, int] = {}
        for spec in self.specs:
            raw = [_extract(row, spec.path) for row in rows]
            nums = [_to_num(value) for value in raw]
            if spec.kind == "state":
                # a present string value is not "missing"
                self._missing[spec.name] = sum(1 for value in raw if value is None)
            else:
                self._missing[spec.name] = sum(1 for value in nums if value is None)
            arr = np.full(len(rows), np.nan, dtype=float)
            for index, value in enumerate(nums):
                if value is not None:
                    arr[index] = value
            self._numeric[spec.name] = arr

    def numeric(self, name: str) -> np.ndarray:
        """Float array for a channel (NaN marks missing/non-numeric values)."""
        return self._numeric[name]

    def missing(self, name: str) -> int:
        return self._missing[name]

    def sample_rate_hz(self) -> Optional[float]:
        if self.interval is None or self.interval <= 0:
            return None
        return 1.0 / self.interval


def _select_specs(frame: _Frame, requested: Optional[list[str]]) -> list[_ChannelSpec]:
    """Resolve a user channel list against the frame; WorkerError on unknowns."""
    by_name = {spec.name: spec for spec in frame.specs}
    if not requested:
        return list(frame.specs)
    unknown = [name for name in requested if name not in by_name]
    if unknown:
        raise WorkerError(f"unknown channel(s): {unknown}; available: {sorted(by_name)}")
    return [by_name[name] for name in requested]


# ---------------------------------------------------------------------------
# command: telemetry-channels
# ---------------------------------------------------------------------------


def cmd_telemetry_channels(args: dict[str, Any]) -> dict[str, Any]:
    """Inventory the channels of a run's telemetry.jsonl."""
    run_path = args.get("runPath") or args.get("runDir")
    run, rows = _load_run(run_path)
    frame = _Frame(rows)
    channels: list[dict[str, Any]] = []
    for spec in frame.specs:
        entry: dict[str, Any] = {"name": spec.name, "kind": spec.kind}
        entry["length"] = len(frame.rows)
        entry["sampleRateHz"] = _round(frame.sample_rate_hz(), 3)
        entry["missing"] = frame.missing(spec.name)
        if spec.kind in ("scalar", "vector"):
            values = frame.numeric(spec.name)
            finite = values[np.isfinite(values)]
            if len(finite):
                entry["min"] = _round(float(finite.min()), 6)
                entry["max"] = _round(float(finite.max()), 6)
        channels.append(entry)
    notes = [
        f"{len(rows)} rows over {frame.t[0]:.3f}s..{frame.t[-1]:.3f}s",
        "structure inferred from the first telemetry row; vectors expanded as name.index",
    ]
    if frame.interval is not None:
        notes.append(f"median interval {frame.interval:.4f}s (~{_round(1.0 / frame.interval, 2)} Hz)")
    missing_total = sum(frame.missing(spec.name) for spec in frame.specs)
    if missing_total:
        notes.append(f"{missing_total} missing values across all channels")
    state_count = sum(1 for spec in frame.specs if spec.kind == "state")
    if state_count:
        notes.append(f"{state_count} state channel(s) (non-numeric, e.g. phase)")
    return {
        "ok": True,
        "runId": run.id,
        "channels": channels,
        "notes": notes,
        "inputArgs": {"runPath": run_path},
    }


# ---------------------------------------------------------------------------
# command: telemetry-window
# ---------------------------------------------------------------------------


def cmd_telemetry_window(args: dict[str, Any]) -> dict[str, Any]:
    """Extract a time window with per-channel values and statistics."""
    run_path = args.get("runPath") or args.get("runDir")
    run, rows = _load_run(run_path)
    frame = _Frame(rows)
    start = float(args["startS"]) if args.get("startS") is not None else float(frame.t[0])
    end = float(args["endS"]) if args.get("endS") is not None else float(frame.t[-1])
    if start > end:
        raise WorkerError(f"startS ({start:g}) must be <= endS ({end:g})")
    mask = (frame.t >= start) & (frame.t <= end)
    indices = np.flatnonzero(mask)
    if len(indices) == 0:
        raise WorkerError(f"no samples in window [{start:g}, {end:g}]")
    specs = _select_specs(frame, args.get("channels"))
    window_rows = [frame.rows[int(index)] for index in indices]
    out_channels: list[dict[str, Any]] = []
    for spec in specs:
        entry: dict[str, Any] = {"name": spec.name, "kind": spec.kind}
        entry["values"] = [_json_value(_extract(row, spec.path)) for row in window_rows]
        if spec.kind in ("scalar", "vector"):
            arr = frame.numeric(spec.name)[indices]
            finite = arr[np.isfinite(arr)]
            if len(finite):
                entry["stats"] = {
                    "count": int(len(finite)),
                    "mean": _round(float(finite.mean()), 6),
                    "std": _round(float(finite.std()), 6),
                    "min": _round(float(finite.min()), 6),
                    "max": _round(float(finite.max()), 6),
                }
            else:
                entry["stats"] = {"count": 0, "mean": None, "std": None, "min": None, "max": None}
        else:
            entry["stats"] = None
        out_channels.append(entry)
    return {
        "ok": True,
        "runId": run.id,
        "window": {"startS": _round(start, 6), "endS": _round(end, 6)},
        "samples": int(len(indices)),
        "channels": out_channels,
        "inputArgs": {"runPath": run_path, "startS": _round(start, 6), "endS": _round(end, 6)},
    }


# ---------------------------------------------------------------------------
# command: anomaly-scan
# ---------------------------------------------------------------------------


def _nonfinite_runs(name: str, t: np.ndarray, values: np.ndarray) -> list[dict[str, Any]]:
    """One anomaly per contiguous run of NaN/Inf samples."""
    bad = ~np.isfinite(values)
    if not bad.any():
        return []
    out: list[dict[str, Any]] = []
    n = len(values)
    index = 0
    while index < n:
        if not bad[index]:
            index += 1
            continue
        j = index
        while j + 1 < n and bad[j + 1]:
            j += 1
        out.append(
            {
                "t": _round(float(t[index]), 6),
                "channel": name,
                "method": "nonfinite",
                "severity": "error",
                "value": None,
                "detail": f"{j - index + 1} non-finite sample(s) (NaN/Inf)",
                "windowStartS": _round(float(t[index]), 6),
                "windowEndS": _round(float(t[j]), 6),
            }
        )
        index = j + 1
    return out


def _threshold_anomalies(name: str, t: np.ndarray, values: np.ndarray, cfg: dict[str, Any]) -> list[dict[str, Any]]:
    try:
        vmin = float(cfg["min"]) if cfg.get("min") is not None else None
        vmax = float(cfg["max"]) if cfg.get("max") is not None else None
    except (TypeError, ValueError) as error:
        raise WorkerError(f"thresholds[{name!r}] min/max must be numbers: {error}") from error
    finite = np.isfinite(values)
    mask = np.zeros(len(values), dtype=bool)
    if vmin is not None:
        mask |= finite & (values < vmin)
    if vmax is not None:
        mask |= finite & (values > vmax)
    bounds = []
    if vmin is not None:
        bounds.append(f"min={vmin:g}")
    if vmax is not None:
        bounds.append(f"max={vmax:g}")
    out: list[dict[str, Any]] = []
    for index in np.flatnonzero(mask):
        value = float(values[index])
        out.append(
            {
                "t": _round(float(t[index]), 6),
                "channel": name,
                "method": "threshold",
                "severity": "error",
                "value": _round(value, 6),
                "detail": f"value {value:.6g} outside [{', '.join(bounds)}]",
                "windowStartS": _round(float(t[index]), 6),
                "windowEndS": _round(float(t[index]), 6),
            }
        )
    return out


def _rate_anomalies(name: str, t: np.ndarray, values: np.ndarray, max_rate: float, window_s: float) -> list[dict[str, Any]]:
    """Max |Δv/Δt| over any pair inside the sliding window exceeding maxRate."""
    out: list[dict[str, Any]] = []
    finite = np.isfinite(values)
    if finite.sum() < 2:
        return out
    n = len(values)
    for j in range(1, n):
        if not finite[j]:
            continue
        i0 = int(np.searchsorted(t, float(t[j]) - window_s, side="left"))
        if i0 >= j:
            continue
        idx = np.flatnonzero(finite[i0:j]) + i0
        if len(idx) == 0:
            continue
        tv = t[idx]
        dt = float(t[j]) - tv
        # duplicate timestamps make dt == 0 -> inf slope -> false maxRate hits
        ok = dt > 0
        if not np.any(ok):
            continue
        slopes = np.full(len(tv), -np.inf)
        slopes[ok] = np.abs(values[j] - values[idx][ok]) / dt[ok]
        k = int(np.argmax(slopes))
        if slopes[k] > max_rate:
            out.append(
                {
                    "t": _round(float(t[j]), 6),
                    "channel": name,
                    "method": "rate",
                    "severity": "warning",
                    "value": _round(float(values[j]), 6),
                    "detail": f"max |Δv/Δt| = {float(slopes[k]):.4g}/s in {window_s:g}s window (maxRate={max_rate:g})",
                    "windowStartS": _round(float(t[int(idx[k])]), 6),
                    "windowEndS": _round(float(t[j]), 6),
                }
            )
    return out


def _spike_anomalies(name: str, t: np.ndarray, values: np.ndarray, sigma: float) -> list[dict[str, Any]]:
    """|value - median| > sigma * MAD (median absolute deviation)."""
    finite = np.isfinite(values)
    fvals = values[finite]
    if len(fvals) < 3:
        return []
    median = float(np.median(fvals))
    mad = float(np.median(np.abs(fvals - median)))
    if mad <= 0.0:
        # MAD == 0 means the channel is constant except for the outlier(s):
        # fall back to a relative epsilon so a single spike is still caught
        # (previously the whole check silently returned []).
        mad = max(mad, 1e-9 * max(1.0, abs(median)))
    limit = sigma * mad
    mask = finite & (np.abs(values - median) > limit)
    out: list[dict[str, Any]] = []
    for index in np.flatnonzero(mask):
        value = float(values[index])
        out.append(
            {
                "t": _round(float(t[index]), 6),
                "channel": name,
                "method": "spike",
                "severity": "error",
                "value": _round(value, 6),
                "detail": f"|value - median| = {abs(value - median):.4g} > {sigma:g}×MAD = {limit:.4g}",
                "windowStartS": _round(float(t[index]), 6),
                "windowEndS": _round(float(t[index]), 6),
            }
        )
    return out


def cmd_anomaly_scan(args: dict[str, Any]) -> dict[str, Any]:
    """Deterministic layer-2 anomaly scan over numeric channels (numpy only)."""
    run_path = args.get("runPath") or args.get("runDir")
    run, rows = _load_run(run_path)
    frame = _Frame(rows)
    method = (args.get("method") or "all").lower()
    if method not in ("threshold", "rate", "spike", "all"):
        raise WorkerError(f"unknown method {method!r}; expected one of threshold|rate|spike|all")
    window_s = DEFAULT_WINDOW_S if args.get("windowS") is None else float(args["windowS"])
    if window_s <= 0:
        raise WorkerError("windowS must be > 0")
    thresholds = args.get("thresholds") or {}
    if not isinstance(thresholds, dict):
        raise WorkerError("thresholds must be an object {channel: {min?, max?, maxRate?, spikeSigma?}}")

    numeric = [spec for spec in frame.specs if spec.kind in ("scalar", "vector")]
    if args.get("channels"):
        numeric = [spec for spec in _select_specs(frame, args.get("channels")) if spec.kind in ("scalar", "vector")]
    if not numeric:
        raise WorkerError("no numeric channels to scan")

    anomalies: list[dict[str, Any]] = []
    scanned: list[dict[str, Any]] = []
    for spec in numeric:
        cfg = thresholds.get(spec.name) or {}
        if not isinstance(cfg, dict):
            raise WorkerError(f"thresholds[{spec.name!r}] must be an object {{min?, max?, maxRate?, spikeSigma?}}")
        values = frame.numeric(spec.name)
        t = frame.t
        anomalies.extend(_nonfinite_runs(spec.name, t, values))
        finite_vals = values[np.isfinite(values)]
        if len(finite_vals) == 0:
            scanned.append({"name": spec.name, "methods": ["nonfinite"]})
            continue
        if float(np.ptp(finite_vals)) < 1e-12:
            first = int(np.flatnonzero(np.isfinite(values))[0])
            last = int(np.flatnonzero(np.isfinite(values))[-1])
            anomalies.append(
                {
                    "t": _round(float(t[first]), 6),
                    "channel": spec.name,
                    "method": "constant",
                    "severity": "info",
                    "value": _round(float(finite_vals[0]), 6),
                    "detail": "channel is constant (variance ≈ 0)",
                    "windowStartS": _round(float(t[first]), 6),
                    "windowEndS": _round(float(t[last]), 6),
                }
            )
        methods_used: list[str] = []
        if method in ("threshold", "all") and (cfg.get("min") is not None or cfg.get("max") is not None):
            methods_used.append("threshold")
            anomalies.extend(_threshold_anomalies(spec.name, t, values, cfg))
        if method in ("rate", "all") and cfg.get("maxRate") is not None:
            methods_used.append("rate")
            try:
                max_rate = float(cfg["maxRate"])
            except (TypeError, ValueError) as error:
                raise WorkerError(f"thresholds[{spec.name!r}].maxRate must be a number: {error}") from error
            anomalies.extend(_rate_anomalies(spec.name, t, values, max_rate, window_s))
        if method in ("spike", "all"):
            methods_used.append("spike")
            sigma = DEFAULT_SPIKE_SIGMA if cfg.get("spikeSigma") is None else float(cfg["spikeSigma"])
            anomalies.extend(_spike_anomalies(spec.name, t, values, sigma))
        scanned.append({"name": spec.name, "methods": methods_used})

    anomalies.sort(key=lambda a: (a["t"], a["channel"]))
    by_method: dict[str, int] = {}
    for anomaly in anomalies:
        by_method[anomaly["method"]] = by_method.get(anomaly["method"], 0) + 1
    return {
        "ok": True,
        "runId": run.id,
        "scannedChannels": scanned,
        "anomalies": anomalies,
        "summary": {"total": len(anomalies), "byMethod": by_method},
        "inputArgs": {
            "runPath": run_path,
            "method": method,
            "windowS": window_s,
            "channels": args.get("channels"),
            "thresholdChannels": sorted(thresholds),
        },
    }


# ---------------------------------------------------------------------------
# command: failure-evidence-collect
# ---------------------------------------------------------------------------


def _select_anomaly(anomalies: list[dict[str, Any]], ref: dict[str, Any]) -> Optional[dict[str, Any]]:
    """Pick an anomaly matching channel and/or nearest to t."""
    channel = ref.get("channel")
    candidates = anomalies
    if channel is not None:
        candidates = [a for a in candidates if a.get("channel") == channel]
    if not candidates:
        return None
    if ref.get("t") is None:
        return candidates[0]
    t_ref = float(ref["t"])
    return min(candidates, key=lambda a: abs(a["t"] - t_ref))


def cmd_failure_evidence_collect(args: dict[str, Any]) -> dict[str, Any]:
    """Collect failure evidence around a focused window; optionally create a case."""
    run_path = args.get("runPath") or args.get("runDir")
    run, rows = _load_run(run_path)
    frame = _Frame(rows)

    anomalies = args.get("anomalies")
    if anomalies is None:
        scan_args: dict[str, Any] = {"runPath": run_path}
        for key in ("channels", "method", "thresholds"):
            if key in args:
                scan_args[key] = args[key]
        anomalies = cmd_anomaly_scan(scan_args)["anomalies"]
    if not isinstance(anomalies, list):
        raise WorkerError("'anomalies' must be a list of anomaly records")
    for anomaly in anomalies:
        if not isinstance(anomaly, dict) or not isinstance(anomaly.get("t"), (int, float)):
            raise WorkerError("each anomaly record must be an object with a numeric 't'")
    kinds = args.get("anomalyKinds")
    if kinds:
        kinds = set(kinds)
        anomalies = [a for a in anomalies if a.get("method") in kinds]

    ref = args.get("anomalyRef")
    explicit = args.get("windowS") is not None or args.get("windowE") is not None
    if ref is not None:
        if not isinstance(ref, dict):
            raise WorkerError("anomalyRef must be an object {t?, channel?}")
        chosen = _select_anomaly(anomalies, ref)
        if chosen is None:
            raise WorkerError("no anomaly matches anomalyRef")
        t_ref = float(chosen["t"])
        start, end = t_ref - 1.0, t_ref + 1.0
    elif explicit:
        start = float(args.get("windowS") if args.get("windowS") is not None else frame.t[0])
        end = float(args.get("windowE") if args.get("windowE") is not None else frame.t[-1])
    elif anomalies:
        start = float(min(a["t"] for a in anomalies)) - 1.0
        end = float(max(a["t"] for a in anomalies)) + 1.0
    else:
        start, end = float(frame.t[0]), float(frame.t[-1])

    mask = (frame.t >= start) & (frame.t <= end)
    indices = np.flatnonzero(mask)
    if len(indices) == 0:
        raise WorkerError(f"no telemetry samples in evidence window [{start:g}, {end:g}]")
    window = {
        "startS": _round(float(frame.t[indices[0]]), 6),
        "endS": _round(float(frame.t[indices[-1]]), 6),
    }
    anomalies = [a for a in anomalies if window["startS"] <= a["t"] <= window["endS"]]

    channels_summary: list[dict[str, Any]] = []
    for spec in frame.specs:
        entry: dict[str, Any] = {"name": spec.name, "kind": spec.kind}
        arr = frame.numeric(spec.name)[indices]
        finite = arr[np.isfinite(arr)]
        entry["count"] = int(len(indices))
        entry["missing"] = int(len(indices) - len(finite))
        if spec.kind in ("scalar", "vector") and len(finite):
            entry["min"] = _round(float(finite.min()), 6)
            entry["max"] = _round(float(finite.max()), 6)
            entry["mean"] = _round(float(finite.mean()), 6)
            entry["std"] = _round(float(finite.std()), 6)
        channels_summary.append(entry)

    artifacts: list[str] = []
    for name in _ARTIFACT_NAMES:
        path = run.artifacts.get(name)
        if path:
            abs_path = os.path.abspath(path)
            if os.path.exists(abs_path):
                artifacts.append(abs_path)

    case_ref: Optional[dict[str, Any]] = None
    case_id: Optional[str] = None
    case_path: Optional[str] = None
    if args.get("createCase"):
        store = _store_for(args)
        case = diagnose(run, [frame.rows[int(index)] for index in indices])
        case_path = os.path.abspath(store.save_case(case))
        case_id = case.id
        case_ref = {"caseId": case.id, "casePath": case_path}

    return {
        "ok": True,
        "runId": run.id,
        "window": window,
        "anomalies": anomalies,
        "evidence": {
            "telemetryRows": int(len(indices)),
            "channels": channels_summary,
            "artifacts": artifacts,
            "diagnosticCaseRef": case_ref,
        },
        "caseId": case_id,
        "casePath": case_path,
        "inputArgs": {"runPath": run_path, "anomalyRef": ref, "anomalyKinds": list(kinds) if kinds else None},
    }


# ---------------------------------------------------------------------------
# command: run-compare
# ---------------------------------------------------------------------------


def _align(t_a: np.ndarray, t_b: np.ndarray, max_gap: Optional[float]) -> list[tuple[int, int]]:
    """Nearest-neighbour time alignment of A samples onto B."""
    pairs: list[tuple[int, int]] = []
    idx = np.searchsorted(t_b, t_a, side="left")
    for i, j0 in enumerate(idx):
        candidates: list[int] = []
        if j0 - 1 >= 0:
            candidates.append(int(j0 - 1))
        if j0 < len(t_b):
            candidates.append(int(j0))
        if not candidates:
            continue
        j = min(candidates, key=lambda jj: abs(float(t_b[jj]) - float(t_a[i])))
        if max_gap is None or abs(float(t_b[j]) - float(t_a[i])) <= max_gap:
            pairs.append((int(i), j))
    return pairs


def cmd_run_compare(args: dict[str, Any]) -> dict[str, Any]:
    """Compare two runs' telemetry channel by channel (nearest-neighbour aligned)."""
    run_a_path = args.get("runA")
    run_b_path = args.get("runB")
    if not run_a_path or not run_b_path:
        raise WorkerError("missing required arguments 'runA' and 'runB'")
    run_a, rows_a = _load_run(run_a_path)
    run_b, rows_b = _load_run(run_b_path)
    frame_a = _Frame(rows_a)
    frame_b = _Frame(rows_b)
    by_name_b = {spec.name: spec for spec in frame_b.specs}

    if args.get("channels"):
        selected = _select_specs(frame_a, args.get("channels"))
        numeric = [
            spec
            for spec in selected
            if spec.kind in ("scalar", "vector")
            and spec.name in by_name_b
            and by_name_b[spec.name].kind in ("scalar", "vector")
        ]
    else:
        numeric = [
            spec
            for spec in frame_a.specs
            if spec.kind in ("scalar", "vector")
            and spec.name in by_name_b
            and by_name_b[spec.name].kind in ("scalar", "vector")
        ]
    if not numeric:
        raise WorkerError("no shared numeric channels between the two runs")

    max_gap = float(args["timeWindowS"]) if args.get("timeWindowS") is not None else None
    pairs = _align(frame_a.t, frame_b.t, max_gap)
    if not pairs:
        raise WorkerError("no aligned samples between the two telemetry streams")

    per_channel: dict[str, Any] = {}
    for spec in numeric:
        va = frame_a.numeric(spec.name)
        vb = frame_b.numeric(spec.name)
        aa: list[float] = []
        bb: list[float] = []
        for i, j in pairs:
            x, y = va[i], vb[j]
            if math.isfinite(x) and math.isfinite(y):
                aa.append(float(x))
                bb.append(float(y))
        if not aa:
            per_channel[spec.name] = {"samples": 0, "rmsDelta": None, "maxDelta": None, "p95Delta": None, "correlation": None}
            continue
        a = np.asarray(aa, dtype=float)
        b = np.asarray(bb, dtype=float)
        delta = np.abs(a - b)
        correlation: Optional[float]
        if np.std(a) > 1e-12 and np.std(b) > 1e-12:
            corr = float(np.corrcoef(a, b)[0, 1])
            correlation = corr if math.isfinite(corr) else None
        else:
            correlation = None
        per_channel[spec.name] = {
            "samples": len(aa),
            "rmsDelta": _round(float(np.sqrt(np.mean(delta**2))), 6),
            "maxDelta": _round(float(np.max(delta)), 6),
            "p95Delta": _round(float(np.percentile(delta, 95)), 6),
            "correlation": _round(correlation, 6) if correlation is not None else None,
        }

    first_div: Optional[dict[str, Any]] = None
    for i, j in pairs:
        best: Optional[tuple[str, float, float, float]] = None
        for spec in numeric:
            x, y = frame_a.numeric(spec.name)[i], frame_b.numeric(spec.name)[j]
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            d = abs(x - y)
            if d > DIVERGENCE_EPSILON and (best is None or d > best[3]):
                best = (spec.name, float(x), float(y), d)
        if best is not None:
            first_div = {
                "t": _round(float(frame_a.t[i]), 6),
                "channel": best[0],
                "valueA": _round(best[1], 6),
                "valueB": _round(best[2], 6),
                "delta": _round(best[3], 6),
            }
            break

    worst: Optional[tuple[str, dict[str, Any]]] = None
    worst_score = -1.0
    for name, stats in per_channel.items():
        score = stats.get("maxDelta")
        if score is not None and score > worst_score:
            worst_score = float(score)
            worst = (name, stats)
    return {
        "ok": True,
        "runA": run_a.id,
        "runB": run_b.id,
        "alignedSamples": len(pairs),
        "firstDivergence": first_div,
        "perChannel": per_channel,
        "summary": {
            "alignedSamples": len(pairs),
            "channels": len(per_channel),
            "anyDivergence": first_div is not None,
            "worstChannel": worst[0] if worst is not None else None,
        },
        "inputArgs": {"runA": run_a_path, "runB": run_b_path},
    }


# ---------------------------------------------------------------------------
# command: timeline-export
# ---------------------------------------------------------------------------


def cmd_timeline_export(args: dict[str, Any]) -> dict[str, Any]:
    """Write the standalone timeline viewer (reuses report.timeline_html)."""
    run_path = args.get("runPath") or args.get("runDir")
    out_path = args.get("outPath")
    if not out_path:
        raise WorkerError("missing required argument 'outPath'")
    if os.path.isdir(out_path):
        raise WorkerError(f"outPath must be a file path, got a directory: {out_path}")
    run, rows = _load_run(run_path)
    case = diagnose(run, rows)
    abs_out = os.path.abspath(out_path)
    parent = os.path.dirname(abs_out)
    if parent:
        os.makedirs(parent, exist_ok=True)
    timeline_html(run, rows, case, abs_out)
    return {
        "ok": True,
        "runId": run.id,
        "caseId": case.id,
        "path": abs_out,
        "inputArgs": {"runPath": run_path, "outPath": abs_out},
    }


# ---------------------------------------------------------------------------
# module interface (worker module contract)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "telemetry-channels": cmd_telemetry_channels,
    "telemetry-window": cmd_telemetry_window,
    "anomaly-scan": cmd_anomaly_scan,
    "failure-evidence-collect": cmd_failure_evidence_collect,
    "run-compare": cmd_run_compare,
    "timeline-export": cmd_timeline_export,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "telemetry.channels",
        "kind": "telemetry",
        "provider": "robotic-harness-worker",
        "input": {"runPath": "string"},
        "output": "channel inventory with rate, range and missing counts",
        "risk": "R0-readonly",
        "description": "List scalar/vector/state telemetry channels and their sampling statistics.",
    },
    {
        "id": "telemetry.window",
        "kind": "telemetry",
        "provider": "robotic-harness-worker",
        "input": {"runPath": "string", "startS": "number?", "endS": "number?", "channels": "string[]?"},
        "output": "windowed channel values and statistics",
        "risk": "R0-readonly",
        "description": "Extract a time window of telemetry with per-channel mean/std/min/max.",
    },
    {
        "id": "telemetry.anomaly_scan",
        "kind": "telemetry",
        "provider": "robotic-harness-worker",
        "input": {"runPath": "string", "method": "string?", "windowS": "number?", "thresholds": "object?"},
        "output": "deterministic anomaly list with summary",
        "risk": "R0-readonly",
        "description": "Layer-2 anomaly scan: threshold, rate, MAD-spike, constant and NaN/Inf channel checks.",
    },
    {
        "id": "telemetry.failure_evidence",
        "kind": "telemetry",
        "provider": "robotic-harness-worker",
        "input": {"runPath": "string", "anomalyRef": "object?", "windowS": "number?", "windowE": "number?", "createCase": "boolean?"},
        "output": "focused evidence window + optional diagnostic case",
        "risk": "R1-derive",
        "description": "Collect telemetry/artifacts evidence around anomalies; optionally persist a diagnostic case.",
    },
    {
        "id": "telemetry.run_compare",
        "kind": "telemetry",
        "provider": "robotic-harness-worker",
        "input": {"runA": "string", "runB": "string", "channels": "string[]?", "timeWindowS": "number?"},
        "output": "aligned per-channel deltas and first divergence",
        "risk": "R0-readonly",
        "description": "Nearest-neighbour aligned comparison of two runs' telemetry.",
    },
    {
        "id": "telemetry.timeline_export",
        "kind": "telemetry",
        "provider": "robotic-harness-worker",
        "input": {"runPath": "string", "outPath": "string"},
        "output": "self-contained timeline HTML file",
        "risk": "R1-derive",
        "description": "Export the standalone timeline viewer for a run.",
    },
]
