"""Control analysis and experiment tooling for the Robotic Harness worker.

Pure numpy + standard-library implementation of control-loop commands:

- ``control-trace-analyze``     metrics + anomaly detection on a control trace
- ``trajectory-validate``       safety/continuity validation of a trajectory
- ``planned-actual-compare``    align and compare planned vs actual trajectories
- ``pid-experiment-prepare``    generate step/sweep experiment templates (no HW)
- ``controller-config-compare`` diff two PID controller configurations
- ``system-identification``     first/second-order model fit on step responses
- ``control-report``            assemble the above into a Markdown report

Every command is read-only with respect to hardware: nothing here moves a
joint, and experiment preparation only produces a template (see its note).
No optional dependencies are used (numpy only), so there is nothing to skip.
"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import time as _time
from typing import Any, Optional

import numpy as np

from .core import WorkerError, new_id

# Fixed declaration every generated report must carry (人工确认声明).
REPORT_DISCLAIMER = "本报告结论需人工确认、不自动应用于真机；任何参数调整必须先经工程师在仿真或受控环境中验证。"

# Tunable analysis defaults (documented; callers may override per command).
MAX_JUMP_DEFAULT = 0.5
START_TOLERANCE_DEFAULT = 0.1
COMPARE_THRESHOLD_DEFAULT = 0.02
SETTLING_BAND = 0.02
SATURATION_FRACTION_THRESHOLD = 0.10
OSCILLATION_CROSSING_RATE_HZ = 3.0
NOISE_HIGH_FREQ_RATIO_THRESHOLD = 0.45
WINDUP_MIN_FRACTION = 0.20
WINDUP_PINNED_FRACTION = 0.80
STEP_TEMPLATE_DT = 0.01


# ---------------------------------------------------------------------------
# shared helpers
# ---------------------------------------------------------------------------


def _safe_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if text == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _r(value: Any, digits: int = 6) -> Any:
    """Round a value to `digits` decimals; None-safe and NaN-safe (-> None)."""
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return value
    if not math.isfinite(number):
        return None
    return round(number, digits)


def _load_rows(path: str) -> tuple[list[dict[str, Any]], list[str]]:
    """Load a CSV or JSONL table into row dicts plus the header (col names)."""
    fmt = os.path.splitext(path)[1].lower().lstrip(".")
    if fmt not in ("csv", "tsv", "jsonl", "ndjson", "json"):
        raise WorkerError(f"unsupported data format {fmt!r}; supported: csv, jsonl")
    rows: list[dict[str, Any]] = []
    header: list[str] = []
    if fmt in ("csv", "tsv"):
        delimiter = "\t" if fmt == "tsv" else ","
        with open(path, encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle, delimiter=delimiter)
            try:
                header = next(reader)
            except StopIteration:
                raise WorkerError(f"file has no header row: {path}")
            if not header:
                raise WorkerError(f"file has no header row: {path}")
            for line in reader:
                if len(line) != len(header):
                    continue
                rows.append(dict(zip(header, line)))
    else:
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(record, dict):
                    continue
                for key in record:
                    if key not in header:
                        header.append(key)
                rows.append(record)
    if not rows:
        raise WorkerError(f"no data rows found in {path}")
    return rows, header


def _column(rows: list[dict[str, Any]], name: str) -> np.ndarray:
    """Extract one column as a float ndarray (NaN for missing/unparsable)."""
    values = np.full(len(rows), np.nan, dtype=float)
    for index, row in enumerate(rows):
        value = _safe_float(row.get(name))
        if value is not None:
            values[index] = value
    return values


def _column_present(rows: list[dict[str, Any]], name: str) -> bool:
    return any(name in row for row in rows)


def _resolve_column(rows: list[dict[str, Any]], configured: Optional[str], fallbacks: list[str]) -> Optional[str]:
    """Pick a column name: explicit `configured` first, else first fallback present."""
    if configured:
        if not _column_present(rows, configured):
            raise WorkerError(f"column {configured!r} not found in data")
        return configured
    for name in fallbacks:
        if _column_present(rows, name):
            return name
    return None


def _sort_by_time(t: np.ndarray, arrays: dict[str, np.ndarray]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Sort arrays by ascending time, dropping rows with non-finite time."""
    finite = np.isfinite(t)
    t = t[finite]
    arrays = {key: value[finite] for key, value in arrays.items()}
    order = np.argsort(t, kind="mergesort")
    return t[order], {key: value[order] for key, value in arrays.items()}


def _crossing_time(t: np.ndarray, y: np.ndarray, level: float, t_start: float) -> Optional[float]:
    """First time >= t_start at which y crosses `level` (linear interp). None if never."""
    if len(t) < 2:
        return None
    i0 = max(int(np.searchsorted(t, t_start, side="left")), 0)
    for i in range(i0, len(t) - 1):
        a, b = float(y[i]), float(y[i + 1])
        if a == b:
            if a == level:
                return float(t[i])
            continue
        if (a - level) * (b - level) <= 0:
            frac = (level - a) / (b - a)
            return float(t[i]) + frac * (float(t[i + 1]) - float(t[i]))
    return None


def _settling_time(t: np.ndarray, y: np.ndarray, final: float, band: float, t_start: float) -> Optional[float]:
    """First time >= t_start after which |y - final| <= band forever. None if unsettled."""
    if len(t) < 2:
        return None
    i0 = max(int(np.searchsorted(t, t_start, side="left")), 0)
    last_violation = -1
    for i in range(i0, len(t)):
        if abs(float(y[i]) - final) > band:
            last_violation = i
    if last_violation < 0:
        return float(t[i0]) if i0 < len(t) else None
    if last_violation >= len(t) - 1:
        return None
    return float(t[last_violation + 1])


def _count_sign_crossings(error: np.ndarray, deadband: float) -> int:
    """Count sign flips of `error` between excursions beyond +/- deadband."""
    count = 0
    prev: Optional[int] = None
    for value in error:
        if value > deadband:
            sign = 1
        elif value < -deadband:
            sign = -1
        else:
            sign = 0
        if sign != 0:
            if prev is not None and prev != 0 and sign != prev:
                count += 1
            prev = sign
    return count


def _longest_same_sign_run(signs: np.ndarray) -> tuple[int, int, int]:
    """Longest run of equal non-zero signs -> (start, end_exclusive, length)."""
    n = len(signs)
    best_start = best_end = best_len = 0
    i = 0
    while i < n:
        if signs[i] == 0:
            i += 1
            continue
        j = i
        while j < n and signs[j] == signs[i]:
            j += 1
        if j - i > best_len:
            best_start, best_end, best_len = i, j, j - i
        i = j
    return best_start, best_end, best_len


def _fit_quality(y_actual: np.ndarray, y_pred: np.ndarray) -> dict[str, Any]:
    resid = y_actual - y_pred
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y_actual - np.mean(y_actual)) ** 2))
    explained = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else None
    return {
        "explainedVariance": _r(explained),
        "residualRms": _r(float(np.sqrt(np.mean(resid ** 2)))),
    }


def _noise_amplification_issue(t: np.ndarray, y: np.ndarray) -> Optional[dict[str, Any]]:
    """High-frequency energy share of the differenced response (noise amplification)."""
    if len(y) < 32:
        return None
    d = np.diff(y)
    d = d - np.mean(d)
    spectrum = np.abs(np.fft.rfft(d)) ** 2
    dt_median = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    freqs = np.fft.rfftfreq(len(d), d=dt_median)
    nyquist = float(freqs[-1]) if len(freqs) else 0.0
    total = float(np.sum(spectrum))
    if total <= 1e-12 or nyquist <= 0:
        return None
    high = freqs > 0.75 * nyquist  # top quarter of the band
    ratio = float(np.sum(spectrum[high])) / total
    if ratio > NOISE_HIGH_FREQ_RATIO_THRESHOLD:
        return {
            "severity": "warning",
            "code": "noise.amplified",
            "message": (
                f"{ratio * 100:.1f}% of the differenced-signal energy sits in the top quarter of the "
                f"spectrum (up to {nyquist:.3f} Hz) — possible noise amplification"
            ),
            "evidence": {"highFrequencyRatio": _r(ratio), "nyquistHz": _r(nyquist)},
        }
    return None


def _position_columns(header: list[str]) -> list[str]:
    q_cols = sorted((c for c in header if re.fullmatch(r"q\d+", c)), key=lambda c: int(c[1:]))
    if q_cols:
        return q_cols
    for name in ("position", "pos", "q"):
        if name in header:
            return [name]
    return []


def _velocity_columns(header: list[str]) -> list[str]:
    dq_cols = sorted((c for c in header if re.fullmatch(r"dq\d+", c)), key=lambda c: int(c[2:]))
    if dq_cols:
        return dq_cols
    for name in ("velocity", "vel", "dq"):
        if name in header:
            return [name]
    return []


def _joint_limits(limits: dict[str, Any], joint: str) -> tuple[Optional[float], Optional[float]]:
    spec = limits.get(joint)
    if spec is None:
        return None, None
    if isinstance(spec, (list, tuple)) and len(spec) == 2:
        return float(spec[0]), float(spec[1])
    if isinstance(spec, dict):
        lo = spec.get("min", spec.get("lower"))
        hi = spec.get("max", spec.get("upper"))
        return (float(lo) if lo is not None else None, float(hi) if hi is not None else None)
    return None, None


def _nearest_indices(query: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """For each query time, the index into `ref` with the closest time."""
    idx = np.searchsorted(ref, query)
    idx = np.clip(idx, 0, len(ref) - 1)
    left = np.clip(idx - 1, 0, len(ref) - 1)
    closer = np.abs(ref[left] - query) < np.abs(ref[idx] - query)
    return np.where(closer, left, idx)


def _estimate_time_offset(t_planned: np.ndarray, q_planned: np.ndarray, t_actual: np.ndarray, q_actual: np.ndarray) -> float:
    """Time shift aligning actual to planned via cross-correlation (seconds).

    Returns a positive value when the actual signal lags the planned one.
    """
    op = np.argsort(t_planned, kind="mergesort")
    oa = np.argsort(t_actual, kind="mergesort")
    tp, qp = t_planned[op], q_planned[op]
    ta, qa = t_actual[oa], q_actual[oa]
    keep_p = np.r_[True, np.diff(tp) > 0]
    tp, qp = tp[keep_p], qp[keep_p]
    keep_a = np.r_[True, np.diff(ta) > 0]
    ta, qa = ta[keep_a], qa[keep_a]
    if len(tp) < 4 or len(ta) < 4:
        return 0.0
    qa_interp = np.interp(tp, ta, qa)
    n = len(tp)
    a = qp - np.mean(qp)
    b = qa_interp - np.mean(qa_interp)
    denom = math.sqrt(float(np.sum(a ** 2)) * float(np.sum(b ** 2)))
    if denom <= 1e-12:
        return 0.0
    correlation = np.correlate(a, b, mode="full")
    lag = int(np.argmax(correlation)) - (n - 1)
    dt = float(np.median(np.diff(tp))) if len(tp) > 1 else 0.0
    return -float(lag) * dt


# ---------------------------------------------------------------------------
# 1. control-trace-analyze
# ---------------------------------------------------------------------------


def analyze_trace(args: dict[str, Any]) -> dict[str, Any]:
    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"data file not found: {path}")
    rows, _header = _load_rows(path)
    time_col = args.get("timeColumn", "t")
    if not _column_present(rows, time_col):
        raise WorkerError(f"time column {time_col!r} not found in data")
    sp_col = _resolve_column(rows, args.get("setpointColumn"), ["setpoint", "reference", "ref", "target"])
    resp_col = _resolve_column(rows, args.get("measurementColumn"), ["measurement", "output", "y", "position"])
    eff_col = _resolve_column(rows, args.get("effortColumn"), ["effort", "u", "command", "torque"])

    t = _column(rows, time_col)
    cols: dict[str, np.ndarray] = {}
    for name in (sp_col, resp_col, eff_col):
        if name:
            cols[name] = _column(rows, name)
    t, cols = _sort_by_time(t, cols)
    if len(t) < 2:
        raise WorkerError("not enough valid time samples for trace analysis")

    step_start = args.get("stepStart")
    step_end = args.get("stepEnd")
    mask = np.ones(len(t), dtype=bool)
    if step_start is not None:
        mask &= t >= float(step_start)
    if step_end is not None:
        mask &= t <= float(step_end)
    tw = t[mask]
    w = {name: arr[mask] for name, arr in cols.items()}
    if len(tw) < 2:
        raise WorkerError("step window is empty or has fewer than 2 samples")

    effort_min = args.get("effortMin")
    effort_max = args.get("effortMax")

    issues: list[dict[str, Any]] = []
    for name, arr in cols.items():
        if np.count_nonzero(np.isfinite(arr)) == 0:
            continue
        n_bad = int(np.count_nonzero(~np.isfinite(arr)))
        if n_bad:
            issues.append({
                "severity": "info",
                "code": "data.non_finite",
                "message": f"column {name!r} has {n_bad} non-finite samples (NaN/Inf)",
                "evidence": {"column": name, "count": n_bad},
            })

    metrics: dict[str, Any] = {
        "riseTimeS": None,
        "settlingTimeS": None,
        "overshootPercent": None,
        "steadyStateError": None,
        "trackingErrorRms": None,
        "controlEffortRms": None,
        "peakError": None,
    }

    resp_full = cols.get(resp_col) if resp_col else None
    resp = w.get(resp_col) if resp_col else None
    if resp is not None:
        ok = np.isfinite(tw) & np.isfinite(resp)
        t_ok, y_ok = tw[ok], resp[ok]
        baseline = final = None
        if len(y_ok):
            if step_start is not None:
                pre = resp_full[t < float(step_start)]
                baseline = float(np.nanmedian(pre)) if len(pre) else float(y_ok[0])
            else:
                baseline = float(y_ok[0])
            final = float(np.nanmedian(y_ok[-max(1, len(y_ok) // 5):]))
        step_origin = float(step_start) if step_start is not None else (float(t_ok[0]) if len(t_ok) else None)
        if baseline is not None and final is not None and abs(final - baseline) > 1e-12 and len(t_ok) >= 3:
            low = baseline + 0.10 * (final - baseline)
            high = baseline + 0.90 * (final - baseline)
            t10 = _crossing_time(t_ok, y_ok, low, step_origin)
            t90 = _crossing_time(t_ok, y_ok, high, step_origin)
            if t10 is not None and t90 is not None:
                metrics["riseTimeS"] = _r(max(0.0, t90 - t10))
            band = SETTLING_BAND * abs(final) if abs(final) > 1e-12 else SETTLING_BAND * abs(final - baseline)
            settle = _settling_time(t_ok, y_ok, final, band, step_origin)
            if settle is not None and step_origin is not None:
                metrics["settlingTimeS"] = _r(max(0.0, settle - step_origin))
            peak = float(np.nanmax(y_ok))
            metrics["overshootPercent"] = _r(max(0.0, peak - final) / abs(final - baseline) * 100.0)
        if len(y_ok) >= 32:
            noise_issue = _noise_amplification_issue(t_ok, y_ok)
            if noise_issue:
                issues.append(noise_issue)

    sp = w.get(sp_col) if sp_col else None
    if sp is not None and resp is not None:
        ok = np.isfinite(sp) & np.isfinite(resp)
        err = sp[ok] - resp[ok]
        if len(err) >= 3:
            tail = err[-max(1, int(0.2 * len(err))):]
            metrics["steadyStateError"] = _r(float(np.mean(tail)))
            metrics["trackingErrorRms"] = _r(float(np.sqrt(np.mean(err ** 2))))
            metrics["peakError"] = _r(float(np.max(np.abs(err))))
        if len(err) >= 8:
            deadband = 0.10 * float(np.max(np.abs(err))) if len(err) else 0.0
            if deadband <= 1e-12:
                deadband = 1e-9
            crossings = _count_sign_crossings(err, deadband)
            ok_t = tw[ok]
            dur = float(ok_t[-1] - ok_t[0]) if len(ok_t) > 1 else 0.0
            rate = crossings / dur if dur > 1e-9 else 0.0
            if rate > OSCILLATION_CROSSING_RATE_HZ:
                issues.append({
                    "severity": "warning",
                    "code": "oscillation.high",
                    "message": f"error zero-crossing rate {rate:.2f} Hz exceeds {OSCILLATION_CROSSING_RATE_HZ} Hz — possible oscillation",
                    "evidence": {
                        "zeroCrossings": int(crossings),
                        "zeroCrossingsPerSec": _r(rate),
                        "deadband": _r(deadband),
                    },
                })

    eff = w.get(eff_col) if eff_col else None
    if eff is not None:
        eff_fin = eff[np.isfinite(eff)]
        if len(eff_fin):
            metrics["controlEffortRms"] = _r(float(np.sqrt(np.mean(eff_fin ** 2))))
            # Saturation needs known limits; data-derived extremes are just the
            # resting range (a settled effort converges to its own min), which
            # would produce systematic false positives — so skip without limits.
            if effort_min is not None or effort_max is not None:
                lo = float(effort_min) if effort_min is not None else float(np.min(eff_fin))
                hi = float(effort_max) if effort_max is not None else float(np.max(eff_fin))
                span = hi - lo
                margin = 0.01 * span if span > 1e-12 else 1e-9
                pinned = (eff_fin >= hi - margin) | (eff_fin <= lo + margin)
                fraction = float(np.mean(pinned))
                if fraction > SATURATION_FRACTION_THRESHOLD:
                    issues.append({
                        "severity": "warning",
                        "code": "saturation.persistent",
                        "message": f"effort pinned near its limits for {fraction * 100:.1f}% of samples",
                        "evidence": {
                            "fraction": _r(fraction),
                            "pinnedSamples": int(np.count_nonzero(pinned)),
                            "effortMin": _r(lo),
                            "effortMax": _r(hi),
                            "limitsInferred": effort_min is None or effort_max is None,
                        },
                    })
        # integral-windup signature: error stuck at one sign while effort sits at a limit
        if sp is not None and resp is not None:
            ok = np.isfinite(sp) & np.isfinite(resp) & np.isfinite(eff)
            if np.count_nonzero(ok) >= 10:
                err = sp[ok] - resp[ok]
                eff_w = eff[ok]
                tw_ok = tw[ok]
                lo = float(effort_min) if effort_min is not None else float(np.min(eff_w))
                hi = float(effort_max) if effort_max is not None else float(np.max(eff_w))
                span = hi - lo
                margin = 0.01 * span if span > 1e-12 else 1e-9
                signs = np.sign(err)
                start, end, length = _longest_same_sign_run(signs)
                if length >= max(5, int(WINDUP_MIN_FRACTION * len(err))) and end > start:
                    duration_run = float(tw_ok[end - 1] - tw_ok[start])
                    if duration_run >= 1.0:
                        pinned_run = (eff_w[start:end] >= hi - margin) | (eff_w[start:end] <= lo + margin)
                        pinned_frac = float(np.mean(pinned_run))
                        if pinned_frac >= WINDUP_PINNED_FRACTION:
                            issues.append({
                                "severity": "warning",
                                "code": "integral.windup_signature",
                                "message": (
                                    f"error kept a single sign for {duration_run:.2f}s while effort stayed pinned "
                                    f"at a limit ({pinned_frac * 100:.0f}%) — possible integral windup"
                                ),
                                "evidence": {
                                    "errorSignDurationS": _r(duration_run),
                                    "effortPinnedFraction": _r(pinned_frac),
                                    "effortMin": _r(lo),
                                    "effortMax": _r(hi),
                                },
                            })

    return {"ok": True, "rows": len(rows), "metrics": metrics, "issues": issues}


def cmd_control_trace_analyze(args: dict[str, Any]) -> dict[str, Any]:
    result = analyze_trace(args)
    result["inputArgs"] = {
        "path": args.get("path"),
        "timeColumn": args.get("timeColumn", "t"),
        "stepStart": args.get("stepStart"),
    }
    return result


# ---------------------------------------------------------------------------
# 2. trajectory-validate
# ---------------------------------------------------------------------------


def validate_trajectory(args: dict[str, Any]) -> dict[str, Any]:
    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"data file not found: {path}")
    rows, header = _load_rows(path)
    time_col = args.get("timeColumn", "t")
    if not _column_present(rows, time_col):
        raise WorkerError(f"time column {time_col!r} not found in data")
    limits = args.get("limits")
    limits = limits if isinstance(limits, dict) else {}
    max_jump = float(args.get("maxJump", MAX_JUMP_DEFAULT))
    velocity_limit = args.get("velocityLimit")
    start_state = args.get("startState")
    start_tol = float(args.get("startTolerance", START_TOLERANCE_DEFAULT))

    t = _column(rows, time_col)
    pos_cols = _position_columns(header)
    vel_cols = _velocity_columns(header)
    issues: list[dict[str, Any]] = []

    # NaN/Inf on every numeric column (time handled separately below)
    for col in header:
        if col == time_col:
            continue
        arr = _column(rows, col)
        if np.count_nonzero(np.isfinite(arr)) == 0:
            continue  # non-numeric column
        n_bad = int(np.count_nonzero(~np.isfinite(arr)))
        if n_bad:
            issues.append({
                "severity": "error",
                "code": "data.non_finite",
                "message": f"column {col!r} has {n_bad} non-finite values (NaN/Inf)",
                "evidence": {"column": col, "count": n_bad},
            })

    finite_t = np.isfinite(t)
    tf = t[finite_t]
    if len(tf) < 2:
        raise WorkerError("need at least 2 valid timestamps for trajectory validation")
    dt = np.diff(tf)
    decreasing = int(np.count_nonzero(dt < 0))
    duplicates = int(np.count_nonzero(dt == 0))
    if decreasing:
        issues.append({
            "severity": "error",
            "code": "time.decreasing",
            "message": f"{decreasing} timestamp pairs go backwards (non-monotonic time)",
            "evidence": {"pairs": decreasing},
        })
    if duplicates:
        issues.append({
            "severity": "warning",
            "code": "time.duplicate",
            "message": f"{duplicates} duplicate timestamps",
            "evidence": {"pairs": duplicates},
        })
    positive = dt[dt > 0]
    median_dt = float(np.median(positive)) if len(positive) else 0.0
    jitter = 0.0
    if len(positive) >= 2 and median_dt > 0:
        jitter = float(np.max(np.abs(positive - median_dt)))
        if jitter > 3 * median_dt:
            issues.append({
                "severity": "warning",
                "code": "time.jitter",
                "message": f"max interval jitter {jitter:.6f}s exceeds 3x the median interval {median_dt:.6f}s",
                "evidence": {"jitterS": _r(jitter), "medianDtS": _r(median_dt)},
            })
    stats = {
        "durationS": _r(float(tf[-1] - tf[0])),
        "meanDtS": _r(float(np.mean(positive))) if len(positive) else None,
        "maxDtS": _r(float(np.max(positive))) if len(positive) else None,
        "jitterS": _r(jitter),
    }

    for col in pos_cols:
        q = _column(rows, col)
        lo, hi = _joint_limits(limits, col)
        if lo is not None or hi is not None:
            low_viol = q < lo if lo is not None else np.zeros(len(q), dtype=bool)
            high_viol = q > hi if hi is not None else np.zeros(len(q), dtype=bool)
            violations = int(np.count_nonzero(low_viol | high_viol))
            if violations:
                issues.append({
                    "severity": "error",
                    "code": "joint.limit_exceeded",
                    "message": f"joint {col!r} exceeds its limits {lo}..{hi} at {violations} samples",
                    "evidence": {"joint": col, "min": _r(lo), "max": _r(hi), "violations": violations},
                })
        jumps = np.abs(np.diff(q))
        jumps = jumps[np.isfinite(jumps)]
        n_jumps = int(np.count_nonzero(jumps > max_jump)) if len(jumps) else 0
        if n_jumps:
            issues.append({
                "severity": "warning",
                "code": "joint.jump",
                "message": f"joint {col!r} jumps more than {max_jump} rad between adjacent samples {n_jumps} times",
                "evidence": {"joint": col, "maxJump": _r(max_jump), "jumps": n_jumps},
            })
        if velocity_limit is not None:
            vlim = float(velocity_limit)
            if len(dt) == len(q) - 1:
                v = np.abs(np.diff(q) / np.where(dt > 0, dt, np.nan))
                v = v[np.isfinite(v)]
                if len(v) and float(np.max(v)) > vlim:
                    issues.append({
                        "severity": "warning",
                        "code": "velocity.exceeded",
                        "message": f"joint {col!r} discrete velocity reaches {float(np.max(v)):.4f} rad/s, above limit {vlim}",
                        "evidence": {"joint": col, "velocityLimit": _r(vlim), "maxVelocity": _r(float(np.max(v)))},
                    })
    if velocity_limit is not None:
        vlim = float(velocity_limit)
        for col in vel_cols:
            v = np.abs(_column(rows, col))
            v = v[np.isfinite(v)]
            if len(v) and float(np.max(v)) > vlim:
                issues.append({
                    "severity": "warning",
                    "code": "velocity.exceeded",
                    "message": f"velocity column {col!r} reaches {float(np.max(v)):.4f} rad/s, above limit {vlim}",
                    "evidence": {"joint": col, "velocityLimit": _r(vlim), "maxVelocity": _r(float(np.max(v)))},
                })

    if start_state is not None:
        def _check_start(name: str, expected: float) -> None:
            q = _column(rows, name)
            if len(q) == 0 or not math.isfinite(float(q[0])):
                return
            delta = abs(float(q[0]) - float(expected))
            if delta > start_tol:
                issues.append({
                    "severity": "warning",
                    "code": "start.state_mismatch",
                    "message": f"first sample of {name!r} differs from startState by {delta:.4f} rad",
                    "evidence": {"joint": name, "startState": _r(expected), "firstValue": _r(float(q[0])), "tolerance": _r(start_tol)},
                })

        if isinstance(start_state, dict):
            for col in pos_cols:
                if col in start_state:
                    _check_start(col, float(start_state[col]))
        elif isinstance(start_state, (list, tuple)):
            for index, col in enumerate(pos_cols):
                if index < len(start_state):
                    _check_start(col, float(start_state[index]))

    return {
        "ok": not any(issue["severity"] == "error" for issue in issues),
        "rows": len(rows),
        "issues": issues,
        "stats": stats,
    }


def cmd_trajectory_validate(args: dict[str, Any]) -> dict[str, Any]:
    result = validate_trajectory(args)
    result["inputArgs"] = {"path": args.get("path"), "timeColumn": args.get("timeColumn", "t")}
    return result


# ---------------------------------------------------------------------------
# 3. planned-actual-compare
# ---------------------------------------------------------------------------


def compare_planned_actual(args: dict[str, Any]) -> dict[str, Any]:
    planned_path = args.get("plannedPath") or args.get("path")
    actual_path = args.get("actualPath")
    if not planned_path or not actual_path:
        raise WorkerError("missing required arguments 'plannedPath' and 'actualPath'")
    for path in (planned_path, actual_path):
        if not os.path.exists(path):
            raise WorkerError(f"data file not found: {path}")
    t_p_col = args.get("plannedTimeColumn", "t")
    t_a_col = args.get("actualTimeColumn", "t")
    threshold = float(args.get("threshold", COMPARE_THRESHOLD_DEFAULT))

    rows_p, header_p = _load_rows(planned_path)
    rows_a, header_a = _load_rows(actual_path)
    pcols = _position_columns(header_p) or (["position"] if "position" in header_p else [])
    acols = _position_columns(header_a) or (["position"] if "position" in header_a else [])
    if not pcols or not acols:
        raise WorkerError("no position columns (q0, q1... or 'position') found in planned/actual data")
    common = [c for c in pcols if c in acols]
    if not common:
        common = pcols[: len(acols)]
        if not common or len(common) != len(acols):
            raise WorkerError("no matching joint columns between planned and actual trajectories")

    t_p = _column(rows_p, t_p_col)
    t_a = _column(rows_a, t_a_col)
    valid_p = np.isfinite(t_p)
    valid_a = np.isfinite(t_a)
    q_p_all: dict[str, np.ndarray] = {}
    q_a_all: dict[str, np.ndarray] = {}
    for name in common:
        q_p = _column(rows_p, name)
        q_a = _column(rows_a, name)
        valid_p &= np.isfinite(q_p)
        valid_a &= np.isfinite(q_a)
        q_p_all[name] = q_p
        q_a_all[name] = q_a
    tp = t_p[valid_p]
    ta = t_a[valid_a]
    if len(tp) < 2 or len(ta) < 2:
        raise WorkerError("need at least 2 valid samples in both planned and actual trajectories")

    idx = _nearest_indices(tp, ta)
    per_joint: dict[str, Any] = {}
    max_abs = np.zeros(len(tp))
    for name in common:
        e = q_p_all[name][valid_p] - q_a_all[name][valid_a][idx]
        abs_e = np.abs(e)
        per_joint[name] = {
            "rms": _r(float(np.sqrt(np.mean(e ** 2)))),
            "max": _r(float(np.max(abs_e))),
            "p50": _r(float(np.percentile(abs_e, 50))),
            "p90": _r(float(np.percentile(abs_e, 90))),
            "p99": _r(float(np.percentile(abs_e, 99))),
        }
        max_abs = np.maximum(max_abs, abs_e)

    time_offset = _estimate_time_offset(
        tp, q_p_all[common[0]][valid_p], ta, q_a_all[common[0]][valid_a]
    )

    order = np.argsort(tp, kind="mergesort")
    tp_sorted = tp[order]
    max_abs_sorted = max_abs[order]
    crossing = np.where(max_abs_sorted > threshold)[0]
    first_divergence = float(tp_sorted[crossing[0]]) if len(crossing) else None

    summary = {
        "alignedSamples": int(len(tp)),
        "timeOffsetS": _r(time_offset),
        "firstDivergenceS": _r(first_divergence),
        "maxRms": _r(max(per_joint[name]["rms"] for name in common)),
        "maxError": _r(max(per_joint[name]["max"] for name in common)),
        "threshold": _r(threshold),
    }
    return {
        "ok": True,
        "alignedSamples": int(len(tp)),
        "timeOffsetS": _r(time_offset),
        "firstDivergenceS": _r(first_divergence),
        "perJoint": per_joint,
        "summary": summary,
    }


def cmd_planned_actual_compare(args: dict[str, Any]) -> dict[str, Any]:
    result = compare_planned_actual(args)
    result["inputArgs"] = {"plannedPath": args.get("plannedPath"), "actualPath": args.get("actualPath")}
    return result


# ---------------------------------------------------------------------------
# 4. pid-experiment-prepare
# ---------------------------------------------------------------------------


def prepare_pid_experiment(args: dict[str, Any]) -> dict[str, Any]:
    controller_id = args.get("controllerId")
    if not controller_id:
        raise WorkerError("missing required argument 'controllerId'")
    joints = args.get("joints")
    if not isinstance(joints, list) or not joints or not all(isinstance(j, str) and j for j in joints):
        raise WorkerError("'joints' must be a non-empty list of joint names")
    amplitude = float(args.get("amplitude", 0.1))
    step_time = float(args.get("stepTimeS", 2.0))
    duration = float(args.get("durationS", 10.0))
    if amplitude <= 0:
        raise WorkerError("amplitude must be positive")
    if step_time <= 0 or duration <= 0:
        raise WorkerError("stepTimeS and durationS must be positive")
    sweep = args.get("sweep")

    n = max(2, int(round(duration / STEP_TEMPLATE_DT)) + 1)
    ts = np.round(np.linspace(0.0, duration, n), 4)
    if sweep:
        if not isinstance(sweep, dict):
            raise WorkerError("sweep must be an object with freqMinHz/freqMaxHz")
        fmin = float(sweep.get("freqMinHz", 0.1))
        fmax = float(sweep.get("freqMaxHz", 2.0))
        if fmin <= 0 or fmax <= fmin:
            raise WorkerError("sweep requires 0 < freqMinHz < freqMaxHz")
        phase = 2.0 * math.pi * (fmin * ts + (fmax - fmin) * ts ** 2 / (2.0 * duration))
        values = amplitude * np.sin(phase)
        kind = "sweep"
    else:
        period = 2.0 * step_time
        values = amplitude * ((ts % period) < step_time).astype(float)
        kind = "step"

    waypoints = [{"t": float(ti), "value": round(float(vi), 6)} for ti, vi in zip(ts, values)]
    jumps = np.abs(np.diff(values))
    max_jump = float(jumps.max()) if len(jumps) else float(amplitude)
    experiment = {
        "id": new_id("exp"),
        "kind": kind,
        "joints": [str(j) for j in joints],
        "amplitude": round(amplitude, 6),
        "durationS": round(duration, 4),
        "waypoints": waypoints,
        "safety": {"maxJump": round(max_jump, 6), "limitMargin": 0.2},
    }
    return {
        "ok": True,
        "experiment": experiment,
        "note": "模板不执行任何硬件操作；同一波形施加于所列全部关节，safety.limitMargin=0.2 表示指令幅度应保持在关节行程的 80% 以内。",
    }


def cmd_pid_experiment_prepare(args: dict[str, Any]) -> dict[str, Any]:
    result = prepare_pid_experiment(args)
    result["inputArgs"] = {"controllerId": args.get("controllerId"), "joints": args.get("joints")}
    return result


# ---------------------------------------------------------------------------
# 5. controller-config-compare
# ---------------------------------------------------------------------------


def _extract_gains(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    gains: dict[str, dict[str, Any]] = {}
    for source in (config.get("joints"), config.get("gains")):
        if not isinstance(source, dict):
            continue
        for joint, gainspec in source.items():
            if not isinstance(gainspec, dict):
                continue
            entry = gains.setdefault(str(joint), {})
            for param in ("kp", "kv", "ki", "clamp"):
                if param in gainspec:
                    entry[param] = gainspec[param]
    return gains


def _impact_text(param: str) -> str:
    return {
        "kp": "kp 越大响应越快但更易振荡/抖振；越小越慢越稳",
        "kv": "kv 越大阻尼越强、响应更慢更稳；越小越易超调",
        "ki": "ki 消除稳态误差但引入积分饱和(windup)风险；越大越易超调与饱和",
        "clamp": "输出限幅影响饱和行为与积分抗饱和策略",
    }.get(param, "参数差异影响控制行为，需人工评估")


def _values_equal(a: Any, b: Any) -> bool:
    if a is None or b is None:
        return a is None and b is None
    try:
        fa, fb = float(a), float(b)
    except (TypeError, ValueError):
        return a == b
    return abs(fa - fb) < 1e-9


def compare_controller_configs(args: dict[str, Any]) -> dict[str, Any]:
    config_a = args.get("configA")
    config_b = args.get("configB")
    if not isinstance(config_a, dict) or not isinstance(config_b, dict):
        raise WorkerError("configA and configB must be JSON objects")
    gains_a = _extract_gains(config_a)
    gains_b = _extract_gains(config_b)

    differences: list[dict[str, Any]] = []
    for joint in sorted(set(gains_a) | set(gains_b)):
        pa, pb = gains_a.get(joint, {}), gains_b.get(joint, {})
        for param in sorted(set(pa) | set(pb)):
            if _values_equal(pa.get(param), pb.get(param)):
                continue
            differences.append({
                "joint": joint,
                "param": param,
                "valueA": pa.get(param),
                "valueB": pb.get(param),
                "impact": _impact_text(param),
            })

    issues: list[dict[str, Any]] = []
    # sanity checks run per config separately so values never mask each other
    for _label, gains in (("configA", gains_a), ("configB", gains_b)):
        for joint in sorted(gains):
            for param in ("kp", "kv", "ki"):
                value = gains[joint].get(param)
                if value is None:
                    continue
                try:
                    number = float(value)
                except (TypeError, ValueError):
                    issues.append({
                        "severity": "warning",
                        "code": "param.non_numeric",
                        "message": f"{joint}.{param} is not numeric",
                        "evidence": {"joint": joint, "param": param, "value": value},
                    })
                    continue
                if number < 0:
                    issues.append({
                        "severity": "error",
                        "code": "param.negative",
                        "message": f"{joint}.{param} is negative ({number})",
                        "evidence": {"joint": joint, "param": param, "value": _r(number)},
                    })
                if param == "kp" and number == 0:
                    issues.append({
                        "severity": "warning",
                        "code": "param.zero_kp",
                        "message": f"{joint}.kp is zero (no proportional action)",
                        "evidence": {"joint": joint},
                    })
                if abs(number) > 1e7:
                    issues.append({
                        "severity": "warning",
                        "code": "param.out_of_range",
                        "message": f"{joint}.{param} magnitude {number} is unusually large",
                        "evidence": {"joint": joint, "param": param, "value": _r(number)},
                    })

    return {
        "ok": not any(issue["severity"] == "error" for issue in issues),
        "configA": {"name": config_a.get("name")},
        "configB": {"name": config_b.get("name")},
        "differences": differences,
        "issues": issues,
        "summary": {
            "jointsCompared": len(set(gains_a) | set(gains_b)),
            "differenceCount": len(differences),
        },
    }


def cmd_controller_config_compare(args: dict[str, Any]) -> dict[str, Any]:
    result = compare_controller_configs(args)
    result["inputArgs"] = {
        "configA": (args.get("configA") or {}).get("name"),
        "configB": (args.get("configB") or {}).get("name"),
    }
    return result


# ---------------------------------------------------------------------------
# 6. system-identification
# ---------------------------------------------------------------------------


def identify_system(args: dict[str, Any]) -> dict[str, Any]:
    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"data file not found: {path}")
    rows, _header = _load_rows(path)
    time_col = args.get("timeColumn", "t")
    if not _column_present(rows, time_col):
        raise WorkerError(f"time column {time_col!r} not found in data")
    m_col = _resolve_column(rows, args.get("measurementColumn"), ["measurement", "output", "y", "position"])
    if m_col is None:
        raise WorkerError("no measurement column found (looked for measurement/output/y/position)")
    order = (args.get("order") or "auto").lower()
    if order not in ("auto", "first", "second"):
        raise WorkerError(f"order must be 'auto', 'first' or 'second', got {order!r}")

    t = _column(rows, time_col)
    y = _column(rows, m_col)
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    if len(t) < 6:
        raise WorkerError("insufficient data for system identification (need at least 6 samples)")
    sort_order = np.argsort(t, kind="mergesort")
    t, y = t[sort_order], y[sort_order]

    # locate the step: explicit stepStart, else setpoint change, else response onset
    step_start = args.get("stepStart")
    if step_start is None:
        sp = _resolve_column(rows, args.get("setpointColumn"), ["setpoint", "reference"])
        if sp is not None:
            sp_arr = _column(rows, sp)[ok][sort_order]
            if len(sp_arr) > 1:
                delta = sp_arr - sp_arr[0]
                span = float(max(float(np.max(np.abs(delta))), 1e-12))
                idx = np.where(np.abs(delta) > 0.1 * span)[0]
                if len(idx):
                    step_start = float(t[idx[0]])
        if step_start is None:
            span_y = float(abs(y[-1] - y[0]))
            if span_y < 1e-12:
                raise WorkerError("response is constant; no step to identify")
            idx = np.where(np.abs(y - y[0]) > 0.1 * span_y)[0]
            if len(idx):
                candidate = float(t[idx[0]])
                span_t = float(t[-1] - t[0])
                step_start = candidate if candidate > float(t[0]) + 0.05 * span_t else float(t[0])
            else:
                step_start = float(t[0])
    step_start = float(step_start)
    i0 = int(np.searchsorted(t, step_start, side="left"))
    if i0 >= len(t) - 3:
        raise WorkerError("step occurs too late in the trace; not enough post-step samples")
    tw, yw = t[i0:], y[i0:]
    y0 = float(np.median(y[:i0])) if i0 >= 2 else float(y[0])
    final = float(np.median(yw[-max(1, len(yw) // 5):]))
    gain = final - y0
    if abs(gain) < 1e-12:
        raise WorkerError("no measurable step response (final value equals baseline)")
    yn = (yw - y0) / gain

    # pure delay: first sample where the normalized response clearly leaves zero
    delay = 0.0
    idx_resp = np.where(np.abs(yn) > 0.05)[0]
    if len(idx_resp):
        delay = max(0.0, float(tw[idx_resp[0]] - step_start))

    def _smooth(values: np.ndarray, width: int = 5) -> np.ndarray:
        kernel = np.ones(width) / width
        return np.convolve(values, kernel, mode="same")

    ys = _smooth(yn)
    tau: Optional[float] = None
    crossing = _crossing_time(tw, ys, 0.632, tw[0])
    if crossing is not None:
        tau = max(0.0, crossing - step_start)

    zeta: Optional[float] = None
    wn: Optional[float] = None
    peak_idx = int(np.argmax(ys))
    mp = float(ys[peak_idx]) - 1.0
    if mp > 0.05:
        ln = math.log(mp)
        zeta = -ln / math.sqrt(math.pi ** 2 + ln ** 2)
        peak_time = float(tw[peak_idx] - step_start)
        if peak_time > 1e-9 and zeta < 1.0:
            wn = math.pi / (peak_time * math.sqrt(1.0 - zeta ** 2))

    if order == "first":
        kind = "first-order"
    elif order == "second":
        kind = "second-order"
    else:
        kind = "second-order" if zeta is not None else "first-order"

    fit: Optional[dict[str, Any]] = None
    if kind == "first-order" and tau is not None and tau > 0:
        pred = 1.0 - np.exp(-(tw - step_start) / tau)
        fit = _fit_quality(yw, y0 + gain * pred)
    elif kind == "second-order" and zeta is not None and wn is not None and zeta < 1.0:
        wd = wn * math.sqrt(1.0 - zeta ** 2)
        s = tw - step_start
        pred = 1.0 - np.exp(-zeta * wn * s) * (
            np.cos(wd * s) + zeta / math.sqrt(1.0 - zeta ** 2) * np.sin(wd * s)
        )
        fit = _fit_quality(yw, y0 + gain * pred)

    model: dict[str, Any] = {"kind": kind, "gain": _r(gain), "delayS": _r(delay)}
    if kind == "first-order":
        model["timeConstantS"] = _r(tau) if tau is not None else None
        model["dampingRatio"] = None
        model["naturalFrequencyHz"] = None
    else:
        model["timeConstantS"] = None
        model["dampingRatio"] = _r(zeta) if zeta is not None else None
        model["naturalFrequencyHz"] = _r(wn / (2.0 * math.pi)) if wn is not None else None
    if fit is not None:
        model["fitQuality"] = fit

    method = "steady-state gain + 63.2% time (first order); peak overshoot & peak time analytic estimates (second order)"
    if order == "auto" and zeta is None:
        method += "; no overshoot observed, fell back to first-order fit"
    notes = [
        "适用于无显著噪声的阶跃响应数据；噪声会以近似比例影响 63.2% 时间与峰值特征。",
        "纯延迟估计的分辨率受采样率限制；提供 stepStart 或 setpoint 列可显著改善其准确度。",
        "二阶参数仅从峰值超调与峰值时间解析估计，对欠阻尼系统有效；过阻尼系统请用一阶模型。",
        "识别结果仅用于分析，不得直接用于真机参数整定（需人工确认）。",
    ]
    return {"ok": True, "model": model, "method": method, "notes": notes}


def cmd_system_identification(args: dict[str, Any]) -> dict[str, Any]:
    result = identify_system(args)
    result["inputArgs"] = {"path": args.get("path"), "order": args.get("order", "auto")}
    return result


# ---------------------------------------------------------------------------
# 7. control-report
# ---------------------------------------------------------------------------


def _render_trace(result: dict[str, Any]) -> list[str]:
    lines = [f"- 数据行数: {result.get('rows')}"]
    metrics = result.get("metrics") or {}
    fields = [
        ("riseTimeS", "上升时间 (s)"),
        ("settlingTimeS", "调节时间 (s)"),
        ("overshootPercent", "超调量 (%)"),
        ("steadyStateError", "稳态误差"),
        ("trackingErrorRms", "跟踪误差 RMS"),
        ("controlEffortRms", "控制量 RMS"),
        ("peakError", "峰值误差"),
    ]
    for key, label in fields:
        value = metrics.get(key)
        lines.append(f"- {label}: {value if value is not None else '—'}")
    issues = result.get("issues") or []
    if issues:
        lines.append("**Issues:**")
        for issue in issues:
            lines.append(f"- [{issue['severity']}] `{issue['code']}`: {issue['message']}")
    else:
        lines.append("- 未检测到异常")
    return lines


def _render_trajectory(result: dict[str, Any]) -> list[str]:
    stats = result.get("stats") or {}
    lines = [
        f"- 数据行数: {result.get('rows')}",
        "- 统计: "
        f"时长 {stats.get('durationS')} s · 平均间隔 {stats.get('meanDtS')} s · "
        f"最大间隔 {stats.get('maxDtS')} s · jitter {stats.get('jitterS')} s",
    ]
    issues = result.get("issues") or []
    if issues:
        lines.append("**Issues:**")
        for issue in issues:
            lines.append(f"- [{issue['severity']}] `{issue['code']}`: {issue['message']}")
    else:
        lines.append("- 未检测到异常")
    return lines


def _render_compare(result: dict[str, Any]) -> list[str]:
    summary = result.get("summary") or {}
    lines = [
        f"- 对齐样本: {result.get('alignedSamples')} · 时间偏移: {result.get('timeOffsetS')} s · "
        f"首次分歧: {result.get('firstDivergenceS') if result.get('firstDivergenceS') is not None else '无'} s",
        "",
        "| 关节 | RMS | 最大误差 | p50 | p90 | p99 |",
        "|---|---|---|---|---|---|",
    ]
    for name, stats in (result.get("perJoint") or {}).items():
        lines.append(
            f"| {name} | {stats.get('rms')} | {stats.get('max')} | "
            f"{stats.get('p50')} | {stats.get('p90')} | {stats.get('p99')} |"
        )
    return lines


def generate_control_report(args: dict[str, Any]) -> dict[str, Any]:
    out_path = args.get("outPath")
    sections = args.get("sections")
    if not out_path:
        raise WorkerError("missing required argument 'outPath'")
    if not isinstance(sections, list) or not sections:
        raise WorkerError("'sections' must be a non-empty list")

    lines = [
        "# 控制分析报告",
        "",
        f"> 生成时间：{_time.strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        f"> ⚠️ {REPORT_DISCLAIMER}",
        "",
    ]
    default_titles = {"trace": "控制跟踪分析", "trajectory": "轨迹校验", "compare": "计划 vs 实际对比"}
    for index, section in enumerate(sections, 1):
        if not isinstance(section, dict):
            raise WorkerError(f"section {index} must be an object with a 'kind'")
        kind = section.get("kind")
        title = section.get("title") or default_titles.get(kind) or str(kind)
        lines.append(f"## {index}. {title}")
        lines.append("")
        section_args = {key: value for key, value in section.items() if key not in ("kind", "title")}
        if kind == "trace":
            lines.extend(_render_trace(analyze_trace(section_args)))
        elif kind == "trajectory":
            lines.extend(_render_trajectory(validate_trajectory(section_args)))
        elif kind == "compare":
            lines.extend(_render_compare(compare_planned_actual(section_args)))
        else:
            raise WorkerError(f"unknown section kind {kind!r}; expected one of trace|trajectory|compare")
        lines.append("")

    lines.append("---")
    lines.append(f"> ⚠️ {REPORT_DISCLAIMER}")
    lines.append("")
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
    return {"ok": True, "path": out_path}


def cmd_control_report(args: dict[str, Any]) -> dict[str, Any]:
    result = generate_control_report(args)
    result["inputArgs"] = {
        "outPath": args.get("outPath"),
        "sections": len(args.get("sections") or []),
    }
    return result


# ---------------------------------------------------------------------------
# module exports
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "control-trace-analyze": cmd_control_trace_analyze,
    "trajectory-validate": cmd_trajectory_validate,
    "planned-actual-compare": cmd_planned_actual_compare,
    "pid-experiment-prepare": cmd_pid_experiment_prepare,
    "controller-config-compare": cmd_controller_config_compare,
    "system-identification": cmd_system_identification,
    "control-report": cmd_control_report,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "control.trace_analyze",
        "kind": "control",
        "risk": "R0-readonly",
        "description": "Analyze a control-loop trace: rise/settling/overshoot/SSE metrics plus oscillation, saturation, windup and noise-amplification issues.",
    },
    {
        "id": "control.trajectory_validate",
        "kind": "control",
        "risk": "R0-readonly",
        "description": "Validate a trajectory: time monotonicity/uniformity, joint limits, adjacent jumps, velocity and start-state continuity.",
    },
    {
        "id": "control.planned_actual_compare",
        "kind": "control",
        "risk": "R0-readonly",
        "description": "Align and compare planned vs actual joint trajectories: per-joint error stats, time offset and first divergence point.",
    },
    {
        "id": "control.system_identification",
        "kind": "control",
        "risk": "R0-readonly",
        "description": "Fit first/second-order models with pure delay to step-response data and report fit quality (R2, residual RMS).",
    },
    {
        "id": "control.experiment_prepare",
        "kind": "control",
        "risk": "R1-derive",
        "description": "Generate step/sweep PID experiment templates as waypoint lists; performs no hardware operations.",
    },
]
