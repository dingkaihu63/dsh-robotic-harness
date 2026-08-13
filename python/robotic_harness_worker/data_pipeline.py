"""Multimodal data pipeline for the Robotic Harness worker (plan chapter 14).

This is the largest data module of the suite. It covers the full data
lifecycle of a robotics dataset: inventory, schema inspection, time
synchronization, stream alignment, non-destructive transforms, episode
segmentation, annotation handling, leakage-safe splits, de-identification,
rosbag conversion, dataset exports (LeRobot / RLDS manifest), immutable
versioning, comparison and data cards.

Design principles enforced throughout (from the plan):

- Raw data is read-only: every command that produces derived data writes a
  new file or a new version; nothing is ever written back to an input path.
- Time synchronization must distinguish interpolable continuous signals from
  categorical/event data: linear interpolation is only ever offered for
  numeric columns, and the outputs carry explicit notes about it.
- Units and frames are explicit (seconds, Hz, mm/m, deg/rad); every numeric
  output is rounded to a stable number of decimals.
- Splits are leakage-safe: the ``group`` split method splits by group key, not
  by frame, and emits a leak-check summary.
- De-identification never claims anonymization: outputs carry the fixed
  privacy notes (de-identification != anonymization, ethics approval needed,
  processing stays local by default).

Dependencies: Python 3.10 stdlib + numpy (required). ``cv2`` and ``PIL`` are
optional and only needed for ``face-blur`` / ``exif-strip``; ``pyarrow`` is
optional and only used for the LeRobot parquet export (CSV fallback otherwise).
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import os
import random
import re
import shutil
import sqlite3
import struct
import time
import xml.etree.ElementTree as ET
from collections import Counter
from typing import Any, Callable, Optional

import numpy as np

from .core import WorkerError, sha256_file

try:  # optional, mirrors vision.py convention
    import cv2  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    cv2 = None  # type: ignore[assignment]

try:
    from PIL import Image  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    Image = None  # type: ignore[assignment]

# ---------------------------------------------------------------------------
# constants / small helpers
# ---------------------------------------------------------------------------

_FORMAT_ALIAS = {"jpeg": "jpg", "ndjson": "jsonl", "yml": "yaml"}

_KNOWN_FORMATS = {
    "csv", "jsonl", "json", "yaml", "parquet", "bag", "db3", "urdf", "xacro",
    "sdf", "mjcf", "stl", "obj", "dae", "step", "png", "jpg", "mp4", "bvh",
    "c3d", "txt", "log", "md", "xml", "toml", "npy", "npz", "h5", "hdf5",
    "pcd", "ply", "tif", "tiff", "webp", "gif", "wav", "zip", "gz", "pickle",
    "pkl", "csv.gz", "jsonl.gz",
}

_UNIT_FACTORS = {"m": 1.0, "mm": 1e-3, "rad": 1.0, "deg": math.pi / 180.0}

_EMAIL_RE = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
_PHONE_RE = re.compile(r"(?<!\d)1[3-9]\d{9}(?!\d)")
_IDCARD_RE = re.compile(r"(?<!\d)\d{17}[\dXx](?!\d)")

_PRIVACY_NOTES = [
    "去标识化 ≠ 匿名化：删除/模糊元数据与标识符不能保证不可再识别，请结合数据使用场景评估重识别风险",
    "涉及人类参与者数据的处理需要伦理审批（IRB/机构伦理委员会），请在使用前确认合规",
    "默认本地处理：PII 扫描与人脸模糊在本地执行，不向任何外部服务上传数据",
]

_COMMON_TIME_COLS = ("t", "time", "timestamp", "ts")


def _j(value: Any) -> Any:
    """Recursively convert numpy types to plain JSON-serializable values."""
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return [_j(v) for v in value.tolist()]
    if isinstance(value, dict):
        return {str(k): _j(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_j(v) for v in value]
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return None
    return value


def _r(value: Any, n: int = 6) -> Any:
    """Round a float (or leave other values alone)."""
    if isinstance(value, float) and math.isfinite(value):
        return round(value, n)
    return value


def _safe_float(value: Any) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _format_of(path: str) -> str:
    """Return the normalized format for a path ('' for no extension)."""
    base = os.path.basename(path)
    if base.lower().endswith(".csv.gz") or base.lower().endswith(".jsonl.gz"):
        ext = base.lower().rsplit(".", 2)[-2] + ".gz"
    else:
        ext = os.path.splitext(base)[1].lower().lstrip(".")
    return _FORMAT_ALIAS.get(ext, ext)


def _slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "-", value).strip("-") or "item"


def _abs(path: str) -> str:
    return os.path.abspath(path)


def _win_drive(path: str) -> str:
    if os.name != "nt":
        return ""
    return os.path.splitdrive(os.path.abspath(path))[0]


def _assert_outdir_not_c(out_dir: str) -> None:
    """Reject C:-drive outputs on Windows (mirrors the ros module policy)."""
    drive = _win_drive(out_dir)
    if drive.upper().startswith("C"):
        raise WorkerError(
            f"outDir must not be on the C: drive (got {out_dir!r}); "
            "pass an outDir on another drive (e.g. TEMP/F:)"
        )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise WorkerError(message)


# ---------------------------------------------------------------------------
# tabular IO (csv / jsonl)
# ---------------------------------------------------------------------------

def _read_csv(path: str) -> tuple[str, list[str], list[dict[str, Any]], int]:
    columns: list[str] = []
    rows: list[dict[str, Any]] = []
    parse_errors = 0
    with open(path, encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration:
            raise WorkerError(f"{path}: file has no header row")
        header = [h.strip() for h in header]
        if not header or all(h == "" for h in header):
            raise WorkerError(f"{path}: empty header row")
        columns = list(header)
        for line in reader:
            if not line:
                continue
            if len(line) != len(header):
                parse_errors += 1
                continue
            rows.append(dict(zip(header, line)))
    return "csv", columns, rows, parse_errors


def _read_jsonl(path: str) -> tuple[str, list[str], list[dict[str, Any]], int]:
    columns: list[str] = []
    rows: list[dict[str, Any]] = []
    parse_errors = 0
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                parse_errors += 1
                continue
            if not isinstance(record, dict):
                parse_errors += 1
                continue
            for key in record:
                if key not in columns:
                    columns.append(key)
            rows.append(record)
    return "jsonl", columns, rows, parse_errors


def _read_table(path: str, fmt: Optional[str] = None) -> tuple[str, list[str], list[dict[str, Any]], int]:
    """Read a csv/jsonl table. Returns (fmt, columns, rows, parse_errors)."""
    path = _abs(path)
    _require(os.path.exists(path), f"file not found: {path}")
    fmt = fmt or _format_of(path)
    if fmt in ("csv", "tsv"):
        return _read_csv(path)
    if fmt in ("jsonl", "ndjson"):
        return _read_jsonl(path)
    raise WorkerError(f"unsupported tabular format {fmt!r}; supported: csv, jsonl (pass format for odd extensions)")


def _union_columns(rows: list[dict[str, Any]]) -> list[str]:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    return columns


def _csv_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        return json.dumps(_j(value), ensure_ascii=False)
    return str(value)


def _write_table(path: str, columns: list[str], rows: list[dict[str, Any]], fmt: Optional[str] = None) -> str:
    path = _abs(path)
    fmt = fmt or ("jsonl" if path.endswith(".jsonl") else "csv")
    if fmt == "jsonl":
        with open(path, "w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(_j(row), ensure_ascii=False) + "\n")
    else:
        cols = columns or _union_columns(rows)
        with open(path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(cols)
            for row in rows:
                writer.writerow([_csv_cell(row.get(c)) for c in cols])
    return path


def _extract_times(rows: list[dict[str, Any]], time_column: str) -> tuple[list[Optional[float]], int]:
    times: list[Optional[float]] = []
    missing = 0
    for row in rows:
        value = _safe_float(row.get(time_column))
        if value is None:
            missing += 1
            times.append(None)
        else:
            times.append(value)
    return times, missing


def _time_range(rows: list[dict[str, Any]], time_column: str) -> Optional[dict[str, Any]]:
    values = [t for t in _extract_times(rows, time_column)[0] if t is not None and math.isfinite(t)]
    if not values:
        return None
    return {"min": round(min(values), 6), "max": round(max(values), 6)}


def _require_time_column(columns: list[str], time_column: str) -> None:
    _require(time_column in columns, f"time column {time_column!r} not found in columns {columns}")


def _is_numeric_col(rows: list[dict[str, Any]], column: str) -> bool:
    """True if every non-empty value of the column parses as a number."""
    values = [_safe_float(r.get(column)) for r in rows]
    if not values:
        return False
    return all(v is not None for v in values)


def _value_kind(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, (int, float)):
        return "number"
    text = str(value).strip()
    if not text:
        return None
    if text.lower() in ("true", "false"):
        return "boolean"
    try:
        float(text)
        return "number"
    except ValueError:
        return "string"


def _infer_dtype(values: list[Any]) -> str:
    kinds = {k for v in values if (k := _value_kind(v)) is not None}
    if not kinds:
        return "missing"
    if len(kinds) == 1:
        return kinds.pop()
    return "mixed"


# ---------------------------------------------------------------------------
# 1. data-inventory
# ---------------------------------------------------------------------------

def _integrity_issue(path: str, fmt: str) -> Optional[dict[str, Any]]:
    """Cheap corruption checks for known formats. Returns None when clean."""
    try:
        size = os.path.getsize(path)
    except OSError as error:
        return {"severity": "warning", "code": "file.unreadable", "message": f"cannot stat: {error}", "path": path}
    if size == 0:
        return {"severity": "warning", "code": "file.empty", "message": "file is empty", "path": path}
    try:
        with open(path, "rb") as handle:
            head = handle.read(16)
    except OSError as error:
        return {"severity": "warning", "code": "file.unreadable", "message": f"cannot read: {error}", "path": path}
    if fmt == "png" and not head.startswith(b"\x89PNG\r\n\x1a\n"):
        return {"severity": "warning", "code": "file.corrupt", "message": "png magic bytes mismatch", "path": path}
    if fmt == "jpg" and not head.startswith(b"\xff\xd8"):
        return {"severity": "warning", "code": "file.corrupt", "message": "jpeg magic bytes mismatch", "path": path}
    if fmt == "mp4" and not (len(head) > 8 and head[4:8] == b"ftyp"):
        return {"severity": "warning", "code": "file.corrupt", "message": "mp4 ftyp box missing", "path": path}
    if fmt == "json":
        try:
            with open(path, encoding="utf-8") as handle:
                json.load(handle)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {"severity": "warning", "code": "file.corrupt", "message": "invalid JSON document", "path": path}
    if fmt == "jsonl":
        bad = 0
        try:
            with open(path, encoding="utf-8") as handle:
                for line in handle:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        json.loads(line)
                    except json.JSONDecodeError:
                        bad += 1
        except UnicodeDecodeError:
            return {"severity": "warning", "code": "file.corrupt", "message": "not UTF-8 text", "path": path}
        if bad:
            return {"severity": "warning", "code": "file.corrupt", "message": f"{bad} invalid JSONL line(s)", "path": path}
    if fmt in ("urdf", "xacro", "sdf", "mjcf", "xml"):
        try:
            ET.parse(path)
        except ET.ParseError as error:
            return {"severity": "warning", "code": "file.corrupt", "message": f"XML parse error: {error}", "path": path}
    return None


def cmd_data_inventory(args: dict[str, Any]) -> dict[str, Any]:
    """Scan a directory or single file and report size, sha256, format, issues."""
    path = args.get("path")
    _require(path, "missing required argument 'path'")
    path = _abs(path)
    _require(os.path.exists(path), f"path not found: {path}")
    recursive = bool(args.get("recursive", True))

    root = path if os.path.isdir(path) else os.path.dirname(path)
    targets: list[str] = []
    if os.path.isfile(path):
        targets.append(path)
    else:
        for dirpath, dirnames, filenames in os.walk(path):
            if not recursive and dirpath != path:
                dirnames[:] = []
                continue
            for name in sorted(filenames):
                targets.append(os.path.join(dirpath, name))
    targets.sort()

    files: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    for target in targets:
        fmt = _format_of(target)
        fmt = fmt if fmt in _KNOWN_FORMATS else ("unknown" if fmt else "unknown")
        entry: dict[str, Any] = {"path": target, "size": 0, "sha256": "", "format": fmt}
        try:
            entry["size"] = os.path.getsize(target)
            entry["modifiedAt"] = round(os.path.getmtime(target), 3)
            entry["sha256"] = sha256_file(target)
        except OSError as error:
            issues.append({"severity": "warning", "code": "file.unreadable", "message": f"cannot read: {error}", "path": target})
        if not entry["sha256"]:
            issues.append({"severity": "warning", "code": "file.unreadable", "message": "sha256 computation failed", "path": target})
        check = _integrity_issue(target, fmt)
        if check:
            issues.append(check)
        files.append(entry)

    formats: dict[str, int] = Counter(f["format"] for f in files)
    total_size = sum(f["size"] for f in files)
    return {
        "ok": True,
        "root": root,
        "files": files,
        "formats": dict(formats),
        "totalSize": total_size,
        "issues": issues,
        "inputArgs": {"path": path, "recursive": recursive},
    }


# ---------------------------------------------------------------------------
# 2. data-schema-inspect
# ---------------------------------------------------------------------------

def cmd_data_schema_inspect(args: dict[str, Any]) -> dict[str, Any]:
    """Inspect column dtypes / missing / samples for a CSV or JSONL table."""
    path = args.get("path")
    _require(path, "missing required argument 'path'")
    fmt, columns, rows, parse_errors = _read_table(path, fmt=args.get("format"))
    time_column = args.get("timeColumn") or _COMMON_TIME_COLS[0]

    schema: list[dict[str, Any]] = []
    for column in columns:
        values = [r.get(column) for r in rows]
        non_empty = [v for v in values if not (v is None or str(v).strip() == "")]
        missing = len(values) - len(non_empty)
        schema.append(
            {
                "name": column,
                "dtype": _infer_dtype(values),
                "missing": missing,
                "sampleValues": [_j(v) for v in non_empty[:3]],
            }
        )

    result: dict[str, Any] = {
        "ok": True,
        "format": fmt,
        "path": _abs(path),
        "columns": schema,
        "rows": len(rows),
        "parseErrors": parse_errors,
    }
    if time_column in columns:
        result["timeColumn"] = time_column
        result["timeRange"] = _time_range(rows, time_column)
    result["inputArgs"] = {"path": path, "timeColumn": time_column}
    return result


# ---------------------------------------------------------------------------
# 3. data-time-sync-estimate
# ---------------------------------------------------------------------------

def _nearest_index(times: list[Optional[float]], target: float) -> Optional[int]:
    """Index of the time value nearest to ``target`` (None when no valid time)."""
    valid = [(i, v) for i, v in enumerate(times) if v is not None and math.isfinite(v)]
    if not valid:
        return None
    ts = [v for _, v in valid]
    idxs = [i for i, _ in valid]
    pos = bisect.bisect_left(ts, target)
    if pos == 0:
        return idxs[0]
    if pos >= len(ts):
        return idxs[-1]
    left, right = pos - 1, pos
    return idxs[left] if (target - ts[left]) <= (ts[right] - target) else idxs[right]


def _nearest_values(grid: np.ndarray, times_ref: np.ndarray, values_ref: np.ndarray) -> np.ndarray:
    """Nearest-neighbour lookup of values_ref at grid times (times_ref sorted)."""
    pos = np.searchsorted(times_ref, grid)
    pos = np.clip(pos, 0, len(times_ref) - 1)
    left = np.clip(pos - 1, 0, len(times_ref) - 1)
    use_left = np.abs(times_ref[left] - grid) < np.abs(times_ref[pos] - grid)
    return np.where(use_left, values_ref[left], values_ref[pos])


def _load_signal(path: str, time_column: str, signal_column: Optional[str]) -> tuple[np.ndarray, np.ndarray, int]:
    _, columns, rows, _ = _read_table(path)
    _require_time_column(columns, time_column)
    if signal_column is not None:
        _require(signal_column in columns, f"signal column {signal_column!r} not found in {path} (columns: {columns})")
    times, missing = _extract_times(rows, time_column)
    valid = [(i, t) for i, t in enumerate(times) if t is not None and math.isfinite(t)]
    _require(len(valid) >= 2, f"{path}: need at least 2 rows with valid time column {time_column!r}")
    t_arr = np.array([v for _, v in valid], dtype=float)
    if signal_column is None:
        return t_arr, np.zeros(len(t_arr)), missing
    values = np.array([_safe_float(rows[i].get(signal_column)) for i, _ in valid], dtype=float)
    _require(np.all(np.isfinite(values)), f"{path}: signal column {signal_column!r} has missing/non-finite values")
    # dedupe timestamps, keep first occurrence
    t_arr, unique_idx = np.unique(t_arr, return_index=True)
    return t_arr, values[unique_idx], missing


def cmd_data_time_sync_estimate(args: dict[str, Any]) -> dict[str, Any]:
    """Estimate the fixed time offset between two streams."""
    path_a = args.get("pathA")
    path_b = args.get("pathB")
    _require(path_a and path_b, "missing required arguments 'pathA' and 'pathB'")
    time_a = args.get("timeColumnA") or "t"
    time_b = args.get("timeColumnB") or "t"
    max_lag = float(args.get("maxLagS", 10.0) or 10.0)
    signals = args.get("signalColumns") or {}
    sig_a = signals.get("a") if isinstance(signals, dict) else None
    sig_b = signals.get("b") if isinstance(signals, dict) else None

    t_a, v_a, _ = _load_signal(path_a, time_a, sig_a)
    t_b, v_b, _ = _load_signal(path_b, time_b, sig_b)

    # grid spacing for lag search
    sample_rate = args.get("sampleRateHz")
    if sample_rate:
        dt = 1.0 / float(sample_rate)
    else:
        combined = np.sort(np.concatenate([t_a, t_b]))
        diffs = np.diff(combined)
        diffs = diffs[diffs > 0]
        dt = float(np.median(diffs)) if len(diffs) else 0.01
    dt = max(dt, 1e-6)

    if sig_a is not None or sig_b is not None:
        _require(sig_a and sig_b, "signalColumns must provide both 'a' and 'b'")
        lags = np.arange(-max_lag, max_lag + dt / 2, dt)
        corrs = np.full(len(lags), np.nan)
        min_valid = max(5, int(0.05 * len(t_a)))
        for i, lag in enumerate(lags):
            target = t_a + lag
            valid = (target >= t_b[0]) & (target <= t_b[-1])
            if valid.sum() < min_valid:
                continue
            va = v_a[valid]
            vb = _nearest_values(target[valid], t_b, v_b)
            if va.std() == 0.0 or vb.std() == 0.0:
                corrs[i] = 0.0
            else:
                corrs[i] = float(np.corrcoef(va, vb)[0, 1])
        if not np.any(np.isfinite(corrs)):
            raise WorkerError("cross-correlation failed: no lag with enough overlapping samples")
        best = int(np.nanargmax(corrs))
        peak = float(corrs[best])
        # parabolic refinement around the peak
        if 0 < best < len(corrs) - 1 and corrs[best - 1] < peak and corrs[best + 1] < peak:
            denom = corrs[best - 1] - 2 * peak + corrs[best + 1]
            if denom != 0:
                delta = 0.5 * (corrs[best - 1] - corrs[best + 1]) / denom
                offset = lags[best] + delta * dt
            else:
                offset = float(lags[best])
        else:
            offset = float(lags[best])
        second_best = float(np.nanmax(np.where(np.arange(len(corrs)) == best, np.nan, corrs)))
        prominence = peak - second_best
        if peak > 0.9 and prominence > 0.05:
            confidence = "high"
        elif peak > 0.7:
            confidence = "medium"
        else:
            confidence = "low"
        note = (
            f"cross-correlation of {sig_a!r} (A) vs {sig_b!r} (B) on a {dt:g}s grid; "
            "offsetS = shift added to B's clock to align with A. Categorical/event columns "
            "should NOT be shifted with this estimate; verify with a semantic check."
        )
        result: dict[str, Any] = {
            "ok": True,
            "offsetS": round(offset, 4),
            "method": "cross-correlation",
            "correlation": round(peak, 4),
            "confidence": confidence,
            "note": note,
            "gridS": round(dt, 6),
            "lagsEvaluated": len(lags),
        }
    else:
        # coarse estimate: mean timestamp difference of nearest neighbours
        diffs: list[float] = []
        for t in t_a:
            if t < t_b[0] or t > t_b[-1]:
                continue
            idx = _nearest_index(list(t_b), float(t))
            if idx is not None:
                diffs.append(float(t_b[idx]) - float(t))
        _require(len(diffs) >= 2, "mean-difference estimate needs >= 2 overlapping timestamps")
        offset = float(np.mean(diffs))
        result = {
            "ok": True,
            "offsetS": round(offset, 4),
            "method": "mean-difference",
            "correlation": None,
            "confidence": "low",
            "note": (
                "mean timestamp difference of nearest neighbours; this is a coarse estimate that "
                "assumes timestamps already correspond, it is sensitive to outliers and does not "
                "validate the signals themselves. Provide signalColumns for cross-correlation."
            ),
            "samplesUsed": len(diffs),
        }
    result["inputArgs"] = {"pathA": path_a, "pathB": path_b, "timeColumnA": time_a, "timeColumnB": time_b}
    return result


# ---------------------------------------------------------------------------
# 4. data-align-streams
# ---------------------------------------------------------------------------

def cmd_data_align_streams(args: dict[str, Any]) -> dict[str, Any]:
    """Align multiple streams onto a primary time axis."""
    strategy = args.get("strategy", "nearest")
    _require(strategy in ("nearest", "exact", "window"), f"unknown strategy {strategy!r}; use nearest|exact|window")
    max_gap = float(args.get("maxGapS", 0.1) or 0.1)

    streams: list[dict[str, Any]] = []
    if args.get("primary"):
        streams.append({"path": _abs(args["primary"]), "timeColumn": args.get("timeColumn") or "t"})
    if args.get("secondary"):
        streams.append({"path": _abs(args["secondary"]), "timeColumn": "t"})
    for item in args.get("files") or []:
        streams.append({"path": _abs(item.get("path")), "timeColumn": item.get("timeColumn") or "t"})
    _require(len(streams) >= 2, "need a 'primary' stream and at least one secondary stream (secondary or files)")

    primary = streams[0]
    _, p_columns, p_rows, _ = _read_table(primary["path"])
    _require_time_column(p_columns, primary["timeColumn"])
    p_times, _ = _extract_times(p_rows, primary["timeColumn"])

    secondaries: list[dict[str, Any]] = []
    for spec in streams[1:]:
        _, columns, rows, _ = _read_table(spec["path"])
        _require_time_column(columns, spec["timeColumn"])
        times, _ = _extract_times(rows, spec["timeColumn"])
        secondaries.append({"spec": spec, "columns": columns, "rows": rows, "times": times})

    # rename colliding columns deterministically (primary keeps its names)
    used: set[str] = set(p_columns)
    rename_maps: list[dict[str, str]] = []
    for index, sec in enumerate(secondaries):
        mapping: dict[str, str] = {}
        for column in sec["columns"]:
            name = column
            if name in used:
                name = f"{column}_{index + 2}"
            mapping[column] = name
            used.add(name)
        rename_maps.append(mapping)

    stats = [
        {"path": sec["spec"]["path"], "timeColumn": sec["spec"]["timeColumn"], "samples": len(sec["rows"]), "matched": 0, "unmatched": 0, "gaps": 0}
        for sec in secondaries
    ]

    def mark(stat: dict[str, Any], matched: bool) -> None:
        stat["matched" if matched else "unmatched"] += 1
        if not matched:
            stat["gaps"] += 1

    aligned: list[dict[str, Any]] = []
    for i, row in enumerate(p_rows):
        base = dict(row)
        t_p = p_times[i]
        for si, sec in enumerate(secondaries):
            mapping = rename_maps[si]
            times_s = sec["times"]
            rows_s = sec["rows"]
            cols_s = sec["columns"]
            if strategy == "nearest":
                if t_p is None:
                    for c in cols_s:
                        base[mapping[c]] = None
                    mark(stats[si], False)
                    continue
                idx = _nearest_index(times_s, t_p)
                if idx is None or abs(times_s[idx] - t_p) > max_gap:
                    for c in cols_s:
                        base[mapping[c]] = None
                    mark(stats[si], False)
                else:
                    for c in cols_s:
                        base[mapping[c]] = rows_s[idx].get(c)
                    mark(stats[si], True)
            elif strategy == "exact":
                if t_p is None:
                    for c in cols_s:
                        base[mapping[c]] = None
                    mark(stats[si], False)
                    continue
                idx = _nearest_index(times_s, t_p)
                if idx is None or abs(times_s[idx] - t_p) > 1e-6:
                    for c in cols_s:
                        base[mapping[c]] = None
                    mark(stats[si], False)
                else:
                    for c in cols_s:
                        base[mapping[c]] = rows_s[idx].get(c)
                    mark(stats[si], True)
            else:  # window: fixed-window mean for numeric columns
                if t_p is None:
                    for c in cols_s:
                        base[mapping[c]] = None
                    mark(stats[si], False)
                    continue
                window_rows = [rows_s[j] for j, tv in enumerate(times_s) if tv is not None and abs(tv - t_p) <= max_gap]
                if not window_rows:
                    for c in cols_s:
                        base[mapping[c]] = None
                    mark(stats[si], False)
                    continue
                for c in cols_s:
                    if _is_numeric_col(window_rows, c):
                        vals = [_safe_float(wr.get(c)) for wr in window_rows]
                        vals = [v for v in vals if v is not None]
                        base[mapping[c]] = round(float(np.mean(vals)), 9) if vals else None
                    else:
                        base[mapping[c]] = window_rows[0].get(c)
                mark(stats[si], True)
        aligned.append(base)

    out_path = None
    if args.get("outPath"):
        out_path = _write_table(args["outPath"], [], aligned)

    note = (
        f"strategy={strategy}, maxGapS={max_gap:g}. After alignment only continuous numeric columns "
        "may be safely interpolated/filled; categorical, event and discrete label columns must not be "
        "linearly interpolated - keep them None when unmatched."
    )
    return {
        "ok": True,
        "alignedSamples": aligned,
        "streams": [{"path": primary["path"], "timeColumn": primary["timeColumn"], "samples": len(p_rows), "matched": len(p_rows), "unmatched": 0, "gaps": 0}, *stats],
        "outPath": out_path,
        "note": note,
        "inputArgs": {"primary": primary["path"], "strategy": strategy, "maxGapS": max_gap},
    }


# ---------------------------------------------------------------------------
# 5. data-transform-apply
# ---------------------------------------------------------------------------

def _iir_forward(x: np.ndarray, alpha: float) -> np.ndarray:
    y = np.empty_like(x)
    acc = float(x[0])
    y[0] = acc
    for i in range(1, len(x)):
        acc = alpha * acc + (1.0 - alpha) * float(x[i])
        y[i] = acc
    return y


def _iir_filtfilt(x: np.ndarray, alpha: float) -> np.ndarray:
    forward = _iir_forward(x, alpha)
    backward = _iir_forward(forward[::-1], alpha)
    return backward[::-1]


def _sliding_median(x: np.ndarray, window: int) -> np.ndarray:
    half = window // 2
    padded = np.pad(x, (half, half), mode="edge")
    shape = (len(x), window)
    strides = (padded.strides[0], padded.strides[0])
    windows = np.lib.stride_tricks.as_strided(padded, shape=shape, strides=strides)
    return np.median(windows, axis=1)


def _op_range_filter(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    _require(column, "range-filter requires 'column'")
    lo = params.get("min")
    hi = params.get("max")
    kept: list[dict[str, Any]] = []
    removed = 0
    for row in rows:
        value = _safe_float(row.get(column))
        if value is None or not math.isfinite(value):
            removed += 1
            continue
        if lo is not None and value < float(lo):
            removed += 1
            continue
        if hi is not None and value > float(hi):
            removed += 1
            continue
        kept.append(row)
    return kept, removed, f"removed {removed} rows outside [{lo if lo is not None else '-inf'}, {hi if hi is not None else 'inf'}] on {column!r}"


def _op_dedupe(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    seen: set[Any] = set()
    kept: list[dict[str, Any]] = []
    removed = 0
    for row in rows:
        value = _safe_float(row.get(time_column))
        key = None if value is None else round(value, 9)
        if key in seen:
            removed += 1
            continue
        seen.add(key)
        kept.append(row)
    return kept, removed, f"removed {removed} duplicate rows by {time_column!r} (kept first occurrence)"


def _op_sort(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    def key(row: dict[str, Any]) -> tuple[int, float]:
        value = _safe_float(row.get(time_column))
        return (0, value) if value is not None else (1, 0.0)

    order = sorted(range(len(rows)), key=lambda i: key(rows[i]))
    moved = sum(1 for a, b in zip(order, range(len(rows))) if a != b)
    return [rows[i] for i in order], moved, f"sorted {len(rows)} rows by {time_column!r} ({moved} rows changed position)"


def _op_interpolate_gaps(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    _require(column, "interpolate-gaps requires 'column'")
    max_gap = float(params.get("maxGapS", 0.5) or 0.5)
    raw_values = [r.get(column) for r in rows]
    unparseable = [v for v in raw_values if str(v).strip() != "" and _safe_float(v) is None]
    if unparseable:
        return rows, 0, f"column {column!r} contains non-numeric values (e.g. {unparseable[0]!r}); skipped (categorical columns must not be interpolated)"
    values = [_safe_float(v) for v in raw_values]
    times = [_safe_float(r.get(time_column)) for r in rows]
    n = len(rows)
    out = [dict(r) for r in rows]
    interpolated = 0
    skipped = 0
    i = 0
    while i < n:
        if values[i] is None or not math.isfinite(values[i]):
            j = i
            while j < n and (values[j] is None or not math.isfinite(values[j])):
                j += 1
            prev = i - 1
            while prev >= 0 and (values[prev] is None or not math.isfinite(values[prev])):
                prev -= 1
            nxt = j
            while nxt < n and (values[nxt] is None or not math.isfinite(values[nxt])):
                nxt += 1
            t_prev = times[prev] if prev >= 0 else None
            t_next = times[nxt] if nxt < n else None
            if prev >= 0 and nxt < n and t_prev is not None and t_next is not None and (t_next - t_prev) <= max_gap:
                span = t_next - t_prev
                for k in range(i, j):
                    t_k = times[k]
                    if t_k is None:
                        skipped += 1
                        continue
                    frac = (t_k - t_prev) / span if span > 0 else 0.5
                    out[k][column] = round(values[prev] + (values[nxt] - values[prev]) * frac, 9)
                    interpolated += 1
            else:
                skipped += j - i
            i = j
        else:
            i += 1
    return out, interpolated, f"interpolated {interpolated} missing values across gaps <= {max_gap:g}s; skipped {skipped} (gap too large or at edges)"


def _op_lowpass(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    _require(column, "lowpass requires 'column'")
    cutoff = float(params.get("cutoffHz", 0.0))
    rate = float(params.get("sampleRateHz", 0.0))
    _require(cutoff > 0 and rate > 0, "lowpass requires positive cutoffHz and sampleRateHz")
    values = [_safe_float(r.get(column)) for r in rows]
    if any(v is None or not math.isfinite(v) for v in values):
        return rows, 0, f"skipped: column {column!r} has missing/non-finite values"
    x = np.asarray(values, dtype=float)
    alpha = math.exp(-2.0 * math.pi * cutoff / rate)
    y = _iir_filtfilt(x, alpha)
    out = [dict(r) for r in rows]
    for i in range(len(rows)):
        out[i][column] = round(float(y[i]), 9)
    return out, len(rows), f"1st-order IIR forward-backward lowpass fc={cutoff:g}Hz fs={rate:g}Hz (alpha={alpha:.4f})"


def _op_median(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    _require(column, "median requires 'column'")
    window = int(params.get("window", 5))
    _require(window >= 1, "median window must be >= 1")
    values = [_safe_float(r.get(column)) for r in rows]
    if any(v is None or not math.isfinite(v) for v in values):
        return rows, 0, f"skipped: column {column!r} has missing/non-finite values"
    x = np.asarray(values, dtype=float)
    y = _sliding_median(x, window)
    out = [dict(r) for r in rows]
    for i in range(len(rows)):
        out[i][column] = round(float(y[i]), 9)
    return out, len(rows), f"sliding median window={window} on {column!r}"


def _op_resample(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    rate = float(params.get("rateHz", 0.0))
    _require(rate > 0, "resample requires positive rateHz")
    valid = [(t, r) for t, r in zip((_safe_float(r.get(time_column)) for r in rows), rows) if t is not None and math.isfinite(t)]
    _require(len(valid) >= 2, "resample needs at least 2 rows with a valid time column")
    valid.sort(key=lambda pair: pair[0])
    ts = np.array([pair[0] for pair in valid], dtype=float)
    columns = _union_columns([pair[1] for pair in valid])
    new_t = np.arange(ts[0], ts[-1] + 1e-9, 1.0 / rate)
    out: list[dict[str, Any]] = []
    forward_filled: list[str] = []
    for tt in new_t:
        row: dict[str, Any] = {time_column: round(float(tt), 9)}
        for column in columns:
            if column == time_column:
                continue
            arr = np.array([_safe_float(pair[1].get(column)) for pair in valid], dtype=float)
            if _is_numeric_col([pair[1] for pair in valid], column) and np.all(np.isfinite(arr)):
                row[column] = round(float(np.interp(tt, ts, arr)), 9)
            else:
                idx = max(0, min(int(np.searchsorted(ts, tt, side="right")) - 1, len(valid) - 1))
                row[column] = valid[idx][1].get(column)
                if column not in forward_filled:
                    forward_filled.append(column)
        out.append(row)
    note_cols = ", ".join(forward_filled) if forward_filled else "none"
    return out, len(out), f"resampled {len(valid)} -> {len(out)} rows at {rate:g}Hz; non-numeric columns forward-filled ({note_cols})"


def _op_detrend(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    _require(column, "detrend requires 'column'")
    times = [_safe_float(r.get(time_column)) for r in rows]
    values = [_safe_float(r.get(column)) for r in rows]
    if any(v is None or not math.isfinite(v) for v in values) or any(t is None or not math.isfinite(t) for t in times):
        return rows, 0, f"skipped: column {column!r} or time column has missing/non-finite values"
    t = np.asarray(times, dtype=float)
    x = np.asarray(values, dtype=float)
    slope, intercept = np.polyfit(t, x, 1)
    y = x - (slope * t + intercept)
    out = [dict(r) for r in rows]
    for i in range(len(rows)):
        out[i][column] = round(float(y[i]), 9)
    return out, len(rows), f"subtracted linear least-squares trend (slope={slope:.6g}, intercept={intercept:.6g}) on {column!r}"


def _op_unit_convert(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    from_unit = params.get("from")
    to_unit = params.get("to")
    _require(column and from_unit and to_unit, "unit-convert requires column, from, to")
    factor = params.get("factor")
    if factor is None:
        _require(
            from_unit in _UNIT_FACTORS and to_unit in _UNIT_FACTORS,
            f"unknown unit {from_unit!r}->{to_unit!r}; supported: {sorted(_UNIT_FACTORS)} or pass an explicit factor",
        )
        factor = _UNIT_FACTORS[from_unit] / _UNIT_FACTORS[to_unit]
    factor = float(factor)
    out = [dict(r) for r in rows]
    affected = 0
    for row in out:
        value = _safe_float(row.get(column))
        if value is not None:
            row[column] = round(value * factor, 9)
            affected += 1
    return out, affected, f"converted {column!r} {from_unit}->{to_unit} (factor={factor:g}, {affected} values)"


def _op_round(rows: list[dict[str, Any]], params: dict[str, Any], time_column: str) -> tuple[list[dict[str, Any]], int, str]:
    column = params.get("column")
    _require(column, "round requires 'column'")
    decimals = int(params.get("decimals", 3))
    out = [dict(r) for r in rows]
    affected = 0
    for row in out:
        value = _safe_float(row.get(column))
        if value is not None:
            row[column] = round(value, decimals)
            affected += 1
    return out, affected, f"rounded {column!r} to {decimals} decimals ({affected} values)"


_OPERATIONS: dict[str, Callable[[list[dict[str, Any]], dict[str, Any], str], tuple[list[dict[str, Any]], int, str]]] = {
    "range-filter": _op_range_filter,
    "dedupe": _op_dedupe,
    "sort": _op_sort,
    "interpolate-gaps": _op_interpolate_gaps,
    "lowpass": _op_lowpass,
    "median": _op_median,
    "resample": _op_resample,
    "detrend": _op_detrend,
    "unit-convert": _op_unit_convert,
    "round": _op_round,
}


def cmd_data_transform_apply(args: dict[str, Any]) -> dict[str, Any]:
    """Chain non-destructive transforms and write the result to outPath."""
    input_path = args.get("inputPath")
    out_path = args.get("outPath")
    _require(input_path and out_path, "missing required arguments 'inputPath' and 'outPath'")
    input_path = _abs(input_path)
    out_path = _abs(out_path)
    _require(os.path.normcase(input_path) != os.path.normcase(out_path), "outPath must differ from inputPath (raw data is read-only)")
    _require(out_path.endswith(".csv") or out_path.endswith(".jsonl"), "outPath must end with .csv or .jsonl")
    time_column = args.get("timeColumn") or "t"
    operations = args.get("operations") or []
    _require(isinstance(operations, list) and operations, "operations must be a non-empty list")

    fmt, columns, rows, _ = _read_table(input_path)
    _require_time_column(columns, time_column)
    before = {"rows": len(rows), "timeRange": _time_range(rows, time_column)}

    applied: list[dict[str, Any]] = []
    current = rows
    notes: list[str] = []
    for spec in operations:
        kind = spec.get("kind")
        _require(kind in _OPERATIONS, f"unknown operation {kind!r}; supported: {sorted(_OPERATIONS)}")
        params = spec.get("params") or {}
        current, affected, detail = _OPERATIONS[kind](current, params, time_column)
        applied.append({"kind": kind, "affectedRows": affected, "detail": detail, "params": params})
        if kind == "interpolate-gaps":
            notes.append("interpolate-gaps only fills continuous numeric columns within maxGapS; categorical/event columns are left untouched")
        if kind == "resample":
            notes.append("resample uses linear interpolation for numeric columns and forward-fill for others - never linearly interpolate categorical columns")

    after = {"rows": len(current), "timeRange": _time_range(current, time_column)}
    out_path = _write_table(out_path, columns if fmt == "csv" else [], current)
    return {
        "ok": True,
        "inputPath": input_path,
        "outPath": out_path,
        "operations": applied,
        "before": before,
        "after": after,
        "notes": notes,
        "inputArgs": {"inputPath": input_path, "outPath": out_path, "timeColumn": time_column},
    }


# ---------------------------------------------------------------------------
# 6. data-segment-episodes
# ---------------------------------------------------------------------------

def cmd_data_segment_episodes(args: dict[str, Any]) -> dict[str, Any]:
    """Split a timeseries into episodes by time gaps."""
    path = args.get("path")
    _require(path, "missing required argument 'path'")
    time_column = args.get("timeColumn") or "t"
    max_gap = float(args.get("maxGapS", 2.0) or 2.0)
    label_column = args.get("labelColumn")

    _, columns, rows, _ = _read_table(path)
    _require_time_column(columns, time_column)
    if label_column:
        _require(label_column in columns, f"label column {label_column!r} not found in columns {columns}")

    valid = [(t, r) for t, r in zip((_safe_float(r.get(time_column)) for r in rows), rows) if t is not None and math.isfinite(t)]
    valid.sort(key=lambda pair: pair[0])

    episodes: list[dict[str, Any]] = []
    gaps: list[dict[str, Any]] = []
    current: list[tuple[float, dict[str, Any]]] = []
    for t, row in valid:
        if current:
            prev_t = current[-1][0]
            delta = t - prev_t
            if delta > max_gap:
                episodes.append(_finalize_episode(len(episodes), current, label_column))
                gaps.append({"t": round(prev_t, 6), "gapS": round(delta, 6)})
                current = []
        current.append((t, row))
    if current:
        episodes.append(_finalize_episode(len(episodes), current, label_column))

    return {
        "ok": True,
        "episodes": episodes,
        "gaps": gaps,
        "inputArgs": {"path": path, "timeColumn": time_column, "maxGapS": max_gap},
    }


def _finalize_episode(index: int, current: list[tuple[float, dict[str, Any]]], label_column: Optional[str]) -> dict[str, Any]:
    first_t = current[0][0]
    last_t = current[-1][0]
    episode: dict[str, Any] = {
        "id": f"episode-{index + 1:04d}",
        "startS": round(first_t, 6),
        "endS": round(last_t, 6),
        "rows": len(current),
        "durationS": round(last_t - first_t, 6),
    }
    if label_column:
        episode["labels"] = dict(Counter(str(row.get(label_column)) for _, row in current if row.get(label_column) is not None))
    return episode


# ---------------------------------------------------------------------------
# 7/8. annotations
# ---------------------------------------------------------------------------

def _normalize_annotation(record: dict[str, Any], time_column: Optional[str] = None) -> dict[str, Any]:
    t_value = record.get(time_column) if time_column else record.get("t", record.get("time"))
    out: dict[str, Any] = {
        "t": _safe_float(t_value),
        "startS": _safe_float(record.get("startS", record.get("start"))),
        "endS": _safe_float(record.get("endS", record.get("end"))),
        "label": None,
        "source": record.get("source"),
        "confidence": _safe_float(record.get("confidence")),
    }
    label = record.get("label", record.get("class", record.get("type")))
    out["label"] = None if label is None or str(label).strip() == "" else str(label)
    if record.get("status") is not None:
        out["status"] = str(record["status"])
    return out


def _read_annotations(path: str, fmt: Optional[str] = None, time_column: Optional[str] = None) -> list[dict[str, Any]]:
    path = _abs(path)
    _require(os.path.exists(path), f"file not found: {path}")
    fmt = fmt or _format_of(path)
    if fmt == "json":
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        records = data.get("annotations") if isinstance(data, dict) and isinstance(data.get("annotations"), list) else data
        _require(isinstance(records, list), f"{path}: JSON must be a list of annotations or {{annotations: [...]}}")
        return [_normalize_annotation(rec, time_column) for rec in records if isinstance(rec, dict)]
    if fmt in ("csv", "jsonl"):
        _, columns, rows, _ = _read_table(path, fmt=fmt)
        return [_normalize_annotation(row, time_column) for row in rows]
    raise WorkerError(f"unsupported annotation format {fmt!r}; supported: csv, jsonl, json")


def _annotation_counts(annotations: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "total": len(annotations),
        "withLabel": sum(1 for a in annotations if a.get("label") is not None),
        "withTime": sum(1 for a in annotations if a.get("t") is not None),
        "withInterval": sum(1 for a in annotations if a.get("startS") is not None and a.get("endS") is not None),
    }


def cmd_data_annotation_import(args: dict[str, Any]) -> dict[str, Any]:
    """Import annotations (episode bounds / events / quality labels)."""
    path = args.get("path")
    _require(path, "missing required argument 'path'")
    schema = args.get("schema") or {}
    time_column = (schema or {}).get("timeColumn")
    annotations = _read_annotations(path, fmt=args.get("format"), time_column=time_column)
    source = args.get("source") or os.path.basename(path)
    for annotation in annotations:
        if not annotation.get("source"):
            annotation["source"] = source

    out_path = None
    if args.get("outPath"):
        out_path = _write_annotations(args["outPath"], annotations)
    return {
        "ok": True,
        "annotations": annotations,
        "counts": _annotation_counts(annotations),
        "outPath": out_path,
        "inputArgs": {"path": path},
    }


def _write_annotations(path: str, annotations: list[dict[str, Any]]) -> str:
    path = _abs(path)
    fmt = "jsonl" if path.endswith(".jsonl") else "csv"
    if fmt == "jsonl":
        with open(path, "w", encoding="utf-8") as handle:
            for annotation in annotations:
                handle.write(json.dumps(_j(annotation), ensure_ascii=False) + "\n")
    else:
        columns = ["id", "t", "startS", "endS", "label", "source", "confidence", "status"]
        with open(path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(columns)
            for annotation in annotations:
                writer.writerow([annotation.get(c) if c != "label" else (annotation.get("label") or "") for c in columns])
    return path


def cmd_data_annotation_review(args: dict[str, Any]) -> dict[str, Any]:
    """List annotations or write a confirmed/rejected copy (never in place)."""
    path = args.get("path")
    _require(path, "missing required argument 'path'")
    action = args.get("action", "list")
    _require(action in ("list", "confirm", "reject"), f"unknown action {action!r}; use list|confirm|reject")
    annotations = _read_annotations(path)
    for index, annotation in enumerate(annotations):
        annotation["id"] = f"a{index + 1}"

    if action == "list":
        confirmed = sum(1 for a in annotations if a.get("status") == "confirmed")
        rejected = sum(1 for a in annotations if a.get("status") == "rejected")
        return {
            "ok": True,
            "annotations": annotations,
            "total": len(annotations),
            "confirmed": confirmed,
            "rejected": rejected,
            "inputArgs": {"path": path, "action": action},
        }

    out_path = args.get("outPath")
    _require(out_path, f"action {action!r} requires 'outPath' (input is read-only)")
    ids = set(args.get("ids") or [])
    updated = 0
    for annotation in annotations:
        if ids and annotation["id"] not in ids:
            continue
        annotation["status"] = "confirmed" if action == "confirm" else "rejected"
        updated += 1
    out_path = _write_annotations(out_path, annotations)
    return {
        "ok": True,
        "action": action,
        "total": len(annotations),
        "updated": updated,
        "outPath": out_path,
        "inputArgs": {"path": path, "action": action},
    }


# ---------------------------------------------------------------------------
# 9/10. splits and leakage
# ---------------------------------------------------------------------------

def _group_key(row: dict[str, Any], group_columns: list[str]) -> str:
    values = [str(row.get(c)) for c in group_columns]
    if len(values) == 1:
        return values[0]
    return json.dumps(values, ensure_ascii=False)


def cmd_data_split_create(args: dict[str, Any]) -> dict[str, Any]:
    """Split rows (random) or groups (group) into train/val/test without leakage."""
    path = args.get("path")
    _require(path, "missing required argument 'path'")
    method = args.get("method", "random")
    _require(method in ("random", "group"), f"unknown method {method!r}; use random|group")
    group_columns = list(args.get("groupColumns") or [])
    if method == "group":
        _require(group_columns, "group method requires 'groupColumns'")
    ratios = dict(args.get("ratios") or {"train": 0.7, "val": 0.15, "test": 0.15})
    for key in ("train", "val", "test"):
        _require(key in ratios and float(ratios[key]) >= 0, f"ratios must contain non-negative {key!r}")
    total_ratio = sum(float(ratios[k]) for k in ("train", "val", "test"))
    _require(total_ratio <= 1.0 + 1e-9, f"ratios sum to {total_ratio:.3f} (>1.0)")
    seed = int(args.get("seed", 42))

    fmt, columns, rows, _ = _read_table(path)
    rng = random.Random(seed)
    n = len(rows)
    n_train = int(round(n * float(ratios["train"])))
    n_val = int(round(n * float(ratios["val"])))
    n_train = min(n_train, n)
    n_val = min(n_val, n - n_train)

    bucket_rows: dict[str, list[dict[str, Any]]] = {"train": [], "val": [], "test": []}
    bucket_groups: dict[str, list[str]] = {"train": [], "val": [], "test": []}

    if method == "random":
        indices = list(range(n))
        rng.shuffle(indices)
        for bucket, span in (("train", range(n_train)), ("val", range(n_train, n_train + n_val))):
            for i in span:
                bucket_rows[bucket].append(rows[indices[i]])
        for i in range(n_train + n_val, n):
            bucket_rows["test"].append(rows[indices[i]])
    else:
        groups: dict[str, list[int]] = {}
        for i, row in enumerate(rows):
            groups.setdefault(_group_key(row, group_columns), []).append(i)
        group_list = list(groups.items())
        rng.shuffle(group_list)
        acc_train = 0
        acc_val = 0
        for key, indices in group_list:
            if acc_train < n_train:
                bucket = "train"
                acc_train += len(indices)
            elif acc_val < n_val:
                bucket = "val"
                acc_val += len(indices)
            else:
                bucket = "test"
            bucket_groups[bucket].append(key)
            for i in indices:
                bucket_rows[bucket].append(rows[i])

    splits: dict[str, Any] = {}
    for bucket in ("train", "val", "test"):
        entry: dict[str, Any] = {"rows": len(bucket_rows[bucket])}
        if method == "group":
            entry["groups"] = bucket_groups[bucket]
        splits[bucket] = entry

    leak_summary: dict[str, Any] = {"groupsTotal": 0, "leaked": 0, "note": ""}
    if method == "group":
        all_keys: list[str] = []
        for bucket in ("train", "val", "test"):
            all_keys.extend(bucket_groups[bucket])
        seen_in: dict[str, set[str]] = {}
        for bucket in ("train", "val", "test"):
            for key in bucket_groups[bucket]:
                seen_in.setdefault(key, set()).add(bucket)
        leaked = [k for k, buckets in seen_in.items() if len(buckets) > 1]
        leak_summary = {
            "groupsTotal": len(seen_in),
            "leaked": len(leaked),
            "leakedGroups": leaked,
            "note": "group-mode split: groups were assigned whole; same group value never crosses splits" if not leaked else "LEAK: groups found in more than one split",
        }

    out_paths: dict[str, Optional[str]] = {"train": None, "val": None, "test": None}
    out_dir = args.get("outDir")
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        ext = "jsonl" if fmt == "jsonl" else "csv"
        for bucket in ("train", "val", "test"):
            target = os.path.join(out_dir, f"{bucket}.{ext}")
            out_paths[bucket] = _write_table(target, columns if fmt == "csv" else [], bucket_rows[bucket])

    note = (
        f"method={method}; group mode guarantees same {group_columns} value never spans splits "
        f"(leak check: {leak_summary['leaked']} leaked groups). Random mode splits rows only - for timeseries prefer group mode."
    )
    return {
        "ok": True,
        "splits": splits,
        "outPaths": out_paths,
        "note": note,
        "leakSummary": leak_summary,
        "inputArgs": {"path": path, "method": method, "seed": seed},
    }


def _read_split_sets(paths: dict[str, str], group_columns: list[str], time_column: Optional[str]) -> dict[str, Any]:
    per_split_keys: dict[str, set[str]] = {}
    per_split_times: dict[str, list[float]] = {}
    for split_name, split_path in paths.items():
        if not split_path:
            continue
        _, columns, rows, _ = _read_table(split_path)
        keys: set[str] = set()
        times: list[float] = []
        for row in rows:
            if all(row.get(c) is not None and str(row.get(c)).strip() != "" for c in group_columns):
                keys.add(_group_key(row, group_columns))
            if time_column:
                value = _safe_float(row.get(time_column))
                if value is not None and math.isfinite(value):
                    times.append(value)
        per_split_keys[split_name] = keys
        per_split_times[split_name] = times
    return {"keys": per_split_keys, "times": per_split_times}


def cmd_data_leakage_check(args: dict[str, Any]) -> dict[str, Any]:
    """Detect group keys present in more than one split (and frame adjacency)."""
    group_columns = list(args.get("groupColumns") or [])
    _require(group_columns, "missing required argument 'groupColumns'")
    time_column = args.get("timeColumn")

    separate = args.get("trainPath") or args.get("valPath") or args.get("testPath")
    if separate:
        paths = {
            "train": args.get("trainPath"),
            "val": args.get("valPath"),
            "test": args.get("testPath"),
        }
        _require(paths["train"] and paths["val"], "leakage-check with separate files needs trainPath and valPath (testPath optional)")
        data = _read_split_sets(paths, group_columns, time_column)
        per_split_keys = data["keys"]
        per_split_times = data["times"]
    else:
        path = args.get("path")
        split_column = args.get("splitColumn")
        _require(path, "need either path+splitColumn or trainPath/valPath/testPath")
        _require(split_column, "single-file leakage check requires 'splitColumn'")
        _, columns, rows, _ = _read_table(path)
        keys_by_split: dict[str, set[str]] = {}
        times_by_split: dict[str, list[float]] = {}
        for row in rows:
            split_name = str(row.get(split_column))
            if all(row.get(c) is not None and str(row.get(c)).strip() != "" for c in group_columns):
                keys_by_split.setdefault(split_name, set()).add(_group_key(row, group_columns))
            if time_column:
                value = _safe_float(row.get(time_column))
                if value is not None and math.isfinite(value):
                    times_by_split.setdefault(split_name, []).append(value)
        per_split_keys = keys_by_split
        per_split_times = times_by_split

    split_names = sorted(per_split_keys)
    key_splits: dict[str, set[str]] = {}
    for split_name in split_names:
        for key in per_split_keys[split_name]:
            key_splits.setdefault(key, set()).add(split_name)
    leaked = [{"key": key, "splits": sorted(buckets)} for key, buckets in key_splits.items() if len(buckets) > 1]

    overlap_summary: dict[str, Any] = {
        "splits": {name: len(per_split_keys[name]) for name in split_names},
        "totalKeys": len(key_splits),
        "leaked": len(leaked),
    }

    adjacency: list[dict[str, Any]] = []
    if time_column:
        ordered = sorted(per_split_times, key=lambda name: min(per_split_times[name]) if per_split_times[name] else math.inf)
        ordered = [name for name in ordered if per_split_times[name]]
        for prev, nxt in zip(ordered, ordered[1:]):
            gap = min(per_split_times[nxt]) - max(per_split_times[prev])
            if 0.0 <= gap <= 1.0:
                adjacency.append({"between": [prev, nxt], "gapS": round(gap, 6)})
        overlap_summary["adjacency"] = adjacency

    verdict = "leak-detected" if leaked else "ok"
    note = (
        f"checked {len(key_splits)} group keys across {len(split_names)} splits: {len(leaked)} leaked."
        if leaked
        else f"checked {len(key_splits)} group keys across {len(split_names)} splits: no group crosses splits."
    )
    if adjacency:
        note += f" NOTE: {len(adjacency)} frame-adjacency warning(s) at split boundaries (within 1s) - frame-level leakage possible."
    return {
        "ok": True,
        "leakedGroups": leaked,
        "overlapSummary": overlap_summary,
        "verdict": verdict,
        "note": note,
        "inputArgs": {"groupColumns": group_columns, "timeColumn": time_column},
    }


# ---------------------------------------------------------------------------
# 11. data-deidentify
# ---------------------------------------------------------------------------

def _mask_email(value: str) -> str:
    at = value.find("@")
    return ("***" + value[at:]) if at > 0 else "***"


def _mask_phone(value: str) -> str:
    digits = re.sub(r"\D", "", value)
    if len(digits) >= 7:
        return digits[:3] + "****" + digits[-4:]
    return "***" + value[3:]


def _mask_idcard(value: str) -> str:
    return "**********" + value[-4:]


def _pii_patterns() -> list[tuple[str, re.Pattern, Callable[[str], str]]]:
    return [
        ("email", _EMAIL_RE, _mask_email),
        ("phone", _PHONE_RE, _mask_phone),
        ("idcard", _IDCARD_RE, _mask_idcard),
    ]


def _scan_text(text: str) -> list[dict[str, Any]]:
    matches: list[dict[str, Any]] = []
    for pattern_name, pattern, masker in _pii_patterns():
        for match in pattern.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            last_nl = text.rfind("\n", 0, match.start())
            column = match.start() - last_nl
            matches.append(
                {
                    "pattern": pattern_name,
                    "line": line,
                    "column": column,
                    "value": match.group(0),
                    "masked": masker(match.group(0)),
                }
            )
    matches.sort(key=lambda m: (m["line"], m["column"]))
    return matches


def _mask_text(text: str) -> str:
    for _, pattern, masker in _pii_patterns():
        text = pattern.sub(lambda m: masker(m.group(0)), text)
    return text


def _scan_file(path: str) -> tuple[list[dict[str, Any]], str]:
    with open(path, encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    return _scan_text(text), text


def _detect_face_regions(img: Any, model_path: Optional[str] = None) -> tuple[list[tuple[int, int, int, int]], str]:
    """Detect face-like regions. Returns ([(x,y,w,h)], method_name).

    Detection chain, newest OpenCV first:

    1. Haar cascade (``cv2.CascadeClassifier`` + bundled XML) - OpenCV <= 4.x.
    2. YuNet (``cv2.FaceDetectorYN``) with an explicit ``modelPath`` ONNX
       model - OpenCV 5.x dropped the Haar API and ships no cascade files.
    3. Skin-tone blob heuristic (YCrCb) as a documented last resort so the
       command stays usable without any model file.
    """
    if cv2 is not None and hasattr(cv2, "CascadeClassifier"):
        cascade_path = os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
        if os.path.exists(cascade_path):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cascade = cv2.CascadeClassifier(cascade_path)
            if not cascade.empty():
                faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
                return [(int(x), int(y), int(w), int(h)) for (x, y, w, h) in faces], "haar-cascade"
    if cv2 is not None and hasattr(cv2, "FaceDetectorYN") and model_path and os.path.exists(model_path):
        height, width = img.shape[:2]
        detector = cv2.FaceDetectorYN.create(model_path, "", (width, height))
        _, faces = detector.detect(img)
        regions: list[tuple[int, int, int, int]] = []
        if faces is not None:
            for face in faces:
                x, y, w, h = face[:4]
                regions.append((int(x), int(y), int(w), int(h)))
        return regions, "yunet"
    if cv2 is not None:
        img_h, img_w = img.shape[:2]
        img_area = img_h * img_w
        ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
        mask = cv2.inRange(ycrcb, (0, 133, 77), (255, 173, 127))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            bbox_area = w * h
            if w < 24 or h < 24:
                continue
            if not (0.35 <= w / h <= 3.0):
                continue
            if bbox_area < 900 or bbox_area > 0.45 * img_area:
                continue
            coverage = float(cv2.countNonZero(mask[y : y + h, x : x + w])) / bbox_area
            if not (0.15 <= coverage <= 0.95):
                continue
            regions.append((x, y, w, h))
        return regions, "skin-tone-heuristic"
    return [], "unavailable"


def _face_blur_image(image_path: str, out_path: str, model_path: Optional[str] = None) -> dict[str, Any]:
    if cv2 is None:
        raise WorkerError(
            "face-blur requires opencv-python (cv2); install it (e.g. pip install opencv-python-headless) or drop the operation"
        )
    img = cv2.imread(image_path)  # type: ignore[attr-defined]
    if img is None:
        raise WorkerError(f"cannot read image {image_path}")
    regions, method = _detect_face_regions(img, model_path)
    if not regions:
        return {"output": None, "detail": f"no face detected ({method}); image not modified"}
    for (x, y, w, h) in regions:
        roi = img[y : y + h, x : x + w]
        img[y : y + h, x : x + w] = cv2.GaussianBlur(roi, (0, 0), sigmaX=15)  # type: ignore[attr-defined]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, img)  # type: ignore[attr-defined]
    return {"output": out_path, "detail": f"blurred {len(regions)} face region(s) using {method}"}


def _strip_exif_image(image_path: str, out_path: str) -> dict[str, Any]:
    if Image is None:
        raise WorkerError(
            "exif-strip requires Pillow (PIL); install it (e.g. pip install Pillow) or drop the operation"
        )
    with Image.open(image_path) as im:
        im.load()
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        im.save(out_path)  # re-save without EXIF
    return {"output": out_path, "detail": "re-saved without EXIF metadata"}


def _sanitize_filename(name: str) -> str:
    sanitized = name
    for _, pattern, masker in _pii_patterns():
        sanitized = pattern.sub(lambda m: masker(m.group(0)), sanitized)
    return sanitized


_IMAGE_EXTS = {"png", "jpg", "jpeg", "webp", "tif", "tiff", "bmp"}
_TEXT_EXTS = {"txt", "csv", "jsonl", "json", "log", "md", "tsv", "yaml", "yml", "xml"}


def cmd_data_deidentify(args: dict[str, Any]) -> dict[str, Any]:
    """De-identification passes: face-blur, exif-strip, pii-scan, filename-sanitize."""
    input_path = args.get("inputPath")
    _require(input_path, "missing required argument 'inputPath'")
    input_path = _abs(input_path)
    _require(os.path.exists(input_path), f"inputPath not found: {input_path}")
    out_dir = args.get("outDir")
    if out_dir:
        out_dir = _abs(out_dir)
        _assert_outdir_not_c(out_dir)
        os.makedirs(out_dir, exist_ok=True)
    operations = list(args.get("operations") or ["pii-scan"])
    for operation in operations:
        _require(operation in ("face-blur", "exif-strip", "pii-scan", "filename-sanitize"), f"unknown operation {operation!r}")

    if os.path.isdir(input_path):
        targets = sorted(
            os.path.join(input_path, name)
            for name in os.listdir(input_path)
            if os.path.isfile(os.path.join(input_path, name))
        )
    else:
        targets = [input_path]

    processed: list[dict[str, Any]] = []
    for target in targets:
        fmt = _format_of(target)
        base = os.path.basename(target)
        for operation in operations:
            if operation == "face-blur" and fmt in _IMAGE_EXTS:
                _require(out_dir, "face-blur requires 'outDir' to write output images")
                out_path = os.path.join(out_dir, "faceblur_" + base)
                entry = _face_blur_image(target, out_path, model_path=args.get("modelPath"))
                processed.append({"input": target, "output": entry["output"], "action": "face-blur", "detail": entry["detail"]})
            elif operation == "exif-strip" and fmt in _IMAGE_EXTS:
                _require(out_dir, "exif-strip requires 'outDir' to write output images")
                out_path = os.path.join(out_dir, "nostrip_" + base)
                entry = _strip_exif_image(target, out_path)
                processed.append({"input": target, "output": entry["output"], "action": "exif-strip", "detail": entry["detail"]})
            elif operation == "pii-scan" and (fmt in _TEXT_EXTS or fmt in ("", "unknown")):
                matches, text = _scan_file(target)
                entry: dict[str, Any] = {"input": target, "output": None, "action": "pii-scan", "detail": f"found {len(matches)} PII match(es)", "matches": matches}
                if matches and out_dir:
                    sanitized_path = os.path.join(out_dir, "sanitized_" + base)
                    with open(sanitized_path, "w", encoding="utf-8") as handle:
                        handle.write(_mask_text(text))
                    entry["output"] = sanitized_path
                processed.append(entry)
            elif operation == "filename-sanitize":
                sanitized = _sanitize_filename(base)
                if sanitized != base:
                    _require(out_dir, "filename-sanitize requires 'outDir' to write renamed files")
                    out_path = os.path.join(out_dir, sanitized)
                    shutil.copy2(target, out_path)
                    processed.append({"input": target, "output": out_path, "action": "filename-sanitize", "detail": f"renamed {base!r} -> {sanitized!r} (copied to outDir)"})

    # every written output must live under outDir
    if out_dir:
        out_prefix = out_dir + os.sep
        for entry in processed:
            if entry.get("output"):
                _require(
                    os.path.abspath(entry["output"]).startswith(out_prefix),
                    f"internal error: output escaped outDir: {entry['output']}",
                )

    return {
        "ok": True,
        "processed": processed,
        "privacyNotes": list(_PRIVACY_NOTES),
        "inputArgs": {"inputPath": input_path, "outDir": out_dir, "operations": operations},
    }


# ---------------------------------------------------------------------------
# 12. data-convert-rosbag
# ---------------------------------------------------------------------------

def _find_rosbag_db(rosbag_path: str) -> str:
    """Resolve the sqlite db3 file for a rosbag2 bag (file or directory)."""
    if os.path.isfile(rosbag_path):
        return rosbag_path
    candidates = sorted(
        os.path.join(rosbag_path, name)
        for name in os.listdir(rosbag_path)
        if name.endswith(".db3") or name.endswith(".db")
    )
    _require(candidates, f"no .db3 sqlite database found under {rosbag_path}")
    return candidates[0]


def _decode_rosbag_message(msg_type: str, data: bytes) -> tuple[Optional[dict[str, Any]], Optional[str]]:
    """Decode a ROS 2 CDR message. Returns (fields, None) or (None, reason)."""
    if msg_type == "std_msgs/msg/Float64":
        if len(data) != 8:
            return None, f"Float64 data is {len(data)} bytes (expected 8)"
        return {"value": round(struct.unpack("<d", data)[0], 9)}, None
    if msg_type == "std_msgs/msg/String":
        if len(data) < 4:
            return None, "String data too short for CDR length prefix"
        length = struct.unpack("<I", data[:4])[0]
        payload = data[4 : 4 + length]
        return {"data": payload.decode("utf-8", errors="replace")}, None
    return None, f"unsupported message type {msg_type!r} (no CDR decoder; wrote t + dataSize only)"


def _read_rosbag2(path: str, topics_filter: Optional[list[str]]) -> dict[str, Any]:
    """Minimal rosbag2 sqlite reader (topics/messages/schema tables).

    Prefers the ``ros`` worker module when it exists; otherwise falls back to
    this built-in implementation (Float64 / String CDR decoding).
    """
    try:
        from . import ros  # type: ignore[attr-defined]

        if hasattr(ros, "read_rosbag2"):
            return ros.read_rosbag2(path, topics_filter)  # type: ignore[attr-defined]
    except (ImportError, AttributeError):
        pass

    db_path = _find_rosbag_db(path)
    connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        cursor = connection.cursor()
        topics_cols = [row[1] for row in cursor.execute("PRAGMA table_info(topics)").fetchall()]
        _require("id" in topics_cols and "name" in topics_cols, "topics table missing id/name columns")
        type_expr = "type" if "type" in topics_cols else "name"
        topics = cursor.execute(f"SELECT id, name, {type_expr} FROM topics").fetchall()

        msg_cols = [row[1] for row in cursor.execute("PRAGMA table_info(messages)").fetchall()]
        ts_col = "timestamp" if "timestamp" in msg_cols else ("t" if "t" in msg_cols else None)
        _require(ts_col is not None, "messages table has no timestamp/t column")
        _require("data" in msg_cols, "messages table has no data column")

        selected = {t for t in (topics_filter or [])}
        out: dict[str, Any] = {"topics": [], "unsupported": []}
        for topic_id, topic_name, msg_type in topics:
            if selected and topic_name not in selected:
                continue
            rows = cursor.execute(
                f"SELECT {ts_col}, data FROM messages WHERE topic_id=? ORDER BY {ts_col}", (topic_id,)
            ).fetchall()
            decoded_rows: list[dict[str, Any]] = []
            decoded = True
            note = ""
            for raw_ts, data in rows:
                t_seconds = float(raw_ts) / 1e9  # rosbag2 timestamps are nanoseconds
                fields, reason = _decode_rosbag_message(msg_type, bytes(data or b""))
                if fields is None:
                    decoded = False
                    note = reason
                    decoded_rows.append({"t": round(t_seconds, 6), "dataSize": len(data or b"")})
                else:
                    decoded_rows.append({"t": round(t_seconds, 6), **fields})
            out["topics"].append(
                {
                    "name": topic_name,
                    "type": msg_type,
                    "rows": decoded_rows,
                    "decoded": decoded,
                    "note": note,
                }
            )
            if not decoded and msg_type not in out["unsupported"]:
                out["unsupported"].append(msg_type)
        return out
    finally:
        connection.close()


def cmd_data_convert_rosbag(args: dict[str, Any]) -> dict[str, Any]:
    """Convert selected rosbag2 topics to one CSV per topic."""
    rosbag_path = args.get("rosbagPath")
    out_dir = args.get("outDir")
    _require(rosbag_path and out_dir, "missing required arguments 'rosbagPath' and 'outDir'")
    _require(os.path.exists(rosbag_path), f"rosbagPath not found: {rosbag_path}")
    out_dir = _abs(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    time_column = args.get("timeColumn") or "t"
    topics_filter = args.get("topics")

    data = _read_rosbag2(rosbag_path, topics_filter)
    out_files: list[dict[str, Any]] = []
    for topic in data["topics"]:
        safe_name = _slug(topic["name"].lstrip("/").replace("/", "_")) or "topic"
        target = os.path.join(out_dir, f"{safe_name}.csv")
        columns = [time_column]
        if topic["rows"]:
            for key in topic["rows"][0]:
                if key != time_column:
                    columns.append(key)
        _write_table(target, columns, topic["rows"])
        out_files.append(
            {
                "topic": topic["name"],
                "type": topic["type"],
                "path": target,
                "rows": len(topic["rows"]),
                "decoded": topic["decoded"],
                "note": topic["note"] or "decoded OK",
            }
        )
    return {
        "ok": True,
        "outFiles": out_files,
        "outDir": out_dir,
        "unsupportedTypes": data["unsupported"],
        "timeColumn": time_column,
        "note": "no silently dropped topics: any undecodable type is listed in unsupportedTypes and written as t + dataSize",
        "inputArgs": {"rosbagPath": rosbag_path, "outDir": out_dir, "topics": topics_filter},
    }


# ---------------------------------------------------------------------------
# 13/14. dataset exports (LeRobot / RLDS)
# ---------------------------------------------------------------------------

def _expand_q_columns(frame: dict[str, Any]) -> tuple[list[float], list[str]]:
    """Extract joint values from a frame (q list or q0..qN columns)."""
    q = frame.get("q")
    if isinstance(q, (list, tuple)):
        values = [_safe_float(v) for v in q]
        return [v for v in values if v is not None], [f"q{i}" for i in range(len(q))]
    columns = sorted((k for k in frame if re.fullmatch(r"q\d*", k)), key=lambda k: (len(k), k))
    if columns:
        return [_safe_float(frame[k]) for k in columns], columns
    return [], []


def _load_lerobot_frames(run_path: Optional[str], episodes_path: Optional[str]) -> tuple[list[tuple[str, list[dict[str, Any]]]], bool]:
    """Load frames grouped by episode. Returns (episodes, success)."""
    success = False
    if run_path:
        telemetry_path = os.path.join(run_path, "telemetry.jsonl")
        _require(os.path.exists(telemetry_path), f"run telemetry not found: {telemetry_path}")
        _, columns, rows, _ = _read_table(telemetry_path)
        run_json = os.path.join(run_path, "run.json")
        if os.path.exists(run_json):
            with open(run_json, encoding="utf-8") as handle:
                success = bool(json.load(handle).get("metrics", {}).get("success", False))
        frames = list(rows)
        return [("episode-000001", frames)], success
    _require(episodes_path, "need either 'runPath' or 'episodesPath'")
    with open(episodes_path, encoding="utf-8") as handle:
        payload = json.load(handle)
    if isinstance(payload, dict):
        payload = payload.get("episodes", payload)
    _require(isinstance(payload, list), "episodesPath must contain a list of episodes or {'episodes': [...]}")
    episodes: list[tuple[str, list[dict[str, Any]]]] = []
    for index, episode in enumerate(payload):
        _require(isinstance(episode, dict), "each episode must be an object")
        frames = episode.get("frames")
        if frames is None and isinstance(episode.get("rows"), list) and episode["rows"] and isinstance(episode["rows"][0], dict):
            frames = episode["rows"]
        if frames is None and episode.get("sourcePath"):
            _, _, frames, _ = _read_table(episode["sourcePath"])
        _require(isinstance(frames, list) and frames and all(isinstance(f, dict) for f in frames), f"episode {index}: missing non-empty 'frames' list")
        episode_id = str(episode.get("id") or f"episode-{index + 1:06d}")
        episodes.append((episode_id, frames))
    return episodes, success


def cmd_data_export_lerobot(args: dict[str, Any]) -> dict[str, Any]:
    """Export episode data as a LeRobotDataset-style directory."""
    run_path = args.get("runPath")
    episodes_path = args.get("episodesPath")
    out_dir = args.get("outDir")
    _require(out_dir, "missing required argument 'outDir'")
    out_dir = _abs(out_dir)
    robot_name = args.get("robotName") or "rh_demo"
    task = args.get("task") or "pick_place"

    episodes, success = _load_lerobot_frames(run_path, episodes_path)
    _require(episodes, "no episodes to export")

    # flatten frames with q columns + action (= q) + success
    total_frames = 0
    all_frames: list[list[dict[str, Any]]] = []
    for episode_id, frames in episodes:
        expanded: list[dict[str, Any]] = []
        for frame in frames:
            q_values, q_columns = _expand_q_columns(frame)
            row: dict[str, Any] = {"timestamp": _r(_safe_float(frame.get("t")) or 0.0, 9)}
            for i, name in enumerate(q_columns):
                row[f"q{i}"] = q_values[i] if i < len(q_values) else None
                row[f"action{i}"] = q_values[i] if i < len(q_values) else None
            row["success"] = bool(success)
            expanded.append(row)
        all_frames.append(expanded)
        total_frames += len(expanded)

    meta_dir = os.path.join(out_dir, "meta")
    chunk_dir = os.path.join(out_dir, "data", "chunk-000")
    os.makedirs(meta_dir, exist_ok=True)
    os.makedirs(chunk_dir, exist_ok=True)

    # parquet requires pyarrow; degrade to CSV otherwise
    format_name = "lerobot-v2"
    notes: list[str] = []
    try:
        import pyarrow as pa  # noqa: PLC0415
        import pyarrow.parquet as pq  # noqa: PLC0415

        parquet_available = True
    except ImportError:
        parquet_available = False

    for index, frames in enumerate(all_frames):
        ep_index = index + 1
        base = f"episode_{ep_index:06d}"
        joint_count = sum(1 for k in frames[0] if re.fullmatch(r"q\d*", k)) if frames else 0
        frame_columns = (
            ["timestamp"]
            + [f"q{i}" for i in range(joint_count)]
            + [f"action{i}" for i in range(joint_count)]
            + ["success"]
        )
        if parquet_available:
            import pyarrow as pa  # noqa: PLC0415
            import pyarrow.parquet as pq  # noqa: PLC0415

            arrays = {c: [f.get(c) for f in frames] for c in frame_columns}
            table = pa.table(arrays)
            pq.write_table(table, os.path.join(chunk_dir, base + ".parquet"))
        else:
            _write_table(os.path.join(chunk_dir, base + ".csv"), frame_columns, frames)
            if format_name == "lerobot-v2":
                format_name = "lerobot-csv-fallback"
                notes.append("parquet 需要 pyarrow，已降级 CSV")
        with open(os.path.join(chunk_dir, base + ".json"), "w", encoding="utf-8") as handle:
            json.dump(
                {
                    "episode_index": index,
                    "length": len(frames),
                    "task": task,
                    "robot_type": robot_name,
                },
                handle,
                ensure_ascii=False,
                indent=2,
            )

    info = {
        "format_version": "1.5",
        "codebase_version": "robotic-harness-dsh",
        "robot_type": robot_name,
        "task": task,
        "total_episodes": len(all_frames),
        "total_frames": total_frames,
        "total_videos": 0,
        "total_chunks": 1,
        "chunks_size": 1,
        "fps": None,
        "splits": {"train": [i for i in range(len(all_frames))]},
        "data_path": "data",
        "video": {"available": False, "note": "无视频流：本数据集未包含摄像头图像；video 字段缺省"},
        "features": {
            "observation.state": {"dtype": "float32", "shape": [joint_count]},
            "action": {"dtype": "float32", "shape": [joint_count]},
            "success": {"dtype": "bool", "shape": []},
        },
        "notes": notes,
    }
    with open(os.path.join(meta_dir, "info.json"), "w", encoding="utf-8") as handle:
        json.dump(info, handle, ensure_ascii=False, indent=2)

    return {
        "ok": True,
        "outDir": out_dir,
        "format": format_name,
        "episodes": len(all_frames),
        "frames": total_frames,
        "notes": notes,
        "inputArgs": {"runPath": run_path, "episodesPath": episodes_path, "outDir": out_dir},
    }


def cmd_data_export_rlds(args: dict[str, Any]) -> dict[str, Any]:
    """Write the RLDS (TFDS) manifest template and directory skeleton."""
    out_dir = args.get("outDir")
    _require(out_dir, "missing required argument 'outDir'")
    out_dir = _abs(out_dir)
    os.makedirs(os.path.join(out_dir, "data"), exist_ok=True)

    features = {
        "features": {
            "observation": {"q": {"dtype": "float32", "shape": [None]}},
            "action": {"dtype": "float32", "shape": [None]},
            "reward": {"dtype": "float32", "shape": []},
            "is_terminal": {"dtype": "bool", "shape": []},
        }
    }
    features_path = os.path.join(out_dir, "features.json")
    with open(features_path, "w", encoding="utf-8") as handle:
        json.dump(features, handle, ensure_ascii=False, indent=2)

    notes = [
        "RLDS(TFDS) 完整导出需要 tensorflow 与 tensorflow_datasets；当前提供 manifest 模板与目录骨架（features 声明 JSON），供后续接入",
        "接入步骤：安装 tensorflow + tensorflow_datasets 后，用 features.json 实现 DatasetBuilder（_info 的 features 与 _generate_examples），"
        "再把逐帧数据（q/action/reward/is_terminal）写入 TFRecord",
    ]
    manifest = {
        "format": "rlds-manifest",
        "datasetName": os.path.basename(out_dir),
        "schemaVersion": 1,
        "featuresPath": "features.json",
        "requires": ["tensorflow", "tensorflow_datasets"],
        "notes": notes,
    }
    manifest_path = os.path.join(out_dir, "manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)

    return {
        "ok": True,
        "outDir": out_dir,
        "format": "rlds-manifest",
        "notes": notes,
        "manifestPath": manifest_path,
        "inputArgs": {"outDir": out_dir},
    }


# ---------------------------------------------------------------------------
# 15/16/17. versioning, comparison, data cards
# ---------------------------------------------------------------------------

def _copy_into(source: str, data_dir: str) -> tuple[int, int]:
    """Copy a file or a tree into data_dir. Returns (files, total_bytes)."""
    files = 0
    total_bytes = 0
    if os.path.isfile(source):
        target = os.path.join(data_dir, os.path.basename(source))
        os.makedirs(os.path.dirname(target), exist_ok=True)
        _copy_file(source, target)
        files += 1
        total_bytes += os.path.getsize(target)
        return files, total_bytes
    for dirpath, _, filenames in os.walk(source):
        for name in sorted(filenames):
            src = os.path.join(dirpath, name)
            rel = os.path.relpath(src, source)
            target = os.path.join(data_dir, rel)
            os.makedirs(os.path.dirname(target), exist_ok=True)
            _copy_file(src, target)
            files += 1
            total_bytes += os.path.getsize(target)
    return files, total_bytes


def _copy_file(source: str, target: str) -> None:
    # Plain copy (never hardlink): a hardlink aliases the source inode, so a
    # later edit of the source would silently corrupt the "frozen" version.
    # Immutable versioning wins over the space saving of hardlinks.
    shutil.copy2(source, target)


def _dir_content_hash(data_dir: str) -> str:
    """Directory-level content hash: sorted (relpath, sha256) then hashed."""
    entries: list[tuple[str, str]] = []
    for dirpath, _, filenames in os.walk(data_dir):
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, data_dir).replace(os.sep, "/")
            entries.append((rel, sha256_file(full)))
    entries.sort()
    digest = "\n".join(f"{rel}:{h}" for rel, h in entries)
    return sha256_of_bytes(digest.encode("utf-8"))


def sha256_of_bytes(payload: bytes) -> str:
    import hashlib

    return hashlib.sha256(payload).hexdigest()


def _load_manifest(dataset: str) -> tuple[dict[str, Any], str]:
    if os.path.isdir(dataset):
        manifest_path = os.path.join(dataset, "manifest.json")
    else:
        manifest_path = dataset
    _require(os.path.exists(manifest_path), f"manifest not found: {manifest_path}")
    with open(manifest_path, encoding="utf-8") as handle:
        manifest = json.load(handle)
    return manifest, os.path.dirname(manifest_path)


def cmd_dataset_version_create(args: dict[str, Any]) -> dict[str, Any]:
    """Immutable dataset version: copy sources, hash content, write manifest."""
    name = args.get("name")
    source_paths = args.get("sourcePaths") or []
    out_dir = args.get("outDir")
    _require(name, "missing required argument 'name'")
    _require(isinstance(source_paths, list) and source_paths, "sourcePaths must be a non-empty list")
    _require(out_dir, "missing required argument 'outDir'")
    out_dir = _abs(out_dir)
    for source in source_paths:
        _require(os.path.exists(source), f"source path not found: {source}")
    version = str(args.get("version") or "0.1.0")

    data_dir = os.path.join(out_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    files, total_bytes = 0, 0
    for source in source_paths:
        n, size = _copy_into(source, data_dir)
        files += n
        total_bytes += size
    _require(files > 0, "no files copied from sourcePaths")

    content_hash = _dir_content_hash(data_dir)
    manifest = {
        "schemaVersion": 1,
        "name": name,
        "version": version,
        "createdAt": time.time(),
        "description": args.get("description"),
        "sourcePaths": [_abs(s) for s in source_paths],
        "contentHash": content_hash,
        "transforms": args.get("transforms"),
        "split": args.get("split"),
        "stats": {"files": files, "totalBytes": total_bytes},
        "parent": args.get("parent"),
    }
    manifest_path = os.path.join(out_dir, "manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)

    return {
        "ok": True,
        "datasetId": f"{_slug(name)}-{version}",
        "version": version,
        "outDir": out_dir,
        "contentHash": content_hash,
        "manifestPath": manifest_path,
        "inputArgs": {"name": name, "outDir": out_dir, "version": version},
    }


def cmd_dataset_compare(args: dict[str, Any]) -> dict[str, Any]:
    """Compare two dataset versions by manifest + per-file hashes."""
    dataset_a = args.get("datasetA")
    dataset_b = args.get("datasetB")
    _require(dataset_a and dataset_b, "missing required arguments 'datasetA' and 'datasetB'")
    manifest_a, root_a = _load_manifest(dataset_a)
    manifest_b, root_b = _load_manifest(dataset_b)
    hash_a = manifest_a.get("contentHash")
    hash_b = manifest_b.get("contentHash")

    def index_data(root: str) -> dict[str, str]:
        data_dir = os.path.join(root, "data")
        found: dict[str, str] = {}
        if os.path.isdir(data_dir):
            for dirpath, _, filenames in os.walk(data_dir):
                for name in filenames:
                    full = os.path.join(dirpath, name)
                    rel = os.path.relpath(full, data_dir).replace(os.sep, "/")
                    found[rel] = full
        return found

    files_a = index_data(root_a)
    files_b = index_data(root_b)
    names = sorted(set(files_a) | set(files_b))
    diffs: list[dict[str, Any]] = []
    hash_equal_all = True
    for name in names:
        entry: dict[str, Any] = {"name": name, "sizeA": None, "sizeB": None, "hashEqual": None}
        if name in files_a:
            entry["sizeA"] = os.path.getsize(files_a[name])
        if name in files_b:
            entry["sizeB"] = os.path.getsize(files_b[name])
        if name in files_a and name in files_b:
            equal = sha256_file(files_a[name]) == sha256_file(files_b[name])
            entry["hashEqual"] = equal
            hash_equal_all = hash_equal_all and equal
        diffs.append(entry)

    same_content = bool(hash_a and hash_b and hash_a == hash_b and hash_equal_all)
    summary = (
        f"datasets are content-identical (hash {hash_a[:12]}...)" if same_content
        else f"datasets differ: hashA={hash_a[:12] if hash_a else None}... hashB={hash_b[:12] if hash_b else None}..."
    )
    return {
        "ok": True,
        "sameContent": same_content,
        "hashA": hash_a,
        "hashB": hash_b,
        "differences": {"files": diffs, "samples": None, "schema": None},
        "summary": summary,
        "inputArgs": {"datasetA": dataset_a, "datasetB": dataset_b},
    }


def _card_stats(dataset_dir: str) -> dict[str, Any]:
    data_dir = os.path.join(dataset_dir, "data")
    stats: dict[str, Any] = {"files": 0, "totalBytes": 0, "tables": 0, "rows": 0, "columns": [], "timeRange": None}
    if not os.path.isdir(data_dir):
        return stats
    for dirpath, _, filenames in os.walk(data_dir):
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            stats["files"] += 1
            stats["totalBytes"] += os.path.getsize(full)
            fmt = _format_of(full)
            if fmt in ("csv", "jsonl"):
                try:
                    _, columns, rows, _ = _read_table(full)
                    stats["tables"] += 1
                    stats["rows"] += len(rows)
                    for column in columns:
                        if column not in stats["columns"]:
                            stats["columns"].append(column)
                    if "t" in columns:
                        time_range = _time_range(rows, "t")
                        if time_range:
                            current = stats["timeRange"]
                            if current is None:
                                stats["timeRange"] = time_range
                            else:
                                stats["timeRange"] = {"min": min(current["min"], time_range["min"]), "max": max(current["max"], time_range["max"])}
                except WorkerError:
                    pass
    return stats


def cmd_dataset_card_generate(args: dict[str, Any]) -> dict[str, Any]:
    """Generate a GitHub-style Markdown data card from a dataset manifest."""
    dataset_path = args.get("datasetPath")
    _require(dataset_path, "missing required argument 'datasetPath'")
    manifest, root = _load_manifest(dataset_path)
    stats = _card_stats(root)

    lines: list[str] = []
    lines.append(f"# Data Card: {manifest.get('name', os.path.basename(root))}")
    lines.append("")
    lines.append(f"> datasetId `{_slug(manifest.get('name', 'dataset'))}-{manifest.get('version', '')}` · schemaVersion {manifest.get('schemaVersion', 1)} · contentHash `{(manifest.get('contentHash') or '')[:16]}…`")
    lines.append("")
    lines.append("## 用途 (Purpose)")
    lines.append("")
    lines.append(f"{manifest.get('description') or '_待填写：数据集用途、采集场景、下游任务说明。_'}")
    lines.append("")
    lines.append("## Schema")
    lines.append("")
    if stats["columns"]:
        lines.append("| 列名 | 说明 |")
        lines.append("|---|---|")
        for column in stats["columns"]:
            lines.append(f"| `{column}` | _待填写_ |")
    else:
        lines.append("_无表格数据（仅文件资产）。_")
    lines.append("")
    lines.append("## 统计 (Statistics)")
    lines.append("")
    lines.append(f"- 文件数: {stats['files']}")
    lines.append(f"- 总字节: {stats['totalBytes']}")
    lines.append(f"- 表格数 (csv/jsonl): {stats['tables']}")
    lines.append(f"- 总行数: {stats['rows']}")
    if stats["timeRange"]:
        lines.append(f"- 时间范围 (t): [{stats['timeRange']['min']}, {stats['timeRange']['max']}] s")
    lines.append("- 缺失率: _待填写_")
    lines.append("")
    lines.append("## 来源与转换 DAG (Provenance)")
    lines.append("")
    lines.append("| 步骤 | 说明 |")
    lines.append("|---|---|")
    for index, source in enumerate(manifest.get("sourcePaths") or []):
        lines.append(f"| source-{index} | `{source}` |")
    for index, transform in enumerate(manifest.get("transforms") or []):
        lines.append(f"| transform-{index} | `{transform.get('kind')}` params=`{json.dumps(transform.get('params') or {}, ensure_ascii=False)}` |")
    if manifest.get("split"):
        lines.append(f"| split | `{json.dumps(manifest['split'], ensure_ascii=False)}` |")
    lines.append("")
    lines.append("## 许可与使用限制 (License & Restrictions)")
    lines.append("")
    lines.append("_待人工填写：许可证、数据来源授权、用途限制、再分发条款。_")
    lines.append("")
    lines.append("## 偏差声明 (Bias Statement)")
    lines.append("")
    lines.append("_待填写模板：采集环境分布（仿真/真实）、传感器与光照条件、场景与物体分布、已知偏差与限制。_")
    lines.append("")
    card_text = "\n".join(lines)

    out_path = args.get("outPath")
    if not out_path:
        out_path = os.path.join(root, "DATASET_CARD.md")
    out_path = _abs(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(card_text)
    return {
        "ok": True,
        "path": out_path,
        "inputArgs": {"datasetPath": dataset_path, "outPath": out_path},
    }


# ---------------------------------------------------------------------------
# registry
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
    "data-inventory": cmd_data_inventory,
    "data-schema-inspect": cmd_data_schema_inspect,
    "data-time-sync-estimate": cmd_data_time_sync_estimate,
    "data-align-streams": cmd_data_align_streams,
    "data-transform-apply": cmd_data_transform_apply,
    "data-segment-episodes": cmd_data_segment_episodes,
    "data-annotation-import": cmd_data_annotation_import,
    "data-annotation-review": cmd_data_annotation_review,
    "data-split-create": cmd_data_split_create,
    "data-leakage-check": cmd_data_leakage_check,
    "data-deidentify": cmd_data_deidentify,
    "data-convert-rosbag": cmd_data_convert_rosbag,
    "data-export-lerobot": cmd_data_export_lerobot,
    "data-export-rlds": cmd_data_export_rlds,
    "dataset-version-create": cmd_dataset_version_create,
    "dataset-compare": cmd_dataset_compare,
    "dataset-card-generate": cmd_dataset_card_generate,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "data.inventory",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "recursive": "boolean?"},
        "output": "file inventory with sha256 and format counts",
        "risk": "R0-readonly",
        "description": "Scan a directory/file: size, sha256, format, integrity issues.",
    },
    {
        "id": "data.schema_inspect",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "timeColumn": "string?"},
        "output": "column dtype / missing / sample report",
        "risk": "R0-readonly",
        "description": "Infer column dtypes (number/string/boolean/mixed/missing) for CSV/JSONL.",
    },
    {
        "id": "data.time_sync_estimate",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"pathA": "string", "pathB": "string", "signalColumns": "object?"},
        "output": "fixed time offset estimate",
        "risk": "R0-readonly",
        "description": "Cross-correlation or mean-difference estimate of the fixed delay between two streams.",
    },
    {
        "id": "data.align_streams",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"primary": "string", "files": "array?", "strategy": "string"},
        "output": "aligned rows on the primary time axis",
        "risk": "R1-derive",
        "description": "Nearest/exact/window alignment of multiple streams; never interpolates categorical columns.",
    },
    {
        "id": "data.transform_apply",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"inputPath": "string", "operations": "array", "outPath": "string"},
        "output": "non-destructive transform chain result",
        "risk": "R1-derive",
        "description": "Chain of 10 transforms (filter/dedupe/sort/interpolate/lowpass/median/resample/detrend/unit/round).",
    },
    {
        "id": "data.segment_episodes",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "maxGapS": "number?"},
        "output": "episode segmentation",
        "risk": "R1-derive",
        "description": "Split a timeseries into episodes by time gaps.",
    },
    {
        "id": "data.annotation_io",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "action": "string?"},
        "output": "imported / reviewed annotations",
        "risk": "R1-derive",
        "description": "Import annotations with column-name compatibility and review them without in-place edits.",
    },
    {
        "id": "data.split_create",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "method": "string", "groupColumns": "array?"},
        "output": "train/val/test splits + leak summary",
        "risk": "R1-derive",
        "description": "Leakage-safe splits: group mode never splits a group across splits.",
    },
    {
        "id": "data.leakage_check",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"trainPath": "string?", "valPath": "string?", "groupColumns": "array"},
        "output": "leak verdict and leaked groups",
        "risk": "R0-readonly",
        "description": "Detect group keys across splits and frame adjacency at split boundaries.",
    },
    {
        "id": "data.deidentify",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"inputPath": "string", "operations": "array?", "outDir": "string?"},
        "output": "de-identification pass results",
        "risk": "R2-simulation",
        "description": "face-blur / exif-strip / pii-scan / filename-sanitize; local processing; de-identification != anonymization.",
    },
    {
        "id": "data.convert_rosbag",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"rosbagPath": "string", "outDir": "string", "topics": "array?"},
        "output": "one CSV per topic",
        "risk": "R1-derive",
        "description": "Convert rosbag2 sqlite topics to CSV; undecodable types are listed, never silently dropped.",
    },
    {
        "id": "data.export_lerobot",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"runPath": "string?", "episodesPath": "string?", "outDir": "string"},
        "output": "LeRobotDataset-style directory",
        "risk": "R1-derive",
        "description": "Export episodes as LeRobot parquet (or CSV fallback) + info.json.",
    },
    {
        "id": "data.export_rlds",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"outDir": "string"},
        "output": "RLDS manifest template",
        "risk": "R1-derive",
        "description": "RLDS/TFDS manifest + features skeleton; full export needs tensorflow.",
    },
    {
        "id": "dataset.versioning",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"name": "string", "sourcePaths": "array", "outDir": "string"},
        "output": "immutable dataset version + manifest",
        "risk": "R1-derive",
        "description": "Immutable versioning with content hashing; compare versions; generate data cards.",
    },
]
