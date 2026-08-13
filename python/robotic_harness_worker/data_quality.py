"""Data quality audit for CSV and JSONL timeseries (read-only, non-destructive).

Implements the plan's Data v0.1 checks: missing/NaN/Inf values, duplicate and
out-of-order timestamps, interval anomalies, constant channels, and basic
per-channel statistics. The audit never modifies the input file; results are
returned as structured JSON.
"""

from __future__ import annotations

import csv
import json
import math
import os
from typing import Any, Optional


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


def audit_csv(path: str, time_column: str = "t", max_rows: int = 200_000) -> dict[str, Any]:
    """Audit a CSV file with a numeric time column."""
    rows_read = 0
    rows: list[dict[str, Any]] = []
    header: list[str] = []
    parse_errors = 0
    with open(path, encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration:
            return _empty_report("csv", path, "file has no header row")
        if not header:
            return _empty_report("csv", path, "file has no header row")
        if time_column not in header:
            return _empty_report("csv", path, f"time column {time_column!r} not found in header {header}")
        for line in reader:
            if len(line) != len(header):
                parse_errors += 1
                continue
            rows.append(dict(zip(header, line)))
            rows_read += 1
            if rows_read >= max_rows:
                break
    return _audit_rows("csv", path, header, rows, time_column, parse_errors)


def audit_jsonl(path: str, time_column: str = "t", max_rows: int = 200_000) -> dict[str, Any]:
    """Audit a JSONL file (each line a JSON object)."""
    rows: list[dict[str, Any]] = []
    parse_errors = 0
    header: list[str] = []
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
                if key not in header:
                    header.append(key)
            rows.append(record)
            if len(rows) >= max_rows:
                break
    if time_column not in header:
        return _empty_report("jsonl", path, f"time column {time_column!r} not found in records")
    return _audit_rows("jsonl", path, header, rows, time_column, parse_errors)


def audit(path: str, format: Optional[str] = None, time_column: str = "t") -> dict[str, Any]:
    """Dispatch by extension or explicit format."""
    fmt = format or os.path.splitext(path)[1].lower().lstrip(".") or ""
    if fmt in ("csv", "tsv"):
        return audit_csv(path, time_column)
    if fmt in ("jsonl", "ndjson", "json"):
        return audit_jsonl(path, time_column)
    raise ValueError(f"unsupported data format {fmt!r}; supported: csv, jsonl (pass format explicitly for odd extensions)")


# ---------------------------------------------------------------------------

def _audit_rows(fmt: str, path: str, header: list[str], rows: list[dict[str, Any]], time_column: str, parse_errors: int) -> dict[str, Any]:
    numeric_columns = [c for c in header if c != time_column]
    timestamps: list[float] = []
    ts_missing = 0
    for row in rows:
        value = _safe_float(row.get(time_column))
        if value is None:
            ts_missing += 1
            continue
        timestamps.append(value)

    # Ordering problems are counted on the RAW sequence (file order); gaps on
    # the sorted sequence.
    duplicates = 0
    out_of_order = 0
    for index in range(1, len(timestamps)):
        delta = timestamps[index] - timestamps[index - 1]
        if delta < 0:
            out_of_order += 1
        elif delta == 0:
            duplicates += 1
    ordered = sorted(timestamps)
    gaps: list[float] = []
    for index in range(1, len(ordered)):
        delta = ordered[index] - ordered[index - 1]
        if delta > 0:
            gaps.append(delta)

    channel_stats: dict[str, Any] = {}
    for column in numeric_columns:
        values: list[float] = []
        missing = 0
        non_finite = 0
        for row in rows:
            value = _safe_float(row.get(column))
            if value is None:
                missing += 1
                continue
            if not math.isfinite(value):
                non_finite += 1
                continue
            values.append(value)
        stats: dict[str, Any] = {
            "missing": missing,
            "nonFinite": non_finite,
            "count": len(values),
            "min": round(min(values), 6) if values else None,
            "max": round(max(values), 6) if values else None,
            "mean": round(sum(values) / len(values), 6) if values else None,
            "constant": len(set(round(v, 6) for v in values)) <= 1 if values else None,
        }
        if len(values) > 1:
            variance = sum((v - stats["mean"]) ** 2 for v in values) / (len(values) - 1)
            stats["std"] = round(math.sqrt(variance), 6)
        channel_stats[column] = stats

    issues: list[dict[str, Any]] = []
    if ts_missing:
        issues.append({"severity": "warning", "code": "ts.missing", "message": f"{ts_missing} rows lack a numeric {time_column!r}"})
    if duplicates:
        issues.append({"severity": "warning", "code": "ts.duplicates", "message": f"{duplicates} duplicate timestamps"})
    if out_of_order:
        issues.append({"severity": "error", "code": "ts.out_of_order", "message": f"{out_of_order} timestamp pairs are out of order"})
    if gaps:
        median_gap = sorted(gaps)[len(gaps) // 2]
        largest = max(gaps)
        if largest > median_gap * 5 + 0.1:
            issues.append(
                {
                    "severity": "warning",
                    "code": "ts.gap",
                    "message": f"largest interval {largest:.4f}s is >5x the median {median_gap:.4f}s (frame drop?)",
                }
            )
        for column, stats in channel_stats.items():
            if stats.get("constant"):
                issues.append({"severity": "info", "code": "channel.constant", "message": f"channel {column!r} is constant"})
    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "format": fmt,
        "path": path,
        "rows": len(rows),
        "parseErrors": parse_errors,
        "timeColumn": time_column,
        "timestamps": {
            "first": timestamps[0] if timestamps else None,
            "last": timestamps[-1] if timestamps else None,
            "count": len(timestamps),
            "missing": ts_missing,
            "duplicates": duplicates,
            "outOfOrder": out_of_order,
            "medianIntervalS": round(sorted(gaps)[len(gaps) // 2], 6) if gaps else None,
        },
        "channels": channel_stats,
        "issues": issues,
    }


def _empty_report(fmt: str, path: str, reason: str) -> dict[str, Any]:
    return {
        "ok": False,
        "format": fmt,
        "path": path,
        "rows": 0,
        "parseErrors": 0,
        "timeColumn": "",
        "timestamps": {},
        "channels": {},
        "issues": [{"severity": "error", "code": "format.invalid", "message": reason}],
    }
