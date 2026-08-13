---
name: rh-audit-dataset-quality
description: Audit a CSV/JSONL timeseries for data quality — missing/NaN/Inf, duplicate or out-of-order timestamps, interval gaps, constant channels — and produce a structured quality report.
whenToUse: Use when the user shares a CSV/JSONL of telemetry, joint states, or any timeseries and wants to know whether it is usable before analysis or training.
modelInvocable: true
userInvocable: true
---

# Audit dataset quality

1. **Resolve the file.** Confirm the absolute path and format (csv or jsonl; pass `format` explicitly for unusual extensions).
2. **Audit.** `rh_data_quality` with path, optional format and timeColumn (default `t`).
3. **Interpret.** Present per-channel statistics (missing, nonFinite, min/max/mean/std, constant) and timestamp findings (duplicates, out-of-order, largest gap vs median). Map findings to consequences: e.g. gaps → frame drops; constant channel → dead sensor or unplugged topic.
4. **Recommend.** Suggest next steps (re-collect, interpolate with care, exclude channel) — never modify the file; the audit is read-only.

## Rules

- Missing values and interpolation must be disclosed, not hidden.
- If the file has no timestamp column, report the error and ask for the right column name.
