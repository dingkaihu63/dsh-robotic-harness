"""Reproducible experiment management for the Robotic Harness worker.

Implements the research-experiment workflow of the plan (chapter 15). An
ExperimentSpec is a reproducible experiment definition — research question,
hypothesis, independent/control variables, baselines, metrics, seed,
repetitions and statistical method — persisted under
``<storeRoot>/.rh/experiments/<id>.json``.

The benchmark executes the variable matrix through
:mod:`robotic_harness_worker.simulation`; metrics aggregation and ablation
comparison are pure functions over run records (no I/O), and every report
carries an explicit "requires human review" declaration. These tools
aggregate simulation results; they never certify them.

Command surface (see :data:`COMMANDS`):

- ``experiment-spec-create``   define and persist an ExperimentSpec
- ``experiment-matrix-expand`` cartesian product x repetitions with seeds
- ``benchmark-start``          run the matrix through pick-place simulation
- ``metrics-compute``          aggregate metrics (pure numpy) over runs
- ``ablation-compare``         group runs by one variable vs its baseline
- ``benchmark-report``         render a Markdown experiment report

Every command accepts either ``{"experimentId": ...}`` (load the persisted
spec) or ``{"spec": {...}}`` (use the spec directly, no disk write).
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Optional

import numpy as np

from .core import RunStore, WorkerError, new_id, snapshot_environment
from .simulation import SCENARIO_PICK_PLACE, load_scenario, run_pick_place

DEFAULT_SEED = 42
DEFAULT_REPETITIONS = 3
DEFAULT_PRIMARY_METRIC = "success_rate"
SPEC_VERSION = 1

# Placeholders substituted by a cell's variable value inside fault templates.
VALUE_MARKERS = ("__VALUE__", "$value")


# ---------------------------------------------------------------------------
# spec persistence
# ---------------------------------------------------------------------------


def _store_root(args: dict[str, Any]) -> str:
    from .core import normalize_store_root

    return normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))


def _experiments_dir(store_root: str) -> str:
    # storeRoot is the RunStore root (the .rh directory); experiments live
    # directly under it.
    return os.path.join(store_root, "experiments")


def _spec_path(store_root: str, experiment_id: str) -> str:
    return os.path.join(_experiments_dir(store_root), f"{experiment_id}.json")


def normalize_spec(raw: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalize a raw spec dict into the canonical form.

    Raises :class:`WorkerError` for missing or invalid required fields.
    """
    if not isinstance(raw, dict):
        raise WorkerError("spec must be a JSON object")
    name = raw.get("name")
    if not isinstance(name, str) or not name.strip():
        raise WorkerError("experiment spec requires a non-empty 'name'")

    independent = raw.get("independentVariables")
    if not isinstance(independent, list) or not independent:
        raise WorkerError(
            "experiment spec requires at least one entry in 'independentVariables' "
            "(each entry: {name, values})"
        )
    normalized_independent: list[dict[str, Any]] = []
    for var in independent:
        if not isinstance(var, dict):
            raise WorkerError("each independentVariables entry must be an object {name, values}")
        var_name = var.get("name")
        values = var.get("values")
        if not isinstance(var_name, str) or not var_name.strip():
            raise WorkerError("each independentVariables entry requires a non-empty 'name'")
        if not isinstance(values, list) or not values:
            raise WorkerError(f"independent variable {var_name!r} requires a non-empty 'values' list")
        normalized_independent.append({"name": var_name, "values": list(values)})

    repetitions = raw.get("repetitions", DEFAULT_REPETITIONS)
    if isinstance(repetitions, bool) or not isinstance(repetitions, int) or repetitions < 1:
        raise WorkerError("'repetitions' must be an integer >= 1")

    seed = raw.get("seed", DEFAULT_SEED)
    if isinstance(seed, bool) or not isinstance(seed, (int, float)):
        raise WorkerError("'seed' must be an integer")
    if isinstance(seed, float) and not seed.is_integer():
        raise WorkerError("'seed' must be an integer")
    seed = int(seed)

    for key in ("controlVariables", "baselines", "metrics"):
        value = raw.get(key)
        if value is not None and not isinstance(value, list):
            raise WorkerError(f"'{key}' must be a list")
    termination = raw.get("termination")
    if termination is not None and not isinstance(termination, dict):
        raise WorkerError("'termination' must be an object")

    return {
        "version": SPEC_VERSION,
        "name": name.strip(),
        "researchQuestion": raw.get("researchQuestion"),
        "hypothesis": raw.get("hypothesis"),
        "primaryMetric": raw.get("primaryMetric") or DEFAULT_PRIMARY_METRIC,
        "independentVariables": normalized_independent,
        "controlVariables": raw.get("controlVariables") or [],
        "baselines": raw.get("baselines") or [],
        "metrics": raw.get("metrics") or [],
        "seed": seed,
        "repetitions": repetitions,
        "termination": termination or {},
        "artifactPolicy": raw.get("artifactPolicy"),
        "requiresApproval": bool(raw.get("requiresApproval", False)),
        "statisticalMethod": raw.get("statisticalMethod"),
    }


def open_questions(spec: dict[str, Any]) -> list[str]:
    """Agent-facing checklist: which study-design elements are still missing.

    Missing items are returned so the calling agent can either fill them in
    or record them as open questions before running the benchmark.
    """
    missing: list[str] = []
    if not spec.get("researchQuestion"):
        missing.append("研究问题未明确（researchQuestion）")
    if not spec.get("baselines"):
        missing.append("基线条件未定义（baselines）")
    if not spec.get("controlVariables"):
        missing.append("对照组/控制变量未定义（controlVariables）")
    if not spec.get("statisticalMethod"):
        missing.append("统计方法未声明（statisticalMethod）")
    return missing


def save_spec(store_root: str, experiment_id: str, spec: dict[str, Any]) -> str:
    """Persist a spec under ``<storeRoot>/.rh/experiments/<id>.json``."""
    experiments_dir = _experiments_dir(store_root)
    os.makedirs(experiments_dir, exist_ok=True)
    path = _spec_path(store_root, experiment_id)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(spec, handle, ensure_ascii=False, indent=2)
    return os.path.abspath(path)


def load_spec(store_root: str, experiment_id: str) -> tuple[dict[str, Any], str]:
    """Load a persisted spec; returns ``(spec, absolute_path)``."""
    path = _spec_path(store_root, experiment_id)
    if not os.path.exists(path):
        raise WorkerError(f"experiment {experiment_id!r} not found at {path}")
    with open(path, encoding="utf-8") as handle:
        try:
            spec = json.load(handle)
        except json.JSONDecodeError as error:
            raise WorkerError(f"experiment spec {path} is not valid JSON: {error}") from error
    return spec, os.path.abspath(path)


def _resolve_spec(args: dict[str, Any]) -> tuple[dict[str, Any], Optional[str]]:
    """Return ``(spec, path_or_None)`` from either a direct spec or experimentId."""
    spec_arg = args.get("spec")
    if isinstance(spec_arg, dict):
        return normalize_spec(spec_arg), None
    experiment_id = args.get("experimentId")
    if experiment_id:
        return load_spec(_store_root(args), experiment_id)
    raise WorkerError("provide either 'experimentId' or 'spec'")


# ---------------------------------------------------------------------------
# pure functions: matrix, faults, metrics, ablation
# ---------------------------------------------------------------------------


def expand_matrix(spec: dict[str, Any], max_cells: Optional[int] = None) -> dict[str, Any]:
    """Cartesian product of independentVariables x repetitions, with seeds.

    Pure and deterministic. Cells are ordered by variable declaration order,
    then value order, then repetition. Each cell's seed is
    ``spec.seed + <flat index>``. ``max_cells`` truncates the flat sequence
    (the result reports the truncation).
    """
    combos: list[dict[str, Any]] = [{}]
    for var in spec["independentVariables"]:
        combos = [
            dict(combo, **{var["name"]: value}) for combo in combos for value in var["values"]
        ]
    requested = len(combos) * spec["repetitions"]
    total = requested
    truncated = False
    if max_cells is not None:
        max_cells = int(max_cells)
        if max_cells < 0:
            raise WorkerError("'maxCells' must be >= 0")
        if total > max_cells:
            total = max_cells
            truncated = True

    cells: list[dict[str, Any]] = []
    index = 0
    for combo in combos:
        for repetition in range(1, spec["repetitions"] + 1):
            if index >= total:
                break
            cells.append(
                {
                    "cellId": f"cell-{index:04d}",
                    "variables": dict(combo),
                    "seed": spec["seed"] + index,
                    "repetition": repetition,
                }
            )
            index += 1
        if index >= total:
            break

    result: dict[str, Any] = {
        "cells": cells,
        "total": len(cells),
        "requestedTotal": requested,
        "truncated": truncated,
    }
    if truncated:
        result["note"] = f"matrix truncated from {requested} to {total} cells by maxCells={max_cells}"
    return result


def _substitute_value(value: Any, replacement: Any) -> Any:
    """Recursively replace ``__VALUE__`` / ``$value`` markers with a cell value."""
    if isinstance(value, dict):
        return {key: _substitute_value(v, replacement) for key, v in value.items()}
    if isinstance(value, list):
        return [_substitute_value(v, replacement) for v in value]
    if isinstance(value, str):
        if value in VALUE_MARKERS:
            return replacement
        if VALUE_MARKERS[0] in value:
            return value.replace(VALUE_MARKERS[0], str(replacement))
        return value
    return value


def fault_for_cell(
    spec: dict[str, Any],
    variables: dict[str, Any],
    fault_templates: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Map one cell's variable values to a simulation fault dict.

    Variables with an entry in ``fault_templates`` pass through the template
    (``__VALUE__`` / ``$value`` placeholders are replaced by the cell's
    value); variables without a template are merged directly as
    ``{name: value}``. When ``fault_templates`` is omitted entirely, every
    independent variable is merged directly as a fault field.
    """
    templates = fault_templates or {}
    fault: dict[str, Any] = {}
    for var in spec["independentVariables"]:
        name = var["name"]
        value = variables[name]
        template = templates.get(name)
        if template is not None:
            merged = _substitute_value(json.loads(json.dumps(template)), value)
            if not isinstance(merged, dict):
                raise WorkerError(f"fault template for variable {name!r} must resolve to an object")
            fault.update(merged)
        else:
            fault[name] = value
    return fault


def _run_success(run: dict[str, Any]) -> bool:
    value = run.get("success")
    if value is None:
        value = (run.get("metrics") or {}).get("success")
    return bool(value)


def _run_variables(run: dict[str, Any]) -> dict[str, Any]:
    variables = run.get("variables")
    return variables if isinstance(variables, dict) else {}


def _value_key(value: Any) -> tuple[str, Any]:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return ("scalar", value)
    return ("json", json.dumps(value, ensure_ascii=False, sort_keys=True))


def compute_metrics(
    runs: list[dict[str, Any]], primary_metric: str = DEFAULT_PRIMARY_METRIC
) -> dict[str, Any]:
    """Aggregate metrics over run records with pure numpy.

    ``runs`` is a list of ``{cellId?, variables?, seed?, repetition?, runId?,
    success?, metrics: {durationS?, trackingErrorRms?, ...}}``. Returns
    JSON-safe values only (no numpy types). ``perVariable`` groups runs by
    each independent variable's value (only variables present in every run
    record are reported, so partially-tagged lists degrade gracefully).
    """
    n = len(runs)
    if n == 0:
        return {"primaryMetric": primary_metric, "successRate": None, "runs": 0}
    successes = np.array([1.0 if _run_success(r) else 0.0 for r in runs], dtype=float)
    durations = [
        float(r.get("metrics", {}).get("durationS"))
        for r in runs
        if isinstance(r.get("metrics", {}).get("durationS"), (int, float))
    ]
    rms_values = [
        float(r.get("metrics", {}).get("trackingErrorRms"))
        for r in runs
        if isinstance(r.get("metrics", {}).get("trackingErrorRms"), (int, float))
    ]
    metrics: dict[str, Any] = {
        "primaryMetric": primary_metric,
        "successRate": round(float(successes.mean()), 3),
        "successStd": round(float(successes.std(ddof=1)), 3) if n > 1 else 0.0,
        "runs": n,
        "completionTimeMeanS": round(sum(durations) / len(durations), 3) if durations else None,
        "trackingRmsMean": round(sum(rms_values) / len(rms_values), 4) if rms_values else None,
    }
    per_variable = _per_variable_stats(runs)
    if per_variable:
        metrics["perVariable"] = per_variable
    return metrics


def _per_variable_stats(runs: list[dict[str, Any]]) -> dict[str, Any]:
    variable_names: Optional[set[str]] = None
    for run in runs:
        names = set(_run_variables(run))
        variable_names = names if variable_names is None else (variable_names & names)
    if not variable_names:
        return {}
    stats: dict[str, Any] = {}
    for name in sorted(variable_names):
        groups: dict[tuple[str, Any], dict[str, Any]] = {}
        for run in runs:
            value = _run_variables(run).get(name)
            key = _value_key(value)
            group = groups.setdefault(key, {"value": value, "successes": [], "runs": 0})
            group["successes"].append(1.0 if _run_success(run) else 0.0)
            group["runs"] += 1
        entries: dict[str, Any] = {}
        for _key, group in groups.items():
            value_key = json.dumps(group["value"], ensure_ascii=False, sort_keys=True)
            entries[value_key] = {
                "value": group["value"],
                "successRate": round(float(np.mean(group["successes"])), 3),
                "runs": group["runs"],
            }
        stats[name] = entries
    return stats


def ablation_compare(
    runs: list[dict[str, Any]],
    ablated_variable: str,
    variables_order: Optional[list[dict[str, Any]]] = None,
    baseline_value: Any = None,
    baseline_variables: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Group runs by one variable and compare each group with the baseline.

    Pure function. Baseline = the group whose ablatedVariable value equals
    ``baseline_value``; when not given, it is the first value declared for
    that variable in ``variables_order`` (the spec's independentVariables),
    falling back to the first group seen. The effect compares the baseline
    group against all other runs pooled.

    Returns ``{baseline, groups, effect, note}``; the note states that
    correlation is not causation and requires human confirmation.
    """
    if not runs:
        raise WorkerError("no runs to compare")
    if not ablated_variable:
        raise WorkerError("'ablatedVariable' is required")

    groups: dict[tuple[str, Any], dict[str, Any]] = {}
    for run in runs:
        variables = _run_variables(run)
        if ablated_variable not in variables:
            continue
        value = variables[ablated_variable]
        key = _value_key(value)
        group = groups.setdefault(key, {"value": value, "successes": [], "runs": 0})
        group["successes"].append(1.0 if _run_success(run) else 0.0)
        group["runs"] += 1
    if not groups:
        raise WorkerError(f"no runs carry the ablated variable {ablated_variable!r}")

    if baseline_value is None:
        if baseline_variables is not None and ablated_variable in baseline_variables:
            baseline_value = baseline_variables[ablated_variable]
        elif variables_order:
            for entry in variables_order:
                if isinstance(entry, dict) and entry.get("name") == ablated_variable:
                    values = entry.get("values") or []
                    if values:
                        baseline_value = values[0]
                    break

    order = list(groups)
    baseline_key = None
    if baseline_value is not None:
        target = _value_key(baseline_value)
        for key in order:
            if key == target:
                baseline_key = key
                break
    if baseline_key is None:
        baseline_key = order[0]

    group_stats = [
        {
            "variables": {ablated_variable: groups[key]["value"]},
            "successRate": round(float(np.mean(groups[key]["successes"])), 3),
            "runs": groups[key]["runs"],
        }
        for key in order
    ]
    baseline = group_stats[order.index(baseline_key)]

    rate_a = float(np.mean(groups[baseline_key]["successes"]))
    other_successes = [
        s for key in order if key != baseline_key for s in groups[key]["successes"]
    ]
    rate_b = float(np.mean(other_successes)) if other_successes else rate_a
    delta = rate_b - rate_a
    if delta > 0.001:
        direction = "improves"
    elif delta < -0.001:
        direction = "hurts"
    else:
        direction = "no-effect"

    return {
        "baseline": baseline,
        "groups": group_stats,
        "effect": {
            "ablatedVariable": ablated_variable,
            "summary": (
                f"移除/改变 {ablated_variable} 使成功率从 {rate_a:.3f} 变为 {rate_b:.3f}（Δ={delta:+.3f}）"
            ),
            "direction": direction,
        },
        "note": "相关性≠因果性，需人工确认",
    }


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------


def cmd_experiment_spec_create(args: dict[str, Any]) -> dict[str, Any]:
    raw = args.get("spec") if isinstance(args.get("spec"), dict) else args
    spec = normalize_spec(raw)
    experiment_id = new_id("exp")
    store_root = _store_root(args)
    path = save_spec(store_root, experiment_id, spec)
    missing = open_questions(spec)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "spec": spec,
        "path": path,
        "openQuestions": missing,
        "agentChecklist": {
            "researchQuestion": bool(spec.get("researchQuestion")),
            "baseline": bool(spec.get("baselines")),
            "control": bool(spec.get("controlVariables")),
            "statisticalMethod": bool(spec.get("statisticalMethod")),
        },
        "note": "实验定义已保存；开始实验前请审阅 openQuestions 中未明确的环节",
        "inputArgs": {"storeRoot": store_root},
    }


def cmd_experiment_matrix_expand(args: dict[str, Any]) -> dict[str, Any]:
    spec, _path = _resolve_spec(args)
    max_cells = args.get("maxCells")
    if max_cells is None:
        max_cells = (spec.get("termination") or {}).get("maxRuns")
    matrix = expand_matrix(spec, max_cells=max_cells)
    result: dict[str, Any] = {
        "ok": True,
        "experimentId": args.get("experimentId"),
        "cells": matrix["cells"],
        "total": matrix["total"],
        "requestedTotal": matrix["requestedTotal"],
        "truncated": matrix["truncated"],
        "inputArgs": {"experimentId": args.get("experimentId"), "maxCells": max_cells},
    }
    if matrix.get("note"):
        result["note"] = matrix["note"]
    return result


def cmd_benchmark_start(args: dict[str, Any]) -> dict[str, Any]:
    spec, spec_path = _resolve_spec(args)
    experiment_id = args.get("experimentId") or new_id("exp")
    max_cells = args.get("maxCells")
    if max_cells is None:
        max_cells = (spec.get("termination") or {}).get("maxRuns")
    matrix = expand_matrix(spec, max_cells=max_cells)
    if not matrix["cells"]:
        raise WorkerError("matrix expanded to zero cells; check independentVariables and maxCells")

    fault_templates = args.get("faultTemplates") or {}
    scenario_config = args.get("scenario") or SCENARIO_PICK_PLACE
    if isinstance(scenario_config, str):
        scenario_config = load_scenario(scenario_config)

    store_root = _store_root(args)
    store = RunStore(store_root)
    store.ensure()

    runs: list[dict[str, Any]] = []
    for cell in matrix["cells"]:
        fault = fault_for_cell(spec, cell["variables"], fault_templates)
        run, _telemetry = run_pick_place(scenario_config, fault, cell["seed"], store=store)
        runs.append(
            {
                "cellId": cell["cellId"],
                "variables": cell["variables"],
                "seed": cell["seed"],
                "repetition": cell["repetition"],
                "runId": run.id,
                "success": bool(run.metrics.get("success")),
                "metrics": run.metrics,
            }
        )

    summary = compute_metrics(runs, primary_metric=spec.get("primaryMetric", DEFAULT_PRIMARY_METRIC))
    result: dict[str, Any] = {
        "ok": True,
        "experimentId": experiment_id,
        "runs": runs,
        "summary": summary,
        "cells": len(runs),
        "truncated": matrix["truncated"],
        "storeRoot": store_root,
        "path": None,
        "note": "simulation-only benchmark; not a statistical study and not real-robot evidence",
    }
    if matrix.get("note"):
        result["note"] += " " + matrix["note"]

    if spec_path:  # persisted experiment -> update its results field
        stored = json.loads(json.dumps(spec))
        stored["results"] = {
            "benchmarkedAt": time.time(),
            "cells": len(runs),
            "scenario": scenario_config.get("name", SCENARIO_PICK_PLACE["name"]),
            "faultTemplates": fault_templates,
            "summary": summary,
            "runs": runs,
            "note": "auto-updated by benchmark-start; requires human review",
        }
        with open(spec_path, "w", encoding="utf-8") as handle:
            json.dump(stored, handle, ensure_ascii=False, indent=2)
        result["path"] = spec_path
    return result


def _load_runs(args: dict[str, Any]) -> tuple[list[dict[str, Any]], Optional[dict[str, Any]]]:
    """Return ``(runs, spec_or_None)`` from a direct runs list or experimentId."""
    runs_arg = args.get("runs")
    if isinstance(runs_arg, list):
        if not runs_arg:
            raise WorkerError("'runs' list is empty")
        return runs_arg, None
    experiment_id = args.get("experimentId")
    if experiment_id:
        spec, _path = load_spec(_store_root(args), experiment_id)
        results = spec.get("results") or {}
        runs = results.get("runs")
        if not isinstance(runs, list) or not runs:
            raise WorkerError(
                f"experiment {experiment_id!r} has no benchmark results yet; run benchmark-start first"
            )
        return runs, spec
    raise WorkerError("provide either 'runs' or 'experimentId'")


def cmd_metrics_compute(args: dict[str, Any]) -> dict[str, Any]:
    runs, loaded_spec = _load_runs(args)
    primary = DEFAULT_PRIMARY_METRIC
    if loaded_spec is not None and loaded_spec.get("primaryMetric"):
        primary = loaded_spec["primaryMetric"]
    elif isinstance(args.get("spec"), dict) and isinstance(args["spec"].get("primaryMetric"), str):
        primary = args["spec"]["primaryMetric"]
    elif isinstance(args.get("primaryMetric"), str):
        primary = args["primaryMetric"]
    metrics = compute_metrics(runs, primary_metric=primary)
    return {
        "ok": True,
        "metrics": metrics,
        "runs": len(runs),
        "notes": [
            "success rate = mean of binary success over runs; std = sample std (ddof=1), computed with pure numpy",
            "no hypothesis test performed; statistical significance and effect sizes require human review",
        ],
        "inputArgs": {
            "experimentId": args.get("experimentId"),
            "runs": len(runs) if isinstance(args.get("runs"), list) else None,
        },
    }


def cmd_ablation_compare(args: dict[str, Any]) -> dict[str, Any]:
    runs, loaded_spec = _load_runs(args)
    variables_order: Optional[list[dict[str, Any]]] = None
    if loaded_spec is not None:
        variables_order = loaded_spec.get("independentVariables") or []
    else:
        inline = args.get("spec")
        if isinstance(inline, dict) and isinstance(inline.get("independentVariables"), list):
            variables_order = inline["independentVariables"]
    ablated = args.get("ablatedVariable")
    if not ablated:
        raise WorkerError("'ablatedVariable' is required")
    baseline = args.get("baseline")
    baseline_value = None
    baseline_variables = None
    if isinstance(baseline, dict):
        baseline_variables = baseline.get("variables")
        if isinstance(baseline_variables, dict):
            baseline_value = baseline_variables.get(ablated)
    result = ablation_compare(
        runs,
        ablated,
        variables_order=variables_order,
        baseline_value=baseline_value,
        baseline_variables=baseline_variables,
    )
    return {
        "ok": True,
        "baseline": result["baseline"],
        "groups": result["groups"],
        "effect": result["effect"],
        "note": result["note"],
        "inputArgs": {"experimentId": args.get("experimentId"), "ablatedVariable": ablated},
    }


def cmd_benchmark_report(args: dict[str, Any]) -> dict[str, Any]:
    out_path = args.get("outPath")
    if not out_path:
        raise WorkerError("missing required argument 'outPath'")

    experiment_id = args.get("experimentId")
    spec: Optional[dict[str, Any]] = None
    spec_path: Optional[str] = None
    runs: Optional[list[dict[str, Any]]] = None
    if experiment_id:
        spec, spec_path = load_spec(_store_root(args), experiment_id)
        results = spec.get("results") or {}
        runs = results.get("runs")
        if not isinstance(runs, list) or not runs:
            raise WorkerError(
                f"experiment {experiment_id!r} has no benchmark results yet; run benchmark-start first"
            )
        name = spec.get("name") or experiment_id
    elif isinstance(args.get("runs"), list):
        runs = args.get("runs")
        if not runs:
            raise WorkerError("'runs' list is empty")
        name = args.get("name")
        if not name:
            raise WorkerError("provide 'name' when passing runs directly")
        if isinstance(args.get("spec"), dict):
            spec = normalize_spec(args["spec"])
    else:
        raise WorkerError("provide either 'experimentId' or {'name', 'runs'}")

    primary = spec.get("primaryMetric", DEFAULT_PRIMARY_METRIC) if spec else args.get(
        "primaryMetric", DEFAULT_PRIMARY_METRIC
    )
    metrics = compute_metrics(runs, primary_metric=primary)
    markdown = render_experiment_report(
        name,
        runs,
        metrics,
        spec=spec,
        experiment_id=experiment_id,
        spec_path=spec_path,
    )
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(markdown)
    return {
        "ok": True,
        "path": os.path.abspath(out_path),
        "experimentId": experiment_id,
        "inputArgs": {"experimentId": experiment_id, "outPath": out_path},
    }


# ---------------------------------------------------------------------------
# report rendering
# ---------------------------------------------------------------------------


def render_experiment_report(
    name: str,
    runs: list[dict[str, Any]],
    metrics: dict[str, Any],
    spec: Optional[dict[str, Any]] = None,
    experiment_id: Optional[str] = None,
    spec_path: Optional[str] = None,
) -> str:
    """Render a Markdown experiment report (own template, not per-run report)."""
    lines: list[str] = []

    def h(level: int, text: str) -> None:
        lines.append(f"{'#' * level} {text}\n")

    env = snapshot_environment()
    h(1, f"实验报告：{name}")
    lines.append(
        f"> 实验 ID：`{experiment_id or '（未持久化）'}` · 生成时间：`{time.strftime('%Y-%m-%d %H:%M:%S')}`\n"
    )

    h(2, "1. 实验定义")
    if spec:
        lines.append(f"- 研究问题：{spec.get('researchQuestion') or '（未明确）'}")
        lines.append(f"- 假设：{spec.get('hypothesis') or '（未明确）'}")
        lines.append(f"- 主要指标：`{spec.get('primaryMetric')}`")
        lines.append(f"- 种子：`{spec.get('seed')}` · 重复次数：`{spec.get('repetitions')}`")
        lines.append(f"- 统计方法：{spec.get('statisticalMethod') or '（未声明）'}")
        if spec.get("controlVariables"):
            h(3, "对照变量")
            lines.append("| 变量 | 取值 |")
            lines.append("|---|---|")
            for cv in spec["controlVariables"]:
                lines.append(
                    f"| {cv.get('name')} | {json.dumps(cv.get('value'), ensure_ascii=False)} |"
                )
        if spec.get("baselines"):
            h(3, "基线")
            lines.append("```json")
            lines.append(json.dumps(spec["baselines"], ensure_ascii=False, indent=2))
            lines.append("```")
        h(3, "自变量")
        lines.append("| 变量 | 取值 |")
        lines.append("|---|---|")
        for var in spec["independentVariables"]:
            lines.append(
                f"| {var['name']} | {json.dumps(var['values'], ensure_ascii=False)} |"
            )
    else:
        lines.append("- 实验定义未提供（仅结果摘要）。")

    h(2, "2. 实验矩阵")
    lines.append("| cellId | 变量 | 种子 | 重复 |")
    lines.append("|---|---|---|---|")
    for run in runs:
        lines.append(
            f"| {run.get('cellId', '')} | {json.dumps(_run_variables(run), ensure_ascii=False)} "
            f"| {run.get('seed')} | {run.get('repetition', '')} |"
        )

    h(2, "3. 结果明细")
    lines.append("| runId | cellId | 种子 | success | 用时(s) | 跟踪RMS |")
    lines.append("|---|---|---|---|---|---|")
    for run in runs:
        m = run.get("metrics") or {}
        lines.append(
            f"| {run.get('runId', '')} | {run.get('cellId', '')} | {run.get('seed')} "
            f"| {run.get('success')} | {m.get('durationS')} | {m.get('trackingErrorRms')} |"
        )

    h(2, "4. 聚合指标")
    lines.append(
        f"- 成功率：**{metrics.get('successRate')}**（{metrics.get('runs')} 次运行，std={metrics.get('successStd')}）"
    )
    if metrics.get("completionTimeMeanS") is not None:
        lines.append(f"- 平均完成时间：{metrics.get('completionTimeMeanS')} s")
    if metrics.get("trackingRmsMean") is not None:
        lines.append(f"- 平均跟踪误差 RMS：{metrics.get('trackingRmsMean')} rad")
    per_variable = metrics.get("perVariable")
    if per_variable:
        h(3, "按自变量分组")
        for var_name, groups in per_variable.items():
            lines.append(f"**{var_name}**")
            lines.append("| 取值 | 成功率 | 运行数 |")
            lines.append("|---|---|---|")
            for value_key, group in groups.items():
                lines.append(f"| {value_key} | {group['successRate']} | {group['runs']} |")

    h(2, "5. Ablation 摘要")
    lines.append("> 相关性≠因果性，需人工确认。")
    if spec:
        ablated_any = False
        for var in spec["independentVariables"]:
            if len(var["values"]) > 1:
                try:
                    ab = ablation_compare(runs, var["name"], variables_order=spec["independentVariables"])
                except WorkerError:
                    continue
                lines.append(
                    f"- **{var['name']}**：{ab['effect']['summary']} → `{ab['effect']['direction']}`"
                )
                ablated_any = True
        if not ablated_any:
            lines.append("- 无含多取值的自变量，未做 ablation 对比。")
    else:
        lines.append("- 未提供实验定义，无法进行 ablation 对比。")

    h(2, "6. 可复现性清单")
    lines.append(f"- 种子方案：`spec.seed + 序号`（本实验种子 `{spec.get('seed') if spec else '?'}`）")
    lines.append(f"- Python：{env.get('python')} · 平台：{env.get('platform')}")
    for mod in ("numpy", "mujoco", "cv2", "matplotlib"):
        lines.append(f"- {mod}：{env.get(mod) or '未安装'}")
    lines.append("- 代码版本（git commit）：未记录（占位，建议实验前记录）。")
    if spec_path:
        lines.append(f"- 实验定义文件：`{spec_path}`")

    h(2, "7. 声明")
    lines.append("> **本报告为自动生成摘要，需人工审阅后再用于任何决策。**")
    lines.append("- 模拟结果不能作为真机安全性的证据（sim-to-real 差距未测量）。")
    lines.append("- 成功率/指标为纯聚合统计，未做显著性检验；相关性≠因果性。")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# module exports (worker module contract)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "experiment-spec-create": cmd_experiment_spec_create,
    "experiment-matrix-expand": cmd_experiment_matrix_expand,
    "benchmark-start": cmd_benchmark_start,
    "metrics-compute": cmd_metrics_compute,
    "ablation-compare": cmd_ablation_compare,
    "benchmark-report": cmd_benchmark_report,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "experiment.spec_create",
        "kind": "experiment",
        "provider": "robotic-harness-worker",
        "input": {"name": "string", "independentVariables": "array"},
        "output": "persisted ExperimentSpec + agent checklist",
        "risk": "R1-derive",
        "description": "Define a reproducible experiment spec (research question, variables, baselines, seed, repetitions, statistical method).",
    },
    {
        "id": "experiment.matrix_benchmark",
        "kind": "simulation",
        "provider": "robotic-harness-worker",
        "input": {"experimentId": "string", "faultTemplates": "object?", "scenario": "object?"},
        "output": "matrix benchmark runs + summary",
        "risk": "R2-simulation",
        "description": "Run the experiment matrix through the pick-place simulation and record per-cell runs.",
    },
    {
        "id": "experiment.analyze",
        "kind": "analysis",
        "provider": "robotic-harness-worker",
        "input": {"experimentId": "string", "ablatedVariable": "string?"},
        "output": "aggregated metrics, ablation comparison, Markdown report",
        "risk": "R0-readonly",
        "description": "Aggregate metrics, compare ablations and render a Markdown experiment report (auto-generated; requires human review).",
    },
]
