"""One-shot JSON CLI for the Robotic Harness worker.

Protocol::

    python -m robotic_harness_worker <command> --input <file|-|none> [--output <file>]

- ``--input`` reads the arguments JSON (use ``-`` for stdin, omit for none).
- Results are written as a single JSON document on stdout.
- Domain failures return exit code 0 with ``{"ok": false, "error": {...}}`` so
  the caller can distinguish them from invocation errors (exit code 1).
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import traceback
from typing import Any, Callable

from . import __version__, WORKER_CAPABILITIES
from .core import RunStore, WorkerError, new_id, snapshot_environment


def _read_args(parser_args: argparse.Namespace) -> dict[str, Any]:
    if getattr(parser_args, "input", None) is None:
        return {}
    if parser_args.input == "-":
        raw = sys.stdin.read()
    else:
        with open(parser_args.input, encoding="utf-8") as handle:
            raw = handle.read()
    if not raw.strip():
        return {}
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as error:
        raise WorkerError(f"input is not valid JSON: {error}") from error
    if not isinstance(data, dict):
        raise WorkerError(f"input must be a JSON object, got {type(data).__name__}")
    return data


def _store_for(args: dict[str, Any]) -> RunStore:
    root = args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")
    store = RunStore(root)
    store.ensure()
    return store


def _write_output(parser_args: argparse.Namespace, result: Any) -> None:
    payload = json.dumps(result, ensure_ascii=False, indent=2 if parser_args.pretty else None)
    if parser_args.output and parser_args.output != "-":
        with open(parser_args.output, "w", encoding="utf-8") as handle:
            handle.write(payload)
    else:
        sys.stdout.write(payload + "\n")


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------

def cmd_ping(args: dict[str, Any]) -> dict[str, Any]:
    return {"ok": True, "service": "robotic-harness-worker", "version": __version__, "environment": snapshot_environment()}


def cmd_capability_list(args: dict[str, Any]) -> dict[str, Any]:
    capabilities = list(WORKER_CAPABILITIES)
    for module_name in _DOMAIN_MODULES:
        module = _import_domain_module(module_name)
        if module is not None:
            for capability in getattr(module, "CAPABILITIES", []):
                capabilities.append(capability)
    return {"ok": True, "capabilities": capabilities}


def cmd_inspect_asset(args: dict[str, Any]) -> dict[str, Any]:
    from .assets import inspect_asset

    path = args.get("path") or args.get("assetPath")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"asset not found: {path}")
    inspection = inspect_asset(path)
    result = inspection.to_dict()
    result["ok"] = not any(i.severity == "error" for i in inspection.issues)
    return result


def cmd_validate_urdf(args: dict[str, Any]) -> dict[str, Any]:
    from .assets import validate_urdf

    path = args.get("path") or args.get("urdfPath")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"URDF not found: {path}")
    return validate_urdf(path)


def cmd_convert_urdf(args: dict[str, Any]) -> dict[str, Any]:
    from .assets import convert_urdf_to_mjcf

    source = args.get("path") or args.get("urdfPath")
    target = args.get("outPath")
    if not source or not target:
        raise WorkerError("missing required arguments 'path' and 'outPath'")
    if not os.path.exists(source):
        raise WorkerError(f"URDF not found: {source}")
    return convert_urdf_to_mjcf(source, target)


def cmd_sim_status(args: dict[str, Any]) -> dict[str, Any]:
    from .simulation import PickPlaceEnv, SCENARIO_PICK_PLACE, load_scenario, validate_scenario

    env_snapshot = snapshot_environment()
    try:
        from .simulation import mujoco  # noqa: PLC0415

        mujoco_ok = mujoco is not None
    except Exception:  # pragma: no cover
        mujoco_ok = False
    result: dict[str, Any] = {
        "ok": True,
        "mujoco": env_snapshot.get("mujoco"),
        "mujocoAvailable": bool(mujoco_ok),
        "opencv": env_snapshot.get("cv2"),
        "matplotlib": env_snapshot.get("matplotlib"),
        "renderer": "unknown",
        "scenario": SCENARIO_PICK_PLACE["name"],
        "scenarioValid": True,
    }
    if mujoco_ok:
        try:
            from .simulation import mujoco as mj

            env = PickPlaceEnv(SCENARIO_PICK_PLACE)
            env.reset()
            image = env.render_rgb()
            result["renderer"] = "offscreen" if image is not None else "unavailable (simulated perception will be used)"
        except Exception as error:  # pragma: no cover
            result["mujocoAvailable"] = False
            result["mujocoError"] = str(error)
            result["scenarioValid"] = False
    try:
        validated = validate_scenario({})
        result["scenarioValid"] = validated["ok"]
        result["scenarioIssues"] = validated["issues"]
    except Exception as error:  # pragma: no cover
        result["scenarioValid"] = False
        result["scenarioError"] = str(error)
    return result


def cmd_sim_validate_scenario(args: dict[str, Any]) -> dict[str, Any]:
    from .simulation import load_scenario, validate_scenario

    config = args.get("scenario", {})
    if isinstance(config, str):
        config = {"path": config}
    path = config.get("path") or args.get("path")
    if path:
        loaded = load_scenario(path)
        return {"ok": True, "name": loaded["name"], "validated": validate_scenario(loaded)}
    validated = validate_scenario(config)
    return {"ok": validated["ok"], "issues": validated["issues"], "resolved": validated["resolved"]}


def cmd_sim_run(args: dict[str, Any]) -> dict[str, Any]:
    from .simulation import run_pick_place

    scenario_config = args.get("scenario", {})
    if isinstance(scenario_config, str):
        from .simulation import load_scenario

        scenario_config = load_scenario(scenario_config)
    fault = args.get("fault", {})
    seed = int(args.get("seed", 42))
    store = _store_for(args)
    run, telemetry = run_pick_place(
        scenario_config,
        fault,
        seed,
        store=store,
        run_id=args.get("runId") or new_id("run"),
    )
    return {
        "ok": True,
        "runId": run.id,
        "state": run.state,
        "success": bool(run.metrics.get("success")),
        "metrics": run.metrics,
        "phases": [p.to_dict() for p in run.phases],
        "anomalies": [a.to_dict() for a in run.anomalies],
        "runDir": store.run_dir(run.id),
        "artifacts": run.artifacts,
    }


def cmd_diagnose_run(args: dict[str, Any]) -> dict[str, Any]:
    from .diagnostics import diagnose, load_run_data
    from .memory import cmd_memory_retrieve

    run_path = args.get("runPath") or args.get("runDir")
    if not run_path:
        raise WorkerError("missing required argument 'runPath'")
    run, telemetry = load_run_data(run_path)
    case = diagnose(run, telemetry)
    store = _store_for(args)
    case_path = store.save_case(case)

    # Project memory: attach the most similar historical cases so the model
    # reasons with prior evidence (scores and rationale included).
    related: list[dict[str, Any]] = []
    try:
        memory = cmd_memory_retrieve(
            {"runPath": run_path, "limit": 3, "excludeRunId": run.id, "storeRoot": store.root}
        )
        related = memory.get("related", [])
    except Exception as error:  # noqa: BLE001 - memory must never break diagnosis
        related = [{"error": f"memory retrieval failed: {error}"}]

    return {
        "ok": True,
        "caseId": case.id,
        "runId": run.id,
        "symptom": case.symptom,
        "findings": [f.to_dict() for f in case.findings],
        "hypotheses": [h.to_dict() for h in case.hypotheses],
        "relatedCases": related,
        "casePath": case_path,
    }


def cmd_evidence_export(args: dict[str, Any]) -> dict[str, Any]:
    from .diagnostics import diagnose, load_run_data
    from .report import export_evidence

    run_path = args.get("runPath") or args.get("runDir")
    out_dir = args.get("outDir")
    if not run_path or not out_dir:
        raise WorkerError("missing required arguments 'runPath' and 'outDir'")
    run, telemetry = load_run_data(run_path)
    case = diagnose(run, telemetry)
    manifest = export_evidence(run_path, case, out_dir)
    return {"ok": True, "bundleDir": os.path.abspath(out_dir), "manifest": manifest}


def cmd_report_generate(args: dict[str, Any]) -> dict[str, Any]:
    from .diagnostics import diagnose, load_run_data
    from .report import generate_report, timeline_html

    run_path = args.get("runPath") or args.get("runDir")
    out_path = args.get("outPath")
    if not run_path or not out_path:
        raise WorkerError("missing required arguments 'runPath' and 'outPath'")
    run, telemetry = load_run_data(run_path)
    case = diagnose(run, telemetry)
    report_path = generate_report(run_path, case, out_path)
    timeline_path = None
    if args.get("timeline"):
        timeline_path = timeline_html(run, telemetry, case, os.path.join(os.path.dirname(out_path), "timeline.html"))
    return {"ok": True, "report": report_path, "timeline": timeline_path}


def cmd_data_quality(args: dict[str, Any]) -> dict[str, Any]:
    from .data_quality import audit

    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"data file not found: {path}")
    return audit(path, format=args.get("format"), time_column=args.get("timeColumn", "t"))


def cmd_demo(args: dict[str, Any]) -> dict[str, Any]:
    """End-to-end demo: happy run + fault run + diagnostics + evidence + report."""
    from .diagnostics import diagnose, load_run_data
    from .report import export_evidence, generate_report, timeline_html
    from .simulation import load_scenario, run_pick_place

    store = _store_for(args)
    seed = int(args.get("seed", 42))
    scenario = load_scenario(args.get("scenario", "mujoco_pick_place"))
    demo_dir = args.get("demoDir") or os.path.join(store.root, "demo")
    os.makedirs(demo_dir, exist_ok=True)

    happy_run, happy_telemetry = run_pick_place(scenario, {}, seed, store=store)
    fault = {"perception_offset_px": [18.0, 6.0], "gripper_slip": True, "tf_offset": [0.015, 0.0]}
    fault_run, fault_telemetry = run_pick_place(scenario, fault, seed + 1, store=store)

    summary: list[dict[str, Any]] = []
    for run, telemetry in ((happy_run, happy_telemetry), (fault_run, fault_telemetry)):
        case = diagnose(run, telemetry)
        case_path = store.save_case(case)
        run_dir = store.run_dir(run.id)
        bundle_dir = os.path.join(demo_dir, f"bundle-{run.id}")
        export_evidence(run_dir, case, bundle_dir)
        report_path = os.path.join(demo_dir, f"report-{run.id}.md")
        generate_report(run_dir, case, report_path, evidence_dir=bundle_dir)
        timeline_path = timeline_html(run, telemetry, case, os.path.join(demo_dir, f"timeline-{run.id}.html"))
        summary.append(
            {
                "runId": run.id,
                "success": bool(run.metrics.get("success")),
                "state": run.state,
                "report": report_path,
                "timeline": timeline_path,
                "evidenceBundle": bundle_dir,
                "casePath": case_path,
                "metrics": run.metrics,
            }
        )
    return {"ok": True, "demoDir": os.path.abspath(demo_dir), "runs": summary}


# ---------------------------------------------------------------------------
# extended simulation / report commands
# ---------------------------------------------------------------------------

def cmd_sim_fault_inject(args: dict[str, Any]) -> dict[str, Any]:
    """Dedicated fault-injection wrapper over sim-run (same engine)."""
    from .simulation import load_scenario, run_pick_place

    scenario_config = args.get("scenario", {})
    if isinstance(scenario_config, str):
        scenario_config = load_scenario(scenario_config)
    store = _store_for(args)
    run, _ = run_pick_place(
        scenario_config,
        args.get("fault", {}),
        int(args.get("seed", 42)),
        store=store,
        run_id=args.get("runId") or new_id("run"),
    )
    return {
        "ok": True,
        "runId": run.id,
        "injectedFault": args.get("fault", {}),
        "success": bool(run.metrics.get("success")),
        "metrics": run.metrics,
        "anomalies": [a.to_dict() for a in run.anomalies],
        "runDir": store.run_dir(run.id),
    }


def cmd_sdf_validate(args: dict[str, Any]) -> dict[str, Any]:
    from .assets import validate_sdf

    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.exists(path):
        raise WorkerError(f"SDF not found: {path}")
    return validate_sdf(path)


def cmd_sim_replay(args: dict[str, Any]) -> dict[str, Any]:
    from .report import replay_run

    run_path = args.get("runPath") or args.get("runDir")
    out_dir = args.get("outDir")
    if not run_path or not out_dir:
        raise WorkerError("missing required arguments 'runPath' and 'outDir'")
    return replay_run(run_path, out_dir)


def cmd_sim_real_gap_report(args: dict[str, Any]) -> dict[str, Any]:
    from .simulation import sim_real_gap

    sim_run = args.get("simRunPath") or args.get("runPath")
    real_csv = args.get("realCsvPath")
    channel_map = args.get("channelMap")
    if not sim_run or not real_csv or not channel_map:
        raise WorkerError("missing required arguments 'simRunPath', 'realCsvPath' and 'channelMap'")
    return sim_real_gap(sim_run, real_csv, channel_map)


def cmd_sim_batch_benchmark(args: dict[str, Any]) -> dict[str, Any]:
    from .simulation import sim_batch_benchmark

    cells = args.get("cells") or args.get("matrix")
    if not cells:
        raise WorkerError("missing required argument 'cells' (list of {label?, fault?, seed?})")
    store = _store_for(args)
    return sim_batch_benchmark(cells, store=store, out_dir=args.get("outDir"))


def cmd_dashboard_generate(args: dict[str, Any]) -> dict[str, Any]:
    from .report import dashboard_html

    store = _store_for(args)
    out_path = args.get("outPath")
    if not out_path:
        raise WorkerError("missing required argument 'outPath'")
    path = dashboard_html(store.root, out_path)
    return {"ok": True, "path": path, "storeRoot": store.root}


# ---------------------------------------------------------------------------
# dispatch
# ---------------------------------------------------------------------------

# Domain modules whose COMMANDS are auto-registered. Each module exports
# ``COMMANDS: dict[str, Callable[[dict], dict]]`` per the worker module
# contract (docs/worker-module-contract.md).
_DOMAIN_MODULES = [
    "control",
    "ros",
    "telemetry",
    "data_pipeline",
    "models",
    "vision_extra",
    "experiment",
    "cad",
    "knowledge",
    "memory",
    "robots",
]


def _import_domain_module(module_name: str):
    try:
        return importlib.import_module(f".{module_name}", __package__)
    except ImportError:
        return None


def _register_domain_commands(commands: dict[str, Callable[[dict[str, Any]], dict[str, Any]]]) -> None:
    for module_name in _DOMAIN_MODULES:
        module = _import_domain_module(module_name)
        if module is None:
            continue
        for name, fn in getattr(module, "COMMANDS", {}).items():
            if name in commands:
                raise RuntimeError(f"duplicate command {name!r} from module {module_name}")
            commands[name] = fn


COMMANDS: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
    "ping": cmd_ping,
    "capability-list": cmd_capability_list,
    "inspect-asset": cmd_inspect_asset,
    "validate-urdf": cmd_validate_urdf,
    "convert-urdf": cmd_convert_urdf,
    "sim-status": cmd_sim_status,
    "sim-validate-scenario": cmd_sim_validate_scenario,
    "sim-run": cmd_sim_run,
    "sim-fault-inject": cmd_sim_fault_inject,
    "sdf-validate": cmd_sdf_validate,
    "sim-replay": cmd_sim_replay,
    "sim-real-gap-report": cmd_sim_real_gap_report,
    "sim-batch-benchmark": cmd_sim_batch_benchmark,
    "diagnose-run": cmd_diagnose_run,
    "evidence-export": cmd_evidence_export,
    "report-generate": cmd_report_generate,
    "dashboard-generate": cmd_dashboard_generate,
    "data-quality": cmd_data_quality,
    "demo": cmd_demo,
}

_register_domain_commands(COMMANDS)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robotic-harness-worker", description="Robotic Harness worker CLI")
    parser.add_argument("command", choices=sorted(COMMANDS))
    parser.add_argument("--input", help="path to arguments JSON, '-' for stdin, omit for none")
    parser.add_argument("--output", help="path to write the result JSON ('-' for stdout, the default)")
    parser.add_argument("--pretty", action="store_true", help="indent the output JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        arguments = _read_args(args)
        result = COMMANDS[args.command](arguments)
        result.setdefault("ok", True)
        _write_output(args, result)
        return 0
    except WorkerError as error:
        _write_output(args, {"ok": False, "error": {"kind": "worker", "message": str(error)}})
        return 0
    except Exception as error:  # noqa: BLE001 - report any failure as a structured error
        _write_output(
            args,
            {"ok": False, "error": {"kind": "internal", "message": f"{type(error).__name__}: {error}", "traceback": traceback.format_exc()}},
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
