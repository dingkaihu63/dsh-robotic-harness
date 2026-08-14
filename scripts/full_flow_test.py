#!/usr/bin/env python
"""Full-flow end-to-end test for Robotic Harness.

Walks the whole R&D chain through the REAL CLI (one subprocess per command,
exactly how the DSH tools invoke the worker), verifying every step returns
ok:true, and writes all artifacts under the output root (never C:).

Run:
    python scripts/full_flow_test.py [--out F:\\dsh\\.flow-test\\<name>]

The script also re-runs the pytest suite and the one-command demo, and
finishes with a Markdown summary + JSON ledger.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import os
import shutil
import subprocess
import sys
import time
import uuid

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHON_DIR = os.path.join(REPO, "python")
PYTHON = os.environ.get("PYTHON") or sys.executable
ENV = {
    **os.environ,
    "PYTHONPATH": PYTHON_DIR,
    "PYTHONUNBUFFERED": "1",
    # force UTF-8 on pipe stdout/stderr regardless of the Windows console code page
    "PYTHONIOENCODING": "utf-8",
    "PYTHONUTF8": "1",
    # keep every temp/cache on F:
    "TMP": "F:\\dsh\\.flow-test\\tmp",
    "TEMP": "F:\\dsh\\.flow-test\\tmp",
}

STEPS: list[dict] = []
FAILURES: list[str] = []


def run(command: str, args: dict, out_dir: str, name: str, expect_ok: bool = True) -> dict:
    """Run one worker command over stdio; returns the parsed result."""
    os.makedirs(out_dir, exist_ok=True)
    input_path = os.path.join(out_dir, f"{name}.args.json")
    with open(input_path, "w", encoding="utf-8") as handle:
        json.dump(args, handle, ensure_ascii=False)
    started = time.time()
    proc = subprocess.run(
        [PYTHON, "-m", "robotic_harness_worker", command, "--input", input_path],
        capture_output=True,
        text=True,
        encoding="utf-8",
        env=ENV,
        timeout=600,
    )
    elapsed = round(time.time() - started, 2)
    result: dict = {}
    if proc.returncode != 0:
        # the worker writes error JSON to stdout even for internal errors
        if proc.stdout:
            try:
                result = json.loads(proc.stdout)
            except json.JSONDecodeError:
                result = {"ok": False, "error": {"kind": "output", "message": proc.stdout[-2000:]}}
        else:
            result = {"ok": False, "error": {"kind": "process", "message": proc.stderr[-2000:] if proc.stderr else "no stderr"}}
    elif not proc.stdout:
        result = {"ok": False, "error": {"kind": "output", "message": proc.stderr[-2000:] if proc.stderr else "no stdout"}}
    else:
        try:
            result = json.loads(proc.stdout)
        except json.JSONDecodeError:
            result = {"ok": False, "error": {"kind": "output", "message": proc.stdout[-2000:]}}
    ok = bool(result.get("ok"))
    if expect_ok and not ok:
        FAILURES.append(f"{command}: {result.get('error')}")
    STEPS.append(
        {
            "command": command,
            "name": name,
            "ok": ok,
            "seconds": elapsed,
            "summary": _summarize(result),
        }
    )
    print(f"  [{'OK ' if ok else 'FAIL'}] {command} ({elapsed:.1f}s) {_summarize(result)}", flush=True)
    return result


def _summarize(result: dict) -> str:
    if not result.get("ok"):
        error = result.get("error") or {}
        return f"error: {error.get('message', error) if isinstance(error, dict) else error}"[:120]
    parts: list[str] = []
    for key in ("runId", "caseId", "experimentId", "datasetId", "path", "outPath", "report", "bundleDir", "verdict", "total", "related", "models", "runs", "episodes", "contentHash"):
        if key in result:
            value = result[key]
            if isinstance(value, (list, dict)):
                value = f"<{len(value)}>"
            parts.append(f"{key}={value}")
    return " ".join(parts)[:120] if parts else "ok"


def gen_step_csv(path: str) -> None:
    import numpy as np

    rng = np.random.default_rng(1)
    t = np.arange(0, 6.0, 0.01)
    tau = 0.5
    y = 1 - np.exp(-t / tau) + rng.normal(0, 0.01, len(t))
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "setpoint", "measurement", "output", "effort"])
        for ti, yi in zip(t, y):
            writer.writerow([f"{ti:.3f}", "1.0", f"{yi:.5f}", f"{yi:.5f}", f"{min(max(yi * 10, 0), 10):.3f}"])


def gen_trajectory_csv(path: str, offset: float = 0.0, delay_s: float = 0.0) -> None:
    import numpy as np

    t = np.arange(0, 5.0, 0.02)
    q = 0.5 * np.sin(1.5 * t) + 0.2 * np.sin(4.0 * t)
    q_delayed = np.interp(t - delay_s, t, q)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "q0"])
        for ti, qi in zip(t, q_delayed + offset):
            writer.writerow([f"{ti:.3f}", f"{qi:.5f}"])


def gen_stl(path: str) -> None:
    import numpy as np

    # one tetrahedron + one degenerate triangle
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
    faces = [(0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 2, 3), (0, 0, 0)]
    with open(path, "wb") as handle:
        handle.write(b"binary stl".ljust(80, b"\0"))
        handle.write(np.uint32(len(faces)).tobytes())
        for face in faces:
            a, b, c = vertices[list(face)]
            normal = np.cross(b - a, c - a).astype(np.float32)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            handle.write(normal.tobytes())
            handle.write(a.tobytes())
            handle.write(b.tobytes())
            handle.write(c.tobytes())
            handle.write(np.uint16(0).tobytes())


def gen_calib_yaml(path: str) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("fx: 600.0\nfy: 600.0\ncx: 320.0\ncy: 320.0\nimageSize: [640, 480]\nreprojectionError: 0.4\ncalibrationDate: '2026-08-01'\nsource: demo\n")


def gen_stream_csv(path: str, delay_s: float = 0.5) -> None:
    import numpy as np

    t = np.arange(0, 10, 0.02)
    signal = np.sin(2 * np.pi * 0.3 * t) + 0.4 * np.sin(2 * np.pi * 1.1 * t)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "value"])
        for ti, si in zip(t, signal):
            writer.writerow([f"{ti:.3f}", f"{si:.5f}"])
    with open(path.replace("a.csv", "b.csv"), "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "value"])
        for ti, si in zip(t + delay_s, signal):
            writer.writerow([f"{ti:.3f}", f"{si:.5f}"])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=os.path.join("F:\\dsh\\.flow-test", f"run-{uuid.uuid4().hex[:8]}"))
    parser.add_argument("--skip-pytest", action="store_true")
    parser.add_argument("--skip-demo", action="store_true")
    args = parser.parse_args()

    out = os.path.abspath(args.out)
    data = os.path.join(out, "data")
    work = os.path.join(out, "work")
    store = os.path.join(out, "store", ".rh")
    os.makedirs(data, exist_ok=True)
    os.makedirs(work, exist_ok=True)
    os.makedirs(store, exist_ok=True)
    os.makedirs(ENV["TMP"], exist_ok=True)
    print(f"output root: {out} (on drive {os.path.splitdrive(out)[0]})", flush=True)

    start = time.time()

    # ------------------------------------------------------------------ unit
    if not args.skip_pytest:
        print("\n== 0) unit test suite (per-file isolation) ==", flush=True)
        proc = subprocess.run(
            [PYTHON, "run_tests.py"],
            cwd=PYTHON_DIR,
            capture_output=True,
            text=True,
            encoding="utf-8",
            env=ENV,
            timeout=1800,
        )
        tail = proc.stdout.strip().splitlines()[-4:]
        ok = proc.returncode == 0
        STEPS.append({"command": "pytest-run_tests", "name": "unit-suite", "ok": ok, "seconds": 0, "summary": " ".join(tail)})
        print(f"  [{'OK ' if ok else 'FAIL'}] pytest {' '.join(tail)}", flush=True)
        if not ok:
            FAILURES.append(f"pytest: {proc.stderr[-500:]}")

    # ------------------------------------------------------------- assets/CAD
    print("\n== 1) assets & CAD ==", flush=True)
    fixtures = os.path.join(REPO, "fixtures", "robot_assets")
    urdf = os.path.join(fixtures, "rh_arm.urdf")
    sdf = os.path.join(fixtures, "rh_arm.sdf")
    stl = os.path.join(data, "demo.stl")
    gen_stl(stl)

    run("ping", {}, work, "ping")
    run("inspect-asset", {"path": urdf}, work, "inspect-urdf")
    run("validate-urdf", {"path": urdf}, work, "validate-urdf")
    run("sdf-validate", {"path": sdf}, work, "sdf-validate")
    run("cad-inventory", {"path": fixtures, "recursive": True}, work, "cad-inventory")
    run("mesh-inspect", {"path": stl}, work, "mesh-inspect")
    run("inertia-validate", {"path": urdf}, work, "inertia-validate")
    run("robot-topology-validate", {"path": urdf}, work, "topology-validate")
    run("urdf-preview", {"path": urdf, "outPath": os.path.join(work, "preview.svg")}, work, "urdf-preview")
    run("convert-urdf", {"path": urdf, "outPath": os.path.join(work, "converted.mjcf")}, work, "urdf-to-mjcf")
    run("export-sim-asset", {"path": urdf, "target": "sdf-compat", "outPath": os.path.join(work, "sdf-compat.md")}, work, "export-sim-asset-sdf")
    run("asset-report", {"path": urdf, "outPath": os.path.join(work, "asset-report.md")}, work, "asset-report")

    # ------------------------------------------------------------ simulation
    print("\n== 2) simulation ==", flush=True)
    run("sim-status", {}, work, "sim-status")
    run("sim-validate-scenario", {"scenario": {}}, work, "sim-validate-scenario")
    happy = run("sim-run", {"seed": 42, "fault": {}}, work, "sim-run-clean")
    fault = run("sim-fault-inject", {"seed": 43, "fault": {"perception_offset_px": [18, 6], "gripper_slip": True, "tf_offset": [0.015, 0]}}, work, "sim-fault-inject")
    run("sim-batch-benchmark", {"cells": [{"label": "clean", "seed": 42}, {"label": "slip", "seed": 43, "fault": {"gripper_slip": True}}], "outDir": os.path.join(work, "benchmark")}, work, "sim-batch-benchmark")
    run("sim-replay", {"runPath": happy.get("runDir"), "outDir": os.path.join(work, "replay")}, work, "sim-replay")

    real_csv = os.path.join(data, "real.csv")
    with open(real_csv, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "joint0"])
        telemetry = os.path.join(happy.get("runDir", ""), "telemetry.jsonl")
        if os.path.exists(telemetry):
            with open(telemetry, encoding="utf-8") as th:
                for line in th:
                    row = json.loads(line)
                    writer.writerow([row["t"], row["q"][0] + 0.03])
    run("sim-real-gap-report", {"simRunPath": happy.get("runDir"), "realCsvPath": real_csv, "channelMap": {"q.0": "joint0"}}, work, "sim-real-gap")

    # --------------------------------------------------- telemetry/diagnostics
    print("\n== 3) telemetry & diagnostics & memory ==", flush=True)
    fault_dir = fault.get("runDir")
    run("telemetry-channels", {"runPath": fault_dir}, work, "telemetry-channels")
    run("telemetry-window", {"runPath": fault_dir, "startS": 0, "endS": 6}, work, "telemetry-window")
    scan = run("anomaly-scan", {"runPath": fault_dir}, work, "anomaly-scan")
    run("failure-evidence-collect", {"runPath": fault_dir, "createCase": True}, work, "failure-evidence-collect")
    diag = run("diagnose-run", {"runPath": fault_dir}, work, "diagnose-run")
    case_id = diag.get("caseId")
    memory = run("memory-retrieve", {"symptom": "grasp missed perception estimate off", "anomalyKinds": ["grasp_missed"]}, work, "memory-retrieve")
    run("memory-ingest", {"caseId": case_id, "status": "verified", "conclusion": "human confirmed: perception offset + slip", "operator": "flow-test"}, work, "memory-ingest")
    run("run-compare", {"runA": happy.get("runDir"), "runB": fault_dir}, work, "run-compare")
    run("timeline-export", {"runPath": fault_dir, "outPath": os.path.join(work, "timeline.html")}, work, "timeline-export")

    # ------------------------------------------------------------- vision
    print("\n== 4) vision & calibration ==", flush=True)
    scene = os.path.join(happy.get("runDir", ""), "artifacts", "scene.png")
    calib = os.path.join(data, "calib.yaml")
    gen_calib_yaml(calib)
    if os.path.exists(scene):
        run("camera-health-check", {"imagePath": scene}, work, "camera-health")
        run("perception-run", {"imagePath": scene, "route": "color", "outPath": os.path.join(work, "annotated.png")}, work, "perception-run")
        run("perception-run", {"imagePath": scene, "route": "saliency", "outPath": os.path.join(work, "annotated-saliency.png")}, work, "perception-run-saliency")
        run("perception-compare", {"imagePathA": scene, "imagePathB": scene, "method": "color"}, work, "perception-compare")
        run("image-dataset-profile", {"path": os.path.join(happy.get("runDir", ""), "artifacts")}, work, "image-profile")
        run("annotate-failure-frame", {"imagePath": scene, "detections": [{"centroidPx": [320, 240], "label": "object"}], "outPath": os.path.join(work, "failure-frame.png")}, work, "annotate-frame")
    run("calibration-inspect", {"path": calib}, work, "calibration-inspect")
    run("pose-transform-validate", {"transforms": [{"matrix": [[1, 0, 0, 0.1], [0, 1, 0, 0.2], [0, 0, 1, 0.3], [0, 0, 0, 1]]}, {"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]}]}, work, "pose-validate")

    # ------------------------------------------------------------- control
    print("\n== 5) control ==", flush=True)
    step_csv = os.path.join(data, "step.csv")
    gen_step_csv(step_csv)
    plan_csv = os.path.join(data, "planned.csv")
    act_csv = os.path.join(data, "actual.csv")
    gen_trajectory_csv(plan_csv)
    gen_trajectory_csv(act_csv, delay_s=0.1)
    run("control-trace-analyze", {"path": step_csv}, work, "control-trace")
    run("trajectory-validate", {"path": plan_csv}, work, "trajectory-validate")
    run("planned-actual-compare", {"plannedPath": plan_csv, "actualPath": act_csv}, work, "planned-actual")
    run("pid-experiment-prepare", {"controllerId": "demo", "joints": ["q0"], "amplitude": 0.1, "durationS": 10}, work, "pid-prepare")
    run("controller-config-compare", {"configA": {"name": "A", "joints": {"q0": {"kp": 100, "kv": 10}}}, "configB": {"name": "B", "joints": {"q0": {"kp": 200, "kv": 25}}}}, work, "controller-compare")
    run("system-identification", {"path": step_csv}, work, "sysid")
    run("control-report", {"sections": [{"kind": "trace", "path": step_csv, "title": "Step response"}, {"kind": "trajectory", "path": plan_csv, "title": "Planned"}], "outPath": os.path.join(work, "control-report.md")}, work, "control-report")

    # -------------------------------------------------------------- models
    print("\n== 6) embodied models ==", flush=True)
    run("model-inventory", {}, work, "model-inventory")
    run("model-health", {"modelId": "demo.color_segmentation"}, work, "model-health")
    run("model-warmup", {"modelId": "demo.scripted_pick_place"}, work, "model-warmup")
    run("model-infer", {"modelId": "demo.scripted_pick_place", "input": {"objectPose": [0.30, 0.19], "targetPose": [-0.16, 0.20]}}, work, "model-infer-ik")
    if os.path.exists(scene):
        run("model-infer", {"modelId": "demo.color_segmentation", "input": {"imagePath": scene, "color": "red"}}, work, "model-infer-vision")
        run("model-benchmark", {"modelId": "demo.color_segmentation", "iterations": 5, "input": {"imagePath": scene, "color": "red"}}, work, "model-benchmark")
    run("capability-route-explain", {"task": "pick_object", "embodiment": ["rh_planar_arm"]}, work, "capability-route")
    run("policy-rollout-compare", {"seeds": [42, 43], "faults": [{}]}, work, "policy-rollout")

    # -------------------------------------------------------------- robots
    print("\n== 7) real-robot state machine ==", flush=True)
    pre = run("robot-preflight", {"robotModel": "rh_planar_arm", "autoRun": True}, work, "robot-preflight")
    exp = run("experiment-prepare", {"name": "flow-test-exp", "requiresApproval": True, "scenario": "mujoco_pick_place"}, work, "experiment-prepare")
    exp_id = exp.get("experimentId")
    run("experiment-request-approval", {"experimentId": exp_id, "operator": "flow-test"}, work, "experiment-request-approval")
    run("experiment-start", {"experimentId": exp_id, "approver": "human-1", "approvalRef": "FLOW-TEST-001"}, work, "experiment-start")
    run("experiment-pause", {"experimentId": exp_id, "operator": "flow-test", "reason": "checkpoint"}, work, "experiment-pause")
    run("experiment-status", {"experimentId": exp_id}, work, "experiment-status")
    run("experiment-safe-cancel", {"experimentId": exp_id, "operator": "flow-test", "reason": "test complete"}, work, "experiment-safe-cancel")
    # note: experiment-finalize is covered by unit tests; safe-cancel already
    # reached the terminal ABORTED state here.

    # ---------------------------------------------------------------- data
    print("\n== 8) data pipeline ==", flush=True)
    stream_a = os.path.join(data, "stream_a.csv")
    gen_stream_csv(stream_a, delay_s=0.0)
    stream_b = os.path.join(data, "stream_b.csv")
    gen_stream_csv(stream_b, delay_s=0.5)
    data_csv = os.path.join(data, "dataset.csv")
    with open(data_csv, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "participant", "q0", "label"])
        for i in range(100):
            writer.writerow([f"{i * 0.1:.2f}", "p1" if i < 50 else "p2", f"{0.5 * (i % 7):.3f}", "a" if i % 2 else "b"])
    run("data-inventory", {"path": data}, work, "data-inventory")
    run("data-schema-inspect", {"path": data_csv}, work, "data-schema")
    run("data-time-sync-estimate", {"pathA": stream_a, "pathB": stream_b, "signalColumns": {"a": "value", "b": "value"}, "sampleRateHz": 50}, work, "data-time-sync")
    run("data-align-streams", {"primary": stream_a, "secondary": stream_b, "strategy": "nearest", "maxGapS": 0.1, "outPath": os.path.join(work, "aligned.csv")}, work, "data-align")
    run("data-transform-apply", {"inputPath": data_csv, "operations": [{"kind": "range-filter", "params": {"column": "q0", "min": 0.0, "max": 3.0}}, {"kind": "unit-convert", "params": {"column": "q0", "from": "m", "to": "mm"}}], "outPath": os.path.join(work, "transformed.csv")}, work, "data-transform")
    run("data-segment-episodes", {"path": data_csv, "timeColumn": "t", "maxGapS": 1.0}, work, "data-segment")
    run("data-split-create", {"path": data_csv, "outDir": os.path.join(work, "splits"), "method": "group", "groupColumns": ["participant"], "ratios": {"train": 0.7, "val": 0.15, "test": 0.15}, "seed": 42}, work, "data-split")
    run("data-leakage-check", {"trainPath": os.path.join(work, "splits", "train.csv"), "valPath": os.path.join(work, "splits", "val.csv"), "testPath": os.path.join(work, "splits", "test.csv"), "groupColumns": ["participant"]}, work, "data-leakage")
    run("data-deidentify", {"inputPath": data_csv, "operations": ["pii-scan"], "outDir": os.path.join(work, "deident")}, work, "data-deidentify")
    rosbag = os.path.join(REPO, "fixtures", "rosbags", "demo_rosbag")
    run("data-convert-rosbag", {"rosbagPath": rosbag, "outDir": os.path.join(work, "rosbag-out"), "topics": ["/signal"]}, work, "data-convert-rosbag")
    run("data-export-lerobot", {"runPath": happy.get("runDir"), "outDir": os.path.join(work, "lerobot"), "robotName": "rh_demo", "task": "pick_place"}, work, "data-export-lerobot")
    run("data-export-rlds", {"outDir": os.path.join(work, "rlds")}, work, "data-export-rlds")
    ver = run("dataset-version-create", {"name": "flow-dataset", "sourcePaths": [data_csv], "outDir": os.path.join(work, "dataset-v1"), "description": "flow test"}, work, "dataset-version")
    run("dataset-compare", {"datasetA": os.path.join(work, "dataset-v1"), "datasetB": os.path.join(work, "dataset-v1")}, work, "dataset-compare")
    run("dataset-card-generate", {"datasetPath": os.path.join(work, "dataset-v1"), "outPath": os.path.join(work, "dataset-card.md")}, work, "dataset-card")

    # ---------------------------------------------------------- experiments
    print("\n== 9) experiment management ==", flush=True)
    spec = run("experiment-spec-create", {"name": "flow-exp", "researchQuestion": "does perception offset hurt?", "independentVariables": [{"name": "perception_offset_px", "values": [0, 18]}], "repetitions": 1, "seed": 42}, work, "experiment-spec")
    run("experiment-matrix-expand", {"experimentId": spec.get("experimentId")}, work, "experiment-matrix")
    run("benchmark-start", {"experimentId": spec.get("experimentId"), "faultTemplates": {"perception_offset_px": {"perception_offset_px": ["__VALUE__", 0.0]}}}, work, "benchmark-start")
    run("metrics-compute", {"experimentId": spec.get("experimentId")}, work, "metrics-compute")
    run("ablation-compare", {"experimentId": spec.get("experimentId"), "ablatedVariable": "perception_offset_px"}, work, "ablation-compare")
    run("benchmark-report", {"experimentId": spec.get("experimentId"), "outPath": os.path.join(work, "benchmark-report.md")}, work, "benchmark-report")

    # ---------------------------------------------------- knowledge & memory
    print("\n== 10) knowledge ==", flush=True)
    run("docs-index", {"path": os.path.join(REPO, "docs"), "outPath": os.path.join(work, "docs-index.json")}, work, "docs-index")
    run("manual-search", {"query": "safety boundary", "path": os.path.join(work, "docs-index.json")}, work, "manual-search")
    run("error-code-lookup", {"code": "1"}, work, "error-code")
    run("case-search", {"query": "grasp missed"}, work, "case-search")
    run("memory-retrieve", {"runPath": fault_dir}, work, "memory-retrieve-run")
    run("memory-retrieve", {"symptom": "grasp missed perception estimate off", "anomalyKinds": ["grasp_missed"]}, work, "memory-retrieve-2")

    # ------------------------------------------------------------- reports
    print("\n== 11) reports & dashboard ==", flush=True)
    run("evidence-export", {"runPath": fault_dir, "outDir": os.path.join(work, "evidence-bundle")}, work, "evidence-export")
    run("report-generate", {"runPath": fault_dir, "outPath": os.path.join(work, "report.md")}, work, "report-generate")
    run("dashboard-generate", {"outPath": os.path.join(work, "dashboard.html")}, work, "dashboard-generate")

    # -------------------------------------------------------- one-command demo
    if not args.skip_demo:
        print("\n== 12) one-command demo ==", flush=True)
        demo_dir = os.path.join(out, "demo-output")
        proc = subprocess.run(
            [PYTHON, "-m", "robotic_harness_worker", "demo", "--input", "-"],
            input=json.dumps({"storeRoot": os.path.join(demo_dir, ".rh"), "demoDir": demo_dir}),
            capture_output=True,
            text=True,
            encoding="utf-8",
            env=ENV,
            timeout=900,
        )
        ok = proc.returncode == 0
        try:
            payload = json.loads(proc.stdout)
            demo_runs = len(payload.get("runs", []))
        except Exception:
            payload = {}
            demo_runs = 0
        STEPS.append({"command": "demo", "name": "demo", "ok": ok, "seconds": 0, "summary": f"runs={demo_runs}"})
        print(f"  [{'OK ' if ok else 'FAIL'}] demo runs={demo_runs}", flush=True)
        if not ok:
            FAILURES.append(f"demo: {proc.stderr[-500:]}")

    # ------------------------------------------------------------------ summary
    elapsed = round(time.time() - start, 1)
    summary = {
        "finishedAt": time.time(),
        "elapsedS": elapsed,
        "outputRoot": out,
        "drive": os.path.splitdrive(out)[0],
        "steps": STEPS,
        "okCount": sum(1 for s in STEPS if s["ok"]),
        "totalCount": len(STEPS),
        "failures": FAILURES,
    }
    ledger = os.path.join(out, "flow-test-ledger.json")
    with open(ledger, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    lines = [
        "# Robotic Harness — full-flow test report",
        "",
        f"- Finished: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"- Elapsed: {elapsed}s",
        f"- Output root: `{out}` (drive `{os.path.splitdrive(out)[0]}` — nothing on C:)",
        f"- Steps: **{summary['okCount']}/{summary['totalCount']} ok**",
        "",
        "| # | command | result | seconds | summary |",
        "|---|---------|--------|---------|---------|",
    ]
    for index, step in enumerate(STEPS, start=1):
        lines.append(f"| {index} | `{step['command']}` | {'✅' if step['ok'] else '❌'} | {step['seconds']:.1f} | {step['summary']} |")
    if FAILURES:
        lines.append("")
        lines.append("## Failures")
        for failure in FAILURES:
            lines.append(f"- {failure}")
    report = os.path.join(out, "flow-test-report.md")
    with open(report, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")

    print(f"\n===== flow test finished: {summary['okCount']}/{summary['totalCount']} ok in {elapsed}s =====", flush=True)
    print(f"report : {report}", flush=True)
    print(f"ledger : {ledger}", flush=True)
    if FAILURES:
        print("FAILURES:", flush=True)
        for failure in FAILURES:
            print(f"  - {failure}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
