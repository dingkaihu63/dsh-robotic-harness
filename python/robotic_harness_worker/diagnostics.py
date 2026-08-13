"""Deterministic, evidence-based diagnostics for Robotic Harness runs.

The engine separates three layers exactly as the plan requires:

- **facts** — values taken from the run record and telemetry, each with a
  timestamp or location;
- **rule findings** — deterministic checks over the facts (thresholds,
  durations, state-machine inconsistencies);
- **hypotheses** — candidate root causes grouped by fault layer, each with
  supporting evidence, counter-evidence, missing evidence and suggested
  read-only checks.

The engine never produces a verdict: the final conclusion is always left to a
human, and every hypothesis is explicitly labeled with its likelihood.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Optional

from .core import DiagnosticCase, Finding, Hypothesis, Run, new_id

# Rule thresholds (deterministic layer-1 detection, per plan section 13.4).
TRACKING_ERROR_ALERT = 0.06  # rad, sustained per-joint
PERCEPTION_OFFSET_ALERT_M = 0.02
SLIP_WINDOW_S = 1.5  # slip considered "during carry" within this many seconds of lift


def load_run_data(run_path: str) -> tuple[Run, list[dict[str, Any]]]:
    """Load run.json plus telemetry.jsonl from a run directory or run.json path."""
    run_dir = run_path
    if os.path.isfile(run_path):
        run_dir = os.path.dirname(run_path)
    with open(os.path.join(run_dir, "run.json"), encoding="utf-8") as handle:
        run = Run.from_dict(json.load(handle))
    telemetry: list[dict[str, Any]] = []
    telemetry_path = os.path.join(run_dir, "telemetry.jsonl")
    if os.path.exists(telemetry_path):
        with open(telemetry_path, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if line:
                    telemetry.append(json.loads(line))
    return run, telemetry


def _tracking_stats(telemetry: list[dict[str, Any]]) -> dict[str, Any]:
    per_joint: list[list[float]] = [[] for _ in range(3)]
    for row in telemetry:
        for index, error in enumerate(row.get("trackingError", [])):
            if index < 3:
                per_joint[index].append(float(error))
    stats = {}
    for index, name in enumerate(["shoulder", "elbow", "wrist"]):
        values = per_joint[index]
        stats[name] = {
            "rms": math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0,
            "max": max(values) if values else 0.0,
            "sustainedOverAlert": sum(1 for v in values if v > TRACKING_ERROR_ALERT),
        }
    return stats


def diagnose(run: Run, telemetry: list[dict[str, Any]]) -> DiagnosticCase:
    """Run the rule engine over a run and build a DiagnosticCase."""
    case = DiagnosticCase(
        id=new_id("case"),
        run_id=run.id,
        symptom=_symptom(run),
    )
    metrics = run.metrics
    config = run.config
    fault = config.get("fault", {})
    anomalies = run.anomalies

    # ------------------------------------------------------------------ facts
    if telemetry:
        case.findings.append(
            Finding(
                origin="fact",
                title="telemetry coverage",
                detail=(
                    f"{len(telemetry)} samples over {telemetry[0]['t']:.2f}s..{telemetry[-1]['t']:.2f}s "
                    f"({run.metrics.get('steps', 0)} physics steps)"
                ),
                evidence=[f"first={telemetry[0]['t']:.3f}s last={telemetry[-1]['t']:.3f}s"],
            )
        )
    case.findings.append(
        Finding(
            origin="fact",
            title="run outcome",
            detail=f"state={run.state}, success={metrics.get('success')}",
            evidence=[f"objectFinal={metrics.get('objectFinal')}", f"targetZone={metrics.get('targetZone', {}).get('center')}"],
        )
    )
    case.findings.append(
        Finding(
            origin="fact",
            title="perception estimate vs ground truth",
            detail=(
                f"route={metrics.get('perceptionRoute')}, renderer={metrics.get('renderer')}, "
                f"estimate={metrics.get('perceptionEstimate')}, true={metrics.get('perceptionTrue')}"
            ),
            evidence=[f"offset={_perception_offset(metrics):.4f} m"],
        )
    )
    case.findings.append(
        Finding(
            origin="fact",
            title="fault configuration",
            detail=json.dumps(fault, ensure_ascii=False),
            evidence=["fault configuration is part of the run snapshot"],
        )
    )
    for anomaly in run.anomalies:
        case.findings.append(
            Finding(
                origin="fact",
                title=f"anomaly: {anomaly.kind}",
                detail=anomaly.detail,
                evidence=[f"t={anomaly.time_s:.3f}s", json.dumps(anomaly.evidence, ensure_ascii=False)],
                time_s=anomaly.time_s,
            )
        )

    # ------------------------------------------------------------------ rules
    tracking = _tracking_stats(telemetry)
    for name, stats in tracking.items():
        if stats["sustainedOverAlert"] > 10:
            case.findings.append(
                Finding(
                    origin="rule",
                    title=f"rule: sustained tracking error on {name}",
                    detail=f"{stats['sustainedOverAlert']} samples above {TRACKING_ERROR_ALERT} rad",
                    evidence=[f"rms={stats['rms']:.4f} rad", f"max={stats['max']:.4f} rad"],
                )
            )
    offset = _perception_offset(metrics)
    if offset > PERCEPTION_OFFSET_ALERT_M:
        case.findings.append(
            Finding(
                origin="rule",
                title="rule: perception estimate diverged from ground truth",
                detail=f"estimated object position is {offset * 1000:.1f} mm away from ground truth",
                evidence=[f"threshold={PERCEPTION_OFFSET_ALERT_M * 1000:.0f} mm"],
            )
        )
    slip_times = [a.time_s for a in anomalies if a.kind == "gripper_slip"]
    grasp_times = [a.time_s for a in anomalies if a.kind == "suction_engaged"]
    if slip_times:
        case.findings.append(
            Finding(
                origin="rule",
                title="rule: object was lost during transport",
                detail=f"slip detected at t={slip_times[0]:.3f}s",
                evidence=[f"suction engaged at t={grasp_times[0]:.3f}s" if grasp_times else "no suction engagement recorded"],
            )
        )
    if not metrics.get("grasped") and not any(a.kind == "perception_failed" for a in anomalies):
        case.findings.append(
            Finding(
                origin="rule",
                title="rule: grasp never engaged although perception succeeded",
                detail="suction did not engage; check approach geometry and grasp radius",
            )
        )
    if metrics.get("success") is False and run.state == "completed":
        case.findings.append(
            Finding(
                origin="rule",
                title="rule: completed run failed the success criteria",
                detail=f"inTargetZone={metrics.get('inTargetZone')}, grasped={metrics.get('grasped')}, slipped={metrics.get('slipped')}",
            )
        )

    # ------------------------------------------------------------ hypotheses
    hypotheses: list[Hypothesis] = []

    if offset > PERCEPTION_OFFSET_ALERT_M:
        hypotheses.append(
            Hypothesis(
                id="h-perception",
                layer="perception",
                title="perception error caused the grasp to miss the object",
                support=[
                    f"perception estimate deviates {offset * 1000:.1f} mm from ground truth",
                    "the grasp target is computed from the perception estimate",
                ],
                counter_evidence=[] if not metrics.get("grasped") else ["suction still engaged, so the grasp succeeded despite the offset"],
                missing_evidence=["per-frame segmentation masks around the grasp moment", "camera intrinsics/ex-trinsics calibration record"],
                suggested_checks=[
                    "inspect the perception record's centroid vs the rendered object pixel position",
                    "re-run with perception_offset_px=0 and compare metrics",
                ],
                likelihood="high" if not metrics.get("grasped") else "medium",
            )
        )
    if slip_times:
        hypotheses.append(
            Hypothesis(
                id="h-mechanical",
                layer="mechanical",
                title="gripper/object interface lost the object (slip)",
                support=[
                    f"slip anomaly at t={slip_times[0]:.3f}s",
                    "suction was engaged before the slip" if grasp_times else "no suction engagement evidence",
                ],
                counter_evidence=[],
                missing_evidence=["fingertip/suction pressure telemetry", "object-contact force curve"],
                suggested_checks=[
                    "review the injected fault: gripper_slip was enabled in the run config",
                    "inspect cup and object friction parameters in the scenario",
                ],
                likelihood="high" if fault.get("gripper_slip") else "medium",
                requires_human=True,
            )
        )
    worst_tracking = max(tracking.values(), key=lambda s: s["rms"])
    if worst_tracking["rms"] > 0.02:
        hypotheses.append(
            Hypothesis(
                id="h-control",
                layer="control",
                title="control tracking error contributed to the outcome",
                support=[
                    f"{max(tracking, key=lambda k: tracking[k]['rms'])} joint RMS error {worst_tracking['rms']:.4f} rad",
                    f"{worst_tracking['sustainedOverAlert']} samples above the alert threshold",
                ],
                counter_evidence=[],
                missing_evidence=["controller gains and commanded trajectory from the same window"],
                suggested_checks=["plot joints.png: compare target vs actual curves", "lower servo gains and re-run"],
                likelihood="low",
            )
        )
    if fault.get("tf_offset", [0.0, 0.0]) != [0.0, 0.0] and any(
        h.layer == "perception" for h in hypotheses
    ):
        hypotheses.append(
            Hypothesis(
                id="h-calibration",
                layer="calibration",
                title="TF/calibration offset shifted the commanded grasp pose",
                support=[
                    f"tf_offset fault was active: {fault.get('tf_offset')}",
                    "the perception estimate is expressed in the robot frame, so a TF error shifts the grasp target",
                ],
                counter_evidence=[],
                missing_evidence=["TF tree snapshot at run time"],
                suggested_checks=["compare the commanded wrist pose with the ground-truth grasp pose"],
                likelihood="medium",
            )
        )
    if not hypotheses:
        hypotheses.append(
            Hypothesis(
                id="h-unexplained",
                layer="system",
                title="no rule-based hypothesis explains the outcome",
                support=["all deterministic checks passed or produced no signal"],
                counter_evidence=[],
                missing_evidence=["manual review of the run artifacts"],
                suggested_checks=["open the timeline and the report; inspect anomalies and charts"],
                likelihood="unknown",
                requires_human=True,
            )
        )

    case.hypotheses = hypotheses
    return case


def _perception_offset(metrics: dict[str, Any]) -> float:
    estimate = metrics.get("perceptionEstimate")
    truth = metrics.get("perceptionTrue")
    if not estimate or not truth:
        return 0.0
    return math.hypot(estimate[0] - truth[0], estimate[2] - truth[2])


def _symptom(run: Run) -> str:
    if run.state == "failed":
        return "run aborted: " + (run.anomalies[-1].detail if run.anomalies else "unknown")
    if not run.metrics.get("success"):
        return f"run completed but success criteria failed (inTargetZone={run.metrics.get('inTargetZone')}, grasped={run.metrics.get('grasped')}, slipped={run.metrics.get('slipped')})"
    return "run succeeded; diagnostics requested for review"
