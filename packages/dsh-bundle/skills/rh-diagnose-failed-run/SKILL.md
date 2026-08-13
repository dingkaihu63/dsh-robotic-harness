---
name: rh-diagnose-failed-run
description: Diagnose a failed Robotic Harness run — load the run record, run the deterministic rule engine, and present facts, rule findings, candidate root causes, missing evidence and read-only checks.
whenToUse: Use when a run failed (or succeeded but missed its success criteria) and the user wants to understand why.
modelInvocable: true
userInvocable: true
---

# Diagnose a failed run

1. **Load the run.** Resolve the run directory (contains run.json + telemetry.jsonl) or run.json path.
2. **Read the outcome.** `rh_diagnose_run` with the runPath. It returns the diagnostic case.
3. **Present in three layers, never mixed:**
   - **Facts** — timestamps, metrics, perception estimate vs ground truth, fault configuration, anomalies. These are data, not opinions.
   - **Rule findings** — deterministic checks (tracking error above threshold, perception divergence, slip detected, grasp never engaged). Label them as `rule`.
   - **Hypotheses** — candidate root causes grouped by layer (perception/calibration/mechanical/control/system) with support, counter-evidence, missing evidence and suggested checks. Label likelihood and mark them as unconfirmed.
4. **Recommend next steps.** Prefer the suggested read-only checks; propose re-running with a fault configuration changed when the fault is the suspected cause.
5. **Be explicit about what is missing.** If telemetry lacks the evidence needed to confirm a hypothesis, say exactly what artifact would confirm it.

## Rules

- Never present a hypothesis as a verdict. The final conclusion belongs to a human.
- Cite evidence (values, timestamps, codes) for every claim; "probably" without evidence is not allowed.
- If the run succeeded, say so and still report the diagnostics for review.
