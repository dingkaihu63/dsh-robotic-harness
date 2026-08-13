---
name: rh-pick-place-demo
description: Run the Robotic Harness MuJoCo pick-place demo end to end — inspect the demo arm, run a clean run, run a fault-injected run, diagnose the failure, and export an evidence bundle plus report.
whenToUse: Use when the user asks to run the demo, reproduce the README workflow, or see the happy path and a failure path with evidence.
modelInvocable: true
userInvocable: true
---

# Run the pick-place demo

The demo proves the full loop: asset → simulation → failure → evidence. Run the steps in order and keep every tool result as evidence.

1. **Check the environment.** `rh_sim_status` — record mujoco availability and renderer mode (offscreen vs simulated perception).
2. **Inspect the demo arm.** `rh_robot_asset_inspect` on the bundled `rh_arm.urdf` fixture (ask the user for the path, or use the one printed by the bundle's README). Record the issue counts.
3. **Run the happy path.** `rh_sim_run` with seed 42 and no fault. Note `success`, `metrics.objectFinal`, and the artifacts (runDir, telemetry.jsonl, charts).
4. **Run a fault-injected run.** `rh_sim_run` with `{fault: {perceptionOffsetPx: [18, 6], gripperSlip: true, tfOffset: [0.015, 0]}, seed: 43}`. Note the anomalies.
5. **Diagnose the failure.** `rh_diagnose_run` on the fault run's runDir. Summarize facts vs rule findings vs hypotheses; always mark hypotheses as unconfirmed.
6. **Export evidence.** `rh_evidence_export` for the fault run into a directory like `<workspace>/.rh/demo/bundle-<runId>`.
7. **Generate the report.** `rh_report_generate` for the fault run; mention the timeline.html viewer (openable without a server).

## Success criteria

- The happy run succeeds (success=true) and the fault run fails with at least one anomaly.
- The diagnostics contain at least one hypothesis whose support cites the fault configuration or telemetry.
- The evidence bundle manifest exists and lists run.json, telemetry.jsonl and diagnostics.json with sha256 hashes.

## Stop conditions

- If `rh_sim_status` reports mujoco unavailable, stop and tell the user which Python environment needs `mujoco` (+ `numpy`, `opencv-python`, `matplotlib` for full output).
- If a run errors, show the error and do not fabricate results.
