---
name: rh-run-pick-place-benchmark
description: Run a small seed/config matrix of pick-place simulations and compare success rates, grasp outcomes and anomaly patterns across configurations.
whenToUse: Use when the user wants to compare configurations, study fault sensitivity, or reproduce a mini benchmark.
modelInvocable: true
userInvocable: true
---

# Run a pick-place benchmark

1. **Define the matrix.** Propose a small matrix (2-4 cells is enough for a demo), for example:
   - clean baseline (no faults);
   - perception offset only;
   - gripper slip only;
   - perception offset + slip.
   Use 2-3 seeds per cell (e.g. 42, 43, 44).
2. **Run each cell.** `rh_sim_run` with the cell's fault config and seed. Record `success`, `grasped`, `slipped`, `inTargetZone` and `trackingErrorRms` per run.
3. **Compare.** Build a table: cell | seeds | success rate | typical anomaly. Identify which fault breaks which outcome.
4. **Diagnose the worst cell.** `rh_diagnose_run` on one representative failed run and summarize the hypotheses.
5. **Report.** State clearly that the numbers are simulation-only and not a safety statement.

## Rules

- Keep the matrix small; this is a demo, not a full experiment.
- Record every run's id so the user can open the evidence later.
