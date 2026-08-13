---
name: rh-export-evidence
description: Export a run into a self-contained evidence bundle (manifest with hashes, run record, telemetry, charts, diagnostics) so results are reproducible outside the workspace.
whenToUse: Use when a run should be archived, shared, or included in a report/paper, or when the user asks for an evidence bundle.
modelInvocable: true
userInvocable: true
---

# Export a reproducible evidence bundle

1. **Resolve the run.** runPath = run directory or run.json.
2. **Export.** `rh_evidence_export` with runPath and an outDir (e.g. `<workspace>/.rh/bundles/<runId>`).
3. **Verify.** Read the returned manifest: it must contain the run id, scenario, success flag, environment snapshot, and a files list with sha256 hashes for run.json, telemetry.jsonl and diagnostics.json (plus charts when available).
4. **Report to the user.** State the bundle directory and what it contains. Note that the bundle is self-contained: it references nothing outside itself, so it can be moved or archived as a unit.

## Rules

- Never copy a bundle into the session as an attachment; report the path instead.
- If the user plans to publish results, remind them that simulation results are not real-robot evidence.
