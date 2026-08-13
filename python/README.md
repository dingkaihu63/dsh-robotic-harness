# robotic-harness-worker

Python sidecar for the Robotic Harness DeepSeek Harness plugin suite.

The worker is intentionally dependency-light: the core asset checks, data
audits, diagnostics and reports run on the Python standard library alone;
simulation (MuJoCo), perception (OpenCV) and charts (matplotlib) are optional
extras.

## Commands

All commands are one-shot JSON-in / JSON-out:

```sh
python -m robotic_harness_worker <command> --input <file|-> [--output <file>]
```

| Command | Description |
|---|---|
| `ping` | Worker version + environment snapshot |
| `capability-list` | Capability manifest (id/kind/risk) |
| `inspect-asset` | URDF/MJCF inspection report with graded issues |
| `validate-urdf` | URDF validation verdict |
| `convert-urdf` | URDF → MJCF via the MuJoCo compiler |
| `sim-status` | MuJoCo / renderer / scenario availability |
| `sim-validate-scenario` | Scenario reachability and parameter validation |
| `sim-run` | One MuJoCo pick-place run (fault injection supported) |
| `diagnose-run` | Deterministic diagnostics: facts, rules, hypotheses |
| `evidence-export` | Self-contained evidence bundle (manifest + hashes) |
| `report-generate` | Markdown report + standalone timeline.html |
| `data-quality` | CSV/JSONL timeseries quality audit |
| `demo` | Full loop: happy + fault runs, diagnostics, evidence, reports |

Run `python -m robotic_harness_worker demo --input -` with
`{"storeRoot": "...", "demoDir": "..."}` for the end-to-end demo.

## Tests

```sh
python -m pytest tests -q
```

Simulation tests are skipped automatically when MuJoCo is missing.

## Layout

```text
robotic_harness_worker/
├── core.py           domain models + on-disk run store
├── assets.py         URDF/MJCF inspection, validation, conversion
├── simulation.py     MuJoCo pick-place environment, policy, fault injection
├── vision.py         color segmentation + saliency fallback + rule router
├── diagnostics.py    rule engine (facts / rules / hypotheses)
├── data_quality.py   CSV/JSONL quality audit
├── report.py         evidence bundles, Markdown reports, timeline HTML
└── cli.py            stdio JSON protocol
```

## License

MIT.
