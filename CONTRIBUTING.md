# Contributing to Robotic Harness

Thanks for considering a contribution! Robotic Harness is an open, modular
plugin suite for DeepSeek Harness — the project welcomes contributors who want
to grow the robotics domain layer together.

## Code of conduct

Be respectful, evidence-first and honest about limitations. This project
explicitly values "demo before promise": working, reproducible results matter
more than ambitious claims.

## What we welcome

- New Capability adapters (perception, policy, simulation backends);
- New Skills (`packages/dsh-bundle/skills/<name>/SKILL.md`);
- New simulation scenarios (`scenarios/`);
- New Failure Cases with known root causes and evidence (see `fixtures/`);
- New data importers/exporters and quality rules (`python/.../data_quality.py`);
- New fixtures (rosbag/robot assets with permissive licenses);
- Documentation, translations, and DSH compatibility tests;
- The ROS 2 read-only bridge (see `packages/dsh-bundle/skills/rh-ros2-health-check`).

## Development environment

- **Node** ≥ 22.19 with pnpm (the repo pins nothing globally; run
  `pnpm install` at the root).
- **Python** ≥ 3.10. The demo environment used by maintainers is an Anaconda
  `python3.10` env with `mujoco`, `numpy`, `opencv-python`, `matplotlib`,
  `pytest`. All caches/stores in this repo are configured to live on non-C
  drives for contributors who care about disk hygiene (see `.gitignore`).

### Layout

```text
packages/dsh-bundle/     the installable DSH bundle (TS plugins + skills + worker copy)
python/                  the Python worker package (robotic_harness_worker) + tests
fixtures/                robot assets (URDF fixtures)
scenarios/               MuJoCo scenario definitions
scripts/                 sync-worker, demo, smoke helpers
docs/                    architecture, safety boundary, roadmap
```

The bundle ships a **copy** of the Python worker under
`packages/dsh-bundle/worker`. After changing `python/`, run:

```sh
node scripts/sync-worker.mjs   # copies worker + fixtures + scenarios into the bundle
pnpm --filter @robotic-harness/dsh-bundle build
```

## Testing

```sh
# Python worker tests (from python/)
python -m pytest tests -q

# TypeScript build + typecheck
pnpm --filter @robotic-harness/dsh-bundle build
pnpm --filter @robotic-harness/dsh-bundle typecheck

# One-command demo (happy + fault + diagnostics + evidence + report)
PYTHON=<your-python3.10> node scripts/demo.mjs

# Bundle -> worker smoke test
node scripts/smoke-worker.mjs --python <your-python3.10>
```

## Contribution workflow

1. Open an issue describing the problem/idea, or pick a `good first issue`.
2. Fork, branch (`feat/<name>` or `fix/<name>`), implement with tests.
3. Keep the demo green: run the tests above.
4. Open a pull request; maintainers review for scope, evidence and honesty of
   claims (no "works on real robots" claims without real-robot evidence).

## Skill authoring requirements

Every Skill must state: applicable problem, non-goals, inputs/preconditions,
the tools it uses, a fixed check order, approval requirements, success
criteria, stop conditions, output artifacts, common misjudgments, and at least
one runnable example. See existing skills under
`packages/dsh-bundle/skills/`.

## Licensing

By contributing you agree that your contributions are licensed under the
MIT License (see LICENSE). Third-party assets must carry their own licenses
and must not be redistributed without permission (see
THIRD_PARTY_NOTICES.md).
