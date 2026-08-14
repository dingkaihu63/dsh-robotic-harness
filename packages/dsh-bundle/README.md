# @robotic-harness/dsh-bundle

Robotic Harness — an embodied-intelligence research plugin suite for DeepSeek Harness.

This package is the installable DSH bundle: it contributes the `rh-core` /
`rh-tools` / `rh-skills` plugin rows (see `cordis.patch.yml`), ~100 `rh_*`
tools, 25 Skills, and a self-contained Python worker copy under `worker/`.

## Install

```sh
dsh plugin --profile rh-demo add @robotic-harness/dsh-bundle
```

Or from a tarball / git checkout — see `docs/publishing.md` in the repository
root. After install, point `rh-tools.pythonPath` at a Python 3.10 interpreter
that has `mujoco`, `numpy`, `opencv-python`, `matplotlib` (see the root README).

## Layout inside the package

```text
lib/              compiled TypeScript plugins (core/tools/skills)
cordis.patch.yml  the bundle patch layer
skills/           25 SKILL.md files
worker/           robotic_harness_worker (Python sidecar, synced from python/)
fixtures/         URDF/SDF test assets + demo rosbag2
scenarios/        MuJoCo scenario definitions
```

The package is always packed with `prepack` (sync worker + build), so a
published/tarball install is self-contained and matches the `python/` sources.
