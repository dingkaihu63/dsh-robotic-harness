---
name: rh-inspect-robot-asset
description: Inspect and validate a robot model (URDF/MJCF) — links, joints, inertials, collisions, tree structure — and report issues with severity before any simulation or conversion work.
whenToUse: Use when the user asks to check a robot model, before converting assets or running simulations, or when a simulation/planning problem may come from the model itself.
modelInvocable: true
userInvocable: true
---

# Inspect a robot asset

Fixed check order — do not skip steps; record each step's result as evidence.

1. **Locate the asset.** Resolve the model path (`.urdf`, expanded `.xacro`, `.xml`/`.mjcf`). If the user gave a `.xacro`, expand it first (e.g. `xacro --inorder model.xacro > model.urdf`).
2. **Check the backend.** Call `rh_sim_status` (or `rh_worker_ping`) to record whether mujoco/opencv are available — this explains what the later steps can actually load.
3. **Inspect the asset.** Call `rh_robot_asset_inspect` with the absolute path.
4. **Validate.** Call `rh_urdf_validate` for URDF inputs.
5. **Interpret.** For every `error` issue, state the exact code and location. Distinguish:
   - facts (what the file contains, e.g. mass values);
   - rule findings (e.g. inertia not positive-definite, missing limit);
   - inferences (what it probably means for simulation/planning — label as such).
6. **Report.** Summarize ok/error/warning counts, root link, joint types, and the single most important issue. Do not silently skip issues. If the asset is clean, say so explicitly.

## Stop conditions

- Stop and ask the user when the path does not exist or the format is unsupported.
- Never modify the asset while inspecting; conversion is a separate explicit step (`rh_urdf_to_mjcf`).

## Common pitfalls

- Do not claim a model is "simulation-ready" — inspection only proves structure and inertials are plausible.
- A missing `package://` mesh URI is a portability warning, not a hard error by itself.
