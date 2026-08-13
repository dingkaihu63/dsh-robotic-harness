# Robotic Harness (RH) — an embodied-intelligence research plugin suite for DeepSeek Harness (Demo)

> An experimental DSH plugin suite that puts robot assets, simulation, capability orchestration and failure evidence into one Agent workflow.
> The current reference implementation covers a **MuJoCo pick-and-place demo** (asset inspection → simulation → fault injection → telemetry → rule-based diagnostics → evidence export → report).
> ROS 2, CAD, vision, control and VLA developers are welcome to help define the next interface.

> **Testers & contributors**: this project is at an early **Demo** stage. It has been validated only in a limited local environment (Windows + Anaconda Python 3.10 + DSH 0.1.0-rc.6). ROS 2, CAD, real-robot and other OS/hardware setups have **not been fully tested** — please forgive rough edges and report any issues you hit.
> Everyone is **welcome to test, modify and extend** this plugin suite, and to combine your own robot-related plugins (ROS 2 / CAD / vision / control / VLA / ...) with this suite to **build one bigger, complete robot plugin bundle together** — every independent module can be published and contributed separately. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Scope & status (read before you test)

| Module | Status in this repo |
|---|---|
| Robot asset inspection (URDF/MJCF: links, joints, inertials, collisions) | ✅ v0.1 demo |
| URDF validation & URDF → MJCF conversion | ✅ v0.1 demo |
| MuJoCo pick-place simulation + fault injection | ✅ v0.1 demo |
| Perception routing (color segmentation → generic saliency) | ✅ v0.1 demo (minimal) |
| Deterministic diagnostics (facts / rules / hypotheses) | ✅ v0.1 demo |
| Data quality audit (CSV/JSONL timeseries) | ✅ v0.1 demo (subset) |
| Evidence bundles, Markdown reports, standalone timeline.html | ✅ v0.1 demo |
| ROS 2 read-only diagnostics (graph/TF/QoS/rosbag) | ⏳ roadmap — only a Skill template (`rh-ros2-health-check`) exists |
| CAD / SolidWorks / FreeCAD | ⏳ roadmap |
| Camera calibration, detection, segmentation, 6D pose | ⏳ roadmap |
| Control analysis (PID/trajectory compare, sys-id) | ⏳ roadmap |
| VLA / VLM model adapters | ⏳ roadmap |
| Live telemetry web dashboard | ⏳ roadmap (timeline.html is the single-file viewer for now) |
| Human demonstration data & privacy pipeline | ⏳ roadmap (requires compliant data) |
| Real-robot experiments | ❌ not in scope for v0.1 — see [docs/safety-boundary.md](docs/safety-boundary.md) |

So: **not everything in the product plan is implemented yet — by design.** The plan document is a full product map; this repo follows its "demo-first, progressive scope" principle and delivers the v0.1 demo slice. The [roadmap](docs/roadmap.md) lists the next phases, and each missing module is an entry point for contributors.

## 30-second architecture

```text
DSH profile (rh-demo)
  └─ @robotic-harness/dsh-bundle (this repo)
       ├─ rh-core    project/run store root (.rh/ layout, all inside the workspace)
       ├─ rh-tools   12 robot-domain tools (asset/simulation/diagnostics/data)
       └─ rh-skills  6 SKILL.md files (inspect-asset / pick-place-demo / diagnose / benchmark / evidence / data-quality)
              │  stdio JSON (one-shot process)
              ▼
       robotic_harness_worker (Python 3.10, shipped inside the bundle)
        ├─ assets       URDF/MJCF inspection, inertia validation, URDF→MJCF conversion
        ├─ simulation   MuJoCo planar 3-DOF suction-cup pick-place + fault injection
        ├─ vision       color segmentation → generic segmentation (rule routing)
        ├─ diagnostics  deterministic rule engine (facts / rules / inferences)
        └─ data         CSV/JSONL timeseries quality audit
```

## One-command demo (no DSH needed, pure Python)

Requirements: Python ≥ 3.10 with `mujoco`, `numpy`, `opencv-python`, `matplotlib`, `pytest` (the Anaconda `python3.10` env is recommended).

```sh
# 1) unit tests
cd python && python -m pytest tests -q

# 2) end-to-end demo: happy run + fault run + diagnostics + evidence + report
PYTHON=<your python3.10> node scripts/demo.mjs
# Output lands in examples/demo-output/:
#   report-run-*.md           experiment report (evidence + hypotheses)
#   timeline-run-*.html       standalone timeline viewer (open in a browser, no server)
#   bundle-run-*/             self-contained evidence bundle (manifest + hashes + telemetry + charts)
```

## Install as a DSH plugin (bundle)

Requirements: DSH CLI (`@deepseek-ai/dsh` ≥ 0.1.0-rc.6), pnpm, a Python 3.10 environment.

```sh
# 0) environment (example: keep everything on the F: drive, DSH_HOME on F:)
export DSH_HOME=/f/dsh/.dsh-home
export PATH="/f/dsh/.tools:$PATH"            # directory containing pnpm

# 1) create a profile and install the bundle
dsh plugin --profile rh-demo add ./packages/dsh-bundle

# 2) enable the Web UI. Note: the upstream npm release of @deepseek-ai/dsh-web-app
#    depends on the private package @deepseek-ai/dsh-frontend (registry 404), so
#    `pnpm add` fails. Built-in bundles resolve from the dsh install directory,
#    so edit the profile manifest instead:
#    set dsh.profile.bundles in $DSH_HOME/profiles/rh-demo/package.json to
#    ["@deepseek-ai/dsh-base", "@deepseek-ai/dsh-web-app", "@robotic-harness/dsh-bundle"]
#    (save as UTF-8 without BOM)

# 3) (machine-local example) in the profile's cordis.patch.yml point
#    rh-tools.pythonPath at your Anaconda python3.10 interpreter
#    (a patch replaces the whole row config, so restate every key)

# 4) start the Web UI
dsh --profile rh-demo --port 3080
```

After installation the Agent has the 12 `rh_*` tools and 6 Skills. For example, ask the Agent:

> "Run the Robotic Harness pick-place demo: inspect the demo arm, run one clean simulation and one fault-injected simulation, diagnose the failure, export the evidence bundle and generate the report."

Tools invoke the Python worker (`python -m robotic_harness_worker <command> --input -`); all runs, telemetry, charts and reports are written to the workspace's `.rh/` directory by default.

### Tool list

| Tool | Risk | Description |
|---|---|---|
| `rh_worker_ping` | R0 | worker health & dependency versions |
| `rh_capability_list` | R0 | capability manifest (asset/simulation/perception/policy/diagnostics/data) |
| `rh_robot_asset_inspect` | R0 | URDF/MJCF structural inspection (links/joints/inertials/collisions + graded issues) |
| `rh_urdf_validate` | R0 | URDF validation (tree, positive-definite inertia, limits, axes, mesh paths) |
| `rh_urdf_to_mjcf` | R1 | controlled URDF→MJCF conversion (MuJoCo compiler + difference report) |
| `rh_sim_status` | R0 | MuJoCo / renderer / scenario availability |
| `rh_sim_validate_scenario` | R0 | scenario reachability & parameter validation |
| `rh_sim_run` | R2 | MuJoCo pick-place run (faults: perception offset / gripper slip / TF offset / sensor noise / occlusion / model timeout) |
| `rh_diagnose_run` | R0 | deterministic diagnostics: facts / rules / candidate root causes (evidence + counter-evidence + missing evidence + suggested checks) |
| `rh_evidence_export` | R1 | self-contained evidence bundle (manifest + sha256 + telemetry + charts) |
| `rh_report_generate` | R1 | Markdown report + standalone timeline.html |
| `rh_data_quality` | R0 | CSV/JSONL timeseries audit (missing/NaN/out-of-order/duplicates/gaps/constant channels) |

## Demo contents (v0.1)

- **Scenario**: planar 3-DOF arm with a suction cup; picks a red box from the table and places it into a target zone (MuJoCo, built from primitives, no external meshes).
- **Perception routing**: color segmentation (low latency) → generic saliency segmentation on failure/occlusion; the routing reason is recorded.
- **Fault injection** (deterministic, seed-controlled): `perception_offset_px`, `gripper_slip`, `tf_offset`, `sensor_noise`, `model_timeout_s`, `occlusion`.
- **Telemetry**: joint target/actual/error, suction state, object pose, perception estimate vs ground truth; charts (joints/tracking/trajectory) and a scene render.
- **Diagnostics**: the rule engine produces layered evidence — facts (with timestamps and values), rule findings (thresholds/state machine), candidate root causes (grouped by perception/calibration/mechanical/control/system layer, with likelihood and missing evidence). **The final conclusion is left to a human.**
- **Evidence**: self-contained evidence bundle (hash manifest + all records) + Markdown report + timeline.html.

### Known limitations (stated honestly)

- The suction grasp is a **kinematic implementation** (the object follows the cup while attached), noted in the run config and reports.
- Perception uses real offscreen rendering when the renderer is available; otherwise it degrades to ground-truth + noise simulation (recorded in telemetry).
- Simulation results are not real-robot evidence; there is no arbitrary topic-publish, real-robot write, or e-stop-release capability.
- ROS 2 / SolidWorks / Isaac / real-robot adapters are not implemented yet (see [docs/roadmap.md](docs/roadmap.md)).

## Repository layout

```text
packages/dsh-bundle/   the installable DSH bundle (TS plugins, skills/, worker copy, fixtures, scenarios)
python/                the robotic_harness_worker Python package + pytest tests
fixtures/              URDF test assets (valid + deliberately broken)
scenarios/             MuJoCo scenario definitions (JSON)
scripts/               sync-worker / demo / smoke-worker
docs/                  architecture, safety boundary, roadmap, demo guide
examples/demo-output/  sample one-command demo output
```

## Documentation

- [Architecture & domain model](docs/architecture.md)
- [Safety boundary](docs/safety-boundary.md)
- [Roadmap](docs/roadmap.md)
- [Demo guide](docs/demo.md)
- [Contributing](CONTRIBUTING.md) · [Security policy](SECURITY.md) · [Third-party notices](THIRD_PARTY_NOTICES.md)
- 中文文档：[README.zh.md](README.zh.md)

## License

MIT. Third-party components and assets carry their own licenses (see THIRD_PARTY_NOTICES.md).
This repository is not affiliated with DeepSeek; DSH is a separate project (MIT, [deepseek-harness](https://github.com/deepseek-ai/deepseek-harness)).
