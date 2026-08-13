# Robotic Harness (RH) — an embodied-intelligence research plugin suite for DeepSeek Harness (Demo)

> An experimental DSH plugin suite that puts robot assets, simulation, capability orchestration and failure evidence into one Agent workflow.
> The current reference implementation covers a **MuJoCo pick-and-place demo** (asset inspection → simulation → fault injection → telemetry → rule-based diagnostics → evidence export → report).
> ROS 2, CAD, vision, control and VLA developers are welcome to help define the next interface.

> **Testers & contributors**: this project is at an early **Demo** stage. It has been validated only in a limited local environment (Windows + Anaconda Python 3.10 + DSH 0.1.0-rc.6). ROS 2, CAD, real-robot and other OS/hardware setups have **not been fully tested** — please forgive rough edges and report any issues you hit.
> Everyone is **welcome to test, modify and extend** this plugin suite, and to combine your own robot-related plugins (ROS 2 / CAD / vision / control / VLA / ...) with this suite to **build one bigger, complete robot plugin bundle together** — every independent module can be published and contributed separately. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Scope & status (read before you test)

| Module | Status in this repo |
|---|---|
| Robot asset inspection (URDF/MJCF/SDF: links, joints, inertials, collisions) | ✅ implemented |
| CAD: inventory, version compare, mesh inspection, inertia/topology validation, SVG preview, URDF→MJCF / SDF-compat export, asset reports | ✅ implemented (SolidWorks files registered, not parsed) |
| URDF validation & URDF → MJCF conversion | ✅ implemented |
| MuJoCo pick-place simulation + fault injection + batch benchmarks + read-only replay + sim-vs-real gap report | ✅ implemented |
| Perception routing (color segmentation → generic saliency) + camera health/calibration inspection + pose checks + perception comparison | ✅ implemented |
| Deterministic diagnostics (facts / rules / hypotheses) + telemetry channels/windows/anomaly scan/evidence collection/run compare | ✅ implemented |
| Data pipeline: inventory, schema, time-sync, alignment, transforms, episodes, annotations, leakage-safe splits, de-identification, rosbag conversion, LeRobot export, dataset versions/cards | ✅ implemented (RLDS = manifest; parquet optional) |
| Experiment management: spec, matrix, benchmark, metrics, ablation, reports | ✅ implemented |
| Control analysis: trace metrics, trajectory validation, planned-vs-actual, PID templates/config compare, system identification | ✅ implemented |
| Embodied model registry: builtin demo models run for real; external/heavy models probed and reported honestly | ✅ implemented (adapter pattern) |
| Knowledge: docs index/search, error-code lookup, case search | ✅ implemented |
| Real-robot experiment state machine + preflight | ✅ implemented as state machine; hardware items are skipped (not faked) without an adapter |
| ROS 2 read-only diagnostics (graph/TF/QoS/controllers) | 🔌 adapter implemented; live probes require the `ros2` CLI; **rosbag2 inspection works without ROS** |
| Single-file dashboard & timeline viewers | ✅ implemented (static snapshot, not real-time) |
| Live DSH web client plugin panels | ⏳ roadmap (static HTML viewers for now) |
| Human demonstration data & privacy pipeline | ✅ implemented (de-identification toolkit); compliant datasets required |
| SolidWorks API / FreeCAD deep integration, Isaac, real-robot adapters | ⏳ roadmap |

So: the full plan's tool/skill surface is implemented as **demo-grade adapters** — pure-software modules are complete and tested; hardware/backend-dependent modules (ROS 2 live, SolidWorks, real robot, heavy VLA) exist as honest adapters that report `backend: "unavailable"` with install instructions instead of pretending. See [docs/tool-inventory.md](docs/tool-inventory.md) for the full 100-tool list and [docs/roadmap.md](docs/roadmap.md) for what still needs real hardware/backends to validate.

## 30-second architecture

```text
DSH profile (rh-demo)
  └─ @robotic-harness/dsh-bundle (this repo)
       ├─ rh-core    project/run store root (.rh/ layout, all inside the workspace)
       ├─ rh-tools   ~100 robot-domain tools (10 domains, see docs/tool-inventory.md)
       └─ rh-skills  25 SKILL.md files (asset/CAD/ROS/control/vision/models/sim/robots/data/experiment/knowledge)
              │  stdio JSON (one-shot process)
              ▼
       robotic_harness_worker (Python 3.10, shipped inside the bundle)
        ├─ assets/cad   URDF/MJCF/SDF inspection, inertia, topology, mesh, SVG preview, conversion
        ├─ simulation   MuJoCo pick-place, fault injection, batch benchmark, replay, sim-real gap
        ├─ vision       color/generic segmentation, camera health, calibration, pose checks
        ├─ control      trace metrics, trajectory validation, sys-id, PID templates
        ├─ models       embodied model registry + builtin demo adapters + honest backend probes
        ├─ diagnostics  rule engine (facts / rules / hypotheses) + telemetry anomaly scan
        ├─ robots       real-robot experiment state machine + preflight
        ├─ data         inventory/sync/transform/split/deidentify/rosbag/lerobot/dataset versions
        ├─ experiment   spec/matrix/benchmark/metrics/ablation
        ├─ ros          ros2 live probes (adapter) + ROS-free rosbag2 inspection
        └─ knowledge    docs index/search, error codes, case search
```

## One-command demo (no DSH needed, pure Python)

Requirements: Python ≥ 3.10 with `mujoco`, `numpy`, `opencv-python`, `matplotlib`, `pytest` (the Anaconda `python3.10` env is recommended).

```sh
# 1) unit tests (each test file runs in its own process to avoid native
#    DLL collisions between mujoco/cv2/pyarrow — matches the one-shot worker)
cd python && python run_tests.py

# 2) end-to-end demo: happy run + fault run + diagnostics + evidence + report
PYTHON=<your python3.10> node scripts/demo.mjs
# Output lands in examples/demo-output/:
#   report-run-*.md           experiment report (evidence + hypotheses)
#   timeline-run-*.html       standalone timeline viewer (open in a browser, no server)
#   bundle-run-*/             self-contained evidence bundle (manifest + hashes + telemetry + charts)
#   dashboard.html            single-file dashboard over the run store
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

After installation the Agent has **~100 `rh_*` tools and 25 Skills** across ten domains: assets/CAD, ROS 2 (adapter), control, vision & calibration, embodied models, simulation, real-robot experiments, telemetry & diagnostics, data processing, experiment management, and knowledge retrieval. The complete table (tool → worker command → risk level) lives in [docs/tool-inventory.md](docs/tool-inventory.md).

For example, ask the Agent:

> "Run the Robotic Harness pick-place demo: inspect the demo arm, run one clean simulation and one fault-injected simulation, diagnose the failure, export the evidence bundle and generate the report."

Tools invoke the Python worker (`python -m robotic_harness_worker <command> --input -`); all runs, telemetry, charts and reports are written to the workspace's `.rh/` directory by default. A short tour:

| Domain | Example tools |
|---|---|
| Assets & CAD | `rh_robot_asset_inspect`, `rh_urdf_validate`, `rh_urdf_to_mjcf`, `rh_sdf_validate`, `rh_cad_inventory`, `rh_mesh_inspect`, `rh_inertia_validate`, `rh_robot_topology_validate`, `rh_urdf_preview`, `rh_export_sim_asset`, `rh_generate_asset_report` |
| ROS 2 | `rh_ros_graph_snapshot`, `rh_ros_topic_profile`, `rh_ros_qos_check`, `rh_ros_tf_audit`, `rh_rosbag_inspect` (ROS-free), `rh_rosbag_start/stop`, `rh_ros_call_whitelisted_action` |
| Control | `rh_control_trace_analyze`, `rh_trajectory_validate`, `rh_planned_actual_compare`, `rh_pid_experiment_prepare`, `rh_controller_config_compare`, `rh_system_identification_job` |
| Vision | `rh_camera_health_check`, `rh_calibration_inspect`, `rh_perception_run`, `rh_perception_compare`, `rh_pose_transform_validate`, `rh_annotate_failure_frame` |
| Models | `rh_model_inventory`, `rh_model_health`, `rh_model_infer_job`, `rh_model_benchmark`, `rh_capability_route_explain`, `rh_policy_rollout_compare` |
| Simulation | `rh_sim_run`, `rh_sim_fault_inject`, `rh_sim_batch_benchmark`, `rh_sim_replay`, `rh_sim_real_gap_report`, `rh_sim_validate_scenario` |
| Robots | `rh_robot_preflight`, `rh_experiment_prepare/request_approval/start/pause/safe_cancel/status/finalize` |
| Telemetry | `rh_telemetry_channels`, `rh_telemetry_window`, `rh_anomaly_scan`, `rh_failure_evidence_collect`, `rh_run_compare`, `rh_diagnose_run`, `rh_timeline_export` |
| Data | `rh_data_inventory`, `rh_data_time_sync_estimate`, `rh_data_align_streams`, `rh_data_transform_apply`, `rh_data_split_create`, `rh_data_leakage_check`, `rh_data_deidentify`, `rh_data_convert_rosbag`, `rh_data_export_lerobot`, `rh_dataset_version_create`, `rh_dataset_card_generate` |
| Experiment | `rh_experiment_spec_create`, `rh_experiment_matrix_expand`, `rh_benchmark_start`, `rh_metrics_compute`, `rh_ablation_compare`, `rh_benchmark_report` |
| Knowledge | `rh_docs_index`, `rh_manual_search`, `rh_error_code_lookup`, `rh_case_search` |
| Reports | `rh_evidence_export`, `rh_report_generate`, `rh_dashboard_generate` |

## Demo contents (v0.1)

- **Scenario**: planar 3-DOF arm with a suction cup; picks a red box from the table and places it into a target zone (MuJoCo, built from primitives, no external meshes).
- **Perception routing**: color segmentation (low latency) → generic saliency segmentation on failure/occlusion; the routing reason is recorded.
- **Fault injection** (deterministic, seed-controlled): `perception_offset_px`, `gripper_slip`, `tf_offset`, `sensor_noise`, `model_timeout_s`, `occlusion`.
- **Telemetry**: joint target/actual/error, suction state, object pose, perception estimate vs ground truth; charts (joints/tracking/trajectory) and a scene render.
- **Diagnostics**: the rule engine produces layered evidence — facts (with timestamps and values), rule findings (thresholds/state machine), candidate root causes (grouped by perception/calibration/mechanical/control/system layer, with likelihood and missing evidence). **The final conclusion is left to a human.**
- **Evidence**: self-contained evidence bundle (hash manifest + all records) + Markdown report + timeline.html.

### Known limitations (stated honestly)

- The suction grasp is a **kinematic implementation** (the object follows the cup while attached), noted in the run config and reports.
- Perception uses real offscreen rendering when the renderer is available; otherwise it degrades to ground-truth + noise simulation (recorded in telemetry). If OpenCV itself crashes natively (e.g., DLL conflicts in exotic environments), perception degrades to the same fallback instead of failing the run.
- Live ROS 2 tools require the `ros2` CLI; without it they return a structured `backend: "unavailable"` diagnostic. rosbag2 inspection/conversion works without ROS.
- Real-robot tools are a state machine + preflight only: hardware items are reported as `skip` (never faked) until a hardware adapter exists. Simulation results are not real-robot evidence; there is no arbitrary topic-publish, real-robot write, or e-stop-release capability.
- SolidWorks files are registered in inventories but not parsed (commercial software); FreeCAD deep integration is optional.
- RLDS export produces a manifest skeleton (full TFDS export requires tensorflow); LeRobot export uses parquet when pyarrow is present, CSV otherwise.

## Repository layout

```text
packages/dsh-bundle/   the installable DSH bundle (TS plugins, skills/, worker copy, fixtures, scenarios)
python/                the robotic_harness_worker Python package + tests (run_tests.py)
fixtures/              URDF/SDF test assets + a demo rosbag2 (no ROS needed)
scenarios/             MuJoCo scenario definitions (JSON)
scripts/               sync-worker / demo / smoke-worker
docs/                  architecture, safety boundary, roadmap, demo guide, tool inventory, worker contract
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
