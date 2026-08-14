/**
 * rh-tools: robot-domain tools for the Robotic Harness bundle.
 *
 * The ~100 tools are declared data-driven in TOOL_SPECS: each spec maps a
 * DSH tool name/description/schema to one worker command. Every tool
 * validates its arguments through the DSH tool DSL, delegates to the Python
 * worker over stdio, honors `exec.signal`, and returns the worker's
 * structured result as the canonical value.
 *
 * Backend-dependent tools (ROS 2, SolidWorks, real-robot adapters, heavy
 * models) never pretend to work: the worker returns a structured
 * `backend: "unavailable"` diagnostic with install instructions instead of
 * an error, so the Agent can still reason about the situation.
 */

import type { Context } from '@deepseek-ai/cordis'
import { defineTool, type ParameterPropertySpec } from '@deepseek-ai/dsh-tools'
import Schema from '@deepseek-ai/schemastery'

import { resolveStoreRoot, resolveWorkerConfig, runWorker, type WorkerConfig } from './worker.ts'

export const name = 'rh-tools'
export const inject = ['tools']

export interface Config extends WorkerConfig {
  /** Run store root ('' means the worker's current directory + '/.rh'). */
  storeRoot: string
}

export const Config: Schema<Config> = Schema.object({
  pythonPath: Schema.string().default('python'),
  workerDir: Schema.string().default(''),
  timeoutMs: Schema.number().default(300_000),
  storeRoot: Schema.string().default(''),
})

function jsonText(value: unknown): Array<{ type: 'text'; text: string }> {
  return [{ type: 'text', text: JSON.stringify(value, null, 2) }]
}

/** One data-driven tool declaration. */
interface ToolSpec {
  /** DSH tool name (rh_*). */
  name: string
  /** Model-facing description. */
  description: string
  /** Worker command name. */
  command: string
  /** Parameter schema (DSH parameter schema spec). */
  parameters: Record<string, ParameterPropertySpec>
  /** Optional argument mapping before sending to the worker. */
  mapArgs?: (args: Record<string, any>) => Record<string, unknown>
}

// ---------------------------------------------------------------------------
// tool manifest
// ---------------------------------------------------------------------------

const TOOL_SPECS: ToolSpec[] = [
  // --- health & capabilities ------------------------------------------------
  {
    name: 'rh_worker_ping',
    description:
      'Check the Robotic Harness Python worker: version, Python environment, and availability of mujoco/opencv/matplotlib.',
    command: 'ping',
    parameters: {},
  },
  {
    name: 'rh_capability_list',
    description: 'List all Robotic Harness capabilities (asset, cad, ros, control, vision, models, simulation, robots, telemetry, data, experiment, knowledge) with risk levels.',
    command: 'capability-list',
    parameters: {},
  },

  // --- assets & CAD ---------------------------------------------------------
  {
    name: 'rh_robot_asset_inspect',
    description: 'Inspect a robot asset (URDF, MJCF/XML, SDF) and return a structured report: links, joints, inertials, collisions, plus issues with severity. Read-only.',
    command: 'inspect-asset',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the asset file' },
      format: { type: 'string', description: 'Optional format override: urdf or mjcf' },
    },
  },
  {
    name: 'rh_urdf_validate',
    description: 'Validate a URDF: XML well-formedness, tree structure, inertial mass and positive-definite inertia, joint axes and limits, mesh paths.',
    command: 'validate-urdf',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
    },
  },
  {
    name: 'rh_urdf_to_mjcf',
    description: 'Convert a URDF to MJCF using the MuJoCo compiler, with loader warnings and known format differences. Source URDF is never modified.',
    command: 'convert-urdf',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
      outPath: { type: 'string', description: 'Absolute path for the generated MJCF' },
    },
  },
  {
    name: 'rh_sdf_validate',
    description: 'Validate an SDF file structurally (links, joints, inertials, versions). Full Gazebo semantics require the Gazebo toolchain.',
    command: 'sdf-validate',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the SDF file' },
    },
  },
  {
    name: 'rh_cad_inventory',
    description: 'Scan a CAD directory (STEP/STL/OBJ/DAE/URDF/SLD*) and produce an inventory with sizes, hashes and issues. SolidWorks files are registered but not parsed.',
    command: 'cad-inventory',
    parameters: {
      path: { type: 'string', required: true, description: 'Directory or file to scan' },
      recursive: { type: 'boolean', description: 'Scan recursively; default true' },
      formats: { type: 'array', description: 'Extensions to include', items: { type: 'string' } },
    },
  },
  {
    name: 'rh_cad_compare_versions',
    description: 'Compare two versions of a robot asset (two URDFs, or two CAD inventories): added/removed links and joints, mass changes, file hashes.',
    command: 'cad-compare-versions',
    parameters: {
      pathA: { type: 'string', required: true, description: 'First version (URDF or inventory JSON)' },
      pathB: { type: 'string', required: true, description: 'Second version (URDF or inventory JSON)' },
    },
  },
  {
    name: 'rh_mesh_inspect',
    description: 'Inspect a triangle mesh (binary/ASCII STL, OBJ): vertex/triangle counts, bounds, approximate volume, degenerate triangles.',
    command: 'mesh-inspect',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the .stl or .obj file' },
    },
  },
  {
    name: 'rh_inertia_validate',
    description: 'Inertia-focused validation of a URDF: mass, positive-definite inertia matrices, unit sanity, per-link report and verdict.',
    command: 'inertia-validate',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
    },
  },
  {
    name: 'rh_robot_topology_validate',
    description: 'Validate the robot kinematic tree: single root, no cycles, no dangling joints, closed-loop detection (reported as needing human confirmation).',
    command: 'robot-topology-validate',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
    },
  },
  {
    name: 'rh_urdf_preview',
    description: 'Generate a static SVG kinematic-chain preview of a URDF (2D XZ projection with joint labels).',
    command: 'urdf-preview',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
      outPath: { type: 'string', description: 'Output SVG path; default <urdf-dir>/<name>.preview.svg' },
    },
  },
  {
    name: 'rh_export_sim_asset',
    description: 'Export a robot asset for simulation: target mjcf converts URDF via the MuJoCo compiler; target sdf-compat produces a compatibility report of URDF→SDF differences.',
    command: 'export-sim-asset',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
      target: { type: 'string', required: true, description: 'mjcf or sdf-compat' },
      outPath: { type: 'string', description: 'Output path' },
    },
  },
  {
    name: 'rh_generate_asset_report',
    description: 'Generate a Markdown asset report (summary + issues + recommendations) for a URDF/MJCF asset.',
    command: 'asset-report',
    parameters: {
      path: { type: 'string', required: true, description: 'Absolute path to the asset file' },
      outPath: { type: 'string', description: 'Output .md path' },
    },
  },

  // --- ROS 2 ----------------------------------------------------------------
  {
    name: 'rh_ros_graph_snapshot',
    description: 'Snapshot the ROS 2 graph (nodes, topics, services, actions with types). Requires the ros2 CLI; otherwise returns backend unavailable.',
    command: 'ros-graph-snapshot',
    parameters: {
      rosDomain: { type: 'number', description: 'ROS_DOMAIN_ID to use' },
      timeoutS: { type: 'number', description: 'Per-command timeout in seconds; default 5' },
    },
  },
  {
    name: 'rh_ros_topic_profile',
    description: 'Measure a ROS 2 topic publish rate (sampled window, not unbounded). Requires ros2 CLI.',
    command: 'ros-topic-profile',
    parameters: {
      topic: { type: 'string', required: true, description: 'Topic name' },
      durationS: { type: 'number', description: 'Sampling duration in seconds; default 2' },
      rate: { type: 'number', description: 'Expected minimum rate Hz; issues flag slower rates' },
    },
  },
  {
    name: 'rh_ros_qos_check',
    description: 'Check QoS compatibility for a ROS 2 topic (reliability/durability/history). Requires ros2 CLI.',
    command: 'ros-qos-check',
    parameters: {
      topic: { type: 'string', required: true, description: 'Topic name' },
    },
  },
  {
    name: 'rh_ros_tf_audit',
    description: 'Audit the TF tree: frames, update rate, staleness. Works against a live ROS 2 system or a rosbag (tf/tf_static).',
    command: 'ros-tf-audit',
    parameters: {
      timeoutS: { type: 'number', description: 'Sampling timeout; default 2' },
      rosbagPath: { type: 'string', description: 'rosbag2 path (db3 or directory) for offline audit' },
    },
  },
  {
    name: 'rh_ros_diagnostics_snapshot',
    description: 'Read the /diagnostics topic and summarize statuses (OK/WARN/ERROR/STALE). Live ROS 2 or rosbag.',
    command: 'ros-diagnostics-snapshot',
    parameters: {
      timeoutS: { type: 'number', description: 'Echo timeout; default 3' },
      rosbagPath: { type: 'string', description: 'rosbag2 path for offline analysis' },
    },
  },
  {
    name: 'rh_ros_controller_status',
    description: 'List ros2_control controllers and their state. Requires ros2 CLI.',
    command: 'ros-controller-status',
    parameters: {
      controllerNames: { type: 'array', description: 'Optional filter', items: { type: 'string' } },
    },
  },
  {
    name: 'rh_ros_moveit_audit',
    description: 'Audit a MoveIt/SRDF configuration: planning groups, joints, end effectors. Requires a config path or ros2 CLI.',
    command: 'ros-moveit-audit',
    parameters: {
      configPath: { type: 'string', description: 'Path to SRDF/config directory' },
      group: { type: 'string', description: 'Optional group filter' },
    },
  },
  {
    name: 'rh_rosbag_inspect',
    description: 'Inspect a rosbag2 (SQLite) without ROS: topics, types, message counts, time range, and best-effort decoding of common message types. Unsupported types are reported, never silently dropped.',
    command: 'rosbag-inspect',
    parameters: {
      path: { type: 'string', required: true, description: 'Path to .db3 file or bag directory' },
    },
  },
  {
    name: 'rh_rosbag_start',
    description: 'Start a controlled rosbag record job (allowlisted topics). Refuses output paths on the C: drive. Requires ros2 CLI.',
    command: 'rosbag-start',
    parameters: {
      bagPath: { type: 'string', required: true, description: 'Absolute output path for the bag (must not be on C:)' },
      topics: { type: 'array', description: 'Topics to record; default all', items: { type: 'string' } },
      compression: { type: 'string', description: 'none or zstd' },
      maxDurationS: { type: 'number', description: 'Optional auto-stop duration' },
    },
  },
  {
    name: 'rh_rosbag_stop',
    description: 'Stop a rosbag record job started with rh_rosbag_start.',
    command: 'rosbag-stop',
    parameters: {
      jobId: { type: 'string', required: true, description: 'Job id from rh_rosbag_start' },
    },
  },
  {
    name: 'rh_ros_call_whitelisted_action',
    description: 'Call a whitelisted ROS 2 action with a validated goal. Actions outside the allowlist are rejected. Requires ros2 CLI and an allowlist.',
    command: 'ros-call-whitelisted-action',
    parameters: {
      action: { type: 'string', required: true, description: 'Action name' },
      goal: { type: 'object', required: true, description: 'Goal fields as JSON', additionalProperties: true },
      allowlist: {
        type: 'array',
        description: 'Optional inline allowlist [{action, fields?}]',
        items: { type: 'object', additionalProperties: true },
      },
    },
  },

  // --- control --------------------------------------------------------------
  {
    name: 'rh_control_trace_analyze',
    description: 'Analyze a control tracking trace (CSV/JSONL): rise time, settling time, overshoot, steady-state error, RMS error, plus oscillation/saturation/windup/noise detection.',
    command: 'control-trace-analyze',
    parameters: {
      path: { type: 'string', required: true, description: 'Trace file path' },
      timeColumn: { type: 'string', description: 'Time column; default t' },
      setpointColumn: { type: 'string', description: 'Reference column; default setpoint' },
      measurementColumn: { type: 'string', description: 'Measurement column; default measurement' },
      outputColumn: { type: 'string', description: 'Controller output column; default output' },
      effortColumn: { type: 'string', description: 'Effort column (alias); default effort' },
      effortMin: { type: 'number', description: 'Effort lower clamp for saturation detection' },
      effortMax: { type: 'number', description: 'Effort upper clamp' },
      stepStart: { type: 'number', description: 'Analyze only this window (start)' },
      stepEnd: { type: 'number', description: 'Analyze only this window (end)' },
    },
  },
  {
    name: 'rh_trajectory_validate',
    description: 'Validate a joint trajectory: monotonic time, jitter, joint limits, position jumps, velocity limits, NaN/Inf.',
    command: 'trajectory-validate',
    parameters: {
      path: { type: 'string', required: true, description: 'Trajectory file path' },
      timeColumn: { type: 'string', description: 'Time column; default t' },
      limits: {
        type: 'object',
        description: 'Joint limits {joint: [min, max]}',
        additionalProperties: true,
      },
      maxJump: { type: 'number', description: 'Max position jump between samples; default 0.5' },
      velocityLimit: { type: 'number', description: 'Optional velocity limit' },
      startState: {
        type: 'object',
        description: 'Expected start state {joint: value}',
        additionalProperties: true,
      },
    },
  },
  {
    name: 'rh_planned_actual_compare',
    description: 'Compare planned vs actual trajectories: per-joint RMS/max error, time offset, first divergence point.',
    command: 'planned-actual-compare',
    parameters: {
      plannedPath: { type: 'string', required: true, description: 'Planned trajectory file' },
      actualPath: { type: 'string', required: true, description: 'Actual trajectory file' },
      timeColumnPlanned: { type: 'string', description: 'Planned time column; default t' },
      timeColumnActual: { type: 'string', description: 'Actual time column; default t' },
      threshold: { type: 'number', description: 'Divergence threshold; default 0.02' },
    },
  },
  {
    name: 'rh_pid_experiment_prepare',
    description: 'Prepare a controlled step/sweep experiment template (waypoints + safety). Generates a template only; never commands hardware.',
    command: 'pid-experiment-prepare',
    parameters: {
      controllerId: { type: 'string', required: true, description: 'Controller identifier' },
      joints: { type: 'array', required: true, description: 'Joint names', items: { type: 'string' } },
      amplitude: { type: 'number', description: 'Step amplitude; default 0.1' },
      stepTimeS: { type: 'number', description: 'Step period; default 2' },
      durationS: { type: 'number', description: 'Duration; default 10' },
      sweep: {
        type: 'object',
        description: 'Sweep config {freqMinHz, freqMaxHz}',
        additionalProperties: true,
      },
    },
  },
  {
    name: 'rh_controller_config_compare',
    description: 'Compare two controller configurations and explain the impact of parameter differences (kp/kv/ki/clamps).',
    command: 'controller-config-compare',
    parameters: {
      configA: { type: 'object', required: true, description: 'Controller config A {name, joints}', additionalProperties: true },
      configB: { type: 'object', required: true, description: 'Controller config B', additionalProperties: true },
    },
  },
  {
    name: 'rh_system_identification_job',
    description: 'Fit a first/second-order model to step-response data (gain, time constant, damping, natural frequency, delay) with fit quality.',
    command: 'system-identification',
    parameters: {
      path: { type: 'string', required: true, description: 'Step response data file' },
      timeColumn: { type: 'string', description: 'Time column; default t' },
      measurementColumn: { type: 'string', description: 'Measurement column; default measurement' },
      stepStartS: { type: 'number', description: 'Step onset time' },
      stepEndS: { type: 'number', description: 'Fit window end' },
    },
  },
  {
    name: 'rh_control_report_generate',
    description: 'Generate a Markdown control analysis report from trace/trajectory/compare sections.',
    command: 'control-report',
    parameters: {
      sections: {
        type: 'array',
        required: true,
        description: 'Sections [{kind: trace|trajectory|compare, title?, ...section args}]',
        items: { type: 'object', additionalProperties: true },
      },
      outPath: { type: 'string', required: true, description: 'Output .md path' },
    },
  },

  // --- vision & calibration -------------------------------------------------
  {
    name: 'rh_camera_health_check',
    description: 'Check camera/image quality: brightness, blur, noise, resolution, exposure anomalies.',
    command: 'camera-health-check',
    parameters: {
      imagePath: { type: 'string', description: 'Single image path' },
      imageDir: { type: 'string', description: 'Directory to sample (max 20 images)' },
      expectedWidth: { type: 'number', description: 'Expected resolution' },
      expectedHeight: { type: 'number', description: 'Expected resolution' },
    },
  },
  {
    name: 'rh_calibration_inspect',
    description: 'Inspect a camera calibration file (intrinsics/distortion/reprojection error, stereo/hand-eye): structural checks and a recalibration hint. Structural only — never claims calibration is correct.',
    command: 'calibration-inspect',
    parameters: {
      path: { type: 'string', required: true, description: 'Calibration JSON/YAML path' },
    },
  },
  {
    name: 'rh_pose_transform_validate',
    description: 'Validate pose/transform numerical properties: orthonormal rotation, determinant, unit quaternion, finite values.',
    command: 'pose-transform-validate',
    parameters: {
      transform: { type: 'object', description: 'One transform {matrix}|{position, quaternion}|{position, rpy}', additionalProperties: true },
      transforms: { type: 'array', description: 'Multiple transforms', items: { type: 'object', additionalProperties: true } },
    },
  },
  {
    name: 'rh_perception_run',
    description: 'Run perception on an image (color or saliency route with rule routing), returning the centroid, confidence and route reason.',
    command: 'perception-run',
    parameters: {
      imagePath: { type: 'string', required: true, description: 'Input image path' },
      route: { type: 'string', description: 'auto|color|saliency; default auto' },
      color: { type: 'string', description: 'Target color for color route; default red' },
      minArea: { type: 'number', description: 'Minimum blob area in px' },
      fault: {
        type: 'object',
        description: 'Fault injection {perceptionOffsetPx, occlusion}',
        additionalProperties: true,
      },
      outPath: { type: 'string', description: 'Optional annotated output image' },
    },
  },
  {
    name: 'rh_perception_compare',
    description: 'Compare two perception results (two images, same method) or against a ground-truth centroid: delta, agreement.',
    command: 'perception-compare',
    parameters: {
      imagePathA: { type: 'string', required: true, description: 'First image' },
      imagePathB: { type: 'string', required: true, description: 'Second image' },
      method: { type: 'string', description: 'color|saliency; default color' },
      color: { type: 'string', description: 'Color for color method' },
      groundTruthCentroidPx: { type: 'array', description: '[x, y] optional ground truth', items: { type: 'number' } },
    },
  },
  {
    name: 'rh_image_dataset_profile',
    description: 'Profile an image dataset directory: counts, sizes, resolution statistics, corrupt files.',
    command: 'image-dataset-profile',
    parameters: {
      path: { type: 'string', required: true, description: 'Directory path' },
      extensions: { type: 'array', description: 'Extensions to include', items: { type: 'string' } },
      maxFiles: { type: 'number', description: 'Cap; default 200' },
    },
  },
  {
    name: 'rh_annotate_failure_frame',
    description: 'Annotate a failure frame: draw detections (bboxes/centroids/labels) onto a copy of the image. The source image is never modified.',
    command: 'annotate-failure-frame',
    parameters: {
      imagePath: { type: 'string', required: true, description: 'Source image' },
      detections: {
        type: 'array',
        description: 'Detections [{bbox?: [x,y,w,h], centroidPx?: [x,y], label?, color?}]',
        items: { type: 'object', additionalProperties: true },
      },
      outPath: { type: 'string', required: true, description: 'Output image path' },
    },
  },

  // --- embodied models ------------------------------------------------------
  {
    name: 'rh_model_inventory',
    description: 'List the embodied model registry (builtin demo models + user registry): kind, modalities, action space, risk, backend.',
    command: 'model-inventory',
    parameters: {
      registryPath: { type: 'string', description: 'Optional registry JSON path' },
    },
  },
  {
    name: 'rh_model_health',
    description: 'Detect a model backend: builtin demo models are ready; external modules/endpoints are probed and reported honestly.',
    command: 'model-health',
    parameters: {
      modelId: { type: 'string', required: true, description: 'Model id from the registry' },
    },
  },
  {
    name: 'rh_model_warmup',
    description: 'Warm up a model (builtin demo models run a minimal call and report latency).',
    command: 'model-warmup',
    parameters: {
      modelId: { type: 'string', required: true, description: 'Model id' },
      timeoutS: { type: 'number', description: 'Timeout; default 30' },
    },
  },
  {
    name: 'rh_model_infer_job',
    description: 'Run one inference: builtin demo models (color/saliency segmentation on an image, scripted pick-place IK) execute for real; external models are dispatched when their backend is available.',
    command: 'model-infer',
    parameters: {
      modelId: { type: 'string', required: true, description: 'Model id' },
      input: { type: 'object', required: true, description: 'Model input (e.g. {imagePath} or {objectPose, targetPose})', additionalProperties: true },
      timeoutMs: { type: 'number', description: 'Timeout in ms' },
    },
  },
  {
    name: 'rh_model_benchmark',
    description: 'Benchmark inference latency over N iterations (mean/p50/p90/max, throughput).',
    command: 'model-benchmark',
    parameters: {
      modelId: { type: 'string', required: true, description: 'Model id' },
      iterations: { type: 'number', description: 'Iterations; default 20' },
      input: { type: 'object', description: 'Input; default from the model', additionalProperties: true },
    },
  },
  {
    name: 'rh_capability_route_explain',
    description: 'Explain capability routing for a task: filter by kind/modalities/embodiment/risk, rank, and show the selection reasons. Rule-based, no LLM call.',
    command: 'capability-route-explain',
    parameters: {
      task: { type: 'string', required: true, description: 'Task: pick_object|detect_object|vqa|navigate|...' },
      modalities: { type: 'array', description: 'Required modalities', items: { type: 'string' } },
      embodiment: { type: 'array', description: 'Supported embodiments', items: { type: 'string' } },
      preferLowLatency: { type: 'boolean', description: 'Prefer low latency' },
      maxRisk: { type: 'string', description: 'Max acceptable risk: R0-R3' },
      gpuAvailable: { type: 'boolean', description: 'GPU availability hint' },
    },
  },
  {
    name: 'rh_policy_rollout_compare',
    description: 'Compare two policies in simulation across seeds/faults: success rates, grasp/slip statistics, typical anomalies.',
    command: 'policy-rollout-compare',
    parameters: {
      policyA: { type: 'object', description: 'Policy A config (e.g. {modelId, graspOffset})', additionalProperties: true },
      policyB: { type: 'object', description: 'Policy B config', additionalProperties: true },
      scenario: { type: 'string', description: 'Scenario; default mujoco_pick_place' },
      seeds: { type: 'array', description: 'Seeds; default [42, 43]', items: { type: 'number' } },
      faults: { type: 'array', description: 'Fault configs to apply to both policies', items: { type: 'object', additionalProperties: true } },
    },
  },

  // --- simulation -----------------------------------------------------------
  {
    name: 'rh_sim_status',
    description: 'Check the simulation backend: mujoco/opencv/matplotlib availability, offscreen renderer status, scenario validity.',
    command: 'sim-status',
    parameters: {},
  },
  {
    name: 'rh_sim_validate_scenario',
    description: 'Validate a pick-place scenario (builtin or JSON file): reachability, placement, parameter sanity.',
    command: 'sim-validate-scenario',
    parameters: {
      scenario: { type: 'object', description: 'Scenario overrides', additionalProperties: true },
      path: { type: 'string', description: 'Scenario JSON file path' },
    },
  },
  {
    name: 'rh_sim_run',
    description: 'Run one MuJoCo pick-place simulation with optional fault injection (perceptionOffsetPx, gripperSlip, tfOffset, sensorNoise, modelTimeoutS, occlusion).',
    command: 'sim-run',
    parameters: {
      scenario: { type: 'object', description: 'Scenario overrides', additionalProperties: true },
      fault: { type: 'object', description: 'Fault injection options', additionalProperties: true },
      seed: { type: 'number', description: 'Deterministic seed; default 42' },
      runId: { type: 'string', description: 'Optional explicit run id' },
    },
  },
  {
    name: 'rh_sim_fault_inject',
    description: 'Dedicated fault-injection entry: run the pick-place simulation with the given fault and report anomalies.',
    command: 'sim-fault-inject',
    parameters: {
      fault: { type: 'object', required: true, description: 'Fault configuration', additionalProperties: true },
      seed: { type: 'number', description: 'Seed; default 42' },
      runId: { type: 'string', description: 'Optional run id' },
      scenario: { type: 'object', description: 'Scenario overrides', additionalProperties: true },
    },
  },
  {
    name: 'rh_sim_replay',
    description: 'Replay a stored run read-only: copy record/telemetry/artifacts, regenerate timeline and report. Nothing is re-executed.',
    command: 'sim-replay',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      outDir: { type: 'string', required: true, description: 'Replay output directory' },
    },
  },
  {
    name: 'rh_sim_real_gap_report',
    description: 'Compare simulated run channels with real data (CSV): distribution-level gap report. Explicitly refuses real-robot safety conclusions.',
    command: 'sim-real-gap-report',
    parameters: {
      simRunPath: { type: 'string', required: true, description: 'Sim run directory or run.json' },
      realCsvPath: { type: 'string', required: true, description: 'Real data CSV' },
      channelMap: {
        type: 'object',
        required: true,
        description: 'Sim channel -> real column map, e.g. {"q.0": "joint0"}',
        additionalProperties: true,
      },
    },
  },
  {
    name: 'rh_sim_batch_benchmark',
    description: 'Run a small matrix of simulation cells and aggregate success rates and anomaly patterns.',
    command: 'sim-batch-benchmark',
    parameters: {
      cells: {
        type: 'array',
        required: true,
        description: 'Cells [{label?, fault?, seed?}]',
        items: { type: 'object', additionalProperties: true },
      },
      outDir: { type: 'string', description: 'Optional output dir for benchmark.json' },
    },
  },

  // --- real-robot experiments ----------------------------------------------
  {
    name: 'rh_robot_preflight',
    description: 'Generate and run a real-robot preflight checklist. Hardware items are skipped (not passed) without a hardware adapter — honest reporting, no fake passes.',
    command: 'robot-preflight',
    parameters: {
      experimentId: { type: 'string', description: 'Optional experiment record' },
      robotModel: { type: 'string', description: 'Robot model id' },
      hardwareAdapter: { type: 'string', description: 'Hardware adapter id (none in demo)' },
      checks: { type: 'array', description: 'Custom check ids', items: { type: 'string' } },
      autoRun: { type: 'boolean', description: 'Run checks; default true' },
    },
  },
  {
    name: 'rh_robot_state_snapshot',
    description: 'Snapshot the robot state from the store (last run, preflight summary). Hardware fields are null without an adapter.',
    command: 'robot-state-snapshot',
    parameters: {
      experimentId: { type: 'string', description: 'Optional experiment id' },
    },
  },
  {
    name: 'rh_experiment_prepare',
    description: 'Create a real-robot experiment record (DRAFT→VALIDATING): name, plan, scenario, safety limits.',
    command: 'experiment-prepare',
    parameters: {
      name: { type: 'string', required: true, description: 'Experiment name' },
      robotModel: { type: 'string', description: 'Robot model' },
      plan: { type: 'string', description: 'Plan description' },
      scenario: { type: 'string', description: 'Scenario reference' },
      requiresApproval: { type: 'boolean', description: 'Default true' },
      safetyLimits: { type: 'object', description: 'Safety limits {maxVelocity, maxForce}', additionalProperties: true },
    },
  },
  {
    name: 'rh_experiment_request_approval',
    description: 'Move an experiment to READY_FOR_APPROVAL. Approval is always performed by a human; the LLM has no approval authority.',
    command: 'experiment-request-approval',
    parameters: {
      experimentId: { type: 'string', required: true, description: 'Experiment id' },
      operator: { type: 'string', description: 'Operator identifier' },
      evidence: { type: 'array', description: 'Evidence references', items: { type: 'string' } },
    },
  },
  {
    name: 'rh_experiment_start',
    description: 'Start an approved experiment (requires a human approval reference). Without a hardware adapter this only records state; no hardware moves.',
    command: 'experiment-start',
    parameters: {
      experimentId: { type: 'string', required: true, description: 'Experiment id' },
      approver: { type: 'string', description: 'Approver name' },
      approvalRef: { type: 'string', description: 'Human approval reference (required)' },
    },
  },
  {
    name: 'rh_experiment_pause',
    description: 'Pause a running experiment (RUNNING→PAUSED/RECOVERING).',
    command: 'experiment-pause',
    parameters: {
      experimentId: { type: 'string', required: true, description: 'Experiment id' },
      operator: { type: 'string', description: 'Operator' },
      reason: { type: 'string', description: 'Reason' },
    },
  },
  {
    name: 'rh_experiment_safe_cancel',
    description: 'Safely cancel a running experiment (→ABORTED). Never clears an e-stop; e-stop release is always performed by on-site personnel.',
    command: 'experiment-safe-cancel',
    parameters: {
      experimentId: { type: 'string', required: true, description: 'Experiment id' },
      operator: { type: 'string', description: 'Operator' },
      reason: { type: 'string', description: 'Reason' },
    },
  },
  {
    name: 'rh_experiment_status',
    description: 'Query experiment state and full state-change history.',
    command: 'experiment-status',
    parameters: {
      experimentId: { type: 'string', required: true, description: 'Experiment id' },
    },
  },
  {
    name: 'rh_experiment_finalize',
    description: 'Finalize an experiment to a terminal state (COMPLETED/FAILED/ABORTED) with a human conclusion.',
    command: 'experiment-finalize',
    parameters: {
      experimentId: { type: 'string', required: true, description: 'Experiment id' },
      outcome: { type: 'string', required: true, description: 'completed|failed|aborted' },
      summary: { type: 'string', description: 'Summary' },
      humanConclusion: { type: 'string', description: 'Human conclusion' },
    },
  },

  // --- telemetry & diagnostics ---------------------------------------------
  {
    name: 'rh_telemetry_channels',
    description: 'List telemetry channels of a run with sample rates, ranges and missing counts.',
    command: 'telemetry-channels',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
    },
  },
  {
    name: 'rh_telemetry_window',
    description: 'Extract a time window of telemetry channels with window statistics.',
    command: 'telemetry-window',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      startS: { type: 'number', description: 'Window start' },
      endS: { type: 'number', description: 'Window end' },
      channels: { type: 'array', description: 'Channel names', items: { type: 'string' } },
    },
  },
  {
    name: 'rh_anomaly_scan',
    description: 'Deterministic anomaly scan over telemetry: threshold/rate/spike/constant/NaN detection with sliding windows.',
    command: 'anomaly-scan',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      channels: { type: 'array', description: 'Channels to scan', items: { type: 'string' } },
      windowS: { type: 'number', description: 'Sliding window; default 1.0' },
      method: { type: 'string', description: 'threshold|rate|spike|all; default all' },
      thresholds: {
        type: 'object',
        description: 'Per-channel thresholds {channel: {min?, max?, maxRate?, spikeSigma?}}',
        additionalProperties: true,
      },
    },
  },
  {
    name: 'rh_failure_evidence_collect',
    description: 'Collect failure evidence around anomalies: frozen window, channel summaries, artifact references, optional diagnostic case creation.',
    command: 'failure-evidence-collect',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      anomalyRef: { type: 'object', description: 'Anomaly reference {t?, channel?}', additionalProperties: true },
      anomalyKinds: { type: 'array', description: 'Filter anomaly kinds', items: { type: 'string' } },
      createCase: { type: 'boolean', description: 'Create a diagnostic case' },
    },
  },
  {
    name: 'rh_diagnose_run',
    description: 'Run the deterministic diagnostics rule engine: facts, rule findings, candidate root causes with evidence and suggested checks. Conclusions are hypotheses, not verdicts.',
    command: 'diagnose-run',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
    },
  },
  {
    name: 'rh_run_compare',
    description: 'Compare two runs channel by channel: RMS/max delta, correlation, first divergence time.',
    command: 'run-compare',
    parameters: {
      runA: { type: 'string', required: true, description: 'First run path' },
      runB: { type: 'string', required: true, description: 'Second run path' },
      channels: { type: 'array', description: 'Channels to compare', items: { type: 'string' } },
      timeWindowS: { type: 'number', description: 'Comparison window' },
    },
  },
  {
    name: 'rh_timeline_export',
    description: 'Export a standalone timeline.html viewer for a run (self-contained, no server).',
    command: 'timeline-export',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      outPath: { type: 'string', required: true, description: 'Output HTML path' },
    },
  },

  // --- data processing ------------------------------------------------------
  {
    name: 'rh_data_inventory',
    description: 'Register data files: format detection, sizes, hashes, issues. Read-only.',
    command: 'data-inventory',
    parameters: {
      path: { type: 'string', required: true, description: 'File or directory' },
      recursive: { type: 'boolean', description: 'Default true' },
    },
  },
  {
    name: 'rh_data_schema_inspect',
    description: 'Inspect the schema of a CSV/JSONL dataset: columns, dtypes, missing counts, time range.',
    command: 'data-schema-inspect',
    parameters: {
      path: { type: 'string', required: true, description: 'Data file' },
      format: { type: 'string', description: 'csv|jsonl override' },
      timeColumn: { type: 'string', description: 'Time column; default t' },
    },
  },
  {
    name: 'rh_data_quality_audit',
    description: 'Audit CSV/JSONL timeseries quality: missing/NaN/Inf, duplicate and out-of-order timestamps, gaps, constant channels.',
    command: 'data-quality',
    parameters: {
      path: { type: 'string', required: true, description: 'Data file' },
      format: { type: 'string', description: 'csv|jsonl override' },
      timeColumn: { type: 'string', description: 'Time column; default t' },
    },
  },
  {
    name: 'rh_data_time_sync_estimate',
    description: 'Estimate the time offset between two recorded streams via cross-correlation (or coarse mean difference).',
    command: 'data-time-sync-estimate',
    parameters: {
      pathA: { type: 'string', required: true, description: 'First stream' },
      pathB: { type: 'string', required: true, description: 'Second stream' },
      timeColumnA: { type: 'string', description: 'Default t' },
      timeColumnB: { type: 'string', description: 'Default t' },
      signalColumns: { type: 'object', description: 'Signal columns {a, b} for cross-correlation', additionalProperties: true },
      maxLagS: { type: 'number', description: 'Max lag; default 10' },
      sampleRateHz: { type: 'number', description: 'Resample rate for correlation' },
    },
  },
  {
    name: 'rh_data_align_streams',
    description: 'Align multiple streams to a primary timeline (nearest/exact/window). Continuous values may be interpolated; categorical/event data is never interpolated.',
    command: 'data-align-streams',
    parameters: {
      primary: { type: 'string', required: true, description: 'Primary stream path' },
      secondary: { type: 'string', description: 'Secondary stream (or use files)' },
      files: { type: 'array', description: 'Streams [{path, timeColumn?}]', items: { type: 'object', additionalProperties: true } },
      strategy: { type: 'string', description: 'nearest|exact|window; default nearest' },
      maxGapS: { type: 'number', description: 'Max match gap; default 0.1' },
      outPath: { type: 'string', description: 'Optional aligned output' },
    },
  },
  {
    name: 'rh_data_transform_apply',
    description: 'Apply a non-destructive transform chain (range filter, dedupe, sort, gap interpolation, lowpass, median, resample, detrend, unit convert, round) writing a NEW file.',
    command: 'data-transform-apply',
    parameters: {
      inputPath: { type: 'string', required: true, description: 'Input data file' },
      operations: {
        type: 'array',
        required: true,
        description:
          'Operations [{kind, params}] — params holds the operation arguments, e.g. {kind: "range-filter", params: {column: "q0", min: 0, max: 3}}; kinds: range-filter, dedupe, sort, interpolate-gaps, lowpass, median, resample, detrend, unit-convert, round',
        items: { type: 'object', additionalProperties: true },
      },
      outPath: { type: 'string', required: true, description: 'Output file (never the input)' },
      timeColumn: { type: 'string', description: 'Time column; default t' },
    },
  },
  {
    name: 'rh_data_segment_episodes',
    description: 'Segment a timeseries into episodes by time gaps, with optional label distributions.',
    command: 'data-segment-episodes',
    parameters: {
      path: { type: 'string', required: true, description: 'Data file' },
      timeColumn: { type: 'string', description: 'Default t' },
      maxGapS: { type: 'number', description: 'Episode gap threshold; default 2.0' },
      labelColumn: { type: 'string', description: 'Optional label column' },
    },
  },
  {
    name: 'rh_data_annotation_import',
    description: 'Import annotations (episodes/events/labels) from CSV/JSONL into a reviewed form.',
    command: 'data-annotation-import',
    parameters: {
      path: { type: 'string', required: true, description: 'Annotation file' },
      format: { type: 'string', description: 'csv|jsonl' },
      outPath: { type: 'string', description: 'Normalized output' },
      schema: { type: 'object', description: 'Column mapping {timeColumn?, labelColumn?}', additionalProperties: true },
    },
  },
  {
    name: 'rh_data_annotation_review',
    description: 'Review annotations: list, confirm or reject (writes a new file with status; never mutates the source).',
    command: 'data-annotation-review',
    parameters: {
      path: { type: 'string', required: true, description: 'Annotation file' },
      action: { type: 'string', required: true, description: 'list|confirm|reject' },
      ids: { type: 'array', description: 'Annotation ids', items: { type: 'string' } },
      outPath: { type: 'string', description: 'Output for confirm/reject' },
    },
  },
  {
    name: 'rh_data_split_create',
    description: 'Create a leakage-safe train/val/test split (group-aware by participant/episode/robot columns).',
    command: 'data-split-create',
    parameters: {
      path: { type: 'string', required: true, description: 'Data file' },
      outDir: { type: 'string', description: 'Output directory for split files' },
      method: { type: 'string', description: 'random|group; default random' },
      groupColumns: { type: 'array', description: 'Group columns for group splits', items: { type: 'string' } },
      ratios: { type: 'object', description: 'Split ratios {train, val, test}', additionalProperties: true },
      seed: { type: 'number', description: 'Default 42' },
    },
  },
  {
    name: 'rh_data_leakage_check',
    description: 'Check splits for leakage: same group across splits, adjacent frames across random splits.',
    command: 'data-leakage-check',
    parameters: {
      path: { type: 'string', description: 'Single file with a split column' },
      splitColumn: { type: 'string', description: 'Split column name' },
      groupColumns: { type: 'array', required: true, description: 'Group key columns', items: { type: 'string' } },
      trainPath: { type: 'string', description: 'Alternative: explicit split files' },
      valPath: { type: 'string', description: 'Alternative: explicit split files' },
      testPath: { type: 'string', description: 'Alternative: explicit split files' },
      timeColumn: { type: 'string', description: 'For adjacency checks' },
    },
  },
  {
    name: 'rh_data_deidentify',
    description: 'De-identify human data: face blur, EXIF strip, PII scan, filename sanitization. Outputs go to outDir. De-identification is NOT anonymization; local processing by default.',
    command: 'data-deidentify',
    parameters: {
      inputPath: { type: 'string', required: true, description: 'Image/text/CSV input' },
      outDir: { type: 'string', description: 'Output directory (required for writes; must not be C:)' },
      operations: { type: 'array', description: 'face-blur|exif-strip|pii-scan|filename-sanitize', items: { type: 'string' } },
    },
  },
  {
    name: 'rh_data_convert_rosbag',
    description: 'Convert rosbag2 topics to CSV (decoded where possible; unsupported types are listed, never dropped silently).',
    command: 'data-convert-rosbag',
    parameters: {
      rosbagPath: { type: 'string', required: true, description: 'rosbag2 path' },
      outDir: { type: 'string', required: true, description: 'Output directory' },
      topics: { type: 'array', description: 'Topics to convert', items: { type: 'string' } },
      timeColumn: { type: 'string', description: 'Default t' },
    },
  },
  {
    name: 'rh_data_export_lerobot',
    description: 'Export episodes to a LeRobot-style dataset (parquet when pyarrow is available, CSV fallback otherwise).',
    command: 'data-export-lerobot',
    parameters: {
      runPath: { type: 'string', description: 'Run directory (alternative to episodesPath)' },
      episodesPath: { type: 'string', description: 'Episode data file' },
      outDir: { type: 'string', required: true, description: 'Dataset output directory' },
      robotName: { type: 'string', description: 'Default rh_demo' },
      task: { type: 'string', description: 'Default pick_place' },
    },
  },
  {
    name: 'rh_data_export_rlds',
    description: 'Export an RLDS-style manifest (full TFDS export requires tensorflow; the manifest and features skeleton are generated).',
    command: 'data-export-rlds',
    parameters: {
      outDir: { type: 'string', required: true, description: 'Output directory' },
    },
  },
  {
    name: 'rh_dataset_version_create',
    description: 'Freeze a dataset version non-destructively: copy sources, content hash, transform DAG, split record, manifest.',
    command: 'dataset-version-create',
    parameters: {
      name: { type: 'string', required: true, description: 'Dataset name' },
      sourcePaths: { type: 'array', required: true, description: 'Source files/directories', items: { type: 'string' } },
      outDir: { type: 'string', required: true, description: 'Version output directory' },
      transforms: { type: 'array', description: 'Transform records', items: { type: 'object', additionalProperties: true } },
      split: { type: 'object', description: 'Split record', additionalProperties: true },
      seed: { type: 'number', description: 'Repro seed' },
      description: { type: 'string', description: 'Description' },
      version: { type: 'string', description: 'Semantic version; default 0.1.0' },
    },
  },
  {
    name: 'rh_dataset_compare',
    description: 'Compare two dataset versions: content hash, file diffs, schema/sample differences.',
    command: 'dataset-compare',
    parameters: {
      datasetA: { type: 'string', required: true, description: 'Version dir or manifest' },
      datasetB: { type: 'string', required: true, description: 'Version dir or manifest' },
    },
  },
  {
    name: 'rh_dataset_card_generate',
    description: 'Generate a Markdown data card (schema, stats, transform DAG, licenses, bias notes) for a dataset version.',
    command: 'dataset-card-generate',
    parameters: {
      datasetPath: { type: 'string', required: true, description: 'Dataset version dir' },
      outPath: { type: 'string', description: 'Output .md path' },
    },
  },

  // --- experiment management ------------------------------------------------
  {
    name: 'rh_experiment_spec_create',
    description: 'Create and validate an experiment specification (question, hypothesis, variables, metrics, seeds, repetitions) with open questions for the Agent.',
    command: 'experiment-spec-create',
    parameters: {
      name: { type: 'string', required: true, description: 'Experiment name' },
      researchQuestion: { type: 'string', description: 'Research question' },
      hypothesis: { type: 'string', description: 'Hypothesis' },
      primaryMetric: { type: 'string', description: 'Primary metric; default success_rate' },
      independentVariables: {
        type: 'array',
        required: true,
        description: 'Variables [{name, values: [...]}]',
        items: { type: 'object', additionalProperties: true },
      },
      controlVariables: { type: 'array', description: 'Fixed controls', items: { type: 'object', additionalProperties: true } },
      baselines: { type: 'array', description: 'Baseline configs', items: { type: 'object', additionalProperties: true } },
      metrics: { type: 'array', description: 'Metrics to compute', items: { type: 'string' } },
      seed: { type: 'number', description: 'Default 42' },
      repetitions: { type: 'number', description: 'Default 3' },
      termination: { type: 'object', description: 'Termination {maxRuns?}', additionalProperties: true },
      artifactPolicy: { type: 'string', description: 'Artifact policy' },
      requiresApproval: { type: 'boolean', description: 'Default false (simulation)' },
    },
  },
  {
    name: 'rh_experiment_matrix_expand',
    description: 'Expand an experiment spec into a cell matrix (variables × repetitions with derived seeds).',
    command: 'experiment-matrix-expand',
    parameters: {
      experimentId: { type: 'string', description: 'Stored experiment id (alternative to spec)' },
      spec: { type: 'object', description: 'Inline spec', additionalProperties: true },
      maxCells: { type: 'number', description: 'Optional cap' },
    },
  },
  {
    name: 'rh_benchmark_start',
    description: 'Execute an experiment matrix in simulation and record per-cell run ids.',
    command: 'benchmark-start',
    parameters: {
      experimentId: { type: 'string', description: 'Stored experiment id (alternative to spec)' },
      spec: { type: 'object', description: 'Inline spec', additionalProperties: true },
      faultTemplates: { type: 'object', description: 'Variable→fault mapping {name: fault}', additionalProperties: true },
      scenario: { type: 'object', description: 'Scenario overrides', additionalProperties: true },
    },
  },
  {
    name: 'rh_metrics_compute',
    description: 'Aggregate metrics over runs: success rate, timing, tracking RMS, per-variable breakdowns.',
    command: 'metrics-compute',
    parameters: {
      experimentId: { type: 'string', description: 'Stored experiment id' },
      runs: { type: 'array', description: 'Inline runs [{metrics: {...}}]', items: { type: 'object', additionalProperties: true } },
    },
  },
  {
    name: 'rh_ablation_compare',
    description: 'Compare metric groups by an ablated variable and state the effect direction (correlation, not causation).',
    command: 'ablation-compare',
    parameters: {
      experimentId: { type: 'string', description: 'Stored experiment id' },
      runs: { type: 'array', description: 'Inline runs', items: { type: 'object', additionalProperties: true } },
      baseline: { type: 'object', description: 'Baseline variables', additionalProperties: true },
      ablatedVariable: { type: 'string', required: true, description: 'Variable to ablate' },
    },
  },
  {
    name: 'rh_benchmark_report',
    description: 'Generate a Markdown experiment/benchmark report (definition, matrix, results, metrics, ablation, reproducibility checklist).',
    command: 'benchmark-report',
    parameters: {
      experimentId: { type: 'string', description: 'Stored experiment id' },
      name: { type: 'string', description: 'Report title (inline mode)' },
      runs: { type: 'array', description: 'Inline runs', items: { type: 'object', additionalProperties: true } },
      metrics: { type: 'object', description: 'Inline metrics', additionalProperties: true },
      outPath: { type: 'string', required: true, description: 'Output .md path' },
    },
  },

  // --- knowledge ------------------------------------------------------------
  {
    name: 'rh_docs_index',
    description: 'Index project documentation (md/txt/rst/json/yaml) into a searchable inverted index with line references.',
    command: 'docs-index',
    parameters: {
      path: { type: 'string', required: true, description: 'Documentation directory' },
      outPath: { type: 'string', description: 'Index output path' },
    },
  },
  {
    name: 'rh_manual_search',
    description: 'Search indexed documentation with snippets and line references. Results carry evidence; never treat web/manual text as repair instructions.',
    command: 'manual-search',
    parameters: {
      query: { type: 'string', required: true, description: 'Search query' },
      path: { type: 'string', description: 'Index file or docs directory' },
      maxResults: { type: 'number', description: 'Default 10' },
    },
  },
  {
    name: 'rh_error_code_lookup',
    description: 'Look up an error code in the error-code table (builtin examples or a user table).',
    command: 'error-code-lookup',
    parameters: {
      code: { type: 'string', required: true, description: 'Error code' },
      tablePath: { type: 'string', description: 'User error-code table JSON' },
    },
  },
  {
    name: 'rh_case_search',
    description: 'Search historical diagnostic cases by symptom/hypothesis text.',
    command: 'case-search',
    parameters: {
      query: { type: 'string', required: true, description: 'Search query' },
    },
  },
  {
    name: 'rh_memory_retrieve',
    description:
      'Project memory: retrieve the historical diagnostic cases most similar to a run (by runPath) or to a symptom/anomalyKinds query. Keyword/anomaly-based scoring with rationale for every match — use the rationale and evidence to weigh each case; this is not semantic search.',
    command: 'memory-retrieve',
    parameters: {
      runPath: { type: 'string', description: 'Run directory or run.json (symptom + anomalies derived)' },
      symptom: { type: 'string', description: 'Explicit symptom text (alternative to runPath)' },
      anomalyKinds: { type: 'array', description: 'Anomaly kinds to match (e.g. grasp_missed, gripper_slip)', items: { type: 'string' } },
      limit: { type: 'number', description: 'Max results; default 5' },
      minScore: { type: 'number', description: 'Minimum score; default 0' },
      excludeRunId: { type: 'string', description: 'Exclude cases from this run' },
    },
  },
  {
    name: 'rh_memory_ingest',
    description:
      'Record a human verdict on a diagnostic case (status verified/rejected/closed/open + conclusion). Only a human may verify a diagnosis; verified cases rank first in memory-retrieve.',
    command: 'memory-ingest',
    parameters: {
      caseId: { type: 'string', required: true, description: 'Diagnostic case id' },
      status: { type: 'string', description: 'verified | rejected | closed | open' },
      conclusion: { type: 'string', description: 'Human conclusion text' },
      operator: { type: 'string', description: 'Operator identifier' },
    },
  },

  // --- research & literature -------------------------------------------------
  {
    name: 'rh_literature_search',
    description:
      'Search public academic literature (arXiv / Semantic Scholar) for a problem or topic. Returns titles, authors, years, abstracts and source URLs. Best-effort network: on failure the backend is reported as unavailable — never fabricate results. Literature is evidence, not verdicts; verify before citing.',
    command: 'literature-search',
    parameters: {
      query: { type: 'string', required: true, description: 'Problem or topic to search for' },
      maxResults: { type: 'number', description: 'Max results per source; default 8' },
      sources: { type: 'array', description: 'Sources: arxiv (default), semantic-scholar', items: { type: 'string' } },
    },
  },
  {
    name: 'rh_problem_solutions',
    description:
      'For a user problem at ANY stage (experiment/model/simulation/data/control/perception), search literature and produce ranked evidence cards plus a solution-proposal scaffold for the user to choose from. The worker only matches keywords — the Agent synthesizes options and the user makes the final call; conclusions must not be presented as verified.',
    command: 'problem-solutions',
    parameters: {
      problem: { type: 'string', required: true, description: 'The problem to find solutions for' },
      stage: { type: 'string', description: 'experiment | model | simulation | data | control | perception | general' },
      context: { type: 'string', description: 'Additional context (hardware, env, constraints)' },
      maxPapers: { type: 'number', description: 'Max papers to consider; default 6' },
      outPath: { type: 'string', description: 'Save the proposal scaffold JSON here' },
    },
  },

  // --- autonomous training ---------------------------------------------------
  {
    name: 'rh_train_server_check',
    description:
      'List explicitly configured training servers (<storeRoot>/train-servers.json) and probe SSH connectivity with a read-only echo. No server config → backend unavailable with setup instructions. Never assume a server exists without config.',
    command: 'train-server-check',
    parameters: {},
  },
  {
    name: 'rh_train_plan_create',
    description:
      'Create an auditable training plan (objective, model, hyperparameters, dataset ids, phases) saved as JSON + Markdown with status=draft. Planning only — it never submits anything.',
    command: 'train-plan-create',
    parameters: {
      objective: { type: 'string', required: true, description: 'Training objective' },
      model: { type: 'string', description: 'Model name; default placeholder-model' },
      serverId: { type: 'string', description: 'Configured training server id (optional for dry-run)' },
      epochs: { type: 'number', description: 'Default 10' },
      batchSize: { type: 'number', description: 'Default 32' },
      learningRate: { type: 'number', description: 'Default 1e-3' },
      optimizer: { type: 'string', description: 'Default adam' },
      validationSplit: { type: 'number', description: 'Default 0.2' },
      datasetIds: { type: 'array', description: 'Data sources: local:path or HF dataset id', items: { type: 'string' } },
      planId: { type: 'string', description: 'Custom plan id (default auto)' },
    },
  },
  {
    name: 'rh_train_data_discovery',
    description:
      'Search public datasets (Hugging Face API) for supplementary training data. Best-effort network; returns backend unavailable on failure. Datasets are candidates only — check license and quality before adding them to a plan.',
    command: 'train-data-discovery',
    parameters: {
      query: { type: 'string', required: true, description: 'Dataset search query (e.g. robot manipulation)' },
      maxResults: { type: 'number', description: 'Default 10' },
    },
  },
  {
    name: 'rh_train_job_prepare',
    description:
      'Prepare a training job from an approved plan: generates train.py (template-based), launcher.sh and a plan snapshot locally — dry-run by default. Real remote submission requires dryRun:false AND confirm:true in the same call plus a reachable configured server; the remote command is allowlisted (only our generated launcher in the configured work dir). Never submit without explicit human confirmation.',
    command: 'train-job-prepare',
    parameters: {
      planId: { type: 'string', required: true, description: 'Plan id from train-plan-create' },
      dryRun: { type: 'boolean', description: 'Default true — only prepare artifacts locally' },
      confirm: { type: 'boolean', description: 'Human confirmation; required for remote submission' },
      serverId: { type: 'string', description: 'Override the plan server id' },
    },
  },
  {
    name: 'rh_train_job_status',
    description:
      'Follow a prepared/submitted training job: job record plus the recent log tail (local run.log, or remote tail for submitted jobs on configured servers).',
    command: 'train-job-status',
    parameters: {
      jobId: { type: 'string', required: true, description: 'Job id (= plan id)' },
    },
  },
  {
    name: 'rh_train_report',
    description:
      'Generate a statistical training report from a training log (epoch,loss[,val_loss]): convergence verdict, relative improvement, Markdown report file. Statistical only — never a release verdict.',
    command: 'train-report',
    parameters: {
      jobId: { type: 'string', description: 'Job id to read run.log from the store' },
      logPath: { type: 'string', description: 'Explicit log CSV path (alternative to jobId)' },
      outPath: { type: 'string', description: 'Report output path' },
    },
  },

  // --- reports & dashboard --------------------------------------------------
  {
    name: 'rh_evidence_export',
    description: 'Export a run (plus diagnostics) into a self-contained evidence bundle: manifest with hashes, run record, telemetry, charts.',
    command: 'evidence-export',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      outDir: { type: 'string', required: true, description: 'Bundle output directory' },
    },
  },
  {
    name: 'rh_report_generate',
    description: 'Generate a Markdown experiment/diagnostic report plus a standalone timeline.html for a run.',
    command: 'report-generate',
    parameters: {
      runPath: { type: 'string', required: true, description: 'Run directory or run.json' },
      outPath: { type: 'string', description: 'Report output path' },
    },
  },
  {
    name: 'rh_dashboard_generate',
    description: 'Generate a single-file dashboard over the run store (overview, runs, diagnostics tabs). Static snapshot, not real-time.',
    command: 'dashboard-generate',
    parameters: {
      outPath: { type: 'string', required: true, description: 'Output HTML path' },
    },
  },
]

// ---------------------------------------------------------------------------

function makeTool(spec: ToolSpec, worker: WorkerConfig, storeRoot: string) {
  return defineTool({
    name: spec.name,
    description: spec.description,
    parameters: spec.parameters,
    output: { schema: { type: 'json' }, render: jsonText },
    async execute(args: Record<string, any>, exec: any) {
      const payload = spec.mapArgs ? spec.mapArgs(args) : { ...args }
      return runWorker(worker, spec.command, { ...payload, storeRoot }, exec.signal)
    },
    // Parameters come from the manifest; runtime argument validation still
    // applies through defineTool's compiled schema.
  } as never)
}

export function apply(ctx: Context, config: Config) {
  const worker = resolveWorkerConfig(config)
  const storeRoot = resolveStoreRoot(config.storeRoot)
  ctx.logger.info(
    `[rh] registering ${TOOL_SPECS.length} tools (python=${worker.pythonPath}, workerDir=${worker.workerDir})`,
  )
  for (const spec of TOOL_SPECS) {
    ctx.tools.register(makeTool(spec, worker, storeRoot))
  }
}
