/**
 * rh-tools: robot-domain tools for the Robotic Harness bundle.
 *
 * Every tool validates its arguments through the DSH tool DSL, delegates to
 * the Python worker over stdio, honors `exec.signal` for cancellation, and
 * returns the worker's structured result as the canonical value. Tools are
 * read-only or simulation-scoped by design; nothing here can command real
 * hardware.
 */

import type { Context } from '@deepseek-ai/cordis'
import { defineTool } from '@deepseek-ai/dsh-tools'
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
  timeoutMs: Schema.number().default(180_000),
  storeRoot: Schema.string().default(''),
})

function jsonText(value: unknown): Array<{ type: 'text'; text: string }> {
  return [{ type: 'text', text: JSON.stringify(value, null, 2) }]
}

export function apply(ctx: Context, config: Config) {
  const worker = resolveWorkerConfig(config)
  const storeRoot = resolveStoreRoot(config.storeRoot)
  ctx.logger.info(`[rh] tools registered (python=${worker.pythonPath}, workerDir=${worker.workerDir})`)

  const tools = [
    defineTool({
      name: 'rh_worker_ping',
      description:
        'Check the Robotic Harness Python worker: version, Python environment and the availability of mujoco/opencv/matplotlib.',
      parameters: {},
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(_args, exec) {
        return runWorker(worker, 'ping', {}, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_capability_list',
      description:
        'List the Robotic Harness capabilities (asset inspection, simulation, perception, policy, diagnostics, data) with their risk levels.',
      parameters: {},
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(_args, exec) {
        return runWorker(worker, 'capability-list', {}, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_robot_asset_inspect',
      description:
        'Inspect a robot asset (URDF, MJCF/XML) and return a structured report: links, joints, inertials, collision geometry, root links, plus issues with severity (error/warning/info). Read-only: never modifies the asset.',
      parameters: {
        path: { type: 'string', required: true, description: 'Absolute path to the .urdf/.xacro(expanded)/.xml/.mjcf file' },
        format: { type: 'string', description: 'Optional format override: urdf or mjcf' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'inspect-asset', { path: args.path, format: args.format }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_urdf_validate',
      description:
        'Validate a URDF: XML well-formedness, link/joint names and tree structure, inertial mass and positive-definite inertia, joint axes and limits, mesh path existence. Returns ok/issueCounts/issues.',
      parameters: {
        path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'validate-urdf', { path: args.path }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_urdf_to_mjcf',
      description:
        'Convert a URDF to MJCF using the MuJoCo compiler. Writes the MJCF to outPath (default: sibling file) and returns a conversion report with loader warnings and known differences. The source URDF is never modified; auto-conversion does not replace human review.',
      parameters: {
        path: { type: 'string', required: true, description: 'Absolute path to the URDF file' },
        outPath: { type: 'string', description: 'Absolute path for the generated MJCF; default: <urdf-dir>/<name>.mjcf' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'convert-urdf', { path: args.path, outPath: args.outPath }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_sim_status',
      description:
        'Check the MuJoCo simulation backend: mujoco/opencv/matplotlib availability, offscreen renderer status, and whether the builtin pick-place scenario validates.',
      parameters: {},
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(_args, exec) {
        return runWorker(worker, 'sim-status', {}, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_sim_validate_scenario',
      description:
        'Validate a pick-place scenario configuration (builtin "mujoco_pick_place" or a JSON scenario file): arm reachability, object and target zone placement, parameter sanity. Returns issues and the resolved scenario.',
      parameters: {
        scenario: {
          type: 'object',
          description: 'Scenario configuration overrides, e.g. {arm: {linkLengths: [...]}, object: {...}}',
          additionalProperties: true,
        },
        path: { type: 'string', description: 'Path to a JSON scenario file (alternative to scenario)' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'sim-validate-scenario', { scenario: args.scenario, path: args.path }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_sim_run',
      description:
        'Run one MuJoCo pick-place simulation with optional fault injection. Faults: perceptionOffsetPx [dx,dy] (pixel offset of the perceived object), gripperSlip (bool), tfOffset [dx,dz] (meters), sensorNoise (rad), modelTimeoutS (perception latency), occlusion (bool). Returns the run summary with metrics, phases, anomalies and artifact paths; the full record is stored under the run store.',
      parameters: {
        scenario: {
          type: 'object',
          description: 'Scenario overrides; default is the builtin mujoco_pick_place',
          additionalProperties: true,
        },
        fault: {
          type: 'object',
          description: 'Fault injection options, e.g. {perceptionOffsetPx: [18, 6], gripperSlip: true}',
          additionalProperties: true,
        },
        seed: { type: 'number', description: 'Deterministic seed; default 42' },
        runId: { type: 'string', description: 'Optional explicit run id' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(
          worker,
          'sim-run',
          {
            scenario: args.scenario ?? 'mujoco_pick_place',
            fault: args.fault ?? {},
            seed: args.seed ?? 42,
            runId: args.runId,
            storeRoot,
          },
          exec.signal,
        )
      },
    }),

    defineTool({
      name: 'rh_diagnose_run',
      description:
        'Run the deterministic diagnostics rule engine over a stored run (runPath = run directory or run.json). Returns a diagnostic case: facts, rule findings, and candidate root causes with evidence, counter-evidence, missing evidence and suggested checks. Conclusions are hypotheses, not verdicts.',
      parameters: {
        runPath: { type: 'string', required: true, description: 'Path to the run directory or run.json' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'diagnose-run', { runPath: args.runPath, storeRoot }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_evidence_export',
      description:
        'Export a run (plus its diagnostics) into a self-contained evidence bundle: manifest with hashes, run.json, telemetry, charts, scene image and diagnostics. The bundle is the reproducibility unit — it references nothing outside itself.',
      parameters: {
        runPath: { type: 'string', required: true, description: 'Path to the run directory or run.json' },
        outDir: { type: 'string', required: true, description: 'Directory for the evidence bundle' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'evidence-export', { runPath: args.runPath, outDir: args.outDir }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_report_generate',
      description:
        'Generate a Markdown experiment/diagnostic report for a stored run, plus a standalone timeline.html viewer (self-contained, no server needed). Returns the written paths.',
      parameters: {
        runPath: { type: 'string', required: true, description: 'Path to the run directory or run.json' },
        outPath: { type: 'string', description: 'Path for the Markdown report; default: <run-dir>/report.md' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'report-generate', { runPath: args.runPath, outPath: args.outPath, timeline: true }, exec.signal)
      },
    }),

    defineTool({
      name: 'rh_data_quality',
      description:
        'Audit a CSV or JSONL timeseries (read-only): missing/NaN/Inf values, duplicate and out-of-order timestamps, interval gaps, constant channels and per-channel statistics. Returns a structured quality report.',
      parameters: {
        path: { type: 'string', required: true, description: 'Absolute path to the data file' },
        format: { type: 'string', description: 'Optional format override: csv or jsonl' },
        timeColumn: { type: 'string', description: 'Timestamp column name; default "t"' },
      },
      output: { schema: { type: 'json' }, render: jsonText },
      async execute(args, exec) {
        return runWorker(worker, 'data-quality', { path: args.path, format: args.format, timeColumn: args.timeColumn }, exec.signal)
      },
    }),
  ]

  for (const tool of tools) {
    ctx.tools.register(tool)
  }
}
