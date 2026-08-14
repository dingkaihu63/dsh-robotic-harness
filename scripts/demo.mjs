// One-command demo for the Robotic Harness worker (no DSH profile needed).
// Runs the full loop: happy run + fault run + diagnostics + evidence + report.
//
// Usage: node scripts/demo.mjs [--python <exe>] [--out <dir>]

import { spawnSync } from 'node:child_process'
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs'
import { dirname, join, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const args = process.argv.slice(2)
const pythonArg = args.indexOf('--python')
const python = pythonArg >= 0 ? args[pythonArg + 1] : (process.env.PYTHON || 'python')
const outArg = args.indexOf('--out')
const outDir = resolve(outArg >= 0 ? args[outArg + 1] : join(root, 'examples', 'demo-output'))
mkdirSync(outDir, { recursive: true })

const env = {
  ...process.env,
  PYTHONPATH: [join(root, 'python'), process.env.PYTHONPATH].filter(Boolean).join(process.platform === 'win32' ? ';' : ':'),
}

const result = spawnSync(
  python,
  ['-m', 'robotic_harness_worker', 'demo', '--input', '-'],
  {
    input: JSON.stringify({ storeRoot: join(outDir, '.rh'), demoDir: outDir }),
    encoding: 'utf8',
    env,
  },
)

if (result.status !== 0) {
  console.error(result.stderr || 'demo failed')
  process.exit(result.status ?? 1)
}

const payload = JSON.parse(result.stdout)
console.log(JSON.stringify(payload, null, 2))
console.log('\n=== demo outputs ===')
for (const run of payload.runs ?? []) {
  console.log(`run ${run.runId}: success=${run.success}`)
  console.log(`  report    : ${run.report}`)
  console.log(`  timeline  : ${run.timeline}`)
  console.log(`  evidence  : ${run.evidenceBundle}`)
}

// Generate the single-file dashboard over the run store.
const dashboard = spawnSync(
  python,
  ['-m', 'robotic_harness_worker', 'dashboard-generate', '--input', '-'],
  {
    input: JSON.stringify({ storeRoot: join(outDir, '.rh'), outPath: join(outDir, 'dashboard.html') }),
    encoding: 'utf8',
    env,
  },
)
if (dashboard.status === 0) {
  const parsed = JSON.parse(dashboard.stdout)
  console.log(`\ndashboard : ${parsed.path}`)
}

// === research & autonomous-training flow (offline-safe, demo-grade) =========
function worker(command, input) {
  const proc = spawnSync(python, ['-m', 'robotic_harness_worker', command, '--input', '-'], {
    input: JSON.stringify({ storeRoot: join(outDir, '.rh'), ...input }),
    encoding: 'utf8',
    env,
  })
  if (proc.status !== 0) throw new Error(`${command} failed: ${proc.stderr || proc.stdout}`)
  return JSON.parse(proc.stdout)
}

console.log('\n=== research & training flow ===')
try {
  const solutions = worker('problem-solutions', {
    problem: 'gripper slippage during pick and place',
    stage: 'experiment',
    maxPapers: 3,
  })
  console.log(`solutions   : backend=${solutions.backend} candidates=${solutions.candidates.length}`)

  const plan = worker('train-plan-create', {
    objective: 'train a pick-and-place policy',
    model: 'diffusion-policy',
    epochs: 5,
    datasetIds: ['local:data/pick'],
  })
  console.log(`plan        : ${plan.planId} (status=${plan.plan.status})`)

  const job = worker('train-job-prepare', { planId: plan.planId, dryRun: true })
  console.log(`job prepare : dryRun=${job.dryRun} artifacts=${job.artifacts.length}`)

  // simulate a finished training log for the report step
  const logPath = join(outDir, '.rh', 'train-jobs', plan.planId, 'run.log')
  const rows = ['epoch,loss,val_loss', ...[1, 2, 3, 4, 5].map((e) => `${e},${(1 / e).toFixed(4)},${(1.05 / e).toFixed(4)}`)]
  mkdirSync(dirname(logPath), { recursive: true })
  writeFileSync(logPath, rows.join('\n'), 'utf8')

  const report = worker('train-report', { jobId: plan.planId })
  console.log(`report      : verdict="${report.report.verdict}" improvement=${report.report.relativeImprovement}`)
} catch (error) {
  console.error(`research/training demo step failed: ${error.message}`)
}
