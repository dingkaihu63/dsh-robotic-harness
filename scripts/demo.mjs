// One-command demo for the Robotic Harness worker (no DSH profile needed).
// Runs the full loop: happy run + fault run + diagnostics + evidence + report.
//
// Usage: node scripts/demo.mjs [--python <exe>] [--out <dir>]

import { spawnSync } from 'node:child_process'
import { existsSync, mkdirSync, readFileSync } from 'node:fs'
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
