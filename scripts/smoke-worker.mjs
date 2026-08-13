// Smoke test: the built bundle's worker invocation against the configured
// Python. Verifies PYTHONPATH wiring and the JSON protocol end to end.
// Usage: node scripts/smoke-worker.mjs [--python <exe>]

import { resolveWorkerConfig, runWorker } from '../packages/dsh-bundle/lib/worker.js'

const args = process.argv.slice(2)
const pythonArg = args.indexOf('--python')
const python = pythonArg >= 0 ? args[pythonArg + 1] : (process.env.PYTHON || 'python')

const config = resolveWorkerConfig({ pythonPath: python, timeoutMs: 60000 })
console.log('worker dir:', config.workerDir)

const ping = await runWorker(config, 'ping', {})
console.log('ping ok:', ping.ok, ping.version)

const caps = await runWorker(config, 'capability-list', {})
console.log('capabilities:', caps.capabilities.length)

const inspect = await runWorker(config, 'inspect-asset', {
  path: new URL('../packages/dsh-bundle/fixtures/robot_assets/rh_arm.urdf', import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, '$1'),
})
console.log('inspect ok:', inspect.ok, 'links:', inspect.summary.linkCount)

console.log('SMOKE OK')
