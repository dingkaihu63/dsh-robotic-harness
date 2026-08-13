// Sync the Python worker package, scenario and fixture assets into the
// dsh-bundle so the npm package is self-contained.
//
// Usage: node scripts/sync-worker.mjs

import { cpSync, mkdirSync, rmSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const bundle = join(root, 'packages', 'dsh-bundle')

const copies = [
  {
    from: join(root, 'python', 'robotic_harness_worker'),
    to: join(bundle, 'worker', 'robotic_harness_worker'),
  },
  {
    from: join(root, 'fixtures', 'robot_assets'),
    to: join(bundle, 'fixtures', 'robot_assets'),
  },
  {
    from: join(root, 'scenarios', 'mujoco_pick_place'),
    to: join(bundle, 'scenarios', 'mujoco_pick_place'),
  },
]

for (const { from, to } of copies) {
  mkdirSync(dirname(to), { recursive: true })
  rmSync(to, { recursive: true, force: true })
  cpSync(from, to, { recursive: true })
  rmSync(join(to, '__pycache__'), { recursive: true, force: true })
  console.log(`synced ${from} -> ${to}`)
}
