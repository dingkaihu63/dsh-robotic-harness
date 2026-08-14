// Sync the Python worker package, scenario and fixture assets into the
// dsh-bundle so the npm package is self-contained.
//
// The copy is made to a sibling temp directory first and then swapped into
// place (rmSync + rename), so a crash between the two steps can never leave
// the bundle with an empty/partial directory. After the swap the target is
// verified non-empty.
//
// Usage: node scripts/sync-worker.mjs

import { cpSync, mkdirSync, readdirSync, renameSync, rmSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { randomUUID } from 'node:crypto'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const bundle = join(root, 'packages', 'dsh-bundle')

const copies = [
  {
    from: join(root, 'python', 'robotic_harness_worker'),
    to: join(bundle, 'worker', 'robotic_harness_worker'),
  },
  {
    from: join(root, 'fixtures'),
    to: join(bundle, 'fixtures'),
  },
  {
    from: join(root, 'scenarios', 'mujoco_pick_place'),
    to: join(bundle, 'scenarios', 'mujoco_pick_place'),
  },
]

function swapCopy(from, to) {
  mkdirSync(dirname(to), { recursive: true })
  const tmp = `${to}.tmp-${process.pid}-${randomUUID().slice(0, 8)}`
  rmSync(tmp, { recursive: true, force: true })
  cpSync(from, tmp, { recursive: true })
  rmSync(join(tmp, '__pycache__'), { recursive: true, force: true })
  // swap: the target is only removed after the temp copy is complete
  rmSync(to, { recursive: true, force: true })
  renameSync(tmp, to)
  if (readdirSync(to).length === 0) {
    throw new Error(`sync produced an empty directory: ${to}`)
  }
  console.log(`synced ${from} -> ${to}`)
}

for (const { from, to } of copies) {
  swapCopy(from, to)
}
