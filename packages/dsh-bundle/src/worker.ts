/**
 * Python worker invocation for the Robotic Harness bundle.
 *
 * Every tool in this bundle delegates to the worker process over stdio with a
 * one-shot JSON protocol: `python -m robotic_harness_worker <command> --input -`.
 * The worker directory (containing the `robotic_harness_worker` package) is
 * shipped inside the bundle and prepended to PYTHONPATH, so no pip install is
 * required for the plugin itself — only the Python runtime plus the optional
 * simulation dependencies (mujoco/numpy/opencv) must exist in the configured
 * interpreter.
 */

import { spawn } from 'node:child_process'
import { existsSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

/** Configuration shared by all worker-invoking tools. */
export interface WorkerConfig {
  /** Python executable used to run the worker. */
  pythonPath: string
  /** Directory that contains the `robotic_harness_worker` package. */
  workerDir: string
  /** Hard timeout for one worker invocation, in milliseconds. */
  timeoutMs: number
}

/** Structured failure raised when the worker cannot run or returns an error. */
export class WorkerInvocationError extends Error {
  constructor(
    message: string,
    readonly exitCode: number | null,
    readonly stderrTail: string,
  ) {
    super(message)
    this.name = 'WorkerInvocationError'
  }
}

/** Resolve the worker package directory shipped with this bundle. */
export function defaultWorkerDir(): string {
  return join(dirname(fileURLToPath(import.meta.url)), '..', 'worker')
}

/** Default worker config resolved from the bundle layout. */
export function resolveWorkerConfig(overrides: Partial<WorkerConfig>): WorkerConfig {
  const workerDir = overrides.workerDir || defaultWorkerDir()
  if (!existsSync(join(workerDir, 'robotic_harness_worker', '__init__.py'))) {
    throw new Error(
      `robotic-harness worker package not found under ${workerDir}; ` +
        're-run the repo "sync-worker" script or check the installed bundle layout',
    )
  }
  return {
    pythonPath: overrides.pythonPath || 'python',
    workerDir,
    timeoutMs: overrides.timeoutMs || 180_000,
  }
}

/** Invoke one worker command; resolves with the parsed JSON result. */
export async function runWorker(
  config: WorkerConfig,
  command: string,
  args: Record<string, unknown>,
  signal?: AbortSignal,
): Promise<any> {
  const child = spawn(config.pythonPath, ['-m', 'robotic_harness_worker', command, '--input', '-'], {
    stdio: ['pipe', 'pipe', 'pipe'],
    env: {
      ...process.env,
      PYTHONPATH: [config.workerDir, process.env.PYTHONPATH].filter(Boolean).join(process.platform === 'win32' ? ';' : ':'),
      PYTHONUNBUFFERED: '1',
    },
    windowsHide: true,
  })

  let stdout = ''
  let stderr = ''
  child.stdout.on('data', (chunk: Buffer) => {
    stdout += chunk.toString('utf8')
  })
  child.stderr.on('data', (chunk: Buffer) => {
    stderr += chunk.toString('utf8')
  })
  // The worker reads its arguments JSON from stdin (--input -).
  child.stdin.write(JSON.stringify(args ?? {}))
  child.stdin.end()

  const onAbort = () => {
    child.kill()
  }
  signal?.addEventListener('abort', onAbort, { once: true })
  const timer = setTimeout(() => child.kill(), config.timeoutMs)

  try {
    await new Promise<void>((resolve, reject) => {
      child.on('error', reject)
      child.on('close', (code) => {
        if (code === 0) resolve()
        else reject(new WorkerInvocationError(`worker exited with code ${code}`, code, stderr.slice(-2000)))
      })
    })
  } finally {
    clearTimeout(timer)
    signal?.removeEventListener('abort', onAbort)
  }

  const payload = stdout.trim()
  if (!payload) {
    throw new WorkerInvocationError(`worker produced no output for ${command}`, 0, stderr.slice(-2000))
  }
  let result: any
  try {
    result = JSON.parse(payload)
  } catch (error) {
    throw new WorkerInvocationError(
      `worker output for ${command} is not valid JSON: ${String(error)}`,
      0,
      stderr.slice(-2000),
    )
  }
  if (result && result.ok === false) {
    const kind = result.error?.kind ?? 'unknown'
    const message = result.error?.message ?? 'unknown worker error'
    throw new WorkerInvocationError(`worker ${command} failed (${kind}): ${message}`, 0, stderr.slice(-2000))
  }
  return result
}

/** The store root a run should be persisted under ('' means the worker's cwd/.rh). */
export function resolveStoreRoot(configured: string): string {
  return configured?.trim() ?? ''
}
