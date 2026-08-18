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
    readonly stdoutPayload?: string,
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
  let timedOut = false
  child.stdout.on('data', (chunk: Buffer) => {
    stdout += chunk.toString('utf8')
  })
  child.stderr.on('data', (chunk: Buffer) => {
    stderr += chunk.toString('utf8')
  })
  // The worker reads its arguments JSON from stdin (--input -).
  // Without an 'error' listener the stdin stream can emit an uncaught
  // EPIPE/'error' when python fails to spawn or exits before consuming stdin.
  child.stdin.on('error', () => {})
  child.stdin.write(JSON.stringify(args ?? {}))
  child.stdin.end()

  const killTree = () => {
    if (process.platform === 'win32') {
      // taskkill /T kills the whole process tree (ros2 bag record, ssh/scp
      // children included); child.kill() alone only SIGTERMs the python proc.
      try {
        spawn('taskkill', ['/pid', String(child.pid), '/T', '/F'], { stdio: 'ignore' })
      } catch {
        child.kill()
      }
    } else {
      // The child is not detached, so it shares our process group: only kill
      // the direct child (its own children are best-effort reaped by it).
      child.kill('SIGKILL')
    }
  }
  const onAbort = () => {
    killTree()
  }
  // If the signal is already aborted the 'abort' event will never fire.
  if (signal?.aborted) {
    killTree()
  } else {
    signal?.addEventListener('abort', onAbort, { once: true })
  }
  const timer = setTimeout(() => {
    timedOut = true
    killTree()
  }, config.timeoutMs)

  try {
    await new Promise<void>((resolve, reject) => {
      child.on('error', reject)
      child.on('close', (code) => {
        if (code === 0) resolve()
        else if (timedOut) {
          reject(new WorkerInvocationError(`worker ${command} timed out after ${config.timeoutMs} ms`, code, stderr.slice(-2000), stdout.slice(-2000)))
        } else {
          // Non-zero exit: the worker may have written a structured
          // {"ok": false, "error": {...}} payload (with traceback) to stdout
          // before exiting 1 — surface it instead of throwing it away.
          let detail = `worker exited with code ${code}`
          try {
            const parsed = JSON.parse(stdout.trim())
            if (parsed?.ok === false && parsed?.error) {
              const { kind, message, traceback } = parsed.error
              detail = `worker ${command} failed (${kind ?? 'unknown'}): ${message ?? 'unknown error'}`
              if (traceback) detail += `\n${String(traceback).slice(-3000)}`
            }
          } catch {
            // stdout was not a structured error payload; keep the generic message
          }
          reject(new WorkerInvocationError(detail, code, stderr.slice(-2000), stdout.slice(-2000)))
        }
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
