/**
 * rh-core: project/run workspace root for the Robotic Harness bundle.
 *
 * The worker owns the actual on-disk store (`.rh/` layout); this plugin
 * resolves the configured store root once and exposes it to the other
 * plugins through the shared config object. It performs no I/O beyond
 * validating the resolved value.
 */

import type { Context } from '@deepseek-ai/cordis'
import Schema from '@deepseek-ai/schemastery'

export const name = 'rh-core'

export interface Config {
  /** Run store root ('' means the worker's current directory + '/.rh'). */
  storeRoot: string
}

export const Config: Schema<Config> = Schema.object({
  storeRoot: Schema.string().default(''),
})

export function apply(ctx: Context, config: Config) {
  ctx.logger.info(`[rh] robotic-harness bundle loaded; storeRoot=${config.storeRoot || '<workspace>/.rh'}`)
}
