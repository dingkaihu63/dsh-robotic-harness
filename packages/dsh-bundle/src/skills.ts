/**
 * rh-skills: registers the bundled Robotic Harness SKILL.md files into the
 * DSH skill registry as runtime contributions.
 *
 * Skill bodies live in `<package>/skills/<name>/SKILL.md` (the standard
 * bundle layout), so the same files also work if a user copies them into a
 * project's `.dsh/skills` directory. Frontmatter requires `name` and
 * `description`; `whenToUse` is optional.
 */

import { readdir, readFile } from 'node:fs/promises'
import { existsSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

import type { Context } from '@deepseek-ai/cordis'
import type { SkillRegistration } from '@deepseek-ai/dsh-skill'
import Schema from '@deepseek-ai/schemastery'
import { parse as parseYaml } from 'yaml'

export const name = 'rh-skills'
export const inject = ['skills']

export interface Config {
  /** Directory that contains `<name>/SKILL.md` bundles; default: the installed package's skills/ dir. */
  skillsDir: string
}

export const Config: Schema<Config> = Schema.object({
  skillsDir: Schema.string().default(''),
})

function defaultSkillsDir(): string {
  return join(dirname(fileURLToPath(import.meta.url)), '..', 'skills')
}

interface SkillFile {
  registration: SkillRegistration
  path: string
}

function parseSkillFile(raw: string, path: string): SkillFile {
  const match = /^---\r?\n([\s\S]*?)\r?\n---\r?\n?([\s\S]*)$/.exec(raw)
  if (!match) {
    throw new Error(`skill file ${path} has no YAML frontmatter`)
  }
  const rawMeta = match[1] ?? ''
  const body = match[2] ?? ''
  let meta: Record<string, unknown>
  try {
    meta = (parseYaml(rawMeta) ?? {}) as Record<string, unknown>
  } catch (error) {
    throw new Error(`skill file ${path} has invalid YAML frontmatter: ${String(error)}`)
  }
  const name = typeof meta.name === 'string' ? meta.name : undefined
  const description = typeof meta.description === 'string' ? meta.description : undefined
  if (!name || !description) {
    throw new Error(`skill file ${path} frontmatter requires 'name' and 'description'`)
  }
  const whenToUse = typeof meta.whenToUse === 'string' ? meta.whenToUse : undefined
  return {
    path,
    registration: {
      name,
      description,
      ...(whenToUse ? { whenToUse } : {}),
      content: body,
      source: 'bundled',
      path,
      ...(meta.metadata && typeof meta.metadata === 'object'
        ? { metadata: meta.metadata as Record<string, unknown> }
        : {}),
    },
  }
}

export async function apply(ctx: Context, config: Config) {
  const skillsDir = config.skillsDir?.trim() || defaultSkillsDir()
  if (!existsSync(skillsDir)) {
    ctx.logger.warn(`[rh] skills dir not found: ${skillsDir}`)
    return
  }

  const entries = await readdir(skillsDir, { withFileTypes: true })
  let registered = 0
  for (const entry of entries) {
    if (!entry.isDirectory()) continue
    // Match the skill file case-insensitively: two bundled skills ship as
    // lowercase `skill.md`, and on case-sensitive filesystems (Linux/macOS)
    // a literal 'SKILL.md' probe would silently skip them.
    const skillFile = (await readdir(join(skillsDir, entry.name))).find(
      (name) => name.toLowerCase() === 'skill.md',
    )
    if (!skillFile) continue
    const skillPath = join(skillsDir, entry.name, skillFile)
    try {
      const raw = await readFile(skillPath, 'utf8')
      const skill = parseSkillFile(raw, skillPath)
      ctx.skills.register(skill.registration)
      registered += 1
      ctx.logger.debug(`[rh] registered skill ${skill.registration.name}`)
    } catch (error) {
      ctx.logger.warn(`[rh] skipping skill ${entry.name}: ${String(error)}`)
    }
  }
  ctx.logger.info(`[rh] registered ${registered} skills from ${skillsDir}`)
}
