/**
 * Robotic Harness bundle entry: exports the three plugin modules so the
 * cordis.patch.yml rows can resolve `@robotic-harness/dsh-bundle/*`.
 */

export { name as coreName, apply as coreApply, Config as CoreConfig } from './core.ts'
export { name as toolsName, apply as toolsApply, Config as ToolsConfig } from './tools.ts'
export { name as skillsName, apply as skillsApply, Config as SkillsConfig } from './skills.ts'
