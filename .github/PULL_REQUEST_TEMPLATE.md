## 变更说明

请用一句话说明这个 PR 做什么，并链接相关 Issue（如有）。

## 变更类型

- [ ] 🐛 Bug fix
- [ ] ✨ Feature
- [ ] 🧩 新模块 / Skill / 场景 / Failure Case
- [ ] 📝 文档
- [ ] 🔧 基础设施（CI / 发布流程）

## 检查清单

- [ ] 本地跑通 `cd python && python run_tests.py`（新增用例：正常路径 + 失败路径）
- [ ] 新增/修改的 worker 命令已按 docs/worker-module-contract.md 导出 `COMMANDS`/`CAPABILITIES`
- [ ] 若改了 `python/robotic_harness_worker/`，已运行 `node scripts/sync-worker.mjs` 并提交 bundle 副本
- [ ] TS 侧（如新增工具）已更新 `packages/dsh-bundle/src/tools.ts` 与 docs/tool-inventory.md
- [ ] 后端缺失时返回结构化 `backend:"unavailable"`（不报错、不假装通过）
- [ ] 无真机安全声明；文档/输出中的限制如实

## 验证证据

请粘贴关键输出（测试摘要、worker 返回的 JSON 摘要、截图/图表路径等），让 reviewer 可以核验，而不是只看到"测试通过"。

## 已知限制 / 未完成部分

如实列出（若有）。
