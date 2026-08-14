---
name: 🧩 New module / capability
about: Propose or contribute a new domain module, Skill, scenario, failure case or data adapter
title: "[module] "
labels: enhancement, good first issue
---

**模块类型**（勾选或说明）
- [ ] 新 Skill（`packages/dsh-bundle/skills/<name>/SKILL.md`）
- [ ] 新仿真场景（`scenarios/`）
- [ ] 新 Failure Case（含已知根因与证据）
- [ ] 新数据 importer/exporter 或质量规则
- [ ] 新领域 worker 模块（遵循 docs/worker-module-contract.md）
- [ ] 其它：______

**对应方案章节**
（如 §6 CAD / §7 ROS 2 / §14 数据处理……）

**范围与接口**
- 工具名与命令名（`rh_xxx` ↔ `xxx-command`）
- 输入参数与关键输出字段
- 风险级别与安全边界（参考 docs/safety-boundary.md）

**测试计划**
- 至少 1 个正常路径 + 1 个失败路径的 pytest 用例
- 后端缺失时的诚实降级行为（`backend:"unavailable"`）

**你是来认领实现的吗？**
- [ ] 是，我提交 PR（会附测试结果与证据）
- [ ] 否，先记录想法供社区认领
