---
name: rh-audit-dataset-leakage
description: 审计数据集泄漏风险——登记数据、按 group 模式防泄漏切分、检查泄漏并给出按 participant/episode 分组的修复建议。
whenToUse: 当用户要切分数据集用于训练/评测，或怀疑现有切分存在泄漏（同一 episode 的帧同时出现在 train 与 test）时。
modelInvocable: true
userInvocable: true
---

# 数据集泄漏审计

固定检查顺序，先登记后切分，时序数据禁止按帧随机切分。

1. **登记数据。** `rh_data_inventory` 登记数据集：样本数、字段、来源与版本。
2. **按组切分。** `rh_data_split_create` 使用 **group 模式**切分：按 participant/episode/场景分组，保证同一组的样本整体落入同一分区。若工具支持多种分组键，优先选择与任务语义一致的键。
3. **检查泄漏。** `rh_data_leakage_check` 检查跨分区重复/邻近样本：同帧、同 episode 的相邻帧、同 participant 的相似样本。
4. **修复建议。** 发现泄漏时给出按 participant/episode 重新分组的方案，量化泄漏样本占比。修复由用户确认后执行。

## 成功判据

- 切分使用 group 模式并有分组键记录；泄漏检查有量化结果，发现泄漏时有明确修复建议。

## 停止条件

- 分组键不明确（数据无 participant/episode 字段）：停止并询问用户，不用文件名猜测。
- 泄漏修复涉及删除/移动数据：先展示方案，等用户确认，不擅自改动。

## 常见误判

- **时序/示范数据不得按帧随机切分**：同一 episode 的帧进不同分区会制造虚假泛化。
- "没有完全相同的帧" ≠ "无泄漏"：同 episode 相邻帧高度相关，同样造成泄漏。
