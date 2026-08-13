---
name: rh-register-embodied-model
description: 登记具身模型到注册表——查看现有条目、录入模型 manifest、检测后端健康并验证能力可路由。
whenToUse: 当用户要注册一个新模型（视觉/策略/VLM）供 Robotic Harness 使用，或现有模型路由失败需要排查登记状态时。
modelInvocable: true
userInvocable: true
---

# 登记具身模型

固定检查顺序，先查现状，再登记，后验证。

1. **查看现有注册表。** `rh_model_inventory` 列出已登记模型与能力。确认目标模型是否已存在、避免重复登记。
2. **收集 manifest 字段。** 通过 registry JSON 录入模型信息；若无 registry JSON，向用户询问 manifest 字段：模型 ID、类型、能力（vision/policy/vlm）、后端地址、模型文件路径、版本。字段缺失时如实列出缺失项。
3. **检测后端。** `rh_model_health` 检测模型后端可达性。重模型（大权重/远程服务）无后端或加载失败时，**如实报告"已登记但后端不可用"**，不得伪造健康状态。
4. **验证可路由。** `rh_capability_route_explain` 验证模型能否被规则能力路由命中：能力标签、路由规则、冲突（多模型同能力时如何裁决）。
5. **汇总。** 输出登记结果：新条目、健康状态、路由结论。登记完成≠可推理，明确区分。

## 成功判据

- 注册表现状已查看，manifest 字段齐全或缺失项已列出；健康与路由验证完成，结论区分"登记成功"与"可用"。

## 停止条件

- 用户未提供必要 manifest 字段且无 registry JSON：停止并询问，不猜字段；后端不可用时登记可完成，但必须报告不可用，不得标记为 ready。

## 常见误判

- 登记成功 ≠ 模型可用：后端未加载时推理会失败，别把登记当就绪。
- 能力标签模糊导致路由错误：路由结论必须以 `rh_capability_route_explain` 结果为准。
