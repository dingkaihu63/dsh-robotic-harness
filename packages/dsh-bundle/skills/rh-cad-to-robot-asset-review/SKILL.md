---
name: rh-cad-to-robot-asset-review
description: 审查 CAD 资产到机器人资产的转换链——扫描 CAD 清单、检查网格、校验惯量与拓扑，并生成资产审查报告。
whenToUse: 当用户提供 CAD 目录（SolidWorks/STEP/STL/OBJ）并要求评估其作为机器人仿真资产的可信度时，或在转换前做资产审查时。
modelInvocable: true
userInvocable: true
---

# CAD 资产审查

固定检查顺序，逐步记录证据；结论留给人，模型只呈现事实与规则发现。

1. **扫描 CAD 清单。** `rh_cad_inventory` 列出目录内文件、类型与哈希。SolidWorks 原生文件只登记、不解析——若清单中出现，明确标注"仅登记，未解析"。
2. **检查网格。** `rh_mesh_inspect` 检查 STL/OBJ 网格：顶点数、面数、非流形边、空网格。记录异常网格的文件与索引。
3. **校验惯量。** `rh_inertia_validate` 检查惯量正定性、量级与质心合理性。零惯量对固定件可接受，先确认部件类型再下结论。
4. **校验拓扑。** `rh_robot_topology_validate` 检查运动链树结构、闭环与冗余自由度。闭链或柔性件（弹簧/线缆）会破坏树假设，必须标记为"需人工确认"，不得自动修复。
5. **生成报告。** `rh_generate_asset_report` 汇总上述发现，输出 Markdown 报告。报告中区分事实、规则发现与推测。

## 成功判据

- 五个步骤全部执行完毕，报告存在且包含各步的 issue 计数；每个 `error` 都给出具体文件/位置，未静默跳过任何发现。

## 停止条件

- CAD 目录不存在或为空：停止并询问用户路径；任何一步返回 `backend: "unavailable"`（如 SolidWorks 后端缺失）时如实报告，不伪造解析结果。

## 常见误判

- 网格存在 ≠ 网格可用：非流形或退化面仍会导致仿真穿模。
- 闭链/柔性件被当成普通树链处理——这是审查报告必须留给人工的结论。
