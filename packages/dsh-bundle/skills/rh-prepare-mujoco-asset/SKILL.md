---
name: rh-prepare-mujoco-asset
description: 为 MuJoCo 准备机器人资产——检查、转换（URDF→MJCF）、校验场景并确认可加载，输出转换报告与差异清单。
whenToUse: 当用户需要把现有 URDF/SDF 资产用于 MuJoCo 仿真，或转换后加载失败需要排查时。
modelInvocable: true
userInvocable: true
---

# 为 MuJoCo 准备资产

固定检查顺序，转换是显式步骤，检查与转换分离。

1. **检查源资产。** `rh_robot_asset_inspect` 检查源 URDF/MJCF 的结构与 issue 计数。存在 error 时先报告，再决定是否继续转换。
2. **转换。** `rh_urdf_to_mjcf` 执行 URDF→MJCF 转换，记录输出路径与警告。若输入是 SDF，改用 `rh_sdf_validate` 先确认结构，再进入转换流程。
3. **校验场景。** `rh_sim_validate_scenario` 校验生成的场景：关节限位、默认姿态、接触对、几何穿透。
4. **确认可加载。** `rh_sim_status` 确认 MuJoCo 后端可用且新场景能被加载，记录加载结果。
5. **输出报告。** 生成转换报告：源/目标路径、转换警告、语义差异清单（丢失的 joint limit、被忽略的 mesh、单位换算），以及"转换后未做动力学验证"的声明。

## 成功判据

- 转换产出 MJCF 文件，且 `rh_sim_status` 确认可加载；差异清单列出所有已知语义变化，未静默丢弃任何属性。

## 停止条件

- 源资产存在 error 级结构问题且用户未确认继续：停止转换；后端缺失（`backend: "unavailable"`）时报告所需环境，不伪造成功。

## 常见误判

- "转换成功" ≠ "语义等价"：friction、limit、damping 等参数可能在转换中丢失。
- 场景校验通过只证明几何/结构合理，不证明动力学行为正确——需仿真运行进一步验证。
