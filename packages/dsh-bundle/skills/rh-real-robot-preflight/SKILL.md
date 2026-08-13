---
name: rh-real-robot-preflight
description: 真机实验准备模板——执行 preflight 清单、创建实验记录、发起人工审批，并说明实验状态机与权限边界。
whenToUse: 当用户要为真机实验做准备、走审批流程，或想了解实验从准备到启动的状态流转时。本 Skill 是模板：无真机适配器时同样适用，只是真机项会被跳过。
modelInvocable: true
userInvocable: true
---

# 真机实验准备（模板）

固定检查顺序，权限边界先行：**LLM 没有审批权，急停解除永远人工**。

1. **执行 preflight。** `rh_robot_preflight` 跑清单。真机相关项（硬件连接、安全回路）在无适配器时返回 skip 或 `backend: "unavailable"`——**如实记录跳过，不假装检查通过**。
2. **创建实验记录。** `rh_experiment_prepare` 创建实验记录：目标、步骤、风险与安全措施、参与人员。记录 preflight 的真实状态（通过/跳过/未验证）。
3. **发起审批。** `rh_experiment_request_approval` 向人工审批人发起审批请求，附上 preflight 与实验记录。**模型不得代替审批**，审批结果以人工返回为准。
4. **说明状态机。** 向用户说明：`prepared → pending_approval → approved → running`，`rh_experiment_status` 可查询状态；启动（`rh_experiment_start`）需要人工凭证且不在本模板自动执行。急停解除、上电等安全动作永远是人工操作。

## 成功判据

- preflight 状态（通过/跳过/未验证）逐项如实记录；审批请求已发出，状态机与权限边界已向用户说明。

## 停止条件

- 审批未通过或未返回：停止，不启动任何实验。
- 任何安全相关项处于"未验证"：高亮并等待人工确认，不继续推进启动。

## 常见误判

- 无适配器时跳过项 ≠ 通过项：报告必须区分，防止把跳过误当验证过。
- 模型请求审批 ≠ 模型批准实验：审批权在人工，模型只传递状态。
