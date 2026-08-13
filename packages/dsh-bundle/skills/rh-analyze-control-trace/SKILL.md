---
name: rh-analyze-control-trace
description: 分析控制跟踪数据（阶跃/跟踪响应），解读 riseTime/overshoot/steadyState 等指标，识别异常并生成控制分析报告。
whenToUse: 当用户提供控制跟踪数据，需要评估控制器响应质量、定位响应异常，或为调参决策提供证据时。
modelInvocable: true
userInvocable: true
---

# 控制跟踪分析

固定检查顺序，指标先行，异常后置。

1. **分析跟踪数据。** `rh_control_trace_analyze` 计算阶跃/跟踪指标：riseTime、overshoot、settlingTime、steadyStateError 等。记录输入数据来源与时间范围。
2. **解读指标。** 逐项解读：riseTime 过长 → 响应慢；overshoot 过大 → 过冲；steadyStateError 非零 → 存在稳态误差。每个判断给出数值依据。
3. **识别异常。** 检查振荡、饱和（输出贴限）、积分 windup 等现象。异常要与指标关联：例如 overshoot 反复出现往往伴随振荡。
4. **生成报告。** `rh_control_report_generate` 输出报告，包含指标表、异常清单与"一次实验不足以定论"的说明。

## 成功判据

- 三个主要指标（riseTime/overshoot/steadyState）都有解读，异常均有数值依据，报告已生成。

## 停止条件

- 数据为空或时间戳无法解析：停止并说明，不编造指标。
- 数据覆盖不足（如未到稳态就结束）：标注指标不完整，不强行补全。

## 常见误判

- **一次阶跃 ≠ 全面结论**：单次响应无法证明鲁棒性，需多次重复或不同工况。
- 饱和不一定是坏事——在有意的限幅设计中是预期行为；先确认控制器的限幅意图再定性。
