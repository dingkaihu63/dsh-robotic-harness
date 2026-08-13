---
name: rh-inspect-rosbag-run
description: 检查一次 rosbag 运行记录——查看元数据、选定 topic 提取数据、审计数据质量并输出检查报告。
whenToUse: 当用户需要了解某次录制的 rosbag 里有什么、某个 topic 的数据是否可信，或为后续分析准备证据时。
modelInvocable: true
userInvocable: true
---

# 检查 rosbag 运行记录

固定检查顺序，先元数据后内容，所有结果留作证据。

1. **检查元数据。** `rh_rosbag_inspect` 读取 rosbag2 元数据（无需 ROS 环境）：时长、消息数、topic 列表与类型。记录文件路径与哈希。
2. **选定 topic。** 根据用户问题选择目标 topic。无目标时优先列出所有 topic 与消息计数供用户选择，不擅自假设。
3. **提取数据。** 用 `rh_telemetry_window` 按时间窗提取，或用 `rh_data_convert_rosbag` 转出 CSV。记录提取范围与行数。
4. **审计质量。** `rh_data_quality_audit` 检查缺失值、时间戳跳变、频率异常与值域异常。
5. **输出报告。** 汇总 topic 清单、提取结果与质量发现；**未解码的消息类型必须显式列出**（类型名+计数），不得静默丢弃。

## 成功判据

- 元数据、提取、质量审计三步都有记录；报告中列出全部 topic 与未解码类型，无静默省略。

## 停止条件

- rosbag 路径不存在或 SQLite 损坏：停止并报告，不尝试猜测内容；用户未指定 topic 且消息类型多样时先问用户，不代选。

## 常见误判

- 消息数正常 ≠ 数据可用：时间戳跳变或频率骤降会使该 topic 不可用。
- 未解码类型不代表"无数据"——它只是未被解码，必须保留在清单里并提示用户。
