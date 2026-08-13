---
name: rh-process-rosbag-to-dataset
description: 把 rosbag 处理成可训练数据集——检查、转换、对齐、转换、切分、导出、版本固化并生成数据卡，全程不改原始数据。
whenToUse: 当用户想把录制的 rosbag 加工成 LeRobot/RLDS 风格数据集，或需要审计/复现某数据集的生产过程时。
modelInvocable: true
userInvocable: true
---

# rosbag 到数据集

固定检查顺序，全程非破坏——原始 rosbag 永不修改。

1. **检查 rosbag。** `rh_rosbag_inspect` 查看元数据：topic、类型、时长、消息数。列出全部 topic 供选择。
2. **转换。** `rh_data_convert_rosbag` 把选定 topic 转成中间格式（CSV 等），记录转换范围。
3. **对齐流。** `rh_data_align_streams` 对齐多流（相机/关节/控制）时间戳，记录对齐方法与残余偏差。
4. **应用转换。** `rh_data_transform_apply` 应用非破坏转换链（归一化、坐标系变换），转换结果写入派生目录。
5. **切分 episode 并导出。** `rh_data_segment_episodes` 按任务边界/时序切分 episode 并记录依据；`rh_data_export_lerobot` 导出 LeRobot 风格，如需 RLDS 用 `rh_data_export_rlds`（生成 manifest，注意其需 TF 增强）。
6. **固化版本并生成数据卡。** `rh_dataset_version_create` 固化版本并记录哈希；`rh_dataset_card_generate` 生成数据卡：来源、字段、统计、已知限制。

## 成功判据

- 全部步骤执行完毕，数据集版本已固化且数据卡存在；原始 rosbag 未发生任何修改（可用哈希证明）。

## 停止条件

- 关键 topic 缺失（如无相机流但任务需要视觉）：停止并询问用户是否继续；导出后端缺失（如 RLDS 需 TF 增强）时报告缺失项，选择替代导出或停止。

## 常见误判

- 转换成功 ≠ 数据对齐正确：时间戳对齐误差可能让"同步帧"错位数毫秒——检查对齐报告；episode 切分错误会让一段数据横跨两个 episode，切分依据必须记录且可复查。
