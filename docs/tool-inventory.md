# Tool inventory（工具清单）

> 每个 DSH 工具 `rh_*` 对应一个 worker 命令。硬件依赖模块（标记 🔌）在
> 后端缺失时返回结构化 `backend: "unavailable"` 诊断，而不是报错。
> 风险分级：R0 只读 · R1 本地派生 · R2 仿真 · R3 受控真机（未实现）· R4 禁止（永不暴露）。

## 资产 / CAD（assets, cad）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_worker_ping | ping | R0 | worker 健康与依赖版本 |
| rh_capability_list | capability-list | R0 | 能力清单 |
| rh_robot_asset_inspect | inspect-asset | R0 | URDF/MJCF 结构检查 |
| rh_urdf_validate | validate-urdf | R0 | URDF 校验 |
| rh_urdf_to_mjcf | convert-urdf | R1 | URDF→MJCF 转换 |
| rh_sdf_validate | sdf-validate | R0 | SDF 结构校验 |
| rh_cad_inventory | cad-inventory | R0 | CAD 目录清单与哈希 |
| rh_cad_compare_versions | cad-compare-versions | R0 | URDF/清单版本对比 |
| rh_mesh_inspect | mesh-inspect | R0 | STL/OBJ 网格统计 |
| rh_inertia_validate | inertia-validate | R0 | 惯量专项校验 |
| rh_robot_topology_validate | robot-topology-validate | R0 | 拓扑树/闭环检查 |
| rh_urdf_preview | urdf-preview | R1 | 运动链 SVG 预览 |
| rh_export_sim_asset | export-sim-asset | R1 | MJCF 导出 / SDF 兼容报告 |
| rh_generate_asset_report | asset-report | R1 | 资产 Markdown 报告 |

## ROS 2（ros）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_ros_graph_snapshot 🔌 | ros-graph-snapshot | R0 | node/topic/service/action 图 |
| rh_ros_topic_profile 🔌 | ros-topic-profile | R0 | topic 频率采样 |
| rh_ros_qos_check 🔌 | ros-qos-check | R0 | QoS 兼容检查 |
| rh_ros_tf_audit 🔌 | ros-tf-audit | R0 | TF 树/频率（rosbag 可用） |
| rh_ros_diagnostics_snapshot 🔌 | ros-diagnostics-snapshot | R0 | /diagnostics 快照 |
| rh_ros_controller_status 🔌 | ros-controller-status | R0 | ros2_control 状态 |
| rh_ros_moveit_audit 🔌 | ros-moveit-audit | R0 | SRDF/planning group 审计 |
| rh_rosbag_inspect | rosbag-inspect | R0 | rosbag2 元数据（SQLite，无需 ROS） |
| rh_rosbag_start 🔌 | rosbag-start | R1 | 受控 rosbag 录制（禁 C 盘） |
| rh_rosbag_stop 🔌 | rosbag-stop | R1 | 停止录制 |
| rh_ros_call_whitelisted_action 🔌 | ros-call-whitelisted-action | R2 | 白名单 Action 调用 |

## 控制（control）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_control_trace_analyze | control-trace-analyze | R0 | 阶跃/跟踪指标与异常 |
| rh_trajectory_validate | trajectory-validate | R0 | 轨迹连续性/限位 |
| rh_planned_actual_compare | planned-actual-compare | R0 | 计划 vs 实际对比 |
| rh_pid_experiment_prepare | pid-experiment-prepare | R1 | 阶跃/扫频实验模板 |
| rh_controller_config_compare | controller-config-compare | R0 | 控制器参数对比 |
| rh_system_identification_job | system-identification | R1 | 一阶/二阶辨识 |
| rh_control_report_generate | control-report | R1 | 控制分析报告 |

## 视觉与标定（vision_extra）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_camera_health_check | camera-health-check | R0 | 图像质量检查 |
| rh_calibration_inspect | calibration-inspect | R0 | 标定文件结构检查 |
| rh_pose_transform_validate | pose-transform-validate | R0 | 变换数值校验 |
| rh_perception_run | perception-run | R1 | 感知执行（颜色/显著度） |
| rh_perception_compare | perception-compare | R1 | 双感知结果对比 |
| rh_image_dataset_profile | image-dataset-profile | R0 | 图像集统计 |
| rh_annotate_failure_frame | annotate-failure-frame | R1 | 失败帧标注 |

## 具身模型（models）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_model_inventory | model-inventory | R0 | 模型注册表 |
| rh_model_health | model-health | R0 | 后端检测 |
| rh_model_warmup | model-warmup | R1 | 预热 |
| rh_model_infer_job | model-infer | R1 | 推理（内置演示模型可跑） |
| rh_model_benchmark | model-benchmark | R1 | 延迟基准 |
| rh_capability_route_explain | capability-route-explain | R0 | 规则能力路由 |
| rh_policy_rollout_compare | policy-rollout-compare | R2 | 策略仿真对比 |

## 仿真（simulation）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_sim_status | sim-status | R0 | 后端与场景状态 |
| rh_sim_validate_scenario | sim-validate-scenario | R0 | 场景校验 |
| rh_sim_run | sim-run | R2 | MuJoCo 抓取运行 |
| rh_sim_fault_inject | sim-fault-inject | R2 | 故障注入专用入口 |
| rh_sim_replay | sim-replay | R0 | 只读回放 |
| rh_sim_real_gap_report | sim-real-gap-report | R0 | 仿真-真机分布对比 |
| rh_sim_batch_benchmark | sim-batch-benchmark | R2 | 批量矩阵基准 |

## 实机实验（robots）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_robot_preflight 🔌 | robot-preflight | R3 | preflight 清单（真机项跳过） |
| rh_robot_state_snapshot 🔌 | robot-state-snapshot | R0 | 状态快照 |
| rh_experiment_prepare | experiment-prepare | R1 | 实验记录创建 |
| rh_experiment_request_approval | experiment-request-approval | R1 | 人工审批请求 |
| rh_experiment_start 🔌 | experiment-start | R3 | 审批后启动（需人工凭证） |
| rh_experiment_pause | experiment-pause | R2 | 暂停 |
| rh_experiment_safe_cancel | experiment-safe-cancel | R2 | 安全取消 |
| rh_experiment_status | experiment-status | R0 | 状态查询 |
| rh_experiment_finalize | experiment-finalize | R1 | 终态固化 |

## 遥测与诊断（telemetry）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_telemetry_channels | telemetry-channels | R0 | 通道清单 |
| rh_telemetry_window | telemetry-window | R0 | 时间窗提取 |
| rh_anomaly_scan | anomaly-scan | R0 | 确定性异常扫描 |
| rh_failure_evidence_collect | failure-evidence-collect | R0 | 失败证据收集 |
| rh_diagnose_run | diagnose-run | R0 | 规则诊断 |
| rh_run_compare | run-compare | R0 | 双 Run 对比 |
| rh_timeline_export | timeline-export | R1 | 时间线导出 |

## 数据处理（data_pipeline）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_data_inventory | data-inventory | R0 | 数据登记 |
| rh_data_schema_inspect | data-schema-inspect | R0 | schema 识别 |
| rh_data_quality_audit | data-quality | R0 | 质量审计 |
| rh_data_time_sync_estimate | data-time-sync-estimate | R0 | 时延估计 |
| rh_data_align_streams | data-align-streams | R1 | 多流对齐 |
| rh_data_transform_apply | data-transform-apply | R1 | 非破坏转换链 |
| rh_data_segment_episodes | data-segment-episodes | R1 | episode 切分 |
| rh_data_annotation_import | data-annotation-import | R1 | 标注导入 |
| rh_data_annotation_review | data-annotation-review | R1 | 标注复核 |
| rh_data_split_create | data-split-create | R1 | 防泄漏切分 |
| rh_data_leakage_check | data-leakage-check | R0 | 泄漏检查 |
| rh_data_deidentify | data-deidentify | R1 | 去标识化（非匿名化） |
| rh_data_convert_rosbag | data-convert-rosbag | R1 | rosbag→CSV |
| rh_data_export_lerobot | data-export-lerobot | R1 | LeRobot 风格导出 |
| rh_data_export_rlds | data-export-rlds | R1 | RLDS manifest（需 TF 增强） |
| rh_dataset_version_create | dataset-version-create | R1 | 版本固化 |
| rh_dataset_compare | dataset-compare | R0 | 版本对比 |
| rh_dataset_card_generate | dataset-card-generate | R1 | 数据卡 |

## 实验管理（experiment）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_experiment_spec_create | experiment-spec-create | R1 | 实验定义 |
| rh_experiment_matrix_expand | experiment-matrix-expand | R1 | 矩阵展开 |
| rh_benchmark_start | benchmark-start | R2 | 矩阵执行 |
| rh_metrics_compute | metrics-compute | R0 | 指标聚合 |
| rh_ablation_compare | ablation-compare | R0 | 消融对比 |
| rh_benchmark_report | benchmark-report | R1 | 实验报告 |

## 知识检索（knowledge）

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_docs_index | docs-index | R0 | 文档索引 |
| rh_manual_search | manual-search | R0 | 手册检索 |
| rh_error_code_lookup | error-code-lookup | R0 | 错误码查询 |
| rh_case_search | case-search | R0 | 案例检索 |

## 报告与面板

| DSH 工具 | Worker 命令 | 风险 | 说明 |
|---|---|---|---|
| rh_evidence_export | evidence-export | R1 | 证据包 |
| rh_report_generate | report-generate | R1 | Markdown 报告 + 时间线 |
| rh_dashboard_generate | dashboard-generate | R1 | 单文件仪表盘 |
| rh_demo | demo | R2 | 一键演示 |

> 🔌 = 需要外部后端（ROS 2 / SolidWorks / 真机适配器 / 重型模型）；缺失时工具可用但返回 `backend:"unavailable"`。
