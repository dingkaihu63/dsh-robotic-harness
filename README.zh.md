# Robotic Harness（RH）— DeepSeek Harness 具身智能研发插件套件（Demo）

> 实验性 DSH 插件套件：把机器人资产、仿真、能力编排和故障证据放入同一个 Agent 工作流。
> 当前参考实现覆盖 **MuJoCo 抓取 Demo**（资产检查 → 仿真 → 故障注入 → 遥测 → 规则诊断 → 证据导出 → 报告）。
> 欢迎 ROS 2、CAD、视觉、控制和 VLA 开发者共同定义下一版接口。

> **致各位测试者与贡献者**：本项目目前处于 Demo 阶段，仅在部分本地环境（Windows + Anaconda Python 3.10 + DSH 0.1.0-rc.6）中验证过，ROS 2、CAD、真机以及其它操作系统/硬件环境**尚未充分试验**，使用中如遇问题敬请谅解，欢迎提出 Issue 反馈。
> 我们**欢迎任何人测试、修改、扩展本插件**，也欢迎把你在 ROS 2 / CAD / 视觉 / 控制 / VLA 等方向上的机器人相关插件与本套件组合在一起，**共同做成一个更大的完整机器人插件包** —— 每个独立模块都可以单独发布与贡献，详见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 开发范围与状态（测试前请先读）

| 模块 | 本仓库状态 |
|---|---|
| 机器人资产检查（URDF/MJCF/SDF：link/joint/惯量/碰撞） | ✅ 已实现 |
| CAD：清单/版本对比/网格检查/惯量与拓扑校验/SVG 预览/URDF→MJCF 与 SDF 兼容导出/资产报告 | ✅ 已实现（SolidWorks 文件只登记不解析） |
| URDF 校验 与 URDF→MJCF 转换 | ✅ 已实现 |
| MuJoCo 抓取仿真 + 故障注入 + 批量基准 + 只读回放 + 仿真-真机差距报告 | ✅ 已实现 |
| 感知路由 + 相机健康/标定检查 + 位姿校验 + 感知对比 | ✅ 已实现 |
| 确定性诊断 + 遥测通道/时间窗/异常扫描/证据收集/Run 对比 | ✅ 已实现 |
| 数据处理：清单/schema/时间同步/对齐/转换/episode/标注/防泄漏切分/去标识化/rosbag 转换/LeRobot 导出/数据集版本与数据卡 | ✅ 已实现（RLDS=manifest；parquet 可选） |
| 实验管理：spec/矩阵/基准/指标/消融/报告 | ✅ 已实现 |
| 控制分析：跟踪指标/轨迹校验/计划-实际对比/PID 模板与配置对比/系统辨识 | ✅ 已实现 |
| 具身模型注册表：内置演示模型真实可跑；外部/重型模型诚实探测 | ✅ 已实现（适配器模式） |
| 知识检索：文档索引/手册检索/错误码/案例检索 | ✅ 已实现 |
| 真机实验状态机 + preflight | ✅ 已实现（状态机）；无适配器时真机项标记 skip，绝不假装通过 |
| ROS 2 只读诊断（graph/TF/QoS/控制器） | 🔌 适配器已实现；实机探测需 `ros2` CLI；**rosbag2 检查无需 ROS** |
| 单文件仪表盘与时间线查看器 | ✅ 已实现（静态快照，非实时） |
| DSH Web 客户端插件面板 | ⏳ 路线图（当前用静态 HTML 查看器代替） |
| 人体示教数据与隐私流程 | ✅ 已实现（去标识化工具集）；需要合规数据 |
| SolidWorks API / FreeCAD 深度集成、Isaac、真机适配器 | ⏳ 路线图 |

**结论**：方案中的工具/Skill 面已按"Demo 级适配器"全部实现——纯软件模块完整且有测试；硬件/后端依赖模块（ROS 2 实机、SolidWorks、真机、重型 VLA）以诚实适配器形式存在，后端缺失时返回结构化 `backend:"unavailable"` 诊断并附安装指引，绝不假装可用。完整 100 工具清单见 [docs/tool-inventory.md](docs/tool-inventory.md)，[路线图](docs/roadmap.md) 列出仍需真实硬件/后端验证的部分。

## 30 秒架构

```text
DSH profile（rh-demo）
  └─ @robotic-harness/dsh-bundle（本仓库）
       ├─ rh-core    项目/Run 存储根解析（.rh/ 布局，全部在 workspace 内）
       ├─ rh-tools   ~100 个机器人领域工具（10 大域，见 docs/tool-inventory.md）
       └─ rh-skills  25 个 SKILL.md（资产/CAD/ROS/控制/视觉/模型/仿真/实机/数据/实验/知识）
              │  stdio JSON（一次性进程）
              ▼
       robotic_harness_worker（Python 3.10，随包分发）
        ├─ assets/cad   URDF/MJCF/SDF 检查、惯量、拓扑、网格、SVG 预览、转换
        ├─ simulation   MuJoCo 抓取、故障注入、批量基准、只读回放、仿真-真机差距
        ├─ vision       颜色/通用分割、相机健康、标定、位姿校验
        ├─ control      跟踪指标、轨迹校验、系统辨识、PID 模板
        ├─ models       具身模型注册表 + 内置演示适配器 + 诚实后端探测
        ├─ diagnostics  规则引擎（事实/规则/假设）+ 遥测异常扫描
        ├─ robots       真机实验状态机 + preflight
        ├─ data         清单/同步/转换/切分/去标识化/rosbag/LeRobot/数据集版本
        ├─ experiment   spec/矩阵/基准/指标/消融
        ├─ ros          ros2 实机探测（适配器）+ 免 ROS 的 rosbag2 检查
        └─ knowledge    文档索引/检索、错误码、案例检索
```

## 一键 Demo（无需 DSH，纯 Python）

环境要求：Python ≥ 3.10，`mujoco`、`numpy`、`opencv-python`、`matplotlib`、`pytest`。
（推荐 Anaconda 的 `python3.10` 环境；本仓库的所有产物与缓存均落在仓库目录内，不写 C 盘。）

```sh
# 1) 单元测试（每个测试文件独立进程运行，规避 mujoco/cv2/pyarrow 原生 DLL 冲突
#    ——与 worker 一次性进程的生产形态一致）
cd python && python run_tests.py

# 2) 端到端 Demo：正常 Run + 故障 Run + 诊断 + 证据包 + Markdown 报告 + 时间线 + 仪表盘
PYTHON=<你的 python3.10> node scripts/demo.mjs
# 输出在 examples/demo-output/ ：
#   report-run-*.md           实验报告（含证据与假设）
#   timeline-run-*.html       独立时间线查看器（浏览器直接打开，无需服务器）
#   bundle-run-*/             自包含证据包（manifest + 哈希 + 遥测 + 图表）
#   dashboard.html            单文件仪表盘
```

## 安装为 DSH 插件（bundle）

要求：DSH CLI（`@deepseek-ai/dsh` ≥ 0.1.0-rc.6）、pnpm、Python 3.10 环境。

```sh
# 0) 环境准备（示例：把一切放在 F 盘，DSH_HOME 指向 F 盘目录）
export DSH_HOME=/f/dsh/.dsh-home
export PATH="/f/dsh/.tools:$PATH"            # pnpm 所在目录

# 1) 创建 profile 并安装 bundle
dsh plugin --profile rh-demo add ./packages/dsh-bundle

# 2) 添加 Web 面板。注意：当前上游 npm 发布中 @deepseek-ai/dsh-web-app
#    依赖私有包 @deepseek-ai/dsh-frontend（registry 404），无法直接 pnpm add。
#    内置 bundle 从 dsh 安装目录解析，因此手动编辑 profile 清单即可：
#    把 $DSH_HOME/profiles/rh-demo/package.json 的 dsh.profile.bundles 改为
#    ["@deepseek-ai/dsh-base", "@deepseek-ai/dsh-web-app", "@robotic-harness/dsh-bundle"]
#    （编辑器注意：文件必须是 UTF-8 无 BOM）

# 3) （本机示例）在 profile 的 cordis.patch.yml 中把 rh-tools.pythonPath
#    指向你的 Anaconda python3.10 解释器（patch 会整体替换 config，需重述全部键）

# 4) 启动 Web UI
dsh --profile rh-demo --port 3080
```

安装后，Agent 拥有 **约 100 个 `rh_*` 工具与 25 个 Skill**，覆盖十大领域：资产/CAD、ROS 2（适配器）、控制、视觉与标定、具身模型、仿真、真机实验、遥测与诊断、数据处理、实验管理、知识检索。完整工具表（工具 → worker 命令 → 风险分级）见 [docs/tool-inventory.md](docs/tool-inventory.md)。

例如对 Agent 说：

> “运行 Robotic Harness 的 pick-place demo：检查 demo 机械臂，跑一次正常仿真和一次带故障的仿真，诊断失败原因，导出证据包并生成报告。”

工具会调用 Python worker（`python -m robotic_harness_worker <command> --input -`），所有 Run、遥测、图表、报告默认写入 workspace 的 `.rh/` 目录。分域速览：

| 领域 | 代表工具 |
|---|---|
| 资产/CAD | `rh_robot_asset_inspect`、`rh_urdf_validate`、`rh_urdf_to_mjcf`、`rh_sdf_validate`、`rh_cad_inventory`、`rh_mesh_inspect`、`rh_inertia_validate`、`rh_robot_topology_validate`、`rh_urdf_preview`、`rh_export_sim_asset`、`rh_generate_asset_report` |
| ROS 2 | `rh_ros_graph_snapshot`、`rh_ros_topic_profile`、`rh_ros_qos_check`、`rh_ros_tf_audit`、`rh_rosbag_inspect`（免 ROS）、`rh_rosbag_start/stop`、`rh_ros_call_whitelisted_action` |
| 控制 | `rh_control_trace_analyze`、`rh_trajectory_validate`、`rh_planned_actual_compare`、`rh_pid_experiment_prepare`、`rh_controller_config_compare`、`rh_system_identification_job` |
| 视觉 | `rh_camera_health_check`、`rh_calibration_inspect`、`rh_perception_run`、`rh_perception_compare`、`rh_pose_transform_validate`、`rh_annotate_failure_frame` |
| 模型 | `rh_model_inventory`、`rh_model_health`、`rh_model_infer_job`、`rh_model_benchmark`、`rh_capability_route_explain`、`rh_policy_rollout_compare` |
| 仿真 | `rh_sim_run`、`rh_sim_fault_inject`、`rh_sim_batch_benchmark`、`rh_sim_replay`、`rh_sim_real_gap_report`、`rh_sim_validate_scenario` |
| 实机 | `rh_robot_preflight`、`rh_experiment_prepare/request_approval/start/pause/safe_cancel/status/finalize` |
| 遥测 | `rh_telemetry_channels`、`rh_telemetry_window`、`rh_anomaly_scan`、`rh_failure_evidence_collect`、`rh_run_compare`、`rh_diagnose_run`、`rh_timeline_export` |
| 数据 | `rh_data_inventory`、`rh_data_time_sync_estimate`、`rh_data_align_streams`、`rh_data_transform_apply`、`rh_data_split_create`、`rh_data_leakage_check`、`rh_data_deidentify`、`rh_data_convert_rosbag`、`rh_data_export_lerobot`、`rh_dataset_version_create`、`rh_dataset_card_generate` |
| 实验 | `rh_experiment_spec_create`、`rh_experiment_matrix_expand`、`rh_benchmark_start`、`rh_metrics_compute`、`rh_ablation_compare`、`rh_benchmark_report` |
| 知识 | `rh_docs_index`、`rh_manual_search`、`rh_error_code_lookup`、`rh_case_search` |
| 报告 | `rh_evidence_export`、`rh_report_generate`、`rh_dashboard_generate` |

## 演示内容（v0.1）

- **场景**：平面 3 自由度机械臂 + 吸盘，桌上红色方块抓取 → 目标区放置（MuJoCo，纯基元构建，无外部网格）。
- **感知路由**：颜色分割（低延迟）→ 失败/遮挡时通用分割（边缘显著度），记录路由原因。
- **故障注入**（确定性，seed 可控）：`perception_offset_px`、`gripper_slip`、`tf_offset`、`sensor_noise`、`model_timeout_s`、`occlusion`。
- **遥测**：关节目标/实际/误差、吸盘状态、物体位姿、感知估计 vs 真值；图表（joints/tracking/trajectory）与场景渲染图。
- **诊断**：规则引擎产出分层证据 —— 事实（含时间戳与数值）、规则判定（阈值/状态机）、候选根因（按 感知/标定/机械/控制/系统 分层，标注可能性与缺失证据），**最终结论留给人**。
- **证据**：自包含证据包（哈希清单 + 全部记录）+ Markdown 报告 + timeline.html。

### 已知限制（如实说明）

- 吸盘抓取为**运动学实现**（吸附后物体位姿跟随吸盘），已在 run 配置与报告中注明。
- 感知在渲染器可用时使用真实离屏渲染；不可用时退化为“真值+噪声”的模拟感知（记录在遥测中）。若 OpenCV 原生崩溃（如极端环境下的 DLL 冲突），感知同样降级到该回退路径，而不是让 Run 失败。
- ROS 2 实机工具需要 `ros2` CLI；缺失时返回结构化 `backend:"unavailable"` 诊断。rosbag2 的检查与转换无需 ROS。
- 真机工具是状态机 + preflight：无硬件适配器时真机项如实标记 `skip`（绝不假装通过）。仿真结果不是真机证据；不提供任意 Topic 发布、真机写操作或急停解除能力。
- SolidWorks 文件只登记不解析（商业软件）；FreeCAD 深度集成可选。
- RLDS 导出为 manifest 骨架（完整 TFDS 导出需 tensorflow）；LeRobot 导出在 pyarrow 可用时用 parquet，否则降级 CSV。

## 目录结构

```text
packages/dsh-bundle/   可安装 DSH bundle（TS 插件、skills/、worker 副本、fixtures、scenarios）
python/                robotic_harness_worker Python 包 + 测试（run_tests.py）
fixtures/              URDF/SDF 测试资产 + 演示 rosbag2（无需 ROS）
scenarios/             MuJoCo 场景定义（JSON）
scripts/               sync-worker / demo / smoke-worker
docs/                  架构、安全边界、路线图、Demo 说明、工具清单、worker 契约
examples/demo-output/  一键 Demo 的输出示例
```

## 文档

- [架构与领域模型](docs/architecture.md)
- [安全边界](docs/safety-boundary.md)
- [路线图](docs/roadmap.md)
- [Demo 说明](docs/demo.md)
- [贡献指南](CONTRIBUTING.md) · [安全策略](SECURITY.md) · [第三方声明](THIRD_PARTY_NOTICES.md)
- English: [README.md](README.md)

## 许可证

MIT。第三方组件与资产遵循各自许可（见 THIRD_PARTY_NOTICES.md）。
本仓库与 DeepSeek 官方无隶属关系；DSH 是独立项目（MIT，[deepseek-harness](https://github.com/deepseek-ai/deepseek-harness)）。
