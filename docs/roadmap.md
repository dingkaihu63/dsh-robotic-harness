# Robotic Harness 路线图

> 依据产品方案（robotic-harness-dsh-plugin-suite-plan.md）的阶段性安排，与当前 Demo 状态对照。

## 状态图例

- ✅ 已完成（本仓库 v0.1 Demo）
- 🔜 下一阶段
- ⏳ 规划中（等待社区/合作）
- ❌ 明确不做（第一版）

## 已交付（v0.1 Demo）

- ✅ DSH bundle + profile 安装流程（`dsh plugin --profile rh-demo add ./packages/dsh-bundle`）
- ✅ 12 个 `rh_*` 工具 + 6 个 Skill
- ✅ Python worker（资产检查 / URDF→MJCF / MuJoCo 抓取 / 故障注入 / 遥测 / 诊断 / 证据包 / 报告 / 时间线 / 数据质量审计）
- ✅ 领域模型（Run / DiagnosticCase / Capability / Artifact）
- ✅ 一键 Demo（`node scripts/demo.mjs`）+ 全部测试（20 个 pytest + TS 构建 + bundle 安装启动验证）

## Phase 1 — 数据模块最小闭环（🔜 下一阶段）

- rosbag/CSV/Parquet 元数据 inventory 与时间对齐；
- episode 切分、leakage-safe split、数据卡；
- LeRobotDataset 或 RLDS 导出（二选一）。

## Phase 2 — ROS 2 只读诊断（⏳ 需要 ROS 2 环境或贡献者）

- ROS graph / Topic/QoS / TF / diagnostics 只读快照；
- rosbag 分析与异常窗口；
- 控制器与 MoveIt 状态审计；
- 不开放真机写权限。
- 现有 Skill 模板：`rh-ros2-health-check`。

## Phase 3 — 视觉与标定（⏳）

- 相机健康检查、标定（内参/手眼）检查；
- 检测/分割/6D 位姿能力适配；
- 失败帧集合与多模型对比。

## Phase 4 — 科研实验管理（⏳）

- ExperimentSpec / 实验矩阵 / benchmark；
- Evidence Bundle 扩展为“可发表单元”（git commit、模型哈希、种子、统计方法）。

## Phase 5 — CAD / SolidWorks / FreeCAD（⏳ 需要 CAD 环境）

- STEP 清单与版本追踪、CAD→URDF 拓扑辅助、网格检查；
- SolidWorks 仅通过 Windows 可选 bridge（不在第一版承诺）。

## Phase 6 — 人体示教数据参考流程（⏳ 需要合规数据）

- 视频姿态/mocap/IMU 对齐；participant/session 级安全切分；脱敏与外部模型上传拦截。

## Phase 7 — 真机（⏳ 需要明确合作与硬件）

- preflight checklist → 审批 → 受控 Skill 执行 → 状态机；
- 遵循 docs/safety-boundary.md 的全部约束。

## 社区贡献入口

- 新 Capability adapter、Skill、Scenario、Failure Case、数据 importer/exporter、可视化面板、文档翻译、DSH 兼容测试。
- 见 CONTRIBUTING.md；每个独立模块可单独发布与使用，不必等待“大平台”。

## 长期方向

- 若插件边界不足（经真实使用证据证明），再讨论派生发行版或与 OpenRAL 的 manifest/diagnostics 互操作实验；当前不提前选择。
