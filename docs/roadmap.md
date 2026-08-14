# Robotic Harness 路线图

> 依据产品方案（robotic-harness-dsh-plugin-suite-plan.md）的阶段性安排，与当前实现状态对照。

## 状态图例

- ✅ 已完成（本仓库已实现并有测试）
- 🔌 适配器已实现（后端缺失时诚实返回 `backend:"unavailable"`）
- ⏳ 规划中（等待社区/合作/真实硬件）
- ❌ 明确不做（第一版）

## 已交付（全量工具面）

- ✅ DSH bundle + profile 安装流程（`dsh plugin --profile rh-demo add ./packages/dsh-bundle`）
- ✅ ~100 个 `rh_*` 工具 + 25 个 Skill（十大领域，见 docs/tool-inventory.md）
- ✅ 资产/CAD：URDF/MJCF/SDF 检查、惯量、拓扑、网格、SVG 预览、URDF→MJCF、SDF 兼容导出、资产报告
- ✅ 仿真：MuJoCo 抓取、故障注入、批量基准、只读回放、sim-real gap 报告
- ✅ 控制：跟踪指标、轨迹校验、计划-实际对比、PID 模板与配置对比、系统辨识、报告
- ✅ 视觉：相机健康、标定检查、位姿校验、感知执行/对比、图像集画像、失败帧标注
- ✅ 模型：注册表 + 内置演示适配器 + 后端探测 + 规则路由 + 策略对比
- ✅ 遥测诊断：通道/时间窗/异常扫描/失败证据收集/Run 对比/时间线
- ✅ 数据处理：清单/schema/时间同步/对齐/非破坏转换/episode/标注/防泄漏切分/去标识化/rosbag 转换/LeRobot 导出/数据集版本与数据卡
- ✅ 实验管理：spec/矩阵/基准/指标/消融/报告
- ✅ 知识检索：文档索引/手册检索/错误码/案例检索
- ✅ 真机实验状态机 + preflight（真机项无适配器时如实 skip）
- ✅ 单文件仪表盘/时间线查看器；一键 Demo（`node scripts/demo.mjs`）
- ✅ 全部测试（274 个用例，`python run_tests.py` 逐文件隔离）+ TS 构建 + bundle 安装启动验证

## 下一阶段：真实后端验证（🔌 → ✅ 需要环境）

- **ROS 2 实机**：在装有 ROS 2 的机器上验证 graph/TF/QoS/diagnostics/controller 探测；开放 rosbag 录制与白名单 Action。
- **SolidWorks / FreeCAD**：验证 STEP 解析增强与装配遍历（FreeCAD 后端）。
- **真机适配器**：接入具体硬件后验证 preflight 检查项与受控执行流程（遵循 docs/safety-boundary.md）。
- **重型模型**：验证 PyTorch/端点上模型的 health/warmup/infer/benchmark 适配。
- **Isaac Lab / Gazebo**：SDF 校验对接与第二仿真后端。

## 后续增量（⏳）

- DSH Web 客户端插件面板（当前为静态 HTML 查看器）；
- LeRobot parquet 全量导出、RLDS TFDS 完整导出；
- 人体示教数据参考流程（需要合规数据）；
- 跨平台 CI（GitHub Actions 无界面回归）。

## 社区贡献入口

- 新 Capability adapter、Skill、Scenario、Failure Case、数据 importer/exporter、可视化面板、文档翻译、DSH 兼容测试。
- 见 CONTRIBUTING.md；每个独立模块可单独发布与使用，不必等待"大平台"。
