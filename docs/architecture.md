# Robotic Harness 架构与领域模型

> 版本：v0.1-demo。本文描述当前实现；Roadmap 见 [roadmap.md](roadmap.md)。

## 1. 总体结构

Robotic Harness 是 DeepSeek Harness 的**下游插件套件**：不复制或修改 DSH 核心，只使用官方扩展点（bundle patch、Tool 注册、Skill 注册、沙箱/审批策略）。

```text
┌──────────────────── DeepSeek Harness ────────────────────┐
│ Agent / Session / Tool / Skill / Sandbox / Approval / Web │
│                        │                                   │
│  ┌──── @robotic-harness/dsh-bundle ────────────────────┐  │
│  │ rh-core   项目/Run 存储根（.rh/ 布局）               │  │
│  │ rh-tools  12 个 rh_* 工具（schema/超时/取消/输出）    │  │
│  │ rh-skills 6 个 SKILL.md（固定检查顺序 + 停止条件）     │  │
│  └──────────────┬──────────────────────────────────────┘  │
└─────────────────┼─────────────────────────────────────────┘
                  │ stdio JSON（一次性进程，可取消）
┌─────────────────▼─────────────────────────────────────────┐
│ robotic_harness_worker（Python ≥3.10）                     │
│ assets / simulation / vision / diagnostics / data / report │
└────────────────────────────────────────────────────────────┘
```

### 为什么用一次性 stdio 进程而不是常驻服务

- 零部署成本：bundle 内随附 worker 包副本，`PYTHONPATH` 由 rh-tools 注入；
- 崩溃隔离：worker 崩溃不影响 DSH 进程；
- 协议简单：`python -m robotic_harness_worker <command> --input -`，stdout 返回单个 JSON；
- 取消语义：工具通过 `exec.signal` 终止子进程。

## 2. 统一领域模型（可移植，无 DSH 依赖）

Python 侧 `core.py` 定义最小领域对象，全部序列化为纯 JSON（未来可对接 OpenRAL 或独立脚本）：

| 对象 | 用途 | 关键字段 |
|---|---|---|
| `RoboticsProject` | 项目身份与环境快照 | id、root、git_commit、env_snapshot |
| `AssetInspection` | 资产检查报告 | summary、issues（severity/code/message/location/evidence） |
| `Capability` | 可被 Agent/Skill 选择的能力 | id、kind、provider、risk（R0–R4）、input/output |
| `Run` | 一次仿真运行 | config 快照、metrics、phases、anomalies、artifacts |
| `DiagnosticCase` | 问题定位记录 | findings（fact/rule/inference）、hypotheses（层/证据/反证/缺失证据/建议检查） |
| `Artifact` | 文件级产物 | 统一索引（路径/哈希），不强制统一保存格式 |

存储布局（默认 `<workspace>/.rh/`）：

```text
.rh/
├── index.json                 # Run 索引
├── runs/<run-id>/run.json     # 运行快照（配置+指标+阶段+异常）
├── runs/<run-id>/telemetry.jsonl
├── runs/<run-id>/artifacts/   # joints.png / tracking.png / trajectory.png / scene.png
└── cases/<case-id>.json       # 诊断案例
```

## 3. 仿真设计（simulation.py）

- **场景**：`mujoco_pick_place` —— 平面 3-DOF 臂（肩/肘/腕，XZ 平面）+ 吸盘，桌上红色方块，目标区在桌面对侧。
- **运动**：解析 IK（双解，按关节限位过滤 + 连续选取最近解，避免中途翻转）；`position` 伺服驱动，逐段轨迹插值 + 终点保持。
- **吸盘**：运动学吸附（吸附后每步覆写物体 qpos 跟随吸盘）；吸附判定 = 吸盘端面与物体中心距离 ≤ `attachRadius`。
- **感知**：相机模型（针孔，与 MuJoCo 约定一致）；渲染可用时做真实颜色分割，否则用真值+噪声模拟；两者记录 `renderer` 模式。
- **故障注入**（全部确定性，`random.Random(seed)`）：
  - `perception_offset_px`：分割质心像素偏移 → 抓取目标偏移；
  - `gripper_slip`：提升后吸盘失效，物体掉落；
  - `tf_offset`：物体估计值整体平移（模拟 TF 错误）；
  - `sensor_noise`：关节测量噪声；
  - `model_timeout_s` / `occlusion`：感知延迟与遮挡 → 路由降级/重观察。
- **遥测**：下采样 JSONL（t、q、qTarget、trackingError、cupPos、objPos、attached、suction、perception）。
- **成功率**：`completed && 物体落在目标区 && 曾吸附 && 未滑落`。

## 4. 诊断设计（diagnostics.py）

三层分离，禁止混写：

1. **事实（fact）**：来自 run.json/telemetry 的数据（时间戳、数值、配置、异常事件）。
2. **规则判定（rule）**：确定性检查（持续跟踪误差 > 阈值、感知估计与真值偏差、滑落事件、未吸附、成功率判定失败）。
3. **候选根因（inference/hypothesis）**：按故障层分组（perception / calibration / mechanical / control / system），带支持证据、反证、缺失证据、建议只读检查、可能性标注；`requiresHuman` 标记需要人工确认的项。

规则引擎从不给出最终结论 —— `DiagnosticCase.status` 默认 `open`，最终结论由人填写。

## 5. 数据质量审计（data_quality.py）

只读审计 CSV/JSONL 时序：缺失/NaN/Inf、重复与乱序时间戳（按原始顺序计数）、间隔异常（最大间隔 > 5×中位数判定为掉帧候选）、恒值通道、每通道统计。不修改原始文件。

## 6. 报告与证据（report.py）

- **EvidenceBundle**：`manifest.json`（schemaVersion、环境快照、run 摘要、文件清单 + sha256）+ run.json + telemetry.jsonl + 图表 + diagnostics.json；自包含、可整体归档。
- **Markdown 报告**：摘要、故障配置、阶段、异常、诊断（分三层）、artifact 列表、限制声明。
- **timeline.html**：单文件、内嵌 JSON、无服务器无网络的查看器（关节曲线、阶段/异常表、诊断表）。

## 7. TypeScript 侧设计（packages/dsh-bundle）

- `cordis.patch.yml`：单层 insert，三行（rh-core / rh-tools / rh-skills），行内 `config` 均可被用户 profile patch 覆盖。
- 工具全部通过 `defineTool` 声明（参数 schema 校验、canonical output、纯 render），执行体调用 `runWorker`（spawn + stdin JSON + 超时 + `exec.signal` 取消）。
- Skills 以 `skills/<name>/SKILL.md` 随包分发，`apply` 时解析 frontmatter 并 `ctx.skills.register`（source=bundled）；同一文件也可被用户拷入项目 `.dsh/skills` 使用。
- worker 副本、fixtures、scenarios 由 `node scripts/sync-worker.mjs` 同步进包，保证 npm 包自包含。

## 8. 兼容性声明

- DSH：`@deepseek-ai/dsh` ≥ 0.1.0-rc.6（CLI、bundle/profile 机制、Tool/Skill 注册 API）。
- Python：≥ 3.10；仿真需要 `mujoco`（可选 `numpy`/`opencv-python`/`matplotlib`）。
- 已知上游限制：`@deepseek-ai/dsh-web-app` 的 npm 发布依赖私有包 `@deepseek-ai/dsh-frontend`，从公共 registry 直接安装 web-app bundle 可能失败；本仓库演示通过 `dsh plugin add ./packages/dsh-bundle`（不依赖该包）加载，Web 面板由内置 dsh 安装目录提供。
