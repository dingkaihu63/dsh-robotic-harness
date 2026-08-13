# Robotic Harness（RH）— DeepSeek Harness 具身智能研发插件套件（Demo）

> 实验性 DSH 插件套件：把机器人资产、仿真、能力编排和故障证据放入同一个 Agent 工作流。
> 当前参考实现覆盖 **MuJoCo 抓取 Demo**（资产检查 → 仿真 → 故障注入 → 遥测 → 规则诊断 → 证据导出 → 报告）。
> 欢迎 ROS 2、CAD、视觉、控制和 VLA 开发者共同定义下一版接口。

> **致各位测试者与贡献者**：本项目目前处于 Demo 阶段，仅在部分本地环境（Windows + Anaconda Python 3.10 + DSH 0.1.0-rc.6）中验证过，ROS 2、CAD、真机以及其它操作系统/硬件环境**尚未充分试验**，使用中如遇问题敬请谅解，欢迎提出 Issue 反馈。
> 我们**欢迎任何人测试、修改、扩展本插件**，也欢迎把你在 ROS 2 / CAD / 视觉 / 控制 / VLA 等方向上的机器人相关插件与本套件组合在一起，**共同做成一个更大的完整机器人插件包** —— 每个独立模块都可以单独发布与贡献，详见 [CONTRIBUTING.md](CONTRIBUTING.md)。

> *Testers & contributors: this is an early demo validated only in a limited local environment (Windows + Anaconda Python 3.10 + DSH 0.1.0-rc.6). ROS 2, CAD, real-robot and other OS/hardware setups are not yet fully tested — please forgive rough edges, report issues, and feel free to test, modify, and extend this plugin suite.*

- **是什么**：DeepSeek Harness 的插件（bundle）+ Python 侧车（worker），不是新平台。
- **不是什么**：不是机器人操作系统，不是仿真器替代品，不是功能安全系统，不是真机控制器。
- **为什么是 DSH 插件**：直接复用 DSH 的 Agent 循环、工具注册、Skill、沙箱与审批机制，把机器人研发全流程组织成可追踪、可复现的实验资产。

## 30 秒架构

```text
DSH profile（rh-demo）
  └─ @robotic-harness/dsh-bundle（本仓库）
       ├─ rh-core    项目/Run 存储根解析（.rh/ 布局，全部在 workspace 内）
       ├─ rh-tools   12 个机器人领域工具（资产检查/仿真/诊断/数据质量）
       └─ rh-skills  6 个 SKILL.md（inspect-asset / pick-place-demo / diagnose / benchmark / evidence / data-quality）
              │  stdio JSON（一次性进程）
              ▼
       robotic_harness_worker（Python 3.10，随包分发）
        ├─ assets       URDF/MJCF 检查、惯量校验、URDF→MJCF 转换
        ├─ simulation   MuJoCo 平面 3 自由度吸盘抓取场景 + 故障注入
        ├─ vision       颜色分割 → 通用分割（规则路由）
        ├─ diagnostics  确定性规则引擎（事实 / 规则 / 推断分层）
        └─ data         CSV/JSONL 时序质量审计
```

## 一键 Demo（无需 DSH，纯 Python）

环境要求：Python ≥ 3.10，`mujoco`、`numpy`、`opencv-python`、`matplotlib`、`pytest`。
（推荐 Anaconda 的 `python3.10` 环境；本仓库的所有产物与缓存均落在仓库目录内，不写 C 盘。）

```sh
# 1) 单元测试
cd python && python -m pytest tests -q

# 2) 端到端 Demo：正常 Run + 故障 Run + 诊断 + 证据包 + Markdown 报告 + 时间线 HTML
PYTHON=<你的 python3.10> node scripts/demo.mjs
# 输出在 examples/demo-output/ ：
#   report-run-*.md           实验报告（含证据与假设）
#   timeline-run-*.html       独立时间线查看器（浏览器直接打开，无需服务器）
#   bundle-run-*/             自包含证据包（manifest + 哈希 + 遥测 + 图表）
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

安装后，Agent 拥有 12 个 `rh_*` 工具与 6 个 Skill。例如对 Agent 说：

> “运行 Robotic Harness 的 pick-place demo：检查 demo 机械臂，跑一次正常仿真和一次带故障的仿真，诊断失败原因，导出证据包并生成报告。”

工具会调用 Python worker（`python -m robotic_harness_worker <command> --input -`），所有 Run、遥测、图表、报告默认写入 workspace 的 `.rh/` 目录。

### 可用工具一览

| 工具 | 风险 | 说明 |
|---|---|---|
| `rh_worker_ping` | R0 | worker 健康与依赖版本 |
| `rh_capability_list` | R0 | 能力清单（资产/仿真/感知/策略/诊断/数据） |
| `rh_robot_asset_inspect` | R0 | URDF/MJCF 结构检查（link/joint/惯量/碰撞 + 问题分级） |
| `rh_urdf_validate` | R0 | URDF 校验（树结构、惯量正定、限位、轴、mesh 路径） |
| `rh_urdf_to_mjcf` | R1 | URDF→MJCF 受控转换（MuJoCo 编译器 + 差异报告） |
| `rh_sim_status` | R0 | MuJoCo/渲染器/场景可用性 |
| `rh_sim_validate_scenario` | R0 | 场景可达性/参数校验 |
| `rh_sim_run` | R2 | MuJoCo 抓取仿真（含故障注入：感知偏移/夹爪滑落/TF 偏移/传感器噪声/遮挡/超时） |
| `rh_diagnose_run` | R0 | 确定性诊断：事实/规则/候选根因（证据 + 反证 + 缺失证据 + 建议检查） |
| `rh_evidence_export` | R1 | 自包含证据包（manifest + sha256 + 遥测 + 图表） |
| `rh_report_generate` | R1 | Markdown 报告 + 独立 timeline.html |
| `rh_data_quality` | R0 | CSV/JSONL 时序质量审计（缺失/NaN/乱序/重复/间隙/恒值通道） |

## 演示内容（v0.1）

- **场景**：平面 3 自由度机械臂 + 吸盘，桌上红色方块抓取 → 目标区放置（MuJoCo，纯基元构建，无外部网格）。
- **感知路由**：颜色分割（低延迟）→ 失败/遮挡时通用分割（边缘显著度），记录路由原因。
- **故障注入**（确定性，seed 可控）：`perception_offset_px`、`gripper_slip`、`tf_offset`、`sensor_noise`、`model_timeout_s`、`occlusion`。
- **遥测**：关节目标/实际/误差、吸盘状态、物体位姿、感知估计 vs 真值；图表（joints/tracking/trajectory）与场景渲染图。
- **诊断**：规则引擎产出分层证据 —— 事实（含时间戳与数值）、规则判定（阈值/状态机）、候选根因（按 感知/标定/机械/控制/系统 分层，标注可能性与缺失证据），**最终结论留给人**。
- **证据**：自包含证据包（哈希清单 + 全部记录）+ Markdown 报告 + timeline.html。

### 已知限制（如实说明）

- 吸盘抓取为**运动学实现**（吸附后物体位姿跟随吸盘），已在 run 配置与报告中注明。
- 感知在渲染器可用时使用真实离屏渲染；不可用时退化为“真值+噪声”的模拟感知（记录在遥测中）。
- 仿真结果不是真机证据；不提供任意 Topic 发布、真机写操作或急停解除能力。
- ROS 2 / SolidWorks / Isaac / 真机适配尚未实现（见 docs/roadmap.md）。

## 目录结构

```text
packages/dsh-bundle/   可安装 DSH bundle（TS 插件、skills/、worker 副本、fixtures、scenarios）
python/                robotic_harness_worker Python 包 + pytest 测试
fixtures/              URDF 测试资产（正常 + 故意损坏）
scenarios/             MuJoCo 场景定义（JSON）
scripts/               sync-worker / demo / smoke-worker
docs/                  架构、安全边界、路线图、Demo 说明
examples/demo-output/  一键 Demo 的输出示例
```

## 文档

- [架构与领域模型](docs/architecture.md)
- [安全边界](docs/safety-boundary.md)
- [路线图](docs/roadmap.md)
- [Demo 说明](docs/demo.md)
- [贡献指南](CONTRIBUTING.md) · [安全策略](SECURITY.md) · [第三方声明](THIRD_PARTY_NOTICES.md)

## 许可证

MIT。第三方组件与资产遵循各自许可（见 THIRD_PARTY_NOTICES.md）。
本仓库与 DeepSeek 官方无隶属关系；DSH 是独立项目（MIT，[deepseek-harness](https://github.com/deepseek-ai/deepseek-harness)）。
