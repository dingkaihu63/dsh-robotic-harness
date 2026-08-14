# Demo 说明：MuJoCo 抓取 + 故障 + 诊断 + 证据

本文是 `node scripts/demo.mjs` 的实际输出解读，也是你向 Agent 描述任务时的参考话术。

## 1. 运行

```sh
PYTHON=<你的 python3.10> node scripts/demo.mjs
```

Demo 依次执行：

1. **正常 Run**（seed 42，无故障）→ 期望 `success: true`，物体落在目标区 `[-0.16, 0, 0.17] ± 0.05`。
2. **故障 Run**（seed 43，`perceptionOffsetPx [18,6] + gripperSlip + tfOffset [0.015, 0]`）→ 期望 `success: false`，异常含 `grasp_missed` / `gripper_slip`。
3. 每个 Run：规则诊断（`diagnose-run`）→ 证据包（`evidence-export`）→ Markdown 报告 + timeline.html。
4. **文献与训练流**（离线安全）：
   - `problem-solutions`：按“gripper slippage during pick and place”检索公开文献 → 证据卡 + 候选方案脚手架（网络不可用时如实返回 `backend:"unavailable"`）；
   - `train-plan-create` → `train-job-prepare`（dry-run，只生成本地产物）→ 写一份模拟训练日志 → `train-report`（收敛统计）。

## 2. 预期输出

```text
=== demo outputs ===
run run-xxxxxxxx: success=true
  report    : .../report-run-xxxxxxxx.md
  timeline  : .../timeline-run-xxxxxxxx.html
  evidence  : .../bundle-run-xxxxxxxx
run run-yyyyyyyy: success=false
  ...

=== research & training flow ===
solutions   : backend=arxiv candidates=3
plan        : plan-xxxxxxxx (status=draft)
job prepare : dryRun=true artifacts=3
report      : verdict="收敛良好" improvement=0.8
```

> `solutions` 行在无网络时显示 `backend=unavailable candidates=0`，这是设计行为：文献检索是尽力而为，绝不伪造论文。

`examples/demo-output/` 下：

| 文件 | 内容 |
|---|---|
| `report-run-*.md` | 摘要、故障配置、阶段、异常、诊断（事实/规则/假设）、限制 |
| `timeline-run-*.html` | 单文件时间线：关节曲线（实际 vs 目标）、阶段/异常表、诊断表 |
| `bundle-run-*/manifest.json` | 证据包清单（文件 + sha256 + 环境快照） |
| `bundle-run-*/diagnostics.json` | 诊断案例（facts / rule findings / hypotheses） |
| `.rh/runs/*/artifacts/*.png` | joints / tracking / trajectory 曲线 + scene 渲染图 |

## 3. 解读要点（诊断输出）

故障 Run 的诊断案例应包含：

- **事实**：`perceptionEstimate [0.354, 0, 0.19]` vs `perceptionTrue [0.30, 0, 0.19]`（偏移约 54 mm）；`gripper_slip` 异常时间戳。
- **规则判定**：`rule: perception estimate diverged from ground truth`；`rule: object was lost during transport`。
- **假设**（可能性标注，需人工确认）：
  - `[perception] perception error caused the grasp to miss the object`（支持：估计偏移 > 阈值；反证：无；缺失证据：抓取时刻分割 mask）；
  - `[mechanical] gripper/object interface lost the object (slip)`（支持：滑落异常 + 故障配置中 gripper_slip=true；requiresHuman=true）。

**注意**：报告中的“假设”不是结论。修改故障配置重跑对照（如只注入 perception offset）即可验证各假设的相对贡献 —— 这正是 Demo 想展示的“可验证证据链”。

## 4. 在 DSH Web 中运行同样流程

安装 bundle 后（见 README），在会话中要求 Agent：

> “运行 rh-pick-place-demo Skill：检查 demo 机械臂 → rh_sim_status → 正常 sim-run → 带故障 sim-run → rh_diagnose_run → rh_evidence_export → rh_report_generate，然后解释失败原因并指出哪些是事实、哪些是假设。”

Agent 会按 Skill 的固定顺序执行并保留每一步结果。所有产物在 workspace 的 `.rh/` 下，可直接用文件工具查看。
