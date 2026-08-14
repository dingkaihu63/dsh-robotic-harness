---
name: 🐛 Bug report
about: Report something that does not work as described
title: "[bug] "
labels: bug
---

**Environment**（请如实填写，缺失项会让排查变慢）
- OS / platform: （如 Windows 11 / Ubuntu 22.04 / macOS）
- Python 版本与来源: （如 3.10.20 / Anaconda）
- DSH 版本: （如 0.1.0-rc.6）
- 安装方式: （源码 checkout / `dsh plugin add` / tarball）
- 相关依赖版本: mujoco / numpy / opencv / matplotlib（`python -m robotic_harness_worker ping` 会输出）

**发生了什么**
请描述现象，并贴出：
1. 复现步骤（命令或对 Agent 说的话）；
2. 实际输出（worker 返回的 JSON / 错误堆栈）；
3. 期望行为。

**最小复现**（可选但强烈建议）
- 涉及哪个工具/命令：`rh_xxx` / `python -m robotic_harness_worker <cmd>`
- 能否在 `python run_tests.py` 之外独立复现？

**补充**
- 该问题是否涉及真实硬件 / ROS 2 / SolidWorks 等后端？若无后端，请确认返回的是 `backend:"unavailable"` 而非报错。
