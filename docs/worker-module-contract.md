# Worker module contract（worker 模块开发契约）

> 面向并行开发：每个新领域模块是一个独立的 Python 文件（或其下子模块），
> 遵循本契约，主集成者负责接入 `cli.py` 与测试总闸。

## 1. 位置与命名

- 模块文件：`python/robotic_harness_worker/<domain>.py`
- 测试文件：`python/tests/test_<domain>.py`
- 命令名（CLI 层）与 DSH 工具名的映射规则：命令 `foo-bar-baz` ↔ 工具 `rh_foo_bar_baz`

## 2. 模块接口（每个领域模块必须导出）

```python
COMMANDS: dict[str, Callable[[dict], dict]]   # 命令名 -> 处理函数
CAPABILITIES: list[dict]                       # 能力清单（可选，并入总清单）
```

每个命令函数签名：`def cmd_xxx(args: dict) -> dict`。

- 成功返回 `{"ok": True, ...你的字段}`
- 预期失败（坏输入、后端缺失、格式不支持）**raise `WorkerError("消息")`**（从 `.core` 导入，已在 `core.py` 定义）
- 不要 import `cli.py`（会循环导入）；不要捕获后静默吞掉异常
- 需要读写 Run 存储时用 `RunStore`（`.core`）；`storeRoot` 参数键统一为 `args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")`

## 3. 结果 JSON 约定

- 顶层字段除 `ok` 外必须可 JSON 序列化（dict/list/str/float/int/bool/None）
- 数值统一 `round(v, N)` 处理；不要输出 numpy 类型（用 `.item()` / `float()` / `list()` 转换）
- 路径字段用绝对路径（`os.path.abspath`）
- 每个命令的返回里保留 `inputArgs` 摘要（可选但推荐：`{"path": ...}` 级别，不包含大内容）

## 4. 依赖策略

- **允许**：Python 3.10 标准库、`numpy`（已装）
- **可选**：`cv2`（已装）、`PIL`（已装）、`mujoco`（已装）、`matplotlib`（已装）
- 可选依赖必须在函数内 try/import，缺失时 raise `WorkerError` 说明缺什么、怎么装
- **禁止**新增第三方依赖（`pyarrow`、`trimesh`、`scipy` 等都不要 pip 安装——用 numpy/纯 Python 实现等价功能；若确实需要，在模块 docstring 里标注"若安装则增强，未安装则降级"）

## 5. 测试约定

- `pytest`，用 `tmp_path`（环境已把 TEMP 指到 F 盘，不会写 C 盘）
- 需要样例数据时：小 fixture 放 `python/tests/fixtures/<domain>/`，由测试内生成或用仓库级 `fixtures/`（若放仓库级，请在交付说明中注明路径）
- 可选依赖缺失时 `pytest.importorskip("cv2")` 等跳过
- 每个命令至少 1 个用例：正常路径 + 至少一个失败路径（坏输入/缺后端）
- 断言结果 JSON 的具体字段名与数值范围，不要只断言 `ok is True`

## 6. 现有参考实现（先读再写）

- `core.py`：RunStore、Run/DiagnosticCase、WorkerError、snapshot_environment
- `data_quality.py`：最简命令模块范例（audit 函数 + 纯函数分层）
- `diagnostics.py`：规则/证据/假设分层范例
- `vision.py`：可选依赖（cv2）处理范例
- `cli.py`：命令如何被调用（`--input -` stdin JSON，stdout JSON）——只读参考，勿 import

## 7. 交付说明（子代理完成时在最终回复中给出）

1. 新增文件清单（含测试）
2. 每个命令名 → 一句话功能 + 关键输出字段
3. 已用哪些 fixture/样例数据（路径）
4. 未能实现的点与原因（如有）
