"""Real-robot experiment state machine and preflight (plan chapter 12).

Safety-first design:

- **Honest hardware reporting**: without a real hardware adapter this module
  marks hardware checks as ``skip`` (documented as *not safety evidence*) or
  ``not-checked`` (blocks a ``ready`` verdict). It never pretends a check
  passed.
- **Persisted state machine**: every transition is appended to the experiment
  record at ``<storeRoot>/experiments/<id>.json`` with a full state history
  ``[{state, at, operator?, reason?}]``. States: DRAFT -> VALIDATING ->
  READY_FOR_APPROVAL -> APPROVED -> ARMED -> RUNNING -> PAUSED/RECOVERING ->
  COMPLETED/FAILED/ABORTED/ESTOPPED.
- **Human approval only**: ``experiment-start`` requires an ``approvalRef``
  credential (approval number / signature text); the LLM has no approval
  authority. ``experiment-safe-cancel`` never releases the E-stop — that is
  always a site-side manual action.
- **No hardware action**: without an adapter, ``RUNNING`` only records state.

Commands (exported via ``COMMANDS``): ``robot-preflight``,
``robot-state-snapshot``, ``experiment-prepare``, ``experiment-request-approval``,
``experiment-start``, ``experiment-pause``, ``experiment-safe-cancel``,
``experiment-status``, ``experiment-finalize``.
"""

from __future__ import annotations

import json
import os
import re
from datetime import datetime, timezone
from typing import Any, Optional

from .core import RunStore, WorkerError, new_id

# ---------------------------------------------------------------------------
# state machine
# ---------------------------------------------------------------------------

EXPERIMENT_STATES = frozenset(
    {
        "DRAFT", "VALIDATING", "READY_FOR_APPROVAL", "APPROVED", "ARMED", "RUNNING",
        "PAUSED", "RECOVERING", "COMPLETED", "FAILED", "ABORTED", "ESTOPPED",
    }
)
TERMINAL_STATES = frozenset({"COMPLETED", "FAILED", "ABORTED", "ESTOPPED"})

_START_SOURCES = frozenset({"READY_FOR_APPROVAL"})
_PAUSE_SOURCES = frozenset({"RUNNING"})
_CANCEL_SOURCES = frozenset({"RUNNING", "PAUSED", "RECOVERING"})
_FINALIZE_SOURCES = frozenset(
    {"READY_FOR_APPROVAL", "APPROVED", "ARMED", "RUNNING", "PAUSED", "RECOVERING", "ESTOPPED"}
)

_BUILTIN_SCENARIO = "mujoco_pick_place"

HARDWARE_NOT_CHECKED_REASON = "无真机适配器（hardwareAdapter 未配置），该项未检查——不代表安全"


def _store_root(args: dict[str, Any]) -> str:
    return args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")


def _slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "-", value).strip("-") or "item"


def _experiments_dir(store_root: str) -> str:
    return os.path.join(store_root, "experiments")


def _record_path(store_root: str, experiment_id: str) -> str:
    return os.path.abspath(os.path.join(_experiments_dir(store_root), f"{_slug(experiment_id)}.json"))


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _load_record(store_root: str, experiment_id: str) -> dict[str, Any]:
    path = _record_path(store_root, experiment_id)
    if not os.path.isfile(path):
        raise WorkerError(f"experiment not found: {experiment_id}（期望记录文件 {path}）")
    with open(path, encoding="utf-8") as handle:
        record = json.load(handle)
    if not isinstance(record, dict) or not record.get("id"):
        raise WorkerError(f"experiment record 损坏：{path}")
    return record


def _save_record(store_root: str, record: dict[str, Any]) -> str:
    os.makedirs(_experiments_dir(store_root), exist_ok=True)
    path = _record_path(store_root, record["id"])
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, ensure_ascii=False, indent=2)
    return path


def _transition(
    record: dict[str, Any], new_state: str, operator: Optional[str] = None, reason: Optional[str] = None
) -> dict[str, str]:
    """Append one history entry and update the record state."""
    entry: dict[str, str] = {"state": new_state, "at": _now_iso()}
    if operator:
        entry["operator"] = operator
    if reason:
        entry["reason"] = reason
    record["state"] = new_state
    record.setdefault("history", []).append(entry)
    return entry


def _require_state(record: dict[str, Any], allowed: set[str], action: str) -> None:
    if record["state"] not in allowed:
        raise WorkerError(
            f"非法状态转移：{action} 要求状态为 {sorted(allowed)}，当前为 {record['state']}"
        )


def _scenario_known(scenario: str) -> bool:
    """A scenario reference is valid if it is the builtin name or an existing file."""
    return scenario == _BUILTIN_SCENARIO or os.path.exists(scenario)


# ---------------------------------------------------------------------------
# robot-preflight
# ---------------------------------------------------------------------------

DEFAULT_PREFLIGHT_CHECKS: list[dict[str, Any]] = [
    {"id": "robot.model", "label": "机器人型号/序列号已确认", "kind": "config"},
    {"id": "estop.released", "label": "急停按钮已释放（未按下）", "kind": "hardware"},
    {"id": "controller.mode", "label": "控制器处于期望模式", "kind": "hardware"},
    {"id": "joint.state", "label": "关节状态（位置/温度/错误）有效", "kind": "hardware"},
    {"id": "tf.tree", "label": "TF 树完整且无超时", "kind": "hardware"},
    {"id": "camera.calibration", "label": "相机标定版本有效", "kind": "file", "mode": "exists"},
    {"id": "end.effector", "label": "末端执行器状态正常", "kind": "hardware"},
    {"id": "workspace.obstacles", "label": "工作空间无未登记障碍", "kind": "hardware"},
    {"id": "limits.velocity_force", "label": "速度/力限制已配置", "kind": "config"},
    {"id": "ros.nodes", "label": "ROS 节点健康（无 crash）", "kind": "hardware"},
    {"id": "recording.ready", "label": "实验记录目录可写", "kind": "file", "mode": "writable"},
    {"id": "approval.valid", "label": "人工审批有效", "kind": "approval"},
]


def _execute_check(check: dict[str, Any], ctx: dict[str, Any]) -> tuple[str, Optional[dict[str, Any]], Optional[str]]:
    """Run one preflight check -> (status, evidence, reason).

    status is one of ``pass`` | ``fail`` | ``skip`` | ``not-checked``.
    """
    kind = check.get("kind", "config")
    check_id = check.get("id", "")
    if kind == "hardware":
        adapter = ctx.get("hardwareAdapter")
        if not adapter:
            return "skip", None, HARDWARE_NOT_CHECKED_REASON
        return (
            "not-checked",
            None,
            f"hardwareAdapter {adapter!r} 已配置但本 worker 未实现真机接口，该项未检查",
        )
    if kind == "file":
        mode = check.get("mode", "exists")
        path = check.get("path")
        if not path:
            if check_id == "camera.calibration":
                path = ctx.get("cameraCalibrationPath")
            elif check_id == "recording.ready":
                path = ctx.get("recordDir")
        if not path:
            return "not-checked", None, f"未提供检查路径（{check_id}），该项未检查"
        if mode == "writable":
            try:
                os.makedirs(path, exist_ok=True)
                probe = os.path.join(path, ".rh-write-probe")
                with open(probe, "w", encoding="utf-8") as handle:
                    handle.write("ok")
                os.remove(probe)
                return "pass", {"path": os.path.abspath(path), "writable": True}, None
            except OSError as error:
                return "fail", {"path": os.path.abspath(path), "writable": False}, f"记录目录不可写：{error}"
        if os.path.exists(path):
            return "pass", {"path": os.path.abspath(path), "exists": True}, None
        return "fail", {"path": os.path.abspath(path), "exists": False}, f"文件不存在：{path}"
    if kind == "config":
        if check_id == "robot.model":
            model = ctx.get("robotModel")
            if not model:
                return "not-checked", None, "未提供 robotModel，无法确认型号与序列号"
            return "pass", {"robotModel": model}, None
        if check_id == "limits.velocity_force":
            limits = ctx.get("safetyLimits") or {}
            present = {key: limits.get(key) for key in ("maxVelocity", "maxForce") if limits.get(key) is not None}
            if not present:
                return "fail", None, "未配置速度/力限制（safetyLimits.maxVelocity / maxForce），不允许实机运行"
            return "pass", {"safetyLimits": present}, None
        return "not-checked", None, f"未实现的 config 检查：{check_id}"
    if kind == "approval":
        record = ctx.get("experimentRecord")
        if record is None:
            return "not-checked", None, "未提供 experimentId，无法核验审批状态"
        state = record.get("state")
        if state in {"READY_FOR_APPROVAL", "APPROVED", "ARMED", "RUNNING", "PAUSED", "RECOVERING"}:
            return "pass", {"experimentId": record.get("id"), "state": state}, None
        return "fail", {"experimentId": record.get("id"), "state": state}, f"审批无效：实验状态为 {state}"
    return "not-checked", None, f"未知检查类型 {kind!r}"


def cmd_robot_preflight(args: dict[str, Any]) -> dict[str, Any]:
    """``robot-preflight``: generate (and optionally execute) a preflight checklist."""
    store_root = _store_root(args)
    experiment_id = args.get("experimentId")
    robot_model = args.get("robotModel")
    hardware_adapter = args.get("hardwareAdapter")
    if not experiment_id and not robot_model and not hardware_adapter:
        raise WorkerError("需提供 experimentId 或 robotModel/hardwareAdapter 之一")
    auto_run = args.get("autoRun", True)
    if not isinstance(auto_run, bool):
        auto_run = True

    ctx: dict[str, Any] = {
        "robotModel": robot_model,
        "hardwareAdapter": hardware_adapter,
        "cameraCalibrationPath": args.get("cameraCalibrationPath"),
        "recordDir": os.path.join(store_root, "experiments"),
        "safetyLimits": args.get("safetyLimits"),
    }
    experiment_record: Optional[dict[str, Any]] = None
    if experiment_id:
        experiment_record = _load_record(store_root, experiment_id)
        ctx["experimentRecord"] = experiment_record
        if not ctx["robotModel"]:
            ctx["robotModel"] = experiment_record.get("robotModel")
        if not ctx["safetyLimits"]:
            ctx["safetyLimits"] = experiment_record.get("safetyLimits")

    checks: list[dict[str, Any]] = list(DEFAULT_PREFLIGHT_CHECKS)
    custom = args.get("checks")
    if custom is not None:
        if not isinstance(custom, list):
            raise WorkerError("checks 必须是 {id, label, kind?, path?} 列表")
        for item in custom:
            if not isinstance(item, dict) or not item.get("id") or not item.get("label"):
                raise WorkerError("每条自定义检查需要 {id, label}")
            checks.append({**{"kind": "config", "mode": "exists"}, **item})

    preflight_id = new_id("preflight")
    executed: list[dict[str, Any]] = []
    pass_count = skip_count = fail_count = not_checked = 0
    if auto_run:
        for check in checks:
            status, evidence, reason = _execute_check(check, ctx)
            item: dict[str, Any] = {"id": check["id"], "label": check["label"], "status": status}
            if evidence:
                item["evidence"] = evidence
            if reason:
                item["reason"] = reason
            executed.append(item)
            if status == "pass":
                pass_count += 1
            elif status == "skip":
                skip_count += 1
            elif status == "fail":
                fail_count += 1
            else:
                not_checked += 1
    else:
        for check in checks:
            executed.append(
                {
                    "id": check["id"],
                    "label": check["label"],
                    "status": "not-checked",
                    "reason": "autoRun=false：仅生成清单，未执行检查",
                }
            )
        not_checked = len(checks)

    if fail_count > 0:
        verdict = "not-ready"
    elif not_checked > 0:
        verdict = "incomplete"
    else:
        verdict = "ready"

    result: dict[str, Any] = {
        "ok": True,
        "preflightId": preflight_id,
        "checks": executed,
        "passCount": pass_count,
        "skipCount": skip_count,
        "failCount": fail_count,
        "notCheckedCount": not_checked,
        "verdict": verdict,
        "note": "preflight 通过不构成功能安全证明",
        "autoRun": auto_run,
        "inputArgs": {"experimentId": experiment_id, "robotModel": robot_model, "hardwareAdapter": hardware_adapter},
    }
    if experiment_record is not None:
        experiment_record["preflight"] = {
            "preflightId": preflight_id,
            "at": _now_iso(),
            "verdict": verdict,
            "passCount": pass_count,
            "skipCount": skip_count,
            "failCount": fail_count,
        }
        result["recordPath"] = _save_record(store_root, experiment_record)
    return result


# ---------------------------------------------------------------------------
# robot-state-snapshot
# ---------------------------------------------------------------------------


def _last_joint_state(run_dir: str, metrics: dict[str, Any]) -> Optional[dict[str, Any]]:
    telemetry_path = os.path.join(run_dir, "telemetry.jsonl")
    if not os.path.isfile(telemetry_path):
        return None
    last: Optional[dict[str, Any]] = None
    with open(telemetry_path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                last = json.loads(line)
            except json.JSONDecodeError:
                continue
    if not last or "q" not in last:
        return None
    return {
        "q": [float(value) for value in last["q"]],
        "t": float(last.get("t", 0.0)),
        "phase": last.get("phase"),
    }


def cmd_robot_state_snapshot(args: dict[str, Any]) -> dict[str, Any]:
    """``robot-state-snapshot``: best-effort snapshot from store + preflight summary."""
    store_root = _store_root(args)
    experiment_id = args.get("experimentId")
    hardware_adapter = args.get("hardwareAdapter")
    snapshot: dict[str, Any] = {
        "at": _now_iso(),
        "robotModel": None,
        "controllerMode": None,
        "jointState": None,
        "lastRunId": None,
        "lastRunSuccess": None,
        "preflightSummary": None,
        "source": "store",
        "issues": [],
    }

    if experiment_id:
        record = _load_record(store_root, experiment_id)
        snapshot["robotModel"] = record.get("robotModel")
        preflight = record.get("preflight")
        if preflight:
            snapshot["preflightSummary"] = {
                "preflightId": preflight.get("preflightId"),
                "verdict": preflight.get("verdict"),
                "passCount": preflight.get("passCount"),
                "skipCount": preflight.get("skipCount"),
                "failCount": preflight.get("failCount"),
            }

    store = RunStore(store_root)
    runs = store.list_runs()
    if runs:
        latest = max(runs, key=lambda run: run.get("createdAt", 0))
        run_id = latest.get("id")
        snapshot["lastRunId"] = run_id
        snapshot["lastRunSuccess"] = bool(latest.get("success"))
        try:
            run = store.load_run(run_id)
            snapshot["controllerMode"] = run.metrics.get("controllerMode")
            snapshot["jointState"] = _last_joint_state(store.run_dir(run_id), run.metrics)
        except Exception as error:  # noqa: BLE001 - snapshot must degrade gracefully
            snapshot["issues"].append(f"读取 run {run_id} 失败：{error}")
    else:
        snapshot["issues"].append("存储中没有 Run 记录")

    if hardware_adapter:
        snapshot["issues"].append(
            f"hardwareAdapter {hardware_adapter!r} 已配置但无真机后端，快照来自存储（source=store）"
        )

    has_data = snapshot["lastRunId"] is not None or snapshot["robotModel"] is not None or snapshot["preflightSummary"] is not None
    note = (
        "无可用数据：存储中无 Run 且未提供 experimentId（或实验不存在）"
        if not has_data
        else "快照来自 Run 存储（source=store），非实机实时状态；无真机适配器时不代表实机安全"
    )
    return {
        "ok": True,
        "snapshot": snapshot,
        "note": note,
        "inputArgs": {"experimentId": experiment_id, "hardwareAdapter": hardware_adapter},
    }


# ---------------------------------------------------------------------------
# experiment state machine commands
# ---------------------------------------------------------------------------


def cmd_experiment_prepare(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-prepare``: create an experiment record (DRAFT -> VALIDATING)."""
    name = args.get("name")
    if not name or not str(name).strip():
        raise WorkerError("missing required argument 'name'")
    store_root = _store_root(args)
    scenario = args.get("scenario")
    plan = args.get("plan")
    safety_limits = args.get("safetyLimits")

    if safety_limits is not None:
        if not isinstance(safety_limits, dict):
            raise WorkerError("safetyLimits 必须是包含 maxVelocity/maxForce 的对象")
        values = [safety_limits[key] for key in ("maxVelocity", "maxForce") if safety_limits.get(key) is not None]
        if not values:
            raise WorkerError("safetyLimits 至少提供 maxVelocity 或 maxForce 之一")
        for value in values:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                raise WorkerError(f"safetyLimits 值必须为正数，got {value!r}")
    if scenario:
        if not _scenario_known(str(scenario)):
            raise WorkerError(f"scenario 引用无效：{scenario!r} 既不是内置场景 {_BUILTIN_SCENARIO!r} 也不是存在的文件路径")
    if plan:
        if not os.path.exists(plan):
            raise WorkerError(f"plan 引用不存在：{plan!r}")

    experiment_id = new_id("exp")
    record: dict[str, Any] = {
        "id": experiment_id,
        "name": str(name),
        "robotModel": args.get("robotModel"),
        "plan": plan,
        "scenario": scenario,
        "safetyLimits": safety_limits,
        "requiresApproval": bool(args.get("requiresApproval", True)),
        "state": "DRAFT",
        "history": [{"state": "DRAFT", "at": _now_iso(), "reason": "实验记录创建"}],
        "preflight": None,
        "notes": [],
    }
    _transition(record, "VALIDATING", reason="plan/scenario 引用校验通过")
    record_path = _save_record(store_root, record)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": "VALIDATING",
        "recordPath": record_path,
        "inputArgs": {"name": name, "scenario": scenario, "plan": plan},
    }


def cmd_experiment_request_approval(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-request-approval``: VALIDATING -> READY_FOR_APPROVAL."""
    experiment_id = args.get("experimentId")
    if not experiment_id:
        raise WorkerError("missing required argument 'experimentId'")
    store_root = _store_root(args)
    record = _load_record(store_root, experiment_id)
    if record["state"] != "VALIDATING":
        raise WorkerError(f"非法状态转移：experiment-request-approval 要求状态为 ['VALIDATING']，当前为 {record['state']}")

    operator = args.get("operator")
    evidence = args.get("evidence") or []
    if not isinstance(evidence, list):
        raise WorkerError("evidence 必须是列表")
    _transition(record, "READY_FOR_APPROVAL", operator=operator, reason=f"请求人工审批（evidence={len(evidence)} 项）")
    if evidence:
        record.setdefault("notes", []).append({"event": "approval-requested", "evidence": evidence, "at": _now_iso()})
    record_path = _save_record(store_root, record)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": "READY_FOR_APPROVAL",
        "requiresHuman": True,
        "note": "审批必须由人工完成，LLM 无审批权",
        "recordPath": record_path,
        "inputArgs": {"experimentId": experiment_id, "operator": operator},
    }


def cmd_experiment_start(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-start``: READY_FOR_APPROVAL -> APPROVED -> ARMED -> RUNNING.

    Requires a human ``approvalRef`` credential; refused by default.
    """
    experiment_id = args.get("experimentId")
    if not experiment_id:
        raise WorkerError("missing required argument 'experimentId'")
    approval_ref = args.get("approvalRef")
    if approval_ref is None or not str(approval_ref).strip():
        raise WorkerError("缺少人工审批凭证 approvalRef（如审批单号/签名文本）；LLM 无权自行批准实机实验")
    approver = args.get("approver")
    store_root = _store_root(args)
    record = _load_record(store_root, experiment_id)
    _require_state(record, set(_START_SOURCES), "experiment-start")

    reference = str(approval_ref).strip()
    _transition(record, "APPROVED", operator=approver, reason=f"人工审批凭证：{reference}")
    _transition(record, "ARMED", operator=approver, reason="审批通过，系统就绪（ARMED）")
    running_entry = _transition(record, "RUNNING", operator=approver, reason="实验开始（RUNNING）")
    record_path = _save_record(store_root, record)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": "RUNNING",
        "startedAt": running_entry["at"],
        "note": "无真机适配器时 RUNNING 仅记录状态，不执行硬件动作",
        "recordPath": record_path,
        "inputArgs": {"experimentId": experiment_id, "approver": approver},
    }


def cmd_experiment_pause(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-pause``: RUNNING -> PAUSED (or RECOVERING on recovery keywords)."""
    experiment_id = args.get("experimentId")
    if not experiment_id:
        raise WorkerError("missing required argument 'experimentId'")
    store_root = _store_root(args)
    record = _load_record(store_root, experiment_id)
    _require_state(record, set(_PAUSE_SOURCES), "experiment-pause")
    operator = args.get("operator")
    reason = str(args.get("reason") or "")
    if "recovery" in reason.lower() or "恢复" in reason:
        new_state = "RECOVERING"
    else:
        new_state = "PAUSED"
    _transition(record, new_state, operator=operator, reason=reason or "操作员暂停")
    record_path = _save_record(store_root, record)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": new_state,
        "recordPath": record_path,
        "inputArgs": {"experimentId": experiment_id, "operator": operator},
    }


def cmd_experiment_safe_cancel(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-safe-cancel``: RUNNING/PAUSED/RECOVERING -> ABORTED."""
    experiment_id = args.get("experimentId")
    if not experiment_id:
        raise WorkerError("missing required argument 'experimentId'")
    store_root = _store_root(args)
    record = _load_record(store_root, experiment_id)
    _require_state(record, set(_CANCEL_SOURCES), "experiment-safe-cancel")
    operator = args.get("operator")
    reason = args.get("reason") or "安全取消"
    _transition(record, "ABORTED", operator=operator, reason=str(reason))
    record_path = _save_record(store_root, record)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": "ABORTED",
        "note": "安全取消不解除急停；急停解除永远由现场人工执行",
        "recordPath": record_path,
        "inputArgs": {"experimentId": experiment_id, "operator": operator},
    }


def cmd_experiment_status(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-status``: return the record state and full history."""
    experiment_id = args.get("experimentId")
    if not experiment_id:
        raise WorkerError("missing required argument 'experimentId'")
    store_root = _store_root(args)
    record = _load_record(store_root, experiment_id)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": record["state"],
        "history": record.get("history", []),
        "recordPath": _record_path(store_root, experiment_id),
        "inputArgs": {"experimentId": experiment_id},
    }


def cmd_experiment_finalize(args: dict[str, Any]) -> dict[str, Any]:
    """``experiment-finalize``: move a non-terminal experiment to a terminal state."""
    experiment_id = args.get("experimentId")
    if not experiment_id:
        raise WorkerError("missing required argument 'experimentId'")
    outcome = args.get("outcome")
    if outcome not in ("completed", "failed", "aborted"):
        raise WorkerError("outcome 必须是 'completed' | 'failed' | 'aborted' 之一")
    terminal = {"completed": "COMPLETED", "failed": "FAILED", "aborted": "ABORTED"}[outcome]
    store_root = _store_root(args)
    record = _load_record(store_root, experiment_id)
    if record["state"] in TERMINAL_STATES:
        raise WorkerError(f"实验已处于终态 {record['state']}，不能再次 finalize")
    _require_state(record, set(_FINALIZE_SOURCES), "experiment-finalize")

    summary = args.get("summary")
    human_conclusion = args.get("humanConclusion")
    reason = f"finalize outcome={outcome}"
    if human_conclusion:
        reason += f"；人工结论：{human_conclusion}"
    _transition(record, terminal, operator=args.get("operator"), reason=reason)
    note: dict[str, Any] = {"event": "finalize", "outcome": outcome, "at": _now_iso()}
    if summary:
        note["summary"] = summary
    if human_conclusion:
        note["humanConclusion"] = human_conclusion
    record.setdefault("notes", []).append(note)
    record_path = _save_record(store_root, record)
    return {
        "ok": True,
        "experimentId": experiment_id,
        "state": terminal,
        "recordPath": record_path,
        "inputArgs": {"experimentId": experiment_id, "outcome": outcome},
    }


# ---------------------------------------------------------------------------
# module exports
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "robot-preflight": cmd_robot_preflight,
    "robot-state-snapshot": cmd_robot_state_snapshot,
    "experiment-prepare": cmd_experiment_prepare,
    "experiment-request-approval": cmd_experiment_request_approval,
    "experiment-start": cmd_experiment_start,
    "experiment-pause": cmd_experiment_pause,
    "experiment-safe-cancel": cmd_experiment_safe_cancel,
    "experiment-status": cmd_experiment_status,
    "experiment-finalize": cmd_experiment_finalize,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "robot.preflight",
        "kind": "robot",
        "provider": "robotic-harness-worker",
        "input": {"experimentId": "string?", "robotModel": "string?", "hardwareAdapter": "string?"},
        "output": "preflight checklist with per-item status and verdict",
        "risk": "R0-readonly",
        "description": "生成并执行实机实验 preflight 清单；无真机适配器时真机项如实标记 skip/not-checked，绝不假装通过。",
    },
    {
        "id": "robot.experiment_state_machine",
        "kind": "robot",
        "provider": "robotic-harness-worker",
        "input": {"experimentId": "string", "action": "string", "approvalRef": "string?"},
        "output": "persisted experiment record with full state history",
        "risk": "R3-real-robot",
        "description": "DRAFT→VALIDATING→READY_FOR_APPROVAL→APPROVED→ARMED→RUNNING→终态 实验状态机；审批必须由人工完成（approvalRef），LLM 无审批权；无适配器时仅记录状态，不执行硬件动作。",
    },
    {
        "id": "robot.state_snapshot",
        "kind": "robot",
        "provider": "robotic-harness-worker",
        "input": {"experimentId": "string?"},
        "output": "robot state snapshot from store (metrics + preflight summary)",
        "risk": "R0-readonly",
        "description": "从 Run 存储与实验记录生成机器人状态快照；无数据时字段为 null 并给出说明。",
    },
]
