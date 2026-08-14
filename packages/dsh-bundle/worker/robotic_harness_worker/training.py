"""Autonomous training assistant: server check, plan, data discovery, job run.

Implements the "自主训练" workflow in a SAFE, demo-grade way:

  train-server-check    →  who is the training server (explicit config only)
  train-plan-create     →  training plan (phases / hyperparams / data)
  train-data-discovery  →  search supplementary datasets (Hugging Face API)
  train-job-prepare     →  generate training script + launcher locally
  train-job-submit      →  upload + start the job REMOTELY (needs confirm)
  train-job-status      →  follow the job (local or remote log tail)
  train-report          →  metrics report from the training log

Safety invariants (docs/safety-boundary.md):
- a remote server must be EXPLICITLY configured in ``<storeRoot>/train-servers.json``;
- ``train-job-prepare`` defaults to dry-run: it only writes artifacts locally;
  real submission requires ``dryRun:false`` + ``confirm:true`` in the SAME call
  and a reachable configured server; the remote command is allowlisted (only
  runs our generated launcher inside the configured work dir);
- the training script is generated from a fixed template + user-approved plan
  values — no free-form remote code execution;
- public dataset search is best-effort network; on failure it returns a
  structured ``backend:"unavailable"`` result, never fabricated data.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from typing import Any, Optional

from .core import WorkerError, normalize_store_root

_HF_API = "https://huggingface.co/api/datasets"
_TIMEOUT_S = 12.0

_TRAIN_SCRIPT_TEMPLATE = '''\
"""Auto-generated training script (robotic-harness train-job-prepare).

Template-based on purpose: the values come from an approved training plan,
the script is deterministic and auditable. Run on the training server.
"""
import argparse
import csv
import json
import os
import time

def load_data(source):
    if source.startswith("local:"):
        path = source[len("local:"):]
        return [os.path.join(path, name) for name in os.listdir(path)] if os.path.isdir(path) else [path]
    # remote source (e.g. HF dataset id): placeholder loader — replace with
    # the dataset-specific loader in a later iteration of the suite.
    return [{{"remote_source": source}}]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", type=int, default={epochs})
    parser.add_argument("--batch-size", type=int, default={batch_size})
    parser.add_argument("--learning-rate", type=float, default={learning_rate})
    parser.add_argument("--log", default="train.log.csv")
    parser.add_argument("--checkpoint", default="checkpoint.pkl")
    args = parser.parse_args()

    sources = {sources_json}
    data = [item for s in sources for item in load_data(s)]
    print(f"[train] sources={{sources}} items={{len(data)}} epochs={{args.epochs}}", flush=True)
    os.makedirs("artifacts", exist_ok=True)
    with open(args.log, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["epoch", "loss", "val_loss", "elapsed_s", "timestamp"])
        for epoch in range(1, args.epochs + 1):
            start = time.time()
            # placeholder training loop: replace with the real model code.
            loss = max(0.0, 1.0 / epoch - args.learning_rate)
            val_loss = loss * 1.05
            writer.writerow([epoch, round(loss, 6), round(val_loss, 6), round(time.time() - start, 2), time.time()])
            fh.flush()
            print(f"[train] epoch {{epoch}}/{{args.epochs}} loss={{loss:.6f}} val_loss={{val_loss:.6f}}", flush=True)
            time.sleep(1)
    print("[train] done", flush=True)

if __name__ == "__main__":
    main()
'''

_LAUNCHER_TEMPLATE = """\
#!/usr/bin/env bash
# Auto-generated launcher (robotic-harness train-job-prepare).
# Runs the training script with nohup and writes the pid file.
set -euo pipefail
cd "$(dirname "$0")"
PYTHON="${PYTHON:-python3}"
"$PYTHON" train.py > run.log 2>&1 &
echo $! > train.pid
echo "started pid $(cat train.pid)"
"""


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _load_json(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _save_json(path: str, data: Any) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)


def _servers_path(store_root: str) -> str:
    return os.path.join(store_root, "train-servers.json")


def _load_servers(store_root: str) -> list[dict[str, Any]]:
    data = _load_json(_servers_path(store_root))
    return data.get("servers", []) if isinstance(data, dict) else []


def _find_server(store_root: str, server_id: str) -> Optional[dict[str, Any]]:
    for server in _load_servers(store_root):
        if server.get("id") == server_id:
            return server
    return None


def _ssh_base(server: dict[str, Any]) -> list[str]:
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]
    if server.get("keyPath"):
        cmd += ["-i", server["keyPath"]]
    port = server.get("port")
    if port:
        cmd += ["-p", str(port)]
    host = server.get("host", "")
    user = server.get("user")
    cmd.append(f"{user}@{host}" if user and host else host)
    return cmd


def _run_ssh(server: dict[str, Any], remote_command: str, timeout_s: float = 30.0) -> tuple[int, str]:
    full = _ssh_base(server) + [remote_command]
    proc = subprocess.run(full, capture_output=True, text=True, timeout=timeout_s)
    return proc.returncode, (proc.stdout or "").strip() + (("\n" + proc.stderr) if proc.stderr and proc.stderr.strip() else "").strip()


# ---------------------------------------------------------------------------
# train-server-check
# ---------------------------------------------------------------------------

def cmd_train_server_check(args: dict[str, Any]) -> dict[str, Any]:
    """``train-server-check``: list configured servers and probe connectivity."""
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    servers = _load_servers(store_root)
    if not servers:
        config_path = _servers_path(store_root)
        return {
            "ok": True,
            "backend": "unavailable",
            "configured": False,
            "servers": [],
            "configPath": config_path,
            "note": f"未配置训练服务器。请在 {config_path} 中写入 {{'servers': [{{'id','host','user','keyPath?','port?','workDir?'}}]}} 后重试。",
        }
    ssh_available = shutil.which("ssh") is not None
    if not ssh_available:
        return {
            "ok": True,
            "backend": "unavailable",
            "configured": True,
            "servers": [{"id": s.get("id"), "host": s.get("host")} for s in servers],
            "note": "本机没有可用的 ssh 客户端，无法探测服务器；请安装 OpenSSH 客户端。",
        }
    results = []
    for server in servers:
        state = "unreachable"
        detail = ""
        try:
            code, output = _run_ssh(server, "echo rh-ssh-ok")
            if code == 0 and "rh-ssh-ok" in output:
                state = "reachable"
            else:
                detail = output[:200]
        except (OSError, subprocess.TimeoutExpired) as error:
            detail = f"{type(error).__name__}: {error}"
        results.append(
            {
                "id": server.get("id"),
                "host": server.get("host"),
                "user": server.get("user"),
                "workDir": server.get("workDir", "~"),
                "gpu": server.get("gpu"),
                "state": state,
                "detail": detail,
            }
        )
    return {
        "ok": True,
        "backend": "ssh",
        "configured": True,
        "servers": results,
        "note": "探测仅执行只读命令（echo）；批量作业提交前仍需显式确认。",
    }


# ---------------------------------------------------------------------------
# train-plan-create
# ---------------------------------------------------------------------------

def cmd_train_plan_create(args: dict[str, Any]) -> dict[str, Any]:
    """``train-plan-create``: build an auditable training plan and save it."""
    objective = str(args.get("objective") or "").strip()
    if not objective:
        raise WorkerError("missing required argument 'objective'")
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    plans_dir = os.path.join(store_root, "train-plans")
    os.makedirs(plans_dir, exist_ok=True)

    plan_id = args.get("planId") or f"plan-{int(time.time())}"
    dataset_ids = [str(d) for d in (args.get("datasetIds") or [])]
    epochs = max(1, int(args.get("epochs", 10)))
    plan: dict[str, Any] = {
        "planId": plan_id,
        "objective": objective,
        "model": str(args.get("model") or "placeholder-model"),
        "serverId": args.get("serverId"),
        "hyperparameters": {
            "epochs": epochs,
            "batchSize": max(1, int(args.get("batchSize", 32))),
            "learningRate": float(args.get("learningRate", 1e-3)),
            "optimizer": str(args.get("optimizer") or "adam"),
            "validationSplit": min(0.9, max(0.0, float(args.get("validationSplit", 0.2)))),
        },
        "datasets": dataset_ids,
        "phases": [
            {"name": "data-prep", "detail": "加载/拆分数据集，校验样本与标注", "dependsOn": []},
            {"name": "train", "detail": f"训练 {epochs} 个 epoch，定期写 checkpoint", "dependsOn": ["data-prep"]},
            {"name": "validate", "detail": "在验证集上评估并记录指标", "dependsOn": ["train"]},
            {"name": "report", "detail": "汇总训练日志生成报告", "dependsOn": ["validate"]},
        ],
        "createdAt": _now_iso(),
        "status": "draft",
    }
    plan_path = os.path.join(plans_dir, f"{plan_id}.json")
    _save_json(plan_path, plan)

    md_lines = [
        f"# 训练计划 {plan_id}",
        "",
        f"- 目标: {objective}",
        f"- 模型: {plan['model']}",
        f"- 服务器: {plan['serverId'] or '未指定（仅本地准备）'}",
        f"- 数据集: {', '.join(dataset_ids) if dataset_ids else '未指定（可由 train-data-discovery 补充）'}",
        "",
        "## 超参数",
        "",
        "| 参数 | 值 |",
        "| --- | --- |",
    ]
    for key, value in plan["hyperparameters"].items():
        md_lines.append(f"| {key} | {value} |")
    md_lines += ["", "## 阶段", ""]
    for phase in plan["phases"]:
        md_lines.append(f"- **{phase['name']}**: {phase['detail']}")
    md_lines += [
        "",
        "## 说明",
        "",
        "计划仅为草稿；远程提交前必须由用户确认，且只能通过 train-job-prepare/submit 执行。",
    ]
    plan_md_path = os.path.join(plans_dir, f"{plan_id}.md")
    with open(plan_md_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(md_lines))

    return {
        "ok": True,
        "planId": plan_id,
        "planPath": plan_path,
        "planMarkdownPath": plan_md_path,
        "plan": plan,
        "note": "计划为草稿（status=draft）；确认无误后再准备训练作业。",
    }


# ---------------------------------------------------------------------------
# train-data-discovery
# ---------------------------------------------------------------------------

def cmd_train_data_discovery(args: dict[str, Any]) -> dict[str, Any]:
    """``train-data-discovery``: search supplementary datasets (Hugging Face)."""
    query = str(args.get("query") or "").strip()
    if not query:
        raise WorkerError("missing required argument 'query'")
    max_results = max(1, int(args.get("maxResults", 10)))
    params = urllib.parse.urlencode({"search": query, "limit": max_results})
    try:
        request = urllib.request.Request(f"{_HF_API}?{params}", headers={"User-Agent": "robotic-harness-worker/0.1"})
        with urllib.request.urlopen(request, timeout=_TIMEOUT_S) as response:
            entries = json.load(response)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError, json.JSONDecodeError) as error:
        return {
            "ok": True,
            "backend": "unavailable",
            "query": query,
            "results": [],
            "error": f"{type(error).__name__}: {error}",
            "note": "Hugging Face 数据集 API 不可用；可改用本地数据清单（inspect-asset / data-profile）或稍后重试。",
        }
    results = []
    for entry in entries if isinstance(entries, list) else []:
        if not isinstance(entry, dict):
            continue
        results.append(
            {
                "id": entry.get("id", ""),
                "downloads": entry.get("downloads"),
                "likes": entry.get("likes"),
                "tags": [t for t in (entry.get("tags") or []) if isinstance(t, str)][:8],
                "lastModified": entry.get("lastModified"),
            }
        )
    return {
        "ok": True,
        "backend": "huggingface",
        "query": query,
        "results": results,
        "note": "数据集未经人工校验；纳入训练计划前应检查许可与数据质量（data-quality-* 工具）。",
    }


# ---------------------------------------------------------------------------
# train-job-prepare / submit / status
# ---------------------------------------------------------------------------

def _render_training_script(plan: dict[str, Any]) -> str:
    hp = plan.get("hyperparameters", {})
    sources_json = json.dumps(plan.get("datasets") or [])
    return _TRAIN_SCRIPT_TEMPLATE.format(
        epochs=int(hp.get("epochs", 10)),
        batch_size=int(hp.get("batchSize", 32)),
        learning_rate=float(hp.get("learningRate", 1e-3)),
        sources_json=sources_json,
    )


def cmd_train_job_prepare(args: dict[str, Any]) -> dict[str, Any]:
    """``train-job-prepare``: prepare (and optionally submit) a training job.

    Dry-run by default: only writes the training script + launcher + plan
    snapshot into ``<storeRoot>/train-jobs/<planId>/``. Real remote submission
    requires ``dryRun:false`` AND ``confirm:true`` in the same call, plus a
    reachable, explicitly configured server.
    """
    plan_id = str(args.get("planId") or "").strip()
    if not plan_id:
        raise WorkerError("missing required argument 'planId'")
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    plan_path = os.path.join(store_root, "train-plans", f"{plan_id}.json")
    if not os.path.exists(plan_path):
        raise WorkerError(f"training plan {plan_id!r} not found at {plan_path}; run train-plan-create first")

    dry_run = bool(args.get("dryRun", True))
    confirm = bool(args.get("confirm", False))
    plan = _load_json(plan_path)
    server_id = args.get("serverId") or plan.get("serverId")
    plan = {**plan, "serverId": server_id}
    job_dir = os.path.join(store_root, "train-jobs", plan_id)
    os.makedirs(job_dir, exist_ok=True)

    with open(os.path.join(job_dir, "train.py"), "w", encoding="utf-8") as handle:
        handle.write(_render_training_script(plan))
    with open(os.path.join(job_dir, "launcher.sh"), "w", encoding="utf-8", newline="\n") as handle:
        handle.write(_LAUNCHER_TEMPLATE)
    _save_json(os.path.join(job_dir, "plan.snapshot.json"), plan)

    artifacts = [
        os.path.join(job_dir, "train.py"),
        os.path.join(job_dir, "launcher.sh"),
        os.path.join(job_dir, "plan.snapshot.json"),
    ]

    job: dict[str, Any] = {
        "jobId": plan_id,
        "planId": plan_id,
        "status": "prepared",
        "createdAt": _now_iso(),
        "serverId": plan.get("serverId"),
        "artifacts": artifacts,
        "submitted": False,
        "pid": None,
        "remoteLog": None,
    }

    if not dry_run:
        if not confirm:
            raise WorkerError("remote submission requires confirm:true — refusing without explicit human approval")
        server = _find_server(store_root, plan.get("serverId") or "")
        if server is None:
            raise WorkerError(f"server {plan.get('serverId')!r} is not configured; run train-server-check first")
        code, output = _run_ssh(server, "echo rh-ssh-ok")
        if code != 0 or "rh-ssh-ok" not in output:
            raise WorkerError(f"training server unreachable: {output[:200] or 'ssh failed'}")
        work_dir = server.get("workDir") or "~"
        # allowlist: only copy our generated artifacts, only start our launcher.
        remote_dir = f"{work_dir}/rh-jobs/{plan_id}"
        if shutil.which("scp") is None:
            raise WorkerError("scp is not available on this machine; cannot upload the job")
        scp = ["scp", "-o", "BatchMode=yes"]
        if server.get("keyPath"):
            scp += ["-i", server["keyPath"]]
        if server.get("port"):
            scp += ["-P", str(server["port"])]
        host = server.get("host", "")
        user = server.get("user")
        target = f"{user}@{host}" if user and host else host
        subprocess.run(scp + ["-r", job_dir, f"{target}:{remote_dir}"], check=True, timeout=120)
        _, start_output = _run_ssh(server, f"mkdir -p {remote_dir} && cd {remote_dir} && nohup bash launcher.sh > /dev/null 2>&1 & echo $!")
        job["status"] = "running"
        job["submitted"] = True
        job["pid"] = start_output.strip().splitlines()[-1] if start_output.strip() else None
        job["remoteDir"] = remote_dir
        job["remoteLog"] = f"{remote_dir}/run.log"
        job["submittedAt"] = _now_iso()

    job_path = os.path.join(job_dir, "job.json")
    _save_json(job_path, job)
    return {
        "ok": True,
        "jobId": plan_id,
        "dryRun": dry_run,
        "jobDir": job_dir,
        "artifacts": artifacts,
        "job": job,
        "jobPath": job_path,
        "note": "已生成可审计的训练脚本与启动器。" + ("" if dry_run else " 作业已提交到远程服务器。" + ("（已获显式确认）" if confirm else "")),
    }


def cmd_train_job_status(args: dict[str, Any]) -> dict[str, Any]:
    """``train-job-status``: follow a prepared/submitted training job."""
    job_id = str(args.get("jobId") or "").strip()
    if not job_id:
        raise WorkerError("missing required argument 'jobId'")
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    job_path = os.path.join(store_root, "train-jobs", job_id, "job.json")
    if not os.path.exists(job_path):
        raise WorkerError(f"no training job {job_id!r}; prepare it first (train-job-prepare)")
    job = _load_json(job_path)
    local_log = os.path.join(store_root, "train-jobs", job_id, "run.log")
    log_tail = ""
    if os.path.exists(local_log):
        with open(local_log, "r", encoding="utf-8", errors="replace") as handle:
            log_tail = "\n".join(handle.read().splitlines()[-30:])
    if job.get("submitted") and job.get("serverId"):
        server = _find_server(store_root, job["serverId"])
        if server:
            remote = job.get("remoteLog") or f"{server.get('workDir', '~')}/rh-jobs/{job_id}/run.log"
            try:
                code, output = _run_ssh(server, f"tail -n 30 {remote} 2>/dev/null || echo 'log not found'")
                if code == 0:
                    log_tail = output
            except (OSError, subprocess.TimeoutExpired):
                pass
    return {
        "ok": True,
        "job": job,
        "logTail": log_tail,
        "note": "状态来自本地记录与远程日志尾部；以服务器实际进程为准。",
    }


# ---------------------------------------------------------------------------
# train-report
# ---------------------------------------------------------------------------

def cmd_train_report(args: dict[str, Any]) -> dict[str, Any]:
    """``train-report``: turn a training log (epoch,loss[,val_loss]) into a report."""
    job_id = args.get("jobId")
    log_path = args.get("logPath")
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    if job_id:
        candidate = os.path.join(store_root, "train-jobs", str(job_id), "run.log")
        if os.path.exists(candidate):
            log_path = candidate
    if not log_path or not os.path.exists(log_path):
        raise WorkerError("training log not found; pass logPath or jobId of a finished job")

    import csv as _csv

    rows: list[dict[str, Any]] = []
    with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
        reader = _csv.DictReader(handle)
        for row in reader:
            try:
                rows.append(
                    {
                        "epoch": int(float(row.get("epoch", 0))),
                        "loss": float(row.get("loss", 0)),
                        "val_loss": float(row["val_loss"]) if row.get("val_loss") not in (None, "") else None,
                    }
                )
            except (ValueError, TypeError):
                continue

    if not rows:
        raise WorkerError(f"no parseable training rows in {log_path}")

    first, last = rows[0], rows[-1]
    loss_delta = last["loss"] - first["loss"]
    improvement = (first["loss"] - last["loss"]) / max(first["loss"], 1e-9)
    final_val = last.get("val_loss")
    verdict = (
        "收敛良好" if improvement > 0.1
        else "轻微改善" if improvement > 0.0
        else "未见收敛（loss 未下降），建议调整学习率/数据或增加轮次"
    )
    report = {
        "jobId": job_id,
        "logPath": os.path.abspath(log_path),
        "epochs": len(rows),
        "firstLoss": first["loss"],
        "finalLoss": last["loss"],
        "relativeImprovement": round(improvement, 4),
        "finalValLoss": final_val,
        "verdict": verdict,
        "note": "结论仅为日志统计，不构成发布依据；模型评估请结合 validation 与真实场景测试。",
    }

    md = [
        f"# 训练报告{f' — {job_id}' if job_id else ''}",
        "",
        f"- 轮次: {len(rows)}",
        f"- 初始 loss: {first['loss']:.6f}",
        f"- 最终 loss: {last['loss']:.6f}",
        f"- 相对改善: {improvement * 100:.2f}%",
        f"- 最终 val_loss: {final_val if final_val is not None else 'N/A'}",
        "",
        f"## 结论",
        "",
        f"{verdict}",
        "",
        "## 说明",
        "",
        "本报告由训练日志自动统计生成；发布模型前需结合验证集与真实场景评估。",
    ]
    out_path = args.get("outPath") or os.path.join(store_root, "train-plans", f"{job_id or 'train'}-report.md")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(md))

    report["reportPath"] = out_path
    return {"ok": True, "report": report}


COMMANDS: dict[str, Any] = {
    "train-server-check": cmd_train_server_check,
    "train-plan-create": cmd_train_plan_create,
    "train-data-discovery": cmd_train_data_discovery,
    "train-job-prepare": cmd_train_job_prepare,
    "train-job-status": cmd_train_job_status,
    "train-report": cmd_train_report,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "training.server_check",
        "kind": "probe",
        "provider": "robotic-harness-worker",
        "input": {"storeRoot?": "string"},
        "output": "configured training servers + connectivity state",
        "risk": "R1-probe",
        "description": "List explicitly configured training servers (train-servers.json) and probe them with a read-only echo.",
    },
    {
        "id": "training.plan_create",
        "kind": "plan",
        "provider": "robotic-harness-worker",
        "input": {"objective": "string", "model?": "string", "epochs?": "integer", "datasetIds?": "list"},
        "output": "auditable training plan (json + markdown)",
        "risk": "R0-readonly",
        "description": "Create a draft training plan; remote execution is never implied.",
    },
    {
        "id": "training.data_discovery",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"query": "string", "maxResults?": "integer"},
        "output": "supplementary dataset candidates (Hugging Face API)",
        "risk": "R0-readonly",
        "description": "Search public datasets to supplement the training data; best-effort network.",
    },
    {
        "id": "training.job_prepare",
        "kind": "execute",
        "provider": "robotic-harness-worker",
        "input": {"planId": "string", "dryRun?": "boolean", "confirm?": "boolean"},
        "output": "generated training artifacts; optional remote submission",
        "risk": "R3-remote",
        "description": "Generate the training script/launcher locally (dry-run default). Real remote submission only with dryRun:false AND confirm:true; commands are allowlisted.",
    },
    {
        "id": "training.job_status",
        "kind": "probe",
        "provider": "robotic-harness-worker",
        "input": {"jobId": "string"},
        "output": "job record + recent log lines",
        "risk": "R1-probe",
        "description": "Follow a training job via local records and remote log tail.",
    },
    {
        "id": "training.report",
        "kind": "report",
        "provider": "robotic-harness-worker",
        "input": {"jobId?": "string", "logPath?": "string"},
        "output": "statistical training report (markdown)",
        "risk": "R0-readonly",
        "description": "Summarize a training log into a report; statistical only, not a release verdict.",
    },
]
