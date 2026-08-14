"""Tests for the training module (server / plan / data / job / report)."""

import json

import pytest

from robotic_harness_worker import training
from robotic_harness_worker.core import WorkerError


def _make_server_config(store_root, servers):
    training._save_json(training._servers_path(store_root), {"servers": servers})


def _root(tmp_path) -> str:
    """storeRoot as the CLI passes it: the RunStore root (<ws>/.rh)."""
    return str(tmp_path / ".rh")


def test_server_check_not_configured(tmp_path):
    result = training.cmd_train_server_check({"storeRoot": _root(tmp_path)})
    assert result["ok"] is True
    assert result["backend"] == "unavailable"
    assert result["configured"] is False


def test_server_check_reachable(monkeypatch, tmp_path):
    _make_server_config(_root(tmp_path), [{"id": "gpu1", "host": "10.0.0.1", "user": "root"}])
    monkeypatch.setattr(training.shutil, "which", lambda name: "C:/tools/ssh.exe" if name == "ssh" else None)
    monkeypatch.setattr(training, "_run_ssh", lambda server, cmd, timeout_s=30: (0, "rh-ssh-ok"))
    result = training.cmd_train_server_check({"storeRoot": _root(tmp_path)})
    assert result["backend"] == "ssh"
    assert result["servers"][0]["state"] == "reachable"


def test_server_check_unreachable(monkeypatch, tmp_path):
    _make_server_config(_root(tmp_path), [{"id": "gpu1", "host": "10.0.0.1", "user": "root"}])
    monkeypatch.setattr(training.shutil, "which", lambda name: "C:/tools/ssh.exe" if name == "ssh" else None)
    monkeypatch.setattr(training, "_run_ssh", lambda server, cmd, timeout_s=30: (255, "Connection refused"))
    result = training.cmd_train_server_check({"storeRoot": _root(tmp_path)})
    assert result["servers"][0]["state"] == "unreachable"


def test_plan_create_writes_plan(tmp_path):
    result = training.cmd_train_plan_create(
        {
            "storeRoot": _root(tmp_path),
            "objective": "train a pick policy",
            "model": "diffusion-policy",
            "epochs": 7,
            "datasetIds": ["local:data/pick"],
        }
    )
    assert result["ok"] is True
    plan = result["plan"]
    assert plan["status"] == "draft"
    assert plan["hyperparameters"]["epochs"] == 7
    assert "数据" in plan["phases"][0]["detail"]
    assert result["planMarkdownPath"].endswith(".md")
    saved = training._load_json(result["planPath"])
    assert saved["objective"] == "train a pick policy"
    assert saved["hyperparameters"]["epochs"] == 7


def test_plan_create_requires_objective(tmp_path):
    with pytest.raises(WorkerError):
        training.cmd_train_plan_create({"storeRoot": _root(tmp_path)})


def test_data_discovery_mocked(monkeypatch):
    entries = [{"id": "lerobot/pick_place", "downloads": 1000, "likes": 5, "tags": ["robot", "manipulation"]}]

    def fake_urlopen(request, timeout=None):
        import io

        class FakeResponse(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        return FakeResponse(json.dumps(entries).encode("utf-8"))

    monkeypatch.setattr(training.urllib.request, "urlopen", fake_urlopen)
    result = training.cmd_train_data_discovery({"query": "pick place", "maxResults": 3})
    assert result["backend"] == "huggingface"
    assert result["results"][0]["id"] == "lerobot/pick_place"


def test_data_discovery_unavailable(monkeypatch):
    def fail(request, timeout=None):
        raise TimeoutError("no network")

    monkeypatch.setattr(training.urllib.request, "urlopen", fail)
    result = training.cmd_train_data_discovery({"query": "pick place"})
    assert result["backend"] == "unavailable"
    assert result["results"] == []


def test_job_prepare_dry_run(tmp_path):
    plan = training.cmd_train_plan_create({"storeRoot": _root(tmp_path), "objective": "train"})
    result = training.cmd_train_job_prepare(
        {"storeRoot": _root(tmp_path), "planId": plan["planId"], "dryRun": True}
    )
    assert result["ok"] is True
    assert result["dryRun"] is True
    assert len(result["artifacts"]) == 3
    assert result["job"]["status"] == "prepared"
    assert result["job"]["submitted"] is False


def test_job_prepare_refuses_without_confirm(tmp_path):
    plan = training.cmd_train_plan_create({"storeRoot": _root(tmp_path), "objective": "train"})
    with pytest.raises(WorkerError, match="confirm"):
        training.cmd_train_job_prepare(
            {"storeRoot": _root(tmp_path), "planId": plan["planId"], "dryRun": False}
        )


def test_job_prepare_refuses_unknown_server(tmp_path):
    _make_server_config(_root(tmp_path), [{"id": "gpu1", "host": "h", "user": "u"}])
    plan = training.cmd_train_plan_create({"storeRoot": _root(tmp_path), "objective": "train"})
    with pytest.raises(WorkerError, match="server"):
        training.cmd_train_job_prepare(
            {"storeRoot": _root(tmp_path), "planId": plan["planId"], "dryRun": False, "confirm": True}
        )


def test_job_prepare_submit(monkeypatch, tmp_path):
    _make_server_config(_root(tmp_path), [{"id": "gpu1", "host": "10.0.0.1", "user": "root", "workDir": "/home/rh"}])
    monkeypatch.setattr(training.shutil, "which", lambda name: "C:/tools/scp.exe" if name == "scp" else "C:/tools/ssh.exe")

    def fake_run_ssh(server, cmd, timeout_s=30):
        if "echo rh-ssh-ok" in cmd:
            return 0, "rh-ssh-ok"
        return 0, "12345"

    monkeypatch.setattr(training, "_run_ssh", fake_run_ssh)

    def fake_subprocess_run(args, **kwargs):
        assert args[0].lower().endswith("scp")
        return None

    monkeypatch.setattr(training.subprocess, "run", fake_subprocess_run)

    plan = training.cmd_train_plan_create(
        {"storeRoot": _root(tmp_path), "objective": "train", "serverId": "gpu1"}
    )
    result = training.cmd_train_job_prepare(
        {"storeRoot": _root(tmp_path), "planId": plan["planId"], "dryRun": False, "confirm": True}
    )
    assert result["job"]["submitted"] is True
    assert result["job"]["status"] == "running"
    assert result["job"]["pid"] == "12345"


def test_job_status_local(tmp_path):
    plan = training.cmd_train_plan_create({"storeRoot": _root(tmp_path), "objective": "train"})
    training.cmd_train_job_prepare({"storeRoot": _root(tmp_path), "planId": plan["planId"], "dryRun": True})
    log = tmp_path / ".rh" / "train-jobs" / plan["planId"] / "run.log"
    log.write_text("epoch,loss\n1,0.5\n", encoding="utf-8")
    result = training.cmd_train_job_status({"storeRoot": _root(tmp_path), "jobId": plan["planId"]})
    assert result["job"]["status"] == "prepared"
    assert "0.5" in result["logTail"]


def test_job_status_unknown(tmp_path):
    with pytest.raises(WorkerError, match="prepare"):
        training.cmd_train_job_status({"storeRoot": _root(tmp_path), "jobId": "nope"})


def test_train_report(tmp_path):
    log = tmp_path / "train.log.csv"
    log.write_text(
        "epoch,loss,val_loss\n1,0.9,0.95\n2,0.6,0.65\n3,0.4,0.45\n4,0.3,0.33\n5,0.25,0.28\n",
        encoding="utf-8",
    )
    result = training.cmd_train_report({"logPath": str(log), "outPath": str(tmp_path / "report.md")})
    assert result["ok"] is True
    report = result["report"]
    assert report["epochs"] == 5
    assert report["finalLoss"] == 0.25
    assert report["relativeImprovement"] == pytest.approx(0.7222, abs=1e-3)
    assert report["verdict"] == "收敛良好"
    assert (tmp_path / "report.md").exists()


def test_train_report_missing_log(tmp_path):
    with pytest.raises(WorkerError, match="log"):
        training.cmd_train_report({"logPath": str(tmp_path / "nope.csv")})
