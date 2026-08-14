"""Tests for the project memory module (memory-retrieve / memory-ingest)."""

from __future__ import annotations

import json
import os
import time

import pytest

from robotic_harness_worker import memory as mem
from robotic_harness_worker.core import RunStore, WorkerError
from robotic_harness_worker.simulation import SCENARIO_PICK_PLACE, run_pick_place


def _write_case(store_root: str, case_id: str, symptom: str, status: str = "open", run_id: str = "", findings=None, conclusion: str = "") -> str:
    case_dir = os.path.join(store_root, "cases")
    os.makedirs(case_dir, exist_ok=True)
    case = {
        "id": case_id,
        "runId": run_id,
        "symptom": symptom,
        "createdAt": time.time(),
        "status": status,
        "humanConclusion": conclusion,
        "findings": findings or [
            {"origin": "fact", "title": "anomaly: perception divergence", "detail": "estimate off by 50 mm", "evidence": []},
            {"origin": "rule", "title": "rule: grasp missed", "detail": "cup tip far from object", "evidence": []},
        ],
        "hypotheses": [
            {"id": "h-perception", "layer": "perception", "title": "perception error caused grasp miss", "support": ["offset"], "counterEvidence": [], "missingEvidence": [], "suggestedChecks": [], "likelihood": "high", "requiresHuman": False}
        ],
    }
    path = os.path.join(case_dir, f"{case_id}.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(case, handle, ensure_ascii=False, indent=2)
    return path


def test_retrieve_finds_similar_case_by_symptom(tmp_path):
    store_root = str(tmp_path / "rh" / ".rh")
    _write_case(store_root, "case-a", "grasp missed: perception estimate off; suction never engaged")
    _write_case(store_root, "case-b", "wheel encoder dropped frames during navigation")
    result = mem.cmd_memory_retrieve(
        {"symptom": "grasp missed perception estimate off suction never engaged", "storeRoot": store_root}
    )
    assert result["ok"] is True
    assert result["total"] >= 1
    assert result["related"][0]["caseId"] == "case-a"
    assert result["related"][0]["score"] > 0
    assert result["related"][0]["rationale"], "rationale must explain the match"
    assert "matchedTerms" in result["related"][0]
    assert result["related"][0]["matchedTerms"]


def test_retrieve_ranks_verified_case_first(tmp_path):
    store_root = str(tmp_path / "rh" / ".rh")
    _write_case(store_root, "case-open", "grasp missed perception estimate off suction never engaged")
    _write_case(store_root, "case-verified", "grasp missed perception estimate off suction never engaged", status="verified", conclusion="confirmed by re-run")
    result = mem.cmd_memory_retrieve(
        {"symptom": "grasp missed perception estimate off", "storeRoot": store_root}
    )
    assert result["related"][0]["caseId"] == "case-verified"
    assert result["related"][0]["verified"] is True
    assert any("人工验证" in r for r in result["related"][0]["rationale"])


def test_retrieve_anomaly_kind_bonus_and_exclude_run(tmp_path):
    store_root = str(tmp_path / "rh" / ".rh")
    _write_case(
        store_root, "case-slip", "object fell during transport", run_id="run-old",
        findings=[{"origin": "fact", "title": "anomaly: gripper_slip", "detail": "suction lost", "evidence": []}],
    )
    _write_case(store_root, "case-other", "motor temperature rising", run_id="run-other")
    # without exclusion the slip case is found (via anomaly-kind bonus)
    found = mem.cmd_memory_retrieve(
        {"symptom": "object fell during transport", "anomalyKinds": ["gripper_slip"], "storeRoot": store_root}
    )
    assert found["related"][0]["caseId"] == "case-slip"
    assert found["related"][0]["matchedAnomalyKinds"] == ["gripper_slip"]
    # excluding its run leaves nothing related
    excluded = mem.cmd_memory_retrieve(
        {
            "symptom": "object fell during transport",
            "anomalyKinds": ["gripper_slip"],
            "excludeRunId": "run-old",
            "storeRoot": store_root,
        }
    )
    assert all(r["runId"] != "run-old" for r in excluded["related"])
    assert excluded["related"] == []


def test_retrieve_from_run_path(tmp_path):
    store = RunStore(str(tmp_path / "rh" / ".rh"))
    run, _ = run_pick_place(SCENARIO_PICK_PLACE, {"gripper_slip": True}, seed=7, store=store)
    _write_case(
        store.root, "case-past", "gripper slip suction lost while carrying",
        findings=[{"origin": "fact", "title": "anomaly: gripper_slip", "detail": "suction lost", "evidence": []}],
    )
    result = mem.cmd_memory_retrieve({"runPath": store.run_dir(run.id), "storeRoot": store.root})
    assert result["ok"] is True
    assert result["query"]["symptom"]
    assert result["query"]["anomalyKinds"], "run anomalies must feed the query"
    assert "gripper_slip" in result["query"]["anomalyKinds"]
    assert any(r["caseId"] == "case-past" for r in result["related"])


def test_retrieve_requires_query(tmp_path):
    with pytest.raises(WorkerError):
        mem.cmd_memory_retrieve({"storeRoot": str(tmp_path)})


def test_retrieve_empty_store(tmp_path):
    result = mem.cmd_memory_retrieve({"symptom": "anything", "storeRoot": str(tmp_path / "empty")})
    assert result["ok"] is True
    assert result["related"] == []
    assert result["total"] == 0


def test_ingest_verifies_case(tmp_path):
    store_root = str(tmp_path / "rh" / ".rh")
    path = _write_case(store_root, "case-1", "grasp missed")
    result = mem.cmd_memory_ingest(
        {"caseId": "case-1", "status": "verified", "conclusion": "confirmed: TF offset", "operator": "alice", "storeRoot": store_root}
    )
    assert result["ok"] is True
    assert result["verified"] is True
    assert result["humanConclusion"] == "confirmed: TF offset"
    stored = json.load(open(path, encoding="utf-8"))
    assert stored["status"] == "verified"
    assert stored["operator"] == "alice"
    assert "updatedAt" in stored

    # verified cases rank first afterwards
    _write_case(store_root, "case-2", "grasp missed")
    retrieved = mem.cmd_memory_retrieve({"symptom": "grasp missed", "storeRoot": store_root})
    assert retrieved["related"][0]["caseId"] == "case-1"


def test_ingest_unknown_case_and_bad_status(tmp_path):
    store_root = str(tmp_path / "rh" / ".rh")
    with pytest.raises(WorkerError, match="unknown diagnostic case"):
        mem.cmd_memory_ingest({"caseId": "case-nope", "storeRoot": store_root})
    _write_case(store_root, "case-1", "grasp missed")
    with pytest.raises(WorkerError, match="invalid status"):
        mem.cmd_memory_ingest({"caseId": "case-1", "status": "maybe", "storeRoot": store_root})


def test_diagnose_run_attaches_related_cases(tmp_path):
    from robotic_harness_worker import cli

    store = RunStore(str(tmp_path / "rh" / ".rh"))
    run, _ = run_pick_place(SCENARIO_PICK_PLACE, {"perception_offset_px": [18.0, 6.0]}, seed=8, store=store)
    # plant a historical case matching the run's symptom tokens and anomaly kinds
    _write_case(
        store.root, "case-old",
        "run completed but success criteria failed grasped False slipped False",
        findings=[
            {"origin": "fact", "title": "anomaly: grasp_missed", "detail": "cup tip far from object", "evidence": []},
        ],
    )
    result = cli.cmd_diagnose_run({"runPath": store.run_dir(run.id), "storeRoot": store.root})
    assert result["ok"] is True
    assert "relatedCases" in result
    assert any(c.get("caseId") == "case-old" for c in result["relatedCases"]), result["relatedCases"]
