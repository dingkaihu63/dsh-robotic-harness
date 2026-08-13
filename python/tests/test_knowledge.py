"""Tests for the knowledge retrieval module (docs-index / manual-search /
error-code-lookup / case-search), per docs/worker-module-contract.md.

Each command is covered with a happy path and a failure path; results are
asserted on concrete fields, not just ``ok``.
"""

from __future__ import annotations

import json
import os

import pytest

from robotic_harness_worker import knowledge
from robotic_harness_worker.core import WorkerError


def _write_docs(tmp_path):
    """Two small doc files with known line numbers."""
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "install.md").write_text(
        "# Installation Guide\n"
        "\n"
        "The robot driver is installed with rosdep.\n"
        "Check the network connection before starting.\n"
        "If the node crash occurs, inspect the log file.\n",
        encoding="utf-8",
    )
    (docs / "calibration.rst").write_text(
        "Camera Calibration\n"
        "==================\n"
        "\n"
        "Run the calibration tool once per day.\n"
        "The calibration version must match the firmware.\n",
        encoding="utf-8",
    )
    return docs


# ---------------------------------------------------------------------------
# docs-index
# ---------------------------------------------------------------------------


def test_docs_index_builds_inverted_index(tmp_path):
    docs = _write_docs(tmp_path)
    out_path = tmp_path / "idx.json"
    result = knowledge.cmd_docs_index({"path": str(docs), "outPath": str(out_path)})
    assert result["ok"] is True
    assert result["root"] == os.path.abspath(str(docs))
    assert len(result["files"]) == 2
    assert len(result["entries"]) == 2
    assert out_path.exists()

    on_disk = json.loads(out_path.read_text(encoding="utf-8"))
    assert len(on_disk["entries"]) == 2

    install = next(e for e in result["entries"] if e["path"].endswith("install.md"))
    assert install["title"] == "Installation Guide"
    assert install["sha256"]
    assert "rosdep" in install["words"]
    assert install["words"]["rosdep"] == [3]  # 1-based line of the hit

    calibration = next(e for e in result["entries"] if e["path"].endswith("calibration.rst"))
    assert calibration["title"] == "Camera Calibration"
    assert "calibration" in calibration["words"]
    assert calibration["words"]["calibration"] == [1, 4, 5]


def test_docs_index_requires_existing_directory(tmp_path):
    with pytest.raises(WorkerError, match="path"):
        knowledge.cmd_docs_index({"path": str(tmp_path / "nope")})
    with pytest.raises(WorkerError, match="path"):
        knowledge.cmd_docs_index({})


# ---------------------------------------------------------------------------
# manual-search
# ---------------------------------------------------------------------------


def test_manual_search_hits_keyword_with_correct_snippet_lines(tmp_path):
    docs = _write_docs(tmp_path)
    out_path = tmp_path / "idx.json"
    knowledge.cmd_docs_index({"path": str(docs), "outPath": str(out_path)})
    result = knowledge.cmd_manual_search({"query": "rosdep", "path": str(out_path)})
    assert result["ok"] is True
    assert result["query"] == "rosdep"
    assert result["total"] >= 1
    top = result["results"][0]
    assert top["path"].endswith("install.md")
    assert "rosdep" in top["matchedTerms"]
    assert top["score"] >= 1
    snippet = top["snippets"][0]
    assert snippet["line"] == 3
    assert "rosdep" in snippet["text"]
    assert len(snippet["text"]) <= 201  # 200 chars + optional ellipsis


def test_manual_search_title_weighting(tmp_path):
    docs = _write_docs(tmp_path)
    out_path = tmp_path / "idx.json"
    knowledge.cmd_docs_index({"path": str(docs), "outPath": str(out_path)})
    # "guide" only appears in the title -> title match (x3) plus its line hit
    result = knowledge.cmd_manual_search({"query": "guide", "path": str(out_path)})
    assert result["total"] == 1
    assert result["results"][0]["score"] >= 3
    assert result["results"][0]["title"] == "Installation Guide"


def test_manual_search_auto_indexes_directory(tmp_path):
    docs = _write_docs(tmp_path)
    # no index file exists; passing the directory triggers auto docs-index
    result = knowledge.cmd_manual_search({"query": "crash", "path": str(docs)})
    assert result["ok"] is True
    assert result["indexBuiltOnTheFly"] is True
    assert result["total"] >= 1
    top = result["results"][0]
    assert "crash" in top["matchedTerms"]
    assert top["path"].endswith("install.md")


def test_manual_search_auto_builds_default_index(tmp_path, monkeypatch):
    _write_docs(tmp_path)
    monkeypatch.chdir(tmp_path)
    result = knowledge.cmd_manual_search({"query": "calibration"})
    assert result["ok"] is True
    assert result["indexBuiltOnTheFly"] is True
    assert result["total"] >= 1
    assert (tmp_path / ".rh" / "docs-index.json").exists()
    # second call uses the persisted default index
    again = knowledge.cmd_manual_search({"query": "firmware"})
    assert again["ok"] is True
    assert again["indexBuiltOnTheFly"] is False
    assert again["total"] >= 1


def test_manual_search_missing_query_and_bad_path(tmp_path):
    with pytest.raises(WorkerError, match="query"):
        knowledge.cmd_manual_search({})
    with pytest.raises(WorkerError, match="index 路径不存在"):
        knowledge.cmd_manual_search({"query": "x", "path": str(tmp_path / "missing.json")})


# ---------------------------------------------------------------------------
# error-code-lookup
# ---------------------------------------------------------------------------


def test_error_code_lookup_builtin_hit():
    result = knowledge.cmd_error_code_lookup({"code": "1"})
    assert result["ok"] is True
    assert result["found"] is True
    assert result["source"] == "builtin-example"
    assert result["entry"]["code"] == "1"
    assert result["entry"]["severity"] == "error"
    assert "示例" in result["note"]


def test_error_code_lookup_builtin_miss_with_closest():
    result = knowledge.cmd_error_code_lookup({"code": "999"})
    assert result["ok"] is True
    assert result["found"] is False
    assert result["closest"] is not None


def test_error_code_lookup_user_table_override(tmp_path):
    table = tmp_path / "codes.json"
    table.write_text(
        json.dumps(
            {
                "codes": [
                    {
                        "code": "E-42",
                        "meaning": "自定义错误",
                        "severity": "warning",
                        "source": "ACME-5000 manual",
                        "advice": "重启控制柜并复测",
                        "url": "https://example.com/manual",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    result = knowledge.cmd_error_code_lookup({"code": "E-42", "tablePath": str(table)})
    assert result["ok"] is True
    assert result["found"] is True
    assert result["source"] == "user"
    assert result["entry"]["meaning"] == "自定义错误"
    assert result["entry"]["url"] == "https://example.com/manual"

    # the builtin table must not leak into the user table
    miss = knowledge.cmd_error_code_lookup({"code": "1", "tablePath": str(table)})
    assert miss["found"] is False


def test_error_code_lookup_missing_table(tmp_path):
    with pytest.raises(WorkerError, match="table not found"):
        knowledge.cmd_error_code_lookup({"code": "1", "tablePath": str(tmp_path / "nope.json")})
    with pytest.raises(WorkerError, match="code"):
        knowledge.cmd_error_code_lookup({})


# ---------------------------------------------------------------------------
# case-search
# ---------------------------------------------------------------------------


def _write_case(store_root, case_id="case-abc123"):
    cases_dir = store_root / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    case = {
        "id": case_id,
        "runId": "run-deadbeef",
        "symptom": "物体抓取后滑落（gripper slip during carry）",
        "createdAt": 1.0,
        "findings": [
            {
                "origin": "fact",
                "title": "perception estimate vs ground truth",
                "detail": "",
                "evidence": [],
                "timeS": None,
                "confidence": "high",
            }
        ],
        "hypotheses": [
            {
                "id": "h1",
                "layer": "mechanical",
                "title": "吸盘磨损",
                "support": ["suction pressure 下降"],
                "counterEvidence": [],
                "missingEvidence": [],
                "suggestedChecks": [],
                "likelihood": "medium",
                "requiresHuman": False,
            }
        ],
        "status": "open",
        "humanConclusion": "",
    }
    (cases_dir / f"{case_id}.json").write_text(json.dumps(case, ensure_ascii=False), encoding="utf-8")


def test_case_search_finds_matching_case(tmp_path):
    store_root = tmp_path / "rh"
    _write_case(store_root)
    result = knowledge.cmd_case_search({"query": "滑落 slip", "storeRoot": str(store_root)})
    assert result["ok"] is True
    assert result["total"] == 1
    hit = result["results"][0]
    assert hit["caseId"] == "case-abc123"
    assert hit["runId"] == "run-deadbeef"
    assert hit["symptom"] == "物体抓取后滑落（gripper slip during carry）"
    assert hit["status"] == "open"
    assert hit["score"] >= 1
    assert hit["matchedField"] == "symptom"


def test_case_search_no_match_returns_empty_list(tmp_path):
    store_root = tmp_path / "rh"
    _write_case(store_root)
    result = knowledge.cmd_case_search({"query": "qqqqzzzz", "storeRoot": str(store_root)})
    assert result["ok"] is True
    assert result["results"] == []
    assert result["total"] == 0


def test_case_search_missing_cases_dir_returns_empty(tmp_path):
    result = knowledge.cmd_case_search({"query": "slip", "storeRoot": str(tmp_path / "empty")})
    assert result["ok"] is True
    assert result["results"] == []


def test_case_search_requires_query(tmp_path):
    with pytest.raises(WorkerError, match="query"):
        knowledge.cmd_case_search({"storeRoot": str(tmp_path)})


def test_module_exports():
    assert "docs-index" in knowledge.COMMANDS
    assert "manual-search" in knowledge.COMMANDS
    assert "error-code-lookup" in knowledge.COMMANDS
    assert "case-search" in knowledge.COMMANDS
    assert 2 <= len(knowledge.CAPABILITIES) <= 3
