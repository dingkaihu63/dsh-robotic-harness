"""Tests for the research module (literature search + solution proposals)."""

import json

import pytest

from robotic_harness_worker import research
from robotic_harness_worker.core import WorkerError

ARXIV_XML = """<?xml version="1.0" encoding="UTF-8"?>
<feed xmlns="http://www.w3.org/2005/Atom">
  <entry>
    <id>http://arxiv.org/abs/2501.00001v1</id>
    <title>Sim-to-Real Transfer for Robot Manipulation</title>
    <summary>We study sim-to-real transfer with a slippage-aware gripper for pick and place.</summary>
    <published>2025-01-01T00:00:00Z</published>
    <author><name>Alice Smith</name></author>
    <author><name>Bob Jones</name></author>
  </entry>
  <entry>
    <id>http://arxiv.org/abs/2502.00002v1</id>
    <title>Unrelated Quantum Decoherence Paper</title>
    <summary>Decoherence in cold atoms and its implications.</summary>
    <published>2025-02-01T00:00:00Z</published>
    <author><name>Carol White</name></author>
  </entry>
</feed>
"""

S2_JSON = {
    "data": [
        {
            "title": "Gripper Slippage Detection",
            "authors": [{"name": "Dan Black"}],
            "year": 2024,
            "url": "https://example.org/slip",
            "abstract": "Detecting slippage during pick and place with tactile sensing.",
            "openAccessPdf": {"url": "https://example.org/slip.pdf"},
        }
    ]
}


@pytest.fixture
def no_network(monkeypatch):
    """Block all HTTP by default; tests opt in via monkeypatch of helpers."""

    def fail(*args, **kwargs):
        raise TimeoutError("network disabled in tests")

    monkeypatch.setattr(research, "_http_get_text", fail)
    monkeypatch.setattr(research, "_http_get_json", fail)


def test_arxiv_query_builds_keyword_and(no_network):
    q = research._arxiv_query("gripper slippage during pick and place")
    assert 'all:"gripper"' in q
    assert "AND" in q


def test_literature_search_parses_arxiv(monkeypatch):
    def fake_get_text(url, timeout_s=None):
        assert "export.arxiv.org" in url
        return ARXIV_XML

    monkeypatch.setattr(research, "_http_get_text", fake_get_text)
    result = research.literature_search("sim-to-real robot manipulation", max_results=2)
    assert result["ok"] is True
    assert result["backend"] == "arxiv"
    assert len(result["results"]) == 2
    first = result["results"][0]
    assert first["title"] == "Sim-to-Real Transfer for Robot Manipulation"
    assert first["year"] == 2025
    assert first["authors"] == ["Alice Smith", "Bob Jones"]
    assert "slippage-aware" in first["abstract"]


def test_literature_search_semantic_scholar(monkeypatch):
    def fake_get_json(url, timeout_s=None):
        assert "semanticscholar.org" in url
        return S2_JSON

    monkeypatch.setattr(research, "_http_get_json", fake_get_json)
    result = research.literature_search("gripper slippage", sources=["semantic-scholar"], max_results=5)
    assert result["backend"] == "semantic-scholar"
    assert result["results"][0]["source"] == "semantic-scholar"
    assert result["results"][0]["url"] == "https://example.org/slip.pdf"


def test_literature_search_unavailable(no_network):
    result = research.literature_search("anything")
    assert result["ok"] is True
    assert result["backend"] == "unavailable"
    assert result["results"] == []
    assert result["failures"]


def test_literature_search_requires_query(no_network):
    with pytest.raises(WorkerError):
        research.literature_search("   ")


def test_problem_solutions_scaffold(monkeypatch):
    def fake_get_text(url, timeout_s=None):
        return ARXIV_XML

    monkeypatch.setattr(research, "_http_get_text", fake_get_text)
    result = research.cmd_problem_solutions(
        {"problem": "gripper slippage during pick and place", "stage": "experiment", "maxPapers": 2}
    )
    assert result["ok"] is True
    assert result["stage"] == "experiment"
    assert len(result["candidates"]) == 2
    # the slippage paper matches keywords; the quantum paper matches none
    assert result["candidates"][0]["matchedKeywords"]  # ranked first
    assert result["candidates"][1]["matchedKeywords"] == []
    proposal = result["proposal"]
    assert len(proposal["candidateOptions"]) == 2
    assert proposal["candidateOptions"][0]["evidence"]
    assert proposal["stageGuidance"].startswith("关注实验设计")
    assert "人工确认" in result["note"]


def test_problem_solutions_stage_guidance(monkeypatch):
    def fake_get_text(url, timeout_s=None):
        return ARXIV_XML

    monkeypatch.setattr(research, "_http_get_text", fake_get_text)
    result = research.cmd_problem_solutions({"problem": "policy fails", "stage": "model"})
    assert result["proposal"]["stageGuidance"].startswith("关注模型架构")


def test_problem_solutions_requires_problem(no_network):
    with pytest.raises(WorkerError):
        research.cmd_problem_solutions({"stage": "experiment"})


def test_problem_solutions_out_path(tmp_path, monkeypatch):
    def fake_get_text(url, timeout_s=None):
        return ARXIV_XML

    monkeypatch.setattr(research, "_http_get_text", fake_get_text)
    out = tmp_path / "proposal.json"
    result = research.cmd_problem_solutions(
        {"problem": "gripper slippage", "outPath": str(out), "storeRoot": str(tmp_path)}
    )
    assert out.exists()
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["problem"] == "gripper slippage"


def test_relevance_keywords_stopwords():
    assert "with" not in research._relevance_keywords("tune with learning rate")
    assert "learning" in research._relevance_keywords("tune with learning rate")
