"""Research assistance: literature search and evidence-based solution proposals.

Helps the Agent help the user at ANY stage (experiment, model, simulation,
data): given a problem, search public literature (arXiv / Semantic Scholar),
rank the results by keyword relevance, and build an evidence card scaffold
that the Agent turns into actionable options.

Design constraints (worker module contract, docs/worker-module-contract.md):
- no LLM calls in the worker — retrieval and evidence stay local/auditable;
- network is best-effort: API failures return a structured
  ``backend: "unavailable"`` diagnostic with instructions, never a fake result;
- every candidate carries its source URL and abstract excerpt so the user
  can verify it; the final choice is always human/Agent confirmed.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from typing import Any, Optional

from .core import WorkerError, normalize_store_root

_ARXIV_API = "https://export.arxiv.org/api/query"
_S2_API = "https://api.semanticscholar.org/graph/v1/paper/search"
_TIMEOUT_S = 12.0

_STAGE_GUIDANCE: dict[str, str] = {
    "experiment": "关注实验设计、对照与评价协议，优先可复现的实证结果",
    "model": "关注模型架构、训练数据与失败模式分析，优先带开源权重的论文",
    "simulation": "关注仿真到真机的迁移（sim-to-real）与资产校验方法",
    "data": "关注数据采集、清洗、标注协议与泄漏防护",
    "control": "关注控制器设计、跟踪性能与稳定性分析",
    "perception": "关注感知算法、标定与鲁棒性评估",
    "general": "综合检索最相关方法与综述",
}


def _http_get_json(url: str, timeout_s: float = _TIMEOUT_S) -> Any:
    request = urllib.request.Request(url, headers={"User-Agent": "robotic-harness-worker/0.1"})
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        return json.load(response)


def _http_get_text(url: str, timeout_s: float = _TIMEOUT_S) -> str:
    request = urllib.request.Request(url, headers={"User-Agent": "robotic-harness-worker/0.1"})
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        return response.read().decode("utf-8", errors="replace")


def _excerpt(text: str, limit: int = 350) -> str:
    text = re.sub(r"\s+", " ", text or "").strip()
    return text if len(text) <= limit else text[: limit - 1] + "…"


def _relevance_keywords(problem: str) -> list[str]:
    tokens = re.findall(r"[a-z0-9]+", problem.lower())
    return [t for t in tokens if len(t) > 3 and t not in {"with", "that", "this", "from", "have", "what", "when", "about", "using", "based", "after", "before"}]


def _arxiv_query(query: str) -> str:
    """Turn a free-text problem into an arXiv-friendly keyword AND query."""
    keywords = _relevance_keywords(query)
    if not keywords:
        return f'all:"{query}"'
    return " AND ".join(f'all:"{k}"' for k in keywords[:5])


# ---------------------------------------------------------------------------
# sources
# ---------------------------------------------------------------------------

def _search_arxiv(query: str, max_results: int) -> list[dict[str, Any]]:
    params = urllib.parse.urlencode(
        {"search_query": f'all:"{query}"', "start": 0, "max_results": max_results, "sortBy": "relevance"}
    )
    xml_text = _http_get_text(f"{_ARXIV_API}?{params}")
    root = ET.fromstring(xml_text)
    ns = {"a": "http://www.w3.org/2005/Atom"}
    results: list[dict[str, Any]] = []
    for entry in root.findall("a:entry", ns):
        title = " ".join((entry.findtext("a:title", default="", namespaces=ns) or "").split())
        summary = " ".join((entry.findtext("a:summary", default="", namespaces=ns) or "").split())
        authors = [a.findtext("a:name", default="", namespaces=ns) for a in entry.findall("a:author", ns)]
        published = entry.findtext("a:published", default="", namespaces=ns) or ""
        link = entry.findtext("a:id", default="", namespaces=ns) or ""
        results.append(
            {
                "title": title,
                "authors": [a for a in authors if a][:8],
                "year": int(published[:4]) if len(published) >= 4 and published[:4].isdigit() else None,
                "url": link,
                "abstract": _excerpt(summary),
                "source": "arxiv",
            }
        )
    return results


def _search_semantic_scholar(query: str, max_results: int) -> list[dict[str, Any]]:
    params = urllib.parse.urlencode(
        {"query": query, "limit": max_results, "fields": "title,authors,year,url,abstract,openAccessPdf"}
    )
    data = _http_get_json(f"{_S2_API}?{params}")
    results: list[dict[str, Any]] = []
    for paper in data.get("data", []) or []:
        pdf = (paper.get("openAccessPdf") or {}).get("url") if isinstance(paper.get("openAccessPdf"), dict) else None
        results.append(
            {
                "title": paper.get("title", ""),
                "authors": [a.get("name", "") for a in (paper.get("authors") or []) if isinstance(a, dict)][:8],
                "year": paper.get("year"),
                "url": pdf or paper.get("url") or "",
                "abstract": _excerpt(paper.get("abstract") or ""),
                "source": "semantic-scholar",
            }
        )
    return results


def literature_search(query: str, max_results: int = 8, sources: Optional[list[str]] = None) -> dict[str, Any]:
    """Search public literature across the requested sources."""
    query = query.strip()
    if not query:
        raise WorkerError("missing required argument 'query'")
    sources = sources or ["arxiv"]
    results: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    for source in sources:
        try:
            if source == "arxiv":
                results.extend(_search_arxiv(_arxiv_query(query), max_results))
                if not results:
                    # graceful degradation: too-specific AND queries can return
                    # nothing — retry with fewer keywords before giving up.
                    keywords = _relevance_keywords(query)
                    for keep in (3, 2):
                        if len(keywords) <= keep:
                            continue
                        looser = " AND ".join(f'all:"{k}"' for k in keywords[:keep])
                        results.extend(_search_arxiv(looser, max_results))
                        if results:
                            break
            elif source == "semantic-scholar":
                results.extend(_search_semantic_scholar(query, max_results))
            else:
                failures.append({"source": source, "error": f"unknown source {source!r}"})
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError, json.JSONDecodeError, ET.ParseError) as error:
            failures.append({"source": source, "error": f"{type(error).__name__}: {error}"})
    backend = "unavailable" if not results and failures else "+".join(sources)
    return {
        "ok": True,
        "query": query,
        "backend": backend,
        "results": results[: max_results * len(sources)],
        "failures": failures,
        "note": "文献检索仅供证据参考；引用前请核对原文与许可，方案需人工确认" if backend != "unavailable" else "所有检索源均不可用；请检查网络后重试",
    }


# ---------------------------------------------------------------------------
# problem → solution proposals
# ---------------------------------------------------------------------------

def _candidate_from_paper(paper: dict[str, Any], keywords: list[str]) -> dict[str, Any]:
    haystack = f"{paper.get('title', '')} {paper.get('abstract', '')}".lower()
    matched = [k for k in keywords if k in haystack]
    return {
        "title": paper.get("title", ""),
        "year": paper.get("year"),
        "url": paper.get("url", ""),
        "source": paper.get("source", ""),
        "abstractExcerpt": paper.get("abstract", ""),
        "matchedKeywords": matched,
    }


def cmd_literature_search(args: dict[str, Any]) -> dict[str, Any]:
    """``literature-search``: query public academic literature (arXiv / Semantic Scholar)."""
    result = literature_search(
        str(args.get("query") or ""),
        max_results=max(1, int(args.get("maxResults", 8))),
        sources=args.get("sources"),
    )
    result["inputArgs"] = {"query": result["query"], "maxResults": int(args.get("maxResults", 8))}
    return result


def cmd_problem_solutions(args: dict[str, Any]) -> dict[str, Any]:
    """``problem-solutions``: search literature for a problem and scaffold solutions.

    The worker returns evidence cards (papers + matched keywords) and a
    proposal scaffold; the Agent (or the user) turns them into concrete
    options. The scaffold never claims a solution is verified.
    """
    problem = str(args.get("problem") or "").strip()
    if not problem:
        raise WorkerError("missing required argument 'problem'")
    stage = str(args.get("stage") or "general").strip()
    guidance = _STAGE_GUIDANCE.get(stage, _STAGE_GUIDANCE["general"])
    context = str(args.get("context") or "").strip()
    max_papers = max(3, int(args.get("maxPapers", 6)))

    search = literature_search(problem, max_results=max_papers, sources=args.get("sources") or ["arxiv"])
    keywords = _relevance_keywords(f"{problem} {context}")
    candidates = [_candidate_from_paper(p, keywords) for p in search["results"]]
    candidates.sort(key=lambda c: (-len(c["matchedKeywords"]), -(c["year"] or 0)))

    scaffold = {
        "problem": problem,
        "stage": stage,
        "stageGuidance": guidance,
        "candidateOptions": [
            {
                "label": f"方案 {index + 1}",
                "title": c["title"],
                "evidence": [c["url"]] if c["url"] else [],
                "relevanceNotes": c["matchedKeywords"],
                "abstractExcerpt": c["abstractExcerpt"],
                "validationSteps": [
                    "阅读原文核实方法与适用条件",
                    "在小范围/仿真中复现关键结论",
                    "对照当前问题的证据（run/日志/数据）评估可行性",
                ],
            }
            for index, c in enumerate(candidates)
        ],
        "suggestedNextSteps": [
            "由 Agent 汇总候选方案的取舍并询问用户偏好",
            "选择 1-2 个方案进入实验/仿真验证",
            "验证结果沉淀为诊断案例与项目记忆",
        ],
    }
    if context:
        scaffold["userContext"] = context

    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    out_path = args.get("outPath")
    if out_path:
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump(scaffold, handle, ensure_ascii=False, indent=2)

    return {
        "ok": True,
        "problem": problem,
        "stage": stage,
        "backend": search["backend"],
        "candidates": candidates,
        "proposal": scaffold,
        "outPath": out_path,
        "storeRoot": os.path.abspath(store_root),
        "note": "候选方案来自文献关键词匹配，未经验证；结论需人工确认",
        "inputArgs": {"problem": problem, "stage": stage},
    }


COMMANDS: dict[str, Any] = {
    "literature-search": cmd_literature_search,
    "problem-solutions": cmd_problem_solutions,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "research.literature_search",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"query": "string", "maxResults?": "integer", "sources?": "list"},
        "output": "literature results with abstracts and URLs",
        "risk": "R0-readonly",
        "description": "Search public academic literature (arXiv / Semantic Scholar); best-effort network.",
    },
    {
        "id": "research.problem_solutions",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"problem": "string", "stage?": "string", "context?": "string"},
        "output": "evidence cards + solution proposal scaffold",
        "risk": "R0-readonly",
        "description": "Search literature for a problem and scaffold solution options with evidence; conclusions need human confirmation.",
    },
]
