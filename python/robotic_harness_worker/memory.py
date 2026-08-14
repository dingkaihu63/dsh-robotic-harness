"""Project memory: retrieve similar historical diagnostic cases to assist
LLM reasoning with prior evidence.

The memory layer is deliberately lightweight and embedding-free: it ranks
historical ``DiagnosticCase`` records by keyword overlap (symptom / findings /
hypotheses), anomaly-kind agreement and verification status, and returns
the rationale for every match. The worker never calls an LLM — retrieval and
evidence stay local and auditable; the model does the reasoning over what
was retrieved.

Conventions per the worker module contract (docs/worker-module-contract.md):
- commands export a plain ``COMMANDS``/``CAPABILITIES`` interface;
- storeRoot is the RunStore root (``.rh`` directory) or the workspace root
  (both are normalized through ``core.normalize_store_root``).
"""

from __future__ import annotations

import json
import os
import re
import time
from typing import Any, Optional

from .core import WorkerError, normalize_store_root

try:
    from .knowledge import _case_searchable_text, tokenize  # type: ignore[attr-defined]
except ImportError:  # pragma: no cover - defensive
    _case_searchable_text = None  # type: ignore[assignment]
    tokenize = None  # type: ignore[assignment]

# Field weights for keyword overlap: symptom is the strongest signal.
_FIELD_WEIGHTS = {"symptom": 3, "title": 2, "hypothesis": 1}
# Bonus per shared anomaly kind between the query run and a stored case.
_ANOMALY_MATCH_BONUS = 2
# Bonus for cases the operator explicitly verified as correct diagnoses.
_VERIFIED_BONUS = 3
_CLOSED_WITH_CONCLUSION_BONUS = 1

_CASE_STATUSES = {"open", "verified", "rejected", "closed"}


def _load_cases(store_root: str) -> list[dict[str, Any]]:
    """Load every case JSON under ``<storeRoot>/cases/`` with its file path."""
    cases_dir = os.path.join(store_root, "cases")
    cases: list[dict[str, Any]] = []
    if not os.path.isdir(cases_dir):
        return cases
    for filename in sorted(os.listdir(cases_dir)):
        if not filename.endswith(".json"):
            continue
        path = os.path.join(cases_dir, filename)
        try:
            with open(path, encoding="utf-8") as handle:
                case = json.load(handle)
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(case, dict):
            continue
        case_id = case.get("id") or os.path.splitext(filename)[0]
        cases.append(
            {
                "case": case,
                "caseId": case_id,
                "runId": case.get("runId", ""),
                "path": path,
            }
        )
    return cases


def _case_anomaly_kinds(case: dict[str, Any]) -> set[str]:
    """Anomaly kinds referenced by the case findings (titles like 'anomaly: xyz')."""
    kinds: set[str] = set()
    for finding in case.get("findings", []):
        if not isinstance(finding, dict):
            continue
        title = str(finding.get("title", ""))
        if title.startswith("anomaly:"):
            kinds.add(title.split(":", 1)[1].strip())
    return kinds


def _query_from_run(run_path: str) -> dict[str, Any]:
    """Build a memory query from a stored run (symptom + anomaly kinds)."""
    from .diagnostics import _symptom, load_run_data  # noqa: PLC0415

    run, _telemetry = load_run_data(run_path)
    anomaly_kinds = [a.kind for a in run.anomalies]
    return {"symptom": _symptom(run), "anomalyKinds": anomaly_kinds, "runId": run.id}


def _score_case(
    case_entry: dict[str, Any],
    query_tokens: list[str],
    query_anomaly_kinds: set[str],
) -> Optional[dict[str, Any]]:
    """Score one stored case against the query; None when nothing matches."""
    case = case_entry["case"]
    if not query_tokens and not query_anomaly_kinds:
        return None
    per_field: dict[str, list[str]] = {}
    for field, text in _case_searchable_text(case).items():
        field_tokens = set(tokenize(text))
        per_field[field] = [token for token in query_tokens if token in field_tokens]
    score = sum(len(matched) * _FIELD_WEIGHTS.get(field, 1) for field, matched in per_field.items())

    matched_kinds = sorted(query_anomaly_kinds & _case_anomaly_kinds(case))
    score += len(matched_kinds) * _ANOMALY_MATCH_BONUS

    status = str(case.get("status", "open"))
    verified = status == "verified"
    has_conclusion = bool(str(case.get("humanConclusion", "")).strip())
    if verified:
        score += _VERIFIED_BONUS
    elif status == "closed" and has_conclusion:
        score += _CLOSED_WITH_CONCLUSION_BONUS

    if score == 0:
        return None

    rationale: list[str] = []
    for field in ("symptom", "title", "hypothesis"):
        if per_field[field]:
            rationale.append(f"{field} 命中 {len(per_field[field])} 个词: {', '.join(sorted(per_field[field])[:8])}")
    if matched_kinds:
        rationale.append(f"异常类型一致: {', '.join(matched_kinds)}")
    if verified:
        rationale.append("该案例已被人工验证为正确诊断")
    elif status == "closed" and has_conclusion:
        rationale.append("该案例已关闭且含人工结论")

    return {
        "caseId": case_entry["caseId"],
        "runId": case_entry["runId"],
        "symptom": str(case.get("symptom", "")),
        "status": status,
        "verified": verified,
        "humanConclusion": str(case.get("humanConclusion", "")),
        "score": score,
        "matchedTerms": sorted({t for matched in per_field.values() for t in matched}),
        "matchedAnomalyKinds": matched_kinds,
        "rationale": rationale,
        "casePath": case_entry["path"],
    }


def cmd_memory_retrieve(args: dict[str, Any]) -> dict[str, Any]:
    """``memory-retrieve``: find the historical cases most relevant to a run.

    Query sources (either one):
    - ``runPath`` — symptom and anomaly kinds are derived from the run record;
    - explicit ``symptom`` / ``anomalyKinds``.

    Every match carries a score and the reasons it matched, so the Agent can
    weigh the memory instead of trusting it blindly.
    """
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))
    limit = max(1, int(args.get("limit", 5)))
    min_score = max(0, int(args.get("minScore", 0)))
    exclude_run_id = args.get("excludeRunId") or ""

    run_path = args.get("runPath")
    if run_path:
        query = _query_from_run(run_path)
    else:
        symptom = args.get("symptom")
        if symptom is None or not str(symptom).strip():
            raise WorkerError("provide 'runPath' or a non-empty 'symptom' to retrieve related cases")
        query = {
            "symptom": str(symptom),
            "anomalyKinds": [str(k) for k in (args.get("anomalyKinds") or [])],
            "runId": str(args.get("runId") or ""),
        }

    query_tokens = tokenize(query["symptom"]) if tokenize else []
    query_anomaly_kinds = set(query["anomalyKinds"])
    if not query_tokens and not query_anomaly_kinds:
        return {
            "ok": True,
            "query": {"symptom": query["symptom"], "anomalyKinds": query["anomalyKinds"]},
            "related": [],
            "total": 0,
            "note": "query produced no searchable terms",
            "storeRoot": os.path.abspath(store_root),
        }

    related: list[dict[str, Any]] = []
    for case_entry in _load_cases(store_root):
        if exclude_run_id and case_entry["runId"] == exclude_run_id:
            continue
        scored = _score_case(case_entry, query_tokens, query_anomaly_kinds)
        if scored is None or scored["score"] < min_score:
            continue
        related.append(scored)
    related.sort(key=lambda item: (-item["score"], item["caseId"]))

    selected = related[:limit]
    return {
        "ok": True,
        "query": {"symptom": query["symptom"], "anomalyKinds": query["anomalyKinds"]},
        "related": selected,
        "total": len(related),
        "limit": limit,
        "note": "similarity is keyword/anomaly based, not semantic; use rationale and evidence to weigh each case",
        "storeRoot": os.path.abspath(store_root),
    }


def cmd_memory_ingest(args: dict[str, Any]) -> dict[str, Any]:
    """``memory-ingest``: record a human verdict on a diagnostic case.

    Only a human may mark a case ``verified``/``rejected``; the LLM has no
    authority to close a diagnosis. Verified cases are ranked first by
    ``memory-retrieve``.
    """
    case_id = args.get("caseId")
    if not case_id or not str(case_id).strip():
        raise WorkerError("missing required argument 'caseId'")
    status = str(args.get("status") or "").strip()
    if status and status not in _CASE_STATUSES:
        raise WorkerError(f"invalid status {status!r}; choose from {sorted(_CASE_STATUSES)}")
    store_root = normalize_store_root(args.get("storeRoot") or os.path.join(os.getcwd(), ".rh"))

    target: Optional[dict[str, Any]] = None
    for case_entry in _load_cases(store_root):
        if case_entry["caseId"] == case_id:
            target = case_entry
            break
    if target is None:
        raise WorkerError(f"unknown diagnostic case id: {case_id}")

    case = target["case"]
    conclusion = args.get("conclusion")
    if conclusion is not None:
        case["humanConclusion"] = str(conclusion)
    if status:
        case["status"] = status
        case["verified"] = status == "verified"
    case["updatedAt"] = time.time()
    operator = args.get("operator")
    if operator:
        case["operator"] = str(operator)

    with open(target["path"], "w", encoding="utf-8") as handle:
        json.dump(case, handle, ensure_ascii=False, indent=2)
    return {
        "ok": True,
        "caseId": case_id,
        "runId": case.get("runId", ""),
        "status": case.get("status", "open"),
        "verified": bool(case.get("verified")),
        "humanConclusion": str(case.get("humanConclusion", "")),
        "casePath": target["path"],
        "note": "verdict recorded; retrieve now ranks verified cases first",
    }


COMMANDS: dict[str, Any] = {
    "memory-retrieve": cmd_memory_retrieve,
    "memory-ingest": cmd_memory_ingest,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "memory.retrieve",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"runPath?": "string", "symptom?": "string", "anomalyKinds?": "list"},
        "output": "related historical cases with scores and rationale",
        "risk": "R0-readonly",
        "description": "Retrieve similar historical diagnostic cases (keyword/anomaly-based, no LLM).",
    },
    {
        "id": "memory.ingest",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"caseId": "string", "status?": "verified|rejected|closed|open", "conclusion?": "string"},
        "output": "updated case verdict",
        "risk": "R1-derive",
        "description": "Record a human verdict on a diagnostic case; verified cases rank first in retrieval.",
    },
]
