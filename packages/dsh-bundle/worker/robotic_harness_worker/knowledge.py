"""Knowledge retrieval for Robotic Harness (plan chapter 16).

Implements local documentation indexing, manual full-text search, error-code
lookup and diagnostic-case search. Design principles:

- **Evidence-first**: every retrieval result carries evidence — file path,
  line numbers, title, source table and advisory URL — so the agent can point
  at the exact place a claim comes from.
- **Source honesty**: results distinguish official documentation, community
  discussion and model inference via ``source``/``kind`` fields; the built-in
  error-code table is explicitly marked as an example that must be replaced
  with the real vendor manual.
- **No auto-repair**: this module NEVER executes repair steps. Safety-related
  advice always routes back to the vendor procedure and human approval.

Commands (exported via ``COMMANDS``): ``docs-index``, ``manual-search``,
``error-code-lookup``, ``case-search``.
"""

from __future__ import annotations

import json
import os
import re
from typing import Any, Optional

from .core import WorkerError, sha256_file

# ---------------------------------------------------------------------------
# tokenization
# ---------------------------------------------------------------------------

_LATIN_RE = re.compile(r"[a-z0-9]+(?:[-_][a-z0-9]+)*")
_CJK_RE = re.compile(r"[\u4e00-\u9fff]+")

# Built-in English stopwords (applied to latin tokens only).
STOPWORDS: frozenset[str] = frozenset(
    {
        "the", "a", "an", "and", "or", "but", "if", "then", "than", "so", "nor", "not", "no",
        "off", "on", "in", "of", "to", "for", "with", "without", "at", "by", "from", "up",
        "down", "into", "onto", "over", "under", "about", "against", "between", "through",
        "during", "before", "after", "above", "below", "again", "further", "once", "here",
        "there", "when", "where", "why", "how", "all", "any", "both", "each", "few", "more",
        "most", "other", "some", "such", "only", "own", "same", "too", "very", "just", "also",
        "can", "will", "would", "should", "could", "may", "might", "must", "shall", "do",
        "does", "did", "done", "have", "has", "had", "having", "be", "is", "are", "was",
        "were", "been", "being", "am", "i", "you", "he", "she", "it", "we", "they", "them",
        "their", "his", "her", "its", "our", "your", "my", "me", "us", "this", "that",
        "these", "those", "who", "whom", "whose", "which", "what", "per", "via", "eg",
        "ie", "etc", "vs", "as",
    }
)

# Single CJK characters that are pure grammar and are dropped when a Chinese
# run is only one character long (bigrams are always kept).
CJK_STOPWORDS: frozenset[str] = frozenset(
    {"的", "了", "是", "在", "和", "与", "及", "或", "有", "无", "为", "之", "其", "这", "那",
     "就", "都", "而", "也", "于", "等", "中", "对", "从", "以", "将", "把", "被", "让",
     "向", "并", "但", "可", "要", "会", "能", "不", "上", "下", "个", "一", "二", "三",
     "者", "所", "些", "里", "后", "前", "时", "年", "月", "日", "自", "至", "因", "果"}
)


def tokenize(text: str) -> list[str]:
    """Lowercase, strip punctuation, split into latin words + CJK bigrams."""
    text = text.lower()
    tokens: list[str] = []
    for match in _LATIN_RE.findall(text):
        if match not in STOPWORDS:
            tokens.append(match)
    for run in _CJK_RE.findall(text):
        if len(run) == 1:
            if run not in CJK_STOPWORDS:
                tokens.append(run)
        else:
            for index in range(len(run) - 1):
                tokens.append(run[index : index + 2])
    return tokens


# ---------------------------------------------------------------------------
# docs-index
# ---------------------------------------------------------------------------

DOC_EXTENSIONS = frozenset({".md", ".txt", ".rst", ".json", ".yaml", ".yml"})


def _default_index_path() -> str:
    return os.path.join(os.getcwd(), ".rh", "docs-index.json")


def _extract_title(lines: list[str]) -> str:
    """First non-empty line (strip leading markdown '#'); max 120 chars."""
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            stripped = stripped.lstrip("#").strip()
        return stripped[:120]
    return ""


def _index_file(path: str) -> Optional[dict[str, Any]]:
    """Build one inverted-index entry: {path, title, words, sha256}."""
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            lines = [line.lstrip("\ufeff") for line in handle.readlines()]
    except OSError:
        return None
    words: dict[str, list[int]] = {}
    for index, raw in enumerate(lines):
        for token in tokenize(raw):
            words.setdefault(token, []).append(index + 1)  # 1-based line numbers
    if not words:
        return None
    return {
        "path": os.path.abspath(path),
        "title": _extract_title(lines),
        "words": words,
        "sha256": sha256_file(path),
    }


def build_index(root: str, exclude: Optional[str] = None) -> dict[str, Any]:
    """Scan a directory tree and build the inverted index (in memory)."""
    root_abs = os.path.abspath(root)
    exclude_abs = os.path.abspath(exclude) if exclude else None
    files: list[str] = []
    entries: list[dict[str, Any]] = []
    for dirpath, dirnames, filenames in os.walk(root_abs):
        dirnames[:] = [d for d in dirnames if not d.startswith(".")]
        for filename in sorted(filenames):
            ext = os.path.splitext(filename)[1].lower()
            if ext not in DOC_EXTENSIONS:
                continue
            path = os.path.abspath(os.path.join(dirpath, filename))
            if exclude_abs and path == exclude_abs:
                continue
            files.append(path)
            entry = _index_file(path)
            if entry is not None:
                entries.append(entry)
    return {"root": root_abs, "files": files, "entries": entries}


def cmd_docs_index(args: dict[str, Any]) -> dict[str, Any]:
    """``docs-index``: scan a directory and persist the inverted index as JSON."""
    path = args.get("path")
    if not path:
        raise WorkerError("missing required argument 'path'")
    if not os.path.isdir(path):
        raise WorkerError(f"docs path is not a directory: {path}")
    out_path = args.get("outPath") or _default_index_path()
    index = build_index(path, exclude=out_path)
    out_dir = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(out_dir, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(index, handle, ensure_ascii=False, indent=2)
    return {
        "ok": True,
        "root": index["root"],
        "files": index["files"],
        "entries": index["entries"],
        "outPath": os.path.abspath(out_path),
        "inputArgs": {"path": path},
    }


# ---------------------------------------------------------------------------
# manual-search
# ---------------------------------------------------------------------------


def _load_index_entries(index_path: str) -> list[dict[str, Any]]:
    with open(index_path, encoding="utf-8") as handle:
        data = json.load(handle)
    entries = data.get("entries")
    if not isinstance(entries, list):
        raise WorkerError(f"invalid index file (no 'entries' list): {index_path}")
    return entries


def _resolve_index(path: Optional[str]) -> tuple[list[dict[str, Any]], str, bool]:
    """Return (entries, index_ref, built_on_the_fly).

    - ``path`` is a directory -> build the index on the fly (auto docs-index).
    - ``path`` is a file -> load the persisted index.
    - ``path`` is given but missing -> WorkerError.
    - ``path`` is None -> default ``.rh/docs-index.json``; if absent, auto-build
      from the current working directory and persist it.
    """
    if path:
        if os.path.isdir(path):
            index = build_index(path)
            return index["entries"], index["root"], True
        if os.path.isfile(path):
            return _load_index_entries(path), os.path.abspath(path), False
        raise WorkerError(f"index 路径不存在：{path}（请先运行 docs-index 生成索引）")
    default = _default_index_path()
    if os.path.isfile(default):
        return _load_index_entries(default), os.path.abspath(default), False
    index = build_index(os.getcwd(), exclude=default)
    os.makedirs(os.path.dirname(default), exist_ok=True)
    with open(default, "w", encoding="utf-8") as handle:
        json.dump(index, handle, ensure_ascii=False, indent=2)
    return index["entries"], os.path.abspath(default), True


def _score_entry(entry: dict[str, Any], query_tokens: list[str]) -> tuple[int, list[str]]:
    """Score = 3 x distinct title tokens matched + total hit lines."""
    title_tokens = set(tokenize(entry.get("title", "")))
    words = entry.get("words", {})
    matched: list[str] = []
    line_hits = 0
    for token in query_tokens:
        lines = words.get(token)
        if not lines:
            continue
        matched.append(token)
        line_hits += len(lines)
    title_matches = [token for token in matched if token in title_tokens]
    return len(title_matches) * 3 + line_hits, matched


def _build_snippets(
    path: str,
    words: dict[str, list[int]],
    matched_terms: list[str],
    top_k: int,
    radius: int = 2,
    limit: int = 200,
) -> list[dict[str, Any]]:
    """Hit-line context windows (±2 lines, truncated to 200 chars)."""
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            lines = [line.lstrip("\ufeff") for line in handle.read().splitlines()]
    except OSError:
        return []
    hit_lines: set[int] = set()
    for token in matched_terms:
        hit_lines.update(words.get(token, []))
    snippets: list[dict[str, Any]] = []
    for line_no in sorted(hit_lines)[:top_k]:
        index = line_no - 1
        if index < 0 or index >= len(lines):
            continue
        start = max(0, index - radius)
        end = min(len(lines), index + radius + 1)
        text = "\n".join(lines[start:end])
        if len(text) > limit:
            text = text[:limit] + "…"
        snippets.append({"line": line_no, "text": text})
    return snippets


def cmd_manual_search(args: dict[str, Any]) -> dict[str, Any]:
    """``manual-search``: full-text search over the doc index with evidence."""
    query = args.get("query")
    if query is None or not str(query).strip():
        raise WorkerError("missing required argument 'query'")
    query = str(query)
    try:
        max_results = int(args.get("maxResults", 10))
    except (TypeError, ValueError):
        max_results = 10
    if max_results < 1:
        max_results = 10
    try:
        top_k = int(args.get("topK", 3))
    except (TypeError, ValueError):
        top_k = 3
    if top_k < 1:
        top_k = 3

    entries, index_ref, built = _resolve_index(args.get("path"))
    query_tokens = tokenize(query)
    if not query_tokens:
        raise WorkerError(f"query 无有效检索词（去停用词后为空）：{query!r}")

    scored: list[tuple[int, dict[str, Any], list[str]]] = []
    for entry in entries:
        score, matched = _score_entry(entry, query_tokens)
        if score > 0:
            scored.append((score, entry, matched))
    scored.sort(key=lambda item: (-item[0], item[1].get("path", "")))

    results: list[dict[str, Any]] = []
    for score, entry, matched in scored[:max_results]:
        snippets = _build_snippets(entry["path"], entry.get("words", {}), matched, top_k)
        results.append(
            {
                "path": entry["path"],
                "title": entry.get("title", ""),
                "score": score,
                "matchedTerms": sorted(matched),
                "snippets": snippets,
            }
        )
    return {
        "ok": True,
        "query": query,
        "results": results,
        "total": len(scored),
        "index": index_ref,
        "indexBuiltOnTheFly": built,
        "inputArgs": {"query": query, "maxResults": max_results, "topK": top_k},
    }


# ---------------------------------------------------------------------------
# error-code-lookup
# ---------------------------------------------------------------------------

# Placeholder examples only — the note field always says the table must be
# replaced with the real vendor manual before production use.
BUILTIN_ERROR_CODES: list[dict[str, Any]] = [
    {
        "code": "1",
        "meaning": "ROS node crash（节点崩溃）",
        "severity": "error",
        "source": "builtin-example",
        "advice": "查看节点日志与 core dump，人工定位崩溃原因后按厂商流程处理；LLM 不得自动执行修复。",
        "url": "https://wiki.ros.org/ROS/Troubleshooting",
    },
    {
        "code": "2",
        "meaning": "ROS master/roscore 不可用或连接失败",
        "severity": "error",
        "source": "builtin-example",
        "advice": "确认 roscore 状态与 ROS_MASTER_URI 配置；现场人工按厂商手册重启后重试。",
        "url": "https://wiki.ros.org/ROS/Troubleshooting",
    },
    {
        "code": "3",
        "meaning": "TF tree 超时/缺失（tf2 timeout）",
        "severity": "warning",
        "source": "builtin-example",
        "advice": "检查 TF broadcaster 频率与坐标系配置；涉及安全停机的 TF 丢失须人工确认后恢复。",
        "url": "https://wiki.ros.org/tf2/Troubleshooting",
    },
    {
        "code": "4",
        "meaning": "关节限位超出（joint limit exceeded）",
        "severity": "error",
        "source": "builtin-example",
        "advice": "停止运动，人工检查关节角度与轨迹规划；限位超限可能造成机械损坏，须现场确认。",
        "url": "https://wiki.ros.org/URDF/XML/joint",
    },
    {
        "code": "5",
        "meaning": "相机标定无效或版本不符",
        "severity": "warning",
        "source": "builtin-example",
        "advice": "重新执行标定并核对标定文件版本；标定结果影响安全距离判断，须人工复核。",
        "url": "",
    },
    {
        "code": "6",
        "meaning": "控制器模式不在期望模式（controller mode mismatch）",
        "severity": "error",
        "source": "builtin-example",
        "advice": "确认控制器处于期望模式（如 position/velocity）；模式切换属于实机操作，须现场人工执行。",
        "url": "",
    },
    {
        "code": "7",
        "meaning": "action goal 超时（action timeout）",
        "severity": "warning",
        "source": "builtin-example",
        "advice": "检查 action server 状态与网络延迟；超时后先人工确认机器人处于安全位姿再继续。",
        "url": "https://wiki.ros.org/actionlib",
    },
]

BUILTIN_NOTE = "内置示例表仅用于演示格式，需替换为真实厂商手册；维修步骤必须由人工执行。"


def _find_code(codes: list[dict[str, Any]], code: str) -> Optional[dict[str, Any]]:
    for entry in codes:
        candidate = str(entry.get("code", "")).strip()
        if candidate == code:
            return entry
        if candidate.isdigit() and code.isdigit() and int(candidate) == int(code):
            return entry
    return None


def _closest_code(codes: list[dict[str, Any]], code: str) -> Optional[dict[str, Any]]:
    """Best-effort closest match: numeric distance first, then string similarity."""
    if code.isdigit():
        numeric = [(abs(int(str(e.get("code", ""))) - int(code)), e) for e in codes if str(e.get("code", "")).isdigit()]
        if numeric:
            return min(numeric, key=lambda item: item[0])[1]
    candidates = [e for e in codes if str(e.get("code", "")).strip()]
    if not candidates:
        return None
    scored = []
    for entry in candidates:
        a, b = code.lower(), str(entry.get("code", "")).lower()
        shorter, longer = sorted((a, b), key=len)
        overlap = sum(1 for i in range(len(shorter)) if shorter[i] == longer[i])
        scored.append((overlap, -abs(len(a) - len(b)), entry))
    return max(scored, key=lambda item: (item[0], item[1]))[2]


def cmd_error_code_lookup(args: dict[str, Any]) -> dict[str, Any]:
    """``error-code-lookup``: look up an error code in the builtin or user table."""
    code = args.get("code")
    if code is None or not str(code).strip():
        raise WorkerError("missing required argument 'code'")
    code = str(code).strip()
    table_path = args.get("tablePath")
    if table_path:
        if not os.path.isfile(table_path):
            raise WorkerError(f"error code table not found: {table_path}")
        with open(table_path, encoding="utf-8") as handle:
            data = json.load(handle)
        codes = data.get("codes")
        if not isinstance(codes, list):
            raise WorkerError(f"invalid error code table (no 'codes' list): {table_path}")
        source = "user"
        note = "来自用户错误码表（tablePath）。"
    else:
        codes = BUILTIN_ERROR_CODES
        source = "builtin-example"
        note = BUILTIN_NOTE

    entry = _find_code(codes, code)
    if entry is None:
        return {
            "ok": True,
            "code": code,
            "found": False,
            "closest": _closest_code(codes, code),
            "source": source,
            "note": "未找到该错误码；如来自厂商手册请提供 tablePath 用户表。",
            "inputArgs": {"code": code},
        }
    return {
        "ok": True,
        "code": code,
        "found": True,
        "entry": entry,
        "source": source,
        "note": note,
        "inputArgs": {"code": code},
    }


# ---------------------------------------------------------------------------
# case-search
# ---------------------------------------------------------------------------


def _case_searchable_text(case: dict[str, Any]) -> dict[str, str]:
    titles = " ".join(str(f.get("title", "")) for f in case.get("findings", []) if isinstance(f, dict))
    hypothesis_parts: list[str] = []
    for hypothesis in case.get("hypotheses", []):
        if not isinstance(hypothesis, dict):
            continue
        hypothesis_parts.append(str(hypothesis.get("title", "")))
        hypothesis_parts.append(" ".join(str(v) for v in hypothesis.get("support", [])))
        hypothesis_parts.append(" ".join(str(v) for v in hypothesis.get("suggestedChecks", [])))
    return {
        "symptom": str(case.get("symptom", "")),
        "title": titles,
        "hypothesis": " ".join(hypothesis_parts),
    }


def _primary_field(per_field: dict[str, list[str]]) -> str:
    best_count = max((len(v) for v in per_field.values()), default=0)
    fields = [field for field in ("symptom", "title", "hypothesis") if len(per_field[field]) == best_count]
    return "+".join(fields)


def cmd_case_search(args: dict[str, Any]) -> dict[str, Any]:
    """``case-search``: full-text search over ``<storeRoot>/cases/*.json``."""
    query = args.get("query")
    if query is None or not str(query).strip():
        raise WorkerError("missing required argument 'query'")
    query = str(query)
    store_root = args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")
    cases_dir = os.path.join(store_root, "cases")
    query_tokens = tokenize(query)
    if not query_tokens or not os.path.isdir(cases_dir):
        return {
            "ok": True,
            "query": query,
            "results": [],
            "total": 0,
            "storeRoot": os.path.abspath(store_root),
            "inputArgs": {"query": query},
        }

    results: list[dict[str, Any]] = []
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
        per_field: dict[str, list[str]] = {}
        for field, text in _case_searchable_text(case).items():
            field_tokens = set(tokenize(text))
            per_field[field] = [token for token in query_tokens if token in field_tokens]
        score = sum(len(matched) * (2 if field == "symptom" else 1) for field, matched in per_field.items())
        if score == 0:
            continue
        results.append(
            {
                "caseId": case.get("id", os.path.splitext(filename)[0]),
                "runId": case.get("runId", ""),
                "symptom": case.get("symptom", ""),
                "status": case.get("status", "open"),
                "score": score,
                "matchedField": _primary_field(per_field),
            }
        )
    results.sort(key=lambda item: (-item["score"], item["caseId"]))
    return {
        "ok": True,
        "query": query,
        "results": results,
        "total": len(results),
        "storeRoot": os.path.abspath(store_root),
        "inputArgs": {"query": query},
    }


# ---------------------------------------------------------------------------
# module exports
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "docs-index": cmd_docs_index,
    "manual-search": cmd_manual_search,
    "error-code-lookup": cmd_error_code_lookup,
    "case-search": cmd_case_search,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "knowledge.docs_search",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"query": "string", "path": "string?"},
        "output": "indexed documentation hits with evidence (file, line, title)",
        "risk": "R0-readonly",
        "description": "全文检索本地 .md/.txt/.rst/.json/.yaml 文档；结果带文件/行号/标题证据，区分官方文档/社区/模型推断，不执行任何维修步骤。",
    },
    {
        "id": "knowledge.error_code_lookup",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"code": "string", "tablePath": "string?"},
        "output": "error code entry with meaning, severity, source and advice",
        "risk": "R0-readonly",
        "description": "错误码查询（内置 ROS 示例表 + 用户表覆盖）；安全相关建议始终回到厂商流程与人工审批。",
    },
    {
        "id": "knowledge.case_search",
        "kind": "knowledge",
        "provider": "robotic-harness-worker",
        "input": {"query": "string", "storeRoot": "string?"},
        "output": "matching diagnostic cases with matched field and score",
        "risk": "R0-readonly",
        "description": "在 Run 存储的 cases 目录中按症状/标题/假设全文检索历史诊断案例。",
    },
]
