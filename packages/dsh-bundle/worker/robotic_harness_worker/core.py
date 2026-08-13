"""Core domain models and the on-disk run store for Robotic Harness.

These models are intentionally portable: they contain no Cordis or DSH
specifics, so the same JSON files can be consumed by the DSH bundle, a future
OpenRAL integration, or plain scripts. All objects serialize to plain JSON via
:func:`to_dict`.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import time
import uuid
from dataclasses import asdict, dataclass, field
from typing import Any, Optional

KIND_ORIGIN = {"fact", "rule", "inference", "manual"}

RUN_STATES = {"draft", "running", "completed", "failed", "aborted"}


def _slug(value: str) -> str:
    """Normalize a string into a safe file-name fragment."""
    value = re.sub(r"[^a-zA-Z0-9_.-]+", "-", value).strip("-")
    return value or "item"


def new_id(prefix: str) -> str:
    """Generate a short unique id like ``run-8f3a2c1d``."""
    return f"{prefix}-{uuid.uuid4().hex[:8]}"


def sha256_file(path: str, chunk: int = 1 << 20) -> str:
    """Compute the SHA-256 of a file without loading it into memory."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


@dataclass
class RoboticsProject:
    """A robotics research project: identity, paths, environment snapshot."""

    id: str
    name: str
    root: str
    description: str = ""
    git_commit: str = ""
    ros_distribution: str = ""
    created_at: str = field(default_factory=lambda: time.time())
    env_snapshot: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Issue:
    """One finding from an asset inspection or validation pass."""

    severity: str  # error | warning | info
    code: str
    message: str
    location: str = ""
    evidence: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class AssetInspection:
    """Structured summary of a robot asset (URDF/MJCF) plus issues."""

    format: str
    path: str
    summary: dict[str, Any]
    issues: list[Issue] = field(default_factory=list)
    warnings_from_loader: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": self.format,
            "path": self.path,
            "summary": self.summary,
            "issues": [i.to_dict() for i in self.issues],
            "warningsFromLoader": self.warnings_from_loader,
        }


@dataclass
class Capability:
    """A capability that an Agent, Workflow or Skill may select."""

    id: str
    kind: str
    provider: str
    description: str
    risk: str = "R0-readonly"
    input: dict[str, Any] = field(default_factory=dict)
    output: str = ""
    version: str = "0.1.0"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class PhaseEvent:
    """A named phase transition inside a run."""

    phase: str
    time_s: float
    outcome: str = "ok"  # ok | failed | recovered
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Anomaly:
    """A deterministic anomaly detected while the run was in progress."""

    kind: str
    time_s: float
    detail: str
    evidence: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Run:
    """One simulation, replay, or real execution."""

    id: str
    project_id: str
    scenario: str
    state: str  # RUN_STATES
    created_at: float = field(default_factory=time.time)
    config: dict[str, Any] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)
    phases: list[PhaseEvent] = field(default_factory=list)
    anomalies: list[Anomaly] = field(default_factory=list)
    artifacts: dict[str, str] = field(default_factory=dict)  # name -> path
    final_result: dict[str, Any] = field(default_factory=dict)
    notes: list[dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "projectId": self.project_id,
            "scenario": self.scenario,
            "state": self.state,
            "createdAt": self.created_at,
            "config": self.config,
            "metrics": self.metrics,
            "phases": [p.to_dict() for p in self.phases],
            "anomalies": [a.to_dict() for a in self.anomalies],
            "artifacts": self.artifacts,
            "finalResult": self.final_result,
            "notes": self.notes,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Run":
        return cls(
            id=data["id"],
            project_id=data.get("projectId", ""),
            scenario=data.get("scenario", ""),
            state=data.get("state", "draft"),
            created_at=data.get("createdAt", time.time()),
            config=data.get("config", {}),
            metrics=data.get("metrics", {}),
            phases=[PhaseEvent(**p) for p in data.get("phases", [])],
            anomalies=[Anomaly(**a) for a in data.get("anomalies", [])],
            artifacts=data.get("artifacts", {}),
            final_result=data.get("finalResult", {}),
            notes=data.get("notes", []),
        )


@dataclass
class Finding:
    """One diagnostic finding: a fact, a rule result, or an inference."""

    origin: str  # KIND_ORIGIN
    title: str
    detail: str
    evidence: list[str] = field(default_factory=list)
    time_s: Optional[float] = None
    confidence: str = "high"  # high | medium | low

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Hypothesis:
    """A candidate root cause with support, counter-evidence and checks."""

    id: str
    layer: str  # e.g. mechanical, calibration, perception, planning, control, system
    title: str
    support: list[str] = field(default_factory=list)
    counter_evidence: list[str] = field(default_factory=list)
    missing_evidence: list[str] = field(default_factory=list)
    suggested_checks: list[str] = field(default_factory=list)
    likelihood: str = "unknown"  # high | medium | low | unknown
    requires_human: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class DiagnosticCase:
    """A problem localization record: symptom, facts, hypotheses, resolution."""

    id: str
    run_id: str
    symptom: str
    created_at: float = field(default_factory=time.time)
    findings: list[Finding] = field(default_factory=list)
    hypotheses: list[Hypothesis] = field(default_factory=list)
    status: str = "open"  # open | verified | closed
    human_conclusion: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "runId": self.run_id,
            "symptom": self.symptom,
            "createdAt": self.created_at,
            "findings": [f.to_dict() for f in self.findings],
            "hypotheses": [h.to_dict() for h in self.hypotheses],
            "status": self.status,
            "humanConclusion": self.human_conclusion,
        }


class RunStore:
    """Filesystem-backed store for runs and diagnostic cases.

    Layout under ``root``::

        <root>/runs/<run-id>/run.json
        <root>/runs/<run-id>/telemetry.jsonl
        <root>/runs/<run-id>/artifacts/...
        <root>/cases/<case-id>.json
        <root>/index.json
    """

    def __init__(self, root: str) -> None:
        self.root = root

    def ensure(self) -> None:
        os.makedirs(os.path.join(self.root, "runs"), exist_ok=True)
        os.makedirs(os.path.join(self.root, "cases"), exist_ok=True)

    def run_dir(self, run_id: str) -> str:
        return os.path.join(self.root, "runs", _slug(run_id))

    def artifact_dir(self, run_id: str) -> str:
        return os.path.join(self.run_dir(run_id), "artifacts")

    def save_run(self, run: Run) -> str:
        self.ensure()
        run_dir = self.run_dir(run.id)
        os.makedirs(run_dir, exist_ok=True)
        path = os.path.join(run_dir, "run.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(run.to_dict(), handle, ensure_ascii=False, indent=2)
        self._touch_index(run)
        return path

    def load_run(self, run_id: str) -> Run:
        path = os.path.join(self.run_dir(run_id), "run.json")
        if not os.path.exists(path):
            raise FileNotFoundError(f"run {run_id} not found at {path}")
        with open(path, encoding="utf-8") as handle:
            return Run.from_dict(json.load(handle))

    def append_telemetry(self, run_id: str, rows: list[dict[str, Any]]) -> None:
        self.ensure()
        os.makedirs(self.run_dir(run_id), exist_ok=True)
        path = os.path.join(self.run_dir(run_id), "telemetry.jsonl")
        with open(path, "a", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, ensure_ascii=False) + "\n")

    def save_case(self, case: DiagnosticCase) -> str:
        self.ensure()
        path = os.path.join(self.root, "cases", f"{_slug(case.id)}.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(case.to_dict(), handle, ensure_ascii=False, indent=2)
        return path

    def load_case(self, case_id: str) -> DiagnosticCase:
        path = os.path.join(self.root, "cases", f"{_slug(case_id)}.json")
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        case = DiagnosticCase(
            id=data["id"],
            run_id=data.get("runId", ""),
            symptom=data.get("symptom", ""),
            created_at=data.get("createdAt", time.time()),
            findings=[Finding(**f) for f in data.get("findings", [])],
            hypotheses=[Hypothesis(**h) for h in data.get("hypotheses", [])],
            status=data.get("status", "open"),
            human_conclusion=data.get("humanConclusion", ""),
        )
        return case

    def list_runs(self) -> list[dict[str, Any]]:
        self.ensure()
        index_path = os.path.join(self.root, "index.json")
        if os.path.exists(index_path):
            with open(index_path, encoding="utf-8") as handle:
                return json.load(handle).get("runs", [])
        return []

    def _touch_index(self, run: Run) -> None:
        index_path = os.path.join(self.root, "index.json")
        entries = self.list_runs()
        entries = [e for e in entries if e.get("id") != run.id]
        entries.append(
            {
                "id": run.id,
                "scenario": run.scenario,
                "state": run.state,
                "createdAt": run.created_at,
                "success": bool(run.metrics.get("success")),
                "runDir": self.run_dir(run.id),
            }
        )
        entries.sort(key=lambda e: e.get("createdAt", 0), reverse=True)
        with open(index_path, "w", encoding="utf-8") as handle:
            json.dump({"runs": entries}, handle, ensure_ascii=False, indent=2)


def snapshot_environment() -> dict[str, Any]:
    """Best-effort environment snapshot recorded in every run."""
    snapshot: dict[str, Any] = {
        "python": os.sys.version.split()[0],
        "platform": os.sys.platform,
    }
    for module_name in ("numpy", "mujoco", "cv2", "matplotlib"):
        try:
            module = __import__(module_name)
            snapshot[module_name] = getattr(module, "__version__", "unknown")
        except Exception:
            snapshot[module_name] = None
    return snapshot
