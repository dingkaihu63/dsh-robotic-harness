"""Evidence bundles, Markdown reports and the standalone timeline viewer.

The EvidenceBundle is the plan's reproducibility unit: a directory with a
manifest (versions, hashes, config snapshot), the run record, telemetry,
charts and the diagnostic case. The timeline HTML is a single self-contained
file with embedded JSON — it needs no server, no build step and no network.
"""

from __future__ import annotations

import html
import json
import os
import time
from typing import Any

from .core import DiagnosticCase, Run, RunStore, sha256_file, snapshot_environment
from .diagnostics import load_run_data


def export_evidence(
    run_path: str,
    case: DiagnosticCase | None,
    out_dir: str,
    include_telemetry: bool = True,
) -> dict[str, Any]:
    """Copy a run's artifacts into a self-contained evidence bundle.

    Returns the bundle manifest. The bundle never references paths outside
    itself, so it can be moved or archived as a unit.
    """
    run, telemetry = load_run_data(run_path)
    run_dir = os.path.dirname(os.path.abspath(run_path)) if os.path.isfile(run_path) else os.path.abspath(run_path)
    os.makedirs(out_dir, exist_ok=True)

    manifest_files: list[dict[str, Any]] = []

    def copy_artifact(source: str, target_name: str) -> None:
        if not os.path.exists(source):
            return
        import shutil

        target = os.path.join(out_dir, target_name)
        shutil.copy2(source, target)
        manifest_files.append({"name": target_name, "sha256": sha256_file(target), "size": os.path.getsize(target)})

    copy_artifact(os.path.join(run_dir, "run.json"), "run.json")
    copy_artifact(os.path.join(run_dir, "telemetry.jsonl"), "telemetry.jsonl")
    artifact_dir = os.path.join(run_dir, "artifacts")
    if os.path.isdir(artifact_dir):
        for entry in sorted(os.listdir(artifact_dir)):
            copy_artifact(os.path.join(artifact_dir, entry), entry)

    case_record: dict[str, Any] | None = None
    if case is not None:
        case_record = case.to_dict()
        case_path = os.path.join(out_dir, "diagnostics.json")
        with open(case_path, "w", encoding="utf-8") as handle:
            json.dump(case_record, handle, ensure_ascii=False, indent=2)
        manifest_files.append({"name": "diagnostics.json", "sha256": sha256_file(case_path), "size": os.path.getsize(case_path)})

    manifest = {
        "schemaVersion": 1,
        "kind": "evidence-bundle",
        "createdAt": time.time(),
        "project": "robotic-harness",
        "run": {
            "id": run.id,
            "scenario": run.scenario,
            "state": run.state,
            "success": bool(run.metrics.get("success")),
            "metrics": run.metrics,
            "config": run.config,
        },
        "environment": snapshot_environment(),
        "files": manifest_files,
        "notes": [
            "the evidence bundle is the unit of reproducibility: keep it whole, verify hashes before analysis",
            "telemetry is a downsampled JSONL; the full physics stream is not recorded by design",
        ],
    }
    manifest_path = os.path.join(out_dir, "manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)
    return manifest


def generate_report(
    run_path: str,
    case: DiagnosticCase | None,
    out_path: str,
    evidence_dir: str | None = None,
) -> str:
    """Generate a Markdown experiment/diagnostic report."""
    run, telemetry = load_run_data(run_path)
    metrics = run.metrics
    lines: list[str] = []

    def h(level: int, text: str) -> None:
        lines.append(f"{'#' * level} {text}\n")

    h(1, f"Robotic Harness experiment report — run {run.id}")
    lines.append(
        f"> Scenario `{run.scenario}` · state `{run.state}` · success **{metrics.get('success')}** · "
        f"seed `{run.config.get('seed')}`\n"
    )

    h(2, "Summary")
    lines.append(
        f"- Steps: {metrics.get('steps')} ({metrics.get('durationS')} s simulated)\n"
        f"- Tracking error RMS: {metrics.get('trackingErrorRms')} rad\n"
        f"- Object final: {metrics.get('objectFinal')} vs target zone {metrics.get('targetZone', {}).get('center')} ± {metrics.get('targetZone', {}).get('radius')} m\n"
        f"- Grasped: {metrics.get('grasped')} · Slipped: {metrics.get('slipped')} · In target zone: {metrics.get('inTargetZone')}\n"
        f"- Perception: route `{metrics.get('perceptionRoute')}` via {metrics.get('renderer')} renderer\n"
    )

    h(2, "Fault configuration")
    lines.append("```json")
    lines.append(json.dumps(run.config.get("fault", {}), ensure_ascii=False, indent=2))
    lines.append("```\n")

    h(2, "Phases")
    for phase in run.phases:
        outcome = "" if phase.outcome == "ok" else f" **({phase.outcome})**"
        detail = f" — {phase.detail}" if phase.detail else ""
        lines.append(f"- `{phase.phase}` at {phase.time_s:.3f}s{outcome}{detail}")
    lines.append("")

    h(2, "Anomalies (deterministic layer-1 detection)")
    if run.anomalies:
        for anomaly in run.anomalies:
            lines.append(
                f"- **{anomaly.kind}** at {anomaly.time_s:.3f}s: {anomaly.detail} "
                f"`{json.dumps(anomaly.evidence, ensure_ascii=False)}`"
            )
    else:
        lines.append("- none\n")

    if case is not None:
        h(2, "Diagnostics")
        h(3, "Facts and rule findings")
        for finding in case.findings:
            stamp = f" at {finding.time_s:.3f}s" if finding.time_s is not None else ""
            lines.append(
                f"- *{finding.origin}*{stamp} — **{finding.title}**: {finding.detail}\n"
                + ("  - Evidence: " + "; ".join(f"`{e}`" for e in finding.evidence) + "\n" if finding.evidence else "")
            )
        h(3, "Candidate root causes (need human confirmation)")
        for hypothesis in case.hypotheses:
            lines.append(
                f"- **[{hypothesis.layer}] {hypothesis.title}** (likelihood: {hypothesis.likelihood}"
                + (", human confirmation required" if hypothesis.requires_human else "")
                + ")\n"
                + (f"  - Support: {hypothesis.support}\n" if hypothesis.support else "")
                + (f"  - Counter-evidence: {hypothesis.counter_evidence}\n" if hypothesis.counter_evidence else "")
                + (f"  - Missing evidence: {hypothesis.missing_evidence}\n" if hypothesis.missing_evidence else "")
                + (f"  - Suggested checks: {hypothesis.suggested_checks}\n" if hypothesis.suggested_checks else "")
            )

    h(2, "Artifacts")
    for name, path in run.artifacts.items():
        lines.append(f"- `{name}` → `{path}`")
    if evidence_dir:
        lines.append(f"- Evidence bundle → `{evidence_dir}`")
    lines.append("")

    h(2, "Limitations")
    lines.append(
        "- This report is auto-generated; conclusions in the diagnostics section are hypotheses, not facts.\n"
        "- Simulation results do not certify real-robot safety (sim-to-real gap is not measured here).\n"
        "- The suction grasp is kinematic; see the run config sim notes.\n"
    )

    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
    return out_path


def replay_run(run_path: str, out_dir: str, include_report: bool = True) -> dict[str, Any]:
    """Replay a stored run into a self-contained replay package.

    Read-only by design: replay never re-executes simulation or tools. It
    copies the run record, telemetry, artifacts and diagnostics into
    ``out_dir`` and regenerates the timeline (and optionally the Markdown
    report) so the run can be inspected without the original workspace.
    """
    from .diagnostics import diagnose, load_run_data

    run, telemetry = load_run_data(run_path)
    run_dir = os.path.dirname(os.path.abspath(run_path)) if os.path.isfile(run_path) else os.path.abspath(run_path)
    os.makedirs(out_dir, exist_ok=True)
    copied: list[str] = []

    import shutil

    for name in ("run.json", "telemetry.jsonl"):
        source = os.path.join(run_dir, name)
        if os.path.exists(source):
            shutil.copy2(source, os.path.join(out_dir, name))
            copied.append(name)
    artifact_dir = os.path.join(run_dir, "artifacts")
    if os.path.isdir(artifact_dir):
        target_dir = os.path.join(out_dir, "artifacts")
        os.makedirs(target_dir, exist_ok=True)
        for entry in sorted(os.listdir(artifact_dir)):
            shutil.copy2(os.path.join(artifact_dir, entry), os.path.join(target_dir, entry))
            copied.append(f"artifacts/{entry}")

    case = diagnose(run, telemetry)
    timeline_path = timeline_html(run, telemetry, case, os.path.join(out_dir, "timeline.html"))
    copied.append(os.path.basename(timeline_path))
    report_path = None
    if include_report:
        report_path = generate_report(run_path, case, os.path.join(out_dir, "report.md"), evidence_dir=out_dir)
        copied.append(os.path.basename(report_path))

    return {
        "ok": True,
        "runId": run.id,
        "outDir": os.path.abspath(out_dir),
        "copied": copied,
        "timeline": timeline_path,
        "report": report_path,
        "note": "replay is read-only: no simulation or tools were re-executed",
    }


def dashboard_html(store_root: str, out_path: str) -> str:
    """Generate a single-file dashboard over the run store.

    Tabs: overview (run list), runs (metrics table), diagnostics (open
    cases), artifacts. Everything is embedded — no server or network needed.
    """
    store = RunStore(store_root)
    runs = store.list_runs()
    cases: list[dict[str, Any]] = []
    cases_dir = os.path.join(store_root, "cases")
    if os.path.isdir(cases_dir):
        for entry in sorted(os.listdir(cases_dir)):
            if entry.endswith(".json"):
                with open(os.path.join(cases_dir, entry), encoding="utf-8") as handle:
                    try:
                        cases.append(json.load(handle))
                    except json.JSONDecodeError:
                        continue

    payload = {"storeRoot": store_root, "runs": runs, "cases": cases}
    embedded = html.escape(json.dumps(payload, ensure_ascii=False), quote=True)
    page = _DASHBOARD_TEMPLATE.replace("__PAYLOAD__", embedded)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(page)
    return out_path


_DASHBOARD_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Robotic Harness — dashboard</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 24px; background: #fafafa; color: #1a1a1a; }
  h1 { font-size: 20px; } h2 { font-size: 15px; margin-top: 24px; }
  table { border-collapse: collapse; width: 100%; font-size: 12px; }
  th, td { border: 1px solid #ddd; padding: 4px 8px; text-align: left; }
  th { background: #eef; }
  .ok { color: #146c43; font-weight: 600; } .bad { color: #b02a37; font-weight: 600; }
  .tab { display: inline-block; padding: 6px 16px; margin-right: 4px; background: #e9ecef;
         border-radius: 6px 6px 0 0; cursor: pointer; font-size: 13px; }
  .tab.active { background: #0d6efd; color: #fff; }
  .panel { display: none; border: 1px solid #dee2e6; padding: 12px; }
  .panel.active { display: block; }
  .metric { display: inline-block; background: #fff; border: 1px solid #ddd; border-radius: 8px;
            padding: 8px 14px; margin: 4px 8px 4px 0; }
  .metric b { display: block; font-size: 18px; }
  .metric span { font-size: 11px; color: #666; }
</style>
</head>
<body>
<h1>Robotic Harness — dashboard</h1>
<div>
  <span class="tab active" onclick="show('overview')">Overview</span>
  <span class="tab" onclick="show('runs')">Runs</span>
  <span class="tab" onclick="show('cases')">Diagnostics</span>
</div>
<div id="overview" class="panel active"></div>
<div id="runs" class="panel"></div>
<div id="cases" class="panel"></div>
<script>
const payload = JSON.parse(decodeURIComponent("__PAYLOAD__"));
const runs = payload.runs || [], cases = payload.cases || [];
function show(name) {
  document.querySelectorAll('.tab').forEach((t, i) => t.classList.toggle('active', i === ['overview','runs','cases'].indexOf(name)));
  document.querySelectorAll('.panel').forEach(p => p.classList.toggle('active', p.id === name));
}
const total = runs.length, success = runs.filter(r => r.success).length;
const failed = total - success;
document.getElementById('overview').innerHTML =
  `<div class="metric"><span>runs</span><b>${total}</b></div>` +
  `<div class="metric"><span>success</span><b class="ok">${success}</b></div>` +
  `<div class="metric"><span>failed</span><b class="bad">${failed}</b></div>` +
  `<div class="metric"><span>open cases</span><b>${cases.filter(c => c.status === 'open').length}</b></div>` +
  `<h2>Recent runs</h2>` + runsTable(runs.slice(0, 10));
function runsTable(list) {
  if (!list.length) return '<p>no runs yet</p>';
  return '<table><tr><th>run</th><th>scenario</th><th>state</th><th>success</th><th>created</th><th>dir</th></tr>' +
    list.map(r => `<tr><td>${r.id}</td><td>${r.scenario}</td><td>${r.state}</td>` +
      `<td class="${r.success ? 'ok' : 'bad'}">${r.success}</td><td>${new Date(r.createdAt * 1000).toLocaleString()}</td>` +
      `<td><code>${r.runDir}</code></td></tr>`).join('') + '</table>';
}
document.getElementById('runs').innerHTML = runsTable(runs);
document.getElementById('cases').innerHTML = cases.length
  ? '<table><tr><th>case</th><th>run</th><th>symptom</th><th>status</th><th>hypotheses</th></tr>' +
    cases.map(c => `<tr><td>${c.id}</td><td>${c.runId}</td><td>${c.symptom}</td>` +
      `<td>${c.status}</td><td>${(c.hypotheses || []).length}</td></tr>`).join('') + '</table>'
  : '<p>no diagnostic cases yet</p>';
</script>
</body>
</html>
"""


def timeline_html(run: Run, telemetry: list[dict[str, Any]], case: DiagnosticCase | None, out_path: str) -> str:
    """Write a self-contained timeline viewer (no server, no network)."""
    payload = {
        "run": run.to_dict(),
        "telemetry": telemetry,
        "case": case.to_dict() if case else None,
    }
    embedded = json.dumps(payload, ensure_ascii=False)
    page = _TIMELINE_TEMPLATE.replace("__PAYLOAD__", html.escape(embedded, quote=True))
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(page)
    return out_path


_TIMELINE_TEMPLATE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Robotic Harness — run timeline</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 24px; background: #fafafa; color: #1a1a1a; }
  h1 { font-size: 20px; } h2 { font-size: 15px; margin-top: 28px; }
  table { border-collapse: collapse; width: 100%; font-size: 12px; }
  th, td { border: 1px solid #ddd; padding: 4px 8px; text-align: left; }
  th { background: #eef; }
  .fact { color: #0a58ca; } .rule { color: #b02a37; } .inference { color: #805b00; }
  .ok { color: #146c43; font-weight: 600; } .bad { color: #b02a37; font-weight: 600; }
  #chart { width: 100%; height: 300px; }
  .metric { display: inline-block; background: #fff; border: 1px solid #ddd; border-radius: 8px;
            padding: 8px 14px; margin: 4px 8px 4px 0; }
  .metric b { display: block; font-size: 18px; }
  .metric span { font-size: 11px; color: #666; }
  .legend span { margin-right: 14px; font-size: 12px; }
</style>
</head>
<body>
<h1>Robotic Harness — run timeline</h1>
<div id="summary"></div>
<h2>Joint positions (actual vs target)</h2>
<canvas id="chart" width="1100" height="300"></canvas>
<h2>Phases &amp; anomalies</h2>
<div id="events"></div>
<h2>Diagnostics</h2>
<div id="diagnostics"></div>
<script>
const payload = JSON.parse(decodeURIComponent("__PAYLOAD__"));
const run = payload.run, telemetry = payload.telemetry, case_ = payload.case;
const summary = document.getElementById("summary");
summary.innerHTML = [
  ['state', run.state], ['success', String(run.metrics.success)], ['steps', run.metrics.steps],
  ['duration (s)', run.metrics.durationS], ['tracking RMS', run.metrics.trackingErrorRms],
  ['route', run.metrics.perceptionRoute], ['renderer', run.metrics.renderer],
].map(([k, v]) => `<div class="metric"><span>${k}</span><b>${v}</b></div>`).join('') +
`<div class="metric"><span>fault</span><b>${JSON.stringify(run.config.fault || {})}</b></div>`;

const canvas = document.getElementById("chart");
const ctx = canvas.getContext("2d");
const names = ['shoulder', 'elbow', 'wrist'];
const colors = ['#d62728', '#1f77b4', '#2ca02c'];
const times = telemetry.map(r => r.t);
const maxT = Math.max(...times, 1), minT = Math.min(...times, 0);
const pad = { l: 40, r: 12, t: 8, b: 22 };
function sx(t) { return pad.l + (t - minT) / (maxT - minT) * (canvas.width - pad.l - pad.r); }
function sy(v, vmin, vmax) { return pad.t + (1 - (v - vmin) / (vmax - vmin)) * (canvas.height - pad.t - pad.b); }
let vmin = -3.5, vmax = 3.5;
ctx.strokeStyle = '#ccc'; ctx.fillStyle = '#333'; ctx.font = '10px system-ui';
for (let i = 0; i <= 6; i++) {
  const v = vmin + (vmax - vmin) * i / 6;
  ctx.beginPath(); ctx.moveTo(pad.l, sy(v, vmin, vmax)); ctx.lineTo(canvas.width - pad.r, sy(v, vmin, vmax)); ctx.stroke();
  ctx.fillText(v.toFixed(1), 2, sy(v, vmin, vmax) + 3);
}
function line(getY, color, dash) {
  ctx.beginPath();
  ctx.strokeStyle = color; ctx.setLineDash(dash || []);
  telemetry.forEach((row, i) => { const y = getY(row); i ? ctx.lineTo(sx(row.t), y) : ctx.moveTo(sx(row.t), y); });
  ctx.stroke(); ctx.setLineDash([]);
}
names.forEach((name, i) => {
  line(r => sy(r.q[i], vmin, vmax), colors[i]);
  line(r => sy(r.qTarget[i], vmin, vmax), colors[i] + '88', [4, 3]);
});
ctx.strokeStyle = '#333';
ctx.beginPath(); ctx.moveTo(pad.l, pad.t); ctx.lineTo(pad.l, canvas.height - pad.b); ctx.lineTo(canvas.width - pad.r, canvas.height - pad.b); ctx.stroke();

const events = document.getElementById("events");
let htmlEvents = '<table><tr><th>t (s)</th><th>kind</th><th>detail</th></tr>';
run.phases.forEach(p => htmlEvents += `<tr><td>${p.timeS !== undefined ? p.timeS.toFixed(3) : ''}</td><td>phase: ${p.phase}</td><td class="${p.outcome === 'ok' ? 'ok' : 'bad'}">${p.outcome} ${p.detail || ''}</td></tr>`);
run.anomalies.forEach(a => htmlEvents += `<tr><td>${a.timeS !== undefined ? a.timeS.toFixed(3) : ''}</td><td>anomaly: ${a.kind}</td><td class="bad">${a.detail}</td></tr>`);
events.innerHTML = htmlEvents + '</table>';

const diag = document.getElementById("diagnostics");
if (case_) {
  let out = '<table><tr><th>origin</th><th>finding</th><th>detail</th></tr>';
  case_.findings.forEach(f => out += `<tr><td class="${f.origin}">${f.origin}</td><td>${f.title}</td><td>${f.detail}</td></tr>`);
  out += '</table><h2>Hypotheses</h2><ul>';
  case_.hypotheses.forEach(h => out += `<li><b>[${h.layer}] ${h.title}</b> (${h.likelihood})<br>support: ${h.support.join('; ')}<br>checks: ${h.suggestedChecks.join('; ')}</li>`);
  diag.innerHTML = out + '</ul>';
} else {
  diag.textContent = 'no diagnostic case attached';
}
</script>
</body>
</html>
"""
