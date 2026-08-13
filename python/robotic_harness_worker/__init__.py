"""Robotic Harness worker package.

A dependency-light Python sidecar for the Robotic Harness DeepSeek Harness
plugin suite. Every command is a one-shot JSON-in / JSON-out process so the
TypeScript bundle can invoke it over stdio without a persistent server.

Commands are dispatched by :mod:`robotic_harness_worker.cli`.
"""

__version__ = "0.1.0"

# Version of the JSON protocol spoken between the bundle and this worker.
PROTOCOL_VERSION = 1

# Public capabilities provided by this worker package. The bundle mirrors this
# list into the agent-facing capability registry.
WORKER_CAPABILITIES = [
    {
        "id": "asset.inspect",
        "kind": "asset",
        "provider": "robotic-harness-worker",
        "input": {"modelPath": "string", "format": "string?"},
        "output": "asset inspection report",
        "risk": "R0-readonly",
        "description": "Inspect URDF/MJCF robot assets: links, joints, inertials, collisions and issues.",
    },
    {
        "id": "asset.convert_urdf_mjcf",
        "kind": "asset",
        "provider": "robotic-harness-worker",
        "input": {"urdfPath": "string", "outPath": "string"},
        "output": "MJCF asset file",
        "risk": "R1-derive",
        "description": "Convert a URDF to MJCF through the MuJoCo compiler.",
    },
    {
        "id": "sim.mujoco_pick_place",
        "kind": "simulation",
        "provider": "robotic-harness-worker",
        "input": {"scenario": "string", "fault": "object?", "seed": "integer?"},
        "output": "run summary + telemetry + artifacts",
        "risk": "R2-simulation",
        "description": "Run the MuJoCo pick-place scenario with optional fault injection.",
    },
    {
        "id": "vision.color_segmentation",
        "kind": "perception",
        "provider": "robotic-harness-worker",
        "input": {"image": "artifact", "color": "string"},
        "output": "object centroid in image",
        "risk": "R1-derive",
        "description": "Classic color segmentation for objects with clear color contrast.",
    },
    {
        "id": "vision.saliency_segmentation",
        "kind": "perception",
        "provider": "robotic-harness-worker",
        "input": {"image": "artifact"},
        "output": "object centroid in image",
        "risk": "R1-derive",
        "description": "Open-vocabulary-ish saliency/edge segmentation fallback.",
    },
    {
        "id": "policy.scripted_pick_place",
        "kind": "policy",
        "provider": "robotic-harness-worker",
        "input": {"objectPose": "object", "targetPose": "object"},
        "output": "joint trajectory",
        "risk": "R2-simulation",
        "description": "Scripted analytic-IK pick-place policy for the demo arm.",
    },
    {
        "id": "diagnostics.rule_engine",
        "kind": "diagnostics",
        "provider": "robotic-harness-worker",
        "input": {"run": "run.json"},
        "output": "facts, rule findings and candidate root causes",
        "risk": "R0-readonly",
        "description": "Deterministic evidence-based diagnostics over a run's telemetry.",
    },
    {
        "id": "data.quality_audit",
        "kind": "data",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "format": "string?"},
        "output": "quality audit report",
        "risk": "R0-readonly",
        "description": "Missing/NaN/ordering/rate/duplicate audit for CSV/JSONL timeseries.",
    },
]
