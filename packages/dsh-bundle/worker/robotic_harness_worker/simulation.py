"""MuJoCo pick-place simulation for the Robotic Harness demo.

The scenario is a planar (XZ) 3-DOF arm with a suction gripper picking a red
box from a table and placing it into a target zone. Everything is built from
primitives (no external mesh assets), the policy is scripted analytic IK, and
perception is classic color segmentation with a saliency fallback.

Design notes (read before changing):

- Determinism: all randomness flows through ``random.Random(seed)``; physics
  uses MuJoCo's default deterministic integrator.
- Suction grasp is kinematic: while attached, the object's qpos is overridden
  each step to follow the cup. This is a deliberate demo simplification,
  recorded in ``run.config.simNotes`` and in the report.
- Perception can run on a real offscreen render when the renderer is
  available, otherwise it is simulated from ground truth plus noise. The
  record structure is identical and the renderer mode is stored in telemetry.
"""

from __future__ import annotations

import json
import math
import os
import random
import time as time_mod
from typing import Any, Optional

import numpy as np

from .core import Anomaly, PhaseEvent, Run, RunStore, new_id, snapshot_environment

try:
    import mujoco  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    mujoco = None  # type: ignore[assignment]

try:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415
except Exception:  # pragma: no cover - environment dependent
    plt = None  # type: ignore[assignment]

from . import vision  # noqa: PLC0415  (import after numpy/cv2 availability checks)

# ---------------------------------------------------------------------------
# scenario
# ---------------------------------------------------------------------------

BUILTIN_SCENARIO_NAME = "mujoco_pick_place"

SCENARIO_PICK_PLACE: dict[str, Any] = {
    "name": "mujoco_pick_place",
    "description": "Planar 3-DOF arm with suction gripper: pick a red box from the table and place it in the target zone.",
    "arm": {
        "shoulderXyz": [0.0, 0.0, 0.45],
        "linkLengths": [0.22, 0.19],
        "cupReach": 0.083,
        "elbow": "down",
        "jointRanges": {"shoulder": [-3.0, 3.0], "elbow": [-2.9, 2.9], "wrist": [-3.1416, 3.1416]},
    },
    "table": {"topZ": 0.17, "halfExtents": [0.40, 0.30, 0.015]},
    "object": {"size": 0.02, "initXyz": [0.30, 0.0, 0.19], "color": "red", "mass": 0.1},
    "targetZone": {"center": [-0.16, 0.0, 0.17], "radius": 0.05},
    "camera": {"pos": [0.45, -1.2, 0.68], "fovy": 50.0, "width": 640, "height": 480},
    "sim": {"dt": 0.01, "maxSteps": 3600, "telemetryEvery": 10, "servoKp": 200.0, "servoKv": 25.0},
    "grasp": {"attachRadius": 0.032, "faceGap": 0.006},
}

DEFAULT_FAULT: dict[str, Any] = {
    "perception_offset_px": [0.0, 0.0],
    "gripper_slip": False,
    "tf_offset": [0.0, 0.0],
    "sensor_noise": 0.0,
    "model_timeout_s": 0.0,
    "occlusion": False,
}


def validate_scenario(config: dict[str, Any]) -> dict[str, Any]:
    """Validate a scenario config; returns ``{ok, issues, resolved}``."""
    issues: list[dict[str, Any]] = []

    def check(condition: bool, code: str, message: str) -> None:
        if not condition:
            issues.append({"severity": "error", "code": code, "message": message})

    resolved = json.loads(json.dumps(SCENARIO_PICK_PLACE))
    for key, value in config.items():
        if key not in resolved:
            issues.append({"severity": "warning", "code": "scenario.unknown_key", "message": f"unknown key {key!r}"})
    if "arm" in config:
        resolved["arm"].update(config["arm"])
    if "object" in config:
        resolved["object"].update(config["object"])
    if "targetZone" in config:
        resolved["targetZone"].update(config["targetZone"])
    if "sim" in config:
        resolved["sim"].update(config["sim"])

    arm = resolved["arm"]
    lengths = arm["linkLengths"]
    check(len(lengths) == 2, "scenario.arm_links", "arm.linkLengths must have exactly 2 positive values")
    for value in lengths:
        check(isinstance(value, (int, float)) and value > 0, "scenario.arm_links", "link lengths must be positive")
    shoulder = arm["shoulderXyz"]
    check(len(shoulder) == 3, "scenario.shoulder", "arm.shoulderXyz must be [x, y, z]")
    obj = resolved["object"]
    check(obj["size"] > 0, "scenario.object_size", "object.size must be positive")
    target = resolved["targetZone"]
    check(target["radius"] > 0, "scenario.target_radius", "targetZone.radius must be positive")

    shoulder_z = shoulder[2]
    reach = sum(lengths)
    min_reach = abs(lengths[0] - lengths[1])
    obj_x, obj_z = obj["initXyz"][0], obj["initXyz"][2]
    d_obj = math.hypot(obj_x - shoulder[0], obj_z - shoulder_z)
    check(d_obj <= reach - 0.012, "scenario.object_unreachable", f"object at distance {d_obj:.3f} m exceeds reach {reach:.3f} m")
    check(d_obj >= min_reach + 0.012, "scenario.object_too_close", f"object at distance {d_obj:.3f} m below min reach {min_reach:.3f} m")
    t_x, t_z = target["center"][0], target["center"][2]
    d_target = math.hypot(t_x - shoulder[0], t_z - shoulder_z)
    check(d_target <= reach - 0.012, "scenario.target_unreachable", f"target zone at distance {d_target:.3f} m exceeds reach")
    check(d_target >= min_reach + 0.012, "scenario.target_too_close", "target zone too close to the shoulder")

    return {"ok": not any(i["severity"] == "error" for i in issues), "issues": issues, "resolved": resolved}


def load_scenario(name_or_path: str) -> dict[str, Any]:
    """Load a builtin scenario by name or a JSON scenario file by path."""
    if name_or_path == BUILTIN_SCENARIO_NAME:
        return json.loads(json.dumps(SCENARIO_PICK_PLACE))
    if os.path.exists(name_or_path):
        with open(name_or_path, encoding="utf-8") as handle:
            data = json.load(handle)
        validated = validate_scenario(data)
        if not validated["ok"]:
            raise ValueError(f"scenario {name_or_path!r} is invalid: {validated['issues']}")
        return validated["resolved"]
    raise ValueError(f"unknown scenario {name_or_path!r}; use {BUILTIN_SCENARIO_NAME!r} or a path to a JSON scenario")


# ---------------------------------------------------------------------------
# planar arm math
# ---------------------------------------------------------------------------

class PlanarArm:
    """2R planar arm in the XZ plane plus a wrist rotation (3rd joint)."""

    def __init__(self, link_lengths: list[float], shoulder: list[float], cup_reach: float, joint_ranges: dict[str, list[float]] | None = None) -> None:
        self.l1, self.l2 = link_lengths[0], link_lengths[1]
        self.shoulder = np.array(shoulder, dtype=float)
        self.cup_reach = cup_reach
        self.ranges = joint_ranges or {
            "shoulder": [-3.0, 3.0],
            "elbow": [-2.9, 2.9],
            "wrist": [-math.pi, math.pi],
        }

    def reach(self) -> float:
        return self.l1 + self.l2

    def min_reach(self) -> float:
        return abs(self.l1 - self.l2)

    def ik_solutions(self, x: float, z: float, phi: float) -> list[list[float]]:
        """All IK solutions within joint limits; q2 is wrapped into [-pi, pi]."""
        dx, dz = x - self.shoulder[0], z - self.shoulder[2]
        distance = math.hypot(dx, dz)
        if distance > self.l1 + self.l2 - 1e-9 or distance < self.min_reach() + 1e-9:
            return []
        cos_q1 = (distance * distance - self.l1 * self.l1 - self.l2 * self.l2) / (2 * self.l1 * self.l2)
        q1 = math.acos(max(-1.0, min(1.0, cos_q1)))
        base = math.atan2(dx, dz)
        solutions: list[list[float]] = []
        for sign in (-1.0, 1.0):
            q1_candidate = sign * q1
            q0 = base - math.atan2(self.l2 * math.sin(q1_candidate), self.l1 + self.l2 * math.cos(q1_candidate))
            q2 = phi - q0 - q1_candidate
            q2 = (q2 + math.pi) % (2 * math.pi) - math.pi  # wrap into [-pi, pi]
            if (
                self.ranges["shoulder"][0] <= q0 <= self.ranges["shoulder"][1]
                and self.ranges["elbow"][0] <= q1_candidate <= self.ranges["elbow"][1]
            ):
                solutions.append([q0, q1_candidate, q2])
        return solutions

    def ik(self, x: float, z: float, phi: float, elbow: str = "down") -> Optional[list[float]]:
        """Preferred-elbow IK for simple callers; None when unreachable."""
        solutions = self.ik_solutions(x, z, phi)
        if not solutions:
            return None
        preferred = solutions[0] if elbow == "up" else solutions[-1]
        # solutions are ordered [-q1, +q1]; down = negative q1 = last entry
        return preferred

    def fk(self, q: list[float]) -> tuple[float, float, float]:
        """Forward kinematics: wrist (x, z) and cup angle phi."""
        q0, q1, q2 = q
        x = self.shoulder[0] + self.l1 * math.sin(q0) + self.l2 * math.sin(q0 + q1)
        z = self.shoulder[2] + self.l1 * math.cos(q0) + self.l2 * math.cos(q0 + q1)
        return x, z, q0 + q1 + q2

    def cup_tip(self, q: list[float]) -> tuple[float, float]:
        x, z, phi = self.fk(q)
        return x + self.cup_reach * math.sin(phi), z + self.cup_reach * math.cos(phi)


# ---------------------------------------------------------------------------
# MuJoCo environment
# ---------------------------------------------------------------------------

def _mujoco_xml(scenario: dict[str, Any]) -> str:
    arm = scenario["arm"]
    table = scenario["table"]
    obj = scenario["object"]
    sim = scenario["sim"]
    sx, sy, sz = arm["shoulderXyz"]
    l1, l2 = arm["linkLengths"]
    ranges = arm["jointRanges"]
    table_top = table["topZ"]
    t_hx, t_hy, t_hz = table["halfExtents"]
    table_center_z = table_top - t_hz
    o = obj["size"]
    mass1, mass2 = 1.2, 0.8
    # box inertia about body axes: m*(dy^2+dz^2)/12 etc. Link bodies are
    # boxes of full size (0.06, 0.06, 2*length) centered at (length, 0, 0).
    def box_inertia(mass: float, dx: float, dy: float, dz: float) -> str:
        ixx = mass * (dy * dy + dz * dz) / 12.0
        iyy = mass * (dx * dx + dz * dz) / 12.0
        izz = mass * (dx * dx + dy * dy) / 12.0
        return f'diaginertia="{ixx:.6f} {iyy:.6f} {izz:.6f}"'

    def joint_range(name: str, default: list[float]) -> str:
        low, high = ranges.get(name, default)
        return f'range="{low} {high}"'

    shoulder_range = joint_range("shoulder", [-3.0, 3.0])
    elbow_range = joint_range("elbow", [-2.9, 2.9])
    wrist_range = joint_range("wrist", [-3.1416, 3.1416])

    return f"""<mujoco model="rh_pick_place">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option gravity="0 0 -9.81" timestep="{sim['dt']}" iterations="8" tolerance="1e-8" integrator="implicitfast"/>
  <size njmax="40" nconmax="40"/>
  <visual>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.6 0.6 0.6"/>
    <global offwidth="{scenario['camera']['width']}" offheight="{scenario['camera']['height']}"/>
  </visual>
  <asset>
    <texture name="table_tex" type="2d" builtin="checker" width="256" height="256" rgb1="0.25 0.25 0.3" rgb2="0.35 0.35 0.42"/>
    <material name="table_mat" texture="table_tex" texrepeat="4 3"/>
    <material name="arm_mat" rgba="0.15 0.45 0.85 1"/>
    <material name="cup_mat" rgba="0.9 0.7 0.1 1"/>
    <material name="obj_mat" rgba="0.85 0.15 0.12 1"/>
    <material name="zone_mat" rgba="0.2 0.8 0.3 0.25"/>
  </asset>
  <worldbody>
    <light pos="0.6 -1.0 1.4" dir="-0.4 0.6 -1" diffuse="0.8 0.8 0.8"/>
    <camera name="cam0" pos="{scenario['camera']['pos'][0]} {scenario['camera']['pos'][1]} {scenario['camera']['pos'][2]}" xyaxes="1 0 0 0 0 1" fovy="{scenario['camera']['fovy']}"/>
    <geom name="ground" type="plane" size="2 2 0.02" pos="0 0 -0.01" rgba="0.5 0.5 0.5 1"/>
    <body name="table" pos="0 0 {table_center_z}">
      <geom name="table_geom" type="box" size="{t_hx} {t_hy} {t_hz}" material="table_mat" friction="0.9 0.01 0.001"/>
    </body>
    <body name="pedestal" pos="{sx} {sy} {sz - 0.15}">
      <geom name="pedestal_geom" type="cylinder" size="0.035 0.10" pos="0 0 0" material="arm_mat"/>
    </body>
    <body name="link1" pos="{sx} {sy} {sz}">
      <joint name="shoulder" type="hinge" axis="0 1 0" {shoulder_range} damping="0.6" armature="0.02"/>
      <geom name="link1_geom" type="box" size="0.03 0.03 0.10" pos="0 0 0.13" material="arm_mat"/>
      <inertial pos="0 0 0.13" mass="{mass1}" {box_inertia(mass1, 0.06, 0.06, 0.20)}/>
      <body name="link2" pos="0 0 {l1}">
        <joint name="elbow" type="hinge" axis="0 1 0" {elbow_range} damping="0.5" armature="0.015"/>
        <geom name="link2_geom" type="box" size="0.025 0.025 {l2 / 2}" pos="0 0 {l2 / 2}" material="arm_mat"/>
        <inertial pos="0 0 {l2 / 2}" mass="{mass2}" {box_inertia(mass2, 0.05, 0.05, 2 * l2)}/>
        <body name="wrist" pos="0 0 {l2}">
          <joint name="wrist_joint" type="hinge" axis="0 1 0" {wrist_range} damping="0.3" armature="0.005"/>
          <geom name="cup_geom" type="box" size="0.018 0.018 0.055" pos="0 0 0.028" material="cup_mat"/>
          <inertial pos="0 0 0.028" mass="0.2" diaginertia="0.0002 0.0002 0.00005"/>
        </body>
      </body>
    </body>
    <body name="object" pos="{obj['initXyz'][0]} {obj['initXyz'][1]} {obj['initXyz'][2]}">
      <freejoint name="object_free"/>
      <geom name="object_geom" type="box" size="{o} {o} {o}" material="obj_mat" friction="0.9 0.005 0.0005"/>
      <inertial pos="0 0 0" mass="{obj['mass']}" diaginertia="{obj['mass'] * 2 * (2 * o) ** 2 / 12:.7f} {obj['mass'] * 2 * (2 * o) ** 2 / 12:.7f} {obj['mass'] * 2 * (2 * o) ** 2 / 12:.7f}"/>
    </body>
    <body name="zone" pos="{scenario['targetZone']['center'][0]} {scenario['targetZone']['center'][1]} {scenario['targetZone']['center'][2] + 0.002}">
      <geom name="zone_geom" type="cylinder" size="{scenario['targetZone']['radius']} 0.002" material="zone_mat" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="a_shoulder" joint="shoulder" kp="{sim['servoKp']}" kv="{sim['servoKv']}" ctrlrange="{ranges['shoulder'][0]} {ranges['shoulder'][1]}"/>
    <position name="a_elbow" joint="elbow" kp="{sim['servoKp']}" kv="{sim['servoKv']}" ctrlrange="{ranges['elbow'][0]} {ranges['elbow'][1]}"/>
    <position name="a_wrist_joint" joint="wrist_joint" kp="{sim['servoKp']}" kv="{sim['servoKv']}" ctrlrange="{ranges['wrist'][0]} {ranges['wrist'][1]}"/>
  </actuator>
</mujoco>"""


class CameraModel:
    """Pinhole camera model matching MuJoCo's convention (look along -Z)."""

    def __init__(self, scenario: dict[str, Any]) -> None:
        cam = scenario["camera"]
        self.pos = np.array(cam["pos"], dtype=float)
        self.fovy = float(cam["fovy"])
        self.width = int(cam["width"])
        self.height = int(cam["height"])
        self.f = (self.height / 2.0) / math.tan(math.radians(self.fovy) / 2.0)
        # xyaxes="1 0 0 0 0 1": camera x=(1,0,0), y=(0,0,1), z = x cross y = (0,-1,0)
        self.x_axis = np.array([1.0, 0.0, 0.0])
        self.y_axis = np.array([0.0, 0.0, 1.0])
        self.z_axis = np.cross(self.x_axis, self.y_axis)

    def px_from_world(self, world: np.ndarray) -> tuple[float, float]:
        rel = world - self.pos
        x_cam = float(np.dot(rel, self.x_axis))
        y_cam = float(np.dot(rel, self.y_axis))
        z_cam = float(np.dot(rel, self.z_axis))
        if z_cam >= -1e-9:
            raise ValueError(f"point {world} is behind the camera")
        # MuJoCo camera looks along -Z; image x right, y down.
        px = self.width / 2.0 - self.f * x_cam / (-z_cam)
        py = self.height / 2.0 + self.f * y_cam / (-z_cam)
        return px, py

    def world_from_px(self, px: float, py: float, z_plane: float) -> np.ndarray:
        """Ray-plane intersection with the horizontal plane z=z_plane."""
        fx = -(px - self.width / 2.0) / self.f
        fy = (py - self.height / 2.0) / self.f
        direction = self.x_axis * fx + self.y_axis * fy - self.z_axis
        direction = direction / np.linalg.norm(direction)
        if abs(direction[2]) < 1e-9:
            raise ValueError("ray is parallel to the requested plane")
        t = (z_plane - self.pos[2]) / direction[2]
        return self.pos + t * direction


class PickPlaceEnv:
    """MuJoCo model + data wrapper for the pick-place scenario."""

    def __init__(self, scenario: dict[str, Any]) -> None:
        if mujoco is None:
            raise RuntimeError("mujoco is not importable in this Python environment")
        self.scenario = scenario
        self.xml = _mujoco_xml(scenario)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.arm = PlanarArm(
            scenario["arm"]["linkLengths"],
            scenario["arm"]["shoulderXyz"],
            scenario["arm"]["cupReach"],
            scenario["arm"].get("jointRanges"),
        )
        self.camera = CameraModel(scenario)
        self.joint_names = ["shoulder", "elbow", "wrist_joint"]
        self.qposadr = {name: int(self.model.jnt(name).qposadr.item()) for name in self.joint_names}
        self.ctrladr = {
            "shoulder": int(self.model.actuator("a_shoulder").id),
            "elbow": int(self.model.actuator("a_elbow").id),
            "wrist_joint": int(self.model.actuator("a_wrist_joint").id),
        }
        self.obj_qposadr = int(self.model.jnt("object_free").qposadr.item())
        self.obj_qveladr = int(self.model.jnt("object_free").dofadr.item())
        self.renderer = None

    def reset(self, q: list[float] | None = None) -> None:
        mujoco.mj_resetData(self.model, self.data)
        if q is not None:
            for name, value in zip(self.joint_names, q):
                self.data.qpos[self.qposadr[name]] = value
        self.data.qpos[self.obj_qposadr + 0 : self.obj_qposadr + 3] = self.scenario["object"]["initXyz"]
        self.data.qpos[self.obj_qposadr + 3 : self.obj_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)

    def qpos(self) -> list[float]:
        return [float(self.data.qpos[self.qposadr[name]]) for name in self.joint_names]

    def set_ctrl(self, q: list[float]) -> None:
        for name, value in zip(self.joint_names, q):
            self.data.ctrl[self.ctrladr[name]] = value

    def step(self, substeps: int = 5) -> None:
        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)

    def cup_tip_pos(self) -> np.ndarray:
        xmat = self.data.body("wrist").xmat.reshape(3, 3)
        return self.data.body("wrist").xpos + xmat @ np.array([0.0, 0.0, self.arm.cup_reach])

    def object_pos(self) -> np.ndarray:
        return self.data.qpos[self.obj_qposadr : self.obj_qposadr + 3].copy()

    def attach_object(self) -> None:
        """Kinematic suction attach: object follows the cup tip.

        The object center is held one object half-size beyond the tip, so the
        cup face stays in contact with the object face.
        """
        tip = self.cup_tip_pos()
        axis = self.data.body("wrist").xmat.reshape(3, 3) @ np.array([0.0, 0.0, 1.0])
        target = tip + axis * self.scenario["object"]["size"]
        self.data.qpos[self.obj_qposadr : self.obj_qposadr + 3] = target
        self.data.qpos[self.obj_qposadr + 3 : self.obj_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[self.obj_qveladr : self.obj_qveladr + 6] = 0.0

    def render_rgb(self) -> Optional[np.ndarray]:
        """Offscreen render of cam0; returns None when the renderer is unavailable."""
        try:
            if self.renderer is None:
                self.renderer = mujoco.Renderer(self.model, self.camera.height, self.camera.width)
            self.renderer.update_scene(self.data, camera="cam0")
            return self.renderer.render().copy()
        except Exception:  # pragma: no cover - platform dependent
            return None


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

PHASES = ["home", "approach", "grasp", "lift", "carry", "place", "retract"]


class PickPlaceRunner:
    """Runs the scripted pick-place policy with optional fault injection."""

    def __init__(self, scenario: dict[str, Any], fault: dict[str, Any], seed: int) -> None:
        self.scenario = scenario
        merged_fault = json.loads(json.dumps(DEFAULT_FAULT))
        merged_fault.update(fault or {})
        self.fault = merged_fault
        self.seed = seed
        self.rng = random.Random(seed)
        self.env = PickPlaceEnv(scenario)
        self.arm = self.env.arm
        self.telemetry: list[dict[str, Any]] = []
        self.anomalies: list[Anomaly] = []
        self.phases: list[PhaseEvent] = []
        self.sim = scenario["sim"]
        self.suction = False
        self.attached = False
        self.step_count = 0

    # -- helpers ------------------------------------------------------------

    def _add_anomaly(self, kind: str, detail: str, **evidence: Any) -> None:
        self.anomalies.append(Anomaly(kind=kind, time_s=round(self.env.data.time, 3), detail=detail, evidence=evidence))

    def _phase(self, name: str, outcome: str = "ok", detail: str = "") -> None:
        self.phases.append(
            PhaseEvent(phase=name, time_s=round(self.env.data.time, 3), outcome=outcome, detail=detail)
        )

    def _telemetry(self, phase: str, q_target: list[float], perception: Any, noisy: bool) -> None:
        q = self.env.qpos()
        measured = q
        if noisy and self.fault.get("sensor_noise", 0.0) > 0:
            measured = [value + self.rng.gauss(0.0, self.fault["sensor_noise"]) for value in q]
        tracking = [abs(target - actual) for target, actual in zip(q_target, measured)]
        self.telemetry.append(
            {
                "t": round(self.env.data.time, 4),
                "phase": phase,
                "q": [round(v, 5) for v in q],
                "qMeasured": [round(v, 5) for v in measured],
                "qTarget": [round(v, 5) for v in q_target],
                "trackingError": [round(v, 5) for v in tracking],
                "cupPos": [round(v, 4) for v in self.env.cup_tip_pos().tolist()],
                "objPos": [round(v, 4) for v in self.env.object_pos().tolist()],
                "attached": self.attached,
                "suction": self.suction,
                "perception": perception,
            }
        )

    def _track_to(self, phase: str, targets: list[tuple[float, float, float]], perception: Any, hold_steps: int = 15) -> bool:
        """Move the wrist through Cartesian waypoints; returns False on timeout.

        IK is solved continuously: at every waypoint we pick the solution
        closest to the previous one (in joint space) so the arm never flips
        arbitrarily mid-path. The final waypoint is held for ``hold_steps``
        control steps so servo transients settle before the next phase.
        """
        current_q = self.env.qpos()
        waypoints: list[list[float]] = []
        previous = current_q
        for x, z, phi in targets:
            solutions = self.arm.ik_solutions(x, z, phi)
            if not solutions:
                self._add_anomaly("ik_no_solution", f"waypoint ({x:.3f}, {z:.3f}, {phi:.2f}) unreachable")
                return False
            chosen = min(solutions, key=lambda s: sum((a - b) ** 2 for a, b in zip(s, previous)))
            waypoints.append(chosen)
            previous = chosen
        full: list[list[float]] = [current_q]
        segments = [current_q] + waypoints
        for index in range(1, len(segments)):
            start, end = segments[index - 1], segments[index]
            steps = 40
            for step in range(1, steps + 1):
                t = step / steps
                full.append([a + (b - a) * t for a, b in zip(start, end)])
        for target in full:
            if self.env.data.time >= self.sim["maxSteps"] * self.sim["dt"]:
                self._add_anomaly("step_timeout", f"exceeded max steps during phase {phase}")
                return False
            self.env.set_ctrl(target)
            self.env.step()
            self.step_count += 1
            if self.attached:
                self.env.attach_object()
            if self.step_count % self.sim["telemetryEvery"] == 0:
                self._telemetry(phase, target, perception, noisy=True)
        for _ in range(hold_steps):
            if self.env.data.time >= self.sim["maxSteps"] * self.sim["dt"]:
                self._add_anomaly("step_timeout", f"exceeded max steps during phase {phase}")
                return False
            self.env.step()
            self.step_count += 1
            if self.attached:
                self.env.attach_object()
            if self.step_count % self.sim["telemetryEvery"] == 0:
                self._telemetry(phase, waypoints[-1], perception, noisy=True)
        return True

    # -- perception -----------------------------------------------------------

    def _perceive(self) -> dict[str, Any]:
        """Perceive the object once (at home pose) and estimate its world position."""
        est_true = np.array(self.scenario["object"]["initXyz"], dtype=float)
        tf_offset = self.fault.get("tf_offset", [0.0, 0.0])
        image = self.env.render_rgb()
        renderer_mode = "offscreen" if image is not None else "simulated"
        if image is not None:
            decision = vision.route_perception(
                image,
                scene=self.scenario,
                rng=self.rng,
                perception="auto",
                color=self.scenario["object"]["color"],
                fault=self.fault,
            )
            perception_record = decision
            if decision.get("ok"):
                px, py = decision["result"]["centroidPx"]
                estimate = self.env.camera.world_from_px(px, py, est_true[2])
                estimate[1] = est_true[1]  # y is fixed by the plane assumption
            else:
                estimate = est_true.copy()
        else:
            # Simulated perception: ground truth pixel + noise + offset fault.
            try:
                px, py = self.env.camera.px_from_world(est_true)
            except ValueError:
                px, py = self.env.camera.width / 2, self.env.camera.height / 2
            px += self.fault.get("perception_offset_px", [0.0, 0.0])[0] + self.rng.uniform(-2, 2)
            py += self.fault.get("perception_offset_px", [0.0, 0.0])[1] + self.rng.uniform(-2, 2)
            estimate = self.env.camera.world_from_px(px, py, est_true[2])
            estimate[1] = est_true[1]
            perception_record = {
                "ok": True,
                "route": "color",
                "reason": "simulated perception (renderer unavailable)",
                "result": {"centroidPx": [round(px, 2), round(py, 2)], "confidence": 0.9},
            }

        estimate[0] += tf_offset[0]
        estimate[2] += tf_offset[1]
        return {
            "renderer": renderer_mode,
            "record": perception_record,
            "estimate": [round(float(v), 4) for v in estimate],
            "true": [round(float(v), 4) for v in est_true],
        }

    # -- policy ---------------------------------------------------------------

    def run(self) -> Run:
        env = self.env
        scenario = self.scenario
        arm = self.arm
        sim = self.sim
        run = Run(
            id=new_id("run"),
            project_id="robotic-harness-demo",
            scenario=scenario["name"],
            state="running",
            config={
                "scenario": scenario,
                "fault": self.fault,
                "seed": self.seed,
                "simNotes": [
                    "suction grasp is kinematic (object qpos follows the cup while attached)",
                    "perception is real offscreen rendering when the renderer is available, otherwise simulated ground truth + noise",
                ],
            },
        )

        try:
            env.reset([0.35, -1.1, -0.2])
            self.suction = False
            self.attached = False
            self._phase("home")
            self._telemetry("home", env.qpos(), None, noisy=False)

            # --- perceive ----------------------------------------------------
            perception = self._perceive()
            if not perception["record"].get("ok"):
                self._add_anomaly("perception_failed", perception["record"].get("reason", "all routes failed"))
            elif perception["record"].get("needsRecheck"):
                self._add_anomaly(
                    "perception_low_confidence",
                    "perception confidence below threshold; treat estimate as uncertain",
                    estimate=perception["estimate"],
                    true=perception["true"],
                )
            obj_est = np.array(perception["estimate"], dtype=float)

            # --- grasp axis ----------------------------------------------------
            # Vertical suction approach: the cup descends onto the object's top
            # face, so the table carries any residual contact force (a side
            # approach would shove the object across the table).
            object_size = scenario["object"]["size"]
            face_gap = env.scenario["grasp"]["faceGap"]
            axis = np.array([0.0, 0.0, -1.0])
            phi_grasp = math.pi
            tip = obj_est + np.array([0.0, 0.0, object_size + face_gap])  # just above the top face
            wrist_grasp = tip - axis * arm.cup_reach
            approach = wrist_grasp - axis * 0.12
            lift_high = wrist_grasp + np.array([0.0, 0.0, 0.22])
            target_center = np.array(scenario["targetZone"]["center"], dtype=float)
            target_wrist = np.array(
                [target_center[0], 0.0, target_center[2] + object_size + face_gap + arm.cup_reach]
            )
            target_high = target_wrist + np.array([0.0, 0.0, 0.22])

            # --- approach -------------------------------------------------------
            self._phase("approach")
            ok = self._track_to("approach", [(approach[0], approach[2], phi_grasp)], perception)
            if not ok:
                raise SimulationAborted("approach phase failed")
            self._phase("grasp")
            ok = self._track_to("grasp", [(wrist_grasp[0], wrist_grasp[2], phi_grasp)], perception)
            if not ok:
                raise SimulationAborted("grasp phase failed")

            # --- suction on ------------------------------------------------------
            self.suction = True
            tip_now = env.cup_tip_pos()
            obj_now = env.object_pos()
            gap = float(np.linalg.norm(tip_now - obj_now))
            self.attached = gap <= scenario["grasp"]["attachRadius"]
            if self.attached:
                env.attach_object()
                self._add_anomaly(
                    "suction_engaged",
                    "suction engaged at grasp pose",
                    objEst=obj_est.tolist(),
                    objTrue=perception["true"],
                    gapM=round(gap, 4),
                )
            else:
                self._add_anomaly(
                    "grasp_missed",
                    f"cup tip {gap * 1000:.1f} mm from the object; suction cannot engage (attach radius {scenario['grasp']['attachRadius'] * 1000:.0f} mm)",
                    objEst=obj_est.tolist(),
                    objTrue=perception["true"],
                    gapM=round(gap, 4),
                )
                self._phase("grasp", outcome="failed", detail="grasp missed the object")

            # --- lift -------------------------------------------------------------
            self._phase("lift")
            ok = self._track_to("lift", [(lift_high[0], lift_high[2], phi_grasp)], perception)
            if not ok:
                raise SimulationAborted("lift phase failed")

            # --- slip fault ---------------------------------------------------------
            if self.fault.get("gripper_slip"):
                if self.attached:
                    self.suction = False
                    self.attached = False
                    self._add_anomaly(
                        "gripper_slip",
                        "gripper slip fault injected: suction lost while carrying the object",
                        objPos=[round(float(v), 4) for v in env.object_pos()],
                    )
                    self._phase("lift", outcome="failed", detail="gripper slip injected")
                    # let the object fall back to the table
                    for _ in range(120):
                        env.step()
                else:
                    self._add_anomaly(
                        "gripper_slip_skipped",
                        "gripper slip fault configured but no object was attached; slip not applicable",
                    )

            # --- carry ---------------------------------------------------------------
            self._phase("carry")
            ok = self._track_to("carry", [(target_high[0], target_high[2], math.pi), (target_wrist[0], target_wrist[2], math.pi)], perception)
            if not ok:
                raise SimulationAborted("carry phase failed")

            # --- place ----------------------------------------------------------------
            self._phase("place")
            ok = self._track_to("place", [(target_wrist[0], target_wrist[2], math.pi)], perception)
            if not ok:
                raise SimulationAborted("place phase failed")

            if not self.attached:
                self._add_anomaly(
                    "place_without_object", "arrived at the target zone without the object (slip or grasp failure)"
                )
            else:
                self.suction = False
                self.attached = False
                self._add_anomaly("suction_released", "suction released at the target zone")
                for _ in range(60):
                    env.step()

            # --- retract --------------------------------------------------------------
            self._phase("retract")
            ok = self._track_to("retract", [(target_high[0], target_high[2], math.pi), (0.05, 0.62, math.pi)], perception)
            if not ok:
                raise SimulationAborted("retract phase failed")

            run.state = "completed"
            self._finalize(run, perception)
            return run
        except SimulationAborted as error:
            run.state = "failed"
            self._add_anomaly("run_aborted", str(error))
            self._finalize(run, {"record": {}, "estimate": [], "true": []})
            return run
        except Exception as error:  # noqa: BLE001 - report any failure
            run.state = "failed"
            self._add_anomaly("run_error", f"{type(error).__name__}: {error}")
            self._finalize(run, {"record": {}, "estimate": [], "true": []})
            return run

    def _finalize(self, run: Run, perception: dict[str, Any]) -> None:
        """Compute metrics and success criteria."""
        env = self.env
        scenario = self.scenario
        obj_final = env.object_pos()
        target = np.array(scenario["targetZone"]["center"], dtype=float)
        target[2] += 0.0
        in_zone = math.hypot(obj_final[0] - target[0], obj_final[2] - target[2]) <= scenario["targetZone"]["radius"] + 0.01
        # object must have been grasped and the run must not have been aborted
        grasped = any(a.kind == "suction_engaged" for a in self.anomalies)
        slipped = any(a.kind == "gripper_slip" for a in self.anomalies)
        success = run.state == "completed" and in_zone and grasped and not slipped

        tracking_errors = [row["trackingError"] for row in self.telemetry if row["trackingError"]]
        rms = (
            math.sqrt(sum(sum(e * e for e in row) for row in tracking_errors) / max(len(tracking_errors), 1))
            if tracking_errors
            else 0.0
        )

        run.metrics = {
            "success": bool(success),
            "steps": self.step_count,
            "durationS": round(float(env.data.time), 3),
            "trackingErrorRms": round(rms, 5),
            "inTargetZone": bool(in_zone),
            "grasped": bool(grasped),
            "slipped": bool(slipped),
            "objectFinal": [round(float(v), 4) for v in obj_final],
            "targetZone": scenario["targetZone"],
            "perceptionRoute": perception.get("record", {}).get("route"),
            "renderer": perception.get("renderer"),
            "perceptionEstimate": perception.get("estimate"),
            "perceptionTrue": perception.get("true"),
            "environment": snapshot_environment(),
        }
        run.final_result = {"success": bool(success), "summary": self._summary_text(success)}
        run.phases = self.phases
        run.anomalies = self.anomalies

    def _summary_text(self, success: bool) -> str:
        outcome = "SUCCESS" if success else "FAILURE"
        return (
            f"{outcome} | {self.scenario['name']} | seed={self.seed} | "
            f"steps={self.step_count} | anomalies={len(self.anomalies)} | "
            f"phases={len(self.phases)}"
        )


class SimulationAborted(RuntimeError):
    """Raised when the policy aborts a phase (timeout, unreachable waypoint)."""


# ---------------------------------------------------------------------------
# artifacts
# ---------------------------------------------------------------------------

def write_telemetry(run_dir: str, rows: list[dict[str, Any]]) -> str:
    os.makedirs(run_dir, exist_ok=True)
    path = os.path.join(run_dir, "telemetry.jsonl")
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
    return path


def render_charts(run: Run, telemetry: list[dict[str, Any]], out_dir: str) -> list[str]:
    """Render joint/tracking/trajectory charts; returns artifact paths."""
    if plt is None:
        return []
    written: list[str] = []
    times = [row["t"] for row in telemetry]
    try:
        # joints
        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        names = ["shoulder", "elbow", "wrist"]
        for index, (name, axis) in enumerate(zip(names, axes)):
            axis.plot(times, [row["q"][index] for row in telemetry], label="actual")
            axis.plot(times, [row["qTarget"][index] for row in telemetry], "--", label="target", alpha=0.8)
            axis.set_ylabel(f"{name} [rad]")
            axis.legend(loc="upper right", fontsize=8)
            axis.grid(alpha=0.3)
        axes[-1].set_xlabel("sim time [s]")
        fig.suptitle(f"run {run.id}: joint positions")
        fig.tight_layout()
        path = os.path.join(out_dir, "joints.png")
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)

        # tracking error
        fig, axis = plt.subplots(figsize=(9, 3.5))
        for index, name in enumerate(names):
            axis.plot(times, [row["trackingError"][index] for row in telemetry], label=name)
        axis.set_xlabel("sim time [s]")
        axis.set_ylabel("|error| [rad]")
        axis.set_title(f"run {run.id}: tracking error")
        axis.legend(fontsize=8)
        axis.grid(alpha=0.3)
        fig.tight_layout()
        path = os.path.join(out_dir, "tracking.png")
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)

        # trajectory in XZ plane
        fig, axis = plt.subplots(figsize=(7, 7))
        cups = np.array([row["cupPos"] for row in telemetry])
        objs = np.array([row["objPos"] for row in telemetry])
        axis.plot(cups[:, 0], cups[:, 2], "-", label="cup path", lw=1.2)
        axis.plot(objs[:, 0], objs[:, 2], "-", label="object path", lw=1.2)
        zone = run.metrics.get("targetZone", {})
        center = zone.get("center", [0.03, 0.0, 0.17])
        radius = zone.get("radius", 0.05)
        circle = plt.Circle((center[0], center[2]), radius, color="green", alpha=0.25, label="target zone")
        axis.add_patch(circle)
        obj_final = run.metrics.get("objectFinal", [0, 0, 0])
        axis.plot(obj_final[0], obj_final[2], "kx", ms=10, label="object final")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("z [m]")
        axis.set_title(f"run {run.id}: trajectory")
        axis.legend(fontsize=8)
        axis.grid(alpha=0.3)
        axis.set_aspect("equal")
        fig.tight_layout()
        path = os.path.join(out_dir, "trajectory.png")
        fig.savefig(path, dpi=110)
        plt.close(fig)
        written.append(path)
    except Exception:  # pragma: no cover - chart rendering must not fail the run
        pass
    return written


def render_scene_image(run_id: str, xml: str, out_dir: str) -> Optional[str]:
    """Render one scene frame with MuJoCo; None when rendering is unavailable."""
    try:
        import mujoco  # noqa: PLC0415

        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 480, 640)
        renderer.update_scene(data, camera="cam0")
        image = renderer.render()
        import PIL.Image  # noqa: PLC0415

        path = os.path.join(out_dir, "scene.png")
        PIL.Image.fromarray(image).save(path)
        return path
    except Exception:  # pragma: no cover - platform dependent
        return None


def sim_batch_benchmark(
    cells: list[dict[str, Any]],
    store: RunStore | None = None,
    out_dir: str | None = None,
) -> dict[str, Any]:
    """Run a small matrix of pick-place cells and aggregate metrics.

    Each cell: ``{label?, fault?, seed?, scenario?}``. Results are aggregated
    per label (or per cell when unlabeled) into success rate and averaged
    metrics. This is a demo-scale benchmark, not a statistical study.
    """
    results: list[dict[str, Any]] = []
    for index, cell in enumerate(cells):
        scenario_config = cell.get("scenario") or json.loads(json.dumps(SCENARIO_PICK_PLACE))
        seed = int(cell.get("seed", 42 + index))
        run, _ = run_pick_place(scenario_config, cell.get("fault", {}), seed, store=store)
        label = cell.get("label") or f"cell-{index}"
        results.append(
            {
                "label": label,
                "seed": seed,
                "runId": run.id,
                "success": bool(run.metrics.get("success")),
                "grasped": bool(run.metrics.get("grasped")),
                "slipped": bool(run.metrics.get("slipped")),
                "inTargetZone": bool(run.metrics.get("inTargetZone")),
                "trackingErrorRms": run.metrics.get("trackingErrorRms"),
                "durationS": run.metrics.get("durationS"),
                "anomalies": [a.kind for a in run.anomalies],
                "fault": cell.get("fault", {}),
            }
        )

    groups: dict[str, list[dict[str, Any]]] = {}
    for result in results:
        groups.setdefault(result["label"], []).append(result)
    summary = {}
    for label, group in groups.items():
        success_rate = sum(1 for r in group if r["success"]) / len(group)
        rms_values = [r["trackingErrorRms"] for r in group if r["trackingErrorRms"] is not None]
        summary[label] = {
            "runs": len(group),
            "successRate": round(success_rate, 3),
            "graspedRate": round(sum(1 for r in group if r["grasped"]) / len(group), 3),
            "slippedCount": sum(1 for r in group if r["slipped"]),
            "avgTrackingErrorRms": round(sum(rms_values) / len(rms_values), 5) if rms_values else None,
            "typicalAnomalies": sorted({a for r in group for a in r["anomalies"]}),
        }

    payload = {"ok": True, "cells": len(results), "results": results, "summary": summary,
               "note": "simulation-only benchmark; not a statistical study and not real-robot evidence"}
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, "benchmark.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        payload["path"] = path
    return payload


def sim_real_gap(
    sim_run_path: str,
    real_csv_path: str,
    channel_map: dict[str, str],
    time_columns: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Compare shared numeric channels between a simulated run and real data.

    ``channel_map`` maps sim telemetry channels (e.g. ``q.0``) to real CSV
    columns (e.g. ``joint0``). The comparison is distribution-level
    (mean/std/percentiles); the report explicitly refuses to draw
    real-robot safety conclusions.
    """
    from .diagnostics import load_run_data

    try:
        import csv as _csv  # noqa: PLC0415
    except ImportError:  # pragma: no cover
        raise

    run, telemetry = load_run_data(sim_run_path)
    real_rows: list[dict[str, float]] = []
    with open(real_csv_path, encoding="utf-8", newline="") as handle:
        reader = _csv.DictReader(handle)
        for row in reader:
            parsed = {}
            for column in channel_map.values():
                try:
                    parsed[column] = float(row[column])
                except (KeyError, TypeError, ValueError):
                    continue
            real_rows.append(parsed)

    def stats(values: list[float]) -> dict[str, float]:
        array = np.array(values, dtype=float)
        if array.size == 0:
            return {}
        return {
            "mean": round(float(array.mean()), 6),
            "std": round(float(array.std()), 6),
            "min": round(float(array.min()), 6),
            "p10": round(float(np.percentile(array, 10)), 6),
            "p50": round(float(np.percentile(array, 50)), 6),
            "p90": round(float(np.percentile(array, 90)), 6),
            "max": round(float(array.max()), 6),
        }

    channels: dict[str, Any] = {}
    largest_gap: dict[str, Any] | None = None
    for sim_channel, real_column in channel_map.items():
        sim_values = []
        for row in telemetry:
            value = row
            ok = True
            for part in sim_channel.split("."):
                if isinstance(value, dict) and part in value:
                    value = value[part]
                elif isinstance(value, list) and part.isdigit() and int(part) < len(value):
                    value = value[int(part)]
                else:
                    ok = False
                    break
            if ok and isinstance(value, (int, float)):
                sim_values.append(float(value))
        real_values = [row[real_column] for row in real_rows if real_column in row]
        sim_stats = stats(sim_values)
        real_stats = stats(real_values)
        if sim_stats and real_stats:
            gap = abs(sim_stats["mean"] - real_stats["mean"])
            gap_relative = gap / max(abs(real_stats["mean"]), 1e-9)
            if largest_gap is None or gap_relative > largest_gap["relativeGap"]:
                largest_gap = {
                    "channel": sim_channel,
                    "simMean": sim_stats["mean"],
                    "realMean": real_stats["mean"],
                    "gap": round(gap, 6),
                    "relativeGap": round(gap_relative, 4),
                }
        channels[sim_channel] = {"sim": sim_stats, "real": real_stats, "samples": {"sim": len(sim_values), "real": len(real_values)}}

    return {
        "ok": True,
        "simRun": run.id,
        "realData": real_csv_path,
        "channels": channels,
        "largestGap": largest_gap,
        "verdict": "simulation and real data differ; simulation is not real-robot evidence",
        "notes": [
            "distribution-level comparison only; timestamps and control modes are not aligned here",
            "a gap in one channel does not imply a single root cause — see diagnostics workflow",
        ],
    }


def run_pick_place(
    scenario_config: dict[str, Any],
    fault: dict[str, Any],
    seed: int,
    store: RunStore | None = None,
    run_id: str | None = None,
) -> tuple[Run, list[dict[str, Any]]]:
    """Execute one pick-place run and persist it plus artifacts.

    Returns the run and its telemetry rows. When ``store`` is given, the run
    JSON, telemetry JSONL, charts and a scene image are written under the
    store, and the run's ``artifacts`` map is filled with the written paths.
    """
    validated = validate_scenario(scenario_config)
    if not validated["ok"]:
        raise ValueError(f"invalid scenario: {validated['issues']}")
    scenario = validated["resolved"]
    runner = PickPlaceRunner(scenario, fault, seed)
    run = runner.run()
    run.id = run_id or run.id

    if store is not None:
        run_dir = store.run_dir(run.id)
        artifact_dir = os.path.join(run_dir, "artifacts")
        os.makedirs(artifact_dir, exist_ok=True)
        telemetry_path = write_telemetry(run_dir, runner.telemetry)
        run.artifacts["telemetry.jsonl"] = telemetry_path
        for chart in render_charts(run, runner.telemetry, artifact_dir):
            run.artifacts[os.path.basename(chart)] = chart
        scene = render_scene_image(run.id, runner.env.xml, artifact_dir)
        if scene:
            run.artifacts["scene.png"] = scene
        run.artifacts["run.json"] = store.save_run(run)
    return run, runner.telemetry
