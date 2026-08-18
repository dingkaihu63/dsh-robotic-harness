"""Robot asset inspection and validation (URDF / MJCF).

Read-only by design: inspection never rewrites the asset. The only command that
produces a new file is the explicit URDF -> MJCF conversion
(:func:`convert_urdf_to_mjcf`), which uses the MuJoCo compiler and writes a
sibling file plus a conversion report.
"""

from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from typing import Any, Optional

from .core import AssetInspection, Issue

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

_XML_ATTR = "{http://www.w3.org/XML/1998/namespace}base"


def _el_attr(el: Optional[ET.Element], attr: str, default: str = "") -> str:
    if el is None:
        return default
    return el.attrib.get(attr, default)


def _float(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _vector(value: str, default: list[float] | None = None) -> list[float]:
    if default is None:
        default = [0.0, 0.0, 0.0]
    try:
        parts = [float(v) for v in value.replace(",", " ").split()]
        return parts if len(parts) == 3 else default
    except (TypeError, ValueError):
        return default


def _inertia_matrix(ixx: float, ixy: float, ixz: float, iyy: float, iyz: float, izz: float) -> list[list[float]]:
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def _is_positive_definite(matrix: list[list[float]], tolerance: float = 1e-12) -> bool:
    """Sylvester's criterion for symmetric 3x3 matrices (pure math, no numpy).

    The determinant comparisons are relative to the product of the diagonal
    entries so tiny-but-valid inertias (10^-5 kg m^2) are not rejected by an
    absolute epsilon.
    """
    a, b, c = matrix[0][0], matrix[1][1], matrix[2][2]
    ab, ac, bc = matrix[0][1], matrix[0][2], matrix[1][2]
    if a <= tolerance:
        return False
    det2 = a * b - ab * ab
    if det2 <= tolerance * max(a * b, 1.0):
        return False
    det3 = a * (b * c - bc * bc) - ab * (ab * c - ac * bc) + ac * (ab * bc - ac * b)
    scale = a * b * c if a * b * c > 0 else 1.0
    return det3 > tolerance * scale


def _check_inertia(mass: float, inertia: list[list[float]], issues: list[Issue], location: str) -> None:
    if mass <= 0:
        issues.append(
            Issue(
                severity="error",
                code="inertial.mass_non_positive",
                message=f"mass must be positive, got {mass}",
                location=location,
            )
        )
    if mass > 1e6:
        issues.append(
            Issue(
                severity="error",
                code="inertial.mass_suspicious_large",
                message=f"mass {mass} kg looks like a unit error (grams or tonnes)",
                location=location,
            )
        )
    if mass < 1e-6 and mass > 0:
        issues.append(
            Issue(
                severity="warning",
                code="inertial.mass_suspicious_small",
                message=f"mass {mass} kg is suspiciously small",
                location=location,
            )
        )
    diagonal = [inertia[i][i] for i in range(3)]
    if max(diagonal) <= 0:
        issues.append(
            Issue(
                severity="error",
                code="inertial.zero",
                message="inertia matrix is zero; the link will not rotate realistically",
                location=location,
            )
        )
    elif not _is_positive_definite(inertia):
        issues.append(
            Issue(
                severity="error",
                code="inertial.not_positive_definite",
                message="inertia matrix is not positive definite (check units and axis order)",
                location=location,
            )
        )
    # Order-of-magnitude sanity: for a box of dimension d, I ~ m*d^2/12.
    # We only flag when diagonal entries differ by more than 1e6x, which
    # indicates a unit or axis-order mistake rather than a legitimate shape.
    if max(diagonal) > 0 and max(diagonal) / max(min(diagonal), 1e-12) > 1e6:
        issues.append(
            Issue(
                severity="warning",
                code="inertial.anisotropic",
                message="inertia diagonal entries differ by >1e6x; check units or axis order",
                location=location,
            )
        )


# ---------------------------------------------------------------------------
# URDF
# ---------------------------------------------------------------------------

class UrdfInspection:
    """Parsed view of a URDF document used by inspection and conversion."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.directory = os.path.dirname(os.path.abspath(path))
        try:
            self.tree = ET.parse(path)
        except ET.ParseError as error:
            raise ValueError(f"URDF is not well-formed XML: {error}") from error
        self.root = self.tree.getroot()
        if self.root.tag != "robot":
            raise ValueError(f"expected <robot> root element, got <{self.root.tag}>")

    @property
    def name(self) -> str:
        return _el_attr(self.root, "name")

    def links(self) -> list[ET.Element]:
        return list(self.root.findall("link"))

    def joints(self) -> list[ET.Element]:
        return list(self.root.findall("joint"))

    def transmissions(self) -> list[ET.Element]:
        return list(self.root.findall("transmission"))

    def materials(self) -> list[ET.Element]:
        return list(self.root.findall("material"))

    def link_names(self) -> set[str]:
        return {_el_attr(el, "name") for el in self.links()}

    def joint_names(self) -> set[str]:
        return {_el_attr(el, "name") for el in self.joints()}

    def parent_map(self) -> dict[str, str]:
        """joint name -> parent link name."""
        return {_el_attr(j, "name"): _el_attr(j.find("parent"), "link") for j in self.joints()}

    def child_map(self) -> dict[str, str]:
        """joint name -> child link name."""
        return {_el_attr(j, "name"): _el_attr(j.find("child"), "link") for j in self.joints()}

    def root_links(self) -> list[str]:
        children = set(self.child_map().values())
        return [name for name in self.link_names() if name not in children]


def inspect_urdf(path: str) -> AssetInspection:
    """Inspect a URDF file and return a structured report plus issues."""
    document = UrdfInspection(path)
    issues: list[Issue] = []

    links = document.links()
    joints = document.joints()
    names = document.link_names()

    # --- identity ---------------------------------------------------------
    summary: dict[str, Any] = {
        "format": "urdf",
        "robotName": document.name,
        "linkCount": len(links),
        "jointCount": len(joints),
        "transmissionCount": len(document.transmissions()),
        "materialCount": len(document.materials()),
        "rootLinks": document.root_links(),
        "jointTypes": {},
        "links": [],
        "joints": [],
    }

    # --- names and tree structure -----------------------------------------
    for link in links:
        link_name = _el_attr(link, "name")
        if not link_name:
            issues.append(Issue("error", "urdf.link_unnamed", "link without a name attribute"))
    seen = set()
    for link in links:
        name = _el_attr(link, "name")
        if name in seen:
            issues.append(Issue("error", "urdf.duplicate_link", f"duplicate link name {name!r}"))
        seen.add(name)
    seen = set()
    for joint in joints:
        name = _el_attr(joint, "name")
        if name in seen:
            issues.append(Issue("error", "urdf.duplicate_joint", f"duplicate joint name {name!r}"))
        seen.add(name)

    parent_map = document.parent_map()
    child_map = document.child_map()
    referenced_parents = {p for p in parent_map.values() if p}
    referenced_children = {c for c in child_map.values() if c}
    for ref in referenced_parents | referenced_children:
        if ref not in names:
            issues.append(
                Issue(
                    "error",
                    "urdf.missing_link",
                    f"joint references link {ref!r} which does not exist",
                )
            )
    if len(document.root_links()) == 0:
        issues.append(Issue("error", "urdf.no_root_link", "no root link found (cycle or empty robot)"))
    elif len(document.root_links()) > 1:
        issues.append(
            Issue(
                "warning",
                "urdf.multiple_roots",
                f"multiple root links: {', '.join(document.root_links())}",
            )
        )

    # --- links: inertial / visual / collision ------------------------------
    link_entries: list[dict[str, Any]] = []
    for link in links:
        link_name = _el_attr(link, "name")
        entry: dict[str, Any] = {"name": link_name, "inertial": None, "visualCount": 0, "collisionCount": 0}
        inertial = link.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            inertia_el = inertial.find("inertia")
            mass = _float(_el_attr(mass_el, "value")) if mass_el is not None else 0.0
            ixx = _float(_el_attr(inertia_el, "ixx")) if inertia_el is not None else 0.0
            ixy = _float(_el_attr(inertia_el, "ixy")) if inertia_el is not None else 0.0
            ixz = _float(_el_attr(inertia_el, "ixz")) if inertia_el is not None else 0.0
            iyy = _float(_el_attr(inertia_el, "iyy")) if inertia_el is not None else 0.0
            iyz = _float(_el_attr(inertia_el, "iyz")) if inertia_el is not None else 0.0
            izz = _float(_el_attr(inertia_el, "izz")) if inertia_el is not None else 0.0
            matrix = _inertia_matrix(ixx, ixy, ixz, iyy, iyz, izz)
            origin = inertial.find("origin")
            origin_xyz = _vector(_el_attr(origin, "xyz")) if origin is not None else [0.0, 0.0, 0.0]
            _check_inertia(mass, matrix, issues, f"link/{link_name}/inertial")
            entry["inertial"] = {
                "mass": mass,
                "inertia": matrix,
                "originXyz": origin_xyz,
                "originRpy": _vector(_el_attr(origin, "rpy")) if origin is not None else [0.0, 0.0, 0.0],
            }
        else:
            issues.append(
                Issue(
                    "warning",
                    "urdf.missing_inertial",
                    f"link {link_name!r} has no inertial block; controllers will use defaults",
                    f"link/{link_name}",
                )
            )
        entry["visualCount"] = len(link.findall("visual"))
        entry["collisionCount"] = len(link.findall("collision"))
        # mesh paths
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                _check_mesh_path(mesh, document.directory, issues, f"link/{link_name}/visual")
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is not None:
                _check_mesh_path(mesh, document.directory, issues, f"link/{link_name}/collision")
        if entry["visualCount"] == 0 and entry["collisionCount"] == 0:
            issues.append(
                Issue(
                    "info",
                    "urdf.no_geometry",
                    f"link {link_name!r} has neither visual nor collision geometry",
                    f"link/{link_name}",
                )
            )
        link_entries.append(entry)
    summary["links"] = link_entries

    # --- joints -------------------------------------------------------------
    joint_entries: list[dict[str, Any]] = []
    for joint in joints:
        joint_name = _el_attr(joint, "name")
        joint_type = _el_attr(joint, "type")
        summary["jointTypes"].setdefault(joint_type, 0)
        summary["jointTypes"][joint_type] += 1
        origin = joint.find("origin")
        axis_el = joint.find("axis")
        limit = joint.find("limit")
        entry: dict[str, Any] = {
            "name": joint_name,
            "type": joint_type,
            "parent": _el_attr(joint.find("parent"), "link"),
            "child": _el_attr(joint.find("child"), "link"),
            "originXyz": _vector(_el_attr(origin, "xyz")) if origin is not None else [0.0, 0.0, 0.0],
            "originRpy": _vector(_el_attr(origin, "rpy")) if origin is not None else [0.0, 0.0, 0.0],
            "axis": _vector(_el_attr(axis_el, "xyz"), [1.0, 0.0, 0.0]) if axis_el is not None else None,
            "limits": None,
            "mimic": _el_attr(joint.find("mimic"), "joint") or None,
        }
        if joint_type == "floating":
            issues.append(
                Issue(
                    "warning",
                    "urdf.floating_joint",
                    f"floating joint {joint_name!r} is rarely supported by simulators",
                    f"joint/{joint_name}",
                )
            )
        if joint_type in ("revolute", "prismatic", "continuous"):
            if axis_el is None:
                issues.append(
                    Issue(
                        "error",
                        "urdf.missing_axis",
                        f"joint {joint_name!r} has no <axis>; default (1,0,0) may be wrong",
                        f"joint/{joint_name}",
                    )
                )
            if limit is None and joint_type in ("revolute", "prismatic"):
                issues.append(
                    Issue(
                        "error",
                        "urdf.missing_limit",
                        f"joint {joint_name!r} is {joint_type} but has no <limit>",
                        f"joint/{joint_name}",
                    )
                )
            elif limit is not None:
                lower = _float(_el_attr(limit, "lower"))
                upper = _float(_el_attr(limit, "upper"))
                if lower >= upper:
                    issues.append(
                        Issue(
                            "error",
                            "urdf.invalid_limit",
                            f"joint {joint_name!r} limit lower={lower} >= upper={upper}",
                            f"joint/{joint_name}",
                        )
                    )
                if joint_type == "revolute" and upper - lower > 2 * math.pi + 1e-6:
                    issues.append(
                        Issue(
                            "info",
                            "urdf.limit_wraps",
                            f"joint {joint_name!r} range exceeds 2*pi; consider 'continuous'",
                            f"joint/{joint_name}",
                        )
                    )
                entry["limits"] = {
                    "lower": lower,
                    "upper": upper,
                    "effort": _float(_el_attr(limit, "effort")),
                    "velocity": _float(_el_attr(limit, "velocity")),
                }
        joint_entries.append(entry)
    summary["joints"] = joint_entries

    return AssetInspection(format="urdf", path=path, summary=summary, issues=issues)


def _check_mesh_path(mesh: ET.Element, base_dir: str, issues: list[Issue], location: str) -> None:
    filename = _el_attr(mesh, "filename")
    if not filename:
        issues.append(Issue("error", "urdf.mesh_missing_filename", "mesh element without filename", location))
        return
    resolved = filename
    if resolved.startswith("package://"):
        issues.append(
            Issue(
                "info",
                "urdf.mesh_package_uri",
                f"mesh uses package:// URI ({resolved}); resolution needs the package on the target machine",
                location,
            )
        )
        return
    candidate = os.path.normpath(os.path.join(base_dir, resolved))
    if not os.path.exists(candidate):
        issues.append(
            Issue(
                "warning",
                "urdf.mesh_missing",
                f"mesh file not found next to the URDF: {resolved}",
                location,
            )
        )


def validate_urdf(path: str) -> dict[str, Any]:
    """Run inspection and return the verdict summary used by the DSH tool."""
    inspection = inspect_urdf(path)
    errors = [i for i in inspection.issues if i.severity == "error"]
    warnings = [i for i in inspection.issues if i.severity == "warning"]
    return {
        "ok": len(errors) == 0,
        "format": "urdf",
        "path": path,
        "summary": inspection.summary,
        "issueCounts": {
            "error": len(errors),
            "warning": len(warnings),
            "info": len([i for i in inspection.issues if i.severity == "info"]),
        },
        "issues": [i.to_dict() for i in inspection.issues],
    }


# ---------------------------------------------------------------------------
# MJCF
# ---------------------------------------------------------------------------

def inspect_mjcf(path: str) -> AssetInspection:
    """Inspect an MJCF/XML file.

    When MuJoCo is importable we load the model through the real parser (this
    catches solver-level errors); otherwise we fall back to plain XML
    inspection and report the loader as unavailable.
    """
    issues: list[Issue] = []
    warnings_from_loader: list[str] = []
    summary: dict[str, Any] = {"format": "mjcf"}

    try:
        import mujoco  # noqa: PLC0415
    except Exception as error:  # pragma: no cover - depends on environment
        summary["loader"] = "unavailable"
        summary["loaderReason"] = str(error)
        issues.append(
            Issue(
                "warning",
                "mjcf.loader_unavailable",
                "mujoco is not importable in this Python environment; XML-only inspection",
            )
        )
    else:
        summary["loader"] = f"mujoco {getattr(mujoco, '__version__', 'unknown')}"
        try:
            model = mujoco.MjModel.from_xml_path(path)
        except Exception as error:
            issues.append(
                Issue(
                    "error",
                    "mjcf.load_failed",
                    f"MuJoCo could not load the file: {error}",
                )
            )
            summary["loadOk"] = False
            return AssetInspection(format="mjcf", path=path, summary=summary, issues=issues)
        summary["loadOk"] = True
        summary["nq"] = int(model.nq)
        summary["nv"] = int(model.nv)
        summary["nu"] = int(model.nu)
        summary["nbody"] = int(model.nbody)
        summary["ngeom"] = int(model.ngeom)
        summary["njnt"] = int(model.njnt)
        summary["ntrq"] = int(model.ntrq) if hasattr(model, "ntrq") else 0
        total_mass = float(model.body_mass.sum())
        summary["totalMassKg"] = round(total_mass, 6)
        joint_names = [model.joint(i).name for i in range(model.njnt)]
        body_names = [model.body(i).name for i in range(model.nbody)]
        summary["joints"] = joint_names
        summary["bodies"] = body_names
        freejoints = [name for name in joint_names if name and "free" in name.lower()]
        if freejoints:
            issues.append(
                Issue(
                    "info",
                    "mjcf.free_joint",
                    f"model has free joint(s): {', '.join(freejoints)}",
                )
            )
        summary["actuators"] = [model.actuator(i).name for i in range(model.nu)]

    # Plain XML fallback view always collected.
    try:
        tree = ET.parse(path)
    except ET.ParseError as error:
        issues.append(Issue("error", "mjcf.bad_xml", f"not well-formed XML: {error}"))
        return AssetInspection(format="mjcf", path=path, summary=summary, issues=issues)
    root = tree.getroot()
    summary["xmlRoot"] = root.tag
    summary["xmlCompiler"] = root.attrib if root.tag == "mujoco" else {}
    return AssetInspection(format="mjcf", path=path, summary=summary, issues=issues, warnings_from_loader=warnings_from_loader)


# ---------------------------------------------------------------------------
# conversion
# ---------------------------------------------------------------------------

def convert_urdf_to_mjcf(urdf_path: str, out_path: str) -> dict[str, Any]:
    """Convert URDF to MJCF using the MuJoCo compiler.

    The conversion is explicit and reported: the source URDF is never
    modified, and the returned report lists the target path, the compiler
    version and any loader warnings. Automatic conversion is a suggestion,
    not a guarantee: differences between the URDF and MJCF semantics are
    recorded in ``report.differences``.
    """
    import io
    import contextlib

    # never allow "conversion" to overwrite the source asset
    if os.path.realpath(os.path.abspath(urdf_path)) == os.path.realpath(os.path.abspath(out_path)):
        raise WorkerError("refusing to overwrite the source URDF: outPath resolves to the input file")

    try:
        import mujoco  # noqa: PLC0415
    except ImportError as error:
        raise WorkerError(
            "MuJoCo is required for URDF->MJCF conversion; install it with 'pip install mujoco'"
        ) from error

    try:
        directory = os.path.dirname(os.path.abspath(urdf_path))
        buffer = io.StringIO()
        with contextlib.redirect_stderr(buffer):
            model = mujoco.MjModel.from_xml_path(urdf_path)
        warnings = [line.strip() for line in buffer.getvalue().splitlines() if line.strip()]

        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        # write atomically so a crash cannot leave a truncated MJCF at outPath
        tmp_path = f"{os.path.abspath(out_path)}.tmp-{os.getpid()}"
        if hasattr(model, "get_xml"):
            mjcf_xml = model.get_xml()
            with open(tmp_path, "w", encoding="utf-8") as handle:
                handle.write(mjcf_xml)
        else:
            # MuJoCo >= 3.x removed MjModel.get_xml(); mj_saveLastXML writes
            # the XML the model was compiled from.
            mujoco.mj_saveLastXML(tmp_path, model)
        os.replace(tmp_path, os.path.abspath(out_path))
    except WorkerError:
        raise
    except Exception as error:  # noqa: BLE001 - structured conversion failure
        raise WorkerError(f"URDF->MJCF conversion failed: {type(error).__name__}: {error}") from error

    return {
        "ok": True,
        "source": urdf_path,
        "target": out_path,
        "compiler": getattr(mujoco, "__version__", "unknown"),
        "loaderWarnings": warnings,
        "differences": [
            "URDF <transmission> and <gazebo> extensions are not preserved by the MuJoCo compiler",
            "visual/collision semantics follow MuJoCo defaults unless the generated MJCF is tuned",
            "inertial origins are normalized to body frames by the MuJoCo compiler",
        ],
        "advice": "review the generated MJCF before use in experiments; auto-conversion does not replace human review",
    }


def inspect_asset(path: str) -> AssetInspection:
    """Dispatch to the right inspector by file extension."""
    lowered = path.lower()
    if lowered.endswith((".urdf", ".xacro")):
        if lowered.endswith(".xacro"):
            raise ValueError(
                "xacro files must be expanded first (xacro --inorder file.xacro > file.urdf); "
                "inspection of raw xacro is not supported"
            )
        return inspect_urdf(path)
    if lowered.endswith((".xml", ".mjcf")):
        return inspect_mjcf(path)
    raise ValueError(
        f"unsupported asset format for {path!r}: expected .urdf, .xacro (expanded), .xml or .mjcf"
    )


# ---------------------------------------------------------------------------
# SDF
# ---------------------------------------------------------------------------

def validate_sdf(path: str) -> dict[str, Any]:
    """Validate an SDF file (XML structure + common robotics checks).

    SDF validation is best-effort: the full Gazebo semantics require the
    Gazebo toolchain. This checker covers structure, links, joints,
    inertials and units; deeper checks are delegated to the simulator.
    """
    issues: list[Issue] = []
    try:
        tree = ET.parse(path)
    except ET.ParseError as error:
        return {
            "ok": False,
            "format": "sdf",
            "path": path,
            "issues": [{"severity": "error", "code": "sdf.bad_xml", "message": f"not well-formed XML: {error}"}],
            "summary": {},
            "note": "SDF validation is structural only; full semantics require Gazebo.",
        }
    root = tree.getroot()
    if root.tag != "sdf":
        issues.append(Issue("error", "sdf.bad_root", f"expected <sdf> root, got <{root.tag}>"))
    version = _el_attr(root, "version")
    if version and version not in ("1.6", "1.7", "1.8", "1.9", "1.10", "1.11", "1.12"):
        issues.append(Issue("warning", "sdf.unknown_version", f"unfamiliar SDF version {version!r}"))

    summary: dict[str, Any] = {"format": "sdf", "version": version or "unknown", "models": []}
    for model in root.findall(".//model"):
        model_name = _el_attr(model, "name")
        entry: dict[str, Any] = {"name": model_name, "links": 0, "joints": 0, "inertialCount": 0}
        links = model.findall("link")
        joints = model.findall("joint")
        entry["links"] = len(links)
        entry["joints"] = len(joints)
        seen_names: set[str] = set()
        for link in links:
            name = _el_attr(link, "name")
            if name in seen_names:
                issues.append(Issue("error", "sdf.duplicate_link", f"duplicate link {name!r} in model {model_name!r}"))
            seen_names.add(name)
            if link.find("inertial") is not None:
                entry["inertialCount"] += 1
            else:
                issues.append(
                    Issue(
                        "warning",
                        "sdf.missing_inertial",
                        f"link {name!r} in model {model_name!r} has no <inertial>",
                    )
                )
            mass_el = link.find("inertial/mass")
            if mass_el is not None:
                try:
                    mass = float(mass_el.text or "0")
                except ValueError:
                    mass = 0.0
                if mass <= 0:
                    issues.append(Issue("error", "sdf.non_positive_mass", f"link {name!r} mass must be positive, got {mass}"))
        for joint in joints:
            jtype = _el_attr(joint, "type")
            if jtype not in ("revolute", "prismatic", "fixed", "continuous", "ball", "universal", "screw", "gearbox"):
                issues.append(Issue("warning", "sdf.unknown_joint_type", f"joint {_el_attr(joint, 'name')!r} type {jtype!r}"))
            if jtype in ("revolute", "prismatic") and joint.find("axis/limit") is None:
                issues.append(
                    Issue(
                        "warning",
                        "sdf.missing_limit",
                        f"joint {_el_attr(joint, 'name')!r} ({jtype}) has no axis limit",
                    )
                )
        summary["models"].append(entry)

    return {
        "ok": not any(i.severity == "error" for i in issues),
        "format": "sdf",
        "path": path,
        "summary": summary,
        "issues": [i.to_dict() for i in issues],
        "note": "SDF validation is structural only; full semantics require Gazebo.",
    }
