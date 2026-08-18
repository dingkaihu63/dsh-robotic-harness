"""CAD / mechanical modeling for Robotic Harness (plan chapter 6).

SolidWorks is commercial software and is **never assumed**: this module works
with "file-level post-processing + optional FreeCAD backend detection". All
inventory, version comparison, mesh inspection, inertia validation, topology
validation and previews are implemented with the Python 3.10 standard library
plus numpy. When a FreeCAD backend is importable (``import FreeCAD`` succeeds)
STEP files get real part/assembly traversal; otherwise STEP files get
header-level metadata (FILE_NAME / FILE_SCHEMA) with an explicit note.

Commands (each ``cmd_xxx(args: dict) -> dict``):
- ``cad-inventory``            scan a CAD asset tree
- ``cad-compare-versions``     diff two URDFs / inventory snapshots / dirs
- ``mesh-inspect``             parse STL (binary/ASCII) and OBJ with numpy
- ``inertia-validate``         URDF per-link inertia validation
- ``robot-topology-validate``  URDF kinematic tree checks
- ``urdf-preview``             static 2D skeleton SVG (XZ projection)
- ``export-sim-asset``         mjcf conversion or sdf-compat report
- ``asset-report``             Markdown asset inspection report

No third-party dependencies beyond numpy. FreeCAD is optional and probed at
runtime; when absent STEP handling degrades to header-level metadata.
"""

from __future__ import annotations

import json
import math
import os
import re
import struct
import time
import xml.etree.ElementTree as ET
from typing import Any, Optional

import numpy as np

from .assets import UrdfInspection, convert_urdf_to_mjcf, inspect_asset, inspect_urdf
from .core import WorkerError, sha256_file

# ---------------------------------------------------------------------------
# constants / helpers
# ---------------------------------------------------------------------------

CAD_FORMATS: dict[str, str] = {
    ".step": "step",
    ".stp": "step",
    ".stl": "stl",
    ".obj": "obj",
    ".dae": "dae",
    ".urdf": "urdf",
    ".xacro": "xacro",
    ".sldprt": "sldprt",
    ".sldasm": "sldasm",
    ".igs": "iges",
    ".iges": "iges",
    ".fcstd": "fcstd",
}

SOLIDWORKS_NOTE = "需要 SolidWorks 或 eDrawings 打开，本工具不做二进制解析"

STEP_HEADER_ONLY_NOTE = (
    "FreeCAD 未安装：仅 STEP 头文件级元数据（FILE_NAME/FILE_SCHEMA）；"
    "安装 FreeCAD 后可做零件/装配遍历"
)


def _require_path(args: dict[str, Any], key: str = "path") -> str:
    """Return the absolute path for a required argument or raise WorkerError."""
    path = args.get(key)
    if not path:
        raise WorkerError(f"missing required argument {key!r}")
    if not os.path.exists(path):
        raise WorkerError(f"path not found: {path}")
    return os.path.abspath(path)


def _attr(el: Optional[ET.Element], name: str, default: str = "") -> str:
    if el is None:
        return default
    return el.attrib.get(name, default)


def _vector(value: Optional[str], default: Optional[list[float]] = None) -> list[float]:
    if default is None:
        default = [0.0, 0.0, 0.0]
    try:
        parts = [float(v) for v in str(value).replace(",", " ").split()]
        return parts if len(parts) == 3 else default
    except (TypeError, ValueError):
        return default


def _iso_time(timestamp: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(timestamp))


def _xml_escape(value: str) -> str:
    return (
        value.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")
    )


def _md_escape(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", " ")


# ---------------------------------------------------------------------------
# optional FreeCAD backend (never assumed present)
# ---------------------------------------------------------------------------

def freecad_backend() -> dict[str, Any]:
    """Probe the optional FreeCAD backend; always returns a serializable dict."""
    try:
        import FreeCAD  # noqa: PLC0415
    except Exception:
        return {"available": False, "version": None}
    version = getattr(FreeCAD, "Version", [])
    return {"available": True, "version": ".".join(str(v) for v in version[:3]) if version else "unknown"}


def _parse_step_header(path: str) -> dict[str, Any]:
    """Header-level STEP metadata without FreeCAD (FILE_NAME/FILE_SCHEMA)."""
    meta: dict[str, Any] = {
        "name": None,
        "timestamp": None,
        "author": None,
        "organization": None,
        "preprocessor": None,
        "system": None,
        "authoringSystem": None,
        "description": None,
        "schema": None,
    }
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            head = handle.read(64 * 1024)
    except OSError:
        return meta
    match = re.search(r"FILE_NAME\((.*?)\);", head, re.S)
    if match:
        # STEP string fields are comma-separated, but author/organization are
        # parenthesized LISTS of quoted strings ('John','Jane') — a plain
        # split(",") shifts every following field by the list length. Split on
        # top-level commas only (outside quotes and parentheses).
        raw = match.group(1)
        parts: list[str] = []
        current: list[str] = []
        depth = 0
        in_quote = False
        for char in raw:
            if char == "'":
                in_quote = not in_quote
                current.append(char)
            elif char == "(" and not in_quote:
                depth += 1
                current.append(char)
            elif char == ")" and not in_quote:
                depth -= 1
                current.append(char)
            elif char == "," and depth == 0 and not in_quote:
                parts.append("".join(current).strip())
                current = []
            else:
                current.append(char)
        if current:
            parts.append("".join(current).strip())
        fields = ["name", "timestamp", "author", "organization", "preprocessor", "system", "authoringSystem"]
        for field, part in zip(fields, parts):
            value = part.strip("'\"").strip("()")
            meta[field] = value or None
    match = re.search(r"FILE_DESCRIPTION\((.*?)\);", head, re.S)
    if match:
        inner = re.search(r"'([^']*)'", match.group(1))
        if inner:
            meta["description"] = inner.group(1)
    match = re.search(r"FILE_SCHEMA\((.*?)\);", head, re.S)
    if match:
        inner = re.search(r"'([^']*)'", match.group(1))
        if inner:
            meta["schema"] = inner.group(1)
    return meta


def _freecad_step_parts(path: str) -> list[dict[str, Any]]:
    """Traverse a STEP file's parts/assemblies through the FreeCAD backend."""
    import FreeCAD  # noqa: PLC0415
    import Import  # noqa: PLC0415

    document = Import.open(path)
    parts: list[dict[str, Any]] = []
    for obj in document.Objects:
        parts.append(
            {
                "name": getattr(obj, "Label", None) or getattr(obj, "Name", "") or "",
                "type": getattr(obj, "TypeId", "") or "",
            }
        )
    try:
        FreeCAD.closeDocument(document.Name)
    except Exception:
        pass
    return parts


def _step_entry(filepath: str, backend: dict[str, Any]) -> dict[str, Any]:
    """Best-effort STEP metadata: FreeCAD traversal when available, else header."""
    entry: dict[str, Any] = {"parse": "header-only", "note": STEP_HEADER_ONLY_NOTE}
    entry.update(_parse_step_header(filepath))
    if backend.get("available"):
        try:
            parts = _freecad_step_parts(filepath)
            entry["parse"] = "freecad"
            entry["parts"] = parts
            entry["note"] = f"通过 FreeCAD {backend.get('version')} 解析零件/装配（{len(parts)} 个对象）"
        except Exception as error:  # pragma: no cover - FreeCAD dependent
            entry["parse"] = "header-only"
            entry["note"] = f"FreeCAD 可用但解析失败（{error}）；仅返回头文件级元数据"
    return entry


# ---------------------------------------------------------------------------
# 1. cad-inventory
# ---------------------------------------------------------------------------

def cmd_cad_inventory(args: dict[str, Any]) -> dict[str, Any]:
    """Scan a CAD asset tree: formats, sizes, SHA-256, mtimes, integrity issues.

    SolidWorks files (``.sldprt`` / ``.sldasm``) are registered but never
    parsed (binary format); STEP files use FreeCAD when available and fall back
    to header-level metadata otherwise.
    """
    path = _require_path(args)
    recursive = bool(args.get("recursive", True))
    allowed: Optional[set[str]] = None
    requested = args.get("formats")
    if requested is not None:
        if not isinstance(requested, list) or not all(isinstance(f, str) for f in requested):
            raise WorkerError("'formats' must be a list of extensions like ['.step', '.stl']")
        allowed = {"." + f.strip().lstrip(".").lower() for f in requested if f.strip()}
        unknown = allowed - set(CAD_FORMATS)
        if unknown:
            raise WorkerError(
                f"unsupported CAD formats: {sorted(unknown)}; supported: {sorted(CAD_FORMATS)}"
            )

    root = path if os.path.isdir(path) else os.path.dirname(path)
    candidates: list[str] = []
    if os.path.isfile(path):
        candidates.append(path)
    else:
        for dirpath, dirnames, filenames in os.walk(path):
            candidates.extend(os.path.join(dirpath, fn) for fn in filenames)
            if not recursive:
                dirnames[:] = []

    backend = freecad_backend()
    files: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []
    seen_names: dict[str, list[str]] = {}
    total_size = 0

    for filepath in candidates:
        ext = os.path.splitext(filepath)[1].lower()
        fmt = CAD_FORMATS.get(ext)
        if fmt is None:
            continue
        if allowed is not None and ext not in allowed:
            continue
        try:
            size = os.path.getsize(filepath)
            modified = os.path.getmtime(filepath)
            digest = sha256_file(filepath)
        except OSError as error:
            issues.append(
                {
                    "severity": "error",
                    "code": "inventory.unreadable",
                    "message": f"cannot read {filepath}: {error}",
                    "path": os.path.abspath(filepath),
                }
            )
            continue
        entry: dict[str, Any] = {
            "path": os.path.abspath(filepath),
            "format": fmt,
            "size": int(size),
            "sha256": digest,
            "modifiedAt": _iso_time(modified),
        }
        if fmt in ("sldprt", "sldasm"):
            entry["note"] = SOLIDWORKS_NOTE
        if fmt == "step":
            entry["step"] = _step_entry(filepath, backend)
        files.append(entry)
        total_size += size
        seen_names.setdefault(os.path.basename(filepath).lower(), []).append(entry["path"])
        if size == 0:
            issues.append(
                {
                    "severity": "warning",
                    "code": "inventory.zero_byte",
                    "message": f"file is 0 bytes: {filepath}",
                    "path": os.path.abspath(filepath),
                }
            )

    for name, paths in seen_names.items():
        if len(paths) > 1:
            issues.append(
                {
                    "severity": "info",
                    "code": "inventory.duplicate_name",
                    "message": f"duplicate file name {name!r} at different paths",
                    "paths": sorted(paths),
                }
            )

    by_format: dict[str, int] = {}
    for entry in files:
        by_format[entry["format"]] = by_format.get(entry["format"], 0) + 1

    solidworks_count = sum(1 for e in files if e["format"] in ("sldprt", "sldasm"))
    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "root": os.path.abspath(root),
        "files": files,
        "counts": {"byFormat": by_format, "totalFiles": len(files)},
        "totalSize": int(total_size),
        "issues": issues,
        "solidWorksFiles": solidworks_count,
        "solidWorksNote": SOLIDWORKS_NOTE,
        "freecad": backend,
        "inputArgs": {"path": path, "recursive": recursive, "formats": requested},
    }


# ---------------------------------------------------------------------------
# 2. cad-compare-versions
# ---------------------------------------------------------------------------

def _urdf_model(path: str) -> dict[str, Any]:
    """Compact URDF model view used for version comparison."""
    inspection = inspect_urdf(path)
    summary = inspection.summary
    links: dict[str, Any] = {}
    for entry in summary["links"]:
        inertial = entry.get("inertial")
        links[entry["name"]] = {"mass": inertial["mass"] if inertial else None, "hasInertial": inertial is not None}
    joints: dict[str, Any] = {}
    for entry in summary["joints"]:
        joints[entry["name"]] = {
            "type": entry["type"],
            "parent": entry.get("parent"),
            "child": entry.get("child"),
            "axis": entry.get("axis"),
        }
    return {"robotName": summary.get("robotName"), "links": links, "joints": joints}


def _compare_urdf(path_a: str, path_b: str) -> dict[str, Any]:
    model_a = _urdf_model(path_a)
    model_b = _urdf_model(path_b)
    links_a = set(model_a["links"])
    links_b = set(model_b["links"])
    joints_a = set(model_a["joints"])
    joints_b = set(model_b["joints"])

    mass_changed: list[dict[str, Any]] = []
    for name in sorted(links_a & links_b):
        mass_a = model_a["links"][name].get("mass")
        mass_b = model_b["links"][name].get("mass")
        if mass_a is None or mass_b is None:
            continue
        pct = abs(mass_b - mass_a) / max(abs(mass_a), 1e-12) * 100.0
        if pct > 10.0:
            mass_changed.append(
                {
                    "link": name,
                    "massA": round(float(mass_a), 6),
                    "massB": round(float(mass_b), 6),
                    "pctChange": round(float(pct), 2),
                }
            )

    type_changed: list[dict[str, Any]] = []
    for name in sorted(joints_a & joints_b):
        type_a = model_a["joints"][name]["type"]
        type_b = model_b["joints"][name]["type"]
        if type_a != type_b:
            type_changed.append({"joint": name, "typeA": type_a, "typeB": type_b})

    summary = {
        "links": {
            "added": sorted(links_b - links_a),
            "removed": sorted(links_a - links_b),
            "massChanged": mass_changed,
        },
        "joints": {
            "added": sorted(joints_b - joints_a),
            "removed": sorted(joints_a - joints_b),
            "typeChanged": type_changed,
        },
    }
    return {
        "ok": True,
        "kind": "urdf",
        "pathA": path_a,
        "pathB": path_b,
        "summary": summary,
        "rawDiff": {"a": model_a, "b": model_b},
    }


def _scan_inventory(path: str) -> dict[str, dict[str, Any]]:
    """Map relative path -> {size, sha256, format} for a dir or snapshot JSON."""
    mapping: dict[str, dict[str, Any]] = {}
    if os.path.isdir(path):
        result = cmd_cad_inventory({"path": path, "recursive": True})
        root = os.path.abspath(path)
        for entry in result["files"]:
            rel = os.path.relpath(entry["path"], root)
            mapping[rel] = {
                "size": entry["size"],
                "sha256": entry["sha256"],
                "format": entry["format"],
            }
        return mapping
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or "files" not in data:
        raise WorkerError(f"not an inventory snapshot JSON: {path}")
    root = data.get("root") or os.path.dirname(os.path.abspath(path))
    for entry in data.get("files", []):
        file_path = entry.get("path", "")
        rel = os.path.relpath(file_path, root) if file_path else os.path.basename(entry.get("name", ""))
        mapping[rel] = {
            "size": entry.get("size"),
            "sha256": entry.get("sha256"),
            "format": entry.get("format"),
        }
    return mapping


def _compare_inventory(path_a: str, path_b: str) -> dict[str, Any]:
    mapping_a = _scan_inventory(path_a)
    mapping_b = _scan_inventory(path_b)
    keys_a = set(mapping_a)
    keys_b = set(mapping_b)
    changed: list[dict[str, Any]] = []
    for key in sorted(keys_a & keys_b):
        if mapping_a[key].get("sha256") != mapping_b[key].get("sha256"):
            changed.append(
                {
                    "path": key,
                    "sizeA": mapping_a[key].get("size"),
                    "sizeB": mapping_b[key].get("size"),
                    "format": mapping_a[key].get("format"),
                }
            )
    summary = {
        "added": sorted(keys_b - keys_a),
        "removed": sorted(keys_a - keys_b),
        "changed": changed,
        "unchangedCount": len(keys_a & keys_b) - len(changed),
    }
    return {
        "ok": True,
        "kind": "inventory",
        "pathA": path_a,
        "pathB": path_b,
        "summary": summary,
    }


def cmd_cad_compare_versions(args: dict[str, Any]) -> dict[str, Any]:
    """Diff two URDFs, two inventory snapshot JSONs, or two CAD directories."""
    path_a = _require_path(args, "pathA")
    path_b = _require_path(args, "pathB")
    lower_a = path_a.lower()
    lower_b = path_b.lower()
    if lower_a.endswith(".urdf") and lower_b.endswith(".urdf"):
        return _compare_urdf(path_a, path_b)
    if lower_a.endswith(".json") and lower_b.endswith(".json"):
        return _compare_inventory(path_a, path_b)
    if os.path.isdir(path_a) and os.path.isdir(path_b):
        return _compare_inventory(path_a, path_b)
    raise WorkerError(
        "cannot infer comparison mode: pass two URDF files, two inventory snapshot JSONs, "
        "or two CAD directories"
    )


# ---------------------------------------------------------------------------
# 3. mesh-inspect
# ---------------------------------------------------------------------------

_VERTEX_RE = re.compile(
    rb"vertex\s+([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s+"
    rb"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s+"
    rb"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)"
)


def _try_binary_stl(path: str, head: bytes) -> Optional[np.ndarray]:
    """Parse a binary STL; returns (N,3,3) float64 or None if not binary."""
    try:
        if len(head) < 84:
            return None
        count = struct.unpack("<I", head[80:84])[0]
        size = os.path.getsize(path)
        if count == 0:
            # a size-84 header with count==0 is a valid EMPTY binary mesh;
            # anything longer with count==0 is garbage (fall through to ASCII,
            # which will also fail -> structured error)
            return np.zeros((0, 3, 3), dtype=np.float64) if size == 84 else None
        # accept trailing bytes after the triangle records: many tools append
        # footers — the old exact-size check rejected them
        if size < 84 + count * 50:
            return None
        with open(path, "rb") as handle:
            handle.seek(84)
            raw = handle.read(count * 50)
        if len(raw) != count * 50:
            return None
        # Per triangle: normal(3 f32) + vertices(3x3 f32) + attribute(2 B).
        # A structured dtype keeps the 2-byte attribute gaps from misaligning
        # the continuous float32 stream.
        record = np.dtype(
            [("normal", "<f4", (3,)), ("verts", "<f4", (3, 3)), ("attr", "<u2")]
        )
        triangles = np.frombuffer(raw, dtype=record, count=count)["verts"]
        return np.asarray(triangles, dtype=np.float64)
    except (ValueError, OSError, struct.error):
        return None


def _parse_stl_ascii(path: str, issues: list[dict[str, Any]]) -> Optional[np.ndarray]:
    """Parse an ASCII STL by collecting vertex triplets; None if not ASCII."""
    try:
        with open(path, "rb") as handle:
            raw = handle.read()
    except OSError as error:
        raise WorkerError(f"cannot read {path!r}: {error}") from error
    matches = _VERTEX_RE.findall(raw)
    if not matches:
        return None
    if len(matches) % 3 != 0:
        issues.append(
            {
                "severity": "warning",
                "code": "stl.ascii.vertex_count",
                "message": f"ASCII STL has {len(matches)} vertex lines, not a multiple of 3; trailing vertices ignored",
            }
        )
        matches = matches[: len(matches) - len(matches) % 3]
    array = np.array([[float(x), float(y), float(z)] for x, y, z in matches], dtype=np.float64)
    return array.reshape(-1, 3, 3)


def _mesh_stats(
    fmt: str,
    path: str,
    triangles: np.ndarray,
    issues: list[dict[str, Any]],
    volume: bool = False,
) -> dict[str, Any]:
    """Shared vertex/bounds/area statistics for STL and OBJ."""
    triangles = np.asarray(triangles, dtype=np.float64)
    count = int(triangles.shape[0])
    if count == 0:
        # an OBJ with vertices but zero valid faces (or an empty binary STL)
        # must produce a structured result, not a numpy reduction crash
        return {
            "format": fmt,
            "path": path,
            "vertices": 0,
            "vertexRecords": 0,
            "triangles": 0,
            "bounds": {"min": [None, None, None], "max": [None, None, None], "size": [None, None, None]},
            "degenerateTriangles": 0,
            "duplicateVertices": 0,
            "issues": issues,
            "note": "网格为空（无有效三角面）",
        }
    flat = triangles.reshape(-1, 3)
    # Unique vertices: round to 1e-6 to merge file-exact duplicates robustly.
    unique, _inverse = np.unique(np.round(flat, 6), axis=0, return_inverse=True)
    vertex_count = int(unique.shape[0])
    duplicate_records = int(flat.shape[0]) - vertex_count
    mins = unique.min(axis=0)
    maxs = unique.max(axis=0)
    bounds = {
        "min": [round(float(v), 6) for v in mins],
        "max": [round(float(v), 6) for v in maxs],
        "size": [round(float(mx - mn), 6) for mx, mn in zip(maxs, mins)],
    }
    v0 = triangles[:, 0, :]
    v1 = triangles[:, 1, :]
    v2 = triangles[:, 2, :]
    cross = np.cross(v1 - v0, v2 - v0)
    areas = 0.5 * np.sqrt(np.einsum("ij,ij->i", cross, cross))
    degenerate = int(np.count_nonzero(areas < 1e-12))
    if degenerate:
        issues.append(
            {
                "severity": "info",
                "code": f"{fmt}.degenerate_triangles",
                "message": f"{degenerate} degenerate triangle(s) with ~zero area (repeated or collinear vertices)",
            }
        )
    result: dict[str, Any] = {
        "format": fmt,
        "path": path,
        "vertices": vertex_count,
        "vertexRecords": int(flat.shape[0]),
        "triangles": count,
        "bounds": bounds,
        "degenerateTriangles": degenerate,
        "duplicateVertices": duplicate_records,
        "issues": issues,
    }
    if volume:
        signed = float(np.einsum("ij,ij->i", v0, cross).sum()) / 6.0
        result["volumeApprox"] = round(abs(signed), 6)
        result["signedVolume"] = round(signed, 6)
        result["note"] = (
            "未做非流形/水密性/自相交检测（需要完整拓扑分析）；volumeApprox 为有向四面体求和近似，"
            "仅对封闭且朝向一致的网格有意义"
        )
    else:
        result["note"] = "OBJ 不保证封闭；未做非流形/自相交检测"
    return result


def _inspect_stl(path: str) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    try:
        with open(path, "rb") as handle:
            head = handle.read(1024)
    except OSError as error:
        raise WorkerError(f"cannot read {path!r}: {error}") from error
    ascii_like = head[:5].strip().lower().startswith(b"solid") and b"facet" in head.lower()
    triangles = _try_binary_stl(path, head)
    mode = "binary"
    if triangles is None:
        triangles = _parse_stl_ascii(path, issues)
        mode = "ascii"
        if triangles is None:
            raise WorkerError(f"not a parseable STL file: {path!r} (neither valid binary nor ASCII)")
    result = _mesh_stats("stl", path, triangles, issues, volume=True)
    result["stlFormat"] = mode
    if mode == "ascii":
        result["note"] = "ASCII STL；未做非流形/水密性检测"
    return result


def _inspect_obj(path: str) -> dict[str, Any]:
    issues: list[dict[str, Any]] = []
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            lines = handle.readlines()
    except OSError as error:
        raise WorkerError(f"cannot read {path!r}: {error}") from error
    raw_vertices: list[list[float]] = []
    faces: list[tuple[int, int, int]] = []
    ngons = 0
    for lineno, line in enumerate(lines, 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        tag = parts[0]
        if tag == "v":
            if len(parts) < 4:
                issues.append(
                    {
                        "severity": "warning",
                        "code": "obj.bad_vertex",
                        "message": f"line {lineno}: vertex with fewer than 3 components",
                    }
                )
                continue
            try:
                raw_vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            except ValueError:
                issues.append(
                    {
                        "severity": "warning",
                        "code": "obj.bad_vertex",
                        "message": f"line {lineno}: non-numeric vertex components",
                    }
                )
        elif tag == "f":
            indices: list[int] = []
            for token in parts[1:]:
                first = token.split("/")[0]
                if not first:
                    continue
                try:
                    idx = int(first)
                except ValueError:
                    issues.append(
                        {
                            "severity": "warning",
                            "code": "obj.bad_face",
                            "message": f"line {lineno}: non-numeric face index {first!r}",
                        }
                    )
                    continue
                if idx < 0:
                    idx = len(raw_vertices) + idx + 1
                indices.append(idx - 1)
            if len(indices) >= 3:
                if len(indices) > 3:
                    ngons += 1
                # real fan triangulation: (v0,v1,v2), (v0,v2,v3), ... — the
                # old code emitted only the first triangle and silently
                # discarded vertices 4..n
                for k in range(1, len(indices) - 1):
                    faces.append((indices[0], indices[k], indices[k + 1]))
            else:
                issues.append(
                    {
                        "severity": "warning",
                        "code": "obj.bad_face",
                        "message": f"line {lineno}: face with fewer than 3 vertices",
                    }
                )
    if not raw_vertices:
        raise WorkerError(f"no vertices found in OBJ file: {path!r}")
    if ngons:
        issues.append(
            {
                "severity": "info",
                "code": "obj.ngon_fan",
                "message": f"{ngons} face(s) with more than 3 vertices were triangulated (fan)",
            }
        )
    vertices = np.array(raw_vertices, dtype=np.float64)
    triangles: list[np.ndarray] = []
    for i0, i1, i2 in faces:
        if max(i0, i1, i2) >= len(vertices):
            issues.append(
                {
                    "severity": "error",
                    "code": "obj.index_out_of_range",
                    "message": "face references a vertex index beyond the file's v lines",
                }
            )
            continue
        triangles.append(np.stack([vertices[i0], vertices[i1], vertices[i2]]))
    array = np.array(triangles, dtype=np.float64) if triangles else np.zeros((0, 3, 3), dtype=np.float64)
    result = _mesh_stats("obj", path, array, issues)
    result["vertexCount"] = len(raw_vertices)
    return result


def cmd_mesh_inspect(args: dict[str, Any]) -> dict[str, Any]:
    """Parse an STL (binary/ASCII) or OBJ mesh with numpy only."""
    path = _require_path(args)
    lowered = path.lower()
    if lowered.endswith(".stl"):
        result = _inspect_stl(path)
    elif lowered.endswith(".obj"):
        result = _inspect_obj(path)
    else:
        raise WorkerError(f"unsupported mesh format for {path!r}: expected .stl or .obj")
    result["ok"] = not any(i["severity"] == "error" for i in result["issues"])
    result["path"] = path
    result["inputArgs"] = {"path": path}
    return result


# ---------------------------------------------------------------------------
# 4. inertia-validate
# ---------------------------------------------------------------------------

def cmd_inertia_validate(args: dict[str, Any]) -> dict[str, Any]:
    """Validate URDF per-link inertials by reusing assets.inspect_urdf."""
    path = _require_path(args)
    try:
        inspection = inspect_urdf(path)
    except ValueError as error:
        raise WorkerError(str(error)) from error

    inertial_issues = [
        issue
        for issue in inspection.issues
        if issue.code.startswith("inertial.") or issue.code == "urdf.missing_inertial"
    ]
    by_link: dict[str, list] = {}
    for issue in inertial_issues:
        name: Optional[str] = None
        if issue.location.startswith("link/"):
            name = issue.location.split("/")[1]
        by_link.setdefault(name, []).append(issue)

    links_out: list[dict[str, Any]] = []
    total_mass = 0.0
    for entry in inspection.summary["links"]:
        name = entry["name"]
        inertial = entry.get("inertial")
        link_issues = [i.to_dict() for i in by_link.get(name, [])]
        if inertial is None:
            links_out.append(
                {"name": name, "mass": None, "inertia": None, "originXyz": None, "issues": link_issues}
            )
            continue
        mass = float(inertial["mass"])
        total_mass += mass
        links_out.append(
            {
                "name": name,
                "mass": round(mass, 6),
                "inertia": [[round(float(v), 9) for v in row] for row in inertial["inertia"]],
                "originXyz": [round(float(v), 6) for v in inertial["originXyz"]],
                "issues": link_issues,
            }
        )

    counts = {"error": 0, "warning": 0, "info": 0}
    for issue in inertial_issues:
        counts[issue.severity] = counts.get(issue.severity, 0) + 1
    if counts["error"] > 0:
        verdict = "error"
    elif counts["warning"] > 0:
        verdict = "warning"
    else:
        verdict = "ok"

    return {
        "ok": verdict != "error",
        "path": path,
        "links": links_out,
        "totalMass": round(total_mass, 6),
        "issueCounts": counts,
        "verdict": verdict,
        "note": "检查惯量正定性/量级/单位可疑项；基于静态解析，非真机测量",
        "inputArgs": {"path": path},
    }


# ---------------------------------------------------------------------------
# 5. robot-topology-validate
# ---------------------------------------------------------------------------

def cmd_robot_topology_validate(args: dict[str, Any]) -> dict[str, Any]:
    """Validate the URDF kinematic tree: single root, acyclic, no dangling joints."""
    path = _require_path(args)
    try:
        document = UrdfInspection(path)
    except ValueError as error:
        raise WorkerError(str(error)) from error

    links = document.link_names()
    joints = document.joints()
    issues: list[dict[str, Any]] = []

    parent_map = document.parent_map()
    child_map = document.child_map()
    roots = document.root_links()

    for ref in sorted(set(parent_map.values()) | set(child_map.values())):
        if ref and ref not in links:
            issues.append(
                {
                    "severity": "error",
                    "code": "topology.dangling_joint",
                    "message": f"joint references link {ref!r} which is not defined",
                }
            )

    if len(roots) == 0:
        issues.append(
            {"severity": "error", "code": "topology.no_root", "message": "no root link found (cycle or empty robot)"}
        )
        root_link: Optional[str] = None
    elif len(roots) > 1:
        issues.append(
            {
                "severity": "warning",
                "code": "topology.multiple_roots",
                "message": f"multiple root links: {', '.join(sorted(roots))}",
            }
        )
        root_link = sorted(roots)[0]
    else:
        root_link = roots[0]

    adjacency: dict[str, list[tuple[str, str, str]]] = {}
    in_degree: dict[str, int] = {}
    joint_list: list[dict[str, Any]] = []
    for joint in joints:
        jname = _attr(joint, "name")
        parent = _attr(joint.find("parent"), "link")
        child = _attr(joint.find("child"), "link")
        jtype = _attr(joint, "type", "unknown")
        if not parent or not child:
            issues.append(
                {
                    "severity": "error",
                    "code": "topology.joint_missing_ends",
                    "message": f"joint {jname!r} is missing parent or child link",
                }
            )
            continue
        adjacency.setdefault(parent, []).append((jname, child, jtype))
        in_degree[child] = in_degree.get(child, 0) + 1
        joint_list.append({"name": jname, "parent": parent, "child": child, "type": jtype})

    multi_parent = sorted(name for name, degree in in_degree.items() if degree > 1)
    if multi_parent:
        issues.append(
            {
                "severity": "warning",
                "code": "topology.multi_parent_link",
                "message": (
                    f"link(s) with more than one parent joint: {', '.join(multi_parent)} "
                    "(闭环机构或固定关节合并，需人工确认)"
                ),
            }
        )
    if len(links) > 0 and len(joint_list) != len(links) - 1:
        issues.append(
            {
                "severity": "info",
                "code": "topology.joint_count_mismatch",
                "message": (
                    f"joint count ({len(joint_list)}) != link count - 1 ({len(links) - 1}); "
                    "possible closed loop, disconnected components, or merged fixed joints"
                ),
            }
        )

    reachable: list[str] = []
    cycle_found = False
    if root_link:
        # iterative DFS: a recursive implementation overflows the interpreter
        # stack on deep auto-generated chains (> ~1000 links)
        visited: set[str] = set()
        stack: list[tuple[str, set[str]]] = [(root_link, set())]
        while stack:
            node, on_path = stack.pop()
            if node in on_path:
                cycle_found = True
                continue
            if node in visited:
                continue
            visited.add(node)
            for _jname, child, _jtype in adjacency.get(node, []):
                stack.append((child, on_path | {node}))
        reachable = sorted(visited)
        if cycle_found:
            issues.append(
                {"severity": "error", "code": "topology.cycle", "message": "cycle detected while walking from the root link"}
            )
        missing = sorted(links - visited)
        if missing:
            issues.append(
                {
                    "severity": "error",
                    "code": "topology.unreachable_links",
                    "message": f"links unreachable from root: {', '.join(missing)}",
                }
            )

    closed_loop = bool(multi_parent) or (len(links) > 0 and len(joint_list) != len(links) - 1)
    if closed_loop:
        note = "检测到闭环机构或固定关节合并（闭环/固定关节合并需人工确认）：普通 URDF 无法完整表达闭环约束"
    else:
        note = "树形拓扑正常（单根、无环、无悬空 joint）"

    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "path": path,
        "rootLink": root_link,
        "linkCount": len(links),
        "jointCount": len(joint_list),
        "reachableLinks": reachable,
        "issues": issues,
        "note": note,
        "inputArgs": {"path": path},
    }


# ---------------------------------------------------------------------------
# 6. urdf-preview
# ---------------------------------------------------------------------------

def _build_skeleton_svg(
    path: str,
    joint_entries: list[dict[str, Any]],
    positions: dict[str, tuple[float, float]],
    root_link: Optional[str],
) -> str:
    width, height = 800, 600
    margin = 44
    points = list(positions.values()) or [(0.0, 0.0)]
    xs = [p[0] for p in points]
    zs = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    scale = min((width - 2 * margin) / max(max_x - min_x, 1e-6), (height - 2 * margin) / max(max_z - min_z, 1e-6))

    def to_svg(x: float, z: float) -> tuple[float, float]:
        sx = margin + (x - min_x) * scale
        sy = height - margin - (z - min_z) * scale
        return sx, sy

    lines: list[str] = []
    for entry in joint_entries:
        parent = entry["parent"]
        child = entry["child"]
        if parent in positions and child in positions:
            x1, y1 = to_svg(*positions[parent])
            x2, y2 = to_svg(*positions[child])
            lines.append(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="#4a90d9" stroke-width="4"/>'
            )

    circles: list[str] = []
    labels: list[str] = []
    for entry in joint_entries:
        if entry["child"] not in positions:
            continue
        cx, cy = to_svg(*positions[entry["child"]])
        circles.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="6" fill="#ffffff" stroke="#222222" stroke-width="2"/>')
        labels.append(
            f'<text x="{cx:.2f}" y="{cy - 12:.2f}" text-anchor="middle" font-size="11" font-family="sans-serif">{_xml_escape(entry["name"])}</text>'
        )

    root_svg = ""
    if root_link:
        rx, ry = to_svg(0.0, 0.0)
        root_svg = (
            f'<circle cx="{rx:.2f}" cy="{ry:.2f}" r="5" fill="#e74c3c"/>'
            f'<text x="{rx:.2f}" y="{ry + 20:.2f}" text-anchor="middle" font-size="11" font-family="sans-serif">{_xml_escape(root_link)}</text>'
        )

    title = f"URDF kinematic skeleton (XZ projection) - {os.path.basename(path)}"
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">\n'
        f'  <rect width="{width}" height="{height}" fill="#fafafa"/>\n'
        f'  <text x="{width / 2:.0f}" y="24" text-anchor="middle" font-size="14" font-family="sans-serif" fill="#333333">{_xml_escape(title)}</text>\n'
        + "".join(f"  {line}\n" for line in lines)
        + "".join(f"  {circle}\n" for circle in circles)
        + f"  {root_svg}\n"
        + "".join(f"  {label}\n" for label in labels)
        + f'  <text x="{width - 12:.0f}" y="{height - 12:.0f}" text-anchor="end" font-size="10" font-family="sans-serif" fill="#888888">static 2D skeleton, joint angles at zero</text>\n'
        + "</svg>\n"
    )


def cmd_urdf_preview(args: dict[str, Any]) -> dict[str, Any]:
    """Generate a static 2D kinematic-chain SVG (XZ plane projection)."""
    path = _require_path(args)
    out_path = args.get("outPath")
    if not out_path:
        directory = os.path.dirname(path)
        base = os.path.splitext(os.path.basename(path))[0]
        out_path = os.path.join(directory, f"{base}.preview.svg")
    try:
        document = UrdfInspection(path)
    except ValueError as error:
        raise WorkerError(str(error)) from error

    joint_entries: list[dict[str, Any]] = []
    for joint in document.joints():
        origin = joint.find("origin")
        axis_el = joint.find("axis")
        joint_entries.append(
            {
                "name": _attr(joint, "name"),
                "type": _attr(joint, "type", "unknown"),
                "parent": _attr(joint.find("parent"), "link"),
                "child": _attr(joint.find("child"), "link"),
                "xyz": _vector(_attr(origin, "xyz")) if origin is not None else [0.0, 0.0, 0.0],
                "rpy": _vector(_attr(origin, "rpy")) if origin is not None else [0.0, 0.0, 0.0],
                "axis": _vector(_attr(axis_el, "xyz"), [1.0, 0.0, 0.0]) if axis_el is not None else None,
            }
        )

    roots = document.root_links()
    root_link = roots[0] if roots else None
    positions: dict[str, tuple[float, float]] = {}
    if root_link:
        positions[root_link] = (0.0, 0.0)
        by_parent: dict[str, list[dict[str, Any]]] = {}
        for entry in joint_entries:
            by_parent.setdefault(entry["parent"], []).append(entry)
        # (link, accumulated yaw of the parent frame). The old code hardcoded
        # theta=0, so any joint origin with a non-zero rpy rendered child
        # translations along unrotated axes; the yaw is now propagated down
        # the chain (2D XZ projection of the accumulated rotation).
        queue: list[tuple[str, float]] = [(root_link, 0.0)]
        index = 0
        while index < len(queue):
            node, theta = queue[index]
            index += 1
            if node not in positions:
                continue
            px, pz = positions[node]
            for entry in by_parent.get(node, []):
                child = entry["child"]
                if child in positions:
                    # cycle guard: never enqueue a link twice (the old BFS
                    # grew forever on cyclic URDFs)
                    continue
                ox, _oy, oz = entry["xyz"]
                dx = ox * math.cos(theta) + oz * math.sin(theta)
                dz = -ox * math.sin(theta) + oz * math.cos(theta)
                positions[child] = (px + dx, pz + dz)
                child_theta = theta + (entry["rpy"][2] if len(entry["rpy"]) > 2 else 0.0)
                queue.append((child, child_theta))

    svg = _build_skeleton_svg(path, joint_entries, positions, root_link)
    out_abs = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    with open(out_abs, "w", encoding="utf-8") as handle:
        handle.write(svg)

    joints_out: list[dict[str, Any]] = []
    for entry in joint_entries:
        item: dict[str, Any] = {
            "name": entry["name"],
            "type": entry["type"],
            "parent": entry["parent"],
            "child": entry["child"],
        }
        if entry["child"] in positions:
            item["x"] = round(positions[entry["child"]][0], 6)
            item["z"] = round(positions[entry["child"]][1], 6)
        joints_out.append(item)

    return {
        "ok": True,
        "path": path,
        "outPath": out_abs,
        "joints": joints_out,
        "note": "静态预览，不含 3D 渲染",
        "inputArgs": {"path": path, "outPath": out_path},
    }


# ---------------------------------------------------------------------------
# 7. export-sim-asset
# ---------------------------------------------------------------------------

def _sdf_compat_differences(path: str) -> list[dict[str, Any]]:
    """URDF -> SDF known differences, each tagged with presence in this file."""
    try:
        inspection = inspect_urdf(path)
    except ValueError as error:
        raise WorkerError(str(error)) from error
    summary = inspection.summary
    differences: list[dict[str, Any]] = []

    transmission_count = int(summary.get("transmissionCount", 0))
    gazebo_count = 0
    try:
        tree = ET.parse(path)
        gazebo_count = len(tree.getroot().findall("gazebo"))
    except ET.ParseError:
        pass
    differences.append(
        {
            "code": "sdf.transmission_gazebo",
            "title": "transmission / gazebo 标签",
            "detail": "URDF 的 <transmission> 与 <gazebo> 扩展在 SDF 中没有直接等价物，需转换为 <plugin>/硬件接口配置",
            "present": transmission_count > 0 or gazebo_count > 0,
            "count": transmission_count + gazebo_count,
        }
    )

    joints = summary.get("joints", [])
    rpy_offsets = [j for j in joints if any(abs(v) > 1e-9 for v in j.get("originRpy", [0.0, 0.0, 0.0]))]
    differences.append(
        {
            "code": "sdf.axis_frame",
            "title": "joint axis 坐标系约定",
            "detail": (
                "URDF 的 joint 原点/轴相对 parent link 帧；SDF 的 axis 相对 joint 帧且语义由 use_parent_model_frame 决定，"
                "带 rpy 的 joint 需人工核对"
            ),
            "present": len(rpy_offsets) > 0,
            "count": len(rpy_offsets),
        }
    )

    floating = [j["name"] for j in joints if j.get("type") == "floating"]
    differences.append(
        {
            "code": "sdf.floating_joint",
            "title": "floating joint",
            "detail": "URDF floating joint 在 SDF 中没有等价类型，需用 6-DOF 关节显式建模",
            "present": bool(floating),
            "count": len(floating),
        }
    )

    mimic = [j["name"] for j in joints if j.get("mimic")]
    differences.append(
        {
            "code": "sdf.mimic",
            "title": "mimic 关节",
            "detail": "URDF <mimic> 在 SDF 中需通过 joint 耦合或控制器实现，无直接等价",
            "present": bool(mimic),
            "count": len(mimic),
        }
    )

    inertial_offsets = [
        j
        for j in summary.get("links", [])
        if j.get("inertial") and any(abs(v) > 1e-9 for v in j["inertial"].get("originXyz", [0.0, 0.0, 0.0]))
    ]
    differences.append(
        {
            "code": "sdf.inertial_origin",
            "title": "inertial 原点/姿态",
            "detail": (
                "URDF 的 inertial origin 相对 link 帧；SDF 中为 <inertial><pose>。单位一致（kg、m、kg·m2）"
                "但需逐 link 核对数值与朝向"
            ),
            "present": len(inertial_offsets) > 0,
            "count": len(inertial_offsets),
        }
    )

    differences.append(
        {
            "code": "sdf.mesh_units",
            "title": "mesh 单位与 scale",
            "detail": "URDF 网格默认米制且无 scale；SDF 中 mesh 单位/scale 需显式设置，跨软件导出常引入单位偏差",
            "present": True,
            "count": None,
        }
    )
    return differences


def _write_compat_report(path: str, out_path: str, differences: list[dict[str, Any]]) -> None:
    if out_path.lower().endswith(".json"):
        payload = {
            "source": path,
            "generatedAt": _iso_time(time.time()),
            "target": "sdf",
            "differences": differences,
            "note": "未做自动 URDF->SDF 转换（需要 gazebo/ignition 工具链）；此报告仅列已知差异",
        }
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        return
    lines = [
        "# URDF → SDF 兼容性检查报告",
        "",
        f"- 源文件：`{path}`",
        f"- 生成时间：{_iso_time(time.time())}",
        "- 工具：Robotic Harness worker（纯静态检查，未调用 gazebo/ignition）",
        "",
        "> 本报告仅列出 URDF→SDF 的**已知差异**，不执行转换；实际转换需要 gazebo/ignition 工具链。",
        "",
        "## 已知差异",
        "",
        "| # | 差异 | 说明 | 存在于本文件 |",
        "|---|------|------|--------------|",
    ]
    for index, diff in enumerate(differences, 1):
        present = "是" if diff.get("present") else "否"
        lines.append(f"| {index} | {diff['title']} | {diff['detail']} | {present} |")
    lines += [
        "",
        "## 建议",
        "",
        "- 转换前先展开 xacro（`xacro --inorder file.xacro > file.urdf`）。",
        "- 转换后重点核对 joint 轴方向、inertial 原点与 mesh 单位/scale。",
        "- 用 gazebo/ignition 的 SDF 校验工具验证转换结果。",
    ]
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def _convert_urdf_to_mjcf(urdf_path: str, out_path: str) -> dict[str, Any]:
    """URDF -> MJCF conversion reusing assets.convert_urdf_to_mjcf.

    Newer mujoco releases (>= 3.1) removed ``MjModel.get_xml()``, which the
    shared converter relies on; when that happens we fall back to an inline
    conversion through ``mj_saveLastXML`` so the command still works.
    """
    try:
        return convert_urdf_to_mjcf(urdf_path, out_path)
    except AttributeError:
        pass  # mujoco removed MjModel.get_xml(); use the compat path below
    import contextlib
    import io

    import mujoco  # noqa: PLC0415

    buffer = io.StringIO()
    with contextlib.redirect_stderr(buffer):
        model = mujoco.MjModel.from_xml_path(urdf_path)
    warnings = [line.strip() for line in buffer.getvalue().splitlines() if line.strip()]
    out_abs = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    mujoco.mj_saveLastXML(out_abs, model)
    return {
        "ok": True,
        "source": urdf_path,
        "target": out_abs,
        "compiler": f"mujoco {getattr(mujoco, '__version__', 'unknown')} (mj_saveLastXML fallback)",
        "loaderWarnings": warnings,
        "differences": [
            "URDF <transmission> and <gazebo> extensions are not preserved by the MuJoCo compiler",
            "visual/collision semantics follow MuJoCo defaults unless the generated MJCF is tuned",
            "inertial origins are normalized to body frames by the MuJoCo compiler",
        ],
        "advice": "review the generated MJCF before use in experiments; auto-conversion does not replace human review",
    }


def cmd_export_sim_asset(args: dict[str, Any]) -> dict[str, Any]:
    """Export a robot asset for simulation: mjcf conversion or sdf-compat report."""
    path = _require_path(args)
    target = str(args.get("target", "mjcf")).lower()
    out_path = args.get("outPath")

    if target == "mjcf":
        if not out_path:
            raise WorkerError("missing required argument 'outPath' for mjcf target")
        try:
            report = _convert_urdf_to_mjcf(path, out_path)
        except Exception as error:
            raise WorkerError(f"MJCF conversion failed: {error}") from error
        return {
            "ok": True,
            "target": "mjcf",
            "source": path,
            "outPath": os.path.abspath(out_path),
            "differences": report.get("differences", []),
            "compiler": report.get("compiler"),
            "loaderWarnings": report.get("loaderWarnings", []),
            "note": "URDF->MJCF 通过 MuJoCo 编译器完成；转换不保留 transmission/gazebo 扩展，需人工复核生成的 MJCF",
        }

    if target == "sdf-compat":
        differences = _sdf_compat_differences(path)
        if not out_path:
            out_path = os.path.join(
                os.path.dirname(path), os.path.splitext(os.path.basename(path))[0] + ".sdf-compat.md"
            )
        out_abs = os.path.abspath(out_path)
        os.makedirs(os.path.dirname(out_abs), exist_ok=True)
        _write_compat_report(path, out_abs, differences)
        return {
            "ok": True,
            "target": "sdf-compat",
            "source": path,
            "outPath": out_abs,
            "differences": differences,
            "note": "未做自动 URDF->SDF 转换（需要 gazebo/ignition 工具链）；仅生成兼容性检查报告",
            "inputArgs": {"path": path, "target": target, "outPath": out_path},
        }

    raise WorkerError(f"unsupported target {target!r}: expected 'mjcf' or 'sdf-compat'")


# ---------------------------------------------------------------------------
# 8. asset-report
# ---------------------------------------------------------------------------

def _scalar_summary(summary: dict[str, Any]) -> dict[str, Any]:
    """Keep only JSON-scalar / short-list summary entries for the Markdown table."""
    out: dict[str, Any] = {}
    for key, value in summary.items():
        if isinstance(value, (str, int, float, bool)) or value is None:
            out[key] = value
        elif isinstance(value, dict) and all(
            isinstance(k, (str, int, float, bool)) and isinstance(v, (str, int, float, bool)) for k, v in value.items()
        ):
            out[key] = value
        elif isinstance(value, list) and len(value) <= 32 and all(
            isinstance(v, (str, int, float, bool)) or v is None for v in value
        ):
            out[key] = value
    return out


def _render_asset_report(
    path: str,
    fmt: str,
    summary: dict[str, Any],
    issues: list[dict[str, Any]],
    counts: dict[str, int],
) -> str:
    lines = [
        f"# 资产检查报告（{fmt.upper()}）",
        "",
        f"- 文件：`{path}`",
        f"- 生成时间：{_iso_time(time.time())}",
        "",
        "## 摘要",
        "",
        "| 字段 | 值 |",
        "|------|-----|",
    ]
    for key, value in summary.items():
        lines.append(f"| {key} | {_md_escape(str(value))} |")
    lines += ["", "## 问题", ""]
    if issues:
        lines += ["| 级别 | 代码 | 位置 | 说明 |", "|------|------|------|------|"]
        for issue in issues:
            lines.append(
                f"| {issue['severity']} | `{issue['code']}` | {_md_escape(str(issue.get('location') or '-'))} | {_md_escape(issue['message'])} |"
            )
    else:
        lines.append("_未发现问题。_")
    lines += ["", "## 建议", ""]
    if counts.get("error"):
        codes = sorted({i["code"] for i in issues if i["severity"] == "error"})
        lines.append(f"- 先修复 {counts['error']} 个 error 级别问题（`{'、'.join(codes)}`）。")
    else:
        lines.append("- 无 error 级别问题；建议对 warning/info 项逐条人工确认。")
    lines += [
        "",
        "> **声明**：检查结果不构成仿真就绪/真机安全证明。本报告基于静态解析与启发式规则，"
        "实际行为须经仿真器加载与真机安全验证。",
    ]
    return "\n".join(lines) + "\n"


def cmd_asset_report(args: dict[str, Any]) -> dict[str, Any]:
    """Generate a Markdown asset inspection report (URDF/MJCF)."""
    path = _require_path(args)
    out_path = args.get("outPath")
    if not out_path:
        out_path = os.path.join(
            os.path.dirname(path), os.path.splitext(os.path.basename(path))[0] + ".report.md"
        )
    try:
        inspection = inspect_asset(path)
    except ValueError as error:
        raise WorkerError(str(error)) from error

    issues = [i.to_dict() for i in inspection.issues]
    counts = {"error": 0, "warning": 0, "info": 0}
    for issue in issues:
        counts[issue["severity"]] = counts.get(issue["severity"], 0) + 1

    summary = _scalar_summary(inspection.summary)
    markdown = _render_asset_report(path, inspection.format, summary, issues, counts)
    out_abs = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    with open(out_abs, "w", encoding="utf-8") as handle:
        handle.write(markdown)

    return {
        "ok": counts["error"] == 0,
        "path": path,
        "outPath": out_abs,
        "format": inspection.format,
        "issueCounts": counts,
        "inputArgs": {"path": path, "outPath": out_path},
    }


# ---------------------------------------------------------------------------
# module exports (worker module contract)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Any] = {
    "cad-inventory": cmd_cad_inventory,
    "cad-compare-versions": cmd_cad_compare_versions,
    "mesh-inspect": cmd_mesh_inspect,
    "inertia-validate": cmd_inertia_validate,
    "robot-topology-validate": cmd_robot_topology_validate,
    "urdf-preview": cmd_urdf_preview,
    "export-sim-asset": cmd_export_sim_asset,
    "asset-report": cmd_asset_report,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "cad.inventory",
        "kind": "cad",
        "provider": "robotic-harness-worker",
        "input": {"path": "string", "recursive": "boolean?", "formats": "array?"},
        "output": "CAD asset inventory with formats, sizes, SHA-256 and issues",
        "risk": "R0-readonly",
        "description": "Scan a CAD asset tree (STEP/STL/OBJ/DAE/URDF/XACRO/IGES/FCSTD/SolidWorks): formats, hashes, zero-byte/unreadable/duplicate-name issues. SolidWorks files are registered but never parsed; STEP uses the optional FreeCAD backend when available.",
    },
    {
        "id": "cad.mesh_inspect",
        "kind": "cad",
        "provider": "robotic-harness-worker",
        "input": {"path": "string"},
        "output": "mesh statistics: vertices, triangles, bounds, volume, degenerate count",
        "risk": "R0-readonly",
        "description": "Parse binary/ASCII STL and OBJ meshes with numpy only: bounds, signed-volume approximation, degenerate-triangle detection, duplicate vertices.",
    },
    {
        "id": "cad.inertia_validate",
        "kind": "cad",
        "provider": "robotic-harness-worker",
        "input": {"path": "string"},
        "output": "per-link inertia validation with verdict",
        "risk": "R0-readonly",
        "description": "Validate URDF per-link inertials: positive-definiteness, mass magnitude and unit sanity; returns per-link issues and a verdict.",
    },
    {
        "id": "cad.robot_topology_validate",
        "kind": "cad",
        "provider": "robotic-harness-worker",
        "input": {"path": "string"},
        "output": "kinematic tree verdict: root, reachability, closed-loop hints",
        "risk": "R0-readonly",
        "description": "Validate the URDF kinematic tree: single root, acyclic DFS reachability, no dangling joints, and closed-loop/fixed-joint-merge hints that plain URDF cannot fully express.",
    },
]
