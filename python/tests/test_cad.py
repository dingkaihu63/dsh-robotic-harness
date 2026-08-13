"""Tests for the CAD / mechanical modeling worker module (cad.py)."""

from __future__ import annotations

import json
import os
import struct

import numpy as np
import pytest

from robotic_harness_worker import cad
from robotic_harness_worker.core import WorkerError

FIXTURES = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "fixtures",
    "robot_assets",
)
GOOD_URDF = os.path.join(FIXTURES, "rh_arm.urdf")
BROKEN_URDF = os.path.join(FIXTURES, "rh_arm_broken.urdf")


def _write_binary_stl(path, triangles, header=b"rh tetra"):
    """Write triangles (N,3,3) as a binary STL with outward normals."""
    triangles = np.asarray(triangles, dtype=np.float64)
    with open(path, "wb") as handle:
        handle.write(header.ljust(80, b"\0")[:80])
        handle.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            handle.write(np.asarray(normal, dtype="<f4").tobytes())
            handle.write(np.asarray(tri, dtype="<f4").tobytes())
            handle.write(struct.pack("<H", 0))


# ---------------------------------------------------------------------------
# mesh-inspect
# ---------------------------------------------------------------------------

def test_mesh_inspect_binary_stl_tetrahedron(tmp_path):
    path = tmp_path / "tetra.stl"
    a, b, c, d = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    triangles = np.array(
        [
            [b, c, d],  # face opposite a, outward
            [a, d, c],  # face opposite b, outward
            [a, b, d],  # face opposite c, outward
            [a, c, b],  # face opposite d, outward
        ],
        dtype=np.float64,
    )
    _write_binary_stl(str(path), triangles)
    result = cad.cmd_mesh_inspect({"path": str(path)})
    assert result["ok"] is True
    assert result["format"] == "stl"
    assert result["vertices"] == 4
    assert result["triangles"] == 4
    assert result["bounds"]["min"] == [0.0, 0.0, 0.0]
    assert result["bounds"]["max"] == [1.0, 1.0, 1.0]
    assert result["bounds"]["size"] == [1.0, 1.0, 1.0]
    # unit tetrahedron volume = 1/6; signed tetrahedron sum / 6
    assert result["volumeApprox"] == pytest.approx(1.0 / 6.0, abs=1e-6)
    assert result["degenerateTriangles"] == 0


def test_mesh_inspect_stl_degenerate_triangle(tmp_path):
    path = tmp_path / "deg.stl"
    triangles = np.array(
        [
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],  # repeated vertex -> zero area
        ],
        dtype=np.float64,
    )
    _write_binary_stl(str(path), triangles)
    result = cad.cmd_mesh_inspect({"path": str(path)})
    assert result["triangles"] == 2
    assert result["degenerateTriangles"] >= 1


def test_mesh_inspect_ascii_stl(tmp_path):
    path = tmp_path / "ascii.stl"
    path.write_text(
        "solid test\n"
        "  facet normal 0 0 1\n"
        "    outer loop\n"
        "      vertex 0 0 0\n"
        "      vertex 1 0 0\n"
        "      vertex 0 1 0\n"
        "    endloop\n"
        "  endfacet\n"
        "endsolid test\n",
        encoding="utf-8",
    )
    result = cad.cmd_mesh_inspect({"path": str(path)})
    assert result["ok"] is True
    assert result["triangles"] == 1
    assert result["vertices"] == 3
    assert result["stlFormat"] == "ascii"


def test_mesh_inspect_obj(tmp_path):
    path = tmp_path / "tri.obj"
    path.write_text(
        "# simple triangle obj\n"
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "v 0 0 1\n"
        "vt 0 0\n"
        "vn 0 0 1\n"
        "f 1/1/1 2/1/1 3/1/1\n"  # v/vt/vn form
        "f 1//1 3//1 4//1\n"  # v//vn form
        "f 4 2 1\n",  # plain index form
        encoding="utf-8",
    )
    result = cad.cmd_mesh_inspect({"path": str(path)})
    assert result["ok"] is True
    assert result["format"] == "obj"
    assert result["vertices"] == 4
    assert result["triangles"] == 3
    assert result["bounds"]["min"] == [0.0, 0.0, 0.0]
    assert result["bounds"]["max"] == [1.0, 1.0, 1.0]


def test_mesh_inspect_unsupported_format(tmp_path):
    path = tmp_path / "notes.txt"
    path.write_text("hello", encoding="utf-8")
    with pytest.raises(WorkerError):
        cad.cmd_mesh_inspect({"path": str(path)})


def test_mesh_inspect_garbage_stl(tmp_path):
    path = tmp_path / "garbage.stl"
    path.write_bytes(b"\x00" * 200)
    with pytest.raises(WorkerError):
        cad.cmd_mesh_inspect({"path": str(path)})


# ---------------------------------------------------------------------------
# cad-inventory
# ---------------------------------------------------------------------------

def test_cad_inventory_counts_and_solidworks_note(tmp_path):
    parts = tmp_path / "parts"
    parts.mkdir()
    (parts / "a.step").write_bytes(
        b"ISO-10303-21;\nHEADER;\n"
        b"FILE_DESCRIPTION(('demo'),'2;1');\n"
        b"FILE_NAME('a.step','2024-01-01T00:00:00',('u'),('o'),'prep','sys','authoring');\n"
        b"FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n"
        b"ENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"
    )
    (parts / "b.stl").write_bytes(b"solid\nendsolid\n")
    (parts / "c.urdf").write_text("<robot name='r'/>", encoding="utf-8")
    (parts / "d.sldprt").write_bytes(b"\x00\x01binary")
    result = cad.cmd_cad_inventory({"path": str(parts)})
    assert result["ok"] is True
    counts = result["counts"]["byFormat"]
    assert counts["step"] == 1
    assert counts["stl"] == 1
    assert counts["urdf"] == 1
    assert counts["sldprt"] == 1
    assert result["counts"]["totalFiles"] == 4
    assert result["totalSize"] == sum(f["size"] for f in result["files"])
    sld = next(f for f in result["files"] if f["format"] == "sldprt")
    assert "SolidWorks" in sld["note"]
    step = next(f for f in result["files"] if f["format"] == "step")
    assert step["step"]["parse"] in ("header-only", "freecad")
    assert step["step"]["name"] == "a.step"
    assert step["step"]["schema"]
    assert result["solidWorksFiles"] == 1
    # every entry has the contract fields
    for entry in result["files"]:
        assert {"path", "format", "size", "sha256", "modifiedAt"} <= set(entry)


def test_cad_inventory_formats_filter_and_zero_byte(tmp_path):
    (tmp_path / "x.stl").write_bytes(b"solid\nendsolid\n")
    (tmp_path / "y.obj").write_bytes(b"v 0 0 0\n")
    (tmp_path / "z.urdf").write_text("<robot/>", encoding="utf-8")
    (tmp_path / "empty.stl").write_bytes(b"")
    result = cad.cmd_cad_inventory({"path": str(tmp_path), "formats": [".stl", ".obj"]})
    formats = {f["format"] for f in result["files"]}
    assert formats == {"stl", "obj"}
    codes = {i["code"] for i in result["issues"]}
    assert "inventory.zero_byte" in codes


def test_cad_inventory_duplicate_names(tmp_path):
    (tmp_path / "x").mkdir()
    (tmp_path / "y").mkdir()
    (tmp_path / "x" / "m.stl").write_bytes(b"solid\nendsolid\n")
    (tmp_path / "y" / "m.stl").write_bytes(b"solid\nendsolid\n")
    result = cad.cmd_cad_inventory({"path": str(tmp_path)})
    codes = {i["code"] for i in result["issues"]}
    assert "inventory.duplicate_name" in codes


def test_cad_inventory_missing_path():
    with pytest.raises(WorkerError):
        cad.cmd_cad_inventory({"path": "Z:/does/not/exist"})
    with pytest.raises(WorkerError):
        cad.cmd_cad_inventory({"path": "Z:/does/not/exist", "formats": [".bogus"]})


# ---------------------------------------------------------------------------
# cad-compare-versions
# ---------------------------------------------------------------------------

_URDF_A = (
    '<robot name="r">'
    '<link name="base"><inertial><mass value="1.0"/>'
    '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>'
    '<link name="l1"><inertial><mass value="2.0"/>'
    '<inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/></inertial></link>'
    '<joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>'
    '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'
    "</robot>"
)

_URDF_B = (
    '<robot name="r">'
    '<link name="base"><inertial><mass value="1.0"/>'
    '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>'
    '<link name="l1"><inertial><mass value="2.4"/>'
    '<inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/></inertial></link>'
    '<link name="l2"><inertial><mass value="0.5"/>'
    '<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>'
    '<joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>'
    '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'
    '<joint name="j2" type="fixed"><parent link="l1"/><child link="l2"/></joint>'
    "</robot>"
)


def test_cad_compare_versions_urdf(tmp_path):
    a = tmp_path / "a.urdf"
    b = tmp_path / "b.urdf"
    a.write_text(_URDF_A, encoding="utf-8")
    b.write_text(_URDF_B, encoding="utf-8")
    result = cad.cmd_cad_compare_versions({"pathA": str(a), "pathB": str(b)})
    assert result["ok"] is True
    assert result["kind"] == "urdf"
    assert result["summary"]["links"]["added"] == ["l2"]
    assert result["summary"]["joints"]["added"] == ["j2"]
    mass_changed = {m["link"]: m for m in result["summary"]["links"]["massChanged"]}
    assert "l1" in mass_changed  # 2.0 -> 2.4 = +20% > 10%
    assert mass_changed["l1"]["massA"] == 2.0
    assert mass_changed["l1"]["massB"] == 2.4
    assert result["rawDiff"]["a"]["links"]["base"]["mass"] == 1.0


def test_cad_compare_versions_inventory_directories(tmp_path):
    dir_a = tmp_path / "a"
    dir_b = tmp_path / "b"
    dir_a.mkdir()
    dir_b.mkdir()
    (dir_a / "m.stl").write_bytes(b"solid x\nendsolid\n")
    (dir_b / "m.stl").write_bytes(b"solid x CHANGED\nendsolid\n")
    (dir_b / "n.stl").write_bytes(b"solid n\nendsolid\n")
    result = cad.cmd_cad_compare_versions({"pathA": str(dir_a), "pathB": str(dir_b)})
    assert result["ok"] is True
    assert result["kind"] == "inventory"
    assert result["summary"]["added"] == ["n.stl"]
    changed = [c for c in result["summary"]["changed"] if c["path"] == "m.stl"]
    assert len(changed) == 1
    assert result["summary"]["unchangedCount"] == 0


def test_cad_compare_versions_inventory_snapshots(tmp_path):
    snap_a = tmp_path / "a.json"
    snap_b = tmp_path / "b.json"
    snap_a.write_text(
        json.dumps(
            {
                "root": str(tmp_path / "root"),
                "files": [{"path": str(tmp_path / "root" / "m.stl"), "format": "stl", "size": 3, "sha256": "aa"}],
            }
        ),
        encoding="utf-8",
    )
    snap_b.write_text(
        json.dumps(
            {
                "root": str(tmp_path / "root"),
                "files": [
                    {"path": str(tmp_path / "root" / "m.stl"), "format": "stl", "size": 3, "sha256": "bb"},
                    {"path": str(tmp_path / "root" / "n.stl"), "format": "stl", "size": 3, "sha256": "cc"},
                ],
            }
        ),
        encoding="utf-8",
    )
    result = cad.cmd_cad_compare_versions({"pathA": str(snap_a), "pathB": str(snap_b)})
    assert result["kind"] == "inventory"
    assert result["summary"]["added"] == ["n.stl"]
    assert [c["path"] for c in result["summary"]["changed"]] == ["m.stl"]


def test_cad_compare_versions_mixed(tmp_path):
    a = tmp_path / "a.urdf"
    b = tmp_path / "b.json"
    a.write_text("<robot/>", encoding="utf-8")
    b.write_text("{}", encoding="utf-8")
    with pytest.raises(WorkerError):
        cad.cmd_cad_compare_versions({"pathA": str(a), "pathB": str(b)})


# ---------------------------------------------------------------------------
# inertia-validate
# ---------------------------------------------------------------------------

def test_inertia_validate_good_urdf():
    result = cad.cmd_inertia_validate({"path": GOOD_URDF})
    assert result["ok"] is True
    assert result["verdict"] == "ok"
    assert result["totalMass"] == pytest.approx(3.25, abs=1e-6)
    names = [link["name"] for link in result["links"]]
    assert names == ["base_link", "link1", "link2", "link3", "cup"]
    base = next(link for link in result["links"] if link["name"] == "base_link")
    assert base["mass"] == pytest.approx(1.0)
    assert base["inertia"][0][0] == pytest.approx(0.001)
    assert base["originXyz"] == [0.0, 0.0, 0.06]
    assert base["issues"] == []


def test_inertia_validate_broken_urdf():
    result = cad.cmd_inertia_validate({"path": BROKEN_URDF})
    assert result["ok"] is False
    assert result["verdict"] == "error"
    assert result["issueCounts"]["error"] >= 1
    all_codes = {i["code"] for link in result["links"] for i in link["issues"]}
    assert "inertial.mass_non_positive" in all_codes
    assert "inertial.zero" in all_codes


def test_inertia_validate_missing_path():
    with pytest.raises(WorkerError):
        cad.cmd_inertia_validate({"path": "Z:/nope.urdf"})


# ---------------------------------------------------------------------------
# robot-topology-validate
# ---------------------------------------------------------------------------

def test_robot_topology_validate_good():
    result = cad.cmd_robot_topology_validate({"path": GOOD_URDF})
    assert result["ok"] is True
    assert result["rootLink"] == "base_link"
    assert result["linkCount"] == 5
    assert result["jointCount"] == 4
    assert len(result["reachableLinks"]) == 5
    assert result["note"] is not None
    assert "闭环" not in result["note"]


def test_robot_topology_validate_closed_loop(tmp_path):
    path = tmp_path / "loop.urdf"
    path.write_text(
        '<robot name="loop">'
        '<link name="a"/><link name="b"/><link name="c"/>'
        '<joint name="j1" type="revolute"><parent link="a"/><child link="b"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'
        '<joint name="j2" type="revolute"><parent link="b"/><child link="c"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'
        '<joint name="j3" type="fixed"><parent link="a"/><child link="c"/></joint>'
        "</robot>",
        encoding="utf-8",
    )
    result = cad.cmd_robot_topology_validate({"path": str(path)})
    codes = {i["code"] for i in result["issues"]}
    assert "topology.multi_parent_link" in codes  # link c has two parents
    assert "topology.joint_count_mismatch" in codes
    assert "闭环" in (result["note"] or "")


def test_robot_topology_validate_dangling_joint(tmp_path):
    path = tmp_path / "dangling.urdf"
    path.write_text(
        '<robot name="d"><link name="a"/>'
        '<joint name="j" type="fixed"><parent link="a"/><child link="missing"/></joint>'
        "</robot>",
        encoding="utf-8",
    )
    result = cad.cmd_robot_topology_validate({"path": str(path)})
    assert result["ok"] is False
    codes = {i["code"] for i in result["issues"]}
    assert "topology.dangling_joint" in codes


# ---------------------------------------------------------------------------
# urdf-preview
# ---------------------------------------------------------------------------

def test_urdf_preview_svg(tmp_path):
    out = tmp_path / "preview.svg"
    result = cad.cmd_urdf_preview({"path": GOOD_URDF, "outPath": str(out)})
    assert result["ok"] is True
    assert os.path.exists(result["outPath"])
    svg = out.read_text(encoding="utf-8")
    assert "<svg" in svg
    assert "shoulder" in svg
    assert "elbow" in svg
    joint_names = [j["name"] for j in result["joints"]]
    assert joint_names == ["base_to_link1", "shoulder", "elbow", "wrist_joint"]
    assert all("x" in j and "z" in j for j in result["joints"])
    assert result["note"] == "静态预览，不含 3D 渲染"


def test_urdf_preview_default_outpath(tmp_path):
    # copy the good URDF into tmp so the default sibling path is writable
    target = tmp_path / "arm.urdf"
    with open(GOOD_URDF, encoding="utf-8") as handle:
        target.write_text(handle.read(), encoding="utf-8")
    result = cad.cmd_urdf_preview({"path": str(target)})
    assert os.path.exists(result["outPath"])
    assert result["outPath"].endswith("arm.preview.svg")


def test_urdf_preview_bad_urdf(tmp_path):
    path = tmp_path / "bad.urdf"
    path.write_text("<not-robot/>", encoding="utf-8")
    with pytest.raises(WorkerError):
        cad.cmd_urdf_preview({"path": str(path)})


# ---------------------------------------------------------------------------
# export-sim-asset
# ---------------------------------------------------------------------------

def test_export_sim_asset_sdf_compat(tmp_path):
    out = tmp_path / "compat.md"
    result = cad.cmd_export_sim_asset({"path": GOOD_URDF, "target": "sdf-compat", "outPath": str(out)})
    assert result["ok"] is True
    assert result["target"] == "sdf-compat"
    assert result["source"] == os.path.abspath(GOOD_URDF)
    assert os.path.exists(result["outPath"])
    content = out.read_text(encoding="utf-8")
    assert "URDF" in content and "SDF" in content
    assert isinstance(result["differences"], list)
    assert len(result["differences"]) >= 3
    codes = {d["code"] for d in result["differences"]}
    assert "sdf.mesh_units" in codes


def test_export_sim_asset_sdf_compat_json(tmp_path):
    out = tmp_path / "compat.json"
    result = cad.cmd_export_sim_asset({"path": GOOD_URDF, "target": "sdf-compat", "outPath": str(out)})
    assert os.path.exists(result["outPath"])
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["target"] == "sdf"
    assert len(payload["differences"]) >= 3


def test_export_sim_asset_mjcf(tmp_path):
    pytest.importorskip("mujoco")
    out = tmp_path / "converted.mjcf"
    result = cad.cmd_export_sim_asset({"path": GOOD_URDF, "target": "mjcf", "outPath": str(out)})
    assert result["ok"] is True
    assert result["target"] == "mjcf"
    assert os.path.exists(result["outPath"])
    assert isinstance(result["differences"], list)


def test_export_sim_asset_bad_target(tmp_path):
    with pytest.raises(WorkerError):
        cad.cmd_export_sim_asset({"path": GOOD_URDF, "target": "bogus"})


def test_export_sim_asset_mjcf_missing_outpath():
    with pytest.raises(WorkerError):
        cad.cmd_export_sim_asset({"path": GOOD_URDF, "target": "mjcf"})


# ---------------------------------------------------------------------------
# asset-report
# ---------------------------------------------------------------------------

def test_asset_report_generates_md(tmp_path):
    out = tmp_path / "report.md"
    result = cad.cmd_asset_report({"path": GOOD_URDF, "outPath": str(out)})
    assert result["ok"] is True
    assert result["format"] == "urdf"
    assert result["issueCounts"]["error"] == 0
    assert os.path.exists(result["outPath"])
    content = out.read_text(encoding="utf-8")
    assert "检查结果不构成仿真就绪/真机安全证明" in content
    assert "rh_arm" in content
    assert "## 摘要" in content and "## 问题" in content


def test_asset_report_default_outpath(tmp_path):
    target = tmp_path / "arm.urdf"
    with open(GOOD_URDF, encoding="utf-8") as handle:
        target.write_text(handle.read(), encoding="utf-8")
    result = cad.cmd_asset_report({"path": str(target)})
    assert result["outPath"].endswith("arm.report.md")
    assert os.path.exists(result["outPath"])


def test_asset_report_unsupported_format(tmp_path):
    path = tmp_path / "x.step"
    path.write_bytes(b"ISO-10303-21;")
    with pytest.raises(WorkerError):
        cad.cmd_asset_report({"path": str(path)})


# ---------------------------------------------------------------------------
# module exports
# ---------------------------------------------------------------------------

def test_module_exports():
    assert "cad-inventory" in cad.COMMANDS
    assert "cad-compare-versions" in cad.COMMANDS
    assert "mesh-inspect" in cad.COMMANDS
    assert "inertia-validate" in cad.COMMANDS
    assert "robot-topology-validate" in cad.COMMANDS
    assert "urdf-preview" in cad.COMMANDS
    assert "export-sim-asset" in cad.COMMANDS
    assert "asset-report" in cad.COMMANDS
    assert 3 <= len(cad.CAPABILITIES) <= 4
    for capability in cad.CAPABILITIES:
        assert capability["id"]
        assert capability["kind"] == "cad"
        assert capability["risk"] == "R0-readonly"
