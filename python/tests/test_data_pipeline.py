"""Tests for the multimodal data pipeline module (plan chapter 14).

Run from the ``python`` directory::

    python -m pytest tests/test_data_pipeline.py -q

Fixtures are generated inside ``tmp_path`` (no repo fixtures needed except the
optional cv2/PIL environment checks). ``cv2`` and ``PIL`` are optional: tests
that need them skip when they are missing.
"""

from __future__ import annotations

import csv
import json
import math
import os
import sqlite3
import struct

import pytest

from robotic_harness_worker import data_pipeline as dp
from robotic_harness_worker.core import WorkerError


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def write_csv(path, header, rows):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def read_csv_rows(path):
    with open(path, encoding="utf-8", newline="") as handle:
        return list(csv.reader(handle))


def write_jsonl(path, rows):
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")


# ---------------------------------------------------------------------------
# 1. data-inventory
# ---------------------------------------------------------------------------

def test_inventory_directory_scan(tmp_path):
    bag = tmp_path / "bag"
    (bag / "sub").mkdir(parents=True)
    (bag / "a.csv").write_text("t,x\n0,1\n", encoding="utf-8")
    (bag / "b.jsonl").write_text('{"t": 0}\n', encoding="utf-8")
    (bag / "c.weird").write_text("hello", encoding="utf-8")
    (bag / "sub" / "d.png").write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00" * 24)

    result = dp.cmd_data_inventory({"path": str(bag)})
    assert result["ok"] is True
    names = {os.path.basename(f["path"]) for f in result["files"]}
    assert names == {"a.csv", "b.jsonl", "c.weird", "d.png"}
    formats = {f["format"] for f in result["files"]}
    assert formats == {"csv", "jsonl", "unknown", "png"}
    for entry in result["files"]:
        assert len(entry["sha256"]) == 64
        assert entry["size"] > 0
    assert result["formats"]["csv"] == 1
    assert result["formats"]["png"] == 1
    assert result["totalSize"] == sum(f["size"] for f in result["files"])
    assert result["issues"] == []

    # non-recursive scan skips the subdirectory
    shallow = dp.cmd_data_inventory({"path": str(bag), "recursive": False})
    assert len(shallow["files"]) == 3


def test_inventory_single_file_and_corruption(tmp_path):
    good = tmp_path / "good.json"
    good.write_text('{"t": 1}', encoding="utf-8")
    result = dp.cmd_data_inventory({"path": str(good)})
    assert result["ok"] is True
    assert len(result["files"]) == 1
    assert result["files"][0]["format"] == "json"

    bad = tmp_path / "bad.json"
    bad.write_text("{not json", encoding="utf-8")
    result = dp.cmd_data_inventory({"path": str(bad)})
    codes = {i["code"] for i in result["issues"]}
    assert "file.corrupt" in codes


# ---------------------------------------------------------------------------
# 2. data-schema-inspect
# ---------------------------------------------------------------------------

def test_schema_inspect_dtype_inference(tmp_path):
    path = tmp_path / "s.csv"
    write_csv(
        path,
        ["t", "x", "label", "flag", "empty"],
        [
            ["0", "1", "a", "true", ""],
            ["1", "2.5", "b", "false", ""],
            ["2", "zzz", "c", "true", ""],
            ["3", "", "d", "", ""],
        ],
    )
    result = dp.cmd_data_schema_inspect({"path": str(path)})
    assert result["ok"] is True
    assert result["format"] == "csv"
    assert result["rows"] == 4
    by_name = {c["name"]: c for c in result["columns"]}
    assert by_name["t"]["dtype"] == "number"
    assert by_name["x"]["dtype"] == "mixed"  # 1, 2.5, zzz
    assert by_name["label"]["dtype"] == "string"
    assert by_name["flag"]["dtype"] == "boolean"
    assert by_name["empty"]["dtype"] == "missing"
    assert by_name["x"]["missing"] == 1
    assert len(by_name["x"]["sampleValues"]) == 3
    assert result["timeColumn"] == "t"
    assert result["timeRange"] == {"min": 0.0, "max": 3.0}


def test_schema_inspect_missing_time_column_still_reports(tmp_path):
    path = tmp_path / "no_t.csv"
    write_csv(path, ["x"], [["1"], ["2"]])
    result = dp.cmd_data_schema_inspect({"path": str(path)})
    assert result["ok"] is True
    assert "timeColumn" not in result


def test_schema_inspect_unsupported_format(tmp_path):
    path = tmp_path / "data.parquet"
    path.write_bytes(b"\x00\x01")
    with pytest.raises(WorkerError, match="unsupported tabular format"):
        dp.cmd_data_schema_inspect({"path": str(path)})


# ---------------------------------------------------------------------------
# 3. data-time-sync-estimate
# ---------------------------------------------------------------------------

def test_time_sync_estimate_cross_correlation(tmp_path):
    # Two-tone sine: a single 1 Hz tone is periodically ambiguous for
    # cross-correlation (a delayed sine is also a sine), so a second
    # incommensurate tone disambiguates the peak while keeping it a sine wave.
    t = [round(i * 0.01, 4) for i in range(1000)]  # 0 .. 9.99 s
    x = [math.sin(2 * math.pi * 1.0 * ti) + 0.3 * math.sin(2 * math.pi * 0.37 * ti) for ti in t]
    path_a = tmp_path / "a.csv"
    path_b = tmp_path / "b.csv"
    write_csv(path_a, ["t", "value"], [[ti, xi] for ti, xi in zip(t, x)])
    write_csv(path_b, ["t", "value"], [[round(ti + 0.5, 4), xi] for ti, xi in zip(t, x)])  # delayed 0.5 s

    result = dp.cmd_data_time_sync_estimate(
        {"pathA": str(path_a), "pathB": str(path_b), "signalColumns": {"a": "value", "b": "value"}, "maxLagS": 5}
    )
    assert result["ok"] is True
    assert result["method"] == "cross-correlation"
    assert abs(result["offsetS"] - 0.5) <= 0.05
    assert result["correlation"] > 0.95
    assert result["confidence"] in ("high", "medium")


def test_time_sync_estimate_mean_difference(tmp_path):
    # The same 5 events recorded by two clocks 0.2s apart (B's clock ahead).
    events = [0.0, 1.0, 2.0, 3.0, 4.0]
    path_a = tmp_path / "a.csv"
    path_b = tmp_path / "b.csv"
    write_csv(path_a, ["t"], [[t] for t in events])
    write_csv(path_b, ["t"], [[round(t + 0.2, 4)] for t in events])
    result = dp.cmd_data_time_sync_estimate({"pathA": str(path_a), "pathB": str(path_b)})
    assert result["ok"] is True
    assert result["method"] == "mean-difference"
    assert result["confidence"] == "low"
    assert abs(result["offsetS"] - 0.2) <= 0.05


def test_time_sync_estimate_missing_signal_column_raises(tmp_path):
    path = tmp_path / "x.csv"
    write_csv(path, ["t", "v"], [["0", "1"]])
    with pytest.raises(WorkerError, match="signal column"):
        dp.cmd_data_time_sync_estimate(
            {"pathA": str(path), "pathB": str(path), "signalColumns": {"a": "nope", "b": "v"}}
        )


# ---------------------------------------------------------------------------
# 4. data-align-streams
# ---------------------------------------------------------------------------

def test_align_streams_nearest(tmp_path):
    primary = tmp_path / "primary.csv"
    secondary = tmp_path / "secondary.csv"
    write_csv(primary, ["t", "v"], [[round(i * 0.1, 4), i] for i in range(11)])  # 0 .. 1.0
    write_csv(secondary, ["t", "w"], [[round(0.05 + i * 0.1, 4), i] for i in range(9)])  # 0.05 .. 0.85

    result = dp.cmd_data_align_streams(
        {"primary": str(primary), "files": [{"path": str(secondary)}], "strategy": "nearest", "maxGapS": 0.1}
    )
    assert result["ok"] is True
    assert len(result["alignedSamples"]) == 11
    stream = result["streams"][1]
    assert stream["path"] == os.path.abspath(str(secondary))
    assert stream["samples"] == 9
    assert stream["matched"] == 10  # t=1.0 has no secondary sample within 0.1s
    assert stream["unmatched"] == 1
    assert stream["gaps"] == 1
    # first primary row t=0 -> nearest secondary t=0.05 -> w=0
    assert result["alignedSamples"][0]["t"] == "0.0" or result["alignedSamples"][0]["t"] == 0.0
    assert result["alignedSamples"][0]["w"] == "0" or result["alignedSamples"][0]["w"] == 0
    assert "interpolat" in result["note"] or "interpolat" in result["note"].lower() or "插值" in result["note"]

    # with outPath -> CSV written with primary rows
    out = tmp_path / "aligned.csv"
    result = dp.cmd_data_align_streams(
        {"primary": str(primary), "files": [{"path": str(secondary)}], "strategy": "nearest", "maxGapS": 0.1, "outPath": str(out)}
    )
    assert result["outPath"] == os.path.abspath(str(out))
    assert len(read_csv_rows(str(out))) == 12  # header + 11 rows


def test_align_streams_window_mean(tmp_path):
    primary = tmp_path / "p.csv"
    secondary = tmp_path / "s.csv"
    write_csv(primary, ["t"], [["0.0"], ["0.2"]])
    write_csv(secondary, ["t", "x"], [["0.01", "1"], ["0.03", "3"], ["0.21", "5"], ["0.22", "7"]])
    result = dp.cmd_data_align_streams(
        {"primary": str(primary), "files": [{"path": str(secondary)}], "strategy": "window", "maxGapS": 0.05}
    )
    assert result["ok"] is True
    first = result["alignedSamples"][0]
    # window around t=0.0 within 0.05 -> values 1 and 3 -> mean 2
    assert abs(float(first["x"]) - 2.0) < 1e-6
    stream = result["streams"][1]
    assert stream["matched"] == 2


# ---------------------------------------------------------------------------
# 5. data-transform-apply
# ---------------------------------------------------------------------------

def test_transform_interpolate_gaps(tmp_path):
    path = tmp_path / "gap.csv"
    out = tmp_path / "gap_out.csv"
    write_csv(
        path,
        ["t", "pos"],
        [
            ["0.0", "0.0"],
            ["0.1", "0.1"],
            ["0.2", "0.2"],
            ["0.3", ""],  # small gap: 0.2 -> 0.5 span 0.3s <= maxGapS
            ["0.4", ""],
            ["0.5", "0.5"],
            ["0.6", "0.6"],
            ["0.7", ""],  # big gap: 0.6 -> 1.0 span 0.4s > maxGapS
            ["0.8", ""],
            ["0.9", ""],
            ["1.0", "1.0"],
        ],
    )
    original = path.read_text(encoding="utf-8")
    result = dp.cmd_data_transform_apply(
        {"inputPath": str(path), "operations": [{"kind": "interpolate-gaps", "params": {"column": "pos", "maxGapS": 0.3}}], "outPath": str(out)}
    )
    assert result["ok"] is True
    op = result["operations"][0]
    assert op["kind"] == "interpolate-gaps"
    assert op["affectedRows"] == 2  # two missing rows interpolated
    assert "skipped 3" in op["detail"]
    assert path.read_text(encoding="utf-8") == original  # input untouched
    rows = {r[0]: r[1] for r in read_csv_rows(str(out))[1:]}
    assert abs(float(rows["0.3"]) - 0.3) < 1e-6
    assert abs(float(rows["0.4"]) - 0.4) < 1e-6
    assert rows["0.7"] == ""  # big gap left untouched
    assert rows["0.8"] == ""
    assert result["before"]["rows"] == 11
    assert result["after"]["rows"] == 11


def test_transform_unit_convert_and_unknown_unit(tmp_path):
    path = tmp_path / "units.csv"
    out = tmp_path / "units_out.csv"
    write_csv(path, ["t", "x"], [["0", "0"], ["1", "100"], ["2", "2000"]])
    result = dp.cmd_data_transform_apply(
        {"inputPath": str(path), "operations": [{"kind": "unit-convert", "params": {"column": "x", "from": "mm", "to": "m"}}], "outPath": str(out)}
    )
    values = [float(r[1]) for r in read_csv_rows(str(out))[1:]]
    assert values == pytest.approx([0.0, 0.1, 2.0], abs=1e-9)

    with pytest.raises(WorkerError, match="unknown unit"):
        dp.cmd_data_transform_apply(
            {"inputPath": str(path), "operations": [{"kind": "unit-convert", "params": {"column": "x", "from": "furlong", "to": "m"}}], "outPath": str(out)}
        )


def test_transform_range_filter(tmp_path):
    path = tmp_path / "filter.csv"
    out = tmp_path / "filter_out.csv"
    write_csv(path, ["t", "x"], [[str(i), str(i)] for i in range(6)])
    result = dp.cmd_data_transform_apply(
        {"inputPath": str(path), "operations": [{"kind": "range-filter", "params": {"column": "x", "min": 2, "max": 4}}], "outPath": str(out)}
    )
    assert result["operations"][0]["affectedRows"] == 3  # removed 0,1,5
    assert result["after"]["rows"] == 3
    assert len(read_csv_rows(str(out))) == 4  # header + 3


def test_transform_lowpass_shape_and_dedupe(tmp_path):
    path = tmp_path / "lp.csv"
    out = tmp_path / "lp_out.csv"
    times = [round(i * 0.01, 4) for i in range(101)]
    write_csv(path, ["t", "x"], [[ti, math.sin(2 * math.pi * 2.0 * ti)] for ti in times])
    result = dp.cmd_data_transform_apply(
        {
            "inputPath": str(path),
            "operations": [
                {"kind": "lowpass", "params": {"column": "x", "cutoffHz": 5, "sampleRateHz": 100}},
                {"kind": "dedupe", "params": {}},
            ],
            "outPath": str(out),
        }
    )
    assert result["ok"] is True
    assert result["operations"][0]["kind"] == "lowpass"
    assert result["operations"][0]["affectedRows"] == 101
    rows = read_csv_rows(str(out))[1:]
    assert len(rows) == 101
    for row in rows:
        assert row[1] != ""  # every value finite after filtfilt


def test_transform_rejects_inplace_output(tmp_path):
    path = tmp_path / "in.csv"
    write_csv(path, ["t", "x"], [["0", "1"]])
    with pytest.raises(WorkerError, match="read-only"):
        dp.cmd_data_transform_apply(
            {"inputPath": str(path), "operations": [{"kind": "round", "params": {"column": "x", "decimals": 2}}], "outPath": str(path)}
        )


# ---------------------------------------------------------------------------
# 6. data-segment-episodes
# ---------------------------------------------------------------------------

def test_segment_episodes_by_gap(tmp_path):
    path = tmp_path / "seg.csv"
    write_csv(
        path,
        ["t", "label"],
        [
            ["0.0", "ok"],
            ["0.1", "ok"],
            ["0.2", "ok"],
            ["2.0", "fail"],
            ["2.1", "fail"],
            ["5.0", "ok"],
        ],
    )
    result = dp.cmd_data_segment_episodes({"path": str(path), "maxGapS": 1.0, "labelColumn": "label"})
    assert result["ok"] is True
    assert len(result["episodes"]) == 3
    first, second, third = result["episodes"]
    assert first["id"] == "episode-0001"
    assert first["rows"] == 3
    assert first["durationS"] == pytest.approx(0.2, abs=1e-6)
    assert second["rows"] == 2
    assert third["rows"] == 1
    assert first["labels"] == {"ok": 3}
    assert second["labels"] == {"fail": 2}
    assert len(result["gaps"]) == 2
    assert result["gaps"][0]["gapS"] == pytest.approx(1.8, abs=1e-6)


# ---------------------------------------------------------------------------
# 7/8. annotations
# ---------------------------------------------------------------------------

def test_annotation_import_csv_column_compat(tmp_path):
    path = tmp_path / "ann.csv"
    write_csv(
        path,
        ["start", "end", "class", "confidence"],
        [["0.0", "1.0", "grasp", "0.9"], ["2.0", "2.5", "place", "0.8"]],
    )
    result = dp.cmd_data_annotation_import({"path": str(path), "format": "csv"})
    assert result["ok"] is True
    assert result["counts"]["total"] == 2
    assert result["counts"]["withLabel"] == 2
    assert result["counts"]["withInterval"] == 2
    first = result["annotations"][0]
    assert first["startS"] == 0.0
    assert first["endS"] == 1.0
    assert first["label"] == "grasp"
    assert first["confidence"] == 0.9
    assert first["source"] == "ann.csv"


def test_annotation_review_confirm_writes_new_file(tmp_path):
    path = tmp_path / "ann.jsonl"
    write_jsonl(path, [{"t": 0.0, "label": "a"}, {"t": 1.0, "label": "b"}])
    listed = dp.cmd_data_annotation_review({"path": str(path), "action": "list"})
    assert listed["total"] == 2
    assert listed["confirmed"] == 0

    out = tmp_path / "reviewed.jsonl"
    result = dp.cmd_data_annotation_review({"path": str(path), "action": "confirm", "ids": [listed["annotations"][0]["id"]], "outPath": str(out)})
    assert result["updated"] == 1
    assert result["outPath"] == os.path.abspath(str(out))
    reviewed = [json.loads(line) for line in out.read_text(encoding="utf-8").splitlines()]
    statuses = {r["label"]: r.get("status") for r in reviewed}
    assert statuses == {"a": "confirmed", "b": None}
    # input untouched
    assert len(path.read_text(encoding="utf-8").splitlines()) == 2

    with pytest.raises(WorkerError, match="outPath"):
        dp.cmd_data_annotation_review({"path": str(path), "action": "confirm"})


# ---------------------------------------------------------------------------
# 9/10. splits and leakage
# ---------------------------------------------------------------------------

def test_split_create_group_no_leak(tmp_path):
    path = tmp_path / "splits.csv"
    rows = [[str(i), "A"] for i in range(10)] + [[str(i), "B"] for i in range(10)]
    write_csv(path, ["t", "g"], rows)
    out_dir = tmp_path / "splits_out"
    result = dp.cmd_data_split_create(
        {"path": str(path), "method": "group", "groupColumns": ["g"], "ratios": {"train": 0.7, "val": 0.15, "test": 0.15}, "seed": 42, "outDir": str(out_dir)}
    )
    assert result["ok"] is True
    splits = result["splits"]
    total = splits["train"]["rows"] + splits["val"]["rows"] + splits["test"]["rows"]
    assert total == 20
    train_groups = set(splits["train"]["groups"])
    val_groups = set(splits["val"]["groups"])
    test_groups = set(splits["test"]["groups"])
    assert not (train_groups & val_groups)
    assert not (train_groups & test_groups)
    assert not (val_groups & test_groups)
    assert len(train_groups | val_groups | test_groups) == 2  # both groups used exactly once
    assert result["leakSummary"]["leaked"] == 0
    for bucket in ("train", "val", "test"):
        assert os.path.exists(os.path.join(str(out_dir), f"{bucket}.csv"))
    # groups stay whole: each split has a multiple of the group size (10)
    assert splits["train"]["rows"] % 10 == 0
    assert splits["val"]["rows"] % 10 == 0


def test_split_create_group_requires_group_columns(tmp_path):
    path = tmp_path / "x.csv"
    write_csv(path, ["t"], [["0"]])
    with pytest.raises(WorkerError, match="groupColumns"):
        dp.cmd_data_split_create({"path": str(path), "method": "group"})


def test_leakage_check_detects_cross_split_group(tmp_path):
    train = tmp_path / "train.csv"
    val = tmp_path / "val.csv"
    write_csv(train, ["t", "g"], [["0", "A"], ["1", "B"]])
    write_csv(val, ["t", "g"], [["2", "B"], ["3", "C"]])
    result = dp.cmd_data_leakage_check({"trainPath": str(train), "valPath": str(val), "groupColumns": ["g"]})
    assert result["ok"] is True
    assert result["verdict"] == "leak-detected"
    assert result["leakedGroups"] == [{"key": "B", "splits": ["train", "val"]}]
    assert result["overlapSummary"]["leaked"] == 1
    assert result["overlapSummary"]["splits"] == {"train": 2, "val": 2}


def test_leakage_check_frame_adjacency_warning(tmp_path):
    train = tmp_path / "train.csv"
    val = tmp_path / "val.csv"
    write_csv(train, ["t", "g"], [["0.0", "A"], ["0.5", "A"]])
    write_csv(val, ["t", "g"], [["0.6", "C"], ["1.0", "C"]])
    result = dp.cmd_data_leakage_check({"trainPath": str(train), "valPath": str(val), "groupColumns": ["g"], "timeColumn": "t"})
    assert result["verdict"] == "ok"  # no group leak
    adjacency = result["overlapSummary"].get("adjacency", [])
    assert len(adjacency) == 1
    assert adjacency[0]["between"] == ["train", "val"]
    assert adjacency[0]["gapS"] == pytest.approx(0.1, abs=1e-6)


# ---------------------------------------------------------------------------
# 11. data-deidentify
# ---------------------------------------------------------------------------

def test_deidentify_pii_scan(tmp_path):
    text_path = tmp_path / "notes.txt"
    text_path.write_text(
        "contact alice@example.com or bob@corp.io phone 13812345678 id 110101199003078811",
        encoding="utf-8",
    )
    result = dp.cmd_data_deidentify({"inputPath": str(text_path), "operations": ["pii-scan"]})
    assert result["ok"] is True
    matches = [m for entry in result["processed"] if entry["action"] == "pii-scan" for m in entry.get("matches", [])]
    patterns = {m["pattern"] for m in matches}
    assert "email" in patterns
    assert "phone" in patterns
    assert "idcard" in patterns
    email = next(m for m in matches if m["pattern"] == "email")
    assert email["masked"].startswith("***@")
    phone = next(m for m in matches if m["pattern"] == "phone")
    assert phone["masked"].startswith("138****")
    assert len(result["privacyNotes"]) == 3
    assert "匿名化" in result["privacyNotes"][0]

    # with outDir -> sanitized copy written, all outputs inside outDir
    out_dir = tmp_path / "sanitized"
    result = dp.cmd_data_deidentify({"inputPath": str(text_path), "outDir": str(out_dir), "operations": ["pii-scan"]})
    entry = next(e for e in result["processed"] if e["action"] == "pii-scan")
    assert entry["output"] and entry["output"].startswith(os.path.abspath(str(out_dir)) + os.sep)
    sanitized = entry["output"].rsplit(os.sep, 1)[-1]
    assert "alice@example.com" not in (out_dir / sanitized).read_text(encoding="utf-8")


def test_deidentify_face_blur_no_face(tmp_path):
    cv2 = pytest.importorskip("cv2")
    image = (np_random_image())
    in_dir = tmp_path / "imgs"
    in_dir.mkdir()
    cv2.imwrite(str(in_dir / "noise.png"), image)
    out_dir = tmp_path / "blurred"
    result = dp.cmd_data_deidentify({"inputPath": str(in_dir), "outDir": str(out_dir), "operations": ["face-blur"]})
    assert result["ok"] is True
    blur_entries = [e for e in result["processed"] if e["action"] == "face-blur"]
    assert len(blur_entries) == 1
    assert "no face" in blur_entries[0]["detail"]


def np_random_image():
    import numpy as np

    rng = np.random.default_rng(7)
    return rng.integers(0, 255, size=(120, 160, 3), dtype=np.uint8)


def test_deidentify_requires_outdir_for_face_blur(tmp_path):
    cv2 = pytest.importorskip("cv2")
    in_dir = tmp_path / "imgs"
    in_dir.mkdir()
    cv2.imwrite(str(in_dir / "x.png"), np_random_image())
    with pytest.raises(WorkerError, match="outDir"):
        dp.cmd_data_deidentify({"inputPath": str(in_dir), "operations": ["face-blur"]})


def test_deidentify_rejects_c_drive_outdir(tmp_path):
    text_path = tmp_path / "x.txt"
    text_path.write_text("hello", encoding="utf-8")
    with pytest.raises(WorkerError, match="C:"):
        dp.cmd_data_deidentify({"inputPath": str(text_path), "outDir": r"C:\rh-test-out", "operations": ["pii-scan"]})


# ---------------------------------------------------------------------------
# 12. data-convert-rosbag
# ---------------------------------------------------------------------------

def _make_demo_rosbag(tmp_path, with_unsupported=False):
    db = tmp_path / "demo_0.db3"
    connection = sqlite3.connect(db)
    connection.execute(
        "CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, offered_qos_profiles TEXT)"
    )
    connection.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    connection.execute("INSERT INTO topics VALUES (1, '/signal', 'std_msgs/msg/Float64', 'cdr', '')")
    connection.execute("INSERT INTO topics VALUES (2, '/note', 'std_msgs/msg/String', 'cdr', '')")
    if with_unsupported:
        connection.execute("INSERT INTO topics VALUES (3, '/img', 'sensor_msgs/msg/Image', 'cdr', '')")
    for index, value in enumerate([1.5, 2.5, 3.5]):
        connection.execute(
            "INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, ?)",
            ((index + 1) * 1_000_000_000, struct.pack("<d", value)),
        )
    connection.execute(
        "INSERT INTO messages (topic_id, timestamp, data) VALUES (2, 1500000000, ?)",
        (struct.pack("<I", 4) + b"ping",),
    )
    if with_unsupported:
        connection.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (3, 1600000000, ?)", (b"\x00\x01\x02",))
    connection.commit()
    connection.close()
    return db


def test_convert_rosbag_decodes_float64(tmp_path):
    db = _make_demo_rosbag(tmp_path)
    out_dir = tmp_path / "out"
    result = dp.cmd_data_convert_rosbag({"rosbagPath": str(db), "outDir": str(out_dir), "topics": ["/signal"]})
    assert result["ok"] is True
    assert result["unsupportedTypes"] == []
    assert len(result["outFiles"]) == 1
    entry = result["outFiles"][0]
    assert entry["topic"] == "/signal"
    assert entry["decoded"] is True
    assert entry["rows"] == 3
    rows = read_csv_rows(entry["path"])
    assert rows[0] == ["t", "value"]
    assert [float(r[1]) for r in rows[1:]] == [1.5, 2.5, 3.5]
    assert [float(r[0]) for r in rows[1:]] == [1.0, 2.0, 3.0]  # ns -> s


def test_convert_rosbag_string_and_unsupported_listed(tmp_path):
    db = _make_demo_rosbag(tmp_path, with_unsupported=True)
    out_dir = tmp_path / "out2"
    result = dp.cmd_data_convert_rosbag({"rosbagPath": str(db), "outDir": str(out_dir)})
    assert len(result["outFiles"]) == 3
    note_file = next(f for f in result["outFiles"] if f["topic"] == "/note")
    assert note_file["decoded"] is True
    note_rows = read_csv_rows(note_file["path"])
    assert note_rows[1][1] == "ping"
    img_file = next(f for f in result["outFiles"] if f["topic"] == "/img")
    assert img_file["decoded"] is False
    assert "sensor_msgs/msg/Image" in result["unsupportedTypes"]
    assert img_file["note"]
    # undecodable topic still written as t + dataSize, never silently dropped
    img_rows = read_csv_rows(img_file["path"])
    assert img_rows[0] == ["t", "dataSize"]


# ---------------------------------------------------------------------------
# 13/14. exports
# ---------------------------------------------------------------------------

def test_export_lerobot_structure(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    write_jsonl(
        run_dir / "telemetry.jsonl",
        [{"t": 0.0, "q": [0.1, 0.2, 0.3]}, {"t": 0.1, "q": [0.15, 0.25, 0.35]}],
    )
    (run_dir / "run.json").write_text(json.dumps({"metrics": {"success": True}}), encoding="utf-8")
    out_dir = tmp_path / "lerobot"
    result = dp.cmd_data_export_lerobot({"runPath": str(run_dir), "outDir": str(out_dir), "robotName": "rh_demo", "task": "pick_place"})
    assert result["ok"] is True
    assert result["format"] in ("lerobot-v2", "lerobot-csv-fallback")
    assert result["episodes"] == 1
    assert result["frames"] == 2
    info = json.loads((out_dir / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 2
    assert info["robot_type"] == "rh_demo"
    assert info["task"] == "pick_place"
    chunk = out_dir / "data" / "chunk-000"
    assert (chunk / "episode_000001.json").exists()
    assert (chunk / "episode_000001.parquet").exists() or (chunk / "episode_000001.csv").exists()
    episode_meta = json.loads((chunk / "episode_000001.json").read_text(encoding="utf-8"))
    assert episode_meta["length"] == 2


def test_export_lerobot_episodes_path(tmp_path):
    episodes = {
        "episodes": [
            {"id": "ep1", "frames": [{"t": 0.0, "q0": 1.0, "q1": 2.0}, {"t": 0.1, "q0": 1.1, "q1": 2.1}]},
            {"id": "ep2", "frames": [{"t": 0.0, "q0": 3.0, "q1": 4.0}]},
        ]
    }
    ep_path = tmp_path / "episodes.json"
    ep_path.write_text(json.dumps(episodes), encoding="utf-8")
    out_dir = tmp_path / "lerobot2"
    result = dp.cmd_data_export_lerobot({"episodesPath": str(ep_path), "outDir": str(out_dir)})
    assert result["episodes"] == 2
    assert result["frames"] == 3
    assert (out_dir / "data" / "chunk-000" / "episode_000002.json").exists()


def test_export_rlds_manifest(tmp_path):
    out_dir = tmp_path / "rlds"
    result = dp.cmd_data_export_rlds({"outDir": str(out_dir)})
    assert result["ok"] is True
    assert result["format"] == "rlds-manifest"
    assert result["manifestPath"] == os.path.abspath(str(out_dir / "manifest.json"))
    features = json.loads((out_dir / "features.json").read_text(encoding="utf-8"))
    assert "observation" in features["features"]
    assert features["features"]["observation"]["q"]["dtype"] == "float32"
    assert features["features"]["is_terminal"]["dtype"] == "bool"
    assert (out_dir / "data").is_dir()
    assert any("tensorflow" in note for note in result["notes"])


# ---------------------------------------------------------------------------
# 15/16/17. versioning / compare / card
# ---------------------------------------------------------------------------

def _make_source(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "a.csv").write_text("t,x\n0,1\n1,2\n", encoding="utf-8")
    (src / "b.jsonl").write_text('{"t":0,"v":1}\n{"t":1,"v":2}\n', encoding="utf-8")
    return src


def test_dataset_version_create_compare_card(tmp_path):
    src = _make_source(tmp_path)
    sources = [str(src / "a.csv"), str(src / "b.jsonl")]

    out_a = tmp_path / "dsA"
    res_a = dp.cmd_dataset_version_create({"name": "demo", "sourcePaths": sources, "outDir": str(out_a), "version": "0.1.0", "description": "demo dataset"})
    assert res_a["ok"] is True
    assert res_a["version"] == "0.1.0"
    manifest = json.loads((out_a / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["contentHash"] == res_a["contentHash"]
    assert manifest["stats"]["files"] == 2
    assert manifest["stats"]["totalBytes"] > 0
    assert (out_a / "data" / "a.csv").exists()
    assert (out_a / "data" / "b.jsonl").exists()

    # identical sources -> identical hash
    out_b = tmp_path / "dsB"
    res_b = dp.cmd_dataset_version_create({"name": "demo", "sourcePaths": sources, "outDir": str(out_b)})
    assert res_b["contentHash"] == res_a["contentHash"]

    compare = dp.cmd_dataset_compare({"datasetA": str(out_a), "datasetB": str(out_b)})
    assert compare["sameContent"] is True
    assert compare["hashA"] == compare["hashB"]
    assert all(f["hashEqual"] is True for f in compare["differences"]["files"])

    # modified source -> different hash and per-file result
    (src / "a.csv").write_text("t,x\n0,1\n1,99\n", encoding="utf-8")
    out_c = tmp_path / "dsC"
    res_c = dp.cmd_dataset_version_create({"name": "demo", "sourcePaths": sources, "outDir": str(out_c)})
    assert res_c["contentHash"] != res_a["contentHash"]
    compare2 = dp.cmd_dataset_compare({"datasetA": str(out_a), "datasetB": str(out_c)})
    assert compare2["sameContent"] is False
    by_name = {f["name"]: f for f in compare2["differences"]["files"]}
    assert by_name["a.csv"]["hashEqual"] is False
    assert by_name["b.jsonl"]["hashEqual"] is True

    # data card
    card = dp.cmd_dataset_card_generate({"datasetPath": str(out_a)})
    assert card["ok"] is True
    assert os.path.exists(card["path"])
    text = open(card["path"], encoding="utf-8").read()
    assert "# Data Card" in text
    assert "demo" in text
    assert "来源与转换 DAG" in text


def test_dataset_version_create_missing_source_raises(tmp_path):
    with pytest.raises(WorkerError, match="source path not found"):
        dp.cmd_dataset_version_create({"name": "x", "sourcePaths": [str(tmp_path / "missing.csv")], "outDir": str(tmp_path / "out")})


def test_compare_missing_manifest_raises(tmp_path):
    with pytest.raises(WorkerError, match="manifest not found"):
        dp.cmd_dataset_compare({"datasetA": str(tmp_path), "datasetB": str(tmp_path)})


# ---------------------------------------------------------------------------
# registry / contract surface
# ---------------------------------------------------------------------------

def test_command_registry_has_all_commands():
    expected = {
        "data-inventory",
        "data-schema-inspect",
        "data-time-sync-estimate",
        "data-align-streams",
        "data-transform-apply",
        "data-segment-episodes",
        "data-annotation-import",
        "data-annotation-review",
        "data-split-create",
        "data-leakage-check",
        "data-deidentify",
        "data-convert-rosbag",
        "data-export-lerobot",
        "data-export-rlds",
        "dataset-version-create",
        "dataset-compare",
        "dataset-card-generate",
    }
    assert expected <= set(dp.COMMANDS)
    assert dp.CAPABILITIES
    for command in expected:
        assert callable(dp.COMMANDS[command])
