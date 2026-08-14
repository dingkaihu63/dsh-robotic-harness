"""ROS 2 adapter module for the Robotic Harness worker.

Design notes
------------
This machine has no ROS 2 installation, so every "live ROS" command follows an
*adapter pattern*:

* :func:`_ros2_available` probes for the ``ros2`` CLI via ``shutil.which``.
* When the CLI is missing, the command returns a *structured diagnostic* with
  ``{"ok": true, "backend": "unavailable", "reason": ..., "instructions": ...}``
  instead of raising -- the tool exists and can be invoked by an Agent, it just
  honestly reports that the backend is absent.
* When the CLI is present, commands shell out to whitelisted ``ros2``
  subcommands and parse their text output. Parsers are pure functions so the
  live-ROS behaviour can be unit-tested with canned output.

Rosbag inspection never needs ROS: rosbag2 files are SQLite3 (``.db3``) plus a
``metadata.yaml``, read directly with the standard library and PyYAML. All
rosbag statistics live in independent pure functions (:func:`inspect_rosbag2`,
:func:`read_rosbag_tf_summary`, :func:`read_rosbag_diagnostics_summary`) that
are reused by the rosbag-backed branches of ``ros-tf-audit`` and
``ros-diagnostics-snapshot``.

Safety
------
* ``rosbag-start`` refuses to record on the C: drive (Windows).
* ``ros-call-whitelisted-action`` only ever sends goals whose action name is in
  an allowlist (argument ``allowlist`` or ``<storeRoot>/.rh/ros-allowlist.json``)
  -- arbitrary calls are rejected with :class:`WorkerError`.
* Rosbag recording state is tracked in ``<storeRoot>/.rh/rosbag-jobs.json`` so
  ``rosbag-stop`` can terminate the spawned ``ros2 bag record`` process.

CDR decoding
------------
rosbag2 stores messages serialized with Fast-CDR little-endian, prefixed by a
4-byte CDR encapsulation header (``00 00 00 00``). The decoders here are
best-effort: they return ``None`` on any structural mismatch and the caller
records a decode issue instead of failing the whole command. Unsupported types
are always listed explicitly in the results (never silently dropped).
"""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import sqlite3
import struct
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from typing import Any, Callable, Optional

from .core import WorkerError, new_id

try:  # PyYAML is installed in the recommended environment (used for metadata.yaml)
    import yaml  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover
    yaml = None

ROS2_INSTRUCTIONS = "Install ROS 2 (e.g. ros-humble-desktop) or provide a rosbag path"

LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


# ---------------------------------------------------------------------------
# backend probe / adapter helpers
# ---------------------------------------------------------------------------


def _ros2_available() -> bool:
    """Return True when the ``ros2`` CLI is on PATH."""
    return shutil.which("ros2") is not None


def _unavailable(reason: str | None = None) -> dict[str, Any]:
    """Structured "backend missing" diagnostic (not an error)."""
    return {
        "ok": True,
        "backend": "unavailable",
        "reason": reason or "ros2 CLI not found on PATH (shutil.which('ros2') is None)",
        "instructions": ROS2_INSTRUCTIONS,
    }


def _creation_flags() -> int:
    return getattr(subprocess, "CREATE_NO_WINDOW", 0)


def _run_ros2(argv: list[str], timeout: float = 10.0, ros_domain: Optional[int] = None) -> str:
    """Run a whitelisted ``ros2`` subcommand and return its text output.

    Raises on timeout; a non-zero exit code is not an exception (the output is
    returned as-is so parsers can deal with error text).
    """
    env = os.environ.copy()
    if ros_domain is not None:
        env["ROS_DOMAIN_ID"] = str(int(ros_domain))
    proc = subprocess.run(
        ["ros2", *argv],
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
        creationflags=_creation_flags(),
    )
    output = proc.stdout or ""
    if not output.strip():
        output = proc.stderr or ""
    return output


def measure_topic_hz(topic: str, duration_s: float, window: int = 50, ros_domain: Optional[int] = None) -> dict[str, Any]:
    """Measure topic rate with ``ros2 topic hz`` (spawned, killed after duration)."""
    env = os.environ.copy()
    if ros_domain is not None:
        env["ROS_DOMAIN_ID"] = str(int(ros_domain))
    try:
        proc = subprocess.Popen(
            ["ros2", "topic", "hz", topic, "--window", str(window)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
            creationflags=_creation_flags(),
        )
    except OSError:
        return _parse_topic_hz("")
    chunks: list[str] = []

    def _drain() -> None:
        try:
            if proc.stdout:
                for line in proc.stdout:
                    chunks.append(line)
        except Exception:  # noqa: BLE001 - reader thread must not crash the command
            pass

    thread = threading.Thread(target=_drain, daemon=True)
    thread.start()
    time.sleep(max(0.5, duration_s))
    try:
        proc.kill()
    except Exception:  # noqa: BLE001
        pass
    try:
        proc.wait(timeout=2)
    except Exception:  # noqa: BLE001
        pass
    return _parse_topic_hz("".join(chunks))


# ---------------------------------------------------------------------------
# pure text parsers (live ROS output)
# ---------------------------------------------------------------------------


def _parse_node_list(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if line.strip()]


def _parse_topic_list(text: str) -> list[dict[str, str]]:
    """Parse ``ros2 topic|service|action list -t`` lines: ``name [type]``."""
    out: list[dict[str, str]] = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        match = re.match(r"^(.*?)\s*\[(.*)\]\s*$", line)
        if match:
            out.append({"name": match.group(1).strip(), "type": match.group(2).strip()})
        else:
            out.append({"name": line})
    return out


def _qos_blocks(text: str) -> list[dict[str, Any]]:
    """Extract ``QoS profile:`` blocks from ``ros2 topic info -v`` output."""
    blocks: list[dict[str, Any]] = []
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        if lines[index].strip() == "QoS profile:":
            block: dict[str, Any] = {"pos": index}
            index += 1
            while index < len(lines) and lines[index].strip():
                match = re.match(r"^\s*([A-Za-z ]+?):\s*(.*)$", lines[index])
                if match:
                    key = match.group(1).strip().lower().replace(" ", "_")
                    block[key] = match.group(2).strip()
                index += 1
            blocks.append(block)
        else:
            index += 1
    return blocks


def _parse_topic_info(text: str) -> dict[str, Any]:
    """Parse ``ros2 topic info <name> -v`` into type/counts/QoS blocks."""
    result: dict[str, Any] = {
        "type": None,
        "publisherCount": 0,
        "subscriberCount": 0,
        "publisherQos": None,
        "subscriberQos": None,
    }
    match = re.search(r"^\s*Type:\s*(.+)$", text, re.MULTILINE)
    if match:
        result["type"] = match.group(1).strip()
    match = re.search(r"Publisher count:\s*(\d+)", text)
    if match:
        result["publisherCount"] = int(match.group(1))
    match = re.search(r"Subscription count:\s*(\d+)", text)
    if match:
        result["subscriberCount"] = int(match.group(1))
    lines = text.splitlines()
    sub_index = next(
        (i for i, line in enumerate(lines) if line.strip().startswith("Subscription count")),
        None,
    )
    blocks = _qos_blocks(text)
    pub_blocks = [b for b in blocks if sub_index is None or b["pos"] < sub_index]
    sub_blocks = [b for b in blocks if sub_index is not None and b["pos"] >= sub_index]
    if result["publisherCount"] > 0 and pub_blocks:
        result["publisherQos"] = {k: v for k, v in pub_blocks[0].items() if k != "pos"}
    if result["subscriberCount"] > 0 and sub_blocks:
        result["subscriberQos"] = {k: v for k, v in sub_blocks[0].items() if k != "pos"}
    return result


def _parse_topic_hz(text: str) -> dict[str, Any]:
    """Parse ``ros2 topic hz`` output (average rate / window / interval stats)."""
    result: dict[str, Any] = {
        "measuredRateHz": None,
        "window": None,
        "samples": None,
        "minIntervalS": None,
        "maxIntervalS": None,
        "stdDevS": None,
        "raw": text[:4000],
    }
    match = re.search(r"average rate:\s*([\d.]+)", text)
    if match:
        result["measuredRateHz"] = round(float(match.group(1)), 4)
    match = re.search(r"window:\s*(\d+)", text)
    if match:
        result["window"] = int(match.group(1))
        result["samples"] = int(match.group(1))
    match = re.search(r"min:\s*([\d.]+)s\s+max:\s*([\d.]+)s\s+std dev:\s*([\d.]+)s", text)
    if match:
        result["minIntervalS"] = round(float(match.group(1)), 6)
        result["maxIntervalS"] = round(float(match.group(2)), 6)
        result["stdDevS"] = round(float(match.group(3)), 6)
    return result


def _parse_tf_static_echo(text: str) -> dict[str, Any]:
    """Parse ``ros2 topic echo /tf_static --once`` (YAML) into frame ids."""
    frames: set[str] = set()
    if yaml is None:
        return {"frames": [], "parseError": "PyYAML is required to parse /tf_static echo output"}
    try:
        data = yaml.safe_load(text)
    except Exception as error:  # noqa: BLE001 - output may be an error message
        return {"frames": [], "parseError": str(error)}

    def _walk(node: Any) -> None:
        if isinstance(node, dict):
            for key in ("child_frame_id", "frame_id"):
                value = node.get(key)
                if isinstance(value, str) and value:
                    frames.add(value)
            for value in node.values():
                _walk(value)
        elif isinstance(node, list):
            for value in node:
                _walk(value)

    _walk(data)
    return {"frames": sorted(frames)}


def _parse_diagnostics_echo(text: str) -> dict[str, Any]:
    """Parse ``ros2 topic echo /diagnostics --once`` (YAML) into statuses."""
    if yaml is None:
        return {"statuses": [], "parseError": "PyYAML is required to parse /diagnostics echo output"}
    try:
        data = yaml.safe_load(text)
    except Exception as error:  # noqa: BLE001
        return {"statuses": [], "parseError": str(error)}
    statuses: list[dict[str, Any]] = []
    for item in (data or {}).get("status") or []:
        if not isinstance(item, dict):
            continue
        level = int(item.get("level", 0) or 0)
        statuses.append(
            {
                "level": level,
                "levelName": LEVEL_NAMES.get(level, "UNKNOWN"),
                "name": str(item.get("name", "") or ""),
                "message": str(item.get("message", "") or ""),
                "hardwareId": str(item.get("hardware_id", "") or ""),
            }
        )
    return {"statuses": statuses}


def _parse_controllers(text: str) -> list[dict[str, Any]]:
    """Parse ``ros2 control list_controllers`` output."""
    controllers: list[dict[str, Any]] = []
    current: Optional[dict[str, Any]] = None
    for line in text.splitlines():
        if not line.strip():
            continue
        if line[:1] in (" ", "\t"):
            match = re.match(r"^-\s*(.+)$", line.strip())
            if match and current is not None:
                current.setdefault("claimedInterfaces", []).append(match.group(1).strip())
            continue
        parts = line.split()
        if len(parts) >= 3:
            current = {"name": parts[0], "type": parts[1], "state": parts[2]}
            controllers.append(current)
        elif parts:
            current = None
    return controllers


# ---------------------------------------------------------------------------
# SRDF (MoveIt config) parsing
# ---------------------------------------------------------------------------


def parse_srdf_groups(path: str) -> dict[str, Any]:
    """Parse planning groups from a MoveIt SRDF file (XML)."""
    tree = ET.parse(path)
    root = tree.getroot()
    end_effectors = [
        {
            "name": ee.get("name") or "",
            "parentLink": ee.get("parent_link") or "",
            "group": ee.get("group") or "",
        }
        for ee in root.findall("end_effector")
    ]
    ee_by_group = {ee["group"]: ee["name"] for ee in end_effectors if ee["group"]}
    groups: list[dict[str, Any]] = []
    for group in root.findall("group"):
        name = group.get("name") or ""
        entry: dict[str, Any] = {
            "name": name,
            "joints": [j.get("name") or "" for j in group.findall("joint")],
            "chains": [
                {"baseLink": c.get("base_link") or "", "tipLink": c.get("tip_link") or ""}
                for c in group.findall("chain")
            ],
        }
        if name in ee_by_group:
            entry["endEffector"] = ee_by_group[name]
        groups.append(entry)
    return {"groups": groups, "endEffectors": end_effectors}


# ---------------------------------------------------------------------------
# rosbag2 sqlite access (never requires ROS)
# ---------------------------------------------------------------------------


def _store_root(args: dict[str, Any]) -> str:
    return args.get("storeRoot") or os.path.join(os.getcwd(), ".rh")


def _state_file(args: dict[str, Any]) -> str:
    return os.path.join(_store_root(args), ".rh", "rosbag-jobs.json")


def _allowlist_file(args: dict[str, Any]) -> str:
    return os.path.join(_store_root(args), ".rh", "ros-allowlist.json")


def _locate_rosbag_db(path: str) -> str:
    """Resolve a rosbag2 directory or ``.db3`` file to the SQLite database path.

    Raises :class:`WorkerError` when the path is not a rosbag2 bag.
    """
    if not os.path.exists(path):
        raise WorkerError(f"rosbag path does not exist: {path}")
    if os.path.isdir(path):
        meta = os.path.join(path, "metadata.yaml")
        if not os.path.exists(meta):
            raise WorkerError(f"not a rosbag2 bag directory (no metadata.yaml): {path}")
        if yaml is None:
            raise WorkerError("PyYAML is required to read rosbag metadata; install it with 'pip install pyyaml'")
        with open(meta, encoding="utf-8") as handle:
            info = yaml.safe_load(handle) or {}
        bag_info = info.get("rosbag2_bagfile_information") or {}
        relative = bag_info.get("relative_file_paths") or []
        if not relative:
            raise WorkerError(f"metadata.yaml has no relative_file_paths: {path}")
        db = os.path.join(path, relative[0])
        if not os.path.exists(db):
            raise WorkerError(f"rosbag database file not found: {db}")
        return db
    if path.lower().endswith(".db3"):
        return path
    raise WorkerError(f"not a rosbag2 bag (expected .db3 file or directory with metadata.yaml): {path}")


def _open_rosbag_db(path: str) -> tuple[sqlite3.Connection, str]:
    """Open the rosbag SQLite database read-only, validating it is rosbag2."""
    db_path = _locate_rosbag_db(path)
    uri = "file:" + db_path.replace("\\", "/") + "?mode=ro"
    try:
        conn = sqlite3.connect(uri, uri=True)
    except sqlite3.Error as error:
        raise WorkerError(f"not a valid rosbag2 sqlite database: {db_path} ({error})") from error
    conn.row_factory = sqlite3.Row
    try:
        tables = {row["name"] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    except sqlite3.Error as error:
        conn.close()
        raise WorkerError(f"not a valid rosbag2 sqlite database: {db_path} ({error})") from error
    if not {"schema", "topics", "messages"} <= tables:
        conn.close()
        raise WorkerError(f"not a rosbag2 sqlite bag (missing schema/topics/messages tables): {db_path}")
    return conn, db_path


# ---------------------------------------------------------------------------
# CDR decoding (best-effort, pure)
# ---------------------------------------------------------------------------


class _CdrReader:
    """Minimal Fast-CDR little-endian reader with per-member alignment."""

    __slots__ = ("data", "offset")

    def __init__(self, data: bytes) -> None:
        self.data = data
        self.offset = 0

    def align(self, size: int) -> None:
        self.offset += (-self.offset) % size

    def read_uint32(self) -> int:
        self.align(4)
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_int32(self) -> int:
        self.align(4)
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_uint8(self) -> int:
        self.align(1)
        value = self.data[self.offset]
        self.offset += 1
        return value

    def read_float64(self) -> float:
        self.align(8)
        value = struct.unpack_from("<d", self.data, self.offset)[0]
        self.offset += 8
        return value

    def read_string(self) -> str:
        length = self.read_uint32()
        if length > len(self.data) - self.offset:
            raise ValueError("CDR string length out of range")
        raw = self.data[self.offset : self.offset + length]
        self.offset += length
        return raw.decode("utf-8", errors="replace")

    def skip(self, count: int) -> None:
        self.offset += count


class _CdrWriter:
    """Mirror of :class:`_CdrReader` used to build CDR test fixtures."""

    __slots__ = ("parts",)

    def __init__(self) -> None:
        self.parts: list[bytes] = []

    def align(self, size: int) -> None:
        current = sum(len(part) for part in self.parts)
        padding = (-current) % size
        if padding:
            self.parts.append(b"\x00" * padding)

    def write_uint32(self, value: int) -> None:
        self.align(4)
        self.parts.append(struct.pack("<I", value))

    def write_int32(self, value: int) -> None:
        self.align(4)
        self.parts.append(struct.pack("<i", value))

    def write_uint8(self, value: int) -> None:
        self.align(1)
        self.parts.append(bytes([value & 0xFF]))

    def write_float64(self, value: float) -> None:
        self.align(8)
        self.parts.append(struct.pack("<d", value))

    def write_string(self, value: str) -> None:
        raw = value.encode("utf-8")
        self.write_uint32(len(raw))
        self.parts.append(raw)

    def to_bytes(self) -> bytes:
        return b"".join(self.parts)


def decode_cdr_float64(data: bytes) -> Optional[float]:
    """Decode a std_msgs/msg/Float64 CDR message.

    Canonical layout (per the module contract): uint32 CDR encapsulation + float64.
    A fallback layout (encapsulation + 4 pad bytes + float64) is also accepted for
    bags written with 8-byte double alignment.
    """
    if len(data) >= 12:
        encapsulation, value = struct.unpack_from("<Id", data, 0)
        if encapsulation == 0:
            return value
    if len(data) >= 16:
        encapsulation = struct.unpack_from("<I", data, 0)[0]
        if encapsulation == 0:
            value = struct.unpack_from("<d", data, 8)[0]
            return value
    return None


def decode_cdr_string(data: bytes) -> Optional[str]:
    """Decode a std_msgs/msg/String CDR message (uint32 encap + uint32 len + utf8)."""
    if len(data) < 8:
        return None
    encapsulation, length = struct.unpack_from("<II", data, 0)
    if encapsulation != 0:
        return None
    end = 8 + length
    if end > len(data):
        return None
    return data[8:end].decode("utf-8", errors="replace")


def decode_tf_message(data: bytes) -> Optional[dict[str, Any]]:
    """Best-effort decode of tf2_msgs/msg/TFMessage (transforms with frame ids)."""
    try:
        reader = _CdrReader(data)
        reader.skip(4)  # CDR encapsulation
        count = reader.read_uint32()
        transforms = []
        for _ in range(count):
            sec = reader.read_int32()
            nanosec = reader.read_uint32()
            frame_id = reader.read_string()
            child_frame_id = reader.read_string()
            translation = [reader.read_float64() for _ in range(3)]
            rotation = [reader.read_float64() for _ in range(4)]
            transforms.append(
                {
                    "frame_id": frame_id,
                    "child_frame_id": child_frame_id,
                    "sec": sec,
                    "nanosec": nanosec,
                    "translation": [round(v, 9) for v in translation],
                    "rotation": [round(v, 9) for v in rotation],
                }
            )
        return {"transforms": transforms}
    except Exception:  # noqa: BLE001 - best-effort decode
        return None


def decode_diagnostic_array(data: bytes) -> Optional[dict[str, Any]]:
    """Best-effort decode of diagnostic_msgs/msg/DiagnosticArray."""
    try:
        reader = _CdrReader(data)
        reader.skip(4)  # CDR encapsulation
        reader.read_int32()  # header.stamp.sec
        reader.read_uint32()  # header.stamp.nanosec
        reader.read_string()  # header.frame_id
        count = reader.read_uint32()
        statuses = []
        for _ in range(count):
            level = reader.read_uint8()
            name = reader.read_string()
            message = reader.read_string()
            hardware_id = reader.read_string()
            value_count = reader.read_uint32()
            values = []
            for _ in range(value_count):
                key = reader.read_string()
                value = reader.read_string()
                values.append({"key": key, "value": value})
            statuses.append(
                {
                    "level": level,
                    "levelName": LEVEL_NAMES.get(level, "UNKNOWN"),
                    "name": name,
                    "message": message,
                    "hardwareId": hardware_id,
                    "values": values,
                }
            )
        return {"statuses": statuses}
    except Exception:  # noqa: BLE001 - best-effort decode
        return None


def encode_cdr_tf_message(transforms: list[dict[str, Any]]) -> bytes:
    """Build CDR bytes for a TFMessage (fixture/testing helper, mirrors decode)."""
    writer = _CdrWriter()
    writer.write_uint32(0)  # CDR encapsulation (little endian)
    writer.write_uint32(len(transforms))
    for transform in transforms:
        writer.write_int32(int(transform.get("sec", 0)))
        writer.write_uint32(int(transform.get("nanosec", 0)))
        writer.write_string(transform.get("frame_id", ""))
        writer.write_string(transform.get("child_frame_id", ""))
        for value in transform.get("translation", [0.0, 0.0, 0.0]):
            writer.write_float64(float(value))
        for value in transform.get("rotation", [0.0, 0.0, 0.0, 1.0]):
            writer.write_float64(float(value))
    return writer.to_bytes()


def encode_cdr_diagnostic_array(statuses: list[dict[str, Any]]) -> bytes:
    """Build CDR bytes for a DiagnosticArray (fixture/testing helper, mirrors decode)."""
    writer = _CdrWriter()
    writer.write_uint32(0)  # CDR encapsulation (little endian)
    writer.write_int32(0)  # header.stamp.sec
    writer.write_uint32(0)  # header.stamp.nanosec
    writer.write_string("")  # header.frame_id
    writer.write_uint32(len(statuses))
    for status in statuses:
        writer.write_uint8(int(status.get("level", 0)))
        writer.write_string(status.get("name", ""))
        writer.write_string(status.get("message", ""))
        writer.write_string(status.get("hardware_id", ""))
        values = status.get("values", [])
        writer.write_uint32(len(values))
        for value in values:
            writer.write_string(value.get("key", ""))
            writer.write_string(value.get("value", ""))
    return writer.to_bytes()


# ---------------------------------------------------------------------------
# rosbag statistics (pure functions, reused by tf-audit / diagnostics)
# ---------------------------------------------------------------------------

_CDR_DECODERS: dict[str, Callable[[bytes], Any]] = {
    "std_msgs/msg/Float64": decode_cdr_float64,
    "std_msgs/msg/String": decode_cdr_string,
}


def inspect_rosbag2(path: str, max_decode_samples: int = 5) -> dict[str, Any]:
    """Inspect a rosbag2 bag (directory with metadata.yaml or a ``.db3`` file).

    Reads schema version, topics, per-topic message counts, time range and size
    statistics straight from SQLite. Common types (Float64, String) are decoded
    (best-effort); every other type is explicitly listed as count-only.
    """
    conn, db_path = _open_rosbag_db(path)
    try:
        version_row = conn.execute("SELECT version FROM schema LIMIT 1").fetchone()
        version = int(version_row["version"]) if version_row is not None else None

        topics: list[dict[str, Any]] = []
        for row in conn.execute("SELECT id, name, type, serialization_format FROM topics ORDER BY id"):
            topics.append(
                {
                    "id": int(row["id"]),
                    "name": row["name"],
                    "type": row["type"],
                    "serializationFormat": row["serialization_format"],
                }
            )

        counts: dict[int, dict[str, Any]] = {}
        for row in conn.execute(
            "SELECT topic_id, COUNT(*) AS c, MIN(timestamp) AS mn, MAX(timestamp) AS mx, "
            "AVG(LENGTH(data)) AS avg_len, MIN(LENGTH(data)) AS min_len, MAX(LENGTH(data)) AS max_len "
            "FROM messages GROUP BY topic_id"
        ):
            counts[int(row["topic_id"])] = {
                "count": int(row["c"]),
                "minStamp": int(row["mn"]),
                "maxStamp": int(row["mx"]),
                "avgSizeBytes": round(float(row["avg_len"] or 0.0), 1),
                "minSizeBytes": int(row["min_len"] or 0),
                "maxSizeBytes": int(row["max_len"] or 0),
            }

        overall = conn.execute("SELECT MIN(timestamp) AS mn, MAX(timestamp) AS mx, COUNT(*) AS c FROM messages").fetchone()
        total_messages = int(overall["c"] or 0)
        duration_s = 0.0
        if overall["mn"] is not None and overall["mx"] is not None:
            duration_s = round((overall["mx"] - overall["mn"]) / 1e9, 6)

        issues: list[dict[str, Any]] = []
        topic_results: list[dict[str, Any]] = []
        for topic in topics:
            stats = counts.get(topic["id"], {"count": 0, "minStamp": None, "maxStamp": None, "avgSizeBytes": 0.0, "minSizeBytes": 0, "maxSizeBytes": 0})
            entry: dict[str, Any] = {
                "name": topic["name"],
                "type": topic["type"],
                "serializationFormat": topic["serializationFormat"],
                "count": stats["count"],
                "minStampS": round(stats["minStamp"] / 1e9, 6) if stats["minStamp"] is not None else None,
                "maxStampS": round(stats["maxStamp"] / 1e9, 6) if stats["maxStamp"] is not None else None,
                "avgSizeBytes": stats["avgSizeBytes"],
                "minSizeBytes": stats["minSizeBytes"],
                "maxSizeBytes": stats["maxSizeBytes"],
            }
            decoder = _CDR_DECODERS.get(topic["type"])
            if decoder is None:
                entry["decoded"] = False
                entry["decodeSummary"] = {
                    "decoded": False,
                    "unsupported": True,
                    "reason": f"CDR decode not implemented for {topic['type']}; count-only",
                }
                issues.append(
                    {
                        "severity": "info",
                        "code": "decode.unsupported",
                        "message": f"topic {topic['name']} ({topic['type']}): not decoded (count-only)",
                    }
                )
            else:
                decoded_samples = 0
                first_value = None
                for (data,) in conn.execute(
                    "SELECT data FROM messages WHERE topic_id=? ORDER BY id", (topic["id"],)
                ).fetchmany(max_decode_samples):
                    value = decoder(data)
                    if value is not None:
                        if decoded_samples == 0:
                            first_value = value
                        decoded_samples += 1
                entry["decoded"] = decoded_samples > 0
                entry["decodeSummary"] = {
                    "decoded": entry["decoded"],
                    "samples": decoded_samples,
                    "firstValue": first_value,
                }
                if not entry["decoded"] and stats["count"] > 0:
                    issues.append(
                        {
                            "severity": "warning",
                            "code": "decode.failed",
                            "message": f"topic {topic['name']} ({topic['type']}): could not decode any sample",
                        }
                    )
            topic_results.append(entry)

        return {
            "ok": True,
            "format": "rosbag2",
            "path": os.path.abspath(path),
            "dbPath": os.path.abspath(db_path),
            "version": version,
            "storageIdentifier": "sqlite3",
            "durationS": duration_s,
            "messageCount": total_messages,
            "topicCount": len(topic_results),
            "topics": topic_results,
            "issues": issues,
        }
    finally:
        conn.close()


def read_rosbag_tf_summary(path: str, max_messages: int = 200) -> dict[str, Any]:
    """Summarize /tf and /tf_static from a rosbag (frames, time range, rate estimate)."""
    conn, db_path = _open_rosbag_db(path)
    try:
        topics = {row["name"]: row for row in conn.execute("SELECT id, name, type FROM topics")}
        issues: list[dict[str, Any]] = []
        frames: set[str] = set()
        tf_count = 0
        tf_static_count = 0
        decode_failures = 0
        time_min: Optional[int] = None
        time_max: Optional[int] = None
        tf_min: Optional[int] = None
        tf_max: Optional[int] = None
        for topic_name, is_static in (("/tf", False), ("/tf_static", True)):
            topic = topics.get(topic_name)
            if topic is None:
                issues.append({"code": "tf.missing_topic", "message": f"no {topic_name} topic in bag"})
                continue
            for timestamp, data in conn.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY id LIMIT ?",
                (topic["id"], max_messages),
            ):
                decoded = decode_tf_message(data)
                if decoded is None:
                    decode_failures += 1
                    continue
                for transform in decoded.get("transforms", []):
                    if transform.get("child_frame_id"):
                        frames.add(transform["child_frame_id"])
                    if transform.get("frame_id"):
                        frames.add(transform["frame_id"])
                if time_min is None or timestamp < time_min:
                    time_min = timestamp
                if time_max is None or timestamp > time_max:
                    time_max = timestamp
                if is_static:
                    tf_static_count += 1
                else:
                    tf_count += 1
                    if tf_min is None or timestamp < tf_min:
                        tf_min = timestamp
                    if tf_max is None or timestamp > tf_max:
                        tf_max = timestamp
        if decode_failures:
            issues.append(
                {"code": "tf.decode_failed", "message": f"{decode_failures} TF message(s) could not be CDR-decoded"}
            )
        tf_rate: Optional[float] = None
        if tf_count >= 2 and tf_min is not None and tf_max is not None and tf_max > tf_min:
            tf_rate = round((tf_count - 1) / ((tf_max - tf_min) / 1e9), 4)
        return {
            "path": os.path.abspath(path),
            "frames": sorted(frames),
            "timeRangeS": {
                "start": round(time_min / 1e9, 6) if time_min is not None else None,
                "end": round(time_max / 1e9, 6) if time_max is not None else None,
            },
            "tfRateHz": tf_rate,
            "tfMessageCount": tf_count,
            "tfStaticMessageCount": tf_static_count,
            "messageCount": tf_count + tf_static_count,
            "issues": issues,
        }
    finally:
        conn.close()


def read_rosbag_diagnostics_summary(path: str, max_messages: int = 200) -> dict[str, Any]:
    """Summarize /diagnostics from a rosbag (statuses, error/warning counts)."""
    conn, db_path = _open_rosbag_db(path)
    try:
        topic = conn.execute("SELECT id, name, type FROM topics WHERE name='/diagnostics'").fetchone()
        if topic is None:
            return {
                "path": os.path.abspath(path),
                "statuses": [],
                "errorCount": 0,
                "warningCount": 0,
                "staleCount": 0,
                "messageCount": 0,
                "issues": [{"code": "diagnostics.missing_topic", "message": "no /diagnostics topic in bag"}],
            }
        rows = conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY id LIMIT ?",
            (topic["id"], max_messages),
        ).fetchall()
        statuses: list[dict[str, Any]] = []
        decode_failures = 0
        for _timestamp, data in rows:
            decoded = decode_diagnostic_array(data)
            if decoded is None:
                decode_failures += 1
                continue
            statuses.extend(decoded.get("statuses", []))
        issues: list[dict[str, Any]] = []
        if decode_failures:
            issues.append(
                {"code": "diagnostics.decode_failed", "message": f"{decode_failures} /diagnostics message(s) could not be CDR-decoded"}
            )
        return {
            "path": os.path.abspath(path),
            "statuses": statuses,
            "errorCount": sum(1 for s in statuses if s["level"] == 2),
            "warningCount": sum(1 for s in statuses if s["level"] == 1),
            "staleCount": sum(1 for s in statuses if s["level"] == 3),
            "messageCount": len(rows),
            "issues": issues,
        }
    finally:
        conn.close()


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------


def cmd_ros_graph_snapshot(args: dict[str, Any]) -> dict[str, Any]:
    """Snapshot the live ROS 2 graph: nodes, topics, services, actions."""
    ros_domain = args.get("rosDomain")
    if ros_domain is not None:
        try:
            ros_domain = int(ros_domain)
        except (TypeError, ValueError) as error:
            raise WorkerError(f"rosDomain must be an integer, got {ros_domain!r}") from error
    timeout_s = max(0.5, float(args.get("timeoutS") or 10))
    if not _ros2_available():
        return _unavailable()

    issues: list[dict[str, Any]] = []
    raw: dict[str, str] = {}

    def _call(sub: str, key: str, parser: Callable[[str], Any], default: Any) -> Any:
        try:
            text = _run_ros2([sub, "list", "-t"], timeout=timeout_s, ros_domain=ros_domain)
            raw[key] = text[:8000]
            return parser(text)
        except Exception as error:  # noqa: BLE001 - report per-command failures
            issues.append({"code": "ros2.call_failed", "message": f"ros2 {sub} list -t failed: {error}"})
            return default

    nodes = _call("node", "nodes", _parse_node_list, [])
    topics = _call("topic", "topics", _parse_topic_list, [])
    services = _call("service", "services", _parse_topic_list, [])
    actions = _call("action", "actions", _parse_topic_list, [])

    truncated = False
    for index, topic in enumerate(topics):
        if index >= 20:
            truncated = True
            break
        try:
            text = _run_ros2(["topic", "info", topic["name"], "-v"], timeout=timeout_s, ros_domain=ros_domain)
            info = _parse_topic_info(text)
            topic["publishers"] = info.get("publisherCount")
            topic["subscribers"] = info.get("subscriberCount")
        except Exception as error:  # noqa: BLE001
            topic["publishers"] = None
            topic["subscribers"] = None
            issues.append({"code": "topic.info_failed", "message": f"ros2 topic info {topic['name']} -v failed: {error}"})
    if truncated:
        issues.append(
            {"code": "graph.truncated", "message": "topic pub/sub detail truncated to the first 20 topics"}
        )
    return {
        "ok": True,
        "backend": "ros2",
        "nodes": nodes,
        "topics": topics,
        "services": services,
        "actions": actions,
        "truncated": truncated,
        "raw": raw,
        "issues": issues,
        "inputArgs": {"rosDomain": ros_domain, "timeoutS": timeout_s},
    }


def cmd_ros_topic_profile(args: dict[str, Any]) -> dict[str, Any]:
    """Measure topic rate with ``ros2 topic hz`` for a bounded window."""
    topic = args.get("topic")
    if not topic:
        raise WorkerError("missing required argument 'topic'")
    duration_s = max(0.5, float(args.get("durationS") or 2))
    expected_rate = args.get("rate")
    if expected_rate is not None:
        try:
            expected_rate = float(expected_rate)
        except (TypeError, ValueError) as error:
            raise WorkerError(f"rate must be a number, got {expected_rate!r}") from error
    if not _ros2_available():
        return _unavailable()

    stats = measure_topic_hz(topic, duration_s, window=50, ros_domain=args.get("rosDomain"))
    measured = stats.get("measuredRateHz")
    if measured is None:
        measured = 0.0
    issues: list[dict[str, Any]] = []
    if measured == 0.0:
        issues.append({"code": "rate.zero", "message": f"no messages received on {topic} within {duration_s:g}s (rate 0)"})
    elif expected_rate is not None and measured < expected_rate:
        issues.append(
            {
                "code": "rate.below_expected",
                "message": f"measured rate {measured:.3f} Hz below expected {expected_rate:g} Hz",
            }
        )
    return {
        "ok": True,
        "backend": "ros2",
        "topic": topic,
        "measuredRateHz": round(measured, 4),
        "window": stats.get("window"),
        "samples": stats.get("samples") or 0,
        "minIntervalS": stats.get("minIntervalS"),
        "maxIntervalS": stats.get("maxIntervalS"),
        "stdDevS": stats.get("stdDevS"),
        "issues": issues,
        "inputArgs": {"topic": topic, "durationS": duration_s, "rate": expected_rate},
    }


def cmd_ros_qos_check(args: dict[str, Any]) -> dict[str, Any]:
    """Inspect QoS of a topic's publishers and subscribers."""
    topic = args.get("topic")
    if not topic:
        raise WorkerError("missing required argument 'topic'")
    if not _ros2_available():
        return _unavailable()
    text = _run_ros2(["topic", "info", topic, "-v"], timeout=float(args.get("timeoutS") or 10))
    info = _parse_topic_info(text)
    publisher = info.get("publisherQos")
    subscriber = info.get("subscriberQos")
    issues: list[dict[str, Any]] = []
    compatible: Optional[bool] = None
    if publisher and subscriber:
        pub_rel = publisher.get("reliability")
        sub_rel = subscriber.get("reliability")
        if pub_rel and sub_rel and pub_rel != sub_rel:
            compatible = False
            issues.append(
                {
                    "code": "qos.reliability_mismatch",
                    "message": f"publisher reliability {pub_rel} != subscriber reliability {sub_rel} (incompatible)",
                }
            )
        else:
            compatible = True
    return {
        "ok": True,
        "backend": "ros2",
        "topic": topic,
        "qos": {"publisher": publisher, "subscriber": subscriber},
        "compatible": compatible,
        "issues": issues,
        "raw": text[:4000],
        "inputArgs": {"topic": topic},
    }


def cmd_ros_tf_audit(args: dict[str, Any]) -> dict[str, Any]:
    """Audit TF: frames from /tf_static and rate of /tf (or from a rosbag)."""
    rosbag_path = args.get("rosbagPath")
    if rosbag_path:
        summary = read_rosbag_tf_summary(rosbag_path)
        return {
            "ok": True,
            "backend": "rosbag",
            "path": summary["path"],
            "frames": summary["frames"],
            "tfRateHz": summary["tfRateHz"],
            "timeRangeS": summary["timeRangeS"],
            "tfMessageCount": summary["tfMessageCount"],
            "tfStaticMessageCount": summary["tfStaticMessageCount"],
            "messageCount": summary["messageCount"],
            "issues": summary["issues"],
            "inputArgs": {"rosbagPath": summary["path"]},
        }
    timeout_s = max(0.5, float(args.get("timeoutS") or 2))
    if not _ros2_available():
        return _unavailable()

    issues: list[dict[str, Any]] = []
    frames: list[str] = []
    try:
        text = _run_ros2(["topic", "echo", "/tf_static", "--once"], timeout=timeout_s)
        parsed = _parse_tf_static_echo(text)
        frames = parsed.get("frames", [])
        if parsed.get("parseError"):
            issues.append({"code": "tf.parse_error", "message": f"could not parse /tf_static echo: {parsed['parseError']}"})
    except Exception as error:  # noqa: BLE001
        issues.append({"code": "tf.echo_failed", "message": f"could not read /tf_static: {error}"})
    hz = measure_topic_hz("/tf", timeout_s, window=20, ros_domain=args.get("rosDomain"))
    tf_rate = hz.get("measuredRateHz")
    if tf_rate in (None, 0.0):
        issues.append({"code": "tf.rate_zero", "message": f"no /tf messages within {timeout_s:g}s"})
    return {
        "ok": True,
        "backend": "ros2",
        "frames": frames,
        "tfRateHz": tf_rate,
        "issues": issues,
        "inputArgs": {"timeoutS": timeout_s},
    }


def cmd_ros_diagnostics_snapshot(args: dict[str, Any]) -> dict[str, Any]:
    """Snapshot /diagnostics statuses (live echo or from a rosbag)."""
    rosbag_path = args.get("rosbagPath")
    if rosbag_path:
        summary = read_rosbag_diagnostics_summary(rosbag_path)
        return {
            "ok": True,
            "backend": "rosbag",
            "path": summary["path"],
            "statuses": summary["statuses"],
            "errorCount": summary["errorCount"],
            "warningCount": summary["warningCount"],
            "staleCount": summary["staleCount"],
            "messageCount": summary["messageCount"],
            "issues": summary["issues"],
            "inputArgs": {"rosbagPath": summary["path"]},
        }
    timeout_s = max(0.5, float(args.get("timeoutS") or 5))
    if not _ros2_available():
        return _unavailable()
    try:
        text = _run_ros2(["topic", "echo", "/diagnostics", "--once"], timeout=timeout_s)
    except Exception as error:  # noqa: BLE001
        return {
            "ok": True,
            "backend": "ros2",
            "statuses": [],
            "errorCount": 0,
            "warningCount": 0,
            "staleCount": 0,
            "issues": [{"code": "diagnostics.echo_failed", "message": f"could not read /diagnostics: {error}"}],
            "inputArgs": {"timeoutS": timeout_s},
        }
    parsed = _parse_diagnostics_echo(text)
    statuses = parsed.get("statuses", [])
    issues: list[dict[str, Any]] = []
    if parsed.get("parseError"):
        issues.append({"code": "diagnostics.parse_error", "message": f"could not parse /diagnostics echo: {parsed['parseError']}"})
    return {
        "ok": True,
        "backend": "ros2",
        "statuses": statuses,
        "errorCount": sum(1 for s in statuses if s["level"] == 2),
        "warningCount": sum(1 for s in statuses if s["level"] == 1),
        "staleCount": sum(1 for s in statuses if s["level"] == 3),
        "issues": issues,
        "inputArgs": {"timeoutS": timeout_s},
    }


def cmd_ros_controller_status(args: dict[str, Any]) -> dict[str, Any]:
    """List controller_manager controllers via ``ros2 control list_controllers``."""
    if not _ros2_available():
        return _unavailable()
    text = _run_ros2(["control", "list_controllers"], timeout=float(args.get("timeoutS") or 10))
    controllers = _parse_controllers(text)
    names = args.get("controllerNames")
    if names:
        wanted = set(names)
        controllers = [c for c in controllers if c["name"] in wanted]
    return {
        "ok": True,
        "backend": "ros2",
        "controllers": controllers,
        "issues": [],
        "inputArgs": {"controllerNames": names},
    }


def cmd_ros_moveit_audit(args: dict[str, Any]) -> dict[str, Any]:
    """Audit MoveIt planning groups: SRDF configPath parsing or (experimental) param get."""
    config_path = args.get("configPath")
    if config_path:
        if not os.path.exists(config_path):
            raise WorkerError(f"config file not found: {config_path}")
        parsed = parse_srdf_groups(config_path)
        groups = parsed["groups"]
        requested = args.get("group")
        if requested:
            groups = [g for g in groups if g["name"] == requested]
        return {
            "ok": True,
            "backend": "config",
            "configPath": os.path.abspath(config_path),
            "groups": groups,
            "endEffectors": parsed["endEffectors"],
            "issues": [],
            "inputArgs": {"configPath": os.path.abspath(config_path), "group": requested},
        }
    if not _ros2_available():
        return _unavailable()
    try:
        text = _run_ros2(["param", "get", "/move_group", "robot_description"], timeout=float(args.get("timeoutS") or 10))
    except Exception as error:  # noqa: BLE001
        return {
            "ok": True,
            "backend": "ros2",
            "experimental": True,
            "groups": [],
            "issues": [{"code": "moveit.param_failed", "message": f"experimental: could not read /move_group robot_description: {error}"}],
            "inputArgs": {},
        }
    return {
        "ok": True,
        "backend": "ros2",
        "experimental": True,
        "groups": [],
        "robotDescription": text[:2000],
        "issues": [
            {
                "code": "moveit.experimental",
                "message": "robot_description via 'ros2 param get' is experimental; SRDF configPath parsing is recommended",
            }
        ],
        "inputArgs": {},
    }


def cmd_rosbag_inspect(args: dict[str, Any]) -> dict[str, Any]:
    """Inspect a rosbag2 bag without any ROS installation."""
    path = args.get("path") or args.get("rosbagPath")
    if not path:
        raise WorkerError("missing required argument 'path'")
    return inspect_rosbag2(path)


def cmd_rosbag_start(args: dict[str, Any]) -> dict[str, Any]:
    """Start ``ros2 bag record`` (allowlisted topics only) and track the job."""
    bag_path = args.get("bagPath")
    if not bag_path:
        raise WorkerError("missing required argument 'bagPath'")
    bag_path = os.path.abspath(bag_path)
    # C: drive guard (Windows): never record to the system drive.
    if os.name == "nt" and bag_path[:2].upper() == "C:":
        raise WorkerError(f"refusing to record rosbag on the C: drive: {bag_path}")
    compression = args.get("compression", "none")
    if compression not in ("none", "zstd"):
        raise WorkerError(f"unsupported compression {compression!r}; use 'none' or 'zstd'")
    if not _ros2_available():
        return _unavailable()

    topics = args.get("topics")
    if topics is not None and not isinstance(topics, list):
        raise WorkerError("'topics' must be a list of topic names")
    argv: list[str] = ["bag", "record"]
    if topics:
        argv.extend(str(t) for t in topics)
    argv += ["-o", bag_path]
    if compression == "zstd":
        argv += ["--compression-mode", "file", "--compression-format", "zstd"]
    env = os.environ.copy()
    ros_domain = args.get("rosDomain")
    if ros_domain is not None:
        env["ROS_DOMAIN_ID"] = str(int(ros_domain))
    os.makedirs(os.path.dirname(bag_path) or ".", exist_ok=True)
    try:
        proc = subprocess.Popen(
            ["ros2", *argv],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=_creation_flags(),
        )
    except OSError as error:
        raise WorkerError(f"failed to start ros2 bag record: {error}") from error
    job_id = new_id("rosbag")
    jobs = _load_jobs(args)
    jobs.append(
        {
            "jobId": job_id,
            "pid": proc.pid,
            "bagPath": bag_path,
            "topics": topics,
            "compression": compression,
            "startedAt": time.time(),
        }
    )
    _save_jobs(args, jobs)
    return {
        "ok": True,
        "backend": "ros2",
        "jobId": job_id,
        "bagPath": bag_path,
        "pid": proc.pid,
        "topics": topics or None,
        "compression": compression,
        "note": None if topics else "recording all topics (no allowlist provided)",
        "inputArgs": {"bagPath": bag_path, "topicCount": len(topics) if topics else None},
    }


def cmd_rosbag_stop(args: dict[str, Any]) -> dict[str, Any]:
    """Stop a tracked rosbag recording job by jobId."""
    job_id = args.get("jobId")
    if not job_id:
        raise WorkerError("missing required argument 'jobId'")
    jobs = _load_jobs(args)
    entry = next((job for job in jobs if job.get("jobId") == job_id), None)
    if entry is None:
        raise WorkerError(f"unknown rosbag job id: {job_id}")
    pid = entry.get("pid")
    stopped = False
    if pid:
        try:
            if os.name == "nt":
                result = subprocess.run(
                    ["taskkill", "/PID", str(pid), "/T", "/F"],
                    capture_output=True,
                    timeout=10,
                    creationflags=_creation_flags(),
                )
                stopped = result.returncode == 0
            else:
                os.kill(int(pid), signal.SIGTERM)
                stopped = True
        except ProcessLookupError:
            # the process is already gone; the job is still considered stopped
            stopped = True
        except Exception:  # noqa: BLE001 - process may already be gone
            stopped = False
    remaining = [job for job in jobs if job.get("jobId") != job_id]
    _save_jobs(args, remaining)
    return {"ok": True, "stopped": stopped, "bagPath": entry.get("bagPath"), "jobId": job_id}


def cmd_ros_call_whitelisted_action(args: dict[str, Any]) -> dict[str, Any]:
    """Send a goal for an action, but only if the action is allowlisted."""
    action = args.get("action")
    goal = args.get("goal")
    if not action or not isinstance(goal, dict):
        raise WorkerError("missing required arguments 'action' (str) and 'goal' (object)")
    allowlist = _load_allowlist(args)
    if not allowlist:
        raise WorkerError(
            "no allowlist configured (pass 'allowlist' or create <storeRoot>/.rh/ros-allowlist.json)"
        )
    entry = next((item for item in allowlist if item.get("action") == action), None)
    if entry is None:
        raise WorkerError(f"action not in allowlist: {action}")
    fields = entry.get("fields")
    if fields:
        unknown = [key for key in goal if key not in fields]
        if unknown:
            raise WorkerError(f"goal fields {unknown} not allowed for action {action} (allowed: {fields})")
    if not _ros2_available():
        return _unavailable()

    action_type = entry.get("type")
    if not action_type:
        try:
            text = _run_ros2(["action", "list", "-t"], timeout=10.0)
            for line in text.splitlines():
                match = re.match(rf"^\s*{re.escape(action)}\s*\[(.*)\]\s*$", line)
                if match:
                    action_type = match.group(1).strip()
                    break
        except Exception:  # noqa: BLE001
            action_type = None
    if not action_type:
        return {
            "ok": True,
            "backend": "ros2",
            "action": action,
            "sent": False,
            "reason": "action type unknown (not in allowlist 'type' and not resolvable); goal not sent",
            "goal": goal,
            "inputArgs": {"action": action},
        }
    payload = json.dumps(goal, separators=(",", ":"))
    try:
        text = _run_ros2(
            ["action", "send_goal", action, action_type, payload],
            timeout=float(args.get("timeoutS") or 15),
        )
    except Exception as error:  # noqa: BLE001
        return {
            "ok": True,
            "backend": "ros2",
            "action": action,
            "sent": False,
            "reason": f"send_goal failed: {error}",
            "goal": goal,
            "inputArgs": {"action": action},
        }
    return {
        "ok": True,
        "backend": "ros2",
        "action": action,
        "sent": True,
        "response": text.strip()[:2000],
        "goal": goal,
        "inputArgs": {"action": action},
    }


def _load_allowlist(args: dict[str, Any]) -> list[dict[str, Any]]:
    """Load the action allowlist from args or from <storeRoot>/.rh/ros-allowlist.json."""
    entries = args.get("allowlist")
    if entries is None:
        path = _allowlist_file(args)
        if os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as handle:
                    data = json.load(handle)
            except (OSError, json.JSONDecodeError):
                data = None
            if isinstance(data, list):
                entries = data
            elif isinstance(data, dict):
                entries = data.get("actions") or data.get("allowlist")
    if not isinstance(entries, list):
        return []
    return [item for item in entries if isinstance(item, dict) and item.get("action")]


def _load_jobs(args: dict[str, Any]) -> list[dict[str, Any]]:
    path = _state_file(args)
    if not os.path.exists(path):
        return []
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, list) else []
    except (OSError, json.JSONDecodeError):
        return []


def _save_jobs(args: dict[str, Any], jobs: list[dict[str, Any]]) -> None:
    path = _state_file(args)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(jobs, handle, ensure_ascii=False, indent=2)


# ---------------------------------------------------------------------------
# module interface (contract: COMMANDS + CAPABILITIES)
# ---------------------------------------------------------------------------

COMMANDS: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
    "ros-graph-snapshot": cmd_ros_graph_snapshot,
    "ros-topic-profile": cmd_ros_topic_profile,
    "ros-qos-check": cmd_ros_qos_check,
    "ros-tf-audit": cmd_ros_tf_audit,
    "ros-diagnostics-snapshot": cmd_ros_diagnostics_snapshot,
    "ros-controller-status": cmd_ros_controller_status,
    "ros-moveit-audit": cmd_ros_moveit_audit,
    "rosbag-inspect": cmd_rosbag_inspect,
    "rosbag-start": cmd_rosbag_start,
    "rosbag-stop": cmd_rosbag_stop,
    "ros-call-whitelisted-action": cmd_ros_call_whitelisted_action,
}

CAPABILITIES: list[dict[str, Any]] = [
    {
        "id": "ros.graph_snapshot",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"rosDomain": "integer?", "timeoutS": "number?"},
        "output": "node/topic/service/action graph",
        "risk": "R0-readonly",
        "description": "Snapshot the live ROS 2 graph (nodes, topics, services, actions) via the ros2 CLI.",
    },
    {
        "id": "ros.topic_profile",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"topic": "string", "durationS": "number?", "rate": "number?"},
        "output": "topic rate measurement",
        "risk": "R0-readonly",
        "description": "Measure a topic's publish rate with ros2 topic hz (bounded window).",
    },
    {
        "id": "ros.qos_check",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"topic": "string"},
        "output": "QoS profile compatibility check",
        "risk": "R0-readonly",
        "description": "Inspect publisher/subscriber QoS profiles of a topic and flag reliability mismatches.",
    },
    {
        "id": "ros.tf_audit",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"rosbagPath": "string?", "timeoutS": "number?"},
        "output": "TF frames and rate",
        "risk": "R0-readonly",
        "description": "Audit TF frames (/tf_static) and /tf rate, live or from a rosbag.",
    },
    {
        "id": "ros.diagnostics_snapshot",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"rosbagPath": "string?", "timeoutS": "number?"},
        "output": "diagnostics status snapshot",
        "risk": "R0-readonly",
        "description": "Snapshot /diagnostics statuses with error/warning counts, live or from a rosbag.",
    },
    {
        "id": "ros.controller_status",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"controllerNames": "array?"},
        "output": "controller list",
        "risk": "R0-readonly",
        "description": "List ros2_control controllers and their states.",
    },
    {
        "id": "ros.moveit_audit",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"configPath": "string?", "group": "string?"},
        "output": "MoveIt planning groups",
        "risk": "R0-readonly",
        "description": "Audit MoveIt planning groups from an SRDF config (or experimental param get).",
    },
    {
        "id": "ros.rosbag_inspect",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"path": "string"},
        "output": "rosbag2 inspection report",
        "risk": "R0-readonly",
        "description": "Inspect a rosbag2 bag (topics, counts, time range, sizes, sample decode) without ROS.",
    },
    {
        "id": "ros.rosbag_start",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"bagPath": "string", "topics": "array?", "compression": "string?", "maxDurationS": "number?"},
        "output": "recording job handle",
        "risk": "R2-simulation",
        "description": "Start ros2 bag record with allowlisted topics; refuses the C: drive.",
    },
    {
        "id": "ros.rosbag_stop",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"jobId": "string"},
        "output": "stop confirmation",
        "risk": "R2-simulation",
        "description": "Stop a tracked rosbag recording job.",
    },
    {
        "id": "ros.call_whitelisted_action",
        "kind": "ros",
        "provider": "robotic-harness-worker",
        "input": {"action": "string", "goal": "object", "allowlist": "array?"},
        "output": "goal send result",
        "risk": "R2-simulation",
        "description": "Send a ros2 action goal, but only for allowlisted actions (never arbitrary calls).",
    },
]
