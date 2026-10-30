"""Decode ROS PointCloud2 messages and stream task maps to binary PCD files.

This module deliberately has no ROS dependency. ``decode_cloud`` accepts any
object with PointCloud2's attributes, including test SimpleNamespace objects.
Output records contain little-endian float32 x, y, z and intensity (zero when
absent). Optional rgb/rgba fields contain the source's packed uint32 bits;
colour bits are never passed through floating-point arithmetic. If both colour
fields exist, both are retained. Only points with finite output XYZ survive.

PCD rgb is declared TYPE F, and rgba TYPE U, following the usual PCL layout.
The uint32 rgb array field represents those same bytes without altering NaN
colour bit patterns. A partial PCD's header is updated at checkpoints; an atomic
sidecar records the fsynced payload boundary. Recovery retains only that boundary.
No full-map point array is retained, and no voxel filtering is performed.
"""

from __future__ import annotations

import numbers
import json
import os
from pathlib import Path
import re
import stat
import tempfile

import numpy as np


HEADER_BYTES = 512
_COUNT_DIGITS = 20
_MAX_POINTS = 10**_COUNT_DIGITS - 1
_MARKER = b"# map_autosave streamed binary PCD v1\n"
_CHECKPOINT_FORMAT = "map_autosave_checkpoint_v1"
_ROS_DTYPES = {
    1: "i1",  # PointField.INT8
    2: "u1",  # PointField.UINT8
    3: "i2",  # PointField.INT16
    4: "u2",  # PointField.UINT16
    5: "i4",  # PointField.INT32
    6: "u4",  # PointField.UINT32
    7: "f4",  # PointField.FLOAT32
    8: "f8",  # PointField.FLOAT64
}


def _integer(value, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise ValueError(f"{name} must be an integer")
    value = int(value)
    if value < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return value


def _output_dtype(colours=()) -> np.dtype:
    return np.dtype(
        [(name, "<f4") for name in ("x", "y", "z", "intensity")]
        + [(name, "<u4") for name in colours]
    )


def _validate_output_dtype(dtype) -> np.dtype:
    dtype = np.dtype(dtype)
    for colours in ((), ("rgb",), ("rgba",), ("rgb", "rgba")):
        expected = _output_dtype(colours)
        if dtype == expected:
            return expected
    raise ValueError(
        "dtype must be packed little-endian float32 x/y/z/intensity, "
        "optionally followed by uint32 rgb and/or rgba"
    )


def decode_cloud(msg) -> tuple[np.ndarray, dict]:
    """Return packed points and received_points/dropped_nonfinite counters.

    Numeric XYZ/intensity fields support all eight standard ROS PointField
    datatypes. Selected fields must have count=1; colours must occupy 32 bits.
    Row padding, per-point padding and message byte order are respected.
    Malformed field definitions, strides and truncated buffers raise ValueError.
    """
    width = _integer(msg.width, "width")
    height = _integer(msg.height, "height")
    point_step = _integer(msg.point_step, "point_step")
    row_step = _integer(msg.row_step, "row_step")
    if width * height and point_step == 0:
        raise ValueError("point_step must be positive for a nonempty cloud")
    if row_step < width * point_step:
        raise ValueError("row_step is shorter than a row of points")
    try:
        data = memoryview(msg.data).cast("B")
    except (TypeError, ValueError) as exc:
        raise ValueError("data must be a contiguous bytes-like buffer") from exc
    if len(data) < height * row_step:
        raise ValueError("PointCloud2 data is truncated")

    endian = ">" if msg.is_bigendian else "<"
    fields = {}
    for field in msg.fields:
        name = field.name
        if not isinstance(name, str) or not name or name in fields:
            raise ValueError("PointCloud2 field names must be nonempty and unique")
        datatype = _integer(field.datatype, f"{name}.datatype", 1)
        if datatype not in _ROS_DTYPES:
            raise ValueError(f"Unsupported PointField datatype for {name}: {datatype}")
        dtype = np.dtype(endian + _ROS_DTYPES[datatype])
        offset = _integer(field.offset, f"{name}.offset")
        count = _integer(field.count, f"{name}.count", 1)
        if offset + count * dtype.itemsize > point_step:
            raise ValueError(f"Field {name} extends past point_step")
        fields[name] = (dtype, offset, count)
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError(f"Required field {name} is missing")
    colours = tuple(name for name in ("rgb", "rgba") if name in fields)
    selected = ("x", "y", "z") + (("intensity",) if "intensity" in fields else ()) + colours
    for name in selected:
        dtype, _, count = fields[name]
        if count != 1:
            raise ValueError(f"Selected field {name} must have count=1")
        if name in colours and dtype.itemsize != 4:
            raise ValueError(f"Colour field {name} must occupy 32 bits")

    received = width * height
    points = np.zeros(received, dtype=_output_dtype(colours))
    if received:
        for name in selected:
            source_dtype, offset, _ = fields[name]
            if name in colours:
                source_dtype = np.dtype(endian + "u4")
            values = np.ndarray(
                (height, width),
                dtype=source_dtype,
                buffer=data,
                offset=offset,
                strides=(row_step, point_step),
            )
            with np.errstate(over="ignore", invalid="ignore"):
                points[name] = values.reshape(-1)
        finite = np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
        dropped = int(received - np.count_nonzero(finite))
        if dropped:
            points = points[finite]
    else:
        dropped = 0
    return points, {"received_points": received, "dropped_nonfinite": dropped}


def _header(dtype: np.dtype, count: int) -> bytes:
    if not 0 <= count <= _MAX_POINTS:
        raise ValueError("Point count exceeds the fixed-width PCD header")
    names = dtype.names
    types = ["U" if name == "rgba" else "F" for name in names]
    text = (
        b"# .PCD v0.7 - Point Cloud Data file format\n"
        + _MARKER
        + (
            "VERSION 0.7\n"
            f"FIELDS {' '.join(names)}\n"
            f"SIZE {' '.join(['4'] * len(names))}\n"
            f"TYPE {' '.join(types)}\n"
            f"COUNT {' '.join(['1'] * len(names))}\n"
            f"WIDTH {count:0{_COUNT_DIGITS}d}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {count:0{_COUNT_DIGITS}d}\n"
        ).encode("ascii")
    )
    tail = b"DATA binary\n"
    padding = HEADER_BYTES - len(text) - len(tail) - 2
    if padding < 0:
        raise ValueError("PCD header does not fit its reserved space")
    return text + b"#" + b" " * padding + b"\n" + tail


def _read_tool_header(file) -> np.dtype:
    """Strictly validate our entire format, allowing stale count values."""
    raw = file.read(HEADER_BYTES)
    normalized = re.sub(
        rb"(?m)^(WIDTH|POINTS) [0-9]{20}$",
        lambda match: match.group(1) + b" " + b"0" * _COUNT_DIGITS,
        raw,
    )
    for colours in ((), ("rgb",), ("rgba",), ("rgb", "rgba")):
        dtype = _output_dtype(colours)
        if normalized == _header(dtype, 0):
            return dtype
    raise ValueError("Recovery accepts only intact map_autosave v1 binary PCD headers")


def _check_destination(source: Path, destination: Path) -> None:
    if source.resolve() == destination.resolve():
        raise ValueError("Final destination must differ from the partial file")
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"Final destination already exists: {destination}")
    if not destination.parent.is_dir():
        raise FileNotFoundError(f"Final directory does not exist: {destination.parent}")


def _checkpoint_path(path: Path) -> Path:
    return path.with_name(path.name + ".checkpoint.json")


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _metadata(dtype: np.dtype, points: int, file_stat) -> dict:
    return {
        "format": _CHECKPOINT_FORMAT,
        "header_bytes": HEADER_BYTES,
        "record_size": dtype.itemsize,
        "points": points,
        "schema": [[name, dtype.fields[name][0].str] for name in dtype.names],
        "source_device": file_stat.st_dev,
        "source_inode": file_stat.st_ino,
    }


def _write_checkpoint(path: Path, metadata: dict) -> None:
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix="." + path.name + ".", suffix=".tmp", delete=False,
        ) as file:
            temporary_path = Path(file.name)
            json.dump(metadata, file, sort_keys=True, separators=(",", ":"))
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def _read_checkpoint(path: Path, dtype: np.dtype, file_stat) -> int:
    if path.is_symlink():
        raise ValueError("Refusing a symlink checkpoint")
    try:
        with path.open("r", encoding="utf-8") as file:
            raw = file.read(16385)
        if len(raw) > 16384:
            raise ValueError("Checkpoint metadata is too large")
        metadata = json.loads(raw)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("No valid checkpoint; inspect the partial file manually") from exc
    if not isinstance(metadata, dict):
        raise ValueError("Invalid checkpoint metadata; manual inspection required")
    count = _integer(metadata.get("points"), "checkpoint points")
    if metadata != _metadata(dtype, count, file_stat):
        raise ValueError("Checkpoint format/schema/source mismatch; manual inspection required")
    return count


class StreamingPCDWriter:
    """Append frames to a new partial PCD, then atomically rename at finalize.

    ``path`` is normally ``map_3d.partial.pcd`` in a unique task directory.
    Existing partial or final files are rejected. The caller owns that directory
    and must ensure no other process creates the final destination concurrently.
    ``checkpoint`` fsyncs a correct fixed-size header and payload, then atomically
    writes ``path + '.checkpoint.json'`` and fsyncs its directory. Construction
    commits an initial zero-point checkpoint. The sidecar is kept after finalize.
    ``close`` preserves a partial file for explicit recovery; it never finalizes.
    Stats report point count and total file bytes, including the 512-byte header.
    """

    def __init__(self, path: Path, dtype):
        self.path = Path(path)
        self.checkpoint_path = _checkpoint_path(self.path)
        self.dtype = _validate_output_dtype(dtype)
        self._points = 0
        self._failed = False
        self._file = self.path.open("xb+")
        try:
            self._file.write(_header(self.dtype, 0))
            self.checkpoint()
        except BaseException:
            self._file.close()
            raise

    def _check_open(self):
        if self._file.closed:
            raise ValueError("PCD writer is closed")
        if self._failed:
            raise OSError("PCD writer failed; close it and explicitly recover the partial file")

    def append(self, points: np.ndarray) -> None:
        self._check_open()
        if not isinstance(points, np.ndarray) or points.ndim != 1 or points.dtype != self.dtype:
            raise ValueError("Points must be a one-dimensional array matching the writer dtype")
        new_count = self._points + len(points)
        if new_count > _MAX_POINTS:
            raise ValueError("Point count exceeds the fixed-width PCD header")
        try:
            self._file.write(points.tobytes(order="C"))
        except BaseException:
            self._failed = True
            raise
        self._points = new_count

    def _stats(self) -> dict:
        payload_bytes = self._points * self.dtype.itemsize
        return {"points": self._points, "bytes": HEADER_BYTES + payload_bytes}

    def checkpoint(self) -> dict:
        self._check_open()
        try:
            self._file.seek(0)
            self._file.write(_header(self.dtype, self._points))
            self._file.seek(0, os.SEEK_END)
            self._file.flush()
            os.fsync(self._file.fileno())
            _write_checkpoint(
                self.checkpoint_path,
                _metadata(self.dtype, self._points, os.fstat(self._file.fileno())),
            )
        except BaseException:
            self._failed = True
            raise
        return self._stats()

    def finalize(self, final_path: Path) -> dict:
        self._check_open()
        final_path = Path(final_path)
        _check_destination(self.path, final_path)
        stats = self.checkpoint()
        self.close()
        os.replace(self.path, final_path)
        _fsync_directory(final_path.parent)
        if self.path.parent.resolve() != final_path.parent.resolve():
            _fsync_directory(self.path.parent)
        return stats

    def close(self) -> None:
        self._file.close()

    def __enter__(self):
        self._check_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def recover_partial(path: Path, final_path: Path) -> dict:
    """Explicitly recover only this tool's most recent committed checkpoint.

    An exact tool-format header is required before any write. Header counts may
    be stale or disagree after an interrupted checkpoint. A valid matching
    ``path + '.checkpoint.json'`` sidecar is also required. Uncommitted payload,
    including complete records after that boundary, is discarded. If the payload
    cannot contain the committed count, recovery refuses to guess. Counts are
    rewritten and fsynced before atomic rename and directory fsync. Unrelated
    files and symlinks are rejected. The checkpoint sidecar is kept for inspection.
    As with finalize, the caller owns the destination directory exclusively.
    """
    path, final_path = Path(path), Path(final_path)
    _check_destination(path, final_path)
    if path.is_symlink():
        raise ValueError("Refusing to recover a symlink")
    with path.open("r+b") as file:
        file_stat = os.fstat(file.fileno())
        if not stat.S_ISREG(file_stat.st_mode):
            raise ValueError("Recovery requires a regular file")
        dtype = _read_tool_header(file)
        payload_bytes = file_stat.st_size - HEADER_BYTES
        available_points = payload_bytes // dtype.itemsize
        points = _read_checkpoint(_checkpoint_path(path), dtype, file_stat)
        if points > available_points:
            raise ValueError("Payload is shorter than its checkpoint; manual inspection required")
        truncated_bytes = payload_bytes - points * dtype.itemsize
        header = _header(dtype, points)
        size = HEADER_BYTES + points * dtype.itemsize
        file.truncate(size)
        file.seek(0)
        file.write(header)
        file.flush()
        os.fsync(file.fileno())
    os.replace(path, final_path)
    _fsync_directory(final_path.parent)
    if path.parent.resolve() != final_path.parent.resolve():
        _fsync_directory(path.parent)
    return {
        "points": points,
        "bytes": size,
        "truncated_bytes": truncated_bytes,
        "available_points": available_points,
        "discarded_uncheckpointed_points": available_points - points,
    }
