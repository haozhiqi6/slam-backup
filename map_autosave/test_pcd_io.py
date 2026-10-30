"""Offline tests; all generated PCD files live in TemporaryDirectory."""

from pathlib import Path
import json
import os
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

try:
    from . import pcd_io
except ImportError:
    import pcd_io

HEADER_BYTES = pcd_io.HEADER_BYTES
StreamingPCDWriter = pcd_io.StreamingPCDWriter
decode_cloud = pcd_io.decode_cloud
recover_partial = pcd_io.recover_partial


def field(name, offset, datatype=7, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def cloud(fields, data, *, width=1, height=1, point_step=16, row_step=None, big=False):
    return SimpleNamespace(
        width=width,
        height=height,
        point_step=point_step,
        row_step=width * point_step if row_step is None else row_step,
        is_bigendian=big,
        fields=fields,
        data=data,
    )


def xyzi_cloud(rows=((1, 2, 3, 4),)):
    return cloud(
        [field(name, offset) for name, offset in zip(("x", "y", "z", "intensity"), (0, 4, 8, 12))],
        b"".join(struct.pack("<ffff", *point) for point in rows),
        width=len(rows),
    )


def header_value(path, name):
    header = path.read_bytes()[:HEADER_BYTES].decode("ascii")
    return next(line.split(" ", 1)[1] for line in header.splitlines() if line.startswith(name + " "))


class DecodeCloudTests(unittest.TestCase):
    def test_little_endian_and_missing_intensity(self):
        msg = cloud(
            [field("x", 0), field("y", 4), field("z", 8)],
            struct.pack("<fff", 1.25, -2.5, 3.75),
            point_step=12,
        )
        points, stats = decode_cloud(msg)
        self.assertEqual(points.dtype.names, ("x", "y", "z", "intensity"))
        self.assertEqual(points.dtype.itemsize, 16)
        self.assertEqual(points.tobytes(), struct.pack("<ffff", 1.25, -2.5, 3.75, 0))
        self.assertEqual(stats, {"received_points": 1, "dropped_nonfinite": 0})

    def test_big_endian_row_and_point_padding(self):
        fields = [field("x", 0), field("y", 4), field("z", 8), field("intensity", 12)]
        rows = [
            b"".join(struct.pack(">ffff", n, n + 1, n + 2, n + 3) + b"PAD!" for n in (1, 5)),
            b"".join(struct.pack(">ffff", n, n + 1, n + 2, n + 3) + b"PAD!" for n in (9, 13)),
        ]
        msg = cloud(fields, b"".join(row + b"ROWPAD!!" for row in rows), width=2, height=2,
                    point_step=20, row_step=48, big=True)
        points, stats = decode_cloud(msg)
        self.assertEqual(points.tobytes(), b"".join(struct.pack("<ffff", n, n + 1, n + 2, n + 3)
                                                 for n in (1, 5, 9, 13)))
        self.assertEqual(stats["received_points"], 4)

    def test_every_ros_numeric_datatype(self):
        for datatype, fmt in ((1, "b"), (2, "B"), (3, "h"), (4, "H"),
                              (5, "i"), (6, "I"), (7, "f"), (8, "d")):
            for big in (False, True):
                with self.subTest(datatype=datatype, big=big):
                    size = struct.calcsize(fmt)
                    fields = [field(name, index * size, datatype)
                              for index, name in enumerate(("x", "y", "z", "intensity"))]
                    msg = cloud(fields, struct.pack((">" if big else "<") + fmt * 4, 1, 2, 3, 4),
                                point_step=size * 4, big=big)
                    points, _ = decode_cloud(msg)
                    self.assertEqual(points.tobytes(), struct.pack("<ffff", 1, 2, 3, 4))

    def test_colour_bits_including_nan_never_pass_through_float(self):
        # These include quiet/signalling NaNs when interpreted as float32.
        colours = (0x12345678, 0x7FC12345, 0x7F800001, 0xFFFFFFFF)
        for big in (False, True):
            for colour_name in ("rgb", "rgba"):
                for datatype in (5, 6, 7):
                    with self.subTest(big=big, colour_name=colour_name, datatype=datatype):
                        endian = ">" if big else "<"
                        msg = cloud([field("x", 0), field("y", 4), field("z", 8),
                                     field(colour_name, 12, datatype)],
                                    b"".join(struct.pack(endian + "fffI", 1, 2, 3, colour)
                                             for colour in colours),
                                    width=len(colours), point_step=16, big=big)
                        points, _ = decode_cloud(msg)
                        self.assertEqual(points[colour_name].tolist(), list(colours))
                        self.assertEqual(points[colour_name].dtype, np.dtype("<u4"))

    def test_both_colour_fields_are_retained(self):
        msg = cloud([field("x", 0), field("y", 4), field("z", 8), field("rgb", 12), field("rgba", 16, 6)],
                    struct.pack("<fffII", 1, 2, 3, 0x7F800001, 0xAB12CD34), point_step=20)
        points, _ = decode_cloud(msg)
        self.assertEqual(points.dtype.names, ("x", "y", "z", "intensity", "rgb", "rgba"))
        self.assertEqual(int(points["rgb"][0]), 0x7F800001)
        self.assertEqual(int(points["rgba"][0]), 0xAB12CD34)

    def test_nonfinite_xyz_are_filtered_but_not_intensity(self):
        msg = xyzi_cloud(((1, 2, 3, float("nan")),
                          (float("nan"), 2, 3, 4),
                          (1, float("inf"), 3, 4),
                          (1, 2, -float("inf"), 4)))
        points, stats = decode_cloud(msg)
        self.assertEqual(len(points), 1)
        self.assertTrue(np.isnan(points["intensity"][0]))
        self.assertEqual(stats, {"received_points": 4, "dropped_nonfinite": 3})

    def test_float64_overflow_cannot_create_infinite_output_xyz(self):
        msg = cloud([field("x", 0, 8), field("y", 8, 8), field("z", 16, 8)],
                    struct.pack("<ddd", 1e100, 2, 3), point_step=24)
        points, stats = decode_cloud(msg)
        self.assertEqual(len(points), 0)
        self.assertEqual(stats["dropped_nonfinite"], 1)

    def test_empty_cloud(self):
        points, stats = decode_cloud(xyzi_cloud(()))
        self.assertEqual(len(points), 0)
        self.assertEqual(stats, {"received_points": 0, "dropped_nonfinite": 0})

    def test_invalid_fields_and_buffers(self):
        cases = []
        msg = xyzi_cloud(); msg.fields = msg.fields[:-2]; cases.append(msg)
        msg = xyzi_cloud(); msg.fields.append(field("x", 0)); cases.append(msg)
        msg = xyzi_cloud(); msg.fields[0].datatype = 99; cases.append(msg)
        msg = xyzi_cloud(); msg.fields[0].offset = -1; cases.append(msg)
        msg = xyzi_cloud(); msg.fields[0].offset = 16; cases.append(msg)
        msg = xyzi_cloud(); msg.fields[0].count = 2; cases.append(msg)
        msg = xyzi_cloud(); msg.fields[0].count = 0; cases.append(msg)
        msg = xyzi_cloud(); msg.fields[-1] = field("rgb", 12, 4); cases.append(msg)
        msg = xyzi_cloud(); msg.row_step = 15; cases.append(msg)
        msg = xyzi_cloud(); msg.data = msg.data[:-1]; cases.append(msg)
        msg = xyzi_cloud(); msg.width = -1; cases.append(msg)
        msg = xyzi_cloud(); msg.point_step = 0; cases.append(msg)
        for index, msg in enumerate(cases):
            with self.subTest(case=index), self.assertRaises(ValueError):
                decode_cloud(msg)


class StreamingPCDTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.partial = self.directory / "map_3d.partial.pcd"
        self.final = self.directory / "map_3d.pcd"
        self.points, _ = decode_cloud(xyzi_cloud(((1, 2, 3, 4), (5, 6, 7, 8))))

    def writer(self, dtype=None):
        writer = StreamingPCDWriter(self.partial, self.points.dtype if dtype is None else dtype)
        self.addCleanup(writer.close)
        return writer

    def test_append_checkpoint_and_atomic_finalize(self):
        with patch.object(pcd_io.os, "fsync", wraps=os.fsync) as fsync:
            writer = self.writer()
            writer.append(self.points[:1])
            stats = writer.checkpoint()
            self.assertEqual(stats, {"points": 1, "bytes": HEADER_BYTES + 16})
            checkpoint = json.loads(writer.checkpoint_path.read_text())
            self.assertEqual(checkpoint["points"], 1)
            self.assertEqual(checkpoint["record_size"], 16)
            self.assertEqual(checkpoint["header_bytes"], HEADER_BYTES)
            self.assertEqual(int(header_value(self.partial, "POINTS")), 1)
            writer.append(self.points[1:])
            self.assertEqual(int(header_value(self.partial, "POINTS")), 1)
            stats = writer.finalize(self.final)
            self.assertGreaterEqual(fsync.call_count, 9)
        self.assertFalse(self.partial.exists())
        self.assertTrue(writer.checkpoint_path.exists())
        self.assertEqual(stats, {"points": 2, "bytes": HEADER_BYTES + 32})
        self.assertEqual(int(header_value(self.final, "WIDTH")), 2)
        self.assertEqual(int(header_value(self.final, "POINTS")), 2)
        self.assertEqual(self.final.read_bytes()[HEADER_BYTES:], self.points.tobytes())
        with self.assertRaises(ValueError):
            writer.append(self.points)

    def test_noncontiguous_frame_and_empty_append(self):
        writer = self.writer()
        writer.append(self.points[::2])
        writer.append(self.points[:0])
        writer.finalize(self.final)
        self.assertEqual(self.final.read_bytes()[HEADER_BYTES:], self.points[:1].tobytes())

    def test_colours_keep_binary_bits_and_declared_types(self):
        for colours, types in ((("rgb",), "F F F F F"), (("rgba",), "F F F F U"),
                               (("rgb", "rgba"), "F F F F F U")):
            with self.subTest(colours=colours):
                dtype = np.dtype([(name, "<f4") for name in ("x", "y", "z", "intensity")]
                                 + [(name, "<u4") for name in colours])
                points = np.zeros(1, dtype=dtype)
                for name in colours:
                    points[name] = 0x7F800001
                partial = self.directory / ("_".join(colours) + ".partial.pcd")
                final = self.directory / ("_".join(colours) + ".pcd")
                with StreamingPCDWriter(partial, dtype) as writer:
                    writer.append(points)
                    writer.finalize(final)
                self.assertEqual(header_value(final, "TYPE"), types)
                self.assertEqual(final.read_bytes()[HEADER_BYTES:], points.tobytes())

    def test_zero_points_finalize(self):
        writer = self.writer()
        self.assertEqual(writer.finalize(self.final), {"points": 0, "bytes": HEADER_BYTES})
        self.assertEqual(self.final.stat().st_size, HEADER_BYTES)

    def test_wrong_dtype_and_existing_paths_are_rejected(self):
        writer = self.writer()
        for invalid in (np.zeros(2), np.zeros((1, 2), dtype=self.points.dtype)):
            with self.assertRaises(ValueError):
                writer.append(invalid)
        with self.assertRaises(FileExistsError):
            StreamingPCDWriter(self.partial, self.points.dtype)
        with self.assertRaises(ValueError):
            writer.finalize(self.partial)
        self.final.write_bytes(b"keep existing")
        with self.assertRaises(FileExistsError):
            writer.finalize(self.final)
        self.assertEqual(self.final.read_bytes(), b"keep existing")
        with self.assertRaises(ValueError):
            StreamingPCDWriter(self.directory / "invalid.pcd", np.dtype([("x", "f8")]))
        self.assertFalse((self.directory / "invalid.pcd").exists())

    def test_recovery_recounts_and_truncates_interrupted_tail(self):
        writer = self.writer()
        writer.append(self.points[:1])
        writer.checkpoint()
        writer.append(self.points[1:])
        writer.close()
        with self.partial.open("ab") as file:
            file.write(b"incomplete")
        stats = recover_partial(self.partial, self.final)
        self.assertEqual(stats, {"points": 1, "bytes": HEADER_BYTES + 16, "truncated_bytes": 26,
                                 "available_points": 2, "discarded_uncheckpointed_points": 1})
        self.assertFalse(self.partial.exists())
        self.assertEqual(int(header_value(self.final, "POINTS")), 1)
        self.assertEqual(self.final.read_bytes()[HEADER_BYTES:], self.points[:1].tobytes())

    def test_recovery_accepts_interrupted_header_count_update(self):
        writer = self.writer()
        writer.append(self.points)
        writer.checkpoint()
        writer.close()
        raw = self.partial.read_bytes()
        raw = raw.replace(b"WIDTH 00000000000000000002", b"WIDTH 00000000000000000123")
        self.partial.write_bytes(raw)
        stats = recover_partial(self.partial, self.final)
        self.assertEqual(stats["points"], 2)
        self.assertEqual(int(header_value(self.final, "WIDTH")), 2)

    def test_recovery_initial_zero_checkpoint_discards_uncommitted_frames(self):
        writer = self.writer()
        writer.append(self.points)
        writer.close()
        stats = recover_partial(self.partial, self.final)
        self.assertEqual(stats["points"], 0)
        self.assertEqual(stats["discarded_uncheckpointed_points"], 2)
        self.assertEqual(self.final.stat().st_size, HEADER_BYTES)

    def test_checkpoint_data_is_synced_before_sidecar_publication(self):
        writer = self.writer()
        events = []
        real_fsync = os.fsync
        real_replace = os.replace

        def sync(descriptor):
            events.append("data_sync" if descriptor == writer._file.fileno() else "other_sync")
            return real_fsync(descriptor)

        def replace(source, target):
            events.append("checkpoint_publish" if Path(target) == writer.checkpoint_path else "rename")
            return real_replace(source, target)

        writer.append(self.points)
        with patch.object(pcd_io.os, "fsync", side_effect=sync), \
             patch.object(pcd_io.os, "replace", side_effect=replace):
            writer.checkpoint()
        self.assertEqual(events, ["data_sync", "other_sync", "checkpoint_publish", "other_sync"])

    def test_checkpoint_publication_failure_keeps_old_commit_recoverable(self):
        writer = self.writer()
        writer.append(self.points[:1])
        writer.checkpoint()
        writer.append(self.points[1:])
        with patch.object(pcd_io, "_write_checkpoint", side_effect=OSError("simulated disk error")):
            with self.assertRaises(OSError):
                writer.checkpoint()
        with self.assertRaises(OSError):
            writer.append(self.points)
        writer.close()
        self.assertTrue(self.partial.exists())
        self.assertFalse(self.final.exists())
        stats = recover_partial(self.partial, self.final)
        self.assertEqual(stats["points"], 1)
        self.assertEqual(self.final.read_bytes()[HEADER_BYTES:], self.points[:1].tobytes())

    def test_recovery_rejects_missing_corrupt_and_mismatched_checkpoint(self):
        writer = self.writer()
        writer.append(self.points)
        writer.checkpoint()
        writer.close()
        original = self.partial.read_bytes()
        valid_metadata = json.loads(writer.checkpoint_path.read_text())
        writer.checkpoint_path.unlink()
        with self.assertRaisesRegex(ValueError, "No valid checkpoint"):
            recover_partial(self.partial, self.final)
        invalid_metadata = [b"{bad json", b"[]"]
        for name, value in (("format", "other_tool"), ("record_size", 20), ("source_inode", -1),
                            ("schema", []), ("points", -1), ("points", 100)):
            metadata = dict(valid_metadata)
            metadata[name] = value
            invalid_metadata.append(json.dumps(metadata).encode())
        for content in invalid_metadata:
            with self.subTest(content=content):
                writer.checkpoint_path.write_bytes(content)
                with self.assertRaises(ValueError):
                    recover_partial(self.partial, self.final)
                self.assertEqual(self.partial.read_bytes(), original)
                self.assertFalse(self.final.exists())

    def test_recovery_rejects_payload_shorter_than_committed_checkpoint(self):
        writer = self.writer()
        writer.append(self.points)
        writer.checkpoint()
        writer.close()
        with self.partial.open("r+b") as file:
            file.truncate(HEADER_BYTES + 16)
        original = self.partial.read_bytes()
        with self.assertRaisesRegex(ValueError, "shorter than its checkpoint"):
            recover_partial(self.partial, self.final)
        self.assertEqual(self.partial.read_bytes(), original)

    def test_recovery_never_modifies_unrelated_or_corrupt_files(self):
        for content in (b"unrelated file\n", b"# .PCD v0.7\nDATA binary\n" + b"a" * 1000):
            self.partial.write_bytes(content)
            with self.assertRaises(ValueError):
                recover_partial(self.partial, self.final)
            self.assertEqual(self.partial.read_bytes(), content)
            self.assertFalse(self.final.exists())
        self.partial.unlink()
        writer = self.writer()
        writer.close()
        content = self.partial.read_bytes().replace(b"TYPE F F F F", b"TYPE U F F F")
        self.partial.write_bytes(content)
        with self.assertRaises(ValueError):
            recover_partial(self.partial, self.final)
        self.assertEqual(self.partial.read_bytes(), content)

    def test_recovery_rejects_existing_final_before_truncating(self):
        writer = self.writer()
        writer.append(self.points)
        writer.close()
        with self.partial.open("ab") as file:
            file.write(b"tail")
        original = self.partial.read_bytes()
        self.final.write_bytes(b"keep final")
        with self.assertRaises(FileExistsError):
            recover_partial(self.partial, self.final)
        self.assertEqual(self.partial.read_bytes(), original)
        self.assertEqual(self.final.read_bytes(), b"keep final")


if __name__ == "__main__":
    unittest.main()
