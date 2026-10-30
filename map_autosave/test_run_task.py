"""Offline tests: importing this module never imports ROS or launches a mapper."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import patch

import yaml

import run_task


def grid(width=3, height=2, data=None, resolution=0.05, x=1.25, y=-2.5, yaw=0.0):
    if data is None:
        data = [-1, 0, 25, 26, 65, 100]
    return NS(
        header=NS(frame_id="map", stamp=NS(sec=123, nanosec=456)),
        info=NS(
            width=width, height=height, resolution=resolution,
            origin=NS(
                position=NS(x=x, y=y, z=9.0),
                orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        ),
        data=data,
    )


class GridFilesTests(unittest.TestCase):
    def test_binary_pgm_flips_grid_rows(self):
        image, metadata = run_task.grid_files(grid())
        self.assertEqual(image, b"P5\n3 2\n255\n" + bytes([205, 0, 0, 205, 254, 254]))
        self.assertEqual(metadata["mode"], "trinary")
        self.assertEqual(metadata["negate"], 0)

    def test_threshold_boundaries_and_unknown_values(self):
        image, metadata = run_task.grid_files(
            grid(width=9, height=1, data=[-2, -1, 0, 25, 26, 64, 65, 100, 101]))
        self.assertEqual(image.split(b"\n", 3)[3], bytes([205, 205, 254, 254, 205, 205, 0, 0, 205]))
        self.assertEqual(metadata["free_thresh"], 0.25)
        self.assertEqual(metadata["occupied_thresh"], 0.65)

    def test_resolution_and_nonzero_rotated_origin(self):
        _, metadata = run_task.grid_files(grid(resolution=0.125, yaw=math.pi / 2))
        self.assertEqual(metadata["resolution"], 0.125)
        self.assertEqual(metadata["origin"][:2], [1.25, -2.5])
        self.assertAlmostEqual(metadata["origin"][2], math.pi / 2)

    def test_invalid_dimensions_and_data_size(self):
        cases = [
            grid(width=0), grid(height=0), grid(width=-1),
            grid(width=4_000_001, height=1), grid(data=[0]),
        ]
        for message in cases:
            with self.subTest(width=message.info.width, height=message.info.height, cells=len(message.data)):
                with self.assertRaises(ValueError):
                    run_task.grid_files(message)

    def test_invalid_resolution_and_position(self):
        for resolution in (0, -0.1, float("nan"), float("inf")):
            with self.subTest(resolution=resolution), self.assertRaises(ValueError):
                run_task.grid_files(grid(resolution=resolution))
        for x in (float("nan"), float("inf")):
            with self.subTest(x=x), self.assertRaises(ValueError):
                run_task.grid_files(grid(x=x))


class WriteGridTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)

    def test_yaml_references_complete_generation_and_reports_source(self):
        result = run_task.write_grid(self.directory, grid(yaw=-0.4))
        metadata = yaml.safe_load((self.directory / "map_2d.yaml").read_text())
        image_path = self.directory / metadata["image"]
        self.assertEqual(image_path.read_bytes(), run_task.grid_files(grid(yaw=-0.4))[0])
        self.assertEqual(result["image"], str(image_path))
        self.assertEqual(result["frame_id"], "map")
        self.assertEqual(result["header_stamp_ns"], 123_000_000_456)
        self.assertEqual(result["kind"], "snapshot_of_external_map_topic")
        self.assertIs(result["grid_history_reset_by_this_tool"], False)
        self.assertAlmostEqual(metadata["origin"][2], -0.4)

    def test_same_ros_stamp_still_uses_new_image_and_removes_old_generation(self):
        with patch.object(run_task.time, "monotonic_ns", side_effect=[100, 101]):
            old = run_task.write_grid(self.directory, grid())
            new = run_task.write_grid(self.directory, grid(width=1, height=1, data=[100]))
        self.assertNotEqual(old["image"], new["image"])
        self.assertFalse(Path(old["image"]).exists())
        self.assertEqual(list(self.directory.glob("grid_*.pgm")), [Path(new["image"])])
        metadata = yaml.safe_load((self.directory / "map_2d.yaml").read_text())
        self.assertEqual(metadata["image"], Path(new["image"]).name)
        self.assertEqual(Path(new["image"]).read_bytes(), b"P5\n1 1\n255\n\x00")

    def test_image_is_committed_before_yaml_and_old_image_kept_until_commit(self):
        old = run_task.write_grid(self.directory, grid())
        original_atomic = run_task.atomic_bytes
        calls = []

        def observe(path, data):
            path = Path(path)
            calls.append(path.suffix)
            if path.suffix == ".yaml":
                metadata = yaml.safe_load(data)
                self.assertTrue((self.directory / metadata["image"]).is_file())
                self.assertTrue(Path(old["image"]).is_file())
            original_atomic(path, data)

        with patch.object(run_task, "atomic_bytes", side_effect=observe):
            run_task.write_grid(self.directory, grid())
        self.assertEqual(calls, [".pgm", ".yaml"])
        self.assertFalse(Path(old["image"]).exists())

    def test_failed_yaml_commit_preserves_previous_valid_pair(self):
        old = run_task.write_grid(self.directory, grid())
        yaml_path = self.directory / "map_2d.yaml"
        old_yaml = yaml_path.read_bytes()
        original_atomic = run_task.atomic_bytes

        def fail_yaml(path, data):
            if Path(path).suffix == ".yaml":
                raise OSError("injected YAML commit failure")
            original_atomic(path, data)

        with patch.object(run_task, "atomic_bytes", side_effect=fail_yaml):
            with self.assertRaises(OSError):
                run_task.write_grid(self.directory, grid())
        self.assertEqual(yaml_path.read_bytes(), old_yaml)
        self.assertTrue(Path(old["image"]).is_file())

    def test_invalid_grid_does_not_replace_previous_pair(self):
        old = run_task.write_grid(self.directory, grid())
        yaml_path = self.directory / "map_2d.yaml"
        old_yaml = yaml_path.read_bytes()
        with self.assertRaises(ValueError):
            run_task.write_grid(self.directory, grid(data=[0]))
        self.assertEqual(yaml_path.read_bytes(), old_yaml)
        self.assertTrue(Path(old["image"]).is_file())

    def test_old_image_path_cannot_delete_outside_task_directory(self):
        sentinel = self.directory.parent / (self.directory.name + "_sentinel.pgm")
        sentinel.write_bytes(b"keep")
        self.addCleanup(sentinel.unlink, missing_ok=True)
        (self.directory / "map_2d.yaml").write_text(yaml.safe_dump({"image": str(sentinel)}))
        run_task.write_grid(self.directory, grid())
        self.assertEqual(sentinel.read_bytes(), b"keep")


class OriginalCommandAuditTests(unittest.TestCase):
    def test_mapper_command_uses_original_binary_and_two_unmodified_parameter_files(self):
        """Evaluate only the command-list expression, never main or a ROS import."""
        tree = ast.parse(Path(run_task.__file__).read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        assignments = [node for node in ast.walk(main) if isinstance(node, ast.Assign)
                       and any(isinstance(target, ast.Name) and target.id == "command" for target in node.targets)]
        self.assertEqual(len(assignments), 1)
        self.assertIsInstance(assignments[0].value, ast.List)
        workspace = Path("/example/original_ws")
        config = workspace / "install/fast_livo/share/fast_livo/config"
        binary = workspace / "install/fast_livo/lib/fast_livo/fastlivo_mapping"
        expression = ast.Expression(body=assignments[0].value)
        command = eval(compile(expression, "<command audit>", "eval"),
                       {"__builtins__": {}, "str": str}, {"binary": binary, "config": config})
        self.assertEqual(command, [
            str(binary), "--ros-args", "-r", "__node:=laserMapping",
            "--params-file", str(config / "avia.yaml"),
            "--params-file", str(config / "camera_pinhole.yaml"),
        ])
        self.assertNotIn("-p", command)
        self.assertNotIn("--param", command)
        self.assertFalse(any("sensor_screening" in word or "common.lid_topic" in word
                             or "common.imu_topic" in word or "pcd_save_en" in word for word in command))


if __name__ == "__main__":
    unittest.main()
