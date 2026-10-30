"""Offline lifecycle tests: fake ROS and fake mapper; no nodes or hardware start."""
from contextlib import ExitStack
import json
from pathlib import Path
import signal
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest import mock

import numpy as np

import run_task


def cloud_message(clock):
    points = np.array([(clock, 2, 3, 4), (5, 6, 7, 8)], dtype="<f4")
    stamp = 1_700_000_000_000_000_000 + int(clock * 1e9)
    return NS(header=NS(frame_id="camera_init", stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9)),
              fields=[NS(name=name, offset=index*4, datatype=7, count=1)
                      for index, name in enumerate(("x","y","z","intensity"))],
              width=2, height=1, point_step=16, row_step=32,
              is_bigendian=False, data=points.tobytes())


def grid_message(clock):
    stamp = cloud_message(clock).header.stamp
    return NS(header=NS(frame_id="camera_init", stamp=stamp),
              info=NS(width=2, height=2, resolution=0.1,
                      origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0.,y=0.,z=0.,w=1.))),
              data=[0, 100, -1, 0])


class LifecycleTests(unittest.TestCase):
    def exercise(self, mode="signal", low_disk=False, fail_init=False, fail_subscription=False,
                 runtime_error=False, zero_grid_stamp=False):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        workspace, outputs = root/"workspace", root/"outputs"
        for relative in (
            "install/fast_livo/lib/fast_livo/fastlivo_mapping",
            "install/fast_livo/share/fast_livo/config/avia.yaml",
            "install/fast_livo/share/fast_livo/config/camera_pinhole.yaml",
            "src/FASTLIVO2_ROS2-master/src/LIVMapper.cpp",
            "src/FASTLIVO2_ROS2-master/src/IMU_Processing.cpp",
            "src/FASTLIVO2_ROS2-master/src/voxel_map.cpp",
            "src/FASTLIVO2_ROS2-master/Log/mat_pre.txt",
            "src/FASTLIVO2_ROS2-master/Log/mat_out.txt",
            "src/FASTLIVO2_ROS2-master/Log/imu.txt",
        ):
            path = workspace/relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("immutable test fixture\n")
        before = {p: p.read_bytes() for p in workspace.rglob("*") if p.is_file()}
        clock, holder, handlers, sent = [0.0], {}, {}, []
        test = self

        class Node:
            def __init__(self, name):
                self.callbacks = {}
                holder["node"] = self
            def create_subscription(self, kind, topic, callback, qos):
                if fail_subscription:
                    raise RuntimeError("fake subscription failure")
                self.callbacks[topic] = callback
                return object()
            def get_publishers_info_by_topic(self, topic):
                return [NS(node_name="costmap", node_namespace="/costmap")]
            def destroy_node(self):
                holder["destroyed"] = True

        class Mapper:
            def __init__(self, command, **kwargs):
                self.command = command
                self.kwargs = kwargs
                self.returncode = None
                self.pid = 987654321
                holder["mapper"] = self
            def poll(self):
                if mode == "exit" and clock[0] >= 8:
                    self.returncode = 1
                return self.returncode

        def kill_group(pid, signum):
            test.assertEqual(pid, holder["mapper"].pid)
            test.assertEqual(signum, signal.SIGINT)
            sent.append((pid, signum))
            holder["mapper"].returncode = 0

        def spin(node, timeout_sec):
            clock[0] += timeout_sec
            child = holder.get("mapper")
            if mode == "cancel" and clock[0] >= 0.2:
                handlers[signal.SIGINT](signal.SIGINT, None)
            if child is None or child.poll() is not None:
                return
            if runtime_error and clock[0] >= 3:
                raise RuntimeError("fake executor failure")
            node.callbacks["/cloud_registered"](cloud_message(clock[0]))
            node.callbacks["/aft_mapped_to_init"](
                NS(header=cloud_message(clock[0]).header,
                   pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),
                                   orientation=NS(x=0.,y=0.,z=0.,w=1.)))))
            grid = grid_message(clock[0])
            if zero_grid_stamp:
                grid.header.stamp = NS(sec=0, nanosec=0)
            node.callbacks["/map"](grid)
            if clock[0] >= 9:
                if mode == "signal":
                    handlers[signal.SIGINT](signal.SIGINT, None)
                elif mode == "file":
                    run = Path((outputs/"latest.txt").read_text().strip())
                    (run/"stop.request").touch()

        fake_ros = ModuleType("rclpy")
        fake_ros.init = mock.Mock(side_effect=RuntimeError("fake init failure") if fail_init else None)
        fake_ros.shutdown = mock.Mock()
        fake_ros.ok = lambda: True
        fake_ros.spin_once = spin
        fake_node = ModuleType("rclpy.node")
        fake_node.Node = Node
        fake_qos = ModuleType("rclpy.qos")
        fake_qos.QoSProfile = lambda **kwargs: NS(**kwargs)
        fake_qos.ReliabilityPolicy = NS(BEST_EFFORT=1, RELIABLE=2)
        fake_qos.DurabilityPolicy = NS(TRANSIENT_LOCAL=1)
        fake_signals = ModuleType("rclpy.signals")
        fake_signals.SignalHandlerOptions = NS(NO=0)
        nav = ModuleType("nav_msgs.msg")
        nav.OccupancyGrid = nav.Odometry = object
        sensor = ModuleType("sensor_msgs.msg")
        sensor.PointCloud2 = object
        modules = {"rclpy":fake_ros, "rclpy.node":fake_node, "rclpy.qos":fake_qos,
                   "rclpy.signals":fake_signals, "nav_msgs":ModuleType("nav_msgs"),
                   "nav_msgs.msg":nav, "sensor_msgs":ModuleType("sensor_msgs"),
                   "sensor_msgs.msg":sensor}
        def disk_usage(_):
            free = (200 if low_disk and clock[0] >= 8 else 4096) * 1024**2
            return NS(free=free)

        with ExitStack() as stack:
            stack.enter_context(mock.patch.dict("sys.modules", modules))
            stack.enter_context(mock.patch.object(run_task, "mapper_processes", return_value=[]))
            stack.enter_context(mock.patch.object(run_task.time, "monotonic", side_effect=lambda:clock[0]))
            stack.enter_context(mock.patch.object(run_task.signal, "signal",
                               side_effect=lambda sig, handler:handlers.__setitem__(sig, handler)))
            stack.enter_context(mock.patch.object(run_task.os, "killpg", side_effect=kill_group))
            stack.enter_context(mock.patch.object(run_task.subprocess, "Popen", side_effect=Mapper))
            stack.enter_context(mock.patch.object(run_task.shutil, "disk_usage", side_effect=disk_usage))
            stack.enter_context(mock.patch("builtins.print"))
            code = None
            error = None
            try:
                code = run_task.main(["--workspace",str(workspace),"--output-root",str(outputs)])
            except RuntimeError as caught:
                error = caught
        run = Path((outputs/"latest.txt").read_text().strip())
        status = json.loads((run/"status.json").read_text())
        self.assertEqual(before, {p:p.read_bytes() for p in before})
        return NS(code=code, error=error, run=run, status=status, holder=holder,
                  sent=sent, ros=fake_ros, workspace=workspace)

    def test_ctrl_c_finalizes_received_clouds(self):
        result = self.exercise()
        self.assertIsNone(result.error)
        self.assertEqual(result.code, 0)
        self.assertEqual(result.status["pcd_state"], "finalized")
        self.assertGreater(result.status["points_written"], 0)
        self.assertTrue((result.run/"map_3d.pcd").is_file())
        self.assertEqual(len(result.sent), 1)
        self.assertTrue(result.status["original_files_unchanged"])
        self.assertTrue(result.holder["mapper"].kwargs["start_new_session"])
        self.assertEqual(result.holder["mapper"].command[-4:], [
            "--params-file",str(result.workspace/"install/fast_livo/share/fast_livo/config/avia.yaml"),
            "--params-file",str(result.workspace/"install/fast_livo/share/fast_livo/config/camera_pinhole.yaml")])
        self.assertTrue((result.run/"map_2d.yaml").is_file())
        self.assertTrue(result.holder.get("destroyed"))
        result.ros.shutdown.assert_called_once()

    def test_stop_file_only_signals_owned_mapper(self):
        result = self.exercise(mode="file")
        self.assertEqual(result.status["stop_reason"], "stop.request")
        self.assertEqual(result.status["pcd_state"], "finalized")
        self.assertEqual(len(result.sent), 1)

    def test_zero_stamp_external_grid_is_saved_with_provenance(self):
        result = self.exercise(zero_grid_stamp=True)
        self.assertTrue((result.run/"map_2d.yaml").is_file())
        snapshot = result.status["grid_snapshot"]
        self.assertEqual(snapshot["header_stamp_ns"], 0)
        self.assertTrue(snapshot["header_stamp_precedes_first_cloud"])
        self.assertFalse(snapshot["grid_history_reset_by_this_tool"])

    def test_unexpected_mapper_exit_still_preserves_map(self):
        result = self.exercise(mode="exit")
        self.assertEqual(result.status["mapper_returncode"], 1)
        self.assertEqual(result.status["stop_reason"], "mapper_exited")
        self.assertEqual(result.status["pcd_state"], "finalized")
        self.assertEqual(result.sent, [])

    def test_low_disk_keeps_partial_and_does_not_stop_mapper_itself(self):
        result = self.exercise(low_disk=True)
        self.assertEqual(result.status["pcd_state"], "partial")
        self.assertFalse((result.run/"map_3d.pcd").exists())
        self.assertTrue((result.run/"map_3d.partial.pcd").exists())
        self.assertTrue(result.status["saving_errors"])
        self.assertEqual(len(result.sent), 1)  # The later explicit Ctrl+C, not the disk warning.

    def test_init_failure_does_not_leave_starting_status(self):
        result = self.exercise(fail_init=True)
        self.assertIsNotNone(result.error)
        self.assertEqual(result.status["status"], "saver_error")
        self.assertNotIn("mapper", result.holder)
        self.assertEqual(result.sent, [])

    def test_subscription_failure_cleans_initialized_ros(self):
        result = self.exercise(fail_subscription=True)
        self.assertIsNotNone(result.error)
        self.assertEqual(result.status["status"], "saver_error")
        self.assertNotIn("mapper", result.holder)
        result.ros.shutdown.assert_called_once()

    def test_saver_runtime_failure_does_not_kill_mapper(self):
        result = self.exercise(runtime_error=True)
        self.assertIsNotNone(result.error)
        self.assertEqual(result.status["status"], "saver_error")
        self.assertTrue(result.status["mapper_still_running"])
        self.assertEqual(result.sent, [])

    def test_cancel_before_start_never_launches_mapper(self):
        result = self.exercise(mode="cancel")
        self.assertEqual(result.status["status"], "cancelled_before_mapping")
        self.assertNotIn("mapper", result.holder)
        self.assertEqual(result.sent, [])


if __name__ == "__main__":
    unittest.main()
