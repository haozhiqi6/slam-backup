#!/usr/bin/env python3
"""Launch the original mapper and save received point clouds in a separate process."""
import argparse
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import numpy as np


def atomic_bytes(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + ".writing")
    with temporary.open("wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path, value):
    atomic_bytes(path, (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode())


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def grid_files(message):
    width, height = int(message.info.width), int(message.info.height)
    resolution = float(message.info.resolution)
    if width <= 0 or height <= 0 or width * height > 4_000_000:
        raise ValueError("Empty grid or grid larger than this tool's 4M-cell limit")
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Invalid grid resolution")
    cells = np.asarray(message.data, dtype=np.int16)
    if cells.size != width * height:
        raise ValueError("Grid dimensions disagree with data")
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[(cells >= 0) & (cells <= 25)] = 254
    pixels[(cells >= 65) & (cells <= 100)] = 0
    position, q = message.info.origin.position, message.info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    origin = [float(position.x), float(position.y), yaw]
    if not all(math.isfinite(value) for value in origin):
        raise ValueError("Invalid grid origin")
    pgm = f"P5\n{width} {height}\n255\n".encode() + np.flipud(pixels.reshape(height, width)).tobytes()
    return pgm, dict(mode="trinary", resolution=resolution, origin=origin,
                     negate=0, occupied_thresh=0.65, free_thresh=0.25)


def write_grid(directory, message):
    """Publish a new image before replacing the YAML which references it."""
    import yaml
    directory = Path(directory)
    data, metadata = grid_files(message)
    stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
    image = f"grid_{stamp}_{time.monotonic_ns()}.pgm"
    yaml_path = directory / "map_2d.yaml"
    old_image = yaml.safe_load(yaml_path.read_text()).get("image") if yaml_path.exists() else None
    atomic_bytes(directory / image, data)
    atomic_bytes(yaml_path, yaml.safe_dump(dict(image=image, **metadata), sort_keys=False).encode())
    if (isinstance(old_image, str) and Path(old_image).name == old_image
            and old_image.startswith("grid_") and old_image.endswith(".pgm")):
        (directory / old_image).unlink(missing_ok=True)
    return dict(yaml=str(yaml_path), image=str(directory/image),
                frame_id=message.header.frame_id, header_stamp_ns=stamp,
                width=int(message.info.width), height=int(message.info.height),
                resolution=float(message.info.resolution),
                kind="snapshot_of_external_map_topic",
                grid_history_reset_by_this_tool=False)


def mapper_processes():
    matches = []
    for proc in Path("/proc").iterdir():
        if proc.name.isdigit():
            try:
                words = proc.joinpath("cmdline").read_bytes().split(b"\0")
                if words and os.fsdecode(words[0]).endswith("/fastlivo_mapping"):
                    matches.append(int(proc.name))
            except OSError:
                pass
    return matches


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/orangepi/dl/livox_ws"))
    parser.add_argument("--output-root", type=Path, default=Path("/home/orangepi/slam_runs"))
    parser.add_argument("--checkpoint-seconds", type=float, default=5.0)
    parser.add_argument("--minimum-start-mib", type=int, default=1024)
    parser.add_argument("--reserve-mib", type=int, default=512)
    parser.add_argument("--recover", type=Path, help="Recover last committed PCD checkpoint of a stopped task")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    from pcd_io import StreamingPCDWriter, decode_cloud, recover_partial
    if args.recover:
        task = args.recover.resolve()
        status_path = task / "status.json"
        if status_path.exists():
            owner = json.loads(status_path.read_text()).get("saver_pid")
            try:
                if owner and b"run_task.py" in Path(f"/proc/{owner}/cmdline").read_bytes():
                    raise RuntimeError("Saver is still running; do not recover a live task")
            except FileNotFoundError:
                pass
        result = recover_partial(task/"map_3d.partial.pcd", task/"map_3d.pcd")
        atomic_json(task/"recovery.json", dict(recovered_at=utc_now(), scope="durable checkpoint only", **result))
        print(json.dumps(result, indent=2))
        return 0
    if args.checkpoint_seconds <= 0 or args.reserve_mib < 64:
        raise ValueError("Positive checkpoint interval and reserve >=64 MiB required")
    if args.minimum_start_mib <= args.reserve_mib:
        raise ValueError("Initial disk requirement must exceed the saving reserve")
    if mapper_processes():
        raise RuntimeError("A mapper is already running; existing processes are never stopped")
    workspace = args.workspace.resolve()
    source = workspace/"src/FASTLIVO2_ROS2-master"
    config = workspace/"install/fast_livo/share/fast_livo/config"
    binary = workspace/"install/fast_livo/lib/fast_livo/fastlivo_mapping"
    audit = [binary, config/"avia.yaml", config/"camera_pinhole.yaml",
             source/"src/LIVMapper.cpp", source/"src/IMU_Processing.cpp", source/"src/voxel_map.cpp"]
    for path in audit:
        if not path.is_file():
            raise FileNotFoundError(path)
    args.output_root.mkdir(parents=True, exist_ok=True)
    lock = (args.output_root/".active.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if shutil.disk_usage(args.output_root).free < args.minimum_start_mib * 1024**2:
        raise RuntimeError("Insufficient disk space to start a saved task")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import PointCloud2

    run = args.output_root/(datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}")
    run.mkdir()
    (run/"config_snapshot").mkdir()
    (run/"previous_logs").mkdir()
    for name in ("avia.yaml", "camera_pinhole.yaml"):
        shutil.copy2(config/name, run/"config_snapshot"/name)
    for name in ("mat_pre.txt", "mat_out.txt", "imu.txt"):
        if (source/"Log"/name).exists():
            shutil.copy2(source/"Log"/name, run/"previous_logs"/name)
    command = [str(binary), "--ros-args", "-r", "__node:=laserMapping",
               "--params-file", str(config/"avia.yaml"),
               "--params-file", str(config/"camera_pinhole.yaml")]
    state = dict(version=1, started_at=utc_now(), task_directory=str(run), saver_pid=os.getpid(),
                 mapper_command=command, status="starting", pcd_state="not_started",
                 source_hashes_before={str(path):digest(path) for path in audit},
                 cloud_qos=dict(reliability="best_effort", depth=5),
                 capture_scope="received /cloud_registered batches; not internal mapper state",
                 grid_scope="external /map snapshot; this tool does not reset or start its publisher",
                 cloud_frames_received=0, cloud_frames_written=0, points_received=0,
                 points_written=0, nonfinite_xyz_discarded=0, cloud_frame_errors=0,
                 odom_messages=0, grid_messages=0, grid_snapshot=None, saving_errors=[],
                 stop_reason=None, checkpoint_seconds=args.checkpoint_seconds)
    atomic_json(run/"status.json", state)
    atomic_bytes(args.output_root/"latest.txt", (str(run)+"\n").encode())
    event_file = trajectory_file = mapper_log = saver = mapper = None
    trajectory = None
    ros_initialized = False
    previous_signal_handlers = {}
    exit_code = 2
    quiet = [False]
    stop = dict(requested=False, reason=None)
    def log(message):
        line = f"{utc_now()} {message}"
        if event_file is not None and not event_file.closed:
            try:
                event_file.write(line+"\n")
            except OSError:
                pass
        if not quiet[0]:
            try:
                print(line, flush=True)
            except OSError:
                quiet[0] = True
    def request_stop(signum, _frame):
        stop.update(requested=True, reason=signal.Signals(signum).name)
    class Saver(Node):
        def __init__(self):
            super().__init__("task_map_autosave")
            self.writer, self.frame_id, self.first_cloud_ns, self.latest_grid = None, None, None, None
            self.writable = True
            cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.saved_subscriptions = [
                self.create_subscription(PointCloud2, "/cloud_registered", self.cloud, cloud_qos),
                self.create_subscription(Odometry, "/aft_mapped_to_init", self.odom, cloud_qos),
                self.create_subscription(OccupancyGrid, "/map", self.grid, grid_qos)]
        def fail_writing(self, reason):
            self.writable = False
            state["pcd_state"] = "partial"
            state["saving_errors"].append(dict(at=utc_now(), reason=str(reason)))
            log("SAVING STOPPED; mapping is not automatically stopped: "+str(reason))
        def cloud(self, message):
            state["cloud_frames_received"] += 1
            state["points_received"] += message.width*message.height
            if not self.writable:
                return
            try:
                points, details = decode_cloud(message)
                state["nonfinite_xyz_discarded"] += details["dropped_nonfinite"]
                if not len(points):
                    return
                frame = message.header.frame_id
                if not frame:
                    raise ValueError("Point cloud frame_id is empty")
                if self.writer is None:
                    self.frame_id = frame
                    self.first_cloud_ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
                    self.writer = StreamingPCDWriter(run/"map_3d.partial.pcd", points.dtype)
                    state.update(coordinate_frame=frame, pcd_fields=list(points.dtype.names), pcd_state="writing")
                if frame != self.frame_id:
                    raise ValueError("Cloud frame_id changed during this task")
                self.writer.append(points)
                state["cloud_frames_written"] += 1
                state["points_written"] += len(points)
            except OSError as error:
                self.fail_writing(error)
            except Exception as error:
                state["cloud_frame_errors"] += 1
                if state["cloud_frame_errors"] <= 3:
                    log("Cloud frame not saved: "+repr(error))
        def odom(self, message):
            state["odom_messages"] += 1
            if self.writable:
                p, q, stamp = message.pose.pose.position, message.pose.pose.orientation, message.header.stamp
                try:
                    trajectory.writerow((f"{stamp.sec}.{stamp.nanosec:09d}",p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                except OSError as error:
                    self.fail_writing(error)
        def grid(self, message):
            state["grid_messages"] += 1
            self.latest_grid = message
        def checkpoint(self):
            state["free_bytes"] = shutil.disk_usage(run).free
            if self.writable and state["free_bytes"] < args.reserve_mib*1024**2:
                self.fail_writing("Free disk space below saving reserve")
            if self.writable:
                try:
                    if self.writer:
                        state["pcd_checkpoint"] = self.writer.checkpoint()
                    trajectory_file.flush()
                    os.fsync(trajectory_file.fileno())
                    if self.latest_grid is not None and self.first_cloud_ns is not None:
                        stamp = self.latest_grid.header.stamp
                        state["grid_snapshot"] = write_grid(run, self.latest_grid)
                        state["grid_snapshot"]["header_stamp_precedes_first_cloud"] = (
                            stamp.sec*10**9+stamp.nanosec < self.first_cloud_ns)
                        state["grid_snapshot"]["publishers"] = [
                            dict(node=item.node_name, namespace=item.node_namespace)
                            for item in self.get_publishers_info_by_topic("/map")]
                except OSError as error:
                    self.fail_writing(error)
                except Exception as error:
                    state["checkpoint_error"] = str(error)
            state["updated_at"] = utc_now()
            try:
                atomic_json(run/"status.json", state)
            except OSError as error:
                log("Could not update task status: "+str(error))
    try:
        event_file = (run/"saver.log").open("a", buffering=1)
        trajectory_file = (run/"trajectory.csv").open("w", newline="", buffering=1)
        trajectory = csv.writer(trajectory_file)
        trajectory.writerow(("timestamp","x","y","z","qx","qy","qz","qw"))
        for signum, handler in ((signal.SIGINT, request_stop), (signal.SIGTERM, request_stop),
                                (signal.SIGHUP, lambda *_: quiet.__setitem__(0, True))):
            previous_signal_handlers[signum] = signal.signal(signum, handler)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        ros_initialized = True
        saver = Saver()
        mapper_log = (run/"mapper.log").open("wb")
        log("Task directory: "+str(run))
        log("No time limit. Ctrl+C requests stop and finalizes received point clouds.")
        log("Closing SSH is not the normal save-and-stop operation.")
        discovery_end = time.monotonic()+2
        while time.monotonic()<discovery_end and not stop["requested"]:
            rclpy.spin_once(saver, timeout_sec=0.1)
        if stop["requested"]:
            state["status"] = "cancelled_before_mapping"
            exit_code = 0
        else:
            if mapper_processes():
                raise RuntimeError("Another mapper started during setup; no mapper was launched by this task")
            env = os.environ.copy()
            env["ROS_LOG_DIR"] = str(run/"ros_log")
            mapper = subprocess.Popen(command, cwd=workspace, env=env, stdin=subprocess.DEVNULL,
                                      stdout=mapper_log, stderr=subprocess.STDOUT, start_new_session=True)
            state.update(status="running", mapper_pid=mapper.pid)
            saver.checkpoint()
            last_checkpoint = last_progress = time.monotonic()
            signalled = False
            while rclpy.ok():
                rclpy.spin_once(saver, timeout_sec=0.1)
                now = time.monotonic()
                if (run/"stop.request").exists():
                    stop.update(requested=True, reason="stop.request")
                if stop["requested"] and not signalled and mapper.poll() is None:
                    try:
                        os.killpg(mapper.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass  # The owned mapper exited between poll() and killpg().
                    signalled = True
                    state["status"] = "stopping"
                    log("Waiting for this task's mapper to exit; no forced kill is issued.")
                if mapper.poll() is not None:
                    state.update(mapper_returncode=mapper.returncode,
                                 stop_reason=stop["reason"] or "mapper_exited")
                    end = time.monotonic()+0.75
                    while time.monotonic()<end:
                        rclpy.spin_once(saver, timeout_sec=0.05)
                    break
                if now-last_checkpoint >= args.checkpoint_seconds:
                    saver.checkpoint()
                    last_checkpoint = now
                if now-last_progress >= 15:
                    last_progress = now
                    log(f"Saved {state['cloud_frames_written']} clouds / {state['points_written']} points.")
            if mapper.poll() is None:
                raise RuntimeError("ROS context ended while the owned mapper is still running")
            saver.checkpoint()
            if saver.writer:
                if saver.writable and state["points_written"]>0:
                    state["pcd_final"] = saver.writer.finalize(run/"map_3d.pcd")
                    state["pcd_state"] = "finalized"
                    log("Saved PCD: "+str(run/"map_3d.pcd"))
                else:
                    state["pcd_state"] = "partial"
            else:
                state["pcd_state"] = "no_cloud_received" if saver.writable else "partial"
            if saver.writable:
                (run/"native_logs").mkdir(exist_ok=True)
                for name in ("mat_pre.txt","mat_out.txt","imu.txt"):
                    if (source/"Log"/name).exists():
                        shutil.copy2(source/"Log"/name, run/"native_logs"/name)
            state["status"] = "finished" if stop["requested"] else "mapper_exited"
            exit_code = 0 if state["pcd_state"]=="finalized" else 2
    except Exception as error:
        state.update(status="saver_error", error=repr(error))
        if saver is not None and saver.writer is not None and state["pcd_state"] != "finalized":
            state["pcd_state"] = "partial"
        if mapper:
            try:
                state["mapper_still_running"] = mapper.poll() is None
            except Exception as poll_error:
                state["mapper_status_error"] = repr(poll_error)
            log("Saver error; mapper is not automatically stopped. Owned PID: "+str(mapper.pid))
        raise
    finally:
        cleanup_errors = []

        def cleanup(label, operation):
            try:
                return operation()
            except Exception as error:
                cleanup_errors.append(dict(stage=label, error=repr(error)))
                log("Cleanup failed at "+label+": "+repr(error))
                return None

        if saver is not None and saver.writer is not None:
            cleanup("pcd_close", saver.writer.close)
        state["ended_at"] = utc_now()
        hashes_after = {}
        for path in audit:
            value = cleanup("source_hash:"+str(path), lambda path=path: digest(path))
            if value is not None:
                hashes_after[str(path)] = value
        state["source_hashes_after"] = hashes_after
        state["original_files_unchanged"] = (
            state["source_hashes_before"] == hashes_after if len(hashes_after) == len(audit) else None)
        if state["original_files_unchanged"] is False:
            cleanup_errors.append(dict(stage="source_audit", error="Original audited files changed during the task"))
        if trajectory_file is not None:
            def sync_trajectory():
                trajectory_file.flush()
                os.fsync(trajectory_file.fileno())
            cleanup("trajectory_sync", sync_trajectory)
            cleanup("trajectory_close", trajectory_file.close)
        if mapper_log is not None:
            cleanup("mapper_log_close", mapper_log.close)
        if saver is not None:
            cleanup("ros_node_destroy", saver.destroy_node)
        if ros_initialized:
            def shutdown_ros():
                if rclpy.ok():
                    rclpy.shutdown()
            cleanup("ros_shutdown", shutdown_ros)
        for signum, handler in previous_signal_handlers.items():
            cleanup("signal_restore:"+str(signum), lambda signum=signum, handler=handler: signal.signal(signum, handler))
        if mapper is not None:
            def record_mapper_status():
                state["mapper_still_running"] = mapper.poll() is None
            cleanup("mapper_status", record_mapper_status)
        if event_file is not None:
            cleanup("event_log_close", event_file.close)
        cleanup("task_lock_close", lock.close)
        if cleanup_errors:
            state["cleanup_errors"] = cleanup_errors
            state["status"] = "saver_error"
            if state["pcd_state"] == "writing":
                state["pcd_state"] = "partial"
            exit_code = 2
        try:
            atomic_json(run/"status.json", state)
        except Exception as error:
            exit_code = 2
            log("Final metadata write failed: "+repr(error))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
