#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 FAST-LIVO2 的 3D 点云压成 2D 激光，并把里程计发成 tf。

为什么需要它：
    slam_toolbox / AMCL / Nav2 都要 2D 的 /scan 和 odom->base_link 的 tf，
    而 FAST-LIVO2 只给 3D 点云和位姿。

它做两件事：
    1. /cloud_registered + /aft_mapped_to_init  →  /scan (LaserScan, frame=base_link)
       做法：把世界系的点先变回机体系，只保留高度在 [z_min, z_max] 的点，
             按角度分桶，每桶取最近距离。
    2. /aft_mapped_to_init  →  tf: odom -> base_link

用法（SLAM 跑起来之后另开一个终端）：
    source /opt/ros/humble/setup.bash
    source ~/dl/livox_ws/install/setup.bash
    python3 ~/pc2scan_tf.py

常用参数：
    --z-min -0.8   机体系里，低于这个高度的点不要（地面以下）
    --z-max  0.6   高于这个高度的点不要（天花板、吊灯）
    --angle-inc 1.0   角分辨率（度）
    --range-max 12    最大距离（米）

高度是相对"传感器"的，不是相对世界。IMU 装在离地 1 米时，
地面大约在 -1.0，所以默认 -0.8 就能把地面滤掉，保留 0.8 米以上的东西。
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import TransformBroadcaster


def quat_to_rot(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    f = {x.name: x for x in msg.fields}
    if not all(k in f for k in ("x", "y", "z")):
        return np.empty((0, 3), np.float32)
    dt = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4"] * 3,
        "offsets": [f["x"].offset, f["y"].offset, f["z"].offset],
        "itemsize": msg.point_step,
    })
    d = np.frombuffer(bytes(msg.data), dtype=dt)
    return np.stack([d["x"], d["y"], d["z"]], axis=1).astype(np.float32)


class CloudToScan(Node):
    def __init__(self, args):
        super().__init__("pc2scan_tf")
        self.args = args
        self.rot = np.eye(3)
        self.pos = np.zeros(3)
        self.have_pose = False
        self.n_scan = 0

        n = int(round(2 * math.pi / math.radians(args.angle_inc)))
        self.n_bins = n
        self.inc = 2 * math.pi / n
        self.angles = -math.pi + self.inc * np.arange(n)      # -pi .. pi

        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, args.cloud, self.on_cloud, cloud_qos)
        self.create_subscription(Odometry, args.odom, self.on_odom, 20)

        self.pub = self.create_publisher(LaserScan, args.scan, 10)
        self.tfb = TransformBroadcaster(self)

        self.get_logger().info(
            "点云→激光: %s (+%s tf) → %s，高度带 [%.2f, %.2f] m，%d 个角度格"
            % (args.cloud, args.odom, args.scan, args.z_min, args.z_max, n))

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z], dtype=float)
        self.rot = quat_to_rot(q.x, q.y, q.z, q.w)
        self.have_pose = True

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.args.odom_frame
        t.child_frame_id = self.args.base_frame
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.tfb.sendTransform(t)

    # ------------------------------------------------------------------
    def on_cloud(self, msg: PointCloud2):
        if not self.have_pose:
            return
        pts = cloud_xyz(msg)
        if len(pts) == 0:
            return

        # 世界系 -> 机体系：p_b = R^T (p_w - t)
        pb = (pts - self.pos[None, :]) @ self.rot

        z = pb[:, 2]
        keep = (z >= self.args.z_min) & (z <= self.args.z_max)
        pb = pb[keep]
        if len(pb) == 0:
            return

        rng = np.hypot(pb[:, 0], pb[:, 1])
        good = (rng >= self.args.range_min) & (rng <= self.args.range_max)
        pb, rng = pb[good], rng[good]
        if len(rng) == 0:
            return

        ang = np.arctan2(pb[:, 1], pb[:, 0])
        idx = np.floor((ang + math.pi) / self.inc).astype(np.int64)
        np.clip(idx, 0, self.n_bins - 1, out=idx)

        ranges = np.full(self.n_bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx, rng.astype(np.float32))
        ranges[~np.isfinite(ranges)] = self.args.range_max + 1.0

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.args.base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.args.range_min
        scan.range_max = self.args.range_max
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)

        self.n_scan += 1
        if self.n_scan % 30 == 0:
            valid = int(np.isfinite(ranges).sum() - (ranges > self.args.range_max).sum())
            self.get_logger().info(
                "已发 %d 帧 /scan，本帧有效光束 %d / %d"
                % (self.n_scan, valid, self.n_bins))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cloud", default="/cloud_registered")
    p.add_argument("--odom", default="/aft_mapped_to_init")
    p.add_argument("--scan", default="/scan")
    p.add_argument("--z-min", type=float, default=-0.8)
    p.add_argument("--z-max", type=float, default=0.6)
    p.add_argument("--angle-inc", type=float, default=1.0)
    p.add_argument("--range-min", type=float, default=0.3)
    p.add_argument("--range-max", type=float, default=12.0)
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="base_link")
    args, _ = p.parse_known_args()

    rclpy.init()
    node = CloudToScan(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
