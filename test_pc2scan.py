#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 pc2scan_tf.py 喂假数据：一个 4m×4m 的房间，验证 /scan 输出对不对。"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

CLOUD_TOPIC = "/test_cloud"
ODOM_TOPIC = "/test_odom"
SCAN_TOPIC = "/test_scan"


def room_points():
    xs = np.arange(-2, 2, 0.05)
    ys = np.arange(-2, 2, 0.05)
    walls = [
        np.column_stack([xs, np.full(xs.size, -2.0), np.zeros(xs.size)]),
        np.column_stack([xs, np.full(xs.size, 2.0), np.zeros(xs.size)]),
        np.column_stack([np.full(ys.size, -2.0), ys, np.zeros(ys.size)]),
        np.column_stack([np.full(ys.size, 2.0), ys, np.zeros(ys.size)]),
    ]
    # 地面和天花板：应该被高度过滤掉
    g = np.random.default_rng(0)
    floor = np.column_stack([g.uniform(-2, 2, 2000), g.uniform(-2, 2, 2000),
                             np.full(2000, -1.0)])
    ceil = np.column_stack([g.uniform(-2, 2, 2000), g.uniform(-2, 2, 2000),
                            np.full(2000, 1.5)])
    return np.vstack(walls + [floor, ceil]).astype(np.float32)


class Tester(Node):
    def __init__(self):
        super().__init__("pc2scan_tester")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.cloud_pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, qos)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 10)
        self.pts = room_points()
        self.got = 0
        self.result = None
        self.create_timer(0.1, self.tick)

    def tick(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.height = 1
        msg.width = len(self.pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z"))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(self.pts)
        msg.is_dense = True
        msg.data = self.pts.tobytes()
        self.cloud_pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        self.got += 1
        if self.got >= 3 and self.result is None:
            r = np.array(msg.ranges)
            finite = r[np.isfinite(r) & (r <= msg.range_max)]
            self.result = dict(
                bins=len(r),
                inc=msg.angle_increment,
                valid=len(finite),
                rmin=float(finite.min()) if len(finite) else float("nan"),
                rmax=float(finite.max()) if len(finite) else float("nan"),
                at_zero=float(r[np.argmin(np.abs(np.arange(len(r)) * msg.angle_increment
                                                 + msg.angle_min))]),
            )


def main():
    rclpy.init()
    node = Tester()
    t0 = time.time()
    while time.time() - t0 < 8 and node.result is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.result is None:
        print("失败：没有收到 /test_scan")
        return 1

    r = node.result
    print("收到 %d 帧 /scan" % node.got)
    print("角度格数 : %d（期望 360）" % r["bins"])
    print("角分辨率 : %.3f 度" % math.degrees(r["inc"]))
    print("有效光束 : %d" % r["valid"])
    print("最近距离 : %.2f m（期望 ≈2.00，墙在 ±2 m）" % r["rmin"])
    print("最远距离 : %.2f m（期望 ≈2.83，墙角）" % r["rmax"])
    print("正前方   : %.2f m（期望 ≈2.00）" % r["at_zero"])
    ok = (r["bins"] == 360 and abs(r["rmin"] - 2.0) < 0.15
          and abs(r["at_zero"] - 2.0) < 0.15)
    print("\n结论:", "通过 ✅" if ok else "不通过 ❌")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
