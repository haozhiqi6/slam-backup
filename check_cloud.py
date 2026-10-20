#!/usr/bin/env python3
"""检查 /cloud_registered 里到底有没有点，以及点的坐标范围。"""

import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

got = []


def first_points(msg, n=3):
    """从 PointCloud2 里读前 n 个点的 x,y,z（PointXYZI: 4 个 float32）。"""
    out = []
    step = msg.point_step or 16
    for i in range(min(n, msg.width * msg.height)):
        base = i * step
        x, y, z = struct.unpack_from("<fff", msg.data, base)
        out.append((x, y, z))
    return out


def main():
    rclpy.init()
    node = Node("check_cloud")

    def cb(msg):
        got.append(
            (
                msg.width,
                msg.height,
                msg.header.frame_id,
                msg.header.stamp.sec,
                len(msg.data),
                first_points(msg),
            )
        )

    node.create_subscription(PointCloud2, "/cloud_registered", cb, 10)
    t0 = time.time()
    while time.time() - t0 < 8 and len(got) < 6:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("收到 %d 条 /cloud_registered" % len(got))
    for w, h, f, s, n, pts in got[:6]:
        print("  点数=%d  高=%d  frame=%s  stamp=%d  字节=%d" % (w, h, f, s, n))
        for x, y, z in pts:
            print("      点: x=%.3f y=%.3f z=%.3f" % (x, y, z))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
