#!/usr/bin/env python3
"""采 /aft_mapped_to_init 的位姿，量静止时的漂移速度。

用法: python3 check_drift.py [秒数]
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

samples = []


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

    rclpy.init()
    node = Node("check_drift")

    def cb(msg):
        p = msg.pose.pose.position
        samples.append((time.time(), p.x, p.y, p.z))

    node.create_subscription(Odometry, "/aft_mapped_to_init", cb, 10)

    print("采 %.0f 秒 /aft_mapped_to_init ……（保持机器静止）" % dur)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.1)

    if len(samples) < 2:
        print("样本太少（%d 条），算法可能没在出位姿" % len(samples))
        return

    ts = [s[0] for s in samples]
    dt = ts[-1] - ts[0]
    p0 = samples[0][1:]
    p1 = samples[-1][1:]
    dist = math.dist(p0, p1)

    path = 0.0
    for a, b in zip(samples, samples[1:]):
        path += math.dist(a[1:], b[1:])

    print("样本 %d 条，时长 %.1f 秒" % (len(samples), dt))
    print("起点: x=%.4f y=%.4f z=%.4f" % p0)
    print("终点: x=%.4f y=%.4f z=%.4f" % p1)
    print("首末位移:      %.4f 米" % dist)
    print("累计路程:      %.4f 米" % path)
    if dt > 0:
        print("平均漂移速度:  %.4f 米/秒   （%.1f 毫米/秒）" % (dist / dt, dist / dt * 1000))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
