#!/usr/bin/env python3
"""看 /livox/lidar 里点的距离分布，判断雷达到底看到了什么。"""

import time

import rclpy
from rclpy.node import Node

try:
    from livox_interfaces.msg import CustomMsg as LidarMsg
except Exception:
    from livox_ros_driver2.msg import CustomMsg as LidarMsg  # type: ignore

latest = {"msg": None}


def main():
    rclpy.init()
    node = Node("check_lidar_range")

    def cb(msg):
        latest["msg"] = msg

    node.create_subscription(LidarMsg, "/livox/lidar", cb, 10)
    t0 = time.time()
    while time.time() - t0 < 6 and latest["msg"] is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    msg = latest["msg"]
    if msg is None:
        print("没收到 /livox/lidar")
        return

    rs = []
    # 用全部点，别只取前 4000 个——Mid-70 的点是按扫描顺序排的，
    # 只取前段会偏向某个角度扇区，读数不准
    step = max(1, len(msg.points) // 4000)
    for p in msg.points[::step]:
        r = (p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5
        rs.append(r)
    rs.sort()
    n = len(rs)
    print("总点数: %d（取前 %d 个算）" % (msg.point_num, n))
    if n:
        print("最近: %.2f m" % rs[0])
        print("中位: %.2f m" % rs[n // 2])
        print("最远: %.2f m" % rs[-1])
        for th in (0.8, 1.0, 2.0, 5.0, 10.0, 50.0):
            c = sum(1 for r in rs if r >= th)
            print("  距离 >= %5.1f m 的点: %5d 个 (%.1f%%)" % (th, c, 100.0 * c / n))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
