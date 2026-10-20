#!/usr/bin/env python3
"""量 /path 轨迹：水平走了多远、竖直变化了多少。

用法: python3 check_path.py
（先让机器水平走一段，再跑这个）
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

latest = {"msg": None}


def main():
    rclpy.init()
    node = Node("check_path")

    def cb(msg):
        latest["msg"] = msg

    node.create_subscription(Path, "/path", cb, 10)
    t0 = time.time()
    while time.time() - t0 < 8 and latest["msg"] is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    msg = latest["msg"]
    if msg is None or len(msg.poses) < 2:
        print("没收到 /path，或者轨迹点太少")
        return

    pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]

    # 水平距离（逐点累加 xy 平面上的位移）
    horiz = 0.0
    vert_up = 0.0
    vert_down = 0.0
    for a, b in zip(pts, pts[1:]):
        horiz += math.dist(a[:2], b[:2])
        dz = b[2] - a[2]
        if dz > 0:
            vert_up += dz
        else:
            vert_down -= dz

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]

    print("轨迹点数: %d" % len(pts))
    print()
    print("x 范围: %.3f ~ %.3f 米" % (min(xs), max(xs)))
    print("y 范围: %.3f ~ %.3f 米" % (min(ys), max(ys)))
    print("z 范围: %.3f ~ %.3f 米   (跨度 %.3f 米)" % (min(zs), max(zs), max(zs) - min(zs)))
    print()
    print("水平累计路程: %.3f 米" % horiz)
    print("竖直累计上升: %.3f 米" % vert_up)
    print("竖直累计下降: %.3f 米" % vert_down)
    print("首末高度差:   %+.3f 米" % (pts[-1][2] - pts[0][2]))
    print()
    if horiz > 0.1:
        print("竖直变化 / 水平路程 = %.3f   （这个值大就说明轨迹是斜的）"
              % ((vert_up + vert_down) / horiz))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
