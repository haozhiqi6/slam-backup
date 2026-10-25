#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采 6 秒点云，打印 z 的分位数，用来判断地面估计为什么偏。"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


def to_xyz(msg):
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


rclpy.init()
n = Node("z_stats")
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
frames = []
acc = []


def cb(m):
    p = to_xyz(m)
    frames.append(1)
    if p.size:
        acc.append(p[::37])       # 抽样，省内存


n.create_subscription(PointCloud2, "/cloud_registered", cb, qos)
t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.1)

if not acc:
    print("没收到点云")
else:
    pts = np.vstack(acc)
    z = pts[:, 2]
    print("帧数 %d，抽样点 %d" % (len(frames), len(z)))
    print("z 分位数：")
    for q in (0.1, 1, 5, 10, 25, 50, 75, 90, 99):
        print("  p%-4s = %8.3f m" % (q, np.percentile(z, q)))
    print("z 范围 = %.3f ~ %.3f m" % (z.min(), z.max()))
    xy = np.linalg.norm(pts[:, :2], axis=1)
    print("水平距离中位数 = %.2f m，最大 = %.2f m" % (np.median(xy), xy.max()))
    print("\n按水平距离分段，看最低那部分点（地面）的 z：")
    print("  距离段        点数     p5_z     p25_z    p50_z")
    for lo, hi in ((0, 1), (1, 2), (2, 3), (3, 4), (4, 5),
                   (5, 6), (6, 7), (7, 9)):
        m = (xy >= lo) & (xy < hi)
        if m.sum() < 50:
            continue
        zz = z[m]
        print("  %3d~%-3d m   %6d  %7.3f  %7.3f  %7.3f"
              % (lo, hi, m.sum(),
                 np.percentile(zz, 5), np.percentile(zz, 25), np.percentile(zz, 50)))

    # 侧视直方图：横轴=水平距离，纵轴=高度
    print("\n侧视图（横=距离 0~9m，纵=高度 +1.0~-3.5m）")
    dmax, zmin, zmax = 9.0, -3.5, 1.0
    nd, nz = 45, 18
    H, _, _ = np.histogram2d(
        xy, z,
        bins=[np.linspace(0, dmax, nd + 1), np.linspace(zmin, zmax, nz + 1)],
    )
    H = np.flipud(H.T)          # 行 0 = 最高
    H = np.log1p(H)
    mx = H.max() if H.max() > 0 else 1.0
    ramp = " .:-=+*#%@"
    for r in range(nz):
        zc = zmax - (zmax - zmin) * (r + 0.5) / nz
        line = "".join(
            ramp[min(len(ramp) - 1, int(v / mx * len(ramp)))] for v in H[r]
        )
        print("%+5.1f |%s" % (zc, line))
    print("      " + "-" * nd)
    print("       距离 0" + " " * (nd - 12) + "9 m")

rclpy.shutdown()
