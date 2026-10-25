#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 3D 点云做成标准的 2D 占据栅格地图（OccupancyGrid）

输出两个文件，格式和 ROS 的 map_server / Nav2 完全兼容：
    mymap.pgm     图片：黑=障碍，白=空地，灰=未知
    mymap.yaml    描述文件：分辨率、原点、阈值

用法（SLAM 正在跑）：
    python3 cloud_to_grid.py --duration 40

    输出 ~/mymap.pgm 和 ~/mymap.yaml

常用参数：
    --duration 40    采集多少秒
    --res 0.05       栅格大小（米）
    --clearance 0.15 高于地面多少米算障碍
    --height 2.0     最高只看到离地 2 米（再高当噪声）
    --inflate 0.15   障碍物膨胀半径（米），给导航留安全余量
    --floor -1.0     手动指定地面高度（默认自动估计）
    --demo           不连 ROS，用假数据测试

原理（射线投射）：
    1. 每一帧知道雷达在哪（传感器原点）
    2. 每个点连一条射线：从原点 → 点
    3. 射线**经过**的格子 = 确认是空的（free）
    4. 射线**终点**的格子 = 有障碍（occupied）
    5. 所有帧的投票累加（对数几率 log-odds）
    6. 没被任何射线扫到的格子 = 未知（unknown）

这比"只把点投到平面上"高明的地方在于：
    它能区分"这里是空的"和"我不知道这里是什么"，
    导航规划器必须要有这个区分。
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import time
import zlib

import numpy as np


# ----------------------------------------------------------------------
# 文件输出
# ----------------------------------------------------------------------
def write_png(path: str, gray: np.ndarray) -> None:
    gray = np.ascontiguousarray(gray.astype(np.uint8))
    h, w = gray.shape
    raw = b"".join(b"\x00" + gray[row].tobytes() for row in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def write_pgm(path: str, img: np.ndarray) -> None:
    """标准 P5 PGM，ROS map_server 认这个格式。"""
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(np.ascontiguousarray(img.astype(np.uint8)).tobytes())


def write_yaml(path: str, image: str, res: float, origin: tuple[float, float]) -> None:
    text = (
        "image: %s\n"
        "resolution: %.4f\n"
        "origin: [%.4f, %.4f, 0.0000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    ) % (image, res, origin[0], origin[1])
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


# ----------------------------------------------------------------------
# 采集
# ----------------------------------------------------------------------
class FrameStore:
    """只存每帧的原点和（抽样后的）障碍点，避免占内存。"""

    def __init__(self, max_points_per_frame: int = 400) -> None:
        self.max_points_per_frame = max_points_per_frame
        self.origin = np.zeros(2)
        self.frames: list[tuple[np.ndarray, np.ndarray]] = []
        self.n_frames = 0
        self.n_points = 0

    def add(self, pts_xyz: np.ndarray) -> None:
        if pts_xyz.size == 0:
            return
        self.n_points += len(pts_xyz)
        if len(pts_xyz) > self.max_points_per_frame:
            idx = np.linspace(0, len(pts_xyz) - 1,
                              self.max_points_per_frame).astype(int)
            pts_xyz = pts_xyz[idx]
        self.frames.append((self.origin.copy(), pts_xyz))
        self.n_frames += 1


def xyz_from_pointcloud2(msg) -> np.ndarray:
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


def run_ros(args, store: FrameStore) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = Node("cloud_to_grid")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

    def odom_cb(msg):
        store.origin[0] = msg.pose.pose.position.x
        store.origin[1] = msg.pose.pose.position.y

    def cloud_cb(msg):
        store.add(xyz_from_pointcloud2(msg))
        if store.n_frames % 30 == 0:
            node.get_logger().info("已收 %d 帧" % store.n_frames)

    node.create_subscription(Odometry, args.odom_topic, odom_cb, 10)
    node.create_subscription(PointCloud2, args.topic, cloud_cb, qos)

    node.get_logger().info(
        "采集 %.0f 秒：%s（栅格 %.2f m）" % (args.duration, args.topic, args.res))
    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("采集结束，共 %d 帧" % store.n_frames)
    node.destroy_node()
    rclpy.shutdown()


# ----------------------------------------------------------------------
# 建图核心
# ----------------------------------------------------------------------
class OccupancyMapper:
    def __init__(self, res: float, clearance: float, height: float,
                 floor: float | None, inflate: float) -> None:
        self.res = res
        self.clearance = clearance
        self.height = height
        self.floor_hint = floor
        self.inflate_cells = max(0, int(round(inflate / res)))
        self.log = np.zeros((0, 0), dtype=np.float32)
        self.x0 = 0.0
        self.y0 = 0.0
        self.occ_pts: list[np.ndarray] = []

    # ---- 先收集所有障碍点，用来估计地面 ----
    def collect(self, pts_xyz: np.ndarray) -> None:
        if pts_xyz.size:
            self.occ_pts.append(pts_xyz)

    def _grow(self, need_x0, need_x1, need_y0, need_y1) -> None:
        """保证网格覆盖到给定范围。"""
        if self.log.size == 0:
            w = int(need_x1 - need_x0 + 1)
            h = int(need_y1 - need_y0 + 1)
            self.log = np.zeros((h, w), dtype=np.float32)
            self.x0, self.y0 = need_x0, need_y0
            return

        cx0, cy0 = self.x0, self.y0
        cx1 = cx0 + self.log.shape[1] - 1
        cy1 = cy0 + self.log.shape[0] - 1
        if (need_x0 >= cx0 and need_x1 <= cx1
                and need_y0 >= cy0 and need_y1 <= cy1):
            return

        nx0, nx1 = int(min(cx0, need_x0)), int(max(cx1, need_x1))
        ny0, ny1 = int(min(cy0, need_y0)), int(max(cy1, need_y1))
        new = np.zeros((ny1 - ny0 + 1, nx1 - nx0 + 1), dtype=np.float32)
        new[cy0 - ny0:cy0 - ny0 + self.log.shape[0],
            cx0 - nx0:cx0 - nx0 + self.log.shape[1]] = self.log
        self.log = new
        self.x0, self.y0 = nx0, ny0

    def build(self, store: FrameStore) -> dict:
        if not store.frames:
            raise RuntimeError("没有采集到点云")

        # 1) 估计地面高度
        allz = np.concatenate([p[:, 2] for p in self.occ_pts]) if self.occ_pts else np.array([0.0])
        floor = float(np.percentile(allz, 5)) if self.floor_hint is None else float(self.floor_hint)
        lo = floor + self.clearance
        hi = floor + self.height

        # 2) 逐帧射线投射
        for origin, pts in store.frames:
            keep = (pts[:, 2] >= lo) & (pts[:, 2] <= hi)
            p = pts[keep]
            if len(p) < 5:
                continue
            ex, ey = p[:, 0], p[:, 1]

            # 射线采样（每 0.4 个格子一个采样点）
            d = np.hypot(ex - origin[0], ey - origin[1])
            kmax = int(np.ceil(d.max() / (self.res * 0.4))) + 2
            kmax = min(max(kmax, 2), 2000)
            t = np.linspace(0.0, 1.0, kmax)[None, :]

            xs = origin[0] + (ex[:, None] - origin[0]) * t
            ys = origin[1] + (ey[:, None] - origin[1]) * t
            cxs = np.floor(xs / self.res).astype(np.int64).ravel()
            cys = np.floor(ys / self.res).astype(np.int64).ravel()

            ox = np.floor(origin[0] / self.res).astype(np.int64)
            oy = np.floor(origin[1] / self.res).astype(np.int64)
            ecx = np.floor(ex / self.res).astype(np.int64)
            ecy = np.floor(ey / self.res).astype(np.int64)

            nx0 = int(min(cxs.min(), ecx.min(), ox))
            nx1 = int(max(cxs.max(), ecx.max(), ox))
            ny0 = int(min(cys.min(), ecy.min(), oy))
            ny1 = int(max(cys.max(), ecy.max(), oy))
            self._grow(nx0, nx1, ny0, ny1)

            # 空闲投票（去重，避免同一格被同帧投很多次）
            gx = cxs - self.x0
            gy = cys - self.y0
            flat = np.unique(gy * self.log.shape[1] + gx)
            self.log.reshape(-1)[flat] -= 0.3

            # 障碍投票
            ogx = ecx - self.x0
            ogy = ecy - self.y0
            oflat = np.unique(ogy * self.log.shape[1] + ogx)
            self.log.reshape(-1)[oflat] += 0.9

        np.clip(self.log, -2.0, 3.5, out=self.log)
        return {"floor": floor, "lo": lo, "hi": hi,
                "size_x": self.log.shape[1] * self.res,
                "size_y": self.log.shape[0] * self.res,
                "origin": (self.x0 * self.res, self.y0 * self.res)}

    def to_image(self) -> np.ndarray:
        """0=障碍(黑) 254=空闲(白) 205=未知(灰)，ROS map_server 约定。"""
        log = self.log
        if self.inflate_cells > 0:
            occ = (log > 0.0).astype(np.uint8)
            k = self.inflate_cells
            pad = np.pad(occ, k)
            grown = pad.copy()
            for dy in range(-k, k + 1):
                for dx in range(-k, k + 1):
                    if dx * dx + dy * dy > k * k:
                        continue
                    grown |= np.roll(np.roll(pad, dy, axis=0), dx, axis=1)
            occ_final = grown[k:-k, k:-k].astype(bool)
        else:
            occ_final = log > 0.0

        free = log < 0.0
        img = np.full(log.shape, 205, dtype=np.uint8)
        img[free] = 254
        img[occ_final] = 0
        return np.flipud(img)      # 行 0 = 最大 y，符合地图图像习惯


def demo_store() -> FrameStore:
    """假数据：8x6 房间，中间一个障碍物，走 5 个位置扫描。"""
    rng = np.random.default_rng(1)
    store = FrameStore(600)

    def room_pts():
        xs = np.arange(-4, 4, 0.04)
        ys = np.arange(-3, 3, 0.04)
        walls = np.vstack([
            np.column_stack([xs, np.full(xs.size, -3.0), rng.uniform(0.1, 1.6, xs.size)]),
            np.column_stack([xs, np.full(xs.size, 3.0), rng.uniform(0.1, 1.6, xs.size)]),
            np.column_stack([np.full(ys.size, -4.0), ys, rng.uniform(0.1, 1.6, ys.size)]),
            np.column_stack([np.full(ys.size, 4.0), ys, rng.uniform(0.1, 1.6, ys.size)]),
        ])
        n = 1500
        box = np.column_stack([rng.uniform(0.5, 1.6, n), rng.uniform(-0.8, 0.2, n),
                               rng.uniform(0.1, 1.2, n)])
        return np.vstack([walls, box]).astype(np.float32)

    pts = room_pts()
    for ox, oy in [(-3, -2), (-3, 2), (0, 0), (3, -2), (3, 2)]:
        store.origin = np.array([ox, oy], dtype=float)
        d = np.hypot(pts[:, 0] - ox, pts[:, 1] - oy)
        vis = d < 6.5                      # 简化成"6.5 米内可见"
        store.add(pts[vis])
    return store


def main() -> int:
    p = argparse.ArgumentParser(description="3D 点云 → 2D 占据栅格（PGM+YAML）")
    p.add_argument("--topic", default="/cloud_registered")
    p.add_argument("--odom-topic", default="/aft_mapped_to_init")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--res", type=float, default=0.05)
    p.add_argument("--clearance", type=float, default=0.15)
    p.add_argument("--height", type=float, default=2.0)
    p.add_argument("--floor", type=float, default=None)
    p.add_argument("--inflate", type=float, default=0.15)
    p.add_argument("--out", default=os.path.expanduser("~/mymap"))
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()

    if args.demo:
        store = demo_store()
        info_floor = None
    else:
        store = FrameStore()
        try:
            run_ros(args, store)
        except Exception as exc:  # noqa: BLE001
            print("ROS 采集失败：%s: %s" % (type(exc).__name__, exc), file=sys.stderr)
            return 1
        info_floor = args.floor

    mapper = OccupancyMapper(args.res, args.clearance, args.height,
                             args.floor if args.demo else info_floor, args.inflate)
    for _, pts in store.frames:
        mapper.collect(pts)
    info = mapper.build(store)
    img = mapper.to_image()

    out = os.path.expanduser(args.out)
    pgm = out + ".pgm"
    yml = out + ".yaml"
    png = out + ".png"
    write_pgm(pgm, img)
    write_yaml(yml, os.path.basename(pgm), args.res, info["origin"])
    write_png(png, img)

    occ = int((img == 0).sum())
    free = int((img == 254).sum())
    unk = int((img == 205).sum())
    total = occ + free + unk

    print("-" * 62)
    print("地图文件 : %s / %s / %s" % (pgm, yml, png))
    print("尺寸     : %d x %d 像素，%.2f m x %.2f m"
          % (img.shape[1], img.shape[0], info["size_x"], info["size_y"]))
    print("分辨率   : %.3f m/像素" % args.res)
    print("原点左下 : (%.2f, %.2f) m" % info["origin"])
    print("地面高度 : %.2f m（自动估计）" % info["floor"])
    print("障碍判据 : 离地 %.2f ~ %.2f m" % (args.clearance, args.height))
    print("障碍膨胀 : %.2f m" % args.inflate)
    print("占比     : 障碍 %.1f%%  空闲 %.1f%%  未知 %.1f%%"
          % (occ / total * 100, free / total * 100, unk / total * 100))
    print("帧数     : %d（共 %d 个点）" % (store.n_frames, store.n_points))
    print("-" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())
