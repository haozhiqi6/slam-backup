#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 3D 点云压成 2D 灰度图（俯视图 / 平面图）

用法（香橙派上，SLAM 正在跑的时候）：
    python3 cloud_to_gray.py --duration 20

    默认订阅 /cloud_registered，采集 20 秒，输出 ~/map_gray.png

常用参数：
    --topic      订阅话题            默认 /cloud_registered
    --duration   采集多少秒          默认 20
    --res        格子大小（米）      默认 0.05
    --clearance  高于地面多少米以上才要   默认 0.15
    --floor      地面高度（米），默认自动估计
    --zmax       忽略高于这个的点（噪声） 默认 5.0
    --mode       height 按高度着色（默认）
                 occupancy 占用为黑、空白为白（像户型图）
    --out        输出文件            默认 ~/map_gray.png
    --demo       不连 ROS，生成一张假数据测试

输出：
    - PNG 灰度图（白 = 高/有东西，黑 = 空/低）
    - 终端打印一张字符缩略图，不用看图就能大致判断
    - 打印地图实际尺寸（米）

原理：
    1. 每来一帧点云，把每个点按 XY 落到 0.05 m 的格子里
    2. 每个格子记下"落进来的点里最高的那个 z"
    3. **自动估计地面高度**（点最多的那个高度就是地面），把地面附近整片格子丢掉
    4. 剩下的格子（墙、桌子、障碍物）按高度映射到 0~255 灰度
    5. 翻转 Y 轴（让 +Y 朝上，符合看地图的习惯）

为什么要剔地面：
    不剔的话整张图被地面铺满，墙和家具全糊在一起，什么都看不出来。
    地面又是点最多的平面，所以用"z 的最低 5% 分位"估计它足够稳。
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
# PNG 输出（不依赖 Pillow，任何环境都能跑）
# ----------------------------------------------------------------------
def write_png(path: str, gray: np.ndarray) -> None:
    """写 8 位灰度 PNG。gray: uint8 二维数组。"""
    gray = np.ascontiguousarray(gray.astype(np.uint8))
    h, w = gray.shape
    raw = b"".join(b"\x00" + gray[row].tobytes() for row in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")

    with open(path, "wb") as f:
        f.write(png)


# ----------------------------------------------------------------------
# 点云累积
# ----------------------------------------------------------------------
class GridAccumulator:
    """把点累加进 XY 栅格，每格记录最高点和点数。

    累积阶段不做高度过滤，过滤放到 render()，这样才有足够信息估计地面。
    """

    def __init__(self, res: float) -> None:
        self.res = float(res)
        self.cells: dict[tuple[int, int], list[float]] = {}
        self.n_points = 0
        self.n_used = 0

    def add(self, xyz: np.ndarray) -> None:
        if xyz.size == 0:
            return
        self.n_points += len(xyz)

        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        if xyz.size == 0:
            return

        z = xyz[:, 2]
        # 只挡掉明显异常的读数，正常高度留给 render 判断
        keep = np.abs(z) < 1000.0
        xyz = xyz[keep]
        if xyz.size == 0:
            return
        self.n_used += len(xyz)

        ix = np.floor(xyz[:, 0] / self.res).astype(np.int64)
        iy = np.floor(xyz[:, 1] / self.res).astype(np.int64)
        zz = xyz[:, 2]

        # 按格子聚合：取每格最高点 + 计数
        order = np.lexsort((zz, iy, ix))
        ix, iy, zz = ix[order], iy[order], zz[order]

        uniq, start = np.unique(
            np.stack([ix, iy], axis=1), axis=0, return_index=True
        )
        # 由于 z 已排序，每段最后一个就是最高点
        ends = np.append(start[1:], len(zz))
        counts = ends - start
        zmax = zz[ends - 1]

        for (cx, cy), zt, cnt in zip(map(tuple, uniq), zmax, counts):
            old = self.cells.get((cx, cy))
            if old is None:
                self.cells[(cx, cy)] = [float(zt), int(cnt)]
            else:
                if zt > old[0]:
                    old[0] = float(zt)
                old[1] += int(cnt)

    def render(
        self,
        mode: str = "height",
        clearance: float = 0.15,
        floor: float | None = None,
        zmax: float = 5.0,
    ) -> tuple[np.ndarray, dict]:
        if not self.cells:
            raise RuntimeError("一个点都没收到，检查话题和 --zmin/--zmax")

        ix = np.fromiter((k[0] for k in self.cells), dtype=np.int64)
        iy = np.fromiter((k[1] for k in self.cells), dtype=np.int64)
        zt = np.fromiter((v[0] for v in self.cells.values()), dtype=np.float64)
        cnt = np.fromiter((v[1] for v in self.cells.values()), dtype=np.int64)

        # ---- 估计地面：取所有格子最高点 z 的低分位 ----
        floor_auto = float(np.percentile(zt, 5)) if floor is None else float(floor)
        cutoff = floor_auto + clearance

        # ---- 只保留高于地面 clearance 的格子，并挡掉过高噪声 ----
        keep = (zt >= cutoff) & (zt <= floor_auto + zmax)
        n_kept = int(keep.sum())
        if n_kept == 0:
            raise RuntimeError(
                "剔掉地面后一个格子都不剩。地面估在 %.2f m，"
                "可以试 --clearance 0.05 或手动指定 --floor" % floor_auto
            )
        ix, iy, zt, cnt = ix[keep], iy[keep], zt[keep], cnt[keep]

        x0, x1 = ix.min(), ix.max()
        y0, y1 = iy.min(), iy.max()
        w = int(x1 - x0 + 1)
        h = int(y1 - y0 + 1)

        gx = (ix - x0).astype(np.int64)
        gy = (iy - y0).astype(np.int64)

        if mode == "occupancy":
            # 有 >= 3 个点落下就算占用（黑），否则白
            img = np.full((h, w), 255, dtype=np.uint8)
            occ = cnt >= 3
            img[gy[occ], gx[occ]] = 0
        else:
            # 高度着色：高于地面越多越白
            img = np.zeros((h, w), dtype=np.uint8)
            lo, hi = cutoff, float(np.percentile(zt, 98))
            if hi - lo < 1e-6:
                hi = lo + 1.0
            norm = np.clip((zt - cutoff) / (hi - cutoff), 0, 1)
            img[gy, gx] = (50 + norm * 205).astype(np.uint8)

        # 翻转 Y：让 +Y 朝上
        img = np.flipud(img)

        info = {
            "width": w,
            "height": h,
            "size_x": w * self.res,
            "size_y": h * self.res,
            "res": self.res,
            "cells": len(self.cells),
            "kept": n_kept,
            "points": self.n_points,
            "used": self.n_used,
            "origin": (x0 * self.res, y0 * self.res),
            "floor": floor_auto,
            "cutoff": cutoff,
        }
        return img, info


def ascii_preview(img: np.ndarray, cols: int = 78, invert: bool = False) -> str:
    """把图缩成字符画，终端里直接看。

    invert=True 用于 occupancy 模式的转置：那里黑=占用，字符画要黑=密集。
    """
    h, w = img.shape
    rows = max(1, int(cols * h / max(w, 1) * 0.5))
    ys = np.linspace(0, h, rows + 1).astype(int)
    xs = np.linspace(0, w, cols + 1).astype(int)
    ramp = " .:-=+*#%@"
    out = []
    for r in range(rows):
        line = []
        for c in range(cols):
            block = img[ys[r]:ys[r + 1], xs[c]:xs[c + 1]]
            # 普通模式取块内最亮（有东西的地方）；占用模式取最暗（墙=黑）
            v = float(block.min() if invert else block.max()) if block.size else 0.0
            if invert:
                v = 255.0 - v
            line.append(ramp[min(len(ramp) - 1, int(v / 256 * len(ramp)))])
        out.append("".join(line))
    return "\n".join(out)


# ----------------------------------------------------------------------
# ROS 2 读取
# ----------------------------------------------------------------------
def xyz_from_pointcloud2(msg) -> np.ndarray:
    """把 sensor_msgs/PointCloud2 转成 N×3 的 numpy 数组（尽量用 numpy 直接解析）。"""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)

    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
            "itemsize": msg.point_step,
        }
    )
    data = np.frombuffer(bytes(msg.data), dtype=dtype)
    if msg.is_bigendian:
        data = data.byteswap()
    if msg.height > 1 and msg.width * msg.height < len(data):
        data = data[: msg.width * msg.height]
    return np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32)


def run_ros(args, acc: GridAccumulator) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node("cloud_to_gray")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    stat = {"frames": 0}

    def cb(msg):
        acc.add(xyz_from_pointcloud2(msg))
        stat["frames"] += 1
        if stat["frames"] % 20 == 0:
            node.get_logger().info(
                "已收 %d 帧，%d 个格子" % (stat["frames"], len(acc.cells))
            )

    node.create_subscription(PointCloud2, args.topic, cb, qos)
    node.get_logger().info(
        "开始采集 %.0f 秒：%s（格子 %.2f m）" % (args.duration, args.topic, args.res)
    )

    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("采集结束，共 %d 帧" % stat["frames"])
    node.destroy_node()
    rclpy.shutdown()


# ----------------------------------------------------------------------
# 假数据（用于自测，不连 ROS）
# ----------------------------------------------------------------------
def demo_points() -> np.ndarray:
    rng = np.random.default_rng(0)
    parts = []

    # 房间四壁（8m x 6m）
    xs = np.arange(-4, 4, 0.05)
    parts.append(np.column_stack([
        xs, np.full(xs.size, -3.0), 0.2 + 0.02 * rng.standard_normal(xs.size)]))
    parts.append(np.column_stack([
        xs, np.full(xs.size, 3.0), 0.2 + 0.02 * rng.standard_normal(xs.size)]))
    ys = np.arange(-3, 3, 0.05)
    parts.append(np.column_stack([
        np.full(ys.size, -4.0), ys, 0.2 + 0.02 * rng.standard_normal(ys.size)]))
    parts.append(np.column_stack([
        np.full(ys.size, 4.0), ys, 0.2 + 0.02 * rng.standard_normal(ys.size)]))

    # 中间一个障碍物（比地面高）
    n = 4000
    parts.append(np.column_stack([
        rng.uniform(0.5, 1.6, n),
        rng.uniform(-0.8, 0.2, n),
        rng.uniform(0.1, 1.2, n),
    ]))

    # 地面
    n = 20000
    parts.append(np.column_stack([
        rng.uniform(-4, 4, n),
        rng.uniform(-3, 3, n),
        rng.uniform(-0.05, 0.05, n),
    ]))

    return np.vstack(parts).astype(np.float32)


def main() -> int:
    p = argparse.ArgumentParser(description="3D 点云 → 2D 灰度图")
    p.add_argument("--topic", default="/cloud_registered")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--res", type=float, default=0.05)
    p.add_argument("--clearance", type=float, default=0.15,
                   help="高于地面多少米以上才画进图（默认 0.15）")
    p.add_argument("--floor", type=float, default=None,
                   help="手动指定地面高度（米）；不填则自动估计")
    p.add_argument("--zmax", type=float, default=5.0,
                   help="高于地面多少米以上的点当噪声丢掉（默认 5.0）")
    p.add_argument("--mode", choices=("height", "occupancy"), default="height")
    p.add_argument("--out", default=os.path.expanduser("~/map_gray.png"))
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()

    acc = GridAccumulator(args.res)

    if args.demo:
        acc.add(demo_points())
    else:
        try:
            run_ros(args, acc)
        except Exception as exc:  # noqa: BLE001
            print("ROS 采集失败：%s: %s" % (type(exc).__name__, exc), file=sys.stderr)
            print("提示：先启动 SLAM，确认 ros2 topic hz %s 有数据" % args.topic, file=sys.stderr)
            return 1

    img, info = acc.render(
        args.mode, clearance=args.clearance, floor=args.floor, zmax=args.zmax
    )
    write_png(os.path.expanduser(args.out), img)

    print("-" * 60)
    print("输出文件 : %s" % os.path.expanduser(args.out))
    print("图片尺寸 : %d x %d 像素" % (info["width"], info["height"]))
    print("实际范围 : %.2f m x %.2f m （分辨率 %.2f m/像素）"
          % (info["size_x"], info["size_y"], info["res"]))
    print("原点左下 : (%.2f, %.2f) m" % info["origin"])
    print("有效格子 : %d" % info["cells"])
    print("入图格子 : %d（剔掉了 %d 个地面/噪声格子）"
          % (info["kept"], info["cells"] - info["kept"]))
    print("地面高度 : %.2f m（自动估计）" % info["floor"])
    print("点数     : 收到 %d，采用 %d" % (info["points"], info["used"]))
    print("-" * 60)
    print(ascii_preview(img, invert=(args.mode == "occupancy")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
