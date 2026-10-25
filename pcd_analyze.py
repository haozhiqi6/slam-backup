#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 PCD（binary/ascii），输出统计 + 俯视灰度图。纯 numpy，无 PCL 依赖。"""

from __future__ import annotations

import os
import struct
import sys
import zlib

import numpy as np


def read_pcd(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("PCD 头不完整")
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, _, val = text.partition(" ")
            header[key.upper()] = val.strip()
            if key.upper() == "DATA":
                break
        body = f.read()

    data = header["DATA"].lower()
    fields = header["FIELDS"].split()
    sizes = [int(x) for x in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(x) for x in header.get("COUNT", " ".join(["1"] * len(fields))).split()]
    npts = int(header["POINTS"])

    if data != "binary":
        arr = np.loadtxt(path, skiprows=11, usecols=(0, 1, 2))
        return arr.astype(np.float32)

    np_types = {"F": "f", "I": "i", "U": "u"}
    names, formats, offsets = [], [], []
    off = 0
    for name, size, typ, cnt in zip(fields, sizes, types, counts):
        for c in range(cnt):
            names.append(name if cnt == 1 else "%s_%d" % (name, c))
            formats.append("<%s%d" % (np_types[typ], size))
            offsets.append(off)
            off += size
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": off})
    pts = np.frombuffer(body[: npts * off], dtype=dt, count=npts)
    return np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32)


def write_png(path: str, gray: np.ndarray) -> None:
    gray = np.ascontiguousarray(gray.astype(np.uint8))
    h, w = gray.shape
    raw = b"".join(b"\x00" + gray[r].tobytes() for r in range(h))

    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def main() -> int:
    path = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(path)[0] + "_top.png"
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05

    p = read_pcd(path)
    print("点云文件:", path)
    print("点数    : %d" % len(p))
    for i, axis in enumerate("xyz"):
        v = p[:, i]
        print("  %s: %8.2f ~ %8.2f m   (中位 %6.2f)"
              % (axis, v.min(), v.max(), np.median(v)))

    floor = float(np.percentile(p[:, 2], 5))
    print("地面估计: %.2f m" % floor)

    obj = p[p[:, 2] > floor + 0.15]
    print("离地 0.15 m 以上的点: %d (%.1f%%)" % (len(obj), len(obj) / len(p) * 100))
    if len(obj) == 0:
        obj = p

    ix = np.floor(obj[:, 0] / res).astype(np.int64)
    iy = np.floor(obj[:, 1] / res).astype(np.int64)
    z = obj[:, 2]
    x0, y0 = ix.min(), iy.min()
    w, h = int(ix.max() - x0 + 1), int(iy.max() - y0 + 1)

    img = np.zeros((h, w), np.uint8)
    gx, gy = ix - x0, iy - y0
    zt = z
    lo, hi = floor + 0.15, float(np.percentile(zt, 98))
    if hi - lo < 1e-6:
        hi = lo + 1.0
    val = (50 + np.clip((zt - lo) / (hi - lo), 0, 1) * 205).astype(np.uint8)
    np.maximum.at(img, (gy, gx), val)
    img = np.flipud(img)

    write_png(out, img)
    print("俯视图  : %s" % out)
    print("尺寸    : %d x %d 像素，%.1f m x %.1f m" % (w, h, w * res, h * res))
    print("范围    : x %.2f~%.2f  y %.2f~%.2f"
          % (x0 * res, (x0 + w) * res, y0 * res, (y0 + h) * res))
    return 0


if __name__ == "__main__":
    sys.exit(main())
