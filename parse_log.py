#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解析 FAST-LIVO2 运行日志，统计有效特征点数和单帧耗时。"""
import re
import statistics
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "/home/orangepi/dl/fastlivo_run.log"

eff, res, cur, avg = [], [], [], []
re_eff = re.compile(r"effective feature num:\s*(\d+).*?average residual:\s*([-\d.eE+nan]+)")
re_cur = re.compile(r"Current Total Time\s*\|\s*([\d.]+)")
re_avg = re.compile(r"Average Total Time\s*\|\s*([\d.]+)")

lines = 0
with open(path, "r", errors="replace") as f:
    for line in f:
        lines += 1
        m = re_eff.search(line)
        if m:
            eff.append(int(m.group(1)))
            try:
                res.append(float(m.group(2)))
            except ValueError:
                pass
        m = re_cur.search(line)
        if m:
            cur.append(float(m.group(1)))
        m = re_avg.search(line)
        if m:
            avg.append(float(m.group(1)))


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * q))]


print("日志行数:", lines)
print()
if eff:
    n0 = sum(1 for x in eff if x == 0)
    good = [x for x in eff if x > 0]
    print("有效特征点（共 %d 帧）" % len(eff))
    print("  0 点的帧: %d  (%.1f%%)" % (n0, n0 / len(eff) * 100))
    if good:
        print("  非零帧: 中位 %d  平均 %.0f  P10 %d  P90 %d  最小 %d  最大 %d"
              % (statistics.median(good), statistics.mean(good),
                 pct(good, 0.1), pct(good, 0.9), min(good), max(good)))
        for lo, hi in ((0, 500), (500, 1000), (1000, 1500),
                       (1500, 2100), (2100, 3000), (3000, 10 ** 9)):
            c = sum(1 for x in good if lo <= x < hi)
            if c:
                print("    %5d~%-5d : %6d 帧 (%.1f%%)" % (lo, hi if hi < 10 ** 9 else 99999,
                                                        c, c / len(good) * 100))
if res:
    ok = [x for x in res if x == x and x < 1.0]
    if ok:
        print("\n平均残差: 中位 %.4f  P90 %.4f  最大 %.4f" %
              (statistics.median(ok), pct(ok, 0.9), max(ok)))
if cur:
    print("\n单帧耗时 Current Total Time（%d 次）" % len(cur))
    print("  中位 %.3f s  P90 %.3f s  最大 %.3f s" %
          (statistics.median(cur), pct(cur, 0.9), max(cur)))
    over = sum(1 for x in cur if x > 0.1)
    print("  超过 100 ms 的: %d 次 (%.1f%%)" % (over, over / len(cur) * 100))
