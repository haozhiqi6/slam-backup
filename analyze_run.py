#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按时间顺序看一次运行的质量变化，找出从哪一刻开始变坏。"""
import re
import sys

path = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 300

re_eff = re.compile(r"effective feature num:\s*(\d+).*?average residual:\s*([-\d.eE+nan]+)")
re_stamp = re.compile(r"\[(\d{10})\.\d+\]")
re_cur = re.compile(r"Current Total Time\s*\|\s*([\d.]+)")
re_sync = re.compile(r"not synced! delta time:\s*([-\d.eE+]+)")

rows = []
syncs = []
t0 = None
tlast = None
with open(path, "r", errors="replace") as f:
    for line in f:
        m = re_eff.search(line)
        if m:
            sm = re_stamp.search(line)
            ts = int(sm.group(1)) if sm else None
            if ts and t0 is None:
                t0 = ts
            if ts:
                tlast = ts
            try:
                res = float(m.group(2))
            except ValueError:
                res = float("nan")
            rows.append((ts, int(m.group(1)), res))
            continue
        m = re_sync.search(line)
        if m:
            try:
                syncs.append(float(m.group(1)))
            except ValueError:
                pass

if not rows:
    print("没有 LIO 记录")
    raise SystemExit(0)

print("文件: %s" % path)
print("总 LIO 帧: %d   跨度: %s 秒" % (len(rows), (tlast - t0) if (t0 and tlast) else "?"))
print()
print("  时间(s)   帧号    有效点   残差")
bad_from = None
for i in range(0, len(rows), step):
    ts, eff, res = rows[i]
    rel = (ts - t0) if (ts and t0) else i
    flag = ""
    if eff == 0:
        flag = "  <== 0 点"
        if bad_from is None:
            bad_from = rel
    elif eff < 500:
        flag = "  <== 偏少"
        if bad_from is None:
            bad_from = rel
    print("  %7.0f  %6d  %7d  %7.4f%s" % (rel, i, eff, res, flag))

print()
if syncs:
    print("时间不同步告警: %d 条，第一条 delta = %.1f 秒" % (len(syncs), syncs[0]))
else:
    print("时间同步: 没有告警（正常）")

zeros = sum(1 for _, e, _ in rows if e == 0)
low = sum(1 for _, e, _ in rows if 0 < e < 500)
print("0 点的帧: %d (%.1f%%)   低于 500 的帧: %d (%.1f%%)"
      % (zeros, zeros / len(rows) * 100, low, low / len(rows) * 100))
if bad_from is not None:
    print("第一次变差: %d 秒（%.1f%% 处）" % (bad_from, bad_from / max(1, (tlast - t0)) * 100))
