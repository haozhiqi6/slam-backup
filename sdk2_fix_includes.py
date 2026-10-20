#!/usr/bin/env python3
"""补齐 Livox-SDK2 里缺失的头文件。

jsDelivr 的文件列表对大仓库会截断，所以有些 .h 没下到。
这个脚本扫描所有源码里的 #include "..."，找出本地不存在的，
再按几个候选位置从 raw.githubusercontent.com 取回来。
"""

from __future__ import annotations

import os
import re
import sys
import time
import urllib.request

ROOT = "/home/orangepi/dl/Livox-SDK2"
REPO = "Livox-SDK/Livox-SDK2"
REF = "master"
# jsDelivr 的 CDN 快（~0.5s/文件），raw 慢（~7s/文件），所以先用 CDN 兜底用 raw
CDN = "https://cdn.jsdelivr.net/gh/%s@%s/%s" % (REPO, REF, "%s")
RAW = "https://raw.githubusercontent.com/%s/%s/%s" % (REPO, REF, "%s")
UA = {"User-Agent": "Mozilla/5.0 (sdk2-fix)"}

INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.M)


def candidate_paths(inc: str, from_file: str) -> list[str]:
    """给一个 include 名，列出它在仓库里可能的路径。"""
    rel_from = os.path.relpath(os.path.dirname(from_file), ROOT)
    cands = []
    if rel_from.startswith(".."):
        rel_from = ""
    cands.append(os.path.normpath(os.path.join(rel_from, inc)))
    for prefix in ("sdk_core", "include", "3rdparty", ""):
        cands.append(os.path.normpath(os.path.join(prefix, inc)))
    seen, out = set(), []
    for c in cands:
        c = c.replace("\\", "/").lstrip("./")
        if c and c not in seen and not c.startswith(".."):
            seen.add(c)
            out.append(c)
    return out


def _download(url: str, timeout: int) -> bytes:
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def fetch(rel: str) -> bool:
    """先试 jsDelivr CDN，失败再试 raw。"""
    target = os.path.join(ROOT, rel)
    parent = os.path.dirname(target)
    if parent:
        os.makedirs(parent, exist_ok=True)
    for url, timeout, tries in ((CDN % rel, 10, 2), (RAW % rel, 15, 2)):
        for attempt in range(tries):
            try:
                data = _download(url, timeout)
                if not data:
                    raise ValueError("空文件")
                with open(target, "wb") as handle:
                    handle.write(data)
                return True
            except Exception:  # noqa: BLE001
                time.sleep(0.5 * (attempt + 1))
    return False


def scan() -> list[tuple[str, str]]:
    """返回 [(缺失的 include 名, 引用它的文件)]"""
    missing: list[tuple[str, str]] = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in ("build", ".git")]
        for fn in filenames:
            if not fn.endswith((".cpp", ".cc", ".h", ".hpp", ".c")):
                continue
            full = os.path.join(dirpath, fn)
            try:
                text = open(full, encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            for inc in INCLUDE_RE.findall(text):
                found = False
                for cand in candidate_paths(inc, full):
                    if os.path.exists(os.path.join(ROOT, cand)):
                        found = True
                        break
                if not found:
                    missing.append((inc, os.path.relpath(full, ROOT)))
    # 去重
    seen, out = set(), []
    for inc, src in missing:
        if inc not in seen:
            seen.add(inc)
            out.append((inc, src))
    return out


def main() -> int:
    missing = scan()
    print("缺失的 include: %d 个" % len(missing))
    if "--list" in sys.argv:
        for inc, src in missing:
            print("  %-55s  <- %s" % (inc, src))
        return 0
    unresolved = []
    for inc, src in missing:
        done = False
        for cand in candidate_paths(inc, os.path.join(ROOT, src)):
            if fetch(cand):
                print("  补回: %s  (被 %s 引用)" % (cand, src))
                done = True
                break
        if not done:
            unresolved.append((inc, src))
            print("  没找到: %s  (被 %s 引用)" % (inc, src))
    print("仍未解决: %d" % len(unresolved))
    return 0 if not unresolved else 1


if __name__ == "__main__":
    sys.exit(main())
