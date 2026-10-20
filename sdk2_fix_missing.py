#!/usr/bin/env python3
"""补齐 Livox-SDK2 里 jsDelivr 漏掉的文件。

jsDelivr 的文件树接口对文件多的仓库会截断，导致部分源码没下到。
这个脚本按 sdk_core/CMakeLists.txt 里声明的源文件清单找出缺失项，
再从 raw.githubusercontent.com 逐个取回来（板子能通这个域名）。
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
RAW = "https://raw.githubusercontent.com/%s/%s/%s" % (REPO, REF, "%s")
UA = {"User-Agent": "Mozilla/5.0 (sdk2-fix)"}

SOURCE_GROUPS = [
    "MAIN_SOURCES", "BASE_SOURCES", "COMM_SOURCES", "UPGRADE_SOURCES",
    "LOGGER_HANDLER_SOURCES", "DATA_HANDLER_SOURCES",
    "COMMAND_HANDLER_SOURCES", "DEBUG_POINT_CLOUD_HANDLER_SOURCES",
]


def declared_sources() -> list[str]:
    path = os.path.join(ROOT, "sdk_core", "CMakeLists.txt")
    text = open(path, encoding="utf-8", errors="replace").read()
    out: list[str] = []
    for name in SOURCE_GROUPS:
        match = re.search(r"set\(" + name + r"\s+(.*?)\n\s*\)", text, re.S)
        if not match:
            continue
        for line in match.group(1).splitlines():
            line = line.split("#")[0].strip().rstrip(")").strip()
            for token in line.split():
                if token.startswith("../"):
                    continue
                rel = "sdk_core/" + token.replace("${PLATFORM}", "unix")
                out.append(rel)
    # 3rdparty 里 FastCRC 是编译必需的
    out.append("3rdparty/FastCRC/FastCRCsw.cpp")
    return out


def fetch(rel: str, retries: int = 4) -> bool:
    target = os.path.join(ROOT, rel)
    os.makedirs(os.path.dirname(target), exist_ok=True)
    for attempt in range(retries):
        try:
            req = urllib.request.Request(RAW % rel, headers=UA)
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = resp.read()
            with open(target, "wb") as handle:
                handle.write(data)
            return True
        except Exception:  # noqa: BLE001
            time.sleep(1.0 * (attempt + 1))
    return False


def main() -> int:
    wanted = declared_sources()
    missing = [p for p in wanted if not os.path.exists(os.path.join(ROOT, p))]
    print("声明源文件 %d 个，缺失 %d 个" % (len(wanted), len(missing)))
    ok = 0
    for rel in missing:
        if fetch(rel):
            ok += 1
            print("  补回: %s" % rel)
        else:
            print("  失败: %s" % rel)
    print("补齐 %d/%d" % (ok, len(missing)))
    still = [p for p in wanted if not os.path.exists(os.path.join(ROOT, p))]
    return 0 if not still else 1


if __name__ == "__main__":
    sys.exit(main())
