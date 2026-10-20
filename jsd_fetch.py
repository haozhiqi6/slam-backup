#!/usr/bin/env python3
"""借 jsDelivr 把 GitHub 仓库整个拉下来（用于 github.com 不可达的环境）。

用法:
    python3 jsd_fetch.py <user/repo> <ref> <目标目录>
    例: python3 jsd_fetch.py Livox-SDK/livox_ros_driver2 1.2.6 ./livox_ros_driver2

原理：先从 data.jsdelivr.com 取完整文件树，再逐个文件从 cdn.jsdelivr.net 下载，
最后还原成目录结构（不保留 .git 历史，源码一个字节不少）。
"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.request

UA = {"User-Agent": "Mozilla/5.0 (jsd-fetch)"}


def get(url: str, retries: int = 4, timeout: int = 30) -> bytes:
    last: Exception | None = None
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return resp.read()
        except Exception as exc:  # noqa: BLE001
            last = exc
            time.sleep(1.5 * (attempt + 1))
    raise RuntimeError("%s 失败: %s" % (url, last))


def collect(nodes, prefix: str, out: list) -> None:
    for node in nodes:
        path = prefix + node["name"]
        if node.get("type") == "directory":
            collect(node.get("files", []), path + "/", out)
        else:
            out.append(path)


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    repo, ref, dest = sys.argv[1], sys.argv[2], sys.argv[3]

    tree_raw = get("https://data.jsdelivr.com/v1/packages/gh/%s@%s" % (repo, ref))
    tree = json.loads(tree_raw)
    files: list[str] = []
    collect(tree.get("files", []), "", files)
    print("文件总数: %d -> %s" % (len(files), dest))

    ok = 0
    bad: list[str] = []
    for index, path in enumerate(files, 1):
        url = "https://cdn.jsdelivr.net/gh/%s@%s/%s" % (repo, ref, path)
        target = os.path.join(dest, path)
        parent = os.path.dirname(target)
        if parent:
            os.makedirs(parent, exist_ok=True)
        try:
            data = get(url)
            with open(target, "wb") as handle:
                handle.write(data)
            if path.endswith(".sh"):
                os.chmod(target, 0o755)
            ok += 1
        except Exception as exc:  # noqa: BLE001
            bad.append("%s (%s)" % (path, exc))
            print("  失败: %s" % path)
        if index % 25 == 0 or index == len(files):
            print("  进度 %d/%d" % (index, len(files)))

    print("完成: 成功 %d, 失败 %d" % (ok, len(bad)))
    for item in bad:
        print("  未取到: %s" % item)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
