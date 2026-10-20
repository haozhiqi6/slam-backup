#!/bin/bash
# ============================================
# 一条命令给当前代码存档
#
# 用法:  ./git_save.sh "这次改了什么"
# 例子:  ./git_save.sh "把 guard 阈值放宽到 5.0"
#
# 每次存档会生成：
#   1) 一个 commit（代码快照）
#   2) 一个 save-年月日-时分秒 的标签（方便回退）
# ============================================
set -e

cd /home/orangepi

MSG="${1:-自动存档 $(date '+%Y-%m-%d %H:%M')}"

git add -A

if git diff --cached --quiet; then
    echo "没有检测到改动，不需要存档。"
    exit 0
fi

git commit -q -m "$MSG"
TAG="save-$(date '+%Y%m%d-%H%M%S')"
git tag "$TAG"

echo "已存档：$TAG"
echo "说明：$MSG"
echo
echo "最近的存档点："
git log --oneline --decorate -6
