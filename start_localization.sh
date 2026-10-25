#!/bin/bash
# ============================================================
# 重启后的"复用地图"入口：加载已有 2D 栅格图 + AMCL 定位
#
# 用法： bash ~/start_localization.sh [地图yaml路径]
# 默认： ~/maps/myroom.yaml
#
# 前置： FAST-LIVO2 和 pc2scan_tf.py 已经在跑（要有 /scan 和 odom->base_link）
# ============================================================
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash

MAP="${1:-$HOME/maps/myroom.yaml}"

if [ ! -f "$MAP" ]; then
    echo "找不到地图文件：$MAP"
    echo "可用地图："
    ls -1 "$HOME/maps"/*.yaml 2>/dev/null | sed 's/^/  /' || echo "  （还没有地图）"
    exit 1
fi

echo "=== 检查 /scan ==="
if ! timeout 5 ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "  没有 /scan —— 先起 FAST-LIVO2 和 pc2scan_tf.py"
    exit 1
fi

echo "=== 加载地图并启动 AMCL ==="
echo "  地图: $MAP"
pkill -f "[a]mcl" 2>/dev/null
pkill -f "[m]ap_server" 2>/dev/null
sleep 2

setsid nohup ros2 launch "$HOME/localize.launch.py" map:="$MAP" \
    > "$HOME/dl/localize.log" 2>&1 < /dev/null &

sleep 10
echo "=== 自检 ==="
echo "--- 地图是否发布 ---"
timeout 5 ros2 topic hz /map 2>&1 | head -2
echo "--- 位姿是否发布（走几步后会更新）---"
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | head -6
echo
echo "日志： ~/dl/localize.log"
echo "如果定位错了，在 RViz/Foxglove 里拖动初始位姿，或重跑本脚本"
