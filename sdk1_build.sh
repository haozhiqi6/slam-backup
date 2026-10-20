#!/bin/bash
# 在香橙派上编译安装 Livox-SDK (v1)
# 日志: /home/orangepi/dl/sdk1_build.log

SRC=/home/orangepi/dl/Livox-SDK
LOG=/home/orangepi/dl/sdk1_build.log

echo "=== 开始 $(date) ===" > "$LOG"

mkdir -p "$SRC/build"
cd "$SRC/build" || { echo "NO_SRC" >> "$LOG"; exit 1; }
rm -rf "$SRC/build"/*

echo "--- cmake ---" >> "$LOG"
# -DCMAKE_POSITION_INDEPENDENT_CODE=ON 是必须的：
# ROS2 驱动生成的是共享库，链接这个静态库时要求所有目标文件都带 -fPIC，
# 否则 ld 会报 "relocation R_AARCH64_ADR_PREL_PG_HI21 ... recompile with -fPIC"
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON >> "$LOG" 2>&1 \
    || { echo "CMAKE_FAIL" >> "$LOG"; exit 1; }

echo "--- make -j4 ---" >> "$LOG"
make -j4 >> "$LOG" 2>&1 || { echo "MAKE_FAIL" >> "$LOG"; exit 1; }

echo "--- make install ---" >> "$LOG"
echo orangepi | sudo -S -p "" make install >> "$LOG" 2>&1 || { echo "INSTALL_FAIL" >> "$LOG"; exit 1; }

echo "BUILD_OK $(date)" >> "$LOG"
