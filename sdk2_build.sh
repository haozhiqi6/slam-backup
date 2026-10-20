#!/bin/bash
# 在香橙派上编译安装 Livox-SDK2
# 用 nohup 跑，日志写 /home/orangepi/dl/sdk2_build.log

S=/home/orangepi/dl/Livox-SDK2
LOG=/home/orangepi/dl/sdk2_build.log

echo "=== 开始 $(date) ===" > "$LOG"

# 上游 master 的 samples/CMakeLists.txt 引用了两个不存在的目录，删掉那两行
sed -i '/add_subdirectory(livox_lidar_ip_set)/d; /add_subdirectory(livox_lidar_info_get)/d' \
    "$S/samples/CMakeLists.txt"
echo "--- samples/CMakeLists.txt 剩余 add_subdirectory ---" >> "$LOG"
grep -n add_subdirectory "$S/samples/CMakeLists.txt" >> "$LOG" 2>&1

cd "$S/build" || { echo "NO_BUILD_DIR" >> "$LOG"; exit 1; }
rm -rf "$S/build"/*

echo "--- cmake ---" >> "$LOG"
cmake .. -DCMAKE_BUILD_TYPE=Release >> "$LOG" 2>&1 || { echo "CMAKE_FAIL" >> "$LOG"; exit 1; }

echo "--- make -j4 ---" >> "$LOG"
make -j4 >> "$LOG" 2>&1 || { echo "MAKE_FAIL" >> "$LOG"; exit 1; }

echo "--- make install ---" >> "$LOG"
echo orangepi | sudo -S -p "" make install >> "$LOG" 2>&1 || { echo "INSTALL_FAIL" >> "$LOG"; exit 1; }

echo "BUILD_OK $(date)" >> "$LOG"
