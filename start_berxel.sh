#!/bin/bash
# Berxel iHawk100Q 一键启动 (Orange Pi 5 Pro)
# 用法: ./start_berxel.sh [样例名]   默认 HawkDepth
# 说明: Pi 的 mesa/libmali GL 对 GLX 应用是坏的(rockchip_dri 不支持 G610),
#       故强制软件渲染; 若日后修好硬件 GL, 把下面这行注释掉即可
export LIBGL_ALWAYS_SOFTWARE=1

SDK=~/berxel_sdk/BerxelSDK-Linux-2.0.140_arm64_7.5.0
SAMPLE=${1:-HawkDepth}

export BERXEL_SDK_LIBRARY="$SDK"
export LD_LIBRARY_PATH="$SDK/libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [ ! -x "$SDK/samples/bin/$SAMPLE" ]; then
    echo "==> 未找到 $SAMPLE, 正在编译..."
    ( cd "$SDK/samples/src" && make ) || exit 1
fi
cd "$SDK/samples/bin" || exit 1
echo "==> 启动 $SAMPLE  (Ctrl+C 退出)"
exec ./"$SAMPLE"
