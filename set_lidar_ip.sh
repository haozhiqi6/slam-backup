#!/bin/bash
# 把雷达网口的静态 IP 写进 NetworkManager 连接配置（一劳永逸，重启也不会丢）
# 之前用 ip addr add 手动加，会被 NetworkManager 清掉

DEV=enP4p65s0
IP=192.168.10.50/24

NAME=$(nmcli -t -f NAME,DEVICE con show 2>/dev/null | grep "$DEV" | head -1 | cut -d: -f1)
echo "网口 $DEV 对应的连接名: ${NAME:-(无)}"

if [ -n "$NAME" ]; then
  echo orangepi | sudo -S -p "" nmcli con mod "$NAME" \
    ipv4.method manual \
    ipv4.addresses "$IP" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.never-default yes
  echo orangepi | sudo -S -p "" nmcli con down "$NAME"
  sleep 1
  echo orangepi | sudo -S -p "" nmcli con up "$NAME"
else
  echo "没找到 NetworkManager 连接，退回手动加 IP"
  echo orangepi | sudo -S -p "" ip addr add "$IP" dev "$DEV"
fi

sleep 3
echo "--- 现状 ---"
ip -br a show "$DEV"
echo "--- ping 雷达 ---"
ping -c 3 -W 2 192.168.10.18 2>&1 | tail -3
