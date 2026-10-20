#!/bin/bash
LOG=/home/orangepi/dl/memwatch.log
: > "$LOG"
for i in $(seq 1 90); do
  pid=$(pgrep -f "fastlivo_mapping" | head -1)
  rss="--"
  if [ -n "$pid" ]; then
    rss=$(awk '/VmRSS/{print $2}' /proc/$pid/status)
  fi
  used=$(free -m | awk 'NR==2{print $3}')
  avail=$(free -m | awk 'NR==2{print $7}')
  rv=$(ps -eo comm=,rss= | awk '$1 ~ /rviz2/ {s+=$2} END{print s+0}')
  fc=$(pgrep -f fastlivo_mapping | wc -l)
  echo "$(date +%H:%M:%S) fastlivo_rss=${rss}kB rviz=${rv}kB proc=${fc} used=${used}MB avail=${avail}MB" >> "$LOG"
  sleep 20
done