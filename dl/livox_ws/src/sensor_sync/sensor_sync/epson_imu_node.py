#!/usr/bin/env python3
"""STM32 转发的 Epson M-G354PD IMU 帧 -> ROS 2 sensor_msgs/Imu（带硬件时间戳对齐）。

这是 Windows 上 ROS1 版 epson_imu_node.py 的 ROS 2 移植，对齐逻辑一字没改：

  * STM32 在 200Hz 触发脉冲的上升沿打微秒时间戳 t_us，随 39 字节帧一起发过来
    —— 这才是真实采样时刻，而不是"主机收到包的时刻"
  * 主机侧持续估计 offset = 主机时间 - t_us*1e-6（取最近 2 秒中位数，抗抖动）
  * 发布时 stamp = offset + t_us*1e-6
  * 每 10 秒回推 UTC 给 STM32（T<秒>,<毫秒>），把它给雷达的 1Hz PPS 相位
    掰到真正的整秒边界上

帧格式（39 字节，小端）：
    AA 55 | seq(2) | t_us(4) | gyro(12) | accl(12) | temp(4) | count(2) | XOR(1)
"""

from __future__ import annotations

import glob
import statistics
import struct
import threading
import time
from collections import deque

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

HEAD = b"\xAA\x55"
FRAME_LEN = 39

# STM32 发来的 t_us 是 32 位微秒计数，有两个坑：
#   1) 满 2^32 µs 绕一圈，约 71.58 分钟；
#   2) STM32 一旦复位（重新下载、按键复位、供电闪断）就从 0 重新数。
# 两者都会让原始值"掉下去"，但纠正量完全不同：
#   绕圈要加一整圈（2^32 µs = 4295 秒）；
#   复位则要接着上一帧的虚拟值往下走，一点都不能加，否则时间戳会凭空
#   跳几百秒，下游（FAST-LIVO2）直接把 IMU 判成坏数据，姿态发散。
# 所以这里把 32 位计数展开成一条连续的虚拟计数，两种情况都处理掉。
US_WRAP = 1 << 32

# 判断"是不是真绕圈"的边界：前值必须已经在顶部 120 秒内、新值又在底部 120 秒内。
# 复位时前值通常停在中间某处，落不进这个窗口，于是走复位分支。
WRAP_EDGE_US = 120 * 1000 * 1000

# 给 STM32 发授时命令时，每个字节之间留多少空隙。
#   0    = 一次发完（需要固件用"接收中断"收，见 stm32_imu_test/main.c）
#   >0   = 一个字节一个字节慢发（老固件是主循环轮询收的，发快了会丢字节，
#          但发慢了命令里的"毫秒"会过时，相位锁不紧）
# 默认 0，配合新固件使用；老固件请用 -p tx_gap:=0.012
DEFAULT_TX_GAP_S = 0.0

# 读串口一次最多读这么多字节。
# 为什么要小：一次读 1024 字节要 22ms 才凑够，这一批 20 多帧会拿到几乎相同的
# "主机时间"，而它们的 t_us 相差上百毫秒，单帧误差被放大成几十毫秒的抖动。
# 小批量快读让"收到时刻"贴近真实到达时刻。
READ_CHUNK = 64

# Epson M-G354PDH0 实测系数（32 位输出要再除以 65536）
# 注意：这颗是 0.016，不是 G366 的 0.00625
GYRO_SF = 0.016 / 65536.0            # (deg/s) / LSB
ACCL_SF = 0.200 / 65536.0            # mG / LSB
G_TO_MS2 = 9.80665
DEG_TO_RAD = 3.141592653589793 / 180.0


class FrameReader(threading.Thread):
    """后台线程：从串口抠帧、算物理量、塞进队列；顺带把授时命令写回 STM32。

    读写都在这一个线程里，避免两个线程同时碰串口。
    """

    def __init__(self, port: str, baud: int, out: deque):
        super().__init__(daemon=True)
        # 这两个必须存下来：reconnect() 里要用。
        # 之前漏了 self.port，结果串口一掉线、去重连时抛 AttributeError，
        # 读数据的线程当场死掉，IMU 数据流永久断开（上游 FAST-LIVO2 随之发散）。
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.002)
        self.out = out
        self.bad = 0
        self.total = 0
        self.running = True
        self.tx_queue: deque = deque(maxlen=64)
        self.lock = threading.Lock()
        # 32 位微秒计数展开用：上一次的原始值 + 虚拟计数相对原始值的固定偏移
        self.t_us_prev = None
        self.t_us_base = 0

    def run(self) -> None:
        buf = bytearray()
        while self.running:
            with self.lock:
                try:
                    chunk = self.ser.read(READ_CHUNK)
                except Exception:
                    chunk = None
            if chunk is None:
                # 串口掉了（CH340 被 USB 重新枚举，名字可能从 ttyUSB0 变成 ttyUSB1）
                print("[imu] 串口断了，尝试重连 …", flush=True)
                if not self.reconnect():
                    time.sleep(1.0)
                buf.clear()
                continue
            if not chunk:
                time.sleep(0.001)
                continue
            buf += chunk

            while True:
                i = buf.find(HEAD)
                if i < 0 or len(buf) - i < FRAME_LEN:
                    if i > 0:
                        del buf[:i]
                    elif len(buf) > 4096:
                        del buf[:-FRAME_LEN]
                    break
                frame = bytes(buf[i : i + FRAME_LEN])
                del buf[: i + FRAME_LEN]

                chk = 0
                for byte in frame[2:38]:
                    chk ^= byte
                if chk != frame[38]:
                    self.bad += 1
                    continue

                t_us_raw = struct.unpack_from("<I", frame, 4)[0]

                # 原始计数突然"掉下去"半个量程时，区分绕圈和复位
                if self.t_us_prev is not None and t_us_raw < self.t_us_prev - US_WRAP // 2:
                    near_top = self.t_us_prev > US_WRAP - WRAP_EDGE_US
                    near_zero = t_us_raw < WRAP_EDGE_US
                    if near_top and near_zero:
                        # 真·绕圈：加一整圈，虚拟计数继续往前走
                        self.t_us_base += US_WRAP
                        print(
                            "[imu] t_us 绕圈（32 位计数器满一圈），已展开，时间戳保持连续",
                            flush=True,
                        )
                    else:
                        # STM32 复位：让虚拟计数从上一帧接着走，时间戳不跳
                        self.t_us_base = self.t_us_prev + self.t_us_base - t_us_raw
                        print(
                            "[imu] STM32 计数器复位（重新下载/按键复位/供电闪断），"
                            "时间戳已平滑接续",
                            flush=True,
                        )
                self.t_us_prev = t_us_raw
                t_us = t_us_raw + self.t_us_base

                gyro = struct.unpack_from("<iii", frame, 8)
                accl = struct.unpack_from("<iii", frame, 20)
                (temp_raw,) = struct.unpack_from("<i", frame, 32)

                self.out.append(
                    {
                        "t_us": t_us,
                        "host": time.time(),
                        "gyro": tuple(g * GYRO_SF * DEG_TO_RAD for g in gyro),   # rad/s
                        "accl": tuple(a * ACCL_SF * 1e-3 * G_TO_MS2 for a in accl),  # m/s^2
                        "temp": 25.0 + temp_raw * (-0.0037918 / 65536.0),
                    }
                )
                self.total += 1

    def stop(self) -> None:
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass

    def reconnect(self) -> bool:
        """串口掉了就关掉重开；顺便在 /dev/ttyUSB* 里找一个能用的。"""
        try:
            self.ser.close()
        except Exception:
            pass
        time.sleep(1.0)

        candidates = [self.port]
        candidates += sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        seen = set()
        for cand in candidates:
            if cand in seen:
                continue
            seen.add(cand)
            try:
                self.ser = serial.Serial(cand, self.baud, timeout=0.002)
                self.port = cand
                print("[imu] 已重新打开 %s" % cand, flush=True)
                return True
            except Exception:
                continue
        print("[imu] 没有可用的串口，1 秒后再试", flush=True)
        return False


class TimePusher(threading.Thread):
    """单独一个线程负责给 STM32 慢慢发授时命令，免得阻塞读串口。"""

    def __init__(self, reader: FrameReader, gap: float = DEFAULT_TX_GAP_S):
        super().__init__(daemon=True)
        self.reader = reader
        self.gap = gap
        self.running = True

    def run(self) -> None:
        while self.running and self.reader.running:
            if not self.reader.tx_queue:
                time.sleep(0.02)
                continue
            if self.gap <= 0.0:
                # 一次发完：配合固件的接收中断
                with self.reader.lock:
                    try:
                        self.reader.ser.write(bytes(self.reader.tx_queue))
                    except Exception:
                        pass
                self.reader.tx_queue.clear()
                time.sleep(0.05)
            else:
                with self.reader.lock:
                    try:
                        self.reader.ser.write(bytes([self.reader.tx_queue.popleft()]))
                    except Exception:
                        pass
                time.sleep(self.gap)

    def stop(self) -> None:
        self.running = False


class EpsonSyncNode(Node):

    def __init__(self) -> None:
        super().__init__("epson_imu_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 460800)
        self.declare_parameter("topic", "/livox/imu")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("push_interval", 10.0)
        self.declare_parameter("rate", 400.0)
        self.declare_parameter("tx_gap", DEFAULT_TX_GAP_S)
        # 调试/校准用：发给 STM32 的 UTC 再加这么多秒。
        # 用来判断雷达的那点固定偏差是不是"我们给的整秒没对准"。
        self.declare_parameter("utc_shift", 0.0)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.push_interval = float(self.get_parameter("push_interval").value)
        rate = float(self.get_parameter("rate").value)
        self.tx_gap = float(self.get_parameter("tx_gap").value)
        self.utc_shift = float(self.get_parameter("utc_shift").value)

        queue: deque = deque(maxlen=2000)
        try:
            self.reader = FrameReader(port, baud, queue)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "打不开串口 %s：%s\n"
                "  检查：1) 串口名字对不对（ls /dev/ttyUSB*）"
                "  2) 有没有被别的进程占着  3) 在不在 dialout 组" % (port, exc)
            )
            raise
        self.reader.start()
        self.pusher = TimePusher(self.reader, self.tx_gap)
        self.pusher.start()
        self.queue = queue

        # 偏移量估计：主机时间 - STM32 时间，最近 2 秒的中位数（400 个样本 @200Hz）
        self.offsets: deque = deque(maxlen=400)
        self.offset = None
        self.last_calc = 0.0
        self.last_push = 0.0

        self.rate_cnt = 0
        self.rate_t0 = time.time()

        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(Imu, topic, qos)

        self.get_logger().info("已打开 %s @ %d，发布 %s" % (port, baud, topic))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self) -> None:
        if not self.queue:
            return
        sample = self.queue.popleft()
        self.offsets.append(sample["host"] - sample["t_us"] * 1e-6)

        now = time.time()
        if self.offset is None or now - self.last_calc > 0.5:
            self.offset = statistics.median(self.offsets) if self.offsets else 0.0
            self.last_calc = now

        # 每 10 秒给 STM32 推一次 UTC：既更新它的时间基准，
        # 也把给雷达的 1Hz PPS 相位掰到真正的整秒边界上
        if now - self.last_push > self.push_interval:
            utc = time.time()
            utc += self.utc_shift
            command = b"T%d,%d\n" % (int(utc), int((utc % 1) * 1000))
            self.reader.tx_queue.clear()
            self.reader.tx_queue.extend(command)
            self.last_push = now

        stamp = self.offset + sample["t_us"] * 1e-6
        sec = int(stamp)
        nsec = int(round((stamp - sec) * 1e9))
        if nsec >= 1000000000:
            sec += 1
            nsec -= 1000000000

        msg = Imu()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.header.frame_id = self.frame_id

        gx, gy, gz = sample["gyro"]
        ax, ay, az = sample["accl"]
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 1e-6
        msg.angular_velocity_covariance[4] = 1e-6
        msg.angular_velocity_covariance[8] = 1e-6
        msg.linear_acceleration_covariance[0] = 1e-4
        msg.linear_acceleration_covariance[4] = 1e-4
        msg.linear_acceleration_covariance[8] = 1e-4

        self.pub.publish(msg)
        self.rate_cnt += 1

        if now - self.rate_t0 >= 2.0:
            self.get_logger().info(
                "%.1f Hz  坏帧 %d  偏移 %.6f s  温度 %.2f C"
                % (
                    self.rate_cnt / (now - self.rate_t0),
                    self.reader.bad,
                    self.offset if self.offset else 0.0,
                    sample["temp"],
                )
            )
            self.rate_cnt = 0
            self.rate_t0 = now

    def destroy_node(self) -> None:
        try:
            self.pusher.stop()
            self.reader.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EpsonSyncNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
