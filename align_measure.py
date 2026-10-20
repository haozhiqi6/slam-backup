import statistics, time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from livox_interfaces.msg import CustomMsg

DUR = 25.0

class M(Node):
    def __init__(self):
        super().__init__("align_measure")
        ql = QoSProfile(depth=300)
        ql.reliability = ReliabilityPolicy.BEST_EFFORT
        ql.durability = DurabilityPolicy.VOLATILE
        qi = QoSProfile(depth=500)
        qi.reliability = ReliabilityPolicy.RELIABLE
        qi.durability = DurabilityPolicy.VOLATILE
        self.dl = deque(); self.di = deque()
        self.create_subscription(CustomMsg, "/livox/lidar", self.lcb, ql)
        self.create_subscription(Imu, "/livox/imu", self.icb, qi)
    def lcb(self, m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.dl.append(s - time.time())
    def icb(self, m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.di.append(s - time.time())

def st(name, d):
    if not d:
        print("%-6s 没收到数据" % name); return None
    v = list(d)
    mu = statistics.mean(v)
    print("%-6s 样本 %5d  平均 %+9.5f s  抖动 %6.1f ms  最大 %6.1f ms"
          % (name, len(v), mu, statistics.pstdev(v) * 1000, max(abs(x - mu) for x in v) * 1000))
    return mu

rclpy.init()
n = M()
print("采 %.0f 秒 ……（请保持静止）" % DUR)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
print("-" * 66)
ml = st("lidar", n.dl)
mi = st("imu", n.di)
if ml is not None and mi is not None:
    print("-> IMU 相对雷达超前 %+.4f 秒" % (mi - ml))
n.destroy_node()
rclpy.shutdown()