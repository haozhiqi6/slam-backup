import statistics, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

gx=[];gy=[];gz=[];ax=[];ay=[];az=[]
class M(Node):
    def __init__(self):
        super().__init__("imu_noise")
        q = QoSProfile(depth=500); q.reliability = ReliabilityPolicy.RELIABLE; q.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Imu, "/livox/imu", self.cb, q)
    def cb(self, m):
        gx.append(m.angular_velocity.x); gy.append(m.angular_velocity.y); gz.append(m.angular_velocity.z)
        ax.append(m.linear_acceleration.x); ay.append(m.linear_acceleration.y); az.append(m.linear_acceleration.z)

rclpy.init(); n=M()
t0=time.time()
while time.time()-t0 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.05)
print("样本数", len(gx))
def st(name, v):
    if len(v)<10:
        print(name,"样本太少"); return
    mu=statistics.mean(v); sd=statistics.pstdev(v)
    print("%-4s 均值 %+10.6f  标准差 %9.6f   (%.3f 度/秒)" % (name, mu, sd, sd*57.2958))
print("--- 陀螺 rad/s ---")
st("gx",gx); st("gy",gy); st("gz",gz)
def sa(name, v):
    if len(v)<10:
        print(name,"样本太少"); return
    mu=statistics.mean(v); sd=statistics.pstdev(v)
    print("%-4s 均值 %+10.5f  标准差 %9.5f" % (name, mu, sd))
print("--- 加速度 m/s^2 ---")
sa("ax",ax); sa("ay",ay); sa("az",az)
rclpy.shutdown()