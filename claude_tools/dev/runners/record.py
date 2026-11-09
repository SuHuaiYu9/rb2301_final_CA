import rclpy, numpy as np, sys, math, time, pickle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
rclpy.init(); n = Node('claude_read_only_recorder')
S, O, C = [], [], []
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
n.create_subscription(LaserScan, 'scan', lambda m: S.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, np.array(m.ranges))), qos_profile_sensor_data)
n.create_subscription(Odometry, 'odom', lambda m: O.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), qos_profile_sensor_data)
n.create_subscription(Twist, 'cmd_vel', lambda m: C.append((time.time(), m.linear.x, m.linear.y)), 10)
t0 = time.time()
while time.time() - t0 < float(sys.argv[2]): rclpy.spin_once(n, timeout_sec=0.02)
pickle.dump(dict(scans=S, odom=O, cmd=C), open(sys.argv[1], 'wb'))
print('scans %d, odom %d, cmd %d; robot from (%.2f, %.2f) to (%.2f, %.2f)' % (len(S), len(O), len(C), O[0][1], O[0][2], O[-1][1], O[-1][2]))
