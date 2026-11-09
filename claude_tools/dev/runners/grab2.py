import rclpy, numpy as np, sys, math, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
class G(Node):
    def __init__(self):
        super().__init__('claude_read_only_grab2')
        self.scans, self.odoms, self.cmds = [], [], []
        self.create_subscription(LaserScan, 'scan', lambda m: self.scans.append(m), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: self.odoms.append(m), qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.cmds.append((time.time(), m.linear.x, m.linear.y)), 10)
rclpy.init(); n = G(); t0 = time.time()
while time.time() - t0 < float(sys.argv[2]): rclpy.spin_once(n, timeout_sec=0.05)
def pose(o):
    p = o.pose.pose; q = p.orientation
    return p.position.x, p.position.y, math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))
P = np.array([pose(o) for o in n.odoms])
print('odom samples %d: x %.3f..%.3f, y %.3f..%.3f, yaw %.1f..%.1f' % (len(P), P[:,0].min(), P[:,0].max(), P[:,1].min(), P[:,1].max(), P[:,2].min(), P[:,2].max()))
print('cmd_vel messages %d' % len(n.cmds), 'distinct (vx, vy):', sorted(set((round(a,2), round(b,2)) for _, a, b in n.cmds))[:12])
step = max(1, len(n.odoms)//12)
for o in n.odoms[::step]: print('  pose (%.3f, %.3f, %.1f)' % pose(o))
m = n.scans[-1]
np.savez(sys.argv[1], ranges=np.array(m.ranges), inc=m.angle_increment, pose=np.array(pose(n.odoms[-1])), traj=P)
n.destroy_node(); rclpy.shutdown()
