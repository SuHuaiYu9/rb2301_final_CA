import rclpy, numpy as np, sys, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
class G(Node):
    def __init__(self):
        super().__init__('claude_read_only_grab')
        self.scan = self.odom = None
        self.create_subscription(LaserScan, 'scan', self.s, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.o, qos_profile_sensor_data)
    def s(self, m): self.scan = m
    def o(self, m): self.odom = m
rclpy.init(); n = G()
for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.scan is not None and n.odom is not None: break
m, o = n.scan, n.odom
p = o.pose.pose; q = p.orientation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
print('scan: n %d, angle_min %.4f, angle_max %.4f, inc %.5f, range_min %.3f, range_max %.3f' % (len(m.ranges), m.angle_min, m.angle_max, m.angle_increment, m.range_min, m.range_max))
print('odom: x %.4f y %.4f yaw %.2f deg, vx %.3f vy %.3f' % (p.position.x, p.position.y, math.degrees(yaw), o.twist.twist.linear.x, o.twist.twist.linear.y))
np.savez(sys.argv[1], ranges=np.array(m.ranges), amin=m.angle_min, inc=m.angle_increment, rmin=m.range_min, pose=np.array((p.position.x, p.position.y, math.degrees(yaw))))
n.destroy_node(); rclpy.shutdown()
