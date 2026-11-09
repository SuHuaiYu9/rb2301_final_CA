import rclpy, numpy as np, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('claude_read_only_lag')
st = {'scan': None, 'odom': None, 'v': 0.0}; lags = []; speeds = []
n.create_subscription(LaserScan, 'scan', lambda m: st.__setitem__('scan', m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), qos_profile_sensor_data)
def od(m):
    st['odom'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; st['v'] = (m.twist.twist.linear.x ** 2 + m.twist.twist.linear.y ** 2) ** 0.5
n.create_subscription(Odometry, 'odom', od, qos_profile_sensor_data)
def tick():
    if st['scan'] is not None and st['odom'] is not None:
        lags.append(st['odom'] - st['scan']); speeds.append(st['v'])
n.create_timer(0.05, tick)
t0 = time.time()
while time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.01)
L = np.array(lags) * 1000; V = np.array(speeds)
print('at each 20 Hz tick, the newest scan is older than the newest pose by: median %.0f ms, 90th pct %.0f ms, max %.0f ms (%d ticks); robot speed median %.2f m/s' % (np.median(L), np.percentile(L, 90), L.max(), len(L), np.median(V)))
