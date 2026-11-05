'''Run the user's node (ROS stubbed) for N seconds in the 2D course: scans at 10 Hz, timer at 20 Hz,
robot moved by the velocities the node publishes. Reports only exceptions (code-level problems).'''
import sys, types, math, time, traceback, numpy as np
user_pkg_dir, sim_dir, T_END = sys.argv[1], sys.argv[2], float(sys.argv[3])
sys.path.insert(0, sim_dir)
class Msg:
    def __init__(self, **k): self.__dict__.update(k)
class Twist:
    def __init__(self): self.linear = Msg(x=0., y=0., z=0.); self.angular = Msg(x=0., y=0., z=0.)
CMD = [0.0, 0.0, 0.0]
class Node:
    def __init__(self, name): self.logs = []
    def get_logger(self):
        node = self
        class L:
            def info(self, m, **k): node.logs.append(('info', m))
            def error(self, m, **k): node.logs.append(('error', m))
        return L()
    def create_subscription(self, *a, **k): return None
    def create_publisher(self, *a, **k):
        class P:
            def publish(self, m): CMD[:] = [m.linear.x, m.linear.y, m.angular.z]
        return P()
    def create_timer(self, *a): return None
mods = {n: types.ModuleType(n) for n in ('rclpy', 'rclpy.node', 'rclpy.logging', 'rclpy.qos', 'geometry_msgs', 'geometry_msgs.msg',
                                         'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg')}
mods['rclpy.node'].Node = Node
mods['rclpy.logging'].set_logger_level = lambda *a, **k: None
mods['rclpy.logging'].LoggingSeverity = types.SimpleNamespace(INFO=0, DEBUG=1)
mods['rclpy.qos'].ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=0); mods['rclpy.qos'].QoSProfile = lambda **k: None
mods['geometry_msgs.msg'].PoseStamped = object; mods['geometry_msgs.msg'].Twist = Twist
mods['nav_msgs.msg'].Odometry = object; mods['sensor_msgs.msg'].LaserScan = object
sys.modules.update(mods)
sys.path.insert(0, user_pkg_dir)
import obstacle_course as oc
from sim2d import World
w = World(np.random.default_rng(int(sys.argv[4]) if len(sys.argv) > 4 else 0))
if len(sys.argv) > 5 and sys.argv[5] == 'noswing': w.swingers = []
TT = []
node = oc.ObstacleCourseNode(is_simulation=True)
x = y = th = 0.0
t, ticks, err = 0.0, 0, None
try:
    while t < T_END:
        if ticks % 2 == 0:                                   # 10 Hz scans
            node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t))))
        node.pose = np.array((x, y, math.degrees(th)))      # what the odometry callback would set
        _t0 = time.perf_counter(); node.timer_callback(); TT.append(time.perf_counter() - _t0)
        c, s = math.cos(th), math.sin(th)                    # move the robot by the published (robot-frame) command
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05; th += CMD[2] * 0.05
        t += 0.05; ticks += 1
except Exception as e:
    err = e
    traceback.print_exc(limit=3)
print('ran %d timer ticks (%.1f s simulated) %s' % (ticks, t, 'then CRASHED' if err else 'without any exception'))
print('last command published (vx, vy, turn):', [round(v, 3) for v in CMD])

TT = np.array(TT) * 1000
print('tick ms: median %.0f  p90 %.0f  max %.0f | ticks over 50 ms: %d of %d' % (np.median(TT), np.percentile(TT, 90), TT.max(), (TT > 50).sum(), len(TT)))
print('odd (planning) ticks median %.0f ms, even ticks median %.0f ms' % (np.median(TT[0::2]), np.median(TT[1::2])))
from collections import Counter
print('logs:', Counter(m for _, m in node.logs))
print('final pose x %.2f y %.2f' % (x, y))
