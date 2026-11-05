'''Run the user's node with is_simulation=False: 720-reading scans (the template keeps every 2nd), maze-frame poses
starting at the measured start point (2.82, 2.94, 96.3 deg). The 2D course is placed into the maze frame by a fixed
rotation + shift. Reports only exceptions.'''
import sys, types, math, time, traceback, numpy as np
TT = []
user_pkg_dir, sim_dir, T_END = sys.argv[1], sys.argv[2], float(sys.argv[3])
sys.path.insert(0, sim_dir)
class Msg:
    def __init__(self, **k): self.__dict__.update(k)
class Twist:
    def __init__(self): self.linear = Msg(x=0., y=0., z=0.); self.angular = Msg(x=0., y=0., z=0.)
CMD = [0.0, 0.0, 0.0]; LOG = []
class Node:
    def __init__(self, name): pass
    def get_logger(self):
        class L:
            def info(self, m, **k): LOG.append(m)
            def error(self, m, **k): LOG.append(m)
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
w = World(np.random.default_rng(0))
node = oc.ObstacleCourseNode(is_simulation=False)
S, PHI = np.array((2.82, 2.94)), math.radians(float(sys.argv[4]))        # course start in maze frame, course +x direction in maze frame
def to_maze(x, y, th):
    c, s = math.cos(PHI), math.sin(PHI)
    return np.array((S[0] + c * x - s * y, S[1] + s * x + c * y, math.degrees(th + PHI)))
x = y = th = 0.0
t, ticks, err = 0.0, 0, None
try:
    while t < T_END:
        if ticks % 2 == 0:
            node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t, n=720))))   # 720 raw readings, like the RPLIDAR
        node.pose = to_maze(x, y, th)
        _t0 = time.perf_counter(); node.timer_callback(); TT.append(time.perf_counter() - _t0)
        c, s = math.cos(th), math.sin(th)
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05; th += CMD[2] * 0.05
        t += 0.05; ticks += 1
except Exception as e:
    err = e; traceback.print_exc(limit=3)
print('real mode: %d ticks (%.1f s) %s | scan length seen by the node: %d | course-frame end (%.2f, %.2f)'
      % (ticks, t, 'then CRASHED' if err else 'without any exception', len(node.last_scan), x, y))

TT = np.array(TT) * 1000
print('tick ms: median %.0f  p90 %.0f  max %.0f | ticks over 50 ms: %d of %d' % (np.median(TT), np.percentile(TT, 90), TT.max(), (TT > 50).sum(), len(TT)))
from collections import Counter
