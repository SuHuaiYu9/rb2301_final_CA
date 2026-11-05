'''Version 2 dynamic-obstacle detection: count the window cells that go occupied (99) -> free (0) at every comparison,
with detection switched off (threshold huge), so the robot keeps planning. Args: <code_dir> <sim_dir> <secs> <seed> <swing|noswing> [door]
'door' starts the robot 0.3 m past the door wall, in the middle of the gap, facing +x.'''
import sys, types, math, time, traceback, numpy as np
user_pkg_dir, sim_dir, T_END, seed, mode = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
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
import builtins; builtins.print = lambda *a, **k: None
_print = sys.stdout.write
try:
    w = World(np.random.default_rng(seed))
except RuntimeError:
    _print('seed %d: no layout\n' % seed); sys.exit()
if mode == 'noswing': w.swingers = []
g = w.geom
node = oc.ObstacleCourseNode(is_simulation=True)
node.dynamic_threshold = 10 ** 9                                   # never switch state: measure only
RES = node.occupancy_grid_resolution
node.origin = (g['xb'] - 0.1, g['c2b'] - 0.1)                       # grid covers this layout (test setup only)
node.grid_n_x = int(round((g['xe'] + 0.2 - node.origin[0]) / RES)); node.grid_n_y = int(round((g['w1'] / 2 + 0.2 - node.origin[1]) / RES))
node.occupancy_grid = np.full((node.grid_n_x, node.grid_n_y), -1, np.int8)
x = y = th = 0.0; t, ticks = 0.0, 0
if len(sys.argv) > 6 and sys.argv[6] == 'door':
    x, y = w.door[0] + 0.3 - 0.0725, 0.5 * (w.door[1] + w.door[2])
rows = []                                                           # (t, robot centre x, freed, swinger within 0.6 m of the window centre)
while t < T_END:
    if ticks % 2 == 0:
        node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t))))
    node.pose = np.array((x, y, math.degrees(th)))
    prev = node.prev_window
    node.timer_callback()
    if node.prev_window is not prev and prev is not None:          # a comparison happened this tick
        pi, pj, pvals = prev
        freed = int(np.count_nonzero((pvals == 99) & (node.occupancy_grid[pi, pj] == 0)))
        cx = x + 0.0725 * math.cos(th); cy = y + 0.0725 * math.sin(th)
        wx, wy = cx + 0.1 * math.cos(th), cy + 0.1 * math.sin(th)
        near = any(abs(s['x'] - wx) < 0.6 and abs(w.swinger_y(s, t) - wy) < 0.8 for s in w.swingers)
        rows.append((t, cx, freed, near))
    c, s = math.cos(th), math.sin(th)
    x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05
    t += 0.05; ticks += 1
r = np.array(rows, dtype=float)
before = r[r[:, 1] < w.door[0]]; after = r[r[:, 1] >= w.door[0]]
def summ(a):
    if len(a) == 0: return 'n=0'
    f = a[:, 2]
    return 'n=%d max %d p99 %d p95 %d median %d' % (len(f), f.max(), np.percentile(f, 99), np.percentile(f, 95), np.median(f))
_print('seed %2d %-7s | before door: %s | after door: %s | after door, swinger near window: %s | robot ends x %.2f (door %.2f)\n'
       % (seed, mode, summ(before), summ(after), summ(after[after[:, 3] == 1]) if len(after) else 'n=0', x + 0.0725, w.door[0]))
