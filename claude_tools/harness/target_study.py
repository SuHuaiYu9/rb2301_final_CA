'''Run the user's node (ROS stubbed) for N seconds in the 2D course: scans at 10 Hz, timer at 20 Hz,
robot moved by the velocities the node publishes. Reports only exceptions (code-level problems).'''
import sys, types, math, traceback, numpy as np
user_pkg_dir, sim_dir, T_END, seed, variant = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
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
def seg_dist(px, py, ax, ay, bx, by):
    ex, ey = bx - ax, by - ay; L = ex * ex + ey * ey
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / L))
    return math.hypot(px - ax - u * ex, py - ay - u * ey)
try:
    w = World(np.random.default_rng(seed))
except RuntimeError:
    _print('seed %d: no layout\n'); sys.exit()
g = w.geom
oc.DOOR_X = w.door[0] + 0.5                                     # user's "passed door" switch: only after this study stops
if variant == 'nearest_free':                                   # target = free, known, non-buffered cell closest to the goal
    oc.frontier_cells = lambda buffered, raw: (~buffered) & (raw == 0)
node = oc.ObstacleCourseNode(is_simulation=True)
RES = node.occupancy_grid_resolution
node.origin = (g['xb'] - 0.1, g['c2b'] - 0.1)                    # grid covers this layout (test setup only)
node.grid_n_x = int(round((g['xe'] + 0.2 - node.origin[0]) / RES)); node.grid_n_y = int(round((g['w1'] / 2 + 0.2 - node.origin[1]) / RES))
node.occupancy_grid = np.full((node.grid_n_x, node.grid_n_y), -1, np.int8)
node.global_goal = (g['xe'] - 0.3, 0.5 * (g['c2t'] + g['c2b']))   # goal past the swinging obstacles
x = y = th = 0.0; t, ticks = 0.0, 0
hist = []; contacts = 0; in_c = False; outcome = 'TIMEOUT'; crash = ''
try:
    while t < T_END:
        if ticks % 2 == 0:
            node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t))))
        node.pose = np.array((x, y, math.degrees(th)))
        node.timer_callback()
        hit = w.collides(x, y, th, t) == 'static'
        if hit and not in_c: contacts += 1
        in_c = hit
        if x + 0.0725 > w.door[0] + 0.15:
            outcome = 'DOOR at %5.1f s' % t; break
        hist.append((x, y))
        if len(hist) > 300 and math.hypot(x - hist[-300][0], y - hist[-300][1]) < 0.05:
            outcome = 'STUCK at %5.1f s' % t; break
        c, s = math.cos(th), math.sin(th)
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05
        t += 0.05; ticks += 1
except Exception as e:
    crash = ' CRASH ' + type(e).__name__
tg = node.target
tw = oc.grid_to_world(int(tg[0]), int(tg[1]), node.origin, RES) if tg is not None else (float('nan'),) * 2
wd = min(seg_dist(tw[0], tw[1], *s) for s in w.static_segs()) if tg is not None else float('nan')
_print('seed %2d %-12s %-16s static contacts %d | robot (%.2f, %.2f) | last target (%.2f, %.2f), %.2f m from nearest wall/obstacle | door x %.2f%s\n'
       % (seed, variant, outcome, contacts, x, y, tw[0], tw[1], wd, w.door[0], crash))
