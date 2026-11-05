'''Why do static cells go 99 -> 0 in the version 2 detection window? No swinging obstacles, detection off (threshold huge).
variant: base (scan every 2nd tick, as Gazebo) | fresh (new scan every tick) | still (robot never moves). Args: <code_dir> <sim_dir> <secs> <seed> <variant>'''
import sys, types, math, time, traceback, numpy as np
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
    u = np.clip(((px - ax) * ex + (py - ay) * ey) / L, 0, 1) if L else 0 * px
    return np.hypot(px - ax - u * ex, py - ay - u * ey)
try:
    w = World(np.random.default_rng(seed))
except RuntimeError:
    _print('seed %d: no layout\n' % seed); sys.exit()
w.swingers = []
g = w.geom
node = oc.ObstacleCourseNode(is_simulation=True)
node.dynamic_threshold = 10 ** 9
RES = node.occupancy_grid_resolution
node.origin = (g['xb'] - 0.1, g['c2b'] - 0.1)                       # grid covers this layout (test setup only)
node.grid_n_x = int(round((g['xe'] + 0.2 - node.origin[0]) / RES)); node.grid_n_y = int(round((g['w1'] / 2 + 0.2 - node.origin[1]) / RES))
node.occupancy_grid = np.full((node.grid_n_x, node.grid_n_y), -1, np.int8)
segs = w.static_segs()
x = y = th = 0.0; t, ticks = 0.0, 0
per_cmp = []                    # freed per comparison
by_tick = {'fresh': 0, 'stale': 0}
dist_hist = []                  # distance of each freed cell's centre from the nearest real surface
win_freed = 0
while t < T_END:
    fresh = (variant == 'fresh') or ticks % 2 == 0
    if fresh:
        node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t))))
    node.pose = np.array((x, y, math.degrees(th)))
    prev = node.prev_window
    before = node.occupancy_grid.copy()
    node.timer_callback()
    if prev is not None:
        pi, pj, pvals = prev
        now_free = (before[pi, pj] == 99) & (node.occupancy_grid[pi, pj] == 0)   # freed during THIS tick, among the saved window cells
        n = int(now_free.sum())
        by_tick['fresh' if fresh else 'stale'] += n
        if n:
            ci = pi[now_free]; cj = pj[now_free]
            px = node.origin[0] + (ci + 0.5) * RES; py = node.origin[1] + (cj + 0.5) * RES
            d = np.min([seg_dist(px, py, *s) for s in segs] + [np.abs(np.hypot(px - c[0], py - c[1]) - c[2]) for c in w.circles], axis=0)
            dist_hist += list(d)
        if node.prev_window is not prev:                                # a comparison happened this tick
            per_cmp.append(int(np.count_nonzero((pvals == 99) & (node.occupancy_grid[pi, pj] == 0))))
    if variant != 'still':
        c, s = math.cos(th), math.sin(th)
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05
    t += 0.05; ticks += 1
pc = np.array(per_cmp); dh = np.array(dist_hist)
tot = by_tick['fresh'] + by_tick['stale']
_print('seed %2d %-6s | per comparison: max %3d p95 %3d median %d | comparisons >= 25: %3d of %d | freed on fresh-scan ticks %4d, on stale-scan ticks %4d'
       ' | freed cells within 1.5 cm of a real surface: %s\n'
       % (seed, variant, pc.max(), np.percentile(pc, 95), np.median(pc), (pc >= 25).sum(), len(pc), by_tick['fresh'], by_tick['stale'],
          ('%d%%' % round(100 * (dh < 0.015).mean())) if len(dh) else '-'))
