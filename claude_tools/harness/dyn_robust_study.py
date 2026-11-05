'''Version 2 robustness: the user's grid-window count (cells 99 -> 0, their own comparisons) vs a scan-based 'seen-through' count
(old LiDAR points that the next scan now sees past by more than 10 cm), with LiDAR noise and pose jitter.
Args: <code_dir> <sim_dir> <secs> <seed> <start|door> <lidar_noise_m> <pose_jitter_m> <move|still> <grid_res_m> [dump_file]
start 'wall' = robot standing with its centre 0.15 m from the end wall of corridor 1 (a surface inside both windows).'''
import sys, types, math, time, traceback, numpy as np
user_pkg_dir, sim_dir, T_END, seed, start, NOISE, PJ, MOVE, RES_OVR = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5], float(sys.argv[6]), float(sys.argv[7]), sys.argv[8] == "move", float(sys.argv[9])
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
if start in ('start', 'wall'): w.swingers = []
g = w.geom
node = oc.ObstacleCourseNode(is_simulation=True)
node.dynamic_threshold = 10 ** 9                                     # never switch: only count
node.occupancy_grid_resolution = RES_OVR
RES = RES_OVR
node.origin = (g['xb'] - 0.1, g['c2b'] - 0.1)                         # grid covers this layout (test setup only)
node.grid_n_x = int(round((g['xe'] + 0.2 - node.origin[0]) / RES)); node.grid_n_y = int(round((g['w1'] / 2 + 0.2 - node.origin[1]) / RES))
node.occupancy_grid = np.full((node.grid_n_x, node.grid_n_y), -1, np.int8)
jr = np.random.default_rng(1000 + seed)
DTH = 6.28 / 359
def seen_through(prev, new, margin=0.10, ahead=1.0, half_w=0.5):
    (lx0, ly0, h0, r0), (lx1, ly1, h1, r1, cx1, cy1) = prev, new
    idx = np.arange(len(r0)); ok = np.isfinite(r0) & (r0 >= 0.05)
    a0 = h0 + idx * DTH
    wx = lx0 + np.where(ok, r0, 0) * np.cos(a0); wy = ly0 + np.where(ok, r0, 0) * np.sin(a0)
    dx, dy = wx - cx1, wy - cy1
    fr = dx * math.cos(h1) + dy * math.sin(h1); la = dx * math.sin(h1) - dy * math.cos(h1)
    inr = ok & (fr >= 0) & (fr <= ahead) & (np.abs(la) <= half_w)
    d = np.hypot(wx - lx1, wy - ly1); b = np.arctan2(wy - ly1, wx - lx1) - h1
    k = np.round(np.mod(b, 2 * math.pi) / DTH).astype(int) % len(r1)
    rn = np.where(np.isfinite(r1), r1, 10.0)
    rmin = np.minimum(np.minimum(rn[(k - 1) % len(r1)], rn[k]), rn[(k + 1) % len(r1)])
    return int(np.count_nonzero(inr & (rmin > d + margin)))
x = y = th = 0.0
if start == 'door':
    x, y = w.door[0] + 0.3 - 0.0725, 0.5 * (w.door[1] + w.door[2])
if start == 'wall':
    x, y = g['x1e'] - 0.15 - 0.0725, 0.0
t, ticks = 0.0, 0
grid_counts, st_counts = [], []          # per comparison / per scan pair, with flag: swinger in the region ahead
last = None
def swinger_ahead(cx, cy, th, t):
    for s in w.swingers:
        sx, sy = s['x'], w.swinger_y(s, t)
        dx, dy = sx - cx, sy - cy
        fr = dx * math.cos(th) + dy * math.sin(th); la = dx * math.sin(th) - dy * math.cos(th)
        if -0.05 <= fr <= 1.05 and abs(la) <= 0.5 + 0.15: return True
    return False
while t < T_END:
    px, py, ph = x + jr.normal(0, PJ), y + jr.normal(0, PJ), th + jr.normal(0, math.radians(0.3) if PJ > 0 else 0)
    cx, cy = x + 0.0725 * math.cos(th), y + 0.0725 * math.sin(th)
    if ticks % 2 == 0:
        r = np.array(w.lidar(x, y, th, t, noise=NOISE))
        node.sub_scan_callback(Msg(ranges=list(r)))
        lx, ly = px + 0.114 * math.cos(ph), py + 0.114 * math.sin(ph)      # what the node would compute from the (jittered) pose
        pcx, pcy = px + 0.0725 * math.cos(ph), py + 0.0725 * math.sin(ph)
        if last is not None:
            st_counts.append((seen_through(last, (lx, ly, ph, r, pcx, pcy)), swinger_ahead(cx, cy, th, t)))
        last = (lx, ly, ph, r)
    node.pose = np.array((px, py, math.degrees(ph)))
    prev = node.prev_window
    node.timer_callback()
    if node.prev_window is not prev and prev is not None:
        pi, pj, pvals = prev
        grid_counts.append((int(np.count_nonzero((pvals == 99) & (node.occupancy_grid[pi, pj] == 0))), swinger_ahead(cx, cy, th, t)))
    if MOVE:
        c, s = math.cos(th), math.sin(th)
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05
    t += 0.05; ticks += 1
def summ(rows, flag):
    v = np.array([c for c, f in rows if f == flag])
    return 'n=%d max %d p95 %d med %d' % (len(v), v.max(), np.percentile(v, 95), np.median(v)) if len(v) else 'n=0'
_print('seed %d %-5s noise %.3f jitter %.3f %-5s res %.3f | GRID no-swinger-ahead: %s | swinger-ahead: %s || SEEN-THROUGH no-swinger-ahead: %s | swinger-ahead: %s\n'
       % (seed, start, NOISE, PJ, 'move' if MOVE else 'still', RES, summ(grid_counts, False), summ(grid_counts, True), summ(st_counts, False), summ(st_counts, True)))
if len(sys.argv) > 10:
    import json; json.dump(dict(seed=seed, start=start, grid=grid_counts, st=st_counts), open(sys.argv[10], 'w'))
