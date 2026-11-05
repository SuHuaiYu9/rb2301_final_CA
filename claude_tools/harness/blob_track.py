'''How many occupied cells change between consecutive grids, on static walls vs on the swinging obstacles?
Uses the user's own mapping code (timer_callback, Gazebo branch). Planning is stubbed out (only the map matters here).
The robot pose is controlled by this script: stationary past the door, or moving at 0.3 m/s.'''
import sys, types, math, numpy as np
code_dir, sim_dir, seed = sys.argv[1], sys.argv[2], int(sys.argv[3])
sys.path.insert(0, sim_dir)
class Msg:
    def __init__(self, **k): self.__dict__.update(k)
class Twist:
    def __init__(self): self.linear = Msg(x=0., y=0., z=0.); self.angular = Msg(x=0., y=0., z=0.)
class Node:
    def __init__(self, name): pass
    def get_logger(self):
        class L:
            def info(self, m, **k): pass
            def error(self, m, **k): pass
        return L()
    def create_subscription(self, *a, **k): return None
    def create_publisher(self, *a, **k):
        class P:
            def publish(self, m): pass
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
sys.path.insert(0, code_dir)
import builtins; _print = builtins.print
import obstacle_course as oc
oc.A_star = lambda *a: []; oc.select_target = lambda *a: (0, 0); oc.select_target2 = lambda *a: (0, 0)
builtins.print = lambda *a, **k: None
from sim2d import World
w = World(np.random.default_rng(seed), swing_model='gazebo')
# Gazebo actor speeds: 1.2 m in 4.0 s and 1.2 m in 2.5 s (0.30 and 0.48 m/s)
for s, half in zip(w.swingers, (4.0, 2.5)):
    s['half'] = half * (s['hi'] - s['lo']) / 1.2
g = w.geom; xd, (gy0, gy1) = g['xd'], w.door[1:]
RES = 0.01

def point_rect(px, py, sx, sy, hx, hy):
    return np.hypot(np.maximum(np.abs(px - sx) - hx, 0), np.maximum(np.abs(py - sy) - hy, 0))


node = oc.ObstacleCourseNode(is_simulation=True)
node.occupancy_grid_resolution = RES
node.origin = (g['xb'] - 0.1, g['c2b'] - 0.1)
node.grid_n_x = int(round((g['xe'] + 0.2 - node.origin[0]) / RES)); node.grid_n_y = int(round((g['w1'] / 2 + 0.2 - node.origin[1]) / RES))
node.occupancy_grid = np.full((node.grid_n_x, node.grid_n_y), -1, np.int8)
I, J = np.meshgrid(np.arange(node.grid_n_x), np.arange(node.grid_n_y), indexing='ij')
WX = node.origin[0] + (I + 0.5) * RES; WY = node.origin[1] + (J + 0.5) * RES
zone = (WY > g['c2b'] + 0.10) & (WY < g['c2t'] - 0.10)
x, y = xd + 0.20 - 0.0725, 0.5 * (w.door[1] + w.door[2])
t, tick = 0.0, 0; prev = [None, None]; err = [[], []]; vis = [0, 0]; n = 0
while t < 12.0:
    if tick % 2 == 0:
        r = w.lidar(x, y, 0.0, t); r = np.where(np.isfinite(r), np.round(r, 2), np.inf)
        node.sub_scan_callback(Msg(ranges=list(r)))
    node.pose = np.array((x, y, 0.0)); node.timer_callback()
    if tick % 2 == 0 and t > 1.0:
        occ = node.occupancy_grid == 99; n += 1
        for k, s in enumerate(w.swingers):
            band = occ & zone & (np.abs(WX - s['x']) < 0.06)        # known x of the swing line
            if band.sum() >= 5:
                vis[k] += 1
                c = WY[band].mean()
                if prev[k] is not None:
                    est = (c - prev[k][0]) / (t - prev[k][1]); true = (w.swinger_y(s, t) - w.swinger_y(s, prev[k][1])) / (t - prev[k][1])
                    err[k].append(abs(est - true))
                prev[k] = (c, t)
            else:
                prev[k] = None
    t += 0.05; tick += 1
for k in range(2):
    e = np.array(err[k]); _print('obstacle %d: seen in %d of %d scans; speed error from blob-centre shift: median %.2f m/s, 90%% %.2f m/s (true speed %.2f m/s)'
        % (k + 1, vis[k], n, np.median(e), np.percentile(e, 90), (w.swingers[k]['hi'] - w.swingers[k]['lo']) / w.swingers[k]['half']))
