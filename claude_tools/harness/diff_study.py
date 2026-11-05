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

def run(scenario, T=8.0):
    node = oc.ObstacleCourseNode(is_simulation=True)
    node.occupancy_grid_resolution = RES
    node.origin = (g['xb'] - 0.1, g['c2b'] - 0.1)                  # cover this 2D layout (test setup only)
    node.grid_n_x = int(round((g['xe'] + 0.2 - node.origin[0]) / RES)); node.grid_n_y = int(round((g['w1'] / 2 + 0.2 - node.origin[1]) / RES))
    node.occupancy_grid = np.full((node.grid_n_x, node.grid_n_y), -1, np.int8)
    ym = 0.5 * (gy0 + gy1)
    def pose_at(t):
        if scenario == 'stationary':
            return xd + 0.20 - 0.0725, ym                             # centre 0.2 m past the door wall
        if scenario == 'moving_x':                                    # through the door at 0.3 m/s
            return xd - 0.6 + 0.3 * t - 0.0725, ym
        if scenario == 'moving_y':                                    # sideways, just past the door wall, 0.3 m/s back and forth
            lo, hi = g['c2b'] + 0.2, g['c2t'] - 0.2
            u = (0.3 * t / (hi - lo)) % 2.0; f = u if u < 1 else 2 - u
            return xd + 0.20 - 0.0725, lo + f * (hi - lo)
    I, J = np.meshgrid(np.arange(node.grid_n_x), np.arange(node.grid_n_y), indexing='ij')
    WX = node.origin[0] + (I + 0.5) * RES; WY = node.origin[1] + (J + 0.5) * RES
    stats = {'cb': [], 'scan': []}
    prev_cb = prev_scan = None; prev_scan_t = None
    t, tick = 0.0, 0
    while t < T:
        x, y = pose_at(t)
        if tick % 2 == 0:
            r = w.lidar(x, y, 0.0, t)
            r = np.where(np.isfinite(r), np.round(r, 2), np.inf)          # Gazebo range resolution 0.01 m, no noise
            node.sub_scan_callback(Msg(ranges=list(r)))
        node.pose = np.array((x, y, 0.0))
        node.timer_callback()
        occ = node.occupancy_grid == 99
        def diff(prev, t_prev, key):
            vac = prev & ~occ; new = ~prev & occ
            near = np.zeros_like(occ)
            for s in w.swingers:
                for tt in (t, t_prev):
                    near |= point_rect(WX, WY, s['x'], w.swinger_y(s, tt), s['hx'], s['hy']) <= 0.03
            nb = new.copy()                                                # the user's idea: a vacated cell with a newly occupied neighbour
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb |= np.roll(np.roll(new, di, 0), dj, 1)
            pair = vac & nb
            if t > 1.0:                                                    # skip the first second (map filling in)
                stats[key].append([(vac & ~near).sum(), (new & ~near).sum(), (pair & ~near).sum(),
                                   (vac & near).sum(), (new & near).sum(), (pair & near).sum()])
        if prev_cb is not None:
            diff(prev_cb, t - 0.05, 'cb')
        if tick % 2 == 0:
            if prev_scan is not None:
                diff(prev_scan, prev_scan_t, 'scan')
            prev_scan, prev_scan_t = occ.copy(), t
        prev_cb = occ.copy()
        t += 0.05; tick += 1
    return {k: np.array(v).mean(0) for k, v in stats.items()}

for sc in ('stationary', 'moving_x', 'moving_y'):
    res = run(sc)
    for k, m in res.items():
        _print('seed %d %-10s %-4s static: vacated %5.1f new %5.1f pairs %5.1f | swinging: vacated %5.1f new %5.1f pairs %5.1f' % ((seed, sc, k) + tuple(m)))
