'''Real-robot mode run of a reactive node in one 2D layout: 720-reading scans, maze-frame poses (course rotated -90 deg, start at (2.82, 2.94)),
robot placed ROT0 deg off the corridor, Optitrack heading off by HERR deg, rotational drift DRIFT deg/s. LiDAR 0.045 m ahead of the node's pose point.
Args: <code_dir> <sim_dir> <secs> <seed> <real|gazebo|none> <lidar_noise_m> <pose_jitter_m> <ROT0> <HERR> <DRIFT> [flip|frozen]
flip = heading sign reversed (tests the spin check); frozen = the pose never changes (tests the forward check).'''
import sys, types, math, time, traceback, numpy as np
user_pkg_dir, sim_dir, T_END, seed, SWING, NOISE, PJ, ROT0, HERR, DRIFT = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5], float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10])
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
import builtins
PR = []; builtins.print = lambda *a, **k: PR.append(' '.join(str(x) for x in a))
_print = sys.stdout.write
try:
    w = World(np.random.default_rng(seed), swing_model='real' if SWING == 'real' else 'gazebo')
except RuntimeError:
    _print('seed %d: no layout\n' % seed); sys.exit()
if SWING == 'none': w.swingers = []
g = w.geom
node = oc.ObstacleCourseNode(is_simulation=False)
node.half_width = 0.11                                   # test setup only: match the simulated robot body (0.22 m wide)
PHI, S = math.radians(-90.0), np.array((2.82, 2.94))
jr = np.random.default_rng(1000 + seed)
def to_maze(px, py):
    c, s = math.cos(PHI), math.sin(PHI)
    return S[0] + c * px - s * py, S[1] + s * px + c * py
x, y, th = 0.0, 0.0, math.radians(ROT0); t, ticks = 0.0, 0
hits_s = hits_w = 0; in_s = in_w = False; hist = []; outcome = 'TIMEOUT'; msgs = []; max_dev = 0.0; crash = ''
try:
    while t < T_END:
        if SWING == 'real': w.update_release(x + 0.0725, t)
        if ticks % 2 == 0:
            node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t, n=720, noise=NOISE))))
        lx, ly = x + 0.114 * math.cos(th), y + 0.114 * math.sin(th)          # simulated LiDAR position
        ppx, ppy = lx - 0.045 * math.cos(th), ly - 0.045 * math.sin(th)      # node's pose point (LiDAR 0.045 ahead of it)
        mx, my = to_maze(ppx, ppy)
        hdg = oc.wrap_deg(math.degrees(th + PHI) + HERR + (jr.normal(0, 0.3) if PJ else 0))
        MODE = sys.argv[11] if len(sys.argv) > 11 else ''
        if MODE == 'flip': hdg = -hdg
        if MODE == 'frozen' and ticks > 0: mx, my, hdg = node.pose[0], node.pose[1], node.pose[2]
        node.pose = np.array((mx + jr.normal(0, PJ), my + jr.normal(0, PJ), hdg))
        CMD[:] = [0.0, 0.0, 0.0]; PR.clear()
        node.timer_callback()
        msgs += ['t %.1f %s' % (t, p) for p in PR if not p.startswith('crossing') and p not in ('pendulum line found',)]
        if node.startup == 'run' and node.tick > 1: max_dev = max(max_dev, abs(math.degrees(th)))
        hit = w.collides(x, y, th, t)
        if hit == 'static' and not in_s: hits_s += 1
        if hit == 'swing' and not in_w: hits_w += 1
        in_s, in_w = hit == 'static', hit == 'swing'
        if node.done: outcome = 'GOAL'; break
        if node.stopped: outcome = 'ABORTED'; break
        hist.append((x, y))
        if len(hist) > 600 and math.hypot(x - hist[-600][0], y - hist[-600][1]) < 0.05 and not node.waiting:
            outcome = 'STUCK'; break
        c, s = math.cos(th), math.sin(th)
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05; th += (CMD[2] + math.radians(DRIFT)) * 0.05
        t += 0.05; ticks += 1
except Exception as e:
    import traceback; crash = ' CRASH %s: %s' % (type(e).__name__, e); traceback.print_exc(limit=3)
cx = x + 0.0725 * math.cos(th)
last_sw = max([s_['x'] for s_ in w.swingers], default=w.door[0])
where = 'past the pendulums' if cx > last_sw + 0.1 else 'past the door' if cx > w.door[0] + 0.1 else 'before the door'
_print('seed %2d %-6s rot0 %+5.1f herr %+6.1f drift %+.1f | %-7s at %5.1f s | ends (%.2f, %.2f) %s, %.2f m from end wall | heading off the course by up to %.1f deg in the run, %.1f at the end | static %d, pendulum %d contacts%s\n   %s\n'
       % (seed, SWING, ROT0, HERR, DRIFT, outcome, t, cx, y, where, g['xe'] - cx, max_dev, math.degrees(th), hits_s, hits_w, crash, ' | '.join(msgs[:4])))
