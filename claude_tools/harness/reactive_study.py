'''Run a reactive (no-grid) node through one 2D layout until it says it reached the goal, gets stuck or times out.
Args: <code_dir> <sim_dir> <secs> <seed> <gazebo|real|none> <lidar_noise_m> <pose_jitter_m>
'real' = pendulums held at one side until the robot is near the door, then released (like the real course).'''
import sys, types, math, time, traceback, numpy as np
user_pkg_dir, sim_dir, T_END, seed, SWING, NOISE, PJ = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4]), sys.argv[5], float(sys.argv[6]), float(sys.argv[7])
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
import os
if os.environ.get('AMP'):                                # test setup: swing half-width (m) for each pendulum, from the range AMP=lo,hi
    _lo, _hi = map(float, os.environ['AMP'].split(','))
    _ar = np.random.default_rng(500 + seed)
    for _s in w.swingers:
        _A = float(_ar.uniform(_lo, _hi)); _mid = 0.5 * (_s['lo'] + _s['hi'])
        if w.swing_model == 'gazebo': _s['lo'], _s['hi'] = _mid - _A, _mid + _A
        else: _s['A'] = _A
g = w.geom
node = oc.ObstacleCourseNode(is_simulation=True)
jr = np.random.default_rng(1000 + seed)
x = y = th = 0.0; t, ticks = 0.0, 0
static_hits = swing_hits = crossings = 0; in_s = in_w = False; waits = 0; hist = []; outcome = 'TIMEOUT'; crash = ''
t_seen = None
try:
    while t < T_END:
        if SWING == 'real': w.update_release(x + 0.0725, t)
        if ticks % 2 == 0:
            node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t, noise=NOISE))))
        node.pose = np.array((x + jr.normal(0, PJ), y + jr.normal(0, PJ), math.degrees(th) + (jr.normal(0, 0.3) if PJ else 0)))
        CMD[:] = [0.0, 0.0, 0.0]
        node.timer_callback()
        if getattr(node, 'waiting', False): waits += 1
        if any(p.startswith('crossing') for p in PR): crossings += 1
        PR.clear()
        if node.pendulums_seen and t_seen is None: t_seen = t
        hit = w.collides(x, y, th, t)
        if hit == 'static' and not in_s: static_hits += 1
        if hit == 'swing' and not in_w: swing_hits += 1
        in_s, in_w = hit == 'static', hit == 'swing'
        if node.done:
            outcome = 'GOAL'; break
        if SWING == 'none' and x + 0.0725 > w.door[0] + 0.4:
            outcome = 'THROUGH'; break                          # navigation-only test: through the door
        hist.append((x, y))
        if len(hist) > 600 and math.hypot(x - hist[-600][0], y - hist[-600][1]) < 0.05 and not getattr(node, 'waiting', False):
            outcome = 'STUCK'; break
        c, s = math.cos(th), math.sin(th)
        x += (c * CMD[0] - s * CMD[1]) * 0.05; y += (s * CMD[0] + c * CMD[1]) * 0.05
        t += 0.05; ticks += 1
except Exception as e:
    import traceback; crash = ' CRASH ' + type(e).__name__ + ': ' + str(e); traceback.print_exc(limit=2)
cx = x + 0.0725
last_sw = max([s['x'] for s in w.swingers], default=w.door[0])
where = ('past the pendulums' if cx > last_sw + 0.1 else 'past the door' if cx > w.door[0] + 0.1 else 'before the door')
_print('seed %2d %-6s noise %.2f | %-7s at %5.1f s | robot ends (%.2f, %.2f), %s, %.2f m from the end wall | static contacts %d, pendulum contacts %d | waited %.1f s, crossings %d, lines found %d | pendulums seen at %s%s\n'
       % (seed, SWING, NOISE, outcome, t, cx, y, where, g['xe'] - cx, static_hits, swing_hits, waits * 0.05, crossings, sum(1 for L in getattr(node, 'lines', []) if L['confirmed']),
          ('%.1f s' % t_seen) if t_seen is not None else 'never', crash))
