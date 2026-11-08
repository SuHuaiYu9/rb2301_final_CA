'''Run the user's reactive code in the 2D sim (no pendulums) and record, at each decision, the goal score and
proximity score of every feasible direction. Args: <code_dir> <sim_dir> <secs> <seed>'''
import sys, types, math, numpy as np, builtins
code_dir, sim_dir, T_END, seed = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
sys.path.insert(0, sim_dir)
class Msg:
    def __init__(self, **k): self.__dict__.update(k)
class Twist:
    def __init__(self): self.linear = Msg(x=0., y=0., z=0.); self.angular = Msg(x=0., y=0., z=0.)
CMD = [0.0, 0.0, 0.0]
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
sys.path.insert(0, code_dir)
builtins_print = builtins.print; builtins.print = lambda *a, **k: None
import obstacle_course as oc
import sim2d; sim2d.ROBOT_BODY = (-0.034, 0.181, 0.095)   # Gazebo nanocar size
from sim2d import World
REC = []
orig = getattr(oc, 'best_direction', None)
def wrapped(visited_points, feas, pose, goal_direction, R, w):
    heading = np.deg2rad(pose[2]); visited = np.array(visited_points).reshape(-1, 2)
    rows = []
    for th in feas:
        px = pose[0] + 0.15 * np.cos(heading + th); py = pose[1] + 0.15 * np.sin(heading + th)
        g = np.cos(th - goal_direction)
        d = np.hypot(visited[:, 0] - px, visited[:, 1] - py); d = d[d <= 0.40]
        p = -w * (sum(R - d) / R) if len(d) else 0.0
        rows.append((th, g, p, len(d)))
    ch = orig(visited_points, feas, pose, goal_direction, R, w)
    REC.append((pose[0], pose[1], rows, ch))
    return ch
import os as _os
SWING = _os.environ.get('SWING', 'gazebo')
w = World(np.random.default_rng(seed), swing_model=SWING)
_HX = _os.environ.get('HX')
if _HX:
    for _s in w.swingers: _s['hx'] = float(_HX)
if _os.environ.get('AMP'):                                # swing half-width range, as in the other runners
    _lo, _hi = map(float, _os.environ['AMP'].split(','))
    _ar = np.random.default_rng(500 + seed)
    for _s in w.swingers:
        _A = float(_ar.uniform(_lo, _hi)); _mid = 0.5 * (_s['lo'] + _s['hi'])
        if w.swing_model == 'gazebo': _s['lo'], _s['hi'] = _mid - _A, _mid + _A
        else: _s['A'] = _A
if _os.environ.get('HALFMAX'):
    for _s in w.swingers: _s['half'] = min(_s['half'], float(_os.environ['HALFMAX']))
LAST = max(_s['x'] + _s['hx'] for _s in w.swingers)   # far side of the last pendulum
ML = []; HL = []; SW = [0]; ST = [0]; MODES = []; _in_sw = [False]; _in_st = [False]
node = oc.ObstacleCourseNode(is_simulation=True)
BOUNCE=[0]; PREV=[None]; HITS=[0]; x = y = th = 0.0; t, ticks = 0.0, 0; err = ''
try:
    while t < T_END:
        if ticks % 2 == 0: node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t, noise=float(__import__("os").environ.get("NOISE", "0"))))))
        node.pose = np.array((x, y, math.degrees(th)))
        CMD[:] = [0.0, 0.0, 0.0]; node.timer_callback()
        sd = getattr(node, 'selected_direction', None)
        if sd is None and getattr(node, 'direction_k', None) is not None: sd = node.direction_k * np.pi / 8
        if sd is not None and PREV[0] is not None and sd != PREV[0]:
            dd = abs((sd - PREV[0] + np.pi) % (2*np.pi) - np.pi)
            if dd >= np.deg2rad(134): BOUNCE[0] += 1
        if sd is not None: PREV[0] = sd
        if SWING == 'real': w.update_release(x + 0.0725, t)
        _h = w.collides(x, y, th, t)
        if _h == 'swing' and not _in_sw[0]:
            SW[0] += 1
            HL.append((round(t, 2), getattr(node, 'mode', None), round(x + 0.0725, 2), round(y, 2), [(round(_s['x'], 2), round(w.swinger_y(_s, t), 2)) for _s in w.swingers],
                       None if getattr(node, 'line', None) is None else round(float(oc.world_to_robot(node.line[None], node.pose)[0, 0]), 2), getattr(node, 'side', None)))
        if _h == 'static' and not _in_st[0]:
            HL.append((round(t, 2), 'STATIC', getattr(node, 'mode', None), round(x + 0.0725, 2), round(y, 2)))
        if _h == 'static' and not _in_st[0]: ST[0] += 1
        _in_sw[0], _in_st[0] = _h == 'swing', _h == 'static'
        _m = getattr(node, 'mode', None)
        if not MODES or MODES[-1][1] != _m:
            MODES.append((round(t, 1), _m))
            _ln = getattr(node, 'line', None)
            ML.append((round(t, 1), _m, 'robot', round(x + 0.0725, 2), round(y, 2), 'line', None if _ln is None else (round(float(_ln[0]), 2), round(float(_ln[1]), 2)), 'swing pts', len(getattr(node, 'swing_points', [])), 'side', getattr(node, 'side', None)))
        if x + 0.0725 - 0.1075 > LAST: break                # robot's rear past the last pendulum (where a goal stop would end the run)
        x += CMD[0] * 0.05; y += CMD[1] * 0.05; t += 0.05; ticks += 1
except Exception as e:
    err = ' CRASH %s: %s' % (type(e).__name__, e)
builtins_print('seed %d: %s (%.2f, %.2f) at %.1f s | pendulum hits %d, static hits %d%s' % (seed, "PAST" if x + 0.0725 - 0.1075 > LAST else "ends", x + 0.0725, y, t, SW[0], ST[0], err))
if _os.environ.get('SHOWMODES'): builtins_print('modes', MODES)

for h in HL: builtins_print('HIT', h)

builtins_print('door', [round(v, 2) for v in w.door], 'pendulums x', [round(_s['x'], 2) for _s in w.swingers], 'corridor y', round(w.geom['c2b'], 2), round(w.geom['c2t'], 2))
for m in ML: builtins_print('MODE', m)

_door = w.door[0]; _p1 = min(_s['x'] for _s in w.swingers)
_appr = [m for m in ML if m[1] == 'observe']
_false = [m for m in _appr if m[6] is None or abs((m[6][0] + 0.0725) - min(abs((m[6][0] + 0.0725) - (_s['x'] - _s['hx'])) for _s in w.swingers) - (m[6][0] + 0.0725)) > 1e9]
_lines = [(m[0], round(m[6][0] + 0.0725, 2)) for m in _appr]
_near = [min(abs(lx - (_s['x'] - _s['hx'])) for _s in w.swingers) for (_, lx) in _lines]
builtins_print('PROC procedures %d, false (line more than 0.1 m from a pendulum face) %d, lines %s' % (len(_lines), sum(d > 0.1 for d in _near), _lines))
