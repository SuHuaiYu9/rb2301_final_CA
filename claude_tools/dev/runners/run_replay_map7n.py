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
orig = getattr(oc, "best_direction", None)
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
w = World(np.random.default_rng(seed)); w.swingers = []
import xml.etree.ElementTree as ET
walls = []
for e in ET.parse(__import__("os").environ["SDF"]).getroot()[0]:
    if e.tag == 'include' and 'nist_maze_wall' in e.find('name').text:
        px, py, pz, r_, p_, yaw = map(float, e.find('pose').text.split())
        walls.append((px - 0.59, py + 0.6, px + 0.59, py + 0.6) if abs(yaw) < 0.1 else (px - 0.6, py - 0.59, px - 0.6, py + 0.59))
w.walls = walls
w.rects = [(2.605040, -1.970090, 0.205, 0.25, 0.0)]                                   # drawer (live Gazebo pose)
w.circles = [(2.279630, -2.517100, 0.226), (2.999540, -3.351750, 0.226), (2.983700, -2.826730, 0.03)]   # litter bins, coke (live poses)
w.door = (3.574, -2.7533, -2.3334)
node = oc.ObstacleCourseNode(is_simulation=True)
PATH=[]; PAST=[None]; BOUNCE=[0]; PREV=[None]; HITS=[0]; x = y = th = 0.0; t, ticks = 0.0, 0; err = ''
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
        if w.collides(x, y, th, t): HITS[0] += 1
        if PAST[0] is None and y < -2.85: PAST[0] = t
        if ticks % 20 == 0: PATH.append((round(t), round(x + 0.0725, 2), round(y, 2)))
        if x + 0.0725 > w.door[0] + 0.4: break
        x += CMD[0] * 0.05; y += CMD[1] * 0.05; t += 0.05; ticks += 1
except Exception as e:
    err = ' CRASH %s: %s' % (type(e).__name__, e)
builtins_print('past the litter bin at %s s; ' % (None if PAST[0] is None else round(PAST[0], 1)) + 'seed %d: %s (%.2f, %.2f) at %.1f s, ticks in contact %d%s reversals %d' % (seed, "THROUGH" if x + 0.0725 > w.door[0] + 0.4 else "ends", x, y, t, HITS[0], err, BOUNCE[0]))

if __import__('os').environ.get('SHOWPATH'): builtins_print('path every 1 s:', PATH)
