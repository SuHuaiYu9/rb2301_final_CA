'''Reproduce: a false pendulum line on a static obstacle. Real-robot mode, no pendulums; a wide box sits in the robot's
path in corridor 1, and a "confirmed line" is injected at the box's front face once the robot is running.'''
import sys, math, numpy as np
H, code = sys.argv[1], sys.argv[2]
src = open(f"{H}/drive_timed.py").read(); head = src[:src.index("import obstacle_course as oc")]
sys.argv = ["x", code, f"{H}/../sim", "60"]; exec(head)
import obstacle_course as oc
from sim2d import World
import builtins; PR = []; builtins.print = lambda *a, **k: PR.append(' '.join(str(x) for x in a))
w = World(np.random.default_rng(0)); w.swingers = []
w.rects = [(1.20, 0.0, 0.08, 0.30, 0.0)]; w.circles = []          # a 0.16 x 0.60 box straight ahead, 1.2 m from the start
node = oc.ObstacleCourseNode(is_simulation=False); node.half_width = 0.11
PHI, S = math.radians(-90.0), np.array((2.82, 2.94))
def to_maze(px, py):
    c, s = math.cos(PHI), math.sin(PHI); return S[0] + c * px - s * py, S[1] + s * px + c * py
x = y = th = 0.0; t, ticks = 0.0, 0; injected = False; events = []
while t < 60 and not node.done and not node.stopped:
    if ticks % 2 == 0: node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t, n=720))))
    lx, ly = x + 0.114 * math.cos(th), y + 0.114 * math.sin(th)
    mx, my = to_maze(lx - 0.045 * math.cos(th), ly - 0.045 * math.sin(th))
    node.pose = np.array((mx, my, oc.wrap_deg(math.degrees(th + PHI))))
    if not injected and node.startup == 'run':
        h = np.deg2rad(node.pose[2] + node.heading_fix)
        fx, fy = to_maze(1.20 - 0.08, 0.0)                         # the box's front face, in the maze frame
        s_face, _ = oc.along_and_left(fx, fy, h)
        line = {'s': float(s_face), 'hits': [node.tick], 'confirmed': True, 'crossed': False}
        if 'ignored' in oc.update_lines.__doc__ or True: line['ignored'] = False
        node.lines.append(line); node.pendulums_seen = True; injected = True
    CMD[:] = [0, 0, 0]; PR.clear(); node.timer_callback()
    for p in PR:
        if 'check' not in p and 'squaring' not in p: events.append("t %5.1f centre (%.2f, %.2f): %s" % (t, x + 0.0725, y, p))
    c, s_ = math.cos(th), math.sin(th)
    x += (c * CMD[0] - s_ * CMD[1]) * 0.05; y += (s_ * CMD[0] + c * CMD[1]) * 0.05; th += CMD[2] * 0.05; t += 0.05; ticks += 1
print_ = sys.stdout.write
print_("\n".join(events[:8]) + "\n")
print_("end: t %.1f, centre (%.2f, %.2f), done=%s, final-approach mode=%s\n" % (t, x + 0.0725, y, node.done,
       bool(any(L['confirmed'] and L['crossed'] and not L.get('ignored', False) for L in node.lines))))
