import sys, math, numpy as np, xml.etree.ElementTree as ET
H, code, sdf, T, margin = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5])
src = open(f"{H}/drive_timed.py").read(); head = src[:src.index("import obstacle_course as oc")]
sys.argv = ["x", code, f"{H}/../sim", str(T)]; exec(head)
import obstacle_course as oc
from sim2d import World
import builtins; PR = []; builtins.print = lambda *a, **k: PR.append(' '.join(str(x) for x in a))
# walls from the world file: yaw 0 -> panel at y = py + 0.6 from px-0.59..px+0.59; yaw 90 -> panel at x = px - 0.6 from py-0.59..py+0.59
walls = []
for line in open(sdf):
    px, py, yaw = map(float, line.split())
    walls.append((px - 0.59, py + 0.6, px + 0.59, py + 0.6) if yaw == 0 else (px - 0.6, py - 0.59, px - 0.6, py + 0.59))
w = World(np.random.default_rng(0))
w.walls = walls; w.swingers = []
w.rects = [(1.035, 0.313, 0.205, 0.25, 0.0)]                         # grey box (drawer 0.41 x 0.50) at the pose Gazebo reported
w.circles = [(2.170, -1.020, 0.226), (2.569, -0.730, 0.03), (2.552, 0.121, 0.03)]   # litter bin, two cans
w.door = (3.48, -2.8547, -2.4453)
node = oc.ObstacleCourseNode(is_simulation=True)
node.side_margin = margin
x = y = th = 0.0; t, ticks = 0.0, 0; path = []; hits = 0; in_c = False
while t < T:
    if ticks % 2 == 0: node.sub_scan_callback(Msg(ranges=list(w.lidar(x, y, th, t))))
    node.pose = np.array((x, y, math.degrees(th))); CMD[:] = [0, 0, 0]
    node.timer_callback()
    hit = w.collides(x, y, th, t) == 'static'
    if hit and not in_c: hits += 1
    in_c = hit
    if ticks % 40 == 0: path.append((round(t), round(x + 0.0725, 2), round(y, 2)))
    if x + 0.0725 > 3.9: break
    c, s_ = math.cos(th), math.sin(th)
    x += (c * CMD[0] - s_ * CMD[1]) * 0.05; y += (s_ * CMD[0] + c * CMD[1]) * 0.05; t += 0.05; ticks += 1
sys.stdout.write("side_margin %.2f: %s at %.1f s, centre (%.2f, %.2f), contacts %d\n  path every 2 s: %s\n" % (margin, 'THROUGH THE DOOR' if x + 0.0725 > 3.9 else 'NOT through', t, x + 0.0725, y, hits, path))
