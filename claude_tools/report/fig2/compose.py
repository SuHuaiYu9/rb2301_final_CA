import cv2, numpy as np
B = '/tmp/claude-1000/-home-jacky-Downloads-rb2301-final-project-main/046b2aa8-8f72-439a-aad2-61e73ef2cc4b/'
bg = cv2.imread(B + 'images/18.webp')                       # clean frame (0:17)
H = np.load(B + 'scratchpad/fig2/H.npy')                    # 0:16 scribble frame -> clean frame
def T(pts): return cv2.perspectiveTransform(np.float32(pts)[None], H)[0]

def chaikin(p, it=3):
    p = np.float32(p)
    for _ in range(it):
        q = [p[0]]
        for a, b in zip(p[:-1], p[1:]): q += [0.75 * a + 0.25 * b, 0.25 * a + 0.75 * b]
        q.append(p[-1]); p = np.float32(q)
    return p

# the user's black line, traced in the 0:16 frame; corners between segments stay sharp
segs = [
    [(160, 960), (215, 860), (265, 770), (300, 700), (322, 640)],                      # 1 -> 2 probe
    [(322, 640), (305, 690), (290, 735)],                                              # back out to 3
    [(290, 735), (480, 728), (900, 735), (1200, 740), (1500, 745), (1525, 738)],       # 4 along the corridor
    [(1525, 738), (1600, 620), (1635, 545), (1690, 600), (1640, 665), (1540, 650),
     (1505, 600), (1560, 545), (1660, 520), (1735, 560), (1740, 640), (1660, 655), (1500, 605)],  # 5 pocket
    [(1500, 605), (1250, 598), (1110, 597)],                                           # 6 new lane
    [(1110, 597), (1000, 515), (905, 445), (905, 335)],                                # gap, door
]
S = 2                                                        # draw at 2x, downscale for smooth edges
big = cv2.resize(bg, None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)
RED, WHITE, DARK = (80, 0, 225), (255, 255, 255), (35, 35, 35)

def draw_path(img, pts, w):
    p = np.int32(np.round(pts * S))
    cv2.polylines(img, [p], False, WHITE, (w + 6) * S, cv2.LINE_AA)
    cv2.polylines(img, [p], False, RED, w * S, cv2.LINE_AA)

def chevron(img, pts, i, size=16):              # small arrowhead along a path at index i
    a, b = pts[i - 2], pts[i + 2]
    d = (b - a) / np.linalg.norm(b - a); n = np.array([-d[1], d[0]])
    tip = pts[i] + d * size * 0.6
    tri = np.int32(np.round(np.array([tip, tip - d * size + n * size * 0.7, tip - d * size - n * size * 0.7]) * S))
    cv2.fillConvexPoly(img, tri, RED, cv2.LINE_AA)
    cv2.polylines(img, [tri], True, WHITE, 2 * S, cv2.LINE_AA)

paths = [chaikin(T(s)) for s in segs]
# trap pocket: dashed outline
cx, cy = T([(1620, 585)])[0]
ell = cv2.ellipse2Poly((int(cx * S), int(cy * S)), (int(255 * S), int(105 * S)), -4, 0, 360, 2)
for j in range(0, len(ell) - 1, 8):
    cv2.polylines(big, [ell[j:j + 5]], False, (0, 215, 255), 5 * S, cv2.LINE_AA)
# robot photos, each taken from the video where the camera sees the robot from that side:
# 17 = rear-right (left of the frame: 1, 2), 19 = rear-left (5), 20 = straight from behind (6, 7)
def load(n, mask, plate_px, squash=1.0):
    spr = cv2.imread(B + f'images/{n}.png')[..., :3]; msk = cv2.imread(B + 'scratchpad/fig2/' + mask, 0)
    x, y, w, h = cv2.boundingRect(msk)
    return dict(s=spr[y:y + h, x:x + w], m=cv2.GaussianBlur(msk[y:y + h, x:x + w], (3, 3), 0),
                plate=plate_px, squash=squash)
SPR = {'17': load('17', 'robot_mask.png', 170, 0.93), '19': load('19', 'robot_mask_19.png', 88),
       '20': load('20', 'robot_mask_20.png', 115)}
# size from depth: the real robot's marker plate is 117 px wide with its wheels at row 755 (0:17 frame);
# apparent size grows linearly with the distance below the horizon row YH; VP = column where the robot is seen head-on
YH, KP, VPX, F = -250.0, 117.0 / (755.0 + 250.0), 976.0, 1400.0
def paste(img, base, name, alpha=0.95):
    bx, by = base; sp = SPR[name]
    cosb = np.cos(np.arctan((bx - VPX) / F))              # the plate looks narrower when seen at an angle
    k = KP * (by - YH) * cosb / sp['plate']
    W, Hh = int(sp['s'].shape[1] * k * S), int(sp['s'].shape[0] * k * sp['squash'] * S)
    s = cv2.resize(sp['s'], (W, Hh), interpolation=cv2.INTER_AREA); a = cv2.resize(sp['m'], (W, Hh)) / 255.0 * alpha
    x0, y0 = int(bx * S - W / 2), int(by * S - Hh)
    roi = img[y0:y0 + Hh, x0:x0 + W]
    roi[:] = (roi * (1 - a[..., None]) + s * a[..., None]).astype(np.uint8)

# robot copies at key moments (0:16 floor points, photo); 4 is the real robot already in the frame
copies = {1: ((175, 965), '17'), 2: ((318, 648), '17'), 5: ((1655, 612), '19'), 6: ((1300, 610), '20'),
          7: ((905, 452), '20')}
for k, (v, name) in copies.items(): paste(big, T([v])[0], name)

# path on top of the copies
for p in paths: draw_path(big, p, 7)
# arrowheads: mid-corridor, mid-lane, and the end at the door
chevron(big, paths[2], len(paths[2]) // 2); chevron(big, paths[2], len(paths[2]) // 5)
chevron(big, paths[4], len(paths[4]) // 2); chevron(big, paths[0], len(paths[0]) // 2)
end = paths[5][-1]; d = (paths[5][-1] - paths[5][-6]); d /= np.linalg.norm(d); n = np.array([-d[1], d[0]])
tri = np.int32(np.round(np.array([end + d * 22, end - d * 6 + n * 18, end - d * 6 - n * 18]) * S))
cv2.fillConvexPoly(big, tri, RED, cv2.LINE_AA); cv2.polylines(big, [tri], True, WHITE, 2 * S, cv2.LINE_AA)

def marker(img, c, n, r=27):
    c = (int(c[0] * S), int(c[1] * S))
    cv2.circle(img, c, (r + 3) * S, DARK, -1, cv2.LINE_AA); cv2.circle(img, c, r * S, WHITE, -1, cv2.LINE_AA)
    t = str(n); fs = 1.25 * S; (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_DUPLEX, fs, 2 * S)
    cv2.putText(img, t, (c[0] - tw // 2, c[1] + th // 2), cv2.FONT_HERSHEY_DUPLEX, fs, DARK, 2 * S, cv2.LINE_AA)
mk = {1: (55, 925), 2: (185, 560), 3: (195, 680), 4: (975, 805), 5: (1835, 470), 6: (1283, 372), 7: (795, 390)}
for n, p in mk.items(): marker(big, T([p])[0], n)

out = cv2.resize(big, None, fx=1 / S, fy=1 / S, interpolation=cv2.INTER_AREA)[95:1075]
cv2.imwrite(B + 'scratchpad/fig2/fig2_trap_run.png', out)
print(out.shape)
