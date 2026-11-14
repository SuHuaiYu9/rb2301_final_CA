# Figure 1 v2: pendulum state machine with orthogonal return routes; each label sits on its own route
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.path import Path
OUT = '/tmp/claude-1000/-home-jacky-Downloads-rb2301-final-project-main/046b2aa8-8f72-439a-aad2-61e73ef2cc4b/scratchpad/report/fig/'
plt.rcParams.update({'font.family': 'DejaVu Sans'})
INK, MUTED, ACC, ACC2 = '#222222', '#5f5f5f', '#1f5f99', '#b5532c'

fig, ax = plt.subplots(figsize=(6.6, 2.17), dpi=300)
ax.set_xlim(0, 16); ax.set_ylim(0, 5.26); ax.axis('off')
states = [('navigate', '16-direction\nscoring'), ('check', 'still 2 s:\nreal motion?'),
          ('position', 'creep, then\nslide to wall'), ('wait', 'passed and\nleft yet?'),
          ('cross', 'dash past\nthe line'), ('find', 'look for the\n2nd one'),
          ('goal', 'straight until\nblocked'), ('done', 'stop')]
w, h, y = 1.66, 1.35, 2.62
xs = [1.0 + i * 2.0 for i in range(8)]
box = {}
for (name, sub), x in zip(states, xs):
    end = name in ('goal', 'done')
    ax.add_patch(FancyBboxPatch((x - w/2, y - h/2), w, h, boxstyle='round,pad=0.02,rounding_size=0.15',
                                fc='#eef6ee' if end else '#eef3f8', ec=INK, lw=0.8))
    ax.text(x, y + 0.33, name, ha='center', va='center', fontsize=7.0, weight='bold', color=INK)
    ax.text(x, y - 0.27, sub, ha='center', va='center', fontsize=5.4, color=MUTED, linespacing=1.1)
    box[name] = x
top, bot = y + h/2, y - h/2

def arrow(p1, p2, color=INK):
    ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle='-|>', mutation_scale=7, lw=0.8, color=color, shrinkA=0, shrinkB=0))
for a_, b_ in [('navigate', 'check'), ('check', 'position'), ('position', 'wait'), ('wait', 'cross'),
               ('cross', 'find'), ('goal', 'done')]:
    arrow((box[a_] + w/2, y), (box[b_] - w/2, y))

def route(pts, color, r=0.22):
    """Polyline with rounded corners and an arrowhead at the end."""
    pts = [np.array(p, float) for p in pts]
    verts, codes = [pts[0]], [Path.MOVETO]
    for a, b, c in zip(pts[:-2], pts[1:-1], pts[2:]):
        u, v = (a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)
        verts += [b + u * r, b, b + v * r]; codes += [Path.LINETO, Path.CURVE3, Path.CURVE3]
    verts.append(pts[-1]); codes.append(Path.LINETO)
    ax.add_patch(FancyArrowPatch(path=Path(verts, codes), arrowstyle='-|>', mutation_scale=7, lw=0.9, color=color))

def label(x, yl, text, color, va):
    ax.text(x, yl, text, ha='center', va=va, fontsize=6, color=color,
            bbox=dict(boxstyle='square,pad=0.08', fc='white', ec='none'))

# above the row
y_det, y_none = 4.55, 3.85
route([(box['find'] - 0.3, top), (box['find'] - 0.3, y_det), (box['check'], y_det), (box['check'], top)], ACC)
label((box['check'] + box['find']) / 2, y_det + 0.1, '2nd pendulum detected', ACC, 'bottom')
route([(box['find'] + 0.35, top), (box['find'] + 0.35, y_none), (box['goal'] - 0.35, y_none), (box['goal'] - 0.35, top)], MUTED)
label((box['find'] + box['goal']) / 2 + 0.05, y_none + 0.1, 'none within t_wait', MUTED, 'bottom')
# below the row
y_fa, y_st, y_cr = 1.35, 0.55, 1.05
route([(box['check'], bot), (box['check'], y_fa), (box['navigate'] + 0.1, y_fa), (box['navigate'] + 0.1, bot)], ACC2)
label((box['check'] + box['navigate'] + 0.1) / 2, y_fa + 0.1, 'false alarm', ACC2, 'bottom')
route([(box['wait'], bot), (box['wait'], y_st), (box['navigate'] - 0.45, y_st), (box['navigate'] - 0.45, bot)], ACC2)
label(4.3, y_st + 0.1, 'static, not a pendulum', ACC2, 'bottom')
route([(box['cross'], bot), (box['cross'], y_cr), (box['goal'], y_cr), (box['goal'], bot)], ACC)
label((box['cross'] + box['goal']) / 2, y_cr + 0.1, 'after the 2nd crossing', ACC, 'bottom')
fig.savefig(OUT + 'fig1_states_raw.png', bbox_inches='tight', pad_inches=0.03)
plt.close(fig)

# pad to the old picture's aspect (1546 x 508) so it drops into the user's document frame unchanged
import cv2
im = cv2.imread(OUT + 'fig1_states_raw.png'); H, W = im.shape[:2]; target = 1546 / 508
if W / H > target: pad = int(round(W / target)) - H; im = cv2.copyMakeBorder(im, pad // 2, pad - pad // 2, 0, 0, cv2.BORDER_CONSTANT, value=(255, 255, 255))
else: pad = int(round(H * target)) - W; im = cv2.copyMakeBorder(im, 0, 0, pad // 2, pad - pad // 2, cv2.BORDER_CONSTANT, value=(255, 255, 255))
cv2.imwrite(OUT + 'fig1_states.png', im); print(im.shape, im.shape[1] / im.shape[0], target)
