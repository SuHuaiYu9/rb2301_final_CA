import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
OUT = '/tmp/claude-1000/-home-jacky-Downloads-rb2301-final-project-main/046b2aa8-8f72-439a-aad2-61e73ef2cc4b/scratchpad/report/fig/'
plt.rcParams.update({'font.family': 'DejaVu Sans'})
INK, MUTED, ACC, ACC2, GRN = '#222222', '#5f5f5f', '#1f5f99', '#b5532c', '#2e7d32'

# ---------------- Figure 1: state machine ----------------
fig, ax = plt.subplots(figsize=(6.6, 2.15), dpi=300)
ax.set_xlim(0, 16); ax.set_ylim(0, 5.2); ax.axis('off')
states = [('navigate', '16-direction\nscoring'), ('check', 'still 2 s:\nreal motion?'),
          ('position', 'creep, then\nslide to wall'), ('wait', 'passed and\nleft yet?'),
          ('cross', 'dash past\nthe line'), ('find', 'look for the\n2nd one'),
          ('goal', 'straight until\nblocked'), ('done', 'stop')]
w, h, y = 1.62, 1.35, 2.55
xs = [1.0 + i * 2.0 for i in range(8)]
box = {}
for (name, sub), x in zip(states, xs):
    end = name in ('goal', 'done')
    ax.add_patch(FancyBboxPatch((x - w/2, y - h/2), w, h, boxstyle='round,pad=0.02,rounding_size=0.15',
                                fc='#eef6ee' if end else '#eef3f8', ec=INK, lw=0.8))
    ax.text(x, y + 0.33, name, ha='center', va='center', fontsize=7.6, weight='bold', color=INK)
    ax.text(x, y - 0.27, sub, ha='center', va='center', fontsize=5.6, color=MUTED, linespacing=1.1)
    box[name] = x
def arrow(p1, p2, rad=0.0, color=INK, lw=0.8):
    ax.add_patch(FancyArrowPatch(p1, p2, connectionstyle=f'arc3,rad={rad}', arrowstyle='-|>', mutation_scale=7,
                                 lw=lw, color=color, shrinkA=0, shrinkB=0))
for a_, b_ in [('navigate', 'check'), ('check', 'position'), ('position', 'wait'), ('wait', 'cross'),
               ('cross', 'find'), ('goal', 'done')]:
    arrow((box[a_] + w/2, y), (box[b_] - w/2, y))
top, bot = y + h/2, y - h/2
# above the row
arrow((box['find'], top), (box['check'], top), rad=0.22, color=ACC)
ax.text((box['find'] + box['check']) / 2, 5.0, '2nd pendulum detected', ha='center', va='center', fontsize=6, color=ACC)
arrow((box['find'] + 0.35, top), (box['goal'] - 0.35, top), rad=-0.75, color=MUTED)
ax.text((box['find'] + box['goal']) / 2, 4.05, 'none within t_wait', ha='center', va='center', fontsize=5.8, color=MUTED)
# below the row
arrow((box['cross'] + 0.2, bot), (box['goal'] - 0.2, bot), rad=0.35, color=ACC)
ax.text((box['cross'] + box['goal']) / 2, 0.55, 'after the 2nd crossing', ha='center', va='center', fontsize=6, color=ACC)
arrow((box['check'], bot), (box['navigate'] + 0.25, bot), rad=-0.7, color=ACC2)
ax.text((box['check'] + box['navigate']) / 2, 0.95, 'false alarm', ha='center', va='center', fontsize=6, color=ACC2)
arrow((box['wait'] - 0.2, bot), (box['navigate'] - 0.25, bot), rad=-0.42, color=ACC2)
ax.text(box['position'] - 0.3, 0.25, 'static, not a pendulum', ha='center', va='center', fontsize=6, color=ACC2)
fig.savefig(OUT + 'fig1_states.png', bbox_inches='tight', pad_inches=0.02)
plt.close(fig)

# ---------------- Figure 3: lane shift ----------------
fig, ax = plt.subplots(figsize=(3.0, 2.55), dpi=300)
ax.set_xlim(-0.50, 0.50); ax.set_ylim(-0.19, 0.66); ax.set_aspect('equal'); ax.axis('off')
gap_lo, gap_hi, wall_y, wall_t = -0.04, 0.24, 0.42, 0.03
ax.add_patch(Rectangle((-0.50, wall_y), gap_lo + 0.50, wall_t, fc='#8a8a8a', ec='none'))
ax.add_patch(Rectangle((gap_hi, wall_y), 0.50 - gap_hi, wall_t, fc='#8a8a8a', ec='none'))
ax.text(0.49, wall_y + wall_t + 0.015, 'door, 0.28 m', ha='right', va='bottom', fontsize=6, color=MUTED)
L, W, s = 0.115, 0.10, 0.10
ax.add_patch(Rectangle((-W, L), 2*W, wall_y - L, fc='#fbeee8', ec=ACC2, lw=0.8, ls='--'))     # straight sweep
ax.add_patch(Rectangle((s - W, L), 2*W, 0.66 - L, fc='#e8f2e8', ec=GRN, lw=0.8, alpha=0.75))   # shifted sweep
ax.add_patch(Rectangle((-W, -L), 2*W, 2*L, fc='#dbe7f3', ec=INK, lw=0.9))                     # robot
ax.add_patch(Rectangle((s - W, -L), 2*W, 2*L, fc='none', ec=GRN, lw=0.8, ls='--'))             # shifted robot
ax.text(0, 0.0, 'robot', ha='center', va='center', fontsize=6.3)
ax.annotate('', xy=(s, -0.15), xytext=(0, -0.15), arrowprops=dict(arrowstyle='-|>', lw=0.8, color=GRN))
ax.text(s / 2, -0.165, 'slide s = 10 cm', ha='center', va='top', fontsize=5.8, color=GRN)
ax.text(-W - 0.02, 0.27, 'straight lane:\nblocked by the\nwall after ~0.2 m', ha='right', va='center', fontsize=5.8, color=ACC2)
ax.text(s + W + 0.03, 0.22, 'shifted lane:\nclear through\nthe door', ha='left', va='center', fontsize=5.8, color=GRN)
fig.savefig(OUT + 'fig3_lane.png', bbox_inches='tight', pad_inches=0.02)
plt.close(fig)
print('ok')
