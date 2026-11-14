import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
OUT = '/tmp/claude-1000/-home-jacky-Downloads-rb2301-final-project-main/046b2aa8-8f72-439a-aad2-61e73ef2cc4b/scratchpad/report/fig/'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8})
INK, MUTED, ACC, ACC2 = '#222222', '#666666', '#1f5f99', '#b5532c'

# ---------------- Figure 1: pendulum state machine ----------------
fig, ax = plt.subplots(figsize=(7.0, 2.35), dpi=300)
ax.set_xlim(0, 14); ax.set_ylim(0, 4.6); ax.axis('off')
states = [('navigate', 'reactive\nnavigation'), ('check', 'stand still 2 s:\n≥5 moving pts?'),
          ('position', 'creep to 10 cm,\nslide to wall'), ('wait', 'until pendulum\npassed & left'),
          ('cross', 'dash until rear\n10 cm past line'), ('find', 'look for 2nd\npendulum'),
          ('goal', 'straight until\n≤5 cm free'), ('done', 'stop')]
xs = [0.9, 2.65, 4.4, 6.15, 7.9, 9.65, 11.4, 13.1]
y = 2.2
box = {}
for (name, sub), x in zip(states, xs):
    w, h = 1.45, 1.05
    p = FancyBboxPatch((x - w/2, y - h/2), w, h, boxstyle='round,pad=0.02,rounding_size=0.12',
                       fc='#eef3f8' if name not in ('goal', 'done') else '#eef6ee', ec=INK, lw=0.8)
    ax.add_patch(p)
    ax.text(x, y + 0.22, name, ha='center', va='center', fontsize=8.5, weight='bold', color=INK)
    ax.text(x, y - 0.22, sub, ha='center', va='center', fontsize=6.2, color=MUTED, linespacing=1.05)
    box[name] = x
def arrow(x1, y1, x2, y2, rad=0.0, color=INK, label=None, lx=None, ly=None, lc=None):
    a = FancyArrowPatch((x1, y1), (x2, y2), connectionstyle=f'arc3,rad={rad}', arrowstyle='-|>',
                        mutation_scale=7, lw=0.8, color=color)
    ax.add_patch(a)
    if label:
        ax.text(lx, ly, label, ha='center', va='center', fontsize=6.2, color=lc or color)
# forward chain
fwd = [('navigate', 'check', 'detected,\npath clear'), ('check', 'position', 'confirmed'), ('position', 'wait', 'at wall'),
       ('wait', 'cross', 'GO'), ('cross', 'find', '1st'), ('goal', 'done', '')]
for a_, b_, lab in fwd:
    arrow(box[a_] + 0.73, y, box[b_] - 0.73, y)
    if lab:
        ax.text((box[a_] + box[b_]) / 2, y + 0.42, lab, ha='center', va='bottom', fontsize=6, color=MUTED, linespacing=1.0)
# cross -> goal (after 2nd), drawn below
arrow(box['cross'] + 0.2, y - 0.53, box['goal'] - 0.2, y - 0.53, rad=0.35, color=ACC, label='after 2nd crossing',
      lx=(box['cross'] + box['goal']) / 2, ly=0.55)
# find -> check (2nd pendulum), above
arrow(box['find'] - 0.1, y + 0.53, box['check'] + 0.1, y + 0.53, rad=0.28, color=ACC, label='2nd pendulum detected',
      lx=(box['find'] + box['check']) / 2, ly=4.35)
# find -> goal (timeout)
arrow(box['find'] + 0.25, y + 0.53, box['goal'] - 0.25, y + 0.53, rad=-0.5, color=MUTED, label='none in t_wait',
      lx=(box['find'] + box['goal']) / 2, ly=3.35)
# back to navigate
arrow(box['check'] - 0.3, y - 0.53, box['navigate'] + 0.3, y - 0.53, rad=-0.55, color=ACC2, label='false alarm',
      lx=(box['check'] + box['navigate']) / 2, ly=0.85)
arrow(box['wait'] - 0.25, y - 0.53, box['navigate'] + 0.1, y - 0.53, rad=-0.42, color=ACC2, label='static obstacle',
      lx=(box['wait'] + box['navigate']) / 2 + 0.2, ly=0.2)
fig.savefig(OUT + 'fig1_states.png', bbox_inches='tight', pad_inches=0.02)
plt.close(fig)

# ---------------- Figure 2: placeholder for the composite trajectory ----------------
fig, ax = plt.subplots(figsize=(7.0, 2.9), dpi=200)
ax.set_xlim(0, 10); ax.set_ylim(0, 4); ax.axis('off')
ax.add_patch(Rectangle((0.05, 0.05), 9.9, 3.9, fc='#f3f3f3', ec='#999999', lw=1, ls='--'))
ax.text(5, 2.35, 'FIGURE 2 PLACEHOLDER', ha='center', va='center', fontsize=12, weight='bold', color='#777777')
ax.text(5, 1.65, 'Composite of graded run [TBD]: frames ~1 s apart overlaid on one image,\n'
                 'markers ①–⑦ (trap escape, door, check, wait at wall, dash, 2nd pendulum, goal)',
        ha='center', va='center', fontsize=8.5, color='#777777')
fig.savefig(OUT + 'fig2_placeholder.png', bbox_inches='tight', pad_inches=0.02)
plt.close(fig)

# ---------------- Figure 3: lane shift ----------------
fig, ax = plt.subplots(figsize=(3.3, 2.75), dpi=300)
ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.17, 0.62); ax.set_aspect('equal'); ax.axis('off')
gap_c, gap_w, wall_y, wall_t = 0.10, 0.28, 0.42, 0.03
ax.add_patch(Rectangle((-0.45, wall_y), (gap_c - gap_w/2) + 0.45, wall_t, fc='#8a8a8a', ec='none'))
ax.add_patch(Rectangle((gap_c + gap_w/2, wall_y), 0.45 - (gap_c + gap_w/2), wall_t, fc='#8a8a8a', ec='none'))
ax.text(-0.43, wall_y + wall_t + 0.015, 'wall with door (0.28 m)', fontsize=6.3, color=MUTED, va='bottom')
L, W = 0.115, 0.10
# robot
ax.add_patch(Rectangle((-W, -L), 2*W, 2*L, fc='#dbe7f3', ec=INK, lw=0.9))
ax.annotate('', xy=(0, L + 0.05), xytext=(0, L - 0.01), arrowprops=dict(arrowstyle='-|>', lw=0.8, color=INK))
ax.text(0, -0.02, 'robot', ha='center', va='center', fontsize=6.5)
# straight sweep (blocked)
ax.add_patch(Rectangle((-W, L), 2*W, wall_y - L, fc='none', ec=ACC2, lw=0.8, ls='--'))
ax.text(-W - 0.01, 0.30, 'straight lane:\nhits the wall\nafter ~0.2 m', ha='right', va='center', fontsize=6.2, color=ACC2)
# shifted sweep (clear through the door)
s = 0.10
ax.add_patch(Rectangle((s - W, L), 2*W, 0.62 - L - 0.005, fc='#e8f2e8', ec='#2e7d32', lw=0.9, alpha=0.6))
ax.annotate('', xy=(s, -0.06), xytext=(0.0, -0.06), arrowprops=dict(arrowstyle='-|>', lw=0.9, color='#2e7d32'))
ax.text(s/2 + 0.01, -0.10, 'slide s = 10 cm', ha='center', va='top', fontsize=6.2, color='#2e7d32')
ax.text(s + W + 0.01, 0.25, 'shifted lane:\nclear through\nthe door', ha='left', va='center', fontsize=6.2, color='#2e7d32')
fig.savefig(OUT + 'fig3_lane.png', bbox_inches='tight', pad_inches=0.02)
plt.close(fig)
print('figures written')
