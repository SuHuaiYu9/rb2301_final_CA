# New sections 4-5 (limitations, future work, reflections) in the user's voice + trims in section 3. Works on a copy.
import re, zipfile
from xml.sax.saxutils import escape
SRC, OUT = 'base.odt', 'rb2301_final_project_report.odt'
zi = zipfile.ZipFile(SRC); x = zi.read('content.xml').decode()

def inline(t):                       # **bold** -> T4 span; text escaped
    out = []
    for part in re.split(r'(\*\*.+?\*\*)', t):
        if part.startswith('**'): out.append('<text:span text:style-name="T4">%s</text:span>' % escape(part[2:-2]))
        else: out.append(escape(part))
    return ''.join(out)
def P(t): return '<text:p text:style-name="P10">%s</text:p>' % inline(t)
def H1(t): return '<text:h text:style-name="Heading_20_1" text:outline-level="1">%s</text:h>' % escape(t)
def H2(t): return '<text:h text:style-name="Heading_20_2" text:outline-level="2">%s</text:h>' % escape(t)
def bullets(items):
    return ('<text:list text:style-name="WWNum1">' + ''.join(
        '<text:list-item><text:p text:style-name="P4">%s</text:p></text:list-item>' % inline(i) for i in items) + '</text:list>')
def code(lines):
    def sp(l): return re.sub(r' {2,}', lambda m: '<text:s text:c="%d"/>' % len(m.group(0)), escape(l))
    st = ['P12'] + ['P13'] * (len(lines) - 2) + ['P14'] if len(lines) > 1 else ['P15']
    return ''.join('<text:p text:style-name="%s"><text:span text:style-name="T8">%s</text:span></text:p>' % (s, sp(l)) for s, l in zip(st, lines))

def one(old, new):
    global x
    assert x.count(old) == 1, (x.count(old), old[:70])
    x = x.replace(old, new)

# ---------------- sections 4 and 5
a = x.index('<text:h text:style-name="Heading_20_1" text:outline-level="1">4. Limitations and future work</text:h>')
b = x.index('</office:text>')
measure = re.search(r'<text:p text:style-name="P11"><text:span text:style-name="T4">Measure before patching\.</text:span>.*?</text:p>', x[a:b]).group(0)

new = ''.join([
  H1('4. Limitations and future work'),
  H2('4.1 Limitations'),
  P('**Pendulum timing.** Waiting for the pendulum to swing away gives the robot the longest gap it can get, but the '
    'pendulum still comes back in about the time the robot needs to fully clear it. It worked in all three graded runs, '
    'but a faster or wider swing could catch the robot mid-dash.'),
  P('**Scans and poses are not matched in time.** Each scan is paired with the latest pose, not the pose from when the '
    'scan was taken, so while the robot moves, static things look like they moved. That is why it has to stop for the '
    '2 s check before trusting a detection.'),
  P('**Always waiting on the right.** If a pendulum only swings over the right side, it never fully leaves the robot\'s '
    'lane, so the robot ends up crossing on the timeout without knowing where the pendulum is.'),
  P('**Other limitations.** The no-turn design assumes the course never requires the robot to rotate, this prevents the '
    'robot from navigating very tight gaps, especially since it is longer than it is wide.'),
  H2('4.2 With more time'),
  bullets([
    '**Time the dash properly.** The robot already watches the pendulum during the 2 s check, so it could estimate the '
    'swing period and leave right at the start of the biggest gap, or skip a swing if the gap looks too short.',
    '**Match each scan to the right pose.** Using the pose from each scan\'s own timestamp would let the robot detect '
    'pendulums while moving, without stopping to check.',
    '**Choose the better side to wait on.** The 2 s check also shows which side the pendulum swings over more, so the '
    'robot could wait on the quieter side.',
    '**A smoother heading hold.** Right now the robot turns at one fixed speed whenever it drifts too far. A P-controller '
    'that turns harder for big errors and gently for small ones would be smoother, and could even correct while driving.',
    '**Tune the weights automatically.** The score weights were tuned by hand; a simple search over lots of random '
    'layouts could find better ones.']),
  H1('5. Reflections and lessons learned'),
  P('**Simple is robust.** Our very first algorithm design maintained a global occupancy grid and planned A* paths through '
    'it. The map needed constant updating, noisy returns became phantom obstacles, the pendulums smeared across it, and '
    'it was hard to tell why the robot did anything. So when the prof mentioned that the course could be done '
    'reactively, I scrapped the map and started over. The reactive controller is simpler '
    'and easier to debug: every decision is local, explainable from the current scan, and uses only a handful of states. '
    'On the real course it was far more robust.'),
  P('**The stale score bug.** Hysteresis took me a few tries. My very first attempt had a small bug that kept resetting '
    'the best score, so the robot happily drove backwards everywhere. The next version compared the best new direction '
    'against the score the current direction had back when it was chosen, but then the robot kept driving towards walls '
    'until it was completely blocked, and bounced back. Turns out, that saved score was stale: as the robot gets closer '
    'to a wall, the room in that direction shrinks, so its real score drops while the saved one stays high. The fix was '
    'a one-line change (simplified):'),
  code(['if best_score > self.last_best_score + self.hysteresis:              # before: saved score',
        'if best_score > scores[self.selected_direction] + self.hysteresis:  # after: score right now']),
  P('Lesson learnt: anything a reactive controller saves from the past can quietly go out of date, so it is safer to '
    'recompute it from what the robot sees right now.'),
  measure,
  P('**Gazebo is not the real robot.** Most of the hardware problems in Section 3.5, like the slow Jetson and the '
    'LiDAR\'s blind spot, never showed up on Gazebo. We only got our own code onto the real robot a few days before the '
    'graded run, so they all had to be fixed in a rush. Next time, I would test even a half-finished version on the real '
    'robot as early as possible.'),
])
x = x[:a] + new + x[b:]

# ---------------- trims in section 3: no 2D-simulator numbers in the prose, no simulator table
one(' In development tests <text:soft-page-break/>(Table 2), adding novelty cut the slowest run from 57 s to 38 s, and lane '
    'shifts cut the time through a 0.28 m gap, where the robot&apos;s cent<text:span text:style-name="T17">er</text:span> has '
    'only a 9 cm band, from 62 s to 21 s.', '')
one('Direction reversals (turns of 135° or more) fell from <text:span text:style-name="T17">about 4</text:span> to 0.<text:span '
    'text:style-name="T17">5</text:span> per run in tests, and on the real course the robot moves in long, smooth segments.',
    'In our tests the robot almost stopped reversing direction, and on the real course it moves in long, smooth segments.')
one('With 100 ms of scan delay in our <text:span text:style-name="T20">private simulated</text:span> tests, the stand-still '
    'check alone raised successful runs from 17/21 to 21/21. On the real course', 'On the real course')
t0 = x.index('<text:p text:style-name="P16"><text:soft-page-break/><text:span text:style-name="T9">Table 2. Effect of each feature')
t1 = x.index('</table:table>', t0) + len('</table:table>')
E = '<text:p text:style-name="P17"/>'
assert x[t1:t1 + len(E)] == E
x = x[:t0] + x[t1 + len(E):]


one('only a window in front of the robot (0.15–1.0 m ahead, ±1.2 m sideways), so nothing beside it or outside the corridor ahead can trigger it;',
    'only a window in front of the robot (0.15–1.0 m ahead, ±1.2 m sideways) can trigger it;')
one('at least 10 moving points to raise an alarm, since a real pendulum within 1 m produces dozens;',
    'at least 10 moving points to raise an alarm (a real pendulum gives dozens);')
one('The LiDAR is 4.5 cm ahead of the centre and the robot&apos;s half-length is 11.5 cm, so anything within about 8 cm of its front is invisible.',
    'Since the LiDAR sits near the robot&apos;s centre, anything within about 8 cm of its front is invisible.')
one('draw:name="Picture 3" text:anchor-type="as-char" svg:y="-3.0709in" svg:width="6.2992in" svg:height="3.0862in"',
    'draw:name="Picture 3" text:anchor-type="as-char" svg:y="-2.6873in" svg:width="5.5118in" svg:height="2.7004in"')
one('Figure 1. Pendulum state machine (pendulum_step). Orange arrows return to navigation when a detection turns out not to be a pendulum.',
    'Figure 1. Pendulum state machine (pendulum_step). Orange arrows go back to navigation when a detection is not a pendulum.')
one('restored about 20 Hz with identical behaviour; the timers use real time, so they stay correct if a tick runs long.',
    'restored about 20 Hz with identical behaviour.')

with zipfile.ZipFile(OUT, 'w') as zo:
    zo.writestr(zi.getinfo('mimetype'), zi.read('mimetype'), compress_type=zipfile.ZIP_STORED)
    for it in zi.infolist():
        if it.filename == 'mimetype': continue
        data = x.encode() if it.filename == 'content.xml' else zi.read(it.filename)
        zo.writestr(it, data, compress_type=zipfile.ZIP_STORED if it.filename.endswith(('.png', '.jpg')) else zipfile.ZIP_DEFLATED)
print('ok')
