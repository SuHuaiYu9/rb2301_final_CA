# Section 3 (3.2-3.5) rewritten in the user's style; 3.1, Figure 2, its caption and Table 2 untouched. Runs on edit_refl.py's output.
import re, zipfile
from xml.sax.saxutils import escape
SRC, OUT = 'rb2301_final_project_report.odt', 'rb2301_final_project_report_s3.odt'
zi = zipfile.ZipFile(SRC); x = zi.read('content.xml').decode()

def inline(t):                       # **bold**, `code`
    out = []
    for part in re.split(r'(\*\*.+?\*\*|`.+?`)', t):
        if part.startswith('**'): out.append('<text:span text:style-name="T4">%s</text:span>' % escape(part[2:-2]))
        elif part.startswith('`'): out.append('<text:span text:style-name="T6">%s</text:span>' % escape(part[1:-1]))
        else: out.append(escape(part))
    return ''.join(out)
def P(t): return '<text:p text:style-name="P10">%s</text:p>' % inline(t)
def H2(t): return '<text:h text:style-name="Heading_20_2" text:outline-level="2">%s</text:h>' % escape(t)
def lst(items, style, pstyle):
    return ('<text:list text:style-name="%s">' % style + ''.join(
        '<text:list-item><text:p text:style-name="%s">%s</text:p></text:list-item>' % (pstyle, inline(i)) for i in items) + '</text:list>')

new = ''.join([
  H2('3.2 Escaping the trap and finding the door'),
  P('Run 3 was set up to exploit a weakness of goal-seeking robots. Our goal direction points forward-right, so the robot '
    'naturally gets pulled into the dead-end pocket on the right of the middle corridor, which is exactly what happened at '
    '⑤. The score has three parts that work together to get it back out:'),
  lst(['**Room_score** rewards directions with lots of free space, so the open way out of a pocket looks good, and a dead '
       'end looks bad even before the robot is in it. That is why at ② it backed out of the small pocket beside the cylinder.',
       '**Novelty_score** drops to zero for every direction into a pocket the robot has already visited, so it does not '
       'bother going back in. At ⑥ it even left on a different lane from the one it came in on.',
       '**Proximity_score (a negative score)** keeps growing the longer the robot stays in one area, so even when '
       'everything nearby has been visited and novelty stops helping, the robot eventually gets fed up and leaves (⑤).'],
      'WWNum1', 'P4'),
  P('Once out, **lane shifts** let the robot line up with the door from wherever it ends up, instead of having to arrive '
    'perfectly aligned, which is the main headache for a robot that never turns (⑦).'),
  H2('3.3 Hysteresis: no more flip-flopping'),
  P('Without hysteresis, the robot could not make up its mind. Near walls, a few directions often had almost the same '
    'score, so the robot kept switching between them and ended up shuffling back and forth. Two simple rules fixed it: '
    'the robot commits to a direction for 10 cm before choosing again, and it only switches if another direction beats '
    'the current one\'s score by a margin (Section 5 explains why this has to be the current score). In our tests the '
    'robot almost stopped reversing direction (Table 2), and on the real course it moved in long, smooth segments.'),
  H2('3.4 Telling real pendulums from false alarms'),
  P('Spotting motion from a robot that is itself moving turned out to be one of the hardest parts of the project. The '
    'LiDAR scan and the Optitrack pose arrive at slightly different times, so whenever the robot speeds up, slows down or '
    'changes direction, perfectly still walls seem to shift a little and look like they moved. Instead of one clever fix, '
    'our detector filters out false alarms in several simple layers:'),
  lst(['both-way test: the ray **and both its neighbours** (explained in 2.3) have to see past the point, which filters '
       'out noise and flickering edges;',
       'only a window in front of the robot (0.15–1.0 m ahead, ±1.2 m sideways) can trigger it;',
       'at least 10 moving points are needed to raise an alarm, since a real pendulum gives dozens;',
       'a 2 s stand-still check that needs 5 moving points at the line: with the robot standing still, the timing errors '
       'disappear and only real motion is left;',
       'spots that turned out to be false alarms are remembered and never trigger again;',
       'a crossing only counts if motion was actually seen at the line, so a false alarm can never send the robot on its '
       'final drive to the goal.'], 'WWNum5', 'P5'),
  P('On the real course, every pendulum in the graded runs was detected, confirmed and crossed. Run 3 took 1 min 25 s, '
    'and in that whole run the robot only had two or three false alarms, each costing just a 2 s pause (③).'),
  H2('3.5 Making it work on the real hardware'),
  P('A few numbers in our code come from the hardware itself rather than from tuning:'),
  lst(['**LiDAR blind spot.** The LiDAR cannot see anything closer than 15 cm, and since it sits near the middle of the '
       'robot, anything within about 8 cm of the robot\'s front is invisible. So the robot waits 10 cm before a pendulum '
       'line, stops when only 5 cm is free in front of the end wall, and keeps an extra 8 cm of free space (`wall_gap`) '
       'in whatever direction it drives, so obstacles never sneak into the blind spot.',
       '**Heading-hold band.** The robot\'s heading slowly drifts as it slides around, and it is never placed perfectly '
       'straight at the start either. A slightly tilted robot dashing "straight" along a wall slowly runs into it, so the '
       'robot holds its starting heading by turning on the spot at 0.2 rad/s. With roughly 0.15 s of delay, each '
       'correction overshoots by about 2°, so the robot starts correcting at 6° and stops at 2°. This band is wider '
       'than the overshoot, so it never wobbles back and forth.',
       '**Detection thresholds.** The 10 points needed to raise an alarm and the 5 needed to confirm one were set by '
       'experiment: a real pendulum gives dozens of moving points, while noise only gives a handful.'], 'WWNum1', 'P4'),
  P('The biggest surprise was the Jetson Nano. Our loop ran at only 5–6 Hz on it, because every tick it scored all 16 '
    'directions along with all their shifted lanes. Our laptops never noticed, but the Jetson definitely did. Scoring '
    'every direction only when a new one has to be chosen, and just the current one in between, brought it back to about '
    '20 Hz without changing how the robot behaves.'),
])

i = x.index('<text:h text:style-name="Heading_20_2" text:outline-level="2">3.2 Always finding the door, even when trapped</text:h>')
j = x.index('Table 2. Effect of each feature'); j = x.rfind('<text:p ', 0, j)
x = x[:i] + new + x[j:]

with zipfile.ZipFile(OUT, 'w') as zo:
    zo.writestr(zi.getinfo('mimetype'), zi.read('mimetype'), compress_type=zipfile.ZIP_STORED)
    for it in zi.infolist():
        if it.filename == 'mimetype': continue
        data = x.encode() if it.filename == 'content.xml' else zi.read(it.filename)
        zo.writestr(it, data, compress_type=zipfile.ZIP_STORED if it.filename.endswith(('.png', '.jpg')) else zipfile.ZIP_DEFLATED)
print('ok')
