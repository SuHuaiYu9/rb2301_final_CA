# Whole report body rewritten in the user's voice (ChatGPT's simplified version used as the tone reference).
# Reuses the cover, title, Table 1, Table 2, the lane-shift layout table and all figure frames from base.odt.
# Figures renumbered in order of appearance: 1 = lane shift, 2 = state machine, 3 = run-3 composite.
import re, zipfile
from xml.sax.saxutils import escape
SRC, OUT = 'base.odt', 'rb2301_final_project_report.odt'
zi = zipfile.ZipFile(SRC); x = zi.read('content.xml').decode()

def inline(t):
    out = []
    for part in re.split(r'(\*\*.+?\*\*|`.+?`|\*[^*]+?\*)', t):
        if part.startswith('**'): out.append('<text:span text:style-name="T4">%s</text:span>' % escape(part[2:-2]))
        elif part.startswith('`'): out.append('<text:span text:style-name="T6">%s</text:span>' % escape(part[1:-1]))
        elif part.startswith('*'): out.append('<text:span text:style-name="T14">%s</text:span>' % escape(part[1:-1]))
        else: out.append(escape(part))
    return ''.join(out)
def P(t, st='P10'): return '<text:p text:style-name="%s">%s</text:p>' % (st, inline(t))
def H1(t): return '<text:h text:style-name="Heading_20_1" text:outline-level="1">%s</text:h>' % escape(t)
def H2(t): return '<text:h text:style-name="Heading_20_2" text:outline-level="2">%s</text:h>' % escape(t)
def CAP(t, st='P8'): return '<text:p text:style-name="%s"><text:span text:style-name="T9">%s</text:span></text:p>' % (st, escape(t))
def LST(items, style='WWNum1', pst='P4'):
    return ('<text:list text:style-name="%s">' % style + ''.join(
        '<text:list-item><text:p text:style-name="%s">%s</text:p></text:list-item>' % (pst, inline(i)) for i in items) + '</text:list>')
def CODE(lines):
    def sp(l): return re.sub(r' {2,}', lambda m: '<text:s text:c="%d"/>' % len(m.group(0)), escape(l))
    st = ['P15'] if len(lines) == 1 else ['P12'] + ['P13'] * (len(lines) - 2) + ['P14']
    return ''.join('<text:p text:style-name="%s"><text:span text:style-name="T8">%s</text:span></text:p>' % (s, sp(l)) for s, l in zip(st, lines))

def grab_para(marker):                      # whole <text:p> containing marker
    i = x.index(marker); a = x.rfind('<text:p ', 0, i); b = x.index('</text:p>', i) + 9
    return x[a:b]
def grab_table(name):
    a = x.index('<table:table table:name="%s"' % name); b = x.index('</table:table>', a) + len('</table:table>')
    return x[a:b]

title_end = x.index('</text:p>', x.index('Map-free Reactive Navigation for a Holonomic Robot That Never Turns')) + 9
head, tail = x[:title_end], x[x.index('</office:text>'):]
fig_lane = re.search(r'<draw:frame [^>]*draw:name="Picture 1".*?</draw:frame>', x, re.S).group(0)
fig_sm = grab_para('draw:name="Picture 2"')
fig_run = grab_para('draw:name="Picture 3"')
table1, table2 = grab_table('Table2'), grab_table('Table3')
lane_tab = grab_table('Table1')
cell_a = re.search(r'(<table:table-cell table:style-name="Table1.A1"[^>]*>)(.*?)(</table:table-cell>)', lane_tab, re.S)
cells = list(re.finditer(r'(<table:table-cell [^>]*>)(.*?)(</table:table-cell>)', lane_tab, re.S))
assert len(cells) == 2
fig_lane_para = re.search(r'<text:p [^>]*>\s*' + re.escape(fig_lane) + r'\s*</text:p>', cells[1].group(2), re.S).group(0)

lane_left = ''.join([
  P('With only 16 directions, 22.5° apart, the robot\'s line of travel can easily miss a gap by a few centimetres. So '
    '`best_lane()` also tests lanes shifted sideways by up to ±12 cm, in 2 cm steps. It only accepts a shift if the robot '
    'can actually slide that far, and if the extra room it gains is worth more than the shift itself:'),
  CODE(['value = f - abs(s)      # room gained minus the shift']),
  P('Once a lane is chosen, the sideways speed is proportional to the remaining shift (up to a limit), so the robot eases '
    'onto the lane. Its forward speed is also limited to the clearance straight ahead, so it creeps past an edge instead '
    'of hitting it (Figure 1).')])
lane_right = fig_lane_para + CAP('Figure 1. The straight lane hits the wall; sliding 10 cm sideways lines the robot up with the door.')
new_lane_tab = (lane_tab[:cells[0].start()] + cells[0].group(1) + lane_left + cells[0].group(3)
                + lane_tab[cells[0].end():cells[1].start()] + cells[1].group(1) + lane_right + cells[1].group(3) + lane_tab[cells[1].end():])

body = ''.join([
  H1('1. Introduction'),
  P('The course had three parts: a zig-zag corridor with static obstacles, a narrow door, and two hand-released pendulums '
    '(aka the dynamic obstacles at the end) before the end wall. I built a **purely reactive** controller for it. It keeps '
    'no map, plans no path and is never told where the goal is, only the general direction to head in. Twenty times a '
    'second, it turns the latest LiDAR scan into obstacle points and decides where to move from those points alone.'),
  P('The robot is holonomic, so it **never turns to navigate**. It slides in any direction while holding the heading it '
    'started with, and only turns on the spot to correct drift. Because of that, its own frame stays lined up with the '
    'course, and every decision can be made in that frame. The OptiTrack pose is only used as a memory: where the robot '
    'has been, where a pendulum is, and how far a dash has gone.'),
  P('**All three graded runs on the real course were successful.** Runs 1 and 2 cleared the course comfortably. For run '
    '3, the cunning Professor placed cylinders to try to trap the robot before the door, but thankfully it still got '
    'through (Figure 3).'),

  H1('2. Code explanation'),
  H2('2.1 Organisation and control loop'),
  P('Everything is in one ROS 2 node, `obstacle_course_reactive.py`. The template provides the subscriptions, pose '
    'callbacks and `move_2D()`. On top of that, I added a parameter block (shared values first, then the few that differ '
    'between Gazebo and the real robot), the navigation helpers (`free_distance`, `best_lane`, `all_lanes`, '
    '`best_direction`) and the pendulum helpers (`seen_past`, `in_front`, `pendulum_step`). It all runs from '
    '`timer_callback()` at 20 Hz:'),
  CODE(['every tick (50 ms):',
        '    turn the latest LiDAR scan into (x, y) points in the robot frame',
        '    if the heading has drifted more than 6 deg: turn back on the spot (until within 2 deg)',
        '    if pendulum_step() is handling a pendulum: let it drive',
        '    if the robot has gone 10 cm in its current direction, or that direction is blocked:',
        '        score the best lane of each of the 16 directions',
        '        keep the current direction unless another one is clearly better',
        '    move along the chosen direction, shifting sideways onto its lane']),
  H2('2.2 Finding a clear lane'),
  P('`free_distance(points, θ)` tells me how far the robot can slide along a direction θ before it would touch an obstacle '
    'point. It sweeps the robot\'s rectangle along θ in 1 cm steps, and NumPy checks all the steps at once. I add a 1 cm '
    'clearance around the rectangle, except on the side the robot is moving away from, so it can always pull away from a '
    'wall it is close to. Since the robot never rotates, this sweep is exact.'),
  new_lane_tab,
  H2('2.3 Choosing a direction'),
  P('A direction is feasible if its best lane has at least 10 cm of clearance (18 cm on the real robot, see Section 3.5). '
    '`best_direction()` then scores every feasible direction with four terms (Table 1):'),
  CODE(['score = goal_score + proximity_score + room_score + novelty_score']),
  CAP('Table 1. The four terms of the direction score. Visited points are the robot\'s world positions, recorded every tick.', 'P16'),
  table1, '<text:p text:style-name="P17"/>',
  P('I let the robot follow its choice for 10 cm (or until the path is blocked) before scoring again. Even then, it keeps '
    'the current direction unless the best alternative beats the current direction\'s score right now by more than 0.1:'),
  CODE(['if not (self.selected_direction in self.feasible_directions',
        '        and best_score <= scores[self.selected_direction] + self.hysteresis):',
        '    self.selected_direction = best']),
  P('This hysteresis stops small changes in score from making the robot shuffle back and forth.'),
  H2('2.4 Detecting and crossing the pendulums'),
  P('`seen_past()` compares the latest LiDAR scan with one from half a second earlier, using the robot\'s pose to place '
    'both in the world frame. For each point that one scan hit, it checks the ray of the other scan pointing at that spot, '
    'plus its two neighbours. If all three see more than 10 cm past the point, the obstacle must have moved:'),
  CODE(['for k in (-1, 0, 1):          # the ray and both neighbours must see past the point',
        '    r = b[(idx + k) % n]',
        '    past &= np.isfinite(r) & (r > dist + margin)']),
  P('Comparing both ways finds things that left as well as things that arrived. A static wall never passes, however the '
    'robot moves, because both scans hit it.'),
  P('When at least 10 moving points appear ahead, `pendulum_step()` takes the nearest one as the pendulum\'s line (Figure '
    '2). Once the straight path to a stopping spot 10 cm before the line is clear, meaning the robot is through the door, '
    'it stops for 2 s to confirm the motion. It then creeps to the stopping spot and slides rightwards to about 6 cm from '
    'the right wall. Since a pendulum is slowest at the end of its swing, waiting there for it to swing in front of the '
    'robot and away again gives the longest gap before it comes back. The dash only starts if the LiDAR shows room for '
    'the whole crossing:'),
  CODE(['if node.seen or node.clock - node.mode_start >= node.t_wait:    # passed and left, or timeout',
        '    ahead = free_distance(node.occupied_points, 0.0, node.L, node.W)',
        '    if ahead >= dash:                                           # only if the whole dash is clear',
        "        node.mode, node.cross_start, node.cross_length = 'cross', node.pose[:2].copy(), dash"]),
  P('If the dash path is blocked, for example by a bent wall, the robot edges away from the wall until it is clear. After '
    'the second crossing, it drives straight on and stops when less than 5 cm is free: the goal.'),
  fig_sm,
  CAP('Figure 2. Pendulum state machine (pendulum_step). Orange arrows go back to navigation when a detection is not a pendulum.'),

  H1('3. Results and observations'),
  H2('3.1 The three graded runs'),
  P('All three graded runs reached the goal, and runs 1 and 2 cleared the course comfortably. For run 3, the one run to '
    'fool them all, cylinders were placed adversarially to try to trap the robot before the door. Figure 3 traces that '
    'run, which took 1 min 25 s.'),
  fig_run,
  CAP('Figure 3. Graded run 3; path traced from the video (approximate). ① start; ② probes past a cylinder, finds little '
      'room ahead and backs out; ③ a 2 s stand-still check rejects a suspected pendulum; ④ slides right along the '
      'corridor; ⑤ the goal pull leads into the dead-end pocket, where the proximity-score penalty builds; ⑥ leaves on a '
      'fresh lane through the gap between obstacles; ⑦ finds the door. Both pendulum crossings and the goal stop follow, '
      'out of frame.'),
  H2('3.2 Escaping the pocket and reaching the door'),
  P('In run 3, the goal score pulled the robot straight into the right-hand pocket of the middle corridor (Figure 3, ⑤), '
    'which is exactly what the cylinders were set up to do. The other three scores got it back out:'),
  LST(['**Room_score** favours directions with space ahead, so the open way out of a pocket looks good, and a dead end '
       'looks bad even before the robot is in it. That is why at ② it backed out of the small pocket beside the cylinder.',
       '**Novelty_score** drops towards zero for any direction into a place the robot has already visited, so going back '
       'into the pocket stops being attractive. At ⑥ it even left on a different lane from the one it came in on.',
       '**Proximity_score (a negative score)** keeps growing the longer the robot stays near the same spot, so it '
       'eventually leaves even when everything nearby has been visited and novelty no longer helps (⑤).']),
  P('Once out, **lane shifts** let the robot line up with the door from wherever it arrived, without having to turn (⑦). '
    'Table 2 shows how much they helped in our 2D tests.'),
  H2('3.3 Keeping motion steady'),
  P('Before hysteresis, the robot kept changing its mind between directions with similar scores, especially near walls, '
    'and shuffled back and forth. Committing to a direction for 10 cm and only switching for a clear improvement almost '
    'stopped these reversals in our 2D tests (Table 2). On the real course, the robot moved in long, steady segments, '
    'like along the corridor in Figure 3. Section 5 explains why the comparison has to use the current score.'),
  H2('3.4 Confirming motion on the real robot'),
  P('Spotting motion from a moving robot turned out to be one of the hardest parts. The LiDAR scan and the OptiTrack pose '
    'arrive at different times, so while the robot is moving, that mismatch can make a static edge look like it shifted '
    'between scans. Instead of one clever fix, I filter out false alarms in a few simple layers:'),
  LST(['the both-way test: a ray and both its neighbours must see past the point (Section 2.4);',
       'only a window in front of the robot (0.15–1.0 m ahead, ±1.2 m sideways) can raise an alarm;',
       'an alarm needs at least 10 moving points, since a real pendulum gives dozens;',
       'a 2 s stand-still check that needs 5 moving points at the line: with the robot stopped, the timing mismatch no '
       'longer matters;',
       'spots that turned out to be false alarms are remembered and ignored after that;',
       'a crossing only counts if motion was actually seen at the line, so a false alarm can never start the final '
       'drive to the goal.'], 'WWNum5', 'P5'),
  P('The robot detected and crossed both pendulums in all three graded runs. Run 3 had only two or three false alarms, '
    'and each one just cost a 2 s pause before navigation carried on (Figure 3, ③).'),
  H2('3.5 Running on the real hardware'),
  P('A few numbers in the code come from the hardware rather than from tuning:'),
  LST(['**LiDAR blind spot.** The LiDAR cannot see anything closer than 15 cm, and since it sits near the middle of the '
       'robot, that hides about 8 cm in front of the robot\'s nose. So the robot waits 10 cm before a pendulum line, '
       'stops when less than 5 cm is free at the end wall, and on hardware asks for an extra 8 cm of clearance '
       '(`wall_gap`) in the direction it drives, so obstacles stay visible.',
       '**Heading-hold band.** The heading slowly drifts as the robot slides, and it is never placed perfectly straight. '
       'A slightly tilted robot dashing "straight" along a wall slowly runs into it, so the robot turns back to its '
       'starting heading on the spot, at 0.2 rad/s. With roughly 0.15 s of delay, each correction overshoots by about '
       '2°, so correction starts at 6° and stops at 2°. The band is wider than the overshoot, so the robot settles '
       'instead of wobbling.',
       '**Detection thresholds.** 10 moving points to raise an alarm and 5 to confirm one were set by experiment: a real '
       'pendulum gives dozens of moving points, noise only a handful.']),
  P('**Control loop.** Scoring every shifted lane in all 16 directions on every tick dragged the Jetson Nano down to '
    '5–6 Hz. I moved that work to the moments when the robot actually chooses a direction, and in between it only checks '
    'the current one. That brought it back to about 20 Hz without changing how it behaves. The timers use the real '
    'clock, so waiting times stay right even if a tick runs long.'),
  H2('3.6 Development tests'),
  P('Between lab sessions, I tested changes in a custom 2D simulator of the course, built with Claude. It runs the '
    'controller on randomised layouts and can add LiDAR noise and scan delay to mimic the real robot. Table 2 sums up '
    'the main results.'),
  CAP('Table 2. Development results from our 2D simulator (21 random layouts unless stated).', 'P16'),
  table2, '<text:p text:style-name="P17"/>',

  H1('4. Limitations and future work'),
  P('**Timing the dash.** Waiting for the pendulum to swing away gives the robot the longest gap it can get, but the '
    'margin is still thin: the pendulum comes back in roughly the time the robot needs to fully clear its path. It worked '
    'in all three graded runs, but a faster or wider swing could catch the robot mid-dash. With more time, I would use '
    'the 2 s check to estimate the swing\'s period and phase, then leave right at the start of the biggest gap, or skip '
    'a swing if the gap is too short.'),
  P('**Matching scans to poses.** Each scan is paired with the latest pose, not the pose from the moment it was taken, '
    'which is why the robot has to stop to confirm a detection. Matching every scan to the pose at its own timestamp '
    'could let it detect pendulums while still moving, and shorten or even skip the stop.'),
  P('**Always waiting on the right.** If a pendulum only swings over the right side, it never fully leaves the robot\'s '
    'lane, and the robot ends up crossing on the timeout. The motion seen during the 2 s check could tell it which side '
    'the pendulum swings over less, so it could wait there instead.'),
  P('**Fixed heading.** The no-turn design assumes the course never requires the robot to rotate, this prevents the robot '
    'from navigating very tight gaps, especially since it is longer than it is wide. Where there is room, the robot could '
    'rotate to put its narrower side towards a tight gap.'),
  P('**Other things I would try.** A P-controller for the heading hold, turning harder for big errors and gently for '
    'small ones, instead of one fixed turning speed. And tuning the score weights automatically over many random '
    'layouts, rather than by hand.'),

  H1('5. Reflections and lessons learned'),
  P('**Simple is robust.** My very first design maintained a global occupancy grid and planned A* paths through it. The '
    'map needed constant updating, noisy returns became phantom obstacles, and the pendulums smeared across it. So when '
    'the prof mentioned that the course could be done reactively, I scrapped the map and started over. With the reactive '
    'controller, I could look at a scan, see which directions the robot had, and explain every choice it made. '
    'Recording where it had been gave it just enough memory to get out of pockets, without a map.'),
  P('**The stale score bug.** Hysteresis took me a few tries. My first attempt had a small bug that kept resetting the '
    'best score, so the robot happily drove backwards everywhere. The next version compared new directions against the '
    'score the current direction had back when it was chosen, but then the robot kept driving towards walls until it was '
    'completely blocked, and bounced back. Turns out, that saved score was stale: as the robot gets closer to a wall, the '
    'room in that direction shrinks, so its real score drops while the saved one stays high. The fix was a one-line '
    'change (simplified):'),
  CODE(['if best_score > self.last_best_score + self.hysteresis:              # before: saved score',
        'if best_score > scores[self.selected_direction] + self.hysteresis:  # after: score right now']),
  P('Lesson learnt: anything a reactive controller saves from the past can quietly go out of date, so it is safer to '
    'recompute it from what the robot sees right now.'),
  P('**Measure before patching.** On my first hardware run, the robot circled and kept raising false alarms, even though '
    'it worked perfectly on Gazebo. I immediately started questioning reality, and frantically tuned parameters and added '
    'new logic before even understanding what was wrong. When I finally checked what the robot thought its heading was, '
    'OptiTrack was about 175° off, which reversed every world-frame calculation. Correcting that offset fixed both '
    'problems. The bigger lesson for me was to have confidence in the logic of my algorithm and debug calmly. Next time, '
    'I will check the pose and coordinate frames before tuning anything: a short forward-and-sideways move on the lab '
    'floor would have shown me straight away whether the reported movement matched what I could see.'),
  P('**Test on the real robot early.** Most of the hardware problems in Section 3.5, like the slow Jetson and the '
    'LiDAR\'s blind spot, never showed up on Gazebo. Our own code only got onto the real robot about a week before the '
    'graded run, so they all had to be fixed in a rush. Next time, I would test even a half-finished version on the real '
    'robot as early as possible.'),
])
x = head + body + tail

with zipfile.ZipFile(OUT, 'w') as zo:
    zo.writestr(zi.getinfo('mimetype'), zi.read('mimetype'), compress_type=zipfile.ZIP_STORED)
    for it in zi.infolist():
        if it.filename == 'mimetype': continue
        data = x.encode() if it.filename == 'content.xml' else zi.read(it.filename)
        zo.writestr(it, data, compress_type=zipfile.ZIP_STORED if it.filename.endswith(('.png', '.jpg')) else zipfile.ZIP_DEFLATED)
print('ok')
