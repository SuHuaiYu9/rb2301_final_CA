# Section 2 prose rewritten in the user's style. Code blocks, Table 1, Figures 1 and 3 and their captions are untouched.
import re, zipfile
from xml.sax.saxutils import escape
SRC, OUT = 'base.odt', 'rb2301_final_project_report.odt'
zi = zipfile.ZipFile(SRC); x = zi.read('content.xml').decode()

def inline(t):                       # **bold**, `code`, *italic*
    out = []
    for part in re.split(r'(\*\*.+?\*\*|`.+?`|\*[^*]+?\*)', t):
        if part.startswith('**'): out.append('<text:span text:style-name="T4">%s</text:span>' % escape(part[2:-2]))
        elif part.startswith('`'): out.append('<text:span text:style-name="T6">%s</text:span>' % escape(part[1:-1]))
        elif part.startswith('*'): out.append('<text:span text:style-name="T14">%s</text:span>' % escape(part[1:-1]))
        else: out.append(escape(part))
    return ''.join(out)

def plain(el):
    t = re.sub(r'<text:s text:c="(\d+)"/>', lambda k: ' ' * int(k.group(1)), el)
    t = re.sub(r'<text:s/>', ' ', t); t = re.sub(r'<[^>]+>', '', t)
    return t.replace('&apos;', "'").replace('&quot;', '"').replace('&lt;', '<').replace('&gt;', '>').replace('&amp;', '&')

def replace_para(prefix, new_text):
    """Replace the one paragraph whose text starts with `prefix`, keeping its paragraph style."""
    global x
    hits = [m for m in re.finditer(r'<text:p text:style-name="([^"]+)">(.*?)</text:p>', x, re.S) if plain(m.group(2)).startswith(prefix)]
    assert len(hits) == 1, (prefix, len(hits))
    m = hits[0]
    x = x[:m.start()] + '<text:p text:style-name="%s">%s</text:p>' % (m.group(1), inline(new_text)) + x[m.end():]

replace_para('Everything is in one ROS 2 node',
  'All our code lives in one ROS 2 node, `obstacle_course_reactive.py`. On top of the given template (the '
  'subscriptions, pose callbacks and `move_2D`), it has four parts: a parameter block, with the shared values first and '
  'then the few that differ between Gazebo and the real robot; `timer_callback`, the 20 Hz loop below; the navigation '
  'helpers (`scan_to_points`, `free_distance`, `best_lane`, `all_lanes`, `best_direction`); and the pendulum helpers '
  '(`seen_past`, `in_front`, `pendulum_step` and two frame conversions).')
replace_para('Collision checking.',
  '**Collision checking.** `free_distance(points, θ)` tells us how far the robot can slide along a direction θ before it '
  'would touch any obstacle point. It does this by sweeping the robot\'s rectangle along θ in 1 cm steps, all checked at '
  'once with NumPy. The rectangle gets a 1 cm buffer on every side except the side the robot is moving away from, so it '
  'never gets stuck on a wall it is trying to leave. And since the robot never rotates, the swept rectangle is exact.')
replace_para('Lane shifts.',
  '**Lane shifts.** With 16 directions 22.5° apart and no turning, a gap almost never lines up perfectly with one of the '
  'robot\'s directions (the course is rarely that kind). So `best_lane` also tries starting from positions shifted '
  'sideways by up to ±12 cm, in 2 cm steps. A shift only counts if the robot can actually slide that far first, and it '
  'has to gain more room than it costs:')
replace_para('While driving, the robot slides onto the chosen lane',
  'While driving, the robot gradually slides onto the chosen lane and limits its forward speed to the free distance '
  'straight ahead, so it creeps past an edge instead of crashing into it (Figure 3).')
replace_para('Choosing a direction.',
  '**Choosing a direction.** A direction counts as feasible if its lane has at least 10 cm free (18 cm on the real '
  'robot, see Section 3.5). Every feasible direction θ then gets a score from `best_direction`, made up of four terms '
  '(Table 1):')
replace_para('Commitment and hysteresis.',
  '**Commitment and hysteresis.** Once a direction is chosen, the robot follows it for 10 cm before choosing again (or '
  'earlier if the path gets blocked), and it only scores all 16 directions at those moments. Even then, it only switches '
  'if the new best direction beats the current direction\'s score right now by more than 0.1:')
replace_para('Motion test.',
  '**Motion test.** `seen_past` compares the newest scan with one taken 0.5 s earlier, after placing both in the world '
  'frame using their poses. For each point that one scan hit, it looks at the ray of the other scan pointing at that same '
  'spot. If that ray and both of its neighbours reach more than 10 cm further, something has moved:')
replace_para('Run both ways, it finds things that left',
  'Running it both ways catches things that left as well as things that arrived. A static wall never passes the test, '
  'no matter how the robot moves, because both scans hit it.')
replace_para('State machine.',
  '**State machine.** The helper function “pendulum_step” runs before navigation every tick (Figure 1). When at least 10 '
  'moving points show up ahead, the nearest one becomes the pendulum\'s *line*. Navigation keeps driving until the '
  'straight path to a stopping spot 10 cm before the line is clear, which means the robot is through the door. The robot '
  'then stands still for a 2 s check, creeps up to the stopping spot and slides rightwards until it is about 6 cm from '
  'the right wall. There it waits for the pendulum to swing in front of it and away again. Since a pendulum is slowest at '
  'the end of its swing, this gives the longest gap before it comes back:')
replace_para('If the dash path is blocked',
  'If the dash path is blocked (for example by a bent wall), the robot edges away from the wall until it is clear. After '
  'the second crossing, it simply drives straight on until less than 5 cm is free in front: that is the goal.')

with zipfile.ZipFile(OUT, 'w') as zo:
    zo.writestr(zi.getinfo('mimetype'), zi.read('mimetype'), compress_type=zipfile.ZIP_STORED)
    for it in zi.infolist():
        if it.filename == 'mimetype': continue
        data = x.encode() if it.filename == 'content.xml' else zi.read(it.filename)
        zo.writestr(it, data, compress_type=zipfile.ZIP_STORED if it.filename.endswith(('.png', '.jpg')) else zipfile.ZIP_DEFLATED)
print('ok')
