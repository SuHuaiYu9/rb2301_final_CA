import re
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_COLOR_INDEX
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

D = '/tmp/claude-1000/-home-jacky-Downloads-rb2301-final-project-main/046b2aa8-8f72-439a-aad2-61e73ef2cc4b/scratchpad/teamlog/'
BODY, CODE = 'Calibri', 'Consolas'

doc = Document()
sec = doc.sections[0]
sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
for side in ('left_margin', 'right_margin'):
    setattr(sec, side, Cm(2.0))
sec.top_margin, sec.bottom_margin = Cm(1.8), Cm(1.8)

fp = sec.footer.paragraphs[0]; fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
for kind, txt in (('begin', None), (None, 'PAGE'), ('end', None)):
    r = fp.add_run()
    if kind:
        fc = OxmlElement('w:fldChar'); fc.set(qn('w:fldCharType'), kind); r._r.append(fc)
    else:
        it = OxmlElement('w:instrText'); it.set(qn('xml:space'), 'preserve'); it.text = txt; r._r.append(it)
    r.font.size = Pt(9)
st = doc.styles['Normal']
st.font.name, st.font.size = BODY, Pt(10.5)
st.element.rPr.rFonts.set(qn('w:eastAsia'), BODY)
st.paragraph_format.space_after = Pt(3)
st.paragraph_format.space_before = Pt(0)
st.paragraph_format.line_spacing = 1.0
for name, size, before, after in (('Heading 1', 12.5, 8, 3), ('Heading 2', 11, 5, 2), ('Title', 16, 0, 2)):
    s = doc.styles[name]
    s.font.name, s.font.size, s.font.bold = BODY, Pt(size), True
    s.font.color.rgb = RGBColor(0x1F, 0x3B, 0x5C) if name != 'Title' else RGBColor(0x11, 0x11, 0x11)
    rpr = s.element.get_or_add_rPr()
    fonts = rpr.find(qn('w:rFonts'))
    if fonts is None:
        fonts = OxmlElement('w:rFonts'); rpr.append(fonts)
    for a in ('w:ascii', 'w:hAnsi', 'w:eastAsia', 'w:cs'):
        fonts.set(qn(a), BODY)
    s.paragraph_format.space_before, s.paragraph_format.space_after = Pt(before), Pt(after)
    s.paragraph_format.keep_with_next = True

TOKEN = re.compile(r'(\*\*.+?\*\*|`.+?`|\[(?:TBD|VET)[^\]]*\]|\*[^*]+?\*)')

def runs(p, text, size=None):
    for part in TOKEN.split(text):
        if not part:
            continue
        if part.startswith('**'):
            r = p.add_run(part[2:-2]); r.bold = True
        elif part.startswith('`'):
            r = p.add_run(part[1:-1]); r.font.name = CODE; r.font.size = Pt((size or 10.5) - 1)
        elif part.startswith('[TBD') or part.startswith('[VET'):
            r = p.add_run(part); r.font.highlight_color = WD_COLOR_INDEX.YELLOW
        elif part.startswith('*'):
            r = p.add_run(part[1:-1]); r.italic = True
        else:
            r = p.add_run(part)
        if size and not part.startswith('`'):
            r.font.size = Pt(size)
    return p

def para(text, size=None, align=None, after=None):
    p = doc.add_paragraph()
    runs(p, text, size)
    p.paragraph_format.alignment = align if align is not None else WD_ALIGN_PARAGRAPH.JUSTIFY
    if after is not None:
        p.paragraph_format.space_after = Pt(after)
    return p

def h1(t): doc.add_heading(t, level=1)
def h2(t): doc.add_heading(t, level=2)

def shade(el, fill):
    pPr = el.get_or_add_pPr() if hasattr(el, 'get_or_add_pPr') else el
    shd = OxmlElement('w:shd'); shd.set(qn('w:val'), 'clear'); shd.set(qn('w:color'), 'auto'); shd.set(qn('w:fill'), fill)
    pPr.append(shd)

def code(lines, container=None):
    for i, line in enumerate(lines):
        p = (container or doc).add_paragraph()
        r = p.add_run(line if line else ' ')
        r.font.name, r.font.size = CODE, Pt(8.3)
        r._element.rPr.rFonts.set(qn('w:eastAsia'), CODE)
        pf = p.paragraph_format
        pf.space_after = Pt(0) if i < len(lines) - 1 else Pt(4)
        pf.space_before = Pt(2) if i == 0 else Pt(0)
        pf.left_indent = Cm(0.3)
        pf.alignment = WD_ALIGN_PARAGRAPH.LEFT
        shade(p._p, 'F2F4F7')

def bullets(items, style='List Bullet', size=None):
    for it in items:
        p = doc.add_paragraph(style=style)
        runs(p, it, size)
        p.paragraph_format.space_after = Pt(1)
def caption(text, container=None, align=WD_ALIGN_PARAGRAPH.CENTER):
    p = (container or doc).add_paragraph()
    runs(p, text, size=9)
    for r in p.runs:
        r.italic = True
    p.paragraph_format.alignment = align
    p.paragraph_format.space_after = Pt(3)
    return p

def cell_shade(cell, fill):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd'); shd.set(qn('w:val'), 'clear'); shd.set(qn('w:color'), 'auto'); shd.set(qn('w:fill'), fill)
    tcPr.append(shd)

def table(header, rows, widths_cm, cap, size=9, keep=False):
    caption(cap, align=WD_ALIGN_PARAGRAPH.LEFT).paragraph_format.keep_with_next = True
    t = doc.add_table(rows=1 + len(rows), cols=len(header))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    for j, wcm in enumerate(widths_cm):
        t.columns[j].width = Cm(wcm)
    for i, row in enumerate([header] + rows):
        trPr = t.rows[i]._tr.get_or_add_trPr()
        cs = OxmlElement('w:cantSplit'); trPr.append(cs)
        if i == 0:
            trPr.append(OxmlElement('w:tblHeader'))
        for j, txt in enumerate(row):
            c = t.cell(i, j)
            c.width = Cm(widths_cm[j])
            c.paragraphs[0].text = ''
            runs(c.paragraphs[0], txt, size=size)
            c.paragraphs[0].paragraph_format.space_after = Pt(0)
            c.paragraphs[0].paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
            if keep and i < len(rows):
                c.paragraphs[0].paragraph_format.keep_with_next = True   # keep the table on one page
            if i == 0:
                for r in c.paragraphs[0].runs:
                    r.bold = True
                cell_shade(c, 'E7EDF4')
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t

# ------------------------------------------------------------------ content
TW = [1.5, 4.3, 6.6, 4.6]          # table column widths (cm), 17 cm in total
def log_table(cap, rows):
    table(['Date', 'Issue', 'What we changed', 'Outcome'], rows, TW, cap, size=8.5)

t = doc.add_paragraph(style='Title'); t.add_run('obstacle_course_reactive.py: development log summary')
para('Group 5 · RB2301 final project · testing from 5 to 13 Nov 2026 (Gazebo, a 2D test simulator, and the real '
     'Bingda robot). Final code: commit `6aadf26`; the submitted file is a tidied copy with identical behaviour.',
     size=9.5, align=WD_ALIGN_PARAGRAPH.LEFT, after=6)

h1('1. How the final controller works (in brief)')
bullets([
    'No map, no path planning and no goal coordinates. Every 50 ms the latest LiDAR scan is converted into obstacle '
    'points in the robot frame, and the decision is made from those points.',
    'The robot keeps the heading it starts with and moves sideways or diagonally (mecanum wheels) along one of '
    '16 directions spaced 22.5° apart.',
    'Each direction gets a weighted score: a **goal** term (pull towards front-right, 315°), a **room** term (how far it '
    'can travel, capped at 1 m), a **novelty** term (prefers ground not yet visited, judged up to 0.4 m ahead) and a '
    '**proximity** penalty (grows the longer the robot stays near the same spot).',
    'A new direction is chosen every 10 cm of travel or as soon as the current one is blocked, and the robot only '
    'switches if another direction scores more than 0.1 higher.',
    'Pendulums are handled by a small state machine: navigate → check → position → wait → cross → find (second '
    'pendulum) → goal → done.'], size=9.5)

h1('2. Background: dropping the occupancy grid (5 Nov)')
bullets([
    'The earlier grid + A* version kept flagging walls as moving obstacles: as the robot drove, cells near a wall '
    'flipped between occupied and free because the LiDAR saw the surface from a slightly different angle each time.',
    'With 1 cm of simulated LiDAR noise, a robot standing still in front of a wall already saw 23–26 grid cells '
    'change between checks, so the method looked fragile for the real robot.',
    'The prof also preferred a reactive solution with nothing hard-coded, so the controller was rewritten from the '
    'template as a purely reactive node.'], size=9.5)

h1('3. Simulation phase (6–9 Nov)')
para('"2D" results come from a simple 2D simulator of the course with 21 random obstacle layouts, unless stated '
     'otherwise; Gazebo was used to check behaviour visually.', size=9.5)
log_table('Table 1. Changes made while testing in simulation.', [
    ['6 Nov', 'Robot froze in a corner ("nowhere to go") although its back and left were open.',
     'The collision sweep enlarged the robot box on all four sides, so it overlapped walls the robot was moving away '
     'from. The box now only grows towards the direction of motion.',
     'No more deadlocks; 21/21 layouts completed in 2D.'],
    ['6–7 Nov', 'Direction kept flipping between options with similar scores (back-and-forth motion).',
     'Hysteresis: commit to a direction for 10 cm, and switch only if a rival beats the current direction\'s '
     'up-to-date score by a margin (a score saved when the direction was chosen stays too high as a wall gets closer).',
     'Reversals of 135° or more: 3.8 → 0.4 per run; slowest run 57 s → 42 s.'],
    ['7 Nov', 'Long "bouncing" in dead ends and corners.',
     'Added the room term (free distance along each direction, up to 1 m).',
     'Helped, but the robot still re-entered corners.'],
    ['7 Nov', 'Still returned to the top-right corner more than once.',
     'Added the novelty term alongside the proximity penalty. Alternatives tested: a heavier proximity penalty only '
     '(slower, and a very heavy one trapped the robot: 15/20 at weight 3.0); novelty only (17–19/21, robot rocked '
     'between two spots).',
     'Both together: 21/21, slowest run 38 s.'],
    ['7 Nov', 'Door entry not smooth in Gazebo.',
     'Goal direction tried at 337.5° instead of 315°.',
     'Smoother in Gazebo; later set back to 315° after tuning on the real course.'],
    ['8 Nov', 'A 0.28 m gap took a long time to find: the robot\'s centre has to be inside a band about 9 cm wide.',
     'Lane shifts: every direction is also tested from lanes offset sideways by up to ±12 cm (2 cm steps); a shift '
     'is worth taking only if the extra room it gives exceeds its size.',
     'Time to pass the gap: 62 s → 21 s (2D copy of the Gazebo map).'],
    ['8 Nov', 'Some commands were very slow (risk of motor deadband).',
     'Minimum speed of 0.075 m/s on the real robot, keeping the direction.', '–'],
    ['8 Nov', 'Heavy loop: 14–23 ms per tick on a laptop.',
     '`free_distance` rewritten with NumPy so all 1 cm steps are checked at once.',
     'About 5 ms per tick, identical results.'],
    ['9 Nov', 'Robot grazed a thin can while sliding sideways.',
     '1 cm clearance on every side of the swept box except the side it moves away from.',
     'No contacts in 21 layouts; very tight gaps get harder with a noisy LiDAR.'],
    ['9 Nov', 'Pendulum handling needed.',
     'Detect motion by comparing scans 0.5 s apart (a point counts only if its ray and both neighbours see at least '
     '10 cm past it); creep to 10 cm before the line, slide to the right wall, wait until the pendulum has passed in '
     'front and left (or a timeout: 10 s sim, 4 s real), then dash until the rear is 10 cm past the line.',
     '252 randomised 2D runs (various swing sizes, LiDAR noise): all reached the end wall, no obstacle hits.'],
    ['9 Nov', 'Gazebo cans sometimes detected as pendulums.',
     'Cause: each scan is paired with a pose up to 100 ms newer, so static edges appear to move while driving. Added '
     'a 2 s stand-still check that needs at least 5 moving points; rejected spots are remembered.',
     'With 100 ms lag in 2D: runs without pendulums 17/21 → 21/21.'],
    ['9 Nov', 'Heading drifted about 5.5° in Gazebo; the tilted dash path hit the wall and the robot waited forever.',
     'Hold the starting heading (turn in place when it drifts too far); wait about 6 cm from the wall instead of 3.5 cm.',
     '21/21 in 2D with start errors and drift of a few degrees.'],
])

h1('4. Real-robot phase (10–13 Nov)')
log_table('Table 2. Changes made on the real robot.', [
    ['10 Nov', 'Control loop ran at only 5–6 Hz on the Jetson Nano.',
     'All 16 directions are scored only when a decision is due; LiDAR conversion vectorised; timers use the ROS '
     'clock instead of counting ticks.', 'About 20 Hz.'],
    ['10 Nov', 'LiDAR `range_min` is 0.15 m, so nothing within about 8 cm of the front is visible.',
     'Pendulum stopping point 10 cm before the line; goal stop at 5 cm free; real `wall_gap` 0.08 m of extra '
     'clearance in the travel direction.', 'No more front bumps.'],
    ['10 Nov', 'Heading hold swayed because of command delays.',
     'Correct only beyond 6° and stop within 2°, turning at 0.2 rad/s.', '–'],
    ['10 Nov', 'Robot 4: constant false alarms and circling in the same area.',
     'Root cause: its Optitrack heading was about 175° off its real forward direction (rigid body defined facing the '
     'other way), so visited points, novelty and motion detection all worked backwards. Corrected with a heading '
     'offset in `optitrack_variables.config` (no longer needed after the robot was recalibrated). Several parameter '
     'tweaks were tried before this was found.', 'Much better behaviour.'],
    ['10–12 Nov', 'Detection window needed adjusting for the wide pendulum corridor.',
     'Detection window 0.15–1.0 m ahead and 1.2 m to each side; an alarm needs at least 10 moving points.',
     '–'],
    ['10–12 Nov', 'The creep towards the line clipped door posts.',
     'The door check uses 2 cm of side clearance.', '–'],
    ['10–12 Nov', 'At the second pendulum a bent wall blocked the dash path.',
     'If the dash path is blocked while waiting, the robot slides away from the wall until it is clear.',
     'Crossed.'],
    ['12 Nov', 'Lining up with the wall made the robot rotate endlessly beside a slightly convex wall.',
     'Wall alignment removed; the robot keeps the heading it was placed with (placed square at the start).', '–'],
    ['10–12 Nov', 'Tried capping forward speed at the free distance ahead.',
     'Reverted: in one run the robot went through the pendulums.', '–'],
    ['13 Nov', 'Final tuning.',
     '`visited_weight` 0.25, `w_new` 1.8, goal 315°, `t_wait` 4 s, `min_speed` 0.075 m/s.', 'Graded-run code.'],
])

h1('5. Graded runs (13 Nov)')
bullets([
    '**3 out of 3 runs successful.** Runs 1 and 2 went through comfortably.',
    'Run 3: the prof placed the cylinders to try to trap the robot before the door. The goal pull took it into the '
    'dead end at the right of the middle corridor; it left again on a different line, found the door, crossed both '
    'pendulums and stopped at the end wall.',
    'Run 3 took 1 min 25 s and had only 2–3 false pendulum alarms, each costing a 2 s stop.'], size=9.5)

h1('6. Final parameters (real robot)')
table(['Parameter', 'Value', 'Parameter', 'Value'], [
    ['Directions', '16, every 22.5°', 'Motion detection', 'scans 0.5 s apart, 10 cm margin'],
    ['Commit distance / hysteresis', '10 cm / 0.1', 'Alarm / confirmation', '≥ 10 points / ≥ 5 points in a 2 s stop'],
    ['Room weight (cap)', '1.0 (1 m)', 'Stop before the line', '10 cm'],
    ['Novelty weight (look-ahead)', '1.8 (0.4 m)', 'Gap to the right wall', 'about 6 cm'],
    ['Proximity weight (radius)', '0.25 (0.15 m)', 'Pendulum timeout', '4 s'],
    ['Lane shifts', '±12 cm, 2 cm steps', 'Extra clearance ahead (`wall_gap`)', '0.08 m'],
    ['Speed / minimum speed', '0.3 / 0.075 m/s', 'Heading hold', '6° on, 2° off, 0.2 rad/s'],
], [4.2, 3.6, 4.6, 4.6], 'Table 3. Main values in the final code.', size=8.5)

doc.save(D + 'Group5_reactive_code_dev_log.docx')
print('saved')
