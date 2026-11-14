# Section 1 (introduction) prose in the user's style; their own sentences (aka..., the cunning Professor...) kept.
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

replace_para('The course has three parts:',
  'The course has three parts: a zig-zag corridor with static obstacles, a narrow door, and two hand-released pendulums '
  '(aka the dynamic obstacles at the end) before the end wall. Our controller is **purely reactive**: it keeps no map, '
  'plans no path and is never told where the goal is, only the general direction to head in. Twenty times a second, it '
  'turns the latest LiDAR scan into obstacle points and decides what to do based on those points alone.')
replace_para('The robot is holonomic,',
  'Since the robot is holonomic, it **never turns to navigate**. Instead, it slides in any direction while keeping the '
  'heading it started with, so its own frame stays lined up with the course and every decision can be made in that '
  'frame. The Optitrack pose is only used as a memory: where the robot has been, where a pendulum is, and how far a dash '
  'has gone.')

with zipfile.ZipFile(OUT, 'w') as zo:
    zo.writestr(zi.getinfo('mimetype'), zi.read('mimetype'), compress_type=zipfile.ZIP_STORED)
    for it in zi.infolist():
        if it.filename == 'mimetype': continue
        data = x.encode() if it.filename == 'content.xml' else zi.read(it.filename)
        zo.writestr(it, data, compress_type=zipfile.ZIP_STORED if it.filename.endswith(('.png', '.jpg')) else zipfile.ZIP_DEFLATED)
print('ok')
