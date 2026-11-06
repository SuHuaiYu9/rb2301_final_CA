import sys, re
from collections import Counter
label = sys.argv[1]
rows = [r for r in sys.stdin.read().strip().splitlines() if r.startswith('seed')]
ok = [r for r in rows if 'GOAL' in r and re.search(r'0\.2\d m from end wall', r)]
pc = [int(re.search(r'pendulum (\d+) contacts', r).group(1)) for r in rows]
cr = Counter(x for r in rows for x in re.search(r'crossings: (\S+)', r).group(1).split(',') if x != '-')
fails = ['seed ' + r.split()[1] + ' ' + r.split('|')[1].split()[0] for r in rows if r not in ok]
print('%s: goal %d/%d | 0 pendulum contacts in %d/%d (%d contacts) | static %d | crossings %s | failures: %s' % (label, len(ok), len(rows), sum(c == 0 for c in pc), len(rows), sum(pc),
      sum(int(re.search(r'static (\d+)', r).group(1)) for r in rows), dict(cr), ', '.join(fails) or 'none'))
