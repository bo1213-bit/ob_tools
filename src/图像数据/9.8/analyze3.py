import csv
from collections import defaultdict
import os

base = os.path.dirname(os.path.abspath(__file__))

rows = defaultdict(list)
with open(os.path.join(base, 'raw_30s.csv')) as f:
    r = csv.DictReader(f)
    for d in r:
        rows[(int(d['deviceIndex']), d['streamType'])].append(
            (int(d['hwTimestampUs']), int(d['globalTimestampUs']), int(d['sysTimestampUs']), int(d['frameNumber'])))

def stats(v, name, unit_ms=True):
    v = sorted(v)
    div = 1e3 if unit_ms else 1.0
    med = v[len(v)//2]/div
    return f'{name}: min={v[0]/div:.2f} med={med:.2f} max={v[-1]/div:.2f}'

for st in ['DEPTH', 'COLOR']:
    print(f'===== {st} =====')
    for dev in range(3):
        d = rows[(dev, st)]
        # sort by frameNumber (index) to get capture order
        d = sorted(d, key=lambda x: x[3])
        print(f'\nDevice {dev}: n={len(d)}  frameNumber first={d[0][3]} last={d[-1][3]}')
        hw0, g0, s0, fn0 = d[0]
        hw1, g1, s1, fn1 = d[-1]
        print(f'  first frame: hw={hw0} global={g0} sys={s0} fn={fn0}')
        print(f'  last  frame: hw={hw1} global={g1} sys={s1} fn={fn1}')

# rank-aligned: all devices have same count, rank k = k-th frame in capture order
# compute spread (max-min) of global, sys, hw across 3 devices at each rank
for st in ['DEPTH', 'COLOR']:
    dseq = []
    for dev in range(3):
        d = sorted(rows[(dev, st)], key=lambda x: x[3])
        dseq.append(d)
    n = min(len(x) for x in dseq)
    gspread, sspread, hwspread = [], [], []
    for k in range(n):
        gs = [dseq[d][k][1] for d in range(3)]
        ss = [dseq[d][k][2] for d in range(3)]
        hs = [dseq[d][k][0] for d in range(3)]
        gspread.append(max(gs)-min(gs))
        sspread.append(max(ss)-min(ss))
        hwspread.append(max(hs)-min(hs))
    print(f'\n===== rank-aligned spread across 3 devices ({st}, n={n}) =====')
    print(stats(gspread, 'globalTs spread(us)', unit_ms=False))
    print(stats(sspread, 'sysTs    spread(us)', unit_ms=False))
    print(stats(hwspread, 'hwTs     spread(us)', unit_ms=False))
    # also print first 5 and last 5 spread values
    print('  global spread first5:', [round(x/1e3,2) for x in gspread[:5]], 'ms')
    print('  global spread last5 :', [round(x/1e3,2) for x in gspread[-5:]], 'ms')