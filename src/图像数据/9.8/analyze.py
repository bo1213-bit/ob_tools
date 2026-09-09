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

for key in sorted(rows):
    v = rows[key]
    hs = [x[0] for x in v]
    gs = [x[1] for x in v]
    ss = [x[2] for x in v]
    fns = [x[3] for x in v]
    print(f'{key}: n={len(v)}')
    print(f'   frameNumber: first={fns[0]} last={fns[-1]}  span={fns[-1]-fns[0]+1}')
    print(f'   hwTs span(s)={(hs[-1]-hs[0])/1e6:.3f} first_dt={(hs[1]-hs[0])/1e3:.2f}ms last_dt={(hs[-1]-hs[-2])/1e3:.2f}ms')
    print(f'   globalTs span(s)={(gs[-1]-gs[0])/1e6:.3f}')
    print(f'   sysTs span(s)={(ss[-1]-ss[0])/1e6:.3f}')
    # inter-frame global gaps
    gg = [gs[i+1]-gs[i] for i in range(len(gs)-1)]
    gg_sorted = sorted(gg)
    print(f'   global inter-frame gaps (ms): min={min(gg)/1e3:.2f} med={gg_sorted[len(gg)//2]/1e3:.2f} max={max(gg)/1e3:.2f}')
