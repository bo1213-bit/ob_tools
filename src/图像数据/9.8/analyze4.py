import csv
from collections import defaultdict
import os

base = os.path.dirname(os.path.abspath(__file__))

rows = defaultdict(list)
with open(os.path.join(base, 'raw_30s.csv')) as f:
    for d in csv.DictReader(f):
        rows[(int(d['deviceIndex']), d['streamType'])].append(
            (int(d['hwTimestampUs']), int(d['globalTimestampUs']), int(d['sysTimestampUs']), int(d['frameNumber'])))

HALF = 16667  # half frame @30fps

def greedy_match(ref, other, tol, field):
    # ref, other: lists of dicts sorted by hw
    ref = sorted(ref, key=lambda x: x[0])
    other = sorted(other, key=lambda x: x[0])
    ot = [o[field] for o in other]
    diffs = []
    for r in ref:
        rt = r[field]
        best = None
        bd = 1e18
        for j, t in enumerate(ot):
            d = abs(rt - t)
            if d < bd:
                bd = d; best = j
        if bd < tol:
            diffs.append(rt - ot[best])
    return diffs

def stat(name, v):
    v = sorted(v)
    med = v[len(v)//2]
    def pct(p):
        idx = int(p/100*(len(v)-1))
        return v[idx]
    print(f'  {name}: n={len(v)}')
    print(f'    min={v[0]/1e3:.3f}ms  p50={med/1e3:.3f}ms  p95={pct(95)/1e3:.3f}ms  max={v[-1]/1e3:.3f}ms')
    return med

for st in ['DEPTH', 'COLOR']:
    print(f'===== {st}: greedy match on HW timestamp (official-style) =====')
    ref = rows[(2, st)]
    for dev in [0, 1]:
        d = greedy_match(ref, rows[(dev, st)], HALF, 0)
        print(f'  dev2(ref) vs dev{dev}  (hw diff, + = other later):')
        stat('', d)

    print(f'  --- same but match on GLOBAL timestamp (current buggy code) ---')
    for dev in [0, 1]:
        d = greedy_match(ref, rows[(dev, st)], HALF, 1)
        print(f'  dev2(ref) vs dev{dev}  (global diff):')
        stat('', d)

    # 3-device spread using hw, ref=dev2
    print(f'  --- 3-device hw spread (ref=dev2) ---')
    r = sorted(rows[(2, st)], key=lambda x: x[0])
    o0 = sorted(rows[(0, st)], key=lambda x: x[0])
    o1 = sorted(rows[(1, st)], key=lambda x: x[0])
    spreads = []
    for rf in r:
        grp = [rf[0]]
        ok = True
        for o in [o0, o1]:
            bd = 1e18; bv = None
            for x in o:
                dd = abs(rf[0] - x[0])
                if dd < bd:
                    bd = dd; bv = x[0]
            if bd >= HALF:
                ok = False; break
            grp.append(bv)
        if ok:
            spreads.append(max(grp)-min(grp))
    stat('  3-device hw spread', spreads)