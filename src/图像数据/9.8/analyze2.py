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

# Compare DEPTH across devices: use frame index (frameNumber) to align, then look at global timestamp offsets
depth = {}
for dev in range(3):
    depth[dev] = rows[(dev, 'DEPTH')]

# For each device, build dict frameNumber -> globalTimestampUs
fn_glob = {}
for dev in range(3):
    fn_glob[dev] = {fn: g for (h, g, s, fn) in depth[dev]}

# Find frame numbers present in all 3 devices
common_fns = sorted(set(fn_glob[0]) & set(fn_glob[1]) & set(fn_glob[2]))
print(f'common frameNumbers across 3 devices: {len(common_fns)}')
if common_fns:
    print(f'  first={common_fns[0]} last={common_fns[-1]}')

# global timestamp offset of each device relative to device 0, at same frameNumber
print('\n--- globalTimestamp offset (dev_i - dev_0) at same frameNumber ---')
for ref in [0]:
    for dev in [1, 2]:
        offs = []
        for fn in common_fns:
            offs.append(fn_glob[dev][fn] - fn_glob[ref][fn])
        offs.sort()
        print(f'dev{dev}-dev{ref}: n={len(offs)} min={offs[0]/1e3:.2f}ms med={offs[len(offs)//2]/1e3:.2f}ms max={offs[-1]/1e3:.2f}ms  first5={[round(o/1e3,2) for o in offs[:5]]} last5={[round(o/1e3,2) for o in offs[-5:]]}')

# drift: compare offset in first 10 frames vs last 10 frames
print('\n--- offset drift over time (first vs last 10 common frames) ---')
for dev in [1, 2]:
    head = [fn_glob[dev][fn] - fn_glob[0][fn] for fn in common_fns[:10]]
    tail = [fn_glob[dev][fn] - fn_glob[0][fn] for fn in common_fns[-10:]]
    print(f'dev{dev}-dev0: head_mean={sum(head)/len(head)/1e3:.2f}ms  tail_mean={sum(tail)/len(tail)/1e3:.2f}ms  drift={((sum(tail)-sum(head))/len(tail))/1e3:.2f}ms')
