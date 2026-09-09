#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# detect_frame_glitches.py
# 读取 raw.csv，检测"重复时间戳 / 跳帧"错位。
# 若 raw.csv 含 frameNumber 列(新格式)，进一步判断每个重复是
#   - 同帧被重投(host 侧 buffer 问题)    : 两条重复记录的 frameNumber 相同
#   - 两帧共用一个时间戳(设备侧时间戳冻结): 两条重复记录的 frameNumber 不同
# 用法:
#   python detect_frame_glitches.py <raw.csv> [--match-ms N]
#   --match-ms: 跨设备"孤儿帧"判定的配对阈值(默认 5ms)，仅影响第 3 段输出

import csv
import sys
import statistics
from collections import defaultdict

# 强制 UTF-8 输出，避免 Windows 下中文标签在终端里乱码
try:
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
except Exception:
    pass


def load(path):
    """读取 raw.csv，兼容 5 列(旧) 和 6 列(含 frameNumber) 两种格式。"""
    rows = defaultdict(list)   # (deviceIndex, streamType) -> list[dict]
    with open(path, newline='', encoding='utf-8-sig') as fh:
        reader = csv.reader(fh)
        header = [h.strip() for h in next(reader)]
        col = {name: i for i, name in enumerate(header)}
        has_fn = 'frameNumber' in col
        for line in reader:
            if not line or line[0].strip() == '':
                continue
            dev = int(line[col['deviceIndex']])
            stream = line[col['streamType']].strip()
            hw = int(line[col['hwTimestampUs']])
            g = int(line[col['globalTimestampUs']])
            s = int(line[col['sysTimestampUs']])
            fn = int(line[col['frameNumber']]) if has_fn else None
            rows[(dev, stream)].append({'hw': hw, 'g': g, 's': s, 'fn': fn})
    return rows, has_fn


def median_interval(d):
    """返回 (帧周期中位数 us, 相邻帧间隔列表 us)。"""
    ivs = [d[i + 1]['hw'] - d[i]['hw'] for i in range(len(d) - 1)]
    pos = [iv for iv in ivs if iv > 0]
    median = statistics.median(pos) if pos else 33333
    return median, ivs


def main():
    if len(sys.argv) < 2:
        print("用法: python detect_frame_glitches.py <raw.csv> [--match-ms N]")
        sys.exit(1)
    path = sys.argv[1]
    match_ms = 5.0
    if '--match-ms' in sys.argv:
        i = sys.argv.index('--match-ms')
        if i + 1 < len(sys.argv):
            match_ms = float(sys.argv[i + 1])

    rows, has_fn = load(path)
    devices = sorted({k[0] for k in rows})
    streams = ['DEPTH', 'COLOR']

    print("=" * 72)
    print(f"文件: {path}")
    print(f"frameNumber 列: {'有(可定性)' if has_fn else '无(仅统计)'}")
    print("=" * 72)

    print("\n--- 1. 逐流 重复时间戳 / 跳帧 统计 ---")
    for dev in devices:
        for stream in streams:
            key = (dev, stream)
            if key not in rows:
                continue
            d = sorted(rows[key], key=lambda x: x['hw'])
            if len(d) < 2:
                continue
            median, ivs = median_interval(d)
            dups = sum(1 for iv in ivs if iv < 0.3 * median)
            gaps = sum(1 for iv in ivs if iv > 1.7 * median)
            uniq = len({x['hw'] for x in d})
            flag = '  <<< 有错位' if (dups or gaps) else ''
            print(f"  dev{dev} {stream:5s}: 总帧={len(d):3d}  唯一时间戳={uniq:3d}  "
                  f"重复={dups:2d}  跳帧={gaps:2d}  帧周期≈{median:.0f}us{flag}")

    print("\n--- 2. 重复帧定性 (同帧重投 vs 设备时间戳冻结) ---")
    if not has_fn:
        print("  (当前 raw.csv 无 frameNumber 列，无法定性；用新代码重跑即可得到该列。)")
    else:
        found = False
        for dev in devices:
            for stream in streams:
                key = (dev, stream)
                if key not in rows:
                    continue
                d = sorted(rows[key], key=lambda x: x['hw'])
                if len(d) < 2:
                    continue
                median, ivs = median_interval(d)
                for i, iv in enumerate(ivs):
                    if iv < 0.3 * median:
                        a, b = d[i], d[i + 1]
                        verdict = '同帧重投(host侧 buffer)' if a['fn'] == b['fn'] else '两帧时间戳冻结(设备侧)'
                        print(f"  dev{dev} {stream}: 第{i + 1}帧 hw={a['hw']}  "
                              f"fn {a['fn']} vs {b['fn']}  ->  {verdict}")
                        found = True
        if not found:
            print("  (未检测到重复帧)")

    print(f"\n--- 3. 跨设备 DEPTH 配对检查 (最近邻阈值 {match_ms}ms) ---")
    depths = {dev: sorted({x['g'] for x in rows[(dev, 'DEPTH')]})
              for dev in devices if (dev, 'DEPTH') in rows}
    if len(depths) >= 2:
        th_us = match_ms * 1000
        for dev in devices:
            if dev not in depths:
                continue
            others = [o for o in depths if o != dev]
            orphan = 0
            for g in depths[dev]:
                nearest = min(abs(g - gg) for o in others for gg in depths[o])
                if nearest > th_us:
                    orphan += 1
            print(f"  dev{dev} DEPTH: {orphan:3d}/{len(depths[dev])} 帧"
                  f"在其它设备找不到 {match_ms}ms 内的配对帧")
    else:
        print("  (缺少多设备 DEPTH 数据，跳过)")

    print("\n说明: 重复=相邻帧硬件时间戳相同; 跳帧=相邻帧间隔约2倍周期(丢帧)。")
    print("      跨设备 orphan≈0 表示最近邻配对能自行对齐，否则配对会错位。")


if __name__ == '__main__':
    main()
