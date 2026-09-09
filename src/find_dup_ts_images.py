#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# find_dup_ts_images.py
# 扫描保存目录里的 color PNG，按文件名里的硬件时间戳找出"时间戳重复"的图片。
# 文件名格式(由 data_collector.cpp 生成): Device{dev}_frame_{seq:06d}_{时间戳us}.png
# 时间戳重复 = 两张(或多张)图片拥有同一个 hw 时间戳，即采集时的"时间戳冻结"点。
# 用法:
#   python find_dup_ts_images.py <图片目录> [--no-copy]
#   --no-copy: 只打印清单，不复制到 <目录>/dup_ts/ 子目录

import os
import re
import shutil
import sys

try:
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
except Exception:
    pass

FN_RE = re.compile(r'^Device(\d+)_frame_(\d{6})_(\d+)\.png$')


def main():
    if len(sys.argv) < 2:
        print("用法: python find_dup_ts_images.py <图片目录> [--no-copy]")
        sys.exit(1)

    src_dir = sys.argv[1]
    copy = '--no-copy' not in sys.argv

    if not os.path.isdir(src_dir):
        print(f"错误: 目录不存在: {src_dir}")
        sys.exit(1)

    # dev -> ts -> [(seq, filename), ...]
    groups = {}
    for fn in sorted(os.listdir(src_dir)):
        m = FN_RE.match(fn)
        if not m:
            continue
        dev, seq, ts = int(m.group(1)), int(m.group(2)), int(m.group(3))
        groups.setdefault(dev, {}).setdefault(ts, []).append((seq, fn))

    # 挑出时间戳重复的组
    dups = []  # (dev, ts, [(seq, filename), ...])
    total = 0
    for dev in sorted(groups):
        for ts, items in groups[dev].items():
            total += len(items)
            if len(items) > 1:
                items.sort()
                dups.append((dev, ts, items))

    print("=" * 72)
    print(f"目录: {src_dir}")
    print(f"共 {total} 张图片, 其中时间戳重复的组 {len(dups)} 个")
    print("=" * 72)

    if not dups:
        print("  (未发现时间戳重复的图片)")
        return

    out_dir = os.path.join(src_dir, 'dup_ts')
    if copy and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    for dev, ts, items in sorted(dups, key=lambda x: (x[0], x[1])):
        names = [f for _, f in items]
        print(f"\ndev{dev}  时间戳 {ts} 出现 {len(items)} 次:")
        for seq, fn in items:
            print(f"    seq {seq:06d}  {fn}")
            if copy:
                shutil.copy2(os.path.join(src_dir, fn), os.path.join(out_dir, fn))

    if copy:
        print(f"\n已把上述 {sum(len(i) for _, _, i in dups)} 张重复图片复制到: {out_dir}")

    print("\n提示: 打开同一组里不同 seq 的两张图对比画面即可——")
    print("      画面相同 = 同一帧被重放(真丢帧); 画面不同 = 两个不同帧被盖了同一时间戳(时间戳打错)。")


if __name__ == '__main__':
    main()
