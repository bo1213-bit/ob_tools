#!/usr/bin/env python3
"""
visualize_timestamps.py — HW timestamp sync precision histogram for TimestampRecorder output.

Reads the CSV produced by TimestampRecorder (TimestampRecorder.cpp):
    groupId,deviceIndex,streamType,hwTimestampUs,globalTimestampUs,sysTimestampUs

Computes cross-device sync precision as:
    max(hwTimestampUs) - min(hwTimestampUs)   across ALL devices per (groupId, streamType)

Then draws a three-in-one style overview chart identical in style to
src/visualize_sync.py:
    X-axis = sync precision time difference (us)
    Y-axis = frequency (count per bin)
  Bin width and tick spacing both fixed at 50 us.

Usage: python visualize_timestamps.py <csv_path> [--output ./charts] [--name overview_combined.png]
"""

import os
import csv
import argparse
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MaxNLocator, MultipleLocator


# ── Palette (identical to src/visualize_sync.py) ─────────────────────────
SLOT1_BLUE     = '#2a78d6'
MEAN_COLOR     = '#1c5cab'
MEDIAN_COLOR   = '#d95926'
GRID_COLOR     = '#e1e0d9'
AXIS_COLOR     = '#c3c2b7'
TEXT_PRIMARY   = '#0b0b0b'
TEXT_SECONDARY = '#52514e'
BG_SURFACE     = '#fcfcfb'

BIN_WIDTH_US = 50  # fixed bin width and tick step (us)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Timestamp sync precision histogram (TimestampRecorder output)')
    parser.add_argument('csv', help='CSV file from TimestampRecorder')
    parser.add_argument('--output', '-o', default='./charts')
    parser.add_argument('--name', '-n', default=None,
                        help='Output filename (default: overview_combined.png)')
    return parser.parse_args()


def compute_cross_device_diff(csv_path: str, max_diff_us: float = 20000.0) -> dict[str, list[float]]:
    """
    Group rows by (groupId, streamType), then compute max(hw) - min(hw) across ALL devices.
    Returns dict: {'depth': [max_diffs...], 'color': [max_diffs...]}.
    """
    # (groupId, streamType) -> {deviceIndex: hwTimestampUs}
    grouped = defaultdict(dict)
    with open(csv_path, 'r') as f:
        for row in csv.DictReader(f):
            gid_str = row['groupId']
            if gid_str.startswith('#'):
                continue
            gid = int(gid_str)
            dev = int(row['deviceIndex'])
            st = row['streamType'].strip().upper()
            try:
                hw = int(row['hwTimestampUs'])
            except (ValueError, KeyError):
                continue
            if st not in ('DEPTH', 'COLOR'):
                continue
            grouped[(gid, st)][dev] = hw

    result = defaultdict(list)
    skipped = 0
    for (gid, st), devmap in grouped.items():
        # need at least 2 devices to form a cross-device diff
        if len(devmap) < 2:
            continue
        # max(hw) - min(hw) across all devices → true worst-case sync precision
        timestamps = list(devmap.values())
        max_diff = max(timestamps) - min(timestamps)
        if max_diff > max_diff_us:
            skipped += 1
            continue
        result[st.lower()].append(float(max_diff))
    if skipped:
        print(f"  Filtered out {skipped} groups with max(ts)-min(ts) > {max_diff_us:.0f} us")
    return dict(result)


def fixed_bins(values: np.ndarray) -> np.ndarray:
    """Bin edges at multiples of BIN_WIDTH_US covering full data range."""
    vmin, vmax = float(values.min()), float(values.max())
    bw = BIN_WIDTH_US
    lo = np.floor(vmin / bw) * bw
    hi = np.ceil(vmax / bw) * bw
    return np.arange(lo, hi + bw, bw)


def compute_stats(values: np.ndarray) -> dict:
    mean = float(np.mean(values))
    std = float(np.std(values, ddof=0))
    median = float(np.median(values))
    n = len(values)
    if n > 2 and std > 0:
        skew = (n / ((n - 1) * (n - 2))) * float(np.sum(((values - mean) / std) ** 3))
    else:
        skew = 0.0
    pcts = np.percentile(values, [50, 75, 90, 95, 99])
    return {
        'count': n, 'min': float(values.min()), 'max': float(values.max()),
        'mean': mean, 'median': median, 'std': std, 'skew': skew,
        'p50': pcts[0], 'p75': pcts[1], 'p90': pcts[2],
        'p95': pcts[3], 'p99': pcts[4],
    }


def describe_skew(stats: dict) -> str:
    s = stats['skew']
    if abs(s) < 0.3:
        return '≈ symmetric'
    d = 'right' if s > 0 else 'left'
    if abs(s) < 1:
        return f'mildly {d} ({s:+.2f})'
    return f'{d}-skewed ({s:+.2f})'


def plot_combined_overview(all_data: dict[str, list[float]],
                           output_dir: str, fname: str = 'overview_combined.png'):
    groups = [
        ('depth',  'Cross-Device  Depth'),
        ('color',  'Cross-Device  Color'),
    ]

    present = [(k, label) for k, label in groups
               if k in all_data and len(all_data[k]) > 0]
    if not present:
        print("  [SKIP] Combined overview: no data")
        return

    n = len(present)
    fig, axes = plt.subplots(n, 1, figsize=(12, 4.0 * n), sharex=False)
    fig.patch.set_facecolor(BG_SURFACE)
    if n == 1:
        axes = [axes]

    for ax, (key, label) in zip(axes, present):
        values = np.array(all_data[key])
        stats = compute_stats(values)
        mean, median, std = stats['mean'], stats['median'], stats['std']

        bins = fixed_bins(values)
        ax.set_facecolor(BG_SURFACE)

        ax.hist(values, bins=bins,
                color=SLOT1_BLUE, edgecolor='#1c5cab', linewidth=0.4,
                alpha=0.85, zorder=3, rwidth=0.94)

        # Mean / median lines
        ax.axvline(mean, color=MEAN_COLOR, linewidth=1.8, linestyle='--',
                   alpha=0.85, zorder=4, label=f'Mean = {mean:.1f} us')
        ax.axvline(median, color=MEDIAN_COLOR, linewidth=1.8, linestyle=':',
                   alpha=0.85, zorder=4, label=f'Median = {median:.1f} us')

        # ── Stats box ─────────────────────────────────────────────
        lines = [
            f'N = {stats["count"]:,}',
            f'Mean = {mean:.2f} us',
            f'Median = {median:.2f} us',
            f'Std = {std:.2f} us',
            f'Skewness: {describe_skew(stats)}',
            f'Range: [{stats["min"]:.0f}, {stats["max"]:.0f}] us',
            f'P50 = {stats["p50"]:.0f} | P95 = {stats["p95"]:.0f}',
            f'P99 = {stats["p99"]:.0f} us',
        ]
        stats_text = '\n'.join(lines)

        x_pos = 0.02 if stats['skew'] <= 0 else 0.98
        ha = 'left' if stats['skew'] <= 0 else 'right'
        ax.text(x_pos, 0.97, stats_text, transform=ax.transAxes,
                fontsize=8.5, fontfamily='monospace', va='top', ha=ha,
                color=TEXT_PRIMARY,
                bbox=dict(boxstyle='round,pad=0.55', facecolor='#f9f9f7',
                           edgecolor='#d0cfc7', alpha=0.93, linewidth=0.7),
                zorder=10)

        # ── Axis styling ──────────────────────────────────────────
        ax.set_title(label, fontsize=12, fontweight='bold',
                     color=TEXT_PRIMARY, pad=10)
        ax.set_xlabel('|HW diff| (us)', fontsize=10, color=TEXT_PRIMARY)
        ax.set_ylabel('Frequency', fontsize=10, color=TEXT_PRIMARY)
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:,.0f}'))

        # Major ticks: auto-spaced labels; Minor ticks: every 50 us (the bin width)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=12, integer=True, steps=[1, 2, 5, 10]))
        ax.xaxis.set_minor_locator(MultipleLocator(BIN_WIDTH_US))
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:.0f}'))

        ax.grid(axis='y', color=GRID_COLOR, linewidth=0.8, alpha=0.7, zorder=0)
        ax.tick_params(axis='x', which='minor', length=4, color=TEXT_SECONDARY)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color(AXIS_COLOR)
        ax.spines['bottom'].set_color(AXIS_COLOR)
        ax.tick_params(colors=TEXT_SECONDARY, labelsize=8.5)

        # Legend
        legend = ax.legend(fontsize=9, framealpha=0.85,
                           edgecolor='#d0cfc7', facecolor='#fcfcfb')
        legend.set_zorder(11)

    fig.tight_layout(pad=2.0)
    out_path = os.path.join(output_dir, fname)
    fig.savefig(out_path, dpi=150, facecolor=BG_SURFACE, edgecolor='none')
    plt.close(fig)
    print(f"  Saved: {out_path}")


def main():
    args = parse_args()
    os.makedirs(args.output, exist_ok=True)

    print(f"Reading: {args.csv}")
    data = compute_cross_device_diff(args.csv)
    for k, v in data.items():
        print(f"  cross_device_{k}: {len(v):,} records")

    fname = args.name if args.name else 'overview_combined.png'
    plot_combined_overview(data, args.output, fname)
    print(f"\nDone: {args.output}/")


if __name__ == '__main__':
    main()
