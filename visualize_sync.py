#!/usr/bin/env python3
# visualize_sync.py
# 模块3: 读取 CSV，生成时间戳同步精度图表
#
# Usage: python visualize_sync.py result.csv [--output ./charts]

import sys
import os
import csv
import argparse
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(description='Visualize timestamp sync CSV')
    parser.add_argument('csv', help='CSV file from SyncAnalyzer')
    parser.add_argument('--output', '-o', default='./charts', help='Output directory (default: ./charts)')
    return parser.parse_args()


def read_csv(csv_path):
    """
    Reads CSV into:
    {
        'cross_stream': [{'device_i':0, 'hw_diff':-120, 'sys_diff':350}, ...],
        'cross_device_depth': [...],
        'cross_device_color': [...]
    }
    """
    data = defaultdict(list)
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            comparison = row['comparison_type']
            stream = row['stream']
            if comparison == 'cross_device':
                key = f"cross_device_{stream}"
            else:
                key = 'cross_stream'

            data[key].append({
                'device_i': int(row['device_i']),
                'device_j': int(row['device_j']),
                'hw_diff': float(row['hw_diff_us']),
                'sys_diff': float(row['sys_diff_us']),
            })
    return data


def compute_stats(values):
    if not values:
        return {'min': 0, 'max': 0, 'mean': 0, 'stddev': 0, 'count': 0}
    arr = np.array(values)
    return {
        'min': np.min(arr),
        'max': np.max(arr),
        'mean': np.mean(arr),
        'stddev': np.std(arr),
        'count': len(arr),
    }


def plot_comparison(grouped_data, key, title, output_path):
    if key not in grouped_data or not grouped_data[key]:
        print(f"  [SKIP] No data for '{key}'")
        return

    pairs = defaultdict(list)
    for d in grouped_data[key]:
        pairs[(d['device_i'], d['device_j'])].append(d)

    n_pairs = len(pairs)
    fig, axes = plt.subplots(1, n_pairs, figsize=(5 * n_pairs, 5), squeeze=False)
    axes = axes[0]

    for ax, ((di, dj), records) in zip(axes, pairs.items()):
        hw_vals = [r['hw_diff'] for r in records]
        sys_vals = [r['sys_diff'] for r in records]

        bp = ax.boxplot([hw_vals, sys_vals], labels=['HW Diff', 'Sys Diff'],
                        patch_artist=True, widths=0.5)
        bp['boxes'][0].set_facecolor('#4ECDC4')
        bp['boxes'][1].set_facecolor('#FF6B6B')

        ax.set_title(f'Device {di} vs Device {dj}')
        ax.set_ylabel('Diff (us)')
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

        hw_stats = compute_stats(hw_vals)
        sys_stats = compute_stats(sys_vals)
        ax.text(0.65, 0.95,
                f"HW: mean={hw_stats['mean']:.1f} std={hw_stats['stddev']:.1f}\n"
                f"Sys: mean={sys_stats['mean']:.1f} std={sys_stats['stddev']:.1f}",
                transform=ax.transAxes, fontsize=7, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    fig.suptitle(title, fontsize=14, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {output_path}")


def generate_html(data, output_dir):
    sections = [
        ('cross_stream', '1. Cross-Stream (Depth vs Color)', 'cross_stream_sync.png'),
        ('cross_device_depth', '2. Cross-Device Depth', 'cross_device_depth.png'),
        ('cross_device_color', '3. Cross-Device Color', 'cross_device_color.png'),
    ]

    table_rows = ""
    for key, title, img in sections:
        if key not in data or not data[key]:
            continue
        pairs = defaultdict(list)
        for d in data[key]:
            pairs[(d['device_i'], d['device_j'])].append(d)

        for (di, dj), records in sorted(pairs.items()):
            hw_vals = [r['hw_diff'] for r in records]
            sys_vals = [r['sys_diff'] for r in records]
            hw_s = compute_stats(hw_vals)
            sys_s = compute_stats(sys_vals)
            table_rows += f"""<tr>
    <td>{title}</td>
    <td>Device {di} vs {dj}</td>
    <td>{hw_s['count']}</td>
    <td>{hw_s['min']:.1f}</td><td>{hw_s['max']:.1f}</td><td>{hw_s['mean']:.1f}</td><td>{hw_s['stddev']:.1f}</td>
    <td>{sys_s['min']:.1f}</td><td>{sys_s['max']:.1f}</td><td>{sys_s['mean']:.1f}</td><td>{sys_s['stddev']:.1f}</td>
</tr>"""

    html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Timestamp Sync Analysis Report</title>
    <style>
        body {{ font-family: -apple-system, 'Segoe UI', sans-serif; max-width: 1200px; margin: auto; padding: 20px; background: #fafafa; }}
        h1 {{ color: #333; border-bottom: 2px solid #4ECDC4; padding-bottom: 10px; }}
        h2 {{ color: #555; margin-top: 40px; }}
        table {{ border-collapse: collapse; margin: 10px 0; width: 100%; font-size: 13px; }}
        th, td {{ border: 1px solid #ddd; padding: 6px 10px; text-align: right; }}
        th {{ background: #4ECDC4; color: white; }}
        td:first-child, td:nth-child(2) {{ text-align: left; }}
        img {{ max-width: 100%; border: 1px solid #eee; border-radius: 4px; margin: 10px 0; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }}
    </style>
</head>
<body>
<h1>Timestamp Sync Analysis Report</h1>

<h2>Summary Statistics</h2>
<table>
    <tr>
        <th>Comparison</th><th>Pair</th><th>Count</th>
        <th colspan="4">HW Timestamp Diff (us)</th>
        <th colspan="4">System Timestamp Diff (us)</th>
    </tr>
    <tr>
        <th></th><th></th><th></th>
        <th>Min</th><th>Max</th><th>Mean</th><th>Stddev</th>
        <th>Min</th><th>Max</th><th>Mean</th><th>Stddev</th>
    </tr>
    {table_rows}
</table>

<h2>Charts</h2>
<h2>1. Cross-Stream (Depth vs Color)</h2>
<img src="cross_stream_sync.png" alt="Cross-Stream Sync">

<h2>2. Cross-Device Depth</h2>
<img src="cross_device_depth.png" alt="Cross-Device Depth">

<h2>3. Cross-Device Color</h2>
<img src="cross_device_color.png" alt="Cross-Device Color">

<p><em>Generated by visualize_sync.py</em></p>
</body>
</html>"""

    html_path = os.path.join(output_dir, 'summary.html')
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"  Saved: {html_path}")


def main():
    args = parse_args()
    os.makedirs(args.output, exist_ok=True)

    print(f"Reading CSV: {args.csv}")
    data = read_csv(args.csv)
    print(f"  cross_stream: {len(data.get('cross_stream', []))} records")
    print(f"  cross_device_depth: {len(data.get('cross_device_depth', []))} records")
    print(f"  cross_device_color: {len(data.get('cross_device_color', []))} records")

    print("\nGenerating charts...")
    plot_comparison(data, 'cross_stream',
                    'Cross-Stream Sync (Depth vs Color)',
                    os.path.join(args.output, 'cross_stream_sync.png'))
    plot_comparison(data, 'cross_device_depth',
                    'Cross-Device Sync (Depth)',
                    os.path.join(args.output, 'cross_device_depth.png'))
    plot_comparison(data, 'cross_device_color',
                    'Cross-Device Sync (Color)',
                    os.path.join(args.output, 'cross_device_color.png'))

    print("\nGenerating HTML report...")
    generate_html(data, args.output)

    print(f"\nReport generated: {args.output}/summary.html")


if __name__ == '__main__':
    main()