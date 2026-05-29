#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算法级评价指标可视化工具。

输入一个或多个 algorithm_metrics.csv，输出：
    1. algorithm_metrics_summary.csv：多场景汇总表。
    2. algorithm_metrics_scoreboard.png：完成率、安全性、平滑性等综合柱状图。
    3. algorithm_metrics_table.png：适合汇报/论文检查的紧凑表格图。
"""

import argparse
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.pyplot as plt
import pandas as pd


LEVEL_ORDER = ['easy', 'medium', 'hard']
LEVEL_COLORS = {
    'easy': '#4e79a7',
    'medium': '#f28e2b',
    'hard': '#e15759',
}


def _scene_name_from_path(path):
    parent = Path(path).parent.name
    for suffix in ['_current', '_orca', '_pingpong_v1']:
        if parent.endswith(suffix):
            return parent[: -len(suffix)]
    return parent


def _level_from_scene(scene):
    text = str(scene).lower()
    if 'easy' in text:
        return 'easy'
    if 'medium' in text:
        return 'medium'
    if 'hard' in text:
        return 'hard'
    return 'unknown'


def _read_metrics(paths):
    rows = []
    for path in paths:
        df = pd.read_csv(path)
        if df.empty:
            continue
        row = df.iloc[0].to_dict()
        row['scene_name'] = _scene_name_from_path(path)
        row['difficulty_level'] = _level_from_scene(row['scene_name'])
        row['metrics_csv'] = path
        rows.append(row)
    if not rows:
        raise ValueError('no metric rows loaded')
    out = pd.DataFrame(rows)
    out['_sort'] = out['difficulty_level'].map({name: idx for idx, name in enumerate(LEVEL_ORDER)}).fillna(99)
    out = out.sort_values(['_sort', 'scene_name']).drop(columns=['_sort'])
    return out


def _colors(df):
    return [LEVEL_COLORS.get(level, '#777777') for level in df['difficulty_level']]


def _bar_labels(ax, bars, fmt='{:.2f}', scale=1.0):
    for bar in bars:
        value = bar.get_height()
        text = fmt.format(value * scale)
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            value,
            text,
            ha='center',
            va='bottom',
            fontsize=8,
            clip_on=True,
        )


def _finish(fig, path):
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_scoreboard(df, output_path):
    names = df['scene_name'].astype(str).tolist()
    x = range(len(df))
    colors = _colors(df)
    fig, axes = plt.subplots(3, 2, figsize=(11, 8.0), dpi=180, sharex=True)
    axes = axes.ravel()
    specs = [
        ('completion_rate', 'Completion rate', (0.0, 1.0), '{:.0f}%', 100.0),
        ('collision_events_total', 'Collision events', None, '{:.0f}', 1.0),
        ('global_min_safety_clearance', 'Min safety clearance (m)', None, '{:.2f}', 1.0),
        ('avoidance_success_rate', 'Avoidance success rate', (0.0, 1.0), '{:.0f}%', 100.0),
        ('control_jerk_rms_mean', 'Control jerk RMS', None, '{:.2f}', 1.0),
        ('real_time_factor', 'Real-time factor', None, '{:.2f}', 1.0),
    ]
    for ax, (col, title, ylim, fmt, scale) in zip(axes, specs):
        values = pd.to_numeric(df[col], errors='coerce').fillna(0.0)
        bars = ax.bar(x, values, color=colors, width=0.62)
        _bar_labels(ax, bars, fmt=fmt, scale=scale)
        if col == 'global_min_safety_clearance':
            ax.axhline(0.0, color='#111111', linewidth=0.9, linestyle='--')
        if col == 'real_time_factor':
            ax.axhline(1.0, color='#111111', linewidth=0.9, linestyle='--')
        if ylim is not None:
            ax.set_ylim(*ylim)
        ax.set_title(title, fontsize=10)
        ax.grid(True, axis='y', linewidth=0.35, alpha=0.35)
    axes[-1].set_xticks(list(x))
    axes[-1].set_xticklabels(names, rotation=20, ha='right')
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    fig.suptitle('Algorithm Evaluation Metrics', fontsize=13)
    _finish(fig, output_path)


def plot_table(df, output_path):
    cols = [
        ('scene_name', 'Scene'),
        ('completion_rate', 'Completion'),
        ('collision_events_total', 'Collisions'),
        ('near_miss_samples_total', 'Near miss samples'),
        ('global_min_safety_clearance', 'Min clearance'),
        ('avoidance_success_rate', 'Avoid success'),
        ('path_length_3d_mean', 'Path length'),
        ('control_jerk_rms_mean', 'Jerk RMS'),
        ('real_time_factor', 'RTF'),
    ]
    table = pd.DataFrame()
    for col, label in cols:
        if col == 'scene_name':
            table[label] = df[col].astype(str)
        elif col in ('completion_rate', 'avoidance_success_rate'):
            table[label] = pd.to_numeric(df[col], errors='coerce').fillna(0.0).map(lambda v: f'{100.0 * v:.0f}%')
        elif col in ('collision_events_total', 'near_miss_samples_total'):
            table[label] = pd.to_numeric(df[col], errors='coerce').fillna(0.0).map(lambda v: f'{v:.0f}')
        else:
            table[label] = pd.to_numeric(df[col], errors='coerce').map(lambda v: '' if pd.isna(v) else f'{v:.2f}')

    fig, ax = plt.subplots(figsize=(11, 1.2 + 0.42 * len(table)), dpi=190)
    ax.axis('off')
    mpl_table = ax.table(
        cellText=table.values,
        colLabels=table.columns,
        cellLoc='center',
        colLoc='center',
        loc='center',
    )
    mpl_table.auto_set_font_size(False)
    mpl_table.set_fontsize(8)
    mpl_table.scale(1.0, 1.35)
    for (row, col), cell in mpl_table.get_celld().items():
        cell.set_linewidth(0.45)
        if row == 0:
            cell.set_facecolor('#eeeeee')
            cell.set_text_props(weight='bold')
        elif row % 2 == 0:
            cell.set_facecolor('#f8f8f8')
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description='Plot algorithm evaluation metrics')
    parser.add_argument('--metrics', nargs='+', required=True, help='one or more algorithm_metrics.csv files')
    parser.add_argument('--output-dir', required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    df = _read_metrics(args.metrics)
    summary_csv = os.path.join(args.output_dir, 'algorithm_metrics_summary.csv')
    df.to_csv(summary_csv, index=False)
    plot_scoreboard(df, os.path.join(args.output_dir, 'algorithm_metrics_scoreboard.png'))
    plot_table(df, os.path.join(args.output_dir, 'algorithm_metrics_table.png'))
    print(f'summary_csv={summary_csv}')
    print(f'plots={args.output_dir}')


if __name__ == '__main__':
    main()
