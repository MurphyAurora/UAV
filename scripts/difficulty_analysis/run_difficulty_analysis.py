#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量运行 Formation Survivability 难度分析。

每个场景目录需要包含：
    scene_config.yaml
    obstacle_trajectory.csv

输出：
    difficulty_summary.csv，以及可选图表。
"""

import argparse
import json
import os

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.pyplot as plt
import pandas as pd

from core import analyze, load_trajectory, load_yaml_config, write_result_outputs


def find_scene_dirs(scene_root):
    """查找包含 scene_config.yaml 和 obstacle_trajectory.csv 的场景目录。"""
    scene_dirs = []
    for root, _, files in os.walk(scene_root):
        file_set = set(files)
        if 'scene_config.yaml' in file_set and 'obstacle_trajectory.csv' in file_set:
            scene_dirs.append(root)
    return sorted(scene_dirs)


def run_one_scene(scene_dir, result_dir):
    """分析单个场景目录。"""
    config_path = os.path.join(scene_dir, 'scene_config.yaml')
    trajectory_path = os.path.join(scene_dir, 'obstacle_trajectory.csv')
    config = load_yaml_config(config_path)
    trajectory = load_trajectory(trajectory_path)
    result = analyze(config, trajectory)

    scene_name = str(result['scene_name'])
    scene_result_dir = os.path.join(result_dir, scene_name)
    os.makedirs(scene_result_dir, exist_ok=True)
    output_path = os.path.join(scene_result_dir, 'formation_survivability_result.json')
    public = write_result_outputs(result, output_path)
    public['scene_dir'] = scene_dir
    public['result_json'] = output_path
    return public


def write_summary(rows, output_path):
    """保存批量汇总 CSV。"""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    df = pd.DataFrame(rows)
    columns = [
        'scene_name',
        'manual_level',
        'S_formation',
        'D_formation',
        'difficulty_level',
        'sample_count',
        'failed_sample_count',
        'failed_sample_ratio',
        'scene_dir',
        'result_json',
        'samples_csv',
    ]
    existing = [col for col in columns if col in df.columns]
    df[existing].to_csv(output_path, index=False)
    return df


def plot_level_bars(df, value_col, title, output_path):
    """绘制按人工等级分组的柱状图。"""
    plot_df = df.copy()
    if 'manual_level' not in plot_df or plot_df['manual_level'].replace('', pd.NA).isna().all():
        plot_df['manual_level'] = plot_df['difficulty_level']
    grouped = plot_df.groupby('manual_level', dropna=False)[value_col].mean().reindex(['Easy', 'Medium', 'Hard'])
    grouped = grouped.dropna()
    plt.figure(figsize=(7, 4), dpi=160)
    grouped.plot(kind='bar', color=['#4e79a7', '#f28e2b', '#e15759'][: len(grouped)])
    plt.ylabel(value_col)
    plt.title(title)
    plt.xticks(rotation=0)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def plot_all(summary_df, output_dir):
    """生成 S_formation 和 D_formation 的等级柱状图。"""
    os.makedirs(output_dir, exist_ok=True)
    plot_level_bars(
        summary_df,
        'S_formation',
        'Mean S_formation by level',
        os.path.join(output_dir, 'S_formation_by_level.png'),
    )
    plot_level_bars(
        summary_df,
        'D_formation',
        'Mean D_formation by level',
        os.path.join(output_dir, 'D_formation_by_level.png'),
    )


def parse_args():
    parser = argparse.ArgumentParser(description='Batch Formation Survivability analysis')
    parser.add_argument('--scene-root', required=True, help='directory containing scene subdirectories')
    parser.add_argument('--output', required=True, help='difficulty_summary.csv')
    parser.add_argument('--result-dir', default='', help='per-scene result directory')
    parser.add_argument('--plot-dir', default='', help='optional directory for summary plots')
    return parser.parse_args()


def main():
    args = parse_args()
    result_dir = args.result_dir or os.path.join(os.path.dirname(os.path.abspath(args.output)), 'scene_results')
    scene_dirs = find_scene_dirs(args.scene_root)
    if not scene_dirs:
        raise RuntimeError(f'no scene directories found under {args.scene_root}')

    rows = [run_one_scene(scene_dir, result_dir) for scene_dir in scene_dirs]
    df = write_summary(rows, args.output)
    if args.plot_dir:
        plot_all(df, args.plot_dir)

    json_path = os.path.splitext(args.output)[0] + '.json'
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(rows, f, indent=2, ensure_ascii=False, sort_keys=True)
        f.write('\n')

    print(f'scenes={len(rows)}')
    print(f'summary={args.output}')
    print(f'summary_json={json_path}')
    if args.plot_dir:
        print(f'plots={args.plot_dir}')


if __name__ == '__main__':
    main()
