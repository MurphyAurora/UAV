#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Formation Survivability 可视化工具。

支持：
    1. S_formation / D_formation 柱状图。
    2. 固定高度 z 的采样点存活时间热力图。
    3. 动态障碍物轨迹和任务评估区域俯视图。
"""

import argparse
import os

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.pyplot as plt
import pandas as pd
import yaml
from matplotlib.patches import Rectangle


def load_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def _normalize_summary(df):
    df = df.copy()
    if 'scene_name' not in df.columns and 'scene' in df.columns:
        df['scene_name'] = df['scene']
    if 'difficulty_level' not in df.columns and 'survivability_level' in df.columns:
        df['difficulty_level'] = df['survivability_level'].astype(str).str.capitalize()
    if 'manual_level' not in df.columns and 'declared_level' in df.columns:
        df['manual_level'] = df['declared_level'].astype(str).str.capitalize()
    return df


def _level_colors(values):
    return values.map({'Easy': '#4e79a7', 'Medium': '#f28e2b', 'Hard': '#e15759'}).fillna('#777777')


def _bool_series(series):
    if series.dtype == bool:
        return series
    return series.astype(str).str.lower().isin(['true', '1', 'yes'])


def _scene_z(config, fallback):
    region = config.get('evaluation_region') or {}
    z_min = region.get('z_min')
    z_max = region.get('z_max')
    if z_min is None or z_max is None:
        return fallback
    return 0.5 * (float(z_min) + float(z_max))


def _draw_region_and_route(ax, config):
    region = config.get('evaluation_region') or {}
    route = config.get('route') or {}
    x_min = region.get('x_min')
    x_max = region.get('x_max')
    y_min = region.get('y_min')
    y_max = region.get('y_max')
    if None not in (x_min, x_max, y_min, y_max):
        rect = Rectangle(
            (float(x_min), float(y_min)),
            float(x_max) - float(x_min),
            float(y_max) - float(y_min),
            fill=False,
            linewidth=1.8,
            edgecolor='#222222',
            linestyle='--',
            label='evaluation region',
        )
        ax.add_patch(rect)
    start = route.get('start')
    target = route.get('target')
    if isinstance(start, list) and isinstance(target, list) and len(start) >= 2 and len(target) >= 2:
        ax.plot([start[0], target[0]], [start[1], target[1]], color='#111111', linewidth=2.0, label='task centerline')
        ax.scatter([start[0]], [start[1]], marker='o', s=70, color='#2ca02c', edgecolor='white', zorder=5, label='start')
        ax.scatter([target[0]], [target[1]], marker='*', s=120, color='#d62728', edgecolor='white', zorder=5, label='target')


def plot_summary_bars(summary_csv, output_dir):
    """绘制批量结果中的 S_formation 和 D_formation 柱状图。"""
    df = _normalize_summary(pd.read_csv(summary_csv))
    ensure_dir(output_dir)
    for value_col, filename in [
        ('S_formation', 'S_formation_bar.png'),
        ('D_formation', 'D_formation_bar.png'),
    ]:
        fig, ax = plt.subplots(figsize=(9, 4.8), dpi=180)
        colors = _level_colors(df['difficulty_level'])
        bars = ax.bar(df['scene_name'].astype(str), df[value_col], color=colors)
        for bar, value, level in zip(bars, df[value_col], df['difficulty_level']):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height(),
                f'{value:.2f}\n{level}',
                ha='center',
                va='bottom',
                fontsize=8,
            )
        if value_col == 'D_formation':
            ax.axhline(1.0, color='#4e79a7', linestyle='--', linewidth=1.0, label='Easy/Medium threshold')
            ax.axhline(3.5, color='#e15759', linestyle='--', linewidth=1.0, label='Medium/Hard threshold')
            ax.set_ylim(0.0, max(10.0, float(df[value_col].max()) * 1.2))
            ax.legend(loc='upper left', fontsize=8)
        plt.ylabel(value_col)
        plt.xticks(rotation=30, ha='right')
        plt.title(f'{value_col} by scene')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, filename))
        plt.close()


def plot_survival_heatmap(
    samples_csv,
    output_path,
    z_value=2.0,
    z_tol=1e-6,
    config_path='',
    trajectory_csv='',
):
    """绘制固定高度切片上的采样点存活时间风险图。"""
    df = pd.read_csv(samples_csv)
    sl = df[(df['z'] - float(z_value)).abs() <= float(z_tol)].copy()
    if sl.empty:
        nearest_z = df.iloc[(df['z'] - float(z_value)).abs().argsort()].iloc[0]['z']
        sl = df[(df['z'] - nearest_z).abs() <= 1e-9].copy()
        z_value = nearest_z

    config = load_config(config_path) if config_path else {}
    traj = pd.read_csv(trajectory_csv) if trajectory_csv else None
    failed = _bool_series(sl['failed']) if 'failed' in sl.columns else pd.Series(False, index=sl.index)

    fig, ax = plt.subplots(figsize=(8, 7), dpi=190)
    if traj is not None and not traj.empty:
        for obstacle_id, group in traj.groupby('obstacle_id'):
            ax.plot(
                group['x'],
                group['y'],
                linewidth=1.4,
                alpha=0.55,
                color='#111111',
                label='obstacle trajectory' if obstacle_id == traj['obstacle_id'].iloc[0] else None,
            )

    _draw_region_and_route(ax, config)

    sc = ax.scatter(
        sl['x'],
        sl['y'],
        c=sl['survival_time'],
        cmap='RdYlGn',
        s=58,
        edgecolor='white',
        linewidth=0.35,
        zorder=3,
    )
    if failed.any():
        ax.scatter(
            sl.loc[failed, 'x'],
            sl.loc[failed, 'y'],
            facecolors='none',
            edgecolors='#d62728',
            s=105,
            linewidth=1.2,
            zorder=4,
            label='failed sample',
        )
    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label('survival_time (s)')
    ax.set_xlabel('world x')
    ax.set_ylabel('world y')
    ax.set_title(f'Formation survivability map at z={float(z_value):g}')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linewidth=0.35, alpha=0.35)
    ax.legend(loc='best', fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def plot_trajectory_topdown(config_path, trajectory_csv, output_path, samples_csv=''):
    """绘制动态障碍物轨迹和任务评估区域俯视图。"""
    config = load_config(config_path)
    traj = pd.read_csv(trajectory_csv)
    samples = pd.read_csv(samples_csv) if samples_csv else None

    fig, ax = plt.subplots(figsize=(8, 7), dpi=190)
    _draw_region_and_route(ax, config)
    if samples is not None and not samples.empty:
        failed = _bool_series(samples['failed']) if 'failed' in samples.columns else pd.Series(False, index=samples.index)
        ax.scatter(samples['x'], samples['y'], s=16, color='#bdbdbd', alpha=0.35, label='sample center')
        if failed.any():
            ax.scatter(
                samples.loc[failed, 'x'],
                samples.loc[failed, 'y'],
                s=24,
                color='#e15759',
                alpha=0.65,
                label='failed sample',
            )
    for obstacle_id, group in traj.groupby('obstacle_id'):
        ax.plot(group['x'], group['y'], marker='.', markersize=2.2, linewidth=1.4, label=f'obstacle: {obstacle_id}')
        ax.scatter(group['x'].iloc[0], group['y'].iloc[0], marker='o', s=35, color='black', zorder=4)
        ax.scatter(group['x'].iloc[-1], group['y'].iloc[-1], marker='x', s=45, color='black', zorder=4)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('world x')
    ax.set_ylabel('world y')
    ax.set_title('Obstacle trajectories, task region, and sampled formation centers')
    ax.grid(True, linewidth=0.35, alpha=0.35)
    ax.legend(loc='best', fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def plot_scene_diagnostics(config_path, trajectory_csv, samples_csv, output_dir, z_value):
    """生成单个场景的诊断图。"""
    ensure_dir(output_dir)
    config = load_config(config_path)
    z = _scene_z(config, z_value)
    plot_survival_heatmap(
        samples_csv,
        os.path.join(output_dir, f'survivability_map_z{z:g}.png'),
        z_value=z,
        config_path=config_path,
        trajectory_csv=trajectory_csv,
    )
    plot_trajectory_topdown(
        config_path,
        trajectory_csv,
        os.path.join(output_dir, 'trajectory_region_samples.png'),
        samples_csv=samples_csv,
    )


def plot_scenes_from_summary(summary_csv, output_dir, z_value):
    """根据 suite CSV 自动为每个场景生成可视化图。"""
    df = _normalize_summary(pd.read_csv(summary_csv))
    required = {'scene_name', 'sample_csv', 'trajectory_csv', 'plot_config'}
    if not required.issubset(df.columns):
        return
    for _, row in df.iterrows():
        scene_dir = os.path.join(output_dir, str(row['scene_name']))
        plot_scene_diagnostics(row['plot_config'], row['trajectory_csv'], row['sample_csv'], scene_dir, z_value)


def parse_args():
    parser = argparse.ArgumentParser(description='Plot Formation Survivability outputs')
    parser.add_argument('--summary-csv', default='', help='batch difficulty_summary.csv')
    parser.add_argument('--config', default='', help='scene_config.yaml for topdown plot')
    parser.add_argument('--trajectory', default='', help='obstacle_trajectory.csv for topdown plot')
    parser.add_argument('--samples', default='', help='samples CSV generated by formation_survivability.py')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--z', type=float, default=2.0)
    parser.add_argument('--z-tol', type=float, default=1e-6)
    parser.add_argument('--plot-scenes-from-summary', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    ensure_dir(args.output_dir)
    if args.summary_csv:
        plot_summary_bars(args.summary_csv, args.output_dir)
    if args.samples:
        plot_survival_heatmap(
            args.samples,
            os.path.join(args.output_dir, f'survivability_map_z{args.z:g}.png'),
            z_value=args.z,
            z_tol=args.z_tol,
            config_path=args.config,
            trajectory_csv=args.trajectory,
        )
    if args.config and args.trajectory:
        plot_trajectory_topdown(
            args.config,
            args.trajectory,
            os.path.join(args.output_dir, 'trajectory_region_samples.png'),
            samples_csv=args.samples,
        )
    if args.summary_csv and args.plot_scenes_from_summary:
        plot_scenes_from_summary(args.summary_csv, args.output_dir, args.z)
    print(f'plots={args.output_dir}')


if __name__ == '__main__':
    main()
