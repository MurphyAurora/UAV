#!/usr/bin/env python3
"""第 2 层：离线场景绘图。

输入：
    场景 YAML，可选离线生成的 obstacles.csv。
输出：
    包含静态墙、参考航线、目标点和动态障碍轨迹的 2D x/y 图。

用于在接入 ROS/Gazebo 前检查场景几何关系是否合理。
"""

import argparse
import csv
import os

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.pyplot as plt

from scenario_demo import generate_rows, load_scene, resolve_path


def finite_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def rows_by_obstacle(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row['name'], []).append(row)
    for values in grouped.values():
        values.sort(key=lambda row: finite_float(row['time']))
    return grouped


def load_rows(path):
    with open(path, 'r', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def draw_walls(ax, scene):
    walls = ((scene.get('static_obstacles') or {}).get('walls')) or []
    for wall in walls:
        x = finite_float(wall.get('x'))
        y = finite_float(wall.get('y'))
        length = finite_float(wall.get('length'), 1.0)
        thickness = finite_float(wall.get('thickness'), 0.25)
        rect = plt.Rectangle(
            (x - thickness * 0.5, y - length * 0.5),
            thickness,
            length,
            facecolor='#777777',
            edgecolor='#333333',
            alpha=0.55,
            label='static wall',
        )
        ax.add_patch(rect)
        ax.text(x, y, str(wall.get('name', 'wall')), fontsize=8, ha='center', va='center')


def draw_target(ax, scene):
    target = scene.get('target') or {}
    x = finite_float(target.get('x'))
    y = finite_float(target.get('y'))
    ax.scatter([x], [y], marker='*', s=180, color='#f2b705', edgecolor='#5f4700', zorder=5, label='target')
    ax.text(x, y + 0.8, 'target', fontsize=9, ha='center')


def draw_reference_route(ax, scene):
    target = scene.get('target') or {}
    x = finite_float(target.get('x'))
    y = finite_float(target.get('y'))
    ax.plot([0.0, x], [0.0, y], linestyle='--', linewidth=1.2, color='#222222', alpha=0.55, label='reference route')
    ax.scatter([0.0], [0.0], marker='o', s=45, color='#222222', zorder=5, label='start')
    ax.text(0.0, -0.8, 'start', fontsize=9, ha='center')


def draw_dynamic_obstacles(ax, rows):
    colors = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#ff7f0e', '#17becf', '#8c564b']
    for idx, (name, obs_rows) in enumerate(rows_by_obstacle(rows).items()):
        xs = [finite_float(row['x']) for row in obs_rows]
        ys = [finite_float(row['y']) for row in obs_rows]
        color = colors[idx % len(colors)]
        ax.plot(xs, ys, linewidth=2.0, color=color, label=name)
        if xs and ys:
            ax.scatter([xs[0]], [ys[0]], marker='o', s=35, color=color, edgecolor='white', zorder=4)
            ax.scatter([xs[-1]], [ys[-1]], marker='x', s=45, color=color, zorder=4)
            ax.text(xs[0], ys[0] + 0.6, name, fontsize=8, color=color)


def plot_scene(scene, rows, output_path):
    meta = scene.get('scene') or {}
    world = scene.get('world') or {}
    bounds = world.get('bounds') or {}

    fig, ax = plt.subplots(figsize=(9, 7), dpi=140)
    ax.set_title(
        f"{meta.get('name', 'scene')} | level={meta.get('level', 'unknown')} | seed={meta.get('seed', 0)}",
        fontsize=12,
    )
    ax.set_xlabel('x (NED)')
    ax.set_ylabel('y (NED)')
    ax.grid(True, linewidth=0.5, alpha=0.35)
    ax.set_aspect('equal', adjustable='box')

    draw_walls(ax, scene)
    draw_reference_route(ax, scene)
    draw_target(ax, scene)
    draw_dynamic_obstacles(ax, rows)

    x_bounds = bounds.get('x')
    y_bounds = bounds.get('y')
    if isinstance(x_bounds, list) and len(x_bounds) == 2:
        ax.set_xlim(finite_float(x_bounds[0]), finite_float(x_bounds[1]))
    if isinstance(y_bounds, list) and len(y_bounds) == 2:
        ax.set_ylim(finite_float(y_bounds[0]), finite_float(y_bounds[1]))

    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    unique = []
    for handle, label in zip(handles, labels):
        if label not in seen:
            unique.append((handle, label))
            seen.add(label)
    ax.legend(
        [item[0] for item in unique],
        [item[1] for item in unique],
        loc='upper left',
        fontsize=8,
        frameon=True,
    )

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def default_output_path(scene):
    meta = scene.get('scene') or {}
    name = str(meta.get('name', 'scene')).strip().replace(' ', '_')
    return os.path.join('runs', f'{name}_plot.png')


def main():
    parser = argparse.ArgumentParser(description='Plot a generated dynamic-obstacle scenario in x/y.')
    parser.add_argument('--scene', default='scenes/demo_dynamic.yaml')
    parser.add_argument('--csv', default='', help='Optional existing obstacles.csv to plot.')
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    scene = load_scene(args.scene)
    rows = load_rows(resolve_path(args.csv)) if args.csv else generate_rows(scene)
    output_path = args.output or default_output_path(scene)
    plot_scene(scene, rows, output_path)
    print(f'plot={output_path}')


if __name__ == '__main__':
    main()
