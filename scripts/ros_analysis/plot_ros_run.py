#!/usr/bin/env python3
"""第 5 层：运行记录可视化脚本。

输入：
    ros_run_recorder.py 生成的输出目录。
输出：
    plots/ 下的 overview.png、zoom_obstacle.png 和 obstacle_avoid.png。

图中对齐真实无人机轨迹、动态障碍轨迹、最近距离点和避障输出时刻，
用于把 Gazebo 中的肉眼观察和记录数据相互校验。
"""

import argparse
import bisect
import csv
import math
import os

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.pyplot as plt


COLORS = [
    '#1f77b4',
    '#ff7f0e',
    '#2ca02c',
    '#d62728',
    '#9467bd',
    '#8c564b',
    '#17becf',
]


def _float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def _read_rows(path):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def _nearest_by_time(items, times, t):
    if not items:
        return None
    idx = bisect.bisect_left(times, t)
    candidates = []
    if idx < len(items):
        candidates.append(items[idx])
    if idx > 0:
        candidates.append(items[idx - 1])
    return min(candidates, key=lambda row: abs(row['time'] - t)) if candidates else None


def _load_run(run_dir):
    run_dir = os.path.abspath(os.path.expanduser(run_dir))
    drones = _read_rows(os.path.join(run_dir, 'drones.csv'))
    obstacles = _read_rows(os.path.join(run_dir, 'obstacles_ros.csv'))
    avoid = _read_rows(os.path.join(run_dir, 'avoid_cmds.csv'))
    return run_dir, drones, obstacles, avoid


def _group_drones(rows):
    grouped = {}
    for row in rows:
        drone_id = int(_float(row.get('drone_id'), -1))
        item = {
            'time': _float(row.get('time')),
            'x': _float(row.get('world_x', row.get('x'))),
            'y': _float(row.get('world_y', row.get('y'))),
            'z': _float(row.get('z')),
        }
        grouped.setdefault(drone_id, []).append(item)
    for values in grouped.values():
        values.sort(key=lambda row: row['time'])
    return grouped


def _named_obstacles(rows):
    out = []
    for row in rows:
        name = str(row.get('name', '')).strip()
        if not name:
            continue
        out.append(
            {
                'time': _float(row.get('time')),
                'scene_time': _float(row.get('scene_time'), -1.0),
                'name': name,
                'x': _float(row.get('x')),
                'y': _float(row.get('y')),
                'z': _float(row.get('z')),
                'radius': _float(row.get('radius'), 0.0),
            }
        )
    out.sort(key=lambda row: row['time'])
    return out


def _avoid_nonzero_by_drone(rows, threshold):
    grouped = {}
    for row in rows:
        drone_id = int(_float(row.get('drone_id'), -1))
        speed = _float(row.get('speed_xy'))
        if speed <= threshold:
            continue
        grouped.setdefault(drone_id, []).append(
            {
                'time': _float(row.get('time')),
                'speed': speed,
                'vx': _float(row.get('vx')),
                'vy': _float(row.get('vy')),
            }
        )
    for values in grouped.values():
        values.sort(key=lambda row: row['time'])
    return grouped


def _closest_points(drones_by_id, obstacles):
    obs_times = [row['time'] for row in obstacles]
    closest = {}
    for drone_id, rows in drones_by_id.items():
        best = None
        for row in rows:
            obs = _nearest_by_time(obstacles, obs_times, row['time'])
            if obs is None:
                continue
            dist = math.hypot(row['x'] - obs['x'], row['y'] - obs['y'])
            if best is None or dist < best['distance_xy']:
                best = {
                    'distance_xy': dist,
                    'drone': row,
                    'obstacle': obs,
                }
        if best is not None:
            closest[drone_id] = best
    return closest


def _sample_rows(rows, stride):
    stride = max(1, int(stride))
    return rows[::stride]


def _positions_at_times(path_rows, event_rows):
    if not path_rows or not event_rows:
        return [], [], []
    times = [row['time'] for row in path_rows]
    xs = []
    ys = []
    speeds = []
    for event in event_rows:
        nearest = _nearest_by_time(path_rows, times, event['time'])
        if nearest is None:
            continue
        xs.append(nearest['x'])
        ys.append(nearest['y'])
        speeds.append(event['speed'])
    return xs, ys, speeds


def _positions_at_obstacle_avoid_times(path_rows, event_rows, obstacles, influence_radius):
    if not path_rows or not event_rows or not obstacles:
        return [], [], []
    path_times = [row['time'] for row in path_rows]
    obs_times = [row['time'] for row in obstacles]
    xs = []
    ys = []
    speeds = []
    for event in event_rows:
        pos = _nearest_by_time(path_rows, path_times, event['time'])
        obs = _nearest_by_time(obstacles, obs_times, event['time'])
        if pos is None or obs is None:
            continue
        threshold = influence_radius + obs.get('radius', 0.0)
        dist = math.hypot(pos['x'] - obs['x'], pos['y'] - obs['y'])
        if dist <= threshold:
            xs.append(pos['x'])
            ys.append(pos['y'])
            speeds.append(event['speed'])
    return xs, ys, speeds


def _set_limits(ax, drones_by_id, obstacles):
    xs = []
    ys = []
    for rows in drones_by_id.values():
        xs.extend(row['x'] for row in rows)
        ys.extend(row['y'] for row in rows)
    xs.extend(row['x'] for row in obstacles)
    ys.extend(row['y'] for row in obstacles)
    if not xs or not ys:
        return
    pad = max(2.0, 0.08 * max(max(xs) - min(xs), max(ys) - min(ys), 1.0))
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)


def _set_zoom_limits(ax, obstacles, closest, pad=2.5):
    xs = [row['x'] for row in obstacles]
    ys = [row['y'] for row in obstacles]
    for item in closest.values():
        xs.append(item['drone']['x'])
        ys.append(item['drone']['y'])
        xs.append(item['obstacle']['x'])
        ys.append(item['obstacle']['y'])
    if not xs or not ys:
        return
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)


def plot_run(
    run_dir,
    output_path,
    nonzero_threshold=0.01,
    influence_radius=2.0,
    title='',
    view='overview',
    marker_stride=6,
    dpi=180,
):
    run_dir, drone_rows, obstacle_rows, avoid_rows = _load_run(run_dir)
    drones_by_id = _group_drones(drone_rows)
    obstacles = _named_obstacles(obstacle_rows)
    avoid_by_id = _avoid_nonzero_by_drone(avoid_rows, nonzero_threshold)
    closest = _closest_points(drones_by_id, obstacles)

    fig, ax = plt.subplots(figsize=(11, 8.5), dpi=dpi)
    view_title = {
        'overview': 'overview',
        'zoom': 'obstacle zoom',
        'obstacle_avoid': 'obstacle avoid points',
    }.get(view, view)
    ax.set_title(f'{title or os.path.basename(run_dir)} | {view_title}', fontsize=12)
    ax.set_xlabel('world_x / NED x')
    ax.set_ylabel('world_y / NED y')
    ax.grid(True, linewidth=0.5, alpha=0.35)
    ax.set_aspect('equal', adjustable='box')

    if obstacles:
        obs_x = [row['x'] for row in obstacles]
        obs_y = [row['y'] for row in obstacles]
        obs_name = obstacles[0]['name']
        ax.plot(obs_x, obs_y, color='#111111', linewidth=2.4, label=f'obstacle: {obs_name}', zorder=4)
        ax.scatter([obs_x[0]], [obs_y[0]], marker='o', s=60, color='#111111', edgecolor='white', zorder=7)
        ax.scatter([obs_x[-1]], [obs_y[-1]], marker='x', s=70, color='#111111', zorder=7)
        ax.text(obs_x[0], obs_y[0] + 0.45, obs_name, fontsize=8, color='#111111')

    for idx, drone_id in enumerate(sorted(drones_by_id)):
        rows = drones_by_id[drone_id]
        color = COLORS[idx % len(COLORS)]
        xs = [row['x'] for row in rows]
        ys = [row['y'] for row in rows]
        ax.plot(xs, ys, linewidth=1.7, color=color, alpha=0.85, label=f'x500_{drone_id}')
        if xs and ys:
            ax.scatter([xs[0]], [ys[0]], marker='o', s=28, color=color, edgecolor='white', zorder=5)
            ax.scatter([xs[-1]], [ys[-1]], marker='s', s=30, color=color, edgecolor='white', zorder=5)

        avoid_events = avoid_by_id.get(drone_id, [])
        show_all_orca = view in ('overview', 'zoom')
        if show_all_orca:
            avoid_x, avoid_y, speeds = _positions_at_times(rows, _sample_rows(avoid_events, marker_stride))
        else:
            avoid_x, avoid_y, speeds = [], [], []
        if avoid_x:
            ax.scatter(
                avoid_x,
                avoid_y,
                s=[max(6.0, min(30.0, 55.0 * speed)) for speed in speeds],
                color=color,
                edgecolor='none',
                alpha=0.16,
                zorder=6,
                label=f'x500_{drone_id} ORCA>0',
            )
        obs_avoid_x, obs_avoid_y, obs_speeds = _positions_at_obstacle_avoid_times(
            rows,
            _sample_rows(avoid_events, marker_stride),
            obstacles,
            influence_radius,
        )
        if obs_avoid_x:
            ax.scatter(
                obs_avoid_x,
                obs_avoid_y,
                s=[max(18.0, min(68.0, 105.0 * speed)) for speed in obs_speeds],
                facecolor='none',
                edgecolor=color,
                linewidth=1.1,
                alpha=0.95,
                zorder=9,
                label=f'x500_{drone_id} obstacle avoid',
            )

    for drone_id, item in closest.items():
        drone = item['drone']
        obs = item['obstacle']
        color = COLORS[(drone_id - 1) % len(COLORS)]
        ax.plot(
            [drone['x'], obs['x']],
            [drone['y'], obs['y']],
            linestyle=':',
            linewidth=1.1,
            color=color,
            alpha=0.75,
            zorder=3,
        )
        ax.scatter([drone['x']], [drone['y']], marker='D', s=34, color=color, edgecolor='white', zorder=8)
        ax.text(
            drone['x'],
            drone['y'] + 0.35,
            f'd{drone_id} min {item["distance_xy"]:.2f}m',
            fontsize=7,
            color=color,
            ha='center',
        )
        threshold = influence_radius + obs.get('radius', 0.0)
        circle = plt.Circle(
            (obs['x'], obs['y']),
            threshold,
            facecolor='none',
            edgecolor=color,
            linestyle='--',
            linewidth=0.8,
            alpha=0.28,
        )
        ax.add_patch(circle)

    if view == 'overview':
        _set_limits(ax, drones_by_id, obstacles)
    else:
        _set_zoom_limits(ax, obstacles, closest)

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
        loc='upper right' if view != 'overview' else 'best',
        fontsize=7,
        frameon=True,
    )

    footer = (
        f'run={run_dir} | nonzero_threshold={nonzero_threshold:.3f} | '
        f'influence_radius={influence_radius:.2f} | marker_stride={marker_stride}'
    )
    fig.text(0.02, 0.01, footer, fontsize=7, color='#444444')
    fig.tight_layout(rect=(0, 0.025, 1, 1))

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)
    obstacle_avoid_count = 0
    for drone_id, rows in drones_by_id.items():
        x_obs, _, _ = _positions_at_obstacle_avoid_times(
            rows,
            avoid_by_id.get(drone_id, []),
            obstacles,
            influence_radius,
        )
        obstacle_avoid_count += len(x_obs)
    return (
        output_path,
        len(drones_by_id),
        len(obstacles),
        sum(len(v) for v in avoid_by_id.values()),
        obstacle_avoid_count,
    )


def default_output(run_dir):
    return os.path.join(os.path.abspath(os.path.expanduser(run_dir)), 'plots', 'trajectory.png')


def default_output_dir(run_dir):
    return os.path.join(os.path.abspath(os.path.expanduser(run_dir)), 'plots')


def default_outputs(run_dir):
    output_dir = default_output_dir(run_dir)
    return [
        ('overview', os.path.join(output_dir, 'overview.png')),
        ('zoom', os.path.join(output_dir, 'zoom_obstacle.png')),
        ('obstacle_avoid', os.path.join(output_dir, 'obstacle_avoid.png')),
    ]


def main():
    parser = argparse.ArgumentParser(description='Plot drones, dynamic obstacle, and ORCA output from ros_run_recorder CSVs.')
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--output', default='')
    parser.add_argument('--nonzero-threshold', type=float, default=0.01)
    parser.add_argument('--influence-radius', type=float, default=2.0)
    parser.add_argument('--title', default='')
    parser.add_argument('--view', choices=['overview', 'zoom', 'obstacle_avoid'], default='overview')
    parser.add_argument('--plot-set', choices=['single', 'standard'], default='standard')
    parser.add_argument('--marker-stride', type=int, default=6)
    parser.add_argument('--dpi', type=int, default=180)
    args = parser.parse_args()

    if args.output or args.plot_set == 'single':
        output = args.output or default_output(args.run_dir)
        targets = [(args.view, output)]
    else:
        targets = default_outputs(args.run_dir)

    last_counts = None
    for view, output in targets:
        path, drone_count, obstacle_count, avoid_count, obstacle_avoid_count = plot_run(
            args.run_dir,
            output,
            nonzero_threshold=args.nonzero_threshold,
            influence_radius=args.influence_radius,
            title=args.title,
            view=view,
            marker_stride=args.marker_stride,
            dpi=args.dpi,
        )
        print(f'plot={path}')
        last_counts = (drone_count, obstacle_count, avoid_count, obstacle_avoid_count)
    if last_counts is not None:
        drone_count, obstacle_count, avoid_count, obstacle_avoid_count = last_counts
        print(
            f'drones={drone_count} named_obstacle_rows={obstacle_count} '
            f'nonzero_avoid_points={avoid_count} obstacle_avoid_points={obstacle_avoid_count}'
        )


if __name__ == '__main__':
    main()
