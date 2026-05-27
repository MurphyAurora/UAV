#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平台场景版 Formation Survivability 难度分级。

输入：
    scenes/*.yaml 单场景文件，或 benchmark suite YAML。
输出：
    formation_survivability.json、survival_samples.csv、obstacle_trajectory.csv，
    scene_config_for_plot.yaml，或 suite 汇总文件。

说明：
    这个入口只负责把现有平台的 scenes/*.yaml 转成通用难度分析输入。
    核心 Formation Survivability 算法在 core.py，不依赖避障算法轨迹。
"""

import argparse
import csv
import json
import math
import os
import sys

import pandas as pd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from core import analyze, safe_float, write_yaml_config
from scenario_demo import finite_float, generate_rows, load_scene, load_suite


def _route_geometry(scene):
    target = scene.get('target') or {}
    start = (0.0, 0.0)
    end = (
        finite_float(target.get('x'), 0.0),
        finite_float(target.get('y'), 0.0),
    )
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return {'start': start, 'end': end, 'ux': 1.0, 'uy': 0.0, 'nx': 0.0, 'ny': 1.0, 'length': 1.0}
    ux = dx / length
    uy = dy / length
    return {'start': start, 'end': end, 'ux': ux, 'uy': uy, 'nx': -uy, 'ny': ux, 'length': length}


def _frange(start, end, step):
    step = max(abs(step), 1e-9)
    values = []
    value = start
    while value <= end + 1e-9:
        values.append(value)
        value += step
    if not values or abs(values[-1] - end) > 1e-6:
        values.append(end)
    return values


def _default_formation_d(scene):
    swarm = scene.get('swarm') or {}
    target = scene.get('target') or {}
    return finite_float(swarm.get('y_spacing', target.get('y_spacing', 1.8)), 1.8)


def _local_triangle_offsets(formation_d):
    d = max(0.0, formation_d)
    return [(0.0, 0.0, 0.0), (-d, -d, 0.0), (-d, d, 0.0)]


def _route_aligned_offsets(route, local_offsets):
    offsets = []
    for dx, dy, dz in local_offsets:
        offsets.append(
            [
                dx * route['ux'] + dy * route['nx'],
                dx * route['uy'] + dy * route['ny'],
                dz,
            ]
        )
    return offsets


def _formation_radius(local_offsets):
    return max(math.hypot(dx, dy) for dx, dy, _ in local_offsets) if local_offsets else 0.0


def _scene_duration(scene):
    return max(0.0, finite_float((scene.get('scene') or {}).get('duration'), 30.0))


def _flight_z(scene, override):
    if override is not None:
        return override
    return finite_float((scene.get('target') or {}).get('z'), -3.0)


def _corridor_half_width(args, local_offsets):
    if args.corridor_half_width is not None:
        return max(0.0, args.corridor_half_width)
    return _formation_radius(local_offsets) + max(0.0, args.avoid_range)


def _world_from_route_local(route, along, lateral, z):
    sx, sy = route['start']
    x = sx + along * route['ux'] + lateral * route['nx']
    y = sy + along * route['uy'] + lateral * route['ny']
    return x, y, z


def build_centers(scene, args, local_offsets):
    route = _route_geometry(scene)
    z0 = _flight_z(scene, args.flight_z)
    half_width = _corridor_half_width(args, local_offsets)
    vertical_range = max(0.0, args.vertical_range)

    rows = []
    center_id = 0
    for z in _frange(z0 - vertical_range, z0 + vertical_range, args.vertical_step):
        for along in _frange(0.0, route['length'], args.along_step):
            for lateral in _frange(-half_width, half_width, args.lateral_step):
                x, y, wz = _world_from_route_local(route, along, lateral, z)
                rows.append(
                    {
                        'center_id': center_id,
                        'x': x,
                        'y': y,
                        'z': wz,
                        'along': along,
                        'lateral': lateral,
                    }
                )
                center_id += 1
    return pd.DataFrame(rows)


def rows_to_trajectory(rows):
    records = []
    for row in rows:
        records.append(
            {
                'time': safe_float(row.get('time'), 0.0),
                'obstacle_id': str(row.get('name', 'dynamic_obstacle')),
                'x': safe_float(row.get('x'), 0.0),
                'y': safe_float(row.get('y'), 0.0),
                'z': safe_float(row.get('z'), 0.0),
                'radius': safe_float(row.get('radius'), 0.5),
            }
        )
    return pd.DataFrame(records)


def scene_to_config(scene, args, centers, local_offsets):
    meta = scene.get('scene') or {}
    target = scene.get('target') or {}
    duration = args.t_max if args.t_max is not None else _scene_duration(scene)
    z0 = _flight_z(scene, args.flight_z)
    return {
        'scene_name': str(meta.get('name', 'unnamed_scene')),
        'manual_level': str(meta.get('level', 'unknown')).capitalize(),
        'duration': float(duration),
        'dt': finite_float(meta.get('dt'), 0.5),
        'evaluation_region': {
            'x_min': float(centers['x'].min()) if not centers.empty else 0.0,
            'x_max': float(centers['x'].max()) if not centers.empty else 0.0,
            'y_min': float(centers['y'].min()) if not centers.empty else 0.0,
            'y_max': float(centers['y'].max()) if not centers.empty else 0.0,
            'z_min': float(centers['z'].min()) if not centers.empty else z0,
            'z_max': float(centers['z'].max()) if not centers.empty else z0,
        },
        'sampling': {
            'dx': args.along_step,
            'dy': args.lateral_step,
            'dz': args.vertical_step,
            'frame': 'route_aligned_corridor',
        },
        'route': {
            'start': [0.0, 0.0, z0],
            'target': [
                finite_float(target.get('x'), 0.0),
                finite_float(target.get('y'), 0.0),
                z0,
            ],
        },
        'formation': {
            'shape': 'triangle',
            'frame': 'route_aligned',
            'spacing': args.formation_d,
            'uav_radius': args.drone_radius,
            'safe_margin': args.safe_margin,
            'local_offsets': [[float(v) for v in offset] for offset in local_offsets],
        },
        'difficulty': {
            'normalize_min': 0.0,
            'normalize_max': float(duration),
            'easy_threshold': args.easy_threshold,
            'medium_threshold': args.hard_threshold,
        },
    }


def analyze_platform_scene(scene, args):
    formation_d = args.formation_d if args.formation_d is not None else _default_formation_d(scene)
    args.formation_d = formation_d
    route = _route_geometry(scene)
    local_offsets = _local_triangle_offsets(formation_d)
    offsets = _route_aligned_offsets(route, local_offsets)
    centers = build_centers(scene, args, local_offsets)
    trajectory = rows_to_trajectory(generate_rows(scene))
    config = scene_to_config(scene, args, centers, local_offsets)
    result = analyze(config, trajectory, centers=centers, offsets=offsets)
    return config, trajectory, result


def _public_survivability(result):
    return {
        'score': result['D_formation'],
        'level': str(result['difficulty_level']).lower(),
        'S_formation': result['S_formation'],
        'D_formation': result['D_formation'],
        'T_max': result['T_max'],
        'sample_count': result['sample_count'],
        'failed_sample_count': result['failed_sample_count'],
        'survived_sample_count': result['sample_count'] - result['failed_sample_count'],
        'failed_sample_ratio': result['failed_sample_ratio'],
        'min_survival_time': result['min_survival_time'],
        'median_survival_time': result['median_survival_time'],
        'max_survival_time': result['max_survival_time'],
    }


def _scene_summary(scene_path, args):
    scene = load_scene(scene_path)
    config, trajectory, result = analyze_platform_scene(scene, args)
    meta = scene.get('scene') or {}
    return {
        'scene': meta.get('name', 'unnamed_scene'),
        'seed': int(finite_float(meta.get('seed'), 0)),
        'declared_level': meta.get('level', 'unknown'),
        'scene_path': scene_path,
        'formation_survivability': _public_survivability(result),
        '_samples': result['samples'],
        '_config': config,
        '_trajectory': trajectory,
    }


def _write_scene_artifacts(row, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    samples_path = os.path.join(output_dir, 'survival_samples.csv')
    trajectory_path = os.path.join(output_dir, 'obstacle_trajectory.csv')
    config_path = os.path.join(output_dir, 'scene_config_for_plot.yaml')
    row['_samples'].to_csv(samples_path, index=False)
    row['_trajectory'].to_csv(trajectory_path, index=False)
    write_yaml_config(row['_config'], config_path)
    row['sample_csv'] = samples_path
    row['trajectory_csv'] = trajectory_path
    row['plot_config'] = config_path
    return samples_path, trajectory_path, config_path


def _clean_row(row):
    public = dict(row)
    public.pop('_samples', None)
    public.pop('_config', None)
    public.pop('_trajectory', None)
    return public


def _write_single(summary, output_dir):
    _write_scene_artifacts(summary, output_dir)
    json_path = os.path.join(output_dir, 'formation_survivability.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(_clean_row(summary), f, indent=2, ensure_ascii=False, sort_keys=True)
        f.write('\n')
    return json_path, summary['sample_csv']


def _write_suite(rows, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, 'suite_survivability.csv')
    json_path = os.path.join(output_dir, 'suite_survivability.json')
    fieldnames = [
        'scene',
        'seed',
        'declared_level',
        'survivability_level',
        'D_formation',
        'S_formation',
        'T_max',
        'sample_count',
        'failed_sample_ratio',
        'failed_sample_count',
        'min_survival_time',
        'median_survival_time',
        'scene_path',
        'sample_csv',
        'trajectory_csv',
        'plot_config',
    ]
    public_rows = []
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            scene_dir = os.path.join(output_dir, f"{row['scene']}_seed{row['seed']}")
            _write_scene_artifacts(row, scene_dir)
            surv = row['formation_survivability']
            writer.writerow(
                {
                    'scene': row['scene'],
                    'seed': row['seed'],
                    'declared_level': row['declared_level'],
                    'survivability_level': surv['level'],
                    'D_formation': surv['D_formation'],
                    'S_formation': surv['S_formation'],
                    'T_max': surv['T_max'],
                    'sample_count': surv['sample_count'],
                    'failed_sample_ratio': surv['failed_sample_ratio'],
                    'failed_sample_count': surv['failed_sample_count'],
                    'min_survival_time': surv['min_survival_time'],
                    'median_survival_time': surv['median_survival_time'],
                    'scene_path': row['scene_path'],
                    'sample_csv': row['sample_csv'],
                    'trajectory_csv': row['trajectory_csv'],
                    'plot_config': row['plot_config'],
                }
            )
            public_rows.append(_clean_row(row))
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump({'scenarios': public_rows}, f, indent=2, ensure_ascii=False, sort_keys=True)
        f.write('\n')
    return csv_path, json_path


def _add_arguments(parser):
    parser.add_argument('--scene', default='scenes/easy_crossing.yaml')
    parser.add_argument('--suite', default='')
    parser.add_argument('--output-dir', default='/tmp/formation_survivability_experiment')
    parser.add_argument('--drone-radius', type=float, default=0.35)
    parser.add_argument('--safe-margin', '--alert-margin', dest='safe_margin', type=float, default=0.6)
    parser.add_argument('--formation-d', type=float, default=None)
    parser.add_argument('--avoid-range', type=float, default=3.0)
    parser.add_argument('--corridor-half-width', type=float, default=None)
    parser.add_argument('--flight-z', type=float, default=None)
    parser.add_argument('--vertical-range', type=float, default=0.0)
    parser.add_argument('--vertical-step', type=float, default=1.0)
    parser.add_argument('--along-step', type=float, default=2.0)
    parser.add_argument('--lateral-step', type=float, default=1.0)
    parser.add_argument('--t-max', type=float, default=None)
    parser.add_argument('--easy-threshold', type=float, default=1.0)
    parser.add_argument('--hard-threshold', type=float, default=3.5)


def main():
    parser = argparse.ArgumentParser(description='Offline Formation Survivability difficulty grading.')
    _add_arguments(parser)
    args = parser.parse_args()
    if args.easy_threshold >= args.hard_threshold:
        raise ValueError('--easy-threshold must be smaller than --hard-threshold')

    if args.suite:
        suite = load_suite(args.suite)
        rows = [_scene_summary(scene_path, args) for scene_path in suite.get('scenarios') or []]
        csv_path, json_path = _write_suite(rows, args.output_dir)
        for row in rows:
            surv = row['formation_survivability']
            print(
                f"{row['scene']}: declared={row['declared_level']} "
                f"survivability={surv['level']} D={surv['D_formation']:.2f}/10 "
                f"S={surv['S_formation']:.2f}s failed={surv['failed_sample_ratio']:.2f}"
            )
        print(f'suite_survivability_csv={csv_path}')
        print(f'suite_survivability_json={json_path}')
        return

    summary = _scene_summary(args.scene, args)
    json_path, samples_path = _write_single(summary, args.output_dir)
    surv = summary['formation_survivability']
    print(
        f"{summary['scene']}: declared={summary['declared_level']} "
        f"survivability={surv['level']} D={surv['D_formation']:.2f}/10 "
        f"S={surv['S_formation']:.2f}s"
    )
    print(f'sample_count={surv["sample_count"]} failed_ratio={surv["failed_sample_ratio"]:.3f}')
    print(f'summary={json_path}')
    print(f'samples={samples_path}')
    print(f'trajectory={summary["trajectory_csv"]}')
    print(f'plot_config={summary["plot_config"]}')


if __name__ == '__main__':
    main()
