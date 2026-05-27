#!/usr/bin/env python3
import argparse
import bisect
import csv
import json
import math
import os


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


def _mean(values):
    return sum(values) / len(values) if values else 0.0


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


def analyze_run(run_dir, obs_center=None, nonzero_threshold=0.01, influence_radius=2.0):
    run_dir = os.path.abspath(os.path.expanduser(run_dir))
    drones_path = os.path.join(run_dir, 'drones.csv')
    obstacles_path = os.path.join(run_dir, 'obstacles_ros.csv')
    avoid_path = os.path.join(run_dir, 'avoid_cmds.csv')

    drone_rows = _read_rows(drones_path)
    obstacle_rows = _read_rows(obstacles_path)
    avoid_rows = _read_rows(avoid_path)

    named_obstacles = []
    for row in obstacle_rows:
        if not str(row.get('name', '')).strip():
            continue
        named_obstacles.append(
            {
                'time': _float(row.get('time')),
                'scene_time': _float(row.get('scene_time'), -1.0),
                'name': str(row.get('name', '')),
                'x': _float(row.get('x')),
                'y': _float(row.get('y')),
                'z': _float(row.get('z')),
                'vx': _float(row.get('vx')),
                'vy': _float(row.get('vy')),
                'vz': _float(row.get('vz')),
                'radius': _float(row.get('radius'), 0.0),
            }
        )
    named_obstacles.sort(key=lambda row: row['time'])
    obstacle_times = [row['time'] for row in named_obstacles]

    obstacle_summary = {
        'named_dynamic_rows': len(named_obstacles),
        'all_obstacle_rows': len(obstacle_rows),
    }
    if named_obstacles:
        xs = [row['x'] for row in named_obstacles]
        ys = [row['y'] for row in named_obstacles]
        speeds = [math.hypot(row['vx'], row['vy']) for row in named_obstacles]
        obstacle_summary.update(
            {
                'name': named_obstacles[0]['name'],
                'x_range': [min(xs), max(xs)],
                'y_range': [min(ys), max(ys)],
                'speed_min': min(speeds),
                'speed_max': max(speeds),
                'speed_avg': _mean(speeds),
                'first_scene_time': named_obstacles[0]['scene_time'],
                'last_scene_time': named_obstacles[-1]['scene_time'],
            }
        )
        if obs_center is not None:
            cx, cy = obs_center
            radii = [math.hypot(row['x'] - cx, row['y'] - cy) for row in named_obstacles]
            obstacle_summary.update(
                {
                    'center_x': cx,
                    'center_y': cy,
                    'radius_from_center_min': min(radii),
                    'radius_from_center_max': max(radii),
                    'radius_from_center_avg': _mean(radii),
                    'radius_from_center_span': max(radii) - min(radii),
                }
            )

    avoid_by_drone = {}
    for row in avoid_rows:
        drone_id = int(_float(row.get('drone_id'), -1))
        speed = _float(row.get('speed_xy'))
        item = avoid_by_drone.setdefault(
            drone_id,
            {
                'rows': 0,
                'nonzero_rows': 0,
                'max_speed_xy': 0.0,
                'first_nonzero_time': None,
            },
        )
        item['rows'] += 1
        item['max_speed_xy'] = max(item['max_speed_xy'], speed)
        if speed > nonzero_threshold:
            item['nonzero_rows'] += 1
            if item['first_nonzero_time'] is None:
                item['first_nonzero_time'] = _float(row.get('time'))

    min_distance_by_drone = {}
    closest_rows = {}
    obstacle_radius_at_closest = {}
    for row in drone_rows:
        obs = _nearest_by_time(named_obstacles, obstacle_times, _float(row.get('time')))
        if obs is None:
            continue
        drone_id = int(_float(row.get('drone_id'), -1))
        world_x = _float(row.get('world_x', row.get('x')))
        world_y = _float(row.get('world_y', row.get('y')))
        z = _float(row.get('z'))
        dx = world_x - obs['x']
        dy = world_y - obs['y']
        dz = z - obs['z']
        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if drone_id not in min_distance_by_drone or dist_xy < min_distance_by_drone[drone_id]:
            min_distance_by_drone[drone_id] = dist_xy
            obstacle_radius_at_closest[drone_id] = obs['radius']
            closest_rows[drone_id] = {
                'time': _float(row.get('time')),
                'scene_time': obs['scene_time'],
                'drone_world_x': world_x,
                'drone_world_y': world_y,
                'drone_z': z,
                'obstacle_x': obs['x'],
                'obstacle_y': obs['y'],
                'obstacle_z': obs['z'],
                'distance_xy': dist_xy,
                'distance_3d': dist_3d,
            }

    contact_by_drone = {}
    for drone_id, dist_xy in min_distance_by_drone.items():
        threshold = influence_radius + obstacle_radius_at_closest.get(drone_id, 0.0)
        contact_by_drone[drone_id] = {
            'min_distance_xy': dist_xy,
            'influence_threshold': threshold,
            'entered_influence': dist_xy < threshold,
            'closest': closest_rows[drone_id],
        }

    any_nonzero_avoid = any(item['nonzero_rows'] > 0 for item in avoid_by_drone.values())
    any_entered_influence = any(item['entered_influence'] for item in contact_by_drone.values())
    result = {
        'run_dir': run_dir,
        'input_files': {
            'drones_csv': drones_path,
            'obstacles_csv': obstacles_path,
            'avoid_cmds_csv': avoid_path,
        },
        'row_counts': {
            'drones': len(drone_rows),
            'obstacles': len(obstacle_rows),
            'avoid_cmds': len(avoid_rows),
        },
        'obstacle_summary': obstacle_summary,
        'avoid_by_drone': {str(k): v for k, v in sorted(avoid_by_drone.items())},
        'distance_by_drone': {str(k): v for k, v in sorted(contact_by_drone.items())},
        'checks': {
            'has_named_dynamic_obstacle': len(named_obstacles) > 0,
            'any_drone_entered_obstacle_influence': any_entered_influence,
            'any_nonzero_avoid_cmd': any_nonzero_avoid,
            'nonzero_threshold': nonzero_threshold,
            'influence_radius': influence_radius,
        },
    }
    return result


def print_human(result):
    print(f'run_dir={result["run_dir"]}')
    counts = result['row_counts']
    print(
        'rows: '
        f'drones={counts["drones"]}, obstacles={counts["obstacles"]}, avoid_cmds={counts["avoid_cmds"]}'
    )

    obs = result['obstacle_summary']
    print(
        'dynamic_obstacle: '
        f'named_rows={obs.get("named_dynamic_rows", 0)}, '
        f'name={obs.get("name", "none")}'
    )
    if obs.get('named_dynamic_rows', 0) > 0:
        print(
            'obstacle_motion: '
            f'x_range={obs.get("x_range")}, y_range={obs.get("y_range")}, '
            f'speed_avg={obs.get("speed_avg", 0.0):.3f}, '
            f'scene_time={obs.get("first_scene_time", 0.0):.2f}->{obs.get("last_scene_time", 0.0):.2f}'
        )
        if 'radius_from_center_span' in obs:
            print(
                'obstacle_center_check: '
                f'radius_avg={obs["radius_from_center_avg"]:.3f}, '
                f'radius_span={obs["radius_from_center_span"]:.6f}'
            )

    print('distance_by_drone:')
    for drone_id, item in result['distance_by_drone'].items():
        closest = item['closest']
        print(
            f'  drone {drone_id}: min_xy={item["min_distance_xy"]:.3f}, '
            f'threshold={item["influence_threshold"]:.3f}, '
            f'entered_influence={item["entered_influence"]}, '
            f'time={closest["time"]:.3f}, scene_time={closest["scene_time"]:.3f}'
        )

    print('avoid_by_drone:')
    for drone_id, item in result['avoid_by_drone'].items():
        print(
            f'  drone {drone_id}: rows={item["rows"]}, '
            f'nonzero_rows={item["nonzero_rows"]}, '
            f'max_speed_xy={item["max_speed_xy"]:.3f}, '
            f'first_nonzero_time={item["first_nonzero_time"]}'
        )

    checks = result['checks']
    verdict = (
        checks['has_named_dynamic_obstacle']
        and checks['any_drone_entered_obstacle_influence']
        and checks['any_nonzero_avoid_cmd']
    )
    print('checks:')
    for key, value in checks.items():
        print(f'  {key}={value}')
    print('verdict=' + ('PASS: obstacle reached route and ORCA produced avoid output' if verdict else 'CHECK: inspect distances and avoid output'))


def main():
    parser = argparse.ArgumentParser(description='Analyze a ros_run_recorder output directory.')
    parser.add_argument('--run-dir', required=True, help='Directory containing drones.csv, obstacles_ros.csv, avoid_cmds.csv')
    parser.add_argument('--output-json', default='', help='Optional path for analysis_summary.json')
    parser.add_argument('--obstacle-center-x', type=float, default=None)
    parser.add_argument('--obstacle-center-y', type=float, default=None)
    parser.add_argument('--nonzero-threshold', type=float, default=0.01)
    parser.add_argument('--influence-radius', type=float, default=2.0)
    args = parser.parse_args()

    center = None
    if args.obstacle_center_x is not None and args.obstacle_center_y is not None:
        center = (args.obstacle_center_x, args.obstacle_center_y)

    result = analyze_run(
        args.run_dir,
        obs_center=center,
        nonzero_threshold=args.nonzero_threshold,
        influence_radius=args.influence_radius,
    )
    print_human(result)

    output_json = args.output_json
    if not output_json:
        output_json = os.path.join(os.path.abspath(os.path.expanduser(args.run_dir)), 'analysis_summary.json')
    try:
        with open(output_json, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, sort_keys=True)
            f.write('\n')
        print(f'summary={output_json}')
    except OSError as e:
        print(f'WARN: failed to write summary={output_json}: {e}')


if __name__ == '__main__':
    main()
