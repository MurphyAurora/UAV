#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import random
from datetime import datetime

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is expected in the ROS env.
    yaml = None


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def finite_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def vector3(value, default=(0.0, 0.0, 0.0)):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return tuple(float(v) for v in default)
    return tuple(finite_float(v) for v in value)


def load_scene(path):
    if yaml is None:
        raise RuntimeError('PyYAML is required to read scenario YAML files')
    scene_path = resolve_path(path)
    with open(scene_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'scene config must be a mapping: {scene_path}')
    return data


def load_suite(path):
    if yaml is None:
        raise RuntimeError('PyYAML is required to read suite YAML files')
    suite_path = resolve_path(path)
    with open(suite_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'suite config must be a mapping: {suite_path}')
    scenarios = data.get('scenarios') or []
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError(f'suite must contain a non-empty scenarios list: {suite_path}')
    return data


def resolve_path(path):
    expanded = os.path.expanduser(path)
    if os.path.isabs(expanded):
        return expanded
    if os.path.exists(expanded):
        return expanded
    return os.path.join(SCRIPT_DIR, expanded)


def line_state(traj, t):
    start_time = finite_float(traj.get('start_time'), 0.0)
    start = vector3(traj.get('start'))
    end = vector3(traj.get('end'), start)
    speed = max(0.0, finite_float(traj.get('speed'), 0.0))
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist <= 1e-9 or speed <= 1e-9 or t <= start_time:
        return (*start, 0.0, 0.0, 0.0)
    alpha = min(1.0, (t - start_time) * speed / dist)
    x = start[0] + dx * alpha
    y = start[1] + dy * alpha
    z = start[2] + dz * alpha
    if alpha >= 1.0:
        return (x, y, z, 0.0, 0.0, 0.0)
    return (x, y, z, dx / dist * speed, dy / dist * speed, dz / dist * speed)


def circle_state(traj, t):
    start_time = finite_float(traj.get('start_time'), 0.0)
    center = vector3(traj.get('center'))
    radius = max(0.0, finite_float(traj.get('radius'), 1.0))
    omega = finite_float(traj.get('angular_speed'), 0.0)
    phase = finite_float(traj.get('phase'), 0.0)
    active_t = max(0.0, t - start_time)
    theta = phase + omega * active_t
    x = center[0] + radius * math.cos(theta)
    y = center[1] + radius * math.sin(theta)
    z = center[2]
    if t < start_time:
        return (x, y, z, 0.0, 0.0, 0.0)
    vx = -radius * omega * math.sin(theta)
    vy = radius * omega * math.cos(theta)
    return (x, y, z, vx, vy, 0.0)


def random_walk_states(traj, duration, dt, rng):
    start = vector3(traj.get('start'))
    speed = max(0.0, finite_float(traj.get('speed'), 0.5))
    turn_std = max(0.0, finite_float(traj.get('turn_std'), 0.6))
    heading = finite_float(traj.get('heading'), rng.uniform(-math.pi, math.pi))
    x, y, z = start
    states = {}
    steps = int(round(duration / dt))
    for idx in range(steps + 1):
        t = round(idx * dt, 10)
        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)
        states[t] = (x, y, z, vx, vy, 0.0)
        heading += rng.gauss(0.0, turn_std) * math.sqrt(max(dt, 1e-9))
        x += vx * dt
        y += vy * dt
    return states


def generate_rows(scene):
    meta = scene.get('scene', {})
    seed = int(finite_float(meta.get('seed'), 0))
    duration = max(0.0, finite_float(meta.get('duration'), 30.0))
    dt = max(0.05, finite_float(meta.get('dt'), 0.5))
    rng = random.Random(seed)
    dynamic_obstacles = scene.get('dynamic_obstacles') or []
    random_cache = {}
    rows = []
    steps = int(round(duration / dt))

    for idx in range(steps + 1):
        t = round(idx * dt, 10)
        for obs in dynamic_obstacles:
            if not isinstance(obs, dict):
                continue
            traj = obs.get('trajectory') or {}
            traj_type = str(traj.get('type', 'line')).strip().lower()
            if traj_type == 'circle':
                state = circle_state(traj, t)
            elif traj_type == 'random_walk':
                name = str(obs.get('name', f'obstacle_{len(random_cache) + 1}'))
                if name not in random_cache:
                    random_cache[name] = random_walk_states(traj, duration, dt, rng)
                state = random_cache[name].get(t, vector3(traj.get('start')) + (0.0, 0.0, 0.0))
            else:
                state = line_state(traj, t)
            rows.append(
                {
                    'time': f'{t:.3f}',
                    'name': str(obs.get('name', 'dynamic_obstacle')),
                    'type': str(obs.get('type', 'sphere')),
                    'x': f'{state[0]:.4f}',
                    'y': f'{state[1]:.4f}',
                    'z': f'{state[2]:.4f}',
                    'vx': f'{state[3]:.4f}',
                    'vy': f'{state[4]:.4f}',
                    'vz': f'{state[5]:.4f}',
                    'radius': f'{finite_float(obs.get("radius"), 0.5):.4f}',
                }
            )
    return rows


def distance3(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


def distance_xy_to_segment(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)
    alpha = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    closest_x = ax + alpha * dx
    closest_y = ay + alpha * dy
    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


def rows_by_obstacle(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row['name'], []).append(row)
    for values in grouped.values():
        values.sort(key=lambda row: finite_float(row['time']))
    return grouped


def obstacle_metrics(rows):
    metrics = {}
    for name, obs_rows in rows_by_obstacle(rows).items():
        path_length = 0.0
        speeds = []
        prev = None
        active_samples = 0
        for row in obs_rows:
            pos = (
                finite_float(row['x']),
                finite_float(row['y']),
                finite_float(row['z']),
            )
            speed = math.sqrt(
                finite_float(row['vx']) ** 2
                + finite_float(row['vy']) ** 2
                + finite_float(row['vz']) ** 2
            )
            speeds.append(speed)
            if speed > 1e-6:
                active_samples += 1
            if prev is not None:
                path_length += distance3(pos, prev)
            prev = pos
        metrics[name] = {
            'sample_count': len(obs_rows),
            'path_length': path_length,
            'average_speed': sum(speeds) / len(speeds) if speeds else 0.0,
            'max_speed': max(speeds) if speeds else 0.0,
            'active_sample_count': active_samples,
        }
    return metrics


def route_interaction_metrics(scene, rows):
    target = scene.get('target') or {}
    route_start = (0.0, 0.0)
    route_end = (
        finite_float(target.get('x'), 0.0),
        finite_float(target.get('y'), 0.0),
    )
    clearance_margin = 1.0
    total_events = 0
    min_clearance = None
    per_obstacle = {}
    for name, obs_rows in rows_by_obstacle(rows).items():
        inside = False
        events = 0
        obs_min = None
        for row in obs_rows:
            radius = finite_float(row['radius'], 0.5)
            dist = distance_xy_to_segment(
                (finite_float(row['x']), finite_float(row['y'])),
                route_start,
                route_end,
            )
            clearance = dist - radius
            obs_min = clearance if obs_min is None else min(obs_min, clearance)
            min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
            near_route = clearance <= clearance_margin
            if near_route and not inside:
                events += 1
            inside = near_route
        per_obstacle[name] = {
            'near_route_events': events,
            'min_route_clearance': obs_min if obs_min is not None else 0.0,
        }
        total_events += events
    return {
        'reference_route': {
            'start_xy': list(route_start),
            'target_xy': list(route_end),
            'clearance_margin': clearance_margin,
        },
        'near_route_event_count': total_events,
        'min_route_clearance': min_clearance if min_clearance is not None else 0.0,
        'per_obstacle': per_obstacle,
    }


def difficulty_score(metrics):
    obstacle_count = metrics['dynamic_obstacle_count']
    max_speed = metrics['max_obstacle_speed']
    route_events = metrics['route_interactions']['near_route_event_count']
    min_clearance = metrics['route_interactions']['min_route_clearance']
    clearance_pressure = max(0.0, 2.0 - min_clearance)
    raw_score = (
        obstacle_count * 8.0
        + max_speed * 12.0
        + route_events * 6.0
        + clearance_pressure * 5.0
    )
    return max(0.0, min(100.0, raw_score))


def round_nested(value, digits=4):
    if isinstance(value, float):
        return round(value, digits)
    if isinstance(value, list):
        return [round_nested(item, digits) for item in value]
    if isinstance(value, dict):
        return {key: round_nested(item, digits) for key, item in value.items()}
    return value


def scene_metrics(scene, rows):
    speeds = [
        math.sqrt(finite_float(row['vx']) ** 2 + finite_float(row['vy']) ** 2 + finite_float(row['vz']) ** 2)
        for row in rows
    ]
    names = sorted({row['name'] for row in rows})
    metrics = {
        'dynamic_obstacle_count': len(names),
        'sample_count': len(rows),
        'max_obstacle_speed': max(speeds) if speeds else 0.0,
        'average_obstacle_speed': sum(speeds) / len(speeds) if speeds else 0.0,
        'obstacles': obstacle_metrics(rows),
        'route_interactions': route_interaction_metrics(scene, rows),
    }
    metrics['difficulty_score'] = difficulty_score(metrics)
    return round_nested(metrics)


def summarize(scene, rows):
    meta = scene.get('scene', {})
    metrics = scene_metrics(scene, rows)
    names = sorted({row['name'] for row in rows})
    return {
        'scene': meta.get('name', 'unnamed_scene'),
        'version': meta.get('version', 1),
        'seed': int(finite_float(meta.get('seed'), 0)),
        'level': meta.get('level', 'unknown'),
        'duration': finite_float(meta.get('duration'), 30.0),
        'dt': finite_float(meta.get('dt'), 0.5),
        'dynamic_obstacle_count': len(names),
        'dynamic_obstacles': names,
        'sample_count': len(rows),
        'max_obstacle_speed': metrics['max_obstacle_speed'],
        'difficulty_score': metrics['difficulty_score'],
        'difficulty': scene.get('difficulty', {}),
        'metrics': metrics,
    }


def write_outputs(scene, rows, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, 'obstacles.csv')
    summary_path = os.path.join(output_dir, 'summary.json')
    resolved_path = os.path.join(output_dir, 'resolved_scene.json')
    fieldnames = ['time', 'name', 'type', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'radius']
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(summarize(scene, rows), f, indent=2, sort_keys=True)
        f.write('\n')
    with open(resolved_path, 'w', encoding='utf-8') as f:
        json.dump(scene, f, indent=2, sort_keys=True)
        f.write('\n')
    return csv_path, summary_path, resolved_path


def suite_row(summary, scene_path, output_dir):
    route = summary['metrics']['route_interactions']
    return {
        'scene': summary['scene'],
        'level': summary['level'],
        'seed': summary['seed'],
        'duration': summary['duration'],
        'dt': summary['dt'],
        'dynamic_obstacle_count': summary['dynamic_obstacle_count'],
        'sample_count': summary['sample_count'],
        'max_obstacle_speed': summary['max_obstacle_speed'],
        'average_obstacle_speed': summary['metrics']['average_obstacle_speed'],
        'near_route_event_count': route['near_route_event_count'],
        'min_route_clearance': route['min_route_clearance'],
        'difficulty_score': summary['difficulty_score'],
        'scene_path': scene_path,
        'output_dir': output_dir,
    }


def write_suite_outputs(suite, rows, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, 'suite_summary.csv')
    json_path = os.path.join(output_dir, 'suite_summary.json')
    suite_path = os.path.join(output_dir, 'resolved_suite.json')
    fieldnames = [
        'scene',
        'level',
        'seed',
        'duration',
        'dt',
        'dynamic_obstacle_count',
        'sample_count',
        'max_obstacle_speed',
        'average_obstacle_speed',
        'near_route_event_count',
        'min_route_clearance',
        'difficulty_score',
        'scene_path',
        'output_dir',
    ]
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump({'suite': suite.get('suite', {}), 'scenarios': rows}, f, indent=2, sort_keys=True)
        f.write('\n')
    with open(suite_path, 'w', encoding='utf-8') as f:
        json.dump(suite, f, indent=2, sort_keys=True)
        f.write('\n')
    return csv_path, json_path, suite_path


def default_output_dir(scene):
    meta = scene.get('scene', {})
    name = str(meta.get('name', 'scene')).strip().replace(' ', '_')
    seed = int(finite_float(meta.get('seed'), 0))
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join('runs', f'{stamp}_{name}_seed{seed}')


def default_suite_output_dir(suite):
    meta = suite.get('suite', {})
    name = str(meta.get('name', 'suite')).strip().replace(' ', '_')
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join('runs', f'{stamp}_{name}')


def run_single_scene(scene_path, output_dir=''):
    scene = load_scene(scene_path)
    rows = generate_rows(scene)
    resolved_output_dir = output_dir or default_output_dir(scene)
    csv_path, summary_path, resolved_path = write_outputs(scene, rows, resolved_output_dir)
    summary = summarize(scene, rows)
    return summary, csv_path, summary_path, resolved_path, resolved_output_dir


def print_scene_result(summary, csv_path, summary_path, resolved_path):
    print(f"scene={summary['scene']} seed={summary['seed']} level={summary['level']}")
    print(f"dynamic_obstacles={summary['dynamic_obstacle_count']} samples={summary['sample_count']}")
    print(f'max_obstacle_speed={summary["max_obstacle_speed"]:.3f}')
    print(f'difficulty_score={summary["difficulty_score"]:.1f}')
    print(f'csv={csv_path}')
    print(f'summary={summary_path}')
    print(f'resolved_scene={resolved_path}')


def run_suite(suite_path, output_dir=''):
    suite = load_suite(suite_path)
    resolved_output_dir = output_dir or default_suite_output_dir(suite)
    os.makedirs(resolved_output_dir, exist_ok=True)
    summary_rows = []
    for scene_path in suite.get('scenarios') or []:
        scene = load_scene(scene_path)
        scene_name = str((scene.get('scene') or {}).get('name', 'scene')).strip().replace(' ', '_')
        seed = int(finite_float((scene.get('scene') or {}).get('seed'), 0))
        scene_output_dir = os.path.join(resolved_output_dir, f'{scene_name}_seed{seed}')
        rows = generate_rows(scene)
        write_outputs(scene, rows, scene_output_dir)
        summary = summarize(scene, rows)
        summary_rows.append(suite_row(summary, scene_path, scene_output_dir))
        print(
            f"{summary['scene']}: level={summary['level']} seed={summary['seed']} "
            f"score={summary['difficulty_score']:.1f}"
        )
    csv_path, json_path, resolved_path = write_suite_outputs(suite, summary_rows, resolved_output_dir)
    return summary_rows, csv_path, json_path, resolved_path, resolved_output_dir


def main():
    parser = argparse.ArgumentParser(description='Generate a reproducible dynamic-obstacle demo trace.')
    parser.add_argument('--scene', default='scenes/demo_dynamic.yaml')
    parser.add_argument('--suite', default='')
    parser.add_argument('--output-dir', default='')
    args = parser.parse_args()

    if args.suite:
        rows, csv_path, json_path, resolved_path, output_dir = run_suite(args.suite, args.output_dir)
        print(f'suite_scenarios={len(rows)}')
        print(f'suite_summary_csv={csv_path}')
        print(f'suite_summary_json={json_path}')
        print(f'resolved_suite={resolved_path}')
        print(f'output_dir={output_dir}')
        return

    summary, csv_path, summary_path, resolved_path, _ = run_single_scene(args.scene, args.output_dir)
    print_scene_result(summary, csv_path, summary_path, resolved_path)


if __name__ == '__main__':
    main()
