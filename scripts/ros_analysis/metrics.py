#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算法运行级评价指标。

输入：
    ros_run_recorder.py 记录的 drones / obstacles / avoid_cmds 行数据。
输出：
    路径长度、任务时间、物理碰撞、安全裕度违规、避障成功率、
    控制平滑度和仿真实时率等指标。

这个模块只做离线分析，不影响 ROS 记录器和控制算法。
"""

import csv
import math
import os


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def group_sorted(rows, key):
    grouped = {}
    for row in rows:
        grouped.setdefault(row[key], []).append(row)
    return {k: sorted(v, key=lambda item: item.get('time', 0.0)) for k, v in sorted(grouped.items())}


def _path_metrics(drone_rows):
    by_drone = group_sorted(drone_rows, 'drone_id')
    result = {}
    for drone_id, rows in by_drone.items():
        if not rows:
            continue
        path_xy = 0.0
        path_3d = 0.0
        speed_xy_values = []
        for prev, cur in zip(rows[:-1], rows[1:]):
            dx = cur['world_x'] - prev['world_x']
            dy = cur['world_y'] - prev['world_y']
            dz = cur['z'] - prev['z']
            path_xy += math.hypot(dx, dy)
            path_3d += math.sqrt(dx * dx + dy * dy + dz * dz)
            speed_xy_values.append(math.hypot(cur.get('vx', 0.0), cur.get('vy', 0.0)))
        duration = max(0.0, rows[-1]['time'] - rows[0]['time'])
        result[str(drone_id)] = {
            'samples': len(rows),
            'start_time': rows[0]['time'],
            'end_time': rows[-1]['time'],
            'duration_sec': duration,
            'path_length_xy': path_xy,
            'path_length_3d': path_3d,
            'avg_speed_xy': path_xy / duration if duration > 1e-9 else 0.0,
            'state_speed_xy_avg': mean(speed_xy_values),
            'state_speed_xy_max': max(speed_xy_values) if speed_xy_values else 0.0,
        }
    return result


def _target_for_drone(target, drone_id, target_leader_id=None, target_y_spacing=None):
    tx, ty, tz = target
    if target_leader_id is None or target_y_spacing is None:
        return tx, ty, tz
    return tx, ty + (int(drone_id) - int(target_leader_id)) * float(target_y_spacing), tz


def _task_metrics(
    drone_rows,
    target=None,
    target_radius=1.0,
    target_leader_id=None,
    target_y_spacing=None,
    physical_collision_times=None,
):
    if not drone_rows:
        return {
            'recording_duration_sec': 0.0,
            'target_configured': False,
            'completion_rate': 0.0,
            'mission_time_sec': None,
            'completion_by_drone': {},
        }
    start_time = min(row['time'] for row in drone_rows)
    end_time = max(row['time'] for row in drone_rows)
    by_drone = group_sorted(drone_rows, 'drone_id')
    if target is None:
        return {
            'recording_duration_sec': end_time - start_time,
            'target_configured': False,
            'completion_rate': None,
            'mission_time_sec': end_time - start_time,
            'completion_by_drone': {},
        }

    tx, ty, tz = target
    use_formation_slots = target_leader_id is not None and target_y_spacing is not None
    physical_collision_times = physical_collision_times or {}
    completion = {}
    for drone_id, rows in by_drone.items():
        slot_x, slot_y, slot_z = _target_for_drone(
            target,
            drone_id,
            target_leader_id=target_leader_id,
            target_y_spacing=target_y_spacing,
        )
        hit_time = None
        min_target_distance = None
        collision_time = physical_collision_times.get(str(drone_id))
        for row in rows:
            if collision_time is not None and row['time'] > collision_time:
                break
            dx = row['world_x'] - slot_x
            dy = row['world_y'] - slot_y
            dz = row['z'] - slot_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            min_target_distance = dist if min_target_distance is None else min(min_target_distance, dist)
            if dist <= target_radius and hit_time is None:
                hit_time = row['time'] - start_time
        completion[str(drone_id)] = {
            'completed': hit_time is not None,
            'completion_time_sec': hit_time,
            'min_target_distance': min_target_distance if min_target_distance is not None else 0.0,
            'invalid_after_physical_collision': collision_time is not None,
            'first_physical_collision_time': collision_time,
            'target_slot': {
                'x': slot_x,
                'y': slot_y,
                'z': slot_z,
                'radius': target_radius,
            },
        }
    completed = [item for item in completion.values() if item['completed']]
    return {
        'recording_duration_sec': end_time - start_time,
        'target_configured': True,
        'target': {
            'x': tx,
            'y': ty,
            'z': tz,
            'radius': target_radius,
            'mode': 'formation_slots' if use_formation_slots else 'single_point',
            'leader_id': target_leader_id,
            'y_spacing': target_y_spacing,
        },
        'completion_rate': len(completed) / max(1, len(completion)),
        'mission_time_sec': max(item['completion_time_sec'] for item in completed) if len(completed) == len(completion) else None,
        'completion_by_drone': completion,
    }


def _nearest_obstacle_group(named_obstacles, obstacle_times, t):
    if not named_obstacles:
        return []
    nearest = min(named_obstacles, key=lambda row: abs(row['time'] - t))
    nearest_time = nearest['time']
    return [row for row in named_obstacles if abs(row['time'] - nearest_time) <= 1e-9]


def _safety_metrics(
    drone_rows,
    named_obstacles,
    drone_radius=0.35,
    safe_margin=0.0,
    near_miss_margin=0.5,
    influence_radius=2.0,
):
    obstacle_times = [row['time'] for row in named_obstacles]
    by_drone = group_sorted(drone_rows, 'drone_id')
    result = {}
    global_min_clearance = None
    global_min_physical_clearance = None
    total_collision_samples = 0
    total_safety_violation_samples = 0
    total_near_miss_samples = 0
    total_influence_samples = 0

    for drone_id, rows in by_drone.items():
        min_clearance = None
        min_physical_clearance = None
        min_distance_3d = None
        min_distance_xy = None
        physical_collision_samples = 0
        collision_samples = 0
        safety_violation_samples = 0
        near_miss_samples = 0
        influence_samples = 0
        closest = None
        prev_physical_collision = False
        prev_collision = False
        prev_safety_violation = False
        physical_collision_events = 0
        collision_events = 0
        safety_violation_events = 0
        first_physical_collision_time = None
        for row in rows:
            candidates = _nearest_obstacle_group(named_obstacles, obstacle_times, row['time'])
            if not candidates:
                continue
            best = None
            for obs in candidates:
                dx = row['world_x'] - obs['x']
                dy = row['world_y'] - obs['y']
                dz = row['z'] - obs['z']
                dist_xy = math.hypot(dx, dy)
                dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
                physical_clearance = dist_3d - (drone_radius + obs['radius'])
                clearance = physical_clearance - safe_margin
                item = (clearance, physical_clearance, dist_3d, dist_xy, obs)
                if best is None or item[0] < best[0]:
                    best = item
            if best is None:
                continue
            clearance, physical_clearance, dist_3d, dist_xy, obs = best
            min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
            min_physical_clearance = (
                physical_clearance if min_physical_clearance is None else min(min_physical_clearance, physical_clearance)
            )
            min_distance_3d = dist_3d if min_distance_3d is None else min(min_distance_3d, dist_3d)
            min_distance_xy = dist_xy if min_distance_xy is None else min(min_distance_xy, dist_xy)
            global_min_clearance = clearance if global_min_clearance is None else min(global_min_clearance, clearance)
            global_min_physical_clearance = (
                physical_clearance
                if global_min_physical_clearance is None
                else min(global_min_physical_clearance, physical_clearance)
            )
            in_physical_collision = physical_clearance <= 0.0
            in_safety_violation = clearance <= 0.0
            if in_physical_collision:
                physical_collision_samples += 1
                if not prev_physical_collision:
                    physical_collision_events += 1
                    if first_physical_collision_time is None:
                        first_physical_collision_time = row['time']
            if in_safety_violation:
                safety_violation_samples += 1
                collision_samples += 1
                if not prev_safety_violation:
                    safety_violation_events += 1
                if not prev_collision:
                    collision_events += 1
            if clearance <= near_miss_margin:
                near_miss_samples += 1
            if clearance <= influence_radius:
                influence_samples += 1
            prev_physical_collision = in_physical_collision
            prev_safety_violation = in_safety_violation
            prev_collision = in_safety_violation
            if closest is None or clearance < closest['safety_clearance']:
                closest = {
                    'time': row['time'],
                    'scene_time': obs.get('scene_time', -1.0),
                    'drone_world_x': row['world_x'],
                    'drone_world_y': row['world_y'],
                    'drone_z': row['z'],
                    'obstacle_id': obs.get('name', ''),
                    'obstacle_x': obs['x'],
                    'obstacle_y': obs['y'],
                    'obstacle_z': obs['z'],
                    'obstacle_radius': obs['radius'],
                    'distance_xy': dist_xy,
                    'distance_3d': dist_3d,
                    'physical_clearance': physical_clearance,
                    'safety_clearance': clearance,
                }
        total_collision_samples += physical_collision_samples
        total_safety_violation_samples += safety_violation_samples
        total_near_miss_samples += near_miss_samples
        total_influence_samples += influence_samples
        result[str(drone_id)] = {
            'samples': len(rows),
            'min_distance_xy': min_distance_xy,
            'min_distance_3d': min_distance_3d,
            'min_physical_clearance': min_physical_clearance,
            'min_safety_clearance': min_clearance,
            'physical_collision_samples': physical_collision_samples,
            'physical_collision_events': physical_collision_events,
            'first_physical_collision_time': first_physical_collision_time,
            'safety_violation_samples': safety_violation_samples,
            'safety_violation_events': safety_violation_events,
            # Backward-compatible aliases. In new reports prefer physical_collision_* and safety_violation_*.
            'collision_samples': physical_collision_samples,
            'collision_events': physical_collision_events,
            'near_miss_samples': near_miss_samples,
            'influence_samples': influence_samples,
            'physical_collision_rate_samples': physical_collision_samples / max(1, len(rows)),
            'safety_violation_rate_samples': safety_violation_samples / max(1, len(rows)),
            'collision_rate_samples': physical_collision_samples / max(1, len(rows)),
            'near_miss_rate_samples': near_miss_samples / max(1, len(rows)),
            'closest': closest,
        }

    return {
        'drone_radius': drone_radius,
        'safe_margin': safe_margin,
        'near_miss_margin': near_miss_margin,
        'influence_radius': influence_radius,
        'physical_collision_threshold_note': 'physical_collision when distance_3d <= drone_radius + obstacle_radius',
        'safety_violation_threshold_note': 'safety_violation when distance_3d <= drone_radius + obstacle_radius + safe_margin',
        'global_min_physical_clearance': global_min_physical_clearance,
        'global_min_safety_clearance': global_min_clearance,
        'physical_collision_samples_total': total_collision_samples,
        'physical_collision_events_total': sum(item['physical_collision_events'] for item in result.values()),
        'physical_collision_drone_count': sum(1 for item in result.values() if item['physical_collision_events'] > 0),
        'safety_violation_samples_total': total_safety_violation_samples,
        'safety_violation_events_total': sum(item['safety_violation_events'] for item in result.values()),
        'safety_violation_drone_count': sum(1 for item in result.values() if item['safety_violation_events'] > 0),
        # Backward-compatible aliases. These now mean physical collision.
        'collision_samples_total': total_collision_samples,
        'collision_events_total': sum(item['physical_collision_events'] for item in result.values()),
        'near_miss_samples_total': total_near_miss_samples,
        'influence_samples_total': total_influence_samples,
        'by_drone': result,
    }


def _control_smoothness(avoid_rows, nonzero_threshold=0.01):
    by_drone = group_sorted(avoid_rows, 'drone_id')
    result = {}
    for drone_id, rows in by_drone.items():
        if not rows:
            continue
        delta_norms = []
        accel_norms = []
        jerk_norms = []
        prev_accel = None
        prev = None
        for row in rows:
            if prev is not None:
                dt = max(1e-9, row['time'] - prev['time'])
                dvx = row['vx'] - prev['vx']
                dvy = row['vy'] - prev['vy']
                dvz = row['vz'] - prev['vz']
                delta = math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz)
                accel = delta / dt
                delta_norms.append(delta)
                accel_norms.append(accel)
                if prev_accel is not None:
                    jerk_norms.append(abs(accel - prev_accel) / dt)
                prev_accel = accel
            prev = row
        nonzero_rows = sum(1 for row in rows if row['speed_xy'] > nonzero_threshold)
        result[str(drone_id)] = {
            'samples': len(rows),
            'nonzero_rows': nonzero_rows,
            'nonzero_ratio': nonzero_rows / max(1, len(rows)),
            'max_speed_xy': max((row['speed_xy'] for row in rows), default=0.0),
            'mean_speed_xy': mean([row['speed_xy'] for row in rows]),
            'cmd_delta_rms': rms(delta_norms),
            'cmd_delta_max': max(delta_norms) if delta_norms else 0.0,
            'cmd_accel_rms': rms(accel_norms),
            'cmd_accel_max': max(accel_norms) if accel_norms else 0.0,
            'cmd_jerk_rms': rms(jerk_norms),
            'cmd_jerk_max': max(jerk_norms) if jerk_norms else 0.0,
        }
    return result


def _real_time_factor(obstacle_rows):
    scene_rows = [row for row in obstacle_rows if row.get('scene_time', -1.0) >= 0.0]
    if len(scene_rows) < 2:
        return {
            'available': False,
            'real_time_factor': None,
            'wall_duration_sec': None,
            'scene_duration_sec': None,
        }
    scene_rows.sort(key=lambda row: row['time'])
    wall_duration = scene_rows[-1]['time'] - scene_rows[0]['time']
    scene_duration = scene_rows[-1]['scene_time'] - scene_rows[0]['scene_time']
    return {
        'available': wall_duration > 1e-9,
        'real_time_factor': scene_duration / wall_duration if wall_duration > 1e-9 else None,
        'wall_duration_sec': wall_duration,
        'scene_duration_sec': scene_duration,
    }


def _avoid_success_metrics(safety_metrics, smoothness_by_drone):
    by_drone = {}
    required = 0
    successful = 0
    for drone_id, safety in safety_metrics['by_drone'].items():
        required_avoid = safety['influence_samples'] > 0
        has_physical_collision = safety['physical_collision_events'] > 0
        has_safety_violation = safety['safety_violation_events'] > 0
        has_nonzero_cmd = smoothness_by_drone.get(drone_id, {}).get('nonzero_rows', 0) > 0
        success = (not required_avoid) or (has_nonzero_cmd and not has_safety_violation)
        if required_avoid:
            required += 1
            successful += 1 if success else 0
        by_drone[drone_id] = {
            'required_avoidance': required_avoid,
            'has_nonzero_avoid_cmd': has_nonzero_cmd,
            'physical_collision_free': not has_physical_collision,
            'safety_violation_free': not has_safety_violation,
            # Backward-compatible alias. Prefer physical_collision_free / safety_violation_free.
            'collision_free': not has_physical_collision,
            'avoidance_success': success,
        }
    return {
        'definition': 'For drones entering the obstacle influence zone, success requires nonzero avoid command and zero safety_violation events.',
        'required_drone_count': required,
        'successful_drone_count': successful,
        'avoidance_success_rate': successful / required if required > 0 else None,
        'by_drone': by_drone,
    }


def parse_drone_rows(drone_rows):
    parsed = []
    for row in drone_rows:
        parsed.append(
            {
                'time': safe_float(row.get('time')),
                'source_stamp': safe_float(row.get('source_stamp')),
                'drone_id': int(safe_float(row.get('drone_id'), -1)),
                'x': safe_float(row.get('x')),
                'y': safe_float(row.get('y')),
                'z': safe_float(row.get('z')),
                'world_x': safe_float(row.get('world_x', row.get('x'))),
                'world_y': safe_float(row.get('world_y', row.get('y'))),
                'vx': safe_float(row.get('vx')),
                'vy': safe_float(row.get('vy')),
                'vz': safe_float(row.get('vz')),
                'heading': safe_float(row.get('heading')),
            }
        )
    return sorted(parsed, key=lambda row: (row['drone_id'], row['time']))


def parse_avoid_rows(avoid_rows):
    parsed = []
    for row in avoid_rows:
        parsed.append(
            {
                'time': safe_float(row.get('time')),
                'drone_id': int(safe_float(row.get('drone_id'), -1)),
                'vx': safe_float(row.get('vx')),
                'vy': safe_float(row.get('vy')),
                'vz': safe_float(row.get('vz')),
                'speed_xy': safe_float(row.get('speed_xy')),
            }
        )
    return sorted(parsed, key=lambda row: (row['drone_id'], row['time']))


def algorithm_metrics(
    drone_rows,
    obstacle_rows,
    avoid_rows,
    named_obstacles,
    nonzero_threshold,
    influence_radius,
    drone_radius,
    safe_margin,
    near_miss_margin,
    target=None,
    target_radius=1.0,
    target_leader_id=None,
    target_y_spacing=None,
):
    parsed_drones = parse_drone_rows(drone_rows)
    parsed_avoid = parse_avoid_rows(avoid_rows)
    safety = _safety_metrics(
        parsed_drones,
        named_obstacles,
        drone_radius=drone_radius,
        safe_margin=safe_margin,
        near_miss_margin=near_miss_margin,
        influence_radius=influence_radius,
    )
    smoothness = _control_smoothness(parsed_avoid, nonzero_threshold=nonzero_threshold)
    avoid_success = _avoid_success_metrics(safety, smoothness)
    path = _path_metrics(parsed_drones)
    physical_collision_times = {
        drone_id: item.get('first_physical_collision_time')
        for drone_id, item in safety.get('by_drone', {}).items()
        if item.get('first_physical_collision_time') is not None
    }
    task = _task_metrics(
        parsed_drones,
        target=target,
        target_radius=target_radius,
        target_leader_id=target_leader_id,
        target_y_spacing=target_y_spacing,
        physical_collision_times=physical_collision_times,
    )
    rtf = _real_time_factor(obstacle_rows)
    return {
        'definitions': {
            'path_length': 'Sum of consecutive drone world-position distances.',
            'task_time': 'If target is configured, max completion time among non-collided drones; otherwise recording duration.',
            'physical_collision': '3D distance <= drone_radius + obstacle_radius. A physical collision invalidates later task completion.',
            'safety_violation': '3D distance <= drone_radius + obstacle_radius + safe_margin.',
            'min_physical_clearance': 'Minimum clearance after subtracting drone radius and obstacle radius.',
            'min_safety_clearance': 'Minimum clearance after subtracting drone radius, obstacle radius, and safe margin.',
            'avoidance_success_rate': 'Among drones entering influence zone, fraction with nonzero avoid command and no safety_violation.',
            'control_smoothness': 'RMS/max changes of avoid_cmd velocity, acceleration-like delta/dt, and jerk-like difference.',
            'real_time_factor': 'scene_time duration / wall-clock recording duration from obstacle messages.',
        },
        'path': path,
        'task': task,
        'safety': safety,
        'avoidance_success': avoid_success,
        'control_smoothness': smoothness,
        'real_time_factor': rtf,
    }


def write_algorithm_metrics_csv(result, output_csv):
    metrics = result.get('algorithm_metrics', {})
    task = metrics.get('task', {})
    safety = metrics.get('safety', {})
    avoid = metrics.get('avoidance_success', {})
    rtf = metrics.get('real_time_factor', {})
    path_by_drone = metrics.get('path', {})
    smooth_by_drone = metrics.get('control_smoothness', {})

    path_lengths = [item.get('path_length_3d', 0.0) for item in path_by_drone.values()]
    path_lengths_xy = [item.get('path_length_xy', 0.0) for item in path_by_drone.values()]
    smooth_accel = [item.get('cmd_accel_rms', 0.0) for item in smooth_by_drone.values()]
    smooth_jerk = [item.get('cmd_jerk_rms', 0.0) for item in smooth_by_drone.values()]

    row = {
        'run_dir': result.get('run_dir', ''),
        'recording_duration_sec': task.get('recording_duration_sec'),
        'mission_time_sec': task.get('mission_time_sec'),
        'completion_rate': task.get('completion_rate'),
        'path_length_3d_mean': mean(path_lengths),
        'path_length_3d_total': sum(path_lengths),
        'path_length_xy_mean': mean(path_lengths_xy),
        'path_length_xy_total': sum(path_lengths_xy),
        'physical_collision_events_total': safety.get('physical_collision_events_total'),
        'physical_collision_samples_total': safety.get('physical_collision_samples_total'),
        'physical_collision_drone_count': safety.get('physical_collision_drone_count'),
        'global_min_physical_clearance': safety.get('global_min_physical_clearance'),
        'safety_violation_events_total': safety.get('safety_violation_events_total'),
        'safety_violation_samples_total': safety.get('safety_violation_samples_total'),
        'safety_violation_drone_count': safety.get('safety_violation_drone_count'),
        'global_min_safety_clearance': safety.get('global_min_safety_clearance'),
        # Backward-compatible aliases. These now refer to physical collision.
        'collision_events_total': safety.get('collision_events_total'),
        'collision_samples_total': safety.get('collision_samples_total'),
        'near_miss_samples_total': safety.get('near_miss_samples_total'),
        'avoidance_success_rate': avoid.get('avoidance_success_rate'),
        'required_avoidance_drone_count': avoid.get('required_drone_count'),
        'successful_avoidance_drone_count': avoid.get('successful_drone_count'),
        'control_accel_rms_mean': mean(smooth_accel),
        'control_jerk_rms_mean': mean(smooth_jerk),
        'real_time_factor': rtf.get('real_time_factor'),
        'scene_duration_sec': rtf.get('scene_duration_sec'),
        'wall_duration_sec': rtf.get('wall_duration_sec'),
    }
    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)
