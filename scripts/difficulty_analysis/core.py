#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Formation Survivability 核心计算逻辑。

这里放两套入口共同使用的算法部分：
    采样中心 + 编队偏移 + 动态障碍轨迹 -> S_formation / D_formation。

入口脚本只负责把不同输入格式转换成这里需要的通用格式。
"""

import json
import os

import numpy as np
import pandas as pd
import yaml


def load_yaml_config(path):
    """读取 YAML 配置文件。"""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'config must be a YAML mapping: {path}')
    return data


def safe_float(value, default=0.0):
    """把任意值安全转换为 float。"""
    try:
        return float(value)
    except Exception:
        return float(default)


def axis_values(low, high, step):
    """生成包含边界的等间隔采样轴。"""
    step = max(float(step), 1e-9)
    values = np.arange(float(low), float(high) + 0.5 * step, step)
    if values.size == 0 or abs(values[-1] - float(high)) > 1e-7:
        values = np.append(values, float(high))
    return values


def generate_sampling_points(config):
    """根据 evaluation_region 和 sampling 生成三维采样中心点。"""
    region = config.get('evaluation_region') or {}
    sampling = config.get('sampling') or {}
    xs = axis_values(region.get('x_min', 0.0), region.get('x_max', 0.0), sampling.get('dx', 1.0))
    ys = axis_values(region.get('y_min', 0.0), region.get('y_max', 0.0), sampling.get('dy', 1.0))
    zs = axis_values(region.get('z_min', 0.0), region.get('z_max', 0.0), sampling.get('dz', 1.0))
    xx, yy, zz = np.meshgrid(xs, ys, zs, indexing='xy')
    points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
    return pd.DataFrame(
        {
            'center_id': np.arange(points.shape[0], dtype=int),
            'x': points[:, 0],
            'y': points[:, 1],
            'z': points[:, 2],
        }
    )


def triangle_formation_offsets(spacing):
    """生成三机三角编队相对中心的偏移。"""
    d = max(0.0, float(spacing))
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [-d, -d, 0.0],
            [-d, d, 0.0],
        ],
        dtype=float,
    )


def formation_offsets_from_config(config):
    """根据配置生成编队偏移。"""
    formation = config.get('formation') or {}
    shape = str(formation.get('shape', 'triangle')).strip().lower()
    if shape != 'triangle':
        raise ValueError(f'unsupported formation.shape={shape}; currently only triangle is supported')
    return triangle_formation_offsets(safe_float(formation.get('spacing'), 1.5))


def load_trajectory(path):
    """读取通用动态障碍轨迹 CSV。"""
    df = pd.read_csv(path)
    return normalize_trajectory(df)


def normalize_trajectory(df):
    """把轨迹表统一成 time, obstacle_id, x, y, z, radius 格式。"""
    df = df.copy()
    if 'obstacle_id' not in df.columns and 'name' in df.columns:
        df['obstacle_id'] = df['name']
    required = {'time', 'obstacle_id', 'x', 'y', 'z', 'radius'}
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f'trajectory missing columns: {missing}')
    for col in ['time', 'x', 'y', 'z', 'radius']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df['obstacle_id'] = df['obstacle_id'].astype(str)
    df = df.dropna(subset=['time', 'x', 'y', 'z', 'radius'])
    return df.sort_values(['time', 'obstacle_id']).reset_index(drop=True)


def first_intrusion_time(center, offsets, trajectory, t_max, uav_radius, safe_margin):
    """计算一个虚拟编队中心第一次被动态障碍安全侵入的时间。"""
    center_xyz = center[['x', 'y', 'z']].to_numpy(dtype=float).reshape(1, 3)
    uav_positions = center_xyz + offsets
    for t, group in trajectory.groupby('time', sort=True):
        obs_pos = group[['x', 'y', 'z']].to_numpy(dtype=float)
        obs_radius = group['radius'].to_numpy(dtype=float)
        for uav_index, uav_pos in enumerate(uav_positions, start=1):
            distances = np.linalg.norm(obs_pos - uav_pos.reshape(1, 3), axis=1)
            thresholds = uav_radius + obs_radius + safe_margin
            hit_indices = np.where(distances <= thresholds)[0]
            if hit_indices.size > 0:
                idx = int(hit_indices[0])
                return {
                    'survival_time': float(t),
                    'failed': True,
                    'first_obstacle_id': str(group.iloc[idx]['obstacle_id']),
                    'first_uav_index': int(uav_index),
                    'first_distance': float(distances[idx]),
                    'intrusion_threshold': float(thresholds[idx]),
                }
    return {
        'survival_time': float(t_max),
        'failed': False,
        'first_obstacle_id': '',
        'first_uav_index': '',
        'first_distance': '',
        'intrusion_threshold': '',
    }


def difficulty_from_survival(s_formation, config):
    """把平均存活时间归一化成 0-10 难度分数。"""
    difficulty = config.get('difficulty') or {}
    norm_min = safe_float(difficulty.get('normalize_min'), 0.0)
    norm_max = safe_float(difficulty.get('normalize_max'), config.get('duration', 30.0))
    denom = max(1e-9, norm_max - norm_min)
    survival_norm = max(0.0, min(1.0, (float(s_formation) - norm_min) / denom))
    return max(0.0, min(10.0, 10.0 * (1.0 - survival_norm)))


def level_from_score(d_formation, config):
    """根据阈值输出 Easy、Medium、Hard。"""
    difficulty = config.get('difficulty') or {}
    easy_threshold = safe_float(difficulty.get('easy_threshold'), 3.5)
    medium_threshold = safe_float(difficulty.get('medium_threshold'), 7.0)
    if d_formation < easy_threshold:
        return 'Easy'
    if d_formation < medium_threshold:
        return 'Medium'
    return 'Hard'


def analyze(config, trajectory, centers=None, offsets=None):
    """执行完整 Formation Survivability 计算。"""
    if centers is None:
        centers = generate_sampling_points(config)
    if offsets is None:
        offsets = formation_offsets_from_config(config)
    centers = pd.DataFrame(centers).copy().reset_index(drop=True)
    trajectory = normalize_trajectory(trajectory)

    formation = config.get('formation') or {}
    duration = safe_float(config.get('duration'), 30.0)
    uav_radius = safe_float(formation.get('uav_radius'), 0.3)
    safe_margin = safe_float(formation.get('safe_margin'), 0.5)
    trajectory = trajectory[trajectory['time'] <= duration + 1e-9].copy()

    sample_rows = []
    for center_id, center in centers.iterrows():
        center_record = center.to_dict()
        center_record['center_id'] = int(center_record.get('center_id', center_id))
        event = first_intrusion_time(center, offsets, trajectory, duration, uav_radius, safe_margin)
        sample_rows.append({**center_record, **event})

    samples = pd.DataFrame(sample_rows)
    s_formation = float(samples['survival_time'].mean()) if not samples.empty else duration
    d_formation = difficulty_from_survival(s_formation, config)
    difficulty_level = level_from_score(d_formation, config)
    failed_count = int(samples['failed'].sum()) if not samples.empty else 0

    return {
        'scene_name': str(config.get('scene_name', 'unnamed_scene')),
        'manual_level': str(config.get('manual_level', '')),
        'S_formation': round(s_formation, 6),
        'D_formation': round(d_formation, 6),
        'difficulty_level': difficulty_level,
        'sample_count': int(len(samples)),
        'failed_sample_count': failed_count,
        'failed_sample_ratio': round(failed_count / max(1, len(samples)), 6),
        'min_survival_time': round(float(samples['survival_time'].min()), 6) if not samples.empty else duration,
        'median_survival_time': round(float(samples['survival_time'].median()), 6) if not samples.empty else duration,
        'max_survival_time': round(float(samples['survival_time'].max()), 6) if not samples.empty else duration,
        'T_max': duration,
        'samples': samples,
    }


def write_result_outputs(result, output_path, samples_output=None):
    """保存 JSON 结果和采样点 CSV。"""
    output_dir = os.path.dirname(os.path.abspath(output_path))
    os.makedirs(output_dir, exist_ok=True)
    if samples_output is None:
        base, _ = os.path.splitext(output_path)
        samples_output = f'{base}_samples.csv'
    samples_dir = os.path.dirname(os.path.abspath(samples_output))
    os.makedirs(samples_dir, exist_ok=True)

    samples = result['samples']
    samples.to_csv(samples_output, index=False)
    public = dict(result)
    public.pop('samples', None)
    public['samples_csv'] = samples_output
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(public, f, indent=2, ensure_ascii=False, sort_keys=True)
        f.write('\n')
    return public


def write_yaml_config(config, output_path):
    """保存可视化/复现实验使用的通用配置。"""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)
