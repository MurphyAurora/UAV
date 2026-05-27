#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立文件版 Formation Survivability 难度计算。

输入：
    scene_config.yaml 和 obstacle_trajectory.csv。
输出：
    S_formation、D_formation、difficulty_level，以及采样点存活时间文件。

这个入口只负责读取“论文复现实验格式”的输入；核心算法在 core.py。
"""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from core import analyze, load_trajectory, load_yaml_config, write_result_outputs


def parse_args():
    parser = argparse.ArgumentParser(description='Offline Formation Survivability analysis')
    parser.add_argument('--config', required=True, help='scene_config.yaml')
    parser.add_argument('--trajectory', required=True, help='obstacle_trajectory.csv')
    parser.add_argument('--output', required=True, help='result JSON path')
    parser.add_argument('--samples-output', default='', help='optional samples CSV path')
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_yaml_config(args.config)
    trajectory = load_trajectory(args.trajectory)
    result = analyze(config, trajectory)
    public = write_result_outputs(result, args.output, args.samples_output or None)
    print(
        f"{public['scene_name']}: "
        f"S_formation={public['S_formation']:.3f}, "
        f"D_formation={public['D_formation']:.3f}, "
        f"difficulty_level={public['difficulty_level']}"
    )
    print(f'result={args.output}')
    print(f"samples={public['samples_csv']}")


if __name__ == '__main__':
    main()
