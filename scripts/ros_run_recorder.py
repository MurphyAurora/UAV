#!/usr/bin/env python3
import argparse
import csv
import json
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class RosRunRecorder(Node):
    def __init__(self, output_dir, duration_sec=0.0):
        super().__init__('ros_run_recorder')
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir))
        self.duration_sec = max(0.0, float(duration_sec))
        self.start_time = time.time()
        self.drone_count = 0
        self.obstacle_count = 0
        self.avoid_count = 0
        self._closed = False

        os.makedirs(self.output_dir, exist_ok=True)
        self.drones_path = os.path.join(self.output_dir, 'drones.csv')
        self.obstacles_path = os.path.join(self.output_dir, 'obstacles_ros.csv')
        self.avoid_path = os.path.join(self.output_dir, 'avoid_cmds.csv')
        self.summary_path = os.path.join(self.output_dir, 'recording_summary.json')

        self.drones_file = open(self.drones_path, 'w', newline='', encoding='utf-8')
        self.obstacles_file = open(self.obstacles_path, 'w', newline='', encoding='utf-8')
        self.avoid_file = open(self.avoid_path, 'w', newline='', encoding='utf-8')
        self.drones_writer = csv.DictWriter(
            self.drones_file,
            fieldnames=[
                'time',
                'source_stamp',
                'drone_id',
                'x',
                'y',
                'z',
                'world_x',
                'world_y',
                'vx',
                'vy',
                'vz',
                'heading',
            ],
        )
        self.obstacles_writer = csv.DictWriter(
            self.obstacles_file,
            fieldnames=[
                'time',
                'source_stamp',
                'scene_time',
                'obs_id',
                'name',
                'x',
                'y',
                'z',
                'vx',
                'vy',
                'vz',
                'radius',
            ],
        )
        self.avoid_writer = csv.DictWriter(
            self.avoid_file,
            fieldnames=[
                'time',
                'drone_id',
                'vx',
                'vy',
                'vz',
                'speed_xy',
            ],
        )
        self.drones_writer.writeheader()
        self.obstacles_writer.writeheader()
        self.avoid_writer.writeheader()

        self.create_subscription(String, '/xtdrone2/swarm/state_exchange', self._state_cb, 10)
        self.create_subscription(String, '/xtdrone2/swarm/dynamic_obstacles', self._obstacle_cb, 10)
        for drone_id in range(1, 21):
            self.create_subscription(
                Twist,
                f'/xtdrone2/x500_{drone_id}/avoid_cmd_vel_ned',
                lambda msg, did=drone_id: self._avoid_cb(did, msg),
                10,
            )
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'ros_run_recorder started: output_dir={self.output_dir}, duration_sec={self.duration_sec:.1f}'
        )

    def _elapsed(self):
        return time.time() - self.start_time

    @staticmethod
    def _float(value, default=0.0):
        try:
            return float(value)
        except Exception:
            return float(default)

    def _state_cb(self, msg):
        now = time.time()
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'invalid state JSON: {e}')
            return
        source_stamp = self._float(payload.get('stamp'), now)
        for state in payload.get('states', []):
            row = {
                'time': f'{now:.6f}',
                'source_stamp': f'{source_stamp:.6f}',
                'drone_id': int(state.get('id', -1)),
                'x': f'{self._float(state.get("x")):.6f}',
                'y': f'{self._float(state.get("y")):.6f}',
                'z': f'{self._float(state.get("z")):.6f}',
                'world_x': f'{self._float(state.get("world_x", state.get("x"))):.6f}',
                'world_y': f'{self._float(state.get("world_y", state.get("y"))):.6f}',
                'vx': f'{self._float(state.get("vx")):.6f}',
                'vy': f'{self._float(state.get("vy")):.6f}',
                'vz': f'{self._float(state.get("vz")):.6f}',
                'heading': f'{self._float(state.get("heading")):.6f}',
            }
            self.drones_writer.writerow(row)
            self.drone_count += 1

    def _obstacle_cb(self, msg):
        now = time.time()
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'invalid obstacle JSON: {e}')
            return
        source_stamp = self._float(payload.get('stamp'), now)
        scene_time = self._float(payload.get('scene_time'), -1.0)
        for obs in payload.get('obstacles', []):
            row = {
                'time': f'{now:.6f}',
                'source_stamp': f'{source_stamp:.6f}',
                'scene_time': f'{scene_time:.6f}',
                'obs_id': int(obs.get('obs_id', -1)),
                'name': str(obs.get('name', '')),
                'x': f'{self._float(obs.get("x")):.6f}',
                'y': f'{self._float(obs.get("y")):.6f}',
                'z': f'{self._float(obs.get("z")):.6f}',
                'vx': f'{self._float(obs.get("vx")):.6f}',
                'vy': f'{self._float(obs.get("vy")):.6f}',
                'vz': f'{self._float(obs.get("vz")):.6f}',
                'radius': f'{self._float(obs.get("radius")):.6f}',
            }
            self.obstacles_writer.writerow(row)
            self.obstacle_count += 1

    def _avoid_cb(self, drone_id, msg):
        now = time.time()
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.linear.z)
        self.avoid_writer.writerow(
            {
                'time': f'{now:.6f}',
                'drone_id': int(drone_id),
                'vx': f'{vx:.6f}',
                'vy': f'{vy:.6f}',
                'vz': f'{vz:.6f}',
                'speed_xy': f'{(vx * vx + vy * vy) ** 0.5:.6f}',
            }
        )
        self.avoid_count += 1

    def _tick(self):
        self.drones_file.flush()
        self.obstacles_file.flush()
        self.avoid_file.flush()
        self.get_logger().info(
            f'recording: elapsed={self._elapsed():.1f}s, drone_rows={self.drone_count}, '
            f'obstacle_rows={self.obstacle_count}, avoid_rows={self.avoid_count}'
        )
        if self.duration_sec > 0.0 and self._elapsed() >= self.duration_sec:
            self.get_logger().info('recording duration reached, shutting down')
            rclpy.shutdown()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.drones_file.flush()
        self.obstacles_file.flush()
        self.avoid_file.flush()
        self.drones_file.close()
        self.obstacles_file.close()
        self.avoid_file.close()
        summary = {
            'output_dir': self.output_dir,
            'duration_sec': self._elapsed(),
            'drone_rows': self.drone_count,
            'obstacle_rows': self.obstacle_count,
            'avoid_rows': self.avoid_count,
            'drones_csv': self.drones_path,
            'obstacles_csv': self.obstacles_path,
            'avoid_cmds_csv': self.avoid_path,
        }
        with open(self.summary_path, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, sort_keys=True)
            f.write('\n')


def default_output_dir():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join('runs_ros', f'{stamp}_ros_recording')


def main():
    parser = argparse.ArgumentParser(description='Record swarm state and dynamic obstacle ROS topics to CSV.')
    parser.add_argument('--output-dir', default='')
    parser.add_argument('--duration-sec', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = RosRunRecorder(args.output_dir or default_output_dir(), duration_sec=args.duration_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print(f'drones_csv={node.drones_path}', flush=True)
        print(f'obstacles_csv={node.obstacles_path}', flush=True)
        print(f'avoid_cmds_csv={node.avoid_path}', flush=True)
        print(f'summary={node.summary_path}', flush=True)


if __name__ == '__main__':
    main()
