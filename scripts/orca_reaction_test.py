#!/usr/bin/env python3
import argparse
import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class OrcaReactionTest(Node):
    def __init__(
        self,
        num_drones=5,
        timeout_sec=8.0,
        min_speed=0.01,
        output_dir='',
        publish_test_obstacle=False,
    ):
        super().__init__('orca_reaction_test')
        self.num_drones = max(1, int(num_drones))
        self.timeout_sec = max(0.5, float(timeout_sec))
        self.min_speed = max(0.0, float(min_speed))
        self.output_dir = str(output_dir or '').strip()
        self.publish_test_obstacle = bool(publish_test_obstacle)
        self.start_time = time.time()
        self.passed = False
        self.last_cmd = None
        self.current_obstacle = None
        self.first_nonzero_time = None
        self.max_speed = 0.0
        self.cmd_count = 0
        self._last_wait_log_t = 0.0

        self.state_pub = self.create_publisher(String, '/xtdrone2/swarm/state_exchange', 10)
        self.obs_pub = self.create_publisher(String, '/xtdrone2/swarm/dynamic_obstacles', 10)
        self.create_subscription(String, '/xtdrone2/swarm/dynamic_obstacles', self._obs_cb, 10)
        self.create_subscription(Twist, '/xtdrone2/x500_1/avoid_cmd_vel_ned', self._cmd_cb, 10)
        self.timer = self.create_timer(0.1, self._publish_state)
        self.obs_timer = self.create_timer(0.1, self._publish_test_obstacle)
        self.get_logger().info(
            'orca reaction test started: waiting for named dynamic obstacle, then publishing fake swarm states near it'
        )

    def _publish_test_obstacle(self):
        if not self.publish_test_obstacle:
            return
        payload = {
            'stamp': time.time(),
            'scene_time': time.time() - self.start_time,
            'obstacles': [
                {
                    'obs_id': 9001,
                    'name': 'orca_test_obstacle',
                    'x': 12.0,
                    'y': -7.3,
                    'z': -3.0,
                    'vx': 0.0,
                    'vy': 0.0,
                    'vz': 0.0,
                    'radius': 0.45,
                }
            ],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.obs_pub.publish(msg)

    def _obs_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        named = [
            obs for obs in payload.get('obstacles', [])
            if str(obs.get('name', '')).strip()
        ]
        if not named:
            return
        obs = named[0]
        self.current_obstacle = {
            'name': str(obs.get('name', 'dynamic_obstacle')),
            'x': float(obs.get('x', 0.0)),
            'y': float(obs.get('y', 0.0)),
            'z': float(obs.get('z', -3.0)),
            'radius': float(obs.get('radius', 0.5)),
        }

    def _publish_state(self):
        if self.current_obstacle is None:
            now = time.time()
            if now - self._last_wait_log_t > 1.0:
                self._last_wait_log_t = now
                self.get_logger().info('waiting for named dynamic obstacle...')
            return

        obs = self.current_obstacle
        states = [
            # Put drone 1 beside the current named dynamic obstacle. Do not place it
            # exactly on top of the obstacle because ORCA needs a non-zero direction.
            {'id': 1, 'x': obs['x'], 'y': obs['y'] + 0.3, 'z': obs['z']},
        ]
        for drone_id in range(2, self.num_drones + 1):
            states.append(
                {
                    'id': drone_id,
                    'x': obs['x'] + 8.0 + 2.0 * (drone_id - 2),
                    'y': obs['y'] + 8.0,
                    'z': obs['z'],
                }
            )
        msg = String()
        msg.data = json.dumps({'stamp': time.time(), 'states': states})
        self.state_pub.publish(msg)

    def _cmd_cb(self, msg):
        speed = math.hypot(float(msg.linear.x), float(msg.linear.y))
        self.last_cmd = msg
        self.cmd_count += 1
        if speed > self.max_speed:
            self.max_speed = speed
        print(
            f'avoid_cmd x={msg.linear.x:.4f} y={msg.linear.y:.4f} speed={speed:.4f}',
            flush=True,
        )
        if speed > self.min_speed and not self.passed:
            self.passed = True
            self.first_nonzero_time = time.time() - self.start_time
            print('PASS: ORCA produced non-zero avoid command', flush=True)

    def timed_out(self):
        return time.time() - self.start_time > self.timeout_sec

    def summary(self):
        last_cmd = {}
        if self.last_cmd is not None:
            last_cmd = {
                'x': float(self.last_cmd.linear.x),
                'y': float(self.last_cmd.linear.y),
                'z': float(self.last_cmd.linear.z),
                'speed_xy': math.hypot(float(self.last_cmd.linear.x), float(self.last_cmd.linear.y)),
            }
        return {
            'passed': bool(self.passed),
            'num_drones': self.num_drones,
            'timeout_sec': self.timeout_sec,
            'min_speed_threshold': self.min_speed,
            'publish_test_obstacle': self.publish_test_obstacle,
            'elapsed_sec': time.time() - self.start_time,
            'first_nonzero_time_sec': self.first_nonzero_time,
            'max_speed_xy': self.max_speed,
            'cmd_count': self.cmd_count,
            'current_obstacle': self.current_obstacle,
            'last_cmd': last_cmd,
        }

    def write_summary(self):
        if not self.output_dir:
            return ''
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, 'reaction_summary.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self.summary(), f, indent=2, sort_keys=True)
            f.write('\n')
        return path


def default_output_dir():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join('runs', f'{stamp}_orca_reaction_test')


def main():
    parser = argparse.ArgumentParser(description='Check that ORCA outputs a non-zero avoid command.')
    parser.add_argument('--num-drones', type=int, default=5)
    parser.add_argument('--timeout-sec', type=float, default=8.0)
    parser.add_argument('--min-speed', type=float, default=0.01)
    parser.add_argument('--output-dir', default='')
    parser.add_argument(
        '--publish-test-obstacle',
        action='store_true',
        help='Publish a named test obstacle on /xtdrone2/swarm/dynamic_obstacles for ORCA-only reaction checks.',
    )
    args = parser.parse_args()

    rclpy.init()
    node = OrcaReactionTest(
        num_drones=args.num_drones,
        timeout_sec=args.timeout_sec,
        min_speed=args.min_speed,
        output_dir=args.output_dir or default_output_dir(),
        publish_test_obstacle=args.publish_test_obstacle,
    )
    try:
        while rclpy.ok() and not node.passed:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.timed_out():
                print('FAIL: no non-zero avoid command within timeout', flush=True)
                break
    finally:
        summary_path = node.write_summary()
        if summary_path:
            print(f'summary={summary_path}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
