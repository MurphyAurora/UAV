#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition


class SwarmStateExchange(Node):
    def __init__(self, num_drones: int = 3, rate_hz: float = 15.0):
        super().__init__('swarm_state_exchange')
        self.num_drones = num_drones
        self.states = {}
        self.pub = self.create_publisher(String, '/xtdrone2/swarm/state_exchange', 10)

        for i in range(1, self.num_drones + 1):
            topic = f'/px4_{i}/fmu/out/vehicle_local_position_v1'
            self.create_subscription(
                VehicleLocalPosition,
                topic,
                lambda msg, drone_id=i: self._pos_cb(drone_id, msg),
                qos_profile_sensor_data,
            )

        self.timer = self.create_timer(max(0.02, 1.0 / rate_hz), self._publish)
        self.get_logger().info(f'state_exchange started: num={self.num_drones}, rate={rate_hz}Hz')

    def _pos_cb(self, drone_id: int, msg: VehicleLocalPosition):
        self.states[drone_id] = {
            'id': drone_id,
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z),
            'vx': float(msg.vx),
            'vy': float(msg.vy),
            'vz': float(msg.vz),
            'heading': float(msg.heading),
            'stamp': time.time(),
        }

    def _publish(self):
        if not self.states:
            return
        msg = String()
        msg.data = json.dumps(
            {
                'num_drones': self.num_drones,
                'states': [self.states[k] for k in sorted(self.states.keys())],
                'stamp': time.time(),
            },
            ensure_ascii=False,
        )
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='Swarm state exchange node')
    parser.add_argument('--num-drones', type=int, default=3)
    parser.add_argument('--rate', type=float, default=20.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SwarmStateExchange(num_drones=args.num_drones, rate_hz=args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
