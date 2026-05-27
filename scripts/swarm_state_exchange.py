#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 3 层：真实 ROS 运行中的集群状态汇总节点。

输入：
    每架无人机的 PX4 VehicleLocalPosition topic。
输出：
    /xtdrone2/swarm/state_exchange，包含 local x/y/z 和可选 world_x/world_y。

当多架 PX4 模型的 local frame 原点不同时，world_x/world_y 用于统一距离计算坐标系。
"""

import json
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition


class SwarmStateExchange(Node):
    def __init__(
        self,
        num_drones: int = 3,
        rate_hz: float = 15.0,
        spawned_formation: bool = False,
        mission_start_x: float = 0.0,
        base_y: float = 0.0,
        y_spacing: float = 1.8,
        leader_id: int = 1,
    ):
        super().__init__('swarm_state_exchange')
        self.num_drones = num_drones
        self.spawned_formation = bool(spawned_formation)
        self.mission_start_x = float(mission_start_x)
        self.base_y = float(base_y)
        self.y_spacing = float(y_spacing)
        self.leader_id = int(leader_id)
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
        self.get_logger().info(
            f'state_exchange started: num={self.num_drones}, rate={rate_hz}Hz, '
            f'spawned_formation={int(self.spawned_formation)}, mission_start_x={self.mission_start_x:.2f}, '
            f'base_y={self.base_y:.2f}, y_spacing={self.y_spacing:.2f}, leader_id={self.leader_id}'
        )

    def _pos_cb(self, drone_id: int, msg: VehicleLocalPosition):
        state = {
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
        if self.spawned_formation:
            spawn_y = self.base_y + (drone_id - self.leader_id) * self.y_spacing
            state['world_x'] = self.mission_start_x + float(msg.x)
            state['world_y'] = spawn_y + float(msg.y)
        else:
            state['world_x'] = float(msg.x)
            state['world_y'] = float(msg.y)
        self.states[drone_id] = state

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
    parser.add_argument('--spawned-formation', type=int, default=0)
    parser.add_argument('--mission-start-x', type=float, default=0.0)
    parser.add_argument('--base-y', type=float, default=0.0)
    parser.add_argument('--y-spacing', type=float, default=1.8)
    parser.add_argument('--leader-id', type=int, default=1)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SwarmStateExchange(
        num_drones=args.num_drones,
        rate_hz=args.rate,
        spawned_formation=(int(args.spawned_formation) != 0),
        mission_start_x=args.mission_start_x,
        base_y=args.base_y,
        y_spacing=args.y_spacing,
        leader_id=args.leader_id,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
