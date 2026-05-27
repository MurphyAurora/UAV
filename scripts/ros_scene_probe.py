#!/usr/bin/env python3
import argparse
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RosSceneProbe(Node):
    def __init__(self, topic, once=False):
        super().__init__('ros_scene_probe')
        self.once = bool(once)
        self.create_subscription(String, topic, self._callback, 10)
        self.get_logger().info(f'listening: {topic}')

    def _callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'invalid JSON payload: {e}')
            return

        scene_time = float(payload.get('scene_time', -1.0))
        dynamic = [
            obs for obs in payload.get('obstacles', [])
            if str(obs.get('name', '')).strip()
        ]
        if not dynamic:
            print(f'scene_time={scene_time:.3f} dynamic=0')
        for obs in dynamic:
            print(
                f"scene_time={scene_time:.3f} "
                f"name={obs.get('name', 'dynamic_obstacle')} "
                f"x={float(obs.get('x', 0.0)):.4f} "
                f"y={float(obs.get('y', 0.0)):.4f} "
                f"z={float(obs.get('z', 0.0)):.4f} "
                f"vx={float(obs.get('vx', 0.0)):.4f} "
                f"vy={float(obs.get('vy', 0.0)):.4f} "
                f"vz={float(obs.get('vz', 0.0)):.4f} "
                f"radius={float(obs.get('radius', 0.0)):.4f}",
                flush=True,
            )
        if self.once:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Print named dynamic obstacles from the scene ROS topic.')
    parser.add_argument('--topic', default='/xtdrone2/swarm/dynamic_obstacles')
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = RosSceneProbe(args.topic, once=args.once)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
