#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import argparse
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class LocalAvoidOrcaLite(Node):
    def __init__(
        self,
        num_drones: int = 3,
        safe_radius: float = 1.2,
        max_speed: float = 0.8,
        gain: float = 0.9,
        obs_gain: float = 0.8,
        obs_influence_radius: float = 2.0,
        enable_after_z: float = -0.8,
    ):
        super().__init__('local_avoid_orca_lite')
        self.num_drones = num_drones
        self.safe_radius = safe_radius
        self.max_speed = max_speed
        self.gain = gain
        self.obs_gain = obs_gain
        self.obs_influence_radius = obs_influence_radius
        self.enable_after_z = enable_after_z
        self.states = {}
        self.dynamic_obstacles = []
        # 新增：速度滤波缓存，解决抖振
        self.last_vel = {i: [0.0, 0.0] for i in range(num_drones)}
        self.filter_alpha = 0.7  # 低通滤波系数
        self.cmd_deadband = 0.03  # 小速度死区，避免在接管阈值附近抖动
        
        # 阶段2：参数热更新（ros2 param set）
        self.declare_parameter('safe_radius', float(self.safe_radius))
        self.declare_parameter('max_speed', float(self.max_speed))
        self.declare_parameter('gain', float(self.gain))
        self.declare_parameter('obs_gain', float(self.obs_gain))
        self.declare_parameter('obs_influence_radius', float(self.obs_influence_radius))
        self.declare_parameter('enable_after_z', float(self.enable_after_z))
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.create_subscription(String, '/xtdrone2/swarm/state_exchange', self._state_cb, 10)
        self.create_subscription(String, '/xtdrone2/swarm/dynamic_obstacles', self._obs_cb, 10)
        self.pubs = {
            i: self.create_publisher(Twist, f'/xtdrone2/x500_{i}/avoid_cmd_vel_ned', 10)
            for i in range(self.num_drones)
        }
        self.timer = self.create_timer(0.05, self._run)  # 20Hz
        self._last_log_t = 0.0
        self._last_gate_log_t = 0.0
        self._gate_unlocked_logged = False
        self._airborne_gate_unlocked = False
        self.get_logger().info(
            f'local_avoid_orca_lite started: num={self.num_drones}, safe_radius={self.safe_radius}, '
            f'max_speed={self.max_speed}, obs_influence_radius={self.obs_influence_radius}, '
            f'enable_after_z={self.enable_after_z}'
        )

    def _on_params_changed(self, params):
        next_safe_radius = self.safe_radius
        next_max_speed = self.max_speed
        next_gain = self.gain
        next_obs_gain = self.obs_gain
        next_obs_influence_radius = self.obs_influence_radius
        next_enable_after_z = self.enable_after_z

        for p in params:
            if p.name == 'safe_radius':
                v = float(p.value)
                if v <= 0.0:
                    return SetParametersResult(successful=False, reason='safe_radius must be > 0')
                next_safe_radius = v
            elif p.name == 'max_speed':
                v = float(p.value)
                if v <= 0.0:
                    return SetParametersResult(successful=False, reason='max_speed must be > 0')
                next_max_speed = v
            elif p.name == 'gain':
                v = float(p.value)
                if v < 0.0:
                    return SetParametersResult(successful=False, reason='gain must be >= 0')
                next_gain = v
            elif p.name == 'obs_gain':
                v = float(p.value)
                if v < 0.0:
                    return SetParametersResult(successful=False, reason='obs_gain must be >= 0')
                next_obs_gain = v
            elif p.name == 'obs_influence_radius':
                v = float(p.value)
                if v <= 0.0:
                    return SetParametersResult(successful=False, reason='obs_influence_radius must be > 0')
                next_obs_influence_radius = v
            elif p.name == 'enable_after_z':
                next_enable_after_z = float(p.value)

        self.safe_radius = next_safe_radius
        self.max_speed = next_max_speed
        self.gain = next_gain
        self.obs_gain = next_obs_gain
        self.obs_influence_radius = next_obs_influence_radius
        self.enable_after_z = next_enable_after_z

        self.get_logger().info(
            'hot-update params: '
            f'safe_radius={self.safe_radius:.2f}, max_speed={self.max_speed:.2f}, gain={self.gain:.2f}, '
            f'obs_gain={self.obs_gain:.2f}, obs_influence_radius={self.obs_influence_radius:.2f}, '
            f'enable_after_z={self.enable_after_z:.2f}'
        )
        return SetParametersResult(successful=True)

    def _state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            arr = data.get('states', [])
            tmp = {}
            for s in arr:
                drone_id = int(s['id'])
                tmp[drone_id] = s
            self.states = tmp
        except Exception:
            return

    def _obs_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            arr = data.get('obstacles', [])
            tmp = []
            for o in arr:
                tmp.append(
                    {
                        'x': float(o.get('x', 0.0)),
                        'y': float(o.get('y', 0.0)),
                        'radius': float(o.get('radius', 0.3)),
                    }
                )
            self.dynamic_obstacles = tmp
        except Exception:
            return

    def _run(self):
        if len(self.states) < 2:
            return

        # 起飞保护：达到离地阈值后一次性解锁，避免中途瞬时高度抖动反复锁回
        min_z = 0.0
        max_z = 0.0
        first = True
        blocking_id = -1
        for i in range(self.num_drones):
            s = self.states.get(i)
            if s is None:
                return
            z_i = float(s['z'])
            if first:
                min_z = z_i
                max_z = z_i
                first = False
            if z_i < min_z:
                min_z = z_i
            if z_i > max_z:
                max_z = z_i
            if z_i > self.enable_after_z and blocking_id < 0:
                blocking_id = i

        if not self._airborne_gate_unlocked and blocking_id >= 0:
            now = time.time()
            if now - self._last_gate_log_t > 1.0:
                self._last_gate_log_t = now
                self.get_logger().info(
                    f'orca gate: waiting airborne, block_id={blocking_id}, max_z={max_z:.2f}, '
                    f'min_z={min_z:.2f}, threshold={self.enable_after_z:.2f}'
                )
            return

        if not self._airborne_gate_unlocked and blocking_id < 0:
            self._airborne_gate_unlocked = True

        if self._airborne_gate_unlocked and not self._gate_unlocked_logged:
            self._gate_unlocked_logged = True
            self.get_logger().info(
                f'orca gate: unlocked (all drones z<={self.enable_after_z:.2f}), avoid output enabled'
            )

        min_dist = 999.0
        for i in range(self.num_drones):
            si = self.states.get(i)
            if si is None:
                continue

            vx = 0.0
            vy = 0.0
            xi, yi = float(si['x']), float(si['y'])

            for j in range(self.num_drones):
                if i == j:
                    continue
                sj = self.states.get(j)
                if sj is None:
                    continue
                xj, yj = float(sj['x']), float(sj['y'])
                dx = xi - xj
                dy = yi - yj
                dist = math.hypot(dx, dy)
                if dist < min_dist:
                    min_dist = dist
                if 1e-6 < dist < self.safe_radius:
                    # ORCA-lite: 近距时做互易速度分离（只做局部修正）
                    strength = self.gain * (self.safe_radius - dist) / self.safe_radius
                    vx += strength * (dx / dist)
                    vy += strength * (dy / dist)

            # 阶段2最小增强：接入动态障碍输入（无输入时不影响原行为）
            for obs in self.dynamic_obstacles:
                dxo = xi - obs['x']
                dyo = yi - obs['y']
                dist_o = math.hypot(dxo, dyo)
                r_eff = self.obs_influence_radius + obs['radius']
                if 1e-6 < dist_o < r_eff:
                    obs_strength = self.obs_gain * (r_eff - dist_o) / max(r_eff, 1e-6)
                    vx += obs_strength * (dxo / dist_o)
                    vy += obs_strength * (dyo / dist_o)

            spd = math.hypot(vx, vy)
            if spd > self.max_speed and spd > 1e-6:
                scale = self.max_speed / spd
                vx *= scale
                vy *= scale

            # 对避障输出做简单低通，减少接管/释放附近的速度跳变
            last_vx, last_vy = self.last_vel[i]
            vx_f = self.filter_alpha * last_vx + (1.0 - self.filter_alpha) * vx
            vy_f = self.filter_alpha * last_vy + (1.0 - self.filter_alpha) * vy
            if math.hypot(vx_f, vy_f) < self.cmd_deadband:
                vx_f = 0.0
                vy_f = 0.0
            self.last_vel[i][0] = vx_f
            self.last_vel[i][1] = vy_f

            cmd = Twist()
            cmd.linear.x = float(vx_f)
            cmd.linear.y = float(vy_f)
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
            self.pubs[i].publish(cmd)

        now = time.time()
        if min_dist < 999.0 and now - self._last_log_t > 1.0:
            self._last_log_t = now
            self.get_logger().info(f'orca-lite running, min_dist={min_dist:.2f}')


def main():
    parser = argparse.ArgumentParser(description='Local avoid ORCA-lite node')
    parser.add_argument('--num-drones', type=int, default=3)
    parser.add_argument('--safe-radius', type=float, default=1.2)
    parser.add_argument('--max-speed', type=float, default=0.8)
    parser.add_argument('--gain', type=float, default=0.9)
    parser.add_argument('--obs-gain', type=float, default=0.8)
    parser.add_argument('--obs-influence-radius', type=float, default=2.0)
    parser.add_argument('--enable-after-z', type=float, default=-0.8)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = LocalAvoidOrcaLite(
        num_drones=args.num_drones,
        safe_radius=args.safe_radius,
        max_speed=args.max_speed,
        gain=args.gain,
        obs_gain=args.obs_gain,
        obs_influence_radius=args.obs_influence_radius,
        enable_after_z=args.enable_after_z,
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
