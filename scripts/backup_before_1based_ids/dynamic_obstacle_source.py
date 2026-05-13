 #!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import argparse
import time
import os
import tempfile
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class DynamicObstacleSource(Node):
    def __init__(
        self,
        rate_hz: float = 10.0,
        world_name: str = 'default',
        visualize_gz: bool = True,
        mode: str = 'moving_ball',
        start_center_x: float = 18.0,
        start_center_y: float = 6.0,
        active_center_x: float = 5.5,
        active_center_y: float = 0.0,
        switch_after_sec: float = 38.0,
        switch_fade_sec: float = 6.0,
    ):
        super().__init__('dynamic_obstacle_source')
        self.pub = self.create_publisher(String, '/xtdrone2/swarm/dynamic_obstacles', 10)
        self.target_pub = self.create_publisher(String, '/xtdrone2/swarm/target_markers', 10)
        self.world_name = world_name
        self.visualize_gz = bool(visualize_gz)
        self.mode = str(mode)
        self.entity_name = 'dynamic_obstacle_ball'
        self.wall_entity_name = 'static_obstacle_wall'
        self.target_ball_entity_name = 'target_ball'

        # 参数可运行中修改（阶段2热更新配套）
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 1.0)
        self.declare_parameter('start_center_x', float(start_center_x))
        self.declare_parameter('start_center_y', float(start_center_y))
        self.declare_parameter('active_center_x', float(active_center_x))
        self.declare_parameter('active_center_y', float(active_center_y))
        self.declare_parameter('switch_after_sec', float(switch_after_sec))
        self.declare_parameter('switch_fade_sec', float(switch_fade_sec))
        # wall_x / wall_y are defined in swarm-control NED frame.
        self.declare_parameter('wall_x', 5.5)
        self.declare_parameter('wall_y', 0.0)
        self.declare_parameter('wall_z', 0.9)
        self.declare_parameter('wall_length', 5.0)
        self.declare_parameter('wall_thickness', 0.25)
        self.declare_parameter('wall_height', 1.8)
        self.declare_parameter('wall_segment_spacing', 0.6)
        self.declare_parameter('target_ball_enable', 1)
        self.declare_parameter('target_ball_num_drones', 3)
        self.declare_parameter('target_ball_leader_id', 0)
        self.declare_parameter('target_ball_target_x', 20.0)
        self.declare_parameter('target_ball_target_z', -3.0)
        self.declare_parameter('target_ball_base_y', 0.0)
        self.declare_parameter('target_ball_y_spacing', 1.8)
        self.declare_parameter('amplitude_x', 1.2)
        self.declare_parameter('amplitude_y', 0.6)
        self.declare_parameter('omega', 0.5)
        self.declare_parameter('radius', 0.35)
        self.declare_parameter('z', 0.6)
        self.declare_parameter('visual_filter_alpha', 0.35)
        self.declare_parameter('visual_max_step', 0.15)

        self.t0 = time.time()
        self.timer = self.create_timer(max(0.05, 1.0 / rate_hz), self._publish)
        self._last_log_t = 0.0
        self._last_pose_set_t = 0.0
        self._last_pose_warn_t = 0.0
        self._last_probe_t = 0.0
        self._last_probe_warn_t = 0.0
        self._last_frame_check_log_t = 0.0
        self._spawn_done = False
        self._gz_set_pose_service = f'/world/{self.world_name}/set_pose'
        self._gz_create_service = f'/world/{self.world_name}/create'
        self._gz_ready = False
        self._visual_last_ok_x = None
        self._visual_last_ok_y = None
        self._visual_last_ok_z = None
        self._switch_logged = False
        self._wall_spawned = False
        self._target_ball_spawned = False
        self.get_logger().info(
            'dynamic_obstacle_source started, publishing /xtdrone2/swarm/dynamic_obstacles, '
            f'visualize_gz={self.visualize_gz}, world={self.world_name}, mode={self.mode}'
        )

        # 注意：目标球/墙体的初始位姿依赖运行参数。
        # 不要在 __init__ 里立刻 spawn，否则会先用 declare_parameter 的默认值创建，
        # 再被 main() 的 set_parameters 覆盖，导致 Gazebo 视觉实体与发布出去的 marker 不一致。

    @staticmethod
    def _enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
        # Gazebo world frame is ENU; swarm control stack uses NED.
        # ENU(E, N, U) -> NED(N, E, D)
        return float(y_enu), float(x_enu), float(-z_enu)

    @staticmethod
    def _enu_vel_to_ned(vx_enu: float, vy_enu: float, vz_enu: float):
        return float(vy_enu), float(vx_enu), float(-vz_enu)

    @staticmethod
    def _ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
        # NED(N, E, D) -> ENU(E, N, U)
        return float(y_ned), float(x_ned), float(-z_ned)

    def _run_gz_service(self, service: str, reqtype: str, req: str, timeout_ms: int = 400) -> bool:
        cmd = [
            'gz',
            'service',
            '-s',
            service,
            '--reqtype',
            reqtype,
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(timeout_ms),
            '--req',
            req,
        ]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=max(0.2, timeout_ms / 1000.0 + 0.3))
            out = ((p.stdout or '') + (p.stderr or '')).lower()
            return p.returncode == 0 and ('data: true' in out or 'true' in out)
        except Exception:
            return False

    def _setup_gz_visual(self):
        if not shutil.which('gz'):
            self.get_logger().warn('gz command not found, skip visual obstacle sync')
            return
        try:
            p = subprocess.run(['gz', 'service', '-l'], capture_output=True, text=True, timeout=4.0)
            if p.returncode != 0 or self._gz_set_pose_service not in (p.stdout or ''):
                now = time.time()
                if now - self._last_probe_warn_t > 2.0:
                    self._last_probe_warn_t = now
                    self.get_logger().warn('gazebo transport set_pose service unavailable, keep retrying...')
                return
            self._gz_ready = True
            self.get_logger().info('gazebo transport service detected, visual sync enabled')
            self._spawn_visual_obstacle()
        except Exception as e:
            now = time.time()
            if now - self._last_probe_warn_t > 2.0:
                self._last_probe_warn_t = now
                self.get_logger().warn(f'gazebo service probe failed, keep retrying: {e}')

    def _spawn_visual_obstacle(self):
        if self.mode == 'static_wall':
            sdf_path = self._prepare_wall_sdf()
            entity_name = self.wall_entity_name
        else:
            sdf_path = os.path.join(os.path.dirname(__file__), 'dynamic_obstacle_ball.sdf')
            entity_name = self.entity_name
        if not os.path.exists(sdf_path):
            self.get_logger().warn(f'visual obstacle sdf not found: {sdf_path}')
            return
        if self.mode == 'static_wall':
            wall_x_ned = float(self._p('wall_x'))
            wall_y_ned = float(self._p('wall_y'))
            wall_z = float(self._p('wall_z'))
            wall_x_enu, wall_y_enu, _ = self._ned_to_enu(wall_x_ned, wall_y_ned, 0.0)
            req = (
                f'name: "{entity_name}", '
                f'allow_renaming: false, '
                f'sdf_filename: "{sdf_path}", '
                f'pose: {{ position: {{ x: {wall_x_enu:.3f}, y: {wall_y_enu:.3f}, z: {wall_z:.3f} }}, '
                f'orientation: {{ z: 0.7071068, w: 0.7071068 }} }}'
            )
            ok = self._run_gz_service(self._gz_create_service, 'gz.msgs.EntityFactory', req, timeout_ms=1200)
            self._spawn_done = bool(ok)
            self._wall_spawned = self._spawn_done
            if self._spawn_done:
                self._visual_last_ok_x = wall_x_enu
                self._visual_last_ok_y = wall_y_enu
                self._visual_last_ok_z = wall_z
                self.get_logger().info(
                    'gazebo static wall spawned successfully (gz transport), '
                    f'NED=({wall_x_ned:.2f},{wall_y_ned:.2f}), ENU=({wall_x_enu:.2f},{wall_y_enu:.2f})'
                )
                self._spawn_target_ball()
            else:
                self.get_logger().warn('gazebo static wall spawn failed (gz transport)')
            return

        # 先尝试直接设置位姿；若实体已存在即可立即联动
        if self._set_visual_pose(0.0, 0.0, float(self._p('z')), force=True):
            self._spawn_done = True
            self._visual_last_ok_x = 0.0
            self._visual_last_ok_y = 0.0
            self._visual_last_ok_z = float(self._p('z'))
            self.get_logger().info('gazebo visual obstacle already exists, pose sync enabled')
            return

        req = (
            f'name: "{self.entity_name}", '
            f'allow_renaming: false, '
            f'sdf_filename: "{sdf_path}", '
            f'pose: {{ position: {{ x: 0.0, y: 0.0, z: {float(self._p("z")):.3f} }}, orientation: {{ w: 1.0 }} }}'
        )
        ok = self._run_gz_service(self._gz_create_service, 'gz.msgs.EntityFactory', req, timeout_ms=1200)
        self._spawn_done = bool(ok)
        if self._spawn_done:
            self.get_logger().info('gazebo visual obstacle spawned successfully (gz transport)')
        else:
            self.get_logger().warn('gazebo visual obstacle spawn failed (gz transport)')

    def _prepare_wall_sdf(self):
        thickness = max(0.1, float(self._p('wall_thickness')))
        length = max(1.0, float(self._p('wall_length')))
        height = max(0.5, float(self._p('wall_height')))
        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="static_obstacle_wall">
    <static>true</static>
    <link name="wall_link">
      <collision name="wall_collision">
        <geometry>
          <box>
            <size>{thickness:.3f} {length:.3f} {height:.3f}</size>
          </box>
        </geometry>
      </collision>
      <visual name="wall_visual">
        <geometry>
          <box>
            <size>{thickness:.3f} {length:.3f} {height:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.55 0.55 0.55 1</ambient>
          <diffuse>0.55 0.55 0.55 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        tmp_path = os.path.join(tempfile.gettempdir(), 'xtd2_static_obstacle_wall_generated.sdf')
        with open(tmp_path, 'w', encoding='utf-8') as f:
            f.write(sdf)
        self.get_logger().info(
            f'wall visual sdf prepared: path={tmp_path}, size=({thickness:.2f},{length:.2f},{height:.2f})'
        )
        return tmp_path

    def _set_visual_pose(self, x: float, y: float, z: float, force: bool = False) -> bool:
        if not self._gz_ready:
            return False
        if not force and not self._spawn_done:
            return False
        req = (
            f'name: "{self.entity_name}", '
            f'position: {{ x: {float(x):.3f}, y: {float(y):.3f}, z: {float(z):.3f} }}, '
            f'orientation: {{ w: 1.0 }}'
        )
        return self._run_gz_service(self._gz_set_pose_service, 'gz.msgs.Pose', req, timeout_ms=600)

    def _set_target_ball_pose_enu(self, x_enu: float, y_enu: float, z_enu: float) -> bool:
        if not self._gz_ready:
            return False
        req = (
            f'name: "{self.target_ball_entity_name}", '
            f'position: {{ x: {float(x_enu):.3f}, y: {float(y_enu):.3f}, z: {float(z_enu):.3f} }}, '
            f'orientation: {{ w: 1.0 }}'
        )
        return self._run_gz_service(self._gz_set_pose_service, 'gz.msgs.Pose', req, timeout_ms=600)

    def _spawn_target_ball(self):
        if self._target_ball_spawned:
            return
        if int(self.get_parameter('target_ball_enable').value) != 1:
            return

        # 绿色target marker已移除：改为使用红色dynamic_obstacle_ball作为静态目标点。
        sdf_path = os.path.join(os.path.dirname(__file__), 'dynamic_obstacle_ball.sdf')
        if not os.path.exists(sdf_path):
            self.get_logger().warn(f'target ball sdf not found: {sdf_path}')
            return

        leader_id = int(self.get_parameter('target_ball_leader_id').value)
        target_x = float(self.get_parameter('target_ball_target_x').value)
        target_z = float(self.get_parameter('target_ball_target_z').value)
        base_y = float(self.get_parameter('target_ball_base_y').value)
        spacing = float(self.get_parameter('target_ball_y_spacing').value)
        target_y = base_y + (0 - leader_id) * spacing
        enu_x, enu_y, enu_z = self._ned_to_enu(target_x, target_y, target_z)

        req = (
            f'name: "{self.target_ball_entity_name}", '
            f'allow_renaming: false, '
            f'sdf_filename: "{sdf_path}", '
            f'pose: {{ position: {{ x: {enu_x:.3f}, y: {enu_y:.3f}, z: {enu_z:.3f} }}, orientation: {{ w: 1.0 }} }}'
        )
        ok = self._run_gz_service(self._gz_create_service, 'gz.msgs.EntityFactory', req, timeout_ms=1200)
        self._target_ball_spawned = bool(ok)

        if self._target_ball_spawned:
            self.get_logger().info(
                f'target ball spawned: entity={self.target_ball_entity_name}, '
                f'ENU=({enu_x:.2f},{enu_y:.2f},{enu_z:.2f}), NED=({target_x:.2f},{target_y:.2f},{target_z:.2f})'
            )
        else:
            self.get_logger().warn('target ball spawn failed')

    def _next_visual_pose(self, target_x: float, target_y: float, target_z: float):
        alpha = min(1.0, max(0.0, self._p('visual_filter_alpha')))
        max_step = max(0.01, self._p('visual_max_step'))

        if self._visual_last_ok_x is None:
            return float(target_x), float(target_y), float(target_z)

        dx = float(target_x) - self._visual_last_ok_x
        dy = float(target_y) - self._visual_last_ok_y
        dz = float(target_z) - self._visual_last_ok_z

        # 先做低通，再做限步，减少跳变感
        cand_x = self._visual_last_ok_x + alpha * dx
        cand_y = self._visual_last_ok_y + alpha * dy
        cand_z = self._visual_last_ok_z + alpha * dz

        step_dx = cand_x - self._visual_last_ok_x
        step_dy = cand_y - self._visual_last_ok_y
        step_dz = cand_z - self._visual_last_ok_z
        step_norm = math.sqrt(step_dx * step_dx + step_dy * step_dy + step_dz * step_dz)
        if step_norm > max_step and step_norm > 1e-6:
            s = max_step / step_norm
            cand_x = self._visual_last_ok_x + step_dx * s
            cand_y = self._visual_last_ok_y + step_dy * s
            cand_z = self._visual_last_ok_z + step_dz * s

        return cand_x, cand_y, cand_z

    def _build_target_markers_payload(self):
        if int(self.get_parameter('target_ball_enable').value) != 1:
            return None

        num_drones = max(1, int(self.get_parameter('target_ball_num_drones').value))
        leader_id = int(self.get_parameter('target_ball_leader_id').value)
        target_x = float(self.get_parameter('target_ball_target_x').value)
        target_z = float(self.get_parameter('target_ball_target_z').value)
        base_y = float(self.get_parameter('target_ball_base_y').value)
        spacing = float(self.get_parameter('target_ball_y_spacing').value)

        markers = []
        for i in range(num_drones):
            ned_x = target_x
            ned_y = base_y + (i - leader_id) * spacing
            ned_z = target_z
            enu_x, enu_y, enu_z = self._ned_to_enu(ned_x, ned_y, ned_z)
            markers.append(
                {
                    'id': i,
                    # Mission/control side consumes x/y/z in NED.
                    'x': ned_x,
                    'y': ned_y,
                    'z': ned_z,
                    # Keep ENU fields for debugging and traceability.
                    'enu_x': enu_x,
                    'enu_y': enu_y,
                    'enu_z': enu_z,
                }
            )

        return {
            'stamp': time.time(),
            'leader_id': leader_id,
            'markers': markers,
        }

    def _log_target_table(self, target_payload, prefix='target-table'):
        markers = target_payload.get('markers', []) if target_payload else []
        if not markers:
            return
        self.get_logger().info(f'{prefix}: begin')
        for m in markers:
            self.get_logger().info(
                f'{prefix}: id={int(m.get("id", -1))} '
                f'NED=({float(m.get("x", 0.0)):.2f},{float(m.get("y", 0.0)):.2f},{float(m.get("z", 0.0)):.2f}) '
                f'ENU=({float(m.get("enu_x", 0.0)):.2f},{float(m.get("enu_y", 0.0)):.2f},{float(m.get("enu_z", 0.0)):.2f})'
            )
        self.get_logger().info(f'{prefix}: end')

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _blend_center(self, t: float):
        start_x = self._p('start_center_x')
        start_y = self._p('start_center_y')
        active_x = self._p('active_center_x')
        active_y = self._p('active_center_y')
        switch_after = max(0.0, self._p('switch_after_sec'))
        switch_fade = max(0.1, self._p('switch_fade_sec'))

        if t <= switch_after:
            return start_x, start_y
        if t >= switch_after + switch_fade:
            if not self._switch_logged:
                self._switch_logged = True
                self.get_logger().info(
                    f'obstacle center switched to active path center=({active_x:.2f}, {active_y:.2f})'
                )
            return active_x, active_y

        ratio = (t - switch_after) / switch_fade
        cx = start_x + (active_x - start_x) * ratio
        cy = start_y + (active_y - start_y) * ratio
        return cx, cy

    def _publish(self):
        t = time.time() - self.t0
        z = self._p('z')

        if self.mode == 'static_wall':
            wall_x_ned = self._p('wall_x')
            wall_y_ned = self._p('wall_y')
            wall_length = max(1.0, self._p('wall_length'))
            wall_thickness = max(0.1, self._p('wall_thickness'))
            wall_segment_spacing = max(0.2, self._p('wall_segment_spacing'))
            segment_radius = max(0.25, wall_thickness * 0.5 + 0.15)
            segment_count = max(3, int(wall_length / wall_segment_spacing) + 1)
            half_span = wall_length * 0.5
            step = wall_length / max(1, segment_count - 1)
            obstacles = []
            for idx in range(segment_count):
                ned_x = wall_x_ned
                ned_y = wall_y_ned - half_span + step * idx
                ned_z = 0.0
                obstacles.append(
                    {
                        'obs_id': idx + 1,
                        'x': ned_x,
                        'y': ned_y,
                        'z': ned_z,
                        'vx': 0.0,
                        'vy': 0.0,
                        'vz': 0.0,
                        'radius': segment_radius,
                    }
                )
            payload = {'stamp': time.time(), 'obstacles': obstacles}
            x, y, _ = self._ned_to_enu(wall_x_ned, wall_y_ned, 0.0)
            vx = 0.0
            vy = 0.0
            r = segment_radius

            # Compute current target ball ENU pose from NED target parameters.
            leader_id = int(self.get_parameter('target_ball_leader_id').value)
            target_x_ned = float(self.get_parameter('target_ball_target_x').value)
            target_z_ned = float(self.get_parameter('target_ball_target_z').value)
            base_y_ned = float(self.get_parameter('target_ball_base_y').value)
            spacing_ned = float(self.get_parameter('target_ball_y_spacing').value)
            target_y_ned = base_y_ned + (0 - leader_id) * spacing_ned
            target_ball_enu = self._ned_to_enu(target_x_ned, target_y_ned, target_z_ned)
        else:
            base_cx, base_cy = self._blend_center(t)
            # center_x/center_y 作为运行时微调偏置，便于热更新
            cx = base_cx + self._p('center_x')
            cy = base_cy + self._p('center_y')
            ax = self._p('amplitude_x')
            ay = self._p('amplitude_y')
            w = self._p('omega')
            r = max(0.05, self._p('radius'))

            x = cx + ax * math.sin(w * t)
            y = cy + ay * math.cos(w * t)
            vx = ax * w * math.cos(w * t)
            vy = -ay * w * math.sin(w * t)

            ned_x, ned_y, ned_z = self._enu_to_ned(x, y, 0.0)
            ned_vx, ned_vy, ned_vz = self._enu_vel_to_ned(vx, vy, 0.0)

            payload = {
                'stamp': time.time(),
                'obstacles': [
                    {
                        'obs_id': 1,
                        'x': ned_x,
                        'y': ned_y,
                        'z': ned_z,
                        'vx': ned_vx,
                        'vy': ned_vy,
                        'vz': ned_vz,
                        'radius': r,
                    }
                ],
            }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

        target_payload = self._build_target_markers_payload()
        if target_payload is not None:
            target_msg = String()
            target_msg.data = json.dumps(target_payload, ensure_ascii=False)
            self.target_pub.publish(target_msg)

        if self.visualize_gz:
            now = time.time()
            if not self._gz_ready and now - self._last_probe_t > 1.5:
                self._last_probe_t = now
                self._setup_gz_visual()
            if now - self._last_pose_set_t > 0.10:
                self._last_pose_set_t = now
                if self.mode == 'static_wall':
                    if not self._wall_spawned and self._gz_ready:
                        self._spawn_visual_obstacle()
                    if self._gz_ready and int(self.get_parameter('target_ball_enable').value) == 1:
                        if not self._target_ball_spawned:
                            self._spawn_target_ball()
                        ok_target_pose = self._set_target_ball_pose_enu(
                            float(target_ball_enu[0]),
                            float(target_ball_enu[1]),
                            float(target_ball_enu[2]),
                        )
                        if not ok_target_pose and now - self._last_pose_warn_t > 1.0:
                            self._last_pose_warn_t = now
                            self.get_logger().warn('target ball pose sync failed, will retry')
                else:
                    vis_x, vis_y, vis_z = self._next_visual_pose(x, y, z)
                    ok = self._set_visual_pose(vis_x, vis_y, vis_z)
                    if not ok and not self._spawn_done and self._gz_ready:
                        self._spawn_visual_obstacle()
                    elif ok:
                        self._visual_last_ok_x = float(vis_x)
                        self._visual_last_ok_y = float(vis_y)
                        self._visual_last_ok_z = float(vis_z)
                    elif not ok:
                        if now - self._last_pose_warn_t > 1.0:
                            self._last_pose_warn_t = now
                            self.get_logger().warn(
                                f'visual pose sync failed, keep last pose, entity={self.entity_name}, world={self.world_name}'
                            )

        now = time.time()
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            if self.mode == 'static_wall':
                self.get_logger().info(
                    f'wall publish: NED=({wall_x_ned:.2f},{wall_y_ned:.2f}), '
                    f'ENU=({x:.2f},{y:.2f},{z:.2f}), segments={len(payload["obstacles"])}'
                )
            else:
                self.get_logger().info(
                    f'obstacle publish (ENU for viz): x={x:.2f}, y={y:.2f}, z={z:.2f}, vx={vx:.2f}, vy={vy:.2f}, r={r:.2f}'
                )
            if target_payload is not None and len(target_payload.get('markers', [])) > 0:
                m0 = target_payload['markers'][0]
                self.get_logger().info(
                    f'target markers publish: count={len(target_payload["markers"])}, '
                    f'leader={target_payload.get("leader_id", 0)}, '
                    f'sample_ned=({m0["x"]:.2f},{m0["y"]:.2f},{m0["z"]:.2f}), '
                    f'sample_enu=({m0["enu_x"]:.2f},{m0["enu_y"]:.2f},{m0["enu_z"]:.2f})'
                )
            if target_payload is not None and len(target_payload.get('markers', [])) > 0 and now - self._last_frame_check_log_t > 10.0:
                self._last_frame_check_log_t = now
                m0 = target_payload['markers'][0]
                self.get_logger().info(
                    'frame-check(10s): same marker '
                    f'ENU=({m0["enu_x"]:.2f},{m0["enu_y"]:.2f},{m0["enu_z"]:.2f}) -> '
                    f'NED=({m0["x"]:.2f},{m0["y"]:.2f},{m0["z"]:.2f})'
                )
                self._log_target_table(target_payload, prefix='target-table(10s)')


def main():
    parser = argparse.ArgumentParser(description='Dynamic obstacle source for hybrid swarm tests')
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--world', type=str, default='default')
    parser.add_argument('--visualize-gz', type=int, default=1, help='1 to visualize obstacle entity in Gazebo')
    parser.add_argument('--mode', type=str, default='moving_ball', choices=['moving_ball', 'static_wall'])
    parser.add_argument('--start-center-x', type=float, default=18.0)
    parser.add_argument('--start-center-y', type=float, default=6.0)
    parser.add_argument('--active-center-x', type=float, default=5.5)
    parser.add_argument('--active-center-y', type=float, default=0.0)
    parser.add_argument('--switch-after-sec', type=float, default=38.0)
    parser.add_argument('--switch-fade-sec', type=float, default=6.0)
    parser.add_argument('--wall-x', type=float, default=5.5)
    parser.add_argument('--wall-y', type=float, default=0.0)
    parser.add_argument('--wall-z', type=float, default=0.9)
    parser.add_argument('--wall-length', type=float, default=5.0)
    parser.add_argument('--wall-thickness', type=float, default=0.25)
    parser.add_argument('--wall-height', type=float, default=1.8)
    parser.add_argument('--wall-segment-spacing', type=float, default=0.6)
    parser.add_argument('--target-ball-enable', '--target-marker-enable', dest='target_ball_enable', type=int, default=1)
    parser.add_argument('--target-ball-num-drones', '--target-marker-num-drones', dest='target_ball_num_drones', type=int, default=3)
    parser.add_argument('--target-ball-leader-id', '--target-marker-leader-id', dest='target_ball_leader_id', type=int, default=0)
    parser.add_argument('--target-ball-target-x', '--target-marker-target-x', dest='target_ball_target_x', type=float, default=20.0)
    parser.add_argument('--target-ball-target-z', '--target-marker-target-z', dest='target_ball_target_z', type=float, default=-3.0)
    parser.add_argument('--target-ball-base-y', '--target-marker-base-y', dest='target_ball_base_y', type=float, default=0.0)
    parser.add_argument('--target-ball-y-spacing', '--target-marker-y-spacing', dest='target_ball_y_spacing', type=float, default=1.8)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DynamicObstacleSource(
        rate_hz=args.rate,
        world_name=args.world,
        visualize_gz=(int(args.visualize_gz) == 1),
        mode=args.mode,
        start_center_x=args.start_center_x,
        start_center_y=args.start_center_y,
        active_center_x=args.active_center_x,
        active_center_y=args.active_center_y,
        switch_after_sec=args.switch_after_sec,
        switch_fade_sec=args.switch_fade_sec,
    )
    node.set_parameters([
        Parameter('wall_x', Parameter.Type.DOUBLE, float(args.wall_x)),
        Parameter('wall_y', Parameter.Type.DOUBLE, float(args.wall_y)),
        Parameter('wall_z', Parameter.Type.DOUBLE, float(args.wall_z)),
        Parameter('wall_length', Parameter.Type.DOUBLE, float(args.wall_length)),
        Parameter('wall_thickness', Parameter.Type.DOUBLE, float(args.wall_thickness)),
        Parameter('wall_height', Parameter.Type.DOUBLE, float(args.wall_height)),
        Parameter('wall_segment_spacing', Parameter.Type.DOUBLE, float(args.wall_segment_spacing)),
        Parameter('target_ball_enable', Parameter.Type.INTEGER, int(args.target_ball_enable)),
        Parameter('target_ball_num_drones', Parameter.Type.INTEGER, int(args.target_ball_num_drones)),
        Parameter('target_ball_leader_id', Parameter.Type.INTEGER, int(args.target_ball_leader_id)),
        Parameter('target_ball_target_x', Parameter.Type.DOUBLE, float(args.target_ball_target_x)),
        Parameter('target_ball_target_z', Parameter.Type.DOUBLE, float(args.target_ball_target_z)),
        Parameter('target_ball_base_y', Parameter.Type.DOUBLE, float(args.target_ball_base_y)),
        Parameter('target_ball_y_spacing', Parameter.Type.DOUBLE, float(args.target_ball_y_spacing)),
    ])
    if node.visualize_gz:
        node._setup_gz_visual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
