#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 3 层：ROS/Gazebo 动态障碍发布节点。

输入：
    旧版 obstacles.yaml，或 scenes/*.yaml 场景文件。
输出：
    /xtdrone2/swarm/dynamic_obstacles 和 /xtdrone2/swarm/target_markers。

在 scene 模式下，它把可复现 YAML 场景实时转换为避障算法消费的 ROS topic。
Gazebo 可视化只做最小展示：显示第一个命名动态障碍；ROS topic 才是权威数据。
"""

import json
import math
import argparse
import csv
import time
import os
import tempfile
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from obstacle_config import WallObstacle, load_obstacle_config, normalized_target, normalized_walls, obstacles_from_config

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is expected in the ROS env.
    yaml = None


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GAZEBO_ASSET_DIR = os.path.join(SCRIPT_DIR, 'gazebo_assets')


def _finite_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def _vector3(value, default=(0.0, 0.0, 0.0)):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return tuple(float(v) for v in default)
    return tuple(_finite_float(v) for v in value)


def _resolve_path(path):
    expanded = os.path.expanduser(str(path or ''))
    if os.path.isabs(expanded):
        return expanded
    if os.path.exists(expanded):
        return expanded
    return os.path.join(SCRIPT_DIR, expanded)


def _asset_path(filename):
    return os.path.join(GAZEBO_ASSET_DIR, filename)


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
        self.scene_entity_name = 'scene_dynamic_obstacle_1'
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
        self.declare_parameter('obstacle_config', '')
        self.declare_parameter('scene_config', '')
        self.declare_parameter('second_wall_enable', 1)
        self.declare_parameter('second_wall_dx', 15.0)
        self.declare_parameter('second_wall_dy', 109.0)
        self.declare_parameter('rear_wall_length', 200.0)
        self.declare_parameter('third_wall_enable', 1)
        self.declare_parameter('target_ball_enable', 1)
        self.declare_parameter('target_ball_num_drones', 3)
        self.declare_parameter('target_ball_leader_id', 1)
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
        self.declare_parameter('trajectory_record_path', '')
        self.declare_parameter('trajectory_record_dt', 0.0)

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
        self._scene_cache_path = ''
        self._scene_cache_mtime = None
        self._scene_cache = {}
        self._scene_warned_no_config = False
        self._scene_warned_unsupported = set()
        self._last_trajectory_record_t = None
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
        elif self.mode == 'scene':
            sdf_path = self._prepare_scene_ball_sdf()
            entity_name = self.scene_entity_name
        else:
            sdf_path = _asset_path('dynamic_obstacle_ball.sdf')
            entity_name = self._visual_obstacle_entity_name()
        if not os.path.exists(sdf_path):
            self.get_logger().warn(f'visual obstacle sdf not found: {sdf_path}')
            return
        if self.mode == 'static_wall':
            base_wall = self._configured_walls()[0]
            wall_x_ned = float(base_wall['x'])
            wall_y_ned = float(base_wall['y'])
            wall_z = float(base_wall['z'])
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

        if self.mode == 'scene':
            scene_x, scene_y, scene_z = self._scene_visual_initial_pose_enu()
            req = (
                f'name: "{entity_name}", '
                f'allow_renaming: false, '
                f'sdf_filename: "{sdf_path}", '
                f'pose: {{ position: {{ x: {scene_x:.3f}, y: {scene_y:.3f}, z: {scene_z:.3f} }}, '
                f'orientation: {{ w: 1.0 }} }}'
            )
            ok = self._run_gz_service(self._gz_create_service, 'gz.msgs.EntityFactory', req, timeout_ms=1200)
            self._spawn_done = bool(ok)
            if self._spawn_done:
                self._visual_last_ok_x = scene_x
                self._visual_last_ok_y = scene_y
                self._visual_last_ok_z = scene_z
                self.get_logger().info(
                    f'gazebo scene obstacle spawned successfully: entity={entity_name}, '
                    f'ENU=({scene_x:.2f},{scene_y:.2f},{scene_z:.2f}), sdf={sdf_path}'
                )
            else:
                # If the entity already exists, set_pose may still succeed.
                self._spawn_done = self._set_visual_pose(scene_x, scene_y, scene_z, force=True)
                if self._spawn_done:
                    self._visual_last_ok_x = scene_x
                    self._visual_last_ok_y = scene_y
                    self._visual_last_ok_z = scene_z
                    self.get_logger().info(
                        f'gazebo scene obstacle already exists, pose sync enabled: entity={entity_name}'
                    )
                else:
                    self.get_logger().warn(
                        f'gazebo scene obstacle spawn failed: entity={entity_name}, sdf={sdf_path}'
                    )
            return

        # 先尝试直接设置位姿；若实体已存在即可立即联动
        if self._set_visual_pose(0.0, 0.0, float(self._p('z')), force=True):
            self._spawn_done = True
            self._visual_last_ok_x = 0.0
            self._visual_last_ok_y = 0.0
            self._visual_last_ok_z = float(self._p('z'))
            self.get_logger().info(f'gazebo visual obstacle already exists, pose sync enabled: entity={entity_name}')
            return

        req = (
            f'name: "{entity_name}", '
            f'allow_renaming: false, '
            f'sdf_filename: "{sdf_path}", '
            f'pose: {{ position: {{ x: 0.0, y: 0.0, z: {float(self._p("z")):.3f} }}, orientation: {{ w: 1.0 }} }}'
        )
        ok = self._run_gz_service(self._gz_create_service, 'gz.msgs.EntityFactory', req, timeout_ms=1200)
        self._spawn_done = bool(ok)
        if self._spawn_done:
            self.get_logger().info(f'gazebo visual obstacle spawned successfully (gz transport): entity={entity_name}')
        else:
            self.get_logger().warn(f'gazebo visual obstacle spawn failed (gz transport): entity={entity_name}')

    def _prepare_wall_sdf(self):
        obstacles = self._configured_obstacles()
        base = obstacles[0]
        link_sdf = ''.join(
            obs.to_sdf_link(base_x=base.x, base_y=base.y, base_z=base.z, index=idx)
            for idx, obs in enumerate(obstacles)
        )
        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="static_obstacle_wall">
    <static>true</static>
{link_sdf}
  </model>
</sdf>
"""
        tmp_path = os.path.join(tempfile.gettempdir(), 'xtd2_static_obstacle_wall_generated.sdf')
        with open(tmp_path, 'w', encoding='utf-8') as f:
            f.write(sdf)
        self.get_logger().info(
            f'wall visual sdf prepared from config: path={tmp_path}, obstacles={len(obstacles)}'
        )
        return tmp_path

    def _scene_visual_initial_pose_enu(self):
        scene = self._load_scene_config()
        dynamic = scene.get('dynamic_obstacles') or []
        for obs in dynamic:
            if not isinstance(obs, dict):
                continue
            x, y, z, _, _, _ = self._scene_dynamic_state(obs, 0.0)
            return self._ned_to_enu(x, y, z)
        return 0.0, 0.0, float(self._p('z'))

    def _prepare_scene_ball_sdf(self):
        scene = self._load_scene_config()
        radius = 0.5
        dynamic = scene.get('dynamic_obstacles') or []
        for obs in dynamic:
            if isinstance(obs, dict) and str(obs.get('name', '')).strip():
                radius = max(0.1, _finite_float(obs.get('radius'), 0.5))
                break
        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{self.scene_entity_name}">
    <static>true</static>
    <link name="body">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>{radius:.3f}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0.05 0.05 1</ambient>
          <diffuse>1 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        tmp_path = os.path.join(tempfile.gettempdir(), 'xtd2_scene_dynamic_obstacle_1.sdf')
        with open(tmp_path, 'w', encoding='utf-8') as f:
            f.write(sdf)
        self.get_logger().info(
            f'scene visual sdf prepared: path={tmp_path}, radius={radius:.2f}'
        )
        return tmp_path

    def _set_visual_pose(self, x: float, y: float, z: float, force: bool = False) -> bool:
        if not self._gz_ready:
            return False
        if not force and not self._spawn_done:
            return False
        req = (
            f'name: "{self._visual_obstacle_entity_name()}", '
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
        sdf_path = _asset_path('dynamic_obstacle_ball.sdf')
        if not os.path.exists(sdf_path):
            self.get_logger().warn(f'target ball sdf not found: {sdf_path}')
            return

        target_cfg = self._configured_target()
        target_x = float(target_cfg['x'])
        target_z = float(target_cfg['z'])
        target_y = float(target_cfg['y'])
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
        target_cfg = self._configured_target()
        target_x = float(target_cfg['x'])
        target_z = float(target_cfg['z'])
        base_y = float(target_cfg['y'])
        spacing = float(target_cfg['y_spacing'])

        markers = []
        for i in range(1, num_drones + 1):
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

    def _visual_obstacle_entity_name(self):
        if self.mode == 'scene':
            return self.scene_entity_name
        return self.entity_name

    def _load_scene_config(self):
        config_path = str(self.get_parameter('scene_config').value or '').strip()
        if not config_path:
            if not self._scene_warned_no_config:
                self._scene_warned_no_config = True
                self.get_logger().warn('scene mode selected but scene_config is empty')
            return {}
        scene_path = _resolve_path(config_path)
        try:
            mtime = os.path.getmtime(scene_path)
        except Exception as e:
            self.get_logger().warn(f'scene config not readable: path={scene_path}, error={e}')
            return {}
        if self._scene_cache_path == scene_path and self._scene_cache_mtime == mtime:
            return self._scene_cache
        if yaml is None:
            raise RuntimeError('PyYAML is required to read scene YAML files')
        with open(scene_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError(f'scene config must be a mapping: {scene_path}')
        self._scene_cache_path = scene_path
        self._scene_cache_mtime = mtime
        self._scene_cache = data
        meta = data.get('scene') or {}
        dyn = data.get('dynamic_obstacles') or []
        walls = ((data.get('static_obstacles') or {}).get('walls')) or []
        self.get_logger().info(
            f'scene config loaded: path={scene_path}, name={meta.get("name", "unnamed")}, '
            f'dynamic_obstacles={len(dyn)}, static_walls={len(walls)}'
        )
        return data

    def _scene_walls(self, scene):
        walls = ((scene.get('static_obstacles') or {}).get('walls')) or []
        normalized = []
        for idx, wall in enumerate(walls, start=1):
            if not isinstance(wall, dict):
                continue
            normalized.append(
                {
                    'name': str(wall.get('name', f'wall_{idx}')),
                    'x': _finite_float(wall.get('x'), 0.0),
                    'y': _finite_float(wall.get('y'), 0.0),
                    'z': _finite_float(wall.get('z'), 0.9),
                    'length': max(1.0, _finite_float(wall.get('length'), 5.0)),
                    'thickness': max(0.1, _finite_float(wall.get('thickness'), 0.25)),
                    'height': max(0.5, _finite_float(wall.get('height'), 1.8)),
                    'segment_spacing': max(0.2, _finite_float(wall.get('segment_spacing'), 0.6)),
                }
            )
        return normalized

    def _scene_line_state(self, traj, t):
        start_time = _finite_float(traj.get('start_time'), 0.0)
        start = _vector3(traj.get('start'))
        end = _vector3(traj.get('end'), start)
        speed = max(0.0, _finite_float(traj.get('speed'), 0.0))
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= 1e-9 or speed <= 1e-9 or t <= start_time:
            return (*start, 0.0, 0.0, 0.0)
        alpha = min(1.0, (t - start_time) * speed / dist)
        x = start[0] + dx * alpha
        y = start[1] + dy * alpha
        z = start[2] + dz * alpha
        if alpha >= 1.0:
            return (x, y, z, 0.0, 0.0, 0.0)
        return (x, y, z, dx / dist * speed, dy / dist * speed, dz / dist * speed)

    def _scene_circle_state(self, traj, t):
        start_time = _finite_float(traj.get('start_time'), 0.0)
        center = _vector3(traj.get('center'))
        radius = max(0.0, _finite_float(traj.get('radius'), 1.0))
        omega = _finite_float(traj.get('angular_speed'), 0.0)
        phase = _finite_float(traj.get('phase'), 0.0)
        active_t = max(0.0, t - start_time)
        theta = phase + omega * active_t
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        z = center[2]
        if t < start_time:
            return (x, y, z, 0.0, 0.0, 0.0)
        vx = -radius * omega * math.sin(theta)
        vy = radius * omega * math.cos(theta)
        return (x, y, z, vx, vy, 0.0)

    def _scene_dynamic_state(self, obs, t):
        traj = obs.get('trajectory') or {}
        traj_type = str(traj.get('type', 'line')).strip().lower()
        if traj_type == 'circle':
            return self._scene_circle_state(traj, t)
        if traj_type not in ('line', 'static'):
            name = str(obs.get('name', 'dynamic_obstacle'))
            key = (name, traj_type)
            if key not in self._scene_warned_unsupported:
                self._scene_warned_unsupported.add(key)
                self.get_logger().warn(f'unsupported scene trajectory type={traj_type}, fallback to line: obstacle={name}')
        return self._scene_line_state(traj, t)

    def _build_scene_payload(self, t):
        scene = self._load_scene_config()
        obstacles = []
        obs_id = 1

        for wall in self._scene_walls(scene):
            points = WallObstacle(**wall).to_collision_points(start_obs_id=obs_id)
            obstacles.extend(points)
            obs_id += len(points)

        for obs in scene.get('dynamic_obstacles') or []:
            if not isinstance(obs, dict):
                continue
            x, y, z, vx, vy, vz = self._scene_dynamic_state(obs, t)
            obstacles.append(
                {
                    'obs_id': obs_id,
                    'name': str(obs.get('name', f'dynamic_obstacle_{obs_id}')),
                    'x': x,
                    'y': y,
                    'z': z,
                    'vx': vx,
                    'vy': vy,
                    'vz': vz,
                    'radius': max(0.05, _finite_float(obs.get('radius'), 0.5)),
                }
            )
            obs_id += 1

        return {'stamp': time.time(), 'scene_time': t, 'obstacles': obstacles}

    def _dynamic_items_for_record(self, payload):
        items = []
        for idx, obs in enumerate(payload.get('obstacles', []), start=1):
            name = str(obs.get('name', '')).strip()
            if self.mode == 'scene' and not name:
                continue
            if self.mode == 'static_wall':
                continue
            obstacle_id = name or f'obs_{obs.get("obs_id", idx)}'
            items.append(
                {
                    'time': float(payload.get('scene_time', time.time() - self.t0)),
                    'obstacle_id': obstacle_id,
                    'x': float(obs.get('x', 0.0)),
                    'y': float(obs.get('y', 0.0)),
                    'z': float(obs.get('z', 0.0)),
                    'radius': float(obs.get('radius', 0.5)),
                }
            )
        return items

    def _record_obstacle_trajectory(self, payload):
        record_path = str(self.get_parameter('trajectory_record_path').value or '').strip()
        record_dt = float(self.get_parameter('trajectory_record_dt').value)
        if not record_path or record_dt <= 0.0:
            return

        scene_time = float(payload.get('scene_time', time.time() - self.t0))
        if self._last_trajectory_record_t is not None:
            if scene_time - self._last_trajectory_record_t < record_dt - 1e-9:
                return

        items = self._dynamic_items_for_record(payload)
        if not items:
            return

        path = _resolve_path(record_path)
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        need_header = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=['time', 'obstacle_id', 'x', 'y', 'z', 'radius'])
            if need_header:
                writer.writeheader()
            for item in items:
                writer.writerow(
                    {
                        'time': f"{item['time']:.4f}",
                        'obstacle_id': item['obstacle_id'],
                        'x': f"{item['x']:.4f}",
                        'y': f"{item['y']:.4f}",
                        'z': f"{item['z']:.4f}",
                        'radius': f"{item['radius']:.4f}",
                    }
                )
        self._last_trajectory_record_t = scene_time

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

    def _configured_walls(self):
        config_path = str(self.get_parameter('obstacle_config').value or '').strip()
        if config_path:
            return normalized_walls(load_obstacle_config(config_path))

        wall_x_ned = float(self._p('wall_x'))
        wall_y_ned = float(self._p('wall_y'))
        wall_length = max(1.0, float(self._p('wall_length')))
        rear_wall_length = max(wall_length, float(self._p('rear_wall_length')))
        wall = {
            'name': 'front_wall',
            'x': wall_x_ned,
            'y': wall_y_ned,
            'z': float(self._p('wall_z')),
            'length': wall_length,
            'thickness': max(0.1, float(self._p('wall_thickness'))),
            'height': max(0.5, float(self._p('wall_height'))),
            'segment_spacing': max(0.2, float(self._p('wall_segment_spacing'))),
        }
        walls = [wall]
        if int(self._p('second_wall_enable')) != 0:
            walls.append(
                {
                    **wall,
                    'name': 'rear_right_wall',
                    'x': wall_x_ned + float(self._p('second_wall_dx')),
                    'y': wall_y_ned + float(self._p('second_wall_dy')),
                    'length': rear_wall_length,
                }
            )
        if int(self._p('third_wall_enable')) != 0:
            walls.append(
                {
                    **wall,
                    'name': 'rear_left_wall',
                    'x': wall_x_ned + float(self._p('second_wall_dx')),
                    'y': wall_y_ned - float(self._p('second_wall_dy')),
                    'length': rear_wall_length,
                }
            )
        return walls

    def _configured_obstacles(self):
        config_path = str(self.get_parameter('obstacle_config').value or '').strip()
        if config_path:
            return obstacles_from_config(load_obstacle_config(config_path))
        return [WallObstacle(**wall) for wall in self._configured_walls()]

    def _configured_target(self):
        scene_path = str(self.get_parameter('scene_config').value or '').strip()
        if self.mode == 'scene' and scene_path:
            scene = self._load_scene_config()
            target = scene.get('target') or {}
            if isinstance(target, dict):
                default = {'x': 20.0, 'y': 0.0, 'z': -3.0, 'y_spacing': 1.8}
                return {
                    'x': _finite_float(target.get('x', default['x']), default['x']),
                    'y': _finite_float(target.get('y', default['y']), default['y']),
                    'z': _finite_float(target.get('z', default['z']), default['z']),
                    'y_spacing': _finite_float(target.get('y_spacing', default['y_spacing']), default['y_spacing']),
                }
        config_path = str(self.get_parameter('obstacle_config').value or '').strip()
        if config_path:
            return normalized_target(load_obstacle_config(config_path))
        return {
            'x': float(self.get_parameter('target_ball_target_x').value),
            'y': float(self.get_parameter('target_ball_base_y').value),
            'z': float(self.get_parameter('target_ball_target_z').value),
            'y_spacing': float(self.get_parameter('target_ball_y_spacing').value),
        }

    def _publish(self):
        t = time.time() - self.t0
        z = self._p('z')

        if self.mode == 'scene':
            payload = self._build_scene_payload(t)
            dynamic_items = [
                obs for obs in payload.get('obstacles', [])
                if str(obs.get('name', '')).strip()
            ]
            if dynamic_items:
                first = dynamic_items[0]
                x, y, z = self._ned_to_enu(first.get('x', 0.0), first.get('y', 0.0), first.get('z', 0.0))
                vx, vy, _ = self._ned_to_enu(first.get('vx', 0.0), first.get('vy', 0.0), 0.0)
                r = float(first.get('radius', 0.3))
            else:
                x = y = vx = vy = 0.0
                r = 0.3
        elif self.mode == 'static_wall':
            obstacles_cfg = self._configured_obstacles()
            base_obstacle = obstacles_cfg[0]
            wall_x_ned = float(base_obstacle.x)
            wall_y_ned = float(base_obstacle.y)
            obstacles = []
            obs_id = 1
            for obs in obstacles_cfg:
                points = obs.to_collision_points(start_obs_id=obs_id)
                obstacles.extend(points)
                obs_id += len(points)
            payload = {'stamp': time.time(), 'obstacles': obstacles}
            x, y, _ = self._ned_to_enu(wall_x_ned, wall_y_ned, 0.0)
            vx = 0.0
            vy = 0.0
            r = getattr(base_obstacle, 'radius', 0.3)

            # Compute current target ball ENU pose from NED target parameters.
            target_cfg = self._configured_target()
            target_x_ned = float(target_cfg['x'])
            target_z_ned = float(target_cfg['z'])
            base_y_ned = float(target_cfg['y'])
            target_y_ned = base_y_ned
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
        self._record_obstacle_trajectory(payload)

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
                                f'visual pose sync failed, keep last pose, '
                                f'entity={self._visual_obstacle_entity_name()}, world={self.world_name}'
                            )

        now = time.time()
        if now - self._last_log_t > 1.0:
            self._last_log_t = now
            if self.mode == 'static_wall':
                self.get_logger().info(
                    f'wall publish: NED=({wall_x_ned:.2f},{wall_y_ned:.2f}), '
                    f'ENU=({x:.2f},{y:.2f},{z:.2f}), segments={len(payload["obstacles"])}'
                )
            elif self.mode == 'scene':
                dynamic_count = len([obs for obs in payload.get('obstacles', []) if str(obs.get('name', '')).strip()])
                self.get_logger().info(
                    f'scene publish: t={payload.get("scene_time", 0.0):.2f}, '
                    f'obstacles={len(payload["obstacles"])}, dynamic={dynamic_count}'
                )
            else:
                self.get_logger().info(
                    f'obstacle publish (ENU for viz): x={x:.2f}, y={y:.2f}, z={z:.2f}, vx={vx:.2f}, vy={vy:.2f}, r={r:.2f}'
                )
            if target_payload is not None and len(target_payload.get('markers', [])) > 0:
                m0 = target_payload['markers'][0]
                self.get_logger().info(
                    f'target markers publish: count={len(target_payload["markers"])}, '
                    f'leader={target_payload.get("leader_id", 1)}, '
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
    parser.add_argument('--mode', type=str, default='moving_ball', choices=['moving_ball', 'static_wall', 'scene'])
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
    parser.add_argument('--obstacle-config', type=str, default='')
    parser.add_argument('--scene-config', type=str, default='')
    parser.add_argument('--second-wall-enable', type=int, default=1)
    parser.add_argument('--second-wall-dx', type=float, default=15.0)
    parser.add_argument('--second-wall-dy', type=float, default=109.0)
    parser.add_argument('--rear-wall-length', type=float, default=200.0)
    parser.add_argument('--third-wall-enable', type=int, default=1)
    parser.add_argument('--target-ball-enable', '--target-marker-enable', dest='target_ball_enable', type=int, default=1)
    parser.add_argument('--target-ball-num-drones', '--target-marker-num-drones', dest='target_ball_num_drones', type=int, default=3)
    parser.add_argument('--target-ball-leader-id', '--target-marker-leader-id', dest='target_ball_leader_id', type=int, default=1)
    parser.add_argument('--target-ball-target-x', '--target-marker-target-x', dest='target_ball_target_x', type=float, default=20.0)
    parser.add_argument('--target-ball-target-z', '--target-marker-target-z', dest='target_ball_target_z', type=float, default=-3.0)
    parser.add_argument('--target-ball-base-y', '--target-marker-base-y', dest='target_ball_base_y', type=float, default=0.0)
    parser.add_argument('--target-ball-y-spacing', '--target-marker-y-spacing', dest='target_ball_y_spacing', type=float, default=1.8)
    parser.add_argument('--trajectory-record-path', type=str, default='')
    parser.add_argument('--trajectory-record-dt', type=float, default=0.0)
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
        Parameter('obstacle_config', Parameter.Type.STRING, str(args.obstacle_config)),
        Parameter('scene_config', Parameter.Type.STRING, str(args.scene_config)),
        Parameter('second_wall_enable', Parameter.Type.INTEGER, int(args.second_wall_enable)),
        Parameter('second_wall_dx', Parameter.Type.DOUBLE, float(args.second_wall_dx)),
        Parameter('second_wall_dy', Parameter.Type.DOUBLE, float(args.second_wall_dy)),
        Parameter('rear_wall_length', Parameter.Type.DOUBLE, float(args.rear_wall_length)),
        Parameter('third_wall_enable', Parameter.Type.INTEGER, int(args.third_wall_enable)),
        Parameter('target_ball_enable', Parameter.Type.INTEGER, int(args.target_ball_enable)),
        Parameter('target_ball_num_drones', Parameter.Type.INTEGER, int(args.target_ball_num_drones)),
        Parameter('target_ball_leader_id', Parameter.Type.INTEGER, int(args.target_ball_leader_id)),
        Parameter('target_ball_target_x', Parameter.Type.DOUBLE, float(args.target_ball_target_x)),
        Parameter('target_ball_target_z', Parameter.Type.DOUBLE, float(args.target_ball_target_z)),
        Parameter('target_ball_base_y', Parameter.Type.DOUBLE, float(args.target_ball_base_y)),
        Parameter('target_ball_y_spacing', Parameter.Type.DOUBLE, float(args.target_ball_y_spacing)),
        Parameter('trajectory_record_path', Parameter.Type.STRING, str(args.trajectory_record_path)),
        Parameter('trajectory_record_dt', Parameter.Type.DOUBLE, float(args.trajectory_record_dt)),
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
