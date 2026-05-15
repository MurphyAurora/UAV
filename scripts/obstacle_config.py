#!/usr/bin/env python3
import argparse
import os
import shlex
from dataclasses import dataclass
from dataclasses import replace

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is available in the current ROS env.
    yaml = None


DEFAULT_CONFIG = {
    'walls': [
        {
            'name': 'front_wall',
            'x': 3.0,
            'y': 1.0,
            'z': 0.9,
            'length': 30.0,
            'thickness': 0.25,
            'height': 4.8,
            'segment_spacing': 0.6,
        },
        {
            'name': 'rear_right_wall',
            'x': 18.0,
            'y': 110.0,
            'z': 0.9,
            'length': 200.0,
            'thickness': 0.25,
            'height': 4.8,
            'segment_spacing': 0.6,
        },
        {
            'name': 'rear_left_wall',
            'x': 18.0,
            'y': -108.0,
            'z': 0.9,
            'length': 200.0,
            'thickness': 0.25,
            'height': 4.8,
            'segment_spacing': 0.6,
        },
    ],
    'target': {
        'x': 30.0,
        'y': 26.0,
        'z': -3.0,
        'y_spacing': 1.8,
    },
}


def _finite_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def _safe_name(value, default='obstacle'):
    text = str(value or default).strip().replace(' ', '_')
    return ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in text) or default


@dataclass
class Obstacle:
    name: str
    x: float
    y: float
    z: float

    def to_sdf_link(self, base_x=0.0, base_y=0.0, base_z=0.0, index=0):
        raise NotImplementedError

    def to_collision_points(self, start_obs_id=1):
        raise NotImplementedError

    def contains_xy(self, x, y, inflation=0.0):
        raise NotImplementedError

    def bounds_xy(self, inflation=0.0):
        raise NotImplementedError

    def planner_rect(self):
        return None

    def translated(self, dx=0.0, dy=0.0, dz=0.0):
        return replace(
            self,
            x=self.x + float(dx),
            y=self.y + float(dy),
            z=self.z + float(dz),
        )


@dataclass
class WallObstacle(Obstacle):
    length: float
    thickness: float
    height: float
    segment_spacing: float = 0.6

    @property
    def radius(self):
        return max(0.25, self.thickness * 0.5 + 0.15)

    def to_sdf_link(self, base_x=0.0, base_y=0.0, base_z=0.0, index=0):
        # The model is spawned with a 90 deg yaw in ENU, so convert NED-relative
        # offsets into the model-local frame used by SDF links.
        local_x = self.x - float(base_x)
        local_y = -(self.y - float(base_y))
        local_z = self.z - float(base_z)
        name = _safe_name(self.name, f'wall_{index + 1}')
        ambient = '0.55 0.55 0.55 1' if index == 0 else '0.50 0.50 0.50 1'
        return f"""
    <link name="{name}_link">
      <pose>{local_x:.3f} {local_y:.3f} {local_z:.3f} 0 0 0</pose>
      <collision name="{name}_collision">
        <geometry>
          <box>
            <size>{self.thickness:.3f} {self.length:.3f} {self.height:.3f}</size>
          </box>
        </geometry>
      </collision>
      <visual name="{name}_visual">
        <geometry>
          <box>
            <size>{self.thickness:.3f} {self.length:.3f} {self.height:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{ambient}</diffuse>
        </material>
      </visual>
    </link>"""

    def to_collision_points(self, start_obs_id=1):
        points = []
        segment_count = max(3, int(self.length / self.segment_spacing) + 1)
        half_span = self.length * 0.5
        step = self.length / max(1, segment_count - 1)
        for idx in range(segment_count):
            points.append(
                {
                    'obs_id': int(start_obs_id) + idx,
                    'x': self.x,
                    'y': self.y - half_span + step * idx,
                    'z': 0.0,
                    'vx': 0.0,
                    'vy': 0.0,
                    'vz': 0.0,
                    'radius': self.radius,
                }
            )
        return points

    def contains_xy(self, x, y, inflation=0.0):
        half_x = max(0.05, self.thickness * 0.5) + float(inflation)
        half_y = max(0.5, self.length * 0.5) + float(inflation)
        return abs(float(x) - self.x) <= half_x and abs(float(y) - self.y) <= half_y

    def bounds_xy(self, inflation=0.0):
        half_x = max(0.05, self.thickness * 0.5) + float(inflation)
        half_y = max(0.5, self.length * 0.5) + float(inflation)
        return self.x - half_x, self.x + half_x, self.y - half_y, self.y + half_y

    def planner_rect(self):
        return (self.x, self.y, self.length)


def load_obstacle_config(path):
    if not path:
        return DEFAULT_CONFIG
    expanded = os.path.expanduser(path)
    if not os.path.exists(expanded):
        return DEFAULT_CONFIG
    if yaml is None:
        raise RuntimeError('PyYAML is required to read obstacles.yaml')
    with open(expanded, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f'obstacle config must be a mapping: {expanded}')
    walls = data.get('walls', DEFAULT_CONFIG['walls'])
    target = data.get('target', DEFAULT_CONFIG['target'])
    if not isinstance(walls, list) or len(walls) == 0:
        walls = DEFAULT_CONFIG['walls']
    if not isinstance(target, dict):
        target = DEFAULT_CONFIG['target']
    return {'walls': walls, 'target': target}


def obstacles_from_config(config):
    obstacles = []
    for wall in normalized_walls(config):
        obstacles.append(WallObstacle(**wall))
    return obstacles


def normalized_walls(config):
    walls = []
    for idx, wall in enumerate(config.get('walls', []), start=1):
        if not isinstance(wall, dict):
            continue
        walls.append(
            {
                'name': str(wall.get('name', f'wall_{idx}')),
                'x': _finite_float(wall.get('x', 0.0)),
                'y': _finite_float(wall.get('y', 0.0)),
                'z': _finite_float(wall.get('z', 0.9)),
                'length': max(1.0, _finite_float(wall.get('length', 5.0), 5.0)),
                'thickness': max(0.1, _finite_float(wall.get('thickness', 0.25), 0.25)),
                'height': max(0.5, _finite_float(wall.get('height', 1.8), 1.8)),
                'segment_spacing': max(0.2, _finite_float(wall.get('segment_spacing', 0.6), 0.6)),
            }
        )
    return walls or normalized_walls(DEFAULT_CONFIG)


def normalized_target(config):
    target = config.get('target', {})
    if not isinstance(target, dict):
        target = {}
    default = DEFAULT_CONFIG['target']
    return {
        'x': _finite_float(target.get('x', default['x']), default['x']),
        'y': _finite_float(target.get('y', default['y']), default['y']),
        'z': _finite_float(target.get('z', default['z']), default['z']),
        'y_spacing': _finite_float(target.get('y_spacing', default['y_spacing']), default['y_spacing']),
    }


def shell_assignments(config):
    walls = normalized_walls(config)
    target = normalized_target(config)
    primary = walls[0]
    rear_walls = walls[1:]
    assignments = {
        'DYNAMIC_OBS_WALL_X': primary['x'],
        'DYNAMIC_OBS_WALL_Y': primary['y'],
        'DYNAMIC_OBS_WALL_Z': primary['z'],
        'DYNAMIC_OBS_WALL_LENGTH': primary['length'],
        'DYNAMIC_OBS_WALL_THICKNESS': primary['thickness'],
        'DYNAMIC_OBS_WALL_HEIGHT': primary['height'],
        'DYNAMIC_OBS_WALL_SEGMENT_SPACING': primary['segment_spacing'],
        'DYNAMIC_OBS_TARGET_X': target['x'],
        'DYNAMIC_OBS_TARGET_Y_BASE': target['y'],
        'MISSION_Z': target['z'],
        'DYNAMIC_OBS_TARGET_Y_SPACING': target['y_spacing'],
    }
    if rear_walls:
        right = max(rear_walls, key=lambda w: w['y'])
        assignments.update(
            {
                'SECOND_WALL_ENABLE': 1,
                'SECOND_WALL_DX': right['x'] - primary['x'],
                'SECOND_WALL_DY': abs(right['y'] - primary['y']),
                'REAR_WALL_LENGTH': right['length'],
                'REAR_WALL_GAP': max(0.0, 2.0 * abs(right['y'] - primary['y']) - right['length']),
                'THIRD_WALL_ENABLE': 1 if len(rear_walls) > 1 else 0,
            }
        )
    return assignments


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('path')
    parser.add_argument('--shell', action='store_true')
    args = parser.parse_args()
    config = load_obstacle_config(args.path)
    if args.shell:
        for key, value in shell_assignments(config).items():
            print(f'{key}={shlex.quote(str(value))}')


if __name__ == '__main__':
    main()
