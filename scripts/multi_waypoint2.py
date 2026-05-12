#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
import json
import math
import os
import time
import sys
import heapq

class FixedPointMission(Node):
    def __init__(self, num_drones=1, leader_id=1, metrics_csv_path=''):
        super().__init__('fixed_point_mission')
        self.num_drones = num_drones
        self.leader_id = leader_id
        self.pubs = {}
        self.vel_pubs = {}
        self.latest_states = {}
        self.latest_target_markers = {}
        self.latest_target_stamp = 0.0
        self._last_metric_log_t = 0.0
        self._last_failsafe_log_t = 0.0
        self.metrics_csv_path = metrics_csv_path
        self.metrics_file = None
        
        for i in range(1, num_drones + 1):
            pub = self.create_publisher(
                Pose, 
                f'/xtdrone2/x500_{i}/cmd_pose_local_ned', 
                10
            )
            self.pubs[i] = pub
            self.vel_pubs[i] = self.create_publisher(
                Twist,
                f'/xtdrone2/x500_{i}/cmd_vel_ned',
                10,
            )

        self.create_subscription(String, '/xtdrone2/swarm/state_exchange', self._state_cb, 10)
        self.create_subscription(String, '/xtdrone2/swarm/target_markers', self._target_markers_cb, 10)
        self._open_metrics_file()
        
        time.sleep(0.5)
        self.get_logger().info(f"初始化{num_drones}架无人机控制器，领航机=无人机{leader_id}")

    def _open_metrics_file(self):
        if not self.metrics_csv_path:
            return
        metrics_dir = os.path.dirname(os.path.abspath(self.metrics_csv_path))
        if metrics_dir:
            os.makedirs(metrics_dir, exist_ok=True)
        self.metrics_file = open(self.metrics_csv_path, 'w', buffering=1)
        self.metrics_file.write(
            'stamp,stage,leader_id,leader_x,leader_y,leader_z,'
            'virtual_x,virtual_y,virtual_z,target_x,target_y,target_z,'
            'formation_rms,formation_max,min_pair_dist,leader_cmd_vx,leader_cmd_vy,leader_cmd_vz\n'
        )
        self.get_logger().info(f'formation metrics csv: {self.metrics_csv_path}')

    def _state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            arr = data.get('states', [])
            tmp = {}
            for s in arr:
                drone_id = int(s['id'])
                tmp[drone_id] = {
                    'x': self._finite_float(s.get('x', 0.0)),
                    'y': self._finite_float(s.get('y', 0.0)),
                    'z': self._finite_float(s.get('z', 0.0)),
                    'vx': self._finite_float(s.get('vx', 0.0)),
                    'vy': self._finite_float(s.get('vy', 0.0)),
                    'vz': self._finite_float(s.get('vz', 0.0)),
                    'heading': self._finite_float(s.get('heading', 0.0)),
                    'stamp': float(s.get('stamp', 0.0)),
                }
            self.latest_states = tmp
        except Exception:
            return

    def _finite_float(self, value, default=0.0):
        try:
            v = float(value)
        except Exception:
            return default
        if not math.isfinite(v):
            return default
        return v

    def _state_fresh(self, st, max_age_sec=1.0):
        stamp = float(st.get('stamp', 0.0))
        if stamp <= 0.0:
            return False
        return (time.time() - stamp) <= max_age_sec

    def _rotate_offset_by_heading(self, dx, dy, heading):
        c = math.cos(heading)
        s = math.sin(heading)
        return dx * c - dy * s, dx * s + dy * c

    def _limit_vector(self, vx, vy, vz, max_norm):
        norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if norm > max_norm and norm > 1e-6:
            scale = max_norm / norm
            return vx * scale, vy * scale, vz * scale
        return vx, vy, vz

    def _distance_xyz(self, a_xyz, b_xyz):
        dx = float(a_xyz[0]) - float(b_xyz[0])
        dy = float(a_xyz[1]) - float(b_xyz[1])
        dz = float(a_xyz[2]) - float(b_xyz[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _state_xyz(self, st):
        return (float(st['x']), float(st['y']), float(st['z']))

    def _state_vel(self, st):
        return (
            self._finite_float(st.get('vx', 0.0)),
            self._finite_float(st.get('vy', 0.0)),
            self._finite_float(st.get('vz', 0.0)),
        )

    def _estimate_virtual_ref_from_states(self, follower_offsets, use_heading_offsets=False, heading=0.0, max_age_sec=1.0):
        centers = []
        for drone_id in range(1, self.num_drones + 1):
            st = self.latest_states.get(drone_id)
            if st is None or not self._state_fresh(st, max_age_sec=max_age_sec):
                continue
            dx, dy, dz = follower_offsets.get(drone_id, (0.0, 0.0, 0.0))
            if use_heading_offsets:
                rdx, rdy = self._rotate_offset_by_heading(dx, dy, heading)
            else:
                rdx, rdy = dx, dy
            centers.append((
                float(st['x']) - rdx,
                float(st['y']) - rdy,
                float(st['z']) - dz,
            ))
        if not centers:
            return None
        inv_n = 1.0 / float(len(centers))
        return (
            sum(p[0] for p in centers) * inv_n,
            sum(p[1] for p in centers) * inv_n,
            sum(p[2] for p in centers) * inv_n,
        )

    def _publish_vel(self, drone_id, vx, vy, vz, yawspeed=0.0):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.linear.y = float(vy)
        tw.linear.z = float(vz)
        tw.angular.z = float(yawspeed)
        self.vel_pubs[drone_id].publish(tw)

    def _publish_zero_vel_all(self):
        for drone_id in range(1, self.num_drones + 1):
            self._publish_vel(drone_id, 0.0, 0.0, 0.0)

    def _publish_pose(self, drone_id, x, y, z):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        self.pubs[drone_id].publish(pose)

    def _estimate_stage_duration(self, start_xyz, target_xyz, min_duration, max_speed, margin_sec=4.0):
        dx = float(target_xyz[0]) - float(start_xyz[0])
        dy = float(target_xyz[1]) - float(start_xyz[1])
        dz = float(target_xyz[2]) - float(start_xyz[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = max(float(max_speed), 0.1)
        return max(float(min_duration), dist / speed + margin_sec)

    def _advance_virtual_point(self, current_xyz, target_xyz, max_speed, dt):
        dx = float(target_xyz[0]) - float(current_xyz[0])
        dy = float(target_xyz[1]) - float(current_xyz[1])
        dz = float(target_xyz[2]) - float(current_xyz[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= 1e-6:
            return current_xyz, (0.0, 0.0, 0.0)
        step = min(dist, max(float(max_speed), 0.1) * dt)
        scale = step / dist
        next_xyz = (
            float(current_xyz[0]) + dx * scale,
            float(current_xyz[1]) + dy * scale,
            float(current_xyz[2]) + dz * scale,
        )
        return next_xyz, (
            (next_xyz[0] - float(current_xyz[0])) / dt,
            (next_xyz[1] - float(current_xyz[1])) / dt,
            (next_xyz[2] - float(current_xyz[2])) / dt,
        )

    def _build_manual_wall_path(
        self,
        start_xyz,
        goal_xyz,
        wall_x,
        wall_y,
        wall_length,
        route_side,
        bypass_margin,
        pre_wall_margin,
        cross_margin,
        lower_side_span,
        upper_side_span,
    ):
        _, sy, _ = (float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2]))
        gx, gy, gz = (float(goal_xyz[0]), float(goal_xyz[1]), float(goal_xyz[2]))
        wall_half = max(0.5, float(wall_length) * 0.5)
        if route_side == 'upper':
            bypass_y = float(wall_y) + wall_half + float(bypass_margin) + float(lower_side_span)
        else:
            bypass_y = float(wall_y) - wall_half - float(bypass_margin) - float(upper_side_span)

        pre_wall_x = float(wall_x) - float(pre_wall_margin)
        wall_cross_x = float(wall_x) + float(cross_margin)
        return [
            {
                'name': 'pre_wall',
                'point': (pre_wall_x, sy, gz),
                'min_duration': 8.0,
            },
            {
                'name': 'side_align',
                'point': (pre_wall_x, bypass_y, gz),
                'min_duration': 8.0,
            },
            {
                'name': 'wall_cross',
                'point': (wall_cross_x, bypass_y, gz),
                'min_duration': 10.0,
            },
            {
                'name': 'go_target',
                'point': (gx, gy, gz),
                'min_duration': None,
            },
        ]

    def _point_in_wall_rect(self, x, y, wall_x, wall_y, wall_length, wall_thickness, inflation):
        half_x = max(0.05, float(wall_thickness) * 0.5) + float(inflation)
        half_y = max(0.5, float(wall_length) * 0.5) + float(inflation)
        return (
            abs(float(x) - float(wall_x)) <= half_x
            and abs(float(y) - float(wall_y)) <= half_y
        )

    def _segment_hits_wall_rect(self, a_xy, b_xy, wall_x, wall_y, wall_length, wall_thickness, inflation, step):
        ax, ay = float(a_xy[0]), float(a_xy[1])
        bx, by = float(b_xy[0]), float(b_xy[1])
        dist = math.hypot(bx - ax, by - ay)
        samples = max(1, int(math.ceil(dist / max(float(step), 0.1))))
        for k in range(samples + 1):
            t = float(k) / float(samples)
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            if self._point_in_wall_rect(x, y, wall_x, wall_y, wall_length, wall_thickness, inflation):
                return True
        return False

    def _simplify_xy_path(self, path_xy, wall_x, wall_y, wall_length, wall_thickness, inflation, resolution):
        if len(path_xy) <= 2:
            return path_xy
        simplified = [path_xy[0]]
        i = 0
        while i < len(path_xy) - 1:
            j = len(path_xy) - 1
            while j > i + 1:
                if not self._segment_hits_wall_rect(
                    path_xy[i],
                    path_xy[j],
                    wall_x,
                    wall_y,
                    wall_length,
                    wall_thickness,
                    inflation,
                    max(float(resolution) * 0.5, 0.2),
                ):
                    break
                j -= 1
            simplified.append(path_xy[j])
            i = j
        return simplified

    def _filter_min_waypoint_dist(
        self,
        path_xy,
        min_waypoint_dist,
        wall_x,
        wall_y,
        wall_length,
        wall_thickness,
        inflation,
        resolution,
    ):
        if len(path_xy) <= 2 or float(min_waypoint_dist) <= 0.0:
            return path_xy
        filtered = [path_xy[0]]
        for idx in range(1, len(path_xy) - 1):
            candidate = path_xy[idx]
            if math.hypot(candidate[0] - filtered[-1][0], candidate[1] - filtered[-1][1]) >= min_waypoint_dist:
                filtered.append(candidate)
                continue
            next_point = path_xy[idx + 1]
            if self._segment_hits_wall_rect(
                filtered[-1],
                next_point,
                wall_x,
                wall_y,
                wall_length,
                wall_thickness,
                inflation,
                max(float(resolution) * 0.5, 0.2),
            ):
                filtered.append(candidate)
        filtered.append(path_xy[-1])
        return filtered

    def _path_length_xy(self, path_xy):
        if len(path_xy) <= 1:
            return 0.0
        total = 0.0
        for idx in range(1, len(path_xy)):
            total += math.hypot(
                float(path_xy[idx][0]) - float(path_xy[idx - 1][0]),
                float(path_xy[idx][1]) - float(path_xy[idx - 1][1]),
            )
        return total

    def _build_astar_wall_path(
        self,
        start_xyz,
        goal_xyz,
        wall_x,
        wall_y,
        wall_length,
        route_side,
        bypass_margin,
        lower_side_span,
        upper_side_span,
    ):
        sx, sy, _ = (float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2]))
        gx, gy, gz = (float(goal_xyz[0]), float(goal_xyz[1]), float(goal_xyz[2]))
        resolution = max(0.25, self._finite_float(os.environ.get('ASTAR_GRID_RESOLUTION', 1.0), 1.0))
        wall_thickness = max(0.05, self._finite_float(os.environ.get('ASTAR_WALL_THICKNESS', 0.25), 0.25))
        min_waypoint_dist = max(
            0.0,
            self._finite_float(os.environ.get('ASTAR_MIN_WAYPOINT_DIST', 2.0), 2.0),
        )
        smooth_enable = bool(int(self._finite_float(os.environ.get('ASTAR_SMOOTH_ENABLE', 1), 1)))
        formation_span = max(float(lower_side_span), float(upper_side_span))
        auto_inflation = max(0.5, float(bypass_margin) + formation_span)
        inflation_text = os.environ.get('ASTAR_OBS_INFLATION', '').strip()
        if inflation_text:
            inflation = max(0.0, self._finite_float(inflation_text, auto_inflation))
            inflation_source = 'env'
        else:
            inflation = auto_inflation
            inflation_source = 'auto'
        padding = max(6.0, inflation + 3.0)
        wall_half = max(0.5, float(wall_length) * 0.5)

        min_x = min(sx, gx, float(wall_x)) - padding
        max_x = max(sx, gx, float(wall_x)) + padding
        min_y = min(sy, gy, float(wall_y) - wall_half - inflation) - padding
        max_y = max(sy, gy, float(wall_y) + wall_half + inflation) + padding
        width = int(math.ceil((max_x - min_x) / resolution)) + 1
        height = int(math.ceil((max_y - min_y) / resolution)) + 1

        def to_cell(x, y):
            ix = int(round((float(x) - min_x) / resolution))
            iy = int(round((float(y) - min_y) / resolution))
            return (
                max(0, min(width - 1, ix)),
                max(0, min(height - 1, iy)),
            )

        def to_xy(cell):
            return (
                min_x + float(cell[0]) * resolution,
                min_y + float(cell[1]) * resolution,
            )

        def blocked(cell):
            x, y = to_xy(cell)
            return self._point_in_wall_rect(x, y, wall_x, wall_y, wall_length, wall_thickness, inflation)

        start = to_cell(sx, sy)
        goal = to_cell(gx, gy)
        if blocked(start) or blocked(goal):
            self.get_logger().warn('A* start or goal is inside inflated obstacle, fallback to manual_wall')
            return None

        side_sign = 1.0 if route_side == 'upper' else -1.0
        neighbors = [
            (1, 0, 1.0),
            (0, int(side_sign), 1.0),
            (1, int(side_sign), math.sqrt(2.0)),
            (1, -int(side_sign), math.sqrt(2.0)),
            (0, -int(side_sign), 1.0),
            (-1, int(side_sign), math.sqrt(2.0)),
            (-1, 0, 1.0),
            (-1, -int(side_sign), math.sqrt(2.0)),
        ]

        def heuristic(cell):
            return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

        open_heap = []
        counter = 0
        heapq.heappush(open_heap, (heuristic(start), counter, start))
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        max_iterations = max(1000, width * height)
        iterations = 0

        while open_heap and iterations < max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                cells = [current]
                while current in came_from:
                    current = came_from[current]
                    cells.append(current)
                cells.reverse()
                raw_xy = [(sx, sy)] + [to_xy(c) for c in cells[1:-1]] + [(gx, gy)]
                if smooth_enable:
                    simplified_xy = self._simplify_xy_path(
                        raw_xy,
                        wall_x,
                        wall_y,
                        wall_length,
                        wall_thickness,
                        inflation,
                        resolution,
                    )
                else:
                    simplified_xy = raw_xy
                filtered_xy = self._filter_min_waypoint_dist(
                    simplified_xy,
                    min_waypoint_dist,
                    wall_x,
                    wall_y,
                    wall_length,
                    wall_thickness,
                    inflation,
                    resolution,
                )
                path_length = self._path_length_xy(filtered_xy)
                waypoints = []
                for wp_index, (x, y) in enumerate(filtered_xy[1:], start=1):
                    is_goal = wp_index == len(filtered_xy) - 1
                    waypoints.append({
                        'name': 'go_target' if is_goal else f'astar_wp_{wp_index}',
                        'point': (float(x), float(y), gz),
                        'min_duration': None if is_goal else 4.0,
                    })
                self.get_logger().info(
                    f"A* planner: resolution={resolution:.2f}, grid={width}x{height}, "
                    f"inflation={inflation:.2f}({inflation_source}), "
                    f"min_waypoint_dist={min_waypoint_dist:.2f}, smooth={int(smooth_enable)}, "
                    f"raw_points={len(raw_xy)}, simplified_points={len(simplified_xy)}, "
                    f"waypoints={len(waypoints)}, path_length={path_length:.2f}, "
                    f"iterations={iterations}"
                )
                return waypoints

            closed.add(current)
            for dx, dy, move_cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if nxt[0] < 0 or nxt[0] >= width or nxt[1] < 0 or nxt[1] >= height:
                    continue
                if nxt in closed or blocked(nxt):
                    continue
                if dx != 0 and dy != 0:
                    if blocked((current[0] + dx, current[1])) or blocked((current[0], current[1] + dy)):
                        continue
                _, ny = to_xy(nxt)
                side_penalty = 0.0
                if (ny - float(wall_y)) * side_sign < 0.0:
                    side_penalty = 0.03
                tentative_g = g_score[current] + move_cost + side_penalty
                if tentative_g >= g_score.get(nxt, float('inf')):
                    continue
                came_from[nxt] = current
                g_score[nxt] = tentative_g
                counter += 1
                heapq.heappush(open_heap, (tentative_g + heuristic(nxt), counter, nxt))

        self.get_logger().warn(
            f"A* planner failed: grid={width}x{height}, iterations={iterations}, fallback to manual_wall"
        )
        return None

    def plan_leader_path(
        self,
        start_xyz,
        goal_xyz,
        planner_mode='manual_wall',
        wall_x=0.0,
        wall_y=0.0,
        wall_length=0.0,
        route_side='upper',
        bypass_margin=1.8,
        pre_wall_margin=3.5,
        cross_margin=6.0,
        lower_side_span=0.0,
        upper_side_span=0.0,
    ):
        """Return leader/virtual-leader waypoints.

        This is the single path-planner entry point. New planners should return
        the same waypoint dictionaries so the mission executor stays unchanged.
        """
        mode = str(planner_mode or 'manual_wall').strip().lower()
        gx, gy, gz = (float(goal_xyz[0]), float(goal_xyz[1]), float(goal_xyz[2]))

        if mode in ('direct', 'follow_target', 'none'):
            return [
                {
                    'name': 'go_target',
                    'point': (gx, gy, gz),
                    'min_duration': None,
                }
            ]

        if mode in ('astar', 'a_star'):
            astar_path = self._build_astar_wall_path(
                start_xyz,
                goal_xyz,
                wall_x,
                wall_y,
                wall_length,
                route_side,
                bypass_margin,
                lower_side_span,
                upper_side_span,
            )
            if astar_path:
                return astar_path

        if mode not in ('manual_wall', 'wall_follow', 'astar', 'a_star'):
            self.get_logger().warn(f"unknown planner_mode={planner_mode}, fallback to manual_wall")

        return self._build_manual_wall_path(
            start_xyz,
            (gx, gy, gz),
            wall_x,
            wall_y,
            wall_length,
            route_side,
            bypass_margin,
            pre_wall_margin,
            cross_margin,
            lower_side_span,
            upper_side_span,
        )

    def _center_drone_id(self):
        return max(1, min(self.num_drones, (self.num_drones + 1) // 2))

    def _write_metric_row(
        self,
        stage_name,
        leader_state,
        virtual_ref,
        leader_target,
        formation_rms,
        formation_max,
        min_pair_dist,
        leader_cmd,
    ):
        if self.metrics_file is None:
            return
        min_dist_text = '' if min_pair_dist is None else f'{min_pair_dist:.4f}'
        self.metrics_file.write(
            f'{time.time():.3f},{stage_name},{self.leader_id},'
            f'{float(leader_state["x"]):.4f},{float(leader_state["y"]):.4f},{float(leader_state["z"]):.4f},'
            f'{float(virtual_ref[0]):.4f},{float(virtual_ref[1]):.4f},{float(virtual_ref[2]):.4f},'
            f'{float(leader_target[0]):.4f},{float(leader_target[1]):.4f},{float(leader_target[2]):.4f},'
            f'{formation_rms:.4f},{formation_max:.4f},{min_dist_text},'
            f'{float(leader_cmd[0]):.4f},{float(leader_cmd[1]):.4f},{float(leader_cmd[2]):.4f}\n'
        )

    def _target_markers_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            markers = data.get('markers', [])
            tmp = {}
            for m in markers:
                drone_id = int(m.get('id', -1))
                if drone_id < 0:
                    continue
                tmp[drone_id] = (
                    float(m.get('x', 0.0)),
                    float(m.get('y', 0.0)),
                    float(m.get('z', 0.0)),
                )
            if tmp:
                self.latest_target_markers = tmp
                self.latest_target_stamp = float(data.get('stamp', time.time()))
        except Exception:
            return

    def _target_markers_fresh(self, max_age_sec=2.0):
        if self.latest_target_stamp <= 0.0:
            return False
        return (time.time() - self.latest_target_stamp) <= max_age_sec

    def wait_for_target_markers(self, timeout_sec=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._target_markers_fresh(max_age_sec=2.0):
                return True
        return False

    def compare_and_align_targets(self, target_stage, pos_tol=0.25):
        if not self._target_markers_fresh(max_age_sec=2.0):
            self.get_logger().warn('Gazebo target markers unavailable/fallback: keep mission targets unchanged')
            return target_stage, False

        aligned = dict(target_stage)
        any_changed = False
        for drone_id in range(1, self.num_drones + 1):
            code_target = target_stage.get(drone_id)
            gz_target = self.latest_target_markers.get(drone_id)
            if code_target is None or gz_target is None:
                continue
            dx = float(code_target[0]) - float(gz_target[0])
            dy = float(code_target[1]) - float(gz_target[1])
            dist_xy = (dx * dx + dy * dy) ** 0.5
            self.get_logger().info(
                f'目标对比 x500_{drone_id}: code=({code_target[0]:.2f},{code_target[1]:.2f},{code_target[2]:.2f}) '
                f'gz=({gz_target[0]:.2f},{gz_target[1]:.2f},{gz_target[2]:.2f}) diff_xy={dist_xy:.2f}'
            )
            if dist_xy > pos_tol:
                aligned[drone_id] = (float(gz_target[0]), float(gz_target[1]), float(code_target[2]))
                any_changed = True

        if any_changed:
            self.get_logger().warn('检测到目标不一致：任务目标已自动对齐到 Gazebo target marker (x,y)')
        else:
            self.get_logger().info('目标一致性检查通过：代码目标与 Gazebo target marker 一致')
        return aligned, any_changed

    def publish_stage(
        self,
        stage_name,
        waypoints_dict,
        duration=10.0,
        hz=10.0,
        wait_until_reached=False,
        reach_tolerance=0.35,
        reach_hold_sec=2.0,
        timeout_factor=3.0,
        state_timeout_sec=1.0,
    ):
        poses = {}
        for drone_id, (x, y, z) in waypoints_dict.items():
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            poses[drone_id] = pose
            self.get_logger().info(f"{stage_name}: 无人机{drone_id} -> ({x}, {y}, {z})")

        total_steps = max(1, int(duration * hz))
        sleep_dt = 1.0 / hz
        self.get_logger().info(f"{stage_name}: 持续 {duration:.1f}s, 发布频率 {hz:.1f}Hz")
        for _ in range(total_steps):
            for drone_id, pose in poses.items():
                self.pubs[drone_id].publish(pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(sleep_dt)
        if not wait_until_reached:
            return

        max_wait = max(float(duration), float(duration) * float(timeout_factor))
        reached_since = None
        wait_start = time.time()
        self.get_logger().info(
            f"{stage_name}: 等待全部无人机到达并保持，tol={reach_tolerance:.2f}m, "
            f"hold={reach_hold_sec:.1f}s, timeout={max_wait:.1f}s"
        )
        while time.time() - wait_start < max_wait:
            for drone_id, pose in poses.items():
                self.pubs[drone_id].publish(pose)
            rclpy.spin_once(self, timeout_sec=0.0)

            all_ok = True
            max_dist = 0.0
            for drone_id, (x, y, z) in waypoints_dict.items():
                st = self.latest_states.get(drone_id)
                if st is None or not self._state_fresh(st, max_age_sec=state_timeout_sec):
                    all_ok = False
                    break
                dx = float(st['x']) - float(x)
                dy = float(st['y']) - float(y)
                dz = float(st['z']) - float(z)
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                max_dist = max(max_dist, dist)
                if dist > reach_tolerance:
                    all_ok = False

            now_t = time.time()
            if all_ok:
                if reached_since is None:
                    reached_since = now_t
                if now_t - reached_since >= reach_hold_sec:
                    self.get_logger().info(
                        f"{stage_name}: 全部无人机已稳定到达，max_dist={max_dist:.2f}, "
                        f"elapsed={now_t - wait_start:.1f}s"
                    )
                    return
            else:
                reached_since = None

            time.sleep(sleep_dt)

        self.get_logger().warn(
            f"{stage_name}: 等待到达超时，继续任务；最后观测max_dist={max_dist:.2f}"
        )

    def evaluate_wall_and_target_success(
        self,
        wall_x,
        target_stage,
        mission_start,
        check_target=True,
        require_crossing=True,
        wall_clearance=0.6,
        target_tolerance=0.8,
        hold_sec=3.0,
        timeout_sec=30.0,
    ):
        crossed = {i: False for i in range(1, self.num_drones + 1)}
        crossed_since = {}
        reached_since = {}
        t0 = time.time()
        self.get_logger().info(
            f"开始判定避障成功: 条件=越过墙x>{wall_x + wall_clearance:.2f} 且到达目标点误差<{target_tolerance:.2f}并保持{hold_sec:.1f}s"
        )

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

            if len(self.latest_states) < self.num_drones:
                continue

            now_t = time.time()
            all_ok = True
            for drone_id in range(1, self.num_drones + 1):
                st = self.latest_states.get(drone_id)
                if st is None:
                    all_ok = False
                    continue
                if not self._state_fresh(st, max_age_sec=1.0):
                    all_ok = False
                    continue

                if st['x'] > wall_x + wall_clearance:
                    crossed[drone_id] = True
                    if drone_id not in crossed_since:
                        crossed_since[drone_id] = now_t

                tx, ty, tz = target_stage[drone_id]
                dx = st['x'] - tx
                dy = st['y'] - ty
                dz = st['z'] - tz
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5

                if check_target:
                    if dist <= target_tolerance:
                        if drone_id not in reached_since:
                            reached_since[drone_id] = now_t
                    else:
                        reached_since.pop(drone_id, None)

                    hold_ok = (drone_id in reached_since) and ((now_t - reached_since[drone_id]) >= hold_sec)
                    crossed_before_reached = (
                        (drone_id in crossed_since)
                        and (drone_id in reached_since)
                        and (crossed_since[drone_id] >= mission_start)
                        and (reached_since[drone_id] >= crossed_since[drone_id])
                    )
                    if require_crossing:
                        ok_flag = crossed[drone_id] and hold_ok and crossed_before_reached
                    else:
                        ok_flag = hold_ok
                    if not ok_flag:
                        all_ok = False
                else:
                    if require_crossing and (not crossed[drone_id]):
                        all_ok = False

            if all_ok:
                self.get_logger().info('判定结果: 避障成功（全部无人机已绕过墙并稳定到达目标点）')
                return True

        for drone_id in range(1, self.num_drones + 1):
            st = self.latest_states.get(drone_id)
            if st is None:
                self.get_logger().warn(f"无人机{drone_id}: 状态缺失，未完成成功判定")
                continue
            tx, ty, tz = target_stage[drone_id]
            dx = st['x'] - tx
            dy = st['y'] - ty
            dz = st['z'] - tz
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            self.get_logger().warn(
                f"无人机{drone_id}: crossed={crossed[drone_id]}, 末态=({st['x']:.2f},{st['y']:.2f},{st['z']:.2f}), target_dist={dist:.2f}"
            )
        if check_target:
            self.get_logger().warn('判定结果: 避障未成功（超时）')
        else:
            self.get_logger().warn('判定结果: 未完成过墙（超时）')
        return False

    def publish_follow_leader_stage(
        self,
        stage_name,
        leader_target,
        follower_offsets,
        duration=10.0,
        hz=10.0,
        formation_kp=0.55,
        leader_kp=0.45,
        max_follower_speed=1.0,
        max_leader_speed=0.8,
        use_heading_offsets=True,
        allow_marker_update=True,
        use_virtual_leader=True,
        state_timeout_sec=1.0,
        reach_tolerance=0.8,
        reach_hold_sec=0.8,
        stage_timeout_factor=3.0,
    ):
        sleep_dt = 1.0 / hz
        max_duration = max(float(duration), float(duration) * float(stage_timeout_factor))
        self.get_logger().info(
            f"{stage_name}: 领航跟随闭环阶段，计划 {duration:.1f}s, 超时 {max_duration:.1f}s, 发布频率 {hz:.1f}Hz, "
            f"Kp={formation_kp:.2f}, leader_Kp={leader_kp:.2f}, heading_offset={use_heading_offsets}, "
            f"virtual_leader={use_virtual_leader}, state_timeout={state_timeout_sec:.2f}s, "
            f"reach_tol={reach_tolerance:.2f}, hold={reach_hold_sec:.1f}s"
        )
        self.get_logger().info(
            f"{stage_name}: leader=x500_{self.leader_id} target=({leader_target[0]:.2f},{leader_target[1]:.2f},{leader_target[2]:.2f})"
        )
        if use_virtual_leader:
            self.get_logger().info(
                f"{stage_name}: virtual leader is an independent moving reference; "
                f"all physical drones track the virtual reference with formation offsets"
            )
        else:
            self.get_logger().info(
                f"{stage_name}: physical leader-following mode; followers track x500_{self.leader_id} actual position"
            )
        for drone_id in range(1, self.num_drones + 1):
            if (not use_virtual_leader) and drone_id == self.leader_id:
                continue
            dx, dy, dz = follower_offsets[drone_id]
            role_text = 'member' if use_virtual_leader else 'follower'
            self.get_logger().info(
                f"{stage_name}: {role_text}=x500_{drone_id} offset=({dx:.2f},{dy:.2f},{dz:.2f})"
            )

        leader_state = self.latest_states.get(self.leader_id)
        leader_state_fresh = leader_state is not None and self._state_fresh(leader_state, max_age_sec=state_timeout_sec)
        leader_heading0 = float(leader_state.get('heading', 0.0)) if leader_state_fresh else 0.0
        estimated_virtual_ref = self._estimate_virtual_ref_from_states(
            follower_offsets,
            use_heading_offsets=use_heading_offsets,
            heading=leader_heading0,
            max_age_sec=state_timeout_sec,
        )
        if use_virtual_leader and estimated_virtual_ref is not None:
            virtual_ref = estimated_virtual_ref
        elif leader_state_fresh:
            virtual_ref = self._state_xyz(leader_state)
        else:
            virtual_ref = (
                float(leader_target[0]),
                float(leader_target[1]),
                float(leader_target[2]),
            )
        rms_sum = 0.0
        rms_count = 0
        stage_max_error = 0.0
        stage_min_pair_dist = None
        reached_since = None
        stage_start_t = time.time()

        while time.time() - stage_start_t < max_duration:
            # Gazebo target marker changes should drive mission target updates.
            if allow_marker_update and self._target_markers_fresh(max_age_sec=2.0):
                gz_leader_target = self.latest_target_markers.get(self.leader_id)
                if gz_leader_target is not None:
                    leader_target = (
                        float(gz_leader_target[0]),
                        float(gz_leader_target[1]),
                        float(leader_target[2]),
                    )

            leader_state = self.latest_states.get(self.leader_id)
            leader_state_fresh = leader_state is not None and self._state_fresh(leader_state, max_age_sec=state_timeout_sec)
            metric_state = leader_state if leader_state_fresh else None
            if use_virtual_leader and metric_state is None:
                for candidate_id in range(1, self.num_drones + 1):
                    candidate_state = self.latest_states.get(candidate_id)
                    if candidate_state is not None and self._state_fresh(candidate_state, max_age_sec=state_timeout_sec):
                        metric_state = candidate_state
                        break
            if metric_state is None or ((not use_virtual_leader) and not leader_state_fresh):
                self._publish_zero_vel_all()
                now_t = time.time()
                if now_t - self._last_failsafe_log_t >= 1.0:
                    self._last_failsafe_log_t = now_t
                    self.get_logger().warn(
                        f'{stage_name}: required swarm state timeout, publish zero velocity failsafe'
                    )
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(sleep_dt)
                continue

            base_x = float(metric_state['x'])
            base_y = float(metric_state['y'])
            base_z = float(metric_state['z'])
            leader_heading = float(metric_state.get('heading', 0.0))
            if use_virtual_leader:
                max_virtual_lag = 0.0
                max_virtual_lag_drone = 0
                all_virtual_states_fresh = True
                for lag_drone_id in range(1, self.num_drones + 1):
                    lag_state = self.latest_states.get(lag_drone_id)
                    if lag_state is None or not self._state_fresh(lag_state, max_age_sec=state_timeout_sec):
                        all_virtual_states_fresh = False
                        break
                    dx, dy, dz = follower_offsets[lag_drone_id]
                    if use_heading_offsets:
                        rdx, rdy = self._rotate_offset_by_heading(dx, dy, leader_heading)
                    else:
                        rdx, rdy = dx, dy
                    lag_ref = (
                        float(virtual_ref[0]) + rdx,
                        float(virtual_ref[1]) + rdy,
                        float(virtual_ref[2]) + dz,
                    )
                    lag_dist = self._distance_xyz(self._state_xyz(lag_state), lag_ref)
                    if lag_dist > max_virtual_lag:
                        max_virtual_lag = lag_dist
                        max_virtual_lag_drone = lag_drone_id
                virtual_lag_limit = max(1.8, reach_tolerance * 2.0)
                if all_virtual_states_fresh and max_virtual_lag <= virtual_lag_limit:
                    virtual_ref, virtual_vel = self._advance_virtual_point(
                        virtual_ref,
                        leader_target,
                        max_leader_speed,
                        sleep_dt,
                    )
                else:
                    virtual_vel = (0.0, 0.0, 0.0)
                    now_t = time.time()
                    if now_t - self._last_failsafe_log_t >= 1.0:
                        self._last_failsafe_log_t = now_t
                        if all_virtual_states_fresh:
                            self.get_logger().warn(
                                f'{stage_name}: virtual leader paused, '
                                f'max_lag=x500_{max_virtual_lag_drone}:{max_virtual_lag:.2f}m '
                                f'> limit={virtual_lag_limit:.2f}m'
                            )
                        else:
                            self.get_logger().warn(f'{stage_name}: virtual leader paused, swarm state incomplete')
                leader_ref = virtual_ref
                leader_ff = virtual_vel
                formation_ref = virtual_ref
                formation_ff = virtual_vel
            else:
                leader_ref = (
                    float(leader_target[0]),
                    float(leader_target[1]),
                    float(leader_target[2]),
                )
                virtual_vel = (0.0, 0.0, 0.0)
                leader_ff = (0.0, 0.0, 0.0)
                formation_ref = (base_x, base_y, base_z)
                formation_ff = self._state_vel(leader_state)
                virtual_ref = formation_ref

            leader_vx = 0.0
            leader_vy = 0.0
            leader_vz = 0.0
            if not use_virtual_leader:
                lvx = float(leader_ff[0]) + leader_kp * (float(leader_ref[0]) - base_x)
                lvy = float(leader_ff[1]) + leader_kp * (float(leader_ref[1]) - base_y)
                lvz = float(leader_ff[2]) + leader_kp * (float(leader_ref[2]) - base_z)
                lvx, lvy, lvz = self._limit_vector(lvx, lvy, lvz, max_leader_speed)
                self._publish_vel(self.leader_id, lvx, lvy, lvz)
                leader_vx = lvx
                leader_vy = lvy
                leader_vz = lvz

            err_values = []
            for drone_id in range(1, self.num_drones + 1):
                if (not use_virtual_leader) and drone_id == self.leader_id:
                    continue
                dx, dy, dz = follower_offsets[drone_id]
                if use_heading_offsets:
                    rdx, rdy = self._rotate_offset_by_heading(dx, dy, leader_heading)
                else:
                    rdx, rdy = dx, dy
                ref_x = float(formation_ref[0]) + rdx
                ref_y = float(formation_ref[1]) + rdy
                ref_z = float(formation_ref[2]) + dz

                st = self.latest_states.get(drone_id)
                if st is None or not self._state_fresh(st, max_age_sec=state_timeout_sec):
                    self._publish_vel(drone_id, 0.0, 0.0, 0.0)
                    continue

                ex = ref_x - float(st['x'])
                ey = ref_y - float(st['y'])
                ez = ref_z - float(st['z'])
                vx = float(formation_ff[0]) + formation_kp * ex
                vy = float(formation_ff[1]) + formation_kp * ey
                vz = float(formation_ff[2]) + formation_kp * ez
                vx, vy, vz = self._limit_vector(vx, vy, vz, max_follower_speed)
                self._publish_vel(drone_id, vx, vy, vz)
                if drone_id == self.leader_id:
                    leader_vx = vx
                    leader_vy = vy
                    leader_vz = vz
                err_values.append(math.sqrt(ex * ex + ey * ey + ez * ez))

            dist_logs = []
            ref_dist_values = []
            all_refs_fresh = True
            for drone_id in range(1, self.num_drones + 1):
                st = self.latest_states.get(drone_id)
                if st is None or not self._state_fresh(st, max_age_sec=state_timeout_sec):
                    all_refs_fresh = False
                    continue
                if drone_id == self.leader_id and not use_virtual_leader:
                    tx, ty, tz = leader_target
                else:
                    dx, dy, dz = follower_offsets[drone_id]
                    if use_heading_offsets:
                        rdx, rdy = self._rotate_offset_by_heading(dx, dy, leader_heading)
                    else:
                        rdx, rdy = dx, dy
                    tx = float(formation_ref[0]) + rdx
                    ty = float(formation_ref[1]) + rdy
                    tz = float(formation_ref[2]) + dz
                dist = ((st['x'] - tx) ** 2 + (st['y'] - ty) ** 2 + (st['z'] - tz) ** 2) ** 0.5
                ref_dist_values.append(dist)
                dist_logs.append(f"x500_{drone_id}:{dist:.2f}")
            min_pair_dist = None
            for i in range(1, self.num_drones + 1):
                si = self.latest_states.get(i)
                if si is None or not self._state_fresh(si, max_age_sec=state_timeout_sec):
                    continue
                for j in range(i + 1, self.num_drones + 1):
                    sj = self.latest_states.get(j)
                    if sj is None or not self._state_fresh(sj, max_age_sec=state_timeout_sec):
                        continue
                    dij = math.sqrt(
                        (float(si['x']) - float(sj['x'])) ** 2
                        + (float(si['y']) - float(sj['y'])) ** 2
                        + (float(si['z']) - float(sj['z'])) ** 2
                    )
                    if min_pair_dist is None or dij < min_pair_dist:
                        min_pair_dist = dij
            now_t = time.time()
            if dist_logs and now_t - self._last_metric_log_t >= 1.0:
                self._last_metric_log_t = now_t
                virtual_target_dist = self._distance_xyz(virtual_ref, leader_target)
                self.get_logger().info(f"{stage_name}: target_dist=" + ", ".join(dist_logs))
                if err_values:
                    rms = math.sqrt(sum(e * e for e in err_values) / len(err_values))
                    rms_sum += rms
                    rms_count += 1
                    stage_max_error = max(stage_max_error, max(err_values))
                    if min_pair_dist is not None:
                        if stage_min_pair_dist is None or min_pair_dist < stage_min_pair_dist:
                            stage_min_pair_dist = min_pair_dist
                    min_dist_text = 'nan' if min_pair_dist is None else f'{min_pair_dist:.2f}'
                    self.get_logger().info(
                        f"{stage_name}: formation_error rms={rms:.2f}, max={max(err_values):.2f}, "
                        f"min_pair_dist={min_dist_text}, virtual_target_dist={virtual_target_dist:.2f}, "
                        f"leader_v=({leader_vx:.2f},{leader_vy:.2f},{leader_vz:.2f})"
                    )
                    self._write_metric_row(
                        stage_name,
                        metric_state,
                        virtual_ref,
                        leader_target,
                        rms,
                        max(err_values),
                        min_pair_dist,
                        (leader_vx, leader_vy, leader_vz),
                    )

            if all_refs_fresh and len(ref_dist_values) == self.num_drones:
                max_ref_dist = max(ref_dist_values)
                virtual_target_dist = self._distance_xyz(virtual_ref, leader_target)
                target_ready = (not use_virtual_leader) or (virtual_target_dist <= reach_tolerance)
                if target_ready and max_ref_dist <= reach_tolerance:
                    if reached_since is None:
                        reached_since = now_t
                    if now_t - reached_since >= reach_hold_sec:
                        self.get_logger().info(
                            f"{stage_name}: reached waypoint, max_ref_dist={max_ref_dist:.2f}, "
                            f"virtual_target_dist={virtual_target_dist:.2f}, elapsed={now_t - stage_start_t:.1f}s"
                        )
                        break
                else:
                    reached_since = None
            else:
                reached_since = None

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(sleep_dt)
        else:
            self.get_logger().warn(
                f"{stage_name}: waypoint reach timeout after {max_duration:.1f}s, continue to next stage"
            )

        if rms_count > 0:
            min_pair_text = 'nan' if stage_min_pair_dist is None else f'{stage_min_pair_dist:.2f}'
            self.get_logger().info(
                f'{stage_name}: stage_summary formation_rms_mean={rms_sum / rms_count:.2f}, '
                f'formation_max={stage_max_error:.2f}, min_pair_dist_min={min_pair_text}'
            )

    def hold_fixed_point(self, drone_id, x, y, z, duration=10):
        """单架无人机悬停在固定点"""
        if drone_id < 1 or drone_id > self.num_drones:
            self.get_logger().warn(f"无人机ID {drone_id} 超出范围！")
            return
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        self.get_logger().info(f"无人机{drone_id}悬停在固定点: ({x}, {y}, {z})")
        
        # 10Hz，持续 duration 秒
        for _ in range(int(duration * 10)):
            self.pubs[drone_id].publish(pose)
            time.sleep(0.1)

    def hold_all_fixed_points(self, waypoints_dict, duration=10):
        """
        所有无人机同时悬停在各自的固定点
        waypoints_dict: {drone_id: (x, y, z), ...}
        例如：{0: (2.0, 0.0, -2.0), 1: (2.0, 1.0, -2.0)}
        """
        poses = {}
        for drone_id, (x, y, z) in waypoints_dict.items():
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            poses[drone_id] = pose
            self.get_logger().info(
                f"无人机{drone_id}将悬停在: ({x}, {y}, {z})"
            )

        # 10Hz，持续 duration 秒，所有无人机同时发布
        for _ in range(int(duration * 10)):
            for drone_id, pose in poses.items():
                self.pubs[drone_id].publish(pose)
            time.sleep(0.1)

        self.get_logger().info(f"所有无人机完成 {duration}s 固定点悬停")


def main():
    num_drones = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    leader_id = int(sys.argv[3]) if len(sys.argv) > 3 else max(1, (num_drones + 1) // 2)
    wall_x = float(sys.argv[4]) if len(sys.argv) > 4 else 5.5
    wall_y = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    wall_length = float(sys.argv[6]) if len(sys.argv) > 6 else 5.0
    # 无墙验证时，目标点必须是显式坐标，不能再默认依赖 wall_x 推导
    target_x = float(sys.argv[7]) if len(sys.argv) > 7 else 20.0
    eval_mode = str(sys.argv[8]) if len(sys.argv) > 8 else 'cross_only'
    mission_mode = str(sys.argv[9]) if len(sys.argv) > 9 else 'wall_follow'
    y_spacing = float(sys.argv[10]) if len(sys.argv) > 10 else 1.8
    takeoff_z = float(sys.argv[11]) if len(sys.argv) > 11 else -2.0
    mission_z = float(sys.argv[12]) if len(sys.argv) > 12 else -3.0
    formation_kp = float(sys.argv[13]) if len(sys.argv) > 13 else 0.55
    leader_kp = float(sys.argv[14]) if len(sys.argv) > 14 else 0.45
    max_follower_speed = float(sys.argv[15]) if len(sys.argv) > 15 else 1.0
    max_leader_speed = float(sys.argv[16]) if len(sys.argv) > 16 else 0.8
    use_heading_offsets = bool(int(sys.argv[17])) if len(sys.argv) > 17 else True
    state_timeout_sec = float(sys.argv[18]) if len(sys.argv) > 18 else 1.0
    use_virtual_leader = bool(int(sys.argv[19])) if len(sys.argv) > 19 else True
    metrics_csv_path = str(sys.argv[20]) if len(sys.argv) > 20 else ''
    mission_start_x = float(sys.argv[21]) if len(sys.argv) > 21 else 0.0
    use_spawned_formation = bool(int(sys.argv[22])) if len(sys.argv) > 22 else False
    path_planner_mode = os.environ.get('PATH_PLANNER_MODE', 'astar').strip().lower()

    rclpy.init()
    node = FixedPointMission(
        num_drones=num_drones,
        leader_id=leader_id,
        metrics_csv_path=metrics_csv_path,
    )

    # 两种流程：无障碍主从跟随 / 墙前绕行
    takeoff_stage = {}
    target_stage = {}
    follower_offsets = {}
    # 机间避障safe_radius默认约1.2m，跟随队形间距需更大，否则会互相顶开导致难以到达目标
    world_wall_x = wall_x
    world_wall_y = wall_y
    world_target_x = target_x
    if use_spawned_formation:
        # PX4 local NED starts from each spawned vehicle origin. The visual wall/target
        # stay in world NED, so convert x into each vehicle's local frame. Lateral
        # formation is provided by spawn positions, so all members receive the same
        # local y references.
        wall_x = world_wall_x - mission_start_x
        wall_y = 0.0
        target_x = world_target_x - mission_start_x
        node.get_logger().info(
            f"spawned-formation local frame: world_wall_x={world_wall_x:.2f}, "
            f"world_target_x={world_target_x:.2f}, spawn_x={mission_start_x:.2f} -> "
            f"local_wall_x={wall_x:.2f}, local_target_x={target_x:.2f}"
        )

    wall_half = max(0.5, wall_length * 0.5)
    # 固定侧绕时，必须保证“靠墙那一侧最外侧飞机”也完整越过墙边。
    # 上侧绕行时补下方编队半宽；下侧绕行时补上方编队半宽。
    route_side = 'upper'
    lower_side_span = max(0.0, float(leader_id - 1) * y_spacing)
    upper_side_span = max(0.0, float(num_drones - leader_id) * y_spacing)
    bypass_margin = 1.8
    if route_side == 'upper':
        bypass_y_base = wall_y + wall_half + bypass_margin + lower_side_span
    else:
        bypass_y_base = wall_y - wall_half - bypass_margin - upper_side_span
    pre_wall_x = wall_x - 3.5
    wall_cross_x = wall_x + 6.0
    for i in range(1, num_drones + 1):
        formation_offset = (i - leader_id) * y_spacing
        if use_spawned_formation:
            y_offset = 0.0
            bypass_y = bypass_y_base - wall_y
            follower_offsets[i] = (0.0, 0.0, 0.0)
        else:
            y_offset = wall_y + formation_offset
            bypass_y = bypass_y_base + formation_offset
            follower_offsets[i] = (0.0, formation_offset, 0.0)
        if mission_mode == 'follow_target':
            takeoff_stage[i] = (0.0 if use_spawned_formation else mission_start_x, y_offset, takeoff_z)
            target_stage[i] = (target_x, y_offset, takeoff_z)
        else:
            takeoff_stage[i] = (0.0 if use_spawned_formation else mission_start_x, y_offset, takeoff_z)
            target_stage[i] = (target_x, y_offset if use_spawned_formation else bypass_y, mission_z)

    node.get_logger().info(
        f"主从设定: leader=x500_{leader_id}, followers={[i for i in range(1, num_drones + 1) if i != leader_id]}"
    )
    if mission_mode == 'follow_target':
        node.get_logger().info(
            f"无障碍跟随规划: target_x={target_x:.2f}, leader将发布主目标位姿，followers按固定偏置跟随"
        )
        node.get_logger().info(
            f"闭环参数: formation_kp={formation_kp:.2f}, leader_kp={leader_kp:.2f}, "
            f"max_follower_speed={max_follower_speed:.2f}, max_leader_speed={max_leader_speed:.2f}, "
            f"use_heading_offsets={use_heading_offsets}, state_timeout={state_timeout_sec:.2f}, "
            f"use_virtual_leader={use_virtual_leader}"
        )
        for i in range(1, num_drones + 1):
            tx, ty, tz = target_stage[i]
            node.get_logger().info(f"目标点: x500_{i} -> ({tx:.2f},{ty:.2f},{tz:.2f})")
    else:
        node.get_logger().info(
            f"绕墙规划: wall_x={wall_x:.2f}, wall_y={wall_y:.2f}, wall_length={wall_length:.2f}, "
            f"mission_start_x={mission_start_x:.2f}, pre_wall_x={pre_wall_x:.2f}, "
            f"bypass_y_base={bypass_y_base:.2f}, wall_cross_x={wall_cross_x:.2f}, "
            f"path_planner_mode={path_planner_mode}"
        )
        if use_heading_offsets:
            node.get_logger().warn('绕墙任务使用世界坐标队形，已自动关闭 heading offset 旋转，避免队形参考点转向墙体')
        use_heading_offsets = False
        node.get_logger().info(
            f"闭环参数: formation_kp={formation_kp:.2f}, leader_kp={leader_kp:.2f}, "
            f"max_follower_speed={max_follower_speed:.2f}, max_leader_speed={max_leader_speed:.2f}, "
            f"use_heading_offsets={use_heading_offsets}, state_timeout={state_timeout_sec:.2f}, "
            f"use_virtual_leader={use_virtual_leader}"
        )
    if eval_mode == 'cross_only':
        node.get_logger().info(f"成功判据: 仅判定过墙(x>{wall_x + 0.6:.2f})")
    elif eval_mode == 'target_only':
        node.get_logger().info("成功判据: 仅判定到达目标并保持3s（不依赖过墙）")
    else:
        node.get_logger().info(f"成功判据: 先过墙(x>{wall_x + 0.6:.2f})，再到目标并稳定保持3s")

    # 打印并比对真实目标点：若与 Gazebo target marker 不一致，自动对齐任务目标。
    got_target_markers = node.wait_for_target_markers(timeout_sec=3.0)
    if not got_target_markers:
        node.get_logger().warn('未在3秒内接收到 Gazebo target markers，将使用代码内目标点继续任务')
    if use_spawned_formation:
        node.get_logger().info('spawned-formation mode: skip Gazebo marker alignment because mission targets are local-frame targets')
    else:
        target_stage, _ = node.compare_and_align_targets(target_stage, pos_tol=0.25)
    node.get_logger().info('mission-target-table: begin')
    for i in range(1, num_drones + 1):
        tx, ty, tz = target_stage[i]
        node.get_logger().info(f"最终生效目标(NED) x500_{i}: ({tx:.2f},{ty:.2f},{tz:.2f})")
    node.get_logger().info('mission-target-table: end')

    leader_path = []
    if mission_mode != 'follow_target':
        leader_path = node.plan_leader_path(
            start_xyz=takeoff_stage[leader_id],
            goal_xyz=target_stage[leader_id],
            planner_mode=path_planner_mode,
            wall_x=wall_x,
            wall_y=wall_y,
            wall_length=wall_length,
            route_side=route_side,
            bypass_margin=bypass_margin,
            pre_wall_margin=3.5,
            cross_margin=6.0,
            lower_side_span=lower_side_span,
            upper_side_span=upper_side_span,
        )
        node.get_logger().info('planner-path-table: begin')
        for idx, wp in enumerate(leader_path, start=2):
            px, py, pz = wp['point']
            node.get_logger().info(
                f"stage{idx}_{wp['name']}: leader_ref=({px:.2f},{py:.2f},{pz:.2f}), "
                f"min_duration={wp['min_duration']}"
            )
        node.get_logger().info('planner-path-table: end')

    node.publish_stage(
        'stage1_takeoff_hold',
        takeoff_stage,
        duration=10.0,
        wait_until_reached=True,
        reach_tolerance=0.35,
        reach_hold_sec=2.0,
        timeout_factor=3.0,
        state_timeout_sec=state_timeout_sec,
    )
    leader_target = target_stage[leader_id]
    if mission_mode == 'follow_target':
        node.publish_follow_leader_stage(
            'stage2_follow_target',
            leader_target,
            follower_offsets,
            duration=float(duration),
            formation_kp=formation_kp,
            leader_kp=leader_kp,
            max_follower_speed=max_follower_speed,
            max_leader_speed=max_leader_speed,
            use_heading_offsets=use_heading_offsets,
            use_virtual_leader=use_virtual_leader,
            state_timeout_sec=state_timeout_sec,
        )
        node.publish_stage('stage3_target_hold', target_stage, duration=6.0)
    else:
        stage_plan = []
        prev_point = takeoff_stage[leader_id]
        for idx, wp in enumerate(leader_path, start=2):
            min_duration = wp['min_duration']
            if min_duration is None:
                min_duration = float(duration)
            stage_duration = node._estimate_stage_duration(
                prev_point,
                wp['point'],
                min_duration=min_duration,
                max_speed=max_leader_speed,
            )
            stage_plan.append((idx, wp['name'], wp['point'], stage_duration))
            prev_point = wp['point']

        duration_text = ', '.join(
            f"{name}={stage_duration:.1f}s" for _, name, _, stage_duration in stage_plan
        )
        node.get_logger().info(f"闭环阶段时长: {duration_text}")

        for idx, name, leader_ref, stage_duration in stage_plan:
            node.publish_follow_leader_stage(
                f'stage{idx}_{name}_lf',
                leader_ref,
                follower_offsets,
                duration=stage_duration,
                formation_kp=formation_kp,
                leader_kp=leader_kp,
                max_follower_speed=max_follower_speed,
                max_leader_speed=max_leader_speed,
                use_heading_offsets=use_heading_offsets,
                allow_marker_update=False,
                use_virtual_leader=use_virtual_leader,
                state_timeout_sec=state_timeout_sec,
            )
        node.publish_stage('stage6_target_hold', target_stage, duration=6.0)

    mission_start = time.time()

    ok = node.evaluate_wall_and_target_success(
        wall_x=wall_x,
        target_stage=target_stage,
        mission_start=mission_start,
        check_target=(eval_mode != 'cross_only'),
        require_crossing=(eval_mode != 'target_only'),
        wall_clearance=0.6,
        target_tolerance=0.8,
        hold_sec=3.0,
        timeout_sec=max(20.0, float(duration)),
    )
    if ok:
        node.get_logger().info('mission result: SUCCESS')
    else:
        node.get_logger().warn('mission result: FAILED')

    if node.metrics_file is not None:
        node.metrics_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
