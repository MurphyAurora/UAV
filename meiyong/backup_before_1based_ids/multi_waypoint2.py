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

class FixedPointMission(Node):
    def __init__(self, num_drones=1, leader_id=0, metrics_csv_path=''):
        super().__init__('fixed_point_mission')
        self.num_drones = num_drones
        self.leader_id = leader_id
        self.pubs = []
        self.vel_pubs = []
        self.latest_states = {}
        self.latest_target_markers = {}
        self.latest_target_stamp = 0.0
        self._last_metric_log_t = 0.0
        self._last_failsafe_log_t = 0.0
        self.metrics_csv_path = metrics_csv_path
        self.metrics_file = None
        
        for i in range(num_drones):
            pub = self.create_publisher(
                Pose, 
                f'/xtdrone2/x500_{i}/cmd_pose_local_ned', 
                10
            )
            self.pubs.append(pub)
            self.vel_pubs.append(
                self.create_publisher(
                    Twist,
                    f'/xtdrone2/x500_{i}/cmd_vel_ned',
                    10,
                )
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

    def _publish_vel(self, drone_id, vx, vy, vz, yawspeed=0.0):
        tw = Twist()
        tw.linear.x = float(vx)
        tw.linear.y = float(vy)
        tw.linear.z = float(vz)
        tw.angular.z = float(yawspeed)
        self.vel_pubs[drone_id].publish(tw)

    def _publish_zero_vel_all(self):
        for drone_id in range(self.num_drones):
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
        for drone_id in range(self.num_drones):
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

    def publish_stage(self, stage_name, waypoints_dict, duration=10.0, hz=10.0):
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
        crossed = {i: False for i in range(self.num_drones)}
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
            for drone_id in range(self.num_drones):
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

        for drone_id in range(self.num_drones):
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
    ):
        total_steps = max(1, int(duration * hz))
        sleep_dt = 1.0 / hz
        self.get_logger().info(
            f"{stage_name}: 领航跟随闭环阶段，持续 {duration:.1f}s, 发布频率 {hz:.1f}Hz, "
            f"Kp={formation_kp:.2f}, leader_Kp={leader_kp:.2f}, heading_offset={use_heading_offsets}, "
            f"virtual_leader={use_virtual_leader}, state_timeout={state_timeout_sec:.2f}s"
        )
        self.get_logger().info(
            f"{stage_name}: leader=x500_{self.leader_id} target=({leader_target[0]:.2f},{leader_target[1]:.2f},{leader_target[2]:.2f})"
        )
        for drone_id in range(self.num_drones):
            if drone_id == self.leader_id:
                continue
            dx, dy, dz = follower_offsets[drone_id]
            self.get_logger().info(
                f"{stage_name}: follower=x500_{drone_id} offset=({dx:.2f},{dy:.2f},{dz:.2f})"
            )

        leader_state = self.latest_states.get(self.leader_id)
        if leader_state is not None and self._state_fresh(leader_state, max_age_sec=state_timeout_sec):
            virtual_ref = (
                float(leader_state['x']),
                float(leader_state['y']),
                float(leader_state['z']),
            )
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

        for _ in range(total_steps):
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
            if leader_state is None or not self._state_fresh(leader_state, max_age_sec=state_timeout_sec):
                self._publish_zero_vel_all()
                now_t = time.time()
                if now_t - self._last_failsafe_log_t >= 1.0:
                    self._last_failsafe_log_t = now_t
                    self.get_logger().warn(
                        f'{stage_name}: leader state timeout, publish zero velocity failsafe'
                    )
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(sleep_dt)
                continue

            base_x = float(leader_state['x'])
            base_y = float(leader_state['y'])
            base_z = float(leader_state['z'])
            leader_heading = float(leader_state.get('heading', 0.0))
            if use_virtual_leader:
                virtual_ref, virtual_vel = self._advance_virtual_point(
                    virtual_ref,
                    leader_target,
                    max_leader_speed,
                    sleep_dt,
                )
            else:
                virtual_ref = (
                    float(leader_target[0]),
                    float(leader_target[1]),
                    float(leader_target[2]),
                )
                virtual_vel = (0.0, 0.0, 0.0)

            lvx = float(virtual_vel[0]) + leader_kp * (float(virtual_ref[0]) - base_x)
            lvy = float(virtual_vel[1]) + leader_kp * (float(virtual_ref[1]) - base_y)
            lvz = float(virtual_vel[2]) + leader_kp * (float(virtual_ref[2]) - base_z)
            lvx, lvy, lvz = self._limit_vector(lvx, lvy, lvz, max_leader_speed)
            self._publish_vel(self.leader_id, lvx, lvy, lvz)
            leader_vx = lvx
            leader_vy = lvy
            leader_vz = lvz

            err_values = []
            for drone_id in range(self.num_drones):
                if drone_id == self.leader_id:
                    continue
                dx, dy, dz = follower_offsets[drone_id]
                if use_heading_offsets:
                    rdx, rdy = self._rotate_offset_by_heading(dx, dy, leader_heading)
                else:
                    rdx, rdy = dx, dy
                ref_x = base_x + rdx
                ref_y = base_y + rdy
                ref_z = base_z + dz

                st = self.latest_states.get(drone_id)
                if st is None or not self._state_fresh(st, max_age_sec=state_timeout_sec):
                    self._publish_vel(drone_id, 0.0, 0.0, 0.0)
                    continue

                ex = ref_x - float(st['x'])
                ey = ref_y - float(st['y'])
                ez = ref_z - float(st['z'])
                vx = leader_vx + formation_kp * ex
                vy = leader_vy + formation_kp * ey
                vz = leader_vz + formation_kp * ez
                vx, vy, vz = self._limit_vector(vx, vy, vz, max_follower_speed)
                self._publish_vel(drone_id, vx, vy, vz)
                err_values.append(math.sqrt(ex * ex + ey * ey + ez * ez))

            dist_logs = []
            for drone_id in range(self.num_drones):
                st = self.latest_states.get(drone_id)
                if st is None:
                    continue
                if drone_id == self.leader_id:
                    tx, ty, tz = leader_target
                else:
                    dx, dy, dz = follower_offsets[drone_id]
                    if use_heading_offsets:
                        rdx, rdy = self._rotate_offset_by_heading(dx, dy, leader_heading)
                    else:
                        rdx, rdy = dx, dy
                    tx = base_x + rdx
                    ty = base_y + rdy
                    tz = base_z + dz
                dist = ((st['x'] - tx) ** 2 + (st['y'] - ty) ** 2 + (st['z'] - tz) ** 2) ** 0.5
                dist_logs.append(f"x500_{drone_id}:{dist:.2f}")
            min_pair_dist = None
            for i in range(self.num_drones):
                si = self.latest_states.get(i)
                if si is None or not self._state_fresh(si, max_age_sec=state_timeout_sec):
                    continue
                for j in range(i + 1, self.num_drones):
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
                        f"min_pair_dist={min_dist_text}, leader_v=({leader_vx:.2f},{leader_vy:.2f},{leader_vz:.2f})"
                    )
                    self._write_metric_row(
                        stage_name,
                        leader_state,
                        virtual_ref,
                        leader_target,
                        rms,
                        max(err_values),
                        min_pair_dist,
                        (leader_vx, leader_vy, leader_vz),
                    )

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(sleep_dt)

        if rms_count > 0:
            min_pair_text = 'nan' if stage_min_pair_dist is None else f'{stage_min_pair_dist:.2f}'
            self.get_logger().info(
                f'{stage_name}: stage_summary formation_rms_mean={rms_sum / rms_count:.2f}, '
                f'formation_max={stage_max_error:.2f}, min_pair_dist_min={min_pair_text}'
            )

    def hold_fixed_point(self, drone_id, x, y, z, duration=10):
        """单架无人机悬停在固定点"""
        if drone_id >= self.num_drones:
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
    leader_id = int(sys.argv[3]) if len(sys.argv) > 3 else 0
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

    rclpy.init()
    node = FixedPointMission(
        num_drones=num_drones,
        leader_id=leader_id,
        metrics_csv_path=metrics_csv_path,
    )

    # 两种流程：无障碍主从跟随 / 墙前绕行
    takeoff_stage = {}
    pre_wall_stage = {}
    side_align_stage = {}
    wall_cross_stage = {}
    target_stage = {}
    follower_offsets = {}
    # 机间避障safe_radius默认约1.2m，跟随队形间距需更大，否则会互相顶开导致难以到达目标
    wall_half = max(0.5, wall_length * 0.5)
    # 固定侧绕时，必须保证“靠墙那一侧最外侧飞机”也完整越过墙边。
    # 上侧绕行时补下方编队半宽；下侧绕行时补上方编队半宽。
    route_side = 'upper'
    lower_side_span = max(0.0, float(leader_id) * y_spacing)
    upper_side_span = max(0.0, float(num_drones - 1 - leader_id) * y_spacing)
    bypass_margin = 1.8
    if route_side == 'upper':
        bypass_y_base = wall_y + wall_half + bypass_margin + lower_side_span
    else:
        bypass_y_base = wall_y - wall_half - bypass_margin - upper_side_span
    pre_wall_x = wall_x - 3.5
    wall_cross_x = wall_x + 2.0
    for i in range(num_drones):
        formation_offset = (i - leader_id) * y_spacing
        y_offset = wall_y + formation_offset
        bypass_y = bypass_y_base + formation_offset
        if mission_mode == 'follow_target':
            takeoff_stage[i] = (0.0, y_offset, takeoff_z)
            target_stage[i] = (target_x, y_offset, takeoff_z)
            follower_offsets[i] = (0.0, formation_offset, 0.0)
        else:
            takeoff_stage[i] = (0.0, y_offset, takeoff_z)
            pre_wall_stage[i] = (pre_wall_x, y_offset, mission_z)
            side_align_stage[i] = (pre_wall_x, bypass_y, mission_z)
            wall_cross_stage[i] = (wall_cross_x, bypass_y, mission_z)
            target_stage[i] = (target_x, bypass_y, mission_z)
            follower_offsets[i] = (0.0, y_offset - wall_y, 0.0)

    node.get_logger().info(
        f"主从设定: leader=x500_{leader_id}, followers={[i for i in range(num_drones) if i != leader_id]}"
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
        for i in range(num_drones):
            tx, ty, tz = target_stage[i]
            node.get_logger().info(f"目标点: x500_{i} -> ({tx:.2f},{ty:.2f},{tz:.2f})")
    else:
        node.get_logger().info(
            f"绕墙规划: wall_x={wall_x:.2f}, wall_y={wall_y:.2f}, wall_length={wall_length:.2f}, "
            f"pre_wall_x={pre_wall_x:.2f}, bypass_y_base={bypass_y_base:.2f}, wall_cross_x={wall_cross_x:.2f}"
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
    target_stage, _ = node.compare_and_align_targets(target_stage, pos_tol=0.25)
    node.get_logger().info('mission-target-table: begin')
    for i in range(num_drones):
        tx, ty, tz = target_stage[i]
        node.get_logger().info(f"最终生效目标(NED) x500_{i}: ({tx:.2f},{ty:.2f},{tz:.2f})")
    node.get_logger().info('mission-target-table: end')

    node.publish_stage('stage1_takeoff_hold', takeoff_stage, duration=10.0)
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
        stage2_duration = node._estimate_stage_duration(
            takeoff_stage[leader_id],
            pre_wall_stage[leader_id],
            min_duration=8.0,
            max_speed=max_leader_speed,
        )
        stage3_duration = node._estimate_stage_duration(
            pre_wall_stage[leader_id],
            side_align_stage[leader_id],
            min_duration=8.0,
            max_speed=max_leader_speed,
        )
        stage4_duration = node._estimate_stage_duration(
            side_align_stage[leader_id],
            wall_cross_stage[leader_id],
            min_duration=10.0,
            max_speed=max_leader_speed,
        )
        stage5_duration = node._estimate_stage_duration(
            wall_cross_stage[leader_id],
            target_stage[leader_id],
            min_duration=float(duration),
            max_speed=max_leader_speed,
        )
        node.get_logger().info(
            f"闭环阶段时长: pre_wall={stage2_duration:.1f}s, side_align={stage3_duration:.1f}s, "
            f"wall_cross={stage4_duration:.1f}s, go_target={stage5_duration:.1f}s"
        )
        node.publish_follow_leader_stage(
            'stage2_pre_wall_lf',
            pre_wall_stage[leader_id],
            follower_offsets,
            duration=stage2_duration,
            formation_kp=formation_kp,
            leader_kp=leader_kp,
            max_follower_speed=max_follower_speed,
            max_leader_speed=max_leader_speed,
            use_heading_offsets=use_heading_offsets,
            allow_marker_update=False,
            use_virtual_leader=use_virtual_leader,
            state_timeout_sec=state_timeout_sec,
        )
        node.publish_follow_leader_stage(
            'stage3_side_align_lf',
            side_align_stage[leader_id],
            follower_offsets,
            duration=stage3_duration,
            formation_kp=formation_kp,
            leader_kp=leader_kp,
            max_follower_speed=max_follower_speed,
            max_leader_speed=max_leader_speed,
            use_heading_offsets=use_heading_offsets,
            allow_marker_update=False,
            use_virtual_leader=use_virtual_leader,
            state_timeout_sec=state_timeout_sec,
        )
        node.publish_follow_leader_stage(
            'stage4_wall_cross_lf',
            wall_cross_stage[leader_id],
            follower_offsets,
            duration=stage4_duration,
            formation_kp=formation_kp,
            leader_kp=leader_kp,
            max_follower_speed=max_follower_speed,
            max_leader_speed=max_leader_speed,
            use_heading_offsets=use_heading_offsets,
            allow_marker_update=False,
            use_virtual_leader=use_virtual_leader,
            state_timeout_sec=state_timeout_sec,
        )
        node.publish_follow_leader_stage(
            'stage5_go_target_lf',
            target_stage[leader_id],
            follower_offsets,
            duration=stage5_duration,
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
