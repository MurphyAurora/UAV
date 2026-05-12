#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XTDrone2 Multirotor Communication Node

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn

Copyright (c) 2025 XTDrone2

This file is part of XTDrone2. XTDrone2 is free software: you can redistribute it
and/or modify it under the terms of the MIT License.

You should have received a copy of the MIT License along with XTDrone2. If not,
see <https://opensource.org/licenses/MIT>.
"""

import rclpy
import time
from rclpy.node import Node
#from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleAttitudeSetpoint
from xtd2_msgs.srv import XTD2Cmd
from xtd2_msgs.msg import XTD2VehicleState
from px4_msgs.msg import VehicleStatus
import sys
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import argparse
from px4_msgs.msg import VehicleCommandAck

class MultirotorCommunication(Node):
    def __init__(self, model, id, namespace="", debug=False, px4_ns="",use_root_fmu=False,target_sys_id=0, enable_avoid=False, avoid_timeout=0.25, avoid_recovery_sec=1.0, avoid_enable_z=-1.8, avoid_min_speed=0.03, avoid_z_track_tol=0.35):
        
        if model.startswith("gz_"):  # 删除gz_前缀
            model = model[3:]
        self.model = model

        self.id = int(id)
        self.debug = debug

        self.namespace = namespace if namespace else f'{model}_{id}'
        self.px4_ns = px4_ns if px4_ns else f'px4_{self.id + 1}'

        self.use_root_fmu = use_root_fmu
        prefix = '/fmu' if self.use_root_fmu else f'/{self.px4_ns}/fmu'

        super().__init__(f'{self.namespace}_communication')

        self.OFFBOARD_STATE = "DISABLED"
        self.cmd = None
        self.cur_vehicle_local_position = None
        self.cur_vehicle_global_position = None
        self.init_vehicle_local_position = None
        self.init_vehicle_global_position = None
        self.target_sys_id = int(target_sys_id) if int(target_sys_id) > 0 else (self.id + 1)
        self.enable_avoid = bool(enable_avoid)
        self.avoid_timeout = float(avoid_timeout)
        self.avoid_cmd = None
        self.avoid_cmd_time = 0.0
        self.avoid_min_speed = float(avoid_min_speed)
        self.avoid_enable_z = float(avoid_enable_z)
        self.avoid_z_track_tol = float(avoid_z_track_tol)
        self.avoid_recovery_sec = float(avoid_recovery_sec)
        self._avoid_recovery_until = 0.0
        self._last_avoid_cmd_for_recovery = None
        self._last_cmd_src = "NONE"
        self._last_arb_log_time = 0.0
        self._last_avoid_rx_log_time = 0.0
        self.task_track_speed = 0.30
        self.avoid_blend_max_speed = 0.35

        self.cur_vehicle_status = None
        self.create_subscription(VehicleStatus,f'{prefix}/out/vehicle_status_v1',self.vehicle_status_callback,qos_profile_sensor_data)
        self.offboard_warmup_count = 0
        self.offboard_ready = False
        self._last_offboard_reassert_t = 0.0
        self.offboard_reassert_period = 2.0

        # 用于限频日志，避免20Hz刷屏
        self._last_log_sec = 0
        
        # XTDrone2 Interface
        self.create_subscription(Pose, f'/xtdrone2/{self.namespace}/cmd_pose_local_ned', self.cmd_pose_local_ned_callback, 10)  # geometry_msgs/Pose
        self.create_subscription(Pose, f'/xtdrone2/{self.namespace}/cmd_pose_local_flu', self.cmd_pose_local_flu_callback, 10)  # geometry_msgs/Pose
        self.create_subscription(Twist, f'/xtdrone2/{self.namespace}/cmd_vel_ned', self.cmd_vel_ned_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, f'/xtdrone2/{self.namespace}/cmd_vel_flu', self.cmd_vel_flu_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, f'/xtdrone2/{self.namespace}/cmd_accel_ned', self.cmd_accel_ned_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, f'/xtdrone2/{self.namespace}/cmd_accel_flu', self.cmd_accel_flu_callback, 10)  # geometry_msgs/Twist
        self.create_subscription(Twist, f'/xtdrone2/{self.namespace}/avoid_cmd_vel_ned', self.avoid_cmd_vel_ned_callback, 10)  # geometry_msgs/Twist
        #暂时禁用
        #self.create_subscription(Twist, f'/xtdrone2/{self.namespace}/cmd_attitude_flu', self.cmd_attitude_flu_callback, 10)  # geometry_msgs/Pose
        
        self.cmd_server = self.create_service(XTD2Cmd, f'/xtdrone2/{self.namespace}/cmd', self.cmd_callback)

        # DDS Interface（统一）
        self.create_subscription(VehicleLocalPosition,f'{prefix}/out/vehicle_local_position_v1',self.vehicle_local_position_callback, qos_profile_sensor_data)
        self.create_subscription(VehicleGlobalPosition,f'{prefix}/out/vehicle_global_position_v1',self.vehicle_global_position_callback, qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck,f'{prefix}/out/vehicle_command_ack',self.vehicle_command_ack_callback, qos_profile_sensor_data)

        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, f'{prefix}/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f'{prefix}/in/offboard_control_mode', 10)
        self.dds_trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', 10)
        self.dds_vehicle_attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint, f'{prefix}/in/vehicle_attitude_setpoint', 10)

        self.timer_ = self.create_timer(0.05, self.timer_callback)
        
        # Debug publisher for vehicle state
        if self.debug:
            self.vehicle_state_publisher = self.create_publisher(XTD2VehicleState, f'/xtdrone2/{self.namespace}/debug/vehicle_state', 10)
            self.debug_timer = self.create_timer(0.1, self.publish_vehicle_state)  # 10Hz
        self.get_logger().info(
            f"START namespace={self.namespace}, id={self.id}, px4_ns={self.px4_ns}, "
            f"use_root_fmu={self.use_root_fmu}, prefix={prefix}, enable_avoid={self.enable_avoid}"
        )
        self.get_logger().info(
            f"[ARB] enable_avoid={self.enable_avoid}, avoid_timeout={self.avoid_timeout:.2f}s, "
            f"avoid_enable_z={self.avoid_enable_z:.2f}, avoid_min_speed={self.avoid_min_speed:.2f}, "
            f"avoid_z_track_tol={self.avoid_z_track_tol:.2f}"
        )
        self.get_logger().info(f'{self.namespace} -> {prefix} communication node started')
    def vehicle_status_callback(self, msg):
        self.cur_vehicle_status = msg
    def is_armed(self):
        # PX4: 1=DISARMED, 2=ARMED
        return self.cur_vehicle_status is not None and self.cur_vehicle_status.arming_state == 2

    def is_offboard(self):
        # 常见PX4里 OFFBOARD nav_state=14（不同版本可打印确认）
        return self.cur_vehicle_status is not None and self.cur_vehicle_status.nav_state == 14
        # FIX: 限频日志，避免20Hz刷屏
    def _throttled_info(self, text, period_sec=1):
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        if now_sec - self._last_log_sec >= period_sec:
            self._last_log_sec = now_sec
            self.get_logger().info(text)

    def timer_callback(self):
        # local position 未就绪，不发
        if self.cur_vehicle_local_position is None:
            return

        # 保底：如果还没有cmd，自动生成当前悬停点
        if self.cmd is None:
            hover_cmd = TrajectorySetpoint()
            hover_cmd.timestamp = self.get_clock_microseconds()
            hover_cmd.position[0] = self.cur_vehicle_local_position.x
            hover_cmd.position[1] = self.cur_vehicle_local_position.y
            hover_cmd.position[2] = self.cur_vehicle_local_position.z
            hover_cmd.yaw = self.cur_vehicle_local_position.heading
            self.cmd = hover_cmd
            self._log_cmd_src("TIMER_HOVER_INIT", hover_cmd)

        # 混合模式下，仅在OFFBOARD时应用最近避障速度修正
        arb_src = "TASK"
        active_cmd = self.cmd
        active_state = self.OFFBOARD_STATE
        now_t = time.time()
        avoid_takeover = False
        if self.enable_avoid and self.is_offboard() and self.avoid_cmd is not None:
            if time.time() - self.avoid_cmd_time <= self.avoid_timeout:
                avoid_speed = math.sqrt(
                    self.avoid_cmd.linear.x ** 2
                    + self.avoid_cmd.linear.y ** 2
                    + self.avoid_cmd.linear.z ** 2
                )
                # 仅当避障输出具有足够幅值且高度已基本建立时才接管，避免打断爬升造成第三架高度不足
                can_avoid_takeover = self.cur_vehicle_local_position.z <= self.avoid_enable_z
                if can_avoid_takeover and active_cmd is not None:
                    target_z = float(active_cmd.position[2])
                    if not math.isnan(target_z):
                        z_err = abs(float(self.cur_vehicle_local_position.z) - target_z)
                        can_avoid_takeover = z_err <= self.avoid_z_track_tol
                if avoid_speed >= self.avoid_min_speed and can_avoid_takeover:
                    task_vx, task_vy, task_vz, task_yawspeed = self._extract_task_velocity_hint(active_cmd, active_state)
                    active_cmd = self._build_blended_avoid_cmd(
                        self.avoid_cmd,
                        task_vx,
                        task_vy,
                        task_vz,
                        task_yawspeed,
                    )
                    active_state = "VEL_NED"
                    arb_src = "AVOID"
                    avoid_takeover = True
                    self._last_avoid_cmd_for_recovery = active_cmd
                    self._avoid_recovery_until = now_t + self.avoid_recovery_sec

        # 阶段2恢复态：避障结束后做短时平滑回归，避免从AVOID硬切到TASK
        if (
            self.enable_avoid
            and (not avoid_takeover)
            and self._last_avoid_cmd_for_recovery is not None
            and now_t < self._avoid_recovery_until
            and self.is_offboard()
        ):
            alpha = max(0.0, (self._avoid_recovery_until - now_t) / max(self.avoid_recovery_sec, 1e-3))
            active_cmd = self._build_recovery_cmd(active_cmd, active_state, alpha)
            active_state = "VEL_NED"
            arb_src = "RECOVERY"

        if self.enable_avoid and active_state in ["VEL_NED", "VEL_FLU", "POSE_LOCAL_NED", "POSE_LOCAL_FLU"]:
            self._log_arb_src(arb_src, active_cmd)

        # ---- 双发保底：每个周期都发 OffboardControlMode + TrajectorySetpoint ----
        mode = OffboardControlMode()
        mode.timestamp = self.get_clock_microseconds()

        # 根据当前控制状态选择控制位；默认 position
        if active_state in ["VEL_NED", "VEL_FLU"]:
            mode.position = False
            mode.velocity = True
            mode.acceleration = False
            mode.attitude = False
            mode.body_rate = False
        elif active_state in ["ACCEL_NED", "ACCEL_FLU"]:
            mode.position = False
            mode.velocity = False
            mode.acceleration = True
            mode.attitude = False
            mode.body_rate = False
        elif active_state == "ATTITUDE_FLU":
            mode.position = False
            mode.velocity = False
            mode.acceleration = False
            mode.attitude = True
            mode.body_rate = False
        else:
            # DISABLED / ENABLED / POSE_* 都走 position 保底
            mode.position = True
            mode.velocity = False
            mode.acceleration = False
            mode.attitude = False
            mode.body_rate = False

        self.offboard_control_mode_pub.publish(mode)

        # setpoint 也每周期发
        active_cmd.timestamp = self.get_clock_microseconds()
        if active_state == "ATTITUDE_FLU":
            self.dds_vehicle_attitude_setpoint_pub.publish(active_cmd)
        else:
            self.dds_trajectory_setpoint_pub.publish(active_cmd)

        # OFFBOARD保活：若已进入外部控制链路但模式意外掉出，周期性重发切模态命令
        now_t = time.time()
        if (
            self.OFFBOARD_STATE != "DISABLED"
            and self.offboard_ready
            and self.is_armed()
            and (not self.is_offboard())
            and (now_t - self._last_offboard_reassert_t) >= self.offboard_reassert_period
        ):
            self._last_offboard_reassert_t = now_t
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
            )
            self.get_logger().warn("OFFBOARD dropped, reassert mode command sent")

        # 预热计数
        self.offboard_warmup_count += 1
        if self.offboard_warmup_count >= 20:
            self.offboard_ready = True
    
    def vehicle_local_position_callback(self, msg):
        if self.init_vehicle_local_position is None:
            self.init_vehicle_local_position = msg
        self.cur_vehicle_local_position = msg
        self._throttled_info(
            f"local_position OK x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, hdg={msg.heading:.2f}",
            period_sec=1
        )

    def vehicle_global_position_callback(self, msg):
        if self.init_vehicle_global_position is None:
            self.init_vehicle_global_position = msg
        self.cur_vehicle_global_position = msg

    def vehicle_command_ack_callback(self, msg):
        self.get_logger().info(
            f"ACK: cmd={msg.command}, result={msg.result}, "
            f"target_system={msg.target_system}, from_external={msg.from_external}"
        )

    def get_clock_microseconds(self):
        t_ = self.get_clock().now().seconds_nanoseconds()
        return int(t_[0]*1e6 + t_[1]/1000)
    
    # 1) 在类里加一个小工具函数
    def _log_cmd_src(self, src, cmd=None):
        if cmd is None:
            self.get_logger().info(f"[CMD_SRC] {src}")
            return
        self.get_logger().info(
            f"[CMD_SRC] {src} pos=({cmd.position[0]:.3f},{cmd.position[1]:.3f},{cmd.position[2]:.3f}) "
            f"vel=({cmd.velocity[0]:.3f},{cmd.velocity[1]:.3f},{cmd.velocity[2]:.3f}) "
            f"acc=({cmd.acceleration[0]:.3f},{cmd.acceleration[1]:.3f},{cmd.acceleration[2]:.3f}) "
            f"yaw={cmd.yaw:.3f}"
        )
    def cmd_pose_local_ned_callback(self, msg):
        self.OFFBOARD_STATE = "POSE_LOCAL_NED"
        # Convert quaternion to euler angles
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = msg.position.x
        cmd.position[1] = msg.position.y
        cmd.position[2] = msg.position.z
        cmd.yaw = yaw
        self.cmd = cmd
        self._log_cmd_src("POSE_NED", cmd)
        self.get_logger().info(f"POSE_NED in: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}")
        self.get_logger().info(f"SET cmd.position=({cmd.position[0]}, {cmd.position[1]}, {cmd.position[2]})")
    def cmd_pose_local_flu_callback(self, msg):
        # FIX: 防止local position尚未到达时崩溃
        if self.cur_vehicle_local_position is None or self.init_vehicle_local_position is None:
            self.get_logger().warn("cmd_pose_local_flu ignored: local/init local position not ready")
            return
        
        self.OFFBOARD_STATE = "POSE_LOCAL_FLU"
        # Convert quaternion to euler angles
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        theta = self.cur_vehicle_local_position.heading

        # Transfer position from FLU to NED, msg.position.xyz is flu, respectively
        p_n = msg.position.x * math.cos(theta) + msg.position.y * math.sin(theta)
        p_e = msg.position.x * math.sin(theta) - msg.position.y * math.cos(theta)
        p_d = -msg.position.z
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = p_n
        cmd.position[1] = p_e
        cmd.position[2] = p_d
        cmd.yaw = self.init_vehicle_local_position.heading + yaw
        self.cmd = cmd
        self._log_cmd_src("POSE_FLU", cmd)

    def cmd_vel_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        if not self.is_offboard():
            self.get_logger().warn("VEL_NED ignored: not in OFFBOARD")
            return
        self.OFFBOARD_STATE = "VEL_NED"
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan
        cmd.velocity[0] = msg.linear.x
        cmd.velocity[1] = msg.linear.y
        cmd.velocity[2] = msg.linear.z
        cmd.yaw = math.nan
        cmd.yawspeed = msg.angular.z
        self.cmd = cmd

    def avoid_cmd_vel_ned_callback(self, msg):
        self.avoid_cmd = msg
        self.avoid_cmd_time = time.time()
        if self.enable_avoid and (self.avoid_cmd_time - self._last_avoid_rx_log_time) >= 1.0:
            self._last_avoid_rx_log_time = self.avoid_cmd_time
            self.get_logger().info(
                f"[ARB] avoid_rx v=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f})"
            )

    def _build_vel_ned_cmd(self, msg):
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan
        cmd.velocity[0] = msg.linear.x
        cmd.velocity[1] = msg.linear.y
        cmd.velocity[2] = msg.linear.z
        cmd.yaw = math.nan
        cmd.yawspeed = msg.angular.z
        return cmd

    def _extract_task_velocity_hint(self, task_cmd, task_state):
        task_vx = 0.0
        task_vy = 0.0
        task_vz = 0.0
        task_yawspeed = 0.0
        if task_cmd is None:
            return task_vx, task_vy, task_vz, task_yawspeed

        if task_state in ["VEL_NED", "VEL_FLU"]:
            task_vx = float(task_cmd.velocity[0])
            task_vy = float(task_cmd.velocity[1])
            task_vz = float(task_cmd.velocity[2])
            task_yawspeed = float(task_cmd.yawspeed)
            return task_vx, task_vy, task_vz, task_yawspeed

        # For pose-tracking tasks, synthesize a bounded velocity hint toward the target
        # so avoid mode still keeps some progress toward mission waypoints.
        if (
            self.cur_vehicle_local_position is not None
            and task_state in ["POSE_LOCAL_NED", "POSE_LOCAL_FLU", "ENABLED", "DISABLED"]
        ):
            target_x = float(task_cmd.position[0])
            target_y = float(task_cmd.position[1])
            target_z = float(task_cmd.position[2])
            if not math.isnan(target_x) and not math.isnan(target_y):
                dx = target_x - float(self.cur_vehicle_local_position.x)
                dy = target_y - float(self.cur_vehicle_local_position.y)
                dist_xy = math.hypot(dx, dy)
                if dist_xy > 1e-6:
                    scale = min(self.task_track_speed, dist_xy) / dist_xy
                    task_vx = dx * scale
                    task_vy = dy * scale
            if not math.isnan(target_z):
                dz = target_z - float(self.cur_vehicle_local_position.z)
                task_vz = max(-0.2, min(0.2, dz))
        return task_vx, task_vy, task_vz, task_yawspeed

    def _build_blended_avoid_cmd(self, avoid_msg, task_vx, task_vy, task_vz, task_yawspeed):
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan
        vx = float(avoid_msg.linear.x) + float(task_vx)
        vy = float(avoid_msg.linear.y) + float(task_vy)
        vz = float(task_vz)
        spd = math.sqrt(vx * vx + vy * vy + vz * vz)
        if spd > self.avoid_blend_max_speed and spd > 1e-6:
            scale = self.avoid_blend_max_speed / spd
            vx *= scale
            vy *= scale
            vz *= scale
        cmd.velocity[0] = vx
        cmd.velocity[1] = vy
        cmd.velocity[2] = vz
        cmd.yaw = math.nan
        cmd.yawspeed = task_yawspeed
        return cmd

    def _build_recovery_cmd(self, task_cmd, task_state, alpha):
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan

        task_vx, task_vy, task_vz, task_yawspeed = self._extract_task_velocity_hint(task_cmd, task_state)

        avoid_cmd = self._last_avoid_cmd_for_recovery
        cmd.velocity[0] = alpha * float(avoid_cmd.velocity[0]) + (1.0 - alpha) * task_vx
        cmd.velocity[1] = alpha * float(avoid_cmd.velocity[1]) + (1.0 - alpha) * task_vy
        cmd.velocity[2] = alpha * float(avoid_cmd.velocity[2]) + (1.0 - alpha) * task_vz
        cmd.yaw = math.nan
        cmd.yawspeed = task_yawspeed
        return cmd

    def _log_arb_src(self, src, cmd):
        now = time.time()
        # 源切换立即打印；同源限频打印，避免20Hz刷屏
        if src != self._last_cmd_src or (now - self._last_arb_log_time) >= 1.0:
            self._last_cmd_src = src
            self._last_arb_log_time = now
            self.get_logger().info(
                f"[ARB] src={src} out_v=({cmd.velocity[0]:.2f},{cmd.velocity[1]:.2f},{cmd.velocity[2]:.2f}) "
                f"offboard={self.is_offboard()}"
            )

    def cmd_vel_flu_callback(self, msg):
        """ Let a be heading angle
        [[cos a , sin a,  0],     [f]   [n]
         [-sin a, cos a,  0],  *  [l] = [e]
         [0     , 0    , -1]]     [u]   [d]
        """
        if self.OFFBOARD_STATE == "DISABLED":
            return
        if not self.is_offboard():
            self.get_logger().warn("VEL_FLU ignored: not in OFFBOARD")
            return
        #FIX
        if self.cur_vehicle_local_position is None:
            self.get_logger().warn("cmd_vel_flu ignored: local position not ready")
            return
        
        self.OFFBOARD_STATE = "VEL_FLU"
        theta = self.cur_vehicle_local_position.heading 
        # Transform velocity from FLU to NED, msg.linear.xyz is flu, respectively
        v_n = msg.linear.x * math.cos(theta) + msg.linear.y * math.sin(theta)
        v_e = msg.linear.x * math.sin(theta) - msg.linear.y * math.cos(theta)
        v_d = -msg.linear.z
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan
        cmd.velocity[0] = v_n
        cmd.velocity[1] = v_e
        cmd.velocity[2] = v_d
        cmd.yaw = math.nan
        cmd.yawspeed = -msg.angular.z
        self.cmd = cmd
        
    def cmd_accel_ned_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return

        self.OFFBOARD_STATE = "ACCEL_NED"
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan
        cmd.velocity[0] = math.nan
        cmd.velocity[1] = math.nan
        cmd.velocity[2] = math.nan
        cmd.acceleration[0] = msg.linear.x
        cmd.acceleration[1] = msg.linear.y
        cmd.acceleration[2] = msg.linear.z
        #TODO: How about yaw?
        self.cmd = cmd
        
    def cmd_accel_flu_callback(self, msg):
        if self.OFFBOARD_STATE == "DISABLED":
            return
        
        # FIX
        if self.cur_vehicle_local_position is None:
            self.get_logger().warn("cmd_accel_flu ignored: local position not ready")
            return
        
        self.OFFBOARD_STATE = "ACCEL_FLU"
        theta = self.cur_vehicle_local_position.heading
        # Transform acceleration from FLU to NED, msg.linear.xyz is flu, respectively
        a_n = msg.linear.x * math.cos(theta) + msg.linear.y * math.sin(theta)
        a_e = msg.linear.x * math.sin(theta) - msg.linear.y * math.cos(theta)
        a_d = -msg.linear.z
        # Construct TrajectorySetpoint message
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = math.nan
        cmd.position[1] = math.nan
        cmd.position[2] = math.nan
        cmd.velocity[0] = math.nan
        cmd.velocity[1] = math.nan
        cmd.velocity[2] = math.nan
        cmd.acceleration[0] = a_n
        cmd.acceleration[1] = a_e
        cmd.acceleration[2] = a_d
        # How about yaw
        self.cmd = cmd
    
    # def cmd_attitude_flu_callback(self, msg):
        # #if self.OFFBOARD_STATE == "DISABLED":
        #     return
        
        # self.OFFBOARD_STATE = "ATTITUDE_FLU"
        # cmd = VehicleAttitudeSetpoint()
        # cmd.timestamp = self.get_clock_microseconds()
        # cmd.q_d = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # cmd.thrust = msg.linear.x
        # self.cmd = cmd
    # 暂时保留但不订阅，避免误触发
    def cmd_attitude_flu_callback(self, msg):
        self.get_logger().warn("ATTITUDE_FLU is temporarily disabled due to message-type mismatch.")
        return

    def cmd_callback(self, request, response):
        command = request.command
        if command == "ARM":
            self.arm()
            response.success = True
        elif command == "DISARM":
            self.disarm()
            response.success = True
        elif command == "HOVER":
            self.hover()
            response.success = True
        elif command == "OFFBOARD":
            # A) 必须有local position
            if self.cur_vehicle_local_position is None:
                self.get_logger().warn("OFFBOARD rejected: local position not ready")
                response.success = False
                return response
               

            # B) 准备悬停setpoint
            #self.OFFBOARD_STATE = "POSE_LOCAL_NED"
            # if self.cmd is None:
            #     self.hover()
            # if self.cmd is None:
            #     self.get_logger().warn("OFFBOARD rejected: hover cmd is None")
            #     response.success = False
            #     return response
            if not self.is_armed():
                self.get_logger().warn("OFFBOARD rejected: vehicle not armed")
                response.success = False
                return response

            # C) 预热计数门槛（依赖 timer_callback 持续发两路流）
            #    例如: self.offboard_warmup_count 在 timer_callback 每次+1
            if self.offboard_warmup_count < 20:
                self.get_logger().warn(
                    f"OFFBOARD rejected: warmup not enough ({self.offboard_warmup_count}/20)"
                )
                response.success = False
                return response

            # 4) 仅发送切模式命令（不在这里sleep循环，不重复ARM）
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
            )

            # 关键修复：服务成功仅代表“真实进入OFFBOARD”，否则返回失败触发上层重试。
            if self.is_offboard():
                self.get_logger().info("OFFBOARD mode reached")
                response.success = True
            else:
                self.get_logger().warn("OFFBOARD mode not reached yet, retry required")
                response.success = False
            return response

        elif command == "TAKEOFF":
            response.success =  self.takeoff()
        elif command == "LAND":
            response.success = self.land()
        elif command == "RTL":
            self.rtl()
            response.success = True
        else:
            self.get_logger().warn(f'Unknown command: {command}')       
            response.success = False 
        return response

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock_microseconds()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        sys_id = int(self.target_sys_id)
        msg.target_system = sys_id
        msg.target_component = 1
        msg.source_system = 250
        msg.source_component = 1
        msg.from_external = True

        topic_cmd = '/fmu/in/vehicle_command' if self.use_root_fmu else f'/{self.px4_ns}/fmu/in/vehicle_command'
        self.get_logger().info(
            f"SEND cmd={msg.command} target_system={msg.target_system} "
            f"source_system={msg.source_system} topic={topic_cmd}"
        )

        self.vehicle_command_publisher.publish(msg)
    def _publish_offboard_pose_stream_once(self):
        if self.cmd is None:
            if self.cur_vehicle_local_position is None:
                return False
            cmd = TrajectorySetpoint()
            cmd.timestamp = self.get_clock_microseconds()
            cmd.position[0] = self.cur_vehicle_local_position.x
            cmd.position[1] = self.cur_vehicle_local_position.y
            cmd.position[2] = self.cur_vehicle_local_position.z
            cmd.yaw = self.cur_vehicle_local_position.heading
            self.cmd = cmd

        # 1) control mode
        mode = OffboardControlMode()
        mode.timestamp = self.get_clock_microseconds()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.offboard_control_mode_pub.publish(mode)

        # 2) trajectory setpoint
        self.cmd.timestamp = self.get_clock_microseconds()
        self.dds_trajectory_setpoint_pub.publish(self.cmd)
        return True
    def arm(self):
        self.offboard_warmup_count = 0
        self.offboard_ready = False
        # TODO：Check if the vehicle is already armed
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command send')
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command send')

  
    def hover(self):
        if self.cur_vehicle_local_position is None:
            self.get_logger().warn('Hover ignored: no local position yet')
            return

        self.OFFBOARD_STATE = "POSE_LOCAL_NED"
        cmd = TrajectorySetpoint()
        cmd.timestamp = self.get_clock_microseconds()
        cmd.position[0] = self.cur_vehicle_local_position.x
        cmd.position[1] = self.cur_vehicle_local_position.y
        cmd.position[2] = self.cur_vehicle_local_position.z
        cmd.yaw = self.cur_vehicle_local_position.heading
        self.cmd = cmd
        self._log_cmd_src("HOVER", cmd)
        self.get_logger().info('Hover setpoint prepared')
    def takeoff(self):
        # | Minimum pitch (if airspeed sensor present), desired pitch without sensor
        # | Empty
        # | Empty
        # | Yaw angle (if magnetometer present), ignored without magnetometer
        # | Latitude
        # | Longitude
        # | Altitude
        #去掉这两个，避免字段混淆param5=self.cur_vehicle_local_position.ref_lat, param6=self.cur_vehicle_local_position.ref_lon, 
        # FIX
        if self.cur_vehicle_local_position is None:
            self.get_logger().warn("Takeoff rejected: local position not ready")
            return False
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param4=self.cur_vehicle_local_position.heading,param7=10.0)
        self.get_logger().info("Take off command send.")
        return True

    def land(self):
        # | Empty
        # | Empty
        # | Empty
        # | Desired yaw angle.
        # | Latitude
        # | Longitude
        # | Altitude|
        # FIX
        if self.cur_vehicle_local_position is None or self.cur_vehicle_global_position is None:
            self.get_logger().warn("Land rejected: local/global position not ready")
            return False
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param4=self.cur_vehicle_local_position.heading, param5=self.cur_vehicle_global_position.lat, param6=self.cur_vehicle_global_position.lon, param7=0.0)
        self.get_logger().info("Land command send.")
        return True

    def rtl(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("RTL command send.")

    def publish_vehicle_state(self):
        
        if not self.debug:
            return
            
        msg = XTD2VehicleState()
        msg.timestamp = self.get_clock_microseconds()
        msg.offboard_state = self.OFFBOARD_STATE

        # 当前位置信息
        if self.cur_vehicle_local_position is not None:
            msg.x = self.cur_vehicle_local_position.x
            msg.y = self.cur_vehicle_local_position.y
            msg.z = self.cur_vehicle_local_position.z
            msg.heading = self.cur_vehicle_local_position.heading
        else:
            msg.x = float('nan')
            msg.y = float('nan')
            msg.z = float('nan')
            msg.heading = float('nan')

        # 全局位置信息
        if self.cur_vehicle_global_position is not None:
            msg.lat = self.cur_vehicle_global_position.lat
            msg.lon = self.cur_vehicle_global_position.lon
            msg.alt = self.cur_vehicle_global_position.alt
        else:
            msg.lat = float('nan')
            msg.lon = float('nan')
            msg.alt = float('nan')

        # 初始位置信息
        if self.init_vehicle_local_position is not None:
            msg.init_x = self.init_vehicle_local_position.x
            msg.init_y = self.init_vehicle_local_position.y
            msg.init_z = self.init_vehicle_local_position.z
            msg.init_heading = self.init_vehicle_local_position.heading
        else:
            msg.init_x = float('nan')
            msg.init_y = float('nan')
            msg.init_z = float('nan')
            msg.init_heading = float('nan')

        # 初始全局位置信息
        if self.init_vehicle_global_position is not None:
            msg.init_lat = self.init_vehicle_global_position.lat
            msg.init_lon = self.init_vehicle_global_position.lon
            msg.init_alt = self.init_vehicle_global_position.alt
        else:
            msg.init_lat = float('nan')
            msg.init_lon = float('nan')
            msg.init_alt = float('nan')

        self.vehicle_state_publisher.publish(msg)

def main():
    rclpy.init(args=sys.argv)

    parser = argparse.ArgumentParser(description='XTDrone2 Multirotor Communication Node')
    
    parser.add_argument('--model', type=str, help='Vehicle type', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id, should be unique in same model', required=True)
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")
    parser.add_argument('--debug', action='store_true', help='Enable debug mode to publish vehicle state', required=False, default=False)
    parser.add_argument('--px4-ns', type=str, required=False, default="",
                    help='PX4 DDS namespace, e.g. px4_1 (default: px4_{id+1})')
    parser.add_argument('--use-root-fmu', action='store_true', default=False,
                    help='Use /fmu/* topics (single-vehicle compatibility). Default: False (use /{px4_ns}/fmu/*)')
    parser.add_argument('--target-sys-id', type=int, default=0,help='PX4 MAV_SYS_ID override; 0 means use id+1')
    parser.add_argument('--enable-avoid', action='store_true', default=False,
                    help='Enable avoid_cmd_vel_ned arbitration path (hybrid mode)')
    parser.add_argument('--avoid-timeout', type=float, default=0.25,
                    help='Avoid command timeout in seconds')
    parser.add_argument('--avoid-recovery-sec', type=float, default=1.0,
                    help='Recovery smoothing time from AVOID back to TASK')
    parser.add_argument('--avoid-enable-z', type=float, default=-1.8,
                    help='Enable avoid takeover only when local z <= this threshold')
    parser.add_argument('--avoid-min-speed', type=float, default=0.03,
                    help='Minimum avoid speed magnitude to trigger arbitration takeover')
    parser.add_argument('--avoid-z-track-tol', type=float, default=0.35,
                    help='Allow avoid takeover only when |z-current - z-target| <= this tolerance')
    args, unknown = parser.parse_known_args()

    multirotor_communication = MultirotorCommunication(
        args.model,
        args.id,
        args.namespace,
        args.debug,
        args.px4_ns,
        args.use_root_fmu,
        args.target_sys_id,
        args.enable_avoid,
        args.avoid_timeout,
        args.avoid_recovery_sec,
        args.avoid_enable_z,
        args.avoid_min_speed,
        args.avoid_z_track_tol,
    )
    rclpy.spin(multirotor_communication)
    multirotor_communication.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
