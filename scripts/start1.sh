#!/bin/bash
# set -e

# 防重复执行锁（彻底杜绝二次执行杀进程）
LOCK_FILE="/tmp/start_drone.lock"
if [ -f "$LOCK_FILE" ]; then
    echo "【错误】脚本已经在运行！！！"
    exit 1
fi
echo $$ > "$LOCK_FILE"
trap 'rm -f "$LOCK_FILE"' EXIT

# 强制创建日志目录，避免tee报错
LOG_DIR=~/ws_xtd2/logs
mkdir -p "$LOG_DIR" || { echo "[ERROR] 无法创建日志目录 $LOG_DIR"; exit 1; }

# 环境变量配置
# PX4 SITL instance "-i 1" usually has MAV_SYS_ID=2. Keep command target_system
# aligned with PX4's MAV_SYS_ID; this is independent from the visible x500_1 name.
PX4_SYS_ID_BASE=${PX4_SYS_ID_BASE:-2}
NUM_DRONES=${1:-5}
TARGET_SYS_IDS_STR="${TARGET_SYS_IDS_STR:-}"
if [ -z "${TARGET_SYS_IDS_STR}" ]; then
  TARGET_SYS_IDS_STR=""
  for ((id=1; id<=NUM_DRONES; id++)); do
    sys_id=$((PX4_SYS_ID_BASE + id - 1))
    if [ -z "${TARGET_SYS_IDS_STR}" ]; then
      TARGET_SYS_IDS_STR="${sys_id}"
    else
      TARGET_SYS_IDS_STR="${TARGET_SYS_IDS_STR},${sys_id}"
    fi
  done
fi
START_PORT=8888
GZ_WORLD=${GZ_WORLD:-default}
AUTO_ARM_OFFBOARD=${AUTO_ARM_OFFBOARD:-1}
WARMUP_SEC=${WARMUP_SEC:-25}
TAKEOFF_Z=${TAKEOFF_Z:--2.0}
MISSION_Z=${MISSION_Z:--3.0}
ARM_TO_OFFBOARD_DELAY=${ARM_TO_OFFBOARD_DELAY:-0.8}
INTER_DRONE_GAP=${INTER_DRONE_GAP:-0.2}
POST_OFFBOARD_HOLD_SEC=${POST_OFFBOARD_HOLD_SEC:-3}
ARM_OFFBOARD_MAX_RETRIES=${ARM_OFFBOARD_MAX_RETRIES:-6}
ARM_RETRY_SLEEP=${ARM_RETRY_SLEEP:-0.6}
OFFBOARD_RETRY_SLEEP=${OFFBOARD_RETRY_SLEEP:-0.8}
VEHICLE_STATUS_TIMEOUT_SEC=${VEHICLE_STATUS_TIMEOUT_SEC:-4}
SWARM_MODE=${SWARM_MODE:-hybrid}
AVOID_SCOPE=${AVOID_SCOPE:-leader}
AVOID_TIMEOUT=${AVOID_TIMEOUT:-0.25}
AVOID_RECOVERY_SEC=${AVOID_RECOVERY_SEC:-1.0}
ORCA_SAFE_RADIUS=${ORCA_SAFE_RADIUS:-1.2}
ORCA_MAX_SPEED=${ORCA_MAX_SPEED:-0.22}
ORCA_GAIN=${ORCA_GAIN:-0.45}
ORCA_OBS_GAIN=${ORCA_OBS_GAIN:-0.85}
ORCA_OBS_INFLUENCE_RADIUS=${ORCA_OBS_INFLUENCE_RADIUS:-2.2}
ORCA_ENABLE_AFTER_Z=${ORCA_ENABLE_AFTER_Z:--0.6}
ORCA_USE_WORLD_STATE=${ORCA_USE_WORLD_STATE:-0}
COMM_AVOID_ENABLE_Z=${COMM_AVOID_ENABLE_Z:--1.8}
COMM_AVOID_MIN_SPEED=${COMM_AVOID_MIN_SPEED:-0.03}
COMM_AVOID_Z_TRACK_TOL=${COMM_AVOID_Z_TRACK_TOL:-0.35}
DYNAMIC_OBS_ENABLE=${DYNAMIC_OBS_ENABLE:-1}
DYNAMIC_OBS_VISUALIZE_GZ=${DYNAMIC_OBS_VISUALIZE_GZ:-1}
DYNAMIC_OBS_WORLD=${DYNAMIC_OBS_WORLD:-${GZ_WORLD}}
OBSTACLE_CONFIG=${OBSTACLE_CONFIG:-~/ws_xtd2/scripts/obstacles.yaml}
# 墙体模式参数保留用于后续实验；当前默认无墙验证(DYNAMIC_OBS_ENABLE=0)不会启动该链路
DYNAMIC_OBS_MODE=${DYNAMIC_OBS_MODE:-static_wall}
DYNAMIC_OBS_SCENE_CONFIG=${DYNAMIC_OBS_SCENE_CONFIG:-~/ws_xtd2/scripts/scenes/easy_crossing.yaml}
DYNAMIC_OBS_START_CENTER_X=${DYNAMIC_OBS_START_CENTER_X:-35.0}
DYNAMIC_OBS_START_CENTER_Y=${DYNAMIC_OBS_START_CENTER_Y:-6.0}
DYNAMIC_OBS_ACTIVE_CENTER_X=${DYNAMIC_OBS_ACTIVE_CENTER_X:-5.5}
DYNAMIC_OBS_ACTIVE_CENTER_Y=${DYNAMIC_OBS_ACTIVE_CENTER_Y:-0.0}
DYNAMIC_OBS_SWITCH_AFTER_SEC=${DYNAMIC_OBS_SWITCH_AFTER_SEC:-$(awk "BEGIN{print int(${WARMUP_SEC}+${POST_OFFBOARD_HOLD_SEC}+10)}")}
DYNAMIC_OBS_SWITCH_FADE_SEC=${DYNAMIC_OBS_SWITCH_FADE_SEC:-6}
DYNAMIC_OBS_WALL_X=${DYNAMIC_OBS_WALL_X:-3.0}
DYNAMIC_OBS_WALL_Y=${DYNAMIC_OBS_WALL_Y:-1.0}
DYNAMIC_OBS_WALL_Z=${DYNAMIC_OBS_WALL_Z:-0.9}
DYNAMIC_OBS_WALL_LENGTH=${DYNAMIC_OBS_WALL_LENGTH:-30.0}
DYNAMIC_OBS_WALL_THICKNESS=${DYNAMIC_OBS_WALL_THICKNESS:-0.25}
DYNAMIC_OBS_WALL_HEIGHT=${DYNAMIC_OBS_WALL_HEIGHT:-4.8}
DYNAMIC_OBS_WALL_SEGMENT_SPACING=${DYNAMIC_OBS_WALL_SEGMENT_SPACING:-0.6}
SECOND_WALL_ENABLE=${SECOND_WALL_ENABLE:-1}
SECOND_WALL_DX=${SECOND_WALL_DX:-15.0}
REAR_WALL_GAP=${REAR_WALL_GAP:-18.0}
REAR_WALL_LENGTH=${REAR_WALL_LENGTH:-200.0}
SECOND_WALL_DY=${SECOND_WALL_DY:-$(awk "BEGIN{printf \"%.2f\", (${REAR_WALL_LENGTH}+${REAR_WALL_GAP})/2.0}")}
THIRD_WALL_ENABLE=${THIRD_WALL_ENABLE:-1}
DYNAMIC_OBS_TARGET_X=${DYNAMIC_OBS_TARGET_X:-30.0}
DYNAMIC_OBS_TARGET_MARGIN_Y=${DYNAMIC_OBS_TARGET_MARGIN_Y:-1.2}
DYNAMIC_OBS_TARGET_Y_BASE=${DYNAMIC_OBS_TARGET_Y_BASE:-$(awk "BEGIN{printf \"%.2f\", ${DYNAMIC_OBS_WALL_Y} + 25.0}")}
DYNAMIC_OBS_TARGET_Y_SPACING=${DYNAMIC_OBS_TARGET_Y_SPACING:-1.8}
if [ -f "${OBSTACLE_CONFIG/#\~/$HOME}" ]; then
  eval "$(python3 ~/ws_xtd2/scripts/obstacle_config.py --shell "${OBSTACLE_CONFIG}")"
fi
START_WALL_CLEARANCE=${START_WALL_CLEARANCE:-8.0}
MISSION_START_X=${MISSION_START_X:-$(awk "BEGIN{printf \"%.2f\", ${DYNAMIC_OBS_WALL_X} - ${START_WALL_CLEARANCE}}")}
USE_SPAWNED_FORMATION=${USE_SPAWNED_FORMATION:-1}
VERTICAL_TAKEOFF=${VERTICAL_TAKEOFF:-1}
LEADER_ID=${LEADER_ID:-$(((NUM_DRONES + 1) / 2))}
FORMATION_KP=${FORMATION_KP:-0.45}
LEADER_TRACK_KP=${LEADER_TRACK_KP:-0.45}
MAX_FOLLOWER_SPEED=${MAX_FOLLOWER_SPEED:-0.85}
MAX_LEADER_SPEED=${MAX_LEADER_SPEED:-0.85}
USE_HEADING_OFFSETS=${USE_HEADING_OFFSETS:-0}
LF_STATE_TIMEOUT=${LF_STATE_TIMEOUT:-4.0}
USE_VIRTUAL_LEADER=${USE_VIRTUAL_LEADER:-1}
FORMATION_METRICS_CSV=${FORMATION_METRICS_CSV:-${LOG_DIR}/formation_metrics.csv}
PATH_PLANNER_MODE=${PATH_PLANNER_MODE:-astar}
ASTAR_GRID_RESOLUTION=${ASTAR_GRID_RESOLUTION:-1.0}
ASTAR_WALL_THICKNESS=${ASTAR_WALL_THICKNESS:-${DYNAMIC_OBS_WALL_THICKNESS}}
ASTAR_OBS_INFLATION=${ASTAR_OBS_INFLATION:-}
ASTAR_MIN_WAYPOINT_DIST=${ASTAR_MIN_WAYPOINT_DIST:-2.0}
ASTAR_SMOOTH_ENABLE=${ASTAR_SMOOTH_ENABLE:-1}
export PATH_PLANNER_MODE
export ASTAR_GRID_RESOLUTION
export ASTAR_WALL_THICKNESS
export ASTAR_OBS_INFLATION
export ASTAR_MIN_WAYPOINT_DIST
export ASTAR_SMOOTH_ENABLE
export OBSTACLE_CONFIG
export VERTICAL_TAKEOFF
export SECOND_WALL_ENABLE
export SECOND_WALL_DX
export REAR_WALL_GAP
export REAR_WALL_LENGTH
export SECOND_WALL_DY
export THIRD_WALL_ENABLE

# 保证Gazebo里的墙顶明显高于任务飞行高度，避免视觉上出现“越墙”而不是“绕墙”
MIN_WALL_HEIGHT=$(awk "BEGIN{h=2.0*((-${MISSION_Z})-${DYNAMIC_OBS_WALL_Z}+0.8); if (h<4.8) h=4.8; printf \"%.2f\", h}")
if awk "BEGIN{exit !(${DYNAMIC_OBS_WALL_HEIGHT} < ${MIN_WALL_HEIGHT})}"; then
  echo "[INFO] raise wall height for side-pass validation: requested=${DYNAMIC_OBS_WALL_HEIGHT}, min_required=${MIN_WALL_HEIGHT}"
  DYNAMIC_OBS_WALL_HEIGHT=${MIN_WALL_HEIGHT}
fi

COMM_EXTRA_ARGS=""
ENABLE_FIXED_MISSION=1
if [ "${SWARM_MODE}" = "hybrid" ]; then
  COMM_EXTRA_ARGS="--avoid-timeout ${AVOID_TIMEOUT} --avoid-recovery-sec ${AVOID_RECOVERY_SEC} --avoid-enable-z ${COMM_AVOID_ENABLE_Z} --avoid-min-speed ${COMM_AVOID_MIN_SPEED} --avoid-z-track-tol ${COMM_AVOID_Z_TRACK_TOL}"
  ENABLE_FIXED_MISSION=0
fi
WARMUP_RUN_SEC=$(awk "BEGIN{print int(${WARMUP_SEC}+${POST_OFFBOARD_HOLD_SEC}+1)}")

echo "[MODE] SWARM_MODE=${SWARM_MODE}"
echo "[MODE] AVOID_SCOPE=${AVOID_SCOPE} (none=no drone AVOID takeover, leader=only center/leader can AVOID takeover, all=each drone can AVOID takeover)"
echo "[MODE] GZ_WORLD=${GZ_WORLD}"
echo "[MODE] COMM_EXTRA_ARGS=${COMM_EXTRA_ARGS:-<none>}"
echo "[MODE] ENABLE_FIXED_MISSION=${ENABLE_FIXED_MISSION}"
echo "[MODE] PX4_SYS_ID_BASE=${PX4_SYS_ID_BASE}"
echo "[MODE] leader-follower: FORMATION_KP=${FORMATION_KP}, LEADER_TRACK_KP=${LEADER_TRACK_KP}, MAX_FOLLOWER_SPEED=${MAX_FOLLOWER_SPEED}, MAX_LEADER_SPEED=${MAX_LEADER_SPEED}, USE_HEADING_OFFSETS=${USE_HEADING_OFFSETS}, LF_STATE_TIMEOUT=${LF_STATE_TIMEOUT}, USE_VIRTUAL_LEADER=${USE_VIRTUAL_LEADER}"
echo "[MODE] FORMATION_METRICS_CSV=${FORMATION_METRICS_CSV}"
echo "[MODE] PATH_PLANNER_MODE=${PATH_PLANNER_MODE}, ASTAR_GRID_RESOLUTION=${ASTAR_GRID_RESOLUTION}, ASTAR_WALL_THICKNESS=${ASTAR_WALL_THICKNESS}, ASTAR_OBS_INFLATION=${ASTAR_OBS_INFLATION:-auto}, ASTAR_MIN_WAYPOINT_DIST=${ASTAR_MIN_WAYPOINT_DIST}, ASTAR_SMOOTH_ENABLE=${ASTAR_SMOOTH_ENABLE}"
echo "[MODE] OBSTACLE_CONFIG=${OBSTACLE_CONFIG}"
echo "[MODE] DYNAMIC_OBS_MODE=${DYNAMIC_OBS_MODE}"
echo "[MODE] DYNAMIC_OBS_SCENE_CONFIG=${DYNAMIC_OBS_SCENE_CONFIG}"
echo "[MODE] SECOND_WALL_ENABLE=${SECOND_WALL_ENABLE}, SECOND_WALL_DX=${SECOND_WALL_DX}, REAR_WALL_GAP=${REAR_WALL_GAP}, REAR_WALL_LENGTH=${REAR_WALL_LENGTH}, SECOND_WALL_DY=${SECOND_WALL_DY}, THIRD_WALL_ENABLE=${THIRD_WALL_ENABLE}"
echo "[MODE] MISSION_START_X=${MISSION_START_X} (wall_x=${DYNAMIC_OBS_WALL_X}, clearance=${START_WALL_CLEARANCE})"
echo "[MODE] USE_SPAWNED_FORMATION=${USE_SPAWNED_FORMATION} (1=PX4 local commands start at each spawned drone origin)"
echo "[MODE] VERTICAL_TAKEOFF=${VERTICAL_TAKEOFF} (1=hold local x/y at 0 during warmup and stage1)"
echo "[TARGET][NED] leader_marker=(x=${DYNAMIC_OBS_TARGET_X}, y=${DYNAMIC_OBS_TARGET_Y_BASE}, z=${MISSION_Z})"

USE_GNOME_TERMINAL=0
if command -v gnome-terminal >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ] && gnome-terminal --version >/dev/null 2>&1; then
  USE_GNOME_TERMINAL=1
else
  echo "[WARN] gnome-terminal 不可用，后台启动所有节点并写入日志"
fi

launch_job() {
  local title="$1"
  local cmd="$2"
  if [ "${USE_GNOME_TERMINAL}" = "1" ]; then
    gnome-terminal --tab --title="${title}" -- bash -lc "${cmd}" || {
      echo "[WARN] gnome-terminal 启动失败: ${title}，回退后台执行"
      bash -lc "${cmd}" &
    }
  else
    bash -lc "${cmd}" &
  fi
}

# 彻底清理旧进程
echo "[INFO] kill old processes..."
pkill -f multirotor_communication || true
pkill -f MicroXRCEAgent || true
pkill -f "px4 -i" || true
pkill -f px4_sitl_default/bin/px4 || true
pkill -f "ros2 topic pub.*warmup" || true
pkill -f gz || true
pkill -f gazebo || true
sleep 5

# 编译+环境配置
echo "[INFO] build + source..."
cd ~/ws_xtd2 || { echo "[ERROR] 无法进入工作目录 ~/ws_xtd2"; exit 1; }
source /opt/ros/jazzy/setup.bash || { echo "[ERROR] 无法加载ROS 2环境"; exit 1; }
colcon build --packages-select xtd2_communication --symlink-install || { echo "[ERROR] 编译失败"; exit 1; }
source install/setup.bash || { echo "[ERROR] 无法加载工作空间环境"; exit 1; }

# 启动MicroXRCEAgent（全部加&后台运行）
echo "[INFO] start MicroXRCEAgent..."
for ((i=1; i<=NUM_DRONES; i++)); do
  port=$((START_PORT + i*10))
  launch_job "Agent_$i" "
source /opt/ros/jazzy/setup.bash;
MicroXRCEAgent udp4 -p ${port} | tee ${LOG_DIR}/agent_${i}.log;
exec bash
"
done
sleep 3

# 启动PX4（全部加&后台运行，避免阻塞）
echo "[INFO] start PX4..."
for ((i=1; i<=NUM_DRONES; i++)); do
  DDS_PORT=$((START_PORT + i*10))
  spawn_ned_x=${MISSION_START_X}
  spawn_ned_y=$(awk "BEGIN{printf \"%.2f\", ${DYNAMIC_OBS_WALL_Y} + ((${i}-${LEADER_ID})*${DYNAMIC_OBS_TARGET_Y_SPACING})}")
  spawn_enu_x=${spawn_ned_y}
  spawn_enu_y=${spawn_ned_x}
  if [ $i -eq 1 ]; then
    launch_job "PX4_$i" "
cd ~/PX4-Autopilot;
PX4_UXRCE_DDS_PORT=${DDS_PORT} PX4_GZ_WORLD=${GZ_WORLD} PX4_GZ_MODEL_POSE=\"${spawn_enu_x},${spawn_enu_y}\" PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i ${i} | tee ${LOG_DIR}/px4_${i}.log;
exec bash
"
  else
    launch_job "PX4_$i" "
cd ~/PX4-Autopilot;
PX4_UXRCE_DDS_PORT=${DDS_PORT} PX4_GZ_WORLD=${GZ_WORLD} PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"${spawn_enu_x},${spawn_enu_y}\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i ${i} | tee ${LOG_DIR}/px4_${i}.log;
exec bash
"
  fi
  sleep 7
done
sleep 15

# 启动通信节点
echo "[INFO] start communication nodes..."
IFS=',' read -r -a SYSIDS <<< "${TARGET_SYS_IDS_STR:-}"
# 提前校验ID数量匹配
if [ -n "${TARGET_SYS_IDS_STR}" ] && [ "${#SYSIDS[@]}" -ne "${NUM_DRONES}" ]; then
  echo "[ERROR] TARGET_SYS_IDS_STR 数量(${#SYSIDS[@]})与无人机数量(${NUM_DRONES})不匹配"
  exit 1
fi

for ((id=1; id<=NUM_DRONES; id++)); do
  ns="x500_${id}"
  px4_ns="px4_${id}"
  if [ -n "${TARGET_SYS_IDS_STR}" ]; then
    target_sys_id="${SYSIDS[$((id-1))]}"
  else
    target_sys_id=$((PX4_SYS_ID_BASE + id - 1))
  fi
  [ -z "${target_sys_id}" ] && { echo "[ERROR] 无人机${id}的target_sys_id为空"; exit 1; }
  role="follower"
  if [ "${USE_VIRTUAL_LEADER}" = "1" ]; then
    role="virtual_member"
    [ "${id}" -eq "${LEADER_ID}" ] && role="virtual_center"
  elif [ "${id}" -eq "${LEADER_ID}" ]; then
    role="LEADER"
  fi
  avoid_args=""
  if [ "${SWARM_MODE}" = "hybrid" ]; then
    if [ "${AVOID_SCOPE}" = "all" ] || { [ "${AVOID_SCOPE}" = "leader" ] && [ "${id}" -eq "${LEADER_ID}" ]; }; then
      avoid_args="--enable-avoid ${COMM_EXTRA_ARGS}"
    fi
  fi
  avoid_role="TASK_ONLY"
  [ -n "${avoid_args}" ] && avoid_role="AVOID_ENABLED"
  echo "[MAP] id=${id} -> control_ns=${ns} -> gazebo_model=x500_${id} -> ${px4_ns} -> target_sys_id=${target_sys_id} -> ${role} -> ${avoid_role}"

  launch_job "Comm_${id}" "
cd ~/ws_xtd2;
source /opt/ros/jazzy/setup.bash;
source install/setup.bash;
ros2 run xtd2_communication multirotor_communication --model gz_x500 --id ${id} --namespace ${ns} --px4-ns ${px4_ns} --target-sys-id ${target_sys_id} ${avoid_args} 2>&1 | tee ${LOG_DIR}/comm_${id}.log;
exec bash
"
done
sleep 5

echo "[INFO] start state_exchange..."
launch_job "StateExchange" "
cd ~/ws_xtd2;
source /opt/ros/jazzy/setup.bash;
source install/setup.bash;
python3 ~/ws_xtd2/scripts/swarm_state_exchange.py --num-drones ${NUM_DRONES} --rate 15 --spawned-formation ${USE_SPAWNED_FORMATION} --mission-start-x ${MISSION_START_X} --base-y ${DYNAMIC_OBS_WALL_Y} --y-spacing ${DYNAMIC_OBS_TARGET_Y_SPACING} --leader-id ${LEADER_ID} 2>&1 | tee ${LOG_DIR}/state_exchange.log;
exec bash
"
if [ "${SWARM_MODE}" = "hybrid" ]; then
  echo "[INFO] SWARM_MODE=hybrid, start local_avoid_orca..."
  launch_job "LocalORCA" "
cd ~/ws_xtd2;
source /opt/ros/jazzy/setup.bash;
source install/setup.bash;
python3 ~/ws_xtd2/scripts/local_avoid_orca.py --num-drones ${NUM_DRONES} --safe-radius ${ORCA_SAFE_RADIUS} --max-speed ${ORCA_MAX_SPEED} --gain ${ORCA_GAIN} --obs-gain ${ORCA_OBS_GAIN} --obs-influence-radius ${ORCA_OBS_INFLUENCE_RADIUS} --enable-after-z ${ORCA_ENABLE_AFTER_Z} --use-world-state ${ORCA_USE_WORLD_STATE} 2>&1 | tee ${LOG_DIR}/local_orca.log;
exec bash
"
  # 无墙目标点验证：默认不启动动态障碍节点；如需恢复墙体验证再打开 DYNAMIC_OBS_ENABLE=1
  if [ "${DYNAMIC_OBS_ENABLE}" = "1" ]; then
    launch_job "DynamicObstacles" "
cd ~/ws_xtd2;
source /opt/ros/jazzy/setup.bash;
source install/setup.bash;
  # DYNAMIC_OBS_MODE=static_wall uses obstacles.yaml; DYNAMIC_OBS_MODE=scene uses scenes/*.yaml.
  python3 ~/ws_xtd2/scripts/dynamic_obstacle_source.py --rate 10 --world ${DYNAMIC_OBS_WORLD} --visualize-gz ${DYNAMIC_OBS_VISUALIZE_GZ} --mode ${DYNAMIC_OBS_MODE} --scene-config ${DYNAMIC_OBS_SCENE_CONFIG} --obstacle-config ${OBSTACLE_CONFIG} --wall-x ${DYNAMIC_OBS_WALL_X} --wall-y ${DYNAMIC_OBS_WALL_Y} --wall-z ${DYNAMIC_OBS_WALL_Z} --wall-length ${DYNAMIC_OBS_WALL_LENGTH} --wall-thickness ${DYNAMIC_OBS_WALL_THICKNESS} --wall-height ${DYNAMIC_OBS_WALL_HEIGHT} --wall-segment-spacing ${DYNAMIC_OBS_WALL_SEGMENT_SPACING} --second-wall-enable ${SECOND_WALL_ENABLE} --second-wall-dx ${SECOND_WALL_DX} --second-wall-dy ${SECOND_WALL_DY} --rear-wall-length ${REAR_WALL_LENGTH} --third-wall-enable ${THIRD_WALL_ENABLE} --target-ball-enable 1 --target-ball-num-drones ${NUM_DRONES} --target-ball-leader-id ${LEADER_ID} --target-ball-target-x ${DYNAMIC_OBS_TARGET_X} --target-ball-base-y ${DYNAMIC_OBS_TARGET_Y_BASE} --target-ball-target-z ${MISSION_Z} --target-ball-y-spacing ${DYNAMIC_OBS_TARGET_Y_SPACING} 2>&1 | tee ${LOG_DIR}/dynamic_obstacles.log;
exec bash
"
  fi
fi

echo "[INFO] start warmup setpoint publishers..."
pkill -f "warmup_.*\.log" || true  # 杀死旧的预热日志进程
pkill -f "ros2 topic pub.*warmup" || true
for ((id=1; id<=NUM_DRONES; id++)); do
  ns="x500_${id}"
  if [ "${VERTICAL_TAKEOFF}" = "1" ]; then
    warmup_x=0.0
    warmup_y=0.0
  elif [ "${USE_SPAWNED_FORMATION}" = "1" ]; then
    warmup_x=0.0
    warmup_y=0.0
  else
    warmup_x=0.0
    warmup_y=$(awk "BEGIN{printf \"%.2f\", ${DYNAMIC_OBS_WALL_Y} + ((${id}-${LEADER_ID})*${DYNAMIC_OBS_TARGET_Y_SPACING})}")
  fi
  
  launch_job "WarmupSP_${id}" "
cd ~/ws_xtd2;
source /opt/ros/jazzy/setup.bash;
source install/setup.bash;
# 单行紧凑YAML格式，彻底解决解析错误
  timeout ${WARMUP_RUN_SEC}s ros2 topic pub -r 20 /xtdrone2/${ns}/cmd_pose_local_ned geometry_msgs/msg/Pose \"{position: {x: ${warmup_x}, y: ${warmup_y}, z: ${TAKEOFF_Z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}\" > "${LOG_DIR}/warmup_${id}.log" 2>&1;
exec bash
"
done

# 预热等待
echo "[INFO] warmup ${WARMUP_SEC}s..."
sleep "${WARMUP_SEC}"

# 集群控制窗口：简化为原始流程，避免复杂控制引发不稳定
CONTROL_CMD=$(cat <<EOF
cd ~/ws_xtd2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
(
echo '[Control] ready'
echo '[Control] log file: ${LOG_DIR}/control_cluster.log'
if [ ${AUTO_ARM_OFFBOARD} -eq 1 ]; then
  echo "[Control] 逐机 ARM -> OFFBOARD（防止前序机等待过久掉臂）..."
  for ((i=1; i<=${NUM_DRONES}; i++)); do
    ok=0
    px4_ns="px4_\${i}"
    for ((k=1; k<=${ARM_OFFBOARD_MAX_RETRIES}; k++)); do
      echo "[Control] x500_\${i} attempt \${k}/${ARM_OFFBOARD_MAX_RETRIES}: ARM -> OFFBOARD"
      arm_out=\$(ros2 service call /xtdrone2/x500_\${i}/cmd xtd2_msgs/srv/XTD2Cmd "{command: 'ARM'}" 2>&1)
      echo "\$arm_out"
      sleep ${ARM_RETRY_SLEEP}
      armed_state=\$(timeout ${VEHICLE_STATUS_TIMEOUT_SEC}s ros2 topic echo --once /\${px4_ns}/fmu/out/vehicle_status_v1 2>/dev/null | grep "arming_state:" || true)
      echo "[Control] \${px4_ns} \${armed_state:-arming_state: <timeout>}"
      if ! echo "\$armed_state" | grep -q "arming_state: 2"; then
        echo "[Control] x500_\${i} not armed yet, retry ARM"
        continue
      fi
      off_out=\$(ros2 service call /xtdrone2/x500_\${i}/cmd xtd2_msgs/srv/XTD2Cmd "{command: 'OFFBOARD'}" 2>&1)
      echo "\$off_out"
      sleep ${OFFBOARD_RETRY_SLEEP}
      nav_state=\$(timeout ${VEHICLE_STATUS_TIMEOUT_SEC}s ros2 topic echo --once /\${px4_ns}/fmu/out/vehicle_status_v1 2>/dev/null | grep "nav_state:" || true)
      echo "[Control] \${px4_ns} \${nav_state:-nav_state: <timeout>}"
      if echo "\$off_out" | grep -q "success=True" && echo "\$nav_state" | grep -q "nav_state: 14"; then
        ok=1
        echo "[Control] x500_\${i} OFFBOARD success"
        break
      fi
      echo "[Control] x500_\${i} OFFBOARD not reached yet"
    done
    if [ \${ok} -ne 1 ]; then
      echo "[Control][WARN] x500_\${i} OFFBOARD failed after ${ARM_OFFBOARD_MAX_RETRIES} retries"
    fi
    sleep ${ARM_TO_OFFBOARD_DELAY}
    sleep ${INTER_DRONE_GAP}
  done
  echo "[Control] OFFBOARD后继续warmup ${POST_OFFBOARD_HOLD_SEC}s，避免第三架未稳就切任务"
  sleep ${POST_OFFBOARD_HOLD_SEC}
  sleep 1

  echo '[Control] stop warmup setpoint publishers before mission...'
  warmup_pids=\$(ps -eo pid=,args= | awk '/timeout .* ros2 topic pub -r 20 \\/xtdrone2\\/.*\\/cmd_pose_local_ned geometry_msgs\\/msg\\/Pose/ {print \$1}')
  if [ -n "\$warmup_pids" ]; then
    echo "[Control] kill warmup pids: \$warmup_pids"
    kill \$warmup_pids || true
  else
    echo '[Control] no warmup publisher left'
  fi
  sleep 1

  if [ ${ENABLE_FIXED_MISSION} -eq 1 ]; then
    echo '[Control] START WALL-FOLLOW MISSION...'
    python3 ~/ws_xtd2/scripts/multi_waypoint2.py ${NUM_DRONES} 32 ${LEADER_ID} ${DYNAMIC_OBS_WALL_X} ${DYNAMIC_OBS_WALL_Y} ${DYNAMIC_OBS_WALL_LENGTH} ${DYNAMIC_OBS_TARGET_X} full wall_follow ${DYNAMIC_OBS_TARGET_Y_SPACING} ${TAKEOFF_Z} ${MISSION_Z} ${FORMATION_KP} ${LEADER_TRACK_KP} ${MAX_FOLLOWER_SPEED} ${MAX_LEADER_SPEED} ${USE_HEADING_OFFSETS} ${LF_STATE_TIMEOUT} ${USE_VIRTUAL_LEADER} ${FORMATION_METRICS_CSV} ${MISSION_START_X} ${USE_SPAWNED_FORMATION} ${DYNAMIC_OBS_TARGET_Y_BASE}
    echo '[Control] mission done'
  else
    echo '[Control] hybrid mode: start wall-follow mission with avoid arbitration active'
    python3 ~/ws_xtd2/scripts/multi_waypoint2.py ${NUM_DRONES} 32 ${LEADER_ID} ${DYNAMIC_OBS_WALL_X} ${DYNAMIC_OBS_WALL_Y} ${DYNAMIC_OBS_WALL_LENGTH} ${DYNAMIC_OBS_TARGET_X} full wall_follow ${DYNAMIC_OBS_TARGET_Y_SPACING} ${TAKEOFF_Z} ${MISSION_Z} ${FORMATION_KP} ${LEADER_TRACK_KP} ${MAX_FOLLOWER_SPEED} ${MAX_LEADER_SPEED} ${USE_HEADING_OFFSETS} ${LF_STATE_TIMEOUT} ${USE_VIRTUAL_LEADER} ${FORMATION_METRICS_CSV} ${MISSION_START_X} ${USE_SPAWNED_FORMATION} ${DYNAMIC_OBS_TARGET_Y_BASE}
    echo '[Control] hybrid staged mission done'
  fi
fi
) 2>&1 | tee ${LOG_DIR}/control_cluster.log
EOF
)

if command -v gnome-terminal >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  if gnome-terminal --tab --title="Control_集群" -- bash -c "${CONTROL_CMD}"; then
    echo "[INFO] Control 已在新终端启动"
  else
    echo "[WARN] gnome-terminal 启动控制窗口失败，回退为当前终端前台执行 Control"
    bash -c "${CONTROL_CMD}"
  fi
else
  echo "[WARN] 未检测到图形终端环境，回退为当前终端前台执行 Control"
  bash -c "${CONTROL_CMD}"
fi

echo "[DONE] started ${NUM_DRONES} drone(s), warmup enabled."
