#!/bin/bash
set -e
# 增强版清理逻辑：
echo "杀死所有PX4/Gazebo/ROS相关仿真进程..."
pkill -f multirotor_communication
pkill -f MicroXRCEAgent
pkill -f px4
pkill -f gz
pkill -f gazebo
pkill -f "ros2 topic pub -r 10 /xtdrone2/x500_"
sleep 2


NUM_DRONES=1
START_PORT=8888

# 启动Agent
for ((i=1; i<=$NUM_DRONES; i++)); do
    gnome-terminal --tab --title="Agent_$i" -- bash -c "MicroXRCEAgent udp4 -p $((START_PORT + i*10)); exec bash"
done
sleep 3

# 【改动】启动多个PX4实例（不用make，直接用参数控制）
for ((i=0; i<$NUM_DRONES; i++)); do
    if [ $i -eq 0 ]; then
        # 第一个实例：主服务器，负责拉起Gazebo
        gnome-terminal --tab --title="PX4_$i" -- bash -c \
        "export DISPLAY=$DISPLAY; cd ~/PX4-Autopilot; PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $((i+1)); exec bash"
    else
        # 其它实例：客户端模式
        gnome-terminal --tab --title="PX4_$i" -- bash -c \
        "export DISPLAY=$DISPLAY; cd ~/PX4-Autopilot; PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,$i\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $((i+1)); exec bash"
    fi
    sleep 7  # 间隔2秒，确保前一个实例启动完毕
done
sleep 15


# 启动通信节点
for ((id=0; id<$NUM_DRONES; id++)); do
    gnome-terminal --tab --title="Comm_${id}" -- bash -c \
    "cd ~/ws_xtd2 && source install/setup.bash && ros2 run xtd2_communication multirotor_communication --model gz_x500 --id $id --px4-ns px4_$((id+1)); exec bash"
done
sleep 5


# 控制终端
gnome-terminal --tab --title="Control_集群" -- bash -c \
"cd ~/ws_xtd2; source install/setup.bash; echo '【集群控制终端】'; exec bash"
