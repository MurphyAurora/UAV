# UAV Swarm A* Planning and Obstacle Avoidance
## 项目简介

本项目用于多无人机集群路径规划与避障仿真实验。系统基于 ROS 2、PX4 SITL、Gazebo 和 XTDrone2 通信框架，实现了虚拟领航者、A* 全局路径规划、ORCA-lite 局部避障、静态墙体障碍和动态障碍物发布等功能。

当前主要功能包括：

- 多架 x500 无人机仿真启动；
- 虚拟领航点带队飞行；
- A* 全局绕墙路径规划；
- ORCA-lite 局部避障；
- 静态墙体和动态障碍球发布；
- 编队误差和任务日志记录。
## 运行环境
建议环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- PX4-Autopilot
- Gazebo Harmonic
- Micro XRCE-DDS Agent
- Python 3
- colcon

## 在其他电脑运行

本节适用于第一次在新电脑部署并运行本项目。

### 1. 克隆本仓库

```bash
git clone git@github.com:MurphyAurora/UAV.git ~/ws_xtd2
cd ~/ws_xtd2
```

如果没有配置 GitHub SSH，也可以使用 HTTPS：

```bash
git clone https://github.com/MurphyAurora/UAV.git ~/ws_xtd2
cd ~/ws_xtd2
```

### 2. 安装外部依赖

本仓库不包含 PX4-Autopilot、Micro-XRCE-DDS-Agent 和 Gazebo，需要在新电脑中提前安装：

- ROS 2 Jazzy
- Gazebo Harmonic
- PX4-Autopilot
- Micro-XRCE-DDS-Agent
- colcon
- Python 3

基础环境搭建可参考：

```text
https://github.com/andy-zhuo-02/XTDrone2
```

注意：`start1.sh` 默认假设 PX4-Autopilot 位于：
`text
~/PX4-Autopilot
`

如果 PX4 路径不同，需要修改 `scripts/start1.sh` 中对应路径。

### 3. 检查 ROS 2 消息依赖

通信节点依赖以下 ROS 2 包：

```text
xtd2_msgs
px4_msgs
xtd2_communication
```

其中 `xtd2_msgs` 和 `xtd2_communication` 已包含在本仓库中。  
如果本仓库没有包含 `src/px4_msgs`，需要在新电脑中额外安装或克隆对应版本的 `px4_msgs` 到 `src/` 目录。

目录结构应至少包含：

```text
ws_xtd2/
├── src/
│   ├── xtd2_communication/
│   ├── xtd2_msgs/
│   └── px4_msgs/
└── scripts/
```

### 4. 编译工作空间

```bash
cd ~/ws_xtd2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果编译失败，优先检查：

- `px4_msgs` 是否存在；
- `xtd2_msgs` 是否存在；
- ROS 2 Jazzy 是否正确 source；
- `tf_transformations` 是否安装。

### 5. 运行实验

```bash
cd ~/ws_xtd2/scripts
chmod +x start1.sh
SWARM_MODE=hybrid AVOID_SCOPE=none USE_VIRTUAL_LEADER=1 TARGET_SYS_IDS_STR=2,3,4 bash start1.sh 3
```

该命令表示启动 3 架无人机，并使用：

- `hybrid` 混合模式；
- A* 全局路径规划；
- 虚拟领航者；
- 关闭单机独立 AVOID 接管。

### 6. 查看运行日志

```bash
grep -n "A\\* planner\\|planner-path-table\\|mission result" ~/ws_xtd2/logs/control_cluster.log
```

如果看到：
`
mission result: SUCCESS
`说明任务完成。
## 系统架构

```text
                         PX4 / Gazebo SITL
                                │
                                │ DDS / uXRCE-DDS
                                ↓
                    ┌─────────────────────────┐
                    │  PX4 uORB / DDS Topics  │
                    └─────────────────────────┘
                                │
                                ↓
                    ┌─────────────────────────┐
                    │ multirotor_communication│
                    └─────────────────────────┘
                         ↑              ↓
                         │              │
      /xtdrone2/x500_i/cmd_*      /px4_i/fmu/in/*
                         │
       ┌─────────────────┼─────────────────┐
       │                 │                 │
       ↓                 ↓                 ↓
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│multi_waypoint│  │ local_orca   │  │ dynamic_obs  │
│ A* + VL      │  │ local avoid  │  │ obstacles    │
└──────────────┘  └──────────────┘  └──────────────┘
       ↑                 ↑                 │
       │                 │                 │
       └──────────┬──────┘                 │
                  │                        │
        /xtdrone2/swarm/state_exchange     │
                  ↑                        │
                  │                        ↓
          ┌──────────────┐     /xtdrone2/swarm/dynamic_obstacles
          │state_exchange│
          └──────────────┘
```
## 仓库结构
```text
ws_xtd2/
├── scripts/                         # 集群实验脚本与任务控制代码
│   ├── start1.sh                    # 主启动脚本
│   ├── multi_waypoint2.py           # 虚拟领航者、A* 全局路径规划、航点执行
│   ├── local_avoid_orca.py          # ORCA-lite 局部避障节点
│   ├── dynamic_obstacle_source.py   # 障碍物发布与 Gazebo 可视化
│   ├── swarm_state_exchange.py      # 多无人机状态聚合与广播
│   ├── static_obstacle_wall.sdf     # 静态墙体模型
│   ├── dynamic_obstacle_ball.sdf    # 动态障碍球/目标点模型
│   └── target_marker.sdf            # 目标点可视化模型
│
├── src/
│   ├── xtd2_communication/          # PX4/XTDrone2 通信与避障仲裁
│   ├── xtd2_msgs/                   # 自定义消息与服务
│   └── px4_msgs/                    # PX4 消息定义
│
├── docs/                            # 设计说明文档
├── build/                           # 编译生成目录，不提交
├── install/                         # 安装生成目录，不提交
├── log/                             # colcon 日志，不提交
└── logs/                            # 实验日志，不提交

```
## 节点说明

| 节点 / 脚本 | 位置 | 功能 |
|---|---|---|
| `start1.sh` | `scripts/` | 主启动脚本，启动 PX4、MicroXRCEAgent、通信节点、状态交换节点、障碍物节点和任务控制节点 |
| `multi_waypoint2.py` | `scripts/` | 任务控制核心，负责 A* 全局路径规划、虚拟领航点、编队跟随和任务阶段执行 |
| `local_avoid_orca.py` | `scripts/` | ORCA-lite 局部避障节点，根据无人机间距和障碍物距离生成局部避障速度 |
| `dynamic_obstacle_source.py` | `scripts/` | 障碍物发布节点，负责发布静态墙体、动态障碍物和目标点 marker |
| `swarm_state_exchange.py` | `scripts/` | 集群状态交换节点，聚合各无人机状态并发布 `/xtdrone2/swarm/state_exchange` |
| `multirotor_communication.py` | `src/xtd2_communication/` | PX4 与 XTDrone2 控制接口之间的通信桥接节点，负责 Offboard setpoint 发布和控制源仲裁 |
| `static_obstacle_wall.sdf` | `scripts/` | Gazebo 静态墙体模型 |
| `dynamic_obstacle_ball.sdf` | `scripts/` | Gazebo 动态障碍球/目标点模型 |
| `target_marker.sdf` | `scripts/` | Gazebo 目标点可视化模型 |

---

## 主要话题

| 话题 | 发布者 | 订阅者 | 作用 |
|---|---|---|---|
| `/xtdrone2/swarm/state_exchange` | `swarm_state_exchange.py` | `multi_waypoint2.py`, `local_avoid_orca.py` | 发布所有无人机的位置、速度、航向等状态 |
| `/xtdrone2/swarm/dynamic_obstacles` | `dynamic_obstacle_source.py` | `local_avoid_orca.py` | 发布动态障碍物或离散化墙体障碍点 |
| `/xtdrone2/swarm/target_markers` | `dynamic_obstacle_source.py` | `multi_waypoint2.py` | 发布目标点 marker，用于任务目标校验 |
| `/xtdrone2/x500_i/cmd_pose_local_ned` | `multi_waypoint2.py` | `multirotor_communication.py` | 位置控制指令 |
| `/xtdrone2/x500_i/cmd_vel_ned` | `multi_waypoint2.py` | `multirotor_communication.py` | 速度控制指令 |
| `/xtdrone2/x500_i/avoid_cmd_vel_ned` | `local_avoid_orca.py` | `multirotor_communication.py` | 局部避障速度指令 |
| `/px4_i/fmu/in/trajectory_setpoint` | `multirotor_communication.py` | PX4 | PX4 Offboard 轨迹设定点 |
| `/px4_i/fmu/in/offboard_control_mode` | `multirotor_communication.py` | PX4 | PX4 Offboard 控制模式 |
| `/px4_i/fmu/in/vehicle_command` | `multirotor_communication.py` | PX4 | ARM、模式切换等 PX4 命令 |

---

## 坐标系

| 组件 | 坐标系 | 说明 |
|---|---|---|
| Gazebo | ENU | `X=东`, `Y=北`, `Z=上` |
| PX4 Local Position | NED | `X=北`, `Y=东`, `Z=下` |
| `multi_waypoint2.py` | NED | A* 规划、虚拟领航点、任务航点均使用 NED |
| `local_avoid_orca.py` | NED | 局部避障速度使用 NED |
| `dynamic_obstacle_source.py` | NED / ENU | 对外发布 NED 障碍物，同时将 NED 转换为 ENU 用于 Gazebo 可视化 |
| `cmd_pose_local_ned` | NED | 无人机本地位置控制指令 |
| `cmd_vel_ned` | NED | 无人机本地速度控制指令 |
| `avoid_cmd_vel_ned` | NED | 局部避障速度指令 |


## 控制模式说明

| 参数 | 当前推荐值 | 说明 |
|---|---|---|
| `SWARM_MODE` | `hybrid` | 启动任务控制和局部避障相关节点 |
| `AVOID_SCOPE` | `none` | 禁止单架无人机独立 AVOID 接管，避免编队拉扯 |
| `USE_VIRTUAL_LEADER` | `1` | 使用虚拟领航点，而不依赖真实 leader |
| `PATH_PLANNER_MODE` | `astar` | 使用 A* 进行全局路径规划 |
| `TARGET_SYS_IDS_STR` | `2,3,4` | 三架 PX4 SITL 无人机对应的 MAV_SYS_ID |

当前控制链路可以概括为：

```text
A* 全局路径
    ↓
虚拟领航点沿航点移动
    ↓
所有无人机跟随虚拟领航点
    ↓
multirotor_communication.py 转换为 PX4 Offboard 控制指令
    ↓
PX4 SITL 执行飞行
```


## 日志查看

运行日志位于：
`~/ws_xtd2/logs/`

重要日志：
```
control_cluster.log      主任务控制日志，包含 A* 路径规划输出
local_orca.log           局部避障日志
dynamic_obstacles.log    障碍物发布日志
comm_1.log               1号无人机通信节点日志
comm_2.log               2号无人机通信节点日志
comm_3.log               3号无人机通信节点日志
formation_metrics.csv    编队误差指标
```
查看 A* 是否生效：

`grep -n "A\\* planner\\|planner-path-table\\|astar_wp" ~/ws_xtd2/logs/control_cluster.log`

查看任务是否成功：

`grep -n "mission result" ~/ws_xtd2/logs/control_cluster.log`


## 成功判定

一次实验通常需要满足以下条件：

- 所有无人机成功起飞；
- 所有无人机进入 OFFBOARD 模式；
- 日志中出现 A* 规划结果；
- 无人机没有穿墙或越墙；
- 编队能够绕过墙体；
- 所有无人机到达目标点；
- 日志中出现 `mission result: SUCCESS`。


## 常见问题

### 1. GitHub 上传时提示密码错误

GitHub 不再支持使用账号密码进行 HTTPS push。建议使用 SSH key，或者使用 Personal Access Token。

### 2. A* 日志没有出现

确认已经重新运行任务，并检查：

`grep -n "A\\* planner" ~/ws_xtd2/logs/control_cluster.log`

### 3. 无人机没有绕墙
检查：
`PATH_PLANNER_MODE` 是否为 astar；
墙体参数是否正确；
gazebo 中墙的位置是否与代码参数一致；
planner-path-table 中航点是否绕过墙。
### 4. 后两架无人机跟不上
可能原因：

虚拟领航点速度过快；
`MAX_FOLLOWER_SPEED`太小；
局部避障接管过强；
队形间距过小；
`virtual leader paused` 频繁出现。

## 注意事项

- 本仓库不包含 `build/`、`install/`、`log/`、`logs/` 等生成目录；
- 本仓库不包含 PX4-Autopilot；
- 本仓库不包含 Micro-XRCE-DDS-Agent；
- 当前 A* 主要面向已知静态墙体障碍；
- 动态障碍主要由 ORCA-lite 局部避障处理；
- 后续可扩展为多障碍物列表和动态重规划。
## 后续计划

- 支持多个静态障碍物；
- 将单墙参数升级为障碍物列表；
- 支持动态障碍重规划；
- 增加路径可视化；
- 优化虚拟领航点轨迹平滑；
- 增加实验结果绘图；
- 对比 A*、RRT、D*、MPC 等算法。









