# 可复现动态障碍场景 Demo

这个 demo 是一个面向无人机集群算法测试的离线动态障碍场景生成组件。

它目前还没有接入 ROS、Gazebo 或具体避障算法。当前阶段的目标是先把“场景配置、轨迹生成、指标摘要、批量 benchmark、可视化”做清楚，避免后续接入实时系统时把平台逻辑和算法逻辑混在一起。

## 文件说明

- `scenario_demo.py`
  - 读取场景 YAML 文件。
  - 生成动态障碍轨迹。
  - 输出 `obstacles.csv`、`summary.json`、`resolved_scene.json`。
  - 支持单场景运行，也支持 benchmark suite 批量运行。

- `plot_scenario.py`
  - 绘制场景的 2D x/y 视图。
  - 显示静态墙、起点、参考航线、目标点和动态障碍轨迹。

- `ros_scene_probe.py`
  - 订阅 `/xtdrone2/swarm/dynamic_obstacles`。
  - 只打印带 `name` 字段的动态障碍，过滤掉静态墙转换出来的大量 collision points。

- `orca_reaction_test.py`
  - 发布假的无人机状态。
  - 订阅 `/xtdrone2/x500_1/avoid_cmd_vel_ned`。
  - 用于验证 ORCA 收到障碍后是否输出非零避障速度。

- `ros_run_recorder.py`
  - 订阅 `/xtdrone2/swarm/state_exchange` 和 `/xtdrone2/swarm/dynamic_obstacles`。
  - 输出无人机轨迹 `drones.csv` 和障碍物轨迹 `obstacles_ros.csv`。
  - 用于后续计算最小距离、近碰撞次数、路径长度等算法指标。

- `analyze_ros_run.py`
  - 读取 `ros_run_recorder.py` 保存的 CSV。
  - 自动统计动态障碍轨迹范围、无人机到障碍物的最近距离、ORCA 避障速度输出。
  - 用于判断“是否真的进入避障范围”和“ORCA 是否真的输出过非零避障速度”。

- `plot_ros_run.py`
  - 读取 `ros_run_recorder.py` 保存的 CSV。
  - 绘制真实无人机轨迹、动态障碍轨迹、最近距离点和 ORCA 非零输出段。
  - 用于把 Gazebo 中的肉眼观察和日志数据对齐。

- `scenes/demo_dynamic.yaml`
  - 最初的小 demo 场景。

- `scenes/easy_crossing.yaml`
  - easy 难度场景：一个慢速穿越障碍。

- `scenes/medium_multi_crossing.yaml`
  - medium 难度场景：多个穿越障碍和巡逻障碍。

- `scenes/hard_dense_crossing.yaml`
  - hard 难度场景：密集穿越和目标区域巡逻。

- `scenes/benchmark_v1.yaml`
  - 批量 benchmark 配置，包含 easy、medium、hard 三个场景。

## 运行单个场景

```bash
cd /home/lililab/ws_xtd2/scripts

python3 scenario_demo.py --scene scenes/easy_crossing.yaml --output-dir /tmp/check_easy
```

预期输出：

```text
/tmp/check_easy/
  obstacles.csv
  summary.json
  resolved_scene.json
```

`obstacles.csv` 中每一行表示某个时间点的某个动态障碍：

```text
time,name,type,x,y,z,vx,vy,vz,radius
```

`summary.json` 中包含场景指标，例如：

- 动态障碍数量
- 最大障碍物速度
- 平均障碍物速度
- 每个障碍物的路径长度
- 靠近参考航线的事件次数
- 到参考航线的最小净距离
- 难度分数

## 运行 Benchmark Suite

```bash
python3 scenario_demo.py --suite scenes/benchmark_v1.yaml --output-dir /tmp/check_benchmark
```

预期输出：

```text
/tmp/check_benchmark/
  suite_summary.csv
  suite_summary.json
  resolved_suite.json
  easy_crossing_seed101/
  medium_multi_crossing_seed202/
  hard_dense_crossing_seed303/
```

总表中应该能看到三个难度等级：

```text
easy:   difficulty_score 约为 31.2
medium: difficulty_score 约为 64.7
hard:   difficulty_score 约为 100.0
```

## 验证可复现性

用同一个 seed 运行同一个场景两次：

```bash
python3 scenario_demo.py --scene scenes/easy_crossing.yaml --output-dir /tmp/check_easy_a
python3 scenario_demo.py --scene scenes/easy_crossing.yaml --output-dir /tmp/check_easy_b
```

比较两次生成的轨迹：

```bash
diff -u /tmp/check_easy_a/obstacles.csv /tmp/check_easy_b/obstacles.csv
```

如果命令没有任何输出，说明两个轨迹文件完全一致。

验证完整 benchmark：

```bash
python3 scenario_demo.py --suite scenes/benchmark_v1.yaml --output-dir /tmp/check_benchmark_a
python3 scenario_demo.py --suite scenes/benchmark_v1.yaml --output-dir /tmp/check_benchmark_b

diff -u /tmp/check_benchmark_a/easy_crossing_seed101/obstacles.csv /tmp/check_benchmark_b/easy_crossing_seed101/obstacles.csv
diff -u /tmp/check_benchmark_a/medium_multi_crossing_seed202/obstacles.csv /tmp/check_benchmark_b/medium_multi_crossing_seed202/obstacles.csv
diff -u /tmp/check_benchmark_a/hard_dense_crossing_seed303/obstacles.csv /tmp/check_benchmark_b/hard_dense_crossing_seed303/obstacles.csv
```

这三个 `diff` 命令都应该没有输出。

## 绘制场景图

直接从场景 YAML 生成轨迹图：

```bash
python3 plot_scenario.py --scene scenes/hard_dense_crossing.yaml --output /tmp/hard_plot.png
```

也可以绘制已经生成好的 CSV：

```bash
python3 plot_scenario.py \
  --scene scenes/medium_multi_crossing.yaml \
  --csv /tmp/check_benchmark/medium_multi_crossing_seed202/obstacles.csv \
  --output /tmp/medium_plot.png
```

## 当前验证标准

如果下面四项都通过，就说明当前离线 demo 是健康的：

1. 每个场景都能生成 `obstacles.csv`。
2. 同一个场景、同一个 seed 运行两次，生成的 `obstacles.csv` 完全一致。
3. `easy`、`medium`、`hard` 的难度分数按顺序递增。
4. `plot_scenario.py` 能生成非空的轨迹图。

## 当前范围

已经实现：

- 参数化场景 YAML
- 基于 seed 的确定性轨迹生成
- easy / medium / hard 三档 benchmark 场景
- 场景指标和难度分数
- benchmark suite 批量运行
- 2D 轨迹可视化
- `dynamic_obstacle_source.py --mode scene` 从场景 YAML 发布 ROS 动态障碍 topic

尚未实现：

- 将新动态障碍轨迹接入 Gazebo 可视化
- 真实无人机集群算法评估
- 无人机轨迹记录和碰撞指标统计

## ROS 发布接入

当前已经可以让 `dynamic_obstacle_source.py` 读取 `scenes/*.yaml`，并把其中的静态墙和动态障碍发布到：

```text
/xtdrone2/swarm/dynamic_obstacles
```

最小运行命令：

```bash
python3 dynamic_obstacle_source.py \
  --mode scene \
  --scene-config scenes/easy_crossing.yaml \
  --visualize-gz 0 \
  --target-ball-enable 0
```

在另一个终端里查看 topic：

```bash
ros2 topic echo /xtdrone2/swarm/dynamic_obstacles
```

由于这个 topic 里会先包含很多静态墙 collision points，直接 `ros2 topic echo` 容易只看到墙体点。建议用 probe 脚本只看动态障碍：

```bash
python3 ros_scene_probe.py
```

easy 场景中，`slow_crossing_ball` 的离线参考值是：

```text
scene_time 约 6.0:  x=12.0000, y=-7.3000, z=-3.0000, vx=0.0000, vy=0.7000
scene_time 约 10.0: x=12.0000, y=-4.5000, z=-3.0000, vx=0.0000, vy=0.7000
```

发布内容仍然使用现有算法已经能读取的 JSON 字段：

```text
obs_id, x, y, z, vx, vy, vz, radius
```

说明：

- `x/y/z` 使用 NED 坐标，和当前 swarm-control 侧保持一致。
- 静态墙会转换成一串 collision points。
- 动态障碍会按 YAML 中的 `line` 或 `circle` 轨迹实时更新。
- 当前只保证 ROS topic 数据发布；Gazebo 中多个动态障碍球的可视化还没有做。

## 算法接收验证

`local_avoid_orca.py` 已经会订阅 `/xtdrone2/swarm/dynamic_obstacles`。为了确认算法节点确实收到 scene 发布出来的障碍物，当前在接收回调中加入了限频日志。

先启动 scene 发布器：

```bash
python3 dynamic_obstacle_source.py \
  --mode scene \
  --scene-config scenes/easy_crossing.yaml \
  --visualize-gz 0 \
  --target-ball-enable 0
```

再启动 ORCA 节点：

```bash
python3 local_avoid_orca.py --num-drones 5
```

如果接收成功，ORCA 日志中应该出现类似内容：

```text
orca obs input: total=52, named_dynamic=1, sample_name=slow_crossing_ball, sample=(12.00,-7.30), radius=0.45
```

含义：

- `total=52`：收到的障碍物总数，包含静态墙 collision points。
- `named_dynamic=1`：收到的带 `name` 字段的动态障碍数量。
- `sample_name=slow_crossing_ball`：当前优先打印动态障碍，而不是墙点。

## ORCA 控制反应验证

算法接收验证只能说明 ORCA 节点收到了障碍物。进一步可以用 `orca_reaction_test.py` 验证 ORCA 是否会输出避障速度。

终端 1：启动 scene 障碍发布器。

```bash
python3 dynamic_obstacle_source.py \
  --mode scene \
  --scene-config scenes/easy_crossing.yaml \
  --visualize-gz 0 \
  --target-ball-enable 0
```

终端 2：启动 ORCA。这里把高度门控设为 `0.0`，让测试中的 `z=-3.0` 状态可以解锁。

```bash
python3 local_avoid_orca.py \
  --num-drones 5 \
  --enable-after-z 0.0
```

终端 3：运行反应测试。测试脚本会先订阅当前动态障碍，然后把假的 1 号无人机放在该动态障碍旁边，因此不要求 scene 发布器刚好处在 `scene_time=6.0`。

```bash
python3 orca_reaction_test.py \
  --num-drones 5 \
  --output-dir /tmp/orca_reaction_check
```

如果成功，应该看到：

```text
avoid_cmd x=... y=... speed=...
PASS: ORCA produced non-zero avoid command
summary=/tmp/orca_reaction_check/reaction_summary.json
```

`reaction_summary.json` 会记录：

- `passed`
- `max_speed_xy`
- `first_nonzero_time_sec`
- `cmd_count`
- `current_obstacle`
- `last_cmd`

这个测试不启动真实无人机，只验证：

```text
fake swarm state + dynamic obstacle -> local_avoid_orca.py -> avoid_cmd_vel_ned
```

如果看到 `FAIL: no non-zero avoid command within timeout`，先检查：

- scene 发布器是否仍在运行。
- `ros_scene_probe.py` 是否能看到带 `name` 的动态障碍。
- ORCA 是否用 `--enable-after-z 0.0` 启动。
- ORCA 日志是否出现 `orca obs input: ... named_dynamic=1`。

如果只是想先排除 ORCA 本身是否会对障碍产生反应，可以让测试脚本自己发布一个测试障碍：

```bash
python3 orca_reaction_test.py \
  --num-drones 5 \
  --publish-test-obstacle \
  --output-dir /tmp/orca_reaction_self_check
```

这个自测不依赖 `dynamic_obstacle_source.py`，只验证：

```text
orca_reaction_test.py 发布测试障碍和 fake swarm state
  -> local_avoid_orca.py
  -> avoid_cmd_vel_ned
```

## 真实仿真集成测试

`start1.sh` 已支持通过环境变量切换动态障碍模式。默认仍然是旧的 `static_wall` 模式；只有显式设置 `DYNAMIC_OBS_MODE=scene` 时才会读取 `scenes/*.yaml`。

建议先跑 `easy_flight_reaction.yaml`。这个场景用于真实飞行反应观察：动态球在参考航线附近做圆形巡逻，不会像 `easy_crossing.yaml` 那样到达终点后停止。

```bash
cd /home/lililab/ws_xtd2/scripts

DYNAMIC_OBS_ENABLE=1 \
DYNAMIC_OBS_MODE=scene \
DYNAMIC_OBS_SCENE_CONFIG=~/ws_xtd2/scripts/scenes/easy_flight_reaction.yaml \
DYNAMIC_OBS_VISUALIZE_GZ=1 \
ORCA_USE_WORLD_STATE=1 \
./start1.sh 5
```

运行后重点看两个日志：

```bash
tail -f ~/ws_xtd2/logs/dynamic_obstacles.log
tail -f ~/ws_xtd2/logs/local_orca.log
```

`dynamic_obstacles.log` 中应该能看到：

```text
scene config loaded: ...
scene publish: ... dynamic=1
gazebo visual obstacle spawned successfully ... entity=scene_dynamic_obstacle_1
```

`local_orca.log` 中应该能看到：

```text
orca obs input: total=..., named_dynamic=1, sample_name=slow_crossing_ball, ...
```

这一步的通过标准不是“飞得完美”，而是：

1. `start1.sh` 能启动 scene 模式动态障碍发布器。
2. `/xtdrone2/swarm/dynamic_obstacles` 中能看到 `slow_crossing_ball`。
3. ORCA 日志能看到 `named_dynamic=1`。
4. 无人机运行过程中没有因为 scene 模式导致脚本崩溃。

Gazebo 可视化当前是最小版：

- `scene` 模式只显示第一个带 `name` 的动态障碍。
- Gazebo entity tree 中应出现 `scene_dynamic_obstacle_1`。
- 多个动态障碍球和 scene 静态墙体的 Gazebo 可视化还没有接入。

### 已知现象和处理

- `easy_crossing.yaml` 中的动态球后期不动是正常的：它是 `line` 轨迹，到达 `end` 后速度会变为 0。
- 如果要观察真实飞行避障，优先使用 `easy_flight_reaction.yaml`。它的动态球是 `circle` 轨迹，会持续在航线附近巡逻。
- `USE_SPAWNED_FORMATION=1` 时，各 PX4 local frame 原点不同。`swarm_state_exchange.py` 现在会额外发布 `world_x/world_y`。
- 为了不影响旧实验，`local_avoid_orca.py` 默认仍使用旧的 local `x/y`。只有设置 `ORCA_USE_WORLD_STATE=1` 时，ORCA 才会优先使用 `world_x/world_y` 做避障距离计算。
- 如果仍然看到 `local_orca.log` 中 `min_dist` 长时间接近 0，优先检查 `ORCA_USE_WORLD_STATE=1` 是否已设置，并检查 `state_exchange.log` 中是否显示 `spawned_formation=1`。

## Gazebo 可视化集成验证记录

当 easy 场景能在 Gazebo 中看到动态障碍球时，建议把这次运行作为一次集成验证记录保存下来。

通过标准：

1. `dynamic_obstacles.log` 中出现 scene 配置加载记录。
2. `dynamic_obstacles.log` 中持续出现 `scene publish: ... dynamic=1`。
3. `dynamic_obstacles.log` 中出现 `scene_dynamic_obstacle_1` 的 Gazebo spawn 成功记录。
4. Gazebo entity tree 中能看到 `scene_dynamic_obstacle_1`。
5. Gazebo 画面中能看到一个红色动态障碍球。
6. `local_orca.log` 中出现 `named_dynamic=1` 和 `sample_name=slow_crossing_ball`。
7. 运行过程中没有因为 scene 模式导致 `dynamic_obstacle_source.py` 或 `local_avoid_orca.py` 崩溃。

建议检查命令：

```bash
grep -E "scene config loaded|scene visual sdf|scene obstacle spawned|scene_dynamic_obstacle_1|scene publish" \
  ~/ws_xtd2/logs/dynamic_obstacles.log

grep -E "orca obs input|named_dynamic|slow_crossing_ball" \
  ~/ws_xtd2/logs/local_orca.log
```

建议保存一组日志：

```bash
mkdir -p ~/ws_xtd2/logs/scenario_demo_gazebo_easy_check

cp ~/ws_xtd2/logs/dynamic_obstacles.log \
  ~/ws_xtd2/logs/scenario_demo_gazebo_easy_check/

cp ~/ws_xtd2/logs/local_orca.log \
  ~/ws_xtd2/logs/scenario_demo_gazebo_easy_check/

cp ~/ws_xtd2/logs/state_exchange.log \
  ~/ws_xtd2/logs/scenario_demo_gazebo_easy_check/ 2>/dev/null || true

cp ~/ws_xtd2/scripts/scenes/easy_crossing.yaml \
  ~/ws_xtd2/logs/scenario_demo_gazebo_easy_check/
```

也可以额外保存离线参考输出，方便之后对照：

```bash
python3 ~/ws_xtd2/scripts/scenario_demo.py \
  --scene ~/ws_xtd2/scripts/scenes/easy_crossing.yaml \
  --output-dir ~/ws_xtd2/logs/scenario_demo_gazebo_easy_check/offline_reference
```

这组记录用于证明：

```text
scene YAML
  -> ROS 动态障碍发布
  -> ORCA 接收
  -> Gazebo 显示第一个动态障碍球
```

## 下一步

下一阶段可以开始观察 easy 场景中的真实飞行反应，并逐步增加无人机轨迹记录和算法级指标。

## ROS 运行轨迹记录

真实仿真运行时，可以单独启动 `ros_run_recorder.py` 保存无人机、动态障碍和 ORCA 避障输出。

建议在启动 `start1.sh` 后，另开一个终端运行：

```bash
cd /home/lililab/ws_xtd2/scripts

python3 ros_run_recorder.py \
  --output-dir ~/ws_xtd2/logs/ros_run_easy_flight_reaction \
  --duration-sec 120
```

输出文件：

```text
~/ws_xtd2/logs/ros_run_easy_flight_reaction/
  drones.csv
  obstacles_ros.csv
  avoid_cmds.csv
  recording_summary.json
```

`drones.csv` 主要字段：

```text
time,source_stamp,drone_id,x,y,z,world_x,world_y,vx,vy,vz,heading
```

`obstacles_ros.csv` 主要字段：

```text
time,source_stamp,scene_time,obs_id,name,x,y,z,vx,vy,vz,radius
```

`avoid_cmds.csv` 主要字段：

```text
time,drone_id,vx,vy,vz,speed_xy
```

记录结束后先检查：

```bash
cat ~/ws_xtd2/logs/ros_run_easy_flight_reaction/recording_summary.json
head ~/ws_xtd2/logs/ros_run_easy_flight_reaction/drones.csv
head ~/ws_xtd2/logs/ros_run_easy_flight_reaction/obstacles_ros.csv
head ~/ws_xtd2/logs/ros_run_easy_flight_reaction/avoid_cmds.csv
```

如果 `drone_rows`、`obstacle_rows` 和 `avoid_rows` 都大于 0，就说明真实运行数据已经保存。下一步可基于这些 CSV 计算算法级指标。

## ROS 运行记录分析

记录结束后，可以用 `analyze_ros_run.py` 自动分析这一轮运行：

```bash
python3 analyze_ros_run.py \
  --run-dir ~/ws_xtd2/logs/ros_run_easy_flight_reaction \
  --obstacle-center-x 12.0 \
  --obstacle-center-y 10.4 \
  --influence-radius 2.0
```

它会输出：

- `dynamic_obstacle`：是否记录到了带 `name` 的动态障碍。
- `obstacle_motion`：动态障碍的运动范围和平均速度。
- `obstacle_center_check`：如果传入圆心，检查障碍是否稳定绕圆心运动。
- `distance_by_drone`：每架无人机到动态障碍的最近距离。
- `avoid_by_drone`：每架无人机的避障输出行数、非零行数和最大避障速度。
- `verdict`：这一轮是否满足“障碍进入影响范围，并且 ORCA 输出过避障速度”。

同时会在运行目录下保存：

```text
analysis_summary.json
```

对于 `easy_flight_reaction.yaml`，如果看到类似结果：

```text
entered_influence=True
nonzero_rows > 0
verdict=PASS: obstacle reached route and ORCA produced avoid output
```

说明这一轮测试已经证明：

```text
动态障碍发布 -> 无人机接近障碍 -> ORCA 输出避障速度
```

如果 Gazebo 画面中小球看起来卡住或被影响，但 `analysis_summary.json` 中障碍轨迹范围、速度和圆心半径都稳定，通常说明问题在 Gazebo 可视化同步，不在 ROS 障碍数据本身。

## ROS 运行轨迹可视化

如果日志显示 ORCA 有输出，但 Gazebo 中肉眼看起来“不明显”或“不像在躲”，可以用 `plot_ros_run.py` 画真实轨迹图：

```bash
python3 plot_ros_run.py \
  --run-dir ~/ws_xtd2/logs/ros_run_easy_flight_reaction_stronger \
  --influence-radius 3.5
```

默认输出位置：

```text
~/ws_xtd2/logs/ros_run_easy_flight_reaction_stronger/plots/overview.png
~/ws_xtd2/logs/ros_run_easy_flight_reaction_stronger/plots/zoom_obstacle.png
~/ws_xtd2/logs/ros_run_easy_flight_reaction_stronger/plots/obstacle_avoid.png
```

图中包含：

- 彩色实线：每架无人机的 `world_x/world_y` 真实轨迹。
- 黑色实线：带 `name` 的动态障碍轨迹。
- 浅色实心点：该无人机 ORCA 非零输出时对应的轨迹位置，可能来自无人机互相避让或动态障碍避让。
- 空心描边点：该无人机 ORCA 非零输出，且同时处在动态障碍影响范围内。
- 菱形点和虚线：每架无人机到动态障碍的最近距离。
- 虚线圆：最近点处的障碍影响范围，半径为 `--influence-radius + obstacle_radius`。

三张图的用途：

- `overview.png`：完整飞行轨迹总览。
- `zoom_obstacle.png`：动态障碍附近放大图。
- `obstacle_avoid.png`：突出显示动态障碍影响范围内的避障输出点。

如果只想生成一张图，可以指定：

```bash
python3 plot_ros_run.py \
  --run-dir ~/ws_xtd2/logs/ros_run_easy_flight_reaction_stronger \
  --influence-radius 3.5 \
  --plot-set single \
  --view zoom
```

这张图用于回答：

```text
无人机是否真的靠近障碍？
ORCA 非零输出发生在轨迹哪里？
避障速度是否只是在任务速度上做小幅修正？
Gazebo 画面中的“没躲”是不是只是视觉上不明显？
```

同时可以查看 ORCA 诊断日志：

```bash
grep "orca obs effect" ~/ws_xtd2/logs/local_orca.log | tail -n 40
```

其中：

- `min_obs_dist`：无人机到最近动态障碍中心的距离。
- `triggers`：本轮 ORCA 中进入障碍影响半径的次数。
- `max_strength`：本轮最大的障碍避让贡献。
- `use_world_state`：是否启用 world 坐标计算。

建议路径：

```text
scene YAML
  -> 离线生成、指标、可视化
  -> ROS 动态障碍发布器
  -> 无人机集群算法测试
  -> 基于无人机轨迹和障碍物轨迹的算法级评估
```
