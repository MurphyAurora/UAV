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

尚未实现：

- 从新场景 YAML 格式实时发布 ROS topic
- 将新动态障碍轨迹接入 Gazebo 可视化
- 真实无人机集群算法评估
- 无人机轨迹记录和碰撞指标统计

## 下一步

下一阶段可以让 `dynamic_obstacle_source.py` 复用同一套场景 YAML 格式。

建议路径：

```text
scene YAML
  -> 离线生成、指标、可视化
  -> ROS 动态障碍发布器
  -> 无人机集群算法测试
  -> 基于无人机轨迹和障碍物轨迹的算法级评估
```
