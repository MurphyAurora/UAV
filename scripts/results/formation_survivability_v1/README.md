# Formation Survivability Benchmark V1

这个目录是“可复现动态障碍场景生成平台”的场景级难度评估 V1 结果包。

## 数据来源

Suite:

```text
scenes/benchmark_v1.yaml
```

包含三个固定 seed 的场景：

```text
easy_crossing
medium_multi_crossing
hard_dense_crossing
```

## 复现命令

在 `scripts/` 根目录运行：

```bash
./run_benchmark_v1.sh
```

等价于：

```bash
python3 difficulty_analysis/formation_survivability_experiment.py \
  --suite scenes/benchmark_v1.yaml \
  --output-dir results/formation_survivability_v1

python3 difficulty_analysis/plot_difficulty_analysis.py \
  --summary-csv results/formation_survivability_v1/suite_survivability.csv \
  --output-dir results/formation_survivability_v1/plots \
  --z -3.0 \
  --plot-scenes-from-summary
```

## 输出文件

```text
suite_survivability.csv
suite_survivability.json
plots/
```

每个场景目录包含：

```text
obstacle_trajectory.csv
scene_config_for_plot.yaml
survival_samples.csv
```

## 指标含义

```text
S_formation
  虚拟编队中心的平均存活时间，越大越简单。

D_formation
  0-10 难度分数，越大越困难。

failed_sample_ratio
  被动态障碍安全侵入的采样中心比例。
```

当前 V1 阈值：

```text
Easy:   D_formation < 1.0
Medium: 1.0 <= D_formation < 3.5
Hard:   D_formation >= 3.5
```
