# V11 设计与开发说明：硬约束与统一评价体系

## 1. 本次升级目标

V11 解决 V10 进入论文实验体系前的两个核心问题：

1. 不同模型的规划代价尺度不同，不能直接比较 `J_plan` 大小。
2. 软约束路径只能说明风险压力大小，不能回答是否存在满足车辆能力边界的可行路径。

因此 V11 新增：

- 统一评价（Unified Evaluation）：所有模型规划出的路径都用 V10 空间附着车辆能力模型重新评价。
- 硬约束可达性（Hard Constraint Reachability）：在 A* 搜索中直接加入 `rho_max <= rho_lim` 的方向边约束。
- 指标命名修正：所有 `J` 改为 `J_plan`，明确为路径相对累计代价，不是焦耳。

## 2. 模型标识

| 标识 | 方法 | 论文作用 |
|---|---|---|
| B0 | 静态坡度代价模型 | 传统标量坡度成本面基线 |
| B1 | 方向纵坡代价模型 | 证明只看纵坡会低估横坡风险 |
| V9 | 常数附着车辆能力利用率模型 | 车辆能力约束基线 |
| V10-Free | 空间附着车辆能力利用率软约束模型 | 主模型软约束版本 |
| V10-Hard | 空间附着车辆能力利用率硬约束模型 | 判断给定能力阈值下是否存在可行路径 |

## 3. 核心公式

规划代价：

```math
J_m^{plan}(P_m)=\sum_{e_k\in P_m} c_m(e_k)l(e_k)
```

车辆能力利用率：

```math
\rho_{max}(i,d)=\max(\rho_{up},\rho_{down},\rho_{roll},\rho_{slide})
```

软约束代价：

```math
c_{V10}(i,d)=1+\rho_{max}(i,d)
```

硬约束代价：

```math
c(i,d)=
\begin{cases}
1+\rho_{max}(i,d), & \rho_{max}(i,d)\leq\rho_{lim}\\
+\infty, & \rho_{max}(i,d)>\rho_{lim}
\end{cases}
```

统一评价指标：

```math
E_{V10}(P_m)=\{\bar\rho_{max}, P(\rho_{max}>1), P(\rho_{slide}>0.7), P(\rho_{down}>0.7), L, D\}
```

## 4. 新增和修改的文件

- `path_planning.py`
  - 新增 `hard_rho_limit` 参数。
  - 当 `c_unit = 1 + rho_max` 且 `rho_max > hard_rho_limit` 时，A* 直接跳过该方向边。

- `run_v11_experiments.py`
  - 新增 V11 主实验脚本。
  - 自动输出模型说明、统一评价逐路径表、按模型汇总表、硬约束可达性逐路径表、按阈值汇总表。

- `run_v10_experiments.py` / `run_single_experiment.py`
  - 修正误导性列名：`路径总代价_J` 改为 `路径相对累计代价_Jplan`。

## 5. 快速运行

合成 DEM 测试：

```bash
python run_v11_experiments.py --synthetic --synthetic-size 180 --roi-size 100 --n-rois 1 --n-pairs-per-scenario 1 --out outputs_v11_synthetic --hard-limits 1.0,1.2,1.5
```

真实 DEM：

```bash
python run_v11_experiments.py --dem "你的DEM.tif" --max-pixels 1000 --out outputs_v11_real --hard-limits 1.0,1.2,1.5
```

如有土地覆盖、土壤和水系栅格：

```bash
python run_v11_experiments.py --dem "你的DEM.tif" --landcover "landcover.tif" --soil "soil.tif" --water "water.tif" --out outputs_v11_real_full
```

## 6. 结果解释边界

- `J_plan` 是相对累计代价，不是焦耳。
- `P_rho_max_gt_1p0` 表示能力超限路径比例，不使用“致命”“死亡”等过度表述。
- V10 的空间化附着参数是 GIS 参数化，不等同于完整土力学实测模型。
- OSM 后续只作为弱监督证据，不是通行能力真值。
