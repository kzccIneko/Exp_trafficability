# prototype_v7_vehicle_constrained_experiments

v7：参数化车辆约束的方向越野通行能力实验包。

## 这版与 v6.1 的区别

v6.1 的经验阻碍度函数 `R=1-exp[-(alpha/alpha0)^2]` 只保留为 baseline。v7 主模型改为车辆能力利用率：

```math
rho = terrain demand / vehicle capacity
```

核心指标包括：

- `rho_up`：上坡牵引能力利用率
- `rho_down`：下坡制动能力利用率
- `rho_roll`：横坡侧翻能力利用率
- `rho_slide`：横向侧滑能力利用率
- `rho_break`：通过角/宏观曲率能力利用率

## 运行顺序

1. 先双击：`run_v7_synthetic_test.bat`
2. 再双击：`run_v7_my_dem_quick.bat`
3. 最后双击：`run_v7_my_dem_full.bat`

你的真实 DEM 路径写在：`my_dem_path.txt`。

## 依赖安装

```bat
python -m pip install -r requirements.txt
```

若真实 GeoTIFF 读取失败，可先使用 synthetic 测试；读取 DEM 至少需要 `tifffile`，精确地理元数据建议安装 `rasterio`。

## 重点输出

运行后看：

- `00_实验总说明_请先看.md`
- `01_公开车辆参数_Jeep_Wrangler_Rubicon/01_车辆参数来源表.csv`
- `02_公式推导与参数来源/01_公式推导与自查说明.md`
- `03_物理一致性合成实验/02_单约束能力利用率曲线.png`
- `05_路径风险暴露实验/00_全部路径车辆约束暴露指标.csv`
- `06_车辆参数敏感性实验/02_车辆参数敏感性_汇总指标.csv`
