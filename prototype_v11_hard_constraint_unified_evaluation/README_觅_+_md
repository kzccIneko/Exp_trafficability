# prototype_v10_spatial_adhesion_trafficability

## 这版做什么

v10 把 v9 中的常数附着能力参数改为空间栅格：

- `mu_s(x,y)`：侧滑附着能力；
- `mu_b(x,y)`：下坡制动附着能力。

它们由土地覆盖基础附着表、DEM 派生 TWI、水系附近湿润影响、可选土壤软弱倾向共同生成。这样，同一个横坡在草地、裸地、湿地区、水系附近会得到不同的侧滑附着利用率和下坡制动利用率。

## 主模型人话解释

每一小段路都计算四个数：

1. 上坡牵引利用率：这段上坡用了多少比例的爬坡能力；
2. 下坡制动利用率：这段下坡用了多少比例的制动附着能力；
3. 侧翻稳定性利用率：这段横坡用了多少比例的抗侧翻能力；
4. 侧滑附着利用率：这段横坡用了多少比例的横向附着能力。

然后取最大值：

```math
rho_max = max(rho_up, rho_down, rho_roll, rho_slide)
```

单位方向代价为：

```math
c(i,d)=1+rho_max(i,d)
```

路径越长，代价越大；路径越多路段接近车辆能力限制，代价也越大。

## 运行方式

先安装依赖：

```bat
python -m pip install -r requirements.txt
```

先跑模拟 DEM：

```bat
run_v10_synthetic_test.bat
```

再跑真实 DEM 快速版：

```bat
run_v10_my_dem_quick.bat
```

再跑完整真实 DEM：

```bat
run_v10_my_dem_full.bat
```

真实 DEM 路径仍写在：

```text
my_dem_path.txt
```

## 可选数据

如果有 12.5m DEM，可先把路径写入 `my_dem_12p5m_path.txt`。当前主流程不强制使用它，后续跨尺度路径一致性检验会读取。


可以通过命令行加入：

```bat
--landcover your_landcover.tif --soil your_soil.tif --water your_water_mask.tif
```

如果没有这些数据，程序会用合成土地覆盖、DEM 派生 TWI 和潜在湿润区完成测试。这样代码包能先跑通。

## 看哪些输出

模拟测试输出在：

```text
outputs_v10_synthetic_test/
```

重点看：

```text
00_公式护照_逐项来源.csv
02_PTF_Lite地表附着参数空间化/
03_方向车辆能力利用率图/
04_路径约束利用率统计实验/
05_空间化参数消融实验/
06_湿度情景实验/
```

## 注意

- TWI 是地形湿润倾向，不是实测土壤含水率。
- 默认附着表是工程情景参数，不是实测摩擦系数。
- OSM 道路只能作为弱监督证据，不是通行能力真值。
- 12.5m 与 30m DEM 的跨尺度复核是可选可靠性分析，不作为主流程。
