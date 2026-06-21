# DEM-only v6.1 方向敏感越野通行能力实验包

## 0. 本版修复了什么

v6.0 的 `.bat` 在部分 Windows CMD 中会报错，原因是：上一版 `.bat` 使用 Unix LF 换行，并且直接在命令行里写了中文 DEM 路径。Windows 批处理对这两点都比较敏感，可能把一行命令拆坏，出现类似：

```text
'D:\' 不是内部或外部命令
'axWarning' 不是内部或外部命令
```

v6.1 已修复：

1. 所有 `.bat` 改为 Windows CRLF 换行；
2. `.bat` 不再直接解析中文 DEM 路径；
3. 真实 DEM 路径写在 `my_dem_path.txt`，由 Python 按 UTF-8 读取；
4. `.bat` 自动检测 `py -3` 或 `python`；
5. 输出目录全部使用英文名，避免命令行编码问题。

## 1. 推荐运行顺序

先双击：

```text
run_v6_synthetic_test.bat
```

模拟 DEM 能跑通后，再双击快速真实 DEM：

```text
run_v6_my_dem_quick.bat
```

快速版没问题后再跑完整真实 DEM：

```text
run_v6_my_dem_full.bat
```

## 2. 修改真实 DEM 路径

真实 DEM 路径在：

```text
my_dem_path.txt
```

当前内容是：

```text
D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif
```

以后换数据，只改这个文件，不改 `.bat`。

## 3. 安装依赖

双击：

```text
install_dependencies.bat
```

或手动运行：

```powershell
python -m pip install numpy scipy matplotlib tifffile
```

可选安装 `rasterio`，用于读取 GeoTIFF 坐标元数据：

```powershell
python -m pip install rasterio
```

## 4. 当前阶段证明什么

本阶段不证明“路径时间最短”或“车辆能耗最低”，因为当前没有车辆速度、油耗、动力学和实车轨迹数据。本版证明的是：

> 方向敏感 DEM 通行代价模型比传统静态坡度模型更能表达越野通行中的方向异质性、横坡风险、上/下坡非对称性，并能在路径长度增加有限的条件下降低危险地形暴露。

因此主指标不是单纯路径长度，而是：

- 风险加权距离；
- 平均单位距离代价；
- 上坡/下坡/近似平坡数量和长度比例；
- 横坡角均值、90% 分位数、最大值；
- 横坡超过 8°/10°/12°/15° 的路径长度占比；
- 下坡超过 5°/10°/15° 的路径长度占比；
- 路径曲折度和绕行率。

## 5. 公式、参数和实验设计说明在哪里

请重点看：

```text
docs/实验公式推导_参数敏感性_汇报说明.md
```

每次运行后，输出目录里也会自动复制一份：

```text
outputs_v6*/00_公式推导_参数敏感性_汇报说明.md
```

里面详细解释了：

1. 为什么从 DEM 梯度推导纵坡/横坡；
2. 为什么从静态代价 `C(i)` 改为方向代价 `C(i,d)`；
3. 为什么纵坡要区分上坡、缓下坡、陡下坡；
4. 为什么横坡要单独进入模型；
5. 为什么采用非补偿合成；
6. 为什么单位距离代价定义为 `c = 1 - ln(P + eps)`；
7. 风险暴露指标如何定义；
8. 参数敏感性范围为什么这样设定。

## 6. 输出结构

```text
outputs_v6*/
00_实验说明_请先看.md
00_公式推导_参数敏感性_汇报说明.md
00_风险暴露指标解释.csv
00_运行参数.txt
01_ROI方向异质性筛选/
02_方向代价场验证/
03_情景路径与风险暴露实验/
04_正反向非对称实验/
05_邻域鲁棒性实验/
06_参数敏感性实验/
07_OSM弱监督验证接口说明/
```

每个 `pair_xxx` 文件夹里都有：

```text
profile_B0.csv
profile_B1.csv
profile_Ours.csv
01_pair路径风险暴露指标.csv
02_路径全局与局部放大.png
03_沿途高程_纵坡_横坡_单位代价剖面.png
04_危险暴露比例柱状图.png
```

## 7. OSM 弱监督验证

本版只保留接口说明，不纳入主运行流程。下一阶段需要提供 OSM 道路 `.shp`、`.geojson` 或 `.gpkg` 文件后，再在 `osm_validation.py` 上实现道路/非道路样本构建、AUC 和 Delta C 计算。
