# 方向敏感越野通行能力实验 v4.2 使用说明

## 1. 直接运行你的 1000×1000 DEM

在 PowerShell 或 VSCode 终端中进入本文件夹后运行：

```powershell
python run_experiment.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --out outputs_real_1000
```

如果只是先检查能否出图、不想等路径规划：

```powershell
python run_experiment.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --no-path --out outputs_real_1000_cost_only
```

## 2. 直接双击运行

本包已经放了 `run_my_dem_1000.bat`。把代码包解压后，双击这个 bat 文件即可运行你的 DEM 路径。

## 3. 安装依赖

```powershell
python -m pip install numpy scipy matplotlib tifffile
python -m pip install rasterio  # 可选但推荐；失败也可先运行
```

## 4. 输出文件

运行后会在输出目录中生成：

- `01_DEM与纵坡横坡分解.png`
- `02_三模型最小代价场对比.png`
- `03_各向异性指数对比.png`
- `04_最优代价与最优方向场.png`
- `05_三模型路径规划对比.png`
- `06_局部代价玫瑰图.png`
- `01_场统计指标.csv`
- `02_路径规划指标.csv`
- `03_指标公式与阅读说明.md`
- `04_运行参数与DEM信息.txt`

## 5. 这套实验回答什么问题

它不是只为了证明“横坡重要”。它的核心问题是：越野通行能力建模是否应该从标量栅格代价升级为方向相关边代价。

- B0：传统标量坡度模型，不考虑方向。
- B1：仅纵坡方向模型，考虑车辆前进方向。
- Ours：纵坡+横坡方向模型，考虑车辆行驶方向和横向稳定风险。

## 6. 邻域搜索关系

当前路径规划固定 8 邻域。纵坡/横坡公式中的 θ 就是 8 邻域移动方向。后续可以做 4/8 邻域敏感性实验，但第一阶段建议固定 8 邻域，保证 B0、B1、Ours 公平比较。


## 7. 如果提示 No module named 'rasterio'

v4.2 已支持没有 rasterio 时用 tifffile 读取 DEM。请先运行：

```powershell
python -m pip install numpy scipy matplotlib tifffile
```

然后运行：

```powershell
python run_experiment.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --cell-size 30 --out outputs_real_1000
```

如果 rasterio 能安装，程序会自动读取 GeoTIFF 的坐标和分辨率；如果不能安装，程序会用 `--cell-size 30` 作为栅格大小继续跑。
