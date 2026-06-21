# DEM方向敏感扩展实验 v5

本版本用于继续做三个实验：

1. **多起终点实验**：检验结论是否依赖单一起终点。
2. **起终点互换实验**：检验方向代价的正反向非对称性。
3. **邻域敏感性实验**：比较 4/8/16/32 邻域搜索对路径规划结果的影响。

## 1. 安装依赖

```powershell
python -m pip install numpy scipy matplotlib tifffile
```

如果你能安装 rasterio，也可以安装：

```powershell
python -m pip install rasterio
```

没有 rasterio 也能跑，脚本会用 tifffile 读取 DEM，并使用 `--cell-size 30`。

## 2. 用你的 1000×1000 DEM 跑完整实验

```powershell
python run_sensitivity_suite.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --cell-size 30 --out outputs_direction_sensitivity
```

也可以直接运行：

```powershell
.\run_direction_sensitivity_my_dem.bat
```

## 3. 如果运行很慢

1000×1000 DEM + 20组起终点 + 4/8/16/32 邻域会比较慢。建议先快速测试：

```powershell
python run_sensitivity_suite.py --dem "D:\VSCode Program\通行能力分析_研\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 500 --cell-size 30 --n-pairs 8 --neighbors 4,8 --out outputs_quick_test
```

确认逻辑和图都正常后，再跑完整 1000×1000。

## 4. 只跑某一类实验

只跑多起终点：

```powershell
python run_sensitivity_suite.py --dem "你的DEM.tif" --cell-size 30 --skip-reverse --skip-neighborhood --out outputs_multistart
```

只跑起终点互换：

```powershell
python run_sensitivity_suite.py --dem "你的DEM.tif" --cell-size 30 --skip-multistart --skip-neighborhood --out outputs_reverse
```

只跑邻域敏感性：

```powershell
python run_sensitivity_suite.py --dem "你的DEM.tif" --cell-size 30 --skip-multistart --skip-reverse --neighbors 4,8,16,32 --out outputs_neighbors
```

## 5. 输出结构

输出目录中有：

```text
00_实验说明_请先看.md
00_运行参数.txt
01_多起终点实验/
02_起终点互换实验/
03_邻域敏感性实验/
```

每个子文件夹中包含 CSV 指标和 PNG 图片。

## 6. 三个实验的理论作用

### 多起终点实验

单一路径可能刚好位于明显沟谷或缓坡通道，导致 B0、B1、Ours 路径差异不明显。多起终点实验用于检验模型差异是否具有空间稳定性。

### 起终点互换实验

方向代价模型中，上坡、缓下坡、陡下坡代价不对称。因此从 A 到 B 与从 B 到 A 的代价和最优路径理论上可能不同。B0 标量模型理论上最接近对称。

### 邻域敏感性实验

方向敏感代价场虽然可以计算多个方向，但 A* 只能沿搜索图允许的方向移动。4/8/16/32 邻域实验用于检验路径结果是否过度依赖邻域设置。

