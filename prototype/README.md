# 越野通行能力建模原型系统

核心创新：将通行能力从标量场升级为方向敏感的矢量场 C(x,y,θ)

## 依赖

```
numpy
scipy
matplotlib
```

可选：
```
osmnx    # OSM路网数据提取
rasterio # GeoTIFF DEM数据读取
```

安装：
```bash
pip install numpy scipy matplotlib
```

## 模块说明

| 模块 | 功能 |
|------|------|
| `terrain_analysis.py` | 地形分析：梯度、坡度、坡向、曲率、特征线提取 |
| `cost_model.py` | 方向敏感的代价计算：坡度、曲率、特征线、综合代价 |
| `path_planning.py` | 路径规划：修改版A*算法（支持方向敏感代价） |
| `osm_validation.py` | OSM验证：基于真实路网的客观验证 |
| `visualization.py` | 可视化：代价玫瑰图、热力图、路径图 |
| `utils.py` | 工具函数：角度处理、滤波、合成DEM生成 |
| `demo.py` | 演示脚本：完整分析流程 |

## 快速开始

```bash
cd prototype
python demo.py
```

## 核心公式

### 方向敏感的坡度代价
```
s_eff(θ) = s · cos(θ - φ)      # 有效坡度
C_s = exp(β · |s_eff|)          # 坡度代价
```

### 方向曲率（Euler公式）
```
κ_n(θ) = κ₁·cos²(θ-θ_p) + κ₂·sin²(θ-θ_p)
```

### 特征线约束代价
```
C_feature = 1 - γ·exp(-d²/2σ²)·|cos(θ-θ_feature)|
```

### 综合代价（乘法模型）
```
C_total = C_s · C_c · C_l · C_soil · C_w · C_ridge · C_valley
```
