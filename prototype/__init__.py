"""
越野通行能力建模原型系统

核心创新：将通行能力从标量场升级为方向敏感的矢量场 C(x,y,θ)

模块：
- terrain_analysis: 地形分析（梯度、曲率、特征线提取）
- cost_model: 方向敏感的代价计算
- path_planning: 基于修改版A*的路径规划
- osm_validation: 基于OSM路网的客观验证
- visualization: 可视化工具
- utils: 通用工具函数
"""

__version__ = "0.1.0"
