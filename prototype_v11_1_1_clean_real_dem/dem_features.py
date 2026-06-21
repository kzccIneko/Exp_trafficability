"""
dem_features.py

DEM 基础地形量：梯度、坡度角、纵坡、横坡。
本模块只做几何投影，不直接判断通行好坏。
"""
from __future__ import annotations
import numpy as np
from cost_model import calculate_gradient_tan, calculate_slope_angle, longitudinal_cross_slope_tan


def slope_deg_from_dem(dem: np.ndarray, cell_size: float) -> np.ndarray:
    p, q = calculate_gradient_tan(dem, cell_size)
    return np.degrees(calculate_slope_angle(p, q))


def directional_slope_angles(p: np.ndarray, q: np.ndarray, theta: float) -> tuple[np.ndarray, np.ndarray]:
    """返回某一通行方向下的纵坡角、横坡角，单位为弧度。"""
    gp, gc = longitudinal_cross_slope_tan(p, q, theta)
    alpha_parallel = np.arctan(gp)
    alpha_cross = np.arctan(np.abs(gc))
    return alpha_parallel, alpha_cross
