"""
vehicle_constraint_model.py

v7：参数化车辆约束的方向通行能力模型（VP-DTM）。

核心逻辑：不再直接用经验 R(alpha)，而先计算车辆能力利用率
rho = terrain demand / vehicle capacity。每个 rho 都对应明确约束：
- rho_up: 上坡牵引能力利用率
- rho_down: 下坡制动/附着能力利用率
- rho_roll: 横坡侧翻能力利用率
- rho_slide: 横坡侧滑能力利用率
- rho_break: 轴距/通过角对应的宏观几何通过能力利用率

注意：rho_break 在 30 m DEM 下只表示宏观起伏风险，不表示车轮尺度托底判断。
"""
from __future__ import annotations

import numpy as np
from dataclasses import asdict

from vehicle_params import VehicleParams
from cost_model import longitudinal_cross_slope_tan

EPS = 1e-9


def hessian_from_dem(dem: np.ndarray, cell_size: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """计算 DEM Hessian：zxx, zxy, zyy。单位约为 1/m。"""
    # np.gradient 返回顺序：对 axis0(row/y)、axis1(col/x) 的导数
    zy, zx = np.gradient(dem.astype(float), cell_size)
    zyy, zyx = np.gradient(zy, cell_size)
    zxy, zxx = np.gradient(zx, cell_size)
    zxy_mean = 0.5 * (zxy + zyx)
    return zxx, zxy_mean, zyy


def directional_curvature(zxx: np.ndarray, zxy: np.ndarray, zyy: np.ndarray, theta: float) -> tuple[np.ndarray, np.ndarray]:
    """沿行驶方向和横向方向的二阶曲率。"""
    ux, uy = np.cos(theta), np.sin(theta)
    nx, ny = -np.sin(theta), np.cos(theta)
    k_par = ux * ux * zxx + 2 * ux * uy * zxy + uy * uy * zyy
    k_cross = nx * nx * zxx + 2 * nx * ny * zxy + ny * ny * zyy
    return k_par, k_cross


def softplus_barrier(rho: np.ndarray, k: float = 8.0) -> np.ndarray:
    """
    phi(rho)=log(1+exp(k(rho-1)))/k。
    rho≈1 附近开始明显增长；rho>1 后近似线性增长。
    """
    return np.logaddexp(0.0, k * (rho - 1.0)) / k


def compute_vehicle_constraint_fields(
    dem: np.ndarray,
    p: np.ndarray,
    q: np.ndarray,
    directions: np.ndarray,
    cell_size: float,
    vehicle: VehicleParams,
    *,
    enable_break: bool = True,
    hard_rho: float | None = None,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """
    返回：
    - unit_cost_field: rows x cols x directions，单位距离代价 c(i,d)
    - parts: 所有 rho 与中间变量
    """
    rows, cols = dem.shape
    N = len(directions)
    shape = (rows, cols, N)
    C = np.ones(shape, dtype=float)
    alpha_par = np.zeros(shape, dtype=float)
    alpha_cross = np.zeros(shape, dtype=float)
    rho_up = np.zeros(shape, dtype=float)
    rho_down = np.zeros(shape, dtype=float)
    rho_roll = np.zeros(shape, dtype=float)
    rho_slide = np.zeros(shape, dtype=float)
    rho_break = np.zeros(shape, dtype=float)
    kpar_all = np.zeros(shape, dtype=float)

    if enable_break:
        zxx, zxy, zyy = hessian_from_dem(dem, cell_size)
    else:
        zxx = zxy = zyy = np.zeros_like(dem, dtype=float)

    fr = float(vehicle.rolling_resistance)
    mu_b = max(float(vehicle.mu_brake), EPS)
    mu_s = max(float(vehicle.mu_slide), EPS)
    tan_grade_capacity = np.tan(vehicle.max_grade_angle_rad)
    roll_capacity = max(vehicle.eta_roll * vehicle.track_width_m / (2.0 * vehicle.cg_height_m), EPS)
    slide_capacity = max(vehicle.eta_slide * mu_s, EPS)
    break_capacity = max(vehicle.eta_break * vehicle.breakover_angle_rad, EPS)

    for idx, theta in enumerate(directions):
        gp_tan, gc_tan = longitudinal_cross_slope_tan(p, q, theta)
        a_par = np.arctan(gp_tan)
        a_cross = np.arctan(np.abs(gc_tan))
        alpha_par[:, :, idx] = a_par
        alpha_cross[:, :, idx] = a_cross

        # 1) 上坡牵引能力利用率：使用最大可接受爬坡角作为 traction capacity proxy。
        uphill_tan = np.tan(np.maximum(a_par, 0.0))
        rho_up[:, :, idx] = (uphill_tan + fr) / (tan_grade_capacity + fr + EPS)
        rho_up[:, :, idx] = np.where(a_par > 0, rho_up[:, :, idx], 0.0)

        # 2) 下坡制动/附着能力利用率。
        beta = np.maximum(-a_par, 0.0)
        demand_down = np.maximum(np.sin(beta) - fr * np.cos(beta), 0.0)
        capacity_down = mu_b * np.cos(beta) + EPS
        rho_down[:, :, idx] = demand_down / capacity_down

        # 3) 横坡侧翻能力利用率。
        rho_roll[:, :, idx] = np.tan(a_cross) / (roll_capacity + EPS)

        # 4) 横坡侧滑能力利用率。
        rho_slide[:, :, idx] = np.tan(a_cross) / (slide_capacity + EPS)

        # 5) 通过角/轴距约束：宏观起伏代理。
        if enable_break:
            kp, _ = directional_curvature(zxx, zxy, zyy, theta)
            kpar_all[:, :, idx] = kp
            rho_break[:, :, idx] = np.abs(kp) * vehicle.wheelbase_m / (break_capacity + EPS)

        phi_up = softplus_barrier(rho_up[:, :, idx], vehicle.softplus_k)
        phi_down = softplus_barrier(rho_down[:, :, idx], vehicle.softplus_k)
        phi_roll = softplus_barrier(rho_roll[:, :, idx], vehicle.softplus_k)
        phi_slide = softplus_barrier(rho_slide[:, :, idx], vehicle.softplus_k)
        phi_break = softplus_barrier(rho_break[:, :, idx], vehicle.softplus_k) if enable_break else 0.0

        C[:, :, idx] = (
            1.0
            + vehicle.lambda_up * phi_up
            + vehicle.lambda_down * phi_down
            + vehicle.lambda_roll * phi_roll
            + vehicle.lambda_slide * phi_slide
            + vehicle.lambda_break * phi_break
        )

    if hard_rho is not None:
        rho_max = np.maximum.reduce([rho_up, rho_down, rho_roll, rho_slide, rho_break])
        C = np.where(rho_max > hard_rho, np.inf, C)

    parts = {
        "alpha_parallel_rad": alpha_par,
        "alpha_cross_rad": alpha_cross,
        "rho_up": rho_up,
        "rho_down": rho_down,
        "rho_roll": rho_roll,
        "rho_slide": rho_slide,
        "rho_break": rho_break,
        "kappa_parallel": kpar_all,
        "vehicle_params": asdict(vehicle),
    }
    return C, parts


def compute_constraint_curves(vehicle: VehicleParams, degrees: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """生成单约束物理一致性曲线，用于绘图和自查。"""
    if degrees is None:
        degrees = np.linspace(0, 45, 181)
    a = np.radians(degrees)
    fr = vehicle.rolling_resistance
    rho_up = (np.tan(a) + fr) / (np.tan(vehicle.max_grade_angle_rad) + fr + EPS)
    beta = a
    rho_down = np.maximum(np.sin(beta) - fr * np.cos(beta), 0.0) / (vehicle.mu_brake * np.cos(beta) + EPS)
    rho_roll = np.tan(a) / (vehicle.eta_roll * vehicle.track_width_m / (2 * vehicle.cg_height_m) + EPS)
    rho_slide = np.tan(a) / (vehicle.eta_slide * vehicle.mu_slide + EPS)
    return {
        "degree": degrees,
        "rho_up": rho_up,
        "rho_down": rho_down,
        "rho_roll": rho_roll,
        "rho_slide": rho_slide,
    }
