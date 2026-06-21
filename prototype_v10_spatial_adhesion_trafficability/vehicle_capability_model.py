"""
vehicle_capability_model.py

v10 车辆能力利用率方向代价模型。
新增点：下坡制动利用率和侧滑附着利用率使用空间栅格 mu_b(x,y), mu_s(x,y)。
"""
from __future__ import annotations
from dataclasses import asdict
import numpy as np
from vehicle_params import VehicleParams
from cost_model import longitudinal_cross_slope_tan

EPS = 1e-9
LIMIT_NAMES = np.array(['上坡牵引', '下坡制动', '侧翻稳定性', '侧滑附着'])


def _as_field(value, shape):
    if np.ndim(value) == 0:
        return np.full(shape, float(value), dtype=float)
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape:
        raise ValueError(f'参数栅格 shape {arr.shape} 与 DEM shape {shape} 不一致')
    return arr


def compute_vehicle_capability_fields(
    dem: np.ndarray,
    p: np.ndarray,
    q: np.ndarray,
    directions: np.ndarray,
    cell_size: float,
    vehicle: VehicleParams,
    *,
    mu_slide_field: np.ndarray | float | None = None,
    mu_brake_field: np.ndarray | float | None = None,
    hard_barrier: np.ndarray | None = None,
    hard_rho: float | None = None,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    rows, cols = dem.shape
    n_dir = len(directions)
    shape3 = (rows, cols, n_dir)
    alpha_par = np.zeros(shape3, dtype=float)
    alpha_cross = np.zeros(shape3, dtype=float)
    rho_up = np.zeros(shape3, dtype=float)
    rho_down = np.zeros(shape3, dtype=float)
    rho_roll = np.zeros(shape3, dtype=float)
    rho_slide = np.zeros(shape3, dtype=float)

    tan_grade_capacity = max(np.tan(vehicle.max_grade_angle_rad), EPS)
    roll_capacity = max(vehicle.track_width_m / (2.0 * vehicle.cg_height_m), EPS)
    mu_b = _as_field(vehicle.mu_brake if mu_brake_field is None else mu_brake_field, (rows, cols))
    mu_s = _as_field(vehicle.mu_slide if mu_slide_field is None else mu_slide_field, (rows, cols))
    mu_b = np.maximum(mu_b, 0.05)
    mu_s = np.maximum(mu_s, 0.05)

    for idx, theta in enumerate(directions):
        gp_tan, gc_tan = longitudinal_cross_slope_tan(p, q, theta)
        a_par = np.arctan(gp_tan)
        a_cross = np.arctan(np.abs(gc_tan))
        alpha_par[:, :, idx] = a_par
        alpha_cross[:, :, idx] = a_cross
        rho_up[:, :, idx] = np.tan(np.maximum(a_par, 0.0)) / (tan_grade_capacity + EPS)
        rho_down[:, :, idx] = np.tan(np.maximum(-a_par, 0.0)) / (mu_b + EPS)
        rho_roll[:, :, idx] = np.tan(a_cross) / (roll_capacity + EPS)
        rho_slide[:, :, idx] = np.tan(a_cross) / (mu_s + EPS)

    rho_max = np.maximum.reduce([rho_up, rho_down, rho_roll, rho_slide])
    dominant_limit = np.argmax(np.stack([rho_up, rho_down, rho_roll, rho_slide], axis=0), axis=0)
    C = 1.0 + rho_max
    if hard_rho is not None:
        C = np.where(rho_max > hard_rho, np.inf, C)
    if hard_barrier is not None:
        C = np.where(hard_barrier[:, :, None], np.inf, C)

    parts = {
        'alpha_parallel_rad': alpha_par,
        'alpha_cross_rad': alpha_cross,
        'rho_up': rho_up,
        'rho_down': rho_down,
        'rho_roll': rho_roll,
        'rho_slide': rho_slide,
        'rho_max': rho_max,
        'dominant_limit': dominant_limit,
        'mu_slide': mu_s,
        'mu_brake': mu_b,
        'hard_barrier': hard_barrier if hard_barrier is not None else np.zeros((rows, cols), dtype=bool),
        'vehicle_params': asdict(vehicle),
    }
    return C, parts


def compute_capability_curves(vehicle: VehicleParams, degrees: np.ndarray | None = None, mu_values=(0.35,0.5,0.7), cg_ratios=(0.40,0.45,0.50,0.55)) -> dict[str, np.ndarray]:
    if degrees is None:
        degrees = np.linspace(0, 45, 181)
    a = np.radians(degrees)
    base = {
        'degree': degrees,
        'rho_up_grade': np.tan(a) / (np.tan(vehicle.max_grade_angle_rad) + EPS),
    }
    for mu in mu_values:
        base[f'rho_down_mu_{mu:g}'] = np.tan(a) / (mu + EPS)
        base[f'rho_slide_mu_{mu:g}'] = np.tan(a) / (mu + EPS)
    for cg in cg_ratios:
        hc = vehicle.vehicle_height_m * cg
        cap = vehicle.track_width_m / (2.0 * hc)
        base[f'rho_roll_cgH_{cg:g}'] = np.tan(a) / (cap + EPS)
    return base
