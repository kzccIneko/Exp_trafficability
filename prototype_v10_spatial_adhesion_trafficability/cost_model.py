"""
cost_model_v3.py
方向敏感越野通行代价模型（审阅修订版）

关键修复：
1. DEM 梯度 p,q 是坡度正切 tan(alpha)，不是弧度角；阻碍函数中统一使用 arctan 转为角度。
2. 代价场输出为“单位距离代价” c_unit = -ln(P)，路径规划时再乘以真实边长，避免距离重复计算。
3. P = 1-R 使用 clip 到 [eps, 1]，保证 R=0 时 c_unit=0，不产生负代价。
4. 保留 B0/B1/Ours 三模型，以支持消融实验。
"""

from __future__ import annotations

import numpy as np


def angle_difference(a: np.ndarray | float, b: np.ndarray | float) -> np.ndarray | float:
    """返回 [-pi, pi] 范围内的周期角差。"""
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def nearest_direction_index(theta: float, directions: np.ndarray) -> int:
    """按周期角差查找最接近的方向索引，修复 0/2pi 边界问题。"""
    return int(np.argmin(np.abs(angle_difference(directions, theta % (2 * np.pi)))))


def calculate_gradient_tan(dem: np.ndarray, cell_size: float) -> tuple[np.ndarray, np.ndarray]:
    """
    计算 DEM 梯度分量。输出 p,q 为 dz/dx, dz/dy，即坡度角的正切值。
    注意：它们不是坡度角本身，进入角度阈值模型前必须 arctan。
    """
    q, p = np.gradient(dem.astype(float), cell_size)
    return p, q


def calculate_slope_angle(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """由梯度正切分量计算坡度角 alpha = arctan(sqrt(p^2+q^2))。"""
    return np.arctan(np.hypot(p, q))


def longitudinal_cross_slope_tan(p: np.ndarray, q: np.ndarray, theta: float) -> tuple[np.ndarray, np.ndarray]:
    """
    将 DEM 梯度投影到行驶方向，得到纵坡/横坡的正切值。

    g_parallel_tan = p*cos(theta) + q*sin(theta)
    g_cross_tan    = -p*sin(theta) + q*cos(theta)

    返回值是 tan(alpha_parallel) 与 tan(alpha_cross)，不是角度。
    """
    g_parallel = p * np.cos(theta) + q * np.sin(theta)
    g_cross = -p * np.sin(theta) + q * np.cos(theta)
    return g_parallel, g_cross


def longitudinal_impedance_from_tan(
    g_parallel_tan: np.ndarray,
    alpha_u: float = 15.0,
    alpha_m: float = 5.0,
    alpha_d: float = 15.0,
) -> np.ndarray:
    """
    纵坡阻碍度 R_parallel。

    参数 alpha_u/alpha_m/alpha_d 单位为“度”，含义是阻碍增长尺度，
    不是车辆极限坡度。内部先将梯度正切转换为坡度角：alpha = arctan(g_parallel_tan)。
    """
    a = np.arctan(g_parallel_tan)
    au = np.radians(alpha_u)
    am = np.radians(alpha_m)
    ad = np.radians(alpha_d)

    R = np.zeros_like(a, dtype=float)
    uphill = a > 0
    R[uphill] = 1.0 - np.exp(-((a[uphill] / au) ** 2))

    steep_down = a < -am
    excess = np.abs(a[steep_down]) - am
    R[steep_down] = 1.0 - np.exp(-((excess / ad) ** 2))
    return np.clip(R, 0.0, 1.0)


def cross_slope_impedance_from_tan(g_cross_tan: np.ndarray, alpha_r: float = 15.0) -> np.ndarray:
    """
    横坡阻碍度 R_cross。

    横坡正负方向约束压力近似对称，因此使用 abs(arctan(g_cross_tan))。
    alpha_r 单位为“度”，表示横坡约束压力增长尺度。
    """
    a = np.arctan(np.abs(g_cross_tan))
    ar = np.radians(alpha_r)
    R = 1.0 - np.exp(-((a / ar) ** 2))
    return np.clip(R, 0.0, 1.0)


def combine_noncompensatory(*impedances: np.ndarray) -> np.ndarray:
    """
    非补偿合成：R = 1 - Π(1-R_k)。
    只要任一因子阻碍很高，综合阻碍就会很高。
    """
    if not impedances:
        raise ValueError("至少需要一个阻碍度数组")
    survival = np.ones_like(impedances[0], dtype=float)
    for R in impedances:
        survival *= (1.0 - np.clip(R, 0.0, 1.0))
    return np.clip(1.0 - survival, 0.0, 1.0)


def unit_cost_from_impedance(R: np.ndarray, eps: float = 1e-6, base_cost: float = 1.0) -> np.ndarray:
    """
    阻碍度 R -> 单位距离代价 c_unit = base_cost - ln(P)，P=1-R。

    v5 曾使用纯约束压力项 -ln(P)，平坦区可能为 0，导致路径总代价不再包含
    基础距离，也会让 A* 启发式退化为 Dijkstra。v6 改为 base_cost - ln(P)：
    - 当 R=0 时，c_unit=1，路径代价等价于几何距离；
    - 当 R>0 时，单位距离代价大于 1，表示利用率加权路径代价；
    - 这里不乘 cell_size；路径规划时按边长乘以 meters。
    """
    P = np.clip(1.0 - np.clip(R, 0.0, 1.0), eps, 1.0)
    return float(base_cost) - np.log(P)


def scalar_slope_impedance(p: np.ndarray, q: np.ndarray, alpha_u: float = 15.0) -> np.ndarray:
    """B0：传统标量坡度阻碍度，每个栅格不区分方向。"""
    slope_angle = calculate_slope_angle(p, q)
    au = np.radians(alpha_u)
    return np.clip(1.0 - np.exp(-((slope_angle / au) ** 2)), 0.0, 1.0)


def compute_b0_unit_cost_field(
    p: np.ndarray,
    q: np.ndarray,
    directions: np.ndarray,
    alpha_u: float = 15.0,
) -> tuple[np.ndarray, np.ndarray]:
    """返回 B0 阻碍度场与单位距离代价场，方向维复制为 N 个方向。"""
    R0 = scalar_slope_impedance(p, q, alpha_u=alpha_u)
    C0 = unit_cost_from_impedance(R0)
    Rf = np.repeat(R0[:, :, None], len(directions), axis=2)
    Cf = np.repeat(C0[:, :, None], len(directions), axis=2)
    return Rf, Cf


def compute_b1_unit_cost_field(
    p: np.ndarray,
    q: np.ndarray,
    directions: np.ndarray,
    alpha_u: float = 15.0,
    alpha_m: float = 5.0,
    alpha_d: float = 15.0,
) -> tuple[np.ndarray, np.ndarray]:
    """B1：仅纵坡方向模型。"""
    rows, cols = p.shape
    Rf = np.zeros((rows, cols, len(directions)), dtype=float)
    Cf = np.zeros_like(Rf)
    for i, theta in enumerate(directions):
        gp, _ = longitudinal_cross_slope_tan(p, q, theta)
        R = longitudinal_impedance_from_tan(gp, alpha_u=alpha_u, alpha_m=alpha_m, alpha_d=alpha_d)
        Rf[:, :, i] = R
        Cf[:, :, i] = unit_cost_from_impedance(R)
    return Rf, Cf


def compute_ours_unit_cost_field(
    p: np.ndarray,
    q: np.ndarray,
    directions: np.ndarray,
    alpha_u: float = 15.0,
    alpha_m: float = 5.0,
    alpha_d: float = 15.0,
    alpha_r: float = 15.0,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """Ours：纵坡 + 横坡非补偿方向模型。"""
    rows, cols = p.shape
    Rf = np.zeros((rows, cols, len(directions)), dtype=float)
    Cf = np.zeros_like(Rf)
    Rpar = np.zeros_like(Rf)
    Rcross = np.zeros_like(Rf)
    for i, theta in enumerate(directions):
        gp, gc = longitudinal_cross_slope_tan(p, q, theta)
        Rp = longitudinal_impedance_from_tan(gp, alpha_u=alpha_u, alpha_m=alpha_m, alpha_d=alpha_d)
        Rc = cross_slope_impedance_from_tan(gc, alpha_r=alpha_r)
        R = combine_noncompensatory(Rp, Rc)
        Rpar[:, :, i] = Rp
        Rcross[:, :, i] = Rc
        Rf[:, :, i] = R
        Cf[:, :, i] = unit_cost_from_impedance(R)
    return Rf, Cf, {"R_parallel": Rpar, "R_cross": Rcross}


def compute_b2_unit_cost_field(
    p: np.ndarray,
    q: np.ndarray,
    directions: np.ndarray,
    alpha_r: float = 15.0,
) -> tuple[np.ndarray, np.ndarray]:
    """B2：仅横坡方向模型，用于证明横坡约束压力不是多余项。"""
    rows, cols = p.shape
    Rf = np.zeros((rows, cols, len(directions)), dtype=float)
    Cf = np.zeros_like(Rf)
    for i, theta in enumerate(directions):
        _, gc = longitudinal_cross_slope_tan(p, q, theta)
        R = cross_slope_impedance_from_tan(gc, alpha_r=alpha_r)
        Rf[:, :, i] = R
        Cf[:, :, i] = unit_cost_from_impedance(R)
    return Rf, Cf
