"""
hydrology_twi.py

从 DEM 估计地形湿润倾向。这里实现的是轻量级 TWI 近似：
1. 可选填洼，减少局部假洼地对汇流累积的影响；
2. D8 最陡下降向汇流累积；
3. TWI = ln(A_s / tan(beta))；
4. 使用 5%—95% 分位归一化到 [0,1]。

注意：该 TWI 是 DEM 派生的地形湿润倾向，不是实测土壤含水率。
"""
from __future__ import annotations
import numpy as np
from scipy.ndimage import minimum_filter, distance_transform_edt
from cost_model import calculate_gradient_tan, calculate_slope_angle

EPS = 1e-9


def fill_nan_nearest(arr: np.ndarray) -> np.ndarray:
    a = arr.astype(float).copy()
    mask = ~np.isfinite(a)
    if not mask.any():
        return a
    _, ind = distance_transform_edt(mask, return_indices=True)
    a[mask] = a[tuple(ind)][mask]
    return a


def fill_sinks_simple(dem: np.ndarray, max_iter: int = 20, eps: float = 1e-4) -> np.ndarray:
    """
    简化填洼：将严格低于 8 邻域最低值的单元抬升到邻域最低值。
    不是完整水文 DEM 处理工具，但足以避免合成/小型 DEM 中的孤立假洼地。
    对严肃水文研究，应使用专业工具生成 hydrologically conditioned DEM。
    """
    z = fill_nan_nearest(dem)
    for _ in range(max_iter):
        mn = minimum_filter(z, size=3, mode='nearest')
        sinks = z < (mn - eps)
        if not sinks.any():
            break
        z[sinks] = mn[sinks] + eps
    return z


def d8_flow_accumulation(dem: np.ndarray, cell_size: float = 30.0) -> np.ndarray:
    """D8 汇流累积。输出以像元数近似贡献面积，后续乘 cell_size。"""
    z = fill_nan_nearest(dem)
    rows, cols = z.shape
    acc = np.ones_like(z, dtype=float)
    receiver = np.full((rows, cols, 2), -1, dtype=int)
    moves = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    dist = np.array([np.hypot(dr, dc) for dr, dc in moves], dtype=float)
    for r in range(rows):
        for c in range(cols):
            best_slope = 0.0
            best = (-1, -1)
            for k,(dr,dc) in enumerate(moves):
                rr, cc = r+dr, c+dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    s = (z[r,c] - z[rr,cc]) / (dist[k] * cell_size + EPS)
                    if s > best_slope:
                        best_slope = s; best = (rr,cc)
            receiver[r,c] = best
    order = np.argsort(z.ravel())[::-1]  # high to low
    for idx in order:
        r, c = divmod(int(idx), cols)
        rr, cc = receiver[r,c]
        if rr >= 0:
            acc[rr,cc] += acc[r,c]
    return acc


def normalize_percentile(arr: np.ndarray, q_low: float = 5, q_high: float = 95) -> np.ndarray:
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return np.zeros_like(arr, dtype=float)
    lo, hi = np.percentile(finite, [q_low, q_high])
    if abs(hi - lo) < EPS:
        return np.zeros_like(arr, dtype=float)
    out = (arr - lo) / (hi - lo + EPS)
    return np.clip(out, 0.0, 1.0)


def compute_twi(dem: np.ndarray, cell_size: float, *, fill_sinks: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    z = fill_sinks_simple(dem) if fill_sinks else fill_nan_nearest(dem)
    acc = d8_flow_accumulation(z, cell_size)
    p, q = calculate_gradient_tan(z, cell_size)
    beta = calculate_slope_angle(p, q)
    specific_area = acc * cell_size
    twi = np.log((specific_area + EPS) / (np.tan(beta) + 0.01))
    twi_n = normalize_percentile(twi)
    return twi, twi_n, acc
