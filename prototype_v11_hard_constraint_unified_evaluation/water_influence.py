"""水系/潜在湿润区影响。"""
from __future__ import annotations
import numpy as np
from scipy.ndimage import distance_transform_edt


def water_influence_from_mask(water_mask: np.ndarray, cell_size: float, decay_m: float = 90.0) -> np.ndarray:
    if water_mask is None or not np.any(water_mask):
        return np.zeros_like(water_mask, dtype=float) if water_mask is not None else None
    dist_cells = distance_transform_edt(~water_mask)
    dist_m = dist_cells * cell_size
    return np.exp(-dist_m / max(decay_m, 1e-9))


def potential_wet_mask_from_twi(twi_n: np.ndarray, quantile: float = 0.90) -> np.ndarray:
    finite = twi_n[np.isfinite(twi_n)]
    if finite.size == 0:
        return np.zeros_like(twi_n, dtype=bool)
    th = np.quantile(finite, quantile)
    return twi_n >= th
