"""
spatial_surface_params.py

v10 PTF-Lite：基于 GIS 的地表附着参数转换模型。
把常数 mu_slide/mu_brake 改为空间栅格 mu_s(x,y), mu_b(x,y)。

它不是完整土壤传递函数，也不是实测摩擦系数反演；当前作用是：
- 用土地覆盖给出基础附着能力；
- 用 TWI、水系距离、土壤软弱倾向降低附着能力；
- 让同一坡度在不同地表和湿润条件下产生不同车辆能力利用率。
"""
from __future__ import annotations
from pathlib import Path
import numpy as np
from hydrology_twi import compute_twi, normalize_percentile
from landcover_lookup import read_lookup_csv, map_landcover_to_mu
from water_influence import water_influence_from_mask, potential_wet_mask_from_twi
from soil_wetness_index import soil_softness_from_classes


def generate_synthetic_landcover(dem: np.ndarray, twi_n: np.ndarray, seed: int = 42) -> np.ndarray:
    """生成可测试的合成土地覆盖：10林地、20草地、30裸地、40潜在湿润/水体、50建成区。"""
    rng = np.random.RandomState(seed)
    rows, cols = dem.shape
    yy, xx = np.mgrid[0:rows, 0:cols]
    elev_n = normalize_percentile(dem)
    lc = np.full((rows, cols), 20, dtype=int)  # grass
    lc[elev_n > 0.70] = 30  # barren/highland
    lc[(elev_n < 0.45) & (twi_n < 0.65)] = 10  # forest-ish
    lc[twi_n > 0.92] = 40  # wet/water-like
    # small built-up patch as hard barrier demonstration
    if rows > 80 and cols > 80:
        rr = slice(rows//2-8, rows//2+8); cc = slice(cols//2-8, cols//2+8)
        if rng.rand() < 0.5:
            lc[rr, cc] = 50
    return lc


def compute_spatial_surface_params(
    dem: np.ndarray,
    cell_size: float,
    lookup_csv: str | Path,
    *,
    landcover: np.ndarray | None = None,
    soil: np.ndarray | None = None,
    water_mask: np.ndarray | None = None,
    scenario: str = 'normal',
    gamma_slide: float = 0.5,
    gamma_brake: float = 0.5,
    water_decay_m: float = 90.0,
    weights: tuple[float,float,float] = (1.0, 1.0, 1.0),
    seed: int = 42,
) -> dict[str, np.ndarray | dict | str]:
    twi, twi_n, flow_acc = compute_twi(dem, cell_size, fill_sinks=True)
    if landcover is None:
        landcover = generate_synthetic_landcover(dem, twi_n, seed=seed)
        lc_source = 'synthetic_landcover_from_dem_twi'
    else:
        landcover = landcover.astype(int)
        lc_source = 'user_landcover'
    lookup = read_lookup_csv(lookup_csv)
    mu_s_base, mu_b_base, hard_from_lc = map_landcover_to_mu(landcover, lookup, scenario=scenario)

    if water_mask is None:
        water_mask = (landcover == 40)
        if not np.any(water_mask):
            water_mask = potential_wet_mask_from_twi(twi_n, quantile=0.92)
        water_source = 'landcover_or_high_twi_proxy'
    else:
        water_mask = water_mask.astype(bool)
        water_source = 'user_water_mask'
    W_water = water_influence_from_mask(water_mask, cell_size, decay_m=water_decay_m)

    S_soil = soil_softness_from_classes(soil)
    soil_source = 'none_zero' if S_soil is None else 'user_soil_or_default_mapping'
    if S_soil is None:
        S_soil = np.zeros_like(dem, dtype=float)

    components = []
    comp_weights = []
    for arr, w in [(twi_n, weights[0]), (W_water, weights[1]), (S_soil, weights[2])]:
        if arr is not None and np.isfinite(arr).any() and w > 0:
            components.append(np.clip(arr.astype(float), 0.0, 1.0))
            comp_weights.append(float(w))
    if not components:
        W = np.zeros_like(dem, dtype=float)
    else:
        total_w = sum(comp_weights)
        W = sum(w * a for w, a in zip(comp_weights, components)) / max(total_w, 1e-9)
        W = np.clip(W, 0.0, 1.0)
    if scenario == 'dry':
        scenario_factor = 0.65
    elif scenario == 'wet':
        scenario_factor = 1.25
    elif scenario == 'post_rain':
        scenario_factor = 1.45
    else:
        scenario_factor = 1.0
    W_eff = np.clip(W * scenario_factor, 0.0, 1.0)

    mu_s = mu_s_base * (1.0 - gamma_slide * W_eff)
    mu_b = mu_b_base * (1.0 - gamma_brake * W_eff)
    mu_s = np.clip(mu_s, 0.08, 0.90)
    mu_b = np.clip(mu_b, 0.08, 0.90)

    hard_barrier = hard_from_lc | (landcover == 40)
    return {
        'mu_slide': mu_s,
        'mu_brake': mu_b,
        'mu_slide_base': mu_s_base,
        'mu_brake_base': mu_b_base,
        'wetness_index': W,
        'wetness_effective': W_eff,
        'twi': twi,
        'twi_norm': twi_n,
        'flow_accumulation': flow_acc,
        'water_influence': W_water,
        'water_mask': water_mask,
        'soil_softness': S_soil,
        'landcover': landcover,
        'hard_barrier': hard_barrier,
        'metadata': {
            'scenario': scenario,
            'gamma_slide': gamma_slide,
            'gamma_brake': gamma_brake,
            'water_decay_m': water_decay_m,
            'landcover_source': lc_source,
            'water_source': water_source,
            'soil_source': soil_source,
            'formula': 'mu = clip(mu_base * (1 - gamma * W_eff), min, max)',
        }
    }
