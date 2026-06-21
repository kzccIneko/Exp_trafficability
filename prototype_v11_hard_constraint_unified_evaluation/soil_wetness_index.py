"""土壤软弱倾向占位模块。没有土壤数据时返回 0。"""
from __future__ import annotations
import numpy as np


def soil_softness_from_classes(soil: np.ndarray | None, mapping: dict[int,float] | None = None) -> np.ndarray | None:
    if soil is None:
        return None
    out = np.zeros_like(soil, dtype=float)
    if mapping is None:
        # 默认：未知土壤均按 0.2 软弱倾向处理，NaN/0 为 0。
        out[np.isfinite(soil) & (soil != 0)] = 0.2
        return out
    for cid, val in mapping.items():
        out[soil == cid] = float(val)
    return np.clip(out, 0.0, 1.0)
