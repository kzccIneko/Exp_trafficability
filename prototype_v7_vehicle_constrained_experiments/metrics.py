"""metrics_v3.py - 实验指标与验证工具（审阅修订版）。"""

from __future__ import annotations

import numpy as np


def anisotropy_index(cost_unit_field: np.ndarray, method: str = "percentile", eps: float = 1e-8) -> np.ndarray:
    """
    各向异性指数。

    method='maxmin'：AI=(max-min)/(max+min)，容易在 min≈0 时饱和。
    method='percentile'：AI=(q90-q10)/(q90+q10+eps)，更稳健，建议论文使用。
    """
    C = np.maximum(cost_unit_field, 0.0)
    if method == "maxmin":
        hi = np.max(C, axis=2)
        lo = np.min(C, axis=2)
    elif method == "percentile":
        hi = np.percentile(C, 90, axis=2)
        lo = np.percentile(C, 10, axis=2)
    else:
        raise ValueError("method must be 'maxmin' or 'percentile'")
    return np.clip((hi - lo) / (hi + lo + eps), 0.0, 1.0)


def summarize_array(name: str, arr: np.ndarray) -> dict[str, float | str]:
    """返回数组统计摘要。"""
    finite = arr[np.isfinite(arr)]
    return {
        "name": name,
        "min": float(np.min(finite)),
        "q10": float(np.percentile(finite, 10)),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "q90": float(np.percentile(finite, 90)),
        "max": float(np.max(finite)),
    }


def auc_lower_cost_positive(positive_costs: np.ndarray, negative_costs: np.ndarray) -> float:
    """
    AUC = P(C_positive < C_negative) + 0.5*P(tie)。
    对通行代价验证来说，positive 通常为 OSM 道路点，negative 为匹配非道路点。
    """
    pos = np.asarray(positive_costs, dtype=float)
    neg = np.asarray(negative_costs, dtype=float)
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    lt = 0
    eq = 0
    for x in pos:
        lt += np.sum(x < neg)
        eq += np.sum(x == neg)
    return float((lt + 0.5 * eq) / (len(pos) * len(neg)))


def paired_delta_stats(positive_costs: np.ndarray, negative_costs: np.ndarray) -> dict[str, float | int]:
    """配对差值 ΔC = C_negative - C_positive，正值越大越符合“道路低代价”。"""
    n = min(len(positive_costs), len(negative_costs))
    if n == 0:
        return {"n": 0, "mean_delta": float("nan"), "median_delta": float("nan"), "ci_low": float("nan"), "ci_high": float("nan")}
    delta = np.asarray(negative_costs[:n], dtype=float) - np.asarray(positive_costs[:n], dtype=float)
    return {
        "n": int(n),
        "mean_delta": float(np.mean(delta)),
        "median_delta": float(np.median(delta)),
        "ci_low": float(np.percentile(delta, 2.5)),
        "ci_high": float(np.percentile(delta, 97.5)),
    }
