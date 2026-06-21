"""路径沿线车辆约束剖面与暴露统计。"""
from __future__ import annotations

import numpy as np
from cost_model import nearest_direction_index


def path_vehicle_profile(path, dem, cost_field, parts, directions, cell_size):
    rows = []
    if not path:
        return rows
    cum = 0.0
    for idx, (r, c) in enumerate(path):
        if idx < len(path) - 1:
            r2, c2 = path[idx + 1]
        elif idx > 0:
            r2, c2 = r, c
            r, c = path[idx - 1]
        else:
            r2, c2 = r, c
        dr, dc = r2 - r, c2 - c
        edge_len = float(np.hypot(dr, dc) * cell_size) if (dr != 0 or dc != 0) else 0.0
        theta = float(np.arctan2(dr, dc) % (2*np.pi)) if edge_len > 0 else 0.0
        d_idx = nearest_direction_index(theta, directions)
        rr, cc = path[idx]
        row = {
            "step": idx,
            "row": rr,
            "col": cc,
            "cum_distance_m": cum,
            "edge_length_m": edge_len,
            "elevation_m": float(dem[rr, cc]),
            "direction_deg": float(np.degrees(theta)),
            "unit_cost": float(cost_field[rr, cc, d_idx]) if np.isfinite(cost_field[rr, cc, d_idx]) else float("nan"),
            "alpha_parallel_deg": float(np.degrees(parts["alpha_parallel_rad"][rr, cc, d_idx])),
            "alpha_cross_deg": float(np.degrees(parts["alpha_cross_rad"][rr, cc, d_idx])),
            "rho_up": float(parts["rho_up"][rr, cc, d_idx]),
            "rho_down": float(parts["rho_down"][rr, cc, d_idx]),
            "rho_roll": float(parts["rho_roll"][rr, cc, d_idx]),
            "rho_slide": float(parts["rho_slide"][rr, cc, d_idx]),
            "rho_break": float(parts["rho_break"][rr, cc, d_idx]),
        }
        row["rho_max"] = max(row["rho_up"], row["rho_down"], row["rho_roll"], row["rho_slide"], row["rho_break"])
        rows.append(row)
        cum += edge_len
    return rows


def _weighted_mean(vals, weights):
    vals = np.asarray(vals, dtype=float)
    weights = np.asarray(weights, dtype=float)
    mask = np.isfinite(vals) & np.isfinite(weights) & (weights >= 0)
    if not np.any(mask):
        return float("nan")
    return float(np.sum(vals[mask] * weights[mask]) / max(np.sum(weights[mask]), 1e-12))


def _exposure(rows, key, thr):
    if not rows:
        return float("nan")
    total = sum(max(float(r.get("edge_length_m", 0.0)), 0.0) for r in rows)
    if total <= 0:
        return float("nan")
    bad = sum(max(float(r.get("edge_length_m", 0.0)), 0.0) for r in rows if float(r.get(key, 0.0)) > thr)
    return float(bad / total)


def summarize_vehicle_profile(rows):
    if not rows:
        return {}
    weights = [r["edge_length_m"] for r in rows]
    summary = {}
    for key in ["alpha_parallel_deg", "alpha_cross_deg", "rho_up", "rho_down", "rho_roll", "rho_slide", "rho_break", "rho_max", "unit_cost"]:
        vals = [r[key] for r in rows]
        summary[f"{key}_mean_weighted"] = _weighted_mean(vals, weights)
        summary[f"{key}_max"] = float(np.nanmax(vals))
        summary[f"{key}_p90"] = float(np.nanpercentile(vals, 90))
    for key in ["rho_up", "rho_down", "rho_roll", "rho_slide", "rho_break", "rho_max"]:
        for thr in [0.5, 0.7, 0.9, 1.0]:
            summary[f"E_{key}_gt_{str(thr).replace('.', 'p')}"] = _exposure(rows, key, thr)
    up_count = sum(1 for r in rows if r["alpha_parallel_deg"] > 1.0)
    down_count = sum(1 for r in rows if r["alpha_parallel_deg"] < -1.0)
    flat_count = len(rows) - up_count - down_count
    summary["上坡段数量_alpha_gt_1deg"] = up_count
    summary["下坡段数量_alpha_lt_minus1deg"] = down_count
    summary["近似平坡段数量_abs_alpha_le_1deg"] = flat_count
    return summary
