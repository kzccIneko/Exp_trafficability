"""
exposure_metrics.py

路径风险暴露与沿途剖面统计。

本模块的目标不是重新计算最优路径，而是把已经得到的路径解释清楚：
- 沿途纵坡角、横坡角、单位距离代价如何变化；
- 上坡/下坡/近似平坡分别出现了多少段；
- 高横坡、陡下坡等危险暴露长度占比是多少；
- 路径曲折度、爬升/下降、高程剖面如何。

这些指标用于回答“路径图上看不出差别时，模型到底在哪些风险维度有改进”。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from cost_model import longitudinal_cross_slope_tan
from path_planning import nearest_direction_index


def edge_sample_points(r: int, c: int, nr: int, nc: int) -> list[tuple[int, int]]:
    """返回边上的采样点，包括起点和终点。"""
    steps = int(max(abs(nr - r), abs(nc - c)))
    if steps <= 1:
        return [(r, c), (nr, nc)]
    pts: list[tuple[int, int]] = []
    for k in range(steps + 1):
        t = k / steps
        rr = int(round(r + t * (nr - r)))
        cc = int(round(c + t * (nc - c)))
        if not pts or pts[-1] != (rr, cc):
            pts.append((rr, cc))
    return pts


def _angle_diff_deg(a: float, b: float) -> float:
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def path_profile(
    path: list[tuple[int, int]],
    dem: np.ndarray,
    p: np.ndarray,
    q: np.ndarray,
    cost_unit_field: np.ndarray,
    directions: np.ndarray,
    cell_size: float,
) -> list[dict[str, float | int]]:
    """
    抽取路径逐边剖面。

    输出的每一行对应一条路径边，包括：
    - 累计距离；
    - 边长；
    - 移动方向；
    - 高程变化；
    - 纵坡角 alpha_parallel_deg；
    - 横坡角 alpha_cross_deg；
    - 单位距离代价 unit_cost；
    - 边代价 edge_cost。
    """
    rows: list[dict[str, float | int]] = []
    if len(path) < 2:
        return rows

    rows_n, cols_n = dem.shape
    cumulative = 0.0
    for edge_id, ((r1, c1), (r2, c2)) in enumerate(zip(path[:-1], path[1:]), start=1):
        dr, dc = int(r2 - r1), int(c2 - c1)
        theta = math.atan2(dr, dc) % (2 * math.pi)
        theta_deg = math.degrees(theta)
        dir_idx = nearest_direction_index(theta, directions)
        edge_len = float(math.hypot(dr, dc) * cell_size)
        if edge_len <= 0:
            continue

        pts = edge_sample_points(r1, c1, r2, c2)
        vals_unit: list[float] = []
        vals_ap: list[float] = []
        vals_ac: list[float] = []
        vals_z: list[float] = []
        for rr, cc in pts:
            if rr < 0 or rr >= rows_n or cc < 0 or cc >= cols_n:
                continue
            gp, gc = longitudinal_cross_slope_tan(p[rr:rr+1, cc:cc+1], q[rr:rr+1, cc:cc+1], theta)
            vals_ap.append(float(math.degrees(math.atan(float(gp[0, 0])))))
            vals_ac.append(float(abs(math.degrees(math.atan(float(gc[0, 0]))))))
            vals_unit.append(float(cost_unit_field[rr, cc, dir_idx]))
            vals_z.append(float(dem[rr, cc]))

        if vals_unit:
            unit_cost = float(np.nanmean(vals_unit))
        else:
            unit_cost = float("nan")
        alpha_p = float(np.nanmean(vals_ap)) if vals_ap else float("nan")
        alpha_c = float(np.nanmean(vals_ac)) if vals_ac else float("nan")
        z_start = float(dem[r1, c1])
        z_end = float(dem[r2, c2])
        dz = z_end - z_start
        edge_cost = unit_cost * edge_len
        cumulative += edge_len
        rows.append({
            "edge_id": edge_id,
            "from_row": int(r1),
            "from_col": int(c1),
            "to_row": int(r2),
            "to_col": int(c2),
            "edge_length_m": edge_len,
            "cum_distance_m": cumulative,
            "direction_deg": theta_deg,
            "dz_m": float(dz),
            "elevation_start_m": z_start,
            "elevation_end_m": z_end,
            "alpha_parallel_deg": alpha_p,
            "alpha_cross_deg": alpha_c,
            "unit_cost": unit_cost,
            "edge_cost": edge_cost,
        })
    return rows


def summarize_profile(
    profile: list[dict[str, float | int]],
    *,
    flat_threshold_deg: float = 1.0,
    cross_thresholds_deg: Iterable[float] = (8.0, 10.0, 12.0, 15.0),
    uphill_thresholds_deg: Iterable[float] = (5.0, 10.0, 15.0),
    downhill_thresholds_deg: Iterable[float] = (5.0, 10.0, 15.0),
) -> dict[str, float | int]:
    """汇总一条路径的风险暴露指标。"""
    if not profile:
        return {
            "profile_edges": 0,
            "path_length_profile_m": float("nan"),
        }

    L = np.array([float(x["edge_length_m"]) for x in profile], dtype=float)
    ap = np.array([float(x["alpha_parallel_deg"]) for x in profile], dtype=float)
    ac = np.array([float(x["alpha_cross_deg"]) for x in profile], dtype=float)
    dz = np.array([float(x["dz_m"]) for x in profile], dtype=float)
    uc = np.array([float(x["unit_cost"]) for x in profile], dtype=float)
    ec = np.array([float(x["edge_cost"]) for x in profile], dtype=float)
    dirs = np.array([float(x["direction_deg"]) for x in profile], dtype=float)
    total_L = float(np.nansum(L))

    uphill = ap > flat_threshold_deg
    downhill = ap < -flat_threshold_deg
    flat = ~(uphill | downhill)

    def wmean(values: np.ndarray, weights: np.ndarray) -> float:
        mask = np.isfinite(values) & np.isfinite(weights) & (weights > 0)
        if not np.any(mask):
            return float("nan")
        return float(np.average(values[mask], weights=weights[mask]))

    def exposure(mask: np.ndarray) -> tuple[float, float]:
        length = float(np.nansum(L[mask]))
        ratio = length / max(total_L, 1e-12)
        return length, ratio

    out: dict[str, float | int] = {
        "profile_edges": int(len(profile)),
        "path_length_profile_m": total_L,
        "risk_weighted_distance_J": float(np.nansum(ec)),
        "mean_unit_cost_weighted": wmean(uc, L),
        "elevation_gain_m": float(np.nansum(np.maximum(dz, 0))),
        "elevation_loss_m": float(np.nansum(np.maximum(-dz, 0))),
        "net_elevation_change_m": float(np.nansum(dz)),
        "uphill_edge_count": int(np.sum(uphill)),
        "downhill_edge_count": int(np.sum(downhill)),
        "flat_edge_count": int(np.sum(flat)),
        "uphill_length_m": exposure(uphill)[0],
        "uphill_length_ratio": exposure(uphill)[1],
        "downhill_length_m": exposure(downhill)[0],
        "downhill_length_ratio": exposure(downhill)[1],
        "flat_length_m": exposure(flat)[0],
        "flat_length_ratio": exposure(flat)[1],
        "mean_uphill_angle_deg": wmean(ap[uphill], L[uphill]) if np.any(uphill) else 0.0,
        "mean_downhill_angle_abs_deg": wmean(np.abs(ap[downhill]), L[downhill]) if np.any(downhill) else 0.0,
        "mean_abs_longitudinal_angle_deg": wmean(np.abs(ap), L),
        "max_uphill_angle_deg": float(np.nanmax(ap)) if np.any(np.isfinite(ap)) else float("nan"),
        "max_downhill_angle_abs_deg": float(abs(np.nanmin(ap))) if np.any(np.isfinite(ap)) else float("nan"),
        "mean_cross_angle_deg": wmean(ac, L),
        "median_cross_angle_deg": float(np.nanmedian(ac)),
        "q90_cross_angle_deg": float(np.nanpercentile(ac, 90)),
        "max_cross_angle_deg": float(np.nanmax(ac)),
    }

    for thr in cross_thresholds_deg:
        mask = ac > thr
        length, ratio = exposure(mask)
        key = str(thr).replace(".", "p")
        out[f"cross_exposure_len_gt_{key}deg_m"] = length
        out[f"cross_exposure_ratio_gt_{key}deg"] = ratio
        out[f"cross_exposure_count_gt_{key}deg"] = int(np.sum(mask))

    for thr in uphill_thresholds_deg:
        mask = ap > thr
        length, ratio = exposure(mask)
        key = str(thr).replace(".", "p")
        out[f"uphill_exposure_len_gt_{key}deg_m"] = length
        out[f"uphill_exposure_ratio_gt_{key}deg"] = ratio
        out[f"uphill_exposure_count_gt_{key}deg"] = int(np.sum(mask))

    for thr in downhill_thresholds_deg:
        mask = ap < -thr
        length, ratio = exposure(mask)
        key = str(thr).replace(".", "p")
        out[f"downhill_exposure_len_gt_{key}deg_m"] = length
        out[f"downhill_exposure_ratio_gt_{key}deg"] = ratio
        out[f"downhill_exposure_count_gt_{key}deg"] = int(np.sum(mask))

    # 路径曲折度：相邻边方向变化绝对值，范围 0~180 度。
    if len(dirs) >= 2:
        turns = np.array([_angle_diff_deg(float(dirs[i+1]), float(dirs[i])) for i in range(len(dirs)-1)], dtype=float)
        out["turn_count"] = int(np.sum(turns > 1e-6))
        out["total_turn_angle_deg"] = float(np.nansum(turns))
        out["mean_turn_angle_deg"] = float(np.nanmean(turns))
        out["max_turn_angle_deg"] = float(np.nanmax(turns))
    else:
        out["turn_count"] = 0
        out["total_turn_angle_deg"] = 0.0
        out["mean_turn_angle_deg"] = 0.0
        out["max_turn_angle_deg"] = 0.0
    return out


def metric_explanation_rows() -> list[dict[str, str]]:
    """风险暴露指标说明，用于输出 CSV/Markdown。"""
    return [
        {"指标": "risk_weighted_distance_J", "含义": "风险加权距离；路径上单位距离代价乘边长后的总和。当前阶段它不是时间或能耗，而是相对通行阻碍总代价。"},
        {"指标": "mean_unit_cost_weighted", "含义": "按边长加权的平均单位距离阻碍。路径长度差异大时，比总代价更适合比较路径穿越地形的平均困难程度。"},
        {"指标": "uphill/downhill/flat_edge_count", "含义": "沿路径被判定为上坡、下坡、近似平坡的边数量；阈值默认为绝对纵坡角 1°。"},
        {"指标": "uphill/downhill/flat_length_ratio", "含义": "上坡、下坡、近似平坡在路径总长度中的比例，比边数量更稳定。"},
        {"指标": "mean_uphill_angle_deg", "含义": "仅统计上坡边的平均纵坡角，用于判断路径是否倾向于避开持续上坡。"},
        {"指标": "mean_downhill_angle_abs_deg", "含义": "仅统计下坡边的平均下坡绝对角度，用于判断路径是否穿越陡下坡风险区。"},
        {"指标": "mean_cross_angle_deg / q90_cross_angle_deg / max_cross_angle_deg", "含义": "沿路径横坡角均值、90%分位数和最大值；用于评价侧倾/侧滑风险暴露。"},
        {"指标": "cross_exposure_ratio_gt_Xdeg", "含义": "横坡角超过 X° 的路径长度占比；这是横坡消融实验的核心指标。"},
        {"指标": "downhill_exposure_ratio_gt_Xdeg", "含义": "下坡角绝对值超过 X° 的路径长度占比；用于评价陡下坡制动和失稳风险。"},
        {"指标": "total_turn_angle_deg", "含义": "路径累计转向角，反映路径曲折程度；不能单独代表好坏，但可与风险暴露共同解释绕行代价。"},
    ]
