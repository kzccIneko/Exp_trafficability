"""
scenario_pairs.py

基于地形情景生成起终点，而不是只用随机点。

核心思想：若要证明方向敏感模型有效，起终点必须覆盖会触发方向差异的地形情景：
- 沿等高线/横切坡面：检验横坡约束压力；
- 顺坡上/下行：检验上坡/下坡非对称；
- 对角跨越：检验综合绕行；
- ROI 内随机远距离点：补充稳健性。
"""
from __future__ import annotations

from pathlib import Path
import csv
import math
import numpy as np

from roi_selector import ROI


def _theta_to_step(theta: float) -> tuple[float, float]:
    """theta 使用 atan2(dr, dc) 约定，返回单位向量 dr,dc。"""
    return math.sin(theta), math.cos(theta)


def _clip_point(r: float, c: float, roi: ROI, edge_buffer: int) -> tuple[int, int]:
    rmin = roi.r0 + edge_buffer
    rmax = roi.r1 - edge_buffer - 1
    cmin = roi.c0 + edge_buffer
    cmax = roi.c1 - edge_buffer - 1
    return int(np.clip(round(r), rmin, rmax)), int(np.clip(round(c), cmin, cmax))


def _pair_along(theta: float, roi: ROI, frac: float = 0.40, edge_buffer: int = 8) -> tuple[tuple[int, int], tuple[int, int]]:
    cr = (roi.r0 + roi.r1 - 1) / 2
    cc = (roi.c0 + roi.c1 - 1) / 2
    half = frac * min(roi.r1 - roi.r0, roi.c1 - roi.c0)
    dr, dc = _theta_to_step(theta)
    s = _clip_point(cr - dr * half, cc - dc * half, roi, edge_buffer)
    g = _clip_point(cr + dr * half, cc + dc * half, roi, edge_buffer)
    if s == g:
        s = _clip_point(roi.r0 + edge_buffer, roi.c0 + edge_buffer, roi, edge_buffer)
        g = _clip_point(roi.r1 - edge_buffer - 1, roi.c1 - edge_buffer - 1, roi, edge_buffer)
    return s, g


def generate_scenario_pairs(
    p: np.ndarray,
    q: np.ndarray,
    rois: list[ROI],
    *,
    edge_buffer: int = 8,
    random_pairs_per_roi: int = 2,
    seed: int = 42,
) -> list[dict[str, object]]:
    rng = np.random.RandomState(seed)
    rows: list[dict[str, object]] = []
    pid = 1
    for roi in rois:
        ps = p[roi.r0:roi.r1, roi.c0:roi.c1]
        qs = q[roi.r0:roi.r1, roi.c0:roi.c1]
        mean_p = float(np.nanmean(ps))
        mean_q = float(np.nanmean(qs))
        # 梯度方向：最陡上坡方向；等高线方向与其垂直。
        grad_theta = math.atan2(mean_q, mean_p) % (2 * math.pi)
        contour_theta = (grad_theta + math.pi / 2) % (2 * math.pi)

        scenarios = [
            ("contour_cross_slope", contour_theta, "沿等高线方向布设起终点，用于检验只看纵坡是否会低估横坡约束压力。"),
            ("upslope_downslope", grad_theta, "沿平均坡向布设起终点，用于检验上坡/下坡代价非对称。"),
            ("diagonal_ne_sw", math.pi / 4, "固定对角跨越，用于观察综合绕行。"),
            ("diagonal_nw_se", 3 * math.pi / 4, "另一个对角跨越，用于避免单一方向偶然性。"),
        ]
        for scenario, theta, note in scenarios:
            s, g = _pair_along(theta, roi, edge_buffer=edge_buffer)
            rows.append({
                "pair_id": pid,
                "roi_id": roi.roi_id,
                "scenario": scenario,
                "start": s,
                "goal": g,
                "direction_hint_deg": math.degrees(theta),
                "note": note,
            })
            pid += 1

        # 随机远距离点，补充非情景化稳健性。
        for k in range(random_pairs_per_roi):
            for _ in range(300):
                s = (int(rng.randint(roi.r0+edge_buffer, max(roi.r0+edge_buffer+1, roi.r1-edge_buffer))),
                     int(rng.randint(roi.c0+edge_buffer, max(roi.c0+edge_buffer+1, roi.c1-edge_buffer))))
                g = (int(rng.randint(roi.r0+edge_buffer, max(roi.r0+edge_buffer+1, roi.r1-edge_buffer))),
                     int(rng.randint(roi.c0+edge_buffer, max(roi.c0+edge_buffer+1, roi.c1-edge_buffer))))
                if math.hypot(g[0]-s[0], g[1]-s[1]) > 0.35 * min(roi.r1-roi.r0, roi.c1-roi.c0):
                    break
            rows.append({
                "pair_id": pid,
                "roi_id": roi.roi_id,
                "scenario": f"random_far_{k+1}",
                "start": s,
                "goal": g,
                "direction_hint_deg": float("nan"),
                "note": "ROI 内随机远距离起终点，用于检查结论是否只依赖手工情景。",
            })
            pid += 1
    return rows


def write_pairs_csv(path: Path, pairs: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        fields = ["pair_id", "roi_id", "scenario", "start_row", "start_col", "goal_row", "goal_col", "direction_hint_deg", "note"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in pairs:
            s = row["start"]
            g = row["goal"]
            writer.writerow({
                "pair_id": row["pair_id"],
                "roi_id": row["roi_id"],
                "scenario": row["scenario"],
                "start_row": s[0],
                "start_col": s[1],
                "goal_row": g[0],
                "goal_col": g[1],
                "direction_hint_deg": row["direction_hint_deg"],
                "note": row["note"],
            })
