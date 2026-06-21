"""
osm_validation.py

OSM 弱监督验证接口说明（v6 仅预留，不纳入主运行流程）。

下一阶段需要接入 OSM 道路矢量文件后，可实现以下流程：
1. 读取 highway=track/path/unclassified/service/tertiary 等山区道路；
2. 将道路缓冲区栅格化为正样本 S+；
3. 在相同坡度带、土地覆盖带或高程带内抽取非道路负样本 S-；
4. 计算道路点代价是否显著低于匹配非道路点：Delta C = mean(C-) - mean(C+)；
5. 计算 AUC = P(C_road < C_random)，并做空间分块交叉验证。

本文件先给出函数签名和 TODO，避免本轮 DEM-only 实验被外部 OSM 数据卡住。
"""
from __future__ import annotations

from pathlib import Path
import numpy as np


def rasterize_osm_roads_placeholder(vector_path: str | Path, reference_dem_path: str | Path):
    """
    TODO: 下一阶段实现。

    建议依赖：geopandas, rasterio, shapely。
    输入：OSM 道路矢量文件，以及参考 DEM GeoTIFF。
    输出：与 DEM 同形状的 bool 道路掩膜。
    """
    raise NotImplementedError(
        "v6 是 DEM-only 实验包，OSM 弱监督验证只预留接口。"
        "下一阶段请提供 OSM 道路 shp/geojson/gpkg 后实现本函数。"
    )


def auc_lower_cost_positive(positive_costs: np.ndarray, negative_costs: np.ndarray) -> float:
    """AUC = P(C_positive < C_negative) + 0.5*P(tie)。"""
    pos = np.asarray(positive_costs, dtype=float)
    neg = np.asarray(negative_costs, dtype=float)
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    total = len(pos) * len(neg)
    score = 0.0
    for x in pos:
        score += float(np.sum(x < neg))
        score += 0.5 * float(np.sum(x == neg))
    return score / max(total, 1)


def matched_delta_cost(positive_costs: np.ndarray, negative_costs: np.ndarray) -> dict[str, float | int]:
    """Delta C = C_negative - C_positive，正值说明道路样本代价更低。"""
    n = min(len(positive_costs), len(negative_costs))
    if n == 0:
        return {"n": 0, "mean_delta": float("nan"), "median_delta": float("nan")}
    delta = np.asarray(negative_costs[:n], dtype=float) - np.asarray(positive_costs[:n], dtype=float)
    return {
        "n": int(n),
        "mean_delta": float(np.nanmean(delta)),
        "median_delta": float(np.nanmedian(delta)),
        "q10_delta": float(np.nanpercentile(delta, 10)),
        "q90_delta": float(np.nanpercentile(delta, 90)),
    }
