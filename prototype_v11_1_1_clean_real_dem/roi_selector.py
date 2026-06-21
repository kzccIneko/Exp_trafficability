"""
roi_selector.py

方向异质性 ROI 自动筛选。

旧实验在 1000x1000 区域上直接画路径，差异常常被空间尺度淹没。
本模块先计算方向各向异性指数 AI 和坡度，再筛选方向差异较强、坡度不太平坦的子区，
用于横坡约束压力消融、路径剖面和局部放大图。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import numpy as np
import warnings
warnings.filterwarnings("ignore", message="Glyph .* missing from font.*")
warnings.filterwarnings("ignore", category=SyntaxWarning)

from cost_model import calculate_slope_angle
from metrics import anisotropy_index


@dataclass
class ROI:
    roi_id: int
    r0: int
    c0: int
    r1: int
    c1: int
    mean_ai: float
    mean_slope_deg: float
    score: float


def select_rois(
    dem: np.ndarray,
    p: np.ndarray,
    q: np.ndarray,
    cost_unit_field: np.ndarray,
    *,
    window_size: int = 240,
    stride: int = 120,
    top_k: int = 4,
    min_mean_slope_deg: float = 2.0,
) -> tuple[np.ndarray, np.ndarray, list[ROI]]:
    """按 AI 与坡度筛选候选 ROI。"""
    rows, cols = dem.shape
    win = int(min(window_size, rows, cols))
    stride = max(1, int(min(stride, win)))
    AI = anisotropy_index(cost_unit_field, method="percentile")
    slope_deg = np.degrees(calculate_slope_angle(p, q))

    rois: list[ROI] = []
    rid = 1
    if rows <= win or cols <= win:
        mask = np.isfinite(AI) & np.isfinite(slope_deg)
        mean_ai = float(np.nanmean(AI[mask])) if np.any(mask) else 0.0
        mean_slope = float(np.nanmean(slope_deg[mask])) if np.any(mask) else 0.0
        rois.append(ROI(rid, 0, 0, rows, cols, mean_ai, mean_slope, mean_ai * np.sqrt(max(mean_slope, 0.0))))
        return AI, slope_deg, rois

    for r0 in range(0, rows - win + 1, stride):
        for c0 in range(0, cols - win + 1, stride):
            r1, c1 = r0 + win, c0 + win
            ai_sub = AI[r0:r1, c0:c1]
            sl_sub = slope_deg[r0:r1, c0:c1]
            mask = np.isfinite(ai_sub) & np.isfinite(sl_sub)
            if not np.any(mask):
                continue
            mean_ai = float(np.nanmean(ai_sub[mask]))
            mean_slope = float(np.nanmean(sl_sub[mask]))
            if mean_slope < min_mean_slope_deg:
                continue
            # AI 表示方向差异，坡度表示地形变化。sqrt 避免陡坡过度支配。
            score = mean_ai * np.sqrt(mean_slope)
            rois.append(ROI(rid, r0, c0, r1, c1, mean_ai, mean_slope, float(score)))
            rid += 1

    rois.sort(key=lambda x: x.score, reverse=True)
    selected: list[ROI] = []
    for roi in rois:
        # 避免候选 ROI 大量重叠。
        ok = True
        for chosen in selected:
            inter_r0 = max(roi.r0, chosen.r0)
            inter_c0 = max(roi.c0, chosen.c0)
            inter_r1 = min(roi.r1, chosen.r1)
            inter_c1 = min(roi.c1, chosen.c1)
            inter = max(0, inter_r1 - inter_r0) * max(0, inter_c1 - inter_c0)
            area = (roi.r1 - roi.r0) * (roi.c1 - roi.c0)
            if inter / max(area, 1) > 0.35:
                ok = False
                break
        if ok:
            roi.roi_id = len(selected) + 1
            selected.append(roi)
        if len(selected) >= top_k:
            break
    if not selected:
        selected = [ROI(1, 0, 0, rows, cols, float(np.nanmean(AI)), float(np.nanmean(slope_deg)), float(np.nanmean(AI)))]
    return AI, slope_deg, selected


def write_roi_csv(path: Path, rois: list[ROI]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=["roi_id", "r0", "c0", "r1", "c1", "mean_ai", "mean_slope_deg", "score"])
        writer.writeheader()
        for roi in rois:
            writer.writerow(roi.__dict__)


def plot_roi_overview(out_dir: Path, AI: np.ndarray, slope_deg: np.ndarray, rois: list[ROI]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    try:
        from run_single_experiment import setup_chinese_font
        setup_chinese_font()
    except Exception:
        pass

    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    im0 = axes[0].imshow(AI, origin="lower", cmap="viridis")
    axes[0].set_title("方向各向异性指数 AI")
    plt.colorbar(im0, ax=axes[0], shrink=0.8)
    im1 = axes[1].imshow(slope_deg, origin="lower", cmap="YlOrRd")
    axes[1].set_title("坡度角 / °")
    plt.colorbar(im1, ax=axes[1], shrink=0.8)
    for ax in axes:
        ax.set_xlabel("列号")
        ax.set_ylabel("行号")
        for roi in rois:
            rect = Rectangle((roi.c0, roi.r0), roi.c1-roi.c0, roi.r1-roi.r0, fill=False, edgecolor="cyan", linewidth=2)
            ax.add_patch(rect)
            ax.text(roi.c0+5, roi.r1-15, f"ROI{roi.roi_id}", color="cyan", fontsize=10, weight="bold")
    fig.suptitle("ROI 自动筛选：优先选择方向差异明显且坡度不太平坦的子区")
    fig.tight_layout()
    fig.savefig(out_dir / "01_roi_selection_AI_and_slope.png", dpi=180, bbox_inches="tight")
    plt.close(fig)
