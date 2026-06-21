"""
profile_plots.py

v6 绘图工具：路径局部放大、剖面曲线、约束利用率统计柱状图。
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
import warnings
warnings.filterwarnings("ignore", message="Glyph .* missing from font.*")
warnings.filterwarnings("ignore", category=SyntaxWarning)


def _setup():
    import matplotlib
    matplotlib.use("Agg")
    try:
        from run_single_experiment import setup_chinese_font
        setup_chinese_font()
    except Exception:
        pass
    import matplotlib.pyplot as plt
    return plt


def plot_cost_field_comparison(out_path: Path, dem, fields: dict[str, np.ndarray], title: str = "方向代价场对比"):
    plt = _setup()
    models = list(fields.keys())
    fig, axes = plt.subplots(1, len(models), figsize=(5.4 * len(models), 5))
    if len(models) == 1:
        axes = [axes]
    for ax, m in zip(axes, models):
        arr = np.nanmin(fields[m], axis=2)
        im = ax.imshow(arr, origin="lower", cmap="RdYlGn_r")
        ax.set_title(f"{m}: 最小方向单位代价")
        ax.set_xlabel("列号")
        ax.set_ylabel("行号")
        plt.colorbar(im, ax=ax, shrink=0.78)
    fig.suptitle(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_paths_global_and_zoom(
    out_path: Path,
    base_field: np.ndarray,
    paths: dict[str, list[tuple[int, int]]],
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    title: str,
    zoom_margin: int = 35,
):
    plt = _setup()
    base = np.nanmin(base_field, axis=2)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    for ax, zoom in zip(axes, [False, True]):
        im = ax.imshow(base, origin="lower", cmap="RdYlGn_r", alpha=0.86)
        for model, path in paths.items():
            if not path:
                continue
            arr = np.asarray(path)
            ax.plot(arr[:, 1], arr[:, 0], linewidth=1.6, label=model)
        ax.plot(start[1], start[0], "go", markersize=7, label="Start")
        ax.plot(goal[1], goal[0], "r*", markersize=10, label="Goal")
        ax.set_xlabel("列号")
        ax.set_ylabel("行号")
        if zoom:
            all_pts = []
            for path in paths.values():
                if path:
                    all_pts.extend(path)
            if all_pts:
                arr = np.asarray(all_pts)
                r0 = max(0, int(np.min(arr[:, 0])) - zoom_margin)
                r1 = min(base.shape[0] - 1, int(np.max(arr[:, 0])) + zoom_margin)
                c0 = max(0, int(np.min(arr[:, 1])) - zoom_margin)
                c1 = min(base.shape[1] - 1, int(np.max(arr[:, 1])) + zoom_margin)
                ax.set_xlim(c0, c1)
                ax.set_ylim(r0, r1)
            ax.set_title("局部放大：看路径差异")
        else:
            ax.set_title("全局图：只看空间位置")
        ax.legend(fontsize=8, loc="upper right")
    fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.78, label="底图：Ours 最小方向单位代价")
    fig.suptitle(title)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_profile_curves(out_path: Path, profiles: dict[str, list[dict]], title: str):
    plt = _setup()
    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    for model, rows in profiles.items():
        if not rows:
            continue
        s = np.array([float(r["cum_distance_m"]) for r in rows])
        ap = np.array([float(r["alpha_parallel_deg"]) for r in rows])
        ac = np.array([float(r["alpha_cross_deg"]) for r in rows])
        uc = np.array([float(r["unit_cost"]) for r in rows])
        elev = np.array([float(r["elevation_end_m"]) for r in rows])
        axes[0].plot(s, elev, label=model)
        axes[1].plot(s, ap, label=model)
        axes[2].plot(s, ac, label=model)
        axes[3].plot(s, uc, label=model)
    axes[0].set_ylabel("高程 / m")
    axes[1].set_ylabel("纵坡角 / °")
    axes[2].set_ylabel("横坡角 / °")
    axes[3].set_ylabel("单位距离代价")
    axes[3].set_xlabel("沿路径累计距离 / m")
    axes[1].axhline(0, color="k", linewidth=0.7, alpha=0.4)
    for y in [8, 10, 12, 15]:
        axes[2].axhline(y, linestyle="--", linewidth=0.6, alpha=0.35)
    for y in [5, 10, 15]:
        axes[1].axhline(y, linestyle="--", linewidth=0.6, alpha=0.30)
        axes[1].axhline(-y, linestyle="--", linewidth=0.6, alpha=0.30)
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, loc="upper right")
    fig.suptitle(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_length_fraction_bars(out_path: Path, rows: list[dict], title: str):
    plt = _setup()
    models = []
    for r in rows:
        m = r.get("模型代码") or r.get("model")
        if m and m not in models:
            models.append(m)
    if not models:
        return
    keys = [
        ("cross_length_fraction_ratio_gt_10p0deg", "横坡>10°"),
        ("cross_length_fraction_ratio_gt_15p0deg", "横坡>15°"),
        ("downhill_length_fraction_ratio_gt_10p0deg", "下坡>10°"),
        ("uphill_length_fraction_ratio_gt_10p0deg", "上坡>10°"),
    ]
    x = np.arange(len(keys))
    width = 0.8 / max(len(models), 1)
    fig, ax = plt.subplots(figsize=(10, 5.6))
    for i, m in enumerate(models):
        sub = [r for r in rows if (r.get("模型代码") or r.get("model")) == m]
        vals = []
        for key, _ in keys:
            v = [float(r.get(key, np.nan)) for r in sub if key in r]
            vals.append(float(np.nanmean(v)) if v else np.nan)
        ax.bar(x + (i - (len(models)-1)/2)*width, vals, width=width, label=m)
    ax.set_xticks(x)
    ax.set_xticklabels([lab for _, lab in keys])
    ax.set_ylabel("路径长度占比")
    ax.set_ylim(0, 1)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_tradeoff_scatter(out_path: Path, rows: list[dict], title: str):
    plt = _setup()
    fig, ax = plt.subplots(figsize=(7.5, 6))
    for m in sorted({r.get("模型代码", "") for r in rows}):
        sub = [r for r in rows if r.get("模型代码") == m]
        x = [float(r.get("路径长度_L_m", np.nan)) for r in sub]
        y = [float(r.get("cross_length_fraction_ratio_gt_10p0deg", np.nan)) for r in sub]
        ax.scatter(x, y, label=m, alpha=0.75)
    ax.set_xlabel("路径长度 / m")
    ax.set_ylabel("横坡>10°路径长度比例")
    ax.set_title(title)
    ax.grid(alpha=0.25)
    ax.legend()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_parameter_sensitivity(out_path: Path, rows: list[dict], x_key: str, metric_keys: list[tuple[str, str]], title: str):
    plt = _setup()
    fig, axes = plt.subplots(1, len(metric_keys), figsize=(5.2 * len(metric_keys), 4.8))
    if len(metric_keys) == 1:
        axes = [axes]
    xs = sorted({float(r[x_key]) for r in rows})
    for ax, (mk, label) in zip(axes, metric_keys):
        vals = []
        for x in xs:
            sub = [float(r[mk]) for r in rows if float(r[x_key]) == x and mk in r]
            vals.append(float(np.nanmean(sub)) if sub else np.nan)
        ax.plot(xs, vals, "o-")
        ax.set_xlabel(x_key)
        ax.set_ylabel(label)
        ax.set_title(label)
        ax.grid(alpha=0.25)
    fig.suptitle(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
