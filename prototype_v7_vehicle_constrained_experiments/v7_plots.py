from __future__ import annotations
from pathlib import Path
import numpy as np


def setup_font():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        from run_single_experiment import setup_chinese_font
        setup_chinese_font()
    except Exception:
        pass
    return plt


def plot_constraint_curves(path, curves, title="车辆约束能力利用率曲线"):
    plt = setup_font()
    fig, ax = plt.subplots(figsize=(9, 5.5))
    x = curves["degree"]
    for key, label in [("rho_up", "上坡牵引 ρ_up"), ("rho_down", "下坡制动 ρ_down"), ("rho_roll", "横坡侧翻 ρ_roll"), ("rho_slide", "横向侧滑 ρ_slide")]:
        ax.plot(x, curves[key], label=label, lw=2)
    ax.axhline(1.0, ls="--", color="black", lw=1, label="能力边界 ρ=1")
    ax.axhline(0.7, ls=":", color="gray", lw=1, label="高利用率参考 ρ=0.7")
    ax.set_xlabel("坡度/横坡角 α / °")
    ax.set_ylabel("能力利用率 ρ")
    ax.set_title(title)
    ax.set_ylim(0, max(1.6, float(np.nanpercentile([curves[k] for k in ["rho_up","rho_down","rho_roll","rho_slide"]], 95))))
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_cost_maps(path, dem, fields, title="单位距离代价场最小方向投影"):
    plt = setup_font()
    n = len(fields)
    fig, axes = plt.subplots(1, n, figsize=(5.2*n, 4.5))
    if n == 1:
        axes = [axes]
    vals = []
    for C in fields.values():
        vals.append(np.nanmin(C, axis=2))
    allv = np.concatenate([v[np.isfinite(v)].ravel() for v in vals])
    vmin, vmax = np.percentile(allv, [2, 98]) if allv.size else (0, 1)
    for ax, (name, C), M in zip(axes, fields.items(), vals):
        im = ax.imshow(M, origin="lower", cmap="viridis", vmin=vmin, vmax=vmax)
        ax.set_title(name)
        ax.set_xticks([]); ax.set_yticks([])
    fig.suptitle(title)
    fig.colorbar(im, ax=axes, shrink=0.75, label="min_d c(i,d)")
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_paths(path, base_field, paths, start, goal, title="路径对比"):
    plt = setup_font()
    fig, ax = plt.subplots(figsize=(8, 7))
    M = np.nanmin(base_field, axis=2)
    im = ax.imshow(M, origin="lower", cmap="gray")
    for name, pth in paths.items():
        if not pth:
            continue
        rr = [p[0] for p in pth]
        cc = [p[1] for p in pth]
        ax.plot(cc, rr, lw=2, label=name)
    ax.scatter([start[1]], [start[0]], marker="o", s=55, label="Start")
    ax.scatter([goal[1]], [goal[0]], marker="*", s=85, label="Goal")
    ax.legend(loc="best", fontsize=8)
    ax.set_title(title)
    fig.colorbar(im, ax=ax, shrink=0.75, label="V7 min cost")
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_vehicle_profiles(path, profiles, title="沿路径车辆约束剖面"):
    plt = setup_font()
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for name, rows in profiles.items():
        if not rows:
            continue
        x = [r["cum_distance_m"] for r in rows]
        axes[0].plot(x, [r["alpha_parallel_deg"] for r in rows], label=name)
        axes[1].plot(x, [r["alpha_cross_deg"] for r in rows], label=name)
        axes[2].plot(x, [r["rho_up"] for r in rows], label=f"{name} up")
        axes[2].plot(x, [r["rho_roll"] for r in rows], ls="--", label=f"{name} roll")
        axes[3].plot(x, [r["rho_slide"] for r in rows], label=f"{name} slide")
        axes[3].plot(x, [r["rho_max"] for r in rows], ls="--", label=f"{name} max")
    axes[0].set_ylabel("纵坡角/°")
    axes[1].set_ylabel("横坡角/°")
    axes[2].set_ylabel("ρ_up / ρ_roll")
    axes[3].set_ylabel("ρ_slide / ρ_max")
    axes[3].set_xlabel("累计距离 / m")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.axhline(0.7, ls=":", color="gray", lw=1)
        ax.axhline(1.0, ls="--", color="black", lw=1)
        ax.legend(fontsize=7, ncol=2)
    fig.suptitle(title)
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_exposure_bars(path, rows, title="车辆约束暴露比例"):
    plt = setup_font()
    keys = ["E_rho_up_gt_0p7", "E_rho_roll_gt_0p7", "E_rho_slide_gt_0p7", "E_rho_max_gt_1p0"]
    labels = ["up>0.7", "roll>0.7", "slide>0.7", "any>1.0"]
    models = [r.get("模型代码", r.get("model", "?")) for r in rows]
    x = np.arange(len(models))
    width = 0.18
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for i, (k, lab) in enumerate(zip(keys, labels)):
        vals = [float(r.get(k, np.nan)) for r in rows]
        ax.bar(x + (i-1.5)*width, vals, width, label=lab)
    ax.set_xticks(x); ax.set_xticklabels(models, rotation=25)
    ax.set_ylabel("路径长度占比")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
