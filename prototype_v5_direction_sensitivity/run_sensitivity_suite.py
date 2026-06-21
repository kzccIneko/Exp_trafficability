"""
run_sensitivity_suite.py

DEM 方向敏感模型扩展实验：
1. 多起终点实验：检验结论是否依赖单一起终点；
2. 起终点互换实验：检验方向代价导致的上/下坡非对称性；
3. 邻域敏感性实验：比较 4/8/16/32 邻域对路径和指标的影响。

示例：
python run_sensitivity_suite.py --dem "D:\\VSCode Program\\通行能力分析_研\\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --cell-size 30 --max-pixels 1000 --out outputs_sensitivity

快速测试：
python run_sensitivity_suite.py --synthetic --out outputs_sensitivity_synthetic --n-pairs 8
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time
import numpy as np

from cost_model import (
    calculate_gradient_tan,
    compute_b0_unit_cost_field,
    compute_b1_unit_cost_field,
    compute_ours_unit_cost_field,
)
from metrics import anisotropy_index, summarize_array
from path_planning import astar_directional, evaluate_path_cost, path_cell_iou, PathResult
from direction_utils import get_directions, get_moves, describe_moves

# 复用单次实验脚本中的 DEM 读取函数与绘图辅助函数
from run_single_experiment import (
    generate_synthetic_dem,
    load_dem_geotiff,
    setup_chinese_font,
    safe_q_limits,
    add_common_axis_labels,
)

MODEL_ORDER = ["B0", "B1", "Ours"]
MODEL_CN = {
    "B0": "B0 标量坡度模型",
    "B1": "B1 仅纵坡方向模型",
    "Ours": "Ours 纵坡+横坡方向模型",
}


def parse_pair(text: str | None) -> tuple[int, int] | None:
    if not text:
        return None
    parts = text.replace(";", ",").split(",")
    if len(parts) != 2:
        raise ValueError("坐标格式应为 row,col，例如 100,120")
    return int(parts[0]), int(parts[1])


def parse_neighbors(text: str) -> list[int]:
    vals = [int(x.strip()) for x in text.split(",") if x.strip()]
    for v in vals:
        if v not in (4, 8, 16, 32):
            raise ValueError("邻域数只能包含 4、8、16、32")
    return vals


def compute_model_cost_fields(p, q, n_neighbors, args):
    directions = get_directions(n_neighbors)
    _, C0 = compute_b0_unit_cost_field(p, q, directions, alpha_u=args.alpha_u)
    _, C1 = compute_b1_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d)
    _, Co, _ = compute_ours_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d, alpha_r=args.alpha_r)
    return directions, {"B0": C0, "B1": C1, "Ours": Co}


def generate_endpoint_pairs(shape, n_pairs, edge_buffer=10, min_distance_ratio=0.35, seed=42):
    """生成随机起终点对，尽量覆盖研究区。"""
    rng = np.random.RandomState(seed)
    rows, cols = shape
    rmin, rmax = edge_buffer, rows - edge_buffer - 1
    cmin, cmax = edge_buffer, cols - edge_buffer - 1
    if rmax <= rmin or cmax <= cmin:
        raise ValueError("DEM 太小或 edge_buffer 太大，无法生成起终点")
    diag = np.hypot(rows, cols)
    min_dist = min_distance_ratio * diag
    pairs = []
    attempts = 0
    while len(pairs) < n_pairs and attempts < n_pairs * 500:
        attempts += 1
        s = (int(rng.randint(rmin, rmax + 1)), int(rng.randint(cmin, cmax + 1)))
        g = (int(rng.randint(rmin, rmax + 1)), int(rng.randint(cmin, cmax + 1)))
        if np.hypot(g[0]-s[0], g[1]-s[1]) >= min_dist:
            pairs.append((s, g))
    if len(pairs) < n_pairs:
        print(f"警告：只生成了 {len(pairs)} 组起终点；可降低 --min-distance-ratio 或 --edge-buffer。")
    return pairs


def default_pair(shape, edge_buffer=10):
    rows, cols = shape
    s = (max(edge_buffer + 1, int(rows * 0.10)), max(edge_buffer + 1, int(cols * 0.10)))
    g = (min(rows - edge_buffer - 2, int(rows * 0.90)), min(cols - edge_buffer - 2, int(cols * 0.90)))
    return s, g


def path_row(exp, pair_id, model, n_neighbors, start, goal, result: PathResult):
    return {
        "实验": exp,
        "起终点编号": pair_id,
        "模型": MODEL_CN[model],
        "模型代码": model,
        "邻域数": n_neighbors,
        "起点_row": start[0],
        "起点_col": start[1],
        "终点_row": goal[0],
        "终点_col": goal[1],
        "路径总代价_J": result.total_cost,
        "路径长度_L_m": result.path_length_m,
        "平均单位距离代价_J除以L": result.average_cost_per_m,
        "绕行率_L除以D": result.detour_ratio,
        "路径节点数": len(result.path),
        "是否找到路径": bool(result.path),
    }


def write_csv(path: Path, rows: list[dict]):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def aggregate_multistart(rows):
    out = []
    for model in MODEL_ORDER:
        sub = [r for r in rows if r["模型代码"] == model and r["是否找到路径"]]
        if not sub:
            continue
        for metric in ["路径总代价_J", "路径长度_L_m", "平均单位距离代价_J除以L", "绕行率_L除以D"]:
            vals = np.array([float(r[metric]) for r in sub], dtype=float)
            out.append({
                "模型": MODEL_CN[model],
                "模型代码": model,
                "指标": metric,
                "样本数": len(vals),
                "均值": float(np.mean(vals)),
                "中位数": float(np.median(vals)),
                "10分位数": float(np.percentile(vals, 10)),
                "90分位数": float(np.percentile(vals, 90)),
                "最小值": float(np.min(vals)),
                "最大值": float(np.max(vals)),
            })
    return out


def plot_multistart(out_dir: Path, dem, Cbase, pairs, all_paths, rows):
    import matplotlib
    matplotlib.use("Agg")
    setup_chinese_font()
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    min_base = np.min(Cbase, axis=2)
    vmin, vmax = safe_q_limits(min_base, q_low=2, q_high=98)

    # 只画 Ours 的多组路径，用于看空间覆盖与是否贴边。
    fig, ax = plt.subplots(figsize=(8, 7))
    im = ax.imshow(min_base, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax, alpha=0.85)
    count = 0
    for key, res in all_paths.items():
        pair_id, model = key
        if model != "Ours" or not res.path:
            continue
        arr = np.asarray(res.path)
        ax.plot(arr[:, 1], arr[:, 0], linewidth=1.0, alpha=0.85)
        count += 1
    for i, (s, g) in enumerate(pairs):
        ax.plot(s[1], s[0], "go", markersize=3)
        ax.plot(g[1], g[0], "r*", markersize=4)
    add_common_axis_labels(ax)
    ax.set_title(f"多起终点实验：Ours 路径空间分布（{count} 条）")
    plt.colorbar(im, ax=ax, shrink=0.8, label="底图：Ours 最小单位距离代价")
    fig.tight_layout()
    fig.savefig(out_dir / "01_多起终点_Ours路径空间分布.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # 指标箱线图。
    metrics = [
        ("平均单位距离代价_J除以L", "平均单位距离代价 J/L"),
        ("绕行率_L除以D", "绕行率 L/D"),
        ("路径长度_L_m", "路径长度 L / m"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    for ax, (mkey, ylabel) in zip(axes, metrics):
        data = []
        labels = []
        for model in MODEL_ORDER:
            vals = [float(r[mkey]) for r in rows if r["模型代码"] == model and r["是否找到路径"]]
            data.append(vals)
            labels.append(model)
        ax.boxplot(data, labels=labels, showmeans=True)
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("多起终点实验：路径指标分布")
    fig.tight_layout()
    fig.savefig(out_dir / "02_多起终点_指标箱线图.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def run_multistart(out: Path, dem, p, q, cell_size, pairs, args):
    print("\n[实验1] 多起终点实验")
    exp_dir = out / "01_多起终点实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    n_neighbors = args.multi_neighbors
    directions, cost_fields = compute_model_cost_fields(p, q, n_neighbors, args)
    all_rows = []
    all_paths = {}
    for pair_id, (start, goal) in enumerate(pairs, start=1):
        print(f"  起终点 {pair_id}/{len(pairs)}: {start} -> {goal}")
        for model in MODEL_ORDER:
            res = astar_directional(cost_fields[model], start, goal, cell_size, directions, n_neighbors=n_neighbors, edge_buffer=args.edge_buffer)
            all_paths[(pair_id, model)] = res
            all_rows.append(path_row("多起终点", pair_id, model, n_neighbors, start, goal, res))
            print(f"    {model}: J={res.total_cost:.2f}, L={res.path_length_m:.1f}, J/L={res.average_cost_per_m:.4f}, detour={res.detour_ratio:.3f}")
    write_csv(exp_dir / "01_多起终点_逐路径指标.csv", all_rows)
    write_csv(exp_dir / "02_多起终点_模型汇总指标.csv", aggregate_multistart(all_rows))
    plot_multistart(exp_dir, dem, cost_fields["Ours"], pairs, all_paths, all_rows)
    return all_rows


def plot_reverse(out_dir: Path, Cbase, start, goal, records, paths_by_key):
    import matplotlib
    matplotlib.use("Agg")
    setup_chinese_font()
    import matplotlib.pyplot as plt

    min_base = np.min(Cbase, axis=2)
    vmin, vmax = safe_q_limits(min_base, q_low=2, q_high=98)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, model in zip(axes, MODEL_ORDER):
        im = ax.imshow(min_base, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax, alpha=0.85)
        fwd = paths_by_key[(model, "forward")]
        rev = paths_by_key[(model, "reverse")]
        if fwd.path:
            a = np.asarray(fwd.path)
            ax.plot(a[:,1], a[:,0], "b-", linewidth=1.6, label="正向最优路径 S→G")
        if rev.path:
            a = np.asarray(rev.path)
            ax.plot(a[:,1], a[:,0], "r--", linewidth=1.4, label="反向最优路径 G→S")
        ax.plot(start[1], start[0], "go", markersize=8, label="S")
        ax.plot(goal[1], goal[0], "r*", markersize=10, label="G")
        ax.set_title(MODEL_CN[model])
        add_common_axis_labels(ax)
        ax.legend(fontsize=7, loc="upper left")
    fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.8, label="底图：Ours 最小单位距离代价")
    fig.suptitle("起终点互换实验：正向路径与反向路径对比")
    fig.savefig(out_dir / "01_起终点互换_正反向路径对比.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def run_reverse(out: Path, dem, p, q, cell_size, start, goal, args):
    print("\n[实验2] 起终点互换实验")
    exp_dir = out / "02_起终点互换实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    n_neighbors = args.reverse_neighbors
    directions, cost_fields = compute_model_cost_fields(p, q, n_neighbors, args)
    rows = []
    paths_by_key = {}
    for model in MODEL_ORDER:
        C = cost_fields[model]
        fwd = astar_directional(C, start, goal, cell_size, directions, n_neighbors=n_neighbors, edge_buffer=args.edge_buffer)
        rev = astar_directional(C, goal, start, cell_size, directions, n_neighbors=n_neighbors, edge_buffer=args.edge_buffer)
        same_geom_rev = evaluate_path_cost(C, list(reversed(fwd.path)), cell_size, directions, n_neighbors=n_neighbors) if fwd.path else PathResult([], np.nan, np.nan, np.nan, np.nan)
        same_geom_fwd_from_rev = evaluate_path_cost(C, list(reversed(rev.path)), cell_size, directions, n_neighbors=n_neighbors) if rev.path else PathResult([], np.nan, np.nan, np.nan, np.nan)
        iou = path_cell_iou(fwd.path, rev.path)
        paths_by_key[(model, "forward")] = fwd
        paths_by_key[(model, "reverse")] = rev
        rows.append({
            "模型": MODEL_CN[model],
            "模型代码": model,
            "邻域数": n_neighbors,
            "正向总代价_J_S到G": fwd.total_cost,
            "反向最优总代价_J_G到S": rev.total_cost,
            "正向几何路径反向行驶代价": same_geom_rev.total_cost,
            "反向几何路径正向行驶代价": same_geom_fwd_from_rev.total_cost,
            "正向路径长度_m": fwd.path_length_m,
            "反向路径长度_m": rev.path_length_m,
            "正向平均单位距离代价": fwd.average_cost_per_m,
            "反向平均单位距离代价": rev.average_cost_per_m,
            "正反向路径单元重合率_IoU": iou,
            "同一几何反向代价变化率": (same_geom_rev.total_cost - fwd.total_cost) / max(fwd.total_cost, 1e-12) if np.isfinite(fwd.total_cost) else np.nan,
            "解释": "若模型方向非对称，则同一几何路径反向行驶代价可能不同；若最优反向路径与正向路径重合率低，说明方向代价影响路径选择。B0理论上应最接近对称。",
        })
        print(f"  {model}: J_fwd={fwd.total_cost:.2f}, J_rev={rev.total_cost:.2f}, same_geom_rev={same_geom_rev.total_cost:.2f}, IoU={iou:.3f}")
    write_csv(exp_dir / "01_起终点互换_指标.csv", rows)
    plot_reverse(exp_dir, cost_fields["Ours"], start, goal, rows, paths_by_key)
    return rows


def plot_neighborhood(out_dir: Path, dem, base_costs, start, goal, metrics_rows, paths_by_key):
    import matplotlib
    matplotlib.use("Agg")
    setup_chinese_font()
    import matplotlib.pyplot as plt

    # Ours path for each neighborhood
    neighs = sorted({int(r["邻域数"]) for r in metrics_rows})
    fig, axes = plt.subplots(1, len(neighs), figsize=(5*len(neighs), 5))
    if len(neighs) == 1:
        axes = [axes]
    last_im = None
    for ax, n in zip(axes, neighs):
        Cbase = base_costs[n]
        min_base = np.min(Cbase, axis=2)
        vmin, vmax = safe_q_limits(min_base, q_low=2, q_high=98)
        last_im = ax.imshow(min_base, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax, alpha=0.85)
        res = paths_by_key[(n, "Ours")]
        if res.path:
            arr = np.asarray(res.path)
            ax.plot(arr[:,1], arr[:,0], "b-", linewidth=1.4, label="Ours路径")
        ax.plot(start[1], start[0], "go", markersize=7, label="起点")
        ax.plot(goal[1], goal[0], "r*", markersize=9, label="终点")
        ax.set_title(f"{n} 邻域")
        add_common_axis_labels(ax)
        ax.legend(fontsize=7, loc="upper left")
    if last_im is not None:
        fig.colorbar(last_im, ax=np.ravel(axes).tolist(), shrink=0.8, label="Ours 最小单位距离代价")
    fig.suptitle("邻域敏感性实验：Ours 路径随邻域数变化")
    fig.savefig(out_dir / "01_邻域敏感性_Ours路径对比.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # All models grid
    fig, axes = plt.subplots(len(neighs), 3, figsize=(15, 4.2*len(neighs)))
    if len(neighs) == 1:
        axes = np.array([axes])
    for i, n in enumerate(neighs):
        Cbase = base_costs[n]
        min_base = np.min(Cbase, axis=2)
        vmin, vmax = safe_q_limits(min_base, q_low=2, q_high=98)
        for j, model in enumerate(MODEL_ORDER):
            ax = axes[i, j]
            im = ax.imshow(min_base, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax, alpha=0.85)
            res = paths_by_key[(n, model)]
            if res.path:
                arr = np.asarray(res.path)
                ax.plot(arr[:,1], arr[:,0], "b-", linewidth=1.2)
            ax.plot(start[1], start[0], "go", markersize=5)
            ax.plot(goal[1], goal[0], "r*", markersize=7)
            ax.set_title(f"{n}邻域 - {model}")
            add_common_axis_labels(ax)
    fig.suptitle("邻域敏感性实验：三模型路径对比")
    fig.tight_layout()
    fig.savefig(out_dir / "02_邻域敏感性_三模型路径矩阵.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # Metric lines
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    metric_info = [
        ("平均单位距离代价_J除以L", "平均单位距离代价 J/L"),
        ("绕行率_L除以D", "绕行率 L/D"),
        ("路径长度_L_m", "路径长度 L / m"),
    ]
    for ax, (key, ylabel) in zip(axes, metric_info):
        for model in MODEL_ORDER:
            xs, ys = [], []
            for n in neighs:
                rec = next((r for r in metrics_rows if r["模型代码"] == model and int(r["邻域数"]) == n), None)
                if rec is not None:
                    xs.append(n)
                    ys.append(float(rec[key]))
            ax.plot(xs, ys, "o-", label=model)
        ax.set_xlabel("邻域方向数")
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)
        ax.grid(alpha=0.25)
        ax.legend()
    fig.suptitle("邻域敏感性实验：路径指标随邻域数变化")
    fig.tight_layout()
    fig.savefig(out_dir / "03_邻域敏感性_指标折线图.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def run_neighborhood(out: Path, dem, p, q, cell_size, start, goal, args):
    print("\n[实验3] 邻域敏感性实验")
    exp_dir = out / "03_邻域敏感性实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    move_rows = []
    paths_by_key = {}
    base_costs = {}
    for n in args.neighbors:
        print(f"  计算 {n} 邻域")
        directions, cost_fields = compute_model_cost_fields(p, q, n, args)
        base_costs[n] = cost_fields["Ours"]
        for row in describe_moves(n):
            row["邻域数"] = n
            move_rows.append(row)
        for model in MODEL_ORDER:
            t = time.time()
            res = astar_directional(cost_fields[model], start, goal, cell_size, directions, n_neighbors=n, edge_buffer=args.edge_buffer)
            paths_by_key[(n, model)] = res
            rows.append(path_row("邻域敏感性", 1, model, n, start, goal, res))
            print(f"    {model}: J={res.total_cost:.2f}, L={res.path_length_m:.1f}, J/L={res.average_cost_per_m:.4f}, detour={res.detour_ratio:.3f}, time={time.time()-t:.2f}s")
    write_csv(exp_dir / "01_邻域敏感性_路径指标.csv", rows)
    write_csv(exp_dir / "02_邻域方向集合说明.csv", move_rows)
    plot_neighborhood(exp_dir, dem, base_costs, start, goal, rows, paths_by_key)
    return rows


def write_explanation(out: Path, args):
    text = r"""# DEM方向敏感扩展实验说明

## 1. 多起终点实验

目的：避免只用一组起终点导致结论偶然化。单一路径可能刚好落在明显沟谷或缓坡通道上，导致 B0、B1、Ours 路径差异不明显。因此需要随机生成多组起终点，统计三模型在不同空间位置下的路径代价、路径长度、绕行率和平均单位距离代价。

基本统计：

$$
J=\sum_i c_{unit}(x_i,y_i,\theta_i)\,l_i
$$

$$
L=\sum_i l_i
$$

$$
\bar c=\frac{J}{L}
$$

$$
DR=\frac{L}{D}
$$

其中，$J$ 为路径总代价，$L$ 为路径长度，$\bar c$ 为平均单位距离代价，$DR$ 为绕行率，$D$ 为起终点直线距离。

阅读重点：

- 若 B1 普遍 $\bar c$ 很低，但路径穿过明显高横坡区，说明仅纵坡模型过度乐观。
- 若 Ours 的 $L$ 或 $DR$ 略高但 $\bar c$ 更稳定，说明模型通过绕行避开高横坡风险。
- 若三模型在多数起终点下都几乎一致，说明当前区域宏观地形通道过强，需要增加横切坡面或跨山脊起终点。

## 2. 起终点互换实验

目的：检验方向代价的非对称性。越野行驶中，从 A 到 B 与从 B 到 A 不一定等价，因为上坡、缓下坡和陡下坡的代价函数不同。

对同一几何路径 $\Gamma$，正向代价为：

$$
J_{A\rightarrow B}(\Gamma)=\sum_i c_{unit}(x_i,y_i,\theta_i)l_i
$$

反向代价为：

$$
J_{B\rightarrow A}(\Gamma^{-1})=\sum_i c_{unit}(x_i,y_i,\theta_i+\pi)l_i
$$

若纵坡代价函数对上坡和下坡不对称，则通常有：

$$
J_{A\rightarrow B}(\Gamma)\ne J_{B\rightarrow A}(\Gamma^{-1})
$$

本实验输出三类量：

1. 正向最优路径 $S\rightarrow G$ 的代价；
2. 反向最优路径 $G\rightarrow S$ 的代价；
3. 将正向几何路径反过来行驶的代价。

阅读重点：

- B0 是标量坡度模型，理论上最接近正反向对称；
- B1 和 Ours 因为有纵坡上/下坡非对称，正反向代价可能不同；
- 如果正反向路径重合率低，说明方向代价不仅改变代价数值，也改变路径选择。

## 3. 邻域敏感性实验

目的：检验路径规划结果是否过度依赖邻域搜索方式。方向敏感代价场本身可以定义多个方向，但 A* 只能沿搜索图允许的边移动。

本代码提供四种邻域：

- 4 邻域：仅上下左右，方向最粗；
- 8 邻域：上下左右加对角线，是基础实验推荐设置；
- 16 邻域：增加斜率 1:2、2:1 的方向；
- 32 邻域：进一步增加更细方向。

对于 16/32 邻域，边会跨越 2 或 3 个像元。为避免跳过中间高代价区域，代码会沿边采样，使用采样点平均单位距离代价计算边代价：

$$
C_{edge}=\left(\frac{1}{m}\sum_{j=1}^{m} c_{unit}(x_j,y_j,\theta)\right)l_\theta
$$

阅读重点：

- 4 邻域路径通常更折线化，可能夸大绕行；
- 8 邻域适合作为主实验；
- 16/32 邻域若与 8 邻域结论一致，说明模型结论较稳定；
- 若 16/32 与 8 差异很大，需要检查方向离散、边采样和转弯约束。

## 4. 当前参数

- $\alpha_u$：上坡阻碍增长尺度，单位度；
- $\alpha_m$：缓下坡容许阈值，单位度；
- $\alpha_d$：陡下坡风险增长尺度，单位度；
- $\alpha_r$：横坡阻碍增长尺度，单位度；
- $\varepsilon$：数值稳定项，只用于防止除零或对数零值，不表示实际地理过程。

当前运行参数由命令行给定，详见 `00_运行参数.txt`。
"""
    with open(out / "00_实验说明_请先看.md", "w", encoding="utf-8") as f:
        f.write(text)
    with open(out / "00_运行参数.txt", "w", encoding="utf-8") as f:
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v}\n")


def main():
    parser = argparse.ArgumentParser(description="DEM方向敏感扩展实验：多起终点、起终点互换、邻域敏感性")
    parser.add_argument("--dem", type=str, default=None, help="真实 DEM GeoTIFF 路径。不传则使用合成 DEM。")
    parser.add_argument("--synthetic", action="store_true", help="强制使用合成 DEM。")
    parser.add_argument("--out", type=str, default="outputs_direction_sensitivity", help="输出目录。")
    parser.add_argument("--max-pixels", type=int, default=1000, help="最大边长像元数；设 0 表示不降采样。")
    parser.add_argument("--cell-size", type=float, default=30.0, help="无 rasterio 时使用的栅格大小，单位 m。")
    parser.add_argument("--start", type=str, default=None, help="互换实验与邻域实验起点 row,col；不填用 0.10 位置。")
    parser.add_argument("--goal", type=str, default=None, help="互换实验与邻域实验终点 row,col；不填用 0.90 位置。")
    parser.add_argument("--edge-buffer", type=int, default=10, help="边界缓冲像元数，防止路径贴边。")
    parser.add_argument("--n-pairs", type=int, default=3, help="多起终点实验的随机起终点组数。1000x1000 上建议先用 10~20。")
    parser.add_argument("--seed", type=int, default=42, help="随机起终点种子。")
    parser.add_argument("--min-distance-ratio", type=float, default=0.35, help="随机起终点最小距离占图幅对角线比例。")
    parser.add_argument("--multi-neighbors", type=int, default=8, choices=[4,8,16,32], help="多起终点实验使用的邻域数，默认 8。")
    parser.add_argument("--reverse-neighbors", type=int, default=8, choices=[4,8,16,32], help="起终点互换实验使用的邻域数，默认 8。")
    parser.add_argument("--neighbors", type=str, default="4,8,16,32", help="邻域敏感性实验列表，例如 4,8,16,32。")
    parser.add_argument("--alpha-u", type=float, default=15.0, help="上坡阻碍尺度，单位度。")
    parser.add_argument("--alpha-m", type=float, default=5.0, help="缓下坡容许尺度，单位度。")
    parser.add_argument("--alpha-d", type=float, default=15.0, help="陡下坡风险尺度，单位度。")
    parser.add_argument("--alpha-r", type=float, default=15.0, help="横坡风险尺度，单位度。")
    parser.add_argument("--skip-multistart", action="store_true", help="跳过多起终点实验。")
    parser.add_argument("--skip-reverse", action="store_true", help="跳过起终点互换实验。")
    parser.add_argument("--skip-neighborhood", action="store_true", help="跳过邻域敏感性实验。")
    args = parser.parse_args()
    args.neighbors = parse_neighbors(args.neighbors)

    t0 = time.time()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    write_explanation(out, args)

    max_pixels = None if args.max_pixels == 0 else args.max_pixels
    if args.synthetic or not args.dem:
        dem, cell_size, meta = generate_synthetic_dem()
    else:
        dem, cell_size, meta = load_dem_geotiff(args.dem, max_pixels=max_pixels, fallback_cell_size=args.cell_size)
    print(f"DEM 行列数: {dem.shape}, 栅格大小: {cell_size:.3f} m")

    rows, cols = dem.shape
    s_default, g_default = default_pair(dem.shape, args.edge_buffer)
    start = parse_pair(args.start) or s_default
    goal = parse_pair(args.goal) or g_default
    print(f"互换/邻域实验起点: {start}, 终点: {goal}")

    p, q = calculate_gradient_tan(dem, cell_size)

    if not args.skip_multistart:
        pairs = generate_endpoint_pairs(dem.shape, args.n_pairs, edge_buffer=args.edge_buffer, min_distance_ratio=args.min_distance_ratio, seed=args.seed)
        run_multistart(out, dem, p, q, cell_size, pairs, args)
    if not args.skip_reverse:
        run_reverse(out, dem, p, q, cell_size, start, goal, args)
    if not args.skip_neighborhood:
        run_neighborhood(out, dem, p, q, cell_size, start, goal, args)

    print(f"\n全部完成。输出目录：{out.resolve()}")
    print("建议先看：00_实验说明_请先看.md，以及三个子文件夹内的 CSV 和 PNG。")
    print(f"总耗时：{time.time()-t0:.2f} s")


if __name__ == "__main__":
    main()
