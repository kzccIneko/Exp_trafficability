"""
run_v6_experiments.py

DEM-only v6 实验闭环：
1. ROI 方向异质性自动筛选；
2. 方向代价场验证；
3. 情景起终点路径实验 + 风险暴露剖面统计；
4. 正反向非对称实验；
5. 邻域鲁棒性实验；
6. 参数敏感性实验；
7. OSM 弱监督验证接口说明。

当前阶段证明目标：
- 不是证明时间最短或能耗最低；
- 而是证明方向敏感通行代价模型能更合理表达横坡、陡下坡、方向非对称等危险地形暴露；
- 在路径长度增加有限的情况下，降低危险暴露比例。
"""
from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
import numpy as np
import warnings
warnings.filterwarnings("ignore", message="Glyph .* missing from font.*")
warnings.filterwarnings("ignore", category=SyntaxWarning)

from cost_model import (
    calculate_gradient_tan,
    calculate_slope_angle,
    compute_b0_unit_cost_field,
    compute_b1_unit_cost_field,
    compute_b2_unit_cost_field,
    compute_ours_unit_cost_field,
)
from direction_utils import get_directions, describe_moves
from path_planning import astar_directional, evaluate_path_cost, path_cell_iou, PathResult
from roi_selector import select_rois, write_roi_csv, plot_roi_overview
from scenario_pairs import generate_scenario_pairs, write_pairs_csv
from exposure_metrics import path_profile, summarize_profile, metric_explanation_rows
from profile_plots import (
    plot_cost_field_comparison,
    plot_paths_global_and_zoom,
    plot_profile_curves,
    plot_exposure_bars,
    plot_tradeoff_scatter,
    plot_parameter_sensitivity,
)
from metrics import anisotropy_index, summarize_array
from run_single_experiment import generate_synthetic_dem, load_dem_geotiff

MODEL_ORDER_MAIN = ["B0", "B1", "B2", "Ours"]
MODEL_ORDER_PATH = ["B0", "B1", "Ours"]
MODEL_CN = {
    "B0": "B0 静态坡度模型",
    "B1": "B1 仅纵坡方向模型",
    "B2": "B2 仅横坡方向模型",
    "Ours": "Ours 纵坡+横坡非补偿模型",
}


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fields: list[str] = []
    for row in rows:
        for k in row.keys():
            if k not in fields:
                fields.append(k)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_neighbors(text: str) -> list[int]:
    vals = [int(x.strip()) for x in text.split(",") if x.strip()]
    for v in vals:
        if v not in (4, 8, 16, 32):
            raise ValueError("邻域数只能为 4, 8, 16, 32")
    return vals


def parse_float_list(text: str) -> list[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def compute_cost_fields(p, q, directions, args):
    _, C0 = compute_b0_unit_cost_field(p, q, directions, alpha_u=args.alpha_u)
    _, C1 = compute_b1_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d)
    _, C2 = compute_b2_unit_cost_field(p, q, directions, alpha_r=args.alpha_r)
    _, Co, parts = compute_ours_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d, alpha_r=args.alpha_r)
    return {"B0": C0, "B1": C1, "B2": C2, "Ours": Co}, parts


def path_row_common(exp, pair, model, n_neighbors, result: PathResult):
    s = pair["start"]
    g = pair["goal"]
    return {
        "实验": exp,
        "pair_id": pair["pair_id"],
        "roi_id": pair["roi_id"],
        "scenario": pair["scenario"],
        "模型": MODEL_CN.get(model, model),
        "模型代码": model,
        "邻域数": n_neighbors,
        "起点_row": s[0],
        "起点_col": s[1],
        "终点_row": g[0],
        "终点_col": g[1],
        "路径总代价_J": result.total_cost,
        "路径长度_L_m": result.path_length_m,
        "平均单位距离代价_J除以L": result.average_cost_per_m,
        "绕行率_L除以D": result.detour_ratio,
        "路径节点数": len(result.path),
        "是否找到路径": bool(result.path),
    }


def save_profile_csv(path: Path, rows: list[dict]) -> None:
    write_csv(path, rows)


def run_roi_selection(out, dem, p, q, cost_fields, args):
    exp_dir = out / "01_ROI方向异质性筛选"
    exp_dir.mkdir(parents=True, exist_ok=True)
    AI, slope_deg, rois = select_rois(
        dem, p, q, cost_fields["Ours"],
        window_size=args.roi_window,
        stride=args.roi_stride,
        top_k=args.roi_top_k,
        min_mean_slope_deg=args.roi_min_slope,
    )
    write_roi_csv(exp_dir / "01_ROI候选区.csv", rois)
    plot_roi_overview(exp_dir, AI, slope_deg, rois)
    # 全区统计，证明 B0 无方向差异、B1/Ours 有方向差异。
    rows = []
    for model, C in cost_fields.items():
        ai = anisotropy_index(C, method="percentile")
        for k, v in summarize_array(f"AI_{model}", ai).items():
            pass
        rec = {"模型": MODEL_CN.get(model, model), "模型代码": model}
        rec.update({f"AI_{k}": v for k, v in summarize_array("AI", ai).items() if k != "name"})
        minc = np.nanmin(C, axis=2)
        rec.update({f"min_cost_{k}": v for k, v in summarize_array("min_cost", minc).items() if k != "name"})
        rows.append(rec)
    write_csv(exp_dir / "02_方向异质性统计.csv", rows)
    return rois, AI, slope_deg


def run_direction_field_validation(out, dem, cost_fields):
    exp_dir = out / "02_方向代价场验证"
    exp_dir.mkdir(parents=True, exist_ok=True)
    plot_cost_field_comparison(exp_dir / "01_B0_B1_B2_Ours_最小方向单位代价.png", dem, cost_fields, title="方向代价场验证：不同模型的最小方向单位代价")
    rows = []
    for model, C in cost_fields.items():
        AI = anisotropy_index(C, method="percentile")
        rec = {"模型代码": model, "模型": MODEL_CN.get(model, model)}
        rec.update({f"AI_{k}": v for k, v in summarize_array("AI", AI).items() if k != "name"})
        rec.update({f"unit_cost_min_direction_{k}": v for k, v in summarize_array("minC", np.nanmin(C, axis=2)).items() if k != "name"})
        rows.append(rec)
    write_csv(exp_dir / "02_方向代价场统计.csv", rows)


def run_scenario_path_experiment(out, dem, p, q, cell_size, directions, cost_fields, pairs, args):
    exp_dir = out / "03_情景路径与风险暴露实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    all_rows: list[dict] = []
    # 为了图表可读，逐 pair 输出路径图和剖面图。
    for pair in pairs:
        pair_dir = exp_dir / f"pair_{int(pair['pair_id']):03d}_{pair['scenario']}"
        pair_dir.mkdir(parents=True, exist_ok=True)
        paths: dict[str, list[tuple[int, int]]] = {}
        profiles_for_plot: dict[str, list[dict]] = {}
        profile_summaries: list[dict] = []
        start, goal = pair["start"], pair["goal"]
        print(f"  情景路径 pair {pair['pair_id']} ROI{pair['roi_id']} {pair['scenario']}: {start}->{goal}")
        for model in MODEL_ORDER_PATH:
            C = cost_fields[model]
            res = astar_directional(C, start, goal, cell_size, directions, n_neighbors=args.main_neighbors, edge_buffer=args.edge_buffer)
            paths[model] = res.path
            base = path_row_common("情景路径风险暴露", pair, model, args.main_neighbors, res)
            prof = path_profile(res.path, dem, p, q, C, directions, cell_size) if res.path else []
            summ = summarize_profile(prof)
            base.update(summ)
            all_rows.append(base)
            profile_summaries.append(base)
            profiles_for_plot[model] = prof
            save_profile_csv(pair_dir / f"profile_{model}.csv", prof)
        write_csv(pair_dir / "01_pair路径风险暴露指标.csv", profile_summaries)
        plot_paths_global_and_zoom(pair_dir / "02_路径全局与局部放大.png", cost_fields["Ours"], paths, start, goal, title=f"pair {pair['pair_id']} {pair['scenario']}：路径全局与局部放大")
        plot_profile_curves(pair_dir / "03_沿途高程_纵坡_横坡_单位代价剖面.png", profiles_for_plot, title=f"pair {pair['pair_id']} {pair['scenario']}：沿途参数剖面")
        plot_exposure_bars(pair_dir / "04_危险暴露比例柱状图.png", profile_summaries, title=f"pair {pair['pair_id']}：危险暴露比例")
    write_csv(exp_dir / "00_全部情景路径风险暴露指标.csv", all_rows)
    # 汇总图：横坡暴露与路径长度权衡。
    plot_exposure_bars(exp_dir / "01_全部pair_平均危险暴露比例.png", all_rows, title="全部情景路径：平均危险暴露比例")
    plot_tradeoff_scatter(exp_dir / "02_路径长度_横坡风险权衡散点图.png", all_rows, title="路径长度—横坡风险权衡：Ours 不追求最短，而是降低危险暴露")
    return all_rows


def run_reverse_experiment(out, dem, p, q, cell_size, directions, cost_fields, pair, args):
    exp_dir = out / "04_正反向非对称实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    paths_for_plot = {}
    profiles_for_plot = {}
    start, goal = pair["start"], pair["goal"]
    reverse_pair = dict(pair)
    reverse_pair["start"] = goal
    reverse_pair["goal"] = start
    reverse_pair["scenario"] = str(pair["scenario"]) + "_reverse"
    print(f"  正反向实验使用 pair {pair['pair_id']} {pair['scenario']}: {start}->{goal}")
    for model in MODEL_ORDER_PATH:
        C = cost_fields[model]
        fwd = astar_directional(C, start, goal, cell_size, directions, n_neighbors=args.main_neighbors, edge_buffer=args.edge_buffer)
        rev = astar_directional(C, goal, start, cell_size, directions, n_neighbors=args.main_neighbors, edge_buffer=args.edge_buffer)
        same_geom_rev = evaluate_path_cost(C, list(reversed(fwd.path)), cell_size, directions, n_neighbors=args.main_neighbors) if fwd.path else PathResult([], np.nan, np.nan, np.nan, np.nan)
        iou = path_cell_iou(fwd.path, rev.path)
        prof_fwd = path_profile(fwd.path, dem, p, q, C, directions, cell_size) if fwd.path else []
        prof_rev = path_profile(rev.path, dem, p, q, C, directions, cell_size) if rev.path else []
        rec = {
            "模型": MODEL_CN[model],
            "模型代码": model,
            "pair_id": pair["pair_id"],
            "scenario": pair["scenario"],
            "正向总代价_J": fwd.total_cost,
            "反向最优总代价_J": rev.total_cost,
            "正向几何路径反向行驶代价_J": same_geom_rev.total_cost,
            "正向路径长度_m": fwd.path_length_m,
            "反向路径长度_m": rev.path_length_m,
            "正反向路径单元重合率_IoU": iou,
            "同一几何反向代价变化率": (same_geom_rev.total_cost - fwd.total_cost) / max(fwd.total_cost, 1e-12) if np.isfinite(fwd.total_cost) else np.nan,
        }
        rec.update({"正向_" + k: v for k, v in summarize_profile(prof_fwd).items()})
        rec.update({"反向_" + k: v for k, v in summarize_profile(prof_rev).items()})
        rows.append(rec)
        paths_for_plot[f"{model}_S到G"] = fwd.path
        paths_for_plot[f"{model}_G到S"] = rev.path
        profiles_for_plot[f"{model}_S到G"] = prof_fwd
        profiles_for_plot[f"{model}_G到S"] = prof_rev
    write_csv(exp_dir / "01_正反向非对称指标.csv", rows)
    plot_paths_global_and_zoom(exp_dir / "02_正反向路径全局与局部放大.png", cost_fields["Ours"], paths_for_plot, start, goal, title="正反向非对称：路径和代价值可能均不对称")
    plot_profile_curves(exp_dir / "03_正反向沿途剖面.png", profiles_for_plot, title="正反向沿途剖面：上坡与下坡遇到方式不同")
    return rows


def run_neighborhood_experiment(out, dem, p, q, cell_size, pair, args):
    exp_dir = out / "05_邻域鲁棒性实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    move_rows = []
    start, goal = pair["start"], pair["goal"]
    for n in args.neighbors:
        directions = get_directions(n)
        fields, _ = compute_cost_fields(p, q, directions, args)
        for move in describe_moves(n):
            move["邻域数"] = n
            move_rows.append(move)
        for model in MODEL_ORDER_PATH:
            C = fields[model]
            res = astar_directional(C, start, goal, cell_size, directions, n_neighbors=n, edge_buffer=args.edge_buffer)
            prof = path_profile(res.path, dem, p, q, C, directions, cell_size) if res.path else []
            rec = path_row_common("邻域鲁棒性", pair, model, n, res)
            rec.update(summarize_profile(prof))
            rows.append(rec)
    write_csv(exp_dir / "01_邻域鲁棒性_路径与暴露指标.csv", rows)
    write_csv(exp_dir / "02_邻域方向集合说明.csv", move_rows)
    plot_exposure_bars(exp_dir / "03_邻域鲁棒性_危险暴露比例.png", rows, title="邻域鲁棒性：不同邻域下危险暴露比例")
    plot_tradeoff_scatter(exp_dir / "04_邻域鲁棒性_长度风险权衡.png", rows, title="邻域鲁棒性：路径长度—横坡风险权衡")
    return rows


def run_parameter_sensitivity(out, dem, p, q, cell_size, base_pairs, args):
    exp_dir = out / "06_参数敏感性实验"
    exp_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    directions = get_directions(args.main_neighbors)
    pairs = base_pairs[:args.sensitivity_pairs]
    for alpha_r in args.alpha_r_values:
        for pair in pairs:
            _, Co, _ = compute_ours_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d, alpha_r=alpha_r)
            start, goal = pair["start"], pair["goal"]
            res = astar_directional(Co, start, goal, cell_size, directions, n_neighbors=args.main_neighbors, edge_buffer=args.edge_buffer)
            prof = path_profile(res.path, dem, p, q, Co, directions, cell_size) if res.path else []
            rec = path_row_common("alpha_r敏感性", pair, "Ours", args.main_neighbors, res)
            rec["alpha_r"] = alpha_r
            rec.update(summarize_profile(prof))
            rows.append(rec)
    write_csv(exp_dir / "01_alpha_r敏感性_逐路径指标.csv", rows)
    plot_parameter_sensitivity(
        exp_dir / "02_alpha_r敏感性_指标折线图.png",
        rows,
        "alpha_r",
        [
            ("cross_exposure_ratio_gt_10p0deg", "横坡>10°暴露比例"),
            ("路径长度_L_m", "路径长度 / m"),
            ("mean_unit_cost_weighted", "平均单位距离代价"),
        ],
        "横坡尺度参数 alpha_r 敏感性：风险下降与绕行代价之间的权衡",
    )
    return rows


def write_markdown_explanation(out: Path, args, dem_meta: dict, rois_count: int, pair_count: int):
    text = fr"""# DEM-only v6 方向敏感通行能力实验说明

## 0. 本版实验要证明什么

本实验包不证明“路径时间最短”或“车辆能耗最低”，因为当前没有车辆速度、油耗、动力学和实车轨迹数据。本版证明的是：

> 方向敏感 DEM 通行代价模型比传统静态坡度模型更能表达越野通行中的方向异质性、横坡风险、上/下坡非对称性，并能在路径长度增加有限的条件下降低危险地形暴露。

因此主指标不是单纯路径长度，而是：

- 风险加权距离；
- 平均单位距离代价；
- 上坡/下坡/近似平坡数量和长度比例；
- 横坡角均值、90% 分位数、最大值；
- 横坡超过 8°/10°/12°/15° 的路径长度占比；
- 下坡超过 5°/10°/15° 的路径长度占比；
- 路径曲折度和绕行率。

## 1. 风险暴露指标是什么

路径由一系列边 $e$ 组成。每条边都有长度 $l_e$、移动方向 $d_e$、纵坡角 $\alpha_{{\parallel,e}}$、横坡角 $\alpha_{{\perp,e}}$ 和单位距离代价 $c_e$。

路径风险加权距离为：

$$
J(\Gamma)=\sum_{{e\in\Gamma}} c_e l_e
$$

其中单位距离代价定义为：

$$
c_e=1-\ln(P_e+\varepsilon)
$$

这里的常数 1 表示基础距离代价；当地形无阻碍时，$J$ 退化为几何路径长度；当地形阻碍增大时，$J$ 成为风险加权距离。它不是时间，也不是能耗，而是相对通行阻碍总代价。

高横坡暴露比例定义为：

$$
E_{{\perp}}^{{>\tau}}=
\frac{{\sum_{{e\in\Gamma}} l_e I(\alpha_{{\perp,e}}>\tau)}}{{\sum_{{e\in\Gamma}} l_e}}
$$

陡下坡暴露比例定义为：

$$
E_{{down}}^{{>\tau}}=
\frac{{\sum_{{e\in\Gamma}} l_e I(\alpha_{{\parallel,e}}<-\tau)}}{{\sum_{{e\in\Gamma}} l_e}}
$$

上坡、下坡、近似平坡数量的默认划分为：

$$
\alpha_{{\parallel}}>1^\circ \Rightarrow \text{{上坡}}
$$

$$
\alpha_{{\parallel}}<-1^\circ \Rightarrow \text{{下坡}}
$$

$$
|\alpha_{{\parallel}}|\le 1^\circ \Rightarrow \text{{近似平坡}}
$$

所有路径都会输出逐边 profile CSV，包含沿途高程、纵坡角、横坡角、单位距离代价和边代价。剖面图展示的是这些逐边参数随累计距离的变化。

## 2. 为什么做 ROI 自动筛选

上一版在 1000×1000 区域上直接画路径，几条路径很容易在宏观尺度上重合，肉眼看不出差别。本版先计算方向各向异性指数：

$$
AI(i)=\frac{{Q_{{90}}(c(i,d))-Q_{{10}}(c(i,d))}}{{Q_{{90}}(c(i,d))+Q_{{10}}(c(i,d))+\varepsilon}}
$$

如果一个区域 $AI$ 高，说明同一位置不同方向的代价差异明显，适合验证方向敏感模型。本次自动筛选 ROI 数：{rois_count}，情景起终点数：{pair_count}。

## 3. 参数敏感性范围为什么这样设定

### 3.1 参数不是车辆极限，而是风险增长尺度

$\alpha_u$、$\alpha_d$、$\alpha_r$ 不是车辆最大可通行坡度，而是阻碍度函数的增长尺度。阻碍度采用：

$$
R=1-\exp\left[-\left(\frac{{\alpha}}{{\alpha_0}}\right)^2\right]
$$

当 $\alpha=\alpha_0$ 时：

$$
R=1-e^{{-1}}\approx0.632
$$

也就是说，$\alpha_0$ 表示阻碍度进入明显增强区间的尺度，而不是“超过就不能通行”的硬阈值。

### 3.2 横坡参数 $\alpha_r$

本版默认测试：

$$
\alpha_r\in\{{8^\circ,10^\circ,12^\circ,15^\circ,20^\circ\}}
$$

理由是：

1. $8^\circ$–$10^\circ$ 代表较保守的横坡风险响应，模型会较早惩罚横坡；
2. $12^\circ$–$15^\circ$ 代表中等风险响应，适合作为默认探索区间；
3. $20^\circ$ 代表较宽松设置，用于检验模型是否只有在强惩罚下才有效。

因为当前没有具体车辆轮距、重心高度、轮胎—地面附着参数，所以不能把某一个角度写成绝对安全阈值。正确表述是：

> 参数范围覆盖了从保守到宽松的横坡风险增长尺度，用敏感性分析检验结论是否依赖单一参数。

### 3.3 上坡与下坡参数

默认值为：

$$
\alpha_u=15^\circ,\quad \alpha_m=5^\circ,\quad \alpha_d=15^\circ
$$

其中 $\alpha_m=5^\circ$ 表示缓下坡容许尺度。下坡角绝对值小于该值时，不强行惩罚；超过后开始按陡下坡风险增长。这个设计用于表达：缓下坡可能不增加阻碍，但陡下坡会带来制动、滑移和失稳风险。

## 4. 输出目录说明

```text
outputs_v6/
01_ROI方向异质性筛选/
02_方向代价场验证/
03_情景路径与风险暴露实验/
04_正反向非对称实验/
05_邻域鲁棒性实验/
06_参数敏感性实验/
07_OSM弱监督验证接口说明/
```

## 5. OSM 弱监督验证下一步怎么接

本版只预留 `osm_validation.py`。下一阶段需要提供 OSM 道路矢量文件，建议格式为 `.shp`、`.geojson` 或 `.gpkg`。后续代码应：

1. 提取 highway=track/path/unclassified/service/tertiary 等道路；
2. 栅格化为道路正样本 $S^+$；
3. 在相同坡度带、高程带、土地覆盖背景中抽取非道路负样本 $S^-$；
4. 计算 $\Delta C=\bar C(S^-)-\bar C(S^+)$；
5. 计算 $AUC=P(C_{{road}}<C_{{random}})$；
6. 做空间分块验证，避免只在局部区域拟合。

## 6. DEM 元数据

```text
{dem_meta}
```

## 7. 当前运行参数

```text
{vars(args)}
```
"""
    (out / "00_实验说明_请先看.md").write_text(text, encoding="utf-8")
    theory_path = Path("docs") / "实验公式推导_参数敏感性_汇报说明.md"
    if theory_path.exists():
        (out / "00_公式推导_参数敏感性_汇报说明.md").write_text(theory_path.read_text(encoding="utf-8"), encoding="utf-8")
    write_csv(out / "00_风险暴露指标解释.csv", metric_explanation_rows())
    with (out / "00_运行参数.txt").open("w", encoding="utf-8") as f:
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v}\n")


def write_osm_interface_note(out: Path):
    exp_dir = out / "07_OSM弱监督验证接口说明"
    exp_dir.mkdir(parents=True, exist_ok=True)
    text = """# OSM 弱监督验证接口说明

本版 v6 是 DEM-only 实验包，不强行运行 OSM 验证，避免因为缺少道路矢量数据导致主实验无法闭环。

下一次可在 `osm_validation.py` 基础上实现：

1. 读取 OSM 道路矢量文件，例如 `.shp`、`.geojson`、`.gpkg`；
2. 筛选道路类型：`highway=track/path/unclassified/service/tertiary` 等；
3. 根据 DEM 栅格范围和分辨率栅格化道路；
4. 道路缓冲区作为正样本 `S+`；
5. 在相同坡度带、高程带、土地覆盖背景下抽取非道路负样本 `S-`；
6. 用模型的最小方向代价或方向平均代价评价道路与非道路样本；
7. 输出 `Delta C = mean(C-) - mean(C+)` 与 `AUC=P(Croad<Crandom)`。

注意：OSM 不是越野车辆真实通行能力的绝对真值，只是长期人类/车辆通行选择的弱证据。
"""
    (exp_dir / "README_OSM下一步接口说明.md").write_text(text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="DEM-only v6 方向敏感通行能力实验闭环")
    parser.add_argument("--dem", type=str, default=None, help="真实 DEM GeoTIFF 路径；不填则使用合成 DEM。")
    parser.add_argument("--dem-config", type=str, default=None, help="从 UTF-8 文本文件读取 DEM 路径；用于避免 Windows .bat 直接解析中文路径。")
    parser.add_argument("--synthetic", action="store_true", help="强制使用合成 DEM。")
    parser.add_argument("--synthetic-size", type=int, default=220, help="合成 DEM 行列数，用于本地快速测试。")
    parser.add_argument("--out", type=str, default="outputs_v6", help="输出目录。")
    parser.add_argument("--max-pixels", type=int, default=1000, help="最大边长像元数；0 表示不降采样。")
    parser.add_argument("--cell-size", type=float, default=30.0, help="无 rasterio 时使用的栅格大小，单位 m。")
    parser.add_argument("--main-neighbors", type=int, default=8, choices=[4, 8, 16, 32], help="主实验邻域，建议 8。")
    parser.add_argument("--neighbors", type=str, default="4,8,16", help="邻域鲁棒性实验邻域列表；真实 1000x1000 上 32 可能较慢。")
    parser.add_argument("--edge-buffer", type=int, default=10, help="路径规划边界缓冲，防止贴边。")
    parser.add_argument("--roi-window", type=int, default=240, help="ROI 窗口大小，像元。")
    parser.add_argument("--roi-stride", type=int, default=120, help="ROI 滑动步长，像元。")
    parser.add_argument("--roi-top-k", type=int, default=3, help="选取 ROI 数。")
    parser.add_argument("--roi-min-slope", type=float, default=2.0, help="ROI 平均坡度最小值，度。")
    parser.add_argument("--random-pairs-per-roi", type=int, default=1, help="每个 ROI 附加随机起终点数量。")
    parser.add_argument("--seed", type=int, default=42, help="随机种子。")
    parser.add_argument("--alpha-u", type=float, default=15.0, help="上坡阻碍增长尺度，度。")
    parser.add_argument("--alpha-m", type=float, default=5.0, help="缓下坡容许尺度，度。")
    parser.add_argument("--alpha-d", type=float, default=15.0, help="陡下坡风险增长尺度，度。")
    parser.add_argument("--alpha-r", type=float, default=15.0, help="横坡风险增长尺度，度。")
    parser.add_argument("--alpha-r-values", type=str, default="8,10,12,15,20", help="横坡参数敏感性列表，度。")
    parser.add_argument("--sensitivity-pairs", type=int, default=4, help="参数敏感性使用前几组 pair。")
    parser.add_argument("--skip-path-plots", action="store_true", help="保留选项，当前未启用；大图慢时可后续扩展。")
    args = parser.parse_args()
    if args.dem_config:
        cfg_path = Path(args.dem_config)
        if not cfg_path.exists():
            raise FileNotFoundError(f"DEM 路径配置文件不存在: {cfg_path}")
        dem_lines = [line.strip().strip('\"').strip("'") for line in cfg_path.read_text(encoding="utf-8-sig").splitlines() if line.strip() and not line.strip().startswith("#")]
        if not dem_lines:
            raise ValueError(f"DEM 路径配置文件为空: {cfg_path}")
        args.dem = dem_lines[0]
    args.neighbors = parse_neighbors(args.neighbors)
    args.alpha_r_values = parse_float_list(args.alpha_r_values)

    t0 = time.time()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    max_pixels = None if args.max_pixels == 0 else args.max_pixels
    if args.synthetic or not args.dem:
        dem, cell_size, meta = generate_synthetic_dem(rows=args.synthetic_size, cols=args.synthetic_size, cell_size=args.cell_size, seed=args.seed)
    else:
        dem, cell_size, meta = load_dem_geotiff(args.dem, max_pixels=max_pixels, fallback_cell_size=args.cell_size)
    print(f"DEM shape={dem.shape}, cell_size={cell_size:.3f}m")

    p, q = calculate_gradient_tan(dem, cell_size)
    directions = get_directions(args.main_neighbors)
    cost_fields, parts = compute_cost_fields(p, q, directions, args)

    print("[1/7] ROI 筛选")
    rois, AI, slope_deg = run_roi_selection(out, dem, p, q, cost_fields, args)

    print("[2/7] 情景起终点生成")
    pairs = generate_scenario_pairs(p, q, rois, edge_buffer=args.edge_buffer, random_pairs_per_roi=args.random_pairs_per_roi, seed=args.seed)
    write_pairs_csv(out / "01_ROI方向异质性筛选" / "03_情景起终点.csv", pairs)

    print("[3/7] 方向代价场验证")
    run_direction_field_validation(out, dem, cost_fields)

    print("[4/7] 情景路径与风险暴露实验")
    scenario_rows = run_scenario_path_experiment(out, dem, p, q, cell_size, directions, cost_fields, pairs, args)

    print("[5/7] 正反向非对称实验")
    # 优先选择 upslope_downslope 情景；没有则用第一组。
    reverse_pair = next((x for x in pairs if x["scenario"] == "upslope_downslope"), pairs[0])
    reverse_rows = run_reverse_experiment(out, dem, p, q, cell_size, directions, cost_fields, reverse_pair, args)

    print("[6/7] 邻域鲁棒性实验")
    neigh_pair = pairs[0]
    neigh_rows = run_neighborhood_experiment(out, dem, p, q, cell_size, neigh_pair, args)

    print("[7/7] 参数敏感性实验与 OSM 接口说明")
    sens_rows = run_parameter_sensitivity(out, dem, p, q, cell_size, pairs, args)
    write_osm_interface_note(out)

    write_markdown_explanation(out, args, meta, len(rois), len(pairs))
    print(f"完成：{out.resolve()}")
    print(f"总耗时：{time.time()-t0:.2f}s")


if __name__ == "__main__":
    main()
