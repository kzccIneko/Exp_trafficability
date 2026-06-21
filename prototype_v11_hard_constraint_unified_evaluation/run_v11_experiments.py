"""
run_v11_experiments.py

v11：硬约束可达性与统一评价体系。

核心思想：
1. 规划阶段：B0/B1/V9/V10 分别按自身代价场寻找路径。
2. 统一评价阶段：所有路径都放回 V10 空间附着车辆能力模型下重评价。
3. 硬约束阶段：在 A* 搜索中加入 rho_max <= rho_lim 的可行性约束。

注意：所有 total_cost 均为路径相对累计代价 J_plan，不是焦耳 J。
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from run_v10_experiments import (
    write_csv,
    load_input_dem,
    load_optional_rasters,
    nearest_free,
    repair_pairs,
    formula_passport_rows,
    parameter_rows,
)
from cost_model import calculate_gradient_tan, compute_b0_unit_cost_field, compute_b1_unit_cost_field
from direction_utils import get_directions
from vehicle_params import jeep_wrangler_rubicon_4door, write_vehicle_params_csv
from vehicle_capability_model import compute_vehicle_capability_fields
from spatial_surface_params import compute_spatial_surface_params
from roi_selector import select_rois, write_roi_csv, plot_roi_overview
from scenario_pairs import generate_scenario_pairs, write_pairs_csv
from path_planning import astar_directional, evaluate_path_cost, PathResult
from path_metrics import path_capability_profile, summarize_profile, indicator_explanations
from plotting_v10 import plot_direction_cost_maps, plot_paths, plot_profiles, plot_bar_metrics, save_map_grid, plot_surface_params


MODEL_DESCRIPTIONS = [
    {
        "模型方案": "B0_Static_Slope",
        "短名": "B0",
        "中文名称": "B0 静态坡度代价模型",
        "方法含义": "仅使用坡度大小构建静态标量代价，不区分车辆行驶方向。",
        "论文作用": "传统 GIS 坡度成本面基线，用于证明静态标量通行能力图的方向盲区。",
        "规划代价形式": "c_B0(i)=1-ln(1-R_slope(i))",
    },
    {
        "模型方案": "B1_Longitudinal_Directional",
        "短名": "B1",
        "中文名称": "B1 方向纵坡代价模型",
        "方法含义": "只考虑车辆前进方向的上坡/下坡纵坡，不考虑横坡。",
        "论文作用": "验证方向性是否必要，并暴露只看纵坡会诱导横切坡面的问题。",
        "规划代价形式": "c_B1(i,d)=1-ln(1-R_parallel(i,d))",
    },
    {
        "模型方案": "V9_Constant_Adhesion",
        "短名": "V9",
        "中文名称": "V9 常数附着车辆能力利用率模型",
        "方法含义": "考虑上坡牵引、下坡制动、侧翻稳定性、侧滑附着，但 mu_s、mu_b 为空间常数。",
        "论文作用": "车辆能力利用率基线，用于区分车辆约束机制与空间化附着机制。",
        "规划代价形式": "c_V9(i,d)=1+rho_max(i,d), mu_s/mu_b=constant",
    },
    {
        "模型方案": "V10_Free",
        "短名": "V10-Free",
        "中文名称": "V10 空间附着车辆能力利用率模型（软约束）",
        "方法含义": "在 V9 基础上将 mu_s、mu_b 空间化，但 rho_max 超限时仍允许通行并提高代价。",
        "论文作用": "主模型软约束版本，用于表达最小风险压力路径。",
        "规划代价形式": "c_V10(i,d)=1+rho_max(i,d), mu_s/mu_b=field(x,y)",
    },
    {
        "模型方案": "V10_Hard_rho_lim",
        "短名": "V10-Hard",
        "中文名称": "V10 硬约束可达性模型",
        "方法含义": "若 rho_max(i,d)>rho_lim，则该方向边直接视为不可通行。",
        "论文作用": "回答是否存在满足车辆能力阈值的可行路径，用于可达性与战术阈值分析。",
        "规划代价形式": "c=1+rho_max if rho_max<=rho_lim else +inf",
    },
]


def parse_hard_limits(text: str) -> list[float]:
    vals: list[float] = []
    for item in text.replace(";", ",").split(","):
        item = item.strip()
        if item:
            vals.append(float(item))
    return vals


def build_free_fields(dem, p, q, directions, cell_size, vehicle, surface, args):
    """构建不含 rho 硬截断的 B0/B1/V9/V10 代价场。"""
    _, C0 = compute_b0_unit_cost_field(p, q, directions, alpha_u=args.v6_alpha_u)
    _, C1 = compute_b1_unit_cost_field(
        p, q, directions,
        alpha_u=args.v6_alpha_u,
        alpha_m=args.v6_alpha_m,
        alpha_d=args.v6_alpha_d,
    )
    C9, parts9 = compute_vehicle_capability_fields(
        dem, p, q, directions, cell_size, vehicle,
        hard_barrier=surface["hard_barrier"],
        hard_rho=None,
    )
    C10, parts10 = compute_vehicle_capability_fields(
        dem, p, q, directions, cell_size, vehicle,
        mu_slide_field=surface["mu_slide"],
        mu_brake_field=surface["mu_brake"],
        hard_barrier=surface["hard_barrier"],
        hard_rho=None,
    )
    fields = {
        "B0_Static_Slope": C0,
        "B1_Longitudinal_Directional": C1,
        "V9_Constant_Adhesion": C9,
        "V10_Free": C10,
    }
    return fields, C10, parts10, parts9


def plan_one(cost_field, pair, cell_size, directions, args, barrier_mask, hard_rho_limit=None) -> PathResult:
    return astar_directional(
        cost_field,
        pair["start"],
        pair["goal"],
        cell_size,
        directions,
        n_neighbors=args.neighbors,
        barrier_mask=barrier_mask,
        edge_buffer=args.edge_buffer,
        hard_rho_limit=hard_rho_limit,
    )


def summarize_under_v10(model_name, pair, planned: PathResult, v10_field, v10_parts, directions, cell_size, dem) -> dict:
    """将任意模型规划出的路径放回 V10 空间附着车辆能力模型下统一评价。"""
    row = {
        "pair_id": pair["pair_id"],
        "roi_id": pair.get("roi_id", ""),
        "scenario": pair.get("scenario", ""),
        "模型方案": model_name,
        "是否找到路径": bool(planned.path),
        "规划模型内部相对累计代价_Jplan": planned.total_cost,
        "规划路径长度_L_m": planned.path_length_m,
        "规划平均单位距离相对代价": planned.average_cost_per_m,
        "规划绕行率_L除以直线距离": planned.detour_ratio,
        "路径节点数": len(planned.path),
    }
    if not planned.path:
        row.update({
            "V10统一重评价相对累计代价_Jplan": np.inf,
            "V10统一重评价平均单位距离相对代价": np.inf,
        })
        return row

    v10_eval = evaluate_path_cost(v10_field, planned.path, cell_size, directions=directions, n_neighbors=len(directions))
    profile = path_capability_profile(planned.path, dem, v10_field, v10_parts, directions, cell_size)
    row.update({
        "V10统一重评价相对累计代价_Jplan": v10_eval.total_cost,
        "V10统一重评价平均单位距离相对代价": v10_eval.average_cost_per_m,
    })
    row.update(summarize_profile(profile))
    return row


def aggregate_by_model(rows: list[dict]) -> list[dict]:
    models = []
    for r in rows:
        if r["模型方案"] not in models:
            models.append(r["模型方案"])
    metric_keys = [
        "规划模型内部相对累计代价_Jplan",
        "规划路径长度_L_m",
        "规划平均单位距离相对代价",
        "规划绕行率_L除以直线距离",
        "V10统一重评价相对累计代价_Jplan",
        "V10统一重评价平均单位距离相对代价",
        "平均上坡牵引利用率",
        "平均下坡制动利用率",
        "平均侧翻稳定性利用率",
        "平均侧滑附着利用率",
        "平均最大利用率",
        "最大rho_max",
        "P_rho_max_gt_1p0",
        "P_rho_max_gt_0p9",
        "P_rho_slide_gt_0p7",
        "P_rho_roll_gt_0p7",
        "P_rho_up_gt_0p7",
        "P_rho_down_gt_0p7",
        "主导限制_上坡牵引_路径比例",
        "主导限制_下坡制动_路径比例",
        "主导限制_侧翻稳定性_路径比例",
        "主导限制_侧滑附着_路径比例",
    ]
    out = []
    for m in models:
        vals = [r for r in rows if r["模型方案"] == m]
        found = [r for r in vals if r.get("是否找到路径")]
        rec = {"模型方案": m, "样本数": len(vals), "找到路径数": len(found), "路径成功率": len(found) / max(len(vals), 1)}
        for key in metric_keys:
            data = []
            for r in found:
                v = r.get(key, np.nan)
                try:
                    v = float(v)
                except Exception:
                    v = np.nan
                if np.isfinite(v):
                    data.append(v)
            rec[key + "_mean"] = float(np.mean(data)) if data else np.nan
            rec[key + "_std"] = float(np.std(data)) if data else np.nan
        out.append(rec)
    return out


def write_v11_readme(out: Path, meta: dict, hard_limits: list[float]) -> None:
    text = f"""# v11 硬约束与统一评价实验说明

生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}

## 1. 为什么需要 v11

v10 已经形成方向相关车辆能力利用率代价场：

```math
c(i,d)=1+\\rho_{{max}}(i,d)
```

但 B0/B1 的规划代价来自经验阻碍度函数，V9/V10 的规划代价来自车辆能力利用率，二者不能直接比较累计代价大小。因此 v11 将“规划代价”和“统一评价指标”分开：

```math
J_m^{{plan}}(P_m)=\\sum_{{e_k\\in P_m}} c_m(e_k)l(e_k)
```

```math
E_{{V10}}(P_m)=\\left\\{{\\bar\\rho_{{max}}, P(\\rho_{{max}}>1), P(\\rho_{{slide}}>0.7), L, D\\right\\}}
```

## 2. 统一评价

同一组起终点下，先让 B0、B1、V9、V10 各自规划路径，再把所有路径放回 V10 空间附着车辆能力模型中重评价。

核心解释：不是比较不同模型自己的代价谁小，而是比较它们规划出的路径在同一物理评价体系下是否更接近车辆能力边界。

## 3. 硬约束可达性

硬约束搜索使用：

```math
c(i,d)=
\\begin{{cases}}
1+\\rho_{{max}}(i,d), & \\rho_{{max}}(i,d)\\leq \\rho_{{lim}} \\\\
+\\infty, & \\rho_{{max}}(i,d)>\\rho_{{lim}}
\\end{{cases}}
```

本次阈值：`{hard_limits}`。

## 4. 结论边界

- `J_plan` 是路径相对累计代价，不是焦耳 J。
- `P_rho_max_gt_1p0` 是能力超限路径比例，不使用“致命”“死亡”等过度表述。
- OSM 只能作为已有道路选择的弱监督证据，不是车辆通行能力真值。
- TWI 是地形湿润倾向，不是实测含水率。
- GIS 空间化附着参数是工程参数化，不等同于实测摩擦系数或完整 NRMM 土力学模型。

## 5. 数据元信息

```json
{json.dumps(meta, ensure_ascii=False, indent=2, default=str)}
```
"""
    (out / "00_v11实验说明_请先看.md").write_text(text, encoding="utf-8")


def run_v11(args):
    pkg_dir = Path(__file__).resolve().parent
    if not Path(args.lookup_csv).is_absolute():
        args.lookup_csv = str(pkg_dir / args.lookup_csv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    hard_limits = parse_hard_limits(args.hard_limits)
    dem, cell_size, meta = load_input_dem(args)
    landcover, soil, water, raster_metas = load_optional_rasters(args, dem.shape)
    meta["可选栅格元数据"] = raster_metas
    meta["v11说明"] = "硬约束与统一评价；若未提供土地覆盖/土壤/水系，仍使用 v10 的合成/DEM 代理机制。"

    p, q = calculate_gradient_tan(dem, cell_size)
    directions = get_directions(args.neighbors)
    vehicle = jeep_wrangler_rubicon_4door()

    write_v11_readme(out, meta, hard_limits)
    write_csv(out / "00_模型标识说明_B0_B1_V9_V10.csv", MODEL_DESCRIPTIONS)
    write_csv(out / "00_公式护照_逐项来源.csv", formula_passport_rows())
    write_csv(out / "00_参数来源与用途说明.csv", parameter_rows(vehicle, args))
    write_csv(out / "00_路径约束利用率指标说明.csv", indicator_explanations())
    write_vehicle_params_csv(out / "01_公开车辆参数" / "01_车辆参数来源表.csv", vehicle)

    surface = compute_spatial_surface_params(
        dem,
        cell_size,
        args.lookup_csv,
        landcover=landcover,
        soil=soil,
        water_mask=water,
        scenario=args.scenario,
        gamma_slide=args.gamma_slide,
        gamma_brake=args.gamma_brake,
        water_decay_m=args.water_decay_m,
        seed=args.seed,
    )
    exp_surface = out / "02_地表附着参数空间化复核"
    exp_surface.mkdir(parents=True, exist_ok=True)
    plot_surface_params(exp_surface / "01_湿润倾向与附着能力图.png", surface)
    write_csv(exp_surface / "02_地表附着参数元数据.csv", [{k: json.dumps(v, ensure_ascii=False) for k, v in surface["metadata"].items()}])

    fields, v10_field, v10_parts, _ = build_free_fields(dem, p, q, directions, cell_size, vehicle, surface, args)
    exp_dir = out / "03_方向代价与主导限制复核"
    exp_dir.mkdir(parents=True, exist_ok=True)
    plot_direction_cost_maps(exp_dir / "01_方向代价图与主导限制图.png", {"V10空间附着": v10_field}, v10_parts)
    finite_v10 = np.where(np.isfinite(v10_field), v10_field, np.nan)
    write_csv(exp_dir / "02_方向代价统计.csv", [
        {"统计量": "mean_min_cost", "数值": float(np.nanmean(np.nanmin(finite_v10, axis=2)))},
        {"统计量": "mean_max_cost", "数值": float(np.nanmean(np.nanmax(finite_v10, axis=2)))},
        {"统计量": "mean_direction_difference", "数值": float(np.nanmean(np.nanmax(finite_v10, axis=2) - np.nanmin(finite_v10, axis=2)))},
    ])

    # 选 ROI 与起终点
    AI, slope_deg, rois = select_rois(
        dem, p, q, v10_field,
        window_size=args.roi_size,
        stride=max(1, args.roi_size // 2),
        top_k=args.n_rois,
    )
    write_roi_csv(exp_dir / "03_ROI候选区.csv", rois)
    try:
        plot_roi_overview(exp_dir, AI, slope_deg, rois)
    except Exception:
        pass
    pairs = generate_scenario_pairs(
        p, q, rois,
        edge_buffer=args.edge_buffer,
        random_pairs_per_roi=args.n_pairs_per_scenario,
        seed=args.seed,
    )
    pairs = repair_pairs(pairs, v10_parts["hard_barrier"])
    if args.max_pairs is not None:
        pairs = pairs[: max(1, min(args.max_pairs, len(pairs)))]
    write_pairs_csv(exp_dir / "05_情景起终点.csv", pairs)

    exp = out / "04_v11统一评价与硬约束可达性"
    exp.mkdir(parents=True, exist_ok=True)
    all_rows = []
    reach_rows = []

    for pair in pairs:
        pair_dir = exp / f"pair_{int(pair['pair_id']):03d}_{pair['scenario']}"
        pair_dir.mkdir(parents=True, exist_ok=True)
        planned_paths: dict[str, list[tuple[int, int]]] = {}
        profiles = {}
        pair_rows = []

        # 软约束/基线路径
        plan_specs = [(name, field, None) for name, field in fields.items()]
        # 硬约束 V10 路径
        for lim in hard_limits:
            plan_specs.append((f"V10_Hard_rho_{str(lim).replace('.', 'p')}", v10_field, lim))

        for model_name, cost_field, hard_lim in plan_specs:
            print(f"pair {pair['pair_id']} | {model_name} | hard={hard_lim} | {pair['start']} -> {pair['goal']}")
            result = plan_one(
                cost_field,
                pair,
                cell_size,
                directions,
                args,
                barrier_mask=v10_parts["hard_barrier"],
                hard_rho_limit=hard_lim,
            )
            planned_paths[model_name] = result.path
            row = summarize_under_v10(model_name, pair, result, v10_field, v10_parts, directions, cell_size, dem)
            if hard_lim is not None:
                row["硬约束rho_lim"] = hard_lim
            pair_rows.append(row)
            all_rows.append(row)

            if result.path:
                profile = path_capability_profile(result.path, dem, v10_field, v10_parts, directions, cell_size)
                profiles[model_name] = profile
                write_csv(pair_dir / f"profile_{model_name}_V10统一评价.csv", profile)

            if model_name.startswith("V10_Hard"):
                reach_rows.append({
                    "pair_id": pair["pair_id"],
                    "scenario": pair.get("scenario", ""),
                    "rho_lim": hard_lim,
                    "是否找到满足硬约束路径": bool(result.path),
                    "路径相对累计代价_Jplan": result.total_cost,
                    "路径长度_L_m": result.path_length_m,
                    "绕行率_L除以直线距离": result.detour_ratio,
                })

        write_csv(pair_dir / "01_pair_v11统一评价指标.csv", pair_rows)
        try:
            plot_paths(pair_dir / "02_pair路径对比图.png", v10_field, planned_paths, pair["start"], pair["goal"], title=f"v11 pair {pair['pair_id']} 统一评价路径对比")
            plot_profiles(pair_dir / "03_pair沿途能力利用率剖面.png", profiles, title=f"v11 pair {pair['pair_id']} V10统一评价剖面")
            plot_bar_metrics(pair_dir / "04_pair高利用率路径比例柱状图.png", pair_rows, title=f"v11 pair {pair['pair_id']} 高利用率路径比例")
        except Exception as e:
            (pair_dir / "plot_warning.txt").write_text(str(e), encoding="utf-8")

    summary = aggregate_by_model(all_rows)
    write_csv(exp / "00_v11统一评价_逐路径指标.csv", all_rows)
    write_csv(exp / "01_v11统一评价_按模型汇总.csv", summary)
    write_csv(exp / "02_v11硬约束可达性_逐路径指标.csv", reach_rows)

    # 按 rho_lim 汇总硬约束成功率
    reach_summary = []
    for lim in hard_limits:
        vals = [r for r in reach_rows if r["rho_lim"] == lim]
        found = [r for r in vals if r["是否找到满足硬约束路径"]]
        rec = {
            "rho_lim": lim,
            "起终点样本数": len(vals),
            "找到路径数": len(found),
            "硬约束路径成功率": len(found) / max(len(vals), 1),
        }
        for key in ["路径相对累计代价_Jplan", "路径长度_L_m", "绕行率_L除以直线距离"]:
            data = [float(r.get(key, np.nan)) for r in found if np.isfinite(float(r.get(key, np.nan)))]
            rec[key + "_mean"] = float(np.mean(data)) if data else np.nan
        reach_summary.append(rec)
    write_csv(exp / "03_v11硬约束可达性_按阈值汇总.csv", reach_summary)
    try:
        plot_bar_metrics(exp / "04_v11全部路径高利用率比例汇总.png", all_rows, title="v11 统一评价：全部路径高利用率比例")
    except Exception as e:
        (exp / "plot_warning.txt").write_text(str(e), encoding="utf-8")

    print(f"完成：{out}")
    return out


def build_arg_parser():
    ap = argparse.ArgumentParser(description="v11 硬约束与统一评价实验")
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--synthetic-size", type=int, default=220)
    ap.add_argument("--dem")
    ap.add_argument("--dem-path-file", default="my_dem_path.txt")
    ap.add_argument("--landcover")
    ap.add_argument("--soil")
    ap.add_argument("--water")
    ap.add_argument("--out", default="outputs_v11")
    ap.add_argument("--max-pixels", type=int, default=700)
    ap.add_argument("--cell-size", type=float, default=30.0)
    ap.add_argument("--neighbors", type=int, default=8, choices=[4, 8, 16, 32])
    ap.add_argument("--edge-buffer", type=int, default=3)
    ap.add_argument("--roi-size", type=int, default=180)
    ap.add_argument("--n-rois", type=int, default=2)
    ap.add_argument("--n-pairs-per-scenario", type=int, default=1)
    ap.add_argument("--max-pairs", type=int, default=None)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--scenario", default="normal", choices=["dry", "normal", "wet", "post_rain"])
    ap.add_argument("--gamma-slide", type=float, default=0.5)
    ap.add_argument("--gamma-brake", type=float, default=0.5)
    ap.add_argument("--water-decay-m", type=float, default=90.0)
    ap.add_argument("--hard-limits", default="1.0,1.2,1.5", help="逗号分隔的 rho_lim，例如 1.0,1.2,1.5")
    ap.add_argument("--lookup-csv", default="config/landcover_mu_lookup.csv")
    ap.add_argument("--v6-alpha-u", type=float, default=15)
    ap.add_argument("--v6-alpha-m", type=float, default=5)
    ap.add_argument("--v6-alpha-d", type=float, default=15)
    # 兼容 parameter_rows 的字段访问；v11 不直接使用 hard_rho。
    ap.set_defaults(hard_rho=None)
    return ap


if __name__ == "__main__":
    run_v11(build_arg_parser().parse_args())
