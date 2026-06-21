"""
run_v7_vehicle_experiments.py

v7 完整实验闭环：参数化车辆约束的方向通行能力模型（VP-DTM）。

主要输出：
1. 公开车辆参数与假设参数表；
2. 单约束物理一致性曲线；
3. B0/B1/v6经验模型/v7车辆约束模型的代价场对比；
4. 情景起终点路径实验与车辆能力利用率剖面；
5. 车辆参数敏感性实验；
6. OSM 弱监督验证接口说明。
"""
from __future__ import annotations

import argparse, csv, json, shutil, time, warnings
from pathlib import Path
import numpy as np
warnings.filterwarnings("ignore", message="Glyph .* missing from font.*")

from cost_model import calculate_gradient_tan, compute_b0_unit_cost_field, compute_b1_unit_cost_field, compute_ours_unit_cost_field
from direction_utils import get_directions
from path_planning import astar_directional, PathResult
from run_single_experiment import generate_synthetic_dem, load_dem_geotiff
from roi_selector import select_rois, write_roi_csv, plot_roi_overview
from scenario_pairs import generate_scenario_pairs, write_pairs_csv
from vehicle_params import jeep_wrangler_rubicon_4door, write_vehicle_params_csv, vehicle_parameter_rows
from vehicle_constraint_model import compute_vehicle_constraint_fields, compute_constraint_curves
from v7_profile_metrics import path_vehicle_profile, summarize_vehicle_profile
from v7_plots import plot_constraint_curves, plot_cost_maps, plot_paths, plot_vehicle_profiles, plot_exposure_bars

MODEL_LABELS = {
    "B0": "B0 静态坡度",
    "B1": "B1 方向纵坡",
    "V6": "V6 经验纵横坡",
    "V7": "V7 车辆约束",
}


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fields = []
    for r in rows:
        for k in r:
            if k not in fields:
                fields.append(k)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader(); w.writerows(rows)


def read_dem_path_file(path_file: str | Path) -> str:
    p = Path(path_file)
    return p.read_text(encoding="utf-8-sig").strip().strip('"')


def load_input_dem(args):
    if args.synthetic:
        return generate_synthetic_dem(rows=args.synthetic_size, cols=args.synthetic_size, cell_size=args.cell_size, seed=args.seed)
    dem_path = args.dem or (read_dem_path_file(args.dem_path_file) if args.dem_path_file else None)
    if not dem_path:
        raise ValueError("真实 DEM 模式需要 --dem 或 --dem-path-file")
    return load_dem_geotiff(dem_path, max_pixels=args.max_pixels, fallback_cell_size=args.cell_size)


def path_common_row(model: str, pair: dict, result: PathResult) -> dict:
    return {
        "pair_id": pair["pair_id"],
        "roi_id": pair["roi_id"],
        "scenario": pair["scenario"],
        "模型代码": model,
        "模型名称": MODEL_LABELS.get(model, model),
        "起点_row": pair["start"][0],
        "起点_col": pair["start"][1],
        "终点_row": pair["goal"][0],
        "终点_col": pair["goal"][1],
        "是否找到路径": bool(result.path),
        "路径总代价_J": result.total_cost,
        "路径长度_L_m": result.path_length_m,
        "平均单位距离代价_J除以L": result.average_cost_per_m,
        "绕行率_L除以直线距离": result.detour_ratio,
        "路径节点数": len(result.path),
    }


def build_fields(dem, p, q, directions, cell_size, vehicle, args):
    _, C0 = compute_b0_unit_cost_field(p, q, directions, alpha_u=args.v6_alpha_u)
    _, C1 = compute_b1_unit_cost_field(p, q, directions, alpha_u=args.v6_alpha_u, alpha_m=args.v6_alpha_m, alpha_d=args.v6_alpha_d)
    _, C6, _ = compute_ours_unit_cost_field(p, q, directions, alpha_u=args.v6_alpha_u, alpha_m=args.v6_alpha_m, alpha_d=args.v6_alpha_d, alpha_r=args.v6_alpha_r)
    C7, parts = compute_vehicle_constraint_fields(dem, p, q, directions, cell_size, vehicle, enable_break=not args.disable_break, hard_rho=args.hard_rho)
    return {"B0": C0, "B1": C1, "V6": C6, "V7": C7}, parts


def copy_docs_to_output(pkg_dir: Path, out: Path):
    docs_src = pkg_dir / "docs_v7"
    if docs_src.exists():
        dst = out / "02_公式推导与参数来源"
        dst.mkdir(parents=True, exist_ok=True)
        for f in docs_src.iterdir():
            if f.is_file():
                shutil.copy2(f, dst / f.name)


def run_physical_consistency(out: Path, vehicle):
    exp = out / "03_物理一致性合成实验"
    exp.mkdir(parents=True, exist_ok=True)
    curves = compute_constraint_curves(vehicle)
    rows = []
    for i in range(len(curves["degree"])):
        rows.append({k: float(v[i]) for k, v in curves.items()})
    write_csv(exp / "01_单约束能力利用率曲线数据.csv", rows)
    plot_constraint_curves(exp / "02_单约束能力利用率曲线.png", curves, title="Jeep 参数化车辆约束：单约束物理一致性曲线")


def run_path_experiments(out, dem, p, q, cell_size, directions, fields, parts, pairs, args):
    exp = out / "05_路径风险暴露实验"
    exp.mkdir(parents=True, exist_ok=True)
    all_rows = []
    for pair in pairs:
        pair_dir = exp / f"pair_{int(pair['pair_id']):03d}_{pair['scenario']}"
        pair_dir.mkdir(parents=True, exist_ok=True)
        start, goal = pair["start"], pair["goal"]
        paths = {}
        profiles = {}
        rows = []
        for model, C in fields.items():
            print(f"  pair {pair['pair_id']} {pair['scenario']} {model}: {start}->{goal}")
            res = astar_directional(C, start, goal, cell_size, directions, n_neighbors=args.neighbors, edge_buffer=args.edge_buffer)
            paths[model] = res.path
            base = path_common_row(model, pair, res)
            if model == "V7" and res.path:
                prof = path_vehicle_profile(res.path, dem, C, parts, directions, cell_size)
                profiles[model] = prof
                write_csv(pair_dir / f"profile_{model}_车辆约束剖面.csv", prof)
                base.update(summarize_vehicle_profile(prof))
            elif res.path:
                # Baselines do not have rho fields; evaluate along V7 parts for fair risk exposure comparison.
                prof = path_vehicle_profile(res.path, dem, fields["V7"], parts, directions, cell_size)
                profiles[model] = prof
                write_csv(pair_dir / f"profile_{model}_按V7约束评价剖面.csv", prof)
                base.update(summarize_vehicle_profile(prof))
            rows.append(base); all_rows.append(base)
        write_csv(pair_dir / "01_pair路径与车辆约束暴露指标.csv", rows)
        plot_paths(pair_dir / "02_路径对比图.png", fields["V7"], paths, start, goal, title=f"pair {pair['pair_id']} {pair['scenario']} 路径对比")
        plot_vehicle_profiles(pair_dir / "03_沿途车辆能力利用率剖面.png", profiles, title=f"pair {pair['pair_id']} 沿途车辆约束剖面")
        plot_exposure_bars(pair_dir / "04_车辆约束暴露比例柱状图.png", rows, title=f"pair {pair['pair_id']} 车辆约束暴露比例")
    write_csv(exp / "00_全部路径车辆约束暴露指标.csv", all_rows)
    plot_exposure_bars(exp / "01_全部路径车辆约束暴露比例汇总.png", all_rows, title="全部路径车辆约束暴露比例")
    return all_rows


def run_vehicle_sensitivity(out, dem, p, q, cell_size, directions, pairs, base_vehicle, args):
    exp = out / "06_车辆参数敏感性实验"
    exp.mkdir(parents=True, exist_ok=True)
    rows = []
    # Reduce paths for speed: first two scenario pairs.
    test_pairs = pairs[:max(1, min(args.sensitivity_pairs, len(pairs)))]
    for cg in args.cg_heights:
        for mu in args.mu_values:
            for grade in args.grade_angles:
                veh = jeep_wrangler_rubicon_4door(cg_height_m=cg, mu_slide=mu, mu_brake=mu, max_grade_angle_deg=grade)
                C7, parts = compute_vehicle_constraint_fields(dem, p, q, directions, cell_size, veh, enable_break=not args.disable_break, hard_rho=args.hard_rho)
                for pair in test_pairs:
                    res = astar_directional(C7, pair["start"], pair["goal"], cell_size, directions, n_neighbors=args.neighbors, edge_buffer=args.edge_buffer)
                    prof = path_vehicle_profile(res.path, dem, C7, parts, directions, cell_size) if res.path else []
                    rec = {
                        "cg_height_m": cg,
                        "mu_slide_mu_brake": mu,
                        "max_grade_angle_deg": grade,
                        "pair_id": pair["pair_id"],
                        "scenario": pair["scenario"],
                        "是否找到路径": bool(res.path),
                        "路径总代价_J": res.total_cost,
                        "路径长度_L_m": res.path_length_m,
                        "平均单位距离代价": res.average_cost_per_m,
                    }
                    rec.update(summarize_vehicle_profile(prof))
                    rows.append(rec)
    write_csv(exp / "01_车辆参数敏感性_逐路径指标.csv", rows)
    # Aggregate
    agg = {}
    for r in rows:
        key = (r["cg_height_m"], r["mu_slide_mu_brake"], r["max_grade_angle_deg"])
        agg.setdefault(key, []).append(r)
    agg_rows = []
    for key, vals in agg.items():
        rec = {"cg_height_m": key[0], "mu_slide_mu_brake": key[1], "max_grade_angle_deg": key[2], "样本数": len(vals)}
        for metric in ["路径总代价_J", "路径长度_L_m", "E_rho_up_gt_0p7", "E_rho_roll_gt_0p7", "E_rho_slide_gt_0p7", "E_rho_max_gt_1p0"]:
            data = [float(v.get(metric, np.nan)) for v in vals]
            rec[f"{metric}_mean"] = float(np.nanmean(data))
        agg_rows.append(rec)
    write_csv(exp / "02_车辆参数敏感性_汇总指标.csv", agg_rows)


def write_readme(out: Path, args, meta, vehicle):
    text = f"""# v7 参数化车辆约束方向通行能力实验输出说明

生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}

## 本版核心变化

v7 不再把经验阻碍度函数 `R=1-exp[-(alpha/alpha0)^2]` 当作主模型。该函数只保留为 V6 baseline。主模型改为：

```math
rho_j(i,d)=terrain_demand_j(i,d)/vehicle_capacity_j
```

然后用软屏障函数得到单位距离代价：

```math
c(i,d)=1+sum_j lambda_j phi(rho_j),  phi(rho)=log(1+exp(k(rho-1)))/k
```

## 示例车辆

{vehicle.name}

- 轴距 L = {vehicle.wheelbase_m:.3f} m
- 轮距 B = {vehicle.track_width_m:.3f} m
- 离地间隙 h_g = {vehicle.ground_clearance_m:.3f} m
- 通过角 = {vehicle.breakover_angle_deg:.1f} deg
- 整备质量 m = {vehicle.mass_kg:.1f} kg
- 质心高度 h_c = {vehicle.cg_height_m:.2f} m（假设/敏感性参数，不是官方值）

## 数据

{meta}

## 重点看哪些文件

1. `01_公开车辆参数_Jeep_Wrangler_Rubicon/01_车辆参数来源表.csv`
2. `02_公式推导与参数来源/01_公式推导与自查说明.md`
3. `03_物理一致性合成实验/02_单约束能力利用率曲线.png`
4. `04_真实DEM方向代价场/01_B0_B1_V6_V7_最小方向代价图.png`
5. `05_路径风险暴露实验/00_全部路径车辆约束暴露指标.csv`
6. `06_车辆参数敏感性实验/02_车辆参数敏感性_汇总指标.csv`

## 论文解释口径

v7 证明的不是路径时间最短或真实能耗最低，而是：在 DEM-GIS 数据条件下，车辆参数化约束模型比经验坡度阻碍度更能解释路径为何应避开接近车辆牵引、制动、侧翻、侧滑和几何通过能力边界的地形。
"""
    (out / "00_实验总说明_请先看.md").write_text(text, encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true", help="使用合成 DEM 测试")
    ap.add_argument("--synthetic-size", type=int, default=220)
    ap.add_argument("--dem", type=str, default=None)
    ap.add_argument("--dem-path-file", type=str, default="my_dem_path.txt")
    ap.add_argument("--cell-size", type=float, default=30.0)
    ap.add_argument("--max-pixels", type=int, default=None)
    ap.add_argument("--out", type=str, default="outputs_v7")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--neighbors", type=int, default=8, choices=[4,8,16,32])
    ap.add_argument("--roi-window", type=int, default=180)
    ap.add_argument("--roi-stride", type=int, default=120)
    ap.add_argument("--roi-top-k", type=int, default=2)
    ap.add_argument("--roi-min-slope", type=float, default=1.0)
    ap.add_argument("--random-pairs-per-roi", type=int, default=1)
    ap.add_argument("--max-pairs", type=int, default=6)
    ap.add_argument("--edge-buffer", type=int, default=8)
    ap.add_argument("--disable-break", action="store_true", help="禁用通过角/曲率约束")
    ap.add_argument("--hard-rho", type=float, default=None, help="超过该 rho 则设为不可通行；默认不启用硬约束，只统计暴露")
    ap.add_argument("--cg-height", type=float, default=0.85)
    ap.add_argument("--mu", type=float, default=0.50)
    ap.add_argument("--grade-angle", type=float, default=30.0)
    ap.add_argument("--eta-roll", type=float, default=0.60)
    # v6 baseline parameters
    ap.add_argument("--v6-alpha-u", type=float, default=15.0)
    ap.add_argument("--v6-alpha-m", type=float, default=5.0)
    ap.add_argument("--v6-alpha-d", type=float, default=15.0)
    ap.add_argument("--v6-alpha-r", type=float, default=15.0)
    # sensitivity lists
    ap.add_argument("--cg-heights", type=lambda s: [float(x) for x in s.split(',')], default=[0.75,0.85,0.95])
    ap.add_argument("--mu-values", type=lambda s: [float(x) for x in s.split(',')], default=[0.35,0.50,0.70])
    ap.add_argument("--grade-angles", type=lambda s: [float(x) for x in s.split(',')], default=[25.0,30.0,35.0])
    ap.add_argument("--sensitivity-pairs", type=int, default=2)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    pkg_dir = Path(__file__).resolve().parent

    dem, cell_size, meta = load_input_dem(args)
    p, q = calculate_gradient_tan(dem, cell_size)
    directions = get_directions(args.neighbors)
    vehicle = jeep_wrangler_rubicon_4door(cg_height_m=args.cg_height, mu_slide=args.mu, mu_brake=args.mu, max_grade_angle_deg=args.grade_angle, eta_roll=args.eta_roll)

    # Output docs and parameters
    write_readme(out, args, meta, vehicle)
    copy_docs_to_output(pkg_dir, out)
    param_dir = out / "01_公开车辆参数_Jeep_Wrangler_Rubicon"
    param_dir.mkdir(parents=True, exist_ok=True)
    write_vehicle_params_csv(param_dir / "01_车辆参数来源表.csv", vehicle)
    (param_dir / "02_vehicle_params.json").write_text(json.dumps(vehicle.__dict__, ensure_ascii=False, indent=2), encoding="utf-8")

    run_physical_consistency(out, vehicle)

    fields, parts = build_fields(dem, p, q, directions, cell_size, vehicle, args)
    cost_dir = out / "04_真实DEM方向代价场"
    cost_dir.mkdir(exist_ok=True)
    plot_cost_maps(cost_dir / "01_B0_B1_V6_V7_最小方向代价图.png", dem, fields, title="B0/B1/V6/V7 最小方向单位距离代价")

    # ROI and pairs
    roi_dir = out / "04_真实DEM方向代价场"
    AI, slope_deg, rois = select_rois(dem, p, q, fields["V7"], window_size=args.roi_window, stride=args.roi_stride, top_k=args.roi_top_k, min_mean_slope_deg=args.roi_min_slope)
    write_roi_csv(roi_dir / "02_ROI候选区.csv", rois)
    plot_roi_overview(roi_dir, AI, slope_deg, rois)
    pairs = generate_scenario_pairs(p, q, rois, edge_buffer=args.edge_buffer, random_pairs_per_roi=args.random_pairs_per_roi, seed=args.seed)
    pairs = pairs[:args.max_pairs]
    write_pairs_csv(roi_dir / "03_情景起终点.csv", pairs)

    run_path_experiments(out, dem, p, q, cell_size, directions, fields, parts, pairs, args)
    run_vehicle_sensitivity(out, dem, p, q, cell_size, directions, pairs, vehicle, args)

    # OSM interface copy / note
    osm_dir = out / "08_OSM弱监督验证接口说明"
    osm_dir.mkdir(parents=True, exist_ok=True)
    note = """# OSM 弱监督验证接口说明\n\n本 v7 包先不自动下载 OSM。下一步需要用户提供研究区内道路矢量文件：shp/geojson/gpkg。\n\n建议正样本：highway=track/path/unclassified/service/tertiary 等山区道路或林道。\n建议负样本：在相同坡度带、相近高程带、相近土地覆盖背景下采样非道路栅格。\n\n验证指标：\n\n- Delta C = mean(C_nonroad)-mean(C_road)\n- AUC = P(C_road < C_nonroad)\n- Delta rho_roll, Delta rho_up, Delta rho_slide\n\nOSM 只能作为弱监督证据，不是真实车辆通行能力真值。\n"""
    (osm_dir / "01_OSM接口说明.md").write_text(note, encoding="utf-8")
    print(f"完成。输出目录：{out.resolve()}")

if __name__ == "__main__":
    main()
