"""
run_v10_experiments.py

v10：基于空间化地表附着参数的车辆能力利用率方向通行能力实验。

主线：
DEM + LandCover/TWI/Water/Soil -> mu_s(x,y), mu_b(x,y)
-> 上坡牵引、下坡制动、侧翻稳定性、侧滑附着四类利用率
-> 方向通行代价 C(i,d)
-> 路径约束利用率统计。
"""
from __future__ import annotations
import argparse, csv, json, shutil, time, warnings
from pathlib import Path
import numpy as np
warnings.filterwarnings('ignore', message='Glyph .* missing from font.*')

from run_single_experiment import generate_synthetic_dem, load_dem_geotiff
from cost_model import calculate_gradient_tan, compute_b0_unit_cost_field, compute_b1_unit_cost_field
from direction_utils import get_directions
from path_planning import astar_directional, PathResult
from roi_selector import select_rois, write_roi_csv, plot_roi_overview
from scenario_pairs import generate_scenario_pairs, write_pairs_csv
from vehicle_params import jeep_wrangler_rubicon_4door, write_vehicle_params_csv
from vehicle_capability_model import compute_vehicle_capability_fields, compute_capability_curves
from spatial_surface_params import compute_spatial_surface_params
from raster_align import load_raster_array, resize_to_shape
from path_metrics import path_capability_profile, summarize_profile, indicator_explanations
from plotting_v10 import plot_surface_params, plot_direction_cost_maps, plot_paths, plot_profiles, plot_bar_metrics, plot_ablation_summary, save_map_grid

MODEL_LABELS = {
    'B0': 'B0 静态坡度代价',
    'B1': 'B1 方向纵坡代价',
    'V9常数附着': 'V9 常数附着车辆能力利用率',
    'V10空间附着': 'V10 空间附着车辆能力利用率',
}


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text('', encoding='utf-8-sig'); return
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    with path.open('w', newline='', encoding='utf-8-sig') as f:
        w=csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)


def read_text_path(path_file: str | Path) -> str:
    return Path(path_file).read_text(encoding='utf-8-sig').strip().strip('"')


def load_input_dem(args):
    if args.synthetic:
        return generate_synthetic_dem(rows=args.synthetic_size, cols=args.synthetic_size, cell_size=args.cell_size, seed=args.seed)
    dem_path = args.dem or (read_text_path(args.dem_path_file) if args.dem_path_file else None)
    if not dem_path:
        raise ValueError('真实 DEM 模式需要 --dem 或 --dem-path-file')
    return load_dem_geotiff(dem_path, max_pixels=args.max_pixels, fallback_cell_size=args.cell_size)


def load_optional_rasters(args, shape):
    landcover = soil = water = None
    metas = {}
    if args.landcover:
        landcover, metas['landcover'] = load_raster_array(args.landcover, target_shape=shape, categorical=True)
        landcover = np.nan_to_num(landcover, nan=0).astype(int)
    if args.soil:
        soil, metas['soil'] = load_raster_array(args.soil, target_shape=shape, categorical=True)
        soil = np.nan_to_num(soil, nan=0).astype(int)
    if args.water:
        water, metas['water'] = load_raster_array(args.water, target_shape=shape, categorical=True)
        water = np.nan_to_num(water, nan=0) > 0
    return landcover, soil, water, metas


def nearest_free(point, barrier):
    r,c = map(int, point)
    rows, cols = barrier.shape
    r=max(0,min(rows-1,r)); c=max(0,min(cols-1,c))
    if not barrier[r,c]: return (r,c)
    for rad in range(1, max(rows,cols)):
        r0=max(0,r-rad); r1=min(rows-1,r+rad); c0=max(0,c-rad); c1=min(cols-1,c+rad)
        for rr in range(r0,r1+1):
            for cc in (c0,c1):
                if not barrier[rr,cc]: return (rr,cc)
        for cc in range(c0,c1+1):
            for rr in (r0,r1):
                if not barrier[rr,cc]: return (rr,cc)
    return (r,c)


def repair_pairs(pairs, barrier):
    out=[]
    for p in pairs:
        pp=dict(p)
        pp['start']=nearest_free(pp['start'], barrier)
        pp['goal']=nearest_free(pp['goal'], barrier)
        out.append(pp)
    return out


def path_common_row(model: str, pair: dict, result: PathResult) -> dict:
    return {
        'pair_id': pair['pair_id'], 'roi_id': pair['roi_id'], 'scenario': pair['scenario'],
        '模型代码': model, '模型名称': MODEL_LABELS.get(model, model),
        '起点_row': pair['start'][0], '起点_col': pair['start'][1],
        '终点_row': pair['goal'][0], '终点_col': pair['goal'][1],
        '是否找到路径': bool(result.path),
        '路径总代价_J': result.total_cost,
        '路径长度_L_m': result.path_length_m,
        '平均单位距离代价_J除以L': result.average_cost_per_m,
        '绕行率_L除以直线距离': result.detour_ratio,
        '路径节点数': len(result.path),
    }


def formula_passport_rows():
    return [
        {'公式':'alpha_parallel = arctan(grad_z · u_d)','人话解释':'车辆前进方向遇到的坡度；正值为上坡，负值为下坡。','来源类型':'DEM 几何投影','是否进入主模型':'是'},
        {'公式':'alpha_cross = arctan(|grad_z · n_d|)','人话解释':'车辆横向方向遇到的横坡，反映车身左右倾斜程度。','来源类型':'DEM 几何投影','是否进入主模型':'是'},
        {'公式':'rho_up = tan(max(alpha_parallel,0))/tan(alpha_grade)','人话解释':'当前上坡用了多少比例的可接受爬坡能力。','来源类型':'斜坡需求 / 车辆爬坡能力场景参数','是否进入主模型':'是'},
        {'公式':'rho_down = tan(max(-alpha_parallel,0))/mu_b(x,y)','人话解释':'当前下坡用了多少比例的制动附着能力。','来源类型':'斜面附着条件 + 空间化制动附着参数','是否进入主模型':'是'},
        {'公式':'rho_roll = tan(|alpha_cross|)/(B/(2h_c))','人话解释':'当前横坡用了多少比例的抗侧翻稳定能力。','来源类型':'车辆静态稳定因子 SSF','是否进入主模型':'是'},
        {'公式':'rho_slide = tan(|alpha_cross|)/mu_s(x,y)','人话解释':'当前横坡用了多少比例的横向附着能力。','来源类型':'斜面附着条件 + 空间化侧滑附着参数','是否进入主模型':'是'},
        {'公式':'rho_max = max(rho_up,rho_down,rho_roll,rho_slide)','人话解释':'每个方向上最接近限制的那一项。','来源类型':'多限制同时满足的最大利用率表达','是否进入主模型':'是'},
        {'公式':'c(i,d)=1+rho_max(i,d)','人话解释':'基础距离成本加上车辆能力利用率附加成本。','来源类型':'本文路径代价构造，需要实验验证','是否进入主模型':'是'},
        {'公式':'TWI=ln(A_s/(tan(beta)+eps))','人话解释':'DEM 派生地形湿润倾向，不是实测土壤含水率。','来源类型':'地形湿润指数','是否进入主模型':'用于 mu 空间化'},
        {'公式':'mu = clip(mu_base*(1-gamma*W), min, max)','人话解释':'地表类别给出基础附着能力，湿润倾向降低附着能力。','来源类型':'GIS 地表附着参数转换模型，默认参数需敏感性分析','是否进入主模型':'是'},
    ]


def parameter_rows(vehicle, args):
    return [
        {'参数':'B','含义':'车辆轮距，用于侧翻稳定性利用率','来源':'Jeep 公开规格','是否敏感性参数':'否'},
        {'参数':'h_c/H','含义':'质心高度占车高比例，用于计算 h_c','来源':'公开资料未给出，作为敏感性参数','是否敏感性参数':'是'},
        {'参数':'alpha_grade','含义':'最大可接受爬坡角场景','来源':'场景参数，不是 Jeep 官方值','是否敏感性参数':'是'},
        {'参数':'mu_s(x,y)','含义':'侧滑附着能力空间栅格','来源':'土地覆盖基础值 + TWI/水系/土壤湿润修正','是否敏感性参数':'是'},
        {'参数':'mu_b(x,y)','含义':'下坡制动附着能力空间栅格','来源':'土地覆盖基础值 + TWI/水系/土壤湿润修正','是否敏感性参数':'是'},
        {'参数':'gamma_s,gamma_b','含义':'湿润倾向对附着能力的削弱幅度','来源':'情景参数','是否敏感性参数':'是'},
        {'参数':'water_decay_m','含义':'水系附近湿润影响衰减距离','来源':'情景参数','是否敏感性参数':'是'},
    ]


def write_main_readme(out: Path, meta, args):
    text = f'''# v10 实验输出说明

生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}

## 本版研究目标

v10 将 v9 中的常数附着能力参数改成空间栅格：

```math
mu_s(x,y), mu_b(x,y)
```

其中 `mu_s` 表示侧滑附着能力，`mu_b` 表示下坡制动附着能力。它们由土地覆盖基础附着表、DEM 派生 TWI、水系附近湿润影响和土壤软弱倾向共同生成。

## 主模型

```math
rho_up = tan(max(alpha_parallel,0)) / tan(alpha_grade)
```

```math
rho_down = tan(max(-alpha_parallel,0)) / mu_b(x,y)
```

```math
rho_roll = tan(|alpha_cross|) / (B/(2h_c))
```

```math
rho_slide = tan(|alpha_cross|) / mu_s(x,y)
```

```math
rho_max = max(rho_up, rho_down, rho_roll, rho_slide)
```

```math
c(i,d) = 1 + rho_max(i,d)
```

人话解释：每一小段路都看车辆最接近哪一种限制：爬坡、制动、侧翻稳定性还是侧滑附着。越接近限制，单位距离代价越高。

## 当前数据说明

{json.dumps(meta, ensure_ascii=False, indent=2, default=str)}

## 重要限制

- TWI 是地形湿润倾向，不是实测土壤含水率。
- 附着能力表是工程情景参数，不是实测摩擦系数。
- OSM 道路只能作为弱监督证据，不是通行能力真值。
- 12.5m 与 30m DEM 的跨尺度复核可作为可选实验，不能把 12.5m 直接称作实车真实世界。
'''
    (out/'00_实验总说明_请先看.md').write_text(text, encoding='utf-8')


def run_formula_consistency(out: Path, vehicle):
    from plotting_v10 import setup_font
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    exp = out/'01_公式物理一致性实验'; exp.mkdir(parents=True, exist_ok=True)
    curves = compute_capability_curves(vehicle)
    rows=[]
    n=len(curves['degree'])
    for i in range(n): rows.append({k: float(v[i]) for k,v in curves.items()})
    write_csv(exp/'01_四类车辆能力利用率曲线数据.csv', rows)
    setup_font(); fig, axes = plt.subplots(2,2,figsize=(11,8))
    deg=curves['degree']
    axes[0,0].plot(deg, curves['rho_up_grade']); axes[0,0].axhline(1,ls='--'); axes[0,0].set_title('上坡牵引利用率')
    for k,v in curves.items():
        if k.startswith('rho_down'):
            axes[0,1].plot(deg,v,label=k.replace('rho_down_',''))
    axes[0,1].axhline(1,ls='--'); axes[0,1].legend(fontsize=8); axes[0,1].set_title('下坡制动利用率')
    for k,v in curves.items():
        if k.startswith('rho_roll'):
            axes[1,0].plot(deg,v,label=k.replace('rho_roll_',''))
    axes[1,0].axhline(1,ls='--'); axes[1,0].legend(fontsize=8); axes[1,0].set_title('侧翻稳定性利用率')
    for k,v in curves.items():
        if k.startswith('rho_slide'):
            axes[1,1].plot(deg,v,label=k.replace('rho_slide_',''))
    axes[1,1].axhline(1,ls='--'); axes[1,1].legend(fontsize=8); axes[1,1].set_title('侧滑附着利用率')
    for ax in axes.ravel(): ax.set_xlabel('坡度角 / °'); ax.set_ylabel('利用率'); ax.grid(True,alpha=0.3)
    fig.tight_layout(); fig.savefig(exp/'02_四类利用率曲线.png', dpi=180, bbox_inches='tight'); plt.close(fig)


def build_fields(dem, p, q, directions, cell_size, vehicle, surface, args):
    _, C0 = compute_b0_unit_cost_field(p, q, directions, alpha_u=args.v6_alpha_u)
    _, C1 = compute_b1_unit_cost_field(p, q, directions, alpha_u=args.v6_alpha_u, alpha_m=args.v6_alpha_m, alpha_d=args.v6_alpha_d)
    C_const, parts_const = compute_vehicle_capability_fields(dem,p,q,directions,cell_size,vehicle, hard_barrier=surface['hard_barrier'], hard_rho=args.hard_rho)
    C10, parts10 = compute_vehicle_capability_fields(dem,p,q,directions,cell_size,vehicle, mu_slide_field=surface['mu_slide'], mu_brake_field=surface['mu_brake'], hard_barrier=surface['hard_barrier'], hard_rho=args.hard_rho)
    return {'B0':C0,'B1':C1,'V9常数附着':C_const,'V10空间附着':C10}, parts10, parts_const


def run_path_stats(out, dem, fields, parts, pairs, directions, cell_size, args):
    exp=out/'04_路径约束利用率统计实验'; exp.mkdir(parents=True,exist_ok=True)
    all_rows=[]
    for pair in pairs:
        pair_dir=exp/f"pair_{int(pair['pair_id']):03d}_{pair['scenario']}"; pair_dir.mkdir(parents=True,exist_ok=True)
        paths={}; profiles={}; rows=[]
        for model,C in fields.items():
            print(f"  pair {pair['pair_id']} {pair['scenario']} {model}: {pair['start']} -> {pair['goal']}")
            res=astar_directional(C, pair['start'], pair['goal'], cell_size, directions, n_neighbors=args.neighbors, barrier_mask=parts['hard_barrier'], edge_buffer=args.edge_buffer)
            paths[model]=res.path
            rec=path_common_row(model,pair,res)
            if res.path:
                prof=path_capability_profile(res.path, dem, fields['V10空间附着'], parts, directions, cell_size)
                profiles[model]=prof
                write_csv(pair_dir/f'profile_{model}_按V10空间附着模型评价.csv', prof)
                rec.update(summarize_profile(prof))
            rows.append(rec); all_rows.append(rec)
        write_csv(pair_dir/'01_pair路径约束利用率统计指标.csv', rows)
        plot_paths(pair_dir/'02_路径对比图.png', fields['V10空间附着'], paths, pair['start'], pair['goal'], title=f"pair {pair['pair_id']} 路径对比")
        plot_profiles(pair_dir/'03_沿途四类车辆能力利用率剖面.png', profiles, title=f"pair {pair['pair_id']} 沿途四类车辆能力利用率")
        plot_bar_metrics(pair_dir/'04_高利用率路径比例柱状图.png', rows, title=f"pair {pair['pair_id']} 高利用率路径比例")
    write_csv(exp/'00_全部路径约束利用率统计指标.csv', all_rows)
    plot_bar_metrics(exp/'01_全部路径高利用率比例汇总.png', all_rows, title='全部路径高利用率路径比例')
    return all_rows


def run_ablation(out, dem, p, q, directions, cell_size, vehicle, base_surface, landcover, soil, water, pairs, args):
    exp=out/'05_空间化参数消融实验'; exp.mkdir(parents=True,exist_ok=True)
    versions=[
        ('V10-constant','常数附着能力', {'use_lc':False,'use_twi':False,'use_water':False,'use_soil':False}),
        ('V10-LC','只使用土地覆盖基础附着', {'use_lc':True,'use_twi':False,'use_water':False,'use_soil':False}),
        ('V10-LC-TWI','土地覆盖 + TWI', {'use_lc':True,'use_twi':True,'use_water':False,'use_soil':False}),
        ('V10-LC-TWI-Water','土地覆盖 + TWI + 水系影响', {'use_lc':True,'use_twi':True,'use_water':True,'use_soil':False}),
        ('V10-Full','土地覆盖 + TWI + 水系 + 土壤软弱倾向', {'use_lc':True,'use_twi':True,'use_water':True,'use_soil':True}),
    ]
    pair_subset=pairs[:max(1,min(args.ablation_pairs,len(pairs)))]
    rows=[]; summary=[]
    for code,desc,opt in versions:
        if code=='V10-constant':
            C, parts = compute_vehicle_capability_fields(dem,p,q,directions,cell_size,vehicle, hard_barrier=base_surface['hard_barrier'], hard_rho=args.hard_rho)
            mu_s = np.full(dem.shape, vehicle.mu_slide); mu_b = np.full(dem.shape, vehicle.mu_brake)
        else:
            weights=(1.0 if opt['use_twi'] else 0.0, 1.0 if opt['use_water'] else 0.0, 1.0 if opt['use_soil'] else 0.0)
            surf=compute_spatial_surface_params(dem, cell_size, args.lookup_csv, landcover=landcover if opt['use_lc'] else None, soil=soil if opt['use_soil'] else None, water_mask=water if opt['use_water'] else None, scenario=args.scenario, gamma_slide=args.gamma_slide, gamma_brake=args.gamma_brake, water_decay_m=args.water_decay_m, weights=weights, seed=args.seed)
            C, parts=compute_vehicle_capability_fields(dem,p,q,directions,cell_size,vehicle, mu_slide_field=surf['mu_slide'], mu_brake_field=surf['mu_brake'], hard_barrier=surf['hard_barrier'], hard_rho=args.hard_rho)
        for pair in pair_subset:
            res=astar_directional(C, pair['start'], pair['goal'], cell_size, directions, n_neighbors=args.neighbors, barrier_mask=parts['hard_barrier'], edge_buffer=args.edge_buffer)
            prof=path_capability_profile(res.path, dem, C, parts, directions, cell_size) if res.path else []
            rec={'版本':code,'说明':desc,'pair_id':pair['pair_id'],'scenario':pair['scenario'],'是否找到路径':bool(res.path),'路径总代价_J':res.total_cost,'路径长度_L_m':res.path_length_m}
            rec.update(summarize_profile(prof)); rows.append(rec)
    write_csv(exp/'01_空间化参数消融_逐路径指标.csv', rows)
    for code,desc,_ in versions:
        vals=[r for r in rows if r['版本']==code]
        rec={'版本':code,'说明':desc,'样本数':len(vals)}
        for key in ['路径总代价_J','路径长度_L_m','平均侧滑附着利用率','平均下坡制动利用率','P_rho_slide_gt_0p7','P_rho_max_gt_1p0']:
            data=[float(v.get(key,np.nan)) for v in vals]
            rec[key+'_mean']=float(np.nanmean(data)) if data else np.nan
        summary.append(rec)
    write_csv(exp/'02_空间化参数消融_汇总指标.csv', summary)
    plot_ablation_summary(exp/'03_空间化参数消融_约束超限比例.png', summary)


def run_humidity_scenarios(out, dem, p, q, directions, cell_size, vehicle, landcover, soil, water, pairs, args):
    exp=out/'06_湿度情景实验'; exp.mkdir(parents=True,exist_ok=True)
    rows=[]; pair_subset=pairs[:max(1,min(args.scenario_pairs,len(pairs)))]
    for scenario in ['dry','normal','wet','post_rain']:
        surf=compute_spatial_surface_params(dem, cell_size, args.lookup_csv, landcover=landcover, soil=soil, water_mask=water, scenario=scenario, gamma_slide=args.gamma_slide, gamma_brake=args.gamma_brake, water_decay_m=args.water_decay_m, seed=args.seed)
        C, parts=compute_vehicle_capability_fields(dem,p,q,directions,cell_size,vehicle, mu_slide_field=surf['mu_slide'], mu_brake_field=surf['mu_brake'], hard_barrier=surf['hard_barrier'], hard_rho=args.hard_rho)
        save_map_grid(exp/f'{scenario}_mu_s_mu_b_W.png',[surf['mu_slide'],surf['mu_brake'],surf['wetness_effective']], [f'{scenario} μs',f'{scenario} μb',f'{scenario} W'], cmap='viridis', suptitle=f'{scenario} 湿度情景')
        for pair in pair_subset:
            res=astar_directional(C, pair['start'], pair['goal'], cell_size, directions, n_neighbors=args.neighbors, barrier_mask=parts['hard_barrier'], edge_buffer=args.edge_buffer)
            prof=path_capability_profile(res.path, dem, C, parts, directions, cell_size) if res.path else []
            rec={'湿度情景':scenario,'pair_id':pair['pair_id'],'scenario':pair['scenario'],'是否找到路径':bool(res.path),'路径总代价_J':res.total_cost,'路径长度_L_m':res.path_length_m}
            rec.update(summarize_profile(prof)); rows.append(rec)
    write_csv(exp/'01_湿度情景_逐路径指标.csv', rows)


def main():
    ap=argparse.ArgumentParser(description='v10 空间化地表附着参数车辆能力利用率实验')
    ap.add_argument('--synthetic', action='store_true')
    ap.add_argument('--synthetic-size', type=int, default=220)
    ap.add_argument('--dem')
    ap.add_argument('--dem-path-file', default='my_dem_path.txt')
    ap.add_argument('--dem-highres', help='可选：12.5m 或更高分辨率 DEM，用于后续跨尺度复核接口')
    ap.add_argument('--landcover', help='可选土地覆盖 GeoTIFF/栅格')
    ap.add_argument('--soil', help='可选土壤分类 GeoTIFF/栅格')
    ap.add_argument('--water', help='可选水体/水系掩膜 GeoTIFF/栅格，非零为水系')
    ap.add_argument('--out', default='outputs_v10')
    ap.add_argument('--max-pixels', type=int, default=700)
    ap.add_argument('--cell-size', type=float, default=30.0)
    ap.add_argument('--neighbors', type=int, default=8)
    ap.add_argument('--edge-buffer', type=int, default=3)
    ap.add_argument('--roi-size', type=int, default=180)
    ap.add_argument('--n-rois', type=int, default=2)
    ap.add_argument('--n-pairs-per-scenario', type=int, default=1)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--scenario', default='normal', choices=['dry','normal','wet','post_rain'])
    ap.add_argument('--gamma-slide', type=float, default=0.5)
    ap.add_argument('--gamma-brake', type=float, default=0.5)
    ap.add_argument('--water-decay-m', type=float, default=90.0)
    ap.add_argument('--hard-rho', type=float, default=None)
    ap.add_argument('--lookup-csv', default='config/landcover_mu_lookup.csv')
    ap.add_argument('--v6-alpha-u', type=float, default=15)
    ap.add_argument('--v6-alpha-m', type=float, default=5)
    ap.add_argument('--v6-alpha-d', type=float, default=15)
    ap.add_argument('--ablation-pairs', type=int, default=2)
    ap.add_argument('--scenario-pairs', type=int, default=2)
    ap.add_argument('--skip-ablation', action='store_true')
    ap.add_argument('--skip-scenarios', action='store_true')
    args=ap.parse_args()

    pkg_dir=Path(__file__).resolve().parent
    if not Path(args.lookup_csv).is_absolute(): args.lookup_csv=str(pkg_dir/args.lookup_csv)
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    dem, cell_size, meta=load_input_dem(args)
    landcover, soil, water, raster_metas=load_optional_rasters(args, dem.shape)
    meta['可选栅格元数据']=raster_metas
    meta['v10说明']='若未提供土地覆盖/土壤/水系，程序使用合成土地覆盖、DEM 派生 TWI 和潜在湿润区完成可运行测试。'

    p,q=calculate_gradient_tan(dem, cell_size)
    directions=get_directions(args.neighbors)
    vehicle=jeep_wrangler_rubicon_4door()

    # 0 docs and passport
    write_main_readme(out, meta, args)
    write_csv(out/'00_公式护照_逐项来源.csv', formula_passport_rows())
    write_csv(out/'00_参数来源与用途说明.csv', parameter_rows(vehicle,args))
    write_csv(out/'00_路径约束利用率指标说明.csv', indicator_explanations())
    write_vehicle_params_csv(out/'01_公开车辆参数'/'01_车辆参数来源表.csv', vehicle)
    (out/'01_公开车辆参数'/'02_vehicle_params.json').write_text(json.dumps(vehicle.__dict__,ensure_ascii=False,indent=2),encoding='utf-8')

    # copy docs
    if (pkg_dir/'docs').exists():
        dst=out/'00_理论与实验说明文档'; dst.mkdir(exist_ok=True)
        for f in (pkg_dir/'docs').glob('*'):
            if f.is_file(): shutil.copy2(f,dst/f.name)

    # 1 formula consistency
    run_formula_consistency(out, vehicle)

    # 2 surface params
    surf=compute_spatial_surface_params(dem, cell_size, args.lookup_csv, landcover=landcover, soil=soil, water_mask=water, scenario=args.scenario, gamma_slide=args.gamma_slide, gamma_brake=args.gamma_brake, water_decay_m=args.water_decay_m, seed=args.seed)
    exp2=out/'02_PTF_Lite地表附着参数空间化'; exp2.mkdir(parents=True,exist_ok=True)
    plot_surface_params(exp2/'01_湿润倾向与附着能力图.png', surf)
    write_csv(exp2/'02_地表附着参数检查样点.csv', [
        {'row':int(r),'col':int(c),'landcover':int(surf['landcover'][r,c]),'W':float(surf['wetness_effective'][r,c]),'mu_s':float(surf['mu_slide'][r,c]),'mu_b':float(surf['mu_brake'][r,c]),'hard_barrier':bool(surf['hard_barrier'][r,c])}
        for r,c in zip(np.random.RandomState(args.seed).randint(0,dem.shape[0],20), np.random.RandomState(args.seed+1).randint(0,dem.shape[1],20))
    ])
    write_csv(exp2/'03_PTF_Lite元数据.csv', [{k:json.dumps(v,ensure_ascii=False) for k,v in surf['metadata'].items()}])

    # fields
    fields, parts10, parts_const=build_fields(dem,p,q,directions,cell_size,vehicle,surf,args)
    exp3=out/'03_方向车辆能力利用率图'; exp3.mkdir(parents=True,exist_ok=True)
    plot_direction_cost_maps(exp3/'01_方向代价图与主导限制图.png', fields, parts10)
    write_csv(exp3/'02_方向代价统计.csv', [
        {'统计量':'mean_min_cost','数值':float(np.nanmean(np.nanmin(np.where(np.isfinite(fields['V10空间附着']), fields['V10空间附着'], np.nan),axis=2)))},
        {'统计量':'mean_max_cost','数值':float(np.nanmean(np.nanmax(np.where(np.isfinite(fields['V10空间附着']), fields['V10空间附着'], np.nan),axis=2)))},
        {'统计量':'mean_direction_difference','数值':float(np.nanmean(np.nanmax(np.where(np.isfinite(fields['V10空间附着']), fields['V10空间附着'], np.nan),axis=2)-np.nanmin(np.where(np.isfinite(fields['V10空间附着']), fields['V10空间附着'], np.nan),axis=2)))},
    ])

    # pairs
    AI, slope_deg, rois = select_rois(dem, p, q, fields['V10空间附着'], window_size=args.roi_size, stride=max(1, args.roi_size//2), top_k=args.n_rois)
    write_roi_csv(exp3/'03_ROI候选区.csv', rois)
    try:
        plot_roi_overview(exp3, AI, slope_deg, rois)
    except Exception:
        pass
    pairs=generate_scenario_pairs(p, q, rois, edge_buffer=args.edge_buffer, random_pairs_per_roi=args.n_pairs_per_scenario, seed=args.seed)
    pairs=repair_pairs(pairs, parts10['hard_barrier'])
    write_pairs_csv(exp3/'05_情景起终点.csv', pairs)

    # main path stats
    run_path_stats(out, dem, fields, parts10, pairs, directions, cell_size, args)

    if not args.skip_ablation:
        run_ablation(out, dem,p,q,directions,cell_size,vehicle,surf,landcover,soil,water,pairs,args)
    if not args.skip_scenarios:
        run_humidity_scenarios(out, dem,p,q,directions,cell_size,vehicle,landcover,soil,water,pairs,args)

    # optional interface docs
    osm_dir=out/'07_OSM弱监督验证接口'; osm_dir.mkdir(exist_ok=True)
    (osm_dir/'README_OSM弱监督验证说明.md').write_text('OSM 仅作为已有道路选择的弱监督证据，不作为通行能力真值。提供 OSM 路网后，可抽取道路缓冲区与匹配非道路样本，比较单位代价、侧滑附着利用率和侧翻稳定性利用率。\n',encoding='utf-8')
    scale_dir=out/'08_跨尺度路径一致性检验_可选'; scale_dir.mkdir(exist_ok=True)
    (scale_dir/'README_跨尺度路径一致性检验说明.md').write_text('如果提供 12.5m 与 30m 同区 DEM，可在 30m 上规划路径，并在 12.5m 参考分辨率下复核四类车辆能力利用率。该实验用于分析分辨率诱导的路径复核误差，不使用夸张表述。\n',encoding='utf-8')
    print(f'完成：{out}')


if __name__=='__main__':
    main()
