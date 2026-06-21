"""
演示脚本 v2：基于纵坡/横坡分解的方向敏感通行阻碍度建模

按 GPT 审阅建议重构：
- 从"代价乘法模型"改为"阻碍度框架"（R ∈ [0,1] → C_edge = -ln(1-R)）
- 纵坡/横坡分解替代简单 s_eff = s·cos(θ-φ)
- 三模型对比：B0（标量）、B1（仅纵坡）、Ours（纵坡+横坡）
- 新验证指标：DC、AUC、配对代价差、各向异性指数 AI

流程：
1. 生成合成DEM（或加载真实DEM）
2. 计算梯度 p, q
3. 对8个方向计算纵坡/横坡/阻碍度/边代价
4. 三模型对比路径规划
5. 计算验证指标
6. 可视化输出

============================================================
数据源切换：修改下方 DATA_SOURCE 变量即可
============================================================
"""

# ============================================================
# 数据源配置 — 修改这里切换合成/真实DEM
# ============================================================
# 'synthetic' = 合成DEM（80×80，快速验证）
# 'real'      = 真实DEM（雅江格西沟 SRTM 30m）
DATA_SOURCE = 'synthetic'  # ← 改为 'real' 即可使用真实DEM

# 真实DEM文件路径（相对于项目根目录）
REAL_DEM_PATH = 'yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif'

# 真实DEM降采样目标边长（1000×1000太大会让A*很慢，降到~200较合适）
REAL_DEM_MAX_PIXELS = 200

import sys
import os
import time
import numpy as np

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from prototype.terrain_analysis import (
    calculate_gradient, calculate_slope, calculate_aspect,
    compute_longitudinal_cross_slope
)
from prototype.cost_model import (
    compute_impedance_field, compute_scalar_cost_field,
    compute_longitudinal_only_cost_field,
    directional_slope_impedance, edge_cost,
    longitudinal_impedance, cross_slope_impedance
)
from prototype.path_planning import (
    modified_astar, find_optimal_path, _evaluate_path_cost
)
from prototype.utils import generate_synthetic_dem, load_real_dem, compute_min_cost, compute_optimal_direction


def compute_anisotropy_index(cost_field, eps=1e-8):
    """
    计算各向异性指数 AI。

    公式：AI(x,y) = (C_max - C_min) / (C_max + C_min + ε)

    AI ∈ [0, 1]：
        - AI ≈ 0：各方向代价相同（平坦区）
        - AI → 1：方向差异极大（陡峭山坡区）

    Parameters
    ----------
    cost_field : np.ndarray
        方向代价场 (rows, cols, N_directions)
    eps : float
        数值稳定项

    Returns
    -------
    AI : np.ndarray
        各向异性指数 (rows, cols)
    """
    # 确保代价非负
    cost_field_pos = np.maximum(cost_field, 0.0)
    C_max = np.max(cost_field_pos, axis=2)
    C_min = np.min(cost_field_pos, axis=2)
    AI = (C_max - C_min) / (C_max + C_min + eps)
    return np.clip(AI, 0.0, 1.0)


def compute_direction_consistency(road_points, road_directions, cost_field, directions):
    """
    计算方向一致性 DC。

    公式：DC = (1/N) Σ |cos(θ_road - θ*)|

    其中 θ* = argmin_θ C_edge(x,y,θ) 是模型预测的最优方向。

    Parameters
    ----------
    road_points : np.ndarray
        道路点坐标 (N, 2)，每行为 (row, col)
    road_directions : np.ndarray
        道路方向（弧度）
    cost_field : np.ndarray
        方向代价场 (rows, cols, N_directions)
    directions : np.ndarray
        方向角度数组

    Returns
    -------
    DC : float
        方向一致性，值越大表示道路方向越接近最优方向
    """
    # 找到每个道路点的最优方向
    dcs = []
    for (r, c), theta_road in zip(road_points, road_directions):
        r, c = int(r), int(c)
        optimal_idx = np.argmin(cost_field[r, c, :])
        theta_optimal = directions[optimal_idx]
        dcs.append(np.abs(np.cos(theta_road - theta_optimal)))

    return np.mean(dcs)


def compute_paired_cost_difference(road_costs, nonroad_costs):
    """
    计算配对代价差和统计检验。

    公式：ΔC_i = C_nonroad,i - C_road,i

    使用 Wilcoxon 符号秩检验判断 ΔC 是否显著大于0。

    Parameters
    ----------
    road_costs : np.ndarray
        道路点的代价值
    nonroad_costs : np.ndarray
        匹配非道路点的代价值

    Returns
    -------
    results : dict
        包含 mean_delta, median_delta, ci_95, wilcoxon_p
    """
    delta = nonroad_costs - road_costs

    results = {
        'mean_delta': np.mean(delta),
        'median_delta': np.median(delta),
        'n_pairs': len(delta),
    }

    # 95% 置信区间
    sorted_delta = np.sort(delta)
    n = len(sorted_delta)
    ci_low = sorted_delta[int(0.025 * n)]
    ci_high = sorted_delta[int(0.975 * n)]
    results['ci_95'] = (ci_low, ci_high)

    # Wilcoxon 符号秩检验
    try:
        from scipy.stats import wilcoxon
        stat, p_value = wilcoxon(delta, alternative='greater')
        results['wilcoxon_p'] = p_value
    except ImportError:
        results['wilcoxon_p'] = None

    return results


def compute_auc(road_costs, nonroad_costs):
    """
    计算 AUC（道路/非道路代价区分能力）。

    AUC = P(C_road < C_nonroad)

    Parameters
    ----------
    road_costs : np.ndarray
        道路点代价值
    nonroad_costs : np.ndarray
        非道路点代价值

    Returns
    -------
    auc : float
        AUC 值，> 0.5 表示模型能区分道路/非道路
    """
    n_road = len(road_costs)
    n_nonroad = len(nonroad_costs)

    # 计算道路代价小于非道路代价的比例
    count = 0
    for rc in road_costs:
        count += np.sum(rc < nonroad_costs)
    auc = count / (n_road * n_nonroad)
    return auc


def build_matched_negative_samples(road_points, road_directions, dem_shape,
                                     cost_field, directions, cell_size,
                                     min_distance_cells=3, seed=42):
    """
    构建匹配负样本（非道路点）。

    对每个道路点，在附近随机采样一个非道路点：
    - 距离道路至少 min_distance_cells 个像元
    - 使用道路点的方向计算负样本代价

    Parameters
    ----------
    road_points : np.ndarray
        道路点坐标 (N, 2)，每行为 (row, col)
    road_directions : np.ndarray
        道路方向（弧度）
    dem_shape : tuple
        DEM 形状
    cost_field : np.ndarray
        方向代价场
    directions : np.ndarray
        方向角度数组
    cell_size : float
        栅格大小
    min_distance_cells : int
        负样本距道路的最小距离（像元数）
    seed : int
        随机种子

    Returns
    -------
    nonroad_costs : np.ndarray
        负样本的代价值
    nonroad_points : np.ndarray
        负样本坐标
    """
    rng = np.random.RandomState(seed)
    rows, cols = dem_shape

    nonroad_costs = []
    nonroad_points = []

    for i, (rp, theta) in enumerate(zip(road_points, road_directions)):
        r, c = int(rp[0]), int(rp[1])

        # 在附近采样
        for _ in range(50):  # 最多尝试50次
            dr = rng.randint(-10, 11)
            dc = rng.randint(-10, 11)

            nr, nc = r + dr, c + dc

            # 边界检查
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue

            # 距离检查
            dist = np.sqrt(dr**2 + dc**2)
            if dist < min_distance_cells:
                continue

            # 使用道路方向计算代价（公平比较，考虑 2π 周期性）
            angle_diffs = np.abs(directions - theta)
            angle_diffs = np.minimum(angle_diffs, 2 * np.pi - angle_diffs)
            dir_idx = np.argmin(angle_diffs)
            cost = cost_field[nr, nc, dir_idx]

            nonroad_costs.append(cost)
            nonroad_points.append((nr, nc))
            break

    return np.array(nonroad_costs), np.array(nonroad_points)


def run_demo():
    """运行完整的演示流程。"""

    print("=" * 60)
    print("越野通行能力建模原型系统 v2 — 纵坡/横坡分解")
    print(f"数据源: {DATA_SOURCE}")
    print("=" * 60)

    # ================================================================
    # 步骤1：加载DEM数据
    # ================================================================
    print(f"\n[步骤1] 加载DEM数据（{DATA_SOURCE}）...")

    if DATA_SOURCE == 'real':
        # 加载真实DEM（雅江格西沟 SRTM 30m）
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        dem_path = os.path.join(project_root, REAL_DEM_PATH)
        dem, cell_size, meta = load_real_dem(dem_path, max_pixels=REAL_DEM_MAX_PIXELS)
        rows, cols = dem.shape
        print(f"  文件: {REAL_DEM_PATH}")
        print(f"  原始尺寸: {meta['original_shape']} → 降采样: {rows} × {cols}")
        print(f"  栅格大小: {cell_size:.1f}m（降采样后）")
        print(f"  CRS: {meta['crs']}")
        print(f"  覆盖范围: {meta['bounds']}")
    else:
        # 生成合成DEM
        rows, cols = 80, 80
        cell_size = 30.0
        dem = generate_synthetic_dem(rows, cols, cell_size, seed=42)
        meta = None
        print(f"  合成DEM: {rows} × {cols}, 栅格 {cell_size}m")

    print(f"  DEM尺寸: {rows} × {cols}")
    print(f"  高程范围: {dem.min():.1f} ~ {dem.max():.1f} m")
    print(f"  高差: {dem.max() - dem.min():.1f} m")

    # ================================================================
    # 步骤2：地形分析
    # ================================================================
    print("\n[步骤2] 地形分析...")

    t0 = time.time()
    p, q = calculate_gradient(dem, cell_size)
    slope = calculate_slope(p, q)
    aspect = calculate_aspect(p, q)
    print(f"  梯度计算完成 ({time.time()-t0:.3f}s)")
    print(f"  坡度范围: {np.degrees(slope.min()):.1f}° ~ {np.degrees(slope.max()):.1f}°")
    print(f"  平均坡度: {np.degrees(slope.mean()):.1f}°")

    # ================================================================
    # 步骤3：三模型对比 — 计算代价场
    # ================================================================
    print("\n[步骤3] 计算三模型代价场...")

    N_directions = 8  # 8方向（与栅格邻接图一致）
    directions = np.linspace(0, 2 * np.pi, N_directions, endpoint=False)
    print(f"  方向数: {N_directions}（间隔 {360/N_directions:.0f}°）")

    # --- B0：传统标量模型 ---
    t1 = time.time()
    cost_field_B0 = compute_scalar_cost_field(p, q, cell_size, alpha_u=15.0)
    print(f"  B0（标量）代价场: {cost_field_B0.shape}, 耗时 {time.time()-t1:.3f}s")

    # --- B1：仅纵坡方向模型 ---
    t2 = time.time()
    cost_field_B1 = compute_longitudinal_only_cost_field(
        p, q, directions, cell_size,
        alpha_u=15.0, alpha_m=5.0, alpha_d=15.0)
    print(f"  B1（仅纵坡）代价场: {cost_field_B1.shape}, 耗时 {time.time()-t2:.3f}s")

    # --- Ours：纵坡+横坡模型 ---
    t3 = time.time()
    impedance_field, cost_field_ours = compute_impedance_field(
        p, q, directions, cell_size,
        alpha_u=15.0, alpha_m=5.0, alpha_d=15.0, alpha_r=10.0)
    print(f"  Ours（纵坡+横坡）代价场: {cost_field_ours.shape}, 耗时 {time.time()-t3:.3f}s")

    # 代价场统计
    print(f"\n  代价场统计:")
    print(f"    B0:   范围 [{cost_field_B0.min():.4f}, {cost_field_B0.max():.4f}], 均值 {cost_field_B0.mean():.4f}")
    print(f"    B1:   范围 [{cost_field_B1.min():.4f}, {cost_field_B1.max():.4f}], 均值 {cost_field_B1.mean():.4f}")
    print(f"    Ours: 范围 [{cost_field_ours.min():.4f}, {cost_field_ours.max():.4f}], 均值 {cost_field_ours.mean():.4f}")

    # ================================================================
    # 步骤4：各向异性指数 AI
    # ================================================================
    print("\n[步骤4] 计算各向异性指数 AI...")

    AI_B1 = compute_anisotropy_index(cost_field_B1)
    AI_ours = compute_anisotropy_index(cost_field_ours)

    print(f"  B1 AI:   范围 [{AI_B1.min():.4f}, {AI_B1.max():.4f}], 均值 {AI_B1.mean():.4f}")
    print(f"  Ours AI: 范围 [{AI_ours.min():.4f}, {AI_ours.max():.4f}], 均值 {AI_ours.mean():.4f}")

    # ================================================================
    # 步骤5：纵坡/横坡分解验证
    # ================================================================
    print("\n[步骤5] 纵坡/横坡分解验证...")

    # 选一个代表方向（45°）展示纵坡/横坡
    theta_test = np.pi / 4  # 45°
    g_par, g_perp = compute_longitudinal_cross_slope(p, q, theta_test)

    print(f"  方向 θ = 45°:")
    print(f"    纵坡 g_∥: 范围 [{g_par.min():.4f}, {g_par.max():.4f}]")
    print(f"    横坡 g_⊥: 范围 [{g_perp.min():.4f}, {g_perp.max():.4f}]")

    # 沿等高线方向（坡向±90°）
    theta_contour = aspect + np.pi / 2  # 沿等高线
    g_par_c, g_perp_c = compute_longitudinal_cross_slope(p, q, 0.0)

    # 在山脊附近验证：沿等高线时 g_∥ ≈ 0 但 g_⊥ ≠ 0
    ridge_area = slope > np.radians(10)  # 坡度>10°的区域
    if np.any(ridge_area):
        print(f"  坡度>10°区域（沿θ=0°）:")
        print(f"    纵坡 |g_∥| 均值: {np.mean(np.abs(g_par_c[ridge_area])):.4f}")
        print(f"    横坡 |g_⊥| 均值: {np.mean(np.abs(g_perp_c[ridge_area])):.4f}")
        ratio = np.mean(np.abs(g_perp_c[ridge_area])) / (np.mean(np.abs(g_par_c[ridge_area])) + 1e-8)
        print(f"    横坡/纵坡比: {ratio:.2f}（>1说明横坡不能忽略）")

    # ================================================================
    # 步骤6：路径规划对比
    # ================================================================
    print("\n[步骤6] 路径规划对比...")

    # 根据DEM大小自动选择起止点（约15%和85%位置，Bug #6：避免边界效应）
    margin = max(5, int(rows * 0.15))
    start = (margin, margin)
    goal = (rows - margin - 1, cols - margin - 1)
    print(f"  起点: {start}, 终点: {goal}")

    # Bug #6 修复：给边界区域添加额外代价，防止路径绕边界
    boundary_penalty_width = max(3, int(rows * 0.05))  # 边界惩罚区域宽度
    for cf in [cost_field_ours, cost_field_B1]:
        for i in range(boundary_penalty_width):
            penalty = 2.0 * (1.0 - i / boundary_penalty_width)  # 线性衰减
            # 上下边界
            cf[i, :, :] += penalty
            cf[-(i+1), :, :] += penalty
            # 左右边界
            cf[:, i, :] += penalty
            cf[:, -(i+1), :] += penalty

    # B0路径（标量）
    # 将标量代价扩展为8方向相同值
    scalar_expanded = np.concatenate([cost_field_B0] * N_directions, axis=2)
    path_B0, total_B0, avg_B0 = find_optimal_path(scalar_expanded, start, goal)
    print(f"  B0 路径: {len(path_B0)} 步, 平均代价 {avg_B0:.4f}")

    # B1路径（仅纵坡）
    path_B1, total_B1, avg_B1 = find_optimal_path(cost_field_B1, start, goal)
    print(f"  B1 路径: {len(path_B1)} 步, 平均代价 {avg_B1:.4f}")

    # Ours路径（纵坡+横坡）
    path_ours, total_ours, avg_ours = find_optimal_path(cost_field_ours, start, goal)
    print(f"  Ours 路径: {len(path_ours)} 步, 平均代价 {avg_ours:.4f}")

    # 代价比
    if avg_B0 > 0:
        cr_ours_vs_B0 = avg_ours / avg_B0
        cr_B1_vs_B0 = avg_B1 / avg_B0
        print(f"\n  代价比（越低越好）:")
        print(f"    B1/B0:   {cr_B1_vs_B0:.4f}")
        print(f"    Ours/B0: {cr_ours_vs_B0:.4f}")

    # ================================================================
    # 步骤7：地形流线验证（Bug #5 修复：不再用模型自身路径作为道路）
    # ================================================================
    print("\n[步骤7] 地形流线验证...")

    # Bug #5 修复说明：
    # 旧方法用 Ours 模型自己的最优路径模拟"道路"进行验证，存在循环论证。
    # 新方法用地形梯度下降流线（物理上等价于雨水冲刷路径）作为验证基准。
    # 流线方向 θ_flow = aspect（梯度方向），不依赖任何通行能力模型。

    # 生成梯度下降流线作为验证基准
    n_streamlines = 10
    rng_stream = np.random.RandomState(seed=123)
    road_points_list = []
    road_dirs_list = []

    for _ in range(n_streamlines):
        # 随机起点（避开边界）
        sr = rng_stream.randint(int(rows*0.1), int(rows*0.9))
        sc = rng_stream.randint(int(cols*0.1), int(cols*0.9))

        # 沿梯度方向下降（物理流线，不依赖模型）
        for step in range(30):
            if sr < 0 or sr >= rows or sc < 0 or sc >= cols:
                break
            road_points_list.append((sr, sc))
            road_dirs_list.append(aspect[sr, sc])
            # 沿梯度方向（最速下降）
            p_local = p[sr, sc]
            q_local = q[sr, sc]
            mag = np.sqrt(p_local**2 + q_local**2)
            if mag < 1e-8:
                break
            sr = int(sr + q_local / mag)
            sc = int(sc + p_local / mag)

    if len(road_points_list) > 10:
        road_points = np.array(road_points_list)
        road_directions = np.array(road_dirs_list)

        # 计算流线点在三个模型中的代价
        road_costs_ours = []
        road_costs_B0 = []
        road_costs_B1 = []
        for (r, c), theta in zip(road_points, road_directions):
            r, c = int(r), int(c)
            angle_diffs = np.abs(directions - theta)
            angle_diffs = np.minimum(angle_diffs, 2 * np.pi - angle_diffs)
            dir_idx = np.argmin(angle_diffs)
            road_costs_ours.append(cost_field_ours[r, c, dir_idx])
            road_costs_B0.append(cost_field_B0[r, c, 0])
            road_costs_B1.append(cost_field_B1[r, c, dir_idx])
        road_costs_ours = np.array(road_costs_ours)
        road_costs_B0 = np.array(road_costs_B0)
        road_costs_B1 = np.array(road_costs_B1)

        # 构建匹配负样本（三模型各自的负样本）
        nonroad_ours, _ = build_matched_negative_samples(
            road_points, road_directions, dem.shape,
            cost_field_ours, directions, cell_size, seed=42)
        nonroad_B0, _ = build_matched_negative_samples(
            road_points, road_directions, dem.shape,
            cost_field_B0[:, :, np.newaxis].repeat(N_directions, axis=2),
            directions, cell_size, seed=42)
        nonroad_B1, _ = build_matched_negative_samples(
            road_points, road_directions, dem.shape,
            cost_field_B1, directions, cell_size, seed=42)

        if len(nonroad_ours) > 0:
            # AUC（三模型对比）
            auc_ours = compute_auc(road_costs_ours, nonroad_ours)
            auc_B0 = compute_auc(road_costs_B0, nonroad_B0)
            auc_B1 = compute_auc(road_costs_B1, nonroad_B1)
            print(f"  AUC（流线/非流线区分，越高越好）:")
            print(f"    B0:   {auc_B0:.4f}")
            print(f"    B1:   {auc_B1:.4f}")
            print(f"    Ours: {auc_ours:.4f}")

            # 配对代价差
            n_pairs = min(len(road_costs_ours), len(nonroad_ours))
            paired_results = compute_paired_cost_difference(
                road_costs_ours[:n_pairs], nonroad_ours[:n_pairs])
            print(f"  配对代价差 ΔC（Ours）: 均值={paired_results['mean_delta']:.4f}, "
                  f"中位数={paired_results['median_delta']:.4f}")
            print(f"    95% CI: [{paired_results['ci_95'][0]:.4f}, {paired_results['ci_95'][1]:.4f}]")
            if paired_results['wilcoxon_p'] is not None:
                print(f"    Wilcoxon p值: {paired_results['wilcoxon_p']:.6f}")

            # 方向一致性 DC
            dc_ours = compute_direction_consistency(road_points, road_directions, cost_field_ours, directions)
            dc_B1 = compute_direction_consistency(road_points, road_directions, cost_field_B1, directions)
            print(f"  方向一致性 DC（越高越好）:")
            print(f"    B1:   {dc_B1:.4f}")
            print(f"    Ours: {dc_ours:.4f}")

            # 纵坡/横坡贡献比
            road_imp_par = []
            road_imp_cross = []
            for (r_pt, c_pt), theta in zip(road_points, road_directions):
                r_pt, c_pt = int(r_pt), int(c_pt)
                g_par_full, g_perp_full = compute_longitudinal_cross_slope(p, q, theta)
                gp = np.array([g_par_full[r_pt, c_pt]])
                gq = np.array([g_perp_full[r_pt, c_pt]])
                R_par = longitudinal_impedance(gp, alpha_u=15.0, alpha_m=5.0, alpha_d=15.0)[0]
                R_cross = cross_slope_impedance(gq, alpha_r=10.0)[0]
                road_imp_par.append(R_par)
                road_imp_cross.append(R_cross)
            road_imp_par = np.array(road_imp_par)
            road_imp_cross = np.array(road_imp_cross)
            ratio = np.mean(road_imp_cross) / (np.mean(road_imp_par) + 1e-8)
            print(f"  纵坡/横坡贡献分析:")
            print(f"    流线点平均纵坡阻碍度 R_∥: {np.mean(road_imp_par):.4f}")
            print(f"    流线点平均横坡阻碍度 R_⊥: {np.mean(road_imp_cross):.4f}")
            print(f"    横坡/纵坡比: {ratio:.2f}（>1 说明横坡不可忽略）")

            # 保存验证数据供 HTML 使用
            validation_data = {
                'n_streamlines': n_streamlines,
                'n_road_points': len(road_points),
                'auc_B0': auc_B0, 'auc_B1': auc_B1, 'auc_ours': auc_ours,
                'dc_B1': dc_B1, 'dc_ours': dc_ours,
                'delta_mean': paired_results['mean_delta'],
                'delta_median': paired_results['median_delta'],
                'ci_low': paired_results['ci_95'][0],
                'ci_high': paired_results['ci_95'][1],
                'wilcoxon_p': paired_results.get('wilcoxon_p'),
                'mean_R_par': np.mean(road_imp_par),
                'mean_R_cross': np.mean(road_imp_cross),
                'cross_par_ratio': ratio,
            }
    else:
        print("  警告：流线生成不足，跳过验证")
        validation_data = None

    # ================================================================
    # 步骤8：可视化
    # ================================================================
    print("\n[步骤8] 生成可视化结果...")

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.font_manager as fm

        # 中文字体配置（与visualization.py保持一致）
        for _fname in ['SimHei', 'Microsoft YaHei', 'SimSun', 'FangSong', 'KaiTi']:
            try:
                _fpath = fm.findfont(fm.FontProperties(family=_fname), fallback_to_default=False)
                if _fpath and 'DejaVu' not in _fpath and 'fallback' not in _fpath.lower():
                    matplotlib.rcParams['font.sans-serif'] = [_fname] + matplotlib.rcParams.get('font.sans-serif', [])
                    matplotlib.rcParams['font.family'] = 'sans-serif'
                    matplotlib.rcParams['axes.unicode_minus'] = False
                    break
            except Exception:
                continue
        else:
            matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
            matplotlib.rcParams['font.family'] = 'sans-serif'
            matplotlib.rcParams['axes.unicode_minus'] = False

        output_dir = os.path.dirname(os.path.abspath(__file__))
        suffix = '_real' if DATA_SOURCE == 'real' else ''

        # --- 图1：纵坡/横坡分解示意图 ---
        fig1, axes1 = plt.subplots(1, 3, figsize=(18, 5))

        theta_vis = np.pi / 4  # 45°方向
        g_par_vis, g_perp_vis = compute_longitudinal_cross_slope(p, q, theta_vis)

        im0 = axes1[0].imshow(np.degrees(slope), cmap='YlOrRd', origin='lower')
        axes1[0].set_title('坡度 (°)')
        plt.colorbar(im0, ax=axes1[0], shrink=0.8)

        im1 = axes1[1].imshow(np.degrees(g_par_vis), cmap='RdBu_r', origin='lower')
        axes1[1].set_title(f'纵坡 g_∥ (θ={np.degrees(theta_vis):.0f}°)')
        plt.colorbar(im1, ax=axes1[1], shrink=0.8)

        im2 = axes1[2].imshow(np.degrees(np.abs(g_perp_vis)), cmap='YlOrRd', origin='lower')
        axes1[2].set_title(f'|横坡 g_⊥| (θ={np.degrees(theta_vis):.0f}°)')
        plt.colorbar(im2, ax=axes1[2], shrink=0.8)

        fig1.suptitle('纵坡/横坡分解（45°方向）', fontsize=14)
        fig1.tight_layout()
        fig1.savefig(os.path.join(output_dir, f'slope_decomposition{suffix}.png'), dpi=150, bbox_inches='tight')
        plt.close(fig1)
        print(f"  已保存: slope_decomposition{suffix}.png")

        # --- 图2：三模型最小代价场对比 ---
        fig2, axes2 = plt.subplots(1, 3, figsize=(18, 5))

        # B0：标量代价
        im_b0 = axes2[0].imshow(cost_field_B0[:, :, 0], cmap='RdYlGn_r', origin='lower')
        axes2[0].set_title('B0: 标量坡度代价')
        plt.colorbar(im_b0, ax=axes2[0], shrink=0.8)

        # B1：最小方向代价
        min_B1 = np.min(cost_field_B1, axis=2)
        im_b1 = axes2[1].imshow(min_B1, cmap='RdYlGn_r', origin='lower')
        axes2[1].set_title('B1: 仅纵坡（最小方向）')
        plt.colorbar(im_b1, ax=axes2[1], shrink=0.8)

        # Ours：最小方向代价
        min_ours = np.min(cost_field_ours, axis=2)
        im_ours = axes2[2].imshow(min_ours, cmap='RdYlGn_r', origin='lower')
        axes2[2].set_title('Ours: 纵坡+横坡（最小方向）')
        plt.colorbar(im_ours, ax=axes2[2], shrink=0.8)

        fig2.suptitle('三模型最小代价场对比', fontsize=14)
        fig2.tight_layout()
        fig2.savefig(os.path.join(output_dir, f'three_model_comparison{suffix}.png'), dpi=150, bbox_inches='tight')
        plt.close(fig2)
        print(f"  已保存: three_model_comparison{suffix}.png")

        # --- 图3：各向异性指数 AI ---
        fig3, axes3 = plt.subplots(1, 2, figsize=(14, 5))

        im_ai1 = axes3[0].imshow(AI_B1, cmap='hot', origin='lower', vmin=0, vmax=1)
        axes3[0].set_title('B1 各向异性指数 AI')
        plt.colorbar(im_ai1, ax=axes3[0], shrink=0.8)

        im_ai2 = axes3[1].imshow(AI_ours, cmap='hot', origin='lower', vmin=0, vmax=1)
        axes3[1].set_title('Ours 各向异性指数 AI')
        plt.colorbar(im_ai2, ax=axes3[1], shrink=0.8)

        fig3.suptitle('各向异性指数对比（AI越高，方向差异越显著）', fontsize=14)
        fig3.tight_layout()
        fig3.savefig(os.path.join(output_dir, f'anisotropy_index{suffix}.png'), dpi=150, bbox_inches='tight')
        plt.close(fig3)
        print(f"  已保存: anisotropy_index{suffix}.png")

        # --- 图4：路径对比 ---
        fig4, axes4 = plt.subplots(1, 3, figsize=(18, 5))

        min_ours_show = np.min(cost_field_ours, axis=2)

        for ax, path_data, title in zip(
            axes4,
            [(path_B0, 'B0'), (path_B1, 'B1'), (path_ours, 'Ours')],
            ['B0: 标量路径', 'B1: 仅纵坡路径', 'Ours: 纵坡+横坡路径']
        ):
            im = ax.imshow(min_ours_show, cmap='RdYlGn_r', origin='lower', alpha=0.7)
            path_data_arr = path_data[0]
            if len(path_data_arr) > 0:
                path_arr = np.array(path_data_arr)
                ax.plot(path_arr[:, 1], path_arr[:, 0], 'b-', linewidth=2, label=path_data[1])
            ax.plot(start[1], start[0], 'go', markersize=10, label='起点')
            ax.plot(goal[1], goal[0], 'r*', markersize=12, label='终点')
            ax.set_title(title)
            ax.legend(loc='upper left', fontsize=8)

        fig4.suptitle('三模型路径规划对比', fontsize=14)
        fig4.tight_layout()
        fig4.savefig(os.path.join(output_dir, f'path_three_models{suffix}.png'), dpi=150, bbox_inches='tight')
        plt.close(fig4)
        print(f"  已保存: path_three_models{suffix}.png")

        # --- 图5：代价玫瑰图 ---
        fig5, axes5 = plt.subplots(2, 2, figsize=(12, 12),
                                    subplot_kw={'projection': 'polar'})

        # 按DEM比例选择采样位置
        q1, q2, q3 = int(rows*0.25), int(rows*0.5), int(rows*0.75)
        sample_positions = [(q1, q1), (q2, q2), (q3, q1), (q3, q3)]
        sample_labels = [f'西南 ({q1},{q1})', f'中心 ({q2},{q2})',
                        f'东南 ({q3},{q1})', f'东北 ({q3},{q3})']

        for ax, pos, label in zip(axes5.flat, sample_positions, sample_labels):
            r, c = pos
            costs_at_pos = cost_field_ours[r, c, :]
            # 填充玫瑰图
            theta_fill = np.append(directions, directions[0])
            costs_fill = np.append(costs_at_pos, costs_at_pos[0])
            ax.fill(theta_fill, costs_fill, alpha=0.3, color='steelblue')
            ax.plot(theta_fill, costs_fill, 'o-', color='steelblue', markersize=4)
            ax.set_title(label, fontsize=10, pad=15)
            ax.set_theta_zero_location('N')
            ax.set_theta_direction(-1)

        fig5.suptitle('不同位置的代价玫瑰图（Ours模型）', fontsize=14, y=1.02)
        fig5.savefig(os.path.join(output_dir, f'cost_rose_v2{suffix}.png'), dpi=150, bbox_inches='tight')
        plt.close(fig5)
        print(f"  已保存: cost_rose_v2{suffix}.png")

        # --- 图6：最优方向场 ---
        fig6, axes6 = plt.subplots(1, 2, figsize=(14, 5))

        min_cost_show = np.min(cost_field_ours, axis=2)
        optimal_dir = directions[np.argmin(cost_field_ours, axis=2)]

        im_od1 = axes6[0].imshow(min_cost_show, cmap='RdYlGn_r', origin='lower')
        axes6[0].set_title('最小方向代价')
        plt.colorbar(im_od1, ax=axes6[0], shrink=0.8)

        im_od2 = axes6[1].imshow(np.degrees(optimal_dir), cmap='hsv', origin='lower')
        axes6[1].set_title('最优方向 θ* (°)')
        plt.colorbar(im_od2, ax=axes6[1], shrink=0.8)

        fig6.suptitle('最优代价与最优方向场', fontsize=14)
        fig6.tight_layout()
        fig6.savefig(os.path.join(output_dir, f'optimal_fields_v2{suffix}.png'), dpi=150, bbox_inches='tight')
        plt.close(fig6)
        print(f"  已保存: optimal_fields_v2{suffix}.png")

    except ImportError as e:
        print(f"  跳过可视化（缺少matplotlib）: {e}")
    except Exception as e:
        print(f"  可视化出错: {e}")
        import traceback
        traceback.print_exc()

    # ================================================================
    # 完成
    # ================================================================
    print("\n" + "=" * 60)
    print("演示完成！")
    print("=" * 60)

    return {
        'dem': dem,
        'p': p, 'q': q, 'slope': slope, 'aspect': aspect,
        'cell_size': cell_size,
        'directions': directions,
        'cost_field_B0': cost_field_B0,
        'cost_field_B1': cost_field_B1,
        'cost_field_ours': cost_field_ours,
        'impedance_field': impedance_field,
        'AI_B1': AI_B1,
        'AI_ours': AI_ours,
        'path_B0': path_B0,
        'path_B1': path_B1,
        'path_ours': path_ours,
        'validation_data': validation_data if 'validation_data' in dir() else None,
        'start': start,
        'goal': goal,
    }


if __name__ == '__main__':
    results = run_demo()
