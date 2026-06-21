"""
代价计算模块（重构版 v2）

基于 GPT 审阅建议，将通行能力建模从"代价乘法模型"重构为"阻碍度框架"：
- 阻碍度 R ∈ [0, 1]：0 = 完全畅通，1 = 完全不可通行
- 通过能力 P = 1 - R
- 边代价 C_edge = l_θ · [-ln(P + ε)]：低阻碍→代价≈0，高阻碍→代价→∞

核心创新：纵坡/横坡分解
- 纵坡 g_∥ = ∇z · u_θ：车辆前进方向的坡度分量（牵引/制动）
- 横坡 g_⊥ = ∇z · n_θ：车辆侧向的坡度分量（侧翻/侧滑风险）
- 沿等高线时 g_∥ ≈ 0 但 g_⊥ 可能很大 → 不能简单认为"沿等高线最优"

参考文献：
- GPT 审阅文档 review_and_exp1_design.html
- Suvinen (2006)：物理力学模型
- Pundir (2022)：多因子综合
"""

import numpy as np


# ============================================================
# 纵坡/横坡分解
# ============================================================

def compute_longitudinal_cross_slope(p, q, theta):
    """
    将 DEM 梯度投影到车辆行驶方向，分解为纵坡和横坡。

    公式：
        g_∥(x,y,θ) = ∇z · u_θ = p·cos(θ) + q·sin(θ)    （纵坡）
        g_⊥(x,y,θ) = ∇z · n_θ = -p·sin(θ) + q·cos(θ)   （横坡）

    其中：
        u_θ = (cos θ, sin θ)  — 行驶方向单位向量
        n_θ = (-sin θ, cos θ) — 横向单位向量（左手法则）

    物理意义：
        - θ = φ（沿最大坡降方向上坡）：g_∥ = |∇z|, g_⊥ = 0
        - θ = φ ± π/2（沿等高线）：g_∥ ≈ 0, g_⊥ ≈ |∇z|
        - θ = φ + π（沿最大坡降方向下坡）：g_∥ = -|∇z|, g_⊥ = 0

    Parameters
    ----------
    p : np.ndarray
        x方向梯度分量 ∂z/∂x
    q : np.ndarray
        y方向梯度分量 ∂z/∂y
    theta : float
        行驶方向（弧度，相对于x轴）

    Returns
    -------
    g_parallel : np.ndarray
        纵向坡度（带符号，正=上坡，负=下坡）
    g_perp : np.ndarray
        横向坡度（带符号，左正右负或反之）
    """
    g_parallel = p * np.cos(theta) + q * np.sin(theta)
    g_perp = -p * np.sin(theta) + q * np.cos(theta)
    return g_parallel, g_perp


# ============================================================
# 纵坡阻碍度（三段式非对称函数）
# ============================================================

def longitudinal_impedance(g_parallel, alpha_u=15.0, alpha_m=5.0, alpha_d=15.0):
    """
    纵坡阻碍度函数 R_∥ ∈ [0, 1]。

    三段式设计：
        上坡 (g_∥ > 0)：
            R_∥ = 1 - exp(-(g_∥ / tan(α_u))²)
            坡度越大，牵引需求越大，阻碍越强。

        缓下坡 (-tan(α_m) ≤ g_∥ ≤ 0)：
            R_∥ = 0
            轻微下坡不增加阻碍，甚至可能降低能耗。

        陡下坡 (g_∥ < -tan(α_m))：
            R_∥ = 1 - exp(-(|g_∥| - tan(α_m))² / tan(α_d)²)
            坡度过陡会增加制动需求和失稳风险。

    维度说明（重要！）：
        输入 g_parallel 来自 gradient (p, q)，物理含义是 tan(坡度角)。
        因此尺度参数也统一用 tan(α) 而非 α（弧度），
        避免 tan 值与弧度值混比较导致的维度不匹配。

    Parameters
    ----------
    g_parallel : np.ndarray
        纵向坡度 tan(β)，正值为上坡，负值为下坡（无量纲）
    alpha_u : float
        上坡阻碍尺度角（度），默认15°。含义：当 tan(坡度)=tan(α_u) 时 R≈0.632
    alpha_m : float
        缓下坡容许角（度），默认5°。在此角度内的下坡不增加阻碍。
    alpha_d : float
        陡下坡风险尺度角（度），默认15°。超过 α_m 后阻碍增长的尺度参数。

    Returns
    -------
    R_parallel : np.ndarray
        纵坡阻碍度，范围 [0, 1]
    """
    # 将角度参数转为 tan 值，与输入 g_parallel 维度一致
    tan_u = np.tan(np.radians(alpha_u))
    tan_m = np.tan(np.radians(alpha_m))
    tan_d = np.tan(np.radians(alpha_d))

    R_parallel = np.zeros_like(g_parallel, dtype=float)

    # 上坡：g_∥ > 0
    uphill = g_parallel > 0
    R_parallel[uphill] = 1.0 - np.exp(-(g_parallel[uphill] / tan_u) ** 2)

    # 缓下坡：-tan(α_m) ≤ g_∥ ≤ 0 → R = 0（已是默认值）

    # 陡下坡：g_∥ < -tan(α_m)
    steep_downhill = g_parallel < -tan_m
    excess = np.abs(g_parallel[steep_downhill]) - tan_m
    R_parallel[steep_downhill] = 1.0 - np.exp(-(excess / tan_d) ** 2)

    return R_parallel


# ============================================================
# 横坡阻碍度
# ============================================================

def cross_slope_impedance(g_perp, alpha_r=10.0):
    """
    横坡阻碍度函数 R_⊥ ∈ [0, 1]。

    公式：
        R_⊥ = 1 - exp(-(|g_⊥| / tan(α_r))²)

    物理意义：
        横坡越大，车辆侧向稳定性风险越高。
        左倾和右倾的风险基本对称，所以取绝对值。

    为什么横坡必须单独建模？
        只用纵坡会错误地认为沿等高线方向总是低代价。
        但沿等高线时纵坡接近0，横坡可能接近地形实际坡度。
        横坡越大，车辆侧滑/横滚/侧翻风险越高。

    维度说明：
        输入 g_perp 来自梯度投影，物理含义是 tan(横向坡度角)。
        尺度参数统一用 tan(α_r)，与输入维度一致。

    Parameters
    ----------
    g_perp : np.ndarray
        横向坡度 tan(横向坡度角)，无量纲
    alpha_r : float
        横坡风险尺度角（度），默认10°。
        横坡对稳定性更敏感，尺度应小于或接近上坡尺度。

    Returns
    -------
    R_cross : np.ndarray
        横坡阻碍度，范围 [0, 1]
    """
    tan_r = np.tan(np.radians(alpha_r))
    R_cross = 1.0 - np.exp(-(np.abs(g_perp) / tan_r) ** 2)
    return R_cross


# ============================================================
# 综合方向坡度阻碍度
# ============================================================

def directional_slope_impedance(g_parallel, g_perp, alpha_u=15.0,
                                 alpha_m=5.0, alpha_d=15.0, alpha_r=10.0):
    """
    综合方向坡度阻碍度（纵坡 + 横坡非补偿合成）。

    公式：
        R_slope(x,y,θ) = 1 - (1 - R_∥)(1 - R_⊥)

    非补偿合成的含义：
        - 只要纵坡或横坡任一阻碍很高，综合阻碍就很高
        - 不会出现"纵坡低+横坡高=总体低"的补偿效应
        - 数学性质：R ∈ [0, 1]，且 R ≥ max(R_∥, R_⊥)

    Parameters
    ----------
    g_parallel : np.ndarray
        纵向坡度（弧度）
    g_perp : np.ndarray
        横向坡度（弧度）
    alpha_u, alpha_m, alpha_d, alpha_r : float
        阻碍尺度参数（度）

    Returns
    -------
    R_slope : np.ndarray
        综合方向坡度阻碍度，范围 [0, 1]
    """
    R_par = longitudinal_impedance(g_parallel, alpha_u, alpha_m, alpha_d)
    R_cross = cross_slope_impedance(g_perp, alpha_r)

    # 非补偿合成
    R_slope = 1.0 - (1.0 - R_par) * (1.0 - R_cross)

    return R_slope


# ============================================================
# 从阻碍度转为边代价
# ============================================================

def edge_cost(R, eps=1e-6):
    """
    将阻碍度转换为路径规划可用的单位距离代价。

    公式：
        c(x,y,θ) = -ln(1 - R + ε)

    其中：
        - R ∈ [0, 1] 是阻碍度
        - 1 - R 可理解为相对通过能力 P
        - -ln(P) 将通过能力转换为可累加的代价
        - ε 是防止 ln(0) 的数值稳定项

    注意：此函数返回的是「单位距离代价」（每米代价）。
    实际边代价 = 单位距离代价 × 移动距离，由路径规划模块负责乘以距离。

    Bug #2 修复说明：
        旧版 edge_cost 内部乘以 l_theta，但 A* 中 move_cost 又乘以 move_dist，
        导致边长被重复计算（平方关系）。现统一由 A* 负责乘距离。

    物理意义：
        - R = 0（完全畅通）→ P = 1 → c = -ln(1) = 0
        - R = 0.5 → P = 0.5 → c = 0.693
        - R = 0.9 → P = 0.1 → c = 2.303
        - R → 1（不可通行）→ P → 0 → c → ∞

    Parameters
    ----------
    R : np.ndarray
        阻碍度，范围 [0, 1]
    eps : float
        数值稳定项，默认 1e-6

    Returns
    -------
    c : np.ndarray
        单位距离代价（≥ 0），无量纲（代价/米）
    """
    P = np.clip(1.0 - R, eps, None)
    c = -np.log(P)
    return c


# ============================================================
# 传统坡度标量阻碍度（Baseline B0）
# ============================================================

def scalar_slope_impedance(slope, alpha_u=15.0):
    """
    传统坡度标量阻碍度（Baseline B0）。

    公式：
        R_scalar = 1 - exp(-(tan(β) / tan(α_u))²)

    这是传统方法：每个栅格只有一个坡度值，不区分方向。

    维度说明：
        输入 slope 来自 arctan(sqrt(p²+q²))，是弧度值。
        内部转为 tan 值以与其他函数维度一致。

    Parameters
    ----------
    slope : np.ndarray
        坡度（弧度），范围 [0, π/2]
    alpha_u : float
        阻碍尺度角（度）

    Returns
    -------
    R_scalar : np.ndarray
        标量坡度阻碍度，范围 [0, 1]
    """
    tan_u = np.tan(np.radians(alpha_u))
    tan_slope = np.tan(slope)  # 将弧度坡度转为 tan 值
    R_scalar = 1.0 - np.exp(-(tan_slope / tan_u) ** 2)
    return R_scalar


# ============================================================
# 仅纵坡阻碍度（Baseline B1）
# ============================================================

def longitudinal_only_impedance(g_parallel, alpha_u=15.0, alpha_m=5.0, alpha_d=15.0):
    """
    仅纵坡方向阻碍度（Baseline B1）。

    不考虑横坡，只用纵坡分量。
    用于对比：验证加入横坡后是否更合理。

    Parameters
    ----------
    g_parallel : np.ndarray
        纵向坡度（弧度）
    alpha_u, alpha_m, alpha_d : float
        阻碍尺度参数（度）

    Returns
    -------
    R_longitudinal : np.ndarray
        仅纵坡阻碍度，范围 [0, 1]
    """
    return longitudinal_impedance(g_parallel, alpha_u, alpha_m, alpha_d)


# ============================================================
# 计算方向阻碍度场（主函数）
# ============================================================

def compute_impedance_field(p, q, directions, cell_size,
                            alpha_u=15.0, alpha_m=5.0, alpha_d=15.0,
                            alpha_r=10.0):
    """
    计算方向相关的阻碍度场和边代价场。

    对每个方向 θ：
    1. 分解梯度为纵坡 g_∥ 和横坡 g_⊥
    2. 计算纵坡阻碍度 R_∥
    3. 计算横坡阻碍度 R_⊥
    4. 非补偿合成 R_slope = 1 - (1-R_∥)(1-R_⊥)
    5. 转换为边代价 C_edge = l_θ · [-ln(1-R+ε)]

    Parameters
    ----------
    p : np.ndarray
        x方向梯度 ∂z/∂x (rows, cols)
    q : np.ndarray
        y方向梯度 ∂z/∂y (rows, cols)
    directions : np.ndarray
        方向角度数组（弧度）
    cell_size : float
        栅格大小（米）
    alpha_u : float
        上坡阻碍尺度（度）
    alpha_m : float
        缓下坡容许区间（度）
    alpha_d : float
        陡下坡风险尺度（度）
    alpha_r : float
        横坡风险尺度（度）

    Returns
    -------
    impedance_field : np.ndarray
        阻碍度场 (rows, cols, N_directions)，值域 [0, 1]
    cost_field : np.ndarray
        边代价场 (rows, cols, N_directions)，值域 [0, +∞)
    """
    rows, cols = p.shape
    N = len(directions)

    impedance_field = np.zeros((rows, cols, N))
    cost_field = np.zeros((rows, cols, N))

    for i, theta in enumerate(directions):
        # 1. 纵坡/横坡分解
        g_parallel, g_perp = compute_longitudinal_cross_slope(p, q, theta)

        # 2. 计算综合方向坡度阻碍度
        R_slope = directional_slope_impedance(
            g_parallel, g_perp, alpha_u, alpha_m, alpha_d, alpha_r)

        # 3. 转换为单位距离代价（不乘 l_theta，由 A* 负责乘距离）
        C = edge_cost(R_slope)

        impedance_field[:, :, i] = R_slope
        cost_field[:, :, i] = C

    return impedance_field, cost_field


def compute_scalar_cost_field(p, q, cell_size, alpha_u=15.0):
    """
    计算传统标量坡度代价场（Baseline B0）。

    每个栅格只有一个代价值（不区分方向），但为了与方向模型兼容，
    返回 (rows, cols, 1) 的数组。

    Parameters
    ----------
    p : np.ndarray
        x方向梯度
    q : np.ndarray
        y方向梯度
    cell_size : float
        栅格大小（米）
    alpha_u : float
        阻碍尺度（度）

    Returns
    -------
    cost_field : np.ndarray
        标量代价场 (rows, cols, 1)
    """
    from .terrain_analysis import calculate_slope
    slope = calculate_slope(p, q)
    R = scalar_slope_impedance(slope, alpha_u)
    C = edge_cost(R)
    return C[:, :, np.newaxis]


def compute_longitudinal_only_cost_field(p, q, directions, cell_size,
                                          alpha_u=15.0, alpha_m=5.0, alpha_d=15.0):
    """
    计算仅纵坡的代价场（Baseline B1）。

    不考虑横坡，只用纵坡分量计算方向代价。

    Parameters
    ----------
    p : np.ndarray
        x方向梯度
    q : np.ndarray
        y方向梯度
    directions : np.ndarray
        方向角度数组（弧度）
    cell_size : float
        栅格大小（米）
    alpha_u, alpha_m, alpha_d : float
        阻碍尺度参数（度）

    Returns
    -------
    cost_field : np.ndarray
        仅纵坡代价场 (rows, cols, N_directions)
    """
    rows, cols = p.shape
    N = len(directions)
    cost_field = np.zeros((rows, cols, N))

    for i, theta in enumerate(directions):
        g_parallel, _ = compute_longitudinal_cross_slope(p, q, theta)

        R_par = longitudinal_impedance(g_parallel, alpha_u, alpha_m, alpha_d)
        cost_field[:, :, i] = edge_cost(R_par)

    return cost_field


# ============================================================
# 以下保留旧版函数供兼容（将在后续清理）
# ============================================================

def land_cover_cost(land_cover_type):
    """
    计算地表覆盖代价（查表法）。
    注意：此函数返回的是阻碍度修正系数，需要转换后使用。
    """
    cost_table = {
        10: 0.0,   # 耕地 - 无额外阻碍
        20: 0.4,   # 林地 - 中等阻碍
        30: 0.1,   # 草地 - 轻微阻碍
        40: 0.35,  # 灌木 - 中等阻碍
        50: 0.6,   # 湿地 - 高阻碍
        60: 0.99,  # 水体 - 几乎不可通行
        70: 0.2,   # 苔原 - 轻微阻碍
        80: 0.0,   # 人造地表 - 无额外阻碍
        90: 0.05,  # 裸地 - 轻微阻碍
        100: 0.8,  # 冰川 - 高阻碍
    }

    R_extra = np.zeros_like(land_cover_type, dtype=float)
    for code, r in cost_table.items():
        mask = land_cover_type == code
        R_extra[mask] = r

    return R_extra


def soil_impedance(RCI, k=50.0):
    """
    土壤承载力阻碍度。
    R_soil = max(0, 1 - RCI/k)，值域 [0, 1]。
    """
    R_soil = np.clip(1.0 - RCI / k, 0.0, 1.0)
    R_soil = np.where(RCI > 0, R_soil, 0.99)
    return R_soil


def water_impedance(d_water, mu=0.01):
    """
    水系障碍阻碍度。
    R_water = exp(-μ · d)，值域 (0, 1]。
    d=0 时 R→1（不可通行），d→∞ 时 R→0（无影响）。
    """
    R_water = np.exp(-mu * d_water)
    return np.clip(R_water, 0.0, 0.999)


# ============================================================
# 旧版兼容接口（标记为 deprecated）
# ============================================================

def slope_cost_eff(s, theta, phi, beta):
    """[已废弃] 使用 directional_slope_impedance 替代。"""
    import warnings
    warnings.warn("slope_cost_eff 已废弃，请使用 directional_slope_impedance",
                  DeprecationWarning)
    s_eff = s * np.cos(theta - phi)
    return np.exp(beta * np.abs(s_eff))


def total_cost(C_s, C_c, C_l, C_soil, C_w, C_ridge=1.0, C_valley=1.0):
    """[已废弃] 旧版乘法模型，保留供参考。"""
    import warnings
    warnings.warn("total_cost 已废弃，请使用 compute_impedance_field",
                  DeprecationWarning)
    return C_s * C_c * C_l * C_soil * C_w * C_ridge * C_valley
