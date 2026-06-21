"""
地形分析模块

实现DEM数据加载、梯度/坡度/坡向计算、曲率分析、特征线提取等功能。
基于Zevenbergen & Thorne (1987) 的偏四次曲面拟合方法。
"""

import numpy as np
from scipy import ndimage


def load_dem(filepath):
    """
    加载DEM数据。

    支持GeoTIFF格式（需要rasterio）和NumPy格式（.npy）。
    如果文件不存在或格式不支持，尝试用numpy加载。

    Parameters
    ----------
    filepath : str
        DEM文件路径

    Returns
    -------
    dem : np.ndarray
        高程数组 (rows, cols)
    metadata : dict
        元数据，包含：
        - cell_size: 栅格大小（米）
        - transform: 仿射变换（如有）
        - crs: 坐标参考系统（如有）
    """
    metadata = {}

    if filepath.endswith('.tif') or filepath.endswith('.tiff'):
        try:
            import rasterio
            with rasterio.open(filepath) as src:
                dem = src.read(1).astype(np.float64)
                transform = src.transform
                metadata['cell_size'] = transform.a  # 像素宽度
                metadata['transform'] = transform
                metadata['crs'] = src.crs
        except ImportError:
            raise ImportError("需要安装 rasterio 库来读取 GeoTIFF: pip install rasterio")
    elif filepath.endswith('.npy'):
        dem = np.load(filepath).astype(np.float64)
        metadata['cell_size'] = 30.0  # 默认30m
    else:
        # 尝试作为numpy文本加载
        dem = np.loadtxt(filepath).astype(np.float64)
        metadata['cell_size'] = 30.0

    # 处理无效值
    if np.any(dem < -1000):
        dem[dem < -1000] = np.nan

    return dem, metadata


def calculate_gradient(dem, cell_size):
    """
    计算梯度向量（使用中心差分法）。

    公式：
        p = ∂z/∂x  （x方向梯度分量，沿列方向）
        q = ∂z/∂y  （y方向梯度分量，沿行方向）

    中心差分法：
        p[i,j] = (z[i,j+1] - z[i,j-1]) / (2 * cell_size)
        q[i,j] = (z[i+1,j] - z[i-1,j]) / (2 * cell_size)

    边界采用前向/后向差分。

    Parameters
    ----------
    dem : np.ndarray
        高程数组 (rows, cols)
    cell_size : float
        栅格大小（米）

    Returns
    -------
    p : np.ndarray
        x方向梯度分量 ∂z/∂x (rows, cols)
    q : np.ndarray
        y方向梯度分量 ∂z/∂y (rows, cols)
    """
    # 使用numpy梯度函数（中心差分，边界用前向/后向差分）
    q, p = np.gradient(dem, cell_size)
    # np.gradient 返回 (行方向梯度, 列方向梯度)
    # 行方向对应y，列方向对应x
    return p, q


def calculate_slope(p, q):
    """
    计算坡度。

    公式：s = arctan(sqrt(p² + q²))

    Parameters
    ----------
    p : np.ndarray
        x方向梯度分量
    q : np.ndarray
        y方向梯度分量

    Returns
    -------
    slope : np.ndarray
        坡度值（弧度），范围 [0, π/2]
    """
    return np.arctan(np.sqrt(p ** 2 + q ** 2))


def calculate_aspect(p, q):
    """
    计算坡向（梯度方向与x轴的夹角）。

    公式：φ = arctan2(q, p)

    物理意义：最大坡度方向（最陡上升方向）相对于x轴的角度。

    Parameters
    ----------
    p : np.ndarray
        x方向梯度分量
    q : np.ndarray
        y方向梯度分量

    Returns
    -------
    aspect : np.ndarray
        坡向（弧度），范围 [-π, π]
    """
    return np.arctan2(q, p)


def calculate_curvature(dem, cell_size):
    """
    计算曲率分量（使用Zevenbergen & Thorne方法）。

    拟合偏四次曲面：
        z = A*x²*y² + B*x²*y + C*x*y² + D*x² + E*y² + F*x*y + G*x + H*y + I

    二阶偏导数：
        r = ∂²z/∂x² = 2D
        t = ∂²z/∂y² = 2E
        s = ∂²z/∂x∂y = F

    Parameters
    ----------
    dem : np.ndarray
        高程数组 (rows, cols)
    cell_size : float
        栅格大小（米）

    Returns
    -------
    r : np.ndarray
        ∂²z/∂x² (rows, cols)
    t : np.ndarray
        ∂²z/∂y² (rows, cols)
    s : np.ndarray
        ∂²z/∂x∂y (rows, cols)
    """
    rows, cols = dem.shape
    delta = cell_size

    # Zevenbergen & Thorne 系数计算
    # 使用3×3窗口，参考点命名：
    # z1 z2 z3    (左上  上  右上)
    # z4 z5 z6    (左   中  右)
    # z7 z8 z9    (左下  下  右下)

    # 填充边界（使用边缘值复制）
    dem_pad = np.pad(dem, 1, mode='edge')

    z1 = dem_pad[0:rows, 0:cols]
    z2 = dem_pad[0:rows, 1:cols+1]
    z3 = dem_pad[0:rows, 2:cols+2]
    z4 = dem_pad[1:rows+1, 0:cols]
    z5 = dem_pad[1:rows+1, 1:cols+1]  # 中心点
    z6 = dem_pad[1:rows+1, 2:cols+2]
    z7 = dem_pad[2:rows+2, 0:cols]
    z8 = dem_pad[2:rows+2, 1:cols+1]
    z9 = dem_pad[2:rows+2, 2:cols+2]

    delta2 = delta ** 2

    # 二阶偏导数（Zevenbergen & Thorne 公式）
    # r = ∂²z/∂x²
    r = (z4 + z6 - 2 * z5) / delta2

    # t = ∂²z/∂y²
    t = (z2 + z8 - 2 * z5) / delta2

    # s = ∂²z/∂x∂y
    s = (z1 - z3 - z7 + z9) / (4 * delta2)

    return r, t, s


def calculate_principal_curvatures(r, t, s):
    """
    计算主曲率和主方向。

    Hessian矩阵 H = [[r, s], [s, t]]，主曲率为H的特征值。

    公式：
        κ1 = (r+t + sqrt((r-t)² + 4s²)) / 2   （最大曲率）
        κ2 = (r+t - sqrt((r-t)² + 4s²)) / 2   （最小曲率）
        θ_principal = 0.5 * arctan(2s / (r-t))  （主方向）

    Parameters
    ----------
    r : np.ndarray
        ∂²z/∂x²
    t : np.ndarray
        ∂²z/∂y²
    s : np.ndarray
        ∂²z/∂x∂y

    Returns
    -------
    kappa1 : np.ndarray
        最大曲率（κ1 ≥ κ2）
    kappa2 : np.ndarray
        最小曲率
    theta_principal : np.ndarray
        主方向（弧度），即最大曲率对应的方向
    """
    sum_rt = r + t
    diff_rt = r - t
    discriminant = np.sqrt(diff_rt ** 2 + 4 * s ** 2)

    kappa1 = (sum_rt + discriminant) / 2.0
    kappa2 = (sum_rt - discriminant) / 2.0

    # 主方向：使用 arctan2 处理特殊情况
    theta_principal = 0.5 * np.arctan2(2 * s, diff_rt)

    return kappa1, kappa2, theta_principal


def calculate_directional_curvature(kappa1, kappa2, theta_principal, theta):
    """
    计算任意方向的法曲率（Euler公式）。

    公式：
        κ_n(θ) = κ1 * cos²(θ - θ_principal) + κ2 * sin²(θ - θ_principal)

    推导：设主方向 e1, e2，任意方向 v = (cosθ, sinθ)
    在主方向上的投影：
        v1 = cos(θ - θ_principal)
        v2 = sin(θ - θ_principal)
    法曲率：κ_n = κ1 * v1² + κ2 * v2²

    Parameters
    ----------
    kappa1 : np.ndarray
        最大曲率
    kappa2 : np.ndarray
        最小曲率
    theta_principal : np.ndarray
        主方向（弧度）
    theta : float or np.ndarray
        目标方向（弧度）

    Returns
    -------
    kappa_n : np.ndarray
        目标方向的法曲率
    """
    dtheta = theta - theta_principal
    kappa_n = kappa1 * np.cos(dtheta) ** 2 + kappa2 * np.sin(dtheta) ** 2
    return kappa_n


def extract_feature_lines(dem, cell_size, threshold):
    """
    提取地形特征线（山脊线、山谷线）。

    基于最大曲率 κ1 的阈值判断：
        山脊线：κ1 < -threshold（凸曲率，地形向上弯曲）
        山谷线：κ1 > threshold（凹曲率，地形向下弯曲）

    Parameters
    ----------
    dem : np.ndarray
        高程数组 (rows, cols)
    cell_size : float
        栅格大小（米）
    threshold : float
        曲率阈值（正值）

    Returns
    -------
    ridge_points : np.ndarray
        山脊线坐标数组 (N, 2)，每行为 (row, col)
    valley_points : np.ndarray
        山谷线坐标数组 (M, 2)，每行为 (row, col)
    ridge_directions : np.ndarray
        山脊线各点的主方向（弧度）
    valley_directions : np.ndarray
        山谷线各点的主方向（弧度）
    """
    r, t, s = calculate_curvature(dem, cell_size)
    kappa1, kappa2, theta_principal = calculate_principal_curvatures(r, t, s)

    # 山脊线：最大曲率为负且绝对值大于阈值
    ridge_mask = kappa1 < -threshold
    ridge_rows, ridge_cols = np.where(ridge_mask)
    ridge_points = np.column_stack([ridge_rows, ridge_cols])
    ridge_directions = theta_principal[ridge_mask]

    # 山谷线：最大曲率为正且大于阈值
    valley_mask = kappa1 > threshold
    valley_rows, valley_cols = np.where(valley_mask)
    valley_points = np.column_stack([valley_rows, valley_cols])
    valley_directions = theta_principal[valley_mask]

    return ridge_points, valley_points, ridge_directions, valley_directions


def compute_distance_to_features(grid_shape, feature_points, cell_size):
    """
    计算栅格点到最近特征线的距离。

    对每个栅格点，计算到所有特征线点的最小欧氏距离。
    使用scipy距离变换实现高效计算。

    Parameters
    ----------
    grid_shape : tuple
        栅格形状 (rows, cols)
    feature_points : np.ndarray
        特征线点坐标 (N, 2)，每行为 (row, col)
    cell_size : float
        栅格大小（米）

    Returns
    -------
    distance : np.ndarray
        到最近特征线的距离（米），形状为 (rows, cols)
    """
    rows, cols = grid_shape

    if feature_points is None or len(feature_points) == 0:
        return np.full((rows, cols), np.inf)

    # 创建特征线的二值掩码
    feature_mask = np.zeros((rows, cols), dtype=bool)
    valid = (feature_points[:, 0] >= 0) & (feature_points[:, 0] < rows) & \
            (feature_points[:, 1] >= 0) & (feature_points[:, 1] < cols)
    valid_points = feature_points[valid].astype(int)
    feature_mask[valid_points[:, 0], valid_points[:, 1]] = True

    # 使用距离变换计算到特征线的距离
    distance_pixels = ndimage.distance_transform_edt(~feature_mask)
    distance = distance_pixels * cell_size

    return distance


def compute_longitudinal_cross_slope(p, q, theta):
    """
    将 DEM 梯度投影到车辆行驶方向，分解为纵坡和横坡。

    公式：
        g_∥(x,y,θ) = ∇z · u_θ = p·cos(θ) + q·sin(θ)    （纵坡）
        g_⊥(x,y,θ) = ∇z · n_θ = -p·sin(θ) + q·cos(θ)   （横坡）

    其中：
        u_θ = (cos θ, sin θ)  — 行驶方向单位向量
        n_θ = (-sin θ, cos θ) — 横向单位向量

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
        横向坡度（带符号）
    """
    g_parallel = p * np.cos(theta) + q * np.sin(theta)
    g_perp = -p * np.sin(theta) + q * np.cos(theta)
    return g_parallel, g_perp
