"""
通用工具函数模块

提供角度处理、滤波等基础工具函数。
"""

import numpy as np


def angle_difference(angle1, angle2):
    """
    计算两个角度的差值（处理2π周期性）。

    将差值映射到 [-π, π] 范围内。

    Parameters
    ----------
    angle1 : float or np.ndarray
        第一个角度（弧度）
    angle2 : float or np.ndarray
        第二个角度（弧度）

    Returns
    -------
    diff : float or np.ndarray
        角度差值，范围 [-π, π]
    """
    diff = angle1 - angle2
    return (diff + np.pi) % (2 * np.pi) - np.pi


def normalize_angle(angle):
    """
    角度归一化到 [0, 2π) 范围。

    Parameters
    ----------
    angle : float or np.ndarray
        输入角度（弧度）

    Returns
    -------
    normalized : float or np.ndarray
        归一化后的角度
    """
    return angle % (2 * np.pi)


def moving_average(data, window_size):
    """
    移动平均滤波。

    使用卷积实现，边界采用反射填充。

    Parameters
    ----------
    data : np.ndarray
        输入数据（1D）
    window_size : int
        窗口大小（奇数）

    Returns
    -------
    smoothed : np.ndarray
        滤波后的数据（与输入等长）
    """
    if window_size < 1:
        raise ValueError("window_size must be >= 1")
    if window_size == 1:
        return data.copy()
    if window_size % 2 == 0:
        window_size += 1  # 强制为奇数

    half = window_size // 2
    padded = np.pad(data, half, mode='reflect')
    kernel = np.ones(window_size) / window_size
    smoothed = np.convolve(padded, kernel, mode='valid')
    return smoothed


def gaussian_filter_1d(data, sigma):
    """
    一维高斯滤波。

    Parameters
    ----------
    data : np.ndarray
        输入数据（1D）
    sigma : float
        高斯核标准差

    Returns
    -------
    filtered : np.ndarray
        滤波后的数据
    """
    if sigma <= 0:
        return data.copy()

    # 构建高斯核，截断范围 ±3σ
    radius = int(np.ceil(3 * sigma))
    x = np.arange(-radius, radius + 1, dtype=float)
    kernel = np.exp(-x ** 2 / (2 * sigma ** 2))
    kernel /= kernel.sum()

    padded = np.pad(data, radius, mode='reflect')
    filtered = np.convolve(padded, kernel, mode='valid')
    return filtered


def load_real_dem(tif_path, max_pixels=None):
    """
    加载真实 GeoTIFF DEM 数据。

    自动处理：坐标系转米制分辨率、NoData 填充、可选裁剪。

    Parameters
    ----------
    tif_path : str
        GeoTIFF 文件路径
    max_pixels : int, optional
        最大边长（像素），超过则等间隔采样缩小。None 表示不缩小。

    Returns
    -------
    dem : np.ndarray
        高程数组 (rows, cols)
    cell_size : float
        实际栅格大小（米）
    meta : dict
        元信息（bounds, crs, res_deg 等）
    """
    import os
    if not os.path.exists(tif_path):
        raise FileNotFoundError(f"DEM 文件不存在: {tif_path}")

    try:
        import rasterio
    except ImportError:
        raise ImportError("需要 rasterio 库：pip install rasterio")

    with rasterio.open(tif_path) as src:
        arr = src.read(1).astype(float)
        nodata = src.nodata

        # 处理 NoData
        if nodata is not None:
            arr[arr == nodata] = np.nan

        # 填充 NaN（用最近邻）
        if np.any(np.isnan(arr)):
            from scipy.ndimage import distance_transform_edt
            _, indices = distance_transform_edt(np.isnan(arr), return_indices=True)
            arr = arr[tuple(indices)]

        # 计算米制分辨率
        lat_center = (src.bounds.bottom + src.bounds.top) / 2
        m_per_deg_lat = 111132.92
        m_per_deg_lon = 111132.92 * np.cos(np.radians(lat_center))
        res_x = src.res[0] * m_per_deg_lon  # 列方向（米）
        res_y = src.res[1] * m_per_deg_lat  # 行方向（米）
        cell_size = (res_x + res_y) / 2  # 取平均

        meta = {
            'bounds': src.bounds,
            'crs': str(src.crs),
            'res_deg': src.res,
            'res_x_m': res_x,
            'res_y_m': res_y,
            'm_per_deg_lon': m_per_deg_lon,
            'm_per_deg_lat': m_per_deg_lat,
            'lat_center': lat_center,
            'original_shape': arr.shape,
        }

        # 可选缩小
        if max_pixels is not None:
            rows, cols = arr.shape
            if rows > max_pixels or cols > max_pixels:
                step = max(rows, cols) // max_pixels
                arr = arr[::step, ::step]
                cell_size *= step
                meta['downsample_step'] = step
                meta['downsampled_shape'] = arr.shape

    return arr, cell_size, meta


def generate_synthetic_dem(rows=100, cols=100, cell_size=30.0, seed=42):
    """
    生成合成DEM数据，用于演示和测试。

    包含山脊、山谷等地形特征。

    Parameters
    ----------
    rows : int
        行数
    cols : int
        列数
    cell_size : float
        栅格大小（米）
    seed : int
        随机种子

    Returns
    -------
    dem : np.ndarray
        高程数组 (rows, cols)
    """
    rng = np.random.RandomState(seed)

    x = np.arange(cols, dtype=float)
    y = np.arange(rows, dtype=float)
    xx, yy = np.meshgrid(x, y)

    # 基础坡面（从西北到东南倾斜）
    base_slope = 0.02 * xx + 0.01 * yy

    # 主山脊（沿对角线方向的隆起）
    ridge_dist = np.abs((xx - cols * 0.3) * 0.6 - (yy - rows * 0.3) * 0.4)
    ridge = 80 * np.exp(-ridge_dist ** 2 / (2 * (cols * 0.15) ** 2))

    # 次山脊
    ridge2_dist = np.abs((xx - cols * 0.7) * 0.3 + (yy - rows * 0.6) * 0.7)
    ridge2 = 50 * np.exp(-ridge2_dist ** 2 / (2 * (cols * 0.1) ** 2))

    # 山谷（负高程修正）
    valley_dist = np.abs((xx - cols * 0.5) * 0.5 + (yy - rows * 0.5) * 0.5)
    valley = -30 * np.exp(-valley_dist ** 2 / (2 * (cols * 0.12) ** 2))

    # 随机地形起伏（小尺度）
    noise = rng.randn(rows, cols) * 5

    # 合成DEM
    dem = 1000 + base_slope * cell_size + ridge + ridge2 + valley + noise

    return dem


def compute_min_cost(cost_field):
    """
    计算代价场中所有方向的最小代价。

    Parameters
    ----------
    cost_field : np.ndarray
        代价场 (rows, cols, N_directions)

    Returns
    -------
    min_cost : np.ndarray
        每个栅格点的最小代价 (rows, cols)
    """
    return np.min(cost_field, axis=2)


def compute_optimal_direction(cost_field, directions):
    """
    计算每个栅格点的最优方向（代价最小的方向）。

    Parameters
    ----------
    cost_field : np.ndarray
        代价场 (rows, cols, N_directions)
    directions : np.ndarray
        方向角度数组（弧度）

    Returns
    -------
    optimal_theta : np.ndarray
        最优方向角度 (rows, cols)
    """
    idx = np.argmin(cost_field, axis=2)
    return directions[idx]
