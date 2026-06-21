"""
raster_align.py

轻量级栅格对齐工具。若只有数组而没有完整地理参考信息，按目标 shape 重采样。
连续数据用 bilinear；分类数据用 nearest，避免产生不存在的地类编号。
"""
from __future__ import annotations
import numpy as np
from scipy.ndimage import zoom


def resize_to_shape(arr: np.ndarray, target_shape: tuple[int,int], method: str = 'bilinear') -> np.ndarray:
    if arr.shape == target_shape:
        return arr.copy()
    factors = (target_shape[0] / arr.shape[0], target_shape[1] / arr.shape[1])
    order = 0 if method in ('nearest','majority','categorical') else 1
    out = zoom(arr, factors, order=order)
    # zoom 可能因四舍五入差 1 个像元，做裁剪/填补
    res = np.empty(target_shape, dtype=out.dtype)
    r = min(target_shape[0], out.shape[0]); c = min(target_shape[1], out.shape[1])
    res[:r,:c] = out[:r,:c]
    if r < target_shape[0]: res[r:,:c] = out[r-1:r,:c]
    if c < target_shape[1]: res[:,:c] = res[:,c-1:c]
    if r < target_shape[0] and c < target_shape[1]: res[r:,c:] = res[r-1,c-1]
    return res


def load_raster_array(path: str, target_shape: tuple[int,int] | None = None, categorical: bool = False, fallback_nodata: float = -9999):
    """读取 GeoTIFF 或普通 TIFF。优先 rasterio，失败则 tifffile。"""
    import os
    if not path or not os.path.exists(path):
        raise FileNotFoundError(path)
    try:
        import rasterio
        with rasterio.open(path) as src:
            arr = src.read(1)
            if src.nodata is not None:
                arr = arr.astype(float)
                arr[arr == src.nodata] = np.nan
            meta = {'crs': str(src.crs), 'transform': tuple(src.transform), 'shape': arr.shape, 'path': path}
    except ImportError:
        import tifffile
        arr = tifffile.imread(path)
        if arr.ndim > 2:
            arr = arr[0] if arr.shape[0] < arr.shape[-1] else arr[...,0]
        arr = arr.astype(float)
        arr[arr <= fallback_nodata] = np.nan
        meta = {'crs': 'unknown', 'transform': None, 'shape': arr.shape, 'path': path, 'reader': 'tifffile'}
    if target_shape is not None:
        arr = resize_to_shape(arr, target_shape, method='nearest' if categorical else 'bilinear')
    return arr, meta
