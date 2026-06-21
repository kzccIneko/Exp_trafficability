"""
run_experiment.py
方向敏感越野通行能力实验 v4.2：支持合成 DEM / 真实 GeoTIFF DEM，输出中文结果图、中文指标表和公式说明。

核心实验：
B0   标量坡度代价：不考虑车辆行驶方向。
B1   仅纵坡方向代价：考虑沿移动方向的上坡/下坡，不考虑横坡。
Ours 纵坡+横坡方向代价：同时考虑行驶方向的纵坡阻力和横向稳定约束压力。

Windows 示例：
python run_experiment.py --dem "D:\\VSCode Program\\通行能力分析_研\\yajiang_gesigou_srtmgl1_30m_1000px_bbox.tif" --max-pixels 1000 --out outputs_real_1000

快速测试：
python run_experiment.py --synthetic --out outputs_synthetic
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time
import numpy as np

from cost_model import (
    calculate_gradient_tan,
    calculate_slope_angle,
    longitudinal_cross_slope_tan,
    compute_b0_unit_cost_field,
    compute_b1_unit_cost_field,
    compute_ours_unit_cost_field,
)
from path_planning import astar_directional
from metrics import anisotropy_index, summarize_array


MODEL_CN = {
    "B0": "B0 标量坡度模型",
    "B1": "B1 仅纵坡方向模型",
    "Ours": "Ours 纵坡+横坡方向模型",
}


def parse_pair(text: str | None, default: tuple[int, int]) -> tuple[int, int]:
    if not text:
        return default
    parts = text.replace(";", ",").split(",")
    if len(parts) != 2:
        raise ValueError("坐标格式应为 row,col，例如 100,120")
    return int(parts[0]), int(parts[1])


def generate_synthetic_dem(rows=240, cols=240, cell_size=30.0, seed=42):
    rng = np.random.RandomState(seed)
    x = np.arange(cols, dtype=float)
    y = np.arange(rows, dtype=float)
    xx, yy = np.meshgrid(x, y)
    base = 0.016 * xx * cell_size + 0.006 * yy * cell_size
    ridge1 = 90 * np.exp(-((0.55 * (xx - cols * 0.25) - 0.35 * (yy - rows * 0.20)) ** 2) / (2 * (cols * 0.075) ** 2))
    ridge2 = 55 * np.exp(-((0.30 * (xx - cols * 0.72) + 0.70 * (yy - rows * 0.60)) ** 2) / (2 * (cols * 0.055) ** 2))
    valley = -55 * np.exp(-((0.45 * (xx - cols * 0.55) + 0.50 * (yy - rows * 0.52)) ** 2) / (2 * (cols * 0.065) ** 2))
    hill = 60 * np.exp(-(((xx - cols * 0.62) ** 2 + (yy - rows * 0.72) ** 2) / (2 * (cols * 0.10) ** 2)))
    noise = rng.normal(0, 2.0, size=(rows, cols))
    return 1000 + base + ridge1 + ridge2 + valley + hill + noise, cell_size, {"source": "synthetic"}


def fill_nan_nearest(arr: np.ndarray) -> np.ndarray:
    if not np.any(~np.isfinite(arr)):
        return arr
    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError as e:
        raise ImportError("DEM 存在 NoData，需要 scipy 填充：pip install scipy") from e
    mask = ~np.isfinite(arr)
    _, indices = distance_transform_edt(mask, return_indices=True)
    return arr[tuple(indices)]


def load_dem_geotiff(path: str, max_pixels: int | None = None, fallback_cell_size: float = 30.0):
    """
    读取真实 DEM。优先使用 rasterio 读取 GeoTIFF 元数据；
    如果当前 Python 环境没有 rasterio，则退回到 tifffile 读取像元数组，
    并使用 --cell-size 指定的栅格大小。

    为什么需要 fallback？
    Windows + Python 3.14 环境下 rasterio 有时没有现成 wheel，安装容易失败；
    但本实验第一阶段主要需要 DEM 高程矩阵和栅格大小，tifffile 已足够跑通纵坡/横坡实验。
    """
    tif = Path(path)
    if not tif.exists():
        raise FileNotFoundError(f"没有找到 DEM 文件：{tif}\n请检查路径是否完整、是否加了英文双引号。")

    try:
        import rasterio
        with rasterio.open(tif) as src:
            arr = src.read(1).astype(float)
            nodata = src.nodata
            if nodata is not None:
                arr[arr == nodata] = np.nan
            arr = fill_nan_nearest(arr)

            transform = src.transform
            if src.crs and src.crs.is_projected:
                res_x_m = abs(transform.a)
                res_y_m = abs(transform.e)
            else:
                lat_center = (src.bounds.bottom + src.bounds.top) / 2
                m_per_deg_lat = 111132.92
                m_per_deg_lon = 111132.92 * np.cos(np.radians(lat_center))
                res_x_m = abs(src.res[0]) * m_per_deg_lon
                res_y_m = abs(src.res[1]) * m_per_deg_lat
            cell_size = float((res_x_m + res_y_m) / 2)

            meta = {
                "数据源": str(tif),
                "读取方式": "rasterio，已读取 GeoTIFF 空间元数据",
                "坐标系": str(src.crs),
                "空间范围": tuple(src.bounds),
                "原始行列数": tuple(arr.shape),
                "x方向分辨率_m": res_x_m,
                "y方向分辨率_m": res_y_m,
                "平均栅格大小_m": cell_size,
            }

    except ImportError:
        try:
            import tifffile
        except ImportError as e:
            raise ImportError(
                "读取 GeoTIFF 需要 rasterio 或 tifffile。\n"
                "最简单安装命令：python -m pip install numpy scipy matplotlib tifffile\n"
                "若要读取精确坐标元数据，再安装：python -m pip install rasterio"
            ) from e

        arr = tifffile.imread(str(tif)).astype(float)
        if arr.ndim > 2:
            arr = arr[0] if arr.shape[0] < arr.shape[-1] else arr[..., 0]
        arr[arr < -10000] = np.nan
        arr = fill_nan_nearest(arr)
        cell_size = float(fallback_cell_size)
        meta = {
            "数据源": str(tif),
            "读取方式": "tifffile fallback，未读取 GeoTIFF 坐标元数据",
            "坐标系": "未读取；如需空间坐标请安装 rasterio",
            "空间范围": "未读取；如需空间坐标请安装 rasterio",
            "原始行列数": tuple(arr.shape),
            "x方向分辨率_m": cell_size,
            "y方向分辨率_m": cell_size,
            "平均栅格大小_m": cell_size,
            "说明": "当前使用 --cell-size 作为栅格大小；SRTM 30m DEM 可先用 30。",
        }

    if max_pixels and max(arr.shape) > max_pixels:
        step = int(np.ceil(max(arr.shape) / max_pixels))
        arr = arr[::step, ::step]
        cell_size *= step
        meta["降采样步长"] = step
        meta["实际使用行列数"] = tuple(arr.shape)
        meta["实际栅格大小_m"] = cell_size
    else:
        meta["降采样步长"] = 1
        meta["实际使用行列数"] = tuple(arr.shape)
        meta["实际栅格大小_m"] = cell_size

    return arr, cell_size, meta


def safe_q_limits(*arrays, q_low=2, q_high=98):
    vals = np.concatenate([a[np.isfinite(a)].ravel() for a in arrays])
    if vals.size == 0:
        return 0.0, 1.0
    vmin, vmax = float(np.percentile(vals, q_low)), float(np.percentile(vals, q_high))
    if np.isclose(vmin, vmax):
        vmax = vmin + 1e-6
    return vmin, vmax


def setup_chinese_font():
    import matplotlib
    import matplotlib.font_manager as fm
    for name in ["SimHei", "Microsoft YaHei", "SimSun", "Noto Sans CJK SC", "WenQuanYi Micro Hei"]:
        try:
            path = fm.findfont(fm.FontProperties(family=name), fallback_to_default=False)
            if path:
                matplotlib.rcParams["font.sans-serif"] = [name] + matplotlib.rcParams.get("font.sans-serif", [])
                matplotlib.rcParams["axes.unicode_minus"] = False
                return name
        except Exception:
            pass
    matplotlib.rcParams["axes.unicode_minus"] = False
    return "未检测到中文字体，可能显示为方框"


def add_common_axis_labels(ax):
    ax.set_xlabel("列号")
    ax.set_ylabel("行号")


def save_figures(out: Path, dem, p, q, directions, C0, C1, Co, paths, start, goal, cell_size):
    import matplotlib
    matplotlib.use("Agg")
    font_name = setup_chinese_font()
    import matplotlib.pyplot as plt

    slope = calculate_slope_angle(p, q)
    min0 = np.min(C0, axis=2)
    min1 = np.min(C1, axis=2)
    mino = np.min(Co, axis=2)
    AI1 = anisotropy_index(C1, method="percentile")
    AIo = anisotropy_index(Co, method="percentile")

    # 1 DEM + slope decomposition
    theta_vis = np.pi / 4
    gp, gc = longitudinal_cross_slope_tan(p, q, theta_vis)
    fig, axes = plt.subplots(1, 4, figsize=(21, 5))
    im = axes[0].imshow(dem, cmap="terrain", origin="lower")
    axes[0].set_title("数字高程模型（DEM）")
    plt.colorbar(im, ax=axes[0], shrink=0.8, label="高程 / m")
    im = axes[1].imshow(np.degrees(slope), cmap="YlOrRd", origin="lower")
    axes[1].set_title("坡度角 β")
    plt.colorbar(im, ax=axes[1], shrink=0.8, label="坡度角 / °")
    vmax_gp = np.nanpercentile(np.abs(np.degrees(np.arctan(gp))), 98)
    im = axes[2].imshow(np.degrees(np.arctan(gp)), cmap="RdBu_r", origin="lower", vmin=-vmax_gp, vmax=vmax_gp)
    axes[2].set_title("纵坡角 α∥（θ=45°）")
    plt.colorbar(im, ax=axes[2], shrink=0.8, label="纵坡角 / °")
    im = axes[3].imshow(np.degrees(np.arctan(np.abs(gc))), cmap="YlOrRd", origin="lower")
    axes[3].set_title("横坡角 |α⊥|（θ=45°）")
    plt.colorbar(im, ax=axes[3], shrink=0.8, label="横坡角 / °")
    for ax in axes: add_common_axis_labels(ax)
    fig.suptitle("地形输入与纵坡/横坡方向分解")
    fig.tight_layout()
    fig.savefig(out / "01_DEM与纵坡横坡分解.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # 2 cost fields
    vmin, vmax = safe_q_limits(min0, min1, mino, q_low=2, q_high=98)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, data, title in zip(axes, [min0, min1, mino], ["B0 标量坡度模型", "B1 仅纵坡方向模型", "Ours 纵坡+横坡方向模型"]):
        im = ax.imshow(data, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax)
        ax.set_title(title)
        add_common_axis_labels(ax)
        plt.colorbar(im, ax=ax, shrink=0.8, label="最小单位距离代价")
    fig.suptitle("三模型最小单位距离代价场对比（统一色标）")
    fig.tight_layout()
    fig.savefig(out / "02_三模型最小代价场对比.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # 3 anisotropy
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    im = axes[0].imshow(AI1, cmap="hot", origin="lower", vmin=0, vmax=1)
    axes[0].set_title("B1 分位数各向异性指数 AIp")
    add_common_axis_labels(axes[0])
    plt.colorbar(im, ax=axes[0], shrink=0.8, label="AIp，0=方向差异弱，1=方向差异强")
    im = axes[1].imshow(AIo, cmap="hot", origin="lower", vmin=0, vmax=1)
    axes[1].set_title("Ours 分位数各向异性指数 AIp")
    add_common_axis_labels(axes[1])
    plt.colorbar(im, ax=axes[1], shrink=0.8, label="AIp，0=方向差异弱，1=方向差异强")
    fig.suptitle("方向差异强度：AIp=(Q90-Q10)/(Q90+Q10+ε)")
    fig.tight_layout()
    fig.savefig(out / "03_各向异性指数对比.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # 4 optimal direction
    opt = directions[np.argmin(Co, axis=2)]
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    im = axes[0].imshow(mino, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax)
    axes[0].set_title("Ours 最小单位距离代价")
    add_common_axis_labels(axes[0])
    plt.colorbar(im, ax=axes[0], shrink=0.8, label="最小单位距离代价")
    im = axes[1].imshow(np.degrees(opt), cmap="hsv", origin="lower", vmin=0, vmax=360)
    axes[1].set_title("Ours 最优通行方向 θ*")
    add_common_axis_labels(axes[1])
    plt.colorbar(im, ax=axes[1], shrink=0.8, label="方向角 / °")
    fig.suptitle("最优代价与最优方向场")
    fig.tight_layout()
    fig.savefig(out / "04_最优代价与最优方向场.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # 5 paths
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    path_titles = {"B0": "B0 标量坡度路径", "B1": "B1 仅纵坡路径", "Ours": "Ours 纵坡+横坡路径"}
    for ax, name in zip(axes, ["B0", "B1", "Ours"]):
        result = paths[name]
        im = ax.imshow(mino, cmap="RdYlGn_r", origin="lower", vmin=vmin, vmax=vmax, alpha=0.82)
        if result.path:
            arr = np.asarray(result.path)
            ax.plot(arr[:, 1], arr[:, 0], "b-", linewidth=1.8, label="规划路径")
        ax.plot(start[1], start[0], "go", markersize=8, label="起点")
        ax.plot(goal[1], goal[0], "r*", markersize=10, label="终点")
        ax.set_title(path_titles[name])
        add_common_axis_labels(ax)
        ax.legend(loc="upper left", fontsize=8)
    fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.8, label="底图：Ours 最小单位距离代价")
    fig.suptitle("三模型路径规划结果对比")
    fig.savefig(out / "05_三模型路径规划对比.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    # 6 rose plots
    rows, cols = dem.shape
    sample_positions = [
        (int(rows*0.25), int(cols*0.25)),
        (int(rows*0.50), int(cols*0.50)),
        (int(rows*0.75), int(cols*0.25)),
        (int(rows*0.75), int(cols*0.75)),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 11), subplot_kw={"projection": "polar"})
    theta_closed = np.append(directions, directions[0])
    for ax, pos in zip(axes.flat, sample_positions):
        r, c = pos
        cst = Co[r, c, :]
        cst_closed = np.append(cst, cst[0])
        ax.plot(theta_closed, cst_closed, "o-", linewidth=1.5, markersize=3)
        ax.fill(theta_closed, cst_closed, alpha=0.25)
        ax.set_title(f"采样位置（行{r}，列{c}）", pad=12)
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
    fig.suptitle("Ours 局部代价玫瑰图：同一位置不同方向代价")
    fig.tight_layout()
    fig.savefig(out / "06_局部代价玫瑰图.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    with open(out / "字体检测.txt", "w", encoding="utf-8") as f:
        f.write(f"Matplotlib 使用字体：{font_name}\n")


def field_metric_rows(model_name: str, C: np.ndarray):
    min_cost = np.min(C, axis=2)
    ai_p = anisotropy_index(C, method="percentile")
    ai_mm = anisotropy_index(C, method="maxmin")
    rows = []
    for metric_name, arr, formula, meaning in [
        (
            "最小单位距离代价",
            min_cost,
            "c_min(x,y)=min_θ c_unit(x,y,θ)",
            "表示该栅格在所有允许通行方向中的最低单位距离代价。值越小，说明至少存在一个相对容易通过的方向。",
        ),
        (
            "分位数各向异性指数 AIp",
            ai_p,
            "AIp=(Q90(c_unit)-Q10(c_unit))/(Q90(c_unit)+Q10(c_unit)+ε)",
            "表示同一栅格不同方向代价差异强度。使用分位数而非最大最小值，可减弱极端方向代价影响。",
        ),
        (
            "最大最小各向异性指数 AImaxmin",
            ai_mm,
            "AI=(max_θ c_unit-min_θ c_unit)/(max_θ c_unit+min_θ c_unit+ε)",
            "用于参考。若某些方向单位代价接近0，该指标容易接近1，论文主指标建议用 AIp。",
        ),
    ]:
        s = summarize_array("", arr)
        rows.append({
            "模型": model_name,
            "指标": metric_name,
            "公式": formula,
            "最小值": s["min"],
            "10分位数": s["q10"],
            "均值": s["mean"],
            "中位数": s["median"],
            "90分位数": s["q90"],
            "最大值": s["max"],
            "解释": meaning,
        })
    return rows


def write_metrics(out: Path, dem, p, q, C0, C1, Co, paths, start, goal, cell_size):
    # 1. 场统计指标表
    rows = []
    for code, C in [("B0", C0), ("B1", C1), ("Ours", Co)]:
        rows.extend(field_metric_rows(MODEL_CN[code], C))
    with open(out / "01_场统计指标.csv", "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=["模型", "指标", "公式", "最小值", "10分位数", "均值", "中位数", "90分位数", "最大值", "解释"])
        writer.writeheader()
        writer.writerows(rows)

    # 2. 路径指标表
    straight_m = np.hypot(goal[0] - start[0], goal[1] - start[1]) * cell_size
    path_rows = []
    for code in ["B0", "B1", "Ours"]:
        result = paths[code]
        path_rows.append({
            "模型": MODEL_CN[code],
            "路径相对累计代价 J_plan": result.total_cost,
            "路径长度 L_m": result.path_length_m,
            "起终点直线距离 D_m": straight_m,
            "平均单位距离相对代价 J_plan除以L": result.average_cost_per_m,
            "绕行率 L除以D": result.detour_ratio,
            "路径节点数": len(result.path),
            "公式说明": "J=Σ c_unit(i,θ_i)·l_i；L=Σl_i；平均单位距离相对代价=J_plan/L；绕行率=L/D。",
        })
    with open(out / "02_路径规划指标.csv", "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=["模型", "路径相对累计代价 J_plan", "路径长度 L_m", "起终点直线距离 D_m", "平均单位距离相对代价 J_plan除以L", "绕行率 L除以D", "路径节点数", "公式说明"])
        writer.writeheader()
        writer.writerows(path_rows)

    # 3. 兼容旧文件名，但内容更清楚
    with open(out / "metrics_summary.csv", "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["说明", "本文件仅为兼容旧版；建议查看 01_场统计指标.csv、02_路径规划指标.csv、03_指标公式与阅读说明.md"])
        writer.writerow([])
        writer.writerow(["模型", "路径相对累计代价J_plan", "路径长度m", "平均单位距离相对代价", "绕行率", "路径节点数"])
        for r in path_rows:
            writer.writerow([r["模型"], r["路径相对累计代价 J_plan"], r["路径长度 L_m"], r["平均单位距离相对代价 J_plan除以L"], r["绕行率 L除以D"], r["路径节点数"]])


def write_metric_explanation(out: Path):
    text = r"""# 指标公式与阅读说明

## 1. 三个模型分别解决什么问题

**B0 标量坡度模型**：传统基线模型。每个栅格只有一个坡度代价，不区分车辆从哪个方向穿越该栅格。

**B1 仅纵坡方向模型**：把车辆移动方向 θ 引入模型，只考虑车辆前进方向上的上坡/下坡。

**Ours 纵坡+横坡方向模型**：同时考虑车辆前进方向的纵坡阻力和车辆横向的侧倾/侧滑压力。它用于检验“沿等高线纵坡小，但横坡可能很大，因此不能简单认为低代价”的假设。

## 2. 纵坡/横坡如何体现车辆行驶方向

DEM 梯度为：

```text
∇z=(p,q)=(∂z/∂x, ∂z/∂y)
```

车辆当前移动方向为：

```text
uθ=(cosθ, sinθ)
```

车辆横向方向为：

```text
nθ=(-sinθ, cosθ)
```

纵坡正切值：

```text
g∥=∇z·uθ=p cosθ + q sinθ
```

横坡正切值：

```text
g⊥=∇z·nθ=-p sinθ + q cosθ
```

代码中先计算正切值，再转换为坡度角：

```text
α∥=arctan(g∥)
α⊥=arctan(|g⊥|)
```

因此，纵坡和横坡已经考虑了车辆行驶方向 θ。当前 θ 来自 A* 的 8 邻域移动方向。

## 3. 阻碍度函数

### 3.1 B0 标量坡度阻碍度

```text
β=arctan(sqrt(p²+q²))
R0=1-exp[-(β/αu)²]
```

含义：坡度越大，阻碍越大；但不区分行驶方向。

### 3.2 纵坡阻碍度

上坡：

```text
R∥=1-exp[-(α∥/αu)²], α∥>0
```

缓下坡：

```text
R∥=0, -αm ≤ α∥ ≤ 0
```

陡下坡：

```text
R∥=1-exp[-((|α∥|-αm)/αd)²], α∥<-αm
```

含义：上坡增加牵引需求；轻微下坡不增加阻碍；过陡下坡增加制动和失稳压力。

### 3.3 横坡阻碍度

```text
R⊥=1-exp[-(|α⊥|/αr)²]
```

含义：横坡越大，车辆横向稳定约束压力越高。左倾和右倾约束压力近似对称，所以取绝对值。

### 3.4 Ours 非补偿合成

```text
R=1-(1-R∥)(1-R⊥)
```

含义：纵坡或横坡任一项很高，综合阻碍都会明显升高，不允许“一个好因子完全抵消一个危险因子”。

## 4. 单位距离代价与路径总代价

通过能力：

```text
P=1-R
```

单位距离代价：

```text
c_unit=-ln(P+ε)
```

路径边代价：

```text
C_edge=c_unit(x,y,θ) · lθ
```

路径总代价：

```text
J=Σ C_edge
```

其中 lθ 是实际移动边长，正交方向为 cell_size，斜向方向为 sqrt(2)·cell_size。

## 5. 各向异性指数 AIp

同一栅格不同方向的单位距离代价为 c_unit(x,y,θ)。

```text
AIp=(Q90(c_unit)-Q10(c_unit))/(Q90(c_unit)+Q10(c_unit)+ε)
```

AIp 接近 0：各方向代价接近，方向差异弱。

AIp 接近 1：不同方向代价差异强，说明该区域明显需要方向敏感建模。

论文建议使用 AIp，而不是 max-min 版 AI，因为 max-min 版容易受极端方向影响。

## 6. 路径指标

路径长度：

```text
L=Σl_i
```

起终点直线距离：

```text
D=sqrt((row_g-row_s)²+(col_g-col_s)²)·cell_size
```

平均单位距离相对代价：

```text
J/L
```

绕行率：

```text
L/D
```

解释：不能只看路径相对累计代价 J_plan。需要同时看路径长度、平均单位距离相对代价、绕行率和路径形态。如果 Ours 总代价高但平均单位距离相对代价低，说明它可能通过绕行避开高约束压力区域；如果 Ours 明显贴边，则要检查边界缓冲或参数 αr 是否过强。

## 7. 邻域搜索方式的影响

当前实验使用 8 邻域，所以车辆每步只能选择 8 个方向。纵坡/横坡分解中的 θ 就是这 8 个移动方向。邻域数会影响路径形态：

- 4 邻域：方向太粗，方向敏感优势会被削弱；
- 8 邻域：基础实验最稳，便于解释；
- 16/32 方向：更接近连续方向，但搜索空间、边长定义和方向匹配都要重新设计。

因此，第一阶段建议固定 8 邻域做 B0/B1/Ours 公平对比；第二阶段再做 4/8 邻域敏感性实验。
"""
    with open(out / "03_指标公式与阅读说明.md", "w", encoding="utf-8") as f:
        f.write(text)


def write_meta(out: Path, meta: dict, args):
    with open(out / "04_运行参数与DEM信息.txt", "w", encoding="utf-8") as f:
        f.write("运行参数\n")
        f.write("========\n")
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v}\n")
        f.write("\nDEM 元数据\n")
        f.write("========\n")
        for k, v in meta.items():
            f.write(f"{k}: {v}\n")


def main():
    parser = argparse.ArgumentParser(
        description="方向敏感越野通行能力实验 v4.2",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--dem", type=str, default=None, help="真实 DEM GeoTIFF 路径。不传则使用合成 DEM。")
    parser.add_argument("--synthetic", action="store_true", help="强制使用合成 DEM。")
    parser.add_argument("--out", type=str, default="outputs_v4_2", help="输出目录。")
    parser.add_argument("--max-pixels", type=int, default=1000, help="最大边长像元数。1000×1000 DEM 默认不降采样；设 0 表示完全不降采样。")
    parser.add_argument("--cell-size", type=float, default=30.0, help="当未安装 rasterio、改用 tifffile 读取时采用的栅格大小，单位 m。SRTM 30m 可设为 30。")
    parser.add_argument("--start", type=str, default=None, help="起点 row,col。不填则使用行列数的 0.10 位置。")
    parser.add_argument("--goal", type=str, default=None, help="终点 row,col。不填则使用行列数的 0.90 位置。")
    parser.add_argument("--edge-buffer", type=int, default=5, help="边界缓冲像元数，防止路径贴边。")
    parser.add_argument("--directions", type=int, default=8, choices=[8], help="路径搜索方向数。目前固定 8 邻域，便于公平对比。")
    parser.add_argument("--alpha-u", type=float, default=15.0, help="上坡阻碍尺度，单位度。")
    parser.add_argument("--alpha-m", type=float, default=5.0, help="缓下坡容许尺度，单位度。")
    parser.add_argument("--alpha-d", type=float, default=15.0, help="陡下坡制动压力尺度，单位度。")
    parser.add_argument("--alpha-r", type=float, default=15.0, help="横坡约束压力尺度，单位度。建议试 10、15、20、25。")
    parser.add_argument("--no-path", action="store_true", help="只输出代价场和图片，不做路径规划。适合超大 DEM 调试。")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    max_pixels = None if args.max_pixels == 0 else args.max_pixels
    if args.synthetic or not args.dem:
        dem, cell_size, meta = generate_synthetic_dem()
    else:
        dem, cell_size, meta = load_dem_geotiff(args.dem, max_pixels=max_pixels, fallback_cell_size=args.cell_size)

    rows, cols = dem.shape
    default_start = (max(args.edge_buffer + 1, int(rows * 0.10)), max(args.edge_buffer + 1, int(cols * 0.10)))
    default_goal = (min(rows - args.edge_buffer - 2, int(rows * 0.90)), min(cols - args.edge_buffer - 2, int(cols * 0.90)))
    start = parse_pair(args.start, default_start)
    goal = parse_pair(args.goal, default_goal)

    print(f"DEM 行列数: {dem.shape}, 栅格大小: {cell_size:.3f} m")
    print(f"起点: {start}, 终点: {goal}, 边界缓冲: {args.edge_buffer} 像元")

    p, q = calculate_gradient_tan(dem, cell_size)
    directions = np.linspace(0, 2*np.pi, args.directions, endpoint=False)

    print("计算 B0/B1/Ours 代价场...")
    _, C0 = compute_b0_unit_cost_field(p, q, directions, alpha_u=args.alpha_u)
    _, C1 = compute_b1_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d)
    _, Co, _ = compute_ours_unit_cost_field(p, q, directions, alpha_u=args.alpha_u, alpha_m=args.alpha_m, alpha_d=args.alpha_d, alpha_r=args.alpha_r)

    paths = {}
    if not args.no_path:
        for name, C in [("B0", C0), ("B1", C1), ("Ours", Co)]:
            print(f"A* 路径规划：{MODEL_CN[name]} ...")
            t = time.time()
            res = astar_directional(C, start, goal, cell_size, directions, edge_buffer=args.edge_buffer)
            paths[name] = res
            print(f"  {name}: 节点数={len(res.path)}, 相对累计代价={res.total_cost:.3f}, 长度={res.path_length_m:.1f} m, 平均单位距离相对代价={res.average_cost_per_m:.6f}, 绕行率={res.detour_ratio:.3f}, 耗时={time.time()-t:.2f} s")
    else:
        from path_planning import PathResult
        paths = {name: PathResult([], np.nan, np.nan, np.nan, np.nan) for name in ["B0", "B1", "Ours"]}

    print("保存指标、公式说明和图片...")
    write_metrics(out, dem, p, q, C0, C1, Co, paths, start, goal, cell_size)
    write_metric_explanation(out)
    write_meta(out, meta, args)
    save_figures(out, dem, p, q, directions, C0, C1, Co, paths, start, goal, cell_size)

    print(f"完成。输出目录：{out.resolve()}")
    print("建议优先查看：01_场统计指标.csv、02_路径规划指标.csv、03_指标公式与阅读说明.md、05_三模型路径规划对比.png")
    print(f"总耗时：{time.time()-t0:.2f} s")


if __name__ == "__main__":
    main()
