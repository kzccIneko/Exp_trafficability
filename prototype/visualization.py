"""
可视化模块

提供代价玫瑰图、代价场热力图、路径可视化、验证结果展示等功能。
"""

import numpy as np

try:
    import matplotlib
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    from matplotlib.patches import FancyArrowPatch
    import matplotlib.font_manager as fm
    HAS_MPL = True

    # ===== 中文字体配置 =====
    # 优先使用SimHei，其次Microsoft YaHei
    _CHINESE_FONT = None
    for _fname in ['SimHei', 'Microsoft YaHei', 'SimSun', 'FangSong', 'KaiTi']:
        try:
            _fpath = fm.findfont(fm.FontProperties(family=_fname), fallback_to_default=False)
            if _fpath and 'DejaVu' not in _fpath and 'fallback' not in _fpath.lower():
                _CHINESE_FONT = _fname
                break
        except Exception:
            continue

    if _CHINESE_FONT:
        matplotlib.rcParams['font.sans-serif'] = [_CHINESE_FONT] + matplotlib.rcParams.get('font.sans-serif', [])
        matplotlib.rcParams['font.family'] = 'sans-serif'
        matplotlib.rcParams['axes.unicode_minus'] = False
    else:
        # 直接写入SimHei作为首选
        matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
        matplotlib.rcParams['font.family'] = 'sans-serif'
        matplotlib.rcParams['axes.unicode_minus'] = False

except ImportError:
    HAS_MPL = False


def _check_matplotlib():
    if not HAS_MPL:
        raise ImportError("需要安装 matplotlib: pip install matplotlib")


def plot_cost_rose(cost_field, pos, n_directions=16, ax=None, title=None):
    """
    绘制代价玫瑰图。

    在某点绘制各方向的代价极坐标图。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    pos : tuple
        位置坐标 (row, col)
    n_directions : int
        显示的方向数
    ax : matplotlib.axes.Axes or None
        绘图坐标轴（需为极坐标投影）
    title : str or None
        图标题

    Returns
    -------
    ax : matplotlib.axes.Axes
        绘图坐标轴
    """
    _check_matplotlib()

    r, c = int(pos[0]), int(pos[1])
    N = cost_field.shape[2]

    # 获取各方向代价
    costs = cost_field[r, c, :]
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False)

    # 如果需要重采样到n_directions
    if n_directions != N:
        new_angles = np.linspace(0, 2 * np.pi, n_directions, endpoint=False)
        costs = np.interp(new_angles, angles, costs)
        angles = new_angles

    # 闭合曲线
    angles_closed = np.append(angles, angles[0])
    costs_closed = np.append(costs, costs[0])

    if ax is None:
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))

    ax.plot(angles_closed, costs_closed, 'b-', linewidth=2)
    ax.fill(angles_closed, costs_closed, alpha=0.25)

    # 标记最小代价方向
    min_idx = np.argmin(costs)
    ax.annotate(f'{costs[min_idx]:.2f}',
                xy=(angles[min_idx], costs[min_idx]),
                fontsize=10, color='green', fontweight='bold')

    # 标记最大代价方向
    max_idx = np.argmax(costs)
    ax.annotate(f'{costs[max_idx]:.2f}',
                xy=(angles[max_idx], costs[max_idx]),
                fontsize=10, color='red', fontweight='bold')

    if title:
        ax.set_title(title, pad=20)

    return ax


def plot_cost_field(cost_field, direction_idx=0, ax=None, title=None,
                    cmap='RdYlGn_r', figsize=(10, 8)):
    """
    绘制某方向的代价场热力图。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    direction_idx : int
        方向索引
    ax : matplotlib.axes.Axes or None
        绘图坐标轴
    title : str or None
        图标题
    cmap : str
        颜色映射
    figsize : tuple
        图大小

    Returns
    -------
    ax : matplotlib.axes.Axes
    """
    _check_matplotlib()

    data = cost_field[:, :, direction_idx]

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    N = cost_field.shape[2]
    direction_angle = direction_idx * 360 / N

    im = ax.imshow(data, cmap=cmap, origin='lower', aspect='equal')
    plt.colorbar(im, ax=ax, label='代价')

    if title:
        ax.set_title(title)
    else:
        ax.set_title(f'代价场 (方向: {direction_angle:.0f}°)')

    ax.set_xlabel('列')
    ax.set_ylabel('行')

    return ax


def plot_path_on_cost(path, cost_field, direction_idx=0, ax=None,
                      title=None, cmap='RdYlGn_r', figsize=(10, 8)):
    """
    在代价场上绘制路径。

    Parameters
    ----------
    path : list of tuple
        路径坐标列表 [(row, col), ...]
    cost_field : np.ndarray
        方向敏感代价场
    direction_idx : int
        方向索引
    ax : matplotlib.axes.Axes or None
    title : str or None
    cmap : str
    figsize : tuple

    Returns
    -------
    ax : matplotlib.axes.Axes
    """
    _check_matplotlib()

    ax = plot_cost_field(cost_field, direction_idx, ax=ax, cmap=cmap,
                          figsize=figsize)

    if len(path) > 0:
        path = np.array(path)
        ax.plot(path[:, 1], path[:, 0], 'b-', linewidth=2.5, label='路径')
        ax.plot(path[0, 1], path[0, 0], 'go', markersize=10, label='起点')
        ax.plot(path[-1, 1], path[-1, 0], 'r*', markersize=12, label='终点')
        ax.legend(loc='upper right')

    if title:
        ax.set_title(title)

    return ax


def plot_validation_results(metrics, ax=None, figsize=(10, 6)):
    """
    绘制验证结果柱状图。

    Parameters
    ----------
    metrics : dict
        验证指标字典
    ax : matplotlib.axes.Axes or None
    figsize : tuple

    Returns
    -------
    ax : matplotlib.axes.Axes
    """
    _check_matplotlib()

    # 选择要显示的指标
    display_metrics = {}
    if 'cost_ratio' in metrics:
        display_metrics['代价比'] = metrics['cost_ratio']
    if 'rank_percentile' in metrics:
        display_metrics['排名百分位 (%)'] = metrics['rank_percentile']
    if 'direction_consistency' in metrics:
        display_metrics['方向一致性'] = metrics['direction_consistency']

    if not display_metrics:
        return None

    names = list(display_metrics.keys())
    values = list(display_metrics.values())

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    colors = ['#2ecc71', '#3498db', '#e74c3c', '#f39c12']
    bars = ax.bar(names, values, color=colors[:len(names)], alpha=0.8, edgecolor='black')

    # 添加数值标签
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
                f'{val:.3f}', ha='center', va='bottom', fontweight='bold')

    # 添加参考线
    ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.5, label='参考线 (CR=1)')
    ax.axhline(y=0.7, color='green', linestyle='--', alpha=0.5, label='良好线 (CR=0.7)')

    ax.set_ylabel('值')
    ax.set_title('验证指标')
    ax.legend()
    ax.grid(axis='y', alpha=0.3)

    return ax


def plot_feature_lines(dem, ridge_points, valley_points, ax=None,
                       figsize=(10, 8)):
    """
    绘制地形特征线。

    Parameters
    ----------
    dem : np.ndarray
        高程数组 (rows, cols)
    ridge_points : np.ndarray
        山脊线坐标 (N, 2)
    valley_points : np.ndarray
        山谷线坐标 (M, 2)
    ax : matplotlib.axes.Axes or None
    figsize : tuple

    Returns
    -------
    ax : matplotlib.axes.Axes
    """
    _check_matplotlib()

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    # 绘制DEM底图
    im = ax.imshow(dem, cmap='terrain', origin='lower', aspect='equal')
    plt.colorbar(im, ax=ax, label='高程 (m)')

    # 绘制山脊线
    if len(ridge_points) > 0:
        ax.scatter(ridge_points[:, 1], ridge_points[:, 0],
                   c='red', s=1, alpha=0.5, label=f'山脊线 ({len(ridge_points)} 个点)')

    # 绘制山谷线
    if len(valley_points) > 0:
        ax.scatter(valley_points[:, 1], valley_points[:, 0],
                   c='blue', s=1, alpha=0.5, label=f'山谷线 ({len(valley_points)} 个点)')

    ax.set_xlabel('列')
    ax.set_ylabel('行')
    ax.set_title('地形特征线')
    ax.legend(loc='upper right')

    return ax


def plot_multi_direction_cost(cost_field, n_show=4, figsize=(16, 4)):
    """
    绘制多个方向的代价场对比图。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    n_show : int
        显示的方向数
    figsize : tuple

    Returns
    -------
    fig : matplotlib.figure.Figure
    """
    _check_matplotlib()

    N = cost_field.shape[2]
    indices = np.linspace(0, N - 1, n_show, dtype=int)

    fig, axes = plt.subplots(1, n_show, figsize=figsize)
    if n_show == 1:
        axes = [axes]

    for i, idx in enumerate(indices):
        angle = idx * 360 / N
        data = cost_field[:, :, idx]
        im = axes[i].imshow(data, cmap='RdYlGn_r', origin='lower')
        axes[i].set_title(f'θ = {angle:.0f}°')
        axes[i].set_xlabel('列')
        axes[i].set_ylabel('行')
        plt.colorbar(im, ax=axes[i], shrink=0.8)

    fig.suptitle('方向敏感代价场', fontsize=14)
    plt.tight_layout()

    return fig
