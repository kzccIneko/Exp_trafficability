"""
OSM验证模块

基于OpenStreetMap路网数据，对通行能力模型进行客观验证。
核心思想：真实的山路是人类长期选择的结果，应对应较低的通行代价。
"""

import numpy as np


def extract_osm_roads(place_name=None, bbox=None,
                      road_types=None):
    """
    提取OSM道路数据。

    使用osmnx库提取指定区域的道路网络。

    Parameters
    ----------
    place_name : str or None
        地名（如 "雅江县, 四川, 中国"）
    bbox : tuple or None
        边界框 (north, south, east, west)，与place_name二选一
    road_types : list of str
        道路类型筛选，默认 ['track', 'path', 'unclassified']

    Returns
    -------
    roads : list of dict
        道路列表，每条道路包含：
        - geometry: 坐标列表 [(lon, lat), ...]
        - highway: 道路类型
        - name: 道路名称（如有）
    """
    if road_types is None:
        road_types = ['track', 'path', 'unclassified']

    try:
        import osmnx as ox
    except ImportError:
        raise ImportError("需要安装 osmnx 库: pip install osmnx")

    # 获取道路网络
    if place_name:
        G = ox.graph_from_place(place_name, network_type='drive')
    elif bbox:
        north, south, east, west = bbox
        G = ox.graph_from_bbox(north, south, east, west, network_type='drive')
    else:
        raise ValueError("必须提供 place_name 或 bbox")

    roads = []
    for u, v, data in G.edges(data=True):
        highway = data.get('highway', '')
        # highway 可能是字符串或列表
        if isinstance(highway, list):
            hw_type = highway[0]
        else:
            hw_type = highway

        if hw_type in road_types:
            # 获取节点坐标
            u_data = G.nodes[u]
            v_data = G.nodes[v]
            geometry = [(u_data['x'], u_data['y']),
                        (v_data['x'], v_data['y'])]

            roads.append({
                'geometry': geometry,
                'highway': hw_type,
                'name': data.get('name', ''),
                'osmid': data.get('osmid', ''),
            })

    return roads


def calculate_road_cost(road_geometry, cost_field, cell_size, directions=None):
    """
    计算OSM道路的平均代价。

    沿道路采样点，计算每个点的代价（考虑道路方向）。

    Parameters
    ----------
    road_geometry : list of tuple
        道路坐标列表 [(lon, lat), ...] 或 [(row, col), ...]
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    cell_size : float
        栅格大小（米）
    directions : np.ndarray or None
        方向角度数组

    Returns
    -------
    avg_cost : float
        道路平均代价
    sample_costs : list of float
        各采样点的代价
    """
    if len(road_geometry) < 2:
        return 0.0, []

    rows, cols, N = cost_field.shape
    if directions is None:
        directions = np.linspace(0, 2 * np.pi, N, endpoint=False)

    sample_costs = []

    for i in range(len(road_geometry) - 1):
        p1 = np.array(road_geometry[i])
        p2 = np.array(road_geometry[i + 1])

        # 计算道路方向
        dp = p2 - p1
        road_theta = np.arctan2(dp[1], dp[0]) % (2 * np.pi)

        # 在路段上采样
        seg_length = np.linalg.norm(dp)
        n_samples = max(2, int(seg_length / cell_size))

        for j in range(n_samples):
            t = j / (n_samples - 1) if n_samples > 1 else 0
            point = p1 + t * dp
            r, c = int(point[1]), int(point[0])  # (row, col)

            # 边界检查
            if 0 <= r < rows and 0 <= c < cols:
                # 找到最接近的方向索引
                dir_idx = np.argmin(np.abs(directions - road_theta))
                sample_costs.append(cost_field[r, c, dir_idx])

    avg_cost = np.mean(sample_costs) if sample_costs else 0.0
    return avg_cost, sample_costs


def calculate_random_path_cost(cost_field, road_endpoints, n_paths=100, seed=42):
    """
    计算随机路径的平均代价。

    生成相同起终点的随机路径，计算平均代价。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    road_endpoints : list of tuple
        道路起终点列表 [((r1,c1), (r2,c2)), ...]
    n_paths : int
        每对起终点生成的随机路径数
    seed : int
        随机种子

    Returns
    -------
    avg_cost : float
        随机路径的平均代价
    all_costs : list of float
        所有随机路径的代价
    """
    from .path_planning import generate_random_paths

    all_costs = []

    for start, goal in road_endpoints:
        _, costs = generate_random_paths(cost_field, start, goal,
                                         n_paths=n_paths, seed=seed)
        all_costs.extend(costs.tolist())

    avg_cost = np.mean(all_costs) if all_costs else 0.0
    return avg_cost, all_costs


def cost_ratio(osm_cost, random_cost):
    """
    计算代价比（Cost Ratio）。

    公式：CR = C_osm / C_random

    物理意义：
    - CR < 1：OSM道路代价低于随机路径 → 模型合理
    - CR ≈ 1：OSM道路没有明显优势 → 模型可能有问题
    - CR > 1：OSM道路代价高于随机路径 → 模型有问题

    Parameters
    ----------
    osm_cost : float
        OSM道路的平均代价
    random_cost : float
        随机路径的平均代价

    Returns
    -------
    cr : float
        代价比
    """
    if random_cost == 0:
        return np.inf
    return osm_cost / random_cost


def rank_percentile(osm_cost, all_costs):
    """
    计算排名百分位（Rank Percentile）。

    公式：RP = 排名 / 总数 × 100%

    物理意义：
    - RP < 30%：OSM道路属于低代价路径 → 模型好
    - RP ≈ 50%：OSM道路处于中等水平 → 模型一般
    - RP > 70%：OSM道路属于高代价路径 → 模型差

    Parameters
    ----------
    osm_cost : float
        OSM道路的平均代价
    all_costs : list of float
        所有路径的代价列表

    Returns
    -------
    rp : float
        排名百分位（0-100）
    """
    if len(all_costs) == 0:
        return 50.0

    all_costs = np.array(all_costs)
    rank = np.sum(all_costs <= osm_cost)
    rp = (rank / len(all_costs)) * 100.0
    return rp


def direction_consistency(road_directions, optimal_directions):
    """
    计算方向一致性（Direction Consistency）。

    公式：DC = mean(|cos(θ_road - θ_optimal)|)

    物理意义：
    - DC ≈ 1：OSM道路方向与最优方向高度一致 → 模型能正确预测最优方向
    - DC ≈ 0.7：有一定一致性 → 模型基本合理
    - DC ≈ 0：方向随机 → 模型无法预测最优方向

    Parameters
    ----------
    road_directions : np.ndarray
        OSM道路各段的方向（弧度）
    optimal_directions : np.ndarray
        对应位置的模型最优方向（弧度）

    Returns
    -------
    dc : float
        方向一致性（0-1）
    """
    if len(road_directions) == 0:
        return 0.0

    road_directions = np.array(road_directions)
    optimal_directions = np.array(optimal_directions)

    dc = np.mean(np.abs(np.cos(road_directions - optimal_directions)))
    return dc


def feature_line_coincidence(road_points, feature_lines, threshold):
    """
    计算特征线重合率（Feature Line Coincidence）。

    公式：FLC = 点数(d < threshold) / 总点数

    物理意义：
    - FLC > 0.7：大部分OSM道路靠近特征线 → 假设成立
    - FLC ≈ 0.5：约一半靠近特征线 → 假设部分成立
    - FLC < 0.3：大部分远离特征线 → 假设不成立

    Parameters
    ----------
    road_points : np.ndarray
        道路采样点坐标 (N, 2)，每行为 (row, col)
    feature_lines : np.ndarray
        特征线点坐标 (M, 2)
    threshold : float
        距离阈值（米）

    Returns
    -------
    flc : float
        特征线重合率（0-1）
    """
    if len(road_points) == 0 or len(feature_lines) == 0:
        return 0.0

    # 计算每个道路点到最近特征线点的距离
    from scipy.spatial import cKDTree
    tree = cKDTree(feature_lines)
    distances, _ = tree.query(road_points)

    # 统计距离小于阈值的点数
    count_close = np.sum(distances < threshold)
    flc = count_close / len(road_points)

    return flc


def run_validation(cost_field, osm_roads, cell_size, directions=None,
                   n_random_paths=100, seed=42):
    """
    运行完整的OSM验证流程。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    osm_roads : list of dict
        OSM道路列表
    cell_size : float
        栅格大小（米）
    directions : np.ndarray or None
        方向角度数组
    n_random_paths : int
        随机路径数量
    seed : int
        随机种子

    Returns
    -------
    metrics : dict
        验证指标字典
    """
    # 计算OSM道路代价
    osm_costs = []
    road_directions = []

    for road in osm_roads:
        avg_cost, sample_costs = calculate_road_cost(
            road['geometry'], cost_field, cell_size, directions)
        if avg_cost > 0:
            osm_costs.append(avg_cost)
            # 计算道路方向
            geom = road['geometry']
            for i in range(len(geom) - 1):
                dp = np.array(geom[i + 1]) - np.array(geom[i])
                road_directions.append(np.arctan2(dp[1], dp[0]))

    avg_osm_cost = np.mean(osm_costs) if osm_costs else 0.0

    # 生成随机路径并计算代价
    rows, cols, N = cost_field.shape
    rng = np.random.RandomState(seed)
    road_endpoints = []
    for road in osm_roads:
        geom = road['geometry']
        if len(geom) >= 2:
            start = (int(geom[0][1]), int(geom[0][0]))
            goal = (int(geom[-1][1]), int(geom[-1][0]))
            if (0 <= start[0] < rows and 0 <= start[1] < cols and
                    0 <= goal[0] < rows and 0 <= goal[1] < cols):
                road_endpoints.append((start, goal))

    avg_random_cost, all_random_costs = calculate_random_path_cost(
        cost_field, road_endpoints[:10],  # 限制数量避免计算过久
        n_paths=n_random_paths // max(len(road_endpoints[:10]), 1),
        seed=seed)

    # 计算最优方向
    if directions is None:
        directions = np.linspace(0, 2 * np.pi, N, endpoint=False)
    min_cost_idx = np.argmin(cost_field, axis=2)
    optimal_directions_field = directions[min_cost_idx]

    # 获取道路各点的最优方向
    optimal_dirs = []
    for road in osm_roads:
        geom = road['geometry']
        for i in range(len(geom) - 1):
            p = geom[i]
            r, c = int(p[1]), int(p[0])
            if 0 <= r < rows and 0 <= c < cols:
                optimal_dirs.append(optimal_directions_field[r, c])

    # 计算验证指标
    metrics = {
        'avg_osm_cost': avg_osm_cost,
        'avg_random_cost': avg_random_cost,
        'cost_ratio': cost_ratio(avg_osm_cost, avg_random_cost),
        'rank_percentile': rank_percentile(avg_osm_cost, all_random_costs),
        'direction_consistency': direction_consistency(
            np.array(road_directions[:len(optimal_dirs)]),
            np.array(optimal_dirs[:len(road_directions)])) if road_directions else 0.0,
        'n_roads': len(osm_roads),
        'n_roads_with_cost': len(osm_costs),
    }

    return metrics
