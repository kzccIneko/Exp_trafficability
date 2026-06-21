"""
路径规划模块

实现支持方向敏感代价的修改版A*算法，以及随机路径生成。
"""

import numpy as np
import heapq


def modified_astar(cost_field, start, goal, directions=8):
    """
    修改版A*算法（支持方向敏感代价）。

    与标准A*的区别：
    1. 代价函数考虑移动方向：从节点到相邻节点的代价取决于移动方向
    2. 启发式函数使用最小代价：h(n) = C_min * 欧氏距离
    3. 节点状态包含方向信息

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    start : tuple
        起点坐标 (row, col)
    goal : tuple
        终点坐标 (row, col)
    directions : int
        移动方向数（4或8，默认8）

    Returns
    -------
    path : list of tuple
        路径坐标列表 [(row, col), ...]
    total_cost : float
        路径总代价
    """
    rows, cols, N = cost_field.shape

    # 预定义移动方向（8邻域）
    if directions == 8:
        moves = [(-1, -1), (-1, 0), (-1, 1),
                 (0, -1), (0, 1),
                 (1, -1), (1, 0), (1, 1)]
        move_dists = [np.sqrt(2), 1, np.sqrt(2),
                      1, 1,
                      np.sqrt(2), 1, np.sqrt(2)]
    else:  # 4邻域
        moves = [(-1, 0), (0, -1), (0, 1), (1, 0)]
        move_dists = [1, 1, 1, 1]

    # 计算每个移动方向对应的角度
    move_angles = np.array([np.arctan2(dr, dc) for dr, dc in moves])
    # 转换到 [0, 2π] 范围
    move_angles = move_angles % (2 * np.pi)

    # 代价场的方向角度
    dir_angles = np.linspace(0, 2 * np.pi, N, endpoint=False)

    # 预计算最小代价（用于启发式函数）
    min_cost = np.min(cost_field, axis=2)
    global_min_cost = max(np.min(min_cost), 0.01)  # 避免除零

    # A*搜索
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    # 开放列表：(f_cost, counter, position)
    counter = 0
    open_list = []
    heapq.heappush(open_list, (0, counter, start))

    # 从起点到当前点的代价
    g_score = {start: 0}

    # 记录路径
    came_from = {}

    # 记录已访问节点
    closed_set = set()

    while open_list:
        f_current, _, current = heapq.heappop(open_list)

        if current == goal:
            # 重建路径
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path, g_score[goal]

        if current in closed_set:
            continue
        closed_set.add(current)

        r, c = current

        for move_idx, (dr, dc) in enumerate(moves):
            nr, nc = r + dr, c + dc

            # 边界检查
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue

            neighbor = (nr, nc)
            if neighbor in closed_set:
                continue

            # 移动方向角度
            move_theta = move_angles[move_idx]
            # 找到代价场中最接近的方向索引（考虑 2π 周期性，Bug #4 修复）
            angle_diffs = np.abs(dir_angles - move_theta)
            angle_diffs = np.minimum(angle_diffs, 2 * np.pi - angle_diffs)
            dir_idx = np.argmin(angle_diffs)

            # 移动代价 = 单位距离代价 × 移动距离
            # （Bug #2 已修复：edge_cost 不再包含 l_theta，此处由 A* 统一乘距离）
            move_cost = cost_field[r, c, dir_idx] * move_dists[move_idx]

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                # 启发式函数：欧氏距离 × 最小代价
                h = global_min_cost * np.sqrt((nr - goal[0]) ** 2 +
                                               (nc - goal[1]) ** 2)
                f = tentative_g + h

                counter += 1
                heapq.heappush(open_list, (f, counter, neighbor))

    # 未找到路径
    return [], float('inf')


def calculate_direction_cost(cost_field, pos, direction_angle):
    """
    计算从某点沿某方向移动的代价。

    在代价场的方向维度中查找最接近的方向。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    pos : tuple
        位置坐标 (row, col)
    direction_angle : float
        方向角度（弧度）

    Returns
    -------
    cost : float
        该方向的代价
    """
    r, c = int(pos[0]), int(pos[1])
    N = cost_field.shape[2]
    dir_angles = np.linspace(0, 2 * np.pi, N, endpoint=False)

    # 找到最接近的方向索引（考虑 2π 周期性）
    target = direction_angle % (2 * np.pi)
    angle_diffs = np.abs(dir_angles - target)
    angle_diffs = np.minimum(angle_diffs, 2 * np.pi - angle_diffs)
    dir_idx = np.argmin(angle_diffs)

    return cost_field[r, c, dir_idx]


def heuristic(pos, goal, min_cost):
    """
    启发式函数（可采纳的）。

    使用欧氏距离乘以全局最小代价，保证不高估实际代价。

    Parameters
    ----------
    pos : tuple
        当前位置 (row, col)
    goal : tuple
        目标位置 (row, col)
    min_cost : float
        全局最小代价

    Returns
    -------
    h : float
        启发式估计值
    """
    return min_cost * np.sqrt((pos[0] - goal[0]) ** 2 +
                               (pos[1] - goal[1]) ** 2)


def generate_random_paths(cost_field, start, goal, n_paths=100, seed=42):
    """
    生成随机路径（用于验证和对比）。

    使用带随机扰动的简单A*（标量代价）生成多条路径。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    start : tuple
        起点坐标 (row, col)
    goal : tuple
        终点坐标 (row, col)
    n_paths : int
        生成路径数量
    seed : int
        随机种子

    Returns
    -------
    paths : list of list
        路径列表，每条路径为坐标列表
    costs : np.ndarray
        每条路径的平均代价
    """
    rng = np.random.RandomState(seed)
    rows, cols, N = cost_field.shape

    # 计算标量代价场（所有方向的平均值）
    scalar_cost = np.mean(cost_field, axis=2)
    scalar_cost = np.maximum(scalar_cost, 0.01)  # 避免零代价

    paths = []
    costs = []

    for _ in range(n_paths):
        # 给标量代价场添加随机扰动
        noise = rng.uniform(0.5, 1.5, size=scalar_cost.shape)
        perturbed_cost = scalar_cost * noise

        # 使用简单Dijkstra算法（标量代价）
        path, path_cost = _simple_dijkstra(perturbed_cost, start, goal)

        if len(path) > 0:
            # 计算路径在方向敏感代价场中的实际代价
            actual_cost = _evaluate_path_cost(cost_field, path)
            paths.append(path)
            costs.append(actual_cost / len(path))  # 平均每步代价

    return paths, np.array(costs)


def _simple_dijkstra(cost_grid, start, goal):
    """
    简单的Dijkstra算法（标量代价场）。

    Parameters
    ----------
    cost_grid : np.ndarray
        标量代价场 (rows, cols)
    start : tuple
        起点
    goal : tuple
        终点

    Returns
    -------
    path : list of tuple
        路径
    total_cost : float
        总代价
    """
    rows, cols = cost_grid.shape
    moves = [(-1, 0), (0, -1), (0, 1), (1, 0)]

    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    open_list = [(0, 0, start)]
    g_score = {start: 0}
    came_from = {}
    closed_set = set()
    counter = 0

    while open_list:
        f, _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path, g_score[goal]

        if current in closed_set:
            continue
        closed_set.add(current)

        r, c = current
        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                neighbor = (nr, nc)
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + cost_grid[nr, nc]

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = np.sqrt((nr - goal[0]) ** 2 + (nc - goal[1]) ** 2)
                    counter += 1
                    heapq.heappush(open_list, (tentative_g + h, counter, neighbor))

    return [], float('inf')


def _evaluate_path_cost(cost_field, path):
    """
    计算路径在方向敏感代价场中的总代价。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    path : list of tuple
        路径坐标列表

    Returns
    -------
    total_cost : float
        路径总代价
    """
    if len(path) < 2:
        return 0.0

    N = cost_field.shape[2]
    dir_angles = np.linspace(0, 2 * np.pi, N, endpoint=False)
    total = 0.0

    for i in range(len(path) - 1):
        r1, c1 = path[i]
        r2, c2 = path[i + 1]
        dr = r2 - r1
        dc = c2 - c1

        move_theta = np.arctan2(dr, dc) % (2 * np.pi)
        angle_diffs = np.abs(dir_angles - move_theta)
        angle_diffs = np.minimum(angle_diffs, 2 * np.pi - angle_diffs)
        dir_idx = np.argmin(angle_diffs)
        dist = np.sqrt(dr ** 2 + dc ** 2)

        total += cost_field[r1, c1, dir_idx] * dist

    return total


def find_optimal_path(cost_field, start, goal):
    """
    使用方向敏感A*找到最优路径。

    Parameters
    ----------
    cost_field : np.ndarray
        方向敏感代价场 (rows, cols, N_directions)
    start : tuple
        起点坐标 (row, col)
    goal : tuple
        终点坐标 (row, col)

    Returns
    -------
    path : list of tuple
        最优路径
    total_cost : float
        路径总代价
    avg_cost : float
        平均每步代价
    """
    path, total_cost = modified_astar(cost_field, start, goal, directions=8)

    if len(path) == 0:
        return [], 0.0, 0.0

    avg_cost = total_cost / len(path)
    return path, total_cost, avg_cost
