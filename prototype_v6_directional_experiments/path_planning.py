"""
path_planning.py

支持 4/8/16/32 邻域的方向敏感 A* 路径规划。

输入 cost_unit_field 表示单位距离代价：
    c_unit(x,y,theta) = -ln(P)
路径边代价：
    C_edge = mean(c_unit along edge) * edge_length_m

对于 16/32 邻域，某些边会跨越 2~3 个像元。为避免路径“跳过”中间高代价区域，
本模块会沿线段采样多个中间像元并计算平均单位距离代价。
"""
from __future__ import annotations

import heapq
from dataclasses import dataclass
import numpy as np

from direction_utils import get_moves, get_directions


def angle_difference(a: np.ndarray | float, b: np.ndarray | float) -> np.ndarray | float:
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def nearest_direction_index(theta: float, directions: np.ndarray) -> int:
    return int(np.argmin(np.abs(angle_difference(directions, theta % (2 * np.pi)))))


@dataclass
class PathResult:
    path: list[tuple[int, int]]
    total_cost: float
    path_length_m: float
    average_cost_per_m: float
    detour_ratio: float


def _edge_sample_points(r: int, c: int, nr: int, nc: int) -> list[tuple[int, int]]:
    """返回边上的采样点，包括起终点。"""
    steps = int(max(abs(nr - r), abs(nc - c)))
    if steps <= 1:
        return [(r, c), (nr, nc)]
    pts = []
    for k in range(steps + 1):
        t = k / steps
        rr = int(round(r + t * (nr - r)))
        cc = int(round(c + t * (nc - c)))
        if not pts or pts[-1] != (rr, cc):
            pts.append((rr, cc))
    return pts


def _edge_unit_cost(
    cost_unit_field: np.ndarray,
    r: int,
    c: int,
    nr: int,
    nc: int,
    dir_idx: int,
    barrier_mask: np.ndarray | None,
) -> float | None:
    rows, cols, _ = cost_unit_field.shape
    vals = []
    for rr, cc in _edge_sample_points(r, c, nr, nc):
        if rr < 0 or rr >= rows or cc < 0 or cc >= cols:
            return None
        if barrier_mask is not None and barrier_mask[rr, cc]:
            return None
        v = cost_unit_field[rr, cc, dir_idx]
        if not np.isfinite(v):
            return None
        vals.append(float(v))
    return float(np.mean(vals))


def astar_directional(
    cost_unit_field: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    cell_size: float,
    directions: np.ndarray | None = None,
    n_neighbors: int = 8,
    barrier_mask: np.ndarray | None = None,
    edge_buffer: int = 0,
) -> PathResult:
    rows, cols, N = cost_unit_field.shape
    if directions is None:
        directions = get_directions(n_neighbors)
    moves = get_moves(n_neighbors)
    if len(directions) != len(moves):
        raise ValueError("directions 数量必须与移动邻域数量一致")

    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    if barrier_mask is None:
        mask = np.zeros((rows, cols), dtype=bool)
    else:
        mask = barrier_mask.astype(bool).copy()

    if edge_buffer > 0:
        mask[:edge_buffer, :] = True
        mask[-edge_buffer:, :] = True
        mask[:, :edge_buffer] = True
        mask[:, -edge_buffer:] = True
        mask[start] = False
        mask[goal] = False

    finite = cost_unit_field[np.isfinite(cost_unit_field)]
    c_min = max(float(np.min(finite)) if finite.size else 0.0, 0.0)

    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start))
    g_score = {start: 0.0}
    length_score = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()

    move_angles = [(np.arctan2(dr, dc) % (2*np.pi)) for dr, dc in moves]
    move_lengths = [float(np.hypot(dr, dc) * cell_size) for dr, dc in moves]
    move_dir_idx = [nearest_direction_index(theta, directions) for theta in move_angles]

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            total = g_score[current]
            length_m = length_score[current]
            straight_m = np.hypot(goal[0]-start[0], goal[1]-start[1]) * cell_size
            return PathResult(path, total, length_m, total / max(length_m, 1e-12), length_m / max(straight_m, 1e-12))
        closed.add(current)
        r, c = current

        for (dr, dc), edge_len_m, idx in zip(moves, move_lengths, move_dir_idx):
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue
            if mask[nr, nc]:
                continue
            neighbor = (nr, nc)
            if neighbor in closed:
                continue
            c_unit = _edge_unit_cost(cost_unit_field, r, c, nr, nc, idx, mask)
            if c_unit is None:
                continue
            tentative = g_score[current] + c_unit * edge_len_m
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                length_score[neighbor] = length_score[current] + edge_len_m
                h = c_min * np.hypot(nr-goal[0], nc-goal[1]) * cell_size
                counter += 1
                heapq.heappush(open_heap, (tentative + h, counter, neighbor))

    return PathResult([], float("inf"), float("inf"), float("inf"), float("inf"))


def evaluate_path_cost(
    cost_unit_field: np.ndarray,
    path: list[tuple[int, int]],
    cell_size: float,
    directions: np.ndarray | None = None,
    n_neighbors: int = 8,
) -> PathResult:
    if len(path) < 2:
        return PathResult(path, 0.0, 0.0, 0.0, 1.0)
    if directions is None:
        directions = get_directions(n_neighbors)
    total = 0.0
    length_m = 0.0
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dr, dc = r2-r1, c2-c1
        theta = np.arctan2(dr, dc) % (2*np.pi)
        idx = nearest_direction_index(theta, directions)
        c_unit = _edge_unit_cost(cost_unit_field, r1, c1, r2, c2, idx, None)
        if c_unit is None:
            return PathResult(path, float("inf"), float("inf"), float("inf"), float("inf"))
        edge_len = float(np.hypot(dr, dc) * cell_size)
        total += c_unit * edge_len
        length_m += edge_len
    start, goal = path[0], path[-1]
    straight_m = np.hypot(goal[0]-start[0], goal[1]-start[1]) * cell_size
    return PathResult(path, total, length_m, total / max(length_m, 1e-12), length_m / max(straight_m, 1e-12))


def path_cell_iou(path_a: list[tuple[int, int]], path_b: list[tuple[int, int]]) -> float:
    if not path_a or not path_b:
        return float("nan")
    A, B = set(path_a), set(path_b)
    return len(A & B) / max(len(A | B), 1)
