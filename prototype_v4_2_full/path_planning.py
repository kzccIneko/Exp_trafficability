"""
path_planning_v3.py
方向敏感 A* 路径规划（审阅修订版）

输入 cost_unit_field 表示单位距离代价 c_unit(x,y,theta) = -ln(P)，
本模块在移动时乘以真实边长 cell_size 或 sqrt(2)*cell_size。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
import numpy as np


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


def eight_neighbor_moves() -> list[tuple[int, int]]:
    return [(-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)]


def astar_directional(
    cost_unit_field: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    cell_size: float,
    directions: np.ndarray | None = None,
    barrier_mask: np.ndarray | None = None,
    edge_buffer: int = 0,
    average_endpoints: bool = True,
) -> PathResult:
    """
    方向敏感 A*。

    cost_unit_field: (rows, cols, N)，单位距离代价。
    edge_cost = mean(c_unit[current], c_unit[neighbor]) * edge_length_m。
    """
    rows, cols, N = cost_unit_field.shape
    if directions is None:
        directions = np.linspace(0, 2 * np.pi, N, endpoint=False)
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    if barrier_mask is None:
        barrier_mask = np.zeros((rows, cols), dtype=bool)
    else:
        barrier_mask = barrier_mask.astype(bool).copy()

    if edge_buffer > 0:
        barrier_mask[:edge_buffer, :] = True
        barrier_mask[-edge_buffer:, :] = True
        barrier_mask[:, :edge_buffer] = True
        barrier_mask[:, -edge_buffer:] = True
        # 保留起终点可用，避免用户选点在缓冲区时无路径
        barrier_mask[start] = False
        barrier_mask[goal] = False

    moves = eight_neighbor_moves()
    move_lengths = [np.hypot(dr, dc) * cell_size for dr, dc in moves]
    move_angles = [(np.arctan2(dr, dc) % (2 * np.pi)) for dr, dc in moves]

    # 可采纳启发：用全局最小单位代价 * 欧氏距离。
    finite_cost = cost_unit_field[np.isfinite(cost_unit_field)]
    c_min = max(float(np.min(finite_cost)) if finite_cost.size else 0.0, 0.0)

    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start))
    g_score = {start: 0.0}
    length_score = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()

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
            straight_m = np.hypot(goal[0] - start[0], goal[1] - start[1]) * cell_size
            return PathResult(
                path=path,
                total_cost=total,
                path_length_m=length_m,
                average_cost_per_m=total / max(length_m, 1e-12),
                detour_ratio=length_m / max(straight_m, 1e-12),
            )
        closed.add(current)
        r, c = current

        for (dr, dc), edge_len_m, theta in zip(moves, move_lengths, move_angles):
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue
            if barrier_mask[nr, nc]:
                continue
            neighbor = (nr, nc)
            if neighbor in closed:
                continue
            idx = nearest_direction_index(theta, directions)
            c_from = cost_unit_field[r, c, idx]
            if average_endpoints:
                c_to = cost_unit_field[nr, nc, idx]
                c_unit = 0.5 * (c_from + c_to)
            else:
                c_unit = c_from
            if not np.isfinite(c_unit):
                continue
            edge_cost = float(c_unit) * edge_len_m
            tentative = g_score[current] + edge_cost
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                length_score[neighbor] = length_score[current] + edge_len_m
                h = c_min * np.hypot(nr - goal[0], nc - goal[1]) * cell_size
                counter += 1
                heapq.heappush(open_heap, (tentative + h, counter, neighbor))

    return PathResult([], float("inf"), float("inf"), float("inf"), float("inf"))


def evaluate_path_cost(
    cost_unit_field: np.ndarray,
    path: list[tuple[int, int]],
    cell_size: float,
    directions: np.ndarray | None = None,
    average_endpoints: bool = True,
) -> PathResult:
    """在给定代价场上评价一条已有路径。"""
    if len(path) < 2:
        return PathResult(path, 0.0, 0.0, 0.0, 1.0)
    N = cost_unit_field.shape[2]
    if directions is None:
        directions = np.linspace(0, 2 * np.pi, N, endpoint=False)

    total = 0.0
    length_m = 0.0
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dr, dc = r2 - r1, c2 - c1
        theta = np.arctan2(dr, dc) % (2 * np.pi)
        idx = nearest_direction_index(theta, directions)
        edge_len = np.hypot(dr, dc) * cell_size
        c_unit = cost_unit_field[r1, c1, idx]
        if average_endpoints:
            c_unit = 0.5 * (c_unit + cost_unit_field[r2, c2, idx])
        total += float(c_unit) * edge_len
        length_m += edge_len
    start, goal = path[0], path[-1]
    straight_m = np.hypot(goal[0] - start[0], goal[1] - start[1]) * cell_size
    return PathResult(path, total, length_m, total / max(length_m, 1e-12), length_m / max(straight_m, 1e-12))
