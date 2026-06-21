"""
direction_utils.py

用于生成 4/8/16/32 邻域移动集合。

约定：
- 栅格坐标为 (row, col)。
- x 轴对应列方向，向右为正；y 轴对应行方向，向下为正。
- 方向角 theta = atan2(dr, dc)，因此 0° 表示向右，90° 表示向下。
- 这与 cost_model 中 g_parallel = p cos(theta)+q sin(theta) 保持一致。
"""
from __future__ import annotations

from math import atan2, pi
from math import gcd
import numpy as np


def _unique_sorted_moves(moves: list[tuple[int, int]]) -> list[tuple[int, int]]:
    uniq = sorted(set(moves), key=lambda m: (atan2(m[0], m[1]) % (2*pi)))
    return uniq


def _mirror_first_quadrant(first_quadrant: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """由第一象限/坐标轴方向生成四象限方向。输入为 (dr>=0, dc>=0)。"""
    moves: set[tuple[int, int]] = set()
    for dr, dc in first_quadrant:
        candidates = [
            ( dr,  dc), ( dr, -dc), (-dr,  dc), (-dr, -dc),
            ( dc,  dr), ( dc, -dr), (-dc,  dr), (-dc, -dr),
        ]
        for m in candidates:
            if m != (0, 0):
                # 保留 primitive direction，避免 (2,2) 与 (1,1) 重复。
                g = gcd(abs(m[0]), abs(m[1]))
                moves.add((m[0]//g, m[1]//g))
    return _unique_sorted_moves(list(moves))


def get_moves(n_neighbors: int) -> list[tuple[int, int]]:
    """
    返回指定邻域数的移动向量。

    4: 仅上下左右。
    8: Moore 邻域，上下左右 + 对角。
    16: 在 8 邻域基础上增加斜率 1/2、2/1 的长边方向。
    32: 进一步增加 1/3、2/3、3/2、3/1 等方向。

    注意：16/32 邻域包含跨 2 或 3 个像元的边。路径规划时会沿边采样，
    防止直接“跳过”中间高代价像元。
    """
    if n_neighbors == 4:
        return _unique_sorted_moves([(0, 1), (1, 0), (0, -1), (-1, 0)])
    if n_neighbors == 8:
        return _mirror_first_quadrant([(0, 1), (1, 1), (1, 0)])
    if n_neighbors == 16:
        return _mirror_first_quadrant([(0, 1), (1, 2), (1, 1), (2, 1), (1, 0)])
    if n_neighbors == 32:
        return _mirror_first_quadrant([
            (0, 1),
            (1, 3),
            (1, 2),
            (2, 3),
            (1, 1),
            (3, 2),
            (2, 1),
            (3, 1),
            (1, 0),
        ])
    raise ValueError("n_neighbors 必须是 4、8、16 或 32")


def get_directions(n_neighbors: int) -> np.ndarray:
    moves = get_moves(n_neighbors)
    return np.array([(atan2(dr, dc) % (2*pi)) for dr, dc in moves], dtype=float)


def move_lengths_m(n_neighbors: int, cell_size: float) -> np.ndarray:
    return np.array([np.hypot(dr, dc) * cell_size for dr, dc in get_moves(n_neighbors)], dtype=float)


def describe_moves(n_neighbors: int) -> list[dict[str, float | int]]:
    rows = []
    for idx, (dr, dc) in enumerate(get_moves(n_neighbors)):
        theta = atan2(dr, dc) % (2*pi)
        rows.append({
            "方向序号": idx,
            "dr": dr,
            "dc": dc,
            "方向角_deg": float(np.degrees(theta)),
            "像元边长倍数": float(np.hypot(dr, dc)),
        })
    return rows
