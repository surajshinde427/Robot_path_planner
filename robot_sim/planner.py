"""A* path planning over an OccupancyGrid."""
from __future__ import annotations

import heapq
import math
from typing import Optional

from robot_sim.grid import OccupancyGrid

Cell = tuple[int, int]

# 8-connected neighborhood: (di, dj, step cost)
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _octile_heuristic(a: Cell, b: Cell) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def astar(grid: OccupancyGrid, start: Cell, goal: Cell) -> Optional[list[Cell]]:
    """Return a list of cells from start to goal (inclusive), or None if unreachable."""
    if not grid.is_free(start) or not grid.is_free(goal):
        return None
    if start == goal:
        return [start]

    open_heap: list[tuple[float, Cell]] = [(0.0, start)]
    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    visited: set[Cell] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return _reconstruct_path(came_from, current)

        for di, dj, step_cost in _NEIGHBORS:
            neighbor = (current[0] + di, current[1] + dj)
            if not grid.is_free(neighbor):
                continue
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + _octile_heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None


def _reconstruct_path(came_from: dict[Cell, Cell], current: Cell) -> list[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
