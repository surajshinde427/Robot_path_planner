"""2D occupancy grid: world-space AABB obstacles rasterized onto a boolean grid."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class AABB:
    """Axis-aligned obstacle footprint in world coordinates (meters)."""
    xmin: float
    ymin: float
    xmax: float
    ymax: float


class OccupancyGrid:
    """A rectangular world discretized into square cells of a given resolution.

    Cell (i, j) covers world x in [xmin + i*res, xmin + (i+1)*res) and
    y in [ymin + j*res, ymin + (j+1)*res).
    """

    def __init__(self, width: float, height: float, resolution: float,
                 obstacles: Iterable[AABB] = (), origin: tuple[float, float] = (0.0, 0.0)):
        if width <= 0 or height <= 0 or resolution <= 0:
            raise ValueError("width, height, and resolution must be positive")
        self.resolution = resolution
        self.origin = origin
        self.cols = int(np.ceil(width / resolution))
        self.rows = int(np.ceil(height / resolution))
        self.occupied = np.zeros((self.cols, self.rows), dtype=bool)
        self.obstacles = list(obstacles)
        for obs in self.obstacles:
            self._rasterize(obs)

    def _rasterize(self, obs: AABB) -> None:
        i0, j0 = self.world_to_cell(obs.xmin, obs.ymin)
        i1, j1 = self.world_to_cell(obs.xmax, obs.ymax)
        i0, i1 = sorted((max(i0, 0), min(i1, self.cols - 1)))
        j0, j1 = sorted((max(j0, 0), min(j1, self.rows - 1)))
        self.occupied[i0:i1 + 1, j0:j1 + 1] = True

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        ox, oy = self.origin
        i = int((x - ox) / self.resolution)
        j = int((y - oy) / self.resolution)
        return i, j

    def cell_to_world(self, i: int, j: int) -> tuple[float, float]:
        ox, oy = self.origin
        cx = ox + (i + 0.5) * self.resolution
        cy = oy + (j + 0.5) * self.resolution
        return cx, cy

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        i, j = cell
        return 0 <= i < self.cols and 0 <= j < self.rows

    def is_free(self, cell: tuple[int, int]) -> bool:
        if not self.in_bounds(cell):
            return False
        i, j = cell
        return not self.occupied[i, j]
