"""Adapter: nav_msgs/OccupancyGrid → OccupancyGrid-like object for official GlobalPlanner.

ROS /nav_demo/obstacle_grid values:
  100 occupied, 50 inflated (viz only), 0 free, -1 unknown.

Planner must see only occupied (100 → OCCUPIED=2). Inflation is re-applied by GlobalPlanner.
"""
from __future__ import annotations

import math
from typing import Iterable

OCCUPIED = 2
FREE = 1


class RosGridAdapter:
    def __init__(self, msg, occupied_code: int = 100) -> None:
        self._res = float(msg.info.resolution)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)
        self._w = int(msg.info.width)
        self._h = int(msg.info.height)
        data = list(msg.data)
        self._cells: dict[tuple[int, int], int] = {}
        for row in range(self._h):
            wy = self._origin_y + (row + 0.5) * self._res
            for col in range(self._w):
                idx = row * self._w + col
                if idx >= len(data):
                    continue
                val = int(data[idx])
                wx = self._origin_x + (col + 0.5) * self._res
                cx, cy = self.world_to_cell(wx, wy)
                if val >= occupied_code:
                    self._cells[(cx, cy)] = OCCUPIED
                elif val == 0:
                    self._cells[(cx, cy)] = FREE

    @property
    def resolution_m(self) -> float:
        return self._res

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor(x / self._res)), int(math.floor(y / self._res))

    def cell_at(self, cx: int, cy: int) -> int:
        return self._cells.get((cx, cy), 0)

    def iter_cells(self) -> list[tuple[tuple[int, int], int]]:
        return list(self._cells.items())

    def occupied_count(self) -> int:
        return sum(1 for v in self._cells.values() if v == OCCUPIED)

    def free_count(self) -> int:
        return sum(1 for v in self._cells.values() if v == FREE)


class FootprintClearGrid:
    """Official A* refuses a start cell that sits in inflation. The robot is already
    there, so occupied cells inside the hull radius are treated as free for planning.
    """

    def __init__(self, inner: RosGridAdapter, x: float, y: float, radius_m: float = 0.40) -> None:
        self._inner = inner
        self._x = float(x)
        self._y = float(y)
        self._r = float(radius_m)
        self._res = inner.resolution_m
        self._cells: dict[tuple[int, int], int] = {}
        for (cx, cy), val in inner.iter_cells():
            wx = (cx + 0.5) * self._res
            wy = (cy + 0.5) * self._res
            if val >= OCCUPIED and math.hypot(wx - self._x, wy - self._y) <= self._r:
                self._cells[(cx, cy)] = FREE
            else:
                self._cells[(cx, cy)] = val

    @property
    def resolution_m(self) -> float:
        return self._res

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return self._inner.world_to_cell(x, y)

    def cell_at(self, cx: int, cy: int) -> int:
        return self._cells.get((cx, cy), 0)

    def iter_cells(self) -> list[tuple[tuple[int, int], int]]:
        return list(self._cells.items())

    def occupied_count(self) -> int:
        return sum(1 for v in self._cells.values() if v == OCCUPIED)

    def free_count(self) -> int:
        return sum(1 for v in self._cells.values() if v == FREE)
