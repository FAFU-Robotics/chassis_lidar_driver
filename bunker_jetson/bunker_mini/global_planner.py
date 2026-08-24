"""Global A* path planner on top of the OccupancyGrid.

在里程系伪地图（``occupancy.OccupancyGrid``）上做 **A\\* 全局路径规划**：
把起点/终点转成格坐标，8 连通 A* 找一条避开障碍（含膨胀半径）的最短
折线，再做「视线拉直」平滑，返回世界系航点序列供 ``navigator.goto``
逐段执行。

不依赖 SLAM：地图来自雷达在线累积的伪地图，建图完成后可直接替换为
队友输出的全局栅格地图（实现 ``iter_cells``/``cell_at``/``world_to_cell``
接口即可无缝接入）。

坐标系与 ``navigator.OdometryPose`` / ``occupancy.OccupancyGrid`` 一致。
"""

from __future__ import annotations

import heapq
import logging
import math
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

# 栅格值：与 occupancy 模块保持一致
_OCCUPIED = 2
_BLOCKED = 3
_OBS_VALUES = frozenset({_OCCUPIED, _BLOCKED})


@dataclass(frozen=True)
class Waypoint:
    """全局航点（世界系，米）。"""
    x: float
    y: float


class GlobalPlanner:
    """A* path planner over an OccupancyGrid-like map. 非线程安全（调用方串行）。"""

    def __init__(
        self,
        grid,
        *,
        inflation_m: float = 0.30,
        step_limit: int = 200_000,
    ) -> None:
        """``grid`` 需提供 world_to_cell / cell_at / iter_cells 三个接口。"""
        self._grid = grid
        self._res = float(getattr(grid, "resolution_m", 0.1))
        self._inflation = max(inflation_m, self._res)
        self._step_limit = step_limit
        # 缓存：起点/终点附近格 + 膨胀障碍集，plan() 之间地图变化后失效
        self._inflated: Optional[frozenset[tuple[int, int]]] = None
        self._obs_cells: frozenset[tuple[int, int]] = frozenset()

    def plan(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> Optional[list[Waypoint]]:
        """返回世界系航点序列（含起点、终点）；无可行路径返回 None。

        直接可达（起点与终点间无膨胀障碍）时返回两点的直线路径，
        省去 A* 开销。
        """
        grid = self._grid
        self._rebuild_inflated()
        start = grid.world_to_cell(start_x, start_y)
        goal = grid.world_to_cell(goal_x, goal_y)
        if self._cell_traversable(start[0], start[1]) is False:
            return None
        if self._cell_traversable(goal[0], goal[1]) is False:
            return None

        if start == goal:
            return [Waypoint(start_x, start_y)]

        # 直接可达快速路径：直线视线内无膨胀障碍 → 两点直达
        if self._line_of_sight(start, goal):
            return [Waypoint(start_x, start_y), Waypoint(goal_x, goal_y)]

        raw = self._a_star(start, goal)
        if not raw:
            return None
        smoothed = self._smooth(raw)
        return [Waypoint((cx + 0.5) * self._res, (cy + 0.5) * self._res)
                for cx, cy in smoothed]

    # -- A* -------------------------------------------------------------

    def _a_star(self, start: tuple[int, int],
                goal: tuple[int, int]) -> Optional[list[tuple[int, int]]]:
        self._rebuild_inflated()
        open_h: list[tuple[float, int, tuple[int, int]]] = []  # (f, tie, cell)
        g: dict[tuple[int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        counter = 0
        heapq.heappush(open_h, (self._heuristic(start, goal), 0, start))
        visited = 0
        while open_h and visited < self._step_limit:
            f, _, current = heapq.heappop(open_h)
            if current == goal:
                return self._reconstruct(came_from, start, goal)
            visited += 1
            for nb in self._neighbors(current):
                cost = g[current] + self._move_cost(current, nb)
                if cost < g.get(nb, math.inf):
                    g[nb] = cost
                    came_from[nb] = current
                    h = self._heuristic(nb, goal)
                    counter += 1
                    heapq.heappush(open_h, (cost + h, counter, nb))
        logger.warning(
            "A* gave up: visited %d cells (step_limit=%d)", visited, self._step_limit)
        return None

    def _reconstruct(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [goal]
        cur = goal
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    def _neighbors(self, cell: tuple[int, int]) -> list[tuple[int, int]]:
        out: list[tuple[int, int]] = []
        cx, cy = cell
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if self._cell_traversable(nx, ny) is not False:
                    out.append((nx, ny))
        return out

    def _move_cost(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1]) * self._res

    def _heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1]) * self._res

    # -- traversability / inflation ------------------------------------

    def _rebuild_inflated(self) -> None:
        """膨胀障碍集：每个障碍格 + 半径 inflation 内的格都不可走。"""
        if self._inflated is not None:
            return
        radius = int(math.ceil(self._inflation / self._res))
        obs: set[tuple[int, int]] = set()
        for (cx, cy), value in self._grid.iter_cells():
            if value not in _OBS_VALUES:
                continue
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    obs.add((cx + dx, cy + dy))
        self._inflated = frozenset(obs)
        self._obs_cells = frozenset(obs)

    def _cell_traversable(self, cx: int, cy: int) -> Optional[bool]:
        """None=未知（放行），True=可行，False=障碍/不可通行。"""
        value = self._grid.cell_at(cx, cy)
        if value in _OBS_VALUES:
            return False
        if self._inflated is not None and (cx, cy) in self._inflated:
            return False
        return None  # 未知格默认放行（伪地图稀疏，未知≠墙）

    def _line_of_sight(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        """Bresenham 采样两点间所有格，若含膨胀障碍则 False。"""
        x0, y0 = a
        x1, y1 = b
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            if self._cell_traversable(x0, y0) is False:
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return True

    # -- smoothing ------------------------------------------------------

    def _smooth(self, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """视线拉直：能直连的中间拐点删除（两点间视线无障碍）。"""
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._line_of_sight(path[i], path[j]):
                    break
                j -= 1
            out.append(path[j])
            i = j
        return out

    # -- public helpers -------------------------------------------------

    @property
    def inflated_cell_count(self) -> int:
        return len(self._obs_cells)

    def reset_cache(self) -> None:
        """地图更新后调用，强制重建膨胀集。"""
        self._inflated = None
