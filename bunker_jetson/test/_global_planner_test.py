#!/usr/bin/env python3
"""Global A* planner tests (run offline, no CAN / LiDAR needed).

用法：
    python3 -u _global_planner_test.py
"""

import sys
import unittest

from bunker_mini.global_planner import GlobalPlanner
from bunker_mini.occupancy import BLOCKED, FREE, OCCUPIED, OccupancyGrid


def _grid_with_cells(cells: dict[tuple[int, int], int],
                     res: float = 0.1) -> OccupancyGrid:
    grid = OccupancyGrid(resolution_m=res)
    with grid._lock:
        grid._cells.update(cells)
    return grid


def _cells_clear_row(row_y: int, x0: int, x1: int,
                     res: float = 0.1) -> dict[tuple[int, int], int]:
    """把整行（world y = row_y*res）从 x0 到 x1 标记为障碍，返回 cells。"""
    return {(cx, row_y): OCCUPIED for cx in range(x0, x1 + 1)}


class GlobalPlannerTest(unittest.TestCase):
    def test_straight_line_no_obstacle(self) -> None:
        grid = _grid_with_cells({})
        planner = GlobalPlanner(grid)
        path = planner.plan(0.0, 0.0, 3.0, 0.0)
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 2)
        self.assertAlmostEqual(path[0].x, 0.0)
        self.assertAlmostEqual(path[-1].x, 3.0)

    def test_goal_in_obstacle_returns_none(self) -> None:
        grid = _grid_with_cells({(30, 30): OCCUPIED})
        planner = GlobalPlanner(grid)
        path = planner.plan(0.0, 0.0, 3.05, 3.05)  # 落在 (30,30)
        self.assertIsNone(path)

    def test_wall_forces_detour(self) -> None:
        # 障碍墙在 world y=1.0m 处横跨 x∈[0.5, 2.5]，从 (0,0) 到 (3,0) 需绕行
        cells = {(cx, 10): OCCUPIED for cx in range(5, 26)}
        cells.update(_cells_clear_row(10, 5, 25))
        grid = _grid_with_cells(cells)
        planner = GlobalPlanner(grid, inflation_m=0.25)
        path = planner.plan(0.0, 0.0, 3.0, 0.0)
        self.assertIsNotNone(path)
        # 路径应避开障碍行 y≈1.0（waypoint 世界 y 不应落在障碍上）
        for wp in path[1:-1]:
            cx, cy = grid.world_to_cell(wp.x, wp.y)
            self.assertNotEqual(grid.cell_at(cx, cy), OCCUPIED)
            self.assertNotEqual(grid.cell_at(cx, cy), BLOCKED)
        # 起点终点正确
        self.assertAlmostEqual(path[0].x, 0.0)
        self.assertAlmostEqual(path[-1].x, 3.0)

    def test_fully_blocked_returns_none(self) -> None:
        # 封闭房间：起点在房内，终点在房外 → 无路可达
        cells: dict[tuple[int, int], int] = {}
        for cx in range(0, 30):          # 房间墙 y=0 与 y=20
            cells[(cx, 0)] = OCCUPIED
            cells[(cx, 20)] = OCCUPIED
        for cy in range(0, 21):          # 房间墙 x=0 与 x=29
            cells[(0, cy)] = OCCUPIED
            cells[(29, cy)] = OCCUPIED
        grid = _grid_with_cells(cells)
        planner = GlobalPlanner(grid)
        # 起点 (1.5,1.5) 在房内（格 15,15），终点 (5.0,5.0) 在房外
        path = planner.plan(1.5, 1.5, 5.0, 5.0)
        self.assertIsNone(path)

    def test_inflation_keeps_clearance(self) -> None:
        # 单点障碍在 (1.0, 0)：膨胀 0.3m → 附近格不可走，路径需绕开
        cells = {(10, 0): OCCUPIED}
        grid = _grid_with_cells(cells)
        planner = GlobalPlanner(grid, inflation_m=0.3)
        path = planner.plan(0.0, 0.0, 3.0, 0.0)
        self.assertIsNotNone(path)
        for wp in path:
            cx, cy = grid.world_to_cell(wp.x, wp.y)
            self.assertNotIn((cx, cy), planner._inflated)

    def test_smooth_reduces_waypoints(self) -> None:
        # 两端无障碍的 L 形：平滑后应少拐点（尽量直连）
        cells = {(cx, 10): OCCUPIED for cx in range(0, 11)}
        grid = _grid_with_cells(cells)
        planner = GlobalPlanner(grid, inflation_m=0.15)
        path = planner.plan(0.0, 0.0, 0.0, 3.0)  # 同 x 直线，不被墙挡
        self.assertIsNotNone(path)
        # 平滑后比原始 A* 拐点少（原始路径可能蛇形）
        raw_len = len(planner._a_star((0, 0), (0, 30)))
        self.assertLessEqual(len(path), raw_len)

    def test_plan_rebuilds_inflation_when_obstacle_moves(self) -> None:
        """旧膨胀不得卡住：障碍挪走后沿 x 轴应能直线通过。"""
        cells = {(10, 0): OCCUPIED}  # world (1.0, 0) 挡在 (0,0)→(3,0) 上
        grid = _grid_with_cells(cells)
        planner = GlobalPlanner(grid, inflation_m=0.3)
        first = planner.plan(0.0, 0.0, 3.0, 0.0)
        self.assertIsNotNone(first)
        # 第一次规划因膨胀会绕开；把障碍搬到 y=1.0
        with grid._lock:
            grid._cells.clear()
            grid._cells[(10, 10)] = OCCUPIED
        second = planner.plan(0.0, 0.0, 3.0, 0.0)
        self.assertIsNotNone(second)
        self.assertEqual(len(second), 2, "障碍离开 x 轴后应恢复直线")


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(GlobalPlannerTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
