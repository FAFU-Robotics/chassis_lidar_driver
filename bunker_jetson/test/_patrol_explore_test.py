#!/usr/bin/env python3
"""Patrol exploration-memory tests (run offline, no CAN / LiDAR hardware).

验证「覆盖式探索」：
  * 无记忆时保持原「往最开阔方向」行为（正前方优先）
  * 有记忆时：刚扫过/走过的方向受罚 → 转向未探索方向
  * 探索记忆的单元格统计可用

用法：
    python3 -u _patrol_explore_test.py
"""

import sys
import unittest

from bunker_mini.navigator import Pose2D
from bunker_mini.occupancy import OccupancyGrid
from bunker_mini.patrol import PatrolConfig, PatrolController


class FakeLidar:
    """预设各方向距离的假雷达（角度 → 距离 m）。"""

    def __init__(self, distances: dict) -> None:
        self._d = distances
        self.is_receiving = True

    def nearest_in_range(self, angle_deg: float, width: float) -> float:
        a = int(round(angle_deg)) % 360
        if a > 180:
            a -= 360
        d = self._d.get(a, self._d.get(-a, 0.2))
        return d if d > 0 else 0.2

    def terrain_sector(self, angle_deg: float):
        return None


class FakeCtrl:
    def __init__(self) -> None:
        self.cmds: list[tuple[float, float]] = []
        self.stopped = False

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        self.stopped = True


def _open_lidar(dist: float) -> FakeLidar:
    return FakeLidar({0: dist, 30: dist, -30: dist, 60: dist, -60: dist,
                      90: dist, -90: dist, 120: dist, -120: dist,
                      150: dist, -150: dist})


class PatrolExploreTest(unittest.TestCase):
    def test_no_memory_keeps_forward_bias(self) -> None:
        ctrl = FakeCtrl()
        lidar = _open_lidar(2.0)
        patrol = PatrolController(ctrl, None, lidar, config=PatrolConfig())
        patrol.start()
        v, w = patrol.update()
        # 无记忆：正前方有 0.3m 加成 → 应选 0°
        self.assertEqual(patrol.heading_deg, 0.0)
        self.assertGreaterEqual(v, 0.0)

    def test_memory_biases_toward_unexplored(self) -> None:
        ctrl = FakeCtrl()
        lidar = _open_lidar(2.0)
        grid = OccupancyGrid(resolution_m=0.1)
        pose = Pose2D(0.0, 0.0, 0.0)
        cfg = PatrolConfig()
        patrol = PatrolController(
            ctrl, None, lidar, config=cfg,
            pose_fn=lambda: pose, explore_grid=grid,
        )
        patrol.start()
        v, w = patrol.update()
        # 0° / ±30° 被 mark 为「已访问」（近期 → 惩罚）；
        # 60° 未访问（→ 加分）。60° 先于 -60° 出现 → 应偏向 +60°。
        self.assertEqual(patrol.heading_deg, 60.0)
        self.assertGreater(patrol.explored_cell_count, 0)
        # 速度仍为正（向前巡游）
        self.assertGreaterEqual(v, 0.0)

    def test_explore_score_boost_and_penalty(self) -> None:
        grid = OccupancyGrid(resolution_m=0.1)
        pose = Pose2D(0.0, 0.0, 0.0)
        patrol = PatrolController(
            FakeCtrl(), None, _open_lidar(2.0), config=PatrolConfig(),
            pose_fn=lambda: pose, explore_grid=grid,
        )
        # 未访问方向 → 加分
        self.assertEqual(patrol._explore_score(60.0, 2.0), patrol.config.explore_boost_m)
        # 模拟访问过 (pose 前方 60° 处格)
        a = 60.0
        import math
        from bunker_mini.patrol import _RAD_PER_DEG
        look = patrol.config.explore_lookahead_m
        cx, cy = grid.world_to_cell(look * math.cos(math.radians(a)),
                                    look * math.sin(math.radians(a)))
        patrol._visited[(cx, cy)] = 0.0  # 很久以前
        # 很久前访问 → 中性
        self.assertEqual(patrol._explore_score(60.0, 2.0), 0.0)
        # 近期访问 → 惩罚
        import time
        patrol._visited[(cx, cy)] = time.time()
        self.assertEqual(patrol._explore_score(60.0, 2.0),
                         -patrol.config.explore_penalty_m)

    def test_heading_picked_even_after_previous_visit(self) -> None:
        """车已走过一条走廊后，再次面对同一条走廊 → 选未走过的岔路。"""
        ctrl = FakeCtrl()
        lidar = FakeLidar({0: 2.0, 30: 2.0, -30: 2.0, 60: 2.0, -60: 2.0,
                           90: 1.0, -90: 1.0, 120: 1.0, -120: 1.0,
                           150: 0.3, -150: 0.3})  # 150 被堵
        grid = OccupancyGrid(resolution_m=0.1)
        pose = Pose2D(1.5, 0.0, 0.0)  # 已深入走廊 1.5m
        # 预先标记 0°/±30° 已访问（走过来的方向）
        import math
        from bunker_mini.patrol import _RAD_PER_DEG
        for ang in (0.0, 30.0, -30.0):
            a = math.radians(ang)
            for d in (0.5, 1.0, 1.5):
                cx, cy = grid.world_to_cell(pose.x + d * math.cos(a),
                                            pose.y + d * math.sin(a))
                grid._cells[(cx, cy)] = 1
        patrol = PatrolController(
            ctrl, None, lidar, config=PatrolConfig(),
            pose_fn=lambda: pose, explore_grid=grid,
        )
        patrol._visited = {c: 0.0 for c in grid._cells if grid._cells[c] == 1}
        patrol.start()
        patrol.update()
        # 前方(0/±30)已访问 → 罚；±60 未访问且开放 → 选 ±60
        self.assertIn(patrol.heading_deg, (60.0, -60.0))


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(PatrolExploreTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
