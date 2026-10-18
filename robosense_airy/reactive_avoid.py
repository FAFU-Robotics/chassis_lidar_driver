"""多扇区反应式避障决策（方式三）。

不建图、不做全局规划，纯实时点云密度驱动：
  * 把 360° 水平面等分成 N 个扇区；
  * 统计每个扇区内（给定半径内）的点数；
  * 按「正前方扇区 / 左右侧扇区」的数量阈值做决策：
        巡航(cruise) → 寻缝转向(spin_seek) → 倒车(reverse) → 缝隙缓行(gap_crawl)

本模块是主动避障：它不只减速/停车，还会主动寻找更空的方向绕行。
纯 numpy 实现，不依赖 wrs / panda3d。

基础巡航速度 CRUISE_VX 已按现场要求从 0.18 调低到 0.10 m/s，
GAP_CRAWL_LINEAR 从 0.10 调低到 0.07 m/s（保证穿缝慢于巡航）。
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import numpy as np

# ---- 基础速度常量（已调低，安全性优先）----
CRUISE_VX = 0.10  # 基础巡航速度（原 0.18，已调低）
CRUISE_VY = 0.0   # angular_z (rad/s)

GAP_CRAWL_LINEAR = 0.07   # 对准后缓行穿门，避免全速蹭履带
GAP_CRAWL_ANGULAR = 0.20  # 缓行时向更空侧微调

REVERSE_LINEAR = -0.10    # 前方被封死时短暂倒车
REVERSE_ANGULAR = 0.0

SPIN_LINEAR = 0.0         # 原地寻缝
SPIN_ANGULAR = 0.55

DECISION_CMDS: Dict[str, Tuple[float, float]] = {
    "cruise": (CRUISE_VX, CRUISE_VY),
    "spin_seek": (SPIN_LINEAR, SPIN_ANGULAR),
    "reverse": (REVERSE_LINEAR, REVERSE_ANGULAR),
    "gap_crawl": (GAP_CRAWL_LINEAR, GAP_CRAWL_ANGULAR),
    "stop": (0.0, 0.0),
}

# 默认扇区数：24 个 → 每扇区 15°
OBSTACLE_N_SECTORS = 24
OBSTACLE_RANGE_M = 1.5  # 只统计水平面该半径内的点（近处障碍才有意义）


def points_to_sector_counts(pcd: np.ndarray,
                            n_sectors: int = OBSTACLE_N_SECTORS,
                            max_range_m: float = OBSTACLE_RANGE_M,
                            min_range_m: float = 0.25) -> np.ndarray:
    """把点云投影到水平面并按方位角分桶，返回每扇区点数。

    pcd: (n, 3) 米制点云（x 前 / y 左 / z 上，车体系）。
    扇区 0 对应正前方，逆时针递增（与 atan2(y, x) 一致，车头向右为负）。
    滤除半径 < min_range_m 的点（紧贴车体的自噪点）。
    """
    counts = np.zeros(n_sectors, dtype=int)
    if pcd is None or len(pcd) == 0:
        return counts
    pcd = np.asarray(pcd, dtype=np.float64)
    r = np.hypot(pcd[:, 0], pcd[:, 1])
    mask = (r >= min_range_m) & (r <= max_range_m)
    pts = pcd[mask]
    if len(pts) == 0:
        return counts
    ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) % 360.0
    idx = (ang / (360.0 / n_sectors)).astype(int)
    idx = np.clip(idx, 0, n_sectors - 1)
    np.add.at(counts, idx, 1)
    return counts


def sector_range(deg_center: float, width_deg: float,
                 n_sectors: int = OBSTACLE_N_SECTORS) -> list[int]:
    """返回以 deg_center 为中心、宽度 width_deg 的扇区下标集合（环形回绕）。"""
    step = 360.0 / n_sectors
    lo = (deg_center - width_deg / 2.0) % 360.0
    hi = (deg_center + width_deg / 2.0) % 360.0
    i0 = int(round(lo / step)) % n_sectors
    i1 = int(round(hi / step)) % n_sectors
    out: list[int] = []
    i = i0
    while True:
        out.append(i % n_sectors)
        if i % n_sectors == i1:
            break
        i += 1
    return sorted(set(out))


class ObstacleAvoidance:
    """基于扇区点密度的反应式避障决策器。

    用法::

        oa = ObstacleAvoidance()
        pcd = lidar.get_pcd()
        counts = points_to_sector_counts(pcd)
        decision, v, w = oa.decide(counts)
        chassis.set_velocity(v, w)
    """

    def __init__(self,
                 n_sectors: int = OBSTACLE_N_SECTORS,
                 front_half_width_deg: float = 30.0,
                 front_block_thresh: int = 40,
                 side_half_width_deg: float = 45.0,
                 side_block_thresh: int = 25,
                 spin_min_angle_deg: float = 15.0) -> None:
        self.n_sectors = n_sectors
        self.front_half = front_half_width_deg
        self.front_thresh = front_block_thresh
        self.side_half = side_half_width_deg
        self.side_thresh = side_block_thresh
        self.spin_min_angle = spin_min_angle_deg

        # 预计算扇区下标（左右侧区与前向区刻意错开，避免边界扇区重叠：
        # 前向 = ±30°，左 = 45..135°，右 = 225..315°）
        self._front = sector_range(0.0, 2 * front_half_width_deg, n_sectors)
        self._left = sector_range(90.0, 2 * side_half_width_deg, n_sectors)
        self._right = sector_range(-90.0, 2 * side_half_width_deg, n_sectors)
        self._all = list(range(n_sectors))

        self.decision: str = "stop"
        self.last_counts: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    def decide(self, sector_counts: np.ndarray) -> Tuple[str, float, float]:
        """输入每扇区点数，返回 (decision, linear, angular)。"""
        counts = np.asarray(sector_counts, dtype=int)
        if counts.shape[0] != self.n_sectors:
            counts = np.resize(counts, self.n_sectors)
        self.last_counts = counts

        front = int(counts[self._front].sum())
        left = int(counts[self._left].sum())
        right = int(counts[self._right].sum())

        front_blocked = front >= self.front_thresh
        left_blocked = left >= self.side_thresh
        right_blocked = right >= self.side_thresh

        # 1) 正前方被封
        if front_blocked:
            # 侧向有空 → 原地转向最空侧（寻缝）
            if not left_blocked and not right_blocked:
                # 两侧都空：比较密度，转向更空一侧
                l_open = self._openness(counts, self._left)
                r_open = self._openness(counts, self._right)
                if r_open >= l_open:
                    self.decision = "spin_seek"
                    return "spin_seek", DECISION_CMDS["spin_seek"]
                self.decision = "spin_seek"
                return "spin_seek", (SPIN_LINEAR, -SPIN_ANGULAR)
            if not left_blocked:
                self.decision = "spin_seek"
                return "spin_seek", DECISION_CMDS["spin_seek"]
            if not right_blocked:
                self.decision = "spin_seek"
                return "spin_seek", (SPIN_LINEAR, -SPIN_ANGULAR)
            # 三面都堵 → 倒车
            self.decision = "reverse"
            return "reverse", DECISION_CMDS["reverse"]

        # 2) 正前方没堵，但两侧点很多 → 说明在窄缝里，缓行并朝空侧微调
        if left_blocked and right_blocked:
            self.decision = "gap_crawl"
            # 朝相对更空的一侧微调
            l_open = self._openness(counts, self._left)
            r_open = self._openness(counts, self._right)
            if r_open > l_open:
                return "gap_crawl", (GAP_CRAWL_LINEAR, GAP_CRAWL_ANGULAR)
            return "gap_crawl", (GAP_CRAWL_LINEAR, -GAP_CRAWL_ANGULAR)

        # 3) 巡航
        self.decision = "cruise"
        return "cruise", DECISION_CMDS["cruise"]

    # ------------------------------------------------------------------
    @staticmethod
    def _openness(counts: np.ndarray, sectors: list[int]) -> float:
        """扇区组的“开阔度” = 反点数（点数越少越空，至少 1 防除零）。"""
        total = int(counts[sectors].sum()) + 1
        return 1.0 / total


def main() -> int:
    """自测：打印典型场景的决策结果。"""
    n = OBSTACLE_N_SECTORS
    oa = ObstacleAvoidance(n_sectors=n)

    empty = np.zeros(n, dtype=int)
    print("空场     →", oa.decide(empty))

    front = np.zeros(n, dtype=int)
    front[oa._front] = 100
    print("前堵     →", oa.decide(front))

    both = np.zeros(n, dtype=int)
    both[oa._left] = 60
    both[oa._right] = 60
    print("双侧堵   →", oa.decide(both))

    all3 = front.copy()
    all3[oa._left] = 60
    all3[oa._right] = 60
    print("三面堵   →", oa.decide(all3))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
