"""Lightweight scan matching (polar-correlation) for odometry drift bounding.

在队友的 SLAM 地图落地之前，里程计漂移无法被外部绝对位姿修正，而松软
月面（轮滑/松土）下漂移随距离无界累积——搜索记忆、目标坐标、回程全都
建立在这套里程上。Airy 每 ~100ms 出一帧 360° 测距图，**足够做一个零
第三方依赖的轻量扫描匹配**：

  1. 用校正后位姿把每一帧极坐标扇区图注册进自身的参考地图；
  2. 匹配时在当前位姿附近小窗口内搜索 (dx, dy, dyaw)，使当前帧的障碍
     点最贴合参考地图障碍；
  3. 得分足够高且「最优 vs 次优」分明（防止对称走廊误匹配）时返回修正量，
     由调用方（agent）以融合系数叠加进导航位姿——把漂移从「无界累积」
     变成「每个匹配周期被界住一次」。

安全设计：
  * 只在小窗口内搜索（默认 ±0.4 m / ±10°），匹配错误最坏也只引入很小的
    位姿偏移，不会跳变；
  * 得分不达标 / 优次不分明 → 返回 None，宁可不修正；
  * 修正量由 agent 以 ``blend`` 系数缓慢融合，单帧不突变。

组员建图完成后，``pose_source`` 提供绝对位姿时本匹配自然退居兜底。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .occupancy import BLOCKED, OCCUPIED, OccupancyGrid

_DEG = math.pi / 180.0

# 障碍取值（含不可通行地形）
_OBS_VALUES = frozenset({OCCUPIED, BLOCKED})


@dataclass
class ScanMatchConfig:
    """Tuning parameters for the lightweight scan matcher."""

    max_tx_m: float = 0.40          # 平移搜索窗口（米）
    max_ty_m: float = 0.40
    max_dyaw_deg: float = 10.0      # 旋转搜索窗口（度）
    yaw_step_deg: float = 1.0       # 旋转搜索步长
    match_tol_cells: int = 1        # 命中容差（格数，参考障碍膨胀半径）
    min_score: float = 0.55         # 最低匹配得分（0~1）
    min_margin: float = 0.10        # 最优与次优得分差下限（防对称误匹配）
    ref_capacity_cells: int = 8000  # 参考地图格数上限（超出重置，防无限增长）
    resolution_m: float = 0.10      # 参考地图分辨率


class ScanMatcher:
    """Polar-correlation scan matcher against a self-built local map.

    坐标约定与 ``navigator.OdometryPose`` / ``occupancy.OccupancyGrid`` 一致：
    车体系 x 右 / y 前 / 方位角 0°=车头左转为正；里程系 yaw=0 时车头沿
    世界 +x，逆时针为正。扇区输入为 ``[(azimuth_deg, distance_m)]``。
    """

    def __init__(
        self,
        grid: Optional[OccupancyGrid] = None,
        *,
        config: Optional[ScanMatchConfig] = None,
    ) -> None:
        self._config = config or ScanMatchConfig()
        self._res = self._config.resolution_m
        self._grid = grid if grid is not None else OccupancyGrid(
            resolution_m=self._res,
            max_range_m=12.0,
        )
        self._ref_exact: frozenset[tuple[int, int]] = frozenset()
        self._ref_near: frozenset[tuple[int, int]] = frozenset()
        self._ref_epoch = -1
        self._map_epoch = 0

    # -- observation -----------------------------------------------------

    def observe(self, sectors, x: float, y: float, yaw_deg: float) -> None:
        """把当前帧极坐标扇区图注册进参考地图（用校正后位姿）。"""
        if sectors:
            self._grid.update(x, y, yaw_deg, sectors, None)
        if self._grid.cell_count > self._config.ref_capacity_cells:
            self._grid.reset()
        self._ref_exact = frozenset()  # 强制重建参考集
        self._ref_near = frozenset()
        self._map_epoch += 1

    def reset(self) -> None:
        """清空参考地图（重定位/重新开始搜索时）。"""
        self._grid.reset()
        self._ref_exact = frozenset()
        self._ref_near = frozenset()
        self._map_epoch += 1

    @property
    def map_cell_count(self) -> int:
        return self._grid.cell_count

    @property
    def has_map(self) -> bool:
        """参考地图是否已积累足够（至少 3 格）可用于匹配。"""
        return self._grid.cell_count >= 3

    # -- matching --------------------------------------------------------

    def match(
        self,
        sectors,
        x: float,
        y: float,
        yaw_deg: float,
    ) -> Optional[tuple[float, float, float]]:
        """匹配当前帧 → 修正量 (dx_m, dy_m, dyaw_deg)；不可信返回 None。

        返回的修正量表示「里程计位姿需要平移 (dx, dy) 米 / 旋转 dyaw 度
        才与参考地图最吻合」。调用方自行决定如何融合。
        """
        cfg = self._config
        if not sectors or not self.has_map:
            return None
        ref_exact, ref_near = self._ref_sets()

        yaw = math.radians(yaw_deg)
        # 扇区点 → 车体系 (vx, vy)：0°=车头(y+)、左转为正
        pts: list[tuple[float, float]] = []
        for az_deg, dist in sectors:
            if dist <= 0 or dist > 12.0:
                continue
            a = math.radians(az_deg)
            pts.append((dist * math.sin(a), dist * math.cos(a)))
        if len(pts) < 8:
            return None
        n = len(pts)

        # 平移搜索在格单位进行（步长 = 分辨率，平移=整格偏移，查表极快）
        max_tc = max(1, int(round(max(cfg.max_tx_m, cfg.max_ty_m) / self._res)))
        tx_cells = list(range(-max_tc, max_tc + 1))
        ty_cells = list(range(-max_tc, max_tc + 1))
        dyaw_steps = list(
            range(-int(round(cfg.max_dyaw_deg / cfg.yaw_step_deg)),
                  int(round(cfg.max_dyaw_deg / cfg.yaw_step_deg)) + 1))

        best: Optional[tuple[float, float, float]] = None
        best_score = -1.0
        second = -1.0
        for dya in dyaw_steps:
            dyaw = dya * cfg.yaw_step_deg
            yaw_d = yaw + dyaw * _DEG
            sy, cy = math.sin(yaw_d), math.cos(yaw_d)
            # 预计算该 dyaw 下每个点相对 (x, y) 的世界系偏移 + 基准格
            base_cells: list[tuple[int, int]] = []
            for vx, vy in pts:
                wx = x + vx * sy + vy * cy
                wy = y - vx * cy + vy * sy
                base_cells.append(self._grid.world_to_cell(wx, wy))
            for dxc in tx_cells:
                for dyc in ty_cells:
                    hits = 0.0
                    for cx, ccy in base_cells:
                        k = (cx + dxc, ccy + dyc)
                        if k in ref_exact:
                            hits += 1.0
                        elif k in ref_near:
                            hits += 0.5
                    score = hits / n
                    if score > best_score:
                        second = best_score
                        best_score = score
                        best = (dxc * self._res, dyc * self._res, dyaw)
                    elif score > second:
                        second = score

        if best is None:
            return None
        if best_score < cfg.min_score or (best_score - second) < cfg.min_margin:
            return None
        return best

    # -- internal --------------------------------------------------------

    def _ref_sets(self) -> tuple[frozenset[tuple[int, int]],
                                 frozenset[tuple[int, int]]]:
        """参考障碍格（精确）+ 膨胀格（近邻），按地图纪元缓存。

        精确命中计满分、近邻命中计半分——既容忍传感器噪声（相邻格），又
        保持「真偏移处得分显著更高」的对比度，避免优次门槛把所有结果
        都拒掉（纯膨胀会让得分面太平）。
        """
        if self._ref_exact and self._ref_epoch == self._map_epoch:
            return self._ref_exact, self._ref_near
        occ = {c for c, v in self._grid.iter_cells() if v in _OBS_VALUES}
        r = max(1, int(self._config.match_tol_cells))
        near: set[tuple[int, int]] = set()
        for cx, ccy in occ:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx == 0 and dy == 0:
                        continue
                    near.add((cx + dx, ccy + dy))
        self._ref_exact = frozenset(occ)
        self._ref_near = frozenset(near)
        self._ref_epoch = self._map_epoch
        return self._ref_exact, self._ref_near
