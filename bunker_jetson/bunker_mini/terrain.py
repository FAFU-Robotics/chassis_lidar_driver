"""Terrain profiling & traversability — stage-2 rugged-lunar-terrain handling.

针对月球溶洞等「凹凸不平 + 碎石 + 岩壁 + 坡 + 坑」的复杂路况，单靠
360° 障碍扇区图（只回答“多远有东西”）不够——它无法区分：

  * 3 cm 的碎石（可通行，慢慢过）
  * 8 cm 的台阶 / 岩壁（不可通行，必须绕）
  * 10° 的坡面（可通行，但该降速，不能当墙急停）
  * 深坑 / 黑洞（数据缺失，保守降速）

本模块用雷达点云的**垂直维度**回答“前方能不能过去”：把点云按
「方位扇区 × 距离 bin」聚合，统计每格的高度，然后逐扇区扫描——

  * 相邻距离格高度**突变** > step_limit  → 不可通行障碍（台阶/岩壁/大石）
  * 高度**连续**爬升、总抬升大 → 坡面（降速不急停）
  * 中段连续无点、远处又有地面点 → 坑/黑洞（数据不可知，保守处理）
  * 格内最高点相对近地地面抬升明显 → 凸起高度（区分“低碎石”与“真障碍”）

纯 Python 实现，仅依赖 ``lidar.LidarPoint``。坐标系：雷达光心在原点，
垂直角 0° 为水平、向上为正（Airy 视场 0~90°），x 右、y 前。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:  # 仅在类型检查时导入，避免与 lidar.py 循环依赖
    from .lidar import LidarPoint

logger = __import__("logging").getLogger(__name__)

# 默认：按方位 3° 一个扇区、0.1 m 一个距离格、探测到车前 2.5 m
DEFAULT_SECTOR_COUNT: int = 120
DEFAULT_BIN_SIZE_M: float = 0.1
DEFAULT_MAX_RANGE_M: float = 2.5

# 默认允许的最大台阶/坑深度（米）。实车离地约 80 mm，可碾过 ≤3–4 cm 凸起，
# 5–6 cm 及以上视为不可通行。建图光心高度仍用 0.365 m，与本阈值无关。
DEFAULT_STEP_LIMIT_M: float = 0.04

# 最大可爬坡度（Δz/Δd，0.5 ≈ 26.6°）：超过视为“墙/岩壁/高堆石”，不可通行。
# BUNKER MINI 2.0 爬坡能力约 25°~30°，取 0.5 留安全余量。
DEFAULT_MAX_CLIMB_GRADE: float = 0.5

# 负障碍（悬崖/坑沿/陨石坑边）检测：相邻有数据格之间连续空档超过该水平距离，
# 且空档之后地面 z 相对空档之前下陷超过 step_limit → 判为不可通行下坠。
# 0.3 m ≈ 3 个 bin，保守覆盖月面小坑沿 / 断层。
DEFAULT_DROP_GAP_M: float = 0.3


@dataclass
class TerrainSectorResult:
    """Through-traffic verdict for one azimuth sector."""

    angle_deg: float                        # 扇区中心方位（车体系 0°=车头）
    obstacle_distance_m: Optional[float] = None  # 最近“不可通行”距离（台阶/岩壁/坑）
    negative_obstacle_distance_m: Optional[float] = None  # 最近“下坠”距离（悬崖/坑沿）
    is_slope: bool = False                  # 该方向是连续坡面
    slope_grade: float = 0.0                # 平均坡度 Δz/Δd（>0 上坡）
    max_height_m: float = 0.0               # 最大凸起高度（相对近地地面）
    unclear: bool = False                   # 数据缺失（黑洞/盲区），判定不可靠
    ground_z: Optional[float] = None        # 近地地面参考高度（雷达系 z）

    @property
    def blocked(self) -> bool:
        """不可通行：前方明确存在超过台阶限高的障碍或下坠地形。"""
        return self.obstacle_distance_m is not None \
            or self.negative_obstacle_distance_m is not None


@dataclass
class _Bin:
    count: int = 0
    sum_z: float = 0.0
    min_z: float = math.inf
    max_z: float = -math.inf


class TerrainProfile:
    """Aggregate point heights into (azimuth sector × range bin) cells.

    用法（由 ``AiryLidar`` 或模拟器在收到一帧后调用）::

        profile.clear()
        profile.add_frame(points)   # 一帧的全部 LidarPoint
        res = profile.sector(0.0)   # 车头方向的通过性判定
    """

    def __init__(
        self,
        sector_count: int = DEFAULT_SECTOR_COUNT,
        bin_size_m: float = DEFAULT_BIN_SIZE_M,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
    ) -> None:
        if sector_count <= 0 or bin_size_m <= 0 or max_range_m <= 0:
            raise ValueError("sector_count / bin_size_m / max_range_m must be > 0")
        self._sector_count = sector_count
        self._bin_size = bin_size_m
        self._n_bins = int(max_range_m / bin_size_m)
        self._sector_deg = 360.0 / sector_count
        self._cells: dict[tuple[int, int], _Bin] = {}

    # -- build ----------------------------------------------------------

    def clear(self) -> None:
        self._cells.clear()

    def add_frame(self, points: list[LidarPoint]) -> None:
        """Aggregate one LiDAR frame's points into the grid.

        Only points with a usable horizontal projection are kept; very
        short-range returns (< 5 cm) are near-field clutter and dropped.
        """
        for p in points:
            hd = p.distance_m * math.cos(math.radians(p.vertical_deg))
            if hd < 0.05 or hd > self._n_bins * self._bin_size:
                continue
            s = int(p.azimuth_deg / self._sector_deg) % self._sector_count
            b = int(hd / self._bin_size)
            cell = self._cells.setdefault((s, b), _Bin())
            cell.count += 1
            cell.sum_z += p.z
            cell.min_z = min(cell.min_z, p.z)
            cell.max_z = max(cell.max_z, p.z)

    # -- query ----------------------------------------------------------

    def sector(self, angle_deg: float, step_limit_m: float = DEFAULT_STEP_LIMIT_M,
               min_points: int = 1,
               max_climb_grade: float = DEFAULT_MAX_CLIMB_GRADE) -> TerrainSectorResult:
        """Terrain verdict for the sector centred on ``angle_deg`` (0°=front)."""
        s = int(angle_deg / self._sector_deg) % self._sector_count
        res = self._assess_sector(s, step_limit_m, min_points)
        # 太陡的“坡面”本质上是墙/岩壁/高堆石 → 视为不可通行
        if res.is_slope and abs(res.slope_grade) > max_climb_grade:
            res.is_slope = False
            res.obstacle_distance_m = self._first_filled_dist(s)
        return res

    def all_sectors(self, step_limit_m: float = DEFAULT_STEP_LIMIT_M,
                    min_points: int = 1,
                    max_climb_grade: float = DEFAULT_MAX_CLIMB_GRADE) -> list[TerrainSectorResult]:
        return [
            self._steep_to_blocked(self._assess_sector(i, step_limit_m, min_points),
                                   max_climb_grade)
            for i in range(self._sector_count)
        ]

    def nearest_blocked_deg(self, step_limit_m: float = DEFAULT_STEP_LIMIT_M,
                            min_points: int = 1,
                            max_climb_grade: float = DEFAULT_MAX_CLIMB_GRADE) -> Optional[float]:
        """Azimuth (deg) of the nearest blocked sector; None if all clear."""
        best: Optional[float] = None
        best_dist = math.inf
        for r in self.all_sectors(step_limit_m, min_points, max_climb_grade):
            if r.blocked and r.obstacle_distance_m is not None \
                    and r.obstacle_distance_m < best_dist:
                best_dist = r.obstacle_distance_m
                best = r.angle_deg
        return best

    # -- internal --------------------------------------------------------

    def _steep_to_blocked(self, res: TerrainSectorResult,
                          max_climb_grade: float) -> TerrainSectorResult:
        if res.is_slope and abs(res.slope_grade) > max_climb_grade:
            res.is_slope = False
            res.obstacle_distance_m = self._first_filled_dist(
                int(res.angle_deg / self._sector_deg))
        return res

    def _first_filled_dist(self, s: int) -> Optional[float]:
        """First distance with any data in sector ``s`` (for steep-wall marking)."""
        best: Optional[float] = None
        for (s2, b), c in self._cells.items():
            if s2 == s and c.count > 0:
                d = (b + 0.5) * self._bin_size
                if best is None or d < best:
                    best = d
        return best

    def _assess_sector(self, s: int, step_limit_m: float, min_points: int) -> TerrainSectorResult:
        angle = (s + 0.5) * self._sector_deg
        res = TerrainSectorResult(angle_deg=angle)

        # 收集该扇区有数据的 (bin_idx, bin)
        filled: list[tuple[int, _Bin]] = sorted(
            ((b, c) for (s2, b), c in self._cells.items() if s2 == s and c.count >= min_points)
        )
        if not filled:
            # 黑洞：该方向完全没有回波（盲区/极端远/传感器未覆盖）。
            res.unclear = True
            return res

        # 近地地面参考 = 最近有数据格的 min_z
        ground_z = filled[0][1].min_z
        res.ground_z = ground_z

        xs = [(b + 0.5) * self._bin_size for b, _ in filled]
        zs = [cell.min_z for _, cell in filled]
        span = xs[-1] - xs[0]
        total_rise = zs[-1] - zs[0]

        # 连续坡面判定：相邻格高度**单调**变化且步长受控（< 1.5×step_limit），
        # 或整扇区线性拟合 R² 高。台阶/岩壁/坑沿是“单步突变”，单调性被
        # 大跳变破坏或拟合残差大 → 判为不可通行。坡面采样点离散，
        # 不能用逐 bin 突变直接判 blocked（下坡/稀疏时会造成假突变）。
        is_slope = False
        grade = 0.0
        r2_ok = False
        if len(filled) >= 4 and span >= 0.3:
            mx = sum(xs) / len(xs)
            mz = sum(zs) / len(zs)
            num = sum((x - mx) * (z - mz) for x, z in zip(xs, zs))
            den = sum((x - mx) ** 2 for x in xs)
            if den > 0 and sum((z - mz) ** 2 for z in zs) > 0:
                slope = num / den
                pred = [mz + slope * (x - mx) for x in xs]
                ss_res = sum((z - p) ** 2 for z, p in zip(zs, pred))
                ss_tot = sum((z - mz) ** 2 for z in zs)
                r2_ok = (1.0 - ss_res / ss_tot) >= 0.6
        if len(filled) >= 2 and span >= 0.2 and abs(total_rise) > step_limit_m:
            diffs = [zs[i] - zs[i - 1] for i in range(1, len(zs))]
            monotonic = all(d >= 0 for d in diffs) or all(d <= 0 for d in diffs)
            small_steps = all(abs(d) <= step_limit_m * 1.5 for d in diffs)
            has_big_step = any(abs(d) > step_limit_m * 1.5 for d in diffs)
            if (monotonic and small_steps) or (r2_ok and not has_big_step):
                is_slope = True
                grade = total_rise / span

        res.is_slope = is_slope
        res.slope_grade = grade

        # 凸起高度（相对地面）与逐 bin 高度突变（仅非连续坡面时判定阻塞）
        prev_z: Optional[float] = None
        obstacle_set = False
        for (b, cell), z_min in zip(filled, zs):
            res.max_height_m = max(res.max_height_m, cell.max_z - ground_z)
            if prev_z is not None and not is_slope:
                if abs(z_min - prev_z) > step_limit_m and not obstacle_set:
                    res.obstacle_distance_m = (b + 0.5) * self._bin_size
                    obstacle_set = True
            prev_z = z_min

        # 负障碍/悬崖/坑沿检测：相邻有数据格之间出现连续空档（bin 跳变），
        # 且跳过空档后地面 z 相对之前下陷超过 step_limit → 不可通行下坠。
        # 典型：月面陨石坑边 / 断层 / 壕沟——近处地面可见、远处地面更深。
        # 区别于「黑洞」（整扇区无回波）：这里是「有近地回波 + 突然下坠」。
        gap_bins = int(math.ceil(DEFAULT_DROP_GAP_M / self._bin_size))
        for (b0, c0), (b1, c1) in zip(filled, filled[1:]):
            if b1 - b0 >= gap_bins and c1.min_z < c0.min_z - step_limit_m:
                res.negative_obstacle_distance_m = (b0 + 0.5) * self._bin_size
                break

        # 悬空岩/钟乳石：脚下地面还在（min_z≈ground），但同一格 max_z
        # 伸进车体高度。只看 min_z 突变会当成「平坦可走」。
        if not obstacle_set:
            for b, cell in filled:
                span_z = cell.max_z - cell.min_z
                if (span_z > step_limit_m
                        and 0.04 <= cell.max_z <= 0.45
                        and abs(cell.min_z - ground_z) <= step_limit_m * 1.5):
                    res.obstacle_distance_m = (b + 0.5) * self._bin_size
                    break

        # 近处是平坦地面、之后一直空到最大探测距离 → 溶洞口/悬崖
        # （底下没有回波，区别于「空隙后再出现更低地面」）。
        # 必须像地面：近处就有点、凸起小。单独一面墙后面全空不算悬崖。
        if (res.negative_obstacle_distance_m is None
                and len(filled) >= 4
                and filled[0][0] <= 2
                and res.max_height_m < step_limit_m * 1.5
                and not is_slope
                and (self._n_bins - 1 - filled[-1][0]) >= gap_bins):
            last_z = filled[-1][1].min_z
            if abs(last_z - ground_z) <= step_limit_m * 2.0:
                res.negative_obstacle_distance_m = (
                    (filled[-1][0] + 0.5) * self._bin_size)
        return res
