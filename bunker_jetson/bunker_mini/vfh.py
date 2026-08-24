"""Vector Field Histogram (VFH) — 基于几何距离的主动避障转向器。

解决「方式三 / steer_away_deg 只比较左右两侧点密度、不验证能否挤进去」
的泛化性短板：本模块**只依赖几何距离场**（每扇区 p25 最近障碍距离），
而不是点数密度，因此对距离 / 反射率 / 点密度都不敏感，泛化性显著更好。

核心流程::

    距离场 → 极坐标直方图（按「障碍距离」开缺口）
           → 提取连续「可通行缺口」（gap）
           → 用车宽 + 安全余量筛选「真能挤进去」的缺口
           → 评分选最优缺口（宽 + 前向加权 + 与目标航向一致性）
           → 输出目标航向与安全速度

不依赖 numpy（纯标准库），与项目 agent 侧模块风格一致。坐标系与
``navigator`` / ``lidar`` 一致：车体系 0°=车头、左转为正。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

_RAD_PER_DEG: float = math.pi / 180.0


def _wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


@dataclass
class Gap:
    """一个可通行缺口（角度区间，车体系度）。"""

    start_deg: float      # 缺口起始方位
    end_deg: float        # 缺口结束方位（含）
    min_distance_m: float # 缺口内最近障碍距离（越大越开阔）
    max_distance_m: float # 缺口内最远障碍距离

    @property
    def center_deg(self) -> float:
        return _wrap_deg((self.start_deg + self.end_deg) / 2.0)

    @property
    def width_deg(self) -> float:
        """缺口角跨度（0~360，环形跨 0° 也正确）。"""
        span = (self.end_deg - self.start_deg) % 360.0
        return 360.0 if span == 0.0 else span


@dataclass
class VFHConfig:
    """VFH 转向器参数。

    核心思想：某方向的「障碍距离」才是唯一输入，所有阈值都是物理量
    （米 / 度 / 车宽），因此随环境鲁棒，不依赖点密度。
    """

    sector_count: int = 72           # 极坐标直方图分辨率（5° 一格）
    clear_distance_m: float = 1.0    # 该方向障碍距离 ≥ 此值 → 视为「缺口/开放」
    vehicle_width_m: float = 0.36    # 车宽（BUNKER MINI 2.0 履带外侧）
    safety_margin_m: float = 0.20    # 缺口两侧额外安全余量（每侧）
    max_range_m: float = 5.0         # 探测距离上限（无回波视为开阔）
    # --- 评分权重 ---
    forward_bias_m: float = 0.5      # 前向方向的「距离当量」加成（少转向）
    goal_bias_deg: float = 0.0       # 目标航向一致性权重（度误差折算距离当量，0=关）
    # --- 速度自适应安全距离（制动距离建模） ---
    stop_base_m: float = 0.25        # 静止时的急停距离
    stop_per_v: float = 0.6          # 每 1 m/s 增加的急停距离（秒当量）
    slow_base_m: float = 0.8         # 静止时的限速距离
    slow_per_v: float = 1.2          # 每 1 m/s 增加的限速距离


class VFHPlanner:
    """Geometric gap-finding planner over a 360° sector distance field.

    用法::

        vfh = VFHPlanner()
        dists = lidar.sector_points()          # [(azimuth_deg, distance_m)]
        heading = vfh.best_heading(dists, target_yaw_deg=0.0, speed=0.3)
        stop_d, slow_d = vfh.safety_distances(speed)
    """

    def __init__(self, config: Optional[VFHConfig] = None) -> None:
        self._cfg = config or VFHConfig()
        self._last_gaps: list[Gap] = []
        self._last_dists: list[float] = [self._cfg.max_range_m] * self._cfg.sector_count

    # -- public ---------------------------------------------------------

    @property
    def config(self) -> VFHConfig:
        return self._cfg

    @property
    def last_gaps(self) -> list[Gap]:
        return self._last_gaps

    def safety_distances(self, speed_m_s: float) -> tuple[float, float]:
        """速度自适应的 (stop_distance_m, slow_distance_m)。

        高速时急停/限速距离线性前移，把制动距离纳入模型，避免
        「0.3m 固定急停」在 1 m/s 以上打滑撞障。
        """
        c = self._cfg
        v = max(0.0, abs(speed_m_s))
        stop = c.stop_base_m + c.stop_per_v * v
        slow = c.slow_base_m + c.slow_per_v * v
        # 限速距离必须大于急停距离（有减速过渡带）
        slow = max(slow, stop + 0.3)
        return stop, slow

    def build_histogram(
        self,
        sectors: list[tuple[float, float]],
    ) -> list[float]:
        """把 [(azimuth_deg, distance_m)] 投影成每扇区最近距离。

        无回波扇区视为 ``max_range_m``（开阔）。输入按「最近距离」聚合，
        天然抗噪（同一扇区多帧取最近）。
        """
        cfg = self._cfg
        step = 360.0 / cfg.sector_count
        dists = [cfg.max_range_m] * cfg.sector_count
        for az, d in sectors:
            if d is None or d <= 0:
                continue
            idx = int(az / step) % cfg.sector_count
            if d < dists[idx]:
                dists[idx] = d
        self._last_dists = dists
        return dists

    def find_gaps(self, dists: Optional[list[float]] = None) -> list[Gap]:
        """从距离场提取「连续开放缺口」。

        某扇区距离 ≥ ``clear_distance_m`` 视为开放；相邻开放扇区合并成缺口。
        距离场不是线性排列（0° 与 359° 相邻），故做环形闭合处理。
        """
        cfg = self._cfg
        if dists is None:
            dists = self._last_dists
        n = len(dists)
        if n == 0:
            return []
        step = 360.0 / n
        open_flags = [d >= cfg.clear_distance_m for d in dists]

        gaps: list[Gap] = []
        # 找连续 open 区间；环形（数组首尾相连）
        start: Optional[int] = None
        # 若首尾都 open，先把尾部并到开头（环形）
        runs: list[list[int]] = []
        run: list[int] = []
        for i in range(n):
            if open_flags[i]:
                run.append(i)
            else:
                if run:
                    runs.append(run)
                    run = []
        if run:
            runs.append(run)
        # 环形闭合：首 run 与末 run 相邻（0° 与 360°）则合并
        if len(runs) >= 2 and runs[0][0] == 0 and runs[-1][-1] == n - 1:
            runs[0] = runs[-1] + runs[0]
            runs = runs[:-1]

        for run in runs:
            s, e = run[0], run[-1]
            dvals = [dists[i] for i in run]
            mn = min(dvals)
            mx = max(dvals)
            gaps.append(Gap(
                start_deg=s * step,
                end_deg=(e + 1) * step - step,
                min_distance_m=mn,
                max_distance_m=mx,
            ))
        self._last_gaps = gaps
        return gaps

    def best_heading(
        self,
        sectors: Optional[list[tuple[float, float]]] = None,
        *,
        target_yaw_deg: float = 0.0,
        speed_m_s: float = 0.0,
    ) -> Optional[float]:
        """返回最优前进航向（车体系度，0°=车头）；无可行航向返回 None。

        采用「车宽卷积」：对每个候选航向 θ，计算车身（车宽/2 + 余量）扫过
        的楔形内**最近障碍距离**（远障碍楔形窄、近障碍楔形宽），只有该距离
        ≥ ``clear_distance_m`` 的航向才可行；在可行航向里取
        「开阔度 + 前向加成 − 目标偏角」最优者。因此不会像「只看缺口中心」
        那样对正前方单障碍给出 180° 掉头或贴着障碍 5° 硬挤的错误解。
        """
        cfg = self._cfg
        if sectors is not None:
            dists = self.build_histogram(sectors)
        else:
            dists = self._last_dists
        n = len(dists)
        if n == 0:
            return None
        step = 360.0 / n
        half_width_m = cfg.vehicle_width_m / 2.0 + cfg.safety_margin_m

        best: Optional[float] = None
        best_score = -math.inf
        for i in range(n):
            theta = i * step
            clearance = self._heading_clearance(dists, theta, half_width_m)
            if clearance < cfg.clear_distance_m:
                continue
            score = clearance \
                - abs(_wrap_deg(theta)) * cfg.forward_bias_m / 90.0 \
                - (abs(_wrap_deg(theta - target_yaw_deg)) * cfg.goal_bias_deg / 90.0
                   if cfg.goal_bias_deg > 0.0 else 0.0)
            if score > best_score:
                best_score = score
                best = theta

        # 保留缺口供诊断/上报（find_gaps 语义不变）
        self._last_gaps = self.find_gaps(dists)
        # 统一返回 [-180, 180)（左正右负），与 navigator / steer_away 约定一致
        return _wrap_deg(best) if best is not None else None

    def _heading_clearance(
        self,
        dists: list[float],
        theta_deg: float,
        half_width_m: float,
    ) -> float:
        """最近会撞到车身的障碍距离（车宽楔形卷积），无则 ``max_range_m``。"""
        n = len(dists)
        if n == 0:
            return self._cfg.max_range_m
        step = 360.0 / n
        best = self._cfg.max_range_m
        for i, d in enumerate(dists):
            if d >= best:
                continue
            az = i * step
            ang = abs(_wrap_deg(az - theta_deg))
            half_ang = math.degrees(
                math.asin(min(1.0, half_width_m / max(d, 0.001))))
            if ang <= half_ang:
                best = d
        return best

    def nearest_blocking_distance(
        self,
        dists: Optional[list[float]] = None,
        front_half_deg: float = 45.0,
    ) -> Optional[float]:
        """正前方 ±front_half_deg 内最近障碍距离（供守卫/诊断）。"""
        cfg = self._cfg
        if dists is None:
            dists = self._last_dists
        n = len(dists)
        if n == 0:
            return None
        step = 360.0 / n
        best: Optional[float] = None
        for i, d in enumerate(dists):
            az = i * step
            if abs(_wrap_deg(az)) <= front_half_deg:
                if best is None or d < best:
                    best = d
        return best
