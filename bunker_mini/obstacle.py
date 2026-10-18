"""Obstacle guard — the shared safety net between LiDAR and motion.

任何会驱动底盘的速度指令（云端 `move`、轨迹回放、`task_submit` 回放、
`goto` 导航）都经过同一个 ``ObstacleGuard`` 检查：根据车头前方的雷达
障碍距离，对指令做「限速」或「硬停车」，从而让现有全部运动通路都获得
雷达兜底，即使上层算法出错也不会直接撞上障碍物。

等级（默认值可配置）::

    障碍 < stop_distance (0.3 m)   → 立即停车（blocked）
    stop ≤ 障碍 < slow_distance(0.8 m) → 按比例限速（距离越近越慢）
    障碍 ≥ slow_distance          → 放行

雷达无数据（未连线 / 掉线）时不拦截指令，但会定期打印警告——开发阶段
方便先跑通底盘；甲方现场若要求「雷达失效必须停车」，可设
``require_sensor=True``。
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .lidar import AiryLidar, LidarPoint
from .terrain import DEFAULT_STEP_LIMIT_M, TerrainSectorResult

logger = logging.getLogger(__name__)

_DEG_PER_RAD: float = 180.0 / 3.141592653589793

# BUNKER MINI 2.0 车宽 ≈ 0.36 m（履带外侧），离地间隙 ≈ 0.14 m。
# 履带外扩检测区：车身半宽外侧 ±[0.19, 0.38]，前伸到 0.45 m（D 迁移）。
TRACK_HALF_WIDTH_M: float = 0.19
TRACK_OUTER_X_M: float = 0.38
TRACK_Y_MIN_M: float = 0.05
TRACK_Y_MAX_M: float = 0.45
# 低矮障碍（碎石）检测区：车前近场、高度低于离地间隙范围（D 迁移）
LOW_OBS_X_HALF_M: float = 0.38
LOW_OBS_Y_MAX_M: float = 1.2
LOW_OBS_Z_MIN_M: float = 0.02
LOW_OBS_Z_MAX_M: float = 0.12


def count_low_obstacles(
    points: list[LidarPoint],
    *,
    x_half_m: float = LOW_OBS_X_HALF_M,
    y_max_m: float = LOW_OBS_Y_MAX_M,
    z_min_m: float = LOW_OBS_Z_MIN_M,
    z_max_m: float = LOW_OBS_Z_MAX_M,
) -> tuple[int, Optional[float]]:
    """Count low obstacles (gravel / rocks) in the near front area.

    返回 (计数, 最近距离 m)。低矮障碍指高度低于离地间隙、能碾过但需要
    减速的小碎石——区别于 ``terrain`` 判定的台阶/岩壁（不可通行）。
    点为雷达系车体坐标（x 右 / y 前 / z 上，光心原点）。
    """
    count = 0
    nearest: Optional[float] = None
    for p in points:
        if (
            abs(p.x) <= x_half_m
            and 0.0 < p.y <= y_max_m
            and z_min_m <= p.z <= z_max_m
        ):
            count += 1
            if nearest is None or p.y < nearest:
                nearest = p.y
    return count, nearest


def track_side_clearance(
    points: list[LidarPoint],
    *,
    half_width_m: float = TRACK_HALF_WIDTH_M,
    outer_x_m: float = TRACK_OUTER_X_M,
    y_min_m: float = TRACK_Y_MIN_M,
    y_max_m: float = TRACK_Y_MAX_M,
) -> tuple[Optional[float], Optional[float]]:
    """Nearest obstacle distance along each track side (left, right).

    返回 ``(left_m, right_m)``：车头左前 / 右前履带外侧区域最近的障碍
    纵向距离（None = 该侧无点）。距离越小越贴墙，巡游/导航应避免
    一侧持续收窄（防止履带刮蹭）。
    """
    left: Optional[float] = None
    right: Optional[float] = None
    for p in points:
        if not (y_min_m <= p.y <= y_max_m):
            continue
        ax = abs(p.x)
        if not (half_width_m <= ax <= outer_x_m):
            continue
        if p.x > 0 and (right is None or p.y < right):
            right = p.y
        elif p.x < 0 and (left is None or p.y < left):
            left = p.y
    return left, right


@dataclass
class ObstaclePolicy:
    """Tuning parameters for the obstacle guard."""

    stop_distance_m: float = 0.3      # 小于该距离 → 停车
    slow_distance_m: float = 0.8      # 小于该距离 → 限速
    slow_speed_factor: float = 0.25   # 贴边时速度降至该比例（0~1）
    fov_deg: float = 60.0             # 前方探测张角（车头方向 ± fov/2）
    obstacle_event_cooldown_s: float = 5.0  # 同一障碍事件去重窗口
    max_steer_offset_deg: float = 45.0      # 转向时前方探测方向的偏置上限

    # --- stage-2 rugged-terrain (月球溶洞等凹凸路况) 参数 ---
    step_limit_m: float = DEFAULT_STEP_LIMIT_M      # 允许的台阶/坑深度，超出视为不可通行
    slope_slow_factor: float = 0.5                  # 坡面上限速比例（不急停）
    terrain_lookahead_m: float = 1.0                # 通过性提前减速的探测距离
    stop_confirm_frames: int = 3                    # 急停需连续 N 帧确认（抗单帧毛刺）

    def __post_init__(self) -> None:
        if self.slow_distance_m <= self.stop_distance_m:
            raise ValueError("slow_distance_m must be > stop_distance_m")
        self.slow_speed_factor = max(0.0, min(1.0, self.slow_speed_factor))
        self.fov_deg = max(5.0, min(360.0, self.fov_deg))
        self.slope_slow_factor = max(0.0, min(1.0, self.slope_slow_factor))
        self.stop_confirm_frames = max(1, int(self.stop_confirm_frames))
        if self.step_limit_m <= 0:
            raise ValueError("step_limit_m must be > 0")


class ObstacleGuard:
    """Velocity modifier that enforces the obstacle policy.

    Call :meth:`guard_velocity` immediately before sending any velocity
    command to the chassis.  Register :meth:`on_blocked` callbacks to react
    to hard stops (e.g. push a ``obstacle`` event to the cloud).
    """

    def __init__(
        self,
        lidar: Optional[AiryLidar],
        policy: Optional[ObstaclePolicy] = None,
        *,
        require_sensor: bool = False,
    ) -> None:
        self._lidar = lidar
        self._policy = policy or ObstaclePolicy()
        self._require_sensor = require_sensor
        self._blocked_callbacks: list[Callable[[float, str], None]] = []
        self._last_blocked_at: float = 0.0
        self._last_blocked_reason: str = ""
        self._last_no_sensor_warn: float = 0.0
        # 多帧确认：连续多少帧都判定“急停”才真正停车，抗碎石/扬尘毛刺
        self._stop_confirm_count: int = 0

    # -- observation -----------------------------------------------------

    def on_blocked(self, callback: Callable[[float, str], None]) -> None:
        """Register a callback invoked on a hard stop with (distance_m, reason)."""
        self._blocked_callbacks.append(callback)

    @property
    def last_blocked_reason(self) -> str:
        return self._last_blocked_reason

    @property
    def sensor_available(self) -> bool:
        return self._lidar is not None and self._lidar.is_receiving

    def forward_distance(self, v: float, w: float) -> Optional[float]:
        """Nearest obstacle in the direction the chassis is heading (m)."""
        if self._lidar is None:
            return None
        fwd = self._heading_deg(v, w)
        return self._lidar.nearest_in_range(fwd, self._policy.fov_deg)

    # -- core ------------------------------------------------------------

    def _terrain_at(self, fwd_deg: float) -> TerrainSectorResult:
        """Terrain verdict for the front sector; default clear when the lidar
        does not expose terrain profiling (backward compatible)."""
        get = getattr(self._lidar, "terrain_sector", None)
        if get is None:
            return TerrainSectorResult(angle_deg=fwd_deg)
        try:
            return get(fwd_deg, self._policy.step_limit_m)
        except TypeError:
            return TerrainSectorResult(angle_deg=fwd_deg)

    @property
    def _has_terrain(self) -> bool:
        return getattr(self._lidar, "terrain_sector", None) is not None

    def guard_velocity(self, v: float, w: float) -> tuple[float, float, bool]:
        """Return the (v, w) that is safe to send, plus whether it was blocked.

        ``blocked=True`` means the command was replaced by a full stop
        (0, 0).  Layered rules (stage-2 rugged terrain):

          1. 常规近距离障碍（p25 抗噪距离 < stop）→ 若确认为真障碍
             （凸起高度 ≥ step_limit）→ 多帧确认后急停；若只是低矮
             碎石（高度差小）→ 只限速。无地形感知的雷达按旧行为直接
             急停（保守）。
          2. 前方地形不可通行（台阶/岩壁/坑，高度突变 > step_limit）
             → 提前减速，进入 stop 距离则急停。
          3. 前方是连续坡面 → 只限速（slope_slow_factor），不当墙急停。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            self._warn_no_sensor()
            if self._require_sensor:
                # fail-safe：雷达本应在线却掉线（网线松动/雷达重启/端口被占），
                # 立即强制停车并通知云端，绝不在无保护下继续行驶。
                self._notify_blocked(0.0, "雷达掉线，避障保护失效，底盘强制停车")
                return 0.0, 0.0, True
            return v, w, False

        policy = self._policy
        fwd = self._heading_deg(v, w)
        d = self._lidar.nearest_in_range(fwd, policy.fov_deg)
        terrain = self._terrain_at(fwd)

        # 先判断“是否需要急停”——独立于 d，直接看地形不可通行
        need_stop = False
        if terrain.blocked:
            # 台阶/岩壁/坑：最近不可通行距离进入急停区
            need_stop = terrain.obstacle_distance_m is not None \
                and terrain.obstacle_distance_m <= policy.stop_distance_m
        if d is not None and d <= policy.stop_distance_m:
            # 常规近距离障碍：有地形感知时只有“真障碍”（凸起够高）才急停；
            # 低矮碎石（高度差 < step_limit）只限速。无感知 → 保守急停。
            need_stop = (not self._has_terrain) or terrain.max_height_m >= policy.step_limit_m

        if need_stop:
            self._stop_confirm_count += 1
            if self._stop_confirm_count >= policy.stop_confirm_frames:
                self._stop_confirm_count = 0
                reason = self._stop_reason(d, terrain)
                self._notify_blocked(d, reason)
                return 0.0, 0.0, True
            # 确认期间：慢速接近，不给全速
            return v * policy.slow_speed_factor, w * policy.slow_speed_factor, False
        self._stop_confirm_count = 0

        # 不可通行地形：在进入急停区之前就提前减速（terrain_lookahead）
        if terrain.blocked and terrain.obstacle_distance_m is not None \
                and terrain.obstacle_distance_m <= policy.terrain_lookahead_m:
            factor = policy.slope_slow_factor
            logger.info(
                "Terrain blocked at %.2f m ahead — slowing (step limit %.0f cm)",
                terrain.obstacle_distance_m, policy.step_limit_m * 100.0,
            )
            return v * factor, w * factor, False

        # 坡面：连续爬升 → 只限速不急停
        if terrain.is_slope and d is not None and d <= policy.slow_distance_m:
            logger.info("Slope ahead (grade %.2f) — slowing", terrain.slope_grade)
            return v * policy.slope_slow_factor, w * policy.slope_slow_factor, False

        # 常规距离限速
        if d is None:
            # 履带侧向贴墙保护（D 迁移）：前方无障碍但侧向极贴墙 → 慢行
            if v > 0 and self.track_scrape_risk():
                return v * policy.slow_speed_factor, w * policy.slow_speed_factor, False
            return v, w, False
        if d <= policy.stop_distance_m:
            # 低矮碎石贴边：保守慢行，不当作不可通行急停
            factor = policy.slow_speed_factor
            return v * factor, w * factor, False
        if d <= policy.slow_distance_m:
            span = policy.slow_distance_m - policy.stop_distance_m
            t = max(0.0, min(1.0, (d - policy.stop_distance_m) / span))
            factor = policy.slow_speed_factor + (1.0 - policy.slow_speed_factor) * t
            return v * factor, w * factor, False
        # 履带侧向贴墙保护（D 迁移）：前方尚可但侧向很贴 → 慢行
        if v > 0 and self.track_scrape_risk():
            return v * policy.slow_speed_factor, w * policy.slow_speed_factor, False
        return v, w, False

    def _stop_reason(self, d: Optional[float], terrain) -> str:
        if terrain.blocked and terrain.obstacle_distance_m is not None:
            return (
                f"前方地形不可通行：台阶/障碍 {terrain.obstacle_distance_m:.2f} m "
                f"(高 {terrain.max_height_m:.2f} m > 限 {self._policy.step_limit_m:.2f} m)"
            )
        return f"障碍 {d:.2f} m 距离过近，急停" if d is not None else "前方地形不可通行，急停"

    def front_terrain_summary(self) -> dict:
        """Cloud-friendly summary of the front sector (for state reports)."""
        if self._lidar is None or not self._lidar.is_receiving:
            return {"online": False}
        terrain = self._terrain_at(0.0)
        return {
            "online": True,
            "blocked": terrain.blocked,
            "obstacleDistance": (
                round(terrain.obstacle_distance_m, 3)
                if terrain.obstacle_distance_m is not None else None
            ),
            "slope": terrain.is_slope,
            "maxHeight": round(terrain.max_height_m, 3),
            "unclear": terrain.unclear,
        }

    def front_blocked_lookahead(self, lookahead_m: float) -> Optional[float]:
        """Distance (m) of a non-traversable obstacle ahead, if within lookahead.

        Returns the nearest blocked-terrain distance when the front sector
        contains a step/rock/pit closer than ``lookahead_m``, else None.
        Used by the navigator to steer around *before* the guard hard-stops.
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return None
        t = self._terrain_at(0.0)
        if t.blocked and t.obstacle_distance_m is not None \
                and t.obstacle_distance_m <= lookahead_m:
            return t.obstacle_distance_m
        return None

    # -- D 迁移：履带外扩 + 低矮障碍（碎石）-----------------------------

    def track_side_clearance(self) -> tuple[Optional[float], Optional[float]]:
        """Nearest obstacle distance along each track side ``(left_m, right_m)``.

        无点云 / 雷达不可用时返回 ``(None, None)``（不拦截）。供巡游与
        导航判断侧向是否持续收窄（防履带刮蹭）。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return None, None
        frame = getattr(self._lidar, "latest_frame", None)
        if frame is None:
            return None, None
        return track_side_clearance(frame.points)

    def track_scrape_risk(self, gap_m: float = 0.10) -> bool:
        """True when either track side is very close to an obstacle."""
        left, right = self.track_side_clearance()
        return (left is not None and left < gap_m) or (
            right is not None and right < gap_m)

    def front_low_obstacles(self) -> tuple[int, Optional[float]]:
        """Low-obstacle (gravel/rock) count in the near front, plus nearest m."""
        if self._lidar is None or not self._lidar.is_receiving:
            return 0, None
        frame = getattr(self._lidar, "latest_frame", None)
        if frame is None:
            return 0, None
        return count_low_obstacles(frame.points)

    def steer_away_deg(self) -> float:
        """Suggested heading offset to steer around a frontal obstruction.

        返回角度（度，左正），供导航器叠加到目标航向上。优先选择
        通过性判定“可通行”的一侧；两侧都不可通行则比较哪边距离更远。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return 0.0
        p = self._policy
        left_t = self._terrain_at(60.0)
        right_t = self._terrain_at(-60.0)
        left_blocked = left_t.blocked or left_t.unclear
        right_blocked = right_t.blocked or right_t.unclear
        if left_blocked and not right_blocked:
            return -p.max_steer_offset_deg
        if right_blocked and not left_blocked:
            return p.max_steer_offset_deg
        left_d = self._lidar.nearest_in_range(60.0, 120.0)
        right_d = self._lidar.nearest_in_range(-60.0, 120.0)
        if left_d is None and right_d is None:
            return 0.0
        if left_d is None:
            return p.max_steer_offset_deg
        if right_d is None:
            return -p.max_steer_offset_deg
        if left_d > right_d:
            return p.max_steer_offset_deg
        return -p.max_steer_offset_deg

    # -- internal --------------------------------------------------------

    def _heading_deg(self, v: float, w: float) -> float:
        """Approximate heading of the chassis given (v, w).

        车头方向 0°；前进 0°、后退 180°；转向时探测方向随 w 偏转
        （角速度 rad/s × 半比例，限制在 ±max_steer_offset_deg）。
        """
        fwd = 0.0 if v >= 0 else 180.0
        offset = w * _DEG_PER_RAD * 0.5
        offset = max(-self._policy.max_steer_offset_deg,
                     min(self._policy.max_steer_offset_deg, offset))
        return (fwd + offset) % 360.0

    def _notify_blocked(self, distance_m: float, reason: str) -> None:
        now = time.time()
        self._last_blocked_reason = reason
        # 去重：同样的硬停车事件不反复推送
        if now - self._last_blocked_at >= self._policy.obstacle_event_cooldown_s:
            self._last_blocked_at = now
            for cb in self._blocked_callbacks:
                try:
                    cb(distance_m, reason)
                except Exception:
                    logger.exception("Obstacle blocked callback error")
        logger.warning("Obstacle guard: %s", reason)

    def _warn_no_sensor(self) -> None:
        now = time.time()
        if now - self._last_no_sensor_warn >= 10.0:
            self._last_no_sensor_warn = now
            if self._require_sensor:
                logger.warning("雷达无数据且 require_sensor=True — 底盘强制停车")
            else:
                logger.warning(
                    "雷达无数据（未连线或掉线）— 避障保护未生效，底盘按原指令行驶"
                )
