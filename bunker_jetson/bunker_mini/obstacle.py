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
from .vfh import VFHConfig, VFHPlanner

logger = logging.getLogger(__name__)

_DEG_PER_RAD: float = 180.0 / 3.141592653589793

# BUNKER MINI 2.0 车宽 ≈ 0.36 m（履带外侧），离地间隙 ≈ 0.14 m。
# 履带外扩检测区：车身半宽外侧 ±[0.19, 0.38]，前伸到 0.45 m（D 迁移）。
VEHICLE_WIDTH_M: float = 0.36
TRACK_HALF_WIDTH_M: float = 0.19
TRACK_OUTER_X_M: float = 0.38
TRACK_Y_MIN_M: float = 0.05
TRACK_Y_MAX_M: float = 0.45
# 低矮障碍（碎石）检测区：车前近场、高度低于离地间隙范围（D 迁移）
LOW_OBS_X_HALF_M: float = 0.38
LOW_OBS_Y_MAX_M: float = 1.2
LOW_OBS_Z_MIN_M: float = 0.02
LOW_OBS_Z_MAX_M: float = 0.12
# 车前立体盒：人/椅/纸箱。y 从 0.16 起，躲开雷达前方线缆/支架自扫。
# 贴到 0.16 m 以内后立体盒会故意变瞎——近场立柱/椅腿/头顶椅面另见
# near_collision_hits()，否则会钻进办公椅五星腿（现场照片）。
BODY_BOX_Y_MIN_M: float = 0.16
BODY_BOX_Y_MAX_M: float = 2.5
BODY_BOX_X_HALF_M: float = 0.42
BODY_BOX_Z_MIN_M: float = 0.08
BODY_BOX_Z_MAX_M: float = 0.70
BODY_MIN_POINTS: int = 4
BODY_CLOSE_M: float = 0.36
# 近场立柱/椅座：与立体盒对齐（y≥0.16），躲开雷达支架/线缆/电源自扫。
# 贴到 0.16 m 以内靠 furniture_latch（接近时已锁）+ 履带刮蹭，不再用
# y=0.04 的旧盒子——空地自扫会被当成「钻进桌椅」。
NEAR_Y_MIN_M: float = 0.16
NEAR_Y_MAX_M: float = 0.55
NEAR_X_HALF_M: float = 0.30
NEAR_Z_MIN_M: float = 0.14
NEAR_Z_MAX_M: float = 0.50
# 履带旁细椅腿：不在 ±30° 锥里，y 可小于立体盒门槛。
LEG_X_MIN_M: float = 0.16
LEG_X_MAX_M: float = 0.44
LEG_Y_MIN_M: float = 0.04
LEG_Y_MAX_M: float = 0.50
LEG_Z_MIN_M: float = 0.04
LEG_Z_MAX_M: float = 0.85
# 已钻进桌椅 / 低矮钟乳石：只收到「会打到车体」的高度。
# 溶洞高顶（雷达上方 0.8 m+）不得进这里，否则室内椅子补丁会在月面误刹。
CANOPY_X_HALF_M: float = 0.36
CANOPY_Y_MIN_M: float = 0.12
CANOPY_Y_MAX_M: float = 0.40
CANOPY_Z_MIN_M: float = 0.30
CANOPY_Z_MAX_M: float = 0.50
CANOPY_MIN_POINTS: int = 12
CANOPY_X_SPAN_M: float = 0.20
# 雷达光心附近的车体/线缆/电源：空地也会有，不得当椅面或立柱。
# 比旧盒子略放宽：现场空地自扫常落在 y=0.18～0.24，旧门槛会漏进近场急停。
MOUNT_CLUTTER_X_HALF_M: float = 0.26
MOUNT_CLUTTER_Y_MAX_M: float = 0.24
MOUNT_CLUTTER_Z_MAX_M: float = 0.22
# 车体扫过体积：钟乳石/悬空岩。z 从 0.16 起，躲开地面噪声和保险杠回波。
HULL_X_HALF_M: float = 0.20
HULL_Y_MIN_M: float = 0.20
HULL_Y_MAX_M: float = 1.20
HULL_Z_MIN_M: float = 0.16
HULL_Z_MAX_M: float = 0.42
HULL_MIN_POINTS: int = 3
HULL_STALACTITE_Z_M: float = 0.22
NEAR_COLUMN_MIN_POINTS: int = 3
NEAR_COLUMN_CLOSE_M: float = 0.28
NEAR_LEG_MIN_POINTS: int = 3
NEAR_LEG_CLOSE_M: float = 0.18
NEAR_HARD_STOP_M: float = 0.32


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


def _is_mount_clutter(x: float, y: float, z: float) -> bool:
    """雷达前方线缆/支架/电源，空地也会有。"""
    return (
        abs(x) <= MOUNT_CLUTTER_X_HALF_M
        and y < MOUNT_CLUTTER_Y_MAX_M
        and 0.0 <= z <= MOUNT_CLUTTER_Z_MAX_M
    )


def near_collision_hits(
    points: list[LidarPoint],
) -> tuple[Optional[float], str, int]:
    """椅腿/中柱/头顶椅面，立体盒 y≥0.16 之后会漏掉的几何。

    返回 ``(最近纵向距离, 种类, 头顶点数)``。种类：
    ``column`` 车头近处立柱，``leg`` 履带旁细杆，``canopy`` 已在桌椅下。
    车体自扫（线缆/电源/雷达后甲板）不算椅面：旧盒子含 y<0，空地会误刹。
    """
    col_d: Optional[float] = None
    col_n = 0
    leg_d: Optional[float] = None
    leg_n = 0
    canopy_xs: list[float] = []
    canopy_d: Optional[float] = None
    for p in points:
        x = float(getattr(p, "x", 0.0))
        y = float(getattr(p, "y", 0.0))
        z = float(getattr(p, "z", 0.0))
        ax = abs(x)
        clutter = _is_mount_clutter(x, y, z)
        if (not clutter
                and NEAR_Y_MIN_M <= y <= NEAR_Y_MAX_M
                and ax <= NEAR_X_HALF_M
                and NEAR_Z_MIN_M <= z <= NEAR_Z_MAX_M):
            col_n += 1
            if col_d is None or y < col_d:
                col_d = y
        if (not clutter
                and LEG_X_MIN_M <= ax <= LEG_X_MAX_M
                and LEG_Y_MIN_M <= y <= LEG_Y_MAX_M
                and LEG_Z_MIN_M <= z <= LEG_Z_MAX_M):
            leg_n += 1
            if leg_d is None or y < leg_d:
                leg_d = y
        if (not clutter
                and ax <= CANOPY_X_HALF_M
                and CANOPY_Y_MIN_M <= y <= CANOPY_Y_MAX_M
                and CANOPY_Z_MIN_M <= z <= CANOPY_Z_MAX_M):
            canopy_xs.append(x)
            if canopy_d is None or y < canopy_d:
                canopy_d = y
    canopy_n = len(canopy_xs)
    span = (max(canopy_xs) - min(canopy_xs)) if canopy_xs else 0.0
    if canopy_n >= CANOPY_MIN_POINTS and span >= CANOPY_X_SPAN_M:
        return canopy_d if canopy_d is not None else 0.05, "canopy", canopy_n
    # 单点远距回波多半是自扫/扬尘：只有成簇，或已经贴到车头，才当立柱/椅腿。
    if col_d is not None and (
            col_n >= NEAR_COLUMN_MIN_POINTS or col_d <= NEAR_COLUMN_CLOSE_M):
        return col_d, "column", canopy_n
    if leg_d is not None and (
            leg_n >= NEAR_LEG_MIN_POINTS or leg_d <= NEAR_LEG_CLOSE_M):
        return leg_d, "leg", canopy_n
    return None, "", canopy_n


def hull_clearance(points: list[LidarPoint]) -> Optional[float]:
    """车体将要扫过的体积里最近的点（钟乳石 / 悬空岩 / 低梁）。

    比人体立体盒矮：溶洞顶板在雷达上方 0.8 m 不算撞。比近场立柱远：
    1 m 外的低矮悬空物也要进限速，不能等贴到 0.16 m。
    单点低矮回波（地面/保险杠）不算：至少 3 点成簇，或明显悬空的近点。
    """
    best: Optional[float] = None
    best_z = 0.0
    n = 0
    for p in points:
        if abs(float(getattr(p, "x", 0.0))) > HULL_X_HALF_M:
            continue
        y = float(getattr(p, "y", 0.0))
        z = float(getattr(p, "z", 0.0))
        if not (HULL_Y_MIN_M <= y <= HULL_Y_MAX_M):
            continue
        if not (HULL_Z_MIN_M <= z <= HULL_Z_MAX_M):
            continue
        if _is_mount_clutter(float(getattr(p, "x", 0.0)), y, z):
            continue
        n += 1
        if best is None or y < best:
            best = y
            best_z = z
    if best is None:
        return None
    if n >= HULL_MIN_POINTS or (
            best_z >= HULL_STALACTITE_Z_M and best <= 0.50):
        return best
    return None


@dataclass
class ObstaclePolicy:
    """Tuning parameters for the obstacle guard."""

    stop_distance_m: float = 0.3      # 小于该距离 → 停车
    slow_distance_m: float = 0.8      # 小于该距离 → 限速
    slow_speed_factor: float = 0.40   # 贴边时速度降至该比例（0~1）；0.25 空地误限时几乎挪不动
    fov_deg: float = 60.0             # 前方探测张角（车头方向 ± fov/2）
    obstacle_event_cooldown_s: float = 5.0  # 同一障碍事件去重窗口
    max_steer_offset_deg: float = 45.0      # 转向时前方探测方向的偏置上限

    # --- stage-2 rugged-terrain (月球溶洞等凹凸路况) 参数 ---
    step_limit_m: float = DEFAULT_STEP_LIMIT_M      # 允许的台阶/坑深度，超出视为不可通行
    slope_slow_factor: float = 0.5                  # 坡面上限速比例（不急停）
    terrain_lookahead_m: float = 1.0                # 通过性提前减速的探测距离
    stop_confirm_frames: int = 3                    # 急停需连续 N 帧确认（抗单帧毛刺）

    # --- 速度自适应安全距离（制动距离建模） ---
    stop_per_v: float = 0.6        # 每 1 m/s 增加的急停距离（秒当量）
    slow_per_v: float = 1.2        # 每 1 m/s 增加的限速距离
    ema_alpha: float = 0.4         # 前方距离时间滤波系数（0~1，越大越灵敏）

    # --- 急停退出迟滞 ---
    # 障碍在 stop 边界附近抖动（扬尘/碎石/雷达噪声）时，若 need_stop
    # 一帧为 False 就立即放行，会造成「急停→前进→急停→前进」的抖动。
    # 急停后保持停车/慢速至少 stop_hold_s，直到障碍稳定消失才真正放行。
    stop_hold_s: float = 0.5
    # 人/椅：立体盒内近于该距离直接当墙急停（0.10 m/s 时普通 stop 只有 0.36 m，
    # 椅背常在 0.45 m，只会减速然后点云一闪又放行）。
    body_stop_distance_m: float = 0.55
    # 扇区/立体盒单帧丢点时，短时沿用上次距离，避免放行全速。
    range_hold_s: float = 0.40
    # 人/椅一旦进入急停距离，前进方向保持急停这么久。Airy 手册盲区 0.1 m，
    # 钻进椅腿后中柱回波变 0，没有这档就会松闸顶上去。
    furniture_latch_s: float = 0.8
    # 履带外侧近于该间隙按墙急停（原先只减速，照片里履带已经顶上椅腿）。
    track_stop_gap_m: float = 0.12
    # 溶洞口/陨石坑：看不见地面且正前方无回波时不得全速冲（只限速）。
    void_slow: bool = True

    def __post_init__(self) -> None:
        if self.slow_distance_m <= self.stop_distance_m:
            raise ValueError("slow_distance_m must be > stop_distance_m")
        self.slow_speed_factor = max(0.0, min(1.0, self.slow_speed_factor))
        self.fov_deg = max(5.0, min(360.0, self.fov_deg))
        self.slope_slow_factor = max(0.0, min(1.0, self.slope_slow_factor))
        self.stop_confirm_frames = max(1, int(self.stop_confirm_frames))
        self.stop_per_v = max(0.0, self.stop_per_v)
        self.slow_per_v = max(0.0, self.slow_per_v)
        self.ema_alpha = max(0.0, min(1.0, self.ema_alpha))
        self.stop_hold_s = max(0.0, self.stop_hold_s)
        self.body_stop_distance_m = max(self.stop_distance_m, self.body_stop_distance_m)
        self.range_hold_s = max(0.0, self.range_hold_s)
        self.furniture_latch_s = max(0.0, self.furniture_latch_s)
        self.track_stop_gap_m = max(0.02, self.track_stop_gap_m)
        if self.step_limit_m <= 0:
            raise ValueError("step_limit_m must be > 0")

    def effective_distances(self, speed_m_s: float) -> tuple[float, float]:
        """速度自适应的 (stop_distance_m, slow_distance_m)。

        高速时急停/限速距离线性前移，把制动距离纳入模型，避免固定 0.3 m
        急停在 1 m/s 以上打滑撞障。低速时回退到基础值。
        """
        v = max(0.0, abs(speed_m_s))
        stop = self.stop_distance_m + self.stop_per_v * v
        slow = self.slow_distance_m + self.slow_per_v * v
        slow = max(slow, stop + 0.3)  # 保证有减速过渡带
        return stop, slow


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
        # 急停退出迟滞：急停后保持停车/慢速的截止时间（monotonic 秒）
        self._stop_hold_until: float = 0.0
        # 时间滤波状态（EMA）：平滑前方距离与不可通行距离，抑制扬尘抖动
        self._fwd_dist_ema: Optional[float] = None
        self._blocked_dist_ema: Optional[float] = None
        # 同一控制拍内可能先被 patrol/nav 调用、再被 agent._drive 调用。
        # 只对「已经是上次输出」的 (v,w) 短路，避免把已减速指令再按更短
        # 安全距离解掉急停；同一目标速度的连续 tick 仍会累加 stop_confirm。
        self._guard_cache: Optional[tuple[float, bool, float, float]] = None
        self._fwd_held: Optional[float] = None
        self._fwd_hold_until: float = 0.0
        self._body_held: Optional[float] = None
        self._body_hold_until: float = 0.0
        self._near_held: Optional[float] = None
        self._near_hold_until: float = 0.0
        self._furniture_latch_until: float = 0.0
        self._furniture_latch_d: Optional[float] = None
        self._furniture_latch_kind: str = ""
        self._furniture_lock: bool = False
        # VFH 几何缺口转向器（用于 gap_heading / steer_away_deg）
        self._vfh = VFHPlanner(VFHConfig(
            vehicle_width_m=VEHICLE_WIDTH_M,
            clear_distance_m=0.6,
        ))

    def _ema(self, prev: Optional[float], value: Optional[float]) -> Optional[float]:
        """一阶指数平滑（仅对「有障碍距离」做低通）。

        ``value=None`` 表示「该方向明确无回波/无阻塞」，立即清零——绝不
        用衰减制造「幽灵障碍」导致障碍消失后仍持续减速/停车。真障碍距离
        抖动（碎石/扬尘）则被低通平滑。
        """
        if value is None:
            # 单帧丢点不立刻当「前方清空」，否则椅腿/人体会闪一下就放行全速。
            if prev is not None and time.monotonic() < self._fwd_hold_until:
                return prev
            return None
        self._fwd_hold_until = time.monotonic() + self._policy.range_hold_s
        alpha = self._policy.ema_alpha
        if prev is None:
            return value
        return alpha * value + (1.0 - alpha) * prev

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

    def _is_low_gravel(self, terrain: TerrainSectorResult) -> bool:
        """True only when terrain actually measured a short bump.

        Airy 视场偏上，墙面高度经常测不出来（max_height_m=0、unclear）。
        旧逻辑把「max_h < step_limit」一律当碎石，近处岩壁只限速不急停。
        碎石必须是测到的矮凸起：0 < max_h < step_limit，且不是 blocked/unclear。
        """
        if not self._has_terrain:
            return False
        if terrain.unclear or terrain.blocked:
            return False
        return 0.0 < terrain.max_height_m < self._policy.step_limit_m

    def _hold_optional(
        self,
        current: Optional[float],
        held_attr: str,
        until_attr: str,
    ) -> Optional[float]:
        """单帧没有回波时，短时沿用上次距离。"""
        now = time.monotonic()
        if current is not None:
            setattr(self, held_attr, current)
            setattr(self, until_attr, now + self._policy.range_hold_s)
            return current
        held = getattr(self, held_attr)
        if held is not None and now < getattr(self, until_attr):
            return held
        setattr(self, held_attr, None)
        return None

    def _remember_guard(self, hard_stop: bool, out_v: float, out_w: float) -> None:
        self._guard_cache = (time.monotonic(), hard_stop, out_v, out_w)

    def _cached_guard(self, v: float, w: float) -> Optional[tuple[float, float, bool]]:
        cache = self._guard_cache
        if cache is None:
            return None
        ts, hard_stop, out_v, out_w = cache
        if time.monotonic() - ts >= 0.02:
            return None
        if hard_stop:
            if abs(v) <= 1e-6 and abs(w) <= 1e-6:
                return 0.0, 0.0, True
            return None
        # 仅当本拍已经应用过限速、_drive 再次带着输出值进来时短路
        if abs(v - out_v) <= 1e-6 and abs(w - out_w) <= 1e-6:
            return v, w, False
        return None

    def guard_velocity(self, v: float, w: float) -> tuple[float, float, bool]:
        """Return the (v, w) that is safe to send, plus whether it was blocked.

        ``blocked=True`` means the command was replaced by a full stop
        (0, 0).  Layered rules (stage-2 rugged terrain):

          1. 常规近距离障碍（p25 抗噪距离 < stop）→ 若确认为真障碍
             （凸起高度 ≥ step_limit）→ 多帧确认后急停；若只是低矮
             碎石（高度差小）→ 只限速。无地形感知的雷达按旧行为直接
             急停（保守）。
          2. 前方地形不可通行（台阶/岩壁/坑 + 负障碍/悬崖，高度突变 >
             step_limit 或地面下陷）→ 提前减速，进入 stop 距离则急停。
          3. 前方是连续坡面 → 只限速（slope_slow_factor），不当墙急停。
          4. 急停/限速距离随车速自适应（制动距离建模），高速提前减速。
          5. 前方距离与不可通行距离做时间滤波（EMA），抗扬尘单帧毛刺。
        """
        cached = self._cached_guard(v, w)
        if cached is not None:
            return cached

        if self._lidar is None or not self._lidar.is_receiving:
            self._warn_no_sensor()
            if self._require_sensor:
                # 停车（kb 进门、松键后的 move 0 0）不必报「掉线急停」。
                # 云端 WebSocket 连上 ≠ 雷达 UDP 在出点；只有真的加油门才拦截。
                if abs(v) <= 1e-6 and abs(w) <= 1e-6:
                    self._remember_guard(False, 0.0, 0.0)
                    return 0.0, 0.0, False
                self._notify_blocked(0.0, "雷达掉线，避障保护失效，底盘强制停车")
                self._remember_guard(True, 0.0, 0.0)
                return 0.0, 0.0, True
            self._remember_guard(False, v, w)
            return v, w, False

        policy = self._policy
        reversing = v < -1e-6
        if reversing:
            # 倒车离开椅腿：先丢掉前进方向的保持，再按车尾重新测。
            self._furniture_lock = False
            self._furniture_latch_until = 0.0
            self._furniture_latch_d = None
            self._fwd_dist_ema = None
            self._fwd_hold_until = 0.0
            self._body_held = None
            self._near_held = None
        fwd = self._heading_deg(v, w)
        # 速度自适应安全距离：高速时急停/限速距离前移（制动距离建模）
        stop_d, slow_d = policy.effective_distances(v)

        # 前方最近障碍距离（p25 抗噪）+ 时间滤波（EMA）
        # p10 而不是 p25：同一锥里的天花/远墙回波不再把人/椅的稀疏近点冲掉。
        raw_d = self._lidar.nearest_in_range(fwd, policy.fov_deg, quantile=0.10)
        # 转向时额外探测正前方：_heading_deg 会把探测方向朝转向侧偏转（最大
        # ±45°），fov=60°（±30°）会在正前方留下 [0°, offset-30°] 的盲区——
        # 原地/急转时车头扫过的弧线可能撞到正前方近场障碍。取两者更近者。
        if abs(w) > 0.2:
            straight = 0.0 if v >= 0 else 180.0
            straight_d = self._lidar.nearest_in_range(
                straight, policy.fov_deg, quantile=0.10)
            if straight_d is not None and (raw_d is None or straight_d < raw_d):
                raw_d = straight_d
        body_d = self._hold_optional(
            self.body_clearance(v), "_body_held", "_body_hold_until")
        near_d, near_kind = self.near_collision()
        near_d = self._hold_optional(near_d, "_near_held", "_near_hold_until")
        hull_d = None if reversing else self.hull_clearance()
        if body_d is not None and (raw_d is None or body_d < raw_d):
            raw_d = body_d
        if (not reversing and near_d is not None
                and (raw_d is None or near_d < raw_d)):
            raw_d = near_d
        if hull_d is not None and (raw_d is None or hull_d < raw_d):
            raw_d = hull_d
        d = self._ema(self._fwd_dist_ema, raw_d)
        self._fwd_dist_ema = d

        terrain = self._terrain_at(fwd)

        # 不可通行距离（台阶/岩壁/坑 + 负障碍/悬崖）→ EMA 平滑
        blocked_raw: Optional[float] = None
        if terrain.blocked:
            cands = [x for x in (terrain.obstacle_distance_m,
                                 terrain.negative_obstacle_distance_m)
                     if x is not None]
            if cands:
                blocked_raw = min(cands)
        blocked_d = self._ema(self._blocked_dist_ema, blocked_raw)
        self._blocked_dist_ema = blocked_d

        # 先判断“是否需要急停”——不可通行地形进入急停区
        need_stop = False
        if blocked_d is not None and blocked_d <= stop_d:
            need_stop = True
        if d is not None and d <= stop_d and not self._is_low_gravel(terrain):
            need_stop = True
        # 人/椅：0.45 m 椅背高于普通 stop（低速约 0.36 m），必须按墙急停。
        body_stop = max(stop_d, policy.body_stop_distance_m)
        furniture_kind = ""
        if (not reversing and body_d is not None
                and body_d <= body_stop):
            need_stop = True
            furniture_kind = "body"
            self._arm_furniture_latch(body_d, "body")
        # 近场立柱/椅腿：旧逻辑「看见就急停」会把空地单点自扫锁死 2s。
        # 只有已经贴上来，或头顶椅面（钻进去了），才硬停。
        if near_d is not None and not reversing:
            if near_kind == "canopy" or near_d <= max(stop_d, NEAR_HARD_STOP_M):
                need_stop = True
                furniture_kind = near_kind or "column"
                self._arm_furniture_latch(near_d, furniture_kind)
        # hull_clearance 已经滤掉地面/自扫；能到这里的是成簇或明显悬空点。
        if (not reversing and hull_d is not None
                and hull_d <= body_stop):
            need_stop = True
            if not furniture_kind:
                furniture_kind = "hanging"
            self._arm_furniture_latch(hull_d, "hanging")
        track_hit = (not reversing) and self.track_scrape_risk(
            gap_m=policy.track_stop_gap_m)
        if track_hit:
            need_stop = True
            if not furniture_kind:
                furniture_kind = "track"
        now_mono = time.monotonic()
        if not furniture_kind and now_mono >= self._furniture_latch_until:
            self._furniture_lock = False
        latch_hold = (
            v > 0
            and self._furniture_lock
            and now_mono < self._furniture_latch_until
        )
        if latch_hold:
            need_stop = True
            if not furniture_kind:
                furniture_kind = "latch"
                if d is None:
                    d = self._furniture_latch_d

        furniture_now = bool(furniture_kind)

        if need_stop:
            self._stop_confirm_count += 1
            # 已贴上/钻进家具：不要再等 3 帧确认——每帧都在往椅腿里拱。
            confirmed = (
                furniture_now
                or self._stop_confirm_count >= policy.stop_confirm_frames
            )
            if confirmed:
                self._stop_confirm_count = 0
                # 急停后保持停车/慢速 stop_hold_s，防止障碍边界抖动导致
                # 「急停→放行→急停」的往复抖动（扬尘/碎石/雷达噪声场景）
                self._stop_hold_until = time.monotonic() + policy.stop_hold_s
                reason = self._stop_reason(
                    d, terrain, blocked_d, furniture_kind)
                dist = blocked_d if blocked_d is not None else d
                if dist is None:
                    dist = self._furniture_latch_d
                self._notify_blocked(dist if dist is not None else 0.0, reason)
                self._remember_guard(True, 0.0, 0.0)
                return 0.0, 0.0, True
            # 确认期间：慢速接近，不给全速
            factor = policy.slow_speed_factor
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False
        self._stop_confirm_count = 0

        # 急停退出迟滞：刚急停过、障碍暂时消失时保持慢速，确认稳定后放行
        if time.monotonic() < self._stop_hold_until:
            factor = policy.slow_speed_factor
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False

        # 不可通行地形：在进入急停区之前就提前减速（terrain_lookahead）
        if blocked_d is not None and blocked_d <= policy.terrain_lookahead_m:
            logger.info(
                "Terrain blocked at %.2f m ahead — slowing (step limit %.0f cm)",
                blocked_d, policy.step_limit_m * 100.0,
            )
            factor = policy.slope_slow_factor
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False

        # 坡面：连续爬升 → 只限速不急停
        if terrain.is_slope and d is not None and d <= slow_d:
            logger.info("Slope ahead (grade %.2f) — slowing", terrain.slope_grade)
            factor = policy.slope_slow_factor
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False

        # 常规距离限速
        if d is None:
            # 履带侧向贴墙保护（D 迁移）：前方无障碍但侧向极贴墙 → 慢行
            if v > 0 and self.track_scrape_risk():
                factor = policy.slow_speed_factor
                out_v, out_w = v * factor, w * factor
                self._remember_guard(False, out_v, out_w)
                return out_v, out_w, False
            # 溶洞口/陨石坑：看不见地面且正前方无回波，不得按「空地」全速冲。
            if (v > 0 and policy.void_slow and self._has_terrain
                    and terrain.unclear):
                factor = policy.slow_speed_factor
                out_v, out_w = v * factor, w * factor
                self._remember_guard(False, out_v, out_w)
                return out_v, out_w, False
            self._remember_guard(False, v, w)
            return v, w, False
        if d <= stop_d:
            # 低矮碎石贴边：保守慢行，不当作不可通行急停
            factor = policy.slow_speed_factor
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False
        if d <= slow_d:
            span = slow_d - stop_d
            t = max(0.0, min(1.0, (d - stop_d) / span))
            factor = policy.slow_speed_factor + (1.0 - policy.slow_speed_factor) * t
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False
        # 履带侧向贴墙保护（D 迁移）：前方尚可但侧向很贴 → 慢行
        if v > 0 and self.track_scrape_risk():
            factor = policy.slow_speed_factor
            out_v, out_w = v * factor, w * factor
            self._remember_guard(False, out_v, out_w)
            return out_v, out_w, False
        self._remember_guard(False, v, w)
        return v, w, False

    def _arm_furniture_latch(self, distance_m: Optional[float], kind: str) -> None:
        self._furniture_lock = True
        self._furniture_latch_until = time.monotonic() + self._policy.furniture_latch_s
        if distance_m is not None:
            self._furniture_latch_d = distance_m
        self._furniture_latch_kind = kind

    def _stop_reason(
        self,
        d: Optional[float],
        terrain,
        blocked_d: Optional[float],
        furniture_kind: str = "",
    ) -> str:
        if furniture_kind == "hanging":
            d0 = d if d is not None else self._furniture_latch_d
            return (
                f"车体高度内悬空岩/钟乳石 {d0:.2f} m，急停"
                if d0 is not None else "车体高度内悬空岩/钟乳石，急停"
            )
        if furniture_kind == "canopy":
            return "已钻进桌椅下方（椅面在雷达上方），急停"
        if furniture_kind == "leg":
            return "履带外侧碰到椅腿/细杆，急停"
        if furniture_kind == "column":
            d0 = d if d is not None else self._furniture_latch_d
            return (
                f"车头近处立柱/椅腿 {d0:.2f} m，急停"
                if d0 is not None else "车头近处立柱/椅腿，急停"
            )
        if furniture_kind == "track":
            return "履带外侧贴障，急停"
        if furniture_kind == "latch":
            d0 = d if d is not None else self._furniture_latch_d
            extra = f"{d0:.2f} m " if d0 is not None else ""
            return f"近处家具回波刚消失（Airy 0.1 m 盲区），{extra}保持急停"
        if terrain.negative_obstacle_distance_m is not None:
            return (
                f"前方地形下坠/悬崖 {terrain.negative_obstacle_distance_m:.2f} m，"
                "不可通行，急停"
            )
        if terrain.blocked and terrain.obstacle_distance_m is not None:
            return (
                f"前方地形不可通行：台阶/障碍 {terrain.obstacle_distance_m:.2f} m "
                f"(高 {terrain.max_height_m:.2f} m > 限 {self._policy.step_limit_m:.2f} m)"
            )
        if blocked_d is not None:
            return f"前方地形不可通行 {blocked_d:.2f} m，急停"
        if d is not None and d <= self._policy.body_stop_distance_m:
            return f"前方人体/椅背 {d:.2f} m，急停"
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
            "negativeObstacleDistance": (
                round(terrain.negative_obstacle_distance_m, 3)
                if terrain.negative_obstacle_distance_m is not None else None
            ),
            "slope": terrain.is_slope,
            "maxHeight": round(terrain.max_height_m, 3),
            "unclear": terrain.unclear,
        }

    def front_blocked_lookahead(self, lookahead_m: float) -> Optional[float]:
        """Distance (m) of a non-traversable obstacle ahead, if within lookahead.

        台阶/岩壁/下坠（terrain.blocked）以及「近处几何回波但不是矮碎石」
        都算不可通行，供导航在守卫急停前转向。只看地形会漏掉测高失败的墙。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return None
        t = self._terrain_at(0.0)
        cands: list[float] = []
        if t.blocked:
            cands.extend(
                x for x in (t.obstacle_distance_m,
                            t.negative_obstacle_distance_m) if x is not None)
        d = self._lidar.nearest_in_range(
            0.0, self._policy.fov_deg, quantile=0.10)
        if d is not None and (
            not self._has_terrain
            or t.unclear
            or t.blocked
            or t.max_height_m >= self._policy.step_limit_m
        ):
            cands.append(d)
        # 椅背/人体往往只出现在立体盒里；不并进来的话 goto 会直行到急停才绕。
        body = self._hold_optional(
            self.body_clearance(0.2), "_body_held", "_body_hold_until")
        if body is not None:
            cands.append(body)
        near_d, _kind = self.near_collision()
        if near_d is not None:
            cands.append(near_d)
        hull = self.hull_clearance()
        if hull is not None:
            cands.append(hull)
        for x in sorted(cands):
            if x <= lookahead_m:
                return x
        return None

    # -- VFH 几何缺口转向 ------------------------------------------------

    def gap_heading(
        self,
        target_yaw_deg: float = 0.0,
        speed_m_s: float = 0.0,
    ) -> Optional[float]:
        """最优可通行缺口航向（车体系度），不可行返回 None。

        用 VFH 几何缺口（距离场 + 车宽通过性），比 ``steer_away_deg``
        的「左右谁更空」更可靠：只返回车宽 + 余量能挤进去的缺口。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return None
        pts_fn = getattr(self._lidar, "sector_points", None)
        if pts_fn is None:
            return None
        pts = pts_fn()
        if not pts:
            return None
        self._vfh.config.goal_bias_deg = 1.0 if abs(target_yaw_deg) > 0.5 else 0.0
        return self._vfh.best_heading(pts, target_yaw_deg=target_yaw_deg,
                                      speed_m_s=speed_m_s)

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

    def hull_clearance(self) -> Optional[float]:
        """车体扫过体积里最近的悬空点。无点云时返回 None。"""
        if self._lidar is None or not self._lidar.is_receiving:
            return None
        frame = getattr(self._lidar, "latest_frame", None)
        if frame is None:
            return None
        return hull_clearance(getattr(frame, "points", None) or ())

    def near_collision(self) -> tuple[Optional[float], str]:
        """椅腿/中柱/头顶椅面。立体盒在 y<0.16 m 后会漏掉这些点。"""
        if self._lidar is None or not self._lidar.is_receiving:
            return None, ""
        frame = getattr(self._lidar, "latest_frame", None)
        if frame is None:
            return None, ""
        d, kind, _n = near_collision_hits(getattr(frame, "points", None) or ())
        return d, kind

    def body_clearance(self, v: float = 0.0) -> Optional[float]:
        """车前/车后立体盒最近距离（米）。抓住椅背、人体等水平切片经常漏掉的目标。

        盒子比自扫线缆更远（y≥0.16 m），比地面碎石更高（z≥0.08 m），
        比溶洞高顶更低（z≤0.70 m，人/椅躯干带）。倒车看车尾对称盒子。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return None
        frame = getattr(self._lidar, "latest_frame", None)
        if frame is None:
            return None
        pts = getattr(frame, "points", None) or ()
        best: Optional[float] = None
        n = 0
        forward = v >= 0.0
        for p in pts:
            y = float(getattr(p, "y", 0.0))
            if forward:
                if not (BODY_BOX_Y_MIN_M <= y <= BODY_BOX_Y_MAX_M):
                    continue
            elif not (-BODY_BOX_Y_MAX_M <= y <= -BODY_BOX_Y_MIN_M):
                continue
            if abs(float(getattr(p, "x", 0.0))) > BODY_BOX_X_HALF_M:
                continue
            z = float(getattr(p, "z", 0.0))
            if z < BODY_BOX_Z_MIN_M or z > BODY_BOX_Z_MAX_M:
                continue
            if _is_mount_clutter(float(getattr(p, "x", 0.0)), y, z):
                continue
            n += 1
            d = abs(y)
            if best is None or d < best:
                best = d
        if best is None:
            return None
        # 空地单点自扫也会落进这个大盒子；远距必须成簇才当人体/椅背。
        if n >= BODY_MIN_POINTS or best <= BODY_CLOSE_M:
            return best
        return None

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

        优先用 VFH 几何缺口（车宽 + 余量能通过的最优缺口中心航向）；
        找不到可行缺口再退化为左右通过性比较（返回 ±max_steer_offset_deg）。
        返回角度（度，左正），供导航器叠加到目标航向上。
        """
        if self._lidar is None or not self._lidar.is_receiving:
            return 0.0
        p = self._policy
        gap = self.gap_heading(target_yaw_deg=0.0)
        if gap is not None:
            return max(-p.max_steer_offset_deg, min(p.max_steer_offset_deg, gap))
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
            logger.warning("Obstacle guard: %s", reason)
            for cb in self._blocked_callbacks:
                try:
                    cb(distance_m, reason)
                except Exception:
                    logger.exception("Obstacle blocked callback error")

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
