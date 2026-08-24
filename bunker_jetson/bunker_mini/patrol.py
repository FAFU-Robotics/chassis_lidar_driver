"""Autonomous recon patrol — "自主探路" (stage-3 finale).

最终任务场景中路线未知、不能人工录制轨迹，小车必须「临场发挥」：
从起点出发，在没有目标坐标的情况下自主决定往哪走，边走边找目标，
找到后去抓取，最后沿探路轨迹原路返回起点。

本模块提供探路运动控制器 :class:`PatrolController`——它只负责
「往开阔且可通行的方向低速前进」，不关心目标检测（由 agent 在巡游
循环里每帧跑 ``ReflectivityDetector``）也不关心轨迹录制（由 agent
用 ``TrackRecorder`` 自动记录）。这样各司其职、便于测试与复用。

巡游决策（每个控制周期调用一次 :meth:`update`）：

  1. 采样多个候选方向（正前方优先），用 LiDAR 累积扇区图查每个方向的
     可通行距离，剔除地形判定「不可通行 / 黑洞」的方向；
  2. **frontier 优先**（有未探索边界目标 → 朝它走）；
  3. **贴壁跟随**（溶洞走廊系统性覆盖：沿固定一侧壁保持偏移，不漏岔路）；
  4. 回退「距离最远 + 前方加权」的开阔方向盲探（含正后方，带惩罚——
     死胡同可转身探索来路分支）；
  5. 航向误差 → 角速度 PID，前方越近线速度越低；交给 ``ObstacleGuard``
     兜底（限速/急停/坡面）；
  6. 所有方向都被堵 → 原地转向重试 → 倒车脱困 → 仍无路则**强制 180°
     调头**换方向系统性探索（不再原地转圈到超时）。

真实环境注意：
  * 巡游是"盲探"，速度必须保守（默认 0.25 m/s），靠多帧累积扇区图
    抗噪（碎石/扬尘）——探测的是 p25 低分位距离，单帧噪点不误导。
  * 贴壁跟随在开阔地形（近侧无壁）时自动退化为开阔方向盲探。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .controller import BunkerMiniController
from .obstacle import ObstacleGuard

logger = logging.getLogger(__name__)

_DEG_PER_RAD: float = 180.0 / math.pi
_RAD_PER_DEG: float = math.pi / 180.0


class PatrolState(Enum):
    MOVING = "moving"      # 正常巡游
    TURNING = "turning"    # 原地转向开阔方向
    BACKING = "backing"    # 倒车脱困
    STUCK = "stuck"        # 长时间无法前进


@dataclass
class PatrolConfig:
    """Tuning parameters for autonomous recon patrol."""

    max_linear_m_s: float = 0.25       # 巡游线速度上限（保守低速）
    max_angular_rad_s: float = 0.60    # 巡游角速度上限
    clear_forward_m: float = 0.8       # 前方距离 ≥ 此值 → 全速直行
    min_clear_m: float = 0.35          # 方向可通行所需的最小距离
    probe_width_deg: float = 60.0      # 每个候选方向探测的扇区宽度
    forward_bonus_m: float = 0.3       # 正前方方向的距离加成（减少频繁转向）
    steer_bias_deg: float = 0.02       # 偏离正前方的惩罚（度/米折算）
    heading_deadband_deg: float = 10.0 # 航向误差死区
    heading_gain: float = 1.5          # 航向误差 → 角速度
    update_interval_s: float = 0.02    # 控制周期：与底盘 0x111 的 20 ms / 50 Hz 对齐
    turn_timeout_s: float = 5.0        # 原地转向超过该时长仍找不到路 → STUCK
    backup_speed_m_s: float = 0.12     # 倒车速度
    backup_duration_s: float = 0.8     # 倒车时长
    max_range_m: float = 3.0           # 探测距离上限（与检测范围一致）

    # --- 探索记忆（避免盲探原地打转） ---
    explore_boost_m: float = 0.8       # 该方向前方「从未访问」→ 距离加分（米当量）
    explore_penalty_m: float = 1.5     # 「近期刚访问」→ 距离减分（米当量）
    explore_recent_s: float = 20.0     # 判定「近期」的时间窗
    explore_lookahead_m: float = 1.5   # 评估方向探索度时向前看的距离
    explore_mark_m: float = 1.5        # 每帧标记为「已访问」的前方扫描范围

    # --- 贴壁跟随（溶洞走廊系统性覆盖的关键） ---
    # 溶洞是走廊网络：随机挑「最开阔方向」会来回横跳、漏掉岔路。贴壁跟随
    # （保持固定侧向偏移沿壁走）能系统性覆盖整个走廊网络，不会漏分支。
    wall_follow_enabled: bool = True
    wall_follow_side: int = 1          # 1=左壁 / -1=右壁（沿一侧走保证覆盖不重不漏）
    wall_follow_offset_m: float = 0.6  # 期望的壁侧向偏移
    wall_follow_range_m: float = 1.5   # 侧向壁最近距离 ≤ 此值才激活贴壁
    wall_follow_max_steer_deg: float = 30.0   # 贴壁最大转向角
    wall_follow_gain_deg_per_m: float = 45.0  # 偏移误差 → 转向角增益

    # --- 死胡同系统性转向 ---
    rear_penalty_m: float = 0.9        # 正后方方向的额外减分（仅在堵死时才选）
    u_turn_duration_s: float = 5.5     # 原地转/倒车超时后强制 180° 调头的时长


class PatrolController:
    """Drive the chassis toward the most open traversable heading.

    Usage::

        patrol = PatrolController(controller, guard, lidar)
        patrol.start()
        while running and not stop.is_set():
            v, w = patrol.update()
            controller.set_velocity(v, w)
            time.sleep(cfg.update_interval_s)
        patrol.stop()
    """

    def __init__(
        self,
        controller: BunkerMiniController,
        guard: Optional[ObstacleGuard],
        lidar,
        *,
        config: Optional[PatrolConfig] = None,
        pose_fn=None,
        explore_grid=None,
        frontier_provider=None,
    ) -> None:
        self._ctrl = controller
        self._guard = guard
        self._lidar = lidar
        self._config = config or PatrolConfig()
        self._state = PatrolState.MOVING
        self._turn_since: Optional[float] = None
        self._backup_until: float = 0.0
        self._heading_deg: float = 0.0
        self._stopped = False
        self._lock = threading.Lock()
        # 探索记忆：pose_fn 提供里程系位姿（navigator.pose），explore_grid
        # 是 OccupancyGrid（world_to_cell 接口）。二者都提供时才启用记忆，
        # 否则退化为纯「往最开阔方向走」的盲探。
        self._pose_fn = pose_fn
        self._explore_grid = explore_grid
        # frontier 探索：frontier_provider 返回世界系「未知边界」目标点
        # (x, y) 或 None。建图完成后替换为基于 SLAM 地图的 frontier 提取，
        # 把盲探升级成主动朝未探索区域覆盖；None 时退化为开阔方向盲探。
        self._frontier_provider = frontier_provider
        self._visited: dict[tuple[int, int], float] = {}
        # 死胡同恢复状态：倒车脱困 → 仍无路则强制 180° 调头
        self._backed_up = False
        self._u_turn_until: Optional[float] = None

    # -- observation -----------------------------------------------------

    @property
    def config(self) -> PatrolConfig:
        return self._config

    @property
    def state(self) -> PatrolState:
        return self._state

    @property
    def heading_deg(self) -> float:
        """Current selected patrol heading (vehicle frame, left positive)."""
        return self._heading_deg

    @property
    def is_stopped(self) -> bool:
        return self._stopped

    # -- control ---------------------------------------------------------

    def start(self) -> None:
        self._stopped = False
        self._state = PatrolState.MOVING
        self._turn_since = None
        self._heading_deg = 0.0
        self._visited.clear()
        self._backed_up = False
        self._u_turn_until = None

    def stop(self) -> None:
        self._stopped = True
        self._ctrl.stop_motion()

    def update(self) -> tuple[float, float]:
        """One patrol control tick → (v, w)."""
        cfg = self._config
        if self._stopped:
            return 0.0, 0.0

        # 探索记忆：把当前位置 + 前方扫描范围标记为「已访问」
        self._mark_visited()

        # 前方不可通行 → 不等急停，提前换方向
        front_blocked = False
        if self._guard is not None:
            blk = self._guard.front_blocked_lookahead(cfg.min_clear_m)
            front_blocked = blk is not None

        # frontier 探索优先：有未知边界目标时朝它走；否则贴壁跟随；
        # 都不可用再回退开阔方向盲探
        heading = self._frontier_heading_target()
        if heading is None:
            heading = self._wall_follow_heading()
        if heading is None:
            heading = self._choose_heading()

        if heading is None or front_blocked:
            # 所有候选方向都被堵：原地左转重试（避免右转）；转向超时 →
            # 先倒车脱困 → 仍无路则强制 180° 调头（换方向系统性探索）
            self._state = PatrolState.TURNING
            now = time.monotonic()
            if self._turn_since is None:
                self._turn_since = now
            if now - self._turn_since > cfg.turn_timeout_s:
                # 阶段 1：倒车脱困（超时后第一次进入才倒）
                if not self._backed_up:
                    self._backed_up = True
                    self._backup_until = now + cfg.backup_duration_s
                    self._state = PatrolState.BACKING
                    return -cfg.backup_speed_m_s, 0.0
                if now < self._backup_until:
                    self._state = PatrolState.BACKING
                    return -cfg.backup_speed_m_s, 0.0
                # 阶段 2：倒车结束仍无路 → 强制 180° 调头
                if self._u_turn_until is None:
                    self._u_turn_until = now + cfg.u_turn_duration_s
                    logger.info("patrol: 死胡同，强制 180° 调头换方向探索")
                self._state = PatrolState.TURNING
                if now < self._u_turn_until:
                    return 0.0, cfg.max_angular_rad_s
                self._u_turn_until = None
                self._turn_since = None
                self._backed_up = False
                return 0.0, 0.0
            return 0.0, cfg.max_angular_rad_s

        self._heading_deg = heading
        self._turn_since = None
        self._u_turn_until = None
        self._backed_up = False
        self._state = PatrolState.MOVING

        # 前方可通行距离 → 线速度（越近越慢；近距离时低速蠕动配合转向，
        # 避免松软地面原地转向打滑）
        fwd_dist = self._clearance(0.0)
        if fwd_dist is None:
            v = cfg.max_linear_m_s
        elif fwd_dist >= cfg.min_clear_m:
            v = cfg.max_linear_m_s * min(
                1.0, max(0.0, fwd_dist - cfg.min_clear_m) / cfg.clear_forward_m)
        else:
            v = cfg.max_linear_m_s * 0.2

        # 航向误差 → 角速度
        err = _wrap_deg(heading)
        if abs(err) <= cfg.heading_deadband_deg:
            w = 0.0
        else:
            w = max(-cfg.max_angular_rad_s,
                    min(cfg.max_angular_rad_s, err * _RAD_PER_DEG * cfg.heading_gain))

        # 避障守卫兜底（急停/限速/坡面）
        if self._guard is not None:
            v, w, blocked = self._guard.guard_velocity(v, w)
            if blocked:
                self._state = PatrolState.TURNING
                self._turn_since = time.monotonic()
        return v, w

    # -- helpers ---------------------------------------------------------

    def _frontier_heading_target(self) -> Optional[float]:
        """frontier 探索优先航向：世界系未知边界目标 → 车体系方位（deg）。

        frontier 目标存在且该方向可通行 → 返回方位；否则返回 None（回退
        到 :meth:`_choose_heading` 的开阔方向盲探）。依赖 pose_fn 做坐标
        换算；未接 frontier_provider / pose_fn 时直接返回 None。
        """
        if self._frontier_provider is None or self._pose_fn is None:
            return None
        try:
            res = self._frontier_provider()
        except Exception:
            logger.debug("frontier_provider failed", exc_info=True)
            return None
        if not res:
            return None
        try:
            fx, fy = res
            pose = self._pose_fn()
        except Exception:
            return None
        world_bearing = math.degrees(math.atan2(fy - pose.y, fx - pose.x))
        rel = (world_bearing - math.degrees(pose.yaw) + 180.0) % 360.0 - 180.0
        if not self._heading_is_open(rel):
            return None
        return rel

    def _wall_follow_heading(self) -> Optional[float]:
        """贴壁跟随航向：沿固定一侧壁保持偏移，系统性覆盖溶洞走廊。

        取跟随侧 ±[30°, 90°] 方位区间内的最近壁距离：
          * 壁比期望偏移近 → 向右（远离壁）转向
          * 壁比期望偏移远 → 向左（靠近壁）转向
          * 恰好 → 直行（沿壁平行）
        返回车体系目标航向（度）；无壁 / 未启用 / 前方被堵时返回 None
        （调用方回退 :meth:`_choose_heading`）。
        """
        cfg = self._config
        if not cfg.wall_follow_enabled or self._lidar is None \
                or not self._lidar.is_receiving:
            return None
        side = 1 if cfg.wall_follow_side >= 0 else -1
        # 跟随侧最近壁距离：采样 30°~90°（左壁）或 -90°~-30°（右壁）
        best_d: Optional[float] = None
        best_az = 0.0
        for off in range(30, 91, 10):
            az = side * off
            d = self._clearance(az)
            if d is not None and (best_d is None or d < best_d):
                best_d, best_az = d, az
        if best_d is None or best_d > cfg.wall_follow_range_m:
            return None
        # 偏移误差 → 转向角（太近往右、太远往左）
        err = best_d - cfg.wall_follow_offset_m
        steer = max(-cfg.wall_follow_max_steer_deg,
                    min(cfg.wall_follow_max_steer_deg,
                        err * cfg.wall_follow_gain_deg_per_m))
        # 前方必须可通行才贴壁（否则由守卫/转向分支处理）
        if self._front_is_open(steer):
            return steer
        return None

    def _front_is_open(self, heading_deg: float) -> bool:
        """朝向是否可通行（前方无地形阻挡 + 有足够距离）。"""
        if self._lidar is None:
            return True
        try:
            t = self._lidar.terrain_sector(heading_deg)
            if t is not None and (t.blocked or t.unclear):
                return False
        except AttributeError:
            pass
        d = self._clearance(heading_deg)
        if d is None:
            return True
        return d >= self._config.min_clear_m

    def _clearance(self, angle_deg: float) -> Optional[float]:
        """Open distance (m) along a vehicle-frame heading, or None if offline."""
        if self._lidar is None or not self._lidar.is_receiving:
            return None
        return self._lidar.nearest_in_range(
            angle_deg, self._config.probe_width_deg)

    def _heading_is_open(self, angle_deg: float) -> bool:
        """True when heading is not blocked by terrain and has open distance."""
        lidar = self._lidar
        if lidar is None:
            return True
        try:
            t = lidar.terrain_sector(angle_deg)
            if t is not None and (t.blocked or t.unclear):
                return False
        except AttributeError:
            pass  # 老接口无地形能力 → 不拦截
        d = self._clearance(angle_deg)
        if d is None:
            return True  # 雷达离线时交由 guard 兜底
        return d >= self._config.min_clear_m

    def _choose_heading(self) -> Optional[float]:
        """Pick the most open traversable heading; None = all directions blocked.

        候选方向：正前方 0° 优先，再按偏角小→大探测左右两侧。**包含正后方
        及斜后方**（带 penalty），使死胡同时小车可以转身去探索来路的分支，
        而不是原地转圈到超时。评分 = 可通行距离 + 前方加成 − 偏角惩罚 +
        探索新奇度。
        """
        cfg = self._config
        candidates: list[float] = []
        for off in (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150):
            candidates.append(float(off))
        # 正后方 / 斜后方：仅在其余全被堵死时才可能胜出（大幅减分）
        for off in (180, -180, 165, -165):
            candidates.append(float(off))

        best: Optional[float] = None
        best_score = -math.inf
        for c in candidates:
            d = self._clearance(c)
            if abs(c) >= 165.0:
                # 正后方：必须「已知开阔」才选（雷达无数据不盲走——那是
                # 死胡同调头，不是往未知区域探险）
                if d is None or d < cfg.min_clear_m:
                    continue
            elif not self._heading_is_open(c):
                continue
            dist = d if d is not None else cfg.max_range_m
            score = dist
            if c == 0.0:
                score += cfg.forward_bonus_m
            else:
                score -= abs(c) * cfg.steer_bias_deg
            if abs(c) >= 165.0:
                score -= cfg.rear_penalty_m
            score += self._explore_score(c, dist)
            if score > best_score:
                best, best_score = c, score
        return best

    def _mark_visited(self) -> None:
        """把当前车格 + 前方扫描范围内的格标记为「已访问」（带时间戳）。

        探索记忆的核心数据：小车真正走过的地方才被记录，回来时这些格
        会使对应方向受罚，引导巡游去新区域。需要 pose_fn + explore_grid
        都可用，否则静默跳过（盲探模式）。
        """
        if self._pose_fn is None or self._explore_grid is None:
            return
        try:
            pose = self._pose_fn()
            grid = self._explore_grid
            now = time.time()
            mark = self._config.explore_mark_m
            cells: list[tuple[int, int]] = [grid.world_to_cell(pose.x, pose.y)]
            # 前方三个距离 × 三个角度，覆盖小车已扫描过的走廊区域
            for ang in (0.0, 30.0, -30.0):
                a = pose.yaw + ang * _RAD_PER_DEG  # pose.yaw 已是弧度
                for d in (mark * 0.5, mark * 0.8, mark):
                    cells.append(grid.world_to_cell(
                        pose.x + d * math.cos(a),
                        pose.y + d * math.sin(a),
                    ))
            for c in cells:
                self._visited[c] = now
        except Exception:
            logger.debug("Patrol visit marking failed", exc_info=True)

    def _explore_score(self, angle_deg: float, dist: float) -> float:
        """该方向前方区域的探索新奇度（米当量）：未探索奖励 / 刚走过惩罚。"""
        if self._pose_fn is None or self._explore_grid is None:
            return 0.0
        cfg = self._config
        try:
            pose = self._pose_fn()
            a = pose.yaw + angle_deg * _RAD_PER_DEG  # pose.yaw 已是弧度
            look = min(dist, cfg.explore_lookahead_m)
            cx, cy = self._explore_grid.world_to_cell(
                pose.x + look * math.cos(a),
                pose.y + look * math.sin(a),
            )
        except Exception:
            return 0.0
        last = self._visited.get((cx, cy))
        if last is None:
            return cfg.explore_boost_m
        age = time.time() - last
        if age < cfg.explore_recent_s:
            return -cfg.explore_penalty_m
        return 0.0

    @property
    def explored_cell_count(self) -> int:
        """已标记为「访问过」的格数量（探索覆盖率诊断）。"""
        return len(self._visited)


def _wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a
