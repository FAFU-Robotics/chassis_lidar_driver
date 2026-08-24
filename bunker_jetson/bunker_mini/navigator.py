"""Point-to-point navigation for BUNKER MINI 2.0 — "目标点自主导航".

不依赖 ROS 的轻量导航：

  * 定位：基于左右轮里程（CAN 0x311）的差速模型航迹推算（dead reckoning）
  * 路径：目标点 (x, y) → 航向角 PID → (v, w) 速度指令
  * 避障：复用 :class:`ObstacleGuard` 做前方限速/停车；正前方被挡时向更开阔
    一侧原地转向，绕开后再朝目标前进

坐标系约定：以「导航原点」为原点 (0, 0)——即 navigator 初始化（agent 启动）
时小车所在位置、车头方向为 yaw=0°（注意：不是底盘物理上电位置；agent 未启动
时小车被搬动，原点按 agent 启动瞬间的位置起算）。逆时针为正（与差速底盘
角速度 w>0 左转一致）。里程轮距 ``wheelbase_m``
需要按底盘实际值配置（BUNKER MINI 2.0 约为 0.5 m，甲方现场需实测标定）。

导航线程每 50 ms 运行一次，直接调用
``BunkerMiniController.set_velocity``，与轨迹回放线程互斥
（agent 保证同一时刻只有一个驱动者在发指令）。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from .controller import BunkerMiniController
from .obstacle import ObstacleGuard

logger = logging.getLogger(__name__)

_DEG_PER_RAD: float = 180.0 / math.pi
_RAD_PER_DEG: float = math.pi / 180.0


@dataclass
class Pose2D:
    """2D pose estimated from odometry (dead reckoning)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # 弧度，逆时针为正

    @property
    def yaw_deg(self) -> float:
        return self.yaw * _DEG_PER_RAD


class OdometryPose:
    """Integrate differential wheel odometry (mm deltas) into a Pose2D.

    差速模型：
        d_center = (d_left + d_right) / 2
        d_yaw    = (d_right - d_left) / wheelbase
    每收到一帧 0x311 里程计就调用一次 :meth:`update`。
    """

    def __init__(self, wheelbase_m: float = 0.5) -> None:
        if wheelbase_m <= 0:
            raise ValueError("wheelbase_m must be > 0")
        self._wheelbase = wheelbase_m
        self._pose = Pose2D()
        self._last_left_mm: Optional[int] = None
        self._last_right_mm: Optional[int] = None

    @property
    def pose(self) -> Pose2D:
        return self._pose

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self._pose = Pose2D(x=x, y=y, yaw=yaw)
        self._last_left_mm = None
        self._last_right_mm = None

    def set_pose(self, x: float, y: float, yaw_deg: float) -> None:
        """Overwrite the pose (external correction — future GPS/vision)."""
        self._pose = Pose2D(x=x, y=y, yaw=yaw_deg * _RAD_PER_DEG)
        self._last_left_mm = None

    def update(self, left_mm: int, right_mm: int) -> None:
        if self._last_left_mm is None:
            self._last_left_mm = left_mm
            self._last_right_mm = right_mm
            return
        dl = (left_mm - self._last_left_mm) / 1000.0
        dr = (right_mm - self._last_right_mm) / 1000.0
        self._last_left_mm = left_mm
        self._last_right_mm = right_mm

        d_center = (dl + dr) / 2.0
        d_yaw = (dr - dl) / self._wheelbase
        # 以弧段中点角度更新位置，减小离散误差
        mid = self._pose.yaw + d_yaw / 2.0
        self._pose.yaw += d_yaw
        self._pose.x += d_center * math.cos(mid)
        self._pose.y += d_center * math.sin(mid)


@dataclass
class NavigateConfig:
    """Tuning parameters for point-to-point navigation."""

    max_linear_m_s: float = 0.30        # 导航线速度上限（保守安全值）
    max_angular_rad_s: float = 0.60     # 导航角速度上限
    goal_tolerance_m: float = 0.08      # 到达目标判定半径（位置，需配合航向对齐）
    heading_gain: float = 1.2           # 航向误差 → 角速度 比例系数
    approach_gain: float = 1.0          # 距离 → 线速度 比例系数
    update_interval_s: float = 0.02     # 控制周期：与底盘 0x111 的 20 ms / 50 Hz 对齐
    stall_timeout_s: float = 5.0        # 全程被障碍挡住超过该时长 → 放弃并回调
    replan_after_s: float = 3.0         # 被堵超过该时长 → 触发全局重规划（replanner）

    # --- 转向优化（大航向误差先原地转正车身，禁止斜着蹭到点） ---
    heading_deadband_rad: float = 0.09  # 航向死区（~5°）：小误差不转向，防末端摆动
    turn_decel_rad: float = 1.05        # 航向误差超该阈值（~60°）→ 压到最低转向速度
    turn_min_speed_ratio: float = 0.05  # 中等转角时的最低线速度比例
    align_in_place_rad: float = 0.40    # ~23°：超过则线速度清零，原地转正再走

    # --- 终点车身姿态：XY 到位后原地转到目标航向 ---
    final_yaw_tolerance_rad: float = 0.09  # ~5°：终点航向容差
    final_align_timeout_s: float = 4.0     # 终点转正超时后接受当前朝向（XY 已到位）
    approach_lock_m: float = 0.25          # 远于该距离时锁定接近航向，避免终点 atan2 抖动

    # --- stage-2 rugged-terrain 参数 ---
    arrive_probe_m: float = 0.16        # 接近目标后启用“停滞判定”的半径（不得在 0.4m 外冒充到达）
    arrive_stall_s: float = 2.0         # 停滞超过该时长 → 中间航点可推进；终点还要航向过关
    arrive_stall_moved_m: float = 0.02  # 停滞期内位移低于该值判定为停滞
    terrain_lookahead_m: float = 1.0    # 前方不可通行地形的提前绕行距离
    backup_speed_m_s: float = 0.12      # 被堵死时倒车速度
    backup_duration_s: float = 0.8      # 倒车时长


class Navigator:
    """Drive the chassis to a goal (x, y) while avoiding obstacles.

    Usage::

        nav = Navigator(controller, guard, wheelbase_m=0.5)
        nav.goto(2.0, 1.5, on_arrived=my_callback)
        ...
        nav.stop()
    """

    def __init__(
        self,
        controller: BunkerMiniController,
        guard: Optional[ObstacleGuard] = None,
        *,
        wheelbase_m: float = 0.5,
        config: Optional[NavigateConfig] = None,
        drive: Optional[Callable[[float, float], None]] = None,
    ) -> None:
        self._ctrl = controller
        self._guard = guard
        self._config = config or NavigateConfig()
        # 底盘速度统一下发口：传入 agent._drive 时，导航也走底盘诊断
        # （STANDBY 自动重使能 + 指令/实测轮速对比）；否则直接 set_velocity。
        self._drive = drive
        self._pose = OdometryPose(wheelbase_m)

        self._lock = threading.Lock()
        self._navigating = False
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._goal: Optional[tuple[float, float]] = None
        self._goal_yaw: Optional[float] = None  # 终点航向（弧度）；None=用接近航向
        self._waypoints: list[tuple[float, float]] = []
        self._goal_speed: Optional[float] = None
        self._on_arrived: Optional[Callable[[], None]] = None
        self._on_abort: Optional[Callable[[str], None]] = None
        # 全局重规划回调：被堵死磕时从当前位置重新规划（agent 注入）
        self._replanner: Optional[Callable[[float, float], Optional[list[tuple[float, float]]]]] = None

    # -- observation -----------------------------------------------------

    @property
    def pose(self) -> Pose2D:
        with self._lock:
            return Pose2D(self._pose.pose.x, self._pose.pose.y, self._pose.pose.yaw)

    @property
    def is_navigating(self) -> bool:
        with self._lock:
            return self._navigating

    @property
    def goal(self) -> Optional[tuple[float, float]]:
        with self._lock:
            return self._goal

    def feed_odometry(self, left_mm: int, right_mm: int) -> None:
        """Call with every fresh 0x311 frame — keeps the pose estimate live."""
        with self._lock:
            self._pose.update(left_mm, right_mm)

    def reset_pose(self) -> None:
        with self._lock:
            self._pose.reset()

    def apply_external_pose(self, x: float, y: float, yaw_deg: float) -> None:
        """用外部绝对位姿（SLAM / 视觉定位）覆盖航迹推算结果。

        建图完成后由定位 tick 周期调用：覆盖后 ``OdometryPose`` 重置轮速
        基准，下一帧里程继续增量累加。这是根治长时间漂移的入口。
        """
        with self._lock:
            self._pose.set_pose(x, y, yaw_deg)

    # -- control ---------------------------------------------------------

    def goto(
        self,
        x: float,
        y: float,
        *,
        on_arrived: Optional[Callable[[], None]] = None,
        on_abort: Optional[Callable[[str], None]] = None,
        speed: Optional[float] = None,
        waypoints: Optional[list[tuple[float, float]]] = None,
        replanner: Optional[Callable[[float, float], Optional[list[tuple[float, float]]]]] = None,
        goal_yaw: Optional[float] = None,
    ) -> bool:
        """Start navigating to (x, y), optionally along pre-planned waypoints.

        ``waypoints``  全局路径规划输出的中间航点序列（不含终点），按序
        逐个到达后最终抵达 (x, y)。用于把「A* 规划出的绕障折线」拆成可
        执行的航段，避免直线 goto 直穿障碍。为 ``None`` 时等价于原直线导航。
        ``speed``      本次导航的线速度上限（m/s），须在 (0, max_linear]
                       内；缺省用 ``NavigateConfig.max_linear_m_s``。
        ``replanner``  可选全局重规划回调 ``fn(goal_x, goal_y) -> 中间航点
                       列表 | None``。导航中被障碍持续阻挡超过
                       ``replan_after_s`` 时，用**当前位置**重新 A* 规划，
                       替换剩余航点，避免反应式绕行死磕到超时放弃。
        ``goal_yaw``   终点车身航向（弧度，逆时针为正）。``None`` 时用接近
                       目标时锁定的航向（最后一段的 atan2），XY 到位后再
                       原地转正，避免只到点、车身斜着停。
        """
        cfg = self._config
        if speed is not None:
            if speed <= 0.0:
                raise ValueError("goto speed must be > 0")
            speed = min(speed, cfg.max_linear_m_s)
        with self._lock:
            if self._navigating:
                logger.warning("Navigator: already navigating, ignoring goto")
                return False
            self._goal = (x, y)
            self._goal_yaw = goal_yaw
            self._waypoints = list(waypoints or [])
            self._on_arrived = on_arrived
            self._on_abort = on_abort
            self._goal_speed = speed
            self._replanner = replanner
            self._navigating = True
            self._stop_event.clear()

        n_wp = len(self._waypoints)
        yaw_txt = "approach" if goal_yaw is None else f"{goal_yaw * _DEG_PER_RAD:.1f}deg"
        logger.info(
            "Navigating to (%.2f, %.2f) yaw=%s via %d waypoint(s) speed=%.2f",
            x, y, yaw_txt, n_wp, speed or cfg.max_linear_m_s,
        )
        self._thread = threading.Thread(
            target=self._nav_loop, name="nav-goto", daemon=True
        )
        self._thread.start()
        return True

    def set_guard(self, guard: Optional[ObstacleGuard]) -> None:
        """Swap the obstacle guard (used when the LiDAR stack is recreated)."""
        self._guard = guard

    def _set_vel(self, v: float, w: float) -> None:
        """Dispatch one velocity command (via drive callback or the controller).

        无 agent._drive 时（单测 / 独立导航）也必须过守卫：绕行与倒车
        旧实现会直接 set_velocity，倒进岩壁/悬崖。急停走 stop_motion。
        """
        if self._drive is not None:
            self._drive(v, w)
            return
        if self._guard is not None:
            v, w, blocked = self._guard.guard_velocity(v, w)
            if blocked:
                self._ctrl.stop_motion()
                return
        self._ctrl.set_velocity(v, w)

    def stop(self) -> None:
        """Abort navigation (thread-safe) and stop the chassis."""
        with self._lock:
            was = self._navigating
            self._navigating = False
            self._goal_speed = None
            thread = self._thread
        if was:
            self._stop_event.set()
            if thread:
                thread.join(timeout=1.0)
            self._ctrl.stop_motion()
            logger.info("Navigation stopped")

    # -- internal --------------------------------------------------------

    def _nav_loop(self) -> None:
        cfg = self._config
        stalled_since: Optional[float] = None
        arrived = False
        abort_reason = ""

        # stage-2: 到达停滞检测 + 倒车脱困状态
        near_goal_since: Optional[float] = None
        last_probe_pos: Optional[tuple[float, float]] = None
        last_probe_time: Optional[float] = None
        backup_until: float = 0.0
        backing = False
        # 绕行方向锁定：持续被堵期间保持同一转向侧，避免走廊/凹槽反复横跳
        detour_side = 0
        # 航点推进：先走规划中间航点，最后到最终目标
        wp_index = 0
        # 全局重规划节流：距上一次 replan 的最短间隔（避免每帧跑 A*）
        replan_at = 0.0
        # 终点车身：远距离锁定接近航向；XY 到位后原地转到该航向（或显式 goal_yaw）
        approach_yaw: Optional[float] = None
        align_since: Optional[float] = None

        def _target_goal() -> tuple[float, float]:
            nonlocal wp_index
            wps = self._waypoints
            if wp_index < len(wps):
                return wps[wp_index]
            return self._goal or (0.0, 0.0)

        def _try_replan() -> bool:
            """被堵死磕时从当前位置重新全局规划，替换剩余航点。

            返回 True 表示已替换航点（调用方 continue 进入下一周期）。
            重规划失败（无路径/无 replanner）返回 False，继续反应式绕行，
            最终由 stall_timeout 兜底放弃。
            """
            nonlocal wp_index, stalled_since, detour_side, replan_at, approach_yaw, align_since
            if self._replanner is None or self._goal is None:
                return False
            now = time.monotonic()
            if now < replan_at:
                return False
            replan_at = now + cfg.replan_after_s
            try:
                new_wps = self._replanner(self._goal[0], self._goal[1])
            except Exception:
                logger.exception("Navigator replanner callback failed")
                return False
            if new_wps is None:
                return False
            self._waypoints = list(new_wps)
            wp_index = 0
            stalled_since = None
            detour_side = 0
            approach_yaw = None
            align_since = None
            logger.info("Navigator replanned via %d waypoint(s)", len(new_wps))
            return True

        try:
            while not self._stop_event.is_set():
                with self._lock:
                    if not self._navigating:
                        break
                    goal_speed = self._goal_speed
                    pose = Pose2D(self._pose.pose.x, self._pose.pose.y, self._pose.pose.yaw)

                if self._goal is None:
                    break
                gx, gy = _target_goal()
                dx = gx - pose.x
                dy = gy - pose.y
                dist = math.hypot(dx, dy)
                is_final = wp_index >= len(self._waypoints)

                # 目标方向
                target_yaw = math.atan2(dy, dx) if dist > 1e-4 else (approach_yaw or pose.yaw)
                if dist > cfg.approach_lock_m:
                    approach_yaw = target_yaw
                elif approach_yaw is None:
                    approach_yaw = target_yaw
                heading_err = _wrap_angle(target_yaw - pose.yaw)
                heading_err_deg = heading_err * _DEG_PER_RAD
                desired_final_yaw = (
                    self._goal_yaw if self._goal_yaw is not None else approach_yaw
                )
                final_yaw_err = _wrap_angle(desired_final_yaw - pose.yaw)

                def _advance_waypoint() -> None:
                    nonlocal wp_index, stalled_since, detour_side
                    nonlocal near_goal_since, last_probe_pos, last_probe_time
                    nonlocal approach_yaw, align_since
                    wp_index += 1
                    stalled_since = None
                    detour_side = 0
                    near_goal_since = None
                    last_probe_pos = None
                    last_probe_time = None
                    approach_yaw = None
                    align_since = None

                if dist <= cfg.goal_tolerance_m:
                    # 中间航点：只过位置，立刻推进（折线途中不必转正车身）
                    if not is_final:
                        _advance_waypoint()
                        continue
                    # 终点：位置到位后必须把车身转到接近航向 / 显式 goal_yaw
                    if abs(final_yaw_err) <= cfg.final_yaw_tolerance_rad:
                        arrived = True
                        break
                    if align_since is None:
                        align_since = time.monotonic()
                    elif time.monotonic() - align_since >= cfg.final_align_timeout_s:
                        logger.info(
                            "Navigator: XY in tolerance, heading align timed out "
                            "(err=%.1f°) — accepting pose",
                            final_yaw_err * _DEG_PER_RAD,
                        )
                        arrived = True
                        break
                    if abs(final_yaw_err) <= cfg.heading_deadband_rad:
                        arrived = True
                        break
                    w_align = max(-cfg.max_angular_rad_s,
                                  min(cfg.max_angular_rad_s,
                                      final_yaw_err * cfg.heading_gain))
                    self._set_vel(0.0, w_align)
                    self._stop_event.wait(timeout=cfg.update_interval_s)
                    continue

                # 到达停滞判定：只在很靠近目标且几乎不动时启用。
                # 旧 0.4m 探针会把「还差半个车身」误判成到达。
                # 终点还要求航向大致对齐，否则继续转正而不是报 arrived。
                if dist <= cfg.arrive_probe_m:
                    now = time.monotonic()
                    pos = (pose.x, pose.y)
                    if last_probe_pos is not None and last_probe_time is not None:
                        moved = math.hypot(pos[0] - last_probe_pos[0],
                                           pos[1] - last_probe_pos[1])
                        if moved < cfg.arrive_stall_moved_m:
                            if near_goal_since is None:
                                near_goal_since = now
                            elif now - near_goal_since >= cfg.arrive_stall_s:
                                if not is_final:
                                    _advance_waypoint()
                                    continue
                                if abs(final_yaw_err) <= cfg.final_yaw_tolerance_rad * 2.0:
                                    arrived = True
                                    break
                                # 位置停滞但车身未转正：进入原地对齐，不要冒充到达
                                near_goal_since = None
                        else:
                            near_goal_since = None
                    last_probe_pos = pos
                    last_probe_time = now
                else:
                    near_goal_since = None
                    last_probe_pos = None

                # 基础指令（航向 PID）；speed 可选参数限制本次导航线速度上限
                speed_cap = goal_speed if goal_speed is not None else cfg.max_linear_m_s
                v_cmd = speed_cap * min(1.0, dist * cfg.approach_gain)
                # 转向：死区防末端摆动；大航向误差原地转正，禁止 25% 速度斜着蹭
                if abs(heading_err) <= cfg.heading_deadband_rad:
                    w_cmd = 0.0
                else:
                    w_cmd = max(-cfg.max_angular_rad_s,
                                min(cfg.max_angular_rad_s,
                                    heading_err * cfg.heading_gain))
                if abs(heading_err) >= cfg.align_in_place_rad:
                    v_cmd = 0.0
                else:
                    turn_w = 1.0 - min(1.0, abs(heading_err) / cfg.turn_decel_rad)
                    v_cmd *= max(cfg.turn_min_speed_ratio, turn_w)

                # 通过性提前绕行：目标方向不可通行 → 不等急停，提前转向开阔侧
                if self._guard is not None:
                    block_dist = self._guard.front_blocked_lookahead(cfg.terrain_lookahead_m)
                    if block_dist is not None and not backing:
                        steer = self._guard.steer_away_deg()
                        if steer != 0.0 and detour_side == 0:
                            detour_side = 1 if steer >= 0 else -1
                        if abs(heading_err_deg) > 45.0 and detour_side == 0:
                            w_cmd = max(-cfg.max_angular_rad_s,
                                        min(cfg.max_angular_rad_s,
                                            heading_err * cfg.heading_gain))
                        elif detour_side != 0:
                            w_cmd = cfg.max_angular_rad_s if detour_side >= 0 else -cfg.max_angular_rad_s
                        elif steer >= 0:
                            w_cmd = cfg.max_angular_rad_s
                        else:
                            w_cmd = -cfg.max_angular_rad_s
                        v_cmd = cfg.max_linear_m_s * 0.25  # 减速贴边转向
                        if stalled_since is None:
                            stalled_since = time.monotonic()
                        if _try_replan():
                            self._stop_event.wait(timeout=cfg.update_interval_s)
                            continue
                        if time.monotonic() - stalled_since > cfg.stall_timeout_s:
                            abort_reason = (
                                f"前方地形不可通行（{block_dist:.2f} m）且长时间无法绕过，"
                                f"放弃目标 ({gx:.2f}, {gy:.2f})"
                            )
                            break
                        self._set_vel(v_cmd, w_cmd)
                        self._stop_event.wait(timeout=cfg.update_interval_s)
                        continue

                # 避障守卫：急停/限速/坡面
                v_safe, w_safe, blocked = (v_cmd, w_cmd, False)
                if self._guard is not None:
                    v_safe, w_safe, blocked = self._guard.guard_velocity(v_cmd, w_cmd)

                if blocked:
                    # 正前方被挡：按锁定的绕行方向转向；两侧都堵则倒车脱困
                    if stalled_since is None:
                        stalled_since = time.monotonic()
                        detour_side = 0          # 新一轮堵车：重新评估绕行方向
                    if _try_replan():
                        self._stop_event.wait(timeout=cfg.update_interval_s)
                        continue
                    steer = 0.0
                    if self._guard is not None:
                        steer = self._guard.steer_away_deg()
                    if backing:
                        # 倒车脱困中：继续倒到 backup_until
                        if time.monotonic() < backup_until:
                            self._set_vel(-cfg.backup_speed_m_s, 0.0)
                            self._stop_event.wait(timeout=cfg.update_interval_s)
                            continue
                        backing = False
                    # 锁定绕行方向：整段堵车期间保持同一侧，避免左右横跳
                    if steer != 0.0 and detour_side == 0:
                        detour_side = 1 if steer >= 0 else -1
                    if detour_side != 0:
                        w_safe = cfg.max_angular_rad_s if detour_side >= 0 else -cfg.max_angular_rad_s
                    else:
                        # 前方、左、右都被堵（steer=0）→ 倒车一段再转向
                        backing = True
                        backup_until = time.monotonic() + cfg.backup_duration_s
                        self._set_vel(-cfg.backup_speed_m_s, 0.0)
                        self._stop_event.wait(timeout=cfg.update_interval_s)
                        continue
                    v_safe = 0.0
                    if time.monotonic() - stalled_since > cfg.stall_timeout_s:
                        abort_reason = (
                            f"障碍长时间阻挡（> {cfg.stall_timeout_s:.0f}s），放弃目标 "
                            f"({gx:.2f}, {gy:.2f})"
                        )
                        break
                else:
                    stalled_since = None
                    detour_side = 0

                self._set_vel(v_safe, w_safe)
                self._stop_event.wait(timeout=cfg.update_interval_s)
        except Exception:
            logger.exception("Navigation loop error")
            abort_reason = "导航线程异常"
        finally:
            self._ctrl.stop_motion()
            with self._lock:
                self._navigating = False
                cb_arrived = self._on_arrived if arrived else None
                cb_abort = self._on_abort if (abort_reason and not self._stop_event.is_set()) else None
                self._on_arrived = None
                self._on_abort = None

            if arrived:
                logger.info("Arrived at goal (%.2f, %.2f)", self._goal[0] if self._goal else 0, self._goal[1] if self._goal else 0)
                if cb_arrived:
                    try:
                        cb_arrived()
                    except Exception:
                        logger.exception("Navigation arrived callback error")
            elif abort_reason:
                logger.warning("Navigation aborted: %s", abort_reason)
                if cb_abort:
                    try:
                        cb_abort(abort_reason)
                    except Exception:
                        logger.exception("Navigation abort callback error")


def _wrap_angle(a: float) -> float:
    """Wrap radians to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a
