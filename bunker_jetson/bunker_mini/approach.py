"""Grasp-approach state machine (stage 3).

任务链「自动导航到该位置 → 由小车搭载的机械臂抓取」之间需要车体提供
一个明确的「服务原语」：**正对目标、停在机械臂工作距离、完全停稳并
上报已就位**。机械臂本身不在此讨论，但车体必须把「到位」做扎实，否则
机械臂会因距离/朝向不对而失败。

状态流转::

    IDLE → ALIGNING → APPROACHING → STABILIZING → READY
           ↑              │
           └── SEARCHING ←┘（目标短暂丢失 → 原地旋转重新对准）
    READY / FAILED / ABORTED 为终态

用法（由 agent 的 find_object 编排线程驱动，每控制周期调用一次）::

    app = ApproachController(config)
    app.start()
    while ...:
        est = detector.detect(points)[0]      # 视觉反馈
        v, w = app.update(est)                # 返回应下发的 (v, w)
        controller.set_velocity(v, w)
        if app.state in (READY, FAILED, ABORTED):
            break
    app.stop()
    controller.stop_motion()

实际环境注意：
  * 对接必须在「视觉反馈」下闭环，不能只靠开环导航——导航误差 + 里程计
    漂移在 1m 内可能达 10cm+，机械臂工作距离通常 0.4~0.6m，必须实时修正。
  * 到达工作距离后必须「停稳确认」若干秒，避免车体抖动导致机械臂抓偏。
  * 目标丢失（被扬尘/遮挡/视觉盲区）时原地低速旋转重寻，而不是盲目乱动。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from .vision import TargetEstimate

logger = logging.getLogger(__name__)


class ApproachState(Enum):
    IDLE = "idle"
    SEARCHING = "searching"      # 目标丢失，原地旋转重寻
    ALIGNING = "aligning"        # 对准目标方位
    APPROACHING = "approaching"  # 保持对准并逼近到工作距离
    STABILIZING = "stabilizing"  # 到位停稳确认
    READY = "ready"              # 已就位，可抓取（终态）
    FAILED = "failed"            # 目标丢失超时（终态）
    ABORTED = "aborted"          # 外部取消（终态）


@dataclass
class ApproachConfig:
    """Tuning parameters for the grasp-approach controller."""

    approach_distance_m: float = 0.5      # 机械臂工作距离（车头到目标**中心**）
    distance_tolerance_m: float = 0.05    # 距离到位容差
    max_linear_m_s: float = 0.20          # 逼近线速度上限（安全低速）
    min_approach_m_s: float = 0.02        # 逼近最小速度（防止末端无限趋近）
    approach_gain: float = 1.0            # 距离误差 → 线速度
    max_angular_rad_s: float = 0.80       # 角速度上限
    align_gain: float = 1.5               # 方位误差(rad) → 角速度
    align_deadband_deg: float = 3.0       # 对准死区（超过才转）
    stabilize_s: float = 1.5              # 停稳确认时长
    search_angular_rad_s: float = 0.60    # 目标丢失时旋转速度
    target_lost_timeout_s: float = 8.0    # 目标丢失超过该时长 → FAILED
    update_interval_s: float = 0.02       # 与底盘 0x111 的 20 ms / 50 Hz 对齐
    # --- 近场盲区完成兜底 ---
    # Airy 视场 0~90°（向上半球）：逼近到很近时，低处标记会落到雷达水平面
    # 以下 → 视觉丢失。此时不应判失败，而应基于「最后一次有效估计的中心
    # 距离已进入机械臂工作区」完成到位。
    blind_zone_complete_m: float = 0.7    # 最后一次中心距离 ≤ 此值 → 进入盲区
    hold_grace_s: float = 2.0             # 盲区内视觉丢失后的停稳确认时长


class ApproachController:
    """Visual-servoing approach controller (state machine only; no I/O).

    不直接驱动底盘——每次调用 :meth:`update` 返回应下发的 (v, w)，
    由调用方 set_velocity，便于测试与复用。
    """

    def __init__(
        self,
        config: Optional[ApproachConfig] = None,
        *,
        on_ready: Optional[Callable[[TargetEstimate], None]] = None,
        on_failed: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._config = config or ApproachConfig()
        self._on_ready = on_ready
        self._on_failed = on_failed
        self._state = ApproachState.IDLE
        self._target_lost_since: Optional[float] = None
        self._stabilize_start: Optional[float] = None
        self._last_est: Optional[TargetEstimate] = None
        self._last_center_dist: Optional[float] = None
        self._aborted = False

    # -- observation -----------------------------------------------------

    @property
    def config(self) -> ApproachConfig:
        return self._config

    @property
    def state(self) -> ApproachState:
        return self._state

    @property
    def last_target(self) -> Optional[TargetEstimate]:
        return self._last_est

    # -- control ---------------------------------------------------------

    def start(self) -> None:
        """Reset to initial state before a new approach run."""
        self._state = ApproachState.SEARCHING
        self._target_lost_since = None
        self._stabilize_start = None
        self._last_est = None
        self._last_center_dist = None
        self._aborted = False

    def stop(self) -> None:
        """Abort the approach (thread-safe)."""
        self._aborted = True
        self._state = ApproachState.ABORTED

    def update(self, est: Optional[TargetEstimate]) -> tuple[float, float]:
        """Advance the state machine one control step; return (v, w).

        ``est=None`` 表示当前无目标观测（视觉丢失）。近距离（已进入机械臂
        工作区）丢失时按「盲区完成」处理，不判失败。
        """
        cfg = self._config
        now = time.monotonic()

        if self._aborted:
            self._state = ApproachState.ABORTED
            return 0.0, 0.0

        if est is None:
            self._last_est = None
            # 近场盲区：最后一次有效估计的目标中心已进入机械臂工作区
            # （目标落进雷达水平面以下 / 盲区），不再依赖视觉，直接按
            # 「已到位」停稳确认 → READY。
            if self._last_center_dist is not None \
                    and self._last_center_dist <= cfg.blind_zone_complete_m:
                self._state = ApproachState.STABILIZING
                if self._stabilize_start is None:
                    self._stabilize_start = now
                if now - self._stabilize_start >= cfg.hold_grace_s:
                    self._state = ApproachState.READY
                    logger.info(
                        "Approach READY in near-field blind zone "
                        "(last center %.2f m <= %.2f m)", self._last_center_dist,
                        cfg.blind_zone_complete_m)
                    if self._on_ready:
                        try:
                            self._on_ready(
                                TargetEstimate(
                                    distance_m=self._last_center_dist))
                        except Exception:
                            logger.exception("Approach on_ready callback error")
                return 0.0, 0.0

            self._stabilize_start = None
            if self._target_lost_since is None:
                self._target_lost_since = now
                self._state = ApproachState.SEARCHING
                return 0.0, cfg.search_angular_rad_s
            if now - self._target_lost_since > cfg.target_lost_timeout_s:
                self._state = ApproachState.FAILED
                logger.warning("Approach FAILED: target lost for %.1fs",
                               cfg.target_lost_timeout_s)
                if self._on_failed:
                    try:
                        self._on_failed("目标丢失超时")
                    except Exception:
                        logger.exception("Approach on_failed callback error")
                return 0.0, 0.0
            return 0.0, cfg.search_angular_rad_s

        # 目标有效：重置丢失计时，记录最近中心距离
        self._last_est = est
        self._target_lost_since = None
        self._last_center_dist = est.center_distance_m

        bearing = est.bearing_deg
        center_dist = est.center_distance_m
        align_err = _wrap_deg(bearing)

        # 1) 对准：方位误差超过死区 → 原地转向（不前进）
        if abs(align_err) > cfg.align_deadband_deg:
            self._state = ApproachState.ALIGNING
            self._stabilize_start = None
            w = max(-cfg.max_angular_rad_s,
                    min(cfg.max_angular_rad_s, align_err * _RAD_PER_DEG * cfg.align_gain))
            return 0.0, w

        # 2) 逼近：对准后边前进边微调方位（目标会因前进而移动到正前方）
        if center_dist > cfg.approach_distance_m + cfg.distance_tolerance_m:
            self._state = ApproachState.APPROACHING
            self._stabilize_start = None
            v = cfg.max_linear_m_s * min(
                1.0, (center_dist - cfg.approach_distance_m) * cfg.approach_gain)
            v = max(cfg.min_approach_m_s, v)  # 末端保持最小速度，避免无限趋近
            w = max(-cfg.max_angular_rad_s,
                    min(cfg.max_angular_rad_s, align_err * _RAD_PER_DEG * cfg.align_gain))
            return v, w

        # 3) 停稳确认：到位后保持静止 N 秒 → READY
        self._state = ApproachState.STABILIZING
        if self._stabilize_start is None:
            self._stabilize_start = now
        if now - self._stabilize_start >= cfg.stabilize_s:
            self._state = ApproachState.READY
            logger.info("Approach READY at %.2f m (center)", center_dist)
            if self._on_ready:
                try:
                    self._on_ready(est)
                except Exception:
                    logger.exception("Approach on_ready callback error")
        return 0.0, 0.0


_RAD_PER_DEG: float = 3.141592653589793 / 180.0


def _wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a
