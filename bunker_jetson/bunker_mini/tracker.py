"""Trajectory recorder and player for BUNKER MINI 2.0 — "轨迹克隆".

Record: manually drive the chassis along a desired path while sampling
odometry and motion; save as a track file.

Play: replay a saved track by covering the recorded wheel distances.
Each segment's (v, w) is recovered from wheel deltas; closed-loop
correction interpolates along the current polyline (heading + cross-track
on w, mild along-track on |v|). Playback issues velocities without a
second accel ramp so kb-recorded transients are not flattened twice.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from typing import Callable, Optional

from .controller import BunkerMiniController
from .navigator import OdometryPose, Pose2D
from .protocol import OdometerFeedback

logger = logging.getLogger(__name__)

DEFAULT_SAMPLE_INTERVAL_S: float = 0.1   # 100 ms recording resolution
DEFAULT_TRACK_DIR: str = "./tracks"

# Track names come from user input / cloud commands, so they must never be
# able to smuggle path separators or other illegal filename characters into
# the file system (path traversal / overwrite protection).
_SAFE_NAME_RE = re.compile(r"[\\/:*?\"<>|\x00-\x1f]")


def sanitize_track_name(name: str) -> str:
    """Return a filesystem-safe track name ('' → timestamp default)."""
    name = _SAFE_NAME_RE.sub("_", name).strip().strip(".")
    if not name:
        name = time.strftime("track_%Y%m%d_%H%M%S")
    return name

# Playback safety clamp. Recorded waypoints are replayed verbatim, but a
# malformed / oversized value (e.g. a track recorded before the angular
# feedback scale fix, or a hand-edited file) must never drive the chassis
# at dangerous speed or crash the command encoder.
MAX_PLAY_LINEAR_M_S: float = 0.5
MAX_PLAY_ANGULAR_RAD_S: float = 1.0

# Per-waypoint wall-clock ceiling for odometry-driven playback.  A waypoint
# normally takes ~100 ms to cover; this is the *floor* of the budget — the
# actual budget is scaled by the remaining distance and current speed so a
# slow, load-heavy chassis is never cut off mid-waypoint (which would
# silently drop recorded distance and corrupt the replay / fb round-trip).
PLAYBACK_WAYPOINT_CEILING_S: float = 2.0
# Scale factor for the distance-based waypoint budget:
#   budget = max(ceiling, remaining_m / |v| * SCALE + pad)
PLAYBACK_WAYPOINT_BUDGET_SCALE: float = 3.0
PLAYBACK_WAYPOINT_BUDGET_PAD_S: float = 1.0
# Odometer "stalled" detection during playback.  If the 0x311 wheel counter
# (or, as a fallback, the 0x221 wheel-speed integral) does not advance for
# this long, the chassis is considered stuck and the waypoint is abandoned.
# This replaces the old fixed total-duration×3 budget that used to cut a
# slow-but-moving replay short ("残缺路径" / fb 无法回程).
PLAYBACK_STALL_S: float = 1.5
# Hard upper bound for a *single* waypoint's wait, even when the chassis is
# still slowly making progress.  The dynamic budget covers normal/slow
# playbacks; this cap only protects against a pathological waypoint (e.g. a
# corrupted track with an enormous distance) keeping the playback loop in
# that waypoint forever.
PLAYBACK_HARD_WAYPOINT_S: float = 20.0
# While odometer frames are missing but the chassis is still moving, we
# integrate 0x221 wheel speeds to keep the replay advancing instead of
# timing out every waypoint.
PLAYBACK_INTEGRATE_S: float = 0.3


DEFAULT_WHEELBASE_M: float = 0.5  # BUNKER MINI 2.0 左右轮距（米）


@dataclass
class PlaybackCorrectionConfig:
    """Closed-loop lateral correction applied during track replay.

    回放时用同一差速模型把「已录制轮里程」积成期望位姿序列，同时用真实
    0x311 积实际位姿；每周期在**当前段内插值**的期望点上算横向 + 航向偏差，
    对 w 做 PID，对 |v| 做轻度纵向修正（不改符号、不抢里程目标）。
    ``_return_home`` 的倒放返回是最大受益者。
    """

    enabled: bool = True
    cross_track_gain: float = 1.8     # 横向偏差 → 角速度修正
    heading_gain: float = 1.2         # 航向偏差 → 角速度修正
    max_correction_rad_s: float = 0.4 # 单周期角速度修正上限（保证修正不失控）
    # 积分项：补偿长期恒定漂移（如单侧轮滑），时间窗限幅防 windup。
    cross_track_integral_gain: float = 0.35
    integral_window_s: float = 3.0
    integral_clamp: float = 0.25      # 积分项最大贡献（rad/s）
    # 航向优先：|航向偏差| 超过该阈值时横偏项乘衰减因子，先转头再纠线，
    # 避免横偏项与航向项互相打架导致 S 形振荡。
    heading_priority_rad: float = 0.35
    heading_priority_decay: float = 0.3
    wheelbase_m: float = DEFAULT_WHEELBASE_M  # 期望/实际位姿积分用的轮距
    wheelbase_tol_m: float = 0.02     # 与录制元数据 wheelbase 的差异告警阈值
    # 纵向：落后略提速、超前略减速（不改符号），缓解负载/跟踪滞后。
    along_track_gain: float = 0.8
    min_v_scale: float = 0.75
    max_v_scale: float = 1.25


@dataclass
class PlaybackDockConfig:
    """末端停靠：收尾段低速爬行 + 停稳确认，避免「到达但差几厘米」."""

    enabled: bool = True
    crawl_speed_m_s: float = 0.05     # 收尾爬行速度
    crawl_start_m: float = 0.25       # 距目标该距离内切爬行
    stall_s: float = 1.0              # 停稳确认时长
    stall_moved_m: float = 0.02       # 停稳期内位移上限
    dock_timeout_s: float = 3.0       # 停稳确认最长等待


def _wrap_angle(a: float) -> float:
    """Wrap radians to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _reached(target_delta: int, moved: int) -> bool:
    """True when a wheel has covered the (signed) required distance.

    Handles both forward (delta > 0) and reverse (delta < 0) segments.
    """
    if target_delta >= 0:
        return moved >= target_delta
    return moved <= target_delta


# 录制停住判定：低于此视为「松键停着」，不往轨迹里堆 idle 航点。
_IDLE_V_M_S: float = 0.015
_IDLE_W_RAD_S: float = 0.03
# kb 转弯/点按：持续转向时按此间隔强制加密（匀速转弯 |Δw| 不再超阈值）。
_CURVE_W_RAD_S: float = 0.08
_CURVE_MAX_TIME_S: float = 0.08
_CURVE_DIST_MM: float = 20.0
# 当前轨迹文件 schema：3 = 2 + 录制起点里程系位姿（倒放后精停用）。
TRACK_SCHEMA_VERSION: int = 3
# 可用于「必须回原点」的里程来源：真实 0x311，或 0x311+0x221 融合。
_RETURN_OK_ODO_SOURCES = frozenset({"real", "fused"})


def _is_idle(v: float, w: float) -> bool:
    return abs(v) < _IDLE_V_M_S and abs(w) < _IDLE_W_RAD_S


def segment_vw(
    prev: Waypoint,
    wp: Waypoint,
    wheelbase_m: float,
) -> tuple[float, float]:
    """从相邻航点的轮子位移 / 时间反推这一段应发的 (v, w)。

    比录制瞬间的 0x221 快照更贴这段实际走过的弧。dt 过小或位移几乎为 0
    时退回航点上记下的 v/w（停住、同戳双点）。
    """
    dt = float(wp.t) - float(prev.t)
    wb = wheelbase_m if wheelbase_m > 0.05 else DEFAULT_WHEELBASE_M
    dl = (wp.left_mm - prev.left_mm) / 1000.0
    dr = (wp.right_mm - prev.right_mm) / 1000.0
    if dt < 1e-3:
        return float(wp.v), float(wp.w)
    if abs(dl) + abs(dr) < 1e-4:
        return float(wp.v), float(wp.w)
    v = (dl + dr) / 2.0 / dt
    w = (dr - dl) / wb / dt
    return v, w


def lerp_pose(a: Pose2D, b: Pose2D, frac: float) -> Pose2D:
    """沿折线在两个期望位姿之间按比例插值（航向走最短角差）。"""
    t = max(0.0, min(1.0, float(frac)))
    dyaw = _wrap_angle(b.yaw - a.yaw)
    return Pose2D(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.yaw + dyaw * t,
    )


def interpolate_expected_pose(
    expected_poses: list[Pose2D],
    waypoints: list[Waypoint],
    idx: int,
    first: Waypoint,
    moved_l: float,
    moved_r: float,
) -> Pose2D:
    """按「本段已走轮程 / 本段目标轮程」插值当前应在的期望位姿。

    纠偏对准线段上的点，而不是下一个航点的终点，避免长段/弯道切角、
    被终点位姿吸过去画 S。
    """
    if not expected_poses:
        return Pose2D()
    idx = max(0, min(idx, len(expected_poses) - 1))
    if idx <= 0 or idx >= len(waypoints):
        return expected_poses[idx]
    prev = waypoints[idx - 1]
    wp = waypoints[idx]
    seg_l = float(wp.left_mm - prev.left_mm)
    seg_r = float(wp.right_mm - prev.right_mm)
    prev_l = float(prev.left_mm - first.left_mm)
    prev_r = float(prev.right_mm - first.right_mm)
    if abs(seg_l) + abs(seg_r) < 1.0:
        return expected_poses[idx]
    # 有符号进度：两轮各自在本段上的完成比例取平均。
    fracs: list[float] = []
    if abs(seg_l) >= 1.0:
        fracs.append((moved_l - prev_l) / seg_l)
    if abs(seg_r) >= 1.0:
        fracs.append((moved_r - prev_r) / seg_r)
    frac = sum(fracs) / len(fracs) if fracs else 1.0
    return lerp_pose(expected_poses[idx - 1], expected_poses[idx], frac)


def fill_vw_from_odometry(
    wps: list[Waypoint],
    wheelbase_m: float,
) -> list[Waypoint]:
    """用相邻航点轮位移覆盖 v/w，使指令与里程目标一致。

    首航点保持原值（通常是起步瞬间的 0x221）；其后每点表示「走到这里
    这一段」的平均速度。不改时间戳和左右轮毫米。
    """
    if len(wps) < 2:
        return wps
    out = [wps[0]]
    for i in range(1, len(wps)):
        v, w = segment_vw(wps[i - 1], wps[i], wheelbase_m)
        p = wps[i]
        out.append(Waypoint(t=p.t, left_mm=p.left_mm, right_mm=p.right_mm,
                            v=round(v, 4), w=round(w, 4)))
    return out


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass
class Waypoint:
    t: float          # seconds from recording start
    left_mm: int      # left wheel cumulative mm
    right_mm: int     # right wheel cumulative mm
    v: float          # linear velocity m/s
    w: float          # angular velocity rad/s


@dataclass
class Track:
    name: str
    created_at: str                         # ISO timestamp
    total_duration_s: float
    waypoints: list[Waypoint] = field(default_factory=list)
    # --- 录制元数据（B：录制质量增强，供回放校验/纠偏）---
    schema_version: int = TRACK_SCHEMA_VERSION
    wheelbase_m: Optional[float] = None     # 录制时的轮距，回放优先用此值积分
    total_distance_m: Optional[float] = None  # 里程总长（近似）
    max_speed_m_s: Optional[float] = None
    max_angular_rad_s: Optional[float] = None
    sample_mode: str = "fixed"              # "fixed" | "adaptive"
    odometer_source: str = "unknown"        # real / fused / synthetic / mixed
    drive_mode: str = "unknown"             # kb / remote / unknown
    # 录制开始时的里程系位姿（agent 启动原点或上次 odom_reset）。
    # 倒放只保证轮脉冲对上；精停用这组坐标做 goto + 航向。
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    start_yaw_deg: Optional[float] = None

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, indent=2)

    def reversed(self) -> Track:
        """Return a copy that drives the recorded path back to its start.

        航点倒序、时间戳翻转；v/w 取反（倒行时保持与录制一致的转弯半径，
        使小车沿原路径原路返回起点）。里程字段保持不变——播放器的里程驱动
        以首航点为基准计算 delta，倒序后 delta 为负，`_reached` 已支持负向。
        元数据原样保留（轮距/总里程/最大速度不变）。
        """
        total = self.total_duration_s
        wps = [
            Waypoint(
                t=round(max(total - w.t, 0.0), 3),
                left_mm=w.left_mm,
                right_mm=w.right_mm,
                v=-w.v,
                w=-w.w,
            )
            for w in reversed(self.waypoints)
        ]
        return Track(
            name=self.name,
            created_at=self.created_at,
            total_duration_s=total,
            waypoints=wps,
            schema_version=self.schema_version,
            wheelbase_m=self.wheelbase_m,
            total_distance_m=self.total_distance_m,
            max_speed_m_s=self.max_speed_m_s,
            max_angular_rad_s=self.max_angular_rad_s,
            sample_mode=self.sample_mode,
            odometer_source=self.odometer_source,
            drive_mode=self.drive_mode,
            start_x=self.start_x,
            start_y=self.start_y,
            start_yaw_deg=self.start_yaw_deg,
        )

    @classmethod
    def from_json(cls, data: str | dict) -> Track:
        if isinstance(data, str):
            data = json.loads(data)
        if not isinstance(data, dict):
            raise ValueError("track JSON root must be an object")

        # Tolerate missing / extra fields so hand-edited or partial files
        # still load instead of crashing with KeyError/TypeError.
        wps: list[Waypoint] = []
        for w in data.get("waypoints", []) or []:
            if not isinstance(w, dict):
                continue
            if "t" not in w:
                continue  # waypoint without a timestamp is meaningless
            try:
                wps.append(
                    Waypoint(
                        t=float(w["t"]),
                        left_mm=int(w.get("left_mm", 0)),
                        right_mm=int(w.get("right_mm", 0)),
                        v=float(w.get("v", 0.0)),
                        w=float(w.get("w", 0.0)),
                    )
                )
            except (TypeError, ValueError):
                continue  # malformed waypoint — skip

        def _opt_float(key: str) -> Optional[float]:
            try:
                val = data.get(key)
                return float(val) if val is not None else None
            except (TypeError, ValueError):
                return None

        return cls(
            name=str(data.get("name", "unnamed")),
            created_at=str(data.get("created_at", "")),
            total_duration_s=float(data.get("total_duration_s", 0.0)),
            waypoints=wps,
            schema_version=int(data.get("schema_version", 1)),
            wheelbase_m=_opt_float("wheelbase_m"),
            total_distance_m=_opt_float("total_distance_m"),
            max_speed_m_s=_opt_float("max_speed_m_s"),
            max_angular_rad_s=_opt_float("max_angular_rad_s"),
            sample_mode=str(data.get("sample_mode", "fixed")),
            odometer_source=str(data.get("odometer_source", "unknown")),
            drive_mode=str(data.get("drive_mode", "unknown")),
            start_x=_opt_float("start_x"),
            start_y=_opt_float("start_y"),
            start_yaw_deg=_opt_float("start_yaw_deg"),
        )


def track_has_wheel_odo(track: Track) -> bool:
    """轨迹里左右轮毫米是否真的走过（有 0x311 一类位移，而不是全 0）。"""
    wps = track.waypoints
    if len(wps) < 2:
        return False
    first = wps[0]
    return any(
        w.left_mm != first.left_mm or w.right_mm != first.right_mm
        for w in wps[1:]
    )


def track_origin_quality(track: Track) -> dict:
    """这条轨迹能不能当「必须回到录制起点」用。

    ``ok``       可以倒放（有轮式里程且来源不是 synthetic）。
    ``dock_ok``  倒放后还能用录制起点做里程系精停。
    """
    has_odo = track_has_wheel_odo(track)
    src = (track.odometer_source or "unknown").strip().lower() or "unknown"
    has_start = track.start_x is not None and track.start_y is not None
    reasons: list[str] = []
    if not has_odo:
        reasons.append("no_wheel_odo")
    if src == "synthetic":
        reasons.append("odometer_source=synthetic")
    elif src in ("unknown", "none", ""):
        reasons.append(f"odometer_source={src or 'unknown'}")
    elif src == "mixed":
        reasons.append("odometer_source=mixed")
    if not has_start:
        reasons.append("no_start_pose")
    ok = has_odo and src != "synthetic"
    return {
        "ok": ok,
        "dock_ok": ok and has_start,
        "has_odo": has_odo,
        "odometer_source": src,
        "drive_mode": track.drive_mode or "unknown",
        "has_start_pose": has_start,
        "reasons": reasons,
    }


def track_web_summary(track: Track) -> dict:
    """网页轨迹列表用的摘要（含能否回原点）。"""
    q = track_origin_quality(track)
    start = None
    if track.start_x is not None and track.start_y is not None:
        start = {
            "x": round(float(track.start_x), 3),
            "y": round(float(track.start_y), 3),
            "yawDeg": None if track.start_yaw_deg is None
            else round(float(track.start_yaw_deg), 1),
        }
    return {
        "name": track.name,
        "duration": round(float(track.total_duration_s), 2),
        "waypoints": len(track.waypoints),
        "odometerSource": q["odometer_source"],
        "driveMode": q["drive_mode"],
        "hasOdo": q["has_odo"],
        "returnOk": q["ok"] and q["odometer_source"] in _RETURN_OK_ODO_SOURCES,
        "dockOk": q["dock_ok"],
        "reasons": q["reasons"],
        "startPose": start,
        "distanceM": track.total_distance_m,
    }


# ---------------------------------------------------------------------------
# TrackRecorder
# ---------------------------------------------------------------------------


class TrackRecorder:
    """Sample chassis state while the vehicle is manually driven.

    Usage::

        recorder = TrackRecorder(controller)
        recorder.start("warehouse_loop")
        # … manually drive the chassis via remote control …
        track = recorder.stop()
        track.save()   # → ./tracks/warehouse_loop.json
    """

    def __init__(
        self,
        controller: BunkerMiniController,
        *,
        sample_interval_s: float = DEFAULT_SAMPLE_INTERVAL_S,
        track_dir: str = DEFAULT_TRACK_DIR,
        adaptive: bool = True,
        min_time_s: float = 0.05,          # 自适应：最小采样间隔（50ms 上限）
        max_time_s: float = 0.4,           # 自适应：须短于底盘 0x111 失联窗 0.5s
        dist_threshold_mm: float = 50.0,   # 自适应：两轮平均位移 ≥ 此值才采样
        vel_threshold_m_s: float = 0.02,   # 自适应：|Δv| ≥ 此值加密采样
        ang_threshold_rad_s: float = 0.05, # 自适应：|Δw| ≥ 此值加密采样
        smoothing_window: int = 1,         # 默认不平滑 v/w（改由轮位移反推）
        wheelbase_m: Optional[float] = None,  # 写入元数据，供回放校验/纠偏
        drive_mode: str = "kb",            # 首选键盘录制；写入轨迹供回放策略
    ) -> None:
        self._ctrl = controller
        self._interval = sample_interval_s
        self._track_dir = Path(track_dir)
        self._adaptive = adaptive
        self._min_time = min_time_s
        self._max_time = max_time_s
        self._dist_thr = dist_threshold_mm
        self._vel_thr = vel_threshold_m_s
        self._ang_thr = ang_threshold_rad_s
        self._smoothing = max(1, int(smoothing_window))
        self._wheelbase_m = wheelbase_m
        self._drive_mode = drive_mode or "unknown"
        self._odo_src_counts: dict[str, int] = {}

        self._recording = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._waypoints: list[Waypoint] = []
        self._name: str = ""

    # -- public ---------------------------------------------------------

    def start(self, name: str = "") -> None:
        with self._lock:
            if self._recording:
                logger.warning("Already recording")
                return
            self._name = sanitize_track_name(name or time.strftime("track_%Y%m%d_%H%M%S"))
            self._waypoints.clear()
            self._odo_src_counts.clear()
            self._recording = True
            self._rec_t0 = time.monotonic()
            self._last_sample_t = 0.0
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._record_loop, name="track-rec", daemon=True)
            self._thread.start()
        logger.info("Recording track '%s' ...", self._name)

    def stop(self) -> Track:
        with self._lock:
            if not self._recording:
                raise RuntimeError("Not recording")
            self._stop_event.set()
            thread = self._thread
        if thread:
            thread.join(timeout=3.0)

        with self._lock:
            self._recording = False

            # 自适应采样下，最后一次位移可能尚未落盘——补一个尾部采样，
            # 保证轨迹末航点的里程 = 实际最终位置（回放里程驱动的终点一致）。
            self._append_sample(force=True, final=True)

            if not self._waypoints:
                raise RuntimeError("No waypoints recorded")

            # 峰值速度从原始 0x221 取；落盘 v/w 改由轮位移反推，与回放段目标一致。
            raw = self._waypoints
            wb = self._wheelbase_m if self._wheelbase_m else DEFAULT_WHEELBASE_M
            wps = fill_vw_from_odometry(list(raw), wb)
            if self._smoothing > 1:
                wps = self._smooth_vw(wps)
            first, last = wps[0], wps[-1]

            total_dist = (
                abs(last.left_mm - first.left_mm)
                + abs(last.right_mm - first.right_mm)
            ) / 2.0 / 1000.0

            track = Track(
                name=self._name,
                created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
                total_duration_s=wps[-1].t,
                waypoints=wps,
                schema_version=TRACK_SCHEMA_VERSION,
                wheelbase_m=self._wheelbase_m,
                total_distance_m=round(total_dist, 3),
                max_speed_m_s=max((abs(w.v) for w in raw), default=0.0),
                max_angular_rad_s=max((abs(w.w) for w in raw), default=0.0),
                sample_mode="adaptive" if self._adaptive else "fixed",
                odometer_source=self._majority_odo_source(),
                drive_mode=self._drive_mode,
            )

            # A track full of zeros means the chassis never moved during
            # recording (or 0x221/0x311 feedback was not received) — warn so
            # the user knows the recording is invalid before replaying it.
            has_motion = any(w.v != 0.0 or w.w != 0.0 for w in track.waypoints)
            has_odo = any(
                w.left_mm != track.waypoints[0].left_mm
                or w.right_mm != track.waypoints[0].right_mm
                for w in track.waypoints[1:]
            )
            if not has_motion and not has_odo:
                logger.warning(
                    "Track '%s' recorded no movement data — the chassis did not "
                    "move, or feedback frames 0x221/0x311 were not received "
                    "during recording. Please re-record while driving.",
                    track.name,
                )
            elif has_motion and not has_odo:
                # 有速度/转向反馈但 0x311 里程一直是 0：车确实在动，但里程计
                # 没记录到距离。这样回放只能退化为「时间回放」（按录制速度/
                # 时长原样重放），没有位置反馈，无法精确回到起点。最常见原因：
                # 录制时底盘处于遥控模式（SWB 未拨到指令档），里程计只在
                # CAN 指令模式下计数。
                logger.warning(
                    "Track '%s' has motion feedback (0x221) but NO odometer "
                    "(0x311) — left/right wheel distance is all 0. Playback "
                    "will fall back to time-based replay and CANNOT accurately "
                    "return to the start. Please re-record in CAN command mode "
                    "(遥控器 SWB 拨到顶部指令档 / 用键盘 r 模式驾驶).",
                    track.name,
                )
        logger.info(
            "Track '%s' recorded: %d waypoints, %.1f s, %.2f m",
            self._name, len(track.waypoints), track.total_duration_s,
            track.total_distance_m or 0.0,
        )
        return track

    @property
    def is_recording(self) -> bool:
        with self._lock:
            return self._recording

    @property
    def waypoint_count(self) -> int:
        with self._lock:
            return len(self._waypoints)

    def _majority_odo_source(self) -> str:
        """录制期间出现最多的里程来源；多种并存则标 mixed。"""
        counts = {k: n for k, n in self._odo_src_counts.items()
                  if k and k != "none"}
        if not counts:
            return "unknown"
        if len(counts) > 1:
            top = max(counts.values())
            leaders = [k for k, n in counts.items() if n == top]
            if len(leaders) > 1:
                return "mixed"
        return max(counts, key=counts.get)

    # -- internal -------------------------------------------------------

    def _record_loop(self) -> None:
        while not self._stop_event.is_set():
            t = time.monotonic() - self._rec_t0
            if self._adaptive:
                if (t - self._last_sample_t) >= self._max_time:
                    self._append_sample(force=True)
                else:
                    self._append_sample(force=False)
            else:
                if (t - self._last_sample_t) >= self._interval:
                    self._append_sample(force=True)
            self._stop_event.wait(timeout=min(self._interval, 0.05))

    def _note_odo_source(self) -> None:
        src = getattr(self._ctrl, "odometer_source", None) or "none"
        self._odo_src_counts[src] = self._odo_src_counts.get(src, 0) + 1

    def _append_sample(self, *, force: bool = False, final: bool = False) -> None:
        """Sample chassis state, skipping redundant points in adaptive mode.

        kb 录制贴合实车：
          * 松键停住后不再每 0.4s 堆 (0,0) 航点（想停顿不会被回放成干等）；
          * 起步/停车边沿立刻采样（点按 0.18s 也能落下起停）；
          * 持续转向按 20mm / 80ms 加密，匀速转弯不再等到 50mm/0.4s。
        ``force=True`` 仍受 idle 折叠约束；``final=True``（stop 补尾）一定落点。
        """
        odo = self._ctrl.latest_odometer
        motion = self._ctrl.latest_motion
        if odo is None and motion is None:
            return  # 底盘尚无任何反馈
        self._note_odo_source()
        # 遥控模式 0x311 可能尚未融合出来：仍用 0x221 速度落点，回放可走时间轴。
        wp = Waypoint(
            t=round(time.monotonic() - self._rec_t0, 3),
            left_mm=odo.left_wheel_mm if odo is not None else 0,
            right_mm=odo.right_wheel_mm if odo is not None else 0,
            v=motion.linear_velocity_m_s if motion else 0.0,
            w=motion.angular_velocity_rad_s if motion else 0.0,
        )

        last = self._waypoints[-1] if self._waypoints else None
        if last is None:
            self._waypoints.append(wp)
            self._last_sample_t = wp.t
            return

        idle_now = _is_idle(wp.v, wp.w)
        idle_last = _is_idle(last.v, last.w)
        if (not final) and idle_now and idle_last:
            # 思考停顿：只保留进入静止的那一个点，更新时间以免 max_time 空转。
            self._last_sample_t = wp.t
            return

        motion_edge = idle_now != idle_last
        dt = wp.t - last.t
        dist = (abs(wp.left_mm - last.left_mm)
                + abs(wp.right_mm - last.right_mm)) / 2.0
        dv = abs(wp.v - last.v)
        dw = abs(wp.w - last.w)
        turning = abs(wp.w) >= _CURVE_W_RAD_S
        curve_due = turning and (
            dist >= _CURVE_DIST_MM or dt >= _CURVE_MAX_TIME_S
        )

        if final or force:
            self._waypoints.append(wp)
            self._last_sample_t = wp.t
            return

        if not self._adaptive:
            self._waypoints.append(wp)
            self._last_sample_t = wp.t
            return

        if (motion_edge or curve_due
                or dist >= self._dist_thr
                or dv >= self._vel_thr
                or dw >= self._ang_thr):
            self._waypoints.append(wp)
            self._last_sample_t = wp.t

    def _smooth_vw(self, wps: list[Waypoint]) -> list[Waypoint]:
        """Light moving average on v/w to remove manual-driving jitter.

        保持端点原值；窗口 3 的 [0.25, 0.5, 0.25] 加权。只平滑速度/转向，
        不改动里程与时间戳——回放仍是里程驱动，平滑只让运动更顺。
        """
        if self._smoothing <= 1 or len(wps) < 3:
            return wps
        out: list[Waypoint] = [wps[0]]
        for i in range(1, len(wps) - 1):
            w = wps[i]
            prev, nxt = wps[i - 1], wps[i + 1]
            out.append(
                Waypoint(
                    t=w.t,
                    left_mm=w.left_mm,
                    right_mm=w.right_mm,
                    v=round(0.25 * prev.v + 0.5 * w.v + 0.25 * nxt.v, 4),
                    w=round(0.25 * prev.w + 0.5 * w.w + 0.25 * nxt.w, 4),
                )
            )
        out.append(wps[-1])
        return out


# ---------------------------------------------------------------------------
# TrackPlayer
# ---------------------------------------------------------------------------


class TrackPlayer:
    """Replay a recorded track by issuing velocity commands at the
    recorded timestamps (open-loop replay).

    Usage::

        player = TrackPlayer(controller)
        player.play(track)   # blocking; Ctrl+C to abort
    """

    def __init__(
        self,
        controller: BunkerMiniController,
        *,
        track_dir: str = DEFAULT_TRACK_DIR,
        velocity_guard=None,
        wheelbase_m: float = DEFAULT_WHEELBASE_M,
        correction: Optional[PlaybackCorrectionConfig] = None,
        dock: Optional[PlaybackDockConfig] = None,
        drive: Optional[Callable[[float, float], None]] = None,
    ) -> None:
        self._ctrl = controller
        self._track_dir = Path(track_dir)
        self._lock = threading.Lock()
        self._playing = False
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # Playback progress 0..1 (elapsed / recorded duration), read by the
        # agent's state loop to report `task.progress` to the cloud.
        self._progress: float = 0.0
        # Optional safety wrapper applied to every waypoint velocity before
        # it reaches the chassis: callable(v, w) -> (v, w, blocked).
        # When blocked=True the playback is aborted immediately (the chassis
        # is stopped; no "arrived" event is emitted).  This is how the agent
        # layers LiDAR obstacle avoidance on top of track replay.
        self._velocity_guard = velocity_guard
        # A: 闭环纠偏——期望/实际位姿差 → 只修正 w。None=关闭。
        self._correction = correction
        # 纠偏积分器状态（横向偏差时间窗积分，rebase 时清零）
        self._cross_int: float = 0.0
        self._cross_int_t: Optional[float] = None
        # 最近一次纠偏输出（rad/s），供诊断/日志
        self._last_correction: float = 0.0
        self._dock = dock
        # 速度分发器：回放默认走 set_velocity_now，避免「录制时已经过斜坡
        # 的 0x221」再套一层斜坡（kb 点按会被第二次斜坡抹平）。
        # agent 注入 _play_drive 时同样 bypass_ramp。
        if drive is not None:
            self._drive_cb = drive
        elif controller is not None:
            now_fn = getattr(controller, "set_velocity_now", None)
            self._drive_cb = now_fn if callable(now_fn) else controller.set_velocity
        else:
            self._drive_cb = None
        self._play_wb = (
            correction.wheelbase_m if correction is not None else wheelbase_m
        )
        self._guard_grace_s = 0.0
        self._play_drive_mode = "unknown"
        # Whether the most recent playback was distance-driven (real 0x311
        # odometer available) or degraded to time-based open-loop replay.
        # The agent uses this to report an honest completion message —
        # a time-based replay finishing does NOT mean the chassis returned
        # to the start pose.
        self._last_playback_had_odo: bool = False
        self._last_playback_stalled_wps: int = 0
        self._last_playback_time_fallback: bool = False
        self._aborted_by_guard: bool = False
        # 反向回放（B / fb 回程）加长守卫观察窗；被挡则中止，不再 skip 航点装到达。
        self._reverse_play: bool = False
        # 已录轨迹是固定轮迹：B / f / fb 回放不过雷达守卫。
        self._bypass_guard: bool = False

    def _set_vel(self, v: float, w: float) -> bool:
        """下发速度。守卫急停时停车并返回 False，调用方应中止整段回放。

        kb 录制的轨迹在录制时不过雷达：回放仍过守卫，但硬挡时先停车观察
        一小段（人走过、扬尘毛刺），解除后再继续，避免刚贴着录的路被误刹
        整段 abort。真障碍一直挡着仍中止。
        """
        if self._drive_cb is None:
            return False
        if (
            self._velocity_guard is not None
            and not self._bypass_guard
            and (abs(v) > 1e-6 or abs(w) > 1e-6)
        ):
            gv, gw, blocked = self._velocity_guard(v, w)
            if blocked:
                if self._guard_grace_s > 0.0 and not self._aborted_by_guard:
                    try:
                        self._ctrl.stop_motion()
                    except Exception:
                        try:
                            self._drive_cb(0.0, 0.0)
                        except Exception:
                            pass
                    deadline = time.monotonic() + self._guard_grace_s
                    while time.monotonic() < deadline:
                        if self._stop_event.is_set():
                            return False
                        self._stop_event.wait(timeout=0.05)
                        nv, nw, still = self._velocity_guard(v, w)
                        if not still:
                            self._drive_cb(nv, nw)
                            return True
                self._aborted_by_guard = True
                try:
                    self._ctrl.stop_motion()
                except Exception:
                    try:
                        self._drive_cb(0.0, 0.0)
                    except Exception:
                        pass
                logger.warning(
                    "Playback aborted by obstacle guard — chassis stopped")
                return False
            v, w = gv, gw
        self._drive_cb(v, w)
        return True

    def _hold_vel(self, v: float, w: float, sleep_s: float) -> bool:
        """按给定速度保持一段时间，每 50 ms 再过守卫（避免开环睡死）。"""
        if sleep_s <= 0:
            return self._set_vel(v, w)
        end = time.monotonic() + sleep_s
        while True:
            if self._stop_event.is_set() or self._aborted_by_guard:
                return False
            if not self._set_vel(v, w):
                return False
            left = end - time.monotonic()
            if left <= 0:
                return True
            self._stop_event.wait(timeout=min(0.05, left))

    def _stop_vel(self) -> None:
        # 必须硬停：drive_cb(0,0) 若只写指令不走 stop_motion，TX 环会
        # 把残速（常见 0.02 m/s）一直保活——回程结束后「停一会再爬」。
        if self._ctrl is not None:
            try:
                self._ctrl.stop_motion()
                return
            except Exception:
                pass
        if self._drive_cb is not None:
            self._drive_cb(0.0, 0.0)

    # -- public ---------------------------------------------------------

    def play(
        self,
        track: Track,
        on_complete: Optional[Callable[[bool], None]] = None,
        reverse: bool = False,
        bypass_guard: bool = False,
    ) -> bool:
        """Replay *track* — blocks until complete or interrupted.

        Returns True when the full track was covered, False when playback was
        stopped early (explicit stop, obstacle abort, or a chassis that never
        covered some waypoints).

        ``on_complete(complete)`` is always invoked when playback ends on its
        own (unless explicitly stopped), so the caller can report the actual
        outcome — even a partial/stalled replay is reported instead of leaving
        the cloud waiting forever.
        """
        with self._lock:
            if self._playing:
                raise RuntimeError("Already playing")
            if not track.waypoints:
                raise ValueError("Track has no waypoints")
            self._playing = True
            self._progress = 0.0
            self._reverse_play = bool(reverse)
            self._bypass_guard = bool(bypass_guard)
        self._stop_event.clear()

        logger.info(
            "Playing track '%s': %d waypoints, %.1f s%s%s",
            track.name, len(track.waypoints), track.total_duration_s,
            " (reverse)" if reverse else "",
            ", guard off" if bypass_guard else "",
        )

        complete = False
        try:
            complete = self._play_open_loop(track)
        finally:
            self._stop_vel()
            with self._lock:
                self._playing = False
            if on_complete is not None and not self._stop_event.is_set():
                try:
                    on_complete(complete)
                except Exception:
                    logger.exception("Track on_complete callback error")
            logger.info("Playback finished")
        return complete

    def play_async(
        self,
        track: Track,
        on_complete: Optional[Callable[[bool], None]] = None,
        reverse: bool = False,
        bypass_guard: bool = False,
    ) -> None:
        """Non-blocking replay. Call stop() to abort."""
        with self._lock:
            if self._playing:
                raise RuntimeError("Already playing")
            if not track.waypoints:
                raise ValueError("Track has no waypoints")
            self._playing = True
            self._progress = 0.0
            self._reverse_play = bool(reverse)
            self._bypass_guard = bool(bypass_guard)
        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._play_async_target, args=(track, on_complete),
            name="track-play", daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._lock:
            thread = self._thread
        if thread:
            thread.join(timeout=2.0)
        self._stop_vel()
        with self._lock:
            self._playing = False

    @property
    def is_playing(self) -> bool:
        with self._lock:
            return self._playing

    @property
    def bypass_guard(self) -> bool:
        """True 当本段回放不过雷达守卫（已录固定轨迹 / B 回程）。"""
        return bool(self._bypass_guard)

    @property
    def progress(self) -> float:
        """Playback progress 0..1 (elapsed / recorded duration)."""
        with self._lock:
            return self._progress

    @property
    def last_playback_had_odo(self) -> bool:
        """Whether the most recent playback was distance-driven (0x311
        odometer available).  When False the replay degraded to time-based
        open-loop, so a "completed" result does NOT mean the chassis
        physically returned to the start pose."""
        with self._lock:
            return self._last_playback_had_odo

    @property
    def last_playback_stalled_wps(self) -> int:
        """最近一次回放未覆盖的航点数（含倒放被挡中止前已计的 stall）。"""
        with self._lock:
            return self._last_playback_stalled_wps

    @property
    def last_playback_time_fallback(self) -> bool:
        """最近一次回放是否落到了时间轴兜底（无 0x311）。"""
        with self._lock:
            return self._last_playback_time_fallback

    @property
    def last_correction(self) -> float:
        """最近一次闭环纠偏的角速度修正量（rad/s），诊断漂移用。"""
        with self._lock:
            return self._last_correction

    def reset_correction(self) -> None:
        """清零纠偏积分器与统计（模式切换/重基准时调用）。"""
        with self._lock:
            self._cross_int = 0.0
            self._cross_int_t = None
            self._last_correction = 0.0

    def _set_progress(self, value: float) -> None:
        with self._lock:
            self._progress = max(0.0, min(1.0, value))

    # -- helpers --------------------------------------------------------

    def load_track(self, name_or_path: str) -> Track:
        """Load a track by name (looks in track_dir) or absolute path.

        Raises FileNotFoundError when the file is missing, and ValueError
        when it exists but is corrupt (e.g. a truncated/partial write from
        an interrupted save).
        """
        path = Path(name_or_path)
        if not path.is_absolute() and not path.exists():
            path = self._track_dir / f"{name_or_path}.json"
            if not path.exists():
                path = self._track_dir / name_or_path
        if not path.exists():
            raise FileNotFoundError(f"Track not found: {name_or_path} (tried {path})")
        try:
            with open(path, "r", encoding="utf-8") as f:
                return Track.from_json(f.read())
        except (ValueError, json.JSONDecodeError) as e:
            raise ValueError(f"Track file is corrupt: {path} ({e})") from e

    def track_exists(self, name: str) -> bool:
        """True when a track file with this name already exists on disk."""
        return (self._track_dir / f"{sanitize_track_name(name)}.json").exists()

    def save_track(self, track: Track) -> str:
        """Persist track to disk atomically. Returns the file path.

        The JSON is first written to a ``.tmp`` file, then renamed onto the
        final path with ``os.replace`` (atomic on both Windows and POSIX).
        A power loss / crash mid-write therefore can never leave a
        half-written track at the final path: on disk there is always either
        the previous complete file or the new complete file.
        """
        self._track_dir.mkdir(parents=True, exist_ok=True)
        path = self._track_dir / f"{sanitize_track_name(track.name)}.json"
        tmp_path = self._track_dir / f"{path.stem}.tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as f:
                f.write(track.to_json())
            os.replace(tmp_path, path)
        finally:
            # Never leave a stray .tmp behind (e.g. after a crash mid-write).
            if tmp_path.exists():
                try:
                    tmp_path.unlink()
                except OSError:
                    pass
        logger.info("Track saved to %s", path)
        return str(path)

    def delete_track(self, name: str) -> str:
        """Delete a saved track file by name. Returns the deleted path.

        Raises FileNotFoundError when no such track exists, so callers can
        report the failure instead of silently succeeding.
        """
        path = self._track_dir / f"{sanitize_track_name(name)}.json"
        if not path.exists():
            raise FileNotFoundError(f"Track not found: {name}")
        path.unlink()
        logger.info("Track deleted: %s", path)
        return str(path)

    # -- internal -------------------------------------------------------

    def _play_async_target(self, track: Track, on_complete: Optional[Callable[[bool], None]] = None) -> None:
        """Wrapper for async playback that guarantees _playing reset."""
        try:
            complete = self._play_open_loop(track)
        except Exception:
            logger.exception("Track playback error")
            complete = False
        finally:
            self._stop_vel()
            with self._lock:
                self._playing = False
            # 回放无论完整与否都上报结果（除非被显式 stop() 打断）：
            # complete=True=全程覆盖，False=残缺/卡住/被避障中止。之前只在
            # 完整时上报，导致「按 B 回程中途卡住 → 云端收不到 arrived →
            # 回程状态一直空等、终端卡死」。现在残缺回放也会上报，云端据此
            # 解除回程状态并提示用户。
            if on_complete is not None and not self._stop_event.is_set():
                try:
                    on_complete(complete)
                except Exception:
                    logger.exception("Track on_complete callback error")

    def _play_open_loop(self, track: Track) -> bool:
        """Replay a track so the chassis covers the recorded wheel distances.

        Distance-driven playback keeps issuing each waypoint's velocity until
        both wheels cover the recorded distance, so the replayed path matches
        the recording regardless of speed-tracking lag.  Falls back to
        time-based replay when the track contains no usable odometry (0x311).

        Returns True when the *full* track was covered; False when some
        waypoints were never covered (stalled chassis, total-budget timeout,
        obstacle-guard abort, or explicit stop).  Callers report the outcome
        either way (always via on_complete / arrived), so the cloud never
        waits forever on a stalled rewind.
        """
        if not track.waypoints:
            return False

        self._aborted_by_guard = False
        reverse_play = bool(self._reverse_play)
        t0 = time.monotonic()
        first = track.waypoints[0]
        clamped = 0
        # 纠偏/段速度用录制轮距；与 agent 配置不一致时仍用轨迹值并告警。
        cfg_wb = (self._correction.wheelbase_m if self._correction
                  else DEFAULT_WHEELBASE_M)
        if track.wheelbase_m is not None and track.wheelbase_m > 0.05:
            self._play_wb = track.wheelbase_m
        else:
            self._play_wb = cfg_wb
        self._play_drive_mode = track.drive_mode or "unknown"
        # kb 录的路回放硬挡先观察；反向回程再多等一会——终点附近车头
        # 常对着录制时身后的椅腿，第一拍正向探测会误刹。
        self._guard_grace_s = 1.2 if self._play_drive_mode == "kb" else 0.0
        if reverse_play:
            self._guard_grace_s = max(self._guard_grace_s, 2.0)
        live_src = getattr(self._ctrl, "odometer_source", None) or "none"
        rec_src = track.odometer_source or "unknown"
        if rec_src not in ("unknown", "") and live_src not in ("none", rec_src):
            logger.warning(
                "Track '%s' was recorded with odometer_source=%s, playback now "
                "sees %s — wheel-count scale may not match (prefer the same "
                "mode you used to record: kb/CAN vs remote)",
                track.name, rec_src, live_src,
            )

        # Does the track actually contain movement / odometry data?
        has_motion = any(wp.v != 0.0 or wp.w != 0.0 for wp in track.waypoints)
        has_odo = any(
            wp.left_mm != first.left_mm or wp.right_mm != first.right_mm
            for wp in track.waypoints[1:]
        )
        if not has_motion:
            logger.warning(
                "Track '%s' has no motion data (v/w all zero) — playback will not "
                "move the chassis. Re-record while the chassis is actually driving.",
                track.name,
            )
        if has_motion and not has_odo:
            logger.warning(
                "Track '%s' has no odometry data (0x311) — falling back to "
                "time-based replay.  This CANNOT accurately retrace the route; "
                "the chassis will not necessarily return to the start. "
                "Re-record in CAN command mode so 0x311 distance is captured.",
                track.name,
            )

        # Reference odometer taken when playback starts.  Distance-driven
        # replay is only used when the track has odometry AND the chassis is
        # currently reporting 0x311 frames; otherwise the original
        # time-based replay is used.
        #
        # 注意：如果启动瞬间 0x311 还没到，不立即降级为时间回放——先在首个
        # 航点上多等一会（首次 odometer 帧 + 首段运动），中途再拿到就切回
        # 里程驱动。只有确实完全没有里程反馈才用时间回放兜底。
        odo0 = self._wait_odometer(timeout=0.5) if has_odo else None
        if has_odo and odo0 is None:
            logger.warning(
                "No odometer feedback (0x311) yet — will retry during the first "
                "waypoint; falling back to time-based only if it never arrives"
            )
        # 里程驱动中途才拿到 0x311：从当前航点重基准（参考航点+里程+纠偏起点
        # 一并更新），避免把时间回放阶段已走过的距离重复计入剩余目标。
        odo_rebase: Optional[OdometerFeedback] = None
        rebase_idx: Optional[int] = None

        # 闭环纠偏 / 末端停靠与雷达守卫无关。bypass_guard 只表示「已录轮迹
        # 不过雷达」，不能连 segment_vw、纠偏、停靠一起关掉——否则 f/fb
        # 都只用航点快照，回起点时角速度对不上。
        correction = self._correction
        play_dock = self._dock
        expected_poses: Optional[list[Pose2D]] = None
        actual: Optional[OdometryPose] = None

        def _init_correction(start_idx: int = 0) -> None:
            nonlocal expected_poses, actual
            # 重基准/启用时清零纠偏积分器，避免跨航点累积陈旧误差
            self._cross_int = 0.0
            self._cross_int_t = None
            if (correction is not None and correction.enabled
                    and odo0 is not None and expected_poses is None):
                expected_poses = self._compute_expected_poses(track)
                actual = OdometryPose(self._play_wb)
                if start_idx > 0:
                    # 里程中途才到达：实际位姿从当前航点的期望位姿起算，
                    # 避免时间回放阶段走过的距离被算成纠偏误差。
                    ep = expected_poses[start_idx]
                    actual.reset(ep.x, ep.y, ep.yaw)
                if track.wheelbase_m is not None and \
                        abs(track.wheelbase_m - cfg_wb) > correction.wheelbase_tol_m:
                    logger.warning(
                        "Track '%s' was recorded with wheelbase=%.3f m but agent "
                        "is configured %.3f m — using the recorded value for "
                        "closed-loop correction",
                        track.name, track.wheelbase_m, cfg_wb,
                    )

        _init_correction()

        # 回放总安全预算：以「录制时长×4」为上限（最短 60s）。慢速但仍在推进
        # 的回放不会因此被截断——每个航点要么被里程覆盖（返回很快），要么因
        # 停滞由 _wait_odometry_target 在 ~1.5s 内快速放弃。该总预算只用来兜底
        # 「小车完全卡死」的情况：否则 1.5s×数千个航点会让回放卡上几十分钟。
        total_dur = max(track.total_duration_s, 1e-6)
        deadline_total = t0 + max(track.total_duration_s * 4.0, 60.0)

        last_idx = len(track.waypoints) - 1
        stalled_wps = 0
        timed_out = False

        def _store_stats() -> None:
            self._last_playback_had_odo = odo0 is not None
            self._last_playback_stalled_wps = stalled_wps
            self._last_playback_time_fallback = bool(has_odo and odo0 is None)

        for idx, wp in enumerate(track.waypoints):
            if self._stop_event.is_set():
                _store_stats()
                return False
            if time.monotonic() > deadline_total:
                timed_out = True
                break
            self._set_progress((time.monotonic() - t0) / total_dur)

            # 段速度一律从相邻航点轮位移反推。reversed() 后 delta 为负，
            # segment_vw 自然给出 v<0、w 取反，曲率与去程一致。
            # 轮差抖动若把倒车积成反号，_waypoint_vw 回退到已取反的快照。
            v, w = self._waypoint_vw(track, idx, wp, reverse_play)
            raw_v, raw_w = v, w
            v = max(-MAX_PLAY_LINEAR_M_S, min(MAX_PLAY_LINEAR_M_S, v))
            w = max(-MAX_PLAY_ANGULAR_RAD_S, min(MAX_PLAY_ANGULAR_RAD_S, w))
            if abs(v - raw_v) > 1e-6 or abs(w - raw_w) > 1e-6:
                clamped += 1

            # 守卫只走 _set_vel（含 grace）。这里再预检一次会在回程第一拍
            # 用车头近障把整段 reverse abort，车完全不动。
            if odo0 is None:
                # 中途再尝试拿里程：启动时 0x311 没到，先时间回放，等到帧后切换
                if has_odo:
                    odo0 = self._wait_odometer(timeout=0.3)
                    if odo0 is not None:
                        odo_rebase = odo0
                        rebase_idx = idx
                        logger.info("Odometer feedback arrived — switched to distance-driven replay")
                        _init_correction(start_idx=idx)
                if odo0 is None:
                    # Time-based fallback: sleep until this waypoint's timestamp
                    target_t = t0 + wp.t
                    sleep_s = target_t - time.monotonic()
                    if not self._hold_vel(v, w, max(0.0, sleep_s)):
                        _store_stats()
                        return False
                    continue

            # 重基准后以「当前航点」为参考：已走过的距离从目标中剔除，
            # 只剩剩余里程，避免时间回放阶段的位移被重复计算。
            ref_first = track.waypoints[rebase_idx] if rebase_idx is not None else first
            base_odo = odo_rebase if odo_rebase is not None else odo0
            if not self._set_vel(v, w):
                _store_stats()
                return False
            covered = self._wait_odometry_target(
                wp, ref_first, base_odo,
                base_v=v, base_w=w,
                expected=expected_poses[idx] if expected_poses else None,
                actual=actual,
                dock=play_dock if idx == last_idx else None,
                max_wait=deadline_total,
                expected_poses=expected_poses,
                waypoints=track.waypoints,
                wp_idx=idx,
            )
            if self._aborted_by_guard:
                _store_stats()
                return False
            if not covered:
                # 该航点未能覆盖（里程停滞/预算耗尽）：**不中止整个回放**，
                # 继续下一航点。这与原 Windows 版一致——每个航点限时尝试，
                # 超时则继续，保证回放总能走完轨迹并上报结果，而不是
                # 「回程一半就卡死、云端永远收不到 arrived」。完整性通过
                # 返回值上报，由 agent 决定怎么通知云端。
                stalled_wps += 1
                logger.warning(
                    "Waypoint %.1f s not fully covered (target %.0f/%.0f mm) — "
                    "continuing playback",
                    wp.t,
                    wp.left_mm - ref_first.left_mm,
                    wp.right_mm - ref_first.right_mm,
                )

        self._stop_vel()
        if timed_out:
            logger.warning(
                "Playback hit the total safety budget (%.0f s) before the full "
                "track was covered — reported as an interrupted replay",
                deadline_total - t0,
            )
        if stalled_wps:
            logger.warning(
                "Playback finished with %d waypoint(s) not fully covered — "
                "reported as an interrupted (incomplete) replay",
                stalled_wps,
            )
        if clamped:
            logger.warning(
                "Playback clamped %d waypoint(s) to safe limits (v≤%.2f m/s, w≤%.2f rad/s)",
                clamped, MAX_PLAY_LINEAR_M_S, MAX_PLAY_ANGULAR_RAD_S,
            )
        _store_stats()
        return not (stalled_wps > 0 or timed_out)

    def _waypoint_vw(
        self,
        track: Track,
        idx: int,
        wp: Waypoint,
        reverse_play: bool,
    ) -> tuple[float, float]:
        """本段应发的 (v, w)：优先轮位移反推，倒车反号时回退录制快照。"""
        if idx <= 0:
            return float(wp.v), float(wp.w)
        sv, sw = segment_vw(track.waypoints[idx - 1], wp, self._play_wb)
        rec_v, rec_w = float(wp.v), float(wp.w)
        if reverse_play and abs(rec_v) >= 0.03 and sv * rec_v < 0:
            return rec_v, rec_w
        return sv, sw

    def _play_recorded_timeline(
        self,
        track: Track,
        t0: float,
        deadline_total: float,
        total_dur: float,
    ) -> bool:
        """无里程时的时间轴兜底（不再作为 fb 的主路径）。

        ``Track.reversed()`` 已把 v/w 取反、时间轴翻转到从终点回家。
        主回放有 0x311 时走里程闭环；这里只按 ``wp.t`` 保持录制速度。
        """
        for wp in track.waypoints:
            if self._stop_event.is_set():
                return False
            if time.monotonic() > deadline_total:
                logger.warning(
                    "Reverse rewind hit the safety budget (%.0f s) before "
                    "the recorded timeline finished",
                    deadline_total - t0,
                )
                self._stop_vel()
                return False
            self._set_progress((time.monotonic() - t0) / total_dur)
            v = max(-MAX_PLAY_LINEAR_M_S, min(MAX_PLAY_LINEAR_M_S, wp.v))
            w = max(-MAX_PLAY_ANGULAR_RAD_S, min(MAX_PLAY_ANGULAR_RAD_S, wp.w))
            sleep_s = (t0 + wp.t) - time.monotonic()
            if not self._hold_vel(v, w, max(0.0, sleep_s)):
                return False
        self._stop_vel()
        return True

    def _wait_odometer(self, timeout: float) -> Optional[OdometerFeedback]:
        """Return the latest odometer frame within *timeout* seconds, else None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            odo = self._ctrl.latest_odometer
            if odo is not None:
                return odo
            time.sleep(0.02)
        return None

    def _wait_odometry_target(
        self,
        wp: Waypoint,
        first: Waypoint,
        odo0: OdometerFeedback,
        *,
        base_v: float = 0.0,
        base_w: float = 0.0,
        expected: Optional[Pose2D] = None,
        actual: Optional[OdometryPose] = None,
        dock: Optional[PlaybackDockConfig] = None,
        max_wait: Optional[float] = None,
        expected_poses: Optional[list[Pose2D]] = None,
        waypoints: Optional[list[Waypoint]] = None,
        wp_idx: int = 0,
    ) -> bool:
        """Hold the current command until both wheels cover the recorded
        distance (relative to the first sample).

        Returns True when the waypoint distance was covered; False when the
        waypoint was abandoned (no progress for PLAYBACK_STALL_S, or the
        waypoint / overall playback budget expired).

        关键改进（修复「r 模式按 B 回程只走一小段就卡死」）：
          * 超时预算按「剩余里程 / 当前速度 × 系数 + 常驻下限」动态估算——
            慢速重载回放不会被中途截断丢里程；
          * 预算到期但里程仍在推进时继续等待，只在**真正停滞**时才放弃；
          * 0x311 里程帧缺失或**值冻结不更新**时，用 0x221 轮速积分估算推进
            （积分以最后一次里程推进量为起点，避免已走距离被清零重算）——
            否则「里程帧在但值不动」会被误判成停滞，回程只走一小段就中止；
          * 单个航点等待设有硬上限，且受整体回放总预算（max_wait）约束，
            杜绝任何路径下无限空转。
        """
        delta_l = wp.left_mm - first.left_mm
        delta_r = wp.right_mm - first.right_mm

        # 动态预算：以剩余里程/目标速度估算所需时长，放大 + 常驻下限。
        remaining_m = ((abs(delta_l) + abs(delta_r)) / 2.0) / 1000.0
        speed = max(abs(base_v), 1e-3)
        est_s = remaining_m / speed
        budget_s = max(
            PLAYBACK_WAYPOINT_CEILING_S,
            est_s * PLAYBACK_WAYPOINT_BUDGET_SCALE + PLAYBACK_WAYPOINT_BUDGET_PAD_S,
        )
        deadline = time.monotonic() + budget_s
        waypoint_hard_deadline = time.monotonic() + PLAYBACK_HARD_WAYPOINT_S

        dock_active = dock is not None and dock.enabled
        crawl_mm = dock.crawl_start_m * 1000.0 if dock else 0.0
        stall_until = time.monotonic() + dock.dock_timeout_s if dock else 0.0
        dock_stall_since: Optional[float] = None
        dock_last_pos: Optional[tuple[float, float]] = None

        # 里程/速度积分推进检测
        last_progress_t = time.monotonic()
        last_odo_key: Optional[tuple[int, int]] = None
        # 0x311 缺失/冻结时的 0x221 轮速积分（毫米），用于判断是否仍在移动
        integral_l: float = 0.0
        integral_r: float = 0.0
        integral_seeded: bool = False
        integral_t = time.monotonic()

        while True:
            if self._stop_event.is_set():
                return False
            now = time.monotonic()
            if max_wait is not None and now > max_wait:
                return False

            odo = self._ctrl.latest_odometer
            motion = self._ctrl.latest_motion
            moved_l: Optional[float] = None
            moved_r: Optional[float] = None
            odo_fresh = False
            odo_moved_l = 0.0
            odo_moved_r = 0.0

            if odo is not None:
                key = (odo.left_wheel_mm, odo.right_wheel_mm)
                if key != last_odo_key:
                    last_odo_key = key
                    odo_fresh = True
                    last_progress_t = now
                odo_moved_l = odo.left_wheel_mm - odo0.left_wheel_mm
                odo_moved_r = odo.right_wheel_mm - odo0.right_wheel_mm
                if actual is not None:
                    actual.update(odo.left_wheel_mm, odo.right_wheel_mm)

            if odo_fresh:
                moved_l, moved_r = odo_moved_l, odo_moved_r
            elif motion is not None:
                # 0x311 缺失（或值冻结不更新）：用 0x221 左右轮速积分估算推进量。
                # 若里程帧曾推进过但中途冻结，以最后一次里程推进量为积分起点，
                # 避免已走距离被清零重算。
                wb = (self._correction.wheelbase_m if self._correction
                      else DEFAULT_WHEELBASE_M)
                v_l = motion.linear_velocity_m_s - motion.angular_velocity_rad_s * wb / 2.0
                v_r = motion.linear_velocity_m_s + motion.angular_velocity_rad_s * wb / 2.0
                dt = now - integral_t
                integral_t = now
                if not integral_seeded:
                    integral_seeded = True
                    integral_l, integral_r = (
                        (odo_moved_l, odo_moved_r) if odo is not None else (0.0, 0.0)
                    )
                if 0.0 < dt < 1.0:
                    integral_l += v_l * dt * 1000.0
                    integral_r += v_r * dt * 1000.0
                if abs(v_l) > 1e-3 or abs(v_r) > 1e-3:
                    last_progress_t = now
                moved_l, moved_r = integral_l, integral_r

            if moved_l is None or moved_r is None:
                # 既无里程帧也无轮速反馈：只能干等。超过停滞时限仍无任何反馈
                # 则放弃本航点，避免无限空转。
                if now - last_progress_t > PLAYBACK_STALL_S:
                    logger.warning(
                        "Waypoint %.1f s has no odometer/wheel feedback for %.1f s — "
                        "abandoning waypoint",
                        wp.t, PLAYBACK_STALL_S,
                    )
                    return False
                time.sleep(0.02)
                continue

            reached = _reached(delta_l, moved_l) and _reached(delta_r, moved_r)

            # 末端停靠·停稳确认阶段：只下停车命令，位移停滞才视为到位
            if reached and dock_active:
                self._stop_vel()
                if actual is None:
                    return True
                pos = (actual.pose.x, actual.pose.y)
                if dock_last_pos is None:
                    dock_last_pos = pos
                    dock_stall_since = now
                else:
                    moved = math.hypot(pos[0] - dock_last_pos[0],
                                       pos[1] - dock_last_pos[1])
                    if moved > dock.stall_moved_m:
                        dock_stall_since = now  # 仍在移动，重新计时
                    elif now - dock_stall_since >= dock.stall_s:
                        return True
                    dock_last_pos = pos
                if now >= stall_until:
                    return True  # 停稳确认超时兜底
                time.sleep(0.02)
                continue

            # 纠偏：段内插值期望位姿 → 修 w；纵向落后/超前轻度改 |v|
            v_out, w_out = base_v, base_w
            if actual is not None:
                exp = expected
                if expected_poses is not None and waypoints is not None:
                    exp = interpolate_expected_pose(
                        expected_poses, waypoints, wp_idx, first,
                        moved_l, moved_r,
                    )
                if exp is not None:
                    w_out = self._apply_correction(
                        exp, actual.pose, base_w, base_v=base_v,
                    )
                    v_out = self._along_track_v(exp, actual.pose, base_v)

            # 末端停靠：接近终点切低速爬行（前进/倒车都限幅）
            if dock_active and odo is not None and not reached:
                remaining = self._remaining_to_wp(wp, first, odo0, odo)
                if remaining is not None and remaining <= crawl_mm:
                    crawl = dock.crawl_speed_m_s
                    if v_out > 0:
                        v_out = min(v_out, crawl)
                    elif v_out < 0:
                        v_out = max(v_out, -crawl)

            if not self._set_vel(v_out, w_out):
                return False

            if reached:
                return True  # 无停靠时到达里程目标即返回

            # 停滞判定：里程+轮速积分都无推进超过阈值 → 放弃本航点
            if now - last_progress_t > PLAYBACK_STALL_S:
                logger.warning(
                    "Waypoint %.1f s stalled: no odometer/wheel progress for %.1f s "
                    "(target %.0f/%.0f mm, moved %.0f/%.0f mm)",
                    wp.t, PLAYBACK_STALL_S, delta_l, delta_r,
                    moved_l, moved_r,
                )
                return False
            if now > deadline:
                # 预算到期但仍在推进：不放弃，继续等（防慢速回放被截断），
                # 但受单航点硬上限约束，杜绝无限空转。
                if now > waypoint_hard_deadline:
                    logger.warning(
                        "Waypoint %.1f s exceeded the hard wait cap (%.0f s) — "
                        "abandoning waypoint",
                        wp.t, PLAYBACK_HARD_WAYPOINT_S,
                    )
                    return False
                deadline = now + max(PLAYBACK_WAYPOINT_CEILING_S, est_s * 1.5)
            time.sleep(0.02)

    def _remaining_to_wp(self, wp: Waypoint, first: Waypoint,
                         odo0: OdometerFeedback,
                         odo: OdometerFeedback) -> Optional[float]:
        """剩余里程（mm）到当前航点目标；无有效基准时返回 None。"""
        if odo is None:
            return None
        delta_l = wp.left_mm - first.left_mm
        delta_r = wp.right_mm - first.right_mm
        moved_l = odo.left_wheel_mm - odo0.left_wheel_mm
        moved_r = odo.right_wheel_mm - odo0.right_wheel_mm
        rem = ((delta_l - moved_l) + (delta_r - moved_r)) / 2.0
        target = (delta_l + delta_r) / 2.0
        # 倒车段目标为负：剩余「还要走多少」必须取正，否则停靠永远认为已到
        if target < 0:
            rem = -rem
        return max(rem, 0.0)

    def _apply_correction(self, expected: Pose2D, actual_pose: Pose2D,
                          base_w: float, base_v: float = 0.0) -> float:
        """横向偏差 + 航向偏差 + 积分项 → 叠加到角速度（只纠方向，不抢距离）。

        强化（消除 S 形振荡 / 恒定漂移）：
          * 航向偏差大时横偏项衰减（先转头再纠线）；
          * 横向偏差时间窗积分，补偿单侧轮滑等恒定漂移；
          * 积分窗限幅 + 输出限幅双重防 windup。
        """
        cfg = self._correction
        if cfg is None or not cfg.enabled:
            return base_w
        ex = expected.x - actual_pose.x
        ey = expected.y - actual_pose.y
        yaw = expected.yaw
        # 横向偏差（车体系左正）：位置误差在期望航向法向上的分量
        cross = -ex * math.sin(yaw) + ey * math.cos(yaw)
        heading_err = _wrap_angle(expected.yaw - actual_pose.yaw)

        # 积分项：只对横偏积分（方向量），时间窗限幅防 windup
        now = time.monotonic()
        if self._cross_int_t is None:
            dt = 0.0
        else:
            dt = now - self._cross_int_t
        self._cross_int_t = now
        if 0.0 < dt < cfg.integral_window_s:
            self._cross_int += cross * dt
        elif dt > cfg.integral_window_s:
            # 长时间断续（重基准/暂停）→ 积分清零重新累积
            self._cross_int = 0.0
        int_clamp = cfg.integral_clamp / max(cfg.cross_track_integral_gain, 1e-3)
        self._cross_int = max(-int_clamp, min(int_clamp, self._cross_int))

        # 航向优先：航向偏差大时削弱横偏项，避免互相打架
        heading_mag = abs(heading_err)
        cross_weight = 1.0
        if heading_mag > cfg.heading_priority_rad:
            cross_weight = cfg.heading_priority_decay

        # 倒车时横向纠偏反向：前进偏左应右转，倒车偏左右转会更偏
        sense = -1.0 if base_v < 0.0 else 1.0
        corr = (
            cfg.heading_gain * heading_err
            + sense * cfg.cross_track_gain * cross * cross_weight
            + sense * cfg.cross_track_integral_gain * self._cross_int
        )
        corr = max(-cfg.max_correction_rad_s,
                   min(cfg.max_correction_rad_s, corr))
        self._last_correction = corr
        return base_w + corr

    def _along_track_v(
        self, expected: Pose2D, actual_pose: Pose2D, base_v: float,
    ) -> float:
        """沿期望切向的位置误差 → 轻度加减线速度（不改符号）。

        along > 0：期望点在车前方（落后）→ 略提速；along < 0：超前 → 略减速。
        """
        cfg = self._correction
        if cfg is None or not cfg.enabled or abs(base_v) < 1e-4:
            return base_v
        ex = expected.x - actual_pose.x
        ey = expected.y - actual_pose.y
        yaw = expected.yaw
        along = ex * math.cos(yaw) + ey * math.sin(yaw)
        # 倒车：期望点在车后方（沿航向 along<0）表示还没倒够 → 应加大 |v|
        if base_v < 0.0:
            along = -along
        scale = 1.0 + cfg.along_track_gain * along
        scale = max(cfg.min_v_scale, min(cfg.max_v_scale, scale))
        v_out = base_v * scale
        limit = MAX_PLAY_LINEAR_M_S
        v_out = max(-limit, min(limit, v_out))
        # 只缩放、不改符号。旧地板 ±0.02 会把抖动的微小正速度抬成前进，
        # 倒车回程被纠成往前爬。
        if base_v > 0.0:
            return max(0.0, v_out)
        if base_v < 0.0:
            return min(0.0, v_out)
        return 0.0

    def _compute_expected_poses(self, track: Track) -> list[Pose2D]:
        """把已录制左右轮里程（0x311 累计值）用同一差速模型积成期望位姿序列。

        期望/实际都用同一模型从 (0,0,0) 起积，因此无滑移时二者逐点相等；
        位姿差即漂移/轮滑的量化，闭环纠偏据此修正。
        """
        wb = self._play_wb if self._play_wb > 0.05 else (
            self._correction.wheelbase_m if self._correction else DEFAULT_WHEELBASE_M
        )
        pose = OdometryPose(wb)
        out: list[Pose2D] = []
        for wp in track.waypoints:
            pose.update(wp.left_mm, wp.right_mm)
            out.append(Pose2D(pose.pose.x, pose.pose.y, pose.pose.yaw))
        return out
