"""把清理后的航点收成停车、直行、原地转，并回答这段该发多少、到了没有。

播放器只问这里，不再对每个约 0.1 秒的采样单独改速度符号。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class MotionKind(str, Enum):
    IDLE = "idle"
    TRANSLATE = "translate"
    SPIN = "spin"


class _WheelPoint(Protocol):
    t: float
    left_mm: int
    right_mm: int


@dataclass(frozen=True)
class MotionSegment:
    kind: MotionKind
    start_idx: int
    end_idx: int
    v: float
    w: float


_DEFAULT_WHEELBASE_M: float = 0.5
# 短于这个的左右轮合计位移视为停车，不单独开一段速度。
_IDLE_STEP_MM: float = 8.0
# 原地转：左右轮反向，中心位移小于约 1.5 cm，且小于轮差的三分之一。
_SPIN_CENTER_MM: float = 15.0
# 相对本段起点，每侧都要走大约这么多，才要求左右轮各自到位。
_SPIN_EACH_WHEEL_MM: float = 8.0
# 整段航向变化小于约 3° 视为直线，不再把轮差当成要一直追的转向。
_STRAIGHT_YAW_RAD: float = 3.0 * math.pi / 180.0


def is_inplace_spin(dl: float, dr: float) -> bool:
    """这一拍是原地转，不是带一点轮差的直行。"""
    dl = float(dl)
    dr = float(dr)
    if dl * dr >= 0.0:
        return False
    center = abs(dl + dr) / 2.0
    spread = abs(dl - dr)
    return center < _SPIN_CENTER_MM and center * 3.0 < spread


def step_kind(dl: float, dr: float) -> MotionKind:
    if abs(float(dl)) + abs(float(dr)) < _IDLE_STEP_MM:
        return MotionKind.IDLE
    if is_inplace_spin(dl, dr):
        return MotionKind.SPIN
    return MotionKind.TRANSLATE


def _command(
    kind: MotionKind,
    dl_mm: float,
    dr_mm: float,
    dt: float,
    wheelbase_m: float,
) -> tuple[float, float]:
    if kind == MotionKind.IDLE or dt < 1e-3:
        return 0.0, 0.0
    wb = wheelbase_m if wheelbase_m > 0.05 else _DEFAULT_WHEELBASE_M
    dl = float(dl_mm) / 1000.0
    dr = float(dr_mm) / 1000.0
    w = (dr - dl) / wb / dt
    if kind == MotionKind.SPIN:
        return 0.0, w
    v = (dl + dr) / 2.0 / dt
    # 整段只偏几度的直行：左右轮同一速度，不把这点轮差变成持续转向。
    if abs(dr - dl) / wb < _STRAIGHT_YAW_RAD:
        return v, 0.0
    return v, w


def _glue_idle_gaps(kinds: list[MotionKind]) -> None:
    """一段动作中间夹一个停车采样，仍算同一段，避免直线被切碎。"""
    for i in range(1, len(kinds) - 1):
        if kinds[i] != MotionKind.IDLE:
            continue
        if kinds[i - 1] == kinds[i + 1] and kinds[i - 1] != MotionKind.IDLE:
            kinds[i] = kinds[i - 1]


def group_motions(
    waypoints: list[_WheelPoint],
    wheelbase_m: float,
) -> list[MotionSegment]:
    """连续同一种动作收成一段。速度用整段轮差，不看中间某一个采样。"""
    if len(waypoints) < 2:
        return []
    kinds = [
        step_kind(
            waypoints[i].left_mm - waypoints[i - 1].left_mm,
            waypoints[i].right_mm - waypoints[i - 1].right_mm,
        )
        for i in range(1, len(waypoints))
    ]
    _glue_idle_gaps(kinds)
    out: list[MotionSegment] = []
    start = 0
    for i in range(1, len(kinds) + 1):
        if i < len(kinds) and kinds[i] == kinds[start]:
            continue
        end_idx = i  # 这一拍结束所在的航点
        kind = kinds[start]
        a = waypoints[start]
        b = waypoints[end_idx]
        v, w = _command(
            kind,
            b.left_mm - a.left_mm,
            b.right_mm - a.right_mm,
            float(b.t) - float(a.t),
            wheelbase_m,
        )
        out.append(MotionSegment(
            kind=kind,
            start_idx=start,
            end_idx=end_idx,
            v=v,
            w=w,
        ))
        start = i
    return out


def _each_wheel(delta_l: float, delta_r: float) -> bool:
    delta_l = float(delta_l)
    delta_r = float(delta_r)
    if delta_l * delta_r >= 0.0:
        return False
    return abs(delta_l) >= _SPIN_EACH_WHEEL_MM and abs(delta_r) >= _SPIN_EACH_WHEEL_MM


def _wheel_met(delta: float, moved: float) -> bool:
    if delta >= 0.0:
        return moved >= delta
    return moved <= delta


def _mean_met(delta_l: float, delta_r: float, moved_l: float, moved_r: float) -> bool:
    target = (float(delta_l) + float(delta_r)) / 2.0
    moved = (float(moved_l) + float(moved_r)) / 2.0
    if target >= 0.0:
        return moved >= target
    return moved <= target


def motion_reached(
    kind: MotionKind | str,
    delta_l: float,
    delta_r: float,
    moved_l: float,
    moved_r: float,
) -> bool:
    """直行看平均轮程。原地转必须左右轮各自走到本段终点。"""
    if kind == MotionKind.SPIN or _each_wheel(delta_l, delta_r):
        return _wheel_met(delta_l, moved_l) and _wheel_met(delta_r, moved_r)
    return _mean_met(delta_l, delta_r, moved_l, moved_r)


def _shortfall(delta: float, moved: float) -> float:
    if delta >= 0.0:
        return max(0.0, float(delta) - float(moved))
    return max(0.0, float(moved) - float(delta))


def motion_shortfall(
    kind: MotionKind | str,
    delta_l: float,
    delta_r: float,
    moved_l: float,
    moved_r: float,
) -> float:
    """还差多少毫米。原地转用左右轮各自剩余之和，避免平均剩余一直是 0。"""
    if kind == MotionKind.SPIN or _each_wheel(delta_l, delta_r):
        return _shortfall(delta_l, moved_l) + _shortfall(delta_r, moved_r)
    target = (float(delta_l) + float(delta_r)) / 2.0
    moved = (float(moved_l) + float(moved_r)) / 2.0
    if target >= 0.0:
        return max(0.0, target - moved)
    return max(0.0, moved - target)


def segment_travel_mm(kind: MotionKind | str, dl: float, dr: float) -> float:
    """这一段要走的路程（毫米）。直行看中心，原地转看两侧轮程。"""
    dl = float(dl)
    dr = float(dr)
    if kind == MotionKind.SPIN or _each_wheel(dl, dr):
        return (abs(dl) + abs(dr)) / 2.0
    return abs(dl + dr) / 2.0


def choose_closer_wheels(
    kind: MotionKind | str,
    delta_l: float,
    delta_r: float,
    options: list[tuple[float, float]],
) -> tuple[float, float]:
    """在几种轮程估计里留下更靠近目标的那一对。

    0x311 跳回旧计数时，不能把 0x221 已经积出来的路程清掉。
    """
    best_l, best_r = options[0]
    best = motion_shortfall(kind, delta_l, delta_r, best_l, best_r)
    for moved_l, moved_r in options[1:]:
        cover = motion_shortfall(kind, delta_l, delta_r, moved_l, moved_r)
        if cover < best:
            best = cover
            best_l, best_r = moved_l, moved_r
    return best_l, best_r


def playback_fraction(done_mm: float, total_mm: float) -> float:
    """已走路程 / 整条轨迹路程，限制在 0 到 1。"""
    if total_mm <= 1.0:
        return 1.0 if done_mm > 0.5 else 0.0
    return max(0.0, min(1.0, float(done_mm) / float(total_mm)))
