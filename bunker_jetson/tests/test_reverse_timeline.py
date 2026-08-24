#!/usr/bin/env python3
"""fb 回程必须走里程闭环，不能按时间轴开环倒放。"""
from __future__ import annotations

import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bunker_mini.tracker import (
    PlaybackCorrectionConfig,
    Track,
    TrackPlayer,
    Waypoint,
    segment_vw,
)


class _Ctrl:
    def __init__(self, left: int = 0, right: int = 0) -> None:
        self.cmds: list[tuple[float, float]] = []
        self._left = left
        self._right = right
        self.latest_motion = type("M", (), {
            "linear_velocity_m_s": 0.0, "angular_velocity_rad_s": 0.0,
        })()
        self.odometer_source = "real"

    @property
    def latest_odometer(self):
        return type("O", (), {
            "left_wheel_mm": self._left, "right_wheel_mm": self._right,
        })()

    def set_odometer(self, left: int, right: int) -> None:
        self._left = left
        self._right = right

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def set_velocity_now(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        self.cmds.append((0.0, 0.0))


def _straight(n: int = 5, step_mm: int = 15, v: float = 0.10) -> Track:
    wps = [
        Waypoint(t=i * 0.15, left_mm=i * step_mm, right_mm=i * step_mm, v=v, w=0.0)
        for i in range(n)
    ]
    return Track(
        name="r1", created_at="t", total_duration_s=(n - 1) * 0.15, waypoints=wps,
    )


def main() -> None:
    track = _straight()
    rev = track.reversed()
    # reversed 段速度应为负，且与轮位移一致（曲率保持）
    v, w = segment_vw(rev.waypoints[0], rev.waypoints[1], 0.5)
    assert v < -0.05, f"reversed segment_vw 应倒车, got v={v}"
    assert abs(v + 0.10) < 0.03, f"reversed 段速度应约 -0.10, got {v}"
    assert abs(w) < 0.05, f"直行倒车 w 应接近 0, got {w}"

    start_mm = 60
    ctrl = _Ctrl(start_mm, start_mm)
    player = TrackPlayer(
        ctrl,
        correction=PlaybackCorrectionConfig(wheelbase_m=0.5),
        dock=None,
    )

    done: list[bool] = []

    def _play() -> None:
        done.append(player.play(rev, reverse=True, bypass_guard=True))

    t = threading.Thread(target=_play, daemon=True)
    t.start()
    # 按录制轮程倒退：从 60mm 回到 0。时间轴开环会在 ~0.6s 内假装走完
    # 且不等里程；闭环必须等轮子真正覆盖。
    for mm in range(start_mm, -1, -5):
        ctrl.set_odometer(mm, mm)
        time.sleep(0.02)
    t.join(timeout=8.0)
    assert done, "回程未返回"
    assert done[0] is True, "里程闭环回程应完整覆盖"
    moving = [v for v, _w in ctrl.cmds if abs(v) > 1e-6]
    assert moving, f"回程未下发速度: {ctrl.cmds}"
    assert all(v < 0.0 for v in moving), f"回程应倒车，得到 {moving}"
    # 段速度约 -0.10，允许纠偏缩放，但不得被地板成 ±0.02 或翻成前进
    assert all(v < -0.04 for v in moving), f"回程不得被纠偏地板成蠕行: {moving}"
    print("PASS reverse uses odometry + segment_vw, not timeline snapshots")


if __name__ == "__main__":
    main()
