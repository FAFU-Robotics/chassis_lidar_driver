"""Tests for trajectory recording / playback quality optimizations.

覆盖三大优化方向：
  A. 闭环纠偏 —— TrackPlayer 期望/实际位姿差 → 只修正角速度 w
  B. 录制质量 —— 自适应采样率、v/w 轻量平滑、Track 元数据
  C. 起终点对齐与末端停靠 —— 返回前航向对齐、收尾爬行 + 停稳确认
"""

import json
import logging
import math
import threading
import time
from pathlib import Path

import pytest

from bunker_mini.navigator import OdometryPose, Pose2D
from bunker_mini.tracker import (
    TRACK_SCHEMA_VERSION,
    PlaybackCorrectionConfig,
    PlaybackDockConfig,
    Track,
    TrackPlayer,
    TrackRecorder,
    Waypoint,
    fill_vw_from_odometry,
    interpolate_expected_pose,
    sanitize_waypoints,
    segment_opposes_path,
    segment_vw,
    approach_speed,
    sanitized_track,
    segment_wait_limits,
    track_cruise_speed,
    _wheels_reached,
)
from bunker_mini.track_motion import MotionKind, choose_closer_wheels, group_motions, playback_fraction


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _odo(l, r):
    return type("O", (), {"left_wheel_mm": l, "right_wheel_mm": r})()


class _Ctrl:
    """Minimal controller fake: odometer + motion + recorded commands."""

    def __init__(self) -> None:
        self._odo = _odo(0, 0)
        self._motion = type("M", (), {
            "linear_velocity_m_s": 0.0, "angular_velocity_rad_s": 0.0,
        })()
        self.cmds: list[tuple[float, float]] = []

    @property
    def latest_odometer(self):
        return self._odo

    @property
    def latest_motion(self):
        return self._motion

    @property
    def odometer_source(self):
        return "real"

    def set_odometer(self, l: int, r: int) -> None:
        self._odo = _odo(l, r)

    def set_motion(self, v: float, w: float) -> None:
        self._motion = type("M", (), {
            "linear_velocity_m_s": v, "angular_velocity_rad_s": w,
        })()

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        self.cmds.append((0.0, 0.0))


def _straight_track(dist_mm: int = 200, v: float = 0.2) -> Track:
    n = max(2, dist_mm // 100 + 1)
    wps = [Waypoint(i / (n - 1), i * 100, i * 100, v, 0.0)
           for i in range(n)]
    return Track(name="straight", created_at="t", total_duration_s=1.0,
                 waypoints=wps)


# --------------------------------------------------------------------------
# A. 闭环纠偏
# --------------------------------------------------------------------------

class TestClosedLoopCorrection:

    def test_cross_track_sign(self):
        """实际位置在期望路径左侧 → 修正角速度为负（右转回到路径）。"""
        player = TrackPlayer(None,
                             correction=PlaybackCorrectionConfig(wheelbase_m=0.5))
        expected = Pose2D(0.0, 0.0, 0.0)
        actual_left = Pose2D(0.0, 0.1, 0.0)   # 左侧 0.1 m
        actual_right = Pose2D(0.0, -0.1, 0.0)  # 右侧 0.1 m
        assert player._apply_correction(expected, actual_left, 0.0) < 0.0
        assert player._apply_correction(expected, actual_right, 0.0) > 0.0

    def test_heading_sign(self):
        """实际航向在期望左侧 → 修正角速度为负（右转）。"""
        player = TrackPlayer(None,
                             correction=PlaybackCorrectionConfig(wheelbase_m=0.5))
        expected = Pose2D(0.0, 0.0, 0.0)
        actual = Pose2D(0.0, 0.0, 0.2)  # 航向偏左 0.2 rad
        assert player._apply_correction(expected, actual, 0.0) < 0.0

    def test_correction_only_touches_w(self):
        """只纠方向：v 不变，w 叠加修正。"""
        cfg = PlaybackCorrectionConfig(wheelbase_m=0.5)
        player = TrackPlayer(None, correction=cfg)
        expected = Pose2D(0.5, 0.0, 0.0)
        actual = Pose2D(0.45, 0.05, 0.0)  # 落后且偏左
        w_out = player._apply_correction(expected, actual, 0.3)
        assert w_out != 0.3

    def test_correction_clamped(self):
        """大偏差修正被限制在 max_correction_rad_s 内。"""
        cfg = PlaybackCorrectionConfig(wheelbase_m=0.5,
                                       max_correction_rad_s=0.3)
        player = TrackPlayer(None, correction=cfg)
        expected = Pose2D(0.0, 0.0, 0.0)
        actual = Pose2D(0.0, 2.0, 0.5)  # 极端偏差
        w = player._apply_correction(expected, actual, 0.0)
        assert abs(w) <= 0.3 + 1e-9

    def test_disabled_returns_base(self):
        player = TrackPlayer(None, correction=None)
        w = player._apply_correction(Pose2D(0.0, 0.0, 0.0),
                                     Pose2D(0.0, 1.0, 0.0), 0.3)
        assert w == 0.3

    def test_compute_expected_poses_straight(self):
        """录制里程 → 期望位姿序列（直行 0.1m/航点）。"""
        player = TrackPlayer(None,
                             correction=PlaybackCorrectionConfig(wheelbase_m=0.5))
        track = _straight_track(dist_mm=200)
        poses = player._compute_expected_poses(track)
        assert len(poses) == len(track.waypoints)
        assert poses[0].x == pytest.approx(0.0)
        assert poses[-1].x == pytest.approx(0.2, abs=1e-3)
        assert poses[-1].yaw == pytest.approx(0.0, abs=1e-6)

    def test_compute_expected_poses_turn(self):
        """转弯段：左右轮里程差产生航向变化。"""
        player = TrackPlayer(None,
                             correction=PlaybackCorrectionConfig(wheelbase_m=0.5))
        track = Track(name="turn", created_at="t", total_duration_s=1.0,
                      waypoints=[
                          Waypoint(0.0, 0, 0, 0.2, 0.0),
                          Waypoint(0.5, 100, 100, 0.2, 0.0),
                          Waypoint(1.0, 100, 200, 0.1, 0.6),  # 原地转
                      ])
        poses = player._compute_expected_poses(track)
        # 第三段 dr-dl = 100mm，wheelbase 0.5 → yaw 增 0.2 rad
        assert poses[-1].yaw == pytest.approx(0.2, abs=1e-3)

    def test_playback_with_correction_steers(self):
        """回放中实际位姿偏离 → 命令 w 与轨迹 w 不同（纠偏生效）。"""
        ctrl = _Ctrl()
        player = TrackPlayer(
            ctrl,
            correction=PlaybackCorrectionConfig(wheelbase_m=0.5),
            dock=PlaybackDockConfig(enabled=False),
        )
        track = _straight_track(dist_mm=200, v=0.2)  # 全程 w=0
        done: list[bool] = []

        def play():
            player._play_open_loop(track)
            done.append(True)

        t = threading.Thread(target=play, daemon=True)
        t.start()
        # 喂入偏离的里程：左右轮不一致 → 实际位姿产生航向/横向偏差
        for l, r in [(0, 0), (90, 110), (190, 210), (290, 310)]:
            ctrl.set_odometer(l, r)
            time.sleep(0.05)
        t.join(timeout=2.0)
        assert done
        # 至少出现过一次带纠偏的转向指令（w != 0 且 v 保持前进）
        steered = [w for v, w in ctrl.cmds if abs(v) > 0.01 and w != 0.0]
        assert steered, "闭环纠偏未向底盘下发修正转向"

    def test_bypass_guard_ignores_blocking_guard(self):
        """B 回程 / 已录轨迹：守卫硬挡也不得掐死回放。"""
        ctrl = _Ctrl()

        def always_block(v, w):
            return 0.0, 0.0, True

        player = TrackPlayer(
            ctrl,
            velocity_guard=always_block,
            correction=None,
            dock=PlaybackDockConfig(enabled=False),
        )
        track = _straight_track(dist_mm=200, v=0.2)
        done: list[bool] = []

        def play():
            done.append(player.play(track, reverse=True, bypass_guard=True))

        t = threading.Thread(target=play, daemon=True)
        t.start()
        for mm in range(0, 201, 25):
            ctrl.set_odometer(mm, mm)
            time.sleep(0.01)
        t.join(timeout=3.0)
        assert done and done[0] is True
        assert player._aborted_by_guard is False
        assert any(v != 0.0 for v, _w in ctrl.cmds)


# --------------------------------------------------------------------------
# B. 录制质量
# --------------------------------------------------------------------------

class TestRecordingQuality:

    def _make_recorder(self, ctrl, **kw) -> TrackRecorder:
        return TrackRecorder(ctrl, **kw)

    def test_adaptive_straight_is_sparse(self):
        """直行匀速：自适应采样远少于固定采样。"""
        ctrl = _Ctrl()
        adaptive = TrackRecorder(ctrl, adaptive=True,
                                 dist_threshold_mm=60.0,
                                 vel_threshold_m_s=1.0,
                                 ang_threshold_rad_s=1.0)
        fixed = TrackRecorder(ctrl, adaptive=False)
        adaptive._rec_t0 = fixed._rec_t0 = time.monotonic()
        adaptive._last_sample_t = fixed._last_sample_t = 0.0

        ctrl.set_motion(0.25, 0.0)
        for step in range(1, 21):  # 模拟 2 秒直行（每 100ms 走 25mm）
            l = r = step * 25
            ctrl.set_odometer(l, r)
            adaptive._append_sample(force=False)
            fixed._append_sample(force=False)
        assert len(adaptive._waypoints) < len(fixed._waypoints) // 2

    def test_adaptive_turn_is_dense(self):
        """转向/速度变化时自适应采样加密。"""
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, adaptive=True,
                            dist_threshold_mm=1000.0,
                            vel_threshold_m_s=0.01,
                            ang_threshold_rad_s=0.01)
        rec._rec_t0 = time.monotonic()
        rec._last_sample_t = 0.0
        ctrl.set_motion(0.1, 0.0)
        for step in range(1, 11):
            ctrl.set_odometer(step * 10, step * 10)
            ctrl.set_motion(0.1, step * 0.1)  # 角速度持续变化
            rec._append_sample(force=False)
        assert len(rec._waypoints) >= 9  # 几乎每步都采

    def test_stop_appends_final_sample(self):
        """stop() 补一个尾部采样，轨迹末端里程 = 实际最终位置。"""
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, adaptive=True, dist_threshold_mm=100.0)
        rec._rec_t0 = time.monotonic()
        rec._last_sample_t = 0.0
        # 起点采样
        ctrl.set_odometer(0, 0)
        rec._append_sample(force=True)
        # 移动 60mm（< 阈值，未采样）
        ctrl.set_odometer(60, 60)
        rec._append_sample(force=False)
        assert len(rec._waypoints) == 1
        # stop 补尾
        rec._recording = True
        rec._name = "tail"
        track = rec.stop()
        assert track.waypoints[-1].left_mm == 60

    def test_smoothing_smooths_vw(self):
        wps = [
            Waypoint(0.0, 0, 0, 0.20, 0.00),
            Waypoint(0.5, 50, 50, 0.28, 0.10),   # 抖动点
            Waypoint(1.0, 100, 100, 0.22, 0.02),
        ]
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, smoothing_window=3)
        out = rec._smooth_vw(wps)
        # 端点不变
        assert out[0].v == 0.20 and out[-1].v == 0.22
        # 中间点被加权平均（0.25*0.20 + 0.5*0.28 + 0.25*0.22 = 0.245）
        assert out[1].v == pytest.approx(0.245)
        assert out[1].w == pytest.approx(0.25 * 0.00 + 0.5 * 0.10 + 0.25 * 0.02)

    def test_idle_pause_not_sampled(self):
        """松键停住后不再堆 (0,0) 航点（kb 思考停顿不该被回放成干等）。"""
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, adaptive=True)
        rec._rec_t0 = time.monotonic()
        rec._last_sample_t = 0.0
        ctrl.set_odometer(0, 0)
        ctrl.set_motion(0.0, 0.0)
        rec._append_sample(force=True)
        for _ in range(6):
            rec._append_sample(force=True)
        assert len(rec._waypoints) == 1
        ctrl.set_motion(0.12, 0.0)
        rec._append_sample(force=False)
        assert len(rec._waypoints) == 2
        assert rec._waypoints[-1].v == pytest.approx(0.12)

    def test_motion_edge_samples_stop(self):
        """从行驶到松键：立刻落一个停车点。"""
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, adaptive=True, dist_threshold_mm=1000.0)
        rec._rec_t0 = time.monotonic()
        rec._last_sample_t = 0.0
        ctrl.set_odometer(0, 0)
        ctrl.set_motion(0.2, 0.0)
        rec._append_sample(force=True)
        ctrl.set_motion(0.0, 0.0)
        rec._append_sample(force=False)
        assert len(rec._waypoints) == 2
        assert rec._waypoints[-1].v == 0.0

    def test_constant_turn_is_dense(self):
        """匀速转弯 |Δw|=0 时仍按 ~80ms 加密，不再等到 50mm/0.4s。"""
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, adaptive=True,
                            dist_threshold_mm=1000.0,
                            vel_threshold_m_s=10.0,
                            ang_threshold_rad_s=10.0)
        rec._rec_t0 = time.monotonic()
        rec._last_sample_t = 0.0
        ctrl.set_motion(0.1, 0.2)
        ctrl.set_odometer(0, 0)
        rec._append_sample(force=True)
        for step in range(1, 6):
            time.sleep(0.09)
            ctrl.set_odometer(step * 5, step * 15)
            rec._append_sample(force=False)
        assert len(rec._waypoints) >= 5

    def test_stop_fills_metadata(self):
        ctrl = _Ctrl()
        rec = TrackRecorder(ctrl, wheelbase_m=0.42)
        rec._rec_t0 = time.monotonic()
        rec._recording = True
        rec._name = "meta"
        rec._waypoints = [
            Waypoint(0.0, 0, 0, 0.0, 0.0),
            Waypoint(0.5, 50, 50, 0.25, 0.0),
            Waypoint(1.0, 200, 200, 0.25, 0.1),
        ]
        # stop() 会补一个尾部采样；把最终里程喂给底盘（真实场景 = 车最终位置）
        ctrl.set_odometer(200, 200)
        ctrl.set_motion(0.0, 0.0)
        track = rec.stop()
        assert track.wheelbase_m == 0.42
        assert track.schema_version == TRACK_SCHEMA_VERSION
        assert track.drive_mode == "kb"
        assert track.odometer_source == "real"
        assert track.total_distance_m == pytest.approx(0.2, abs=1e-3)
        assert track.max_speed_m_s == pytest.approx(0.25)
        assert track.max_angular_rad_s == pytest.approx(0.1)
        assert track.sample_mode == "adaptive"

    def test_metadata_survives_reversed_and_json(self):
        track = Track(
            name="t", created_at="c", total_duration_s=1.0,
            waypoints=[Waypoint(0.0, 0, 0, 0.0, 0.0),
                       Waypoint(1.0, 100, 100, 0.2, 0.0)],
            schema_version=1, wheelbase_m=0.42, total_distance_m=0.5,
            max_speed_m_s=0.3, max_angular_rad_s=0.6, sample_mode="adaptive",
            odometer_source="real", drive_mode="kb",
            start_x=1.25, start_y=-0.4, start_yaw_deg=90.0,
        )
        rev = track.reversed()
        assert rev.wheelbase_m == 0.42 and rev.total_distance_m == 0.5
        assert rev.max_speed_m_s == 0.3 and rev.sample_mode == "adaptive"
        assert rev.odometer_source == "real" and rev.drive_mode == "kb"
        assert rev.start_x == 1.25 and rev.start_y == -0.4
        assert rev.start_yaw_deg == 90.0

        parsed = Track.from_json(track.to_json())
        assert parsed.wheelbase_m == 0.42
        assert parsed.total_distance_m == 0.5
        assert parsed.schema_version == 1
        assert parsed.sample_mode == "adaptive"
        assert parsed.start_x == 1.25 and parsed.start_yaw_deg == 90.0

        # 兼容无元数据的旧轨迹文件
        legacy = Track.from_json(
            {"name": "old", "created_at": "", "total_duration_s": 0.0,
             "waypoints": []})
        assert legacy.wheelbase_m is None
        assert legacy.total_distance_m is None
        assert legacy.sample_mode == "fixed"
        assert legacy.drive_mode == "unknown"
        assert legacy.odometer_source == "unknown"
        assert legacy.start_x is None and legacy.start_yaw_deg is None


# --------------------------------------------------------------------------
# C. 起终点对齐与末端停靠
# --------------------------------------------------------------------------

class TestDocking:

    def test_wheelbase_mismatch_warns(self, caplog) -> None:
        """录制 wheelbase 与 agent 配置不一致 → 告警，回放仍用轨迹轮距。"""
        ctrl = _Ctrl()
        track = Track(
            name="wb", created_at="t", total_duration_s=1.0,
            waypoints=[Waypoint(0.0, 0, 0, 0.2, 0.0),
                       Waypoint(0.5, 100, 100, 0.2, 0.0),
                       Waypoint(1.0, 200, 200, 0.2, 0.0)],
            wheelbase_m=0.42, schema_version=1, total_distance_m=0.2,
            max_speed_m_s=0.2, max_angular_rad_s=0.0,
        )
        player = TrackPlayer(
            ctrl,
            correction=PlaybackCorrectionConfig(wheelbase_m=0.8),  # 差异 > tol
            dock=PlaybackDockConfig(enabled=False),
        )
        with caplog.at_level(logging.WARNING):
            done: list[bool] = []
            t = threading.Thread(
                target=lambda: (player._play_open_loop(track), done.append(True)),
                daemon=True)
            t.start()
            for mm in range(0, 201, 25):
                ctrl.set_odometer(mm, mm)
                time.sleep(0.01)
            t.join(timeout=2.0)
        assert done
        assert any("wheelbase" in r.message for r in caplog.records)
        assert player._play_wb == pytest.approx(0.42)

    def test_dock_crawl_then_stall(self):
        """末端停靠：接近终点切低速爬行，到达后停稳确认再返回。"""
        ctrl = _Ctrl()
        dock = PlaybackDockConfig(enabled=True, crawl_speed_m_s=0.05,
                                  crawl_start_m=0.15, stall_s=0.05,
                                  stall_moved_m=0.02, dock_timeout_s=0.5)
        player = TrackPlayer(ctrl,
                             correction=PlaybackCorrectionConfig(wheelbase_m=0.5),
                             dock=dock)
        first = Waypoint(0.0, 0, 0, 0.25, 0.0)
        wp = Waypoint(1.0, 500, 500, 0.25, 0.0)  # 500mm 直行
        odo0 = _odo(0, 0)
        actual = OdometryPose(0.5)
        done: list[bool] = []

        def wait():
            player._wait_odometry_target(
                wp, first, odo0, base_v=0.25, base_w=0.0,
                actual=actual, dock=dock)
            done.append(True)

        t = threading.Thread(target=wait, daemon=True)
        t.start()
        for mm in range(0, 501, 20):
            ctrl.set_odometer(mm, mm)
            time.sleep(0.02)
        t.join(timeout=2.0)
        assert done
        vs = [c[0] for c in ctrl.cmds]
        assert 0.25 in vs               # 正常速度段
        assert 0.05 in vs               # 爬行段
        assert ctrl.cmds[-1][0] == 0.0  # 最后命令停车

    def test_no_dock_returns_on_reach(self):
        """未启用停靠：到达里程目标即返回（不额外等待停稳）。"""
        ctrl = _Ctrl()
        player = TrackPlayer(ctrl, correction=None, dock=None)
        first = Waypoint(0.0, 0, 0, 0.25, 0.0)
        wp = Waypoint(1.0, 100, 100, 0.25, 0.0)
        odo0 = _odo(0, 0)
        t0 = time.monotonic()
        ctrl.set_odometer(100, 100)
        player._wait_odometry_target(wp, first, odo0,
                                     base_v=0.25, base_w=0.0)
        assert time.monotonic() - t0 < 0.5


# --------------------------------------------------------------------------
# 回放精度：段内插值 / 轮位移反推 v/w / 纵向纠偏
# --------------------------------------------------------------------------

class TestPlaybackAccuracy:

    def test_segment_vw_from_wheel_deltas(self):
        prev = Waypoint(0.0, 0, 0, 9.0, 9.0)
        wp = Waypoint(1.0, 200, 200, 9.0, 9.0)
        v, w = segment_vw(prev, wp, 0.5)
        assert v == pytest.approx(0.2)
        assert w == pytest.approx(0.0)

    def test_segment_vw_turn(self):
        prev = Waypoint(0.0, 0, 0, 0.0, 0.0)
        wp = Waypoint(1.0, 0, 100, 0.0, 0.0)  # 右轮多走 0.1m，轮距 0.5
        v, w = segment_vw(prev, wp, 0.5)
        assert v == pytest.approx(0.05)
        assert w == pytest.approx(0.2)

    def test_fill_vw_overwrites_except_first(self):
        wps = [
            Waypoint(0.0, 0, 0, 0.0, 0.0),
            Waypoint(1.0, 100, 100, 9.0, 9.0),
        ]
        out = fill_vw_from_odometry(wps, 0.5)
        assert out[0].v == 0.0
        assert out[1].v == pytest.approx(0.1)
        assert out[1].left_mm == 100

    def test_interpolate_mid_segment(self):
        poses = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
        wps = [
            Waypoint(0.0, 0, 0, 0.2, 0.0),
            Waypoint(1.0, 1000, 1000, 0.2, 0.0),
        ]
        mid = interpolate_expected_pose(poses, wps, 1, wps[0], 500, 500)
        assert mid.x == pytest.approx(0.5)
        assert mid.y == pytest.approx(0.0)

    def test_along_track_speeds_up_when_behind(self):
        cfg = PlaybackCorrectionConfig(wheelbase_m=0.5, along_track_gain=1.0)
        player = TrackPlayer(None, correction=cfg)
        faster = player._along_track_v(
            Pose2D(0.2, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0), 0.20)
        slower = player._along_track_v(
            Pose2D(0.0, 0.0, 0.0), Pose2D(0.2, 0.0, 0.0), 0.20)
        assert faster > 0.20
        assert slower < 0.20
        assert faster > 0.0 and slower > 0.0

    def test_along_track_disabled_without_correction(self):
        player = TrackPlayer(None, correction=None)
        assert player._along_track_v(
            Pose2D(1.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0), 0.20) == 0.20

    def test_along_track_reverse_speeds_up_when_not_far_enough(self):
        """倒车：期望点在车后方（还没倒够）应加大 |v|，不得翻成前进。"""
        cfg = PlaybackCorrectionConfig(wheelbase_m=0.5, along_track_gain=1.0)
        player = TrackPlayer(None, correction=cfg)
        # heading=0，期望 x=-0.2，实际 x=0 → 沿航向 along<0，倒车应加速
        faster = player._along_track_v(
            Pose2D(-0.2, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0), -0.20)
        overshot = player._along_track_v(
            Pose2D(0.0, 0.0, 0.0), Pose2D(-0.2, 0.0, 0.0), -0.20)
        assert faster < -0.20
        assert overshot > -0.20
        assert faster < 0.0 and overshot < 0.0

    def test_along_track_does_not_floor_tiny_positive_to_crawl(self):
        cfg = PlaybackCorrectionConfig(wheelbase_m=0.5, along_track_gain=0.0)
        player = TrackPlayer(None, correction=cfg)
        # 旧逻辑把 +0.01 地板成 +0.02 前进，倒车被纠反
        v = player._along_track_v(
            Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0), -0.10)
        assert v == pytest.approx(-0.10)

    def test_segment_vw_on_reversed_track_keeps_curvature(self):
        track = Track(
            name="arc", created_at="t", total_duration_s=0.2,
            waypoints=[
                Waypoint(0.0, 0, 0, 0.15, 0.2),
                Waypoint(0.2, 20, 40, 0.15, 0.2),
            ],
        )
        fv, fw = segment_vw(track.waypoints[0], track.waypoints[1], 0.5)
        rev = track.reversed()
        rv, rw = segment_vw(rev.waypoints[0], rev.waypoints[1], 0.5)
        assert rv == pytest.approx(-fv, abs=1e-6)
        assert rw == pytest.approx(-fw, abs=1e-6)

    def test_remaining_to_wp_reverse_is_positive(self):
        player = TrackPlayer(None)
        first = Waypoint(0.0, 300, 300, -0.2, 0.0)
        wp = Waypoint(1.0, 0, 0, -0.2, 0.0)
        odo0 = _odo(1000, 1000)
        odo = _odo(850, 850)  # 已倒退 150mm，目标 300mm
        rem = player._remaining_to_wp(wp, first, odo0, odo)
        assert rem == pytest.approx(150.0)

    def test_reverse_cross_track_flips_steer(self):
        """倒车偏左应与前进偏左转向相反。"""
        cfg = PlaybackCorrectionConfig(
            wheelbase_m=0.5, heading_gain=0.0, cross_track_integral_gain=0.0,
        )
        player = TrackPlayer(None, correction=cfg)
        # 期望在原点沿 +x，车偏左 (y=0.05)
        w_fwd = player._apply_correction(
            Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.05, 0.0), 0.0, base_v=0.2)
        w_rev = player._apply_correction(
            Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.05, 0.0), 0.0, base_v=-0.2)
        assert w_fwd < 0.0
        assert w_rev > 0.0

    def test_bypass_guard_still_uses_segment_vw(self):
        """bypass_guard 只关雷达，不得退回航点快照（快照 w 与轮弧不一致）。"""
        ctrl = _Ctrl()
        player = TrackPlayer(ctrl, correction=None, dock=None)
        # 快照写 w=0，但左右轮差说明这段在转弯
        track = Track(
            name="turn", created_at="t", total_duration_s=0.2,
            waypoints=[
                Waypoint(0.0, 0, 0, 0.15, 0.0),
                Waypoint(0.2, 20, 40, 0.15, 0.0),
            ],
        )
        player._bypass_guard = True
        player._reverse_play = False
        v, w = player._waypoint_vw(track, 1, track.waypoints[1], False)
        assert w != 0.0
        assert v == pytest.approx(0.15, abs=0.02)


class TestGlitchReplay:
    def test_sanitize_drops_recovering_snap(self):
        wps = [
            Waypoint(0.0, 0, 0, 0.2, 0.0),
            Waypoint(0.4, 80, 80, 0.2, 0.0),
            Waypoint(0.5, 20, 0, -0.5, 1.2),
            Waypoint(0.9, 160, 160, 0.2, 0.0),
            Waypoint(1.3, 240, 240, 0.2, 0.0),
        ]
        out = sanitize_waypoints(wps, 0.5)
        assert all(p.left_mm != 20 for p in out)
        assert all(p.v >= -0.02 for p in out[1:])
        assert not any(segment_opposes_path(out, i) for i in range(len(out)))

    def test_sustained_reverse_is_kept(self):
        wps = [
            Waypoint(0.0, 0, 0, 0.2, 0.0),
            Waypoint(1.0, 400, 400, 0.2, 0.0),
            Waypoint(2.0, 300, 300, -0.2, 0.0),
            Waypoint(3.0, 200, 200, -0.2, 0.0),
            Waypoint(4.0, 100, 100, -0.2, 0.0),
        ]
        out = sanitize_waypoints(wps, 0.5)
        assert any(p.left_mm == 100 for p in out)
        assert any(p.v < -0.05 for p in out)

    def test_straight_snap_does_not_command_reverse_or_spin(self):
        ctrl = _Ctrl()
        player = TrackPlayer(
            ctrl,
            correction=PlaybackCorrectionConfig(wheelbase_m=0.5),
            dock=PlaybackDockConfig(enabled=False),
        )
        track = Track(
            name="straight", created_at="t", total_duration_s=1.3,
            waypoints=[
                Waypoint(0.0, 0, 0, 0.2, 0.0),
                Waypoint(0.4, 80, 80, 0.2, 0.0),
                Waypoint(0.5, 20, 0, -0.5, 1.2),
                Waypoint(0.9, 160, 160, 0.2, 0.0),
                Waypoint(1.3, 240, 240, 0.2, 0.0),
            ],
            odometer_source="real",
            wheelbase_m=0.5,
        )
        done: list[bool] = []

        def play():
            done.append(player._play_open_loop(track))

        t = threading.Thread(target=play, daemon=True)
        t.start()
        for mm in (0, 80, 160, 240, 280):
            ctrl.set_odometer(mm, mm)
            time.sleep(0.04)
        t.join(timeout=3.0)
        assert done
        bad = [(v, w) for v, w in ctrl.cmds if v < -0.05 or abs(w) > 0.25]
        assert not bad, bad

    def test_already_reached_waypoint_sends_no_velocity(self):
        ctrl = _Ctrl()
        player = TrackPlayer(ctrl, correction=None, dock=None)
        first = Waypoint(0.0, 0, 0, -0.5, 1.0)
        wp = Waypoint(0.2, 100, 100, -0.5, 1.0)
        ctrl.set_odometer(100, 100)
        ok = player._wait_odometry_target(
            wp, first, _odo(0, 0), base_v=-0.5, base_w=1.0)
        assert ok is True
        assert ctrl.cmds == []

    def test_moving_away_abandons_waypoint(self):
        ctrl = _Ctrl()
        player = TrackPlayer(ctrl, correction=None, dock=None)
        first = Waypoint(0.0, 0, 0, 0.2, 0.0)
        wp = Waypoint(1.0, 200, 200, 0.2, 0.0)
        ctrl.set_odometer(-40, -40)
        t0 = time.monotonic()
        ok = player._wait_odometry_target(
            wp, first, _odo(0, 0), base_v=-0.4, base_w=0.8)
        elapsed = time.monotonic() - t0
        assert ok is False
        assert elapsed < 4.0


def _track_json(name: str) -> str:
    return json.dumps({
        "name": name,
        "created_at": "t",
        "total_duration_s": 1.0,
        "waypoints": [
            {"t": 0.0, "left_mm": 0, "right_mm": 0, "v": 0.0, "w": 0.0},
            {"t": 1.0, "left_mm": 100, "right_mm": 100, "v": 0.1, "w": 0.0},
        ],
    })


class TestTrackDeleteSearch:
    def _dirs(self):
        import tempfile
        from pathlib import Path
        raw = tempfile.mkdtemp()
        tmp_path = Path(raw)
        primary = tmp_path / "bunker_jetson" / "tracks"
        other = tmp_path / "tracks"
        primary.mkdir(parents=True)
        other.mkdir()
        return primary, other

    def test_load_and_delete_only_in_second_dir(self):
        primary, other = self._dirs()
        (other / "路径1.json").write_text(_track_json("路径1"), encoding="utf-8")
        player = TrackPlayer(None, track_dir=str(primary))
        loaded = player.load_track("路径1")
        assert loaded.name == "路径1"
        deleted = player.delete_track("路径1")
        assert "路径1.json" in deleted
        assert not (other / "路径1.json").exists()
        try:
            player.load_track("路径1")
        except FileNotFoundError:
            return
        raise AssertionError("deleted track should not load")

    def test_delete_removes_both_copies(self):
        primary, other = self._dirs()
        (primary / "r1.json").write_text(_track_json("r1"), encoding="utf-8")
        (other / "r1.json").write_text(_track_json("r1"), encoding="utf-8")
        player = TrackPlayer(None, track_dir=str(primary))
        player.delete_track("r1")
        assert not (primary / "r1.json").exists()
        assert not (other / "r1.json").exists()
        assert not player.track_exists("r1")


class TestStraightPrecision:
    def test_approach_slows_inside_window(self):
        assert approach_speed(0.2, 1.0) == pytest.approx(0.2)
        assert approach_speed(0.2, 0.02) == pytest.approx(0.1)
        assert approach_speed(0.2, 0.0) == 0.0

    def test_wait_brakes_before_the_end(self):
        ctrl = _Ctrl()
        player = TrackPlayer(ctrl, correction=None, dock=None)
        first = Waypoint(0.0, 0, 0, 0.2, 0.0)
        wp = Waypoint(1.0, 100, 100, 0.2, 0.0)
        ctrl.set_odometer(90, 90)

        def wait():
            player._wait_odometry_target(
                wp, first, _odo(0, 0), base_v=0.2, base_w=0.0, final_leg=True)

        t = threading.Thread(target=wait, daemon=True)
        t.start()
        time.sleep(0.08)
        player._stop_event.set()
        t.join(timeout=1.0)
        assert ctrl.cmds
        assert abs(ctrl.cmds[0][0]) < 0.12

    def test_long_segment_is_not_cut_off_at_one_and_a_half_seconds(self):
        """r3 这种约 9 m 的直线，计数停 1.5 秒不能结束整段回程。"""
        _budget, hard, stall = segment_wait_limits(8.9, 0.17)
        assert stall > 1.5
        assert hard > 20.0
        short_budget, short_hard, short_stall = segment_wait_limits(0.05, 0.1)
        assert short_stall == 1.5
        assert short_hard >= 20.0
        assert short_budget < hard

    def test_odometer_snap_does_not_erase_wheel_speed_progress(self):
        """0x311 停在 1.4 m 时，0x221 已经积到更远，进度不能被拉回去。"""
        chosen = choose_closer_wheels(
            "translate", 8882, 8910, [(1400, 1380), (3200, 3180)],
        )
        assert chosen == (3200, 3180)
        assert playback_fraction(0, 8900) == 0.0
        assert playback_fraction(4450, 8900) == pytest.approx(0.5, abs=0.01)

    def test_r4_straight_reverse_has_zero_yaw_rate(self):
        """轮差只有约 1.6° 的长直线，倒放角速度为 0，不再持续转向。"""
        path = Path(__file__).resolve().parents[1] / "tracks" / "r4.json"
        track = sanitized_track(Track.from_json(path.read_text(encoding="utf-8")))
        motions = [
            seg for seg in group_motions(track.reversed().waypoints, 0.5)
            if seg.kind == MotionKind.TRANSLATE
        ]
        assert motions
        longest = max(motions, key=lambda seg: seg.end_idx - seg.start_idx)
        assert longest.v < -0.1
        assert longest.w == 0.0
        # 轮差明显的弯仍保留角速度。
        curve = group_motions(
            [
                Waypoint(0.0, 0, 0, 0.0, 0.0),
                Waypoint(2.0, 400, 200, 0.15, -0.2),
            ],
            0.5,
        )
        assert curve[-1].kind == MotionKind.TRANSLATE
        assert curve[-1].w != 0.0

    def test_reverse_uses_cruise_not_stop_speed(self):
        track = Track(
            name="line", created_at="t", total_duration_s=0.7,
            waypoints=[
                Waypoint(0.0, 0, 0, 0.0, 0.0),
                Waypoint(0.5, 100, 100, 0.2, 0.0),
                Waypoint(0.7, 110, 110, 0.05, 0.0),
            ],
            wheelbase_m=0.5,
        )
        rev = track.reversed()
        player = TrackPlayer(None, correction=None, dock=None)
        player._reverse_cruise = track_cruise_speed(rev.waypoints, 0.5)
        assert player._reverse_cruise == pytest.approx(0.2, abs=0.03)
        v, w = player._waypoint_vw(rev, 1, rev.waypoints[1], True)
        assert v < -0.15
        assert abs(w) < 0.05

    def test_odometer_rebase_keeps_reverse_speed_negative(self):
        """r2：转弯中轮计数从约 800 mm 跳回几毫米，倒放不得变成前进。"""
        wps = [
            Waypoint(0.0, 0, 0, 0.0, 0.0),
            Waypoint(7.96, 793, 319, 0.10, -0.40),
            Waypoint(8.06, 4, 1, -5.5, 9.4),
            Waypoint(8.17, 25, 1, 0.10, -0.40),
            Waypoint(8.40, 50, 10, 0.10, -0.20),
        ]
        cleaned = sanitize_waypoints(wps, 0.5)
        for i in range(1, len(cleaned)):
            v, w = segment_vw(cleaned[i - 1], cleaned[i], 0.5)
            assert abs(v) < 1.0, (cleaned[i].t, v, w)
            assert abs(w) < 2.0, (cleaned[i].t, v, w)
        jump = next(wp for wp in cleaned if abs(wp.t - 8.06) < 0.02)
        prev = cleaned[cleaned.index(jump) - 1]
        assert abs(jump.left_mm - prev.left_mm) < 5
        assert abs(jump.right_mm - prev.right_mm) < 5
        track = Track(
            name="r2", created_at="t", total_duration_s=cleaned[-1].t,
            waypoints=cleaned, wheelbase_m=0.5,
        )
        motions = [
            seg for seg in group_motions(track.reversed().waypoints, 0.5)
            if seg.kind != MotionKind.IDLE
        ]
        assert motions
        for seg in motions:
            assert abs(seg.v) < 1.0, (seg.v, seg.w)
            assert abs(seg.w) < 2.0, (seg.v, seg.w)
            if seg.kind == MotionKind.TRANSLATE:
                assert seg.v < 0.0, (seg.v, seg.w)

    def test_slight_positive_wheel_delta_stays_reverse(self):
        wps = [
            Waypoint(0.0, 0, 0, -0.2, 0.0),
            Waypoint(0.2, -40, -40, -0.2, 0.0),
            Waypoint(0.4, -30, -30, 0.05, 0.0),
            Waypoint(0.8, 80, 80, 0.2, 0.0),
        ]
        player = TrackPlayer(None, correction=None, dock=None)
        player._play_wb = 0.5
        player._reverse_cruise = 0.15
        track = Track(
            name="t", created_at="t", total_duration_s=0.8,
            waypoints=wps, wheelbase_m=0.5,
        )
        v_back, _ = player._waypoint_vw(track, 1, wps[1], True)
        v_small, _ = player._waypoint_vw(track, 2, wps[2], True)
        v_fwd, _ = player._waypoint_vw(track, 3, wps[3], True)
        assert v_back < 0.0
        assert v_small < 0.0
        assert v_fwd > 0.0

    def test_saved_r2_jump_does_not_reverse_into_forward(self):
        path = Path(__file__).resolve().parents[1] / "tracks" / "r2.json"
        track = sanitized_track(Track.from_json(path.read_text(encoding="utf-8")))
        for i in range(1, len(track.waypoints)):
            v, w = segment_vw(track.waypoints[i - 1], track.waypoints[i], 0.5)
            assert abs(v) < 1.0, (track.waypoints[i].t, v, w)
            assert abs(w) < 2.0, (track.waypoints[i].t, v, w)
        rev = sanitized_track(track.reversed())
        motions = [
            seg for seg in group_motions(rev.waypoints, 0.5)
            if seg.kind != MotionKind.IDLE
        ]
        assert motions[0].kind == MotionKind.SPIN
        assert abs(motions[0].v) < 0.05
        assert abs(motions[0].w) == pytest.approx(0.4, abs=0.08)
        straight = next(seg for seg in motions if seg.kind == MotionKind.TRANSLATE)
        assert straight.v < 0.0

    def test_reverse_turns_in_place_before_backing_up(self):
        """先直线再原地转：倒放第一段应是转头，不是直线倒车。"""
        wps = [Waypoint(0.0, 0, 0, 0.0, 0.0)]
        for i in range(1, 9):
            wps.append(Waypoint(i * 0.5, i * 50, i * 50, 0.10, 0.0))
        base_l, base_r = wps[-1].left_mm, wps[-1].right_mm
        t0 = wps[-1].t
        for i in range(1, 9):
            wps.append(Waypoint(
                t0 + i * 0.1,
                base_l + i * 10,
                base_r - i * 10,
                0.0,
                -0.40,
            ))
        track = Track(
            name="spin", created_at="t", total_duration_s=wps[-1].t,
            waypoints=wps, wheelbase_m=0.5,
        )
        rev = track.reversed()
        motions = [
            seg for seg in group_motions(rev.waypoints, 0.5)
            if seg.kind != MotionKind.IDLE
        ]
        assert motions[0].kind == MotionKind.SPIN
        assert abs(motions[0].v) < 0.03
        assert abs(motions[0].w) > 0.15
        straight = [seg for seg in motions if seg.kind == MotionKind.TRANSLATE]
        assert straight and all(seg.v < -0.05 for seg in straight)
        # 平均轮程不变时，车还没转，不能算到位。
        assert not _wheels_reached(-80, 80, 0, 0)
        assert _wheels_reached(-80, 80, -80, 80)
        assert _wheels_reached(100, 100, 110, 90)
