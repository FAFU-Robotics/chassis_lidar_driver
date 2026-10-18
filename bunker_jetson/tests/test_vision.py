"""Stage-3 vision tests: reflectivity target detection, coordinate mapping,
and the grasp-approach state machine."""

import math
import time

import pytest

from bunker_mini.approach import ApproachConfig, ApproachController, ApproachState
from bunker_mini.lidar import LidarPoint
from bunker_mini.vision import TargetEstimate, ReflectivityDetector, target_to_odom


def _pt(az_deg: float, vert_deg: float, dist_m: float,
        refl: int = 230, z: float | None = None, ch: int = 1) -> LidarPoint:
    if z is None:
        z = dist_m * math.sin(math.radians(vert_deg))
    xy = dist_m * math.cos(math.radians(vert_deg))
    az_r = math.radians(az_deg)
    return LidarPoint(
        azimuth_deg=az_deg, vertical_deg=vert_deg, distance_m=dist_m,
        reflectivity=refl, channel=ch,
        x=xy * math.sin(az_r), y=xy * math.cos(az_r), z=z,
    )


# ---------------------------------------------------------------------------
# ReflectivityDetector
# ---------------------------------------------------------------------------


def test_detects_high_reflectivity_cluster():
    det = ReflectivityDetector(threshold=180, min_points=3)
    # 目标圆柱：正前方 1.0 m，占方位 358°~4°（跨 0° 边界），6 个高反射点
    pts = [
        _pt(az, -2.0, 1.0, refl=235, z=0.3)
        for az in [357.0, 358.0, 359.0, 1.0, 2.0, 3.0]
    ]
    # 低反射干扰（碎石/地面）
    pts += [_pt(30.0, -8.0, 0.4, refl=90), _pt(45.0, -8.0, 0.5, refl=110)]
    ests = det.detect(pts)
    assert len(ests) == 1
    e = ests[0]
    # 方位圆形平均 ~ 0°；距离取近端 1.0
    assert abs(e.bearing_deg - 0.0) < 2.0 or abs(e.bearing_deg - 360.0) < 2.0
    assert e.distance_m == pytest.approx(1.0, abs=0.01)
    assert e.point_count == 6
    assert e.reflectivity == 235


def test_low_reflectivity_points_ignored():
    det = ReflectivityDetector(threshold=180)
    pts = [_pt(0.0, -8.0, 0.5, refl=90), _pt(10.0, -8.0, 0.6, refl=110)]
    assert det.detect(pts) == []


def test_mount_yaw_shifts_bearing():
    det = ReflectivityDetector(threshold=180, mount_yaw_deg=90.0)
    pts = [_pt(0.0, -2.0, 1.0, refl=230, z=0.3) for _ in range(4)]
    ests = det.detect(pts)
    assert len(ests) == 1
    assert abs(ests[0].bearing_deg - 90.0) < 2.0  # 雷达 0° + 安装偏置 90°


def test_insufficient_points_rejected():
    det = ReflectivityDetector(threshold=180, min_points=4)
    pts = [_pt(0.0, -2.0, 1.0, refl=230, z=0.3) for _ in range(3)]
    assert det.detect(pts) == []


# ---------------------------------------------------------------------------
# 目标半径估计 & 尺寸合理性过滤（防误检大石/岩壁高亮/扬尘）
# ---------------------------------------------------------------------------


def test_radius_estimated_from_azimuth_span():
    det = ReflectivityDetector(threshold=180)
    # 圆柱目标：方位跨度 ~12°（354°~6°），距离 1.0 m → 半径 ≈ 1.0·tan(6°) ≈ 0.105
    pts = [
        _pt(az, -2.0, 1.0, refl=235, z=0.3)
        for az in [354.0, 356.0, 358.0, 2.0, 4.0, 6.0]
    ]
    ests = det.detect(pts)
    assert len(ests) == 1
    e = ests[0]
    assert e.radius_estimate_m == pytest.approx(0.105, abs=0.02)
    assert e.center_distance_m == pytest.approx(1.0 + e.radius_estimate_m, abs=0.02)


def test_wide_bright_region_rejected_as_wall_highlight():
    det = ReflectivityDetector(threshold=180, max_az_width_deg=30.0)
    # 整面亮岩壁：方位跨度 120°（大量命中）→ 不应被当作目标
    pts = [
        _pt(az, -2.0, 1.0, refl=235, z=0.3)
        for az in range(0, 121, 3)
    ]
    assert det.detect(pts) == []


def test_scattered_bright_points_rejected_as_dust():
    det = ReflectivityDetector(threshold=180, max_dist_spread_m=0.6)
    # 同一方位但距离差 1.0 m → 散落亮点（扬尘），不是紧致目标
    pts = [
        _pt(0.0, -2.0, 0.4, refl=235, z=0.1),
        _pt(1.0, -2.0, 1.4, refl=235, z=0.2),
        _pt(2.0, -2.0, 0.8, refl=235, z=0.1),
    ]
    assert det.detect(pts) == []


# ---------------------------------------------------------------------------
# target_to_odom
# ---------------------------------------------------------------------------


def test_target_to_odom_straight_ahead():
    # 位姿原点、车头朝 0°、目标正前方 2 m → (2, 0)
    est = ReflectivityDetector().detect(
        [_pt(0.0, -2.0, 2.0, refl=230, z=0.3) for _ in range(4)]
    )[0]
    gx, gy = target_to_odom(0.0, 0.0, 0.0, est)
    assert gx == pytest.approx(2.0, abs=0.05)
    assert gy == pytest.approx(0.0, abs=0.05)


def test_target_to_odom_with_pose_offset():
    # 位姿 (1, 2, yaw=90°=π/2)，目标正前方 1.5 m（车体系 bearing=0）
    est = type("E", (), {"bearing_deg": 0.0, "distance_m": 1.5})()
    gx, gy = target_to_odom(1.0, 2.0, math.pi / 2, est)
    # 车头朝 +y，正前方即 (1, 2+1.5)
    assert gx == pytest.approx(1.0, abs=1e-6)
    assert gy == pytest.approx(3.5, abs=1e-6)


def test_target_to_odom_bearing_offset():
    # 车头 0°，目标 bearing=90°（正左）→ 里程系 (-1, 0) 方向（yaw 右=+x 左=-x）
    est = type("E", (), {"bearing_deg": 90.0, "distance_m": 1.0})()
    gx, gy = target_to_odom(0.0, 0.0, 0.0, est)
    assert gx == pytest.approx(0.0, abs=1e-6)
    assert gy == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# ApproachController
# ---------------------------------------------------------------------------


def _est(bearing_deg: float, dist_m: float, radius: float = 0.0):
    return TargetEstimate(bearing_deg=bearing_deg, distance_m=dist_m,
                          radius_estimate_m=radius, height_m=0.3,
                          name="t", source="test")


def test_approach_aligns_then_approaches_then_ready():
    cfg = ApproachConfig(approach_distance_m=0.5, stabilize_s=0.05,
                         align_deadband_deg=3.0)
    app = ApproachController(cfg)
    app.start()

    # 目标偏左 10° → ALIGNING，原地左转（w>0）
    v, w = app.update(_est(10.0, 1.2))
    assert app.state == ApproachState.ALIGNING
    assert v == 0.0 and w > 0.0

    # 对准后仍远 → APPROACHING，前进 + 微调
    v, w = app.update(_est(1.0, 1.2))
    assert app.state == ApproachState.APPROACHING
    assert v > 0.0

    # 到位 → STABILIZING（静止），等 stabilize_s 后 READY
    v, w = app.update(_est(0.0, 0.5))
    assert app.state == ApproachState.STABILIZING
    assert v == 0.0 and w == 0.0
    # 轮询等待（单次 sleep 在满载下易抖动）直到 READY
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        v, w = app.update(_est(0.0, 0.5))
        if app.state == ApproachState.READY:
            break
        time.sleep(0.01)
    assert app.state == ApproachState.READY


def test_approach_target_lost_fails_after_timeout():
    cfg = ApproachConfig(target_lost_timeout_s=0.1)
    app = ApproachController(cfg)
    app.start()
    # 目标丢失：SEARCHING，原地旋转
    v, w = app.update(None)
    assert app.state == ApproachState.SEARCHING
    assert v == 0.0 and w > 0.0
    time.sleep(0.12)
    v, w = app.update(None)
    assert app.state == ApproachState.FAILED


def test_approach_target_reacquired_after_loss():
    cfg = ApproachConfig(target_lost_timeout_s=5.0, align_deadband_deg=3.0)
    app = ApproachController(cfg)
    app.start()
    app.update(None)  # SEARCHING
    v, w = app.update(_est(30.0, 1.0))  # 重新出现，偏 30°
    assert app.state == ApproachState.ALIGNING


def test_approach_abort():
    app = ApproachController(ApproachConfig())
    app.start()
    app.update(_est(5.0, 1.0))
    app.stop()
    v, w = app.update(_est(5.0, 1.0))
    assert app.state == ApproachState.ABORTED
    assert v == 0.0 and w == 0.0


def test_approach_uses_target_center_distance():
    """带半径的目标：按中心距离逼近，表面更近时就该停车（防止够不着）。"""
    cfg = ApproachConfig(approach_distance_m=0.5, distance_tolerance_m=0.05,
                         stabilize_s=0.05)
    app = ApproachController(cfg)
    app.start()
    # 表面 0.45 m / 半径 0.15 → 中心 0.60 m > 0.5 → 仍在逼近
    v, w = app.update(_est(0.0, 0.45, radius=0.15))
    assert app.state == ApproachState.APPROACHING
    assert v > 0.0
    # 表面 0.35 m / 半径 0.15 → 中心 0.50 m ≤ 0.5+0.05 → 停稳
    v, w = app.update(_est(0.0, 0.35, radius=0.15))
    assert app.state == ApproachState.STABILIZING
    assert v == 0.0 and w == 0.0


def test_approach_near_field_blind_zone_completes_ready():
    """近场盲区完成兜底：已进入机械臂工作区后目标丢失 → 不应 FAILED。"""
    cfg = ApproachConfig(approach_distance_m=0.5,
                         blind_zone_complete_m=0.7, hold_grace_s=0.05,
                         target_lost_timeout_s=1.0)
    app = ApproachController(cfg)
    app.start()
    # 最后一次有效观测：中心 0.55 m（进入工作区）
    app.update(_est(0.0, 0.40, radius=0.15))  # 中心 0.55
    # 随后目标落入雷达盲区消失 → 近场完成路径
    v, w = app.update(None)
    assert app.state == ApproachState.STABILIZING
    assert v == 0.0 and w == 0.0
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        v, w = app.update(None)
        if app.state == ApproachState.READY:
            break
        time.sleep(0.01)
    assert app.state == ApproachState.READY


def test_approach_near_field_out_of_workspace_still_searches():
    """距目标还远时丢失 → 照常 SEARCHING，不误判盲区完成。"""
    cfg = ApproachConfig(blind_zone_complete_m=0.7)
    app = ApproachController(cfg)
    app.start()
    app.update(_est(0.0, 2.0))  # 中心 2.0 m，未进入工作区
    v, w = app.update(None)
    assert app.state == ApproachState.SEARCHING
    assert v == 0.0 and w > 0.0
