"""VFH 几何缺口转向器测试：距离场投影、缺口提取、车宽通过性、速度自适应。"""

import pytest

from bunker_mini.vfh import VFHConfig, VFHPlanner


def test_build_histogram_projects_nearest_distance():
    vfh = VFHPlanner(VFHConfig(sector_count=72))  # 5°/sector
    dists = vfh.build_histogram([(0.0, 2.0), (2.0, 0.5), (200.0, 3.0)])
    # 2° 与 0° 同扇区 → 取最近 0.5
    assert dists[0] == pytest.approx(0.5)
    # 200° → 扇区 40，距离 3.0
    assert dists[40] == pytest.approx(3.0)
    # 其余无回波扇区 = max_range（开阔）
    assert dists[10] == vfh.config.max_range_m


def test_find_gaps_extracts_open_region():
    vfh = VFHPlanner(VFHConfig(sector_count=36, clear_distance_m=1.0,
                                max_range_m=5.0))
    dists = [5.0] * 36
    # 前方 0°~20° 被墙挡住（0.5 m），其余开阔
    for i in range(4):
        dists[i] = 0.5
    gaps = vfh.find_gaps(dists)
    assert gaps
    # 唯一大缺口应从 20°（扇区 4）延伸到 355°（扇区 35）
    assert gaps[0].min_distance_m > 1.0
    assert gaps[0].width_deg >= 300.0


def test_best_heading_steers_around_front_obstacle():
    vfh = VFHPlanner(VFHConfig(
        sector_count=72, clear_distance_m=1.0, vehicle_width_m=0.36,
        safety_margin_m=0.20))
    heading = vfh.best_heading([(0.0, 0.5)])
    assert heading is not None
    # 正前方 0.5 m 单障碍：车宽楔形要求绕开 ~49°，而非 180° 掉头或 5° 硬挤
    assert 40.0 <= abs(heading) <= 65.0


def test_best_heading_none_when_enclosed():
    vfh = VFHPlanner(VFHConfig(
        sector_count=72, clear_distance_m=1.0, vehicle_width_m=0.36,
        safety_margin_m=0.20))
    # 四周每 10° 都有 0.3 m 近障碍 → 任何航向车宽楔形内都有障碍
    sectors = [(az, 0.3) for az in range(0, 360, 10)]
    assert vfh.best_heading(sectors) is None


def test_best_heading_prefers_goal_direction():
    cfg = VFHConfig(sector_count=72, clear_distance_m=1.0, goal_bias_deg=1.0)
    vfh = VFHPlanner(cfg)
    # 前方开阔，目标在左侧 90° → 航向应明显偏左
    h_left = vfh.best_heading([], target_yaw_deg=90.0)
    assert h_left is not None and h_left > 0
    # 目标在右侧 -90° → 偏右
    h_right = vfh.best_heading([], target_yaw_deg=-90.0)
    assert h_right is not None and h_right < 0


def test_safety_distances_scale_with_speed():
    vfh = VFHPlanner()
    s0, l0 = vfh.safety_distances(0.0)
    s1, l1 = vfh.safety_distances(1.0)
    assert s1 > s0
    assert l1 > l0
    assert l1 > s1  # 限速距离始终大于急停距离（有减速过渡带）
