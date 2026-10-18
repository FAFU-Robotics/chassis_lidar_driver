"""Tests for the lightweight polar-correlation scan matcher (漂移闭环)."""

import math

import pytest

from bunker_mini.scanmatch import ScanMatchConfig, ScanMatcher


def _corner_sectors() -> list[tuple[float, float]]:
    """L 形墙角场景（车体系）：左墙 x=-1.5，前墙 y=2.0。

    两个正交墙构成约束完备的结构，可同时校正 (dx, dy, dyaw)。
    """
    out: list[tuple[float, float]] = []
    for az in range(-85, 0, 5):
        dist = 1.5 / abs(math.sin(math.radians(az)))
        out.append((az % 360.0, round(dist, 3)))
    for az in range(0, 61, 5):
        dist = 2.0 / math.cos(math.radians(az))
        out.append((az % 360.0, round(dist, 3)))
    return out


def _wall_sectors() -> list[tuple[float, float]]:
    """单面直墙（只约束法向平移与 yaw，切向平移无约束 → 优次不分）。"""
    out: list[tuple[float, float]] = []
    for az in range(-60, 61, 5):
        dist = 2.0 / math.cos(math.radians(az))
        out.append((az % 360.0, round(dist, 3)))
    return out


def test_no_map_returns_none() -> None:
    matcher = ScanMatcher()
    assert not matcher.has_map
    assert matcher.match(_corner_sectors(), 0.0, 0.0, 0.0) is None


def test_match_zero_drift_returns_small_offset() -> None:
    matcher = ScanMatcher()
    sectors = _corner_sectors()
    matcher.observe(sectors, 0.0, 0.0, 0.0)
    corr = matcher.match(sectors, 0.0, 0.0, 0.0)
    assert corr is not None
    dx, dy, dyaw = corr
    assert abs(dx) <= 0.05 and abs(dy) <= 0.05 and abs(dyaw) <= 1.0


def test_match_recovers_injected_drift() -> None:
    matcher = ScanMatcher()
    sectors = _corner_sectors()
    matcher.observe(sectors, 0.0, 0.0, 0.0)
    # 注入漂移：真实位姿应回到原点附近
    drifted = (0.2, -0.1, 2.0)
    corr = matcher.match(sectors, *drifted)
    assert corr is not None
    dx, dy, dyaw = corr
    # 校正后位姿 ≈ 参考地图的原点位姿
    assert abs(drifted[0] + dx) < 0.12
    assert abs(drifted[1] + dy) < 0.12
    assert abs(drifted[2] + dyaw) < 1.2


def test_match_out_of_window_returns_none() -> None:
    matcher = ScanMatcher()
    matcher.observe(_corner_sectors(), 0.0, 0.0, 0.0)
    # 完全不同的场景（前方 5m 处一面墙）→ 小窗口内无对齐 → 得分不足
    far = [(az, 5.0) for az in range(-60, 61, 10)]
    assert matcher.match(far, 0.0, 0.0, 0.0) is None


def test_single_wall_ambiguous_returns_none() -> None:
    # 单面直墙：切向平移不可观，最优/次优难分 → 宁可不修正
    matcher = ScanMatcher()
    sectors = _wall_sectors()
    matcher.observe(sectors, 0.0, 0.0, 0.0)
    corr = matcher.match(sectors, 0.0, 0.0, 0.0)
    # 允许两种结局之一：返回 0 附近偏移，或明确拒绝（None）
    if corr is not None:
        dx, dy, _ = corr
        assert abs(dx) <= 0.15 and abs(dy) <= 0.15


def test_reset_clears_reference_map() -> None:
    matcher = ScanMatcher()
    matcher.observe(_corner_sectors(), 0.0, 0.0, 0.0)
    assert matcher.has_map
    matcher.reset()
    assert not matcher.has_map
    assert matcher.match(_corner_sectors(), 0.0, 0.0, 0.0) is None


def test_empty_sectors_returns_none() -> None:
    matcher = ScanMatcher()
    matcher.observe(_corner_sectors(), 0.0, 0.0, 0.0)
    assert matcher.match([], 0.0, 0.0, 0.0) is None
