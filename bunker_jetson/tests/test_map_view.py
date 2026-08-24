"""Occupancy Matplotlib 可视化：坐标变换与 OccupancyGrid.update 对齐。"""

import math
import os

import pytest

from bunker_mini.map_view import (
    cells_to_image,
    map_view_should_start,
    probe_local_display,
    sectors_to_world,
    vehicle_to_world,
)
from bunker_mini.occupancy import OCCUPIED, OccupancyGrid


def test_vehicle_to_world_yaw0_forward_is_plus_x():
    wx, wy = vehicle_to_world(0.0, 0.0, 0.0, vx=0.0, vy=2.0)
    assert wx == pytest.approx(2.0)
    assert wy == pytest.approx(0.0)


def test_vehicle_to_world_yaw0_right_is_minus_y():
    wx, wy = vehicle_to_world(0.0, 0.0, 0.0, vx=1.5, vy=0.0)
    assert wx == pytest.approx(0.0)
    assert wy == pytest.approx(-1.5)


def test_sectors_to_world_matches_occupancy_update_hits():
    """扇区投影落点必须落在 OccupancyGrid.update 标成 occupied 的格上。"""
    pose_x, pose_y, yaw_deg = 1.2, -0.4, 35.0
    sectors = [(0.0, 3.0), (90.0, 2.0), (-45.0, 1.5)]
    grid = OccupancyGrid(resolution_m=0.1, ttl_s=0.0)
    grid.update(pose_x, pose_y, yaw_deg, sectors)

    pts = sectors_to_world(
        pose_x, pose_y, yaw_deg, sectors, max_range_m=grid.max_range_m,
    )
    assert len(pts) == 3
    for wx, wy in pts:
        assert grid.cell_state(wx, wy) == "occupied"
        cx, cy = grid.world_to_cell(wx, wy)
        assert grid.cell_at(cx, cy) == OCCUPIED


def test_sectors_to_world_skips_non_positive_and_over_range():
    pts = sectors_to_world(
        0.0, 0.0, 0.0,
        [(0.0, 0.0), (10.0, -1.0), (0.0, 20.0), (0.0, 2.0)],
        max_range_m=12.0,
    )
    assert len(pts) == 1
    assert pts[0][0] == pytest.approx(2.0)
    assert pts[0][1] == pytest.approx(0.0)


def test_cells_to_image_extent_matches_png_convention():
    grid = OccupancyGrid(resolution_m=0.1, ttl_s=0.0)
    grid.import_map([(0.05, 0.05, 1), (0.15, 0.05, 2)])
    built = cells_to_image(grid.iter_cells(), grid.resolution_m, margin_cells=0)
    assert built is not None
    img, extent = built
    assert extent == (0.0, 0.2, 0.0, 0.1)
    assert img[0][0] == [255, 255, 255]  # FREE
    assert img[0][1] == [0, 0, 0]        # OCCUPIED


def test_cells_to_image_empty_is_none():
    assert cells_to_image([], 0.1) is None


def test_map_view_should_start_env(monkeypatch):
    monkeypatch.setenv("BUNKER_MAP_VIEW", "0")
    assert map_view_should_start() is False
    monkeypatch.setenv("BUNKER_MAP_VIEW", "1")
    assert map_view_should_start() is True


def test_probe_local_display_keeps_existing(monkeypatch):
    monkeypatch.setenv("DISPLAY", ":99")
    assert probe_local_display() == ":99"


def test_yaw90_forward_is_plus_y():
    """yaw=90° 车头沿 +y：正前方 1 m → 世界 (0, 1)。"""
    wx, wy = sectors_to_world(0.0, 0.0, 90.0, [(0.0, 1.0)])[0]
    assert wx == pytest.approx(0.0, abs=1e-9)
    assert wy == pytest.approx(1.0, abs=1e-9)
    # 与 OccupancyGrid 同一公式的旋转项
    assert math.hypot(wx, wy) == pytest.approx(1.0)
