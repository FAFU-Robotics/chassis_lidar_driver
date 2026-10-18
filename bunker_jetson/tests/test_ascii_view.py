"""点云俯视图渲染（ascii_view）测试。"""

import math
import os

import pytest

from bunker_mini.ascii_view import (
    render_top_down_ascii,
    save_occupancy_png,
    save_point_cloud_png,
)
from bunker_mini.occupancy import BLOCKED, FREE, OCCUPIED, OccupancyGrid


def test_render_top_down_ascii_centers_car():
    text = render_top_down_ascii([(0.0, 0.0)])
    assert "@" in text
    lines = [ln for ln in text.splitlines()[1:] if "@" in ln]
    assert lines, "车体 @ 应出现在某一行"
    assert text.splitlines()[1].startswith("前")
    assert any(ln.startswith("左") and ln.rstrip().endswith("右") for ln in text.splitlines())


def test_render_top_down_ascii_density_glyphs():
    """同一格内多点累积成高密度符号。"""
    pts = [(0.15, 0.0)] * 20  # 20 点都落在 col=1,row=0
    text = render_top_down_ascii(pts, cell_m=0.1)
    lines = [ln for ln in text.splitlines()[1:] if ln.strip()]
    assert any("#" in ln for ln in lines)
    assert not any("o" in ln for ln in lines)  # 应全部合并进高密度格


def test_render_top_down_ascii_out_of_range_ignored():
    text = render_top_down_ascii([(100.0, 100.0)])
    assert "@" in text


def test_render_top_down_ascii_colors_wraps_glyphs():
    """colors=True 时高密度/中等/稀疏格带 ANSI 颜色码，车体为白色。"""
    pts = [(0.15, 0.0)] * 20  # 同格 20 点 → '#'
    pts += [(0.25, 0.0)] * 4  # 另一格 4 点 → 'o'
    pts += [(0.35, 0.0)] * 1  # 另一格 1 点 → '.'
    colored = render_top_down_ascii(pts, cell_m=0.1, colors=True)
    assert "\033[" in colored
    assert "\033[0m" in colored
    assert "\033[91m" in colored      # 红 = 密集 '#'
    assert "\033[93m" in colored      # 黄 = 中等 'o'
    assert "\033[92m" in colored      # 绿 = 稀疏 '.'
    assert "\033[97m" in colored      # 白 = 车体 '@'
    # 纯文本模式不受影响（默认）
    plain = render_top_down_ascii(pts, cell_m=0.1)
    assert "\033[" not in plain


def test_save_point_cloud_png(tmp_path):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    pts = [[x, math.sin(x) * 0.5, 0.3, 100] for x in range(-50, 51)]
    path = save_point_cloud_png(pts, prefix="test_pc", out_dir=str(tmp_path))
    if path is None:
        pytest.skip("matplotlib 不可用")
    assert os.path.isfile(path)
    assert path.startswith(str(tmp_path))


def test_save_point_cloud_png_empty_returns_none(tmp_path):
    assert save_point_cloud_png([], out_dir=str(tmp_path)) is None


def test_save_occupancy_png_empty_returns_none(tmp_path):
    assert save_occupancy_png(OccupancyGrid(), out_dir=str(tmp_path)) is None


def test_save_occupancy_png_writes_file(tmp_path):
    pytest.importorskip("matplotlib")
    grid = OccupancyGrid(resolution_m=0.1)
    grid.import_map([
        (0.05, 0.05, FREE),
        (0.15, 0.05, OCCUPIED),
        (0.25, 0.05, BLOCKED),
    ])
    path = save_occupancy_png(
        grid, prefix="test_occ", out_dir=str(tmp_path), pose=(0.05, 0.05, 0.0),
    )
    if path is None:
        pytest.skip("matplotlib 不可用")
    assert os.path.isfile(path)
    assert path.startswith(str(tmp_path))
    assert os.path.getsize(path) > 0
