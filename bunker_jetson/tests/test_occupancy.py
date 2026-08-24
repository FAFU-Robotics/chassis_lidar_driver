"""OccupancyGrid 2D 伪地图（射线栅格化 + 地形不可通行标记）测试。"""

import pytest

from bunker_mini.lidar import AccumulatingSectors
from bunker_mini.occupancy import BLOCKED, OCCUPIED, OccupancyGrid


def test_front_sector_marks_obstacle_and_free_ray():
    """正前方障碍：边界格占用，边界前射线格自由，更远处未知。"""
    g = OccupancyGrid(resolution_m=0.1, max_range_m=12.0)
    # yaw=0 车头朝世界 +x；方位角 0°（正前方）2 m 处有障碍
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    assert g.cell_state(2.0, 0.0) == "occupied"
    assert g.cell_state(1.0, 0.0) == "free"
    assert g.cell_state(3.0, 0.0) == "unknown"


def test_blocked_sector_marks_non_traversable():
    """地形不可通行扇区：边界格 blocked，边界以内仍可通行。"""
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[], blocked_sectors=[(0.0, 1.0)])
    assert g.cell_state(1.0, 0.0) == "blocked"
    assert g.cell_state(0.5, 0.0) == "free"


def test_update_transforms_with_yaw():
    """车体方位角注册：yaw=90°（车头朝世界 +y）时正前方障碍落在世界 (0, 1)。"""
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 90.0, sectors=[(0.0, 1.0)])
    assert g.cell_state(0.0, 1.0) == "occupied"
    # 本代码库方位角约定 x = d·sin(az)，即 az=90° 是车体系右侧；
    # yaw=90°（朝 +y）时右侧 = 世界 +x
    g2 = OccupancyGrid(resolution_m=0.1)
    g2.update(0.0, 0.0, 90.0, sectors=[(90.0, 1.0)])
    assert g2.cell_state(1.0, 0.0) == "occupied"


def test_check_target_reports_state_and_nearest_obstacle():
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    r = g.check_target(0.5, 0.0)
    assert r["state"] == "free"
    assert r["nearestObstacle"] is not None
    assert 1.0 <= r["nearestObstacle"] <= 1.6
    r2 = g.check_target(2.0, 0.0)
    assert r2["state"] == "occupied"
    assert r2["stateZh"] == "有障碍"
    assert r["stateZh"] == "可以走"


def test_snapshot_and_ascii_text():
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 0.0,
             sectors=[(0.0, 2.0)],
             blocked_sectors=[(90.0, 1.0)])
    snap = g.snapshot(0.0, 0.0, 0.0)
    assert snap["resolution"] == 0.1
    assert snap["occupiedCells"] >= 1
    assert snap["blockedCells"] >= 1
    assert snap["freeCells"] >= 1
    assert snap["online"] is True
    assert "@" in snap["text"]
    assert "▲" in snap["text"]
    # 90°（左侧）在 yaw=0 时对应世界 -y
    assert g.cell_state(0.0, -1.0) == "blocked"
    # 目标点预检附加字段
    snap2 = g.snapshot(0.0, 0.0, 0.0)
    assert "targetCheck" not in snap2


def test_snapshot_include_free_returns_free_cells():
    """include_free=True 时快照附带自由格坐标（全局地图模式渲染用）。"""
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    default = g.snapshot(0.0, 0.0, 0.0)
    assert default["free"] == []
    snap = g.snapshot(0.0, 0.0, 0.0, include_free=True)
    assert len(snap["free"]) == snap["freeCells"] >= 1
    # 自由格坐标格式 [x, y]（世界系、格中心）
    assert all(len(p) == 2 for p in snap["free"])
    # 障碍边界格不进自由格
    assert g.cell_state(2.0, 0.0) == "occupied"
    assert [2.0, 0.0] not in snap["free"]


def test_ascii_text_is_vehicle_oriented():
    """俯视图以车头朝上：车正前方的障碍应显示在车体上方。"""
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 1.0)])
    text = g.as_text(0.0, 0.0, 0.0)
    assert "前" in text and "后" in text and "左" in text
    lines = [ln for ln in text.splitlines()[1:] if ln.strip()]
    car_row = next(i for i, ln in enumerate(lines) if "@" in ln)
    # 车头▲在车体上一行；正前方 1m 的障碍在车体上方的某一行
    assert "▲" in lines[car_row - 1]
    assert any("#" in ln for ln in lines[:car_row])


def test_ascii_text_rotates_with_yaw():
    """yaw 旋转后，车正前方的障碍依然显示在车体上方。"""
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 90.0, sectors=[(0.0, 1.0)])
    text = g.as_text(0.0, 0.0, 90.0)
    lines = [ln for ln in text.splitlines()[1:] if ln.strip()]
    car_row = next(i for i, ln in enumerate(lines) if "@" in ln)
    assert "▲" in lines[car_row - 1]
    assert any("#" in ln for ln in lines[:car_row])


def test_summary_and_reset():
    g = OccupancyGrid(resolution_m=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    s = g.summary()
    assert s["occupiedCells"] >= 1
    assert s["freeCells"] >= 1
    assert s["online"] is True
    g.reset()
    assert g.cell_count == 0
    assert g.cell_state(1.0, 0.0) == "unknown"
    assert g.summary()["online"] is False


def test_stale_occupied_cells_expire_after_ttl():
    """时间衰减：障碍消失后（不再被观测到）其占用格在 TTL 后被清除。"""
    import time

    g = OccupancyGrid(resolution_m=0.1, ttl_s=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 1.0)])
    assert g.cell_state(1.0, 0.0) == "occupied"
    time.sleep(0.25)
    # 新一轮观测：该方向不再有障碍（无扇区回波）→ 占用格过期清除
    g.update(0.0, 0.0, 0.0, sectors=[])
    assert g.cell_state(1.0, 0.0) == "unknown"
    # 车体自身格仍在（每次 update 重新观测）
    assert g.cell_state(0.0, 0.0) == "free"


def test_free_ray_does_not_revive_stale_occupied_cell():
    """自由射线扫过占用格不得为其「续命」——障碍移走后残留格照样过期。"""
    import time

    g = OccupancyGrid(resolution_m=0.1, ttl_s=0.1)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    assert g.cell_state(2.0, 0.0) == "occupied"
    time.sleep(0.25)
    # 障碍移到 3.0 m：射线仍穿过旧占用格 (2.0, 0)，但该格是「占用」而非
    # 「自由」，不应被续命 → 过期后清除；新障碍格 (3.0, 0) 正常保留
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 3.0)])
    assert g.cell_state(2.0, 0.0) == "unknown"
    assert g.cell_state(3.0, 0.0) == "occupied"


def test_set_ttl_extends_decay_window():
    """find_object 探路期间拉长 TTL 后，刚扫过的走廊不会 5s 内被清掉。"""
    import time

    g = OccupancyGrid(resolution_m=0.1, ttl_s=0.05)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    assert g.cell_state(2.0, 0.0) == "occupied"
    g.set_ttl(2.0)
    time.sleep(0.2)
    g.update(0.0, 0.0, 0.0, sectors=[(90.0, 1.0)])
    assert g.cell_state(2.0, 0.0) == "occupied"


def test_imported_map_cells_are_permanent():
    """map_upload 导入的全局地图格不参与时间衰减（权威地图数据）。"""
    import time

    g = OccupancyGrid(resolution_m=0.1, ttl_s=0.1)
    g.import_map([(2.0, 0.0, OCCUPIED), (0.0, 2.0, BLOCKED)])
    time.sleep(0.25)
    g.update(0.0, 0.0, 0.0, sectors=[(0.0, 1.0)])
    assert g.cell_state(2.0, 0.0) == "occupied"
    assert g.cell_state(0.0, 2.0) == "blocked"


def test_accumulating_sectors_occupied_polar():
    """累积扇区图暴露 p25 极坐标边界（伪地图的数据源）。"""
    a = AccumulatingSectors(120)
    a.add(0.0, 1.0)
    a.add(0.0, 2.0)
    a.add(0.0, 3.0)
    pts = a.occupied_polar()
    assert len(pts) == 1
    az, d = pts[0]
    assert abs(az) < 3.0
    assert d == pytest.approx(1.0)  # p25 of [1, 2, 3] = 1.0
