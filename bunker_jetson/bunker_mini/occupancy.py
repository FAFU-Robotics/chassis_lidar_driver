"""2D occupancy pseudo-map built from the Airy LiDAR — cloud-side goto planning aid.

把雷达「累积 360° 障碍扇区图」和「地形通过性判定（台阶/岩壁/坑/坡）」
投影成一张 2D 栅格图（occupancy grid）：

  * 每个被占用的扇区沿射线做「自由 → 占用」栅格化（ray casting），
    车体与障碍之间的无障碍走廊被标记为自由；
  * 地形判定「不可通行」的扇区（台阶/岩壁/坑沿）边界被标记为 blocked（X），
    直接回答云端「哪个位置不能走」；
  * 随小车运动，把每一帧按里程计位姿注册到**里程系全局坐标**，
    地图逐步展开成一张累计伪地图——云端据此设计 ``goto`` 目标点更安全，
    agent 的 ``goto`` 入口也会用这张图做目标点预检；
  * **时间衰减**：雷达实时观测格若在 TTL（``PSEUDO_MAP_TTL_S``，默认 5 s）
    内未被再次观测到会被清除，避免历史瞬时观测（人走动/障碍移动/安装时
    杂物/标定前错位点云）永久残留、误拒 ``goto``——实时避障守卫本就看
    滑动窗口，伪地图必须与之保持一致。通过 ``map_upload`` 导入的全局
    地图格是永久的（权威数据），不参与衰减。

纯 Python，零第三方依赖。坐标系与 ``navigator.OdometryPose`` 一致：
车体系 x 右 / y 前 / 方位角 0°=车头、左转为正；里程系 yaw=0 时车头沿
世界 +x，逆时针为正。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

# 栅格单元取值语义（合并时取最大值）
FREE = 1      # 该格被雷达射线扫过、确认可通行
OCCUPIED = 2  # 该格存在障碍（雷达边界）
BLOCKED = 3   # 该格地形不可通行（台阶/岩壁/坑，超过 step_limit）

_STATE_NAMES = {0: "unknown", FREE: "free", OCCUPIED: "occupied", BLOCKED: "blocked"}
_STATE_NAMES_CN = {
    "unknown": "没扫到",
    "free": "可以走",
    "occupied": "有障碍",
    "blocked": "过不去",
}
_SYMBOLS = {0: " ", FREE: ".", OCCUPIED: "#", BLOCKED: "X"}

# 单次地图快照最多返回的占用/不可通行单元格数（超出按抽样截断，防报文过大）
SNAPSHOT_CELL_CAP: int = 1500

# 伪地图单元保留时长（秒）：``update`` 写入的单元若超过该时长未被再次观测
# 到，会被清除（时间衰减）。防止早期/瞬时观测（人走动、障碍移动、安装时
# 杂物、标定前错位点云）被永久烧录——否则小车停在原地时这些陈旧格会一直
# 挡住 ``goto`` 预检（实时避障守卫看的是滑动窗口，两者会互相矛盾）。
# 通过 ``map_upload`` 导入的全局地图单元是永久的，不受此限制。
PSEUDO_MAP_TTL_S: float = 5.0


class OccupancyGrid:
    """Global-frame 2D occupancy grid accumulated from LiDAR scans.

    用法::

        grid = OccupancyGrid()
        grid.update(pose.x, pose.y, pose.yaw_deg,
                    sectors=lidar.sector_points(),
                    blocked_sectors=blocked)
        state = grid.check_target(2.0, 1.5)["state"]   # free/blocked/unknown
        data = grid.snapshot(pose.x, pose.y, pose.yaw_deg)
    """

    def __init__(
        self,
        resolution_m: float = 0.1,
        max_range_m: float = 12.0,
        free_margin_m: float = 0.15,
        ttl_s: float = PSEUDO_MAP_TTL_S,
    ) -> None:
        if resolution_m <= 0 or max_range_m <= 0:
            raise ValueError("resolution_m / max_range_m must be > 0")
        self._res = float(resolution_m)
        self._max_range = float(max_range_m)
        # 自由射线在障碍前多少距离收住，避免障碍边界格被同时标成自由
        self._margin = float(free_margin_m)
        # 时间衰减：单元最近一次「写入该状态」的时刻（``import_map`` 的
        # 全局地图格为永久 ``float('inf')``）。仅凭「被自由射线扫过」不算
        # 重新观测——否则障碍移走后其残留格会被后续自由射线无限续命。
        self._ttl_s = float(ttl_s)
        self._touch: dict[tuple[int, int], float] = {}
        self._cells: dict[tuple[int, int], int] = {}
        self._lock = threading.Lock()
        self._updated_at = 0.0

    # -- basic ----------------------------------------------------------

    @property
    def resolution_m(self) -> float:
        return self._res

    @property
    def max_range_m(self) -> float:
        return self._max_range

    @property
    def updated_at(self) -> float:
        return self._updated_at

    @property
    def cell_count(self) -> int:
        with self._lock:
            return len(self._cells)

    @property
    def ttl_s(self) -> float:
        """当前时间衰减窗口（秒）；≤0 表示不衰减。"""
        return self._ttl_s

    def set_ttl(self, ttl_s: float) -> None:
        """运行时切换时间衰减窗口。

        ``find_object`` 探路期间把 TTL 拉长到探路时限，避免 5 s 默认窗口
        把刚扫过的溶洞走廊清掉，导致 A* 对着空地图直线穿墙。任务结束
        后应恢复 ``PSEUDO_MAP_TTL_S``。``ttl_s <= 0`` 关闭衰减。
        """
        with self._lock:
            self._ttl_s = float(ttl_s)

    def reset(self) -> None:
        with self._lock:
            self._cells.clear()
            self._touch.clear()
            self._updated_at = 0.0

    def import_map(self, cells: list[tuple[float, float, int]]) -> int:
        """导入外部栅格地图（云端上传的 SLAM/全局地图通道）。

        ``cells``: [(world_x, world_y, value)]，value 取 FREE/OCCUPIED/
        BLOCKED 常量。合并语义与 ``_mark`` 一致（取最大值），导入后刷新
        ``updated_at``。**导入的单元是永久的**（全局地图权威数据，不参与
        时间衰减），只有雷达实时观测格会随时间清除。返回实际落格的单元数。
        """
        with self._lock:
            for wx, wy, value in cells:
                if value <= 0:
                    continue
                self._mark(wx, wy, int(value), permanent=True)
            if cells:
                self._updated_at = time.time()
        return len(cells)

    # -- register -------------------------------------------------------

    def update(
        self,
        pose_x: float,
        pose_y: float,
        yaw_deg: float,
        sectors: list[tuple[float, float]],
        blocked_sectors: Optional[list[tuple[float, float]]] = None,
    ) -> None:
        """Register one LiDAR observation into the global odometry frame.

        ``sectors``        障碍边界 [(azimuth_deg, distance_m)]，车体系，
                           来自累积扇区图（``lidar.sector_points()``）。
        ``blocked_sectors``地形「不可通行」边界（同格式），来自地形剖面。
        """
        yaw = math.radians(yaw_deg)
        siny, cosy = math.sin(yaw), math.cos(yaw)

        def to_global(vx: float, vy: float) -> tuple[float, float]:
            # 车体系 (x 右 / y 前) → 里程系：前向=世界 (cos yaw, sin yaw)
            return (pose_x + vx * siny + vy * cosy,
                    pose_y - vx * cosy + vy * siny)

        def ray(vx: float, vy: float) -> None:
            """沿射线以 cell 步长标记自由，障碍边界格标占用。"""
            length = math.hypot(vx, vy)
            if length < self._res * 0.5:
                return
            n = int(length / self._res)
            stop_t = max(0.0, (length - self._margin)) / length
            for i in range(1, n + 1):
                t = i / n
                if t >= stop_t:
                    break
                self._mark(*to_global(vx * t, vy * t), FREE)
            self._mark(*to_global(vx, vy), OCCUPIED)

        with self._lock:
            # 车体自身格：已知可通行
            self._mark(pose_x, pose_y, FREE)
            for az_deg, dist in sectors:
                if dist <= 0 or dist > self._max_range:
                    continue
                a = math.radians(az_deg)
                ray(dist * math.sin(a), dist * math.cos(a))
            for az_deg, dist in blocked_sectors or []:
                if dist <= 0 or dist > self._max_range:
                    continue
                a = math.radians(az_deg)
                vx, vy = dist * math.sin(a), dist * math.cos(a)
                ray(vx, vy)
                # 覆盖为「不可通行」（BLOCKED > OCCUPIED，取最大值保留）
                self._mark(*to_global(vx, vy), BLOCKED)
            self._prune_expired_locked(time.monotonic())
            self._updated_at = time.time()

    # -- query ----------------------------------------------------------

    def cell_state(self, x: float, y: float) -> str:
        """Traversability of a global target point (free/blocked/occupied/unknown)."""
        cx, cy = self._world_to_cell(x, y)
        with self._lock:
            val = self._cells.get((cx, cy), 0)
        return _STATE_NAMES.get(val, "unknown")

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """世界坐标 → 格坐标（供全局路径规划 / 可视化复用）。"""
        return self._world_to_cell(x, y)

    def cell_at(self, cx: int, cy: int) -> int:
        """格坐标直接查值（0=unknown / FREE / OCCUPIED / BLOCKED）。"""
        with self._lock:
            return self._cells.get((cx, cy), 0)

    def iter_cells(self) -> list[tuple[tuple[int, int], int]]:
        """所有已知格 [(格坐标, 值)] 的只读快照（供路径规划膨胀/寻路）。"""
        with self._lock:
            return list(self._cells.items())

    def check_target(self, x: float, y: float) -> dict:
        """Pre-check a ``goto`` target: state + nearest obstacle distance."""
        cx, cy = self._world_to_cell(x, y)
        with self._lock:
            val = self._cells.get((cx, cy), 0)
            nearest = self._nearest_locked(x, y, {OCCUPIED, BLOCKED})
        return {
            "x": round(x, 2),
            "y": round(y, 2),
            "cell": [cx, cy],
            "state": _STATE_NAMES.get(val, "unknown"),
            "stateZh": _STATE_NAMES_CN.get(
                _STATE_NAMES.get(val, "unknown"), "没扫到"),
            "nearestObstacle": round(nearest, 2) if nearest is not None else None,
        }

    def summary(self) -> dict:
        """Light-weight status for periodic state reports."""
        with self._lock:
            occ = sum(1 for v in self._cells.values() if v == OCCUPIED)
            blk = sum(1 for v in self._cells.values() if v == BLOCKED)
            free = sum(1 for v in self._cells.values() if v == FREE)
        return {
            "online": self._updated_at > 0,
            "resolution": self._res,
            "occupiedCells": occ,
            "blockedCells": blk,
            "freeCells": free,
            "updatedAt": round(self._updated_at, 1),
        }

    def snapshot(
        self,
        pose_x: float,
        pose_y: float,
        yaw_deg: float,
        *,
        online: bool = True,
        cap: int = SNAPSHOT_CELL_CAP,
        include_free: bool = False,
    ) -> dict:
        """Full map snapshot for the cloud (occupied/blocked cell coordinates).

        ``include_free=True`` 时额外返回自由格坐标（``free`` 数组），
        供云端「全局地图模式」渲染已扫描过的可通行区域（SLAM 建图效果）。
        默认关闭以保持报文精简。
        """
        with self._lock:
            occ_cells = [c for c, v in self._cells.items() if v == OCCUPIED]
            blk_cells = [c for c, v in self._cells.items() if v == BLOCKED]
            free_cells = ([c for c, v in self._cells.items() if v == FREE]
                          if include_free else [])
            free_count = sum(1 for v in self._cells.values() if v == FREE)
            updated = self._updated_at

        def _to_world(cells: list[tuple[int, int]]) -> list[list[float]]:
            if len(cells) > cap:
                step = max(1, int(math.ceil(len(cells) / cap)))
                cells = cells[::step]
            return [[round((cx + 0.5) * self._res, 2),
                     round((cy + 0.5) * self._res, 2)] for cx, cy in cells]

        return {
            "online": online,
            "resolution": self._res,
            "maxRange": self._max_range,
            "cellCount": len(occ_cells) + len(blk_cells),
            "occupiedCells": len(occ_cells),
            "blockedCells": len(blk_cells),
            "freeCells": free_count,
            "occupied": _to_world(occ_cells),
            "blocked": _to_world(blk_cells),
            "free": _to_world(free_cells) if include_free else [],
            "pose": {"x": round(pose_x, 2), "y": round(pose_y, 2),
                     "yawDeg": round(yaw_deg, 1)},
            "updatedAt": round(updated, 1),
            "text": self.as_text(pose_x, pose_y, yaw_deg),
        }

    def as_text(
        self,
        pose_x: float,
        pose_y: float,
        yaw_deg: float,
        radius_x: int = 24,
        radius_y: int = 12,
    ) -> str:
        """ASCII 俯视图：以车体为中心、**车头朝上**（随 yaw 旋转）。

        顶行 = 车头方向（▲），车体 ``@`` 居中。图例:
        ``X`` 不可通行（台阶/岩壁/坑）、``#`` 障碍、``.`` 已知自由、
        ``空格`` 未知。旋转到车体系后，「车前方」永远在屏幕上方。
        """
        yaw = math.radians(yaw_deg)
        siny, cosy = math.sin(yaw), math.cos(yaw)
        with self._lock:
            cells = dict(self._cells)
        body: list[str] = []
        for row in range(radius_y, -radius_y - 1, -1):
            buf = []
            for col in range(-radius_x, radius_x + 1):
                if col == 0 and row == 0:
                    buf.append("@")
                elif col == 0 and row == 1:
                    buf.append("▲")
                else:
                    # 屏幕 (col, row) → 车体系 (x 右 / y 前) → 世界系
                    vx, vy = col * self._res, row * self._res
                    wx = pose_x + vx * siny + vy * cosy
                    wy = pose_y - vx * cosy + vy * siny
                    cx, cy = self._world_to_cell(wx, wy)
                    buf.append(_SYMBOLS.get(cells.get((cx, cy), 0), " "))
            body.append("".join(buf).rstrip())
        width = max((len(r) for r in body), default=0)
        labeled: list[str] = []
        mid = radius_y
        for i, raw in enumerate(body):
            pad = raw.ljust(width)
            if i == 0:
                labeled.append("前 " + pad)
            elif i == len(body) - 1:
                labeled.append("后 " + pad)
            elif i == mid:
                labeled.append("左 " + pad + " 右")
            else:
                labeled.append("   " + pad)
        header = (
            f"俯视图 每格 {self._res:.2f} m  车在 ({pose_x:.2f}, {pose_y:.2f}) "
            f"朝向 {yaw_deg:.0f}°    上=车头  @=车  ▲=车头  "
            f"X=过不去  #=有东西  .=走过的空地  空白=还没扫到"
        )
        return "\n".join([header, *labeled])

    # -- internal --------------------------------------------------------

    def _mark(self, x: float, y: float, value: int, *,
              permanent: bool = False) -> None:
        """Set a cell (world coords) to max(current, value). Lock held by caller.

        ``permanent=True``（``import_map`` 的全局地图格）不参与时间衰减。
        观测时间只在 ``value >= cur``（新状态或同一状态被再次观测）时刷新：
        自由射线扫过占用格不会给占用格「续命」，避免障碍移走后残留格被
        后续自由射线无限保留。
        """
        cx, cy = self._world_to_cell(x, y)
        key = (cx, cy)
        cur = self._cells.get(key, 0)
        if value >= cur:
            self._touch[key] = float("inf") if permanent else time.monotonic()
        if value > cur:
            self._cells[key] = value

    def _prune_expired_locked(self, now: float) -> None:
        """清除超过 TTL 未被重新观测的单元（永久/导入格除外）。"""
        if self._ttl_s <= 0:
            return
        expired = [
            key for key, touched in self._touch.items()
            if touched != float("inf") and now - touched > self._ttl_s
        ]
        for key in expired:
            self._cells.pop(key, None)
            self._touch.pop(key, None)

    def _nearest_locked(self, x: float, y: float,
                        values: set[int]) -> Optional[float]:
        """Nearest cell (m) matching any of ``values`` to the target point."""
        best: Optional[float] = None
        for (cx, cy), v in self._cells.items():
            if v not in values:
                continue
            d = math.hypot((cx + 0.5) * self._res - x,
                           (cy + 0.5) * self._res - y)
            if best is None or d < best:
                best = d
        return best

    def find_frontier(
        self,
        x: float,
        y: float,
        search_radius_m: float = 8.0,
    ) -> Optional[tuple[float, float]]:
        """返回离 (x, y) 最近的「未知边界」格中心（世界系），无则 None。

        frontier = 已知自由格(FREE)相邻、但尚未被任何射线扫过的未知格。
        建图完成后可直接把组员的 SLAM 地图实现同一接口（含明确 unknown
        区域）替换本伪地图，``patrol`` 的 frontier 探索逻辑无需改动。
        伪地图阶段 frontier 即「扫描边界外侧一格」，指向未探索方向。
        """
        with self._lock:
            cells = dict(self._cells)
        cx, cy = self._world_to_cell(x, y)
        r = int(math.ceil(search_radius_m / self._res))
        best: Optional[tuple[int, int]] = None
        best_d2: Optional[int] = None
        for (fx, fy), v in cells.items():
            if v != FREE:
                continue
            if abs(fx - cx) > r or abs(fy - cy) > r:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = fx + dx, fy + dy
                if (nx, ny) in cells:
                    continue  # 已知格（自由/障碍/不可通行），非边界
                d2 = (nx - cx) * (nx - cx) + (ny - cy) * (ny - cy)
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best = (nx, ny)
        if best is None:
            return None
        return ((best[0] + 0.5) * self._res, (best[1] + 0.5) * self._res)

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor(x / self._res)), int(math.floor(y / self._res))
