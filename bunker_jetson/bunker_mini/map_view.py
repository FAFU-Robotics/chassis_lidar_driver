"""Live Matplotlib view of the 2D OccupancyGrid (RViz-like, no ROS).

只读 ``OccupancyGrid.iter_cells()`` / ``resolution_m`` 与 ``nav.pose``，
由 Agent 主线程 ``start()`` + ``pump()`` 刷新（Tk 必须在主线程）：

  * 栅格底图（世界系 extent，颜色与 ``ascii_view.save_occupancy_png`` 一致）
  * 进程内 ``deque`` 轨迹（采 ``pose.x/y``，不用 ``record_track``）
  * 当前位姿圆点 + yaw 箭头（yaw=0 朝世界 +x）
  * 可选：当前 ``sector_points()`` 投到世界系后的散点

车体→世界变换与 ``OccupancyGrid.update`` 内联的 ``to_global`` 逐字一致，
不另建一套坐标。不写栅格、不碰雷达/CAN/避障。

启用（agent 钩子，默认 ``auto`` = 探测到本机 DISPLAY 才开窗）::

    BUNKER_MAP_VIEW=1 bash start_agent.sh
    BUNKER_MAP_VIEW=0 bash start_agent.sh   # 强制关闭

独立冒烟（不启动底盘）::

    python3 -m bunker_mini.map_view --seconds 8
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from collections import deque
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

# 与 occupancy.py / ascii_view.py 取值、配色对齐（此处不改那些模块）
_OCC_FREE = 1
_OCC_OCCUPIED = 2
_OCC_BLOCKED = 3
_COLOR_UNKNOWN = (128, 128, 128)
_COLOR_FREE = (255, 255, 255)
_COLOR_OCCUPIED = (0, 0, 0)
_COLOR_BLOCKED = (139, 0, 0)

TRAJ_MAX = 5000
TRAJ_MIN_STEP_M = 0.02
ARROW_LEN_M = 0.40
DEFAULT_HZ = 5.0
MAP_REFRESH_S = 1.0
MARGIN_CELLS = 2
DEFAULT_VIEW_PAD_M = 1.5


def _empty_xy() -> Any:
    try:
        import numpy as np
        return np.empty((0, 2))
    except Exception:
        return []


# ---------------------------------------------------------------------------
# 与 OccupancyGrid.update 相同的车体→世界投影（勿改公式）
# ---------------------------------------------------------------------------

def vehicle_to_world(
    pose_x: float,
    pose_y: float,
    yaw_deg: float,
    vx: float,
    vy: float,
) -> tuple[float, float]:
    """车体系 (x 右 / y 前) → 里程系。与 ``OccupancyGrid.update`` 的 ``to_global`` 一致。

    前向 = 世界 ``(cos yaw, sin yaw)``；``yaw=0`` 时车头沿 +x。
    """
    yaw = math.radians(yaw_deg)
    siny, cosy = math.sin(yaw), math.cos(yaw)
    return (pose_x + vx * siny + vy * cosy,
            pose_y - vx * cosy + vy * siny)


def sectors_to_world(
    pose_x: float,
    pose_y: float,
    yaw_deg: float,
    sectors: list[tuple[float, float]],
    *,
    max_range_m: Optional[float] = None,
) -> list[tuple[float, float]]:
    """把 ``sector_points()`` 的 ``[(azimuth_deg, distance_m)]`` 投到世界系。

    极坐标展开与 ``OccupancyGrid.update`` 相同：``vx=d*sin(az), vy=d*cos(az)``。
    ``dist<=0`` 或超出 ``max_range_m`` 的点跳过（与 ``update`` 一致）。
    """
    out: list[tuple[float, float]] = []
    for az_deg, dist in sectors:
        if dist <= 0:
            continue
        if max_range_m is not None and dist > max_range_m:
            continue
        a = math.radians(az_deg)
        out.append(vehicle_to_world(
            pose_x, pose_y, yaw_deg,
            dist * math.sin(a), dist * math.cos(a),
        ))
    return out


def cells_to_image(
    cells: list[tuple[tuple[int, int], int]],
    resolution_m: float,
    margin_cells: int = MARGIN_CELLS,
) -> Optional[tuple[list[list[list[int]]], tuple[float, float, float, float]]]:
    """``iter_cells()`` → (RGB 图像, extent 米)。extent 与 ``save_occupancy_png`` 相同。"""
    if not cells or resolution_m <= 0:
        return None
    xs = [c[0][0] for c in cells]
    ys = [c[0][1] for c in cells]
    pad = max(0, int(margin_cells))
    min_cx, max_cx = min(xs) - pad, max(xs) + pad
    min_cy, max_cy = min(ys) - pad, max(ys) + pad
    width = max_cx - min_cx + 1
    height = max_cy - min_cy + 1
    img = [
        [list(_COLOR_UNKNOWN) for _ in range(width)]
        for _ in range(height)
    ]
    color_of = {
        _OCC_FREE: list(_COLOR_FREE),
        _OCC_OCCUPIED: list(_COLOR_OCCUPIED),
        _OCC_BLOCKED: list(_COLOR_BLOCKED),
    }
    for (cx, cy), value in cells:
        col = cx - min_cx
        row = cy - min_cy
        if 0 <= col < width and 0 <= row < height:
            img[row][col] = color_of.get(int(value), list(_COLOR_UNKNOWN))
    extent = (
        min_cx * resolution_m,
        (max_cx + 1) * resolution_m,
        min_cy * resolution_m,
        (max_cy + 1) * resolution_m,
    )
    return img, extent


def probe_local_display() -> Optional[str]:
    """探测本机 X DISPLAY（含 SSH 无 DISPLAY 时的 :0）。找到则写回环境变量。"""
    existing = os.environ.get("DISPLAY")
    if existing:
        return existing
    x11_dir = "/tmp/.X11-unix"
    try:
        sockets = sorted(n for n in os.listdir(x11_dir) if n.startswith("X"))
    except OSError:
        sockets = []
    for sock in sockets:
        num = sock[1:]
        display = f":{num}"
        xauth = None
        for cand in (
            os.path.join(os.path.expanduser("~"), ".Xauthority"),
            f"/run/user/{os.getuid()}/gdm/Xauthority",
        ):
            if os.path.exists(cand):
                xauth = cand
                break
        os.environ["DISPLAY"] = display
        if xauth:
            os.environ["XAUTHORITY"] = xauth
        return display
    return None


def map_view_should_start() -> bool:
    """agent 钩子：``BUNKER_MAP_VIEW`` = 1/0/auto（默认 auto）。"""
    raw = os.environ.get("BUNKER_MAP_VIEW", "auto").strip().lower()
    if raw in ("0", "false", "no", "off"):
        return False
    if raw in ("1", "true", "yes", "on"):
        return True
    return probe_local_display() is not None


# ---------------------------------------------------------------------------
# Live window
# ---------------------------------------------------------------------------

PoseFn = Callable[[], Optional[tuple[float, float, float]]]
GridFn = Callable[[], Any]
SectorsFn = Callable[[], list[tuple[float, float]]]


class OccupancyMapView:
    """主线程 Matplotlib 窗口；回调只读栅格/位姿/扇区，不写 OccupancyGrid。

    旧实现曾在后台线程 ``plt.subplots``（Tk 会 warning/超时）。现改为：
    ``start()`` 在调用线程（Agent ``run()`` 主线程）开窗，同一线程定期 ``pump()``。
    """

    def __init__(
        self,
        grid_fn: GridFn,
        pose_fn: PoseFn,
        sectors_fn: Optional[SectorsFn] = None,
        *,
        hz: float = DEFAULT_HZ,
        show_scan: bool = True,
        traj_maxlen: int = TRAJ_MAX,
    ) -> None:
        self._grid_fn = grid_fn
        self._pose_fn = pose_fn
        self._sectors_fn = sectors_fn
        self._hz = max(1.0, float(hz))
        self._show_scan = bool(show_scan) and sectors_fn is not None
        self._traj: deque[tuple[float, float]] = deque(maxlen=int(traj_maxlen))
        self._stop = threading.Event()
        self.last_error: Optional[str] = None
        self._plt: Any = None
        self._fig: Any = None
        self._ax: Any = None
        self._im: Any = None
        self._art: dict[str, Any] = {}
        self._map_next_at = 0.0
        self._pump_next_at = 0.0
        self._xlim: Optional[tuple[float, float]] = None
        self._ylim: Optional[tuple[float, float]] = None

    @property
    def running(self) -> bool:
        return self._fig is not None and not self._stop.is_set()

    def start(self) -> bool:
        """在当前线程（须为主线程）非阻塞开窗。失败返回 False，不抛给调用方。

        开窗后须由同一线程调用 ``pump()``；不要再开 GUI 后台线程。
        """
        if self.running:
            return True
        if not self._prepare_display():
            return False
        self._stop.clear()
        self.last_error = None
        try:
            self._plt = self._init_pyplot()
            self._setup_figure()
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            logger.warning("Occupancy map view failed to start: %s", self.last_error)
            self._close_figure()
            return False
        if self._fig is None:
            self.last_error = "matplotlib 未创建 figure"
            return False
        return True

    def pump(self) -> None:
        """主线程泵一拍：只读回调 + set_data，并处理窗口事件。不调用 grid.update。"""
        if self._fig is None or self._stop.is_set():
            return
        try:
            now = time.monotonic()
            if now >= self._pump_next_at:
                self._pump_next_at = now + (1.0 / self._hz)
                self._tick()
                self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            logger.warning("Occupancy map view pump failed: %s", self.last_error)
            self._close_figure()

    def run_blocking(self) -> bool:
        """独立演示：本线程 start + 循环 pump，直到 ``stop()``。"""
        if not self.start():
            return False
        interval = 1.0 / self._hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.pump()
            remain = interval - (time.monotonic() - t0)
            if remain > 0:
                self._stop.wait(remain)
        self._close_figure()
        return self.last_error is None

    def _prepare_display(self) -> bool:
        display = probe_local_display()
        if not display:
            self.last_error = "无 DISPLAY / 未找到本机 X server（/tmp/.X11-unix）"
            logger.info("Occupancy map view skipped: %s", self.last_error)
            return False
        return True

    def stop(self) -> None:
        """请求停止泵循环。关窗请在开窗的同一线程再调 ``close()``。"""
        self._stop.set()

    def close(self) -> None:
        """主线程销毁 figure（勿从 Timer/后台线程调用）。"""
        self._stop.set()
        self._close_figure()

    def _init_pyplot(self) -> Any:
        import matplotlib

        last_err: Exception | None = None
        plt = None
        for backend in ("TkAgg", "GTK3Agg", "Qt5Agg"):
            try:
                matplotlib.use(backend, force=True)
                import matplotlib.pyplot as plt  # noqa: F811
                break
            except Exception as exc:
                last_err = exc
                import sys
                sys.modules.pop("matplotlib.pyplot", None)
        if plt is None:
            raise last_err or RuntimeError("没有可用的 matplotlib 图形后端")
        return plt

    def _setup_figure(self) -> None:
        plt = self._plt
        fig, ax = plt.subplots(figsize=(8, 8))
        try:
            fig.canvas.manager.set_window_title("Bunker Occupancy Map")
        except Exception:
            pass
        ax.set_aspect("equal")
        ax.grid(True, linestyle=":", alpha=0.4)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Occupancy map")
        ax.axhline(0.0, color="0.55", lw=0.6, zorder=2)
        ax.axvline(0.0, color="0.55", lw=0.6, zorder=2)
        empty = [[[128, 128, 128]]]
        self._im = ax.imshow(
            empty,
            origin="lower",
            interpolation="nearest",
            extent=(-1.0, 1.0, -1.0, 1.0),
            zorder=1,
        )
        (self._art["traj"],) = ax.plot(
            [], [], "-", color="tab:blue", lw=1.4, alpha=0.85,
            label="trajectory", zorder=3,
        )
        self._art["scan"] = ax.scatter(
            [], [], s=10, c="lime", alpha=0.75,
            label="lidar scan", zorder=4,
        )
        (self._art["pose"],) = ax.plot(
            [], [], "o", color="cyan", ms=8, zorder=5, label="robot",
        )
        self._art["arrow"] = ax.annotate(
            "",
            xy=(0.0, 0.0),
            xytext=(0.0, 0.0),
            arrowprops={"arrowstyle": "-|>", "color": "cyan", "lw": 1.8},
            zorder=6,
        )
        from matplotlib.patches import Patch
        ax.legend(
            handles=[
                Patch(facecolor=[c / 255 for c in _COLOR_UNKNOWN],
                      edgecolor="0.4", label="unknown"),
                Patch(facecolor=[c / 255 for c in _COLOR_FREE],
                      edgecolor="0.4", label="free"),
                Patch(facecolor=[c / 255 for c in _COLOR_OCCUPIED],
                      edgecolor="0.4", label="occupied"),
                Patch(facecolor=[c / 255 for c in _COLOR_BLOCKED],
                      edgecolor="0.4", label="blocked"),
                self._art["traj"],
                self._art["pose"],
            ],
            loc="upper right",
            fontsize=8,
        )
        self._fig = fig
        self._ax = ax
        plt.show(block=False)
        fig.canvas.flush_events()

    def _tick(self) -> None:
        pose = None
        try:
            pose = self._pose_fn()
        except Exception:
            logger.debug("map view pose_fn failed", exc_info=True)
        if pose is not None:
            px, py, yaw_deg = float(pose[0]), float(pose[1]), float(pose[2])
            last = self._traj[-1] if self._traj else None
            if last is None or math.hypot(px - last[0], py - last[1]) >= TRAJ_MIN_STEP_M:
                self._traj.append((px, py))
            self._art["pose"].set_data([px], [py])
            yaw = math.radians(yaw_deg)
            self._art["arrow"].xy = (
                px + ARROW_LEN_M * math.cos(yaw),
                py + ARROW_LEN_M * math.sin(yaw),
            )
            self._art["arrow"].xytext = (px, py)
        traj = list(self._traj)
        self._art["traj"].set_data(
            [p[0] for p in traj], [p[1] for p in traj],
        )

        now = time.monotonic()
        if now >= self._map_next_at:
            self._map_next_at = now + MAP_REFRESH_S
            self._refresh_map()

        if self._show_scan and pose is not None:
            self._refresh_scan(float(pose[0]), float(pose[1]), float(pose[2]))
        elif self._show_scan:
            self._art["scan"].set_offsets(_empty_xy())

        self._autoscale(pose)

    def _refresh_map(self) -> None:
        grid = None
        try:
            grid = self._grid_fn()
        except Exception:
            logger.debug("map view grid_fn failed", exc_info=True)
            return
        if grid is None:
            return
        try:
            cells = list(grid.iter_cells())
            res = float(grid.resolution_m)
        except Exception:
            return
        built = cells_to_image(cells, res)
        if built is None:
            return
        img, extent = built
        self._im.set_data(img)
        self._im.set_extent(extent)
        self._ax.set_title(f"Occupancy map  {res:.2f} m/cell  ({len(cells)} cells)")

    def _refresh_scan(self, px: float, py: float, yaw_deg: float) -> None:
        sectors: list[tuple[float, float]] = []
        max_range = None
        try:
            if self._sectors_fn is not None:
                sectors = list(self._sectors_fn() or [])
            grid = self._grid_fn()
            if grid is not None:
                max_range = float(grid.max_range_m)
        except Exception:
            sectors = []
        pts = sectors_to_world(px, py, yaw_deg, sectors, max_range_m=max_range)
        self._art["scan"].set_offsets(pts if pts else _empty_xy())

    def _autoscale(self, pose: Optional[tuple[float, float, float]]) -> None:
        xs: list[float] = []
        ys: list[float] = []
        if self._im is not None:
            ext = self._im.get_extent()
            xs.extend([ext[0], ext[1]])
            ys.extend([ext[2], ext[3]])
        for x, y in self._traj:
            xs.append(x)
            ys.append(y)
        if pose is not None:
            xs.append(float(pose[0]))
            ys.append(float(pose[1]))
        if not xs:
            return
        pad = DEFAULT_VIEW_PAD_M
        nx = (min(xs) - pad, max(xs) + pad)
        ny = (min(ys) - pad, max(ys) + pad)
        # 只扩大、避免每帧抖动；收缩超过 4 m 才收回
        if self._xlim is None:
            self._xlim, self._ylim = nx, ny
        else:
            ox0, ox1 = self._xlim
            oy0, oy1 = self._ylim
            grow_x = nx[0] < ox0 - 0.05 or nx[1] > ox1 + 0.05
            grow_y = ny[0] < oy0 - 0.05 or ny[1] > oy1 + 0.05
            shrink = (nx[0] > ox0 + 4.0) or (ox1 > nx[1] + 4.0) or (
                ny[0] > oy0 + 4.0) or (oy1 > ny[1] + 4.0)
            if grow_x or grow_y or shrink:
                self._xlim = (min(ox0, nx[0]), max(ox1, nx[1])) if not shrink else nx
                self._ylim = (min(oy0, ny[0]), max(oy1, ny[1])) if not shrink else ny
        self._ax.set_xlim(*self._xlim)
        self._ax.set_ylim(*self._ylim)

    def _close_figure(self) -> None:
        fig = self._fig
        self._fig = None
        self._ax = None
        self._im = None
        self._art.clear()
        if fig is None or self._plt is None:
            return
        try:
            self._plt.close(fig)
        except Exception:
            pass


def _demo(seconds: float = 0.0, show_scan: bool = True) -> int:
    """不连底盘：用合成栅格+运动验证窗口能否弹出。"""
    from .occupancy import OccupancyGrid

    grid = OccupancyGrid(resolution_m=0.1, ttl_s=0.0)
    state = {"x": 0.0, "y": 0.0, "yaw": 0.0, "t0": time.monotonic()}

    def _drive() -> None:
        t = time.monotonic() - state["t0"]
        state["x"] = 0.15 * t
        state["y"] = 0.04 * math.sin(t)
        state["yaw"] = math.degrees(0.15 * math.sin(0.4 * t))
        az = [(-40.0 + 10.0 * i, 1.6 + 0.15 * i) for i in range(9)]
        grid.update(state["x"], state["y"], state["yaw"], az)

    def pose_fn() -> tuple[float, float, float]:
        _drive()
        return (state["x"], state["y"], state["yaw"])

    view = OccupancyMapView(
        grid_fn=lambda: grid,
        pose_fn=pose_fn,
        sectors_fn=lambda: [(-40.0 + 10.0 * i, 1.6 + 0.15 * i) for i in range(9)],
        show_scan=show_scan,
    )
    if seconds > 0:
        threading.Timer(seconds, view.stop).start()
    print("Occupancy map 窗口打开中（Ctrl+C 或等待 --seconds 结束）", flush=True)
    try:
        ok = view.run_blocking()
    except KeyboardInterrupt:
        view.stop()
        ok = True
    if not ok:
        print(f"窗口未能打开: {view.last_error}", flush=True)
        return 2
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="OccupancyGrid Matplotlib 实时窗口（合成数据冒烟 / 供 agent 嵌入）")
    parser.add_argument(
        "--seconds", type=float, default=0.0,
        help="演示秒数（0=直到 Ctrl+C）",
    )
    parser.add_argument(
        "--no-scan", action="store_true",
        help="不叠加当前扇区扫描点",
    )
    args = parser.parse_args(argv)
    return _demo(seconds=args.seconds, show_scan=not args.no_scan)


if __name__ == "__main__":
    raise SystemExit(main())
