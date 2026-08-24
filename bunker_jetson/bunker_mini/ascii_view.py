"""Human-readable top-down views of LiDAR data for terminal / PNG output.

纯 ASCII 俯视图（车头朝上）+ 可选 matplotlib 俯视图 PNG。零强依赖：
matplotlib 仅在保存 PNG 时按需导入，缺失则自动跳过（返回 None）。

坐标约定与 ``navigator``/``lidar`` 一致：车体系 x 右 / y 前。
"""

from __future__ import annotations

import datetime
import os
from typing import Any, Iterable, Optional, Sequence

# ANSI 颜色（仅终端渲染用，绝不写进网络报文）
_ANSI = {
    "red": "\033[91m",
    "yellow": "\033[93m",
    "green": "\033[92m",
    "white": "\033[97m",
    "cyan": "\033[96m",
    "reset": "\033[0m",
}


def _strip_ansi(s: str) -> str:
    out = s
    for code in _ANSI.values():
        out = out.replace(code, "")
    return out


def render_top_down_ascii(
    points: Iterable[Sequence[float]],
    radius_x: int = 24,
    radius_y: int = 12,
    cell_m: float = 0.1,
    colors: bool = False,
) -> str:
    """Bird's-eye ASCII density view of a point cloud (vehicle frame).

    ``points`` 元素为 ``[x, y, ...]``（x 右 / y 前）。车体 ``@`` 居中、
    车头朝上。密度分级：``.`` 1~2 点、``o`` 3~5、``O`` 6~9、``#`` ≥10。
    ``colors=True`` 时输出 ANSI 彩色（红=密集/黄=中等/绿=稀疏/白=车体），
    适合终端实时刷新；默认纯文本以便写入文件/报文。
    """
    density: dict[tuple[int, int], int] = {}
    max_x = radius_x * cell_m
    for p in points:
        x, y = float(p[0]), float(p[1])
        if abs(x) > max_x or abs(y) > radius_y * cell_m:
            continue
        key = (int(x / cell_m), int(y / cell_m))
        density[key] = density.get(key, 0) + 1

    def glyph(n: int) -> str:
        if n >= 10:
            return "#"
        if n >= 6:
            return "O"
        if n >= 3:
            return "o"
        return "."

    def styled(g: str) -> str:
        if not colors:
            return g
        color = {"#": "red", "O": "yellow", "o": "yellow", ".": "green", "@": "white"}
        return f"{_ANSI[color.get(g, 'reset')]}{g}{_ANSI['reset']}"

    body: list[str] = []
    for row in range(radius_y, -radius_y - 1, -1):
        buf = []
        for col in range(-radius_x, radius_x + 1):
            if col == 0 and row == 0:
                buf.append(styled("@"))
                continue
            n = density.get((col, row), 0)
            buf.append(styled(glyph(n)) if n else " ")
        body.append("".join(buf).rstrip())
    width = max((len(_strip_ansi(r)) for r in body), default=0)
    labeled: list[str] = []
    mid = radius_y
    for i, raw in enumerate(body):
        pad = raw + " " * max(0, width - len(_strip_ansi(raw)))
        if i == 0:
            labeled.append("前 " + pad)
        elif i == len(body) - 1:
            labeled.append("后 " + pad)
        elif i == mid:
            labeled.append("左 " + pad + " 右")
        else:
            labeled.append("   " + pad)
    header = (
        f"点云俯视图 每格 {cell_m:.2f} m    上=车头  @=车  "
        f"#=很密  O=较密  o=稀疏  .=零星  空白=没扫到"
    )
    return "\n".join([header, *labeled])


def save_point_cloud_png(
    points: Iterable[Sequence[float]],
    prefix: str = "pointcloud",
    out_dir: str = ".",
    max_range_m: float = 8.0,
) -> Optional[str]:
    """Save a bird's-eye scatter PNG of the point cloud (x 右 / y 前).

    用强度（第 4 列）着色，缺失则用高度 z（第 3 列）。返回保存路径；
    matplotlib 不可用时返回 None。
    """
    pts = list(points)
    if not pts:
        return None
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None

    xs = [float(p[0]) for p in pts]
    ys = [float(p[1]) for p in pts]
    colors = []
    has_intensity = len(pts[0]) >= 4 and all(float(p[3]) != 0.0 for p in pts[:100])
    for p in pts:
        if has_intensity:
            colors.append(float(p[3]))
        elif len(p) >= 3:
            colors.append(float(p[2]))
        else:
            colors.append(0.0)

    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(xs, ys, c=colors, cmap="jet", s=3, alpha=0.7)
    fig.colorbar(sc, ax=ax, label="intensity" if has_intensity else "z (m)")
    # 车体轮廓（Bunker Mini：长 0.69 / 宽 0.57，车头朝上）
    ax.add_patch(plt.Rectangle((-0.285, 0.0), 0.57, 0.69,
                               fill=False, edgecolor="red", linewidth=2))
    ax.annotate("front", xy=(0.0, 0.69), xytext=(0.0, 0.95),
                ha="center", color="red", fontsize=10)
    ax.set_aspect("equal")
    ax.set_xlim(-max_range_m, max_range_m)
    ax.set_ylim(-max_range_m, max_range_m)
    ax.set_xlabel("x right (m)")
    ax.set_ylabel("y forward (m)")
    ax.set_title("RoboSense Airy - top view (front = up)")
    ax.grid(True, linestyle=":", alpha=0.4)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(
        out_dir,
        f"{prefix}_{datetime.datetime.now():%Y%m%d_%H%M%S}.png",
    )
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


# OccupancyGrid 取值（与 occupancy.py 常量一致，此处不导入以免绑死模块）
_OCC_FREE = 1
_OCC_OCCUPIED = 2
_OCC_BLOCKED = 3

_COLOR_UNKNOWN = (128, 128, 128)
_COLOR_FREE = (255, 255, 255)
_COLOR_OCCUPIED = (0, 0, 0)
_COLOR_BLOCKED = (139, 0, 0)


def save_occupancy_png(
    grid: Any,
    prefix: str = "occupancy_map",
    out_dir: str = ".",
    pose: Optional[tuple[float, float, float]] = None,
    margin_cells: int = 2,
) -> Optional[str]:
    """把 OccupancyGrid 渲成俯视 PNG（unknown / free / occupied）。

    只读 ``grid.iter_cells()`` 与 ``grid.resolution_m``，不改栅格内容。
    未写入的格视为 unknown（灰）；``BLOCKED`` 用深红标出，仍属占用一类。
    ``pose`` 为可选 ``(x, y, yaw_deg)``，世界系，yaw=0 朝 +x。
    matplotlib 不可用或栅格为空时返回 None。
    """
    try:
        cells = list(grid.iter_cells())
        res = float(grid.resolution_m)
    except Exception:
        return None
    if not cells or res <= 0:
        return None
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except Exception:
        return None

    xs = [c[0][0] for c in cells]
    ys = [c[0][1] for c in cells]
    pad = max(0, int(margin_cells))
    min_cx, max_cx = min(xs) - pad, max(xs) + pad
    min_cy, max_cy = min(ys) - pad, max(ys) + pad
    width = max_cx - min_cx + 1
    height = max_cy - min_cy + 1

    # img[row][col]；origin='lower' 时 row=0 对应 min_cy（世界 +y 向上）
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

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(
        img,
        origin="lower",
        interpolation="nearest",
        extent=(
            min_cx * res,
            (max_cx + 1) * res,
            min_cy * res,
            (max_cy + 1) * res,
        ),
    )
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Occupancy map  {res:.2f} m/cell")
    ax.legend(
        handles=[
            Patch(facecolor=[c / 255 for c in _COLOR_UNKNOWN], edgecolor="0.4",
                  label="unknown"),
            Patch(facecolor=[c / 255 for c in _COLOR_FREE], edgecolor="0.4",
                  label="free"),
            Patch(facecolor=[c / 255 for c in _COLOR_OCCUPIED], edgecolor="0.4",
                  label="occupied"),
            Patch(facecolor=[c / 255 for c in _COLOR_BLOCKED], edgecolor="0.4",
                  label="blocked"),
        ],
        loc="upper right",
        fontsize=8,
    )
    if pose is not None:
        import math
        px, py, yaw_deg = float(pose[0]), float(pose[1]), float(pose[2])
        yaw = math.radians(yaw_deg)
        ax.plot(px, py, marker="o", color="cyan", markersize=7, zorder=5)
        ax.annotate(
            "",
            xy=(px + 0.4 * math.cos(yaw), py + 0.4 * math.sin(yaw)),
            xytext=(px, py),
            arrowprops={"arrowstyle": "->", "color": "cyan", "lw": 1.5},
        )

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(
        out_dir,
        f"{prefix}_{datetime.datetime.now():%Y%m%d_%H%M%S}.png",
    )
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path
