"""Human-readable top-down views of LiDAR data for terminal / PNG output.

纯 ASCII 俯视图（车头朝上）+ 可选 matplotlib 俯视图 PNG。零强依赖：
matplotlib 仅在保存 PNG 时按需导入，缺失则自动跳过（返回 None）。

坐标约定与 ``navigator``/``lidar`` 一致：车体系 x 右 / y 前。
"""

from __future__ import annotations

import datetime
import os
from typing import Iterable, Optional, Sequence

# ANSI 颜色（仅终端渲染用，绝不写进网络报文）
_ANSI = {
    "red": "\033[91m",
    "yellow": "\033[93m",
    "green": "\033[92m",
    "white": "\033[97m",
    "cyan": "\033[96m",
    "reset": "\033[0m",
}


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

    lines = ["point cloud 俯视图（车头朝上）  @=车体  #=密集  O=中等  o=稀疏  .=零星"]
    for row in range(radius_y, -radius_y - 1, -1):
        buf = []
        for col in range(-radius_x, radius_x + 1):
            if col == 0 and row == 0:
                buf.append(styled("@"))
                continue
            n = density.get((col, row), 0)
            buf.append(styled(glyph(n)) if n else " ")
        lines.append("".join(buf).rstrip())
    return "\n".join(lines)


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
