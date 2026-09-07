"""Nav2 / map_server 2D 栅格 → 网页点选层（占用格世界坐标）。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Optional

from .occupancy import SNAPSHOT_CELL_CAP
from .qualified_maps import (
    click_layer_path,
    load_qualified,
    maps_dir,
    normalize_map_id,
    qualified_entries,
)


def _cap_cells(cells: list[list[float]], cap: int) -> list[list[float]]:
    if cap <= 0 or len(cells) <= cap:
        return cells
    step = max(1, int(math.ceil(len(cells) / cap)))
    return cells[::step][:cap]


def parse_ros_map_yaml(text: str) -> dict[str, Any]:
    """解析 map_server yaml 的常用标量，不依赖 PyYAML。"""
    out: dict[str, Any] = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, _, val = line.partition(":")
        key = key.strip()
        val = val.strip()
        if not key:
            continue
        if val.startswith("[") and val.endswith("]"):
            nums: list[float] = []
            for part in val[1:-1].split(","):
                part = part.strip()
                if not part:
                    continue
                try:
                    nums.append(float(part))
                except ValueError:
                    continue
            out[key] = nums
            continue
        if (val.startswith('"') and val.endswith('"')) or (
            val.startswith("'") and val.endswith("'")
        ):
            out[key] = val[1:-1]
            continue
        low = val.lower()
        if low in ("true", "false"):
            out[key] = low == "true"
            continue
        try:
            out[key] = float(val) if "." in val else int(val)
        except ValueError:
            out[key] = val
    return out


def _pgm_pixels(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as fp:
        magic = fp.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"不支持的 PGM 魔数 {magic!r}")

        def _next() -> bytes:
            while True:
                line = fp.readline()
                if not line:
                    raise ValueError("PGM 头不完整")
                s = line.strip()
                if not s or s.startswith(b"#"):
                    continue
                return s

        token = _next().split()
        while len(token) < 2:
            token += _next().split()
        width, height = int(token[0]), int(token[1])
        rest = token[2:]
        if rest:
            maxval = int(rest[0])
        else:
            maxval = int(_next().split()[0])
        if width <= 0 or height <= 0 or maxval <= 0:
            raise ValueError("PGM 尺寸非法")
        if magic == b"P5":
            data = fp.read(width * height)
            if len(data) < width * height:
                raise ValueError("PGM 像素不足")
            return width, height, maxval, data
        nums = fp.read().split()
        if len(nums) < width * height:
            raise ValueError("PGM 像素不足")
        data = bytes(int(x) for x in nums[: width * height])
        return width, height, maxval, data


def load_nav2_layer(
    yaml_or_pgm: Path,
    *,
    cap: int = SNAPSHOT_CELL_CAP,
) -> dict[str, Any]:
    path = Path(yaml_or_pgm)
    meta: dict[str, Any] = {}
    image = path
    if path.suffix.lower() in (".yaml", ".yml"):
        meta = parse_ros_map_yaml(path.read_text(encoding="utf-8"))
        raw_img = str(meta.get("image") or path.with_suffix(".pgm").name)
        image = Path(raw_img)
        if not image.is_absolute():
            image = path.parent / image
    if not image.is_file():
        raise FileNotFoundError(f"点击层图像不存在: {image}")
    width, height, maxval, pixels = _pgm_pixels(image)
    resolution = float(meta.get("resolution") or 0.05)
    origin = meta.get("origin") or [0.0, 0.0, 0.0]
    if not isinstance(origin, list) or len(origin) < 2:
        origin = [0.0, 0.0, 0.0]
    ox, oy = float(origin[0]), float(origin[1])
    negate = int(meta.get("negate") or 0)
    occupied_thresh = float(meta.get("occupied_thresh") or 0.65)
    occupied: list[list[float]] = []
    free_n = 0
    for row in range(height):
        row_off = row * width
        for col in range(width):
            pix = pixels[row_off + col]
            occ = (pix / float(maxval)) if negate else ((maxval - pix) / float(maxval))
            if occ > occupied_thresh:
                wx = ox + (col + 0.5) * resolution
                wy = oy + (height - row - 0.5) * resolution
                occupied.append([round(wx, 3), round(wy, 3)])
            elif occ < float(meta.get("free_thresh") or 0.196):
                free_n += 1
    total = len(occupied)
    occupied = _cap_cells(occupied, cap)
    return {
        "ready": True,
        "kind": "nav2",
        "resolution": resolution,
        "origin": [ox, oy, float(origin[2]) if len(origin) > 2 else 0.0],
        "width": width,
        "height": height,
        "occupied": occupied,
        "occupiedCells": total,
        "freeCells": free_n,
        "image": image.name,
        "sketch": False,
    }


def load_click_layer(
    map_id: str,
    folder: Optional[os.PathLike | str] = None,
    *,
    cap: int = SNAPSHOT_CELL_CAP,
) -> dict[str, Any]:
    mid = normalize_map_id(map_id)
    root = maps_dir(folder)
    if not mid:
        return {"ready": False, "reason": "no_map", "occupied": []}
    entry = None
    for item in qualified_entries(load_qualified(root)):
        if item["id"].lower() == mid.lower():
            entry = item
            break
    path = click_layer_path(mid, root, entry)
    if path is None:
        return {
            "ready": False,
            "reason": "no_2d_layer",
            "occupied": [],
            "hint": f"还没有 {mid}.yaml/{mid}.pgm。可先填数字坐标；有 2D 层后再点选。",
        }
    try:
        layer = load_nav2_layer(path, cap=cap)
    except Exception as exc:
        return {"ready": False, "reason": "layer_error", "occupied": [], "hint": str(exc)}
    layer["mapId"] = mid
    return layer
