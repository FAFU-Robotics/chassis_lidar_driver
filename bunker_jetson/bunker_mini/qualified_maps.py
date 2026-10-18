"""合格定位图清单：显式 mapId，拒绝 lab / lab2。

``maps/qualified.json`` 由人勾选，脚本和网页都只信这份名单，不会因为
磁盘上有 ``lab2.simplemap`` 就去定位。
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
SELECTED_NAME = "selected_map.local"
QUALIFIED_NAME = "qualified.json"
BANNED_DEFAULT = ("lab", "lab2")


def maps_dir(override: Optional[os.PathLike | str] = None) -> Path:
    if override is not None:
        return Path(override)
    env = os.environ.get("BUNKER_MAPS_DIR", "").strip()
    if env:
        return Path(env)
    return REPO_ROOT / "maps"


def _read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as fp:
            data = json.load(fp)
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def load_qualified(folder: Optional[os.PathLike | str] = None) -> dict[str, Any]:
    path = maps_dir(folder) / QUALIFIED_NAME
    doc = _read_json(path)
    banned = doc.get("banned")
    if not isinstance(banned, list) or not banned:
        doc["banned"] = list(BANNED_DEFAULT)
    maps = doc.get("maps")
    if not isinstance(maps, list):
        doc["maps"] = []
    return doc


def normalize_map_id(map_id: str) -> str:
    raw = (map_id or "").strip()
    if not raw:
        return ""
    name = Path(raw).name
    if name.endswith(".simplemap"):
        name = name[: -len(".simplemap")]
    elif name.endswith(".mm"):
        name = name[:-3]
    elif name.endswith(".yaml"):
        name = name[:-5]
    elif name.endswith(".pgm"):
        name = name[:-4]
    return name.strip()


def banned_ids(doc: Optional[dict[str, Any]] = None) -> set[str]:
    data = doc if doc is not None else load_qualified()
    out = {str(x).strip().lower() for x in (data.get("banned") or []) if str(x).strip()}
    out.update(BANNED_DEFAULT)
    return out


def is_banned(map_id: str, doc: Optional[dict[str, Any]] = None) -> bool:
    mid = normalize_map_id(map_id).lower()
    return bool(mid) and mid in banned_ids(doc)


def qualified_entries(doc: Optional[dict[str, Any]] = None) -> list[dict[str, Any]]:
    data = doc if doc is not None else load_qualified()
    out: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in data.get("maps") or []:
        if isinstance(item, str):
            mid = normalize_map_id(item)
            entry: dict[str, Any] = {"id": mid}
        elif isinstance(item, dict):
            mid = normalize_map_id(str(item.get("id") or item.get("name") or ""))
            entry = dict(item)
            entry["id"] = mid
        else:
            continue
        if not mid or mid.lower() in seen or is_banned(mid, data):
            continue
        seen.add(mid.lower())
        out.append(entry)
    return out


def qualified_ids(doc: Optional[dict[str, Any]] = None) -> list[str]:
    return [str(e["id"]) for e in qualified_entries(doc)]


def is_qualified(map_id: str, doc: Optional[dict[str, Any]] = None) -> bool:
    mid = normalize_map_id(map_id)
    if not mid or is_banned(mid, doc):
        return False
    return mid.lower() in {x.lower() for x in qualified_ids(doc)}


def disk_prefixes(folder: Optional[os.PathLike | str] = None) -> list[str]:
    root = maps_dir(folder)
    stems: set[str] = set()
    try:
        names = list(root.iterdir())
    except OSError:
        return []
    simple = {p.stem for p in names if p.is_file() and p.suffix.lower() == ".simplemap"}
    mm = {p.stem for p in names if p.is_file() and p.suffix.lower() == ".mm"}
    stems.update(simple & mm)
    return sorted(stems)


def click_layer_path(map_id: str, folder: Optional[os.PathLike | str] = None,
                     entry: Optional[dict[str, Any]] = None) -> Optional[Path]:
    root = maps_dir(folder)
    mid = normalize_map_id(map_id)
    if not mid:
        return None
    candidates: list[Path] = []
    if entry:
        raw = str(entry.get("clickLayer") or entry.get("yaml") or "").strip()
        if raw:
            p = Path(raw)
            candidates.append(p if p.is_absolute() else root / p)
    candidates.extend((
        root / f"{mid}.yaml",
        root / f"{mid}.yml",
        root / f"{mid}.pgm",
    ))
    for path in candidates:
        if path.is_file():
            return path
    return None


def catalog(folder: Optional[os.PathLike | str] = None) -> list[dict[str, Any]]:
    root = maps_dir(folder)
    doc = load_qualified(root)
    entries = {e["id"].lower(): e for e in qualified_entries(doc)}
    ids = set(disk_prefixes(root))
    ids.update(e["id"] for e in qualified_entries(doc))
    ids.update(banned_ids(doc))
    items: list[dict[str, Any]] = []
    for mid in sorted(ids, key=str.lower):
        if not mid:
            continue
        entry = entries.get(mid.lower())
        banned = is_banned(mid, doc)
        qualified = bool(entry) and not banned
        simple = (root / f"{mid}.simplemap").is_file()
        mm = (root / f"{mid}.mm").is_file()
        layer = click_layer_path(mid, root, entry)
        if banned:
            usable = "not_for_loc"
        elif qualified:
            usable = "qualified"
        else:
            usable = "not_qualified"
        items.append({
            "id": mid,
            "qualified": qualified,
            "banned": banned,
            "hasSimplemap": simple,
            "hasMm": mm,
            "hasClickLayer": layer is not None,
            "clickLayer": layer.name if layer is not None else None,
            "usable": usable,
            "notes": (entry or {}).get("notes") or "",
        })
    return items


def load_selected_map_id(folder: Optional[os.PathLike | str] = None) -> str:
    path = maps_dir(folder) / SELECTED_NAME
    try:
        text = path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""
    if text.startswith("{"):
        doc = _read_json(path)
        text = str(doc.get("mapId") or doc.get("id") or "")
    return normalize_map_id(text.splitlines()[0] if text else "")


def save_selected_map_id(map_id: str, folder: Optional[os.PathLike | str] = None) -> str:
    mid = normalize_map_id(map_id)
    path = maps_dir(folder) / SELECTED_NAME
    path.write_text(mid + "\n", encoding="utf-8")
    return mid


def resolve_localization_prefix(
    map_id: Optional[str] = None,
    folder: Optional[os.PathLike | str] = None,
) -> Path:
    """返回定位脚本要用的文件前缀（不含 .mm / .simplemap）。

    顺序：参数 / ``BUNKER_LOC_MAP`` / ``selected_map.local`` / 名单里唯一一张。
    lab / lab2 以及未写入 ``qualified.json`` 的前缀一律拒绝。
    """
    root = maps_dir(folder)
    doc = load_qualified(root)
    mid = normalize_map_id(map_id or "")
    if not mid:
        mid = normalize_map_id(os.environ.get("BUNKER_LOC_MAP", ""))
    if not mid:
        mid = load_selected_map_id(root)
    if not mid:
        qids = qualified_ids(doc)
        if len(qids) == 1:
            mid = qids[0]
    if not mid:
        raise ValueError(
            "未指定定位图。请传入地图 id，例如: "
            "start_mola_localization.sh <mapId>  或设置 BUNKER_LOC_MAP。"
            "合格图写在 maps/qualified.json，不要用 lab / lab2。"
        )
    if is_banned(mid, doc):
        raise ValueError(
            f"拒绝定位图 '{mid}'：lab / lab2 几何不合格，不能当自动导航定位图。"
        )
    if not is_qualified(mid, doc):
        raise ValueError(
            f"拒绝定位图 '{mid}'：未出现在 maps/qualified.json。"
            "验收合格后再把 id 写进 maps 列表。"
        )
    prefix = root / mid
    if not prefix.with_suffix(".mm").is_file() or not prefix.with_suffix(".simplemap").is_file():
        raise ValueError(
            f"合格图 '{mid}' 缺少 {mid}.mm 或 {mid}.simplemap（目录 {root}）"
        )
    return prefix
