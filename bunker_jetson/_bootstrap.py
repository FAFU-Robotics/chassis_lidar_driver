"""Ensure the project root is importable when running scripts directly.

Scripts live at the project root (next to the ``bunker_mini`` package), so
this just adds that directory to ``sys.path``.  It works both when a script
is invoked from the project root and when invoked from elsewhere (so the
``bunker_mini`` package remains importable either way).

雷达包 ``robosense_airy`` 由组员维护，位于上级目录 ``chassis_lidar_drivers/``
（与本目录 ``bunker_jetson`` 同级）。这里同时把上级目录加入 ``sys.path``，
且保持本目录在更高优先级——这样 ``bunker_mini`` 始终解析到本目录的完整包，
而 ``robosense_airy`` 解析到组员维护的上级目录副本。
"""

from __future__ import annotations

import sys
from pathlib import Path


def ensure_project_root() -> None:
    root = Path(__file__).resolve().parent
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)

    # 组员雷达包 robosense_airy 在上级目录；放在本目录之后，避免其同名
    # bunker_mini 目录（雷达专用子集）遮蔽本目录的完整 bunker_mini 包。
    parent_str = str(root.parent)
    if parent_str not in sys.path:
        sys.path.insert(1, parent_str)

