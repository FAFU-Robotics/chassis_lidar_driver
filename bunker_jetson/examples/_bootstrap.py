"""Ensure the project root is importable when running scripts directly.

Scripts live at the project root (next to the ``bunker_mini`` package), so
this just adds that directory to ``sys.path``.  It works both when a script
is invoked from the project root and when invoked from elsewhere (so the
``bunker_mini`` package remains importable either way).
"""

from __future__ import annotations

import sys
from pathlib import Path


def ensure_project_root() -> None:
    root = Path(__file__).resolve().parent
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
