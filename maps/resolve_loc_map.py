#!/usr/bin/python3
"""Print the MOLA localization file prefix for a qualified map id. stdout = path only."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "bunker_jetson"))

from bunker_mini.qualified_maps import resolve_localization_prefix  # noqa: E402


def main() -> int:
    map_id = sys.argv[1] if len(sys.argv) > 1 else ""
    try:
        prefix = resolve_localization_prefix(map_id or None)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(prefix)
    return 0


if __name__ == "__main__":
    sys.exit(main())
