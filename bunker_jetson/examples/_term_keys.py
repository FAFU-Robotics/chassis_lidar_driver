"""Compatibility shim. Canonical implementation: ``../_term_keys.py``."""
from __future__ import annotations

import importlib.util
from pathlib import Path

_CANONICAL = Path(__file__).resolve().parent.parent / "_term_keys.py"
_spec = importlib.util.spec_from_file_location("_term_keys_canonical", _CANONICAL)
if _spec is None or _spec.loader is None:
    raise ImportError(f"cannot load canonical _term_keys.py from {_CANONICAL}")
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

TermKeyReader = _mod.TermKeyReader
_KeyState = _mod._KeyState
__all__ = ["TermKeyReader", "_KeyState"]
