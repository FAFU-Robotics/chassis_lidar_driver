"""Load official bunker_mini modules without executing bunker_mini/__init__.py (agent)."""
from __future__ import annotations

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MINI = ROOT / "bunker_jetson" / "bunker_mini"


def ensure_package() -> None:
    if "bunker_mini" in sys.modules and getattr(sys.modules["bunker_mini"], "__path__", None):
        return
    jetson = str(ROOT / "bunker_jetson")
    if jetson not in sys.path:
        sys.path.insert(0, jetson)
    pkg = types.ModuleType("bunker_mini")
    pkg.__path__ = [str(MINI)]  # type: ignore[attr-defined]
    pkg.__package__ = "bunker_mini"
    sys.modules["bunker_mini"] = pkg


def stub_python_can() -> None:
    """Allow importing controller.py without installing python-can (no bus opened)."""
    if "can" in sys.modules:
        return
    can = types.ModuleType("can")

    class CanError(Exception):
        pass

    class Message:
        def __init__(self, *args, **kwargs) -> None:
            pass

    class BusABC:
        pass

    def detect_available_configs(*args, **kwargs):
        return []

    can.CanError = CanError
    can.Message = Message
    can.BusABC = BusABC
    can.Bus = BusABC
    can.detect_available_configs = detect_available_configs
    can.interface = types.ModuleType("can.interface")
    sys.modules["can"] = can
    sys.modules["can.interface"] = can.interface


def load_occupancy():
    ensure_package()
    from bunker_mini import occupancy as mod

    return mod


def load_global_planner():
    ensure_package()
    from bunker_mini import global_planner as mod

    return mod


def load_navigator():
    ensure_package()
    stub_python_can()
    from bunker_mini import navigator as mod

    return mod


def load_obstacle():
    ensure_package()
    stub_python_can()
    from bunker_mini import obstacle as mod

    return mod
