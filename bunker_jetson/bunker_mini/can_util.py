"""Platform-aware CAN bus helpers for python-can.

Supports:
  - gs_usb / candle: candleLight USB-CAN adapters (WinUSB, native USB bulk)
  - slcan:       serial-line CAN adapters (virtual COM port)
  - socketcan:   Linux SocketCAN (native kernel interface)
  - pcan:        PEAK PCAN-USB adapters

On Windows, ``can.detect_available_configs()`` is used first to discover
candleLight (gs_usb) devices.  If none are found the code falls back to
scanning COM ports for slcan-compatible serial adapters.
"""

from __future__ import annotations

import argparse
import logging
import os
import platform
import re
import subprocess
import sys
import threading
import time
import warnings
from contextlib import contextmanager, redirect_stderr
from dataclasses import dataclass
from io import StringIO
from typing import Any, Generator, Optional

# python-can's socketcan interface prints platform warnings at import
# time.  Suppress stderr while the library loads, then restore.
_stderr_sink = StringIO()
with redirect_stderr(_stderr_sink):
    import can

CAN_BITRATE = 500_000
ENV_INTERFACE = "BUNKER_CAN_INTERFACE"
ENV_CHANNEL = "BUNKER_CAN_CHANNEL"

# 底盘周期广播（任收一帧即可认定该口通）。探测用 listen-only，不依赖 ACK。
CHASSIS_RX_IDS = frozenset({0x211, 0x221, 0x241, 0x251, 0x261, 0x311, 0x361})
PROBE_BITRATES = (500_000, 250_000, 1_000_000)


# ---------------------------------------------------------------------------
# Logging / noise suppression
# ---------------------------------------------------------------------------

_log = logging.getLogger(__name__)

# python-can logs a WARNING for every interface whose driver is not
# installed.  On Windows with only one adapter this produces ~15 lines
# of noise every time can.detect_available_configs() runs.  We squash
# those down to CRITICAL (effectively silent) during detection.

def _can_log_level() -> int:
    return logging.getLogger("can").level


def _set_can_log_level(level: int) -> None:
    logging.getLogger("can").setLevel(level)


@contextmanager
def quiet_can_detection() -> Generator[None, None, None]:
    """Suppress python-can backend warnings AND stderr noise during device detection."""
    can_logger = logging.getLogger("can")
    old_level = can_logger.level
    can_logger.setLevel(logging.ERROR)
    stderr_sink = StringIO()
    with warnings.catch_warnings(), redirect_stderr(stderr_sink):
        warnings.simplefilter("ignore", UserWarning)
        try:
            yield
        finally:
            can_logger.setLevel(old_level)


# Make sure the can logger exists early so level sticks
logging.getLogger("can").addHandler(logging.NullHandler())

# INTERFACE_NAME -> canonical python-can bustype.
# NOTE: "candle" (python-can-candle) and "gs_usb" are two separate
# python-can interfaces that both talk to candleLight hardware via
# different drivers.  We keep them distinct and auto-pick whichever is
# available.
_CANONICAL_BUSTYPE: dict[str, str] = {
    "candlelight": "gs_usb",   # user-friendly alias
    "candle": "candle",        # python-can-candle package
    "gs_usb": "gs_usb",        # gs-usb package
    "slcan": "slcan",
    "serial": "slcan",
    "socketcan": "socketcan",
    "pcan": "pcan",
    "ixxat": "ixxat",
}

# All candleLight-family interface names (pyusb / libusb)
_CANDLELIGHT_BUSTYPES = frozenset({"gs_usb", "candle", "candlelight"})

# Interfaces that use pyserial (COM port)
_SERIAL_INTERFACES = frozenset({"slcan", "serial"})

# Common USB-CAN adapter keywords for heuristic matching
_CAN_ADAPTER_HINTS = (
    "can", "usb", "serial", "ch340", "cp210", "ftdi",
    "slcan", "lawicel", "candle", "gs_usb",
    "peak", "pcan", "kvaser", "zlg", "周立功",
)


class CanConfigError(RuntimeError):
    """Raised when CAN interface settings are invalid for the current platform."""


@dataclass(frozen=True)
class SerialPortInfo:
    device: str
    description: str
    manufacturer: str | None = None

    @property
    def looks_like_can_adapter(self) -> bool:
        text = " ".join(
            part for part in (self.device, self.description, self.manufacturer or "") if part
        ).lower()
        return any(hint in text for hint in _CAN_ADAPTER_HINTS)


# ---------------------------------------------------------------------------
# Platform / interface helpers
# ---------------------------------------------------------------------------

def is_windows() -> bool:
    return platform.system() == "Windows"


def canon_bustype(interface: str) -> str:
    """Map user-friendly names (candle, candlelight) to python-can bustype."""
    return _CANONICAL_BUSTYPE.get(interface.lower(), interface)


def default_interface() -> str:
    env = os.environ.get(ENV_INTERFACE)
    if env:
        return env
    if is_windows():
        # Prefer candleLight (gs_usb or candle) if detected; fall back to slcan
        detected = _detect_usb_can_devices()
        if detected:
            return str(detected[0].get("interface", "gs_usb"))
        return "slcan"
    return "socketcan"


def detect_socketcan_channel() -> Optional[str]:
    """Pick the best Linux SocketCAN channel.

    Candidates are ranked by likelihood of being the *chassis* link:
      1. USB-backed, UP, healthy     (candleLight — standard BUNKER link)
      2. USB-backed but DOWN         (bring-up happens next; still chassis)
      3. other UP + healthy          (onboard mttcan — only if no USB-CAN)
      4. anything else healthy
      5. ERROR-PASSIVE / BUS-OFF last
         (these are recovered by ``recover_unhealthy_socketcan`` before probe)

    USB-CAN 的内核名是 can0 还是 can1 不固定；排名按 sysfs 是否 USB，
    不按接口名。ERROR-PASSIVE 不再当健康口排第一，否则探测会听死口。
    板载 mttcan 不能插到 USB-CAN 前面：适配器一掉线就会误绑板载口。
    """
    cands = candidate_socketcan_channels()
    return cands[0] if cands else "can0"


def socketcan_ctrl_unhealthy(channel: str) -> bool:
    """True when the controller cannot usefully TX/RX (passive or bus-off)."""
    return socketcan_ctrl_state(channel) in ("error-passive", "bus-off")


def candidate_socketcan_channels() -> list[str]:
    """SocketCAN netdevs ranked by chassis-link likelihood (see above)."""
    ifaces = list_socketcan_interfaces()
    if not ifaces:
        return []

    def _up(name: str) -> bool:
        return _socketcan_operstate(name) == "up"

    def _healthy(name: str) -> bool:
        return not socketcan_ctrl_unhealthy(name)

    usb_up = [n for n in ifaces if _is_usb_netdev(n) and _up(n) and _healthy(n)]
    usb_down = [n for n in ifaces if _is_usb_netdev(n) and not _up(n)]
    up = [n for n in ifaces if not _is_usb_netdev(n) and _up(n) and _healthy(n)]
    rest = [n for n in ifaces if not _is_usb_netdev(n) and not _up(n) and _healthy(n)]
    unhealthy = [n for n in ifaces if not _healthy(n)]
    # USB-CAN（即使暂时 DOWN）必须排在板载 mttcan 前面。
    # 否则 candleLight 一掉线，探测会绑到 Jetson 板载 can1，100Hz 0x111
    # 打到空总线上，随后 bus-off；USB 再插回来也不会自动用对口。
    return usb_up + usb_down + up + rest + unhealthy


def usb_socketcan_channels() -> list[str]:
    """USB-backed can* names (order follows ``list_socketcan_interfaces``)."""
    return [n for n in list_socketcan_interfaces() if _is_usb_netdev(n)]


def _is_usb_netdev(name: str) -> bool:
    try:
        return "usb" in os.path.realpath(
            os.path.join("/sys/class/net", name, "device")).lower()
    except OSError:
        return False


# ---------------------------------------------------------------------------
# SocketCAN pre-flight checks
# ---------------------------------------------------------------------------

def _socketcan_operstate(channel: str) -> str:
    """Link state of a SocketCAN interface: 'up' / 'down' / 'unknown' / ..."""
    try:
        with open(os.path.join("/sys/class/net", channel, "operstate"), "r") as f:
            return f.read().strip().lower()
    except OSError:
        return "unknown"


def socketcan_operstate(channel: str) -> str:
    """Public link-state query for the agent's CAN health reporting."""
    return _socketcan_operstate(channel)


def socketcan_ctrl_state(channel: str) -> str:
    """Controller state from ``ip -details``: error-active / bus-off / unknown."""
    try:
        out = subprocess.check_output(
            ["ip", "-details", "link", "show", channel],
            text=True, timeout=2,
        )
    except Exception:
        return "unknown"
    match = re.search(r"can state ([A-Z-]+)", out)
    return match.group(1).lower() if match else "unknown"


def list_socketcan_interfaces() -> list[str]:
    """Names of all SocketCAN netdevs present on this host (e.g. ['can0', 'can1'])."""
    try:
        return sorted(n for n in os.listdir("/sys/class/net") if n.startswith("can"))
    except OSError:
        return []


def format_socketcan_status() -> str:
    """Human-readable listing of SocketCAN interfaces + link state + kind."""
    lines: list[str] = []
    for name in list_socketcan_interfaces():
        state = _socketcan_operstate(name)
        try:
            device = os.path.realpath(os.path.join("/sys/class/net", name, "device"))
            kind = "USB-CAN" if "usb" in device.lower() else "板载/其它"
        except OSError:
            kind = "?"
        ctrl = socketcan_ctrl_state(name)
        extra = f" {ctrl.upper()}" if ctrl not in ("unknown", "error-active") else ""
        if socketcan_is_listen_only(name):
            extra += " LISTEN-ONLY"
        lines.append(f"  {name}: {state.upper():<8} ({kind}){extra}")
    return "\n".join(lines)


def ensure_socketcan_interface(channel: Optional[str]) -> None:
    """Verify a SocketCAN interface exists and is UP; best-effort bring it up.

    On a DOWN real CAN link this tries ``ip link set <ch> up type can bitrate
    500000`` automatically (non-interactive sudo, so it never hangs waiting for
    a password).  Raises :class:`CanConfigError` with actionable instructions
    when the interface is missing or cannot be brought up — this replaces the
    confusing ``OSError: Network is down`` traceback the agent used to crash
    with when the CAN interface had not been configured.
    """
    if not channel or not channel.startswith("can") or os.name != "posix":
        return  # vcan*/loopback/non-Linux: let python-can handle those as-is
    netdir = "/sys/class/net"
    if not os.path.isdir(os.path.join(netdir, channel)):
        status = format_socketcan_status()
        raise CanConfigError(
            f"SocketCAN 网卡 {channel} 不存在。\n"
            f"{status if status else '  (当前没有可用的 can* 网卡)'}\n"
            "请确认 USB-CAN 适配器已插入并先执行开机脚本：\n"
            "  bash bringup_gs_usb_can.sh   # USB-CAN 适配器 (candleLight)\n"
            "  bash bringup_can0.sh         # 板载 CAN (mttcan)"
        )
    state = _socketcan_operstate(channel)
    if state in ("up", "unknown", "present"):
        return  # up, or a vcan-like link — usable as-is
    # DOWN → try to bring it up automatically (non-interactive sudo to fail fast)
    cmd = ["ip", "link", "set", channel, "up", "type", "can",
           "bitrate", str(CAN_BITRATE), "restart-ms", "100"]
    if hasattr(os, "geteuid") and os.geteuid() != 0:
        cmd = ["sudo", "-n"] + cmd
    result = None
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    except Exception:
        result = None
    if result is not None and result.returncode == 0:
        _log.info("SocketCAN 网卡 %s 已自动开启 (bitrate=%d)", channel, CAN_BITRATE)
        time.sleep(0.2)
        return
    detail = (result.stderr or "").strip() if result is not None else ""
    raise CanConfigError(
        f"SocketCAN 网卡 {channel} 处于 DOWN 状态，自动开启失败"
        f"{('：' + detail) if detail else ''}。\n"
        f"请手动执行（需要 sudo）：\n"
        f"  sudo ip link set {channel} up type can bitrate {CAN_BITRATE}\n"
        f"或运行项目开机脚本：\n"
        f"  bash bringup_gs_usb_can.sh   # USB-CAN 适配器 (candleLight)\n"
        f"  bash bringup_can0.sh         # 板载 CAN (mttcan)\n"
        f"当前 CAN 网卡状态：\n{format_socketcan_status()}"
    )


def ensure_all_socketcan_up() -> None:
    """Best-effort bring up every existing SocketCAN netdev (500 kbps).

    The agent calls this at startup: a candleLight adapter that was plugged
    in but never configured (``ip link set can1 up``) would otherwise leave
    the USB-CAN link DOWN and the chassis unreachable.
    """
    for name in list_socketcan_interfaces():
        try:
            ensure_socketcan_interface(name)
        except Exception:
            _log.warning("无法自动开启 SocketCAN 网卡 %s", name, exc_info=True)


def _run_ip(args: list[str]) -> Optional[subprocess.CompletedProcess]:
    cmd = list(args)
    if hasattr(os, "geteuid") and os.geteuid() != 0:
        cmd = ["sudo", "-n"] + cmd
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    except Exception:
        return None


def socketcan_is_listen_only(channel: str) -> bool:
    """True when the iface is in listen-only (no TX, no ACK required)."""
    try:
        out = subprocess.check_output(
            ["ip", "-details", "link", "show", channel],
            text=True, timeout=2,
        )
    except Exception:
        return False
    return "LISTEN-ONLY" in out.upper()


def bring_socketcan_up(
    channel: str,
    *,
    bitrate: int = CAN_BITRATE,
    listen_only: bool = False,
) -> bool:
    """Down/up a can* iface. listen-only avoids ERROR-PASSIVE while probing."""
    if not channel or not channel.startswith("can") or os.name != "posix":
        return False
    down = _run_ip(["ip", "link", "set", channel, "down"])
    if down is None or down.returncode != 0:
        detail = (down.stderr or "").strip() if down is not None else ""
        _log.warning("拉起 %s: down 失败%s", channel, f"：{detail}" if detail else "")
        return False
    time.sleep(0.12)
    cmd = [
        "ip", "link", "set", channel, "up", "type", "can",
        "bitrate", str(bitrate), "restart-ms", "100",
        "listen-only", "on" if listen_only else "off",
    ]
    up = _run_ip(cmd)
    if up is None or up.returncode != 0:
        detail = (up.stderr or "").strip() if up is not None else ""
        _log.warning("拉起 %s: up 失败%s", channel, f"：{detail}" if detail else "")
        return False
    time.sleep(0.15)
    set_socketcan_txqueuelen(channel, 1000)
    _log.info(
        "SocketCAN %s 已拉起 bitrate=%d listen-only=%s",
        channel, bitrate, listen_only,
    )
    return True


def set_socketcan_txqueuelen(channel: str, qlen: int = 1000) -> bool:
    """Raise TX queue without down/up. Default qlen=10 fills on the first ENOBUFS."""
    if not channel or not channel.startswith("can") or os.name != "posix":
        return False
    result = _run_ip(["ip", "link", "set", channel, "txqueuelen", str(int(qlen))])
    return result is not None and result.returncode == 0


def flush_socketcan_mailbox(channel: str) -> bool:
    """Drop stuck unacked TX by one down/up. Only for ENOBUFS while driving."""
    return bring_socketcan_up(channel, listen_only=False)


def leave_listen_only(channel: str, bitrate: int = CAN_BITRATE) -> bool:
    """Exit listen-only so 0x111/0x421 can be transmitted.

    No-op when the iface is already TX-capable — down/up here was emptying
    the TX queue every few seconds (3 sent / 150+ dropped).
    """
    if not socketcan_is_listen_only(channel) and _socketcan_operstate(channel) == "up":
        return True
    return bring_socketcan_up(channel, bitrate=bitrate, listen_only=False)


def prepare_listen_only(channel: str, bitrate: int = CAN_BITRATE) -> bool:
    """Put *channel* in listen-only. Skip down/up if already listening and healthy."""
    if (
        _socketcan_operstate(channel) == "up"
        and socketcan_is_listen_only(channel)
        and not socketcan_ctrl_unhealthy(channel)
    ):
        return True
    return bring_socketcan_up(channel, bitrate=bitrate, listen_only=True)


def reset_socketcan_controller(channel: str, *, listen_only: bool = False) -> bool:
    """Soft-reset a SocketCAN controller (clears ERROR-PASSIVE / BUS-OFF).

    Default is listen-only so the next probe does not immediately re-enter
    ERROR-PASSIVE. This is a software controller reset, not a USB reconnect.
    """
    return bring_socketcan_up(
        channel, bitrate=CAN_BITRATE, listen_only=listen_only,
    )


def recover_unhealthy_socketcan(*, listen_only: bool = False) -> list[str]:
    """Soft-reset every ERROR-PASSIVE / BUS-OFF can* interface."""
    recovered: list[str] = []
    for name in list_socketcan_interfaces():
        if not socketcan_ctrl_unhealthy(name):
            continue
        state = socketcan_ctrl_state(name)
        _log.warning("通道 %s 为 %s，软复位控制器后再探测", name, state)
        if reset_socketcan_controller(name, listen_only=listen_only):
            recovered.append(name)
    return recovered


def _gs_usb_sysfs_path() -> Optional[str]:
    try:
        entries = os.listdir("/sys/bus/usb/devices")
    except OSError:
        return None
    for entry in sorted(entries):
        base = os.path.join("/sys/bus/usb/devices", entry)
        try:
            with open(os.path.join(base, "idVendor"), "r") as f:
                vendor = f.read().strip().lower()
            with open(os.path.join(base, "idProduct"), "r") as f:
                product = f.read().strip().lower()
        except OSError:
            continue
        if vendor == "1d50" and product == "606f":
            return base
        if vendor == "1209" and product == "2323":
            return base
    return None


def reset_gs_usb_adapter() -> Optional[str]:
    """Software-reenumerate candleLight (usbreset / authorized). No cable unplug.

    ``ip link down/up`` cannot unwedge a silent gs_usb firmware: cansend returns
    0 but TX packets stay frozen. Returns the USB-CAN netdev name if it returns.
    """
    if _gs_usb_sysfs_path() is None:
        try:
            missing = subprocess.run(
                ["lsusb", "-d", "1d50:606f"],
                capture_output=True, text=True, timeout=3,
            )
            alt = subprocess.run(
                ["lsusb", "-d", "1209:2323"],
                capture_output=True, text=True, timeout=3,
            )
        except Exception:
            missing = None
            alt = None
        if (missing is None or missing.returncode != 0) and (
            alt is None or alt.returncode != 0
        ):
            _log.warning("没有 candleLight USB 设备，跳过软复位（避免空等、误绑板载 CAN）")
            return None
    _log.warning("gs_usb 固件软复位（usbreset / authorized），不拔线")
    try:
        out = subprocess.check_output(
            ["lsusb", "-d", "1d50:606f"], text=True, timeout=3,
        )
        parts = out.split()
        bus, addr = parts[1], parts[3].rstrip(":")
        node = f"/dev/bus/usb/{int(bus):03d}/{int(addr):03d}"
        cmd = ["usbreset", node]
        if hasattr(os, "geteuid") and os.geteuid() != 0:
            cmd = ["sudo", "-n"] + cmd
        subprocess.run(cmd, capture_output=True, timeout=8)
        time.sleep(2.0)
    except Exception as exc:
        _log.debug("usbreset 失败: %s", exc)
    devpath = _gs_usb_sysfs_path()
    if devpath:
        auth = os.path.join(devpath, "authorized")
        for val in ("0", "1"):
            try:
                if hasattr(os, "geteuid") and os.geteuid() == 0:
                    with open(auth, "w") as f:
                        f.write(val + "\n")
                else:
                    subprocess.run(
                        ["sudo", "-n", "tee", auth],
                        input=val + "\n",
                        text=True,
                        capture_output=True,
                        timeout=5,
                    )
                time.sleep(1.6 if val == "0" else 2.4)
            except Exception as exc:
                _log.warning("authorized %s 失败: %s", val, exc)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        usb = usb_socketcan_channels()
        if usb:
            bring_socketcan_up(usb[0], listen_only=False)
            return usb[0]
        time.sleep(0.25)
    _log.warning("gs_usb 软复位后未重新出现 USB-CAN 网卡")
    return None


def _parallel_listen(
    cands: list[str],
    interface: str,
    expected: set[int],
    timeout_s: float,
) -> list[tuple[str, bool]]:
    found: list[tuple[str, bool]] = []
    lock = threading.Lock()

    def _listen(ch: str) -> None:
        try:
            bus = create_can_bus(ch, interface)
        except Exception as exc:
            _log.warning("probe: 通道 %s 无法打开（%s），跳过", ch, exc)
            return
        try:
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                try:
                    msg = bus.recv(timeout=0.25)
                except Exception:
                    return
                if msg is not None and msg.arbitration_id in expected:
                    _log.info(
                        "probe: CAN 通道 %s 收到底盘反馈 0x%X",
                        ch, msg.arbitration_id,
                    )
                    with lock:
                        found.append((ch, _is_usb_netdev(ch)))
                    return
        finally:
            try:
                bus.shutdown()
            except Exception:
                pass

    threads = [
        threading.Thread(target=_listen, args=(ch,), daemon=True, name=f"can-probe-{ch}")
        for ch in cands
    ]
    for t in threads:
        t.start()
    join_s = timeout_s + 1.0
    for t in threads:
        t.join(timeout=join_s)
    return found


def restore_tx_mode(channels: Optional[list[str]] = None) -> None:
    """Ensure listed can* ifaces can transmit (leave listen-only)."""
    names = channels if channels is not None else list_socketcan_interfaces()
    for ch in names:
        try:
            leave_listen_only(ch)
            set_socketcan_txqueuelen(ch, 1000)
        except Exception:
            _log.debug("restore TX %s 失败", ch, exc_info=True)


def probe_chassis_channel(
    interface: str = "socketcan",
    candidates: Optional[list[str]] = None,
    expected_ids: Optional[set[int]] = None,
    timeout_s: float = 5.0,
    bitrates: Optional[tuple[int, ...]] = None,
    *,
    listen_only: bool = False,
) -> Optional[str]:
    """Parallel-listen for chassis RX. Default keeps TX-capable link mode.

    ``listen_only=True`` is only for the first startup probe. The function
    always restores TX mode on USB-CAN before returning, so the agent is
    never left unable to send 0x111.
    """
    expected = set(expected_ids or CHASSIS_RX_IDS)
    rates = tuple(bitrates or (CAN_BITRATE,))
    try:
        ensure_all_socketcan_up()
    except Exception:
        _log.debug("probe: bring-up failed", exc_info=True)

    ifaces = set(list_socketcan_interfaces())
    cands = [ch for ch in (candidates or candidate_socketcan_channels()) if ch in ifaces]
    if not cands:
        return None

    chosen: Optional[str] = None
    try:
        for bitrate in rates:
            if listen_only:
                for ch in cands:
                    try:
                        prepare_listen_only(ch, bitrate=bitrate)
                    except Exception:
                        _log.debug("probe: listen-only %s@%d 失败", ch, bitrate, exc_info=True)
            found = _parallel_listen(cands, interface, expected, timeout_s)
            if not found:
                continue
            usb_hits = [ch for ch, is_usb in found if is_usb]
            chosen = usb_hits[0] if usb_hits else found[0][0]
            _log.info("probe: 选定通道 %s @ %d", chosen, bitrate)
            break
    finally:
        if listen_only:
            restore_tx_mode(cands)
    return chosen


def default_channel(interface: str) -> Optional[str]:
    env_channel = os.environ.get(ENV_CHANNEL)
    if env_channel:
        return env_channel
    bustype = canon_bustype(interface)
    if bustype == "socketcan":
        return detect_socketcan_channel()
    if bustype == "pcan":
        return "PCAN_USBBUS1"
    if bustype == "ixxat":
        return "0"
    return None


def _require_pyserial(interface: str) -> None:
    bustype = canon_bustype(interface)
    if bustype not in _SERIAL_INTERFACES:
        return
    try:
        import serial  # noqa: F401
    except ImportError as exc:
        raise CanConfigError(
            f"接口 '{interface}' 需要安装 pyserial。\n"
            "  pip install pyserial"
        ) from exc

def _require_pyusb(interface: str) -> None:
    bustype = canon_bustype(interface)
    if bustype not in _CANDLELIGHT_BUSTYPES:
        return
    try:
        import usb  # noqa: F401
    except ImportError:
        pass  # pyusb is auto-installed with gs_usb or candle-api

    if bustype == "gs_usb":
        try:
            import gs_usb  # noqa: F401
        except ImportError as exc:
            raise CanConfigError(
                "接口 'gs_usb' (candleLight) 需要安装 python-can[gs_usb]。\n"
                "  pip install \"python-can[gs_usb]\"\n"
                "Windows 上还需使用 Zadig (https://zadig.akeo.ie/) 为 candleLight 设备安装 WinUSB 驱动。"
            ) from exc
    elif bustype == "candle":
        try:
            import candle  # noqa: F401
        except ImportError:
            pass  # candle-api package handles this internally



# ---------------------------------------------------------------------------
# Device detection
# ---------------------------------------------------------------------------


def _detect_usb_can_devices() -> list[dict[str, Any]]:
    """Use python-can autodetection to find candleLight devices.

    Checks both ``gs_usb`` and ``candle`` interface names, because the
    installed driver package (gs-usb vs python-can-candle) determines
    which name python-can registers.
    """
    configs: list[dict[str, Any]] = []
    with quiet_can_detection():
        for iface in ("gs_usb", "candle"):
            try:
                configs.extend(can.detect_available_configs(interfaces=[iface]))
            except Exception:
                pass
    # De-duplicate by channel
    seen: set[str] = set()
    unique: list[dict[str, Any]] = []
    for item in configs:
        ch = str(item.get("channel", ""))
        if ch and ch not in seen:
            seen.add(ch)
            unique.append(item)
    return unique


def list_serial_ports() -> list[SerialPortInfo]:
    _require_pyserial("slcan")
    from serial.tools import list_ports

    return [
        SerialPortInfo(device=p.device, description=p.description or "", manufacturer=p.manufacturer)
        for p in list_ports.comports()
    ]


def auto_detect_channel(interface: str) -> Optional[str]:
    """Auto-detect a channel for *interface* (gs_usb / candle / slcan)."""
    bustype = canon_bustype(interface)
    if bustype in _CANDLELIGHT_BUSTYPES:
        devs = _detect_usb_can_devices()
        if devs:
            return str(devs[0].get("channel", ""))
        return None
    if bustype == "slcan":
        ports = list_serial_ports()
        if not ports:
            return None
        preferred = [p for p in ports if p.looks_like_can_adapter]
        candidates = preferred or ports
        if len(candidates) == 1:
            return candidates[0].device
        return None
    return None


def detect_can_devices(timeout: float = 3.0) -> list[dict[str, Any]]:
    """Return all available CAN device configurations.

    Uses python-can auto-detection for gs_usb and other interfaces,
    plus a serial-port scan for slcan on Windows.
    """
    configs: list[dict[str, Any]] = []

    # Built-in python-can detection (finds gs_usb, pcan, etc.)
    with quiet_can_detection():
        try:
            configs.extend(can.detect_available_configs(timeout=timeout))
        except Exception:
            pass

    # Windows slcan: enumerate COM ports
    if is_windows():
        try:
            for port in list_serial_ports():
                configs.append({
                    "interface": "slcan",
                    "channel": port.device,
                    "description": port.description,
                    "bitrate": CAN_BITRATE,
                })
        except CanConfigError:
            pass

    # De-duplicate
    seen: set[tuple[str, str]] = set()
    unique: list[dict[str, Any]] = []
    for item in configs:
        iface = str(item.get("interface", ""))
        channel = str(item.get("channel", ""))
        key = (iface, channel)
        if not iface or not channel or key in seen:
            continue
        seen.add(key)
        unique.append(item)
    return unique


def format_device_list() -> str:
    lines = ["检测到的 CAN 设备 / 通道:", ""]
    devices = detect_can_devices()

    if not devices:
        lines.append("  (未检测到可用设备)")
        lines.append("")
        if is_windows():
            lines.extend([
                "Windows 排查步骤:",
                "  1. 确认 candleLight / USB-CAN 适配器已插入",
                "  2. 如果使用 candleLight: 用 Zadig 安装 WinUSB 驱动",
                "     https://zadig.akeo.ie/",
                "     - 插上设备 → 打开 Zadig → Options → List All Devices",
                "     - 选择 candleLight → 驱动选 WinUSB → Replace Driver",
                "  3. 如果使用串口 CAN 适配器: 在设备管理器中查看「端口(COM 和 LPT)」",
                "  4. 安装依赖: pip install \"python-can[gs_usb]\" pyserial",
                "  5. 手动指定: --interface gs_usb --channel <设备序列号>",
            ])
        return "\n".join(lines)

    # Group by interface — candleLight family first
    candle_devs = [d for d in devices if d.get("interface") in _CANDLELIGHT_BUSTYPES]
    other_devs = [d for d in devices if d.get("interface") not in _CANDLELIGHT_BUSTYPES]

    if candle_devs:
        lines.append("  [candleLight 设备]")
        for idx, item in enumerate(candle_devs, start=1):
            ch = item.get("channel", "?")
            iface = item.get("interface", "?")
            desc = item.get("description", "")
            suffix = f"  ({desc})" if desc else ""
            lines.append(f"    [{idx}] interface={iface}  channel={ch}{suffix}")
        lines.append("")

    if other_devs:
        lines.append("  [其他 CAN 设备 / 串口]")
        for idx, item in enumerate(other_devs, start=1):
            iface = item.get("interface", "?")
            ch = item.get("channel", "?")
            desc = item.get("description", "")
            suffix = f"  ({desc})" if desc else ""
            lines.append(f"    [{idx}] interface={iface}  channel={ch}{suffix}")

    if devices:
        first = devices[0]
        lines.extend([
            "",
            "使用示例:",
            f"  python examples/get_robot_info.py --interface {first.get('interface')} --channel {first.get('channel')}",
        ])
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Config resolution + bus creation
# ---------------------------------------------------------------------------


def resolve_can_config(
    channel: Optional[str],
    interface: Optional[str],
    *,
    allow_auto_channel: bool = True,
) -> tuple[str, str]:
    """Resolve (channel, interface) pair, applying defaults and auto-detection."""
    iface_raw = interface or default_interface()
    bustype = canon_bustype(iface_raw)
    ch = channel.strip() if channel else None

    if is_windows() and bustype == "socketcan":
        raise CanConfigError(
            "当前系统是 Windows，不支持 socketcan（仅 Linux 可用）。\n"
            "请改用 gs_usb (candleLight) 或 slcan 接口。运行 --list-devices 查看可用通道。"
        )

    # Validate dependencies
    _require_pyserial(iface_raw)
    _require_pyusb(iface_raw)

    if not ch:
        ch = default_channel(iface_raw)

    if not ch and allow_auto_channel:
        ch = auto_detect_channel(iface_raw)
        if ch:
            print(f"已自动选择 {bustype} 通道: {ch}", file=sys.stderr)

    # Auto-fallback: if requested interface (e.g. gs_usb) isn't the one
    # actually detected (e.g. candle), switch to the detected one.
    # Must run *even if* we already found a channel – the channel may
    # belong to a different interface family.
    if bustype in _CANDLELIGHT_BUSTYPES and allow_auto_channel:
        all_usb = _detect_usb_can_devices()
        for dev in all_usb:
            fb_type = str(dev.get("interface", ""))
            fb_ch = str(dev.get("channel", ""))
            if fb_type not in _CANDLELIGHT_BUSTYPES:
                continue
            if fb_type != bustype or not ch:
                if fb_type != bustype:
                    print(
                        f"注意: 请求 {bustype} 接口，但仅检测到 {fb_type} 设备，自动切换为 {fb_type}",
                        file=sys.stderr,
                    )
                bustype = fb_type
                ch = fb_ch
                break

    if not ch:
        device_list = format_device_list()
        msg = f"接口 '{iface_raw}' ({bustype}) 需要指定 --channel。\n\n{device_list}"
        # If the user didn't pick an interface and candleLight wasn't found,
        # tell them what's going on.
        candles = _detect_usb_can_devices()
        if not candles and bustype not in _CANDLELIGHT_BUSTYPES:
            msg += (
                "\n未检测到 candleLight 设备 — 请确认:\n"
                "  1. candleLight 已通过 USB 连接且指示灯亮起\n"
                "  2. WinUSB 驱动已用 Zadig 安装\n"
                "  3. 运行 python examples/get_robot_info.py --list-devices 确认\n"
                "\n如果已连接 candleLight 但未检测到, 手动指定:\n"
                "  python ... --interface candle\n"
            )
        msg += (
            f"\n\n使用示例:\n"
            f"  python examples/get_robot_info.py --interface gs_usb --channel <设备序列号>\n"
            f"  python examples/get_robot_info.py --interface slcan --channel COM3\n"
            f"\n或设置环境变量:\n"
            f"  {ENV_INTERFACE}=gs_usb\n"
            f"  {ENV_CHANNEL}=<设备序列号>"
        )
        raise CanConfigError(msg)

    return ch, bustype


def create_can_bus(
    channel: Optional[str] = None,
    interface: Optional[str] = None,
    bustype_kwargs: Optional[dict[str, Any]] = None,
    *,
    allow_auto_channel: bool = True,
) -> can.BusABC:
    ch, bustype = resolve_can_config(
        channel, interface, allow_auto_channel=allow_auto_channel,
    )
    kwargs = dict(bustype_kwargs or {})
    kwargs.setdefault("bitrate", CAN_BITRATE)

    # SocketCAN pre-flight: the interface must exist and be UP.  A down CAN
    # link makes every send raise `OSError: Network is down` — catching it here
    # (with a best-effort auto bring-up + clear instructions) is far more
    # useful than a raw traceback deep inside the agent.
    if bustype == "socketcan":
        ensure_socketcan_interface(ch)

    try:
        return can.interface.Bus(channel=ch, interface=bustype, **kwargs)
    except OSError as exc:
        hint = connection_hint(bustype, ch)
        raise CanConfigError(
            f"无法打开 CAN 总线 (interface={bustype}, channel={ch}): {exc}\n{hint}"
        ) from exc
    except Exception as exc:
        hint = connection_hint(bustype, ch)
        raise CanConfigError(
            f"无法打开 CAN 总线 (interface={bustype}, channel={ch}): {exc}\n{hint}"
        ) from exc


def connection_hint(bustype: str, channel: str) -> str:
    """Actionable troubleshooting hint for a failed CAN connection."""
    if bustype in _CANDLELIGHT_BUSTYPES:
        return (
            "candleLight 设备排查:\n"
            "  1. 确认设备已插入且指示灯亮起\n"
            "  2. 确认已用 Zadig 安装了 WinUSB 驱动\n"
            "     https://zadig.akeo.ie/ → Options → List All Devices\n"
            "     选择 candleLight / gs_usb → 驱动选 WinUSB → Replace Driver\n"
            f"  3. 确认 CAN 驱动包已安装 (检测到接口: {bustype})\n"
            "  4. 确认 CAN_H/CAN_L 已连接小车，波特率 500K\n"
            "  5. 运行 --list-devices 查看检测结果"
        )
    if bustype == "slcan":
        return (
            f"串口 CAN 设备排查:\n"
            f"  1. COM 口 {channel} 存在且未被其他程序占用\n"
            "  2. 已安装 pyserial (pip install pyserial)\n"
            "  3. USB-CAN 适配器已连接并安装驱动\n"
            "  4. 波特率为 500K（BUNKER MINI 2.0 要求）"
        )
    if bustype == "socketcan":
        return (
            f"SocketCAN 排查:\n"
            f"  1. 已执行: sudo ip link set {channel} up type can bitrate 500000\n"
            f"  2. {channel} 已连接且未被占用"
        )
    return "请检查 CAN 适配器驱动、通道名和波特率 (500K)。"


_connection_hint = connection_hint  # backward-compatible alias


# ---------------------------------------------------------------------------
# CLI helpers
# ---------------------------------------------------------------------------


def add_can_cli_args(parser: argparse.ArgumentParser) -> None:
    iface_default = default_interface()

    parser.add_argument(
        "--interface", "-i",
        default=iface_default,
        help=(
            f"python-can 接口 (默认: {iface_default})。\n"
            "candleLight: gs_usb  |  串口CAN: slcan  |  Linux: socketcan"
        ),
    )
    parser.add_argument(
        "--channel", "-c",
        default=None,
        help=(
            "CAN 通道名（留空时启动自动检测：优先能收到底盘反馈的口，"
            "其次 USB-CAN，最后 can0）。"
            "例: --channel can1"
        ),
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="列出当前系统可用的 CAN 设备/串口后退出",
    )


def print_can_config_help() -> None:
    print("BUNKER MINI 2.0 CAN 连接说明", file=sys.stderr)
    print(file=sys.stderr)
    print(format_device_list(), file=sys.stderr)
    print(file=sys.stderr)
    if is_windows():
        print("Windows 常见用法:", file=sys.stderr)
        print("  # 查看可用设备:", file=sys.stderr)
        print("  python examples/get_robot_info.py --list-devices", file=sys.stderr)
        print("", file=sys.stderr)
        print("  # candleLight (推荐 — 无需指定 channel，自动检测):", file=sys.stderr)
        print("  python examples/get_robot_info.py --interface gs_usb", file=sys.stderr)
        print("", file=sys.stderr)
        print("  # candleLight (手动指定序列号):", file=sys.stderr)
        print("  python examples/get_robot_info.py --interface gs_usb --channel 0030001F4148570C20343133:0", file=sys.stderr)
        print("", file=sys.stderr)
        print("  # 串口 CAN 适配器:", file=sys.stderr)
        print("  python examples/get_robot_info.py --interface slcan --channel COM3", file=sys.stderr)
        print("", file=sys.stderr)
        print("  # 或设置环境变量:", file=sys.stderr)
        print(f"  {ENV_INTERFACE}=gs_usb", file=sys.stderr)
        print(f"  {ENV_CHANNEL}=0030001F4148570C20343133:0", file=sys.stderr)
    else:
        print("Linux 常见用法:", file=sys.stderr)
        print("  sudo ip link set can0 up type can bitrate 500000", file=sys.stderr)
        print("  python examples/get_robot_info.py --interface socketcan --channel can0", file=sys.stderr)
