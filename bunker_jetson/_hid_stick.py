"""本机 HID 按键状态 → 遥控器式双轴。

遥控器 0x241 每帧带两个独立通道，底盘从不分辨「按了几个键」。
Windows 直连之所以跟手，是因为 ``keyboard.is_pressed`` 读的是操作系统
按键状态：W 和 A 可以同时为 True，松开立刻为 False。

SSH/终端做不到这件事——它通常只连发最后一个字符，且没有键松开。
本模块只在「物理键盘所在的那台机器」上采样，供 hid_kb_client 使用。
"""

from __future__ import annotations

import glob
import os
import select
import struct
import sys
import threading
from typing import Callable, Optional

# Linux input-event-codes.h（不依赖 evdev 包；工控机断网时装不了 pip）
_EV_KEY = 1
_KEY_CODES = {
    16: "q",
    17: "w",
    30: "a",
    31: "s",
    32: "d",
    48: "b",
    57: "space",
    12: "-",
    13: "=",
    78: "+",
    74: "-",
}
_INPUT_EVENT_FMT = "llHHi"
_INPUT_EVENT_SIZE = struct.calcsize(_INPUT_EVENT_FMT)


class HidStick:
    """Sample WASD as two independent axes (v = W-S, w = A-D)."""

    def __init__(
        self,
        down: Callable[[str], bool],
        *,
        backend: str = "keyboard",
    ) -> None:
        self._down = down
        self.backend = backend

    @classmethod
    def open(cls) -> "HidStick":
        """Open a real HID/key-state backend. Raises if only a TTY is available."""
        errors: list[str] = []
        # Linux：优先标准库读 /dev/input（无需 pip），再 evdev / pynput / keyboard
        linux = sys.platform.startswith("linux")
        order = (
            ("raw_evdev", "evdev", "pynput", "keyboard")
            if linux
            else ("keyboard", "pynput", "evdev", "raw_evdev")
        )
        for name in order:
            try:
                if name == "raw_evdev":
                    return cls._open_raw_evdev()
                if name == "evdev":
                    return cls._open_evdev()
                if name == "pynput":
                    return cls._open_pynput()
                import keyboard  # type: ignore

                keyboard.is_pressed("w")
                return cls(lambda n: bool(keyboard.is_pressed(n)), backend="keyboard")
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        raise RuntimeError(cls._fail_message(errors))

    @staticmethod
    def _fail_message(errors: list[str]) -> str:
        ssh = bool(os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_CLIENT"))
        lines = [
            "本机没有可用的 HID 键盘通道。",
            "  " + "； ".join(errors),
        ]
        if ssh:
            lines.append("  当前是 SSH/远程终端：这里按的键到不了 /dev/input。")
        lines.extend([
            "  正确用法：在【插着键盘的那台电脑】跑 teleop_tcp_client.py",
            "    Windows: pip install keyboard",
            "    Linux:   pip install evdev，或直接用本仓库的 raw /dev/input 通道",
            "            （sudo usermod -aG input $USER 后重新登录）",
            "    然后: python teleop_tcp_client.py --host <工控机IP>",
            "  若就在工控机本机：插上 USB 键盘后再开；不要对本机填公网 IP，用 127.0.0.1。",
        ])
        return "\n".join(lines)

    @classmethod
    def _open_raw_evdev(cls) -> "HidStick":
        """Read /dev/input/event* with stdlib only (no pip evdev)."""
        paths = sorted(glob.glob("/dev/input/event*"))
        if not paths:
            raise RuntimeError("没有 /dev/input/event* 设备")

        chosen_path: Optional[str] = None
        chosen_fd: Optional[int] = None
        best = -1
        perm_err: Optional[Exception] = None
        for path in paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except PermissionError as exc:
                perm_err = exc
                continue
            except OSError:
                continue
            score = cls._raw_keyboard_score(path)
            if score > best:
                if chosen_fd is not None:
                    try:
                        os.close(chosen_fd)
                    except OSError:
                        pass
                best = score
                chosen_fd = fd
                chosen_path = path
            else:
                try:
                    os.close(fd)
                except OSError:
                    pass
        if chosen_fd is None or chosen_path is None or best <= 0:
            if perm_err is not None:
                raise PermissionError(
                    f"读不了 /dev/input（需要 input 组）。"
                    f"执行: sudo usermod -aG input $USER  然后重新登录"
                ) from perm_err
            raise RuntimeError("没有 USB 键盘（仅有 gpio-keys/声卡等）")

        held: dict[str, bool] = {}
        lock = threading.Lock()
        fd = chosen_fd

        def loop() -> None:
            try:
                while True:
                    r, _, _ = select.select([fd], [], [], 1.0)
                    if not r:
                        continue
                    data = os.read(fd, _INPUT_EVENT_SIZE * 32)
                    if not data:
                        break
                    for off in range(0, len(data) - _INPUT_EVENT_SIZE + 1, _INPUT_EVENT_SIZE):
                        _s, _u, typ, code, value = struct.unpack_from(
                            _INPUT_EVENT_FMT, data, off,
                        )
                        if typ != _EV_KEY:
                            continue
                        name = _KEY_CODES.get(code)
                        if not name:
                            continue
                        with lock:
                            held[name] = value != 0
            except Exception:
                return
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass

        threading.Thread(target=loop, name="hid-raw", daemon=True).start()

        def down(name: str) -> bool:
            aliases = {
                "+": ("+", "="),
                "=": ("+", "="),
                "-": ("-",),
                "_": ("-",),
            }
            keys = aliases.get(name, (name,))
            with lock:
                return any(held.get(k, False) for k in keys)

        return cls(down, backend=f"raw:{chosen_path}")

    @staticmethod
    def _raw_keyboard_score(path: str) -> int:
        """Only accept a real keyboard. gpio-keys / 声卡得 0，避免假开 HID 后 WASD 全是 0。"""
        sys_name = ""
        try:
            base = os.path.basename(path)
            sys_path = f"/sys/class/input/{base}/device/name"
            with open(sys_path, "r", encoding="utf-8", errors="replace") as fh:
                sys_name = fh.read().strip().lower()
        except OSError:
            pass
        blob = f"{path.lower()} {sys_name}"
        if any(x in blob for x in (
            "gpio-keys", "hda", "hdmi", "audio", "power", "lid",
            "sleep", "headset",
        )):
            return 0
        if "keyboard" in blob or "kbd" in blob:
            return 80
        if "usb" in blob:
            return 40
        return 0

    @classmethod
    def _open_evdev(cls) -> "HidStick":
        from evdev import InputDevice, ecodes, list_devices  # type: ignore

        key_w = ecodes.ecodes["KEY_W"]
        key_a = ecodes.ecodes["KEY_A"]
        chosen: Optional[InputDevice] = None
        best = -1
        opened: list[InputDevice] = []
        try:
            for path in list_devices():
                try:
                    dev = InputDevice(path)
                except PermissionError as exc:
                    raise PermissionError(
                        f"读不了 {path}（需要 input 组）。"
                        "执行: sudo usermod -aG input $USER  然后重新登录"
                    ) from exc
                opened.append(dev)
                score = cls._evdev_keyboard_score(dev, key_w, key_a)
                if score > best:
                    best = score
                    chosen = dev
            if chosen is None or best <= 0:
                names = ", ".join(f"{d.path}({d.name})" for d in opened) or "无设备"
                raise RuntimeError(
                    f"没有 USB 键盘，只有: {names}"
                )
            for dev in opened:
                if dev is not chosen:
                    try:
                        dev.close()
                    except Exception:
                        pass
        except Exception:
            for dev in opened:
                if chosen is None or dev is not chosen:
                    try:
                        dev.close()
                    except Exception:
                        pass
            raise

        held: dict[str, bool] = {}
        lock = threading.Lock()
        names = {
            ecodes.ecodes["KEY_W"]: "w",
            ecodes.ecodes["KEY_S"]: "s",
            ecodes.ecodes["KEY_A"]: "a",
            ecodes.ecodes["KEY_D"]: "d",
            ecodes.ecodes["KEY_Q"]: "q",
            ecodes.ecodes["KEY_B"]: "b",
            ecodes.ecodes["KEY_SPACE"]: "space",
            ecodes.ecodes["KEY_EQUAL"]: "=",
            ecodes.ecodes["KEY_MINUS"]: "-",
            ecodes.ecodes["KEY_KPPLUS"]: "+",
            ecodes.ecodes["KEY_KPMINUS"]: "-",
        }

        def loop() -> None:
            try:
                for event in chosen.read_loop():
                    if event.type != ecodes.EV_KEY:
                        continue
                    name = names.get(event.code)
                    if not name:
                        continue
                    with lock:
                        held[name] = event.value != 0
            except Exception:
                return

        threading.Thread(target=loop, name="hid-evdev", daemon=True).start()

        def down(name: str) -> bool:
            aliases = {
                "+": ("+", "="),
                "=": ("+", "="),
                "-": ("-",),
                "_": ("-",),
            }
            keys = aliases.get(name, (name,))
            with lock:
                return any(held.get(k, False) for k in keys)

        return cls(down, backend=f"evdev:{chosen.path}")

    @staticmethod
    def _evdev_keyboard_score(dev, key_w: int, key_a: int) -> int:
        """Prefer a real USB keyboard; skip gpio-keys / audio / power buttons."""
        from evdev import ecodes  # type: ignore

        name = (getattr(dev, "name", None) or "").lower()
        if any(x in name for x in (
            "gpio-keys", "hda", "hdmi", "audio", "power", "lid",
            "sleep", "headset", "button",
        )):
            return 0
        caps = dev.capabilities()
        keys = set(caps.get(ecodes.EV_KEY, []))
        if key_w not in keys or key_a not in keys:
            return 0
        score = 10
        if ecodes.EV_REP in caps:
            score += 50
        if "keyboard" in name or "kbd" in name:
            score += 40
        if len(keys) >= 50:
            score += 20
        return score

    @classmethod
    def _open_pynput(cls) -> "HidStick":
        from pynput import keyboard as pynput_keyboard  # type: ignore

        held: set[str] = set()
        lock = threading.Lock()
        mapping = {
            "w": "w", "s": "s", "a": "a", "d": "d", "q": "q", "b": "b",
            "+": "+", "=": "=", "-": "-", "_": "-",
        }

        def _name(key) -> Optional[str]:
            if key == pynput_keyboard.Key.space:
                return "space"
            char = getattr(key, "char", None)
            if not char:
                return None
            return mapping.get(char.lower(), char.lower())

        def on_press(key) -> None:
            name = _name(key)
            if name:
                with lock:
                    held.add(name)

        def on_release(key) -> None:
            name = _name(key)
            if name:
                with lock:
                    held.discard(name)

        listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.daemon = True
        listener.start()

        def down(name: str) -> bool:
            aliases = {
                "+": ("+", "="),
                "=": ("+", "="),
                "-": ("-", "_"),
                "_": ("-", "_"),
            }
            keys = aliases.get(name, (name,))
            with lock:
                return any(k in held for k in keys)

        return cls(down, backend="pynput")

    def axes(self) -> tuple[int, int]:
        """Independent channels: forward/back and left/right, never steal each other."""
        v = (1 if self._down("w") else 0) - (1 if self._down("s") else 0)
        w = (1 if self._down("a") else 0) - (1 if self._down("d") else 0)
        return v, w

    def frame(self) -> dict:
        v, w = self.axes()
        return {
            "v": v,
            "w": w,
            "q": self._down("q"),
            "space": self._down("space"),
            "plus": self._down("+") or self._down("="),
            "minus": self._down("-") or self._down("_"),
            "b": self._down("b"),
        }
