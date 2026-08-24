"""Terminal-native key input for mock_cloud on Linux / SSH.

The Windows build uses ``msvcrt`` for per-keystroke console input; on
Linux the ``keyboard`` package cannot read keys without a local
``/dev/input`` device or X session (both missing on a headless Jetson
accessed over SSH).  This module provides the equivalent using raw
``termios`` reads on the controlling terminal, which works over SSH.

TermKeyReader
    - ``pressed(name)``        True while the key is held (like
                               ``keyboard.is_pressed``).
    - ``read_char(timeout)``   One decoded keypress (blocks up to
                               timeout), or None.
    - ``drain()``              Discard any pending buffered keystrokes.
    - ``set_echo(bool)``       Toggle terminal echo.
    - ``close()``              Restore the original terminal settings.

Key state is maintained by a daemon thread in raw mode.  Because
terminal raw mode is exclusive, only one reader may own the TTY at a
time — create a single reader for the whole program.

"Held" is inferred from auto-repeat timing (terminals never report key
releases): each key's hold window adapts to its measured repeat gap, so
releases are detected quickly while held keys stay solid — and multiple
keys (W+A) are tracked independently.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
from typing import Optional


class _KeyState:
    """Per-key hold state with hysteresis.

    A key becomes "latched" the moment its first event arrives and stays
    latched (reported as pressed) until *no* event has arrived for longer
    than the current hold window.  The window is derived from the measured
    auto-repeat gap, with a generous floor so that:

      - releasing one key of a W+A combo does not falsely drop the other
        (real terminals pause the surviving key's auto-repeat for
        200-500ms while it "restarts");
      - a held key never flickers between pressed/released (which would
        make the vehicle's speed unstable).
    """

    __slots__ = ("events", "latched", "combo_until", "gap_ema", "consumed_at")

    def __init__(self) -> None:
        self.events: list[float] = []
        self.latched: bool = False
        #: If this key ever overlapped another key's press, the surviving key
        #: stays in "combo mode" (generous hold window) until this deadline,
        #: even if the partner stopped firing.  Monotonic seconds.
        self.combo_until: float = 0.0
        #: EMA of the "inter-batch gap" (the silent interval between bursts of
        #: auto-repeat events).  SSH delivers repeats in *bursts* (several
        #: events nearly simultaneous) separated by silent gaps; tracking this
        #: gap lets the hold window cover the real silence instead of being
        #: fooled by the ~0ms intra-batch gaps.
        self.gap_ema: float = 0.0
        #: Last event timestamp already consumed by take_press().
        self.consumed_at: float = 0.0


class TermKeyReader:
    """Raw-mode terminal key reader with press/release state tracking.

    Terminals report keystrokes as a byte stream and never tell us when a
    key is physically released.  We infer "held" from the timing of
    auto-repeat events: while a key is held, the terminal driver (and the
    SSH client's local keyboard) keeps emitting the same character, so a
    steady stream of events means "held".

    Each key is tracked independently with a *latched* state: the first
    event latches the key as pressed, and only a silent gap longer than
    the adaptive hold window unlatches it.  This keeps W+A combos stable
    (releasing A does not drop W while W's auto-repeat restarts) and
    prevents the pressed/released flicker that would make velocity
    commands jitter.
    """

    #: Floor hold window while auto-repeat is still ramping up.  Covers the
    #: 200-500ms gap a surviving combo key sees when the released key's
    #: auto-repeat restarts — and, crucially, the terminal's *initial*
    #: auto-repeat delay (Linux default 500-660ms, more over SSH).  A single
    #: press whose repeat hasn't started yet must not be reported as released,
    #: otherwise holding a key produces stop/start jitter (speed instability).
    #: 点按（只有 1 个事件）：超过该时长不再当「驾驶按住」。
    #: 必须远短于 INITIAL_HOLD_S，否则点一下会按住窗口 + duration 连跑 1s+。
    #: 长按会在终端 auto-repeat 开始（约 0.5s）后切到下面的自适应窗口。
    TAP_DRIVE_S: float = 0.18
    #: drive_stick 不再用 0.58s 首击窗 / 0.70s 和弦窗「猜还按着」。
    #: 连发已确认后，静默超过此时长才清零。这不是「对齐底盘 10ms」，
    #: 而是 SSH/终端 auto-repeat 批次空白（常 80–200ms）的覆盖窗。
    #: 0.08s 会在长按中间把杆打成 0，底盘 100Hz 跟着停——角速度尤其明显。
    TELEOP_LIVE_S: float = 0.08
    TELEOP_HOLD_S: float = 0.30
    #: 该轴还只有 1 个事件：只盖住点按。长按连发起来后走 TELEOP_HOLD_S。
    TELEOP_FIRST_S: float = 0.22
    #: 驾驶松键窗口：最后一次连发之后超过此时长即停车。
    #: 不得再用 MIN_HOLD_S（0.7s）或 COMBO_HOLD_S（1.6s），否则松键后
    #: 底盘还会按原方向再冲一段。功能键 pressed() 仍走较长窗口。
    DRIVE_RELEASE_S: float = 0.20
    #: SSH 批次空白最多把驾驶窗口抬到这里，再高就会把真松开拖成溜车。
    DRIVE_RELEASE_MAX_S: float = 0.32
    #: 和弦里被抢走连发的键：搭档停了之后，等本键连发重启的最长时间。
    #: 必须盖住 Linux/SSH 的 200-500ms restart；单键松开仍走 DRIVE_RELEASE_S。
    CHORD_RESTART_S: float = 0.70
    INITIAL_HOLD_S: float = 1.0
    #: Single-key release window range (clamped from measured repeat gap).
    #: A low floor keeps quick taps responsive; the combo path widens the
    #: window separately so releases stay snappy outside combo scenarios.
    #:
    #: **抗 SSH 抖动下限**：终端 auto-repeat 稳定在 ~33ms 时，``worst*2.5``
    #: 只有 ~80ms，若 MIN_HOLD_S 太小（如 0.15），hold window 会收缩到
    #: 150ms——SSH 网络抖动让下一个 repeat 事件延迟 200-500ms 时，等待中的
    #: 查询会误判为「松开」→ mock_cloud 下发停车 → 长按一停一顿。自适应
    #: 窗口只能反映「历史抖动」，无法预判「未来突发抖动」，所以唯一可靠
    #: 的抵御手段是足够大的绝对下限。2026-08 二次加固：0.45 → 0.7s，覆盖
    #: 更差的网络抖动；松开响应代价 +0.25s（驾驶可接受，急停走 SPACE）。
    MIN_HOLD_S: float = 0.7
    MAX_HOLD_S: float = 1.5
    #: Window used while a combo (this key overlapped another key's press
    #: recently) is detected — generous enough to cover the surviving key's
    #: auto-repeat restart gap (250-500ms typical, up to ~1s over SSH) plus
    #: test/driver polling latency.  A combo-survivor key stays "held"
    #: through even slow restarts; single-key release stays fast because
    #: this window only applies while the key is in combo mode.
    COMBO_HOLD_S: float = 1.6
    #: Window used once a combo key is *confirmed released* (its partner is
    #: still actively repeating).  This is deliberately short so releasing
    #: one key of a W+A combo takes effect quickly; it is safe because the
    #: partner's steady repeat stream proves this key really stopped.  Must
    #: NOT be used to hold a possibly-surviving key (that's COMBO_HOLD_S).
    COMBO_RELEASE_S: float = 0.3
    #: Gaps shorter than this count as auto-repeat events.
    REPEAT_GAP_S: float = 0.6
    #: 最近两下间隔大于此时长，是又点了一下，不是终端连发。
    #: 必须短于 auto-repeat 初始延迟（约 0.5s），否则连续点按会被收成长按。
    TAP_REPEAT_S: float = 0.22
    #: Gaps longer than this are treated as "inter-batch silence" (the real
    #: gap between SSH-delivered bursts of auto-repeat events), and fed into
    #: each key's gap_ema so the hold window covers real network silence
    #: instead of being fooled by ~0ms intra-batch gaps.
    BATCH_GAP_S: float = 0.05
    #: Two keys whose events arrive within this gap are considered a combo
    #: (pressed together); the surviving key keeps combo mode active for
    #: this long after its last event.
    #: SSH 批次空白常见 200-400ms，0.35 会让「还按着的 W + 新按下的 A」
    #: 建不成和弦。顺序换键测试用 0.50s，必须短于该值。
    COMBO_OVERLAP_S: float = 0.45
    #: How long a key stays in combo mode after its last combo overlap.
    #: 每次当前连发键的事件都续期，避免按住 W+A 超过此时长后 W 掉线。
    COMBO_PERSIST_S: float = 5.0
    #: 全局静默超过此时长，才算「连发重启间隙」（松开一键后另一键重开
    #: auto-repeat），而不是「后按的键抢走了终端唯一的连发」。
    #: 抢走连发通常 <50ms；松开后重启另一键连发常见 200-500ms。
    RESTART_GAP_S: float = 0.15
    #: 与上次事件间隔超过此时长 → 视为一次新的按下（清空旧 tap/hold
    #: 历史）。否则第二次点按仍带着第一次的 2 个事件，会被判成长按。
    FRESH_PRESS_GAP_S: float = 1.2
    #: Event timestamps kept per key (enough to compute robust gaps).
    MAX_EVENTS: int = 16
    #: Reader thread select() timeout. 5ms 以便按键一到就进 drive_stick。
    POLL_S: float = 0.005
    _DRIVE_KEYS = frozenset("wasd")
    # 对向键不能进和弦：W+S / A+D 叠在一起会抵消或「记窜」。
    _OPPOSITE = {"w": "s", "s": "w", "a": "d", "d": "a"}

    def __init__(self, fd: Optional[int] = None) -> None:
        self._fd = fd if fd is not None else sys.stdin.fileno()
        self._is_tty = os.isatty(self._fd)
        self._saved_attrs: Optional[list] = None
        # key (as typed, may be uppercase) -> per-key hold state
        self._key_states: dict[str, _KeyState] = {}
        self._queue: list[str] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._echo_enabled = True
        # WASD 和弦：终端（尤其 SSH）同一时刻通常只 auto-repeat *最后*
        # 按下的那个键。先按的 W 会被 A 抢走连发，W 的事件流停掉，但人
        # 还按着。下面这组状态用来区分「抢走连发」和「真的松开了一键」。
        self._motion_last_t: float = 0.0
        self._motion_last_key: str = ""
        self._restart_gap_end: float = 0.0
        self._restart_key_before: str = ""
        self._restart_keys_after: set[str] = set()
        # 遥控器式双轴：线速度 / 角速度各锁一档，互不顶掉。
        # 终端只连发最后一键，不能按「每个键是否还在」来开轴。
        self._stick_v: int = 0
        self._stick_w: int = 0
        self._stick_v_t: float = 0.0
        self._stick_w_t: float = 0.0
        self._stick_v_n: int = 0
        self._stick_w_n: int = 0
        self._stick_repeat_t: float = 0.0
        self._drive_evt = threading.Event()

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        if not self._is_tty:
            return  # not a real terminal — nothing we can do
        self._saved_attrs = termios.tcgetattr(self._fd)
        # cbreak 之外额外关闭 ISIG：tty.setcbreak() 会保留 ISIG，导致 Ctrl+C
        # 被终端驱动转成 SIGINT → KeyboardInterrupt 中断整个事件循环，而不是
        # 作为 \x03 字节交给 read_char()。关闭 ISIG 后 Ctrl+C 会以 \x03 字节
        # 正常进入按键队列，命令模式才能区分「只关雷达查看器」vs「退出云端」。
        attrs = termios.tcgetattr(self._fd)
        attrs[0] &= ~(termios.ICRNL | termios.IXON)          # iflag
        # oflag：**保留 OPOST 并确保 ONLCR**。tty.setcbreak() 会把 OPOST
        # 一起清掉，导致终端里的 \n 只换行、不回车（光标停留在当前列），
        # 后续每一行都从上一行末尾的列位置开始打印——help 文本行首出现
        # 随机前导空格、逐行右移的排版错乱。显式置位 OPOST|ONLCR 让
        # \n → \r\n，输出恢复正常排版（输入侧 raw 不受影响）。
        attrs[1] |= termios.OPOST | termios.ONLCR
        attrs[3] &= ~(termios.ECHO | termios.ICANON           # lflag
                      | termios.IEXTEN | termios.ISIG)
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSADRAIN, attrs)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name="term-keys", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._saved_attrs is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved_attrs)
            except Exception:
                pass
            self._saved_attrs = None

    # -- input API -------------------------------------------------------

    def pressed(self, name: str) -> bool:
        """True if the named key is currently held (latched)."""
        name = name.lower()
        now = time.monotonic()
        with self._lock:
            for stored, st in self._key_states.items():
                if stored.lower() != name or not st.events:
                    continue
                held = (now - st.events[-1]) < self._hold_for(st, now)
                if not held:
                    held = self._chord_sticky(stored, st, now)
                st.latched = held
                return held
            return False

    def wait_drive(self, timeout: float) -> bool:
        """Block until a WASD event, or ``timeout``. Used by the 100 Hz stick pump."""
        ok = self._drive_evt.wait(timeout)
        if ok:
            self._drive_evt.clear()
        return ok

    def drive_event_age(self) -> float:
        """Seconds since the last WASD event, or inf if none."""
        now = time.monotonic()
        with self._lock:
            last = self._wasd_last_t()
        if last <= 0.0:
            return float("inf")
        return now - last

    def event_count(self, name: str) -> int:
        name = name.lower()
        with self._lock:
            for stored, st in self._key_states.items():
                if stored.lower() == name:
                    return len(st.events)
            return 0

    def take_press(self, name: str) -> bool:
        """每个物理按下只返回一次 True（功能键边沿，不依赖短时间窗）。

        ``just_pressed`` 若轮询晚于窗口会漏掉 B；``pressed`` 又会把一次
        点击闩住约 1s，随后的 A 看起来像还在按 B。
        """
        name = name.lower()
        with self._lock:
            for stored, st in self._key_states.items():
                if stored.lower() != name or not st.events:
                    continue
                last = st.events[-1]
                if last <= st.consumed_at:
                    return False
                st.consumed_at = last
                return True
            return False

    def just_pressed(self, name: str, within_s: float = 0.12) -> bool:
        """True 仅当该键最近 ``within_s`` 内有新事件（短于 pressed 的 1s 窗口）。

        给 B 这类一次性功能键用：``pressed()`` 会把一次点击当成按住 1s，
        容易和随后的 A/WASD 叠在一起，看起来像「按 A 却触发了 B」。
        """
        name = name.lower()
        now = time.monotonic()
        with self._lock:
            for stored, st in self._key_states.items():
                if stored.lower() != name or not st.events:
                    continue
                return (now - st.events[-1]) <= within_s
            return False

    def _superseded_by_newer_drive_key(
        self, name: str, st: _KeyState, now: float,
    ) -> bool:
        """后按的方向键顶替本键：对向键立即顶替，斜向只在非和弦时顶替。"""
        if name.lower() not in self._DRIVE_KEYS or not st.events:
            return False
        my_last = st.events[-1]
        me = name.lower()
        opp = self._OPPOSITE.get(me)
        for stored, other in self._key_states.items():
            if stored.lower() not in self._DRIVE_KEYS:
                continue
            if stored.lower() == me or not other.events:
                continue
            other_last = other.events[-1]
            if stored.lower() == opp:
                if other_last > my_last:
                    return True
                continue
            if other_last <= my_last + 0.08:
                continue
            if st.combo_until > now:
                continue
            if self._looks_preempted(st, now):
                continue
            if (now - my_last) > self.COMBO_OVERLAP_S:
                return True
        return False

    def hold_confirmed(self, name: str) -> bool:
        """True 仅当该键*当前*正在 auto-repeat。

        两次间隔像「又点了一下」（> TAP_REPEAT_S）不算长按；要等到连发
        节拍（第三下，或最近两下间隔很短）才确认。否则连续点按会被
        收成 0.90s 长按。
        """
        name = name.lower()
        now = time.monotonic()
        with self._lock:
            for stored, st in self._key_states.items():
                if stored.lower() != name or len(st.events) < 2:
                    continue
                if (now - st.events[-1]) > self.DRIVE_RELEASE_S:
                    return False
                last_gap = st.events[-1] - st.events[-2]
                if last_gap > self.TAP_REPEAT_S:
                    return False
                return True
            return False

    def motion_held(self, name: str) -> bool:
        """驾驶用：点按只认 TAP_DRIVE_S；出现连发才走长按窗口。

        单事件若也走 INITIAL_HOLD_S（1s），点一下会当成长按。对向键
        后按顶替、W+A 和弦粘滞仍然有效。
        """
        name = name.lower()
        now = time.monotonic()
        with self._lock:
            for stored, st in self._key_states.items():
                if stored.lower() != name or not st.events:
                    continue
                # 后按的对向/新方向键优先：必须在和弦粘滞之前判断，
                # 否则 W 松了再按 S 仍会被 _chord_sticky 当成还在按 W。
                if self._superseded_by_newer_drive_key(stored, st, now):
                    st.combo_until = 0.0
                    st.latched = False
                    return False
                if self._chord_sticky(stored, st, now):
                    st.latched = True
                    return True
                age = now - st.events[-1]
                last_gap = (
                    (st.events[-1] - st.events[-2]) if len(st.events) >= 2 else 1e9
                )
                if last_gap <= self.TAP_REPEAT_S:
                    held = age < self._drive_hold_for(st, now)
                else:
                    held = age < self.TAP_DRIVE_S
                if not held and age >= self.CHORD_RESTART_S:
                    st.combo_until = 0.0
                st.latched = held
                return held
            return False

    def drive_stick(self) -> tuple[int, int]:
        """遥控器式双轴：线速度、角速度独立，互不抢占。

        遥控器 0x241 每帧带左右摇杆两个通道，底盘从不分辨「按了几个键」。
        SSH 终端只连发最后一键，不能按键位按住来合成 (v,w)。这里把 W/S
        锁在线速度轴、A/D 锁在角速度轴：一轴还在连发时，另一轴保持上次
        锁存，直到该轴自己超时或对向键换向。
        """
        now = time.monotonic()
        with self._lock:
            return self._drive_stick_unlocked(now)

    def drive_stick_is_hold(self) -> bool:
        """任一轴已进入连发（或被抢走连发后仍锁着），用长保活。"""
        v, w = self.drive_stick()
        if v == 0 and w == 0:
            return False
        with self._lock:
            return self._stick_v_n >= 2 or self._stick_w_n >= 2

    def _drive_stick_unlocked(self, now: float) -> tuple[int, int]:
        """双轴只在「最近一次 WASD」仍新鲜时输出；静默即双轴清零。

        不再用首击 0.58s / 和弦 0.70s 猜按住——那就是松键溜车的来源。
        被抢走连发的轴：只要另一轴还在连发（last_any 新鲜）就保持。
        """
        v, w = self._stick_v, self._stick_w
        vt, wt = self._stick_v_t, self._stick_w_t
        last_any = self._stick_repeat_t or max(vt if v else 0.0, wt if w else 0.0)
        if v == 0 and w == 0:
            return 0, 0
        hold = (v and self._stick_v_n >= 2) or (w and self._stick_w_n >= 2)
        if hold:
            live = self.TELEOP_HOLD_S
        elif (v and self._stick_v_n < 2) or (w and self._stick_w_n < 2):
            live = max(self.TELEOP_LIVE_S, self.TELEOP_FIRST_S)
        else:
            live = self.TELEOP_LIVE_S
        if last_any <= 0.0 or (now - last_any) >= live:
            self._reset_stick()
            return 0, 0
        return v, w

    def _note_stick(self, key: str, now: float) -> None:
        k = key.lower()
        if k not in self._DRIVE_KEYS:
            return
        live = self.TELEOP_LIVE_S
        self._stick_repeat_t = now
        if k in "ws":
            starting = self._stick_v == 0
            if starting and self._stick_w and (now - self._stick_w_t) >= live:
                self._stick_w = 0
                self._stick_w_n = 0
                self._stick_w_t = 0.0
            sign = 1 if k == "w" else -1
            self._stick_v_n = 1 if self._stick_v != sign else self._stick_v_n + 1
            self._stick_v = sign
            self._stick_v_t = now
            return
        starting = self._stick_w == 0
        if starting and self._stick_v and (now - self._stick_v_t) >= live:
            self._stick_v = 0
            self._stick_v_n = 0
            self._stick_v_t = 0.0
        sign = 1 if k == "a" else -1
        self._stick_w_n = 1 if self._stick_w != sign else self._stick_w_n + 1
        self._stick_w = sign
        self._stick_w_t = now

    def read_char(self, timeout: float = 0.0) -> Optional[str]:
        """Return one decoded keypress, or None after ``timeout``."""
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if self._queue:
                    return self._queue.pop(0)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            time.sleep(min(0.01, remaining))

    def drain(self) -> None:
        with self._lock:
            self._queue.clear()

    def clear_press_states(self) -> None:
        """Forget all tracked key states and pending input.

        Called when entering kb mode so characters typed to build a command
        (e.g. ``kb``, ``r route``) do not linger as "held" keys — otherwise
        ``b`` from ``kb`` would trigger the B-key logic while driving.
        """
        with self._lock:
            self._key_states.clear()
            self._queue.clear()
            self._motion_last_t = 0.0
            self._motion_last_key = ""
            self._restart_gap_end = 0.0
            self._restart_key_before = ""
            self._restart_keys_after = set()
            self._reset_stick()

    def _reset_stick(self) -> None:
        self._stick_v = 0
        self._stick_w = 0
        self._stick_v_t = 0.0
        self._stick_w_t = 0.0
        self._stick_v_n = 0
        self._stick_w_n = 0
        self._stick_repeat_t = 0.0

    def set_echo(self, enabled: bool) -> None:
        """Toggle terminal echo (kb mode should turn it off)."""
        if enabled == self._echo_enabled or not self._is_tty:
            return
        self._echo_enabled = enabled
        try:
            attrs = termios.tcgetattr(self._fd)
            if enabled:
                attrs[3] |= termios.ECHO
            else:
                attrs[3] &= ~termios.ECHO
            termios.tcsetattr(self._fd, termios.TCSADRAIN, attrs)
        except Exception:
            pass

    # -- internals -------------------------------------------------------

    def _keep_chord_across_gap(self, key: str, st: _KeyState, now: float) -> bool:
        """True：间隔虽超过 FRESH_PRESS，但是和弦里被抢走连发，不是新点按。"""
        if st.combo_until > now:
            return True
        me = key.lower()
        opp = self._OPPOSITE.get(me)
        for stored, other in self._key_states.items():
            if other is st or stored.lower() not in self._DRIVE_KEYS:
                continue
            if stored.lower() == opp:
                continue
            if other.combo_until > now:
                return True
            if (
                len(other.events) >= 2
                and (now - other.events[-1]) < self.DRIVE_RELEASE_S
                and (other.events[-1] - other.events[-2]) <= self.TAP_REPEAT_S
            ):
                return True
        return False

    def _note_motion_event(self, key: str, now: float) -> None:
        """Track WASD event stream to distinguish steal vs true one-key release.

        SSH / terminals typically auto-repeat only the last key.  Pressing A
        while holding W stops W's repeats immediately (<50ms) — that is a
        *steal*, not a release.  Releasing A then waiting 200-500ms for W to
        resume is a *restart gap*; keys that do not resume are released.
        """
        k = key.lower()
        if k not in self._DRIVE_KEYS:
            return
        if (
            self._motion_last_t > 0.0
            and (now - self._motion_last_t) >= self.RESTART_GAP_S
        ):
            self._restart_gap_end = now
            self._restart_key_before = self._motion_last_key
            self._restart_keys_after = {k}
        elif self._restart_gap_end > 0.0:
            self._restart_keys_after.add(k)
        self._motion_last_key = k
        self._motion_last_t = now

    def _wasd_confirmed_repeat(self) -> bool:
        return any(
            stored.lower() in self._DRIVE_KEYS and len(st.events) >= 2
            for stored, st in self._key_states.items()
        )

    def _looks_preempted(self, st: _KeyState, now: float) -> bool:
        """本键刚还在连发，随后另一键成了唯一连发源 = 被抢走，不是松开。"""
        if len(st.events) < 2:
            return False
        if (st.events[-1] - st.events[-2]) > self.TAP_REPEAT_S:
            return False
        if (now - st.events[-1]) >= self.CHORD_RESTART_S:
            return False
        # 只要另一键在连发就认为是抢走。顺序换键（W 已松再按 A）时
        # A 还没连发，这里是 False，W 仍可被顶替。
        return self._partner_repeating(st, now)

    def _partner_repeating(self, st: _KeyState, now: float) -> bool:
        """另一方向键此刻仍在连发（不是已经松开、只是历史里有过 2 个事件）。"""
        for stored, other in self._key_states.items():
            if other is st or stored.lower() not in self._DRIVE_KEYS:
                continue
            if len(other.events) < 2:
                continue
            if (now - other.events[-1]) >= self.DRIVE_RELEASE_S:
                continue
            if (other.events[-1] - other.events[-2]) <= self.TAP_REPEAT_S:
                return True
        return False

    def _wasd_last_t(self) -> float:
        last = 0.0
        for stored, st in self._key_states.items():
            if stored.lower() in self._DRIVE_KEYS and st.events:
                last = max(last, st.events[-1])
        return last

    def _i_lost_the_restart(self, name: str, now: float) -> bool:
        """True after a quiet gap when another WASD resumed and this one did not."""
        me = name.lower()
        if self._restart_gap_end <= 0.0:
            return False
        # Still in the silent restart gap — keep the chord latched.
        if self._motion_last_t > 0.0 and (now - self._motion_last_t) > 0.25:
            return False
        if me in self._restart_keys_after:
            return False
        # Same last-key repeater continued after an SSH batch / steal.
        if self._restart_key_before in self._restart_keys_after:
            return False
        if now - self._restart_gap_end < self.COMBO_RELEASE_S:
            return False
        return True

    def _chord_sticky(self, name: str, st: _KeyState, now: float) -> bool:
        """Keep a preempted WASD key held while the chord is still live.

        终端同一时刻通常只连发最后一键。W 被 A 抢走连发后，W 的事件会停，
        但人还按着 W——必须粘住。松掉 A 之后还有 200-500ms 等 W 重开连发，
        这段也不能把 W 丢掉。

        真正松开的是「最后那个连发键」：它不是被抢走的，走短驾驶窗口。
        全部松开后，被抢走的键最多再粘 CHORD_RESTART_S，不会再冲 1.6s。
        """
        if name.lower() not in self._DRIVE_KEYS:
            return False
        if not st.events:
            return False
        if st.combo_until <= now and not self._looks_preempted(st, now):
            return False
        if self._i_lost_the_restart(name, now):
            return False
        if self._partner_repeating(st, now):
            return True
        last_any = self._wasd_last_t()
        my_last = st.events[-1]
        # 本键不是最近那一下 = 被抢走连发。阈值不能太大：W+A 交替写入时
        # 两键只差十几毫秒，0.05s 会把 W 当成「自己就是最后一键」而丢掉。
        if last_any > my_last and (now - last_any) < self.CHORD_RESTART_S:
            return True
        return False

    def _drive_hold_for(self, st: _KeyState, now: float) -> float:
        """驾驶松键窗口：跟手，不用功能键那套 0.7s 下限。"""
        base = self.DRIVE_RELEASE_S
        if st.gap_ema > 0.0:
            base = max(base, min(st.gap_ema * 1.15, self.DRIVE_RELEASE_MAX_S))
        return base

    def _hold_for(self, st: _KeyState, now: float) -> float:
        """Adaptive hold window for a key.

        Base = 2.5x the worst recent auto-repeat gap, clamped to the
        single-key range (low floor → quick taps release fast).

        **2026-08 突发+空白修复**：SSH 下 auto-repeat 事件按「批次」到达
        （一批几个几乎同时，间隔 <5ms），批次之间是真实网络空白。若只看
        「最近几个 gap」，批内 gap≈0 会让窗口收缩到 MIN_HOLD_S，而下一批
        空白 > MIN_HOLD_S 时误判松开 → 长按一停一顿/爆发。因此额外用
        ``gap_ema``（历史跨批次空白）作为下限：窗口 ≥ 2.5×历史空白，覆盖
        真实网络沉默，而不是被批内 ~0ms gap 欺骗。

        In combo mode (this key overlapped another key's press recently)
        two cases exist:

        - another key is *still actively firing* → this key was released
          while the combo continues, so release it quickly (short window);
        - *no* key is firing (they may both be in an auto-repeat restart
          gap) → keep the generous combo window so the survivor is not
          dropped.
        """
        base = self.INITIAL_HOLD_S
        if len(st.events) >= 2:
            gaps = [b - a for a, b in zip(st.events, st.events[1:])]
            recent = gaps[-4:]
            worst = max(recent)
            if worst < self.REPEAT_GAP_S:
                base = min(max(worst * 2.5, self.MIN_HOLD_S), self.MAX_HOLD_S)
        # 跨批次空白下限：覆盖真实网络沉默（突发+空白事件模式）
        if st.gap_ema > 0.0:
            base = max(base, min(st.gap_ema * 2.5, self.MAX_HOLD_S))
        if st.combo_until > now:
            if self._released_with_partner_active(st, now):
                # this key has stopped firing while the partner keeps going
                # → truly released; release quickly so single-key transitions
                # stay responsive（用独立的短窗口，不与抗抖动的 MIN_HOLD_S 混用）
                base = self.COMBO_RELEASE_S
            else:
                base = max(base, self.COMBO_HOLD_S)
        return base

    def _released_with_partner_active(self, st: _KeyState, now: float) -> bool:
        """True only after a quiet gap when another WASD resumed without this key.

        Terminals typically auto-repeat only the last key.  A silent W while
        A is repeating is usually *preemption* (W still held), not a release.
        Shrinking the hold window in that case is what made the chassis stop
        when one key of a chord was released.
        """
        name = ""
        for stored, other in self._key_states.items():
            if other is st:
                name = stored
                break
        if not name:
            return False
        return self._i_lost_the_restart(name, now)

    def _read_loop(self) -> None:
        """Read bytes in raw mode, tracking press/release state."""
        pending = b""
        while not self._stop.is_set():
            try:
                r, _, _ = select.select([self._fd], [], [], self.POLL_S)
                if not r:
                    continue
                chunk = os.read(self._fd, 64)
            except OSError:
                break
            if not chunk:
                continue
            pending += chunk
            while pending:
                key, rest = self._decode_key(pending)
                if key is None:
                    break  # incomplete sequence — wait for more bytes
                pending = rest
                if key is not None:
                    self._record_press(key)

    def _decode_key(self, buf: bytes) -> tuple[Optional[str], bytes]:
        """Decode one keypress from the byte buffer.

        Returns ``(key, remaining_bytes)``; ``key is None`` means the buffer
        holds only a partial escape sequence.  Handles single chars, ESC-]
        function keys (ignored) and common UTF-8.
        """
        if not buf:
            return None, b""
        b0 = buf[0]
        if b0 == 0x1B:  # ESC — function/arrow key or ESC itself
            if len(buf) == 1:
                # lone ESC: treat as escape key (may also be a partial seq)
                return "esc", b""
            # Try to swallow a full CSI/SS3 sequence
            if buf[1] in (0x5B, 0x4F):  # '[' or 'O'
                i = 2
                while i < len(buf) and not (0x40 <= buf[i] <= 0x7E):
                    i += 1
                if i < len(buf):
                    return None, buf[i + 1:]  # consumed seq
                return None, b""  # incomplete seq
            return "esc", buf[1:]
        # UTF-8 continuation handling
        if 0xC0 <= b0 <= 0xDF:
            if len(buf) < 2:
                return None, b""
            return buf[:2].decode("utf-8", "replace"), buf[2:]
        if 0xE0 <= b0 <= 0xEF:
            if len(buf) < 3:
                return None, b""
            return buf[:3].decode("utf-8", "replace"), buf[3:]
        ch = chr(b0)
        if ch == " ":
            return "space", buf[1:]
        if ch == "\x7f" or ch == "\x08":  # DEL / Backspace
            return "backspace", buf[1:]
        if ch in ("\r",):
            return "\r", buf[1:]
        return ch, buf[1:]

    def _record_press(self, key: str, now: Optional[float] = None) -> None:
        if now is None:
            now = time.monotonic()
        with self._lock:
            st = self._key_states.setdefault(key, _KeyState())
            if st.events and (now - st.events[-1]) >= self.FRESH_PRESS_GAP_S:
                # 被抢走连发的 WASD，间隔经常超过 1.2s。若当成新点按清掉
                # 和弦，松 A 后 W 只会被认成 0.18s 点按，底盘收不到长按。
                if not self._keep_chord_across_gap(key, st, now):
                    st.events.clear()
                    st.combo_until = 0.0
                    st.gap_ema = 0.0
            st.events.append(now)
            if len(st.events) > self.MAX_EVENTS:
                del st.events[: len(st.events) - self.MAX_EVENTS]
            st.latched = True
            # 追踪跨批次空白（SSH 突发+空白事件模式）：本事件与上一事件的
            # 间隔若超过 BATCH_GAP_S，说明中间有一段真实网络沉默，计入
            # gap_ema（指数平滑），供 _hold_for 作为 hold window 下限。
            if len(st.events) >= 2:
                gap = now - st.events[-2]
                if gap > self.BATCH_GAP_S:
                    st.gap_ema = gap if st.gap_ema <= 0.0 else (
                        0.5 * gap + 0.5 * st.gap_ema
                    )
            self._note_motion_event(key, now)
            self._note_stick(key, now)
            if key.lower() in self._DRIVE_KEYS:
                self._drive_evt.set()
            # WASD 和弦：后按键会抢走终端唯一的 auto-repeat。只要搭档仍
            # 像按住（最近 overlap，或已确认长按且仍在 MIN_HOLD 内），两边
            # 都进入 combo，被抢走连发的键靠 _chord_sticky 继续判按住。
            if key.lower() in self._DRIVE_KEYS:
                for stored, other in self._key_states.items():
                    if other is st or not other.events:
                        continue
                    if stored.lower() not in self._DRIVE_KEYS:
                        continue
                    if stored.lower() == self._OPPOSITE.get(key.lower()):
                        continue
                    other_age = now - other.events[-1]
                    # 重叠建和弦；已经在和弦里则每次连发都续期——否则
                    # 只连发 A 时 W 的 last 会越来越旧，5s 后 W 掉线。
                    partner_held = other_age < self.COMBO_OVERLAP_S
                    already = st.combo_until > now or other.combo_until > now
                    if partner_held or already:
                        st.combo_until = now + self.COMBO_PERSIST_S
                        other.combo_until = now + self.COMBO_PERSIST_S
            # Queue every keypress so read_char() can serve command-mode line
            # building.  Space/WASD also matter for pressed() queries (via
            # key states), so they live in both places.
            self._queue.append(key)
