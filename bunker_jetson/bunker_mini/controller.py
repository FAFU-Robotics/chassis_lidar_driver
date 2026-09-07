"""High-level CAN controller for BUNKER MINI 2.0 chassis.

Provides BMS / odometer feedback parsing and callbacks (used by the cloud
agent, track recorder / player and dead-reckoned navigation), plus
resilient TX/RX loops that survive transient bus errors instead of dying.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from typing import Callable, Optional

import can

from .can_util import CanConfigError, canon_bustype, connection_hint, create_can_bus



from .protocol import (
    CONTROL_PERIOD_S,
    CONTROL_TIMEOUT_S,
    BmsFeedback,
    CanId,
    ControlMode,
    FaultClearCommand,
    MotionCommand,
    MotionFeedback,
    OdometerFeedback,
    RemoteControlFeedback,
    SystemStatus,
    encode_fault_clear,
    encode_mode_setting,
)

# 0x241 遥控反馈手册周期 20 ms。超过该窗口仍无帧 = 遥控器已关机。
RC_STALE_S: float = 0.50
# 幽灵遥控（0x211 仍报 REMOTE，但 0x241 已停）时清故障的最小间隔。
RC_CLEAR_PERIOD_S: float = 0.40

logger = logging.getLogger(__name__)

# BUNKER MINI 2.0 左右轮距（米）。合成里程用 0x221 v/w 还原左右轮距时需要。
DEFAULT_WHEELBASE_M: float = 0.5

# 速度斜坡（统一加速度限制）：所有驱动者（导航/避障/回放/patrol/approach/
# kb 的 move 命令）最终都汇入 ``BunkerMiniController.set_velocity``，这里是
# 最后一个总出口。斜坡让速度平滑渐变——起步不再有 0→v 的阶跃冲（爆发）、
# 避障解除后不会突然恢复到全速、转向切换不再瞬间换向。加速柔和
# （``MAX_ACCEL_*``），减速陡（``MAX_DECEL_*``，停车/松开跟手）。
# ESTOP、运动看门狗停车必须立即生效，走 ``set_velocity_now`` 直通不走斜坡。
MAX_ACCEL_M_S2: float = 0.5     # 线加速度上限 m/s²（0→0.5 m/s 用 1.0s）
MAX_DECEL_M_S2: float = 1.5     # 线减速度上限 m/s²（0.5→0 用 0.33s）
MAX_ACCEL_RAD_S2: float = 1.0   # 角加速度上限 rad/s²（0→1.0 rad/s 用 1.0s）
MAX_DECEL_RAD_S2: float = 3.0   # 角减速度上限 rad/s²（1.0→0 用 0.33s）
# 两次 set_velocity 之间的时间间隔上限：超过则按该上限计斜坡步长，避免
# 「某驱动者长时间未发指令、恢复后一步到位」破坏斜坡的平滑性。
RAMP_DT_CAP_S: float = 0.5
# 与目标速度差小于该值视为对齐，直接取目标值（避免每帧 1mm/s 级微小爬升）。
RAMP_DEADBAND_M_S: float = 0.002
RAMP_DEADBAND_RAD_S: float = 0.002





class BunkerMiniController:
    """Send motion commands and parse chassis feedback over CAN."""

    def __init__(
        self,
        channel: str | None = None,
        interface: str | None = None,
        bustype_kwargs: Optional[dict] = None,
        *,
        wheelbase_m: float = DEFAULT_WHEELBASE_M,
    ) -> None:
        # Remember bus creation args so start() can rebuild after stop()
        self._channel = channel
        self._interface = interface
        self._bustype_kwargs = bustype_kwargs
        self._wheelbase_m = wheelbase_m
        self._bus = create_can_bus(
            channel=channel,
            interface=interface,
            bustype_kwargs=bustype_kwargs,
        )
        self._closed = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._command = MotionCommand(0, 0)
        # 速度斜坡状态：当前实际下发的指令速度（斜坡逼近目标速度）
        self._cur_v: float = 0.0
        self._cur_w: float = 0.0
        self._last_ramp_t: Optional[float] = None
        self._latest_status: Optional[SystemStatus] = None
        self._latest_motion: Optional[MotionFeedback] = None
        self._latest_bms: Optional[BmsFeedback] = None
        self._latest_odometer: Optional[OdometerFeedback] = None
        self._latest_remote: Optional[RemoteControlFeedback] = None
        self._remote_seen_at: float = 0.0
        # 手册：遥控器未开时默认待机，需发 0x421=0x01 才进 CAN 指令模式。
        # 关机前若停在遥控档，底盘会锁在 0x03+0x04，单次 0x421 会被遥控权限吃掉。
        # 代理始终要 CAN 控车，TX 环持续保持 0x421。
        self._hold_can_mode: bool = True
        self._last_auto_clear_at: float = 0.0
        self._status_callbacks: list[Callable[[SystemStatus], None]] = []
        self._motion_callbacks: list[Callable[[MotionFeedback], None]] = []
        self._bms_callbacks: list[Callable[[BmsFeedback], None]] = []
        self._odometer_callbacks: list[Callable[[OdometerFeedback], None]] = []
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._tx_ok: int = 0
        self._tx_fail: int = 0
        self._last_tx_error: str = ""
        self._mailbox_stuck: bool = False
        self._last_mailbox_flag_at: float = 0.0
        self._unheard_driving: bool = False
        self._unheard_burst_left: int = 0

        # Synthetic odometer: BUNKER MINI does not accumulate the 0x311
        # odometer while the chassis is driven by the physical remote control
        # (遥控模式) — the frames arrive but stay at 0, while 0x221 v/w is
        # still broadcast.  We integrate 0x221 into a synthetic left/right
        # wheel distance and use it whenever the real 0x311 is not advancing.
        self._synth_left_mm: float = 0.0
        self._synth_right_mm: float = 0.0
        self._synth_t: Optional[float] = None
        self._synth_moved: bool = False
        self._real_odo_key: Optional[tuple[int, int]] = None
        self._real_odo_moved: bool = False
        self._real_odo_moved_t: float = 0.0

        # Fused odometer (D-融合): 融合里程 = 最后一次真实 0x311 值 +
        # 合成里程自那次以来的增量。真实活跃时它就是真实值；真实卡死
        # （遥控模式 0x311 恒 0 / 帧丢失）时无缝续接，不再有「真实卡死后
        # 0.5s 里程读数冻结不动」的滞后窗口。synth_ref 是 rebase 时合成
        # 里程的基准，用于计算增量。
        self._fused_left_mm: float = 0.0
        self._fused_right_mm: float = 0.0
        self._synth_ref_left: float = 0.0
        self._synth_ref_right: float = 0.0













    @property
    def wheelbase_m(self) -> float:
        return self._wheelbase_m

    def set_wheelbase(self, wheelbase_m: float) -> None:
        """Update track width used by synthetic odometry (0x221 fallback)."""
        wb = float(wheelbase_m)
        if wb <= 0:
            raise ValueError("wheelbase_m must be > 0")
        self._wheelbase_m = wb

    def start(self) -> None:
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._stop_event.clear()
        # Rebuild the bus if it was shut down by a previous stop() call.
        if self._closed:
            self._bus = create_can_bus(
                channel=self._channel,
                interface=self._interface,
                bustype_kwargs=self._bustype_kwargs,
            )
            self._closed = False
        self._rx_thread = threading.Thread(target=self._rx_loop, name="bunker-rx", daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, name="bunker-tx", daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

    def stop(self) -> None:
        self.set_velocity_now(0.0, 0.0)
        time.sleep(CONTROL_PERIOD_S)
        self._stop_event.set()
        if self._tx_thread:
            self._tx_thread.join(timeout=1.0)
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        self._bus.shutdown()
        self._closed = True

    def rebuild_bus(self) -> None:
        """Recreate the python-can socket after a SocketCAN soft-reset."""
        self.switch_channel(self._channel, self._interface, force=True)

    def switch_channel(
        self,
        channel: str,
        interface: Optional[str] = None,
        *,
        force: bool = False,
    ) -> None:
        """Hot-swap the CAN bus to a different channel/interface.

        Used when the chassis is later discovered on another SocketCAN netdev
        (e.g. USB-CAN came up as ``can1`` after the agent started on ``can0``):
        the agent can re-target the controller without rebuilding recorder /
        player / navigator, which all hold a reference to this controller.
        ``force=True`` rebuilds even when the name is unchanged (after a
        controller soft-reset the old socket is stale).
        """
        if (
            not force
            and channel == self._channel
            and (interface or self._interface) == self._interface
        ):
            return
        self.set_velocity_now(0.0, 0.0)
        time.sleep(CONTROL_PERIOD_S)
        self._stop_event.set()
        if self._tx_thread:
            self._tx_thread.join(timeout=1.0)
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        try:
            self._bus.shutdown()
        except Exception:
            pass
        self._channel = channel
        if interface:
            self._interface = interface
        self._bus = create_can_bus(
            channel=self._channel,
            interface=self._interface,
            bustype_kwargs=self._bustype_kwargs,
        )
        self._closed = False
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="bunker-rx", daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, name="bunker-tx", daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

    def enable_can_control(self) -> None:
        """Switch chassis from standby to CAN command mode (ID 0x421)."""
        self._hold_can_mode = True
        self._send_checked(CanId.MODE_SETTING, encode_mode_setting(True), "切换到 CAN 指令模式")

    def disable_can_control(self) -> None:
        self._hold_can_mode = False
        self._send_checked(CanId.MODE_SETTING, encode_mode_setting(False), "切回待机模式")

    @property
    def remote_alive(self) -> bool:
        """True when 0x241 remote frames are still arriving (遥控器开着)."""
        return (
            self._remote_seen_at > 0.0
            and (time.monotonic() - self._remote_seen_at) < RC_STALE_S
        )

    @property
    def remote_swb_command(self) -> bool:
        """True when the live remote has SWB in the command (top) position."""
        rc = self._latest_remote
        return bool(self.remote_alive and rc is not None and rc.swb_is_command)

    def ensure_can_mode(self, timeout_s: float = 2.0) -> bool:
        """Clear a latched remote-lost lock and wait until 0x211 reports CAN.

        手册：遥控器未开 → 待机，再发 0x421。若关机前停在遥控档，底盘会
        一直报 REMOTE + 失联；此时先 0x441 再连续 0x421。真遥控开着且 SWB
        在中档则不抢权。未听到 0x211 时不发送，避免把总线打进 ERROR-PASSIVE。
        """
        if self._latest_status is None:
            return False
        self._hold_can_mode = True
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            st = self._latest_status
            if st is not None and st.control_mode == ControlMode.CAN_COMMAND:
                return True
            if self.remote_alive and not self.remote_swb_command:
                return False
            try:
                if st is not None and (
                    int(st.fault_code or 0) & 0x04
                    or st.control_mode == ControlMode.REMOTE_CONTROL
                ):
                    self.clear_faults()
                self.enable_can_control()
            except Exception:
                logger.exception("ensure_can_mode: 清故障/切 CAN 失败")
            if time.monotonic() >= deadline:
                return bool(
                    st is not None and st.control_mode == ControlMode.CAN_COMMAND
                )
            time.sleep(0.1)

    def clear_faults(self, command: FaultClearCommand = FaultClearCommand.CLEAR_NON_CRITICAL) -> None:
        self._send_checked(CanId.FAULT_CLEAR, encode_fault_clear(command), "清除故障")




    def set_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        """下发底盘速度指令（应用统一加速度斜坡）。

        所有驱动者（导航/避障/回放/patrol/approach/kb 的 move 命令）最终都
        汇入这里，斜坡保证速度平滑渐变——起步无 0→v 阶跃（爆发）、避障解除
        后不突然恢复全速、转向切换不瞬间换向。加速柔和（MAX_ACCEL_*）、
        减速陡（MAX_DECEL_*，停车跟手）。ESTOP 和运动看门狗停车必须立即
        生效，用 :meth:`set_velocity_now` 直通，不走斜坡。
        """
        with self._lock:
            now = time.monotonic()
            # 首次调用无基准时刻：按一个控制周期（20ms）起步，避免起步
            # 第一拍 dt=0 导致输出仍为 0（多一拍延迟）。
            dt = CONTROL_PERIOD_S if self._last_ramp_t is None else now - self._last_ramp_t
            self._last_ramp_t = now
            dt = min(dt, RAMP_DT_CAP_S)
            rv = self._ramp_one(self._cur_v, linear_m_s, dt,
                                MAX_ACCEL_M_S2, MAX_DECEL_M_S2)
            rw = self._ramp_one(self._cur_w, angular_rad_s, dt,
                                MAX_ACCEL_RAD_S2, MAX_DECEL_RAD_S2)
            self._cur_v, self._cur_w = rv, rw
            self._command = MotionCommand.from_si(rv, rw)

    def set_velocity_now(self, linear_m_s: float, angular_rad_s: float) -> None:
        """直通设置速度指令，不走斜坡（立即生效）。

        供 ESTOP / 运动看门狗停车等「必须立刻停」的场景使用，把当前指令
        速度一步置为目标值，并把斜坡基准同步到该值。
        """
        with self._lock:
            self._cur_v = float(linear_m_s)
            self._cur_w = float(angular_rad_s)
            self._last_ramp_t = time.monotonic()
            self._command = MotionCommand.from_si(linear_m_s, angular_rad_s)

    @staticmethod
    def _ramp_one(cur: float, target: float, dt: float,
                  accel: float, decel: float) -> float:
        """把当前速度朝目标逼近一步（斜坡）。"""
        if abs(target - cur) <= RAMP_DEADBAND_M_S:
            return float(target)
        if target > cur:
            return min(target, cur + accel * dt)
        return max(target, cur - decel * dt)

    def stop_motion(self) -> None:
        self.set_velocity_now(0.0, 0.0)

    def on_status(self, callback: Callable[[SystemStatus], None]) -> None:
        self._status_callbacks.append(callback)

    def on_motion_feedback(self, callback: Callable[[MotionFeedback], None]) -> None:
        self._motion_callbacks.append(callback)

    def on_bms(self, callback: Callable[[BmsFeedback], None]) -> None:
        self._bms_callbacks.append(callback)

    def on_odometer(self, callback: Callable[[OdometerFeedback], None]) -> None:
        self._odometer_callbacks.append(callback)

    @property
    def latest_status(self) -> Optional[SystemStatus]:
        return self._latest_status

    @property
    def latest_motion(self) -> Optional[MotionFeedback]:
        return self._latest_motion

    @property
    def latest_bms(self) -> Optional[BmsFeedback]:
        return self._latest_bms

    @property
    def latest_remote(self) -> Optional[RemoteControlFeedback]:
        return self._latest_remote

    @property
    def latest_odometer(self) -> Optional[OdometerFeedback]:
        """Return the best available wheel odometer (fused).

        融合策略（D-融合）：
          * 真实 0x311 刚推进过（≤0.5s）→ 直接返回真实值（硬件权威，
            mm 整数，无积分漂移）；
          * 真实推进过但已卡死（帧丢失 / bus 抖动）→ 返回
            ``最后一次真实值 + 合成里程增量``，里程读数继续前进，不再
            出现「卡住 0.5s 不动」的滞后窗口；
          * 真实帧在但**从未推进**（遥控模式 0x311 恒 0）→ 真实基准无效，
            返回纯合成里程；
          * 完全没见过真实帧且合成里程已积分 → 纯合成兜底。
        """
        real = self._latest_odometer
        real_alive = (
            self._real_odo_moved
            and (time.monotonic() - self._real_odo_moved_t) < 0.5
        )
        if real is not None and real_alive:
            return real
        if self._real_odo_moved:
            # 真实推进过 → 融合值（最后一次真实值 + 合成增量）无缝续接
            return OdometerFeedback(
                int(round(self._fused_left_mm
                          + (self._synth_left_mm - self._synth_ref_left))),
                int(round(self._fused_right_mm
                          + (self._synth_right_mm - self._synth_ref_right))),
            )
        if self._real_odo_key is not None:
            # 真实帧在但从未推进（遥控模式 0x311 恒 0）→ 真实基准无效
            if self._synth_moved:
                return OdometerFeedback(
                    int(round(self._synth_left_mm)),
                    int(round(self._synth_right_mm)),
                )
            return real
        if self._synth_moved:
            return OdometerFeedback(
                int(round(self._synth_left_mm)),
                int(round(self._synth_right_mm)),
            )
        return real

    @property
    def live_odometer(self) -> Optional[OdometerFeedback]:
        """Navigation odometer: always last-real + synth increment.

        ``latest_odometer`` 在真实 0x311 仍存活时直接返回硬件值，两帧之间
        的位移被丢掉，goto 会觉得自己没动。导航 / 扫描匹配必须用本属性，
        在 0x311 间隙用 0x221 积分补上。
        """
        if self._real_odo_moved:
            return OdometerFeedback(
                int(round(self._fused_left_mm
                          + (self._synth_left_mm - self._synth_ref_left))),
                int(round(self._fused_right_mm
                          + (self._synth_right_mm - self._synth_ref_right))),
            )
        if self._synth_moved:
            return OdometerFeedback(
                int(round(self._synth_left_mm)),
                int(round(self._synth_right_mm)),
            )
        return self._latest_odometer

    def _notify_live_odometer(self) -> None:
        odo = self.live_odometer
        if odo is None:
            return
        for callback in self._odometer_callbacks:
            callback(odo)

    @property
    def odometer_source(self) -> str:
        """'real' / 'fused' / 'synthetic' / 'none' — 诊断用当前里程来源。"""
        real = self._latest_odometer
        if real is not None and (
            self._real_odo_moved
            and (time.monotonic() - self._real_odo_moved_t) < 0.5
        ):
            return "real"
        if self._real_odo_moved:
            return "fused"
        if self._real_odo_key is not None:
            if self._synth_moved:
                return "synthetic"
            return "real"
        if self._synth_moved:
            return "synthetic"
        return "none"













    @property
    def tx_ok(self) -> int:
        return self._tx_ok

    @property
    def tx_fail(self) -> int:
        return self._tx_fail

    @property
    def last_tx_error(self) -> str:
        return self._last_tx_error

    @property
    def mailbox_stuck(self) -> bool:
        return self._mailbox_stuck

    def note_mailbox_recovered(self) -> None:
        self._mailbox_stuck = False

    @property
    def is_can_mode(self) -> bool:
        return (
            self._latest_status is not None
            and self._latest_status.control_mode == ControlMode.CAN_COMMAND
        )

    # ------------------------------------------------------------------
    # TX / RX loops
    # ------------------------------------------------------------------

    def _send(self, arbitration_id: int, data: bytes) -> None:
        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )
        self._bus.send(message)
        self._tx_ok += 1

    def _send_checked(self, arbitration_id: int, data: bytes, what: str) -> None:
        """One-shot send that converts a dead CAN link into an actionable error.

        The TX loop deliberately swallows errors and keeps retrying, but
        one-shot commands (mode switch / fault clear) must surface a clear
        message instead of a raw ``OSError: Network is down`` traceback.
        """
        try:
            self._send(arbitration_id, data)
        except Exception as exc:
            hint = connection_hint(
                canon_bustype(self._interface or "socketcan"),
                self._channel or "?",
            )
            raise CanConfigError(
                f"发送 CAN 指令失败（{what}）: {exc}\n{hint}"
            ) from exc




    def _maybe_clear_ghost_remote(self) -> None:
        """遥控器已关但 0x211 仍报遥控模式：清失联锁，让后续 0x421 生效。"""
        st = self._latest_status
        if st is None or st.control_mode == ControlMode.CAN_COMMAND:
            return
        if self.remote_alive and not self.remote_swb_command:
            return
        now = time.monotonic()
        if now - self._last_auto_clear_at < RC_CLEAR_PERIOD_S:
            return
        if not (
            int(st.fault_code or 0) & 0x04
            or st.control_mode == ControlMode.REMOTE_CONTROL
        ):
            return
        try:
            self._send(CanId.FAULT_CLEAR, encode_fault_clear(
                FaultClearCommand.CLEAR_NON_CRITICAL))
            self._last_auto_clear_at = now
        except Exception:
            logger.debug("ghost-remote 清故障失败", exc_info=True)

    @staticmethod
    def _is_mailbox_error(text: str) -> bool:
        return (
            "No buffer space" in text
            or "Error Code 105" in text
            or "Network is down" in text
            or "buffer full" in text.lower()
            or "Transmit buffer full" in text
        )

    def _tx_loop(self) -> None:
        next_send = time.monotonic()
        last_err = 0.0
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    cmd = self._command
                    payload = cmd.to_bytes()
                    hold_can = self._hold_can_mode
                heard = self._latest_status is not None
                # 手册 Table 3.5：未开遥控时默认待机，须周期发 0x421=0x01
                # （20ms，超时 500ms）后 0x111 才生效。
                # 没听到 0x211 = 总线上没有对端 ACK。空闲或按键都不能发：
                # 网页 100Hz 一按 WASD，gs_usb 发送邮箱立刻堵死（ENOBUFS），
                # 之后连探测都废。等底盘自己广播后再发。
                if not heard:
                    time.sleep(0.05)
                    next_send = time.monotonic() + CONTROL_PERIOD_S
                    continue
                self._send(CanId.MOTION_CONTROL, payload)
                if hold_can:
                    self._send(CanId.MODE_SETTING, encode_mode_setting(True))
                    self._maybe_clear_ghost_remote()
            except Exception as exc:
                now = time.monotonic()
                text = str(exc)
                self._tx_fail += 1
                self._last_tx_error = text
                bus_dead = self._is_mailbox_error(text)
                if bus_dead and now - self._last_mailbox_flag_at >= 3.0:
                    self._mailbox_stuck = True
                    self._last_mailbox_flag_at = now
                if now - last_err > (5.0 if bus_dead else 1.0):
                    hint = (
                        "（USB-CAN 发送邮箱已堵，按键帧出不去。启动时会清一次。）"
                        if bus_dead else ""
                    )
                    print(f"[controller] TX error: {exc}{hint}", file=sys.stderr)
                    last_err = now
                time.sleep(0.05 if bus_dead else 0.1)
                next_send = time.monotonic() + CONTROL_PERIOD_S
                continue

            next_send += CONTROL_PERIOD_S
            sleep_for = next_send - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_send = time.monotonic()

    def _rx_loop(self) -> None:
        last_err = 0.0
        while not self._stop_event.is_set():
            try:
                message = self._bus.recv(timeout=CONTROL_TIMEOUT_S)
            except Exception as exc:
                now = time.monotonic()
                # Never let the RX thread die silently — a dead RX thread
                # means stale battery/speed/fault data is reported forever.
                if now - last_err > 1.0:
                    print(f"[controller] RX error: {exc}", file=sys.stderr)
                    last_err = now
                time.sleep(0.1)
                continue
            if message is None:
                continue
            try:
                self._handle_message(message)
            except Exception:
                logger.exception("Failed to handle RX message 0x%X", message.arbitration_id)

    def _handle_message(self, message: can.Message) -> None:
        if message.arbitration_id == CanId.SYSTEM_STATUS:
            status = SystemStatus.from_bytes(bytes(message.data))
            self._latest_status = status
            for callback in self._status_callbacks:
                callback(status)
        elif message.arbitration_id == CanId.MOTION_FEEDBACK:
            motion = MotionFeedback.from_bytes(bytes(message.data))
            self._latest_motion = motion
            self._integrate_synthetic_odometer(motion)
            self._notify_live_odometer()
            for callback in self._motion_callbacks:
                callback(motion)



        elif message.arbitration_id == CanId.REMOTE_CONTROL:
            remote = RemoteControlFeedback.from_bytes(bytes(message.data))
            self._latest_remote = remote
            self._remote_seen_at = time.monotonic()
        elif message.arbitration_id == CanId.BMS:
            bms = BmsFeedback.from_bytes(bytes(message.data))
            self._latest_bms = bms
            for callback in self._bms_callbacks:
                callback(bms)
        elif message.arbitration_id == CanId.ODOMETER:
            odometer = OdometerFeedback.from_bytes(bytes(message.data))
            self._latest_odometer = odometer
            key = (odometer.left_wheel_mm, odometer.right_wheel_mm)
            if self._real_odo_key is None:
                # First real frame.  Seed the synthetic baseline only when we
                # have no motion yet; if the car is already moving (remote
                # control with 0x311 stuck at 0) keep the synthetic estimate.
                if not self._synth_moved:
                    self._synth_left_mm = float(odometer.left_wheel_mm)
                    self._synth_right_mm = float(odometer.right_wheel_mm)
                self._real_odo_key = key
                self._rebase_fused()
            elif key != self._real_odo_key:
                # Real odometer advanced → it is alive; rebase both the
                # synthetic estimate and the fused value so switching between
                # the two is seamless.
                self._real_odo_key = key
                self._real_odo_moved = True
                self._real_odo_moved_t = time.monotonic()
                self._synth_left_mm = float(odometer.left_wheel_mm)
                self._synth_right_mm = float(odometer.right_wheel_mm)
                self._rebase_fused()
            self._notify_live_odometer()

    def _rebase_fused(self) -> None:
        """Rebase the fused odometer onto the latest real 0x311 value.

        融合里程 = 真实值 + (合成里程 - 此刻合成基准)。真实推进时调用，
        让融合值严格等于真实值，合成只贡献真实卡死期间的增量。
        """
        real = self._latest_odometer
        if real is None:
            return
        self._fused_left_mm = float(real.left_wheel_mm)
        self._fused_right_mm = float(real.right_wheel_mm)
        self._synth_ref_left = self._synth_left_mm
        self._synth_ref_right = self._synth_right_mm










    def _integrate_synthetic_odometer(
        self, motion: MotionFeedback, now: Optional[float] = None
    ) -> None:
        """Integrate 0x221 wheel speeds into a synthetic odometer.

        Used as a fallback when the real 0x311 odometer never advances
        (remote-control driving on BUNKER MINI).  Differential drive:
          v_left  = v - w * wheelbase/2
          v_right = v + w * wheelbase/2
        distance_mm += v * dt * 1000.
        """
        if now is None:
            now = time.monotonic()
        if self._synth_t is not None:
            dt = now - self._synth_t
            # Ignore implausible gaps (frames dropped / bus paused) so a long
            # silence does not inject a giant distance spike.
            if 0.0 < dt < 1.0:
                wb = self._wheelbase_m
                v_l = motion.linear_velocity_m_s - motion.angular_velocity_rad_s * wb / 2.0
                v_r = motion.linear_velocity_m_s + motion.angular_velocity_rad_s * wb / 2.0
                self._synth_left_mm += v_l * dt * 1000.0
                self._synth_right_mm += v_r * dt * 1000.0
                if abs(v_l) > 1e-3 or abs(v_r) > 1e-3:
                    self._synth_moved = True
        self._synth_t = now








