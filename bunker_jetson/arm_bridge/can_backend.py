"""CAN 报文后端：机械臂控制器并入底盘同一 socketcan 总线，发送约定帧表示
抓取完成。

**协议约定（机械臂方）**：发送标准帧 ID ``0x3A1``（默认，可配置），数据
第 0 字节为 ``0x01`` 表示「抓取完成」。（沿用 CAN 大端、8 字节定长风格；
机械臂方若用不同 ID/字节，通过 ``frame_id`` / ``ok_byte`` 对齐即可。）

复用 ``python-can``（项目已有依赖），与底盘控制器各自独立占用一条总线
实例，互不干扰。帧 ID ``0x3A1`` 落在底盘协议使用范围（0x211~0x441）之外，
不会误解析。
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from .feedback import ArmFeedbackBackend, ArmFeedbackEvent

logger = logging.getLogger(__name__)

#: 默认抓取完成帧 ID（标准帧，与底盘 0x111/0x211/... 不冲突）
DEFAULT_GRASP_FRAME_ID = 0x3A1


class CanArmBackend(ArmFeedbackBackend):
    """CAN 报文信号通道：收到 frame_id 且 data[0]==ok_byte 即 GRASPED。"""

    name = "can"

    def __init__(
        self,
        channel: str | None = None,
        interface: str = "socketcan",
        frame_id: int = DEFAULT_GRASP_FRAME_ID,
        ok_byte: int = 0x01,
    ) -> None:
        self._channel = channel
        self._interface = interface
        self._frame_id = int(frame_id)
        self._ok_byte = ok_byte
        self._bus = None

    def start(self) -> None:
        import can  # 延迟导入：未启用该通道时不拉依赖

        self._bus = can.Bus(channel=self._channel, interface=self._interface)
        logger.info("armfb/can: 监听 %s 上帧 0x%03X (data[0]==0x%02X)",
                    self._channel or "自动通道", self._frame_id, self._ok_byte)

    def stop(self) -> None:
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None

    def wait_event(self, timeout_s: float) -> Optional[ArmFeedbackEvent]:
        bus = self._bus
        if bus is None:
            return None
        deadline = time.monotonic() + timeout_s
        while True:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                return None
            try:
                message = bus.recv(timeout=min(remaining_s, 0.25))
            except Exception:
                return None
            if message is None:
                continue
            if message.arbitration_id == self._frame_id and message.data:
                if message.data[0] == self._ok_byte:
                    return ArmFeedbackEvent.GRASPED
