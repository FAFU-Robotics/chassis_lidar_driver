#!/usr/bin/env python3
"""SocketCAN 底盘适配层（Jetson / Linux 版）。

替代 Windows 版的 ``gs_usb_chassis.py``：通过 Linux 内核 SocketCAN 与
USB-CAN（gs_usb / candleLight）通信，屏蔽底层 python-can 差异，给
``run_obstacle_avoidance.py`` 等入口脚本一个一致的接口。

关键点：USB-CAN 接口名（can0/can1…）在重启后不稳定，绝不能硬编码。
``detect_gs_usb_channel()`` 扫描 ``/sys/class/net`` 找出真正挂在 USB 总线上
的 can 网卡（``can_util.detect_socketcan_channel`` 的实现），找不到时退化为
任意 can 网卡，再退化为 can0。
"""

from __future__ import annotations

import time
from typing import Optional

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.can_util import (
    detect_socketcan_channel,
    ensure_socketcan_interface,
    format_socketcan_status,
    list_socketcan_interfaces,
)
from bunker_mini.controller import BunkerMiniController


def detect_gs_usb_channel() -> Optional[str]:
    """Return the USB-backed can* interface (the chassis lives there).

    扫描 ``/sys/class/net`` 里所有 ``can*``，选第一个 device 路径解析到
    USB 总线的网卡；没有 USB 网卡时退回任意 can 网卡，最后退化 can0。
    """
    return detect_socketcan_channel()


class SocketcanBunkerController:
    """BunkerMiniController 的轻量包装：握手 + 速度指令 + 状态读取。

    用法::

        robot = SocketcanBunkerController(interface="socketcan", channel="can1")
        robot.start()
        if not robot.enable_and_handshake(timeout_s=5.0):
            print("使能失败")
            robot.stop()
            sys.exit(2)
        robot.set_velocity(0.2, 0.0)   # 巡航线速度
        ... 每帧调 set_velocity ...
        robot.stop_motion()
        robot.stop()
    """

    def __init__(self, interface: str = "socketcan",
                 channel: Optional[str] = None) -> None:
        self._interface = interface
        self._channel = channel
        self._controller: Optional[BunkerMiniController] = None

    # ------------------------------------------------------------------
    @property
    def controller(self) -> Optional[BunkerMiniController]:
        return self._controller

    @property
    def channel(self) -> Optional[str]:
        return self._channel

    # ------------------------------------------------------------------
    def start(self) -> None:
        channel = self._channel or detect_gs_usb_channel() or "can0"
        self._channel = channel
        # 自动把 DOWN 的接口 bring up；起不来直接给可操作报错
        ensure_socketcan_interface(channel)
        self._controller = BunkerMiniController(channel=channel, interface=self._interface)
        self._controller.start()

    def stop(self) -> None:
        if self._controller is not None:
            try:
                self._controller.stop_motion()
                time.sleep(0.05)
            finally:
                self._controller.stop()
                self._controller = None

    # ------------------------------------------------------------------
    def wait_heartbeat(self, timeout_s: float = 5.0) -> bool:
        """等待底盘 0x211 心跳（SystemStatus 至少来一帧）。"""
        if self._controller is None:
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._controller.latest_status is not None:
                return True
            time.sleep(0.05)
        return False

    def enable_and_handshake(self, timeout_s: float = 5.0) -> bool:
        channel = getattr(self, "_channel", None)
        print("[底盘诊断] 等待 0x211（socketcan）…")
        if not self.wait_heartbeat(timeout_s):
            hint = f"candump {channel}" if channel else "candump can0"
            print(
                f"[底盘诊断] ❌ 未收到心跳。检查: bash bringup_gs_usb_can.sh && {hint}"
            )
            return False
        if self._controller is not None:
            # 切到 CAN 指令模式 + 清非关键故障（遥控器失联保护等）
            self._controller.enable_can_control()
            time.sleep(0.05)
            self._controller.clear_faults()
        return True

    # ------------------------------------------------------------------
    def set_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        if self._controller is not None:
            self._controller.set_velocity(linear_m_s, angular_rad_s)

    def stop_motion(self) -> None:
        if self._controller is not None:
            self._controller.stop_motion()

    # ------------------------------------------------------------------
    @property
    def latest_status(self):
        return self._controller.latest_status if self._controller else None

    @property
    def latest_motion(self):
        return self._controller.latest_motion if self._controller else None

    @property
    def latest_odometer(self):
        return self._controller.latest_odometer if self._controller else None

    @property
    def latest_bms(self):
        return self._controller.latest_bms if self._controller else None

    @property
    def is_can_mode(self) -> bool:
        return bool(self._controller and self._controller.is_can_mode)

    # ------------------------------------------------------------------
    def __repr__(self) -> str:  # pragma: no cover
        return (f"SocketcanBunkerController(channel={self._channel!r}, "
                f"interface={self._interface!r})")


def print_can_status() -> None:
    """列出系统当前所有 CAN 网卡及 USB 归属（诊断用）。"""
    ifaces = list_socketcan_interfaces()
    print("[底盘诊断] 检测到的 CAN 网卡:", ", ".join(ifaces) or "（无）")
    print(format_socketcan_status())


if __name__ == "__main__":
    print_can_status()
    ch = detect_gs_usb_channel()
    print("[底盘诊断] USB-CAN 通道 →", ch)
