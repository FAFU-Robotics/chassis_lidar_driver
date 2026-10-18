"""云端指令后端：机械臂控制器（或其操作员）经云端下发 ``grasp_done`` 指令。

这是**零硬件**的信号通道——机械臂方只需在抓取完成后把 ``{"action":
"grasp_done"}`` 经甲方云端（或联调用 ``mock_cloud``）下发给底盘 agent。
agent 收到指令后调用桥接层的 :meth:`ArmBridge.notify_grasped`，正在
``wait_grasp`` 的等待立即返回 GRASPED。

本后端自身不轮询，只作为「已启用通道」占位并暴露置位入口。
"""

from __future__ import annotations

import threading
from typing import Optional

from .feedback import ArmFeedbackBackend, ArmFeedbackEvent


class WsArmBackend(ArmFeedbackBackend):
    """云端 ``grasp_done`` 指令通道。"""

    name = "ws"

    def __init__(self, grasped_event: Optional[threading.Event] = None) -> None:
        # 由桥接层注入共享事件；未注入时自建（外部无法置位，仅占位）
        self._grasped = grasped_event or threading.Event()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def wait_event(self, timeout_s: float) -> Optional[ArmFeedbackEvent]:
        # 外部经 notify_grasped 置位，本通道不轮询硬件
        return None

    def notify_grasped(self) -> None:
        """外部（agent 收到 grasp_done 指令）调用的置位入口。"""
        self._grasped.set()
