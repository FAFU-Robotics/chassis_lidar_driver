"""机械臂 → 底盘「抓取完成」反馈信号抽象层。

任务链（``find_object``）在机械臂就位（``grasp_ready``）后，需要**停稳等待
机械臂真正完成抓取**再返回起点。本模块定义统一的反馈事件与后端接口：

  * :class:`ArmFeedbackEvent` —— 机械臂反馈事件（当前只有 GRASPED 会触发返回）
  * :class:`ArmFeedbackBackend` —— 单条信号通道的抽象（GPIO / CAN / 云端指令 /
    联调文件…），各自实现 ``wait_event`` 阻塞等待
  * :class:`GraspWaitResult` —— 桥接层 ``wait_grasp`` 的返回结果

多通道可同时启用，任一通道先报告 ``GRASPED`` 即视为抓取完成（取最早到达者），
避免单条通道失效（断线/接线松/误触发）导致任务卡死。

接线/协议约定见 :file:`arm_bridge/README.md`（机械臂方对照接入）。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional


class ArmFeedbackEvent(Enum):
    """机械臂反馈事件。"""

    GRASPING = "grasping"  # 机械臂开始抓取（可选信号，暂不参与状态流转）
    GRASPED = "grasped"    # 抓取完成 → 底盘应停止等待并返回起点


class GraspWaitResult(Enum):
    """``ArmBridge.wait_grasp`` 的返回结果。"""

    GRASPED = "grasped"        # 收到抓取完成信号
    TIMEOUT = "timeout"        # 等待超时
    CANCELLED = "cancelled"    # 任务被取消 / 急停 / 桥接层停止
    DISABLED = "disabled"      # 未配置任何信号通道


class ArmFeedbackBackend(ABC):
    """单条机械臂反馈通道（GPIO / CAN / 云端指令 / 联调文件…）。"""

    #: 通道名，用于日志与 ``parse_arm_signal_spec`` 识别
    name: str = "base"

    @abstractmethod
    def start(self) -> None:
        """打开通道。失败应抛异常（由桥接层捕获并降级为不可用）。"""

    @abstractmethod
    def stop(self) -> None:
        """关闭通道（幂等）。"""

    @abstractmethod
    def wait_event(self, timeout_s: float) -> Optional[ArmFeedbackEvent]:
        """阻塞等待事件，最多 ``timeout_s`` 秒；超时返回 ``None``。

        实现应在内部高效阻塞（GPIO 用 ``wait_for_edge``、CAN 用 ``recv``
        带超时、文件用短轮询），由桥接层以 0.25 s 为片调用以检查取消。
        """
