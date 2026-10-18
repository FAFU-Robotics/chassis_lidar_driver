"""机械臂 → 底盘「抓取完成」信号桥接包。

任务链在机械臂就位后等待抓取完成信号再返回。支持多通道信号：
云端 ``grasp_done`` 指令 / CAN 报文 / GPIO 电平 / 联调文件。

用法见 :file:`README.md`（机械臂方接入说明）与 ``simulate_arm.py``（联调模拟）。
"""

from .arm_bridge import ArmBridge, parse_arm_signal_spec
from .feedback import ArmFeedbackEvent, GraspWaitResult

__all__ = [
    "ArmBridge",
    "parse_arm_signal_spec",
    "ArmFeedbackEvent",
    "GraspWaitResult",
]
