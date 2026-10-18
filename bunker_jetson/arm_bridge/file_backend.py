"""联调文件后端：以文件内容作为抓取完成信号（无硬件即可端到端测试）。

机械臂方（或测试脚本）向指定文件写入 ``1`` / ``grasped`` / ``done`` / ``true``
（任意大小写、可带空白）即视为抓取完成。适用于：
  * 开发机离线验证整条任务链（探路→对接→等待抓取→返回）；
  * 甲方现场用机械臂控制器脚本写文件替代硬件接线快速联调。

也可配合 :file:`simulate_arm.py` 使用。
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from .feedback import ArmFeedbackBackend, ArmFeedbackEvent

_GRASPED_TOKENS = frozenset({"1", "grasped", "done", "true", "ok"})


class FileArmBackend(ArmFeedbackBackend):
    """文件信号通道：文件内容为完成标记即返回 GRASPED。"""

    name = "file"

    def __init__(self, path: str, poll_s: float = 0.1) -> None:
        self._path = path
        self._poll_s = poll_s

    def start(self) -> None:
        # 通道就绪即可（文件不存在也允许——机械臂方稍后创建/写入）
        return None

    def stop(self) -> None:
        pass

    def wait_event(self, timeout_s: float) -> Optional[ArmFeedbackEvent]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                text = Path(self._path).read_text(encoding="utf-8").strip().lower()
                if text in _GRASPED_TOKENS:
                    return ArmFeedbackEvent.GRASPED
            except FileNotFoundError:
                pass
            except OSError:
                pass
            time.sleep(self._poll_s)
        return None
