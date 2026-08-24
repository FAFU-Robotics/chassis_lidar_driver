"""机械臂「抓取完成」信号桥接层：多通道等待 + 信号规格解析。

:class:`ArmBridge` 聚合若干 :class:`ArmFeedbackBackend`，任一通道报告
``GRASPED`` 即触发返回（取最早者）。``wait_grasp`` 支持超时与取消：

  * 收到抓取完成 → :data:`GraspWaitResult.GRASPED`
  * 超时未收到 → :data:`GraspWaitResult.TIMEOUT`（车已停稳，由上层决定）
  * 任务被取消（``cancel`` / ``estop``）→ :data:`GraspWaitResult.CANCELLED`

信号规格字符串（``--arm-grasp-signal`` / 配置项 ``ARM_GRASP_SIGNAL``）：
逗号分隔的多通道列表，例::

    ws                                     # 仅云端 grasp_done 指令
    ws,file:/tmp/grasp.sig                 # 云端 + 联调文件
    ws,can:can1:0x3A1                      # 云端 + CAN 报文（can1，帧 0x3A1）
    ws,gpio:18:high                        # 云端 + GPIO BOARD pin18 高电平有效
    ws,can:can0,gpio:18:low                # 全通道
    none / ""                              # 关闭机械臂等待（旧行为：就位即返回）

规格格式::

    ws
    file:<路径>
    can:<通道>[:<帧ID:hex|dec>]
    gpio:<引脚>[:high|low]
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Iterable, Optional

from .can_backend import CanArmBackend
from .feedback import ArmFeedbackBackend, ArmFeedbackEvent, GraspWaitResult
from .file_backend import FileArmBackend
from .gpio_backend import GpioArmBackend
from .ws_backend import WsArmBackend

logger = logging.getLogger(__name__)


class ArmBridge:
    """聚合多条机械臂反馈通道，提供「等待抓取完成」语义。"""

    def __init__(
        self,
        backends: Iterable[ArmFeedbackBackend] = (),
        wait_timeout_s: float = 90.0,
    ) -> None:
        self._backends: list[ArmFeedbackBackend] = list(backends)
        self._wait_timeout_s = wait_timeout_s
        self._grasped = threading.Event()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._started = False

    # -- 通道管理 ------------------------------------------------------

    @property
    def enabled(self) -> bool:
        """是否有任何可用信号通道（决定任务链是否进入「等待抓取」）。"""
        return bool(self._backends)

    @property
    def backends(self) -> list[ArmFeedbackBackend]:
        return list(self._backends)

    def start(self) -> None:
        if self._started:
            return
        self._stop.clear()
        for backend in self._backends:
            try:
                backend.start()
            except Exception as exc:
                logger.warning("armfb: 通道 %s 启动失败，已禁用: %s",
                               backend.name, exc)
                continue
            thread = threading.Thread(
                target=self._pump, args=(backend,),
                name=f"armfb-{backend.name}", daemon=True)
            thread.start()
            self._threads.append(thread)
            logger.info("armfb: 通道 %s 已启用", backend.name)
        self._started = True

    def reset(self) -> None:
        """开始新一轮任务链前调用：清除上一轮的抓取信号。

        ``_grasped`` 在 ``start()`` 里**不**清除——若机械臂在进入等待前
        （如操作员提前手动下发 ``grasp_done``）已置位，等待开始后立即生效，
        不会丢失早到的信号。
        """
        self._grasped.clear()

    def stop(self) -> None:
        self._stop.set()
        self._grasped.set()  # 唤醒正在等待的调用方
        for thread in self._threads:
            thread.join(timeout=0.5)
        self._threads.clear()
        for backend in self._backends:
            try:
                backend.stop()
            except Exception:
                pass
        self._started = False

    def notify_grasped(self, source: str = "external") -> None:
        """外部置位抓取完成（云端 ``grasp_done`` 指令 / 操作员确认）。"""
        logger.info("armfb: 收到外部抓取完成信号（%s）", source)
        self._grasped.set()

    # -- 等待语义 ------------------------------------------------------

    def wait_grasp(self, timeout_s: Optional[float] = None,
                   cancel: Optional[threading.Event] = None) -> GraspWaitResult:
        """阻塞等待抓取完成。

        :param timeout_s: 覆盖构造时的默认超时；None 用默认值。
        :param cancel: 可选的取消事件（任务 stop event），置位立即返回
            CANCELLED（配合 ``cancel``/``estop`` 及时打断，而非等满超时）。
        """
        if not self.enabled:
            return GraspWaitResult.DISABLED
        timeout = self._wait_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            if self._stop.is_set():
                return GraspWaitResult.CANCELLED
            if cancel is not None and cancel.is_set():
                return GraspWaitResult.CANCELLED
            if self._grasped.is_set():
                return GraspWaitResult.GRASPED
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return GraspWaitResult.TIMEOUT
            self._grasped.wait(min(0.25, remaining))

    # -- 内部 ----------------------------------------------------------

    def _pump(self, backend: ArmFeedbackBackend) -> None:
        """轮询单个通道，GRASPED 即置位共享事件并退出。"""
        while not self._stop.is_set():
            try:
                event = backend.wait_event(0.25)
            except Exception:
                logger.exception("armfb: 通道 %s 轮询异常", backend.name)
                return
            if event == ArmFeedbackEvent.GRASPED:
                logger.info("armfb: 通道 %s 报告抓取完成", backend.name)
                self._grasped.set()
                return


def _parse_int(value: str, default: int, what: str = "值") -> int:
    try:
        return int(value, 0) if value.lower().startswith(("0x", "0X")) \
            else int(value)
    except ValueError:
        logger.warning("armfb: %s 解析失败（%r），使用默认 %d", what, value, default)
        return default


def parse_arm_signal_spec(spec: str) -> list[ArmFeedbackBackend]:
    """把 ``--arm-grasp-signal`` 规格字符串解析为后端列表。

    空串 / ``none`` → 空列表（不启用机械臂等待）。
    """
    spec = (spec or "").strip().lower()
    if not spec or spec == "none":
        return []
    backends: list[ArmFeedbackBackend] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if part == "ws":
            backends.append(WsArmBackend())
        elif part.startswith("file:"):
            path = part[len("file:"):].strip()
            if path:
                backends.append(FileArmBackend(path))
            else:
                logger.warning("armfb: file: 缺少路径，跳过")
        elif part.startswith("can:"):
            rest = part[len("can:"):].strip()
            channel = rest
            frame_id = 0x3A1
            if ":" in rest:
                channel, _, id_str = rest.partition(":")
                frame_id = _parse_int(id_str, 0x3A1, "CAN 帧 ID")
            backends.append(CanArmBackend(channel=channel or None,
                                          frame_id=frame_id))
        elif part.startswith("gpio:"):
            rest = part[len("gpio:"):].strip()
            pin = 18
            active_high = True
            if ":" in rest:
                pin_str, _, pol = rest.partition(":")
                pin = _parse_int(pin_str, 18, "GPIO 引脚")
                active_high = (pol != "low")
            else:
                pin = _parse_int(rest, 18, "GPIO 引脚")
            backends.append(GpioArmBackend(pin=pin, active_high=active_high))
        else:
            logger.warning("armfb: 未知信号规格段 %r，跳过（可选: ws / "
                           "file:路径 / can:通道[:帧ID] / gpio:引脚[:high|low]）",
                           part)
    return backends
