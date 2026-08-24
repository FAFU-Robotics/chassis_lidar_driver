"""GPIO 电平后端：机械臂控制器拉高/拉低 Jetson 40-pin 一路 IO 表示抓取完成。

**接线约定（机械臂方）**：抓取完成时把目标引脚置为约定的有效电平（默认
高电平有效）。Jetson 侧用 ``Jetson.GPIO`` 边沿检测等待，边沿出现后再做
短暂消抖确认（防止电平抖动误触发）。

依赖 ``Jetson.GPIO``（``pip install Jetson.GPIO``）。库缺失或引脚非法时
``start`` 抛异常，由桥接层捕获并降级为「该通道不可用」，不影响其他通道。
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from .feedback import ArmFeedbackBackend, ArmFeedbackEvent

logger = logging.getLogger(__name__)


class GpioArmBackend(ArmFeedbackBackend):
    """Jetson GPIO 电平信号通道（边沿检测 + 消抖）。"""

    name = "gpio"

    def __init__(
        self,
        pin: int = 18,
        active_high: bool = True,
        debounce_s: float = 0.15,
    ) -> None:
        self._pin = int(pin)
        self._active_high = bool(active_high)
        self._debounce_s = debounce_s
        self._gpio = None  # Jetson.GPIO 模块（start 时加载）
        self._lock = threading.Lock()

    def start(self) -> None:
        try:
            import Jetson.GPIO as gpio
        except ImportError as exc:
            raise RuntimeError(
                "Jetson.GPIO 未安装（pip install Jetson.GPIO）—— GPIO 信号通道不可用"
            ) from exc
        with self._lock:
            self._gpio = gpio
            gpio.setmode(gpio.BOARD)
            gpio.setup(self._pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        logger.info("armfb/gpio: 监听 BOARD pin %d（%s 有效）",
                    self._pin, "高电平" if self._active_high else "低电平")

    def stop(self) -> None:
        with self._lock:
            gpio = self._gpio
            if gpio is not None:
                try:
                    gpio.cleanup(self._pin)
                except Exception:
                    logger.debug("armfb/gpio: cleanup 失败（忽略）")
                self._gpio = None

    def wait_event(self, timeout_s: float) -> Optional[ArmFeedbackEvent]:
        gpio = self._gpio
        if gpio is None:
            return None
        edge = gpio.RISING if self._active_high else gpio.FALLING
        deadline = time.monotonic() + timeout_s
        while True:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                return None
            remaining_ms = max(5, int(remaining_s * 1000))
            try:
                # 阻塞等待边沿（内核事件，非忙轮询）
                if gpio.wait_for_edge(self._pin, edge, timeout=remaining_ms) is None:
                    return None
            except Exception:
                return None
            # 消抖：有效电平必须持续 debounce_s 才确认
            if self._confirm_level(gpio, edge):
                return ArmFeedbackEvent.GRASPED
            # 消抖失败 → 继续等下一次边沿（重新计算剩余时间）

    def _confirm_level(self, gpio, edge: str) -> bool:
        """边沿后确认电平稳定持续 debounce_s。"""
        try:
            level = gpio.input(self._pin)
        except Exception:
            return False
        stable_t = time.monotonic()
        while time.monotonic() - stable_t < self._debounce_s:
            time.sleep(0.02)
            try:
                if gpio.input(self._pin) != level:
                    return False  # 抖动了，放弃本次
            except Exception:
                return False
        return True
