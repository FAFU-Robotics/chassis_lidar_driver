#!/usr/bin/env python3
"""工控机低延迟 TCP 遥操服务（systemd: bunker-teleop.service）。

笔记本 HID → TCP :9100 → 本进程 → set_velocity_now → CAN 50 Hz。
不经过 SSH 字符流，也不经过 mock_cloud WebSocket JSON。

两种跑法:
  1) 车侧代理已启动：不必再开本进程。agent 默认会在 9100 拉起同一套服务。
  2) 只要遥控、不开完整代理：本进程 --direct-can，由 systemd 拉起。

    python3 teleop_tcp_server.py --direct-can
    python3 teleop_tcp_server.py --direct-can --bind 0.0.0.0 --port 9100
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.can_util import CanConfigError, add_can_cli_args
from bunker_mini.controller import BunkerMiniController
from bunker_mini.teleop_tcp import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpServer

logger = logging.getLogger("teleop-tcp")


def main() -> int:
    parser = argparse.ArgumentParser(description="Bunker 低延迟 TCP 遥操服务")
    add_can_cli_args(parser)
    parser.add_argument("--bind", default=os.environ.get("BUNKER_TELEOP_HOST", DEFAULT_HOST))
    parser.add_argument("--port", type=int, default=int(os.environ.get("BUNKER_TELEOP_PORT", str(DEFAULT_PORT))))
    parser.add_argument("--token", default=os.environ.get("BUNKER_TELEOP_TOKEN", DEFAULT_TOKEN))
    parser.add_argument(
        "--direct-can",
        action="store_true",
        help="本进程独占 CAN（不要和正在跑的 agent 同时开）",
    )
    parser.add_argument("--no-can", action="store_true", help="只测 TCP，不下发 CAN")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    controller: BunkerMiniController | None = None
    if args.direct_can and not args.no_can:
        try:
            controller = BunkerMiniController(channel=args.channel, interface=args.interface)
            controller.start()
            controller.enable_can_control()
            time.sleep(0.15)
        except CanConfigError as exc:
            logger.error("CAN 配置错误:\n%s", exc)
            return 2
        logger.info("CAN 已占用（direct-can）")

    def on_stick(v: float, w: float) -> None:
        if controller is not None:
            if abs(v) < 1e-6 and abs(w) < 1e-6:
                controller.stop_motion()
            else:
                controller.set_velocity_now(v, w)

    def on_estop() -> None:
        if controller is not None:
            controller.stop_motion()

    def on_query() -> tuple[float, float]:
        if controller is None:
            return 0.0, 0.0
        fb = controller.latest_motion
        if fb is None:
            return 0.0, 0.0
        return fb.linear_velocity_m_s, fb.angular_velocity_rad_s

    server = TeleopTcpServer(
        host=args.bind,
        port=args.port,
        token=args.token,
        on_stick=on_stick,
        on_estop=on_estop,
        on_idle_stop=on_estop,
        on_query=on_query,
        on_move=lambda v, w, _d, _b: on_stick(v, w),
    )
    server.start()
    logger.info(
        "teleop TCP 监听 %s:%s token=%s  （笔记本: python3 teleop_tcp_client.py --host <本机IP>）",
        args.bind, args.port, args.token,
    )

    stop = False

    def _stop(*_a) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        while not stop:
            time.sleep(0.5)
    finally:
        server.stop()
        if controller is not None:
            try:
                controller.stop_motion()
                controller.stop()
            except Exception:
                logger.exception("关闭 CAN 失败")
        logger.info("teleop TCP 已退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
