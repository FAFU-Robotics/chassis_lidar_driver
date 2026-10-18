#!/usr/bin/env python3
"""Car-side Agent entry point — run this on the vehicle's onboard computer.

日常请从仓库根目录 ``python3 run_local.py`` 拉起本进程的 ``--local`` 模式。
WebSocket 云端通道已废弃，不建议再传 ``--ws-url`` 去连 mock_cloud / 甲方云。

Usage::

    # 推荐：由根目录 run_local.py 调用 --local
    python3 run_agent.py --local --config agent.conf

    # 已废弃的云端入口（不建议）:
    python3 run_agent.py --config agent.conf

    # Minimal (uses env vars / auto-detection for CAN):
    python3 run_agent.py --ws-url wss://your-cloud.example.com/ws

    # Full (SocketCAN on Jetson Linux):
    python3 run_agent.py \
        --ws-url wss://your-cloud.example.com/ws \
        --device-id BUNKER-TEST01 \
        --bind-code TEST-BIND-xxxx \
        --interface socketcan --channel can1

    # With env vars (Linux export):
    export BUNKER_WS_URL="wss://your-cloud.example.com/ws"
    export BUNKER_DEVICE_ID="BUNKER-TEST01"
    export BUNKER_BIND_CODE="TEST-BIND-xxxx"
    python3 run_agent.py

参数优先级：命令行参数 > 环境变量 > 配置文件 > 内置默认值。
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.agent import BunkerMiniAgent
from bunker_mini.can_util import CanConfigError, add_can_cli_args, format_device_list, print_can_config_help
from bunker_mini.navigator import load_wheelbase_m
from bunker_mini.terrain import DEFAULT_STEP_LIMIT_M


def _env_or(name: str, default: str = "") -> str:
    return os.environ.get(name, default)


def _load_config_file(path: str) -> None:
    """Load a ``KEY=VALUE`` config file into os.environ (never overrides
    environment variables that are already set)."""
    if not os.path.exists(path):
        print(f"警告: 配置文件不存在，跳过: {path}", file=sys.stderr)
        return
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip())


def main() -> int:
    parser = argparse.ArgumentParser(
        description="BUNKER MINI 2.0 车侧代理（推荐 --local；WebSocket 云端已废弃）",
    )

    # ---- 第一遍解析：只取 --config，把配置文件注入环境变量 ----
    # 之后第二遍完整解析时，参数默认值即可读到配置文件里的值。
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", default=None)
    pre_args, _ = pre.parse_known_args()
    if pre_args.config:
        _load_config_file(pre_args.config)
    elif os.path.exists("agent.conf"):
        _load_config_file("agent.conf")

    add_can_cli_args(parser)

    parser.add_argument(
        "--config",
        default=None,
        help="配置文件路径（KEY=VALUE 格式，示例见 agent.conf；默认自动读取当前目录 agent.conf）",
    )
    parser.add_argument(
        "--ws-url",
        default=_env_or("BUNKER_WS_URL"),
        help="Cloud WebSocket URL (env: BUNKER_WS_URL)",
    )
    parser.add_argument(
        "--device-id",
        default=_env_or("BUNKER_DEVICE_ID", "BUNKER-TEST01"),
        help="Unique vehicle ID (env: BUNKER_DEVICE_ID, default: BUNKER-TEST01)",
    )
    parser.add_argument(
        "--bind-code",
        default=_env_or("BUNKER_BIND_CODE", "TEST-BIND-xxxx"),
        help="Binding code for auth (env: BUNKER_BIND_CODE, default: TEST-BIND-xxxx)",
    )
    parser.add_argument(
        "--track-dir",
        default=_env_or("BUNKER_TRACK_DIR"),
        help="轨迹文件保存目录 (env: BUNKER_TRACK_DIR, 默认 ./tracks)",
    )
    parser.add_argument(
        "--log-file",
        default=_env_or("BUNKER_LOG_FILE"),
        help="日志输出文件路径，留空则只输出到控制台 (env: BUNKER_LOG_FILE)",
    )
    parser.add_argument(
        "--log-level",
        default=_env_or("BUNKER_LOG_LEVEL", "INFO"),
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )
    parser.add_argument(
        "--enable-lidar",
        dest="enable_lidar",
        action="store_true",
        default=_env_or("BUNKER_ENABLE_LIDAR", "1") not in ("0", "false", "False", "no"),
        help="启用 Airy 激光雷达避障/导航（默认开启；设置 BUNKER_ENABLE_LIDAR=0 或 --no-lidar 关闭）",
    )
    parser.add_argument(
        "--no-lidar",
        dest="enable_lidar",
        action="store_false",
        help="关闭激光雷达避障/导航",
    )
    parser.add_argument(
        "--lidar-port",
        type=int,
        default=int(_env_or("BUNKER_LIDAR_PORT", "6699")),
        help="雷达 MSOP UDP 端口 (env: BUNKER_LIDAR_PORT, 默认 6699)",
    )
    parser.add_argument(
        "--lidar-pcap",
        default=_env_or("BUNKER_LIDAR_PCAP"),
        help="离线回放：用 PCAP 抓包文件替代 UDP 雷达（B 迁移能力，无雷达也可跑通避障/导航；env: BUNKER_LIDAR_PCAP）",
    )
    _src = _env_or("BUNKER_LIDAR_SOURCE", "auto").strip().lower()
    if _src not in ("auto", "udp", "ros"):
        _src = "auto"
    parser.add_argument(
        "--lidar-source",
        default=_src,
        choices=["auto", "udp", "ros"],
        help="雷达数据源：auto=6699 被占则订 /rslidar_points；udp=只绑 MSOP；ros=只订话题 (env: BUNKER_LIDAR_SOURCE)",
    )
    parser.add_argument(
        "--lidar-mount-yaw",
        type=float,
        default=float(_env_or("BUNKER_LIDAR_MOUNT_YAW", "0")),
        help="雷达 0° 相对车头的偏置角（度，左转为正，env: BUNKER_LIDAR_MOUNT_YAW, 默认 0）",
    )
    parser.add_argument(
        "--lidar-pitch",
        type=float,
        default=float(_env_or("BUNKER_LIDAR_PITCH_DEG", "0")),
        help="雷达安装俯仰角（度，正=抬头，用于 Airy 立装时的倾角补偿；env: BUNKER_LIDAR_PITCH_DEG, 默认 0）",
    )
    parser.add_argument(
        "--lidar-height",
        type=float,
        default=float(_env_or("BUNKER_LIDAR_HEIGHT_M", "0")),
        help="雷达光心离地高度（米），点云 z 平移为离地高度（F 迁移；env: BUNKER_LIDAR_HEIGHT_M, 默认 0）",
    )
    parser.add_argument(
        "--no-lidar-self-mask",
        dest="lidar_self_mask",
        action="store_false",
        default=_env_or("BUNKER_LIDAR_SELF_MASK", "1") not in ("0", "false", "False", "no"),
        help="关闭自扫硬件过滤（默认开启；设置 BUNKER_LIDAR_SELF_MASK=0 或 --no-lidar-self-mask 关闭）",
    )
    parser.add_argument(
        "--wheelbase",
        type=float,
        default=float(_env_or("BUNKER_WHEELBASE") or load_wheelbase_m()),
        help="左右轮间距（米），导航航迹推算用 "
             "(env: BUNKER_WHEELBASE，否则 bunker_jetson/wheelbase.local，默认 0.5)",
    )
    parser.add_argument(
        "--step-limit",
        type=float,
        default=float(_env_or("BUNKER_STEP_LIMIT", str(DEFAULT_STEP_LIMIT_M))),
        help="允许的最大台阶/坑深度（米），超过则判定不可通行需绕行；"
             "实车离地约 80 mm，碾过 ≤3–4 cm（env: BUNKER_STEP_LIMIT，默认 0.04）。"
             "实验室椅子场景另设 BUNKER_OA_SCENE=office",
    )
    parser.add_argument(
        "--recon-max-duration",
        type=float,
        default=float(_env_or("BUNKER_RECON_MAX_DURATION", "120")),
        help="自主探路最长时长（秒），超时沿轨迹返回起点 (env: BUNKER_RECON_MAX_DURATION, 默认 120)",
    )
    parser.add_argument(
        "--recon-max-distance",
        type=float,
        default=float(_env_or("BUNKER_RECON_MAX_DISTANCE", "20")),
        help="自主探路最远直线距离（米），超距沿轨迹返回起点 (env: BUNKER_RECON_MAX_DISTANCE, 默认 20)",
    )
    parser.add_argument(
        "--arm-grasp-signal",
        default=_env_or("ARM_GRASP_SIGNAL"),
        help="机械臂「抓取完成」信号通道，逗号分隔（env: ARM_GRASP_SIGNAL），"
             "例: ws,can:can1:0x3A1 / ws,file:/tmp/grasp.sig / ws,gpio:18:high；"
             "留空或 none = 关闭机械臂等待（就位即返回，旧行为）",
    )
    parser.add_argument(
        "--arm-wait-timeout",
        type=float,
        default=float(_env_or("ARM_WAIT_TIMEOUT", "90")),
        help="等待机械臂抓取完成的最长秒数 (env: ARM_WAIT_TIMEOUT, 默认 90)",
    )
    parser.add_argument(
        "--auto-mission",
        default=_env_or("BUNKER_AUTO_MISSION"),
        help="启动后自动开任务（find_object），不等云端键盘 (env: BUNKER_AUTO_MISSION)",
    )
    parser.add_argument(
        "--mission-target",
        default=_env_or("BUNKER_MISSION_TARGET", "target"),
        help="自动任务目标名 (env: BUNKER_MISSION_TARGET, 默认 target)",
    )
    parser.add_argument(
        "--mission-approach",
        dest="mission_approach",
        action="store_true",
        default=_env_or("BUNKER_MISSION_APPROACH", "1") not in ("0", "false", "False", "no"),
        help="自动任务进入对接（默认开；BUNKER_MISSION_APPROACH=0 则到停靠点就返回）",
    )
    parser.add_argument(
        "--no-mission-approach",
        dest="mission_approach",
        action="store_false",
        help="自动任务跳过对接",
    )
    parser.add_argument(
        "--mission-lock",
        dest="mission_lock",
        action="store_true",
        default=_env_or("BUNKER_MISSION_LOCK", "0") not in ("0", "false", "False", "no", ""),
        help="实战锁：任务进行中忽略键盘/move 等遥控 (env: BUNKER_MISSION_LOCK=1)",
    )
    parser.add_argument(
        "--wait-lidar",
        type=float,
        default=float(_env_or("BUNKER_MISSION_WAIT_LIDAR", "20")),
        help="自动任务等待雷达在线的秒数 (env: BUNKER_MISSION_WAIT_LIDAR, 默认 20)",
    )
    parser.add_argument(
        "--local",
        action="store_true",
        default=_env_or("BUNKER_LOCAL_TCP", "") not in ("", "0", "false", "False", "no"),
        help="本地 TCP 模式：不连云端 WebSocket，指令走 teleop TCP :9100 "
             "(env: BUNKER_LOCAL_TCP=1；配合仓库根目录 run_local.py)",
    )
    parser.add_argument(
        "--autonav-yaw-match",
        dest="autonav_yaw_match",
        action="store_true",
        default=_env_or("BUNKER_AUTONAV_YAW_MATCH", "1") not in ("0", "false", "False", "no", "off"),
        help="出发系/现场图前往时用雷达扫描匹配只修航向（默认开；"
             "BUNKER_AUTONAV_YAW_MATCH=0 或 --no-autonav-yaw-match 关）",
    )
    parser.add_argument(
        "--no-autonav-yaw-match",
        dest="autonav_yaw_match",
        action="store_false",
        help="出发系前往关闭航向扫描匹配，纯轮式原点",
    )
    args = parser.parse_args()

    if args.list_devices:
        print(format_device_list())
        return 0

    handlers: list[logging.Handler] = []
    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
    handlers.append(console)
    if args.log_file:
        file_handler = logging.FileHandler(args.log_file, encoding="utf-8")
        file_handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
        )
        handlers.append(file_handler)
        print(f"日志将同时写入: {os.path.abspath(args.log_file)}")
    logging.basicConfig(level=getattr(logging, args.log_level), handlers=handlers)

    if args.local:
        args.ws_url = args.ws_url or "local://tcp"
    elif not args.ws_url:
        print(f"错误: 必须提供 --ws-url 或设置环境变量 BUNKER_WS_URL", file=sys.stderr)
        print("本地无云端请用: python3 run_agent.py --local", file=sys.stderr)
        print("", file=sys.stderr)
        print("用法示例:", file=sys.stderr)
        print("  # SocketCAN (Jetson Linux, 自动检测通道):", file=sys.stderr)
        print("  python3 run_agent.py --ws-url ws://127.0.0.1:9000 --interface socketcan", file=sys.stderr)
        print("  # 手动指定 CAN 通道:", file=sys.stderr)
        print("  python3 run_agent.py --ws-url wss://... --interface socketcan --channel can1", file=sys.stderr)
        return 2

    try:
        agent = BunkerMiniAgent(
            ws_url=args.ws_url,
            device_id=args.device_id,
            bind_code=args.bind_code,
            channel=args.channel,
            interface=args.interface,
            track_dir=args.track_dir,
            enable_lidar=args.enable_lidar,
            lidar_port=args.lidar_port,
            lidar_pcap=args.lidar_pcap,
            lidar_source=args.lidar_source,
            lidar_mount_yaw_deg=args.lidar_mount_yaw,
            lidar_pitch_deg=args.lidar_pitch,
            lidar_height_m=args.lidar_height,
            lidar_self_mask=args.lidar_self_mask,
            wheelbase_m=args.wheelbase,
            step_limit_m=args.step_limit,
            recon_max_duration_s=args.recon_max_duration,
            recon_max_distance_m=args.recon_max_distance,
            arm_grasp_signal=args.arm_grasp_signal,
            arm_wait_timeout_s=args.arm_wait_timeout,
            auto_mission=args.auto_mission,
            auto_mission_target=args.mission_target,
            auto_mission_approach=args.mission_approach,
            auto_mission_wait_lidar_s=args.wait_lidar,
            mission_lock=args.mission_lock,
            local_mode=args.local,
            autonav_yaw_match=args.autonav_yaw_match,
        )
    except CanConfigError as exc:
        print(f"\nCAN 配置错误:\n{exc}", file=sys.stderr)
        print(file=sys.stderr)
        print_can_config_help()
        return 2

    # Graceful shutdown on SIGTERM / SIGINT
    def _shutdown(signum, frame):
        print("\nShutting down agent...")
        agent.stop()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    if args.local:
        print(f"Agent starting (本地 TCP): device={args.device_id}  不连 WebSocket")
    else:
        print(f"Agent starting: device={args.device_id} ws={args.ws_url}")
    print(
        f"CAN: interface={args.interface} "
        f"channel={args.channel or '(自动检测)'}"
    )
    agent.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
