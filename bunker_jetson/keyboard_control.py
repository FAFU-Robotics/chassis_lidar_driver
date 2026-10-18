#!/usr/bin/env python3
"""Real-time keyboard control for BUNKER MINI 2.0 (no Enter required).

On Linux the keys are read directly from the terminal (termios raw mode),
so it works over SSH on the Jetson without a local keyboard / X session.
On Windows it uses the ``keyboard`` library.

Requirements
    pip install keyboard          (仅 Windows / 有本地输入设备的机器需要)

Usage
    cd /home/fafu_robot/Desktop/chassis_lidar_drivers/bunker_jetson

    # Auto-detect CAN (SocketCAN on Jetson):
    python3 keyboard_control.py

    # Manual:
    python3 keyboard_control.py --interface socketcan --channel can1

    # Drive with keyboard AND record a track at the same time
    # (no remote controller needed; press R to stop & save, auto-saves on quit):
    python3 keyboard_control.py --record route1

Controls (hold to move, release to stop)
    W          Forward
    S          Backward
    A          Turn left (CCW)
    D          Turn right (CW)
    X / SPACE  Emergency STOP
    R          Stop & save recording (when --record is active)
    + / =      Increase speed step
    - / _      Decrease speed step
    Q / ESC    Quit

Speed limits (conservative)
    Linear  velocity: 0.00 ~ 0.20 m/s
    Angular velocity: 0.00 ~ 0.50 rad/s
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import warnings

# ── suppress python-can import noise ────────────────────────────────────
logging.basicConfig(level=logging.WARNING, format="%(message)s")
logging.getLogger("can").setLevel(logging.ERROR)
warnings.filterwarnings("ignore")

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini import BunkerMiniController, ControlMode, VehicleState
from bunker_mini.can_util import (
    CanConfigError,
    add_can_cli_args,
    format_device_list,
    quiet_can_detection,
)
from bunker_mini.tracker import TrackPlayer, TrackRecorder

# 键盘输入通道：Linux/SSH 下用 termios 原生终端读取（无需 /dev/input 或
# X 环境）；Windows / 有本地输入设备时用 keyboard 库。
try:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from _term_keys import TermKeyReader
    _term_reader: TermKeyReader | None = TermKeyReader()
except Exception:  # pragma: no cover
    _term_reader = None

try:
    import keyboard
except ImportError:
    keyboard = None  # type: ignore[assignment]


def _pressed(key: str) -> bool:
    """跨平台按键查询：优先 termios（Linux/SSH），回退 keyboard 库。"""
    if _term_reader is not None and _term_reader._is_tty:
        return _term_reader.pressed(key)
    if keyboard is not None:
        try:
            return keyboard.is_pressed(key)
        except Exception:
            return False
    return False

# ── constants ───────────────────────────────────────────────────────────
MAX_LINEAR_M_S = 0.20
MAX_ANGULAR_RAD_S = 0.50
LINEAR_STEP = 0.05
ANGULAR_STEP = 0.15


# ── main ────────────────────────────────────────────────────────────────
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Real-time keyboard control for BUNKER MINI 2.0 (no Enter needed)"
    )
    add_can_cli_args(parser)
    parser.add_argument(
        "--record", "-r", default="",
        help="同时录制轨迹（名称）；按 R 停止并保存，退出时自动保存",
    )
    parser.add_argument(
        "--track-dir", default="./tracks",
        help="轨迹保存目录 (默认: ./tracks)",
    )
    args = parser.parse_args()

    if args.list_devices:
        with quiet_can_detection():
            print(format_device_list())
        return 0

    # ── init controller ─────────────────────────────────────────────
    try:
        controller = BunkerMiniController(channel=args.channel, interface=args.interface)
    except CanConfigError as exc:
        print(f"CAN 配置错误:\n{exc}", file=sys.stderr)
        return 2

    linear_vel = 0.0
    angular_vel = 0.0
    linear_step = LINEAR_STEP
    angular_step = ANGULAR_STEP

    print("等待底盘状态 (最多 3 秒)...")
    try:
        controller.start()
    except Exception as exc:
        print(f"控制器启动失败: {exc}", file=sys.stderr)
        return 1

    # Wait for first status frame
    deadline = time.monotonic() + 3.0
    status = None
    while time.monotonic() < deadline:
        status = controller.latest_status
        if status is not None:
            break
        time.sleep(0.05)

    if status is None:
        print("错误: 未收到底盘状态——检查 CAN 接线、波特率 (500K) 及 WinUSB 驱动。")
        controller.stop()
        return 1

    if status.vehicle_state != VehicleState.NORMAL:
        print(f"底盘状态异常: {status.vehicle_state.name}")
        controller.stop()
        return 1

    if status.control_mode == ControlMode.REMOTE_CONTROL:
        print("遥控器模式——请先切到 CAN 指令模式。")
        controller.stop()
        return 1

    print("切换 CAN 指令模式...")
    controller.enable_can_control()
    time.sleep(0.2)

    # One-time info
    bms = controller.latest_bms
    soc_s = f"SOC={bms.soc_percent}%" if bms else ""
    temp_s = f"  {bms.temperature_c:.0f}°C" if bms else ""
    print(
        f"\n"
        f"底盘就绪 | 电池 {status.battery_voltage_v:.1f}V  {soc_s}{temp_s}  |  故障 0x{status.fault_code:02X}\n"
    )

    print(
        "BUNKER MINI 2.0 实时键盘控制 (按住走、松开停)\n"
        "==============================================\n"
        "  W          前进          + / =      增大步长\n"
        "  S          后退          - / _      减小步长\n"
        "  A          左转 (CCW)    X / SPACE  急停\n"
        "  D          右转 (CW)     Q / ESC    退出\n"
        "\n"
        f"当前步长: 线速度 {linear_step:.2f} m/s, 角速度 {angular_step:.2f} rad/s  "
        f"上限: {MAX_LINEAR_M_S:.2f} / {MAX_ANGULAR_RAD_S:.2f}\n"
    )

    # ── optional track recording (keyboard-driving + recording in one process) ──
    recorder = None
    if args.record:
        try:
            recorder = TrackRecorder(
                controller, track_dir=args.track_dir, drive_mode="kb",
            )
            recorder.start(args.record)
            print(f"轨迹录制已开始: {args.record}  (按 R 停止并保存，退出自动保存)")
        except Exception as exc:
            print(f"录制启动失败: {exc}", file=sys.stderr)
            recorder = None

    # ── main loop (33 Hz) ───────────────────────────────────────────
    if _term_reader is not None and _term_reader._is_tty:
        _term_reader.start()
        _term_reader.clear_press_states()  # 丢弃启动前终端缓冲的残留按键
        print("终端键盘输入已启用（SSH 下可直接 W/A/S/D 驾驶）")
    try:
        while True:
            if _pressed("q") or _pressed("esc"):
                _clear_status()
                print("退出")
                break

            # Emergency stop
            if _pressed("x") or _pressed("space"):
                linear_vel = 0.0
                angular_vel = 0.0
                controller.stop_motion()
                _clear_status()
                print("→ 急停")
                time.sleep(0.1)
                continue

            # Recording: R toggles stop&save / start
            if recorder is not None and _pressed("r"):
                _clear_status()
                if recorder.is_recording:
                    try:
                        track = recorder.stop()
                        player = TrackPlayer(controller, track_dir=args.track_dir)
                        path = player.save_track(track)
                        print(
                            f"轨迹已保存: {path}  "
                            f"({len(track.waypoints)} 航点, {track.total_duration_s:.1f}s)"
                        )
                        print("再按 R 可开始新一段录制")
                    except RuntimeError as e:
                        print(f"录制数据不足，未保存: {e}")
                else:
                    new_name = f"{args.record}_{time.strftime('%H%M%S')}"
                    recorder.start(new_name)
                    print(f"开始新录制: {new_name}")
                time.sleep(0.2)
                continue

            # Speed step
            if _pressed("+") or _pressed("="):
                linear_step = min(0.10, linear_step + 0.01)
                angular_step = min(0.30, angular_step + 0.03)
                _clear_status()
                print(f"步长 ↑ 线速度={linear_step:.2f}  角速度={angular_step:.2f}")
                time.sleep(0.15)
                continue

            if _pressed("-") or _pressed("_"):
                linear_step = max(0.01, linear_step - 0.01)
                angular_step = max(0.05, angular_step - 0.03)
                _clear_status()
                print(f"步长 ↓ 线速度={linear_step:.2f}  角速度={angular_step:.2f}")
                time.sleep(0.15)
                continue

            # Movement — 和遥控器一样走双轴，不按「每个键是否还在连发」拼速度。
            target_linear = 0.0
            target_angular = 0.0
            if _term_reader is not None and hasattr(_term_reader, "drive_stick"):
                vs, ws = _term_reader.drive_stick()
                target_linear = vs * linear_step
                target_angular = ws * angular_step
            else:
                if _pressed("w"):
                    target_linear += linear_step
                if _pressed("s"):
                    target_linear -= linear_step
                if _pressed("a"):
                    target_angular += angular_step
                if _pressed("d"):
                    target_angular -= angular_step

            target_linear = max(-MAX_LINEAR_M_S, min(MAX_LINEAR_M_S, target_linear))
            target_angular = max(-MAX_ANGULAR_RAD_S, min(MAX_ANGULAR_RAD_S, target_angular))

            if abs(target_linear - linear_vel) > 0.001 or abs(target_angular - angular_vel) > 0.001:
                linear_vel = target_linear
                angular_vel = target_angular
                controller.set_velocity(linear_vel, angular_vel)

            # Refresh battery data every ~1s; always update velocity
            _show_status(controller, linear_vel, angular_vel, linear_step, angular_step, recorder)
            time.sleep(0.02)  # 50Hz 轮询，松开后 ~20ms 内响应

    except KeyboardInterrupt:
        _clear_status()
        print("用户中断 (Ctrl+C)")
    finally:
        # Auto-save an in-progress recording so no track data is lost on exit
        if recorder is not None and recorder.is_recording:
            try:
                track = recorder.stop()
                player = TrackPlayer(controller, track_dir=args.track_dir)
                path = player.save_track(track)
                print(f"录制已自动保存: {path}")
            except RuntimeError as e:
                print(f"录制数据不足，未保存: {e}")
        if controller is not None:
            print("停止小车并关闭 CAN 总线...")
            controller.stop_motion()
            time.sleep(0.1)
            controller.stop()
            print("完成.")
        else:
            print("控制器未创建，无需清理。")
        if _term_reader is not None:
            _term_reader.close()

    return 0


def _clear_status() -> None:
    """Erase the current status line."""
    sys.stdout.write("\r\033[K")
    sys.stdout.flush()


def _show_status(
    controller: BunkerMiniController,
    v: float,
    w: float,
    step_v: float,
    step_w: float,
    recorder: TrackRecorder | None = None,
) -> None:
    """Overwrite the current terminal line with velocity + chassis info."""
    bms = controller.latest_bms
    st = controller.latest_status

    soc_s = f"{bms.soc_percent}%" if bms else "?"
    temp_s = f"{bms.temperature_c:.0f}°C" if bms else "?"
    bat_s = f"{st.battery_voltage_v:.1f}V" if st else "?"
    fault_s = f"0x{st.fault_code:02X}" if st else "?"

    rec_s = ""
    if recorder is not None:
        rec_s = f"  |  录制中 {recorder.waypoint_count} 点" if recorder.is_recording else ""

    sys.stdout.write(
        f"\r  速度  v={v:+.2f} m/s  w={w:+.2f} rad/s"
        f"  |  电池 {bat_s}  SOC={soc_s}  {temp_s}"
        f"  |  故障 {fault_s}"
        f"  |  步长 {step_v:.2f}/{step_w:.2f}"
        f"{rec_s}"
        f"  \033[K"
    )
    sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main())
