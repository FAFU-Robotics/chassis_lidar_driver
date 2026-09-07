#!/usr/bin/env python3
"""本地任务启动器（推荐入口）：指令走 TCP，不连已废弃的云端 WebSocket。

放在仓库根目录 ``chassis_lidar_drivers/``。

    python3 run_local.py
    python3 run_local.py --daemon --no-lidar   # 开机服务用：无终端、不开雷达窗

原理
    拉起车侧代理 ``run_agent.py --local``（CAN + 任务编排，
    **不连已废弃的云端、不开 WebSocket**）。交互窗口可敲 ``fo`` / ``goto``
    / ``r``；无头 ``--daemon`` 只听 ``:9100`` + 网页 ``:9101``，供笔记本客户端连接。

    老师要求的 100Hz / ≤20ms 遥控 **不是** 本窗口里敲 ``kb``。
    Cursor / SSH 终端只有字符流、没有 key-up，输入只有 10～30Hz。
    正确做法：本进程只听 :9100；笔记本浏览器打开 ``:9101``（WASD + 任务按钮）
    或本机跑 ``teleop_from_laptop.py``。

    雷达窗口启动本机脚本（``view_lidar.py`` / ``live_airview.py``）。

Ctrl+C / 停止服务：立即急停并退出。物理急停始终有效。
不要和已废弃的 ``run_mission.py`` / ``mock_cloud`` 或 ``bunker-teleop.service`` 抢 9100。
开机自启见仓库根目录 ``install_local_service.sh``（``bunker-local.service``）。
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
JETSON = ROOT / "bunker_jetson"
TRACKS = JETSON / "tracks"
AGENT_CONF = JETSON / "agent.conf"
MISSION_CONF = ROOT / "mission.conf"

if str(JETSON) not in sys.path:
    sys.path.insert(0, str(JETSON))

from bunker_mini.teleop_tcp import (  # noqa: E402
    DEFAULT_PORT,
    DEFAULT_TOKEN,
    TeleopTcpClient,
)

try:
    from teleop_web import DEFAULT_WEB_PORT, TeleopWebServer
except Exception:  # pragma: no cover
    DEFAULT_WEB_PORT = 9101  # type: ignore[misc,assignment]
    TeleopWebServer = None  # type: ignore[misc,assignment]

try:
    from _term_keys import TermKeyReader
except Exception:  # pragma: no cover
    TermKeyReader = None  # type: ignore[misc,assignment]

CMD_ALIASES = {
    "m": "move",
    "e": "estop",
    "s": "estop",
    "g": "goto",
    "fo": "find_object",
    "fobj": "find_object",
    "go": "go_home",
    "home": "go_home",
    "c": "cancel",
    "abort": "cancel",
    "z": "odom_reset",
    "zero": "odom_reset",
    "gd": "grasp_done",
    "arm_done": "grasp_done",
    "mu": "map_upload",
    "pa": "pose_align",
    "wb": "set_wheelbase",
    "wbcal": "calibrate_wheelbase",
    "mr": "map_return",
    "q": "query",
    "can": "can_check",
    "canstat": "can_check",
    "r": "track_record",
    "f": "track_follow",
    "fb": "track_follow_back",
    "d": "track_delete",
    "t": "tracks",
    "tk": "task_submit",
    "task": "task_submit",
    "p": "ping",
    "lo": "lidar_on",
    "loff": "lidar_off",
    "lf": "lidar_off",
    "ls": "lidar_status",
    "map": "lidar_map",
    "lm": "lidar_map",
    "view": "point_cloud",
    "pc": "point_cloud",
    "vl": "view_live",
    "vm": "view_map",
    "vb": "view_both",
    "vo": "view_off",
    "sm": "smart",
    "qt": "quiet",
    "h": "help",
    "?": "help",
    "x": "quit",
}

KB_LINEAR = 0.10
KB_ANGULAR = 0.40
# 本终端 stick：100 Hz 采样并保活（对齐老师 10ms/帧；看门狗 0.05s）
KB_SAMPLE_S = 0.01
KB_KEEPALIVE_S = 0.01
_KB_SHOW_EVENTS = frozenset({
    "arrived", "fault", "estop", "track_record", "track_follow", "auto_stop",
})

_MISSION_CN = {
    "recon": "探路",
    "found": "已发现",
    "navigating": "导航中",
    "approaching": "对接中",
    "ready": "可抓取",
    "holding": "等待抓取",
    "grasped": "已抓取",
    "returning": "返回中",
    "done": "完成",
    "failed": "失败",
    "cancelled": "已取消",
    "interrupted": "断电中断",
}


def _load_conf(path: Path, environ: dict[str, str]) -> None:
    if not path.is_file():
        return
    with path.open("r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            environ.setdefault(key.strip(), value.strip())


def _port_open(host: str, port: int) -> bool:
    sock = socket.socket()
    sock.settimeout(0.25)
    try:
        sock.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        try:
            sock.close()
        except OSError:
            pass


def _wait_port(
    host: str,
    port: int,
    timeout_s: float,
    proc: subprocess.Popen | None = None,
) -> bool:
    """等端口就绪。代理进程若已死立刻失败；Ctrl+C 向上抛，由调用方收尾。"""
    deadline = time.time() + timeout_s
    last_note = 0.0
    t0 = time.time()
    while time.time() < deadline:
        if proc is not None and proc.poll() is not None:
            print(
                f"代理进程已退出 (code={proc.returncode})，: {port} 不会再起来",
                file=sys.stderr,
            )
            return False
        if _port_open(host, port):
            return True
        now = time.time()
        if now - last_note >= 2.0:
            last_note = now
            print(
                f"  等待 {host}:{port} … {now - t0:.0f}s "
                f"（CAN 探测最多约十几秒，不要急着 Ctrl+C）"
            )
        time.sleep(0.15)
    return False


def _now_ms() -> int:
    return int(time.time() * 1000)


def _lan_ips() -> list[str]:
    """本机非回环 IPv4。雷达网 / Docker 网桥放到最后，避免笔记本客户端抄错。"""
    ips: list[str] = []
    try:
        out = os.popen("hostname -I 2>/dev/null").read().split()
        for tok in out:
            if tok.count(".") == 3 and not tok.startswith("127."):
                ips.append(tok)
    except Exception:
        pass

    def _skip_for_laptop(ip: str) -> bool:
        return ip.startswith("192.168.1.") or ip.startswith("172.17.")

    client = [ip for ip in ips if not _skip_for_laptop(ip)]
    other = [ip for ip in ips if _skip_for_laptop(ip)]
    return client + other


def _fmt_battery(p: dict) -> str:
    """状态栏电量：SOC 百分比 + 0x211 电池包电压，避免把 0.89 误读成电压。"""
    soc = p.get("battery")
    volt = p.get("batteryVoltageV")
    if volt is None:
        chassis = p.get("chassis") or {}
        sys = chassis.get("system") if isinstance(chassis, dict) else None
        if isinstance(sys, dict):
            volt = sys.get("batteryVoltageV")
    parts: list[str] = []
    if soc is not None:
        try:
            s = float(soc)
            if 0.0 <= s <= 1.0:
                parts.append(f"{s * 100.0:.0f}%")
            elif s > 5.0:
                parts.append(f"{s:.1f}V")
            else:
                parts.append(f"{s:.0f}%")
        except (TypeError, ValueError):
            parts.append(str(soc))
    if volt is not None:
        try:
            v = float(volt)
            if v > 5.0:
                txt = f"{v:.1f}V"
                if txt not in parts:
                    parts.append(txt)
                if v < 24.0:
                    parts.append("低电")
        except (TypeError, ValueError):
            pass
    return " ".join(parts) if parts else "?"


def _print_jetson_power_hint() -> None:
    """Orin Nano 走 5V + MAXN_SUPER 时，电机一起步就容易把工控机拉断电重启。"""
    vdd_mv = None
    iin_ma = None
    try:
        vdd_mv = int(open("/sys/class/hwmon/hwmon1/in1_input", encoding="ascii").read())
        iin_ma = int(open("/sys/class/hwmon/hwmon1/curr1_input", encoding="ascii").read())
    except (OSError, ValueError):
        pass
    mode = ""
    try:
        out = subprocess.check_output(["nvpmodel", "-q"], text=True, timeout=2)
        for line in out.splitlines():
            if "Power Mode" in line:
                mode = line.split(":", 1)[-1].strip()
                break
    except Exception:
        mode = ""
    if vdd_mv is None and not mode:
        return
    bits = []
    if mode:
        bits.append(f"功耗档={mode}")
    if vdd_mv is not None:
        bits.append(f"VDD_IN={vdd_mv/1000.0:.2f}V")
    if iin_ma is not None:
        bits.append(f"Iin={iin_ma}mA")
    print("  Jetson: " + " ".join(bits))
    if vdd_mv is not None and vdd_mv < 5300:
        print("  ⚠ 工控机输入约 5V，裕量很小。网页遥控起步电流一大，")
        print("    5V 跌落到约 4.75V 就会整机重启（不是 run_local 写了 reboot）。")
        print("    建议：Jetson 独立供电/加粗 5V 线；功耗档改 25W：sudo nvpmodel -m 1")
        print("    网页默认 0.10 m/s，先别把 + 加到 0.50。")


def _print_can_health() -> None:
    """启动时点名 CAN：缺 USB-CAN / BUS-OFF 时 kb 看起来像按了没动。"""
    try:
        from bunker_mini.can_util import (
            format_socketcan_status,
            list_socketcan_interfaces,
            restore_tx_mode,
            socketcan_ctrl_state,
            socketcan_is_listen_only,
        )
    except Exception:
        return
    try:
        restore_tx_mode()
    except Exception:
        pass
    ifaces = list_socketcan_interfaces()
    status = format_socketcan_status()
    print("  CAN 网卡:")
    print(status or "    (没有 can* 网卡)")
    usb = "USB-CAN" in (status or "")
    states = {n: socketcan_ctrl_state(n) for n in ifaces}
    busoff = any(s == "bus-off" for s in states.values())
    passive = any(s == "error-passive" for s in states.values())
    listening = [n for n in ifaces if socketcan_is_listen_only(n)]
    if not usb:
        print("  ⚠ 未发现 USB-CAN（candleLight）。板载 mttcan 通常连不上 Bunker。")
        print("    插入适配器后执行: bash bunker_jetson/bringup_gs_usb_can.sh")
    if busoff:
        print("  ⚠ 有通道 BUS-OFF：已尝试软复位。")
    if passive:
        print("  ⚠ 有通道 ERROR-PASSIVE：启动后会软复位并恢复发送。")
    if listening:
        print(f"  ⚠ {','.join(listening)} 仍是 LISTEN-ONLY，0x111 发不出去。")


def _headless_env(environ: dict[str, str]) -> dict[str, str]:
    """无头服务：关地图窗、默认关雷达驱动，清掉 DISPLAY，避免挤占桌面。"""
    out = dict(environ)
    out["BUNKER_MAP_VIEW"] = "0"
    out["BUNKER_ENABLE_LIDAR"] = "0"
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY"):
        out.pop(key, None)
    return out


def _wait_daemon_stop(agent: subprocess.Popen | None) -> int:
    """阻塞直到 SIGTERM/SIGINT，或子代理退出（交给 systemd 重启）。"""
    stop = threading.Event()

    def _on_signal(signum: int, _frame) -> None:
        name = "SIGTERM" if signum == signal.SIGTERM else "SIGINT"
        print(f"无头模式收到 {name}，准备急停退出")
        stop.set()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    print("无头模式已就绪（:9100 代理 + :9101 网页）。桌面不弹窗；雷达请在网页里再开。")
    while not stop.wait(1.0):
        if agent is not None and agent.poll() is not None:
            print(
                f"代理进程已退出 code={agent.returncode}，无头启动器退出",
                file=sys.stderr,
            )
            return 1
    return 0


def _print_help() -> None:
    print(
        "可用指令（括号内为简写）。本启动器走 TCP，是推荐入口；不要再开云端模式。\n"
        "  move / m <v> <w> [t]  运动，例: m 0.2 0.1 或 m 0.2 0.1 10\n"
        "  kb / keyboard         本终端字符驾驶（SSH/Cursor 达不到 100Hz，仅调试）\n"
        "                       真遥控/任务：笔记本浏览器打开 http://<工控机IP>:9101\n"
        "                       双击 start_teleop_client.sh/.bat 也可打开该页面\n"
        "                       或本机 python teleop_from_laptop.py --host <工控机IP>\n"
        "  estop / e / s         紧急停止\n"
        "  goto / g <x> <y> [speed] [yawDeg]  目标点自主导航（到位后转正车身）\n"
        "                       短测例: z 然后 g 0.6 0 0.12\n"
        "  find_object / fo / fobj  探路→识别→靠近→返回；fo 目标A n 跳过对接\n"
        "  go_home / go / home    返回导航原点 (0,0)\n"
        "  cancel / c / abort     中止当前任务链并返回起点\n"
        "  odom_reset / z / zero  导航原点重置为当前位置\n"
        "  grasp_done / gd        机械臂抓取完成 → 底盘返回\n"
        "  map_return / mr on|off 地图优先返回\n"
        "  pose_align / pa x y yaw  地图系↔里程系标定\n"
        "  set_wheelbase / wb <m>   设置轮距（写入 wheelbase.local）\n"
        "  calibrate_wheelbase / wbcal <实测yaw°>\n"
        "                       重置原点并原地转到已知角后，用地面航向标定轮距\n"
        "  map_upload / mu <file> 导入全局栅格地图（本机读文件，不下发大包）\n"
        "  lidar on / lo          开启雷达\n"
        "  lidar off / loff / lf  关闭雷达\n"
        "  lidar status / ls      雷达/避障自检\n"
        "  map / lm / vm          打开本机雷达地图/扇区窗口（不走 WebSocket）\n"
        "  view / pc / vl         打开本机点云/扇区窗口\n"
        "  vb                     点云窗口 + 3D（若有 live_airview.py）\n"
        "  vo                     关闭雷达窗口\n"
        "  can / canstat          本机检测 USB-CAN / 网卡 / 是否听到底盘 0x211\n"
        "  query / q              请求立即上报一次状态\n"
        "  track_record / r <名>  键盘驾驶录制；Q 保存，B 保存后原路返回\n"
        "  track_follow / f <名>  回放；B 原路返回，Q 退出，SPACE 急停\n"
        "  track_follow_back / fb 往返：到终点后自动返回起点\n"
        "  track_delete / d <名>  删除已保存轨迹\n"
        "  task / tk <名> [ID]    点到点运输（沿已录轨迹回放）\n"
        "  tracks / t             列出车侧 tracks/\n"
        "  ping / p               TCP 心跳（看 RTT）\n"
        "  verbose / smart / quiet  状态打印模式\n"
        "  help / h / ?           本帮助\n"
        "  quit / exit / x        停车并退出\n"
    )


def _print_tracks() -> None:
    try:
        names = sorted(f for f in os.listdir(TRACKS) if f.endswith(".json"))
    except FileNotFoundError:
        names = []
    if not names:
        print("本地 tracks/ 没有轨迹。先录制: r <名称>")
        return
    print(f"车侧 tracks/ 已保存 {len(names)} 条轨迹（回放: f <名称>）:")
    for n in names:
        path = TRACKS / n
        try:
            with path.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
            dur = float(data.get("total_duration_s", 0.0))
            wp = len(data.get("waypoints", []))
            print(f"  {n[:-5]}  ({dur:.1f} s, {wp} 航点)")
        except Exception:
            print(f"  {n[:-5]}  (解析失败)")


class LocalConsole:
    def __init__(self, client: TeleopTcpClient, env: dict[str, str]) -> None:
        self.client = client
        self.env = env
        self._state_mode = "smart"
        self._last_state_key: tuple | None = None
        self._last_state: dict | None = None
        self._last_event: dict | None = None
        self._event_lock = threading.Event()
        self._state_seq = 0
        self._event_seq = 0
        self._viewers: list[subprocess.Popen] = []
        self._stop = False
        self._keys: TermKeyReader | None = None
        self._kb_active = False
        self._kb_mode = "drive"
        self._kb_record = ""
        self._kb_play = ""
        self._kb_v = KB_LINEAR
        self._kb_w = KB_ANGULAR
        self._kb_last_send = 0.0
        self._kb_last_vw = (0.0, 0.0)
        self._kb_axis = (0, 0)
        self._kb_plus_prev = False
        self._kb_minus_prev = False
        self._kb_b_prev = False
        self._kb_rewinding = False
        self._kb_status_t = 0.0
        self._kb_space_sent = False
        self._kb_zero_sent = True
        self._kb_drive_stop = threading.Event()
        self._kb_drive_th: threading.Thread | None = None
        self._pump = threading.Thread(target=self._pump_events, name="tcp-evt", daemon=True)
        self._pump.start()

    def _cmd(self, payload: dict) -> None:
        body = dict(payload)
        body.setdefault("ts", _now_ms())
        try:
            self.client.cmd(body)
        except (TimeoutError, OSError) as exc:
            print(f"[TCP] 指令未应答: {exc}")

    def _estop(self) -> None:
        try:
            self.client.estop()
        except (TimeoutError, OSError):
            try:
                self.client.stick(0.0, 0.0)
            except (TimeoutError, OSError):
                pass

    def _pump_events(self) -> None:
        while not self._stop:
            try:
                events = self.client.poll_events()
            except OSError:
                time.sleep(0.2)
                continue
            for msg in events:
                self._on_msg(msg)
            time.sleep(0.05)

    def _on_msg(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "state":
            payload = msg.get("payload") or {}
            if isinstance(payload, dict):
                self._last_state = payload
                self._state_seq += 1
                self._show_state(payload, force=False)
            return
        if kind != "event":
            return
        payload = msg.get("payload") or {}
        if not isinstance(payload, dict):
            return
        self._last_event = payload
        self._event_seq += 1
        self._event_lock.set()
        ev = str(payload.get("event") or "")
        text = str(payload.get("msg") or "")
        if self._kb_active and ev not in _KB_SHOW_EVENTS:
            return
        if ev == "chassis":
            return
        if ev:
            print(f"[事件] {ev} — {text}")

    def _await_state(self, start_seq: int, timeout_s: float = 1.5) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._state_seq > start_seq and self._last_state is not None:
                return True
            time.sleep(0.04)
        return False

    def _await_named_event(
        self, names: set[str], start_seq: int, timeout_s: float = 1.8,
    ) -> dict | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ev = self._last_event
            if (
                self._event_seq > start_seq
                and ev
                and str(ev.get("event") or "") in names
            ):
                return ev
            time.sleep(0.04)
        return None

    def _refresh_state(self, timeout_s: float = 1.5) -> dict | None:
        """下发 query 并等到下一帧状态（query 的 ACK 早于状态推送）。"""
        seq = self._state_seq
        self._cmd({"action": "query"})
        if self._await_state(seq, timeout_s):
            return self._last_state
        return self._last_state

    def _nav_preflight(self, action: str) -> None:
        """goto / fo 前尽量拿到一帧状态；雷达没出点就先警告。"""
        st = self._last_state
        if st is None:
            st = self._refresh_state(1.2)
        lidar = (st or {}).get("lidar") or {}
        if st is None:
            print(f"⚠ 尚未收到车侧状态。{action} 仍会下发，结果看事件。")
            return
        if not lidar.get("online"):
            print(
                f"⚠ 雷达未出点。车侧会拒绝 {action}。"
                "先等几秒或输入 ls，看到「雷达=在线」再试。"
            )

    def _show_state(self, p: dict, *, force: bool) -> None:
        if self._kb_active and not force:
            return
        if self._state_mode == "quiet" and not force:
            return
        pose = p.get("pose") or {}
        lidar = p.get("lidar") or {}
        mission = p.get("mission") or {}
        drive = p.get("drive") or {}
        navigating = bool(p.get("navigating") or drive.get("kind") == "goto")
        goto_info = drive.get("goto") or {}
        dist = goto_info.get("distM")
        front = lidar.get("frontObstacle")
        key = (
            round(float(p.get("speed") or 0), 2),
            str(p.get("mode") or ""),
            bool(lidar.get("online")),
            str(mission.get("status") or ""),
            bool(navigating),
            str(drive.get("kind") or ""),
            None if dist is None else round(float(dist), 2),
            round(float(pose.get("x") or 0), 2),
            round(float(pose.get("y") or 0), 2),
        )
        if self._state_mode == "smart" and not force and key == self._last_state_key:
            return
        self._last_state_key = key
        mcn = _MISSION_CN.get(str(mission.get("status") or ""), mission.get("status") or "-")
        loc = p.get("localization") or {}
        last_goto = p.get("lastGoto") or {}
        loc_src = str(loc.get("source") or "odom")
        wb = loc.get("wheelbaseM")
        if navigating:
            nav_txt = (
                f"进行中 剩{float(dist):.2f}m" if dist is not None else "进行中"
            )
            yaw_left = goto_info.get("errYawDeg")
            if yaw_left is not None:
                nav_txt += f" 航向差{float(yaw_left):.1f}°"
        else:
            nav_txt = "-"
            err_m = last_goto.get("errM")
            if last_goto.get("arrived") and err_m is not None:
                nav_txt = f"上次残差{float(err_m):.3f}m"
        if front is None:
            front_txt = "-"
        else:
            try:
                front_txt = f"{float(front):.2f}m"
            except (TypeError, ValueError):
                front_txt = "-"
        print(
            "状态  pose=({:.2f},{:.2f},yaw={:.1f}°)  v={:.2f}  mode={}  "
            "雷达={}  定位={}  轮距={}  导航={}  前障={}  任务={}  电量={}".format(
                float(pose.get("x") or 0),
                float(pose.get("y") or 0),
                float(pose.get("yaw") or 0),
                float(p.get("speed") or 0),
                p.get("mode") or "?",
                "在线" if lidar.get("online") else "离线",
                loc_src,
                f"{float(wb):.3f}m" if wb is not None else "-",
                nav_txt,
                front_txt,
                mcn,
                _fmt_battery(p),
            )
        )

    def _close_viewers(self) -> bool:
        closed = False
        for proc in self._viewers:
            if proc.poll() is None:
                proc.terminate()
                closed = True
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._viewers = [p for p in self._viewers if p.poll() is None]
        return closed

    def _viewer_running(self) -> bool:
        self._viewers = [p for p in self._viewers if p.poll() is None]
        return bool(self._viewers)

    def _probe_display(self) -> str | None:
        if os.environ.get("DISPLAY"):
            return os.environ["DISPLAY"]
        try:
            socks = sorted(
                n for n in os.listdir("/tmp/.X11-unix") if n.startswith("X")
            )
        except OSError:
            return None
        return f":{socks[0][1:]}" if socks else None

    def _launch_script(self, script: Path, extra: list[str] | None = None) -> bool:
        if not script.is_file():
            print(f"未找到任务脚本: {script}")
            return False
        env = dict(os.environ)
        if not env.get("DISPLAY"):
            display = self._probe_display()
            if display:
                env["DISPLAY"] = display
                xa = os.path.expanduser("~/.Xauthority")
                if os.path.exists(xa):
                    env["XAUTHORITY"] = xa
        cmd = [sys.executable, str(script)] + (extra or [])
        try:
            log_path = JETSON / ".local_viewer.log"
            logf = open(log_path, "a", encoding="utf-8")
            proc = subprocess.Popen(
                cmd,
                cwd=str(script.parent),
                env=env,
                stdout=logf,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._viewers.append(proc)
            print(f"已启动 {script.name}（看工控机桌面；vo 或 Ctrl+C 关窗口）")
            return True
        except Exception as exc:
            print(f"启动 {script.name} 失败: {exc}")
            return False

    def _open_lidar_view(self, mode: str) -> None:
        """雷达窗口走本机脚本，不经过 WebSocket 点云流。"""
        if mode in ("cloud", "live", "map", "both"):
            self._launch_script(JETSON / "view_lidar.py")
        if mode in ("both", "air"):
            air = JETSON / "live_airview.py"
            if not air.is_file():
                air = ROOT / "live_airview.py"
            if air.is_file():
                self._launch_script(air)

    def dispatch(self, line: str) -> bool:
        line = line.strip()
        if not line:
            return True
        parts = line.split()
        cmd = CMD_ALIASES.get(parts[0].lower(), parts[0].lower())

        if cmd in ("quit", "exit"):
            print("退出本地启动器")
            self._close_viewers()
            try:
                self.client.estop()
            except Exception:
                pass
            return False
        if cmd in ("help", "?"):
            _print_help()
        elif cmd in ("verbose", "verb", "v"):
            self._state_mode = "full"
            print("状态打印: 全量")
        elif cmd in ("smart", "auto"):
            self._state_mode = "smart"
            print("状态打印: 智能（只在显著变化时打印）")
        elif cmd in ("quiet", "mute"):
            self._state_mode = "quiet"
            print("状态打印: 静默")
        elif cmd in ("kb", "keyboard", "keys"):
            self._kb_start()
        elif cmd == "ping":
            rtt = self.client.ping()
            print(f"[TCP] ping RTT={rtt / 1000.0:.1f} ms")
        elif cmd == "move":
            if len(parts) < 3:
                print("用法: move <线速度> <角速度> [秒]，例: m 0.2 0.1 3")
                return True
            try:
                v, w = float(parts[1]), float(parts[2])
                duration = float(parts[3]) if len(parts) > 3 else 0.0
            except ValueError:
                print("参数需为数字")
                return True
            self._cmd({"action": "move", "v": v, "w": w, "duration": max(0.0, duration)})
            print(f"[TCP] move v={v:.2f} w={w:.2f}" + (f" {duration:.1f}s" if duration else ""))
        elif cmd in ("estop", "stop"):
            self._estop()
            print("!!!! [TCP] ESTOP !!!!")
        elif cmd == "goto":
            if len(parts) < 3:
                print("用法: goto <x> <y> [speed] [yawDeg]，例: g 0.6 0 0.12")
                return True
            try:
                x, y = float(parts[1]), float(parts[2])
            except ValueError:
                print("参数需为数字")
                return True
            speed = 0.0
            if len(parts) >= 4:
                try:
                    speed = float(parts[3])
                except ValueError:
                    print("speed 需为数字")
                    return True
            if speed < 0:
                print("speed 须 > 0，已忽略，改用车侧默认")
                speed = 0.0
            elif speed > 0.5:
                print(f"speed {speed:.2f} 超过安全上限 0.50，已钳到 0.50")
                speed = 0.5
            yaw_deg = None
            if len(parts) >= 5:
                try:
                    yaw_deg = float(parts[4])
                except ValueError:
                    print("yawDeg 需为数字（度）")
                    return True
            self._nav_preflight("goto")
            ev_seq = self._event_seq
            st_seq = self._state_seq
            payload = {"action": "goto", "x": x, "y": y}
            if speed > 0:
                payload["speed"] = speed
            if yaw_deg is not None:
                payload["yawDeg"] = yaw_deg
            self._cmd(payload)
            extra = ""
            if speed > 0:
                extra += f" ≤{speed:.2f} m/s"
            if yaw_deg is not None:
                extra += f" yaw={yaw_deg:.1f}°"
            print(f"[TCP] goto ({x:.2f}, {y:.2f})" + extra)
            ev = self._await_named_event(
                {"goto", "fault", "failsafe"}, ev_seq, 1.8,
            )
            if ev is None:
                print("车侧尚未确认 goto（再 q 看导航栏，或看 [事件]）")
            elif self._await_state(st_seq, 0.8) and self._last_state:
                self._show_state(self._last_state, force=True)
        elif cmd == "find_object":
            skip = ("n", "no", "skip", "noa")
            approach_tok = ("a", "approach")
            name = "target"
            if len(parts) > 1 and parts[1].lower() not in skip + approach_tok:
                name = parts[1]
            extras = [p.lower() for p in parts[1:]]
            do_approach = not any(p in skip for p in extras)
            self._nav_preflight("find_object")
            ev_seq = self._event_seq
            self._cmd({"action": "find_object", "name": name, "approach": do_approach})
            print(f"[TCP] find_object '{name}' approach={do_approach}")
            if self._await_named_event(
                {"find_object", "fault", "failsafe"}, ev_seq, 1.8,
            ) is None:
                print("车侧尚未确认 find_object（再 q 看任务栏）")
        elif cmd == "go_home":
            self._nav_preflight("go_home")
            ev_seq = self._event_seq
            st_seq = self._state_seq
            self._cmd({"action": "go_home"})
            print("[TCP] go_home → 导航原点 (0,0)")
            ev = self._await_named_event(
                {"goto", "fault", "failsafe"}, ev_seq, 1.8,
            )
            if ev is None:
                print("车侧尚未确认 go_home（再 q 看导航栏）")
            elif self._await_state(st_seq, 0.8) and self._last_state:
                self._show_state(self._last_state, force=True)
        elif cmd == "cancel":
            name = parts[1] if len(parts) > 1 else "target"
            self._cmd({"action": "cancel", "name": name})
            print("[TCP] cancel — 中止任务并返回起点")
        elif cmd == "odom_reset":
            ev_seq = self._event_seq
            self._cmd({"action": "odom_reset"})
            print("[TCP] odom_reset — 原点改到当前位置")
            self._await_named_event({"odom_reset", "fault"}, ev_seq, 1.2)
            st = self._refresh_state(1.2)
            if st is not None:
                self._show_state(st, force=True)
        elif cmd == "map_return":
            on = True
            if len(parts) > 1:
                on = parts[1].lower() in ("on", "1", "true")
            self._cmd({"action": "map_return", "mapReturn": on})
            print(f"[TCP] map_return {'on' if on else 'off'}")
        elif cmd == "grasp_done":
            self._cmd({"action": "grasp_done"})
            print("[TCP] grasp_done")
        elif cmd == "pose_align":
            if len(parts) < 4:
                print("用法: pose_align <x> <y> <yawDeg>")
                return True
            try:
                px, py, yaw = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                print("参数需为数字")
                return True
            self._cmd({"action": "pose_align", "x": px, "y": py, "yawDeg": yaw})
            print(f"[TCP] pose_align ({px:.2f},{py:.2f}) yaw={yaw:.1f}°")
        elif cmd == "set_wheelbase":
            if len(parts) < 2:
                loc = (self._last_state or {}).get("localization") or {}
                cur = loc.get("wheelbaseM")
                print("用法: set_wheelbase / wb <米>，例: wb 0.48")
                if cur is not None:
                    print(f"当前轮距 {float(cur):.3f} m")
                return True
            try:
                wb = float(parts[1])
            except ValueError:
                print("轮距需为数字（米）")
                return True
            self._cmd({"action": "set_wheelbase", "wheelbaseM": wb})
            print(f"[TCP] set_wheelbase {wb:.3f} m")
        elif cmd == "calibrate_wheelbase":
            if len(parts) < 2:
                print("用法: wbcal <地面实测航向°>，先 z 重置原点再原地转到已知角")
                return True
            try:
                yaw = float(parts[1])
            except ValueError:
                print("航向需为数字（度）")
                return True
            self._cmd({"action": "calibrate_wheelbase", "yawDeg": yaw})
            print(f"[TCP] calibrate_wheelbase actualYaw={yaw:.1f}°")
        elif cmd == "map_upload":
            if len(parts) < 2:
                print("用法: map_upload <file.json>")
                return True
            path = Path(parts[1]).expanduser()
            if not path.is_file():
                print(f"找不到地图文件: {path}")
                return True
            self._cmd({"action": "map_upload", "file": str(path.resolve())})
            print(f"[TCP] map_upload {path}")
        elif cmd == "lidar":
            if len(parts) < 2:
                print("用法: lidar <on|off|status|map|view>")
                return True
            sub = parts[1].lower()
            rest = " ".join(parts[2:])
            if sub == "on":
                return self.dispatch("lo")
            if sub == "off":
                return self.dispatch("loff")
            if sub in ("status", "test"):
                return self.dispatch("ls")
            if sub == "map":
                return self.dispatch(("map " + rest).strip())
            if sub in ("view", "pc"):
                return self.dispatch("view")
            print(f"未知 lidar 子命令: {sub}（可用 on/off/status/map/view）")
            return True
        elif cmd == "lidar_on":
            self._cmd({"action": "lidar_on"})
            print("[TCP] lidar on")
        elif cmd == "lidar_off":
            self._cmd({"action": "lidar_off"})
            print("[TCP] lidar off")
        elif cmd == "lidar_status":
            self._cmd({"action": "lidar_status"})
            print("[TCP] lidar status")
        elif cmd == "lidar_map":
            payload: dict = {"action": "lidar_map"}
            if len(parts) >= 3:
                try:
                    payload["x"] = float(parts[1])
                    payload["y"] = float(parts[2])
                except ValueError:
                    print("map <x> <y> 需为数字")
                    return True
            self._cmd(payload)
            self._open_lidar_view("map")
            print("[TCP] lidar_map + 本机雷达窗口")
        elif cmd in ("point_cloud", "view_live"):
            self._cmd({"action": "point_cloud"})
            self._open_lidar_view("cloud")
            print("[TCP] point_cloud + 本机点云/扇区窗口")
        elif cmd == "view_map":
            self._cmd({"action": "lidar_map"})
            self._open_lidar_view("map")
        elif cmd == "view_both":
            self._cmd({"action": "point_cloud"})
            self._open_lidar_view("both")
        elif cmd == "view_off":
            if self._close_viewers():
                print("雷达窗口已关闭")
            else:
                print("没有正在跑的雷达窗口")
        elif cmd == "can_check":
            try:
                import check_can
                check_can.report()
            except Exception as exc:
                print(f"CAN 检测失败: {exc}")
        elif cmd == "query":
            st = self._refresh_state(1.5)
            if st is not None:
                self._show_state(st, force=True)
            else:
                print("[TCP] query 已下发，尚未收到状态（等一两秒再 q）")
        elif cmd == "tracks":
            _print_tracks()
        elif cmd == "track_record":
            if len(parts) < 2:
                print("用法: r <轨迹名>")
                return True
            name = parts[1]
            path = TRACKS / f"{name}.json"
            if path.is_file():
                if not self._await_yes_no(f"轨迹 '{name}' 已存在，覆盖? (y/n): "):
                    print("已取消录制")
                    return True
                self._cmd({"action": "track_delete", "trackId": name})
                time.sleep(0.3)
            self._kb_start(record_name=name)
        elif cmd == "track_follow":
            if len(parts) < 2:
                print("用法: f <轨迹名>")
                return True
            self._kb_start(play_name=parts[1])
        elif cmd == "track_follow_back":
            if len(parts) < 2:
                print("用法: fb <轨迹名>")
                return True
            self._kb_start(play_name=parts[1], roundtrip=True)
        elif cmd == "task_submit":
            if len(parts) < 2:
                print("用法: task <轨迹名> [taskId]")
                return True
            task_id = parts[2] if len(parts) > 2 else f"T{int(time.time() * 1000) % 100000}"
            self._cmd({
                "action": "task_submit",
                "taskId": task_id,
                "trackId": parts[1],
                "from": "",
                "to": "",
            })
            print(f"[TCP] task_submit taskId={task_id} trackId={parts[1]}")
        elif cmd == "track_delete":
            if len(parts) < 2:
                print("用法: d <轨迹名>")
                return True
            self._cmd({"action": "track_delete", "trackId": parts[1]})
            print(f"[TCP] track_delete {parts[1]}")
        else:
            print(f"未知指令: {cmd}（输入 help 查看）")
        return True

    def _await_yes_no(self, prompt: str) -> bool:
        sys.stdout.write(prompt)
        sys.stdout.flush()
        keys = self._keys
        if keys is None:
            print("n（无交互终端，不覆盖）")
            return False
        buf = ""
        while True:
            ch = keys.read_char(0.1)
            if ch is None:
                continue
            if ch in ("\r", "\n"):
                sys.stdout.write("\n")
                sys.stdout.flush()
                return buf.strip().lower().startswith("y")
            if ch in ("\x03", "\x04"):
                sys.stdout.write("\n")
                sys.stdout.flush()
                return False
            if ch in ("backspace", "\x7f", "\x08"):
                if buf:
                    buf = buf[:-1]
                    sys.stdout.write("\b \b")
                    sys.stdout.flush()
                continue
            if ch in ("esc", "space"):
                continue
            buf += ch
            sys.stdout.write(ch)
            sys.stdout.flush()

    def _kb_sample(self) -> tuple[int, int, dict]:
        keys = self._keys
        if keys is None:
            return 0, 0, {"q": True, "space": False, "plus": False, "minus": False, "b": False}
        fv, fw = keys.drive_stick()
        return fv, fw, {
            "q": keys.pressed("q") or keys.take_press("q") or keys.take_press("\x03"),
            "space": keys.pressed("space"),
            "plus": keys.pressed("+") or keys.pressed("="),
            "minus": keys.pressed("-") or keys.pressed("_"),
            "b": keys.take_press("b"),
        }

    def _kb_send_stick(self, v: float, w: float) -> None:
        try:
            if abs(v) < 1e-6 and abs(w) < 1e-6:
                if not self._kb_zero_sent:
                    self.client.stick(0.0, 0.0)
                    self._kb_last_vw = (0.0, 0.0)
                    self._kb_zero_sent = True
                return
            now = time.monotonic()
            if now - self._kb_last_send < KB_KEEPALIVE_S and self._kb_last_vw == (v, w):
                return
            self.client.stick(v, w)
            self._kb_last_send = now
            self._kb_last_vw = (v, w)
            self._kb_zero_sent = False
        except (TimeoutError, OSError) as exc:
            print(f"\n[TCP] stick 失败: {exc}")

    def _kb_drive_loop(self) -> None:
        """独立 100Hz：按键一到立刻 stick，不跟命令行抢节拍。"""
        keys = self._keys
        while not self._kb_drive_stop.is_set() and self._kb_active:
            if keys is not None:
                keys.wait_drive(KB_SAMPLE_S)
            else:
                time.sleep(KB_SAMPLE_S)
            if not self._kb_active or self._kb_drive_stop.is_set():
                break
            if self._kb_mode not in ("drive", "record") or self._kb_rewinding:
                continue
            if self._kb_space_sent:
                continue
            if keys is None:
                continue
            fv, fw = keys.drive_stick()
            # 终端字符只有 10～30Hz，底盘 TX 是 100Hz。空档不能发 (0,0)，
            # 否则 CAN 会跟着停——角速度没有线速度惯性，卡顿最明显。
            # 有过非零轴且最近还有 WASD 事件时，按 100Hz 重复最后一次杆量。
            if fv == 0 and fw == 0:
                latched = getattr(self, "_kb_axis", (0, 0))
                if latched != (0, 0) and keys.drive_event_age() < 0.30:
                    fv, fw = latched
                else:
                    self._kb_axis = (0, 0)
            else:
                self._kb_axis = (fv, fw)
            self._kb_send_stick(fv * self._kb_v, fw * self._kb_w)
        try:
            self.client.stick(0.0, 0.0)
        except (TimeoutError, OSError):
            pass

    def _kb_start(self, record_name: str = "", play_name: str = "",
                  roundtrip: bool = False) -> None:
        if self._keys is None:
            print("当前不是交互终端，无法 kb。请在工控机本机或 SSH TTY 里运行。")
            return
        try:
            self.client.stick(0.0, 0.0)
        except (TimeoutError, OSError):
            pass
        self._keys.drain()
        self._keys.clear_press_states()
        self._kb_mode = (
            "fb" if roundtrip else ("play" if play_name else ("record" if record_name else "drive"))
        )
        self._kb_record = record_name
        self._kb_play = play_name
        self._kb_v = KB_LINEAR
        self._kb_w = KB_ANGULAR
        self._kb_last_send = 0.0
        self._kb_last_vw = (0.0, 0.0)
        self._kb_axis = (0, 0)
        self._kb_plus_prev = False
        self._kb_minus_prev = False
        self._kb_b_prev = False
        self._kb_rewinding = False
        self._kb_status_t = 0.0
        self._kb_space_sent = False
        self._kb_zero_sent = True
        self._kb_drive_stop.clear()
        self._kb_active = True
        self._kb_drive_th = threading.Thread(
            target=self._kb_drive_loop, name="kb-stick", daemon=True,
        )
        self._kb_drive_th.start()
        hint = (
            f"档位 v={self._kb_v:.2f} w={self._kb_w:.2f}。"
            "这是终端字符流，不是 HID：松键约 50ms 才停，无真双轴，"
            "达不到老师的 100Hz/20ms。真遥控请用笔记本 teleop_from_laptop.py。"
            " WASD  +/-点按  SPACE急停  Q 退出。"
        )
        if record_name:
            self._cmd({"action": "track_record", "name": record_name})
            print(f"录制已开启（{record_name}）。{hint}")
        elif play_name:
            self._cmd({
                "action": "track_follow",
                "trackId": play_name,
                "bypassGuard": True,
            })
            extra = "到终点后自动返回。" if roundtrip else "按 B 原路返回。"
            print(f"回放已开启（{play_name}）。{extra}{hint}")
        else:
            print(f"键盘控制。{hint}")

    def _kb_tick(self) -> bool:
        """一拍键盘控制。返回 False 表示退出 kb，回到命令行。"""
        keys = self._keys
        if keys is None:
            return False
        keys.drain()
        _fv, _fw, btn = self._kb_sample()

        if btn["q"]:
            self._kb_drive_stop.set()
            try:
                self.client.stick(0.0, 0.0)
            except (TimeoutError, OSError):
                pass
            if self._kb_record:
                self._cmd({"action": "track_record"})
                print("录制已停止并保存")
            print("已退出键盘模式")
            return False
        if btn["space"]:
            if not self._kb_space_sent:
                self._estop()
                self._kb_space_sent = True
            return True
        self._kb_space_sent = False

        plus_now, minus_now = btn["plus"], btn["minus"]
        if plus_now and not self._kb_plus_prev:
            self._kb_v = min(0.50, self._kb_v + 0.01)
            self._kb_w = min(1.00, self._kb_w + 0.02)
            print(f"档位 ↑ v={self._kb_v:.2f} w={self._kb_w:.2f}")
        if minus_now and not self._kb_minus_prev:
            self._kb_v = max(0.05, self._kb_v - 0.01)
            self._kb_w = max(0.10, self._kb_w - 0.02)
            print(f"档位 ↓ v={self._kb_v:.2f} w={self._kb_w:.2f}")
        self._kb_plus_prev, self._kb_minus_prev = plus_now, minus_now

        ev = self._last_event
        if ev and str(ev.get("event") or "") == "arrived":
            text = str(ev.get("msg") or "")
            reverse = "Reverse" in text or "back at start" in text
            if self._kb_mode == "fb" and not reverse and not self._kb_rewinding:
                self._cmd({
                    "action": "track_follow",
                    "trackId": self._kb_play,
                    "reverse": True,
                    "bypassGuard": True,
                })
                self._kb_rewinding = True
                print("往返：已到终点，正在原路返回")
            elif self._kb_mode == "fb" and reverse:
                print("往返完成，回到命令模式")
                self._last_event = None
                return False
            self._last_event = None

        b_edge = btn["b"] and not self._kb_b_prev
        self._kb_b_prev = btn["b"]
        if b_edge and not self._kb_rewinding:
            if self._kb_record:
                self._cmd({"action": "track_record"})
                time.sleep(0.2)
                self._cmd({
                    "action": "track_follow",
                    "trackId": self._kb_record,
                    "reverse": True,
                    "bypassGuard": True,
                })
                self._kb_rewinding = True
                print("已保存，正在原路返回起点")
            elif self._kb_play:
                self._cmd({
                    "action": "track_follow",
                    "trackId": self._kb_play,
                    "reverse": True,
                    "bypassGuard": True,
                })
                self._kb_rewinding = True
                print("正在原路返回起点")

        return True

    def run(self) -> bool:
        """交互主循环。True=正常退出。全程共用一套 termios 按键，不再用 input()。"""
        keys = None
        tty_fd: int | None = None
        if TermKeyReader is not None:
            try:
                tty_fd = os.open("/dev/tty", os.O_RDWR)
                keys = TermKeyReader(fd=tty_fd)
            except OSError:
                keys = TermKeyReader()
                tty_fd = None
            if not getattr(keys, "_is_tty", False):
                keys = None
                if tty_fd is not None:
                    os.close(tty_fd)
                    tty_fd = None
        if keys is None:
            print("当前 stdin 不是 TTY，无法进入 kb。命令仍可用，但请在 SSH/本机终端运行。")
            return self._run_input_fallback()

        self._keys = keys
        keys.start()
        buf: list[str] = []
        prompt = "local> "
        sys.stdout.write(prompt)
        sys.stdout.flush()
        running = True
        try:
            while running:
                if self._kb_active:
                    if not self._kb_tick():
                        self._kb_active = False
                        self._kb_drive_stop.set()
                        try:
                            self.client.stick(0.0, 0.0)
                        except (TimeoutError, OSError):
                            pass
                        sys.stdout.write(prompt)
                        sys.stdout.flush()
                    time.sleep(0.02)
                    continue
                ch = keys.read_char(0.05)
                if ch is None:
                    continue
                if ch in ("\r", "\n"):
                    sys.stdout.write("\n")
                    sys.stdout.flush()
                    line = "".join(buf)
                    buf.clear()
                    if not line:
                        sys.stdout.write(prompt)
                        sys.stdout.flush()
                        continue
                    if not self.dispatch(line):
                        running = False
                        break
                    if not self._kb_active:
                        sys.stdout.write(prompt)
                        sys.stdout.flush()
                    continue
                if ch in ("\x03", "\x04"):
                    self._estop()
                    self._close_viewers()
                    print("Ctrl+C — 已急停并退出")
                    running = False
                    break
                if ch in ("\x7f", "backspace"):
                    if buf:
                        buf.pop()
                        sys.stdout.write("\b \b")
                        sys.stdout.flush()
                    continue
                if ch in ("esc",):
                    continue
                if ch == "space":
                    ch = " "
                buf.append(ch)
                sys.stdout.write(ch)
                sys.stdout.flush()
        finally:
            self._kb_active = False
            self._kb_drive_stop.set()
            try:
                keys.close()
            except Exception:
                pass
            self._keys = None
            if tty_fd is not None:
                try:
                    os.close(tty_fd)
                except OSError:
                    pass
        return True

    def _run_input_fallback(self) -> bool:
        while True:
            try:
                line = input("local> ")
            except EOFError:
                return True
            except KeyboardInterrupt:
                print()
                self._estop()
                self._close_viewers()
                print("Ctrl+C — 已急停并退出")
                return True
            if not self.dispatch(line):
                return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="本地任务启动器（推荐）：TCP 实时控制，不走已废弃的云端 WebSocket",
    )
    parser.add_argument("--bringup", action="store_true",
                        help="启动前先跑 bringup_gs_usb_can.sh")
    parser.add_argument("--host", default=None, help="车侧 TCP 地址（默认 127.0.0.1）")
    parser.add_argument("--port", type=int, default=None, help="车侧 TCP 端口（默认 9100）")
    parser.add_argument("--token", default=None, help="teleop token")
    parser.add_argument("--attach", action="store_true",
                        help="只连已有 :9100，不新开代理")
    parser.add_argument("--agent-logs", action="store_true",
                        help="把车侧日志也打到本终端（默认只写 agent.log，避免冲掉 local>）")
    parser.add_argument("--no-web", action="store_true",
                        help="不启动网页遥控 :9101")
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT,
                        help="网页遥控端口（默认 9101）")
    parser.add_argument(
        "--daemon", action="store_true",
        help="无头模式：不读终端、不弹桌面窗，供 systemd 开机自启",
    )
    parser.add_argument(
        "--no-lidar", action="store_true",
        help="启动时代理不开雷达（网页里仍可 lidar_on）。--daemon 默认带上",
    )
    args = parser.parse_args()

    if args.daemon:
        args.no_lidar = True

    if not JETSON.is_dir():
        print(f"找不到车侧目录: {JETSON}", file=sys.stderr)
        return 2

    env = os.environ.copy()
    _load_conf(AGENT_CONF, env)
    # mission.conf 是无人值守实战用的：BUNKER_MISSION_LOCK=1 会让代理
    # 默默丢掉全部 kb/move（终端有速度、底盘不动）。本地控制台禁止继承。
    _load_conf(MISSION_CONF, env)
    env["BUNKER_LOCAL_TCP"] = "1"
    env["BUNKER_MISSION_LOCK"] = "0"
    env.pop("BUNKER_AUTO_MISSION", None)
    if args.daemon:
        env = _headless_env(env)
    elif args.no_lidar:
        env["BUNKER_ENABLE_LIDAR"] = "0"

    host = args.host or env.get("BUNKER_TELEOP_HOST", "127.0.0.1")
    if host in ("0.0.0.0", ""):
        host = "127.0.0.1"
    port = int(args.port or env.get("BUNKER_TELEOP_PORT", str(DEFAULT_PORT)))
    token = args.token or env.get("BUNKER_TELEOP_TOKEN", DEFAULT_TOKEN)
    bringup = args.bringup or env.get("BRINGUP_CAN", "0") not in (
        "0", "false", "False", "no",
    )

    lan_list = _lan_ips()
    lan = " ".join(lan_list) or "<工控机IP>"
    lan0 = lan_list[0] if lan_list else "59.79.233.120"
    radar_ips = [ip for ip in lan_list if ip.startswith("192.168.1.")]
    print("════════════════════════════════════════════════════════")
    if args.daemon:
        print("  本地控制台无头模式（systemd / --daemon；不开终端、不弹桌面窗）")
        print("  雷达默认关，笔记本打开网页后可点「雷达开」")
    else:
        print("  本地任务启动器（推荐入口；云端模式已废弃，请勿再开 mock_cloud）")
    print(f"  本进程将连  {host}:{port}   agent 自带 9100，勿开 bunker-teleop.service")
    print(f"  代理起来后听 0.0.0.0:{port}  token={token}  （局域网可连，勿暴露外网）")
    print(f"  本机地址: {lan}")
    if radar_ips:
        print(f"  注意: {' '.join(radar_ips)} 是雷达网口，笔记本不要填这个")
    print()
    print("  笔记本打开网页（Wi-Fi/校园网 IP，不是 192.168.1.x）：")
    print(f"    http://{lan0}:{args.web_port}")
    if not args.daemon:
        print("    或 PowerShell: python teleop_from_laptop.py --host " + lan0)
        print("  本窗口 kb = SSH 字符流，只有 10～30Hz，仅调试。")
        print("  本窗口仍可输入 fo / goto / r / f。Ctrl+C 立即急停退出。")
    print("  车侧日志: bunker_jetson/agent.log")
    if not args.daemon:
        _print_can_health()
        _print_jetson_power_hint()
    print("════════════════════════════════════════════════════════")

    if bringup:
        script = JETSON / "bringup_gs_usb_can.sh"
        print("拉起 USB-CAN ...")
        subprocess.call(["bash", str(script)], cwd=str(JETSON))

    children: list[subprocess.Popen] = []
    started_agent = False
    agent = None
    logf = None

    def _stop_children() -> None:
        for proc in reversed(children):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        deadline = time.time() + 5.0
        for proc in children:
            remain = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=remain)
            except subprocess.TimeoutExpired:
                proc.kill()

    if args.attach:
        if not _port_open(host, port):
            print(f"--attach 但 {host}:{port} 没有在听", file=sys.stderr)
            return 1
        print(f"只连接已有 TCP {host}:{port}，不新开代理")
    elif _port_open(host, port):
        print(
            f"{host}:{port} 已被占用。不要复用可能带着「实战锁」的旧代理，"
            "否则终端有速度、底盘不动。\n"
            "请先结束旧进程再开本启动器：\n"
            "  sudo systemctl stop bunker-local\n"
            "  pkill -f 'run_agent.py'\n"
            "  pkill -f run_local.py\n"
            "确认旧代理是本地模式且无实战锁时才可用: python3 run_local.py --attach",
            file=sys.stderr,
        )
        return 1
    else:
        log_path = Path(env.get("BUNKER_LOG_FILE") or "agent.log")
        if not log_path.is_absolute():
            log_path = JETSON / log_path
        agent_cmd = [
            sys.executable, "run_agent.py",
            "--local",
            "--config", "agent.conf",
            "--log-file", str(log_path),
            "--interface", env.get("BUNKER_CAN_INTERFACE", "socketcan"),
        ]
        if env.get("BUNKER_CAN_CHANNEL"):
            agent_cmd.extend(["--channel", env["BUNKER_CAN_CHANNEL"]])
        if args.no_lidar:
            agent_cmd.append("--no-lidar")
        popen_kw: dict = {"cwd": str(JETSON), "env": env}
        if not args.agent_logs:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            logf = open(log_path, "a", encoding="utf-8")
            popen_kw["stdout"] = logf
            popen_kw["stderr"] = subprocess.STDOUT
        agent = subprocess.Popen(agent_cmd, **popen_kw)
        children.append(agent)
        started_agent = True
        print("车侧代理已在本进程里拉起（本地 TCP，不连云端）")
        print(f"车侧详细日志: {log_path}   （想看刷屏加 --agent-logs）")
        print(f"正在等代理监听 {host}:{port} …")
        try:
            ready = _wait_port(host, port, 20.0, proc=agent)
        except KeyboardInterrupt:
            print("\n启动未完成：代理还在初始化 CAN，:9100 尚未就绪。已中断。")
            _stop_children()
            if logf is not None:
                logf.close()
            return 130
        if not ready:
            print(f"代理未在 {host}:{port} 监听，退出", file=sys.stderr)
            if log_path.is_file():
                try:
                    tail = log_path.read_text(encoding="utf-8", errors="replace")[-1500:]
                    print(tail, file=sys.stderr)
                except OSError:
                    pass
            _stop_children()
            if logf is not None:
                logf.close()
            return 1

    try:
        client = TeleopTcpClient(host, port, token)
        rtt = client.hello()
        print(f"TCP 已连接  hello RTT={rtt / 1000.0:.1f} ms")
    except Exception as exc:
        print(f"连接 TCP {host}:{port} 失败: {exc}", file=sys.stderr)
        _stop_children()
        return 1

    web = None
    if not args.no_web and TeleopWebServer is not None:
        try:
            web = TeleopWebServer(
                tcp_host=host, tcp_port=port, token=token, http_port=args.web_port,
            )
            web.start()
            print("网页控制台已开。笔记本打开（WASD 遥控 + 任务按钮）：")
            for url in web.urls():
                print(f"    {url}")
        except OSError as exc:
            print(f"网页遥控 :{args.web_port} 未拉起: {exc}", file=sys.stderr)
            web = None

    console = None
    rc = 0
    try:
        if args.daemon:
            rc = _wait_daemon_stop(agent)
        else:
            console = LocalConsole(client, env)
            _print_help()
            try:
                console.run()
            except KeyboardInterrupt:
                print("\n停车退出")
                rc = 130
    finally:
        if console is not None:
            console._stop = True
            console._close_viewers()
        if web is not None:
            web.stop()
        try:
            client.estop()
        except Exception:
            pass
        client.close()
        if started_agent:
            _stop_children()
        if logf is not None:
            try:
                logf.close()
            except OSError:
                pass
    return rc


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已中断")
        raise SystemExit(130)
