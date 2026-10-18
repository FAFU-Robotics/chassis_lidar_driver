#!/usr/bin/env python3
"""RoboSense Airy 点云/全局地图实时图形查看器（WebSocket 云端链路）。

连接「模拟云端/甲方云端」→ 鉴权 → 开启点云流 → 持续接收 ``point_cloud``
事件渲染点云窗口，并可切换到 **全局地图模式**（轮询 ``lidar_map``，
渲染 SLAM 式逐步展开的世界系 2D 伪地图 + 小车箭头 + 历史轨迹 + goto 目标预检）。

三种视图模式：
  - ``cloud``：实时点云（2D 俯视 / --3d 3D），默认；
  - ``map``：全局 2D 伪地图（世界系固定视角，小车移动地图逐步展开）；
  - ``both``：点云 + 全局地图左右并排。

无 GUI / 无 matplotlib 环境自动回退终端渲染（ANSI 清屏动画 + 颜色）。

与车侧代理共用同一 deviceId/bindCode（同一「车」），因此：
  - 在 mock_cloud 控制台输入 ``view live`` / ``view map`` / ``view both``
    即可自动打开（子进程方式）；
  - 也可手动启动：``python examples\\live_viewer.py --mode map``。

依赖：matplotlib（可选，无则回退终端渲染）。安装：
    pip install matplotlib
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import deque

try:
    import websocket
except ImportError:
    print(
        "缺少 'websocket-client' 包。请安装:\n"
        "    pip install websocket-client",
        file=sys.stderr,
    )
    sys.exit(1)

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.ascii_view import (  # noqa: E402
    render_top_down_ascii,
)

# 车体轮廓（Bunker Mini 2.0：长 0.69 m / 宽 0.57 m，车头朝上）
CAR_HALF_W = 0.285
CAR_LEN = 0.69
DEFAULT_MAX_RANGE_M = 8.0
DEFAULT_MAP_RANGE_M = 12.0
TRAJ_MAX = 5000          # 轨迹线最多保留点数
MAP_POLL_INTERVAL_S = 1.0  # 地图模式轮询 lidar_map 的周期


def _log(msg: str) -> None:
    print(f"[viewer] {msg}", flush=True)


def _probe_local_display() -> Optional[str]:
    """SSH/无头环境下探测本机桌面 X server（Jetson 工控机接显示器）。

    返回可用的 DISPLAY 值（如 ``:0``），找不到返回 None。探测顺序：
      1. 环境变量已有的 DISPLAY；
      2. ``/tmp/.X11-unix/`` 下的 X socket（X0/X1...），即本机物理桌面。

    找到后会把 ``DISPLAY`` 与 ``XAUTHORITY`` 一并写回进程环境，使
    matplotlib/Tk 能连上 Jetson 桌面弹窗——否则 SSH 会话里 ``DISPLAY`` 为空，
    图形窗口初始化失败会静默回退成终端 ASCII，也就是「雷达结果只能看文字」。
    """
    if os.environ.get("DISPLAY"):
        return os.environ["DISPLAY"]

    x11_dir = "/tmp/.X11-unix"
    try:
        sockets = sorted(
            n for n in os.listdir(x11_dir) if n.startswith("X"))
    except OSError:
        sockets = []
    for sock in sockets:
        num = sock[1:]  # "X0" -> "0"
        display = f":{num}"
        xauth = None
        for cand in (os.path.join(os.path.expanduser("~"), ".Xauthority"),
                     f"/run/user/{os.getuid()}/gdm/Xauthority"):
            if os.path.exists(cand):
                xauth = cand
                break
        os.environ["DISPLAY"] = display
        if xauth:
            os.environ["XAUTHORITY"] = xauth
        # 轻量验证：能读到 X socket 即假定可用（xauthority 已带上）
        return display
    return None


def connect_and_auth(url: str, device_id: str, bind_code: str):
    """连接云端并完成鉴权，返回 (ws, token)。"""
    ws = websocket.create_connection(url, timeout=10)
    ws.settimeout(5.0)
    ws.send(
        json.dumps(
            {
                "type": "auth",
                "deviceId": device_id,
                "ts": int(time.time() * 1000),
                "token": "",
                "payload": {"bindCode": bind_code},
            },
            ensure_ascii=False,
        )
    )
    deadline = time.time() + 15.0
    while time.time() < deadline:
        try:
            msg = json.loads(ws.recv())
        except Exception:
            continue
        if msg.get("type") == "auth":
            code = msg.get("code", 0)
            token = msg.get("token") or (msg.get("payload") or {}).get("token", "")
            if code == 401 or not token:
                raise RuntimeError(f"鉴权失败 (code={code}): 请核对 deviceId/bindCode")
            return ws, token
    raise RuntimeError("鉴权超时：未收到 token（云端是否在运行？）")


def _send_cmd(ws, device_id: str, token: str, payload: dict) -> None:
    ws.send(
        json.dumps(
            {
                "type": "cmd",
                "deviceId": device_id,
                "ts": int(time.time() * 1000),
                "token": token,
                "payload": payload,
            },
            ensure_ascii=False,
        )
    )


# ---------------------------------------------------------------------------
# 2D 点云渲染器
# ---------------------------------------------------------------------------


def _build_2d_axes(fig, ax):
    """车体轮廓 + 网格，绘制一次，每帧只更新散点。"""
    from matplotlib.patches import Rectangle

    ax.add_patch(
        Rectangle((-CAR_HALF_W, 0.0), 2 * CAR_HALF_W, CAR_LEN,
                  fill=False, edgecolor="red", linewidth=2)
    )
    ax.annotate("车头", xy=(0.0, CAR_LEN), xytext=(0.0, CAR_LEN + 0.4),
                ha="center", color="red", fontsize=9)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.set_xlabel("右（米）")
    ax.set_ylabel("前（米）")
    ax.set_title("实时点云（上=车头，颜色=反射强度）")


class LivePointCloudRenderer:
    """matplotlib 实时渲染器：2D 俯视 / 3D，GUI 不可用时走终端 ASCII。"""

    def __init__(self, fig, ax, use_3d: bool = False,
                 max_range_m: float = DEFAULT_MAX_RANGE_M):
        self.max_range = max_range_m
        self.use_3d = use_3d
        self.fig = fig
        self.ax = ax
        self.sc = None
        self._frame_count = 0
        if not use_3d:
            _build_2d_axes(fig, ax)
            ax.set_xlim(-max_range_m, max_range_m)
            ax.set_ylim(-max_range_m, max_range_m)

    def render(self, data: dict) -> None:
        """更新一帧。data 来自 point_cloud 事件。"""
        pts = data.get("points") or []
        self._frame_count += 1
        if not pts:
            return
        xs = [float(p[0]) for p in pts]
        ys = [float(p[1]) for p in pts]
        has_intensity = len(pts[0]) >= 4
        colors = [float(p[3]) for p in pts] if has_intensity else "tab:blue"

        if self.use_3d:
            zs = [float(p[2]) for p in pts]
            if self.sc is None:
                self.sc = self.ax.scatter(xs, ys, zs, c=colors, s=2, alpha=0.7)
            else:
                # 3D 无 set_offsets2D 快捷路径，直接清空重建（帧率低可接受）
                self.sc.remove()
                self.sc = self.ax.scatter(xs, ys, zs, c=colors, s=2, alpha=0.7)
        else:
            import numpy as np

            if self.sc is None:
                self.sc = self.ax.scatter(
                    xs, ys, c=colors, cmap="jet", s=2, alpha=0.7)
            else:
                self.sc.set_offsets(np.stack([xs, ys], axis=-1))
                if has_intensity:
                    self.sc.set_array(np.asarray(colors))
        self.fig.canvas.draw_idle()


# ---------------------------------------------------------------------------
# 全局地图渲染器（SLAM 式建图）
# ---------------------------------------------------------------------------

_MAP_STATE_COLOR = {"free": "green", "occupied": "orange", "blocked": "red"}


class GlobalMapRenderer:
    """世界系固定视角的 2D 伪地图渲染：格点 + 小车箭头 + 轨迹 + 目标预检。

    世界系约定（与 occupancy 一致）：yaw=0 时车头沿世界 +x，逆时针为正。
    """

    def __init__(self, fig, ax, max_range_m: float = DEFAULT_MAP_RANGE_M,
                 target: tuple[float, float] | None = None,
                 save_dir: str | None = None):
        self.max_range = max_range_m
        self.fig = fig
        self.ax = ax
        self.target = target
        self.save_dir = save_dir
        self.follow = True
        self._pose = (0.0, 0.0, 0.0)
        self._traj: deque[tuple[float, float]] = deque(maxlen=TRAJ_MAX)
        self._occupied: list = []
        self._blocked: list = []
        self._free: list = []
        self._target_state: str | None = None
        self._updates = 0
        self._art = {}

    def setup(self) -> None:
        ax = self.ax
        ax.set_aspect("equal")
        ax.grid(True, linestyle=":", alpha=0.4)
        ax.set_xlabel("地面 x（米）  0°朝向这一边")
        ax.set_ylabel("地面 y（米）")
        ax.set_title("雷达地图（车在走、图在长）绿=空地 橙=障碍 红叉=过不去")
        self._art["sc_free"] = ax.scatter([], [], s=1, c="green", alpha=0.35,
                                          label="空地")
        self._art["sc_occ"] = ax.scatter([], [], s=6, c="orange", alpha=0.85,
                                         label="障碍")
        self._art["sc_blk"] = ax.scatter([], [], s=12, marker="x", c="red",
                                         label="过不去")
        (self._art["traj"],) = ax.plot([], [], "-", color="tab:blue", lw=1.2,
                                       alpha=0.8, label="走过的路")
        self._art["arrow"] = ax.annotate(
            "", xy=(0, 0), xytext=(0, 0), ha="center", fontsize=11,
            arrowprops=dict(arrowstyle="-|>", color="black", lw=2.5))
        if self.target is not None:
            (self._art["target"],) = ax.plot([self.target[0]], [self.target[1]],
                                             marker="*", ms=16, ls="none",
                                             color="magenta",
                                             label="目标点")
            self._art["target_label"] = ax.text(
                self.target[0], self.target[1], " 目标", color="magenta",
                fontsize=9, va="center")
        ax.legend(loc="upper right", fontsize=8)

    def update_map(self, snap: dict) -> None:
        """用最新一次 lidar_map 快照整体替换格点集合（代理端地图已全局累积）。"""
        self._occupied = snap.get("occupied") or []
        self._blocked = snap.get("blocked") or []
        self._free = snap.get("free") or []
        tc = snap.get("targetCheck")
        if tc:
            self._target_state = tc.get("state")
        self._updates += 1

    def add_pose(self, pose: dict) -> None:
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        self._pose = (x, y, float(pose.get("yawDeg", 0.0)))
        self._traj.append((x, y))

    def render(self) -> None:
        ax = self.ax
        x, y, yaw = self._pose
        self._art["sc_free"].set_offsets(self._free)
        self._art["sc_occ"].set_offsets(self._occupied)
        self._art["sc_blk"].set_offsets(self._blocked)
        traj = list(self._traj)
        self._art["traj"].set_data([p[0] for p in traj], [p[1] for p in traj])
        # 小车箭头：世界系 yaw=0 → 车头沿 +x
        import math

        dx, dy = math.cos(math.radians(yaw)) * 0.4, math.sin(math.radians(yaw)) * 0.4
        self._art["arrow"].xy = (x + dx, y + dy)
        self._art["arrow"].xytext = (x, y)
        if self.target is not None and self._target_state:
            color = _MAP_STATE_COLOR.get(self._target_state, "gray")
            self._art["target"].set_color(color)
            zh = {"free": "可以走", "occupied": "有障碍",
                  "blocked": "过不去", "unknown": "没扫到"}.get(
                self._target_state, self._target_state)
            self._art["target_label"].set_text(
                f" 目标 ({self.target[0]:.1f},{self.target[1]:.1f}) [{zh}]")
            self._art["target_label"].set_color(color)
        # 跟随/固定视角
        if self.follow:
            ax.set_xlim(x - self.max_range, x + self.max_range)
            ax.set_ylim(y - self.max_range, y + self.max_range)
        self.fig.canvas.draw_idle()
        self._maybe_save_snapshot()

    def _maybe_save_snapshot(self) -> None:
        """每 ~30 次地图更新自动存一张 PNG（--save-dir，回放建图过程）。"""
        if not self.save_dir or self._updates % 30 != 0:
            return
        try:
            import datetime

            os.makedirs(self.save_dir, exist_ok=True)
            path = os.path.join(
                self.save_dir,
                f"map_{datetime.datetime.now():%Y%m%d_%H%M%S}.png",
            )
            self.fig.savefig(path, dpi=110)
            _log(f"地图快照已保存: {path}")
        except Exception as exc:
            _log(f"地图快照保存失败: {exc}")


# ---------------------------------------------------------------------------
# 终端渲染（ANSI 动画 + 颜色）
# ---------------------------------------------------------------------------


class TerminalRenderer:
    """终端模式：可选 ANSI 清屏动画；非 TTY 时回退为逐行日志。"""

    def __init__(self, animate: bool, colors: bool):
        self.animate = animate and sys.stdout.isatty()
        self.colors = colors and self.animate
        self._count = 0

    @staticmethod
    def _clear() -> str:
        return "\033[2J\033[H"

    def show_cloud(self, data: dict) -> None:
        self._count += 1
        n = data.get("pointCount", 0)
        pose = data.get("pose") or {}
        body = (
            f"帧#{self._count}  点数={n}  pose=({pose.get('x', 0):.2f}, "
            f"{pose.get('y', 0):.2f}, yaw {pose.get('yawDeg', 0):.1f}°)"
        )
        ascii_view = render_top_down_ascii(
            data.get("points") or [], colors=self.colors)
        if self.animate:
            out = self._clear() + body + "\n" + ascii_view
            print(out, end="")
        else:
            _log(body)
            if self._count % 5 == 0:
                for line in ascii_view.splitlines():
                    _log("  | " + line)

    def show_map_stats(self, snap: dict, note: str = "") -> None:
        pose = snap.get("pose") or {}
        tc = snap.get("targetCheck") or {}
        line = (
            f"[map] 占用格={snap.get('occupiedCells', 0)} "
            f"不可通行={snap.get('blockedCells', 0)} "
            f"自由格={snap.get('freeCells', 0)}  "
            f"pose=({pose.get('x', 0):.2f}, {pose.get('y', 0):.2f}, "
            f"yaw {pose.get('yawDeg', 0):.1f}°)"
        )
        if tc:
            zh = tc.get("stateZh") or tc.get("state")
            line += f"  目标({tc.get('x')},{tc.get('y')})→{zh}"
        if note:
            line += f"  {note}"
        _log(line)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def _make_figure(ax_spec: str):
    """按模式创建 figure / axes。返回 (fig, cloud_ax, map_ax)。

    Jetson 桌面不一定有 Tk，按 TkAgg → GTK3Agg → Qt5Agg 试后端。
    """
    import matplotlib

    last_err: Exception | None = None
    plt = None
    for backend in ("TkAgg", "GTK3Agg", "Qt5Agg"):
        try:
            matplotlib.use(backend, force=True)
            import matplotlib.pyplot as plt  # noqa: F401
            break
        except Exception as exc:
            last_err = exc
            sys.modules.pop("matplotlib.pyplot", None)
    else:
        raise last_err or RuntimeError("没有可用的 matplotlib 图形后端")

    if ax_spec == "map":
        fig = plt.figure(figsize=(7, 7))
        map_ax = fig.add_subplot(111)
        return fig, None, map_ax
    if ax_spec == "both":
        fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(13, 6.5))
        return fig, ax_l, ax_r
    fig = plt.figure(figsize=(7, 7))
    return fig, fig.add_subplot(111), None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RoboSense Airy 点云/全局地图实时查看器（WebSocket 云端链路）")
    parser.add_argument("--ws-url", default="ws://127.0.0.1:9000",
                        help="云端 WebSocket 地址（默认 ws://127.0.0.1:9000）")
    parser.add_argument("--device-id", default="BUNKER-TEST01")
    parser.add_argument("--bind-code", default="TEST-BIND-xxxx")
    parser.add_argument("--hz", type=float, default=3.0,
                        help="点云推送频率 Hz（默认 3，车侧会钳制到 1~10）")
    parser.add_argument("--mode", choices=["cloud", "map", "both"], default="cloud",
                        help="视图模式：cloud=点云（默认）/ map=全局地图 / both=并排")
    parser.add_argument("--3d", action="store_true", help="点云用 3D 视图（帧率较低）")
    parser.add_argument("--range", type=float, default=None,
                        help="视图范围半径 m（默认：点云 8 / 地图 12）")
    parser.add_argument("--target", nargs=2, type=float, default=None, metavar=("X", "Y"),
                        help="goto 目标点 (x, y)，地图模式叠加预检结果标记")
    parser.add_argument("--save-dir", default=None,
                        help="地图快照自动保存目录（约每 30s 一张 PNG，回放建图过程）")
    parser.add_argument("--no-window", action="store_true",
                        help="强制终端渲染（不打开图形窗口）")
    parser.add_argument("--require-window", action="store_true",
                        help="图形窗口打不开就退出（给 mock_cloud 用，避免偷偷在日志里刷屏）")
    parser.add_argument("--display", default=None,
                        help="X DISPLAY（如 :0）。默认自动探测本机桌面 X server，"
                             "SSH/无头会话下也能在 Jetson 桌面弹窗")
    parser.add_argument("--no-animate", action="store_true",
                        help="终端模式禁用 ANSI 清屏动画（改逐行日志）")
    parser.add_argument("--no-start-lidar", action="store_true",
                        help="不自动下发 lidar on（默认会尝试开启雷达）")
    args = parser.parse_args()

    mode = args.mode
    pc_range = args.range or DEFAULT_MAX_RANGE_M
    map_range = args.range or DEFAULT_MAP_RANGE_M

    _log(f"连接云端 {args.ws_url}（device={args.device_id}，mode={mode}）...")
    ws, token = connect_and_auth(args.ws_url, args.device_id, args.bind_code)
    _log("鉴权通过，开始下发指令")

    if not args.no_start_lidar:
        _send_cmd(ws, args.device_id, token, {"action": "lidar_on"})
        _log("已下发 lidar on（可加 --no-start-lidar 跳过）")
    _send_cmd(ws, args.device_id, token, {"action": "pc_stream", "hz": args.hz})
    _log(f"已开启点云流 @ {args.hz:.1f} Hz（Ctrl+C 退出并关流）")

    use_gui = not args.no_window
    fig = cloud_renderer = map_renderer = None
    if use_gui:
        # SSH/无头会话里 DISPLAY 通常为空 → 先探测本机桌面 X server，
        # 让图形窗口弹出在 Jetson 工控机的显示器上而非回退终端 ASCII。
        if args.display:
            os.environ["DISPLAY"] = args.display
        else:
            disp = _probe_local_display()
            if disp:
                _log(f"检测到本机桌面 X display={disp}，将在桌面弹出图形窗口")
        try:
            ax_spec = mode
            fig, cloud_ax, map_ax = _make_figure(ax_spec)
            if cloud_ax is not None and mode in ("cloud", "both"):
                cloud_renderer = LivePointCloudRenderer(
                    fig, cloud_ax, use_3d=getattr(args, "3d", False), max_range_m=pc_range)
            if map_ax is not None and mode in ("map", "both"):
                map_renderer = GlobalMapRenderer(
                    fig, map_ax, max_range_m=map_range,
                    target=args.target, save_dir=args.save_dir)
                map_renderer.setup()
            import matplotlib.pyplot as plt

            plt.ion()
            plt.show()

            def _on_key(event):
                if map_renderer is not None and event.key == "f":
                    map_renderer.follow = not map_renderer.follow
                    _log(f"地图视角: {'跟随' if map_renderer.follow else '固定'}")

            fig.canvas.mpl_connect("key_press_event", _on_key)
        except Exception as exc:
            _log(f"图形窗口初始化失败（{exc}）→ 回退终端渲染")
            use_gui = False
            if fig is not None:
                try:
                    import matplotlib.pyplot as plt

                    plt.close(fig)
                except Exception:
                    pass
            fig = cloud_renderer = map_renderer = None
            if args.require_window:
                _log("要求必须有桌面窗口，退出让云端改用终端中文简图")
                try:
                    ws.close()
                except Exception:
                    pass
                sys.exit(2)

    term = TerminalRenderer(animate=not args.no_animate, colors=True)
    target_payload = {"x": args.target[0], "y": args.target[1]} if args.target else {}
    next_map_poll = time.time() + 1.0

    try:
        while True:
            try:
                raw = ws.recv()
            except websocket.WebSocketTimeoutException:
                raw = None
            except Exception as exc:
                _log(f"连接中断：{exc}")
                break

            # 地图模式轮询 lidar_map（silent 避免云端控制台刷屏）。
            # 放在循环开头而非 recv 超时分支：点云流高频时 recv 几乎不超时，
            # 若放在超时分支里地图轮询会因超时被跳过而永远不触发。
            if mode in ("map", "both") and time.time() >= next_map_poll:
                _send_cmd(ws, args.device_id, token, {
                    "action": "lidar_map",
                    "silent": True,
                    "includeFree": True,
                    **target_payload,
                })
                next_map_poll = time.time() + MAP_POLL_INTERVAL_S

            if raw is None:
                if use_gui and fig is not None:
                    try:
                        import matplotlib.pyplot as plt

                        plt.pause(0.02)
                    except Exception:
                        pass
                continue

            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") != "event":
                continue
            payload = msg.get("payload") or {}
            evt = payload.get("event")
            data = payload.get("data") or {}

            if evt == "point_cloud":
                if use_gui and (cloud_renderer is not None or map_renderer is not None):
                    pose = data.get("pose") or {}
                    if map_renderer is not None:
                        map_renderer.add_pose(pose)
                    if cloud_renderer is not None:
                        cloud_renderer.render(data)
                else:
                    term.show_cloud(data)
            elif evt == "lidar_map":
                if use_gui and map_renderer is not None:
                    map_renderer.update_map(data)
                    map_renderer.render()
                else:
                    term.show_map_stats(data)

            # 每帧刷新 GUI（流高频时 recv 几乎不超时，不能只依赖超时分支的 pause）
            if use_gui and fig is not None:
                try:
                    import matplotlib.pyplot as plt

                    plt.pause(0.02)
                except Exception:
                    pass

    except KeyboardInterrupt:
        _log("用户中断")
    finally:
        try:
            _send_cmd(ws, args.device_id, token, {"action": "pc_stream", "hz": 0})
            _log("已下发 pc_stream off")
        except Exception:
            pass
        try:
            ws.close()
        except Exception:
            pass
        if fig is not None:
            try:
                import matplotlib.pyplot as plt

                plt.close(fig)
            except Exception:
                pass


if __name__ == "__main__":
    main()
