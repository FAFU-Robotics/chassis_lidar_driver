"""Car-side Agent for BUNKER MINI 2.0 — WebSocket ↔ CAN bridge.

Connects to the cloud relay service via WebSocket, translates incoming remote
commands into CAN bus motion instructions, and periodically reports chassis
status back to the cloud.

Protocol version: v1.0 (per 运输智能小车·车侧接入工作流程手册)
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional

import websocket

from .approach import ApproachConfig, ApproachController
from .ascii_view import save_occupancy_png
from .can_util import (
    CanConfigError,
    ensure_all_socketcan_up,
    flush_socketcan_mailbox,
    list_socketcan_interfaces,
    probe_chassis_channel,
    leave_listen_only,
    resolve_can_config,
    restore_tx_mode,
    socketcan_is_listen_only,
    socketcan_operstate,
    usb_socketcan_channels,
    _is_usb_netdev,
)
from .controller import BunkerMiniController
from .lidar import AiryLidar, LidarError, MSOP_PORT, SelfMaskConfig
from .navigator import NavigateConfig, Navigator, OdometryPose, Pose2D
from .failsafe import (
    ACTIVE_MISSION_STATUSES,
    FailsafeMonitor,
    FailsafePolicy,
    LinkClass,
    PowerClass,
    checkpoint_path_for,
    should_return_on_power,
)
from .obstacle import ObstacleGuard, ObstaclePolicy
from .occupancy import BLOCKED, FREE, OCCUPIED, OccupancyGrid, PSEUDO_MAP_TTL_S
from .global_planner import GlobalPlanner
from .localization import MapAlignment
from .pcap import PcapReplaySource, PcapError
from .patrol import PatrolConfig, PatrolController
from .protocol import (
    ControlMode,
    FaultFlags,
    VehicleState,
    control_mode_label,
    vehicle_state_label,
)
from .scanmatch import ScanMatcher, ScanMatchConfig
from .teleop_tcp import TeleopTcpServer
from .tracker import (
    DEFAULT_TRACK_DIR,
    PlaybackCorrectionConfig,
    PlaybackDockConfig,
    Track,
    TrackPlayer,
    TrackRecorder,
    sanitize_track_name,
)
from .vision import ReflectivityDetector, target_to_odom
# 机械臂「抓取完成」信号桥接（独立包 arm_bridge/，仅在配置信号通道时启用）。
# 防御式导入：包缺失时机械臂等待自动关闭，不影响底盘/导航/云端既有功能。
try:
    from arm_bridge import ArmBridge, parse_arm_signal_spec
except ImportError:
    ArmBridge = None  # type: ignore[assignment,misc]
    parse_arm_signal_spec = None  # type: ignore[assignment]

    def _parse_arm_signal_spec_unavailable(spec: str) -> list:
        logging.getLogger(__name__).warning(
            "armfb: arm_bridge 包不可用，机械臂等待关闭")
        return []

    parse_arm_signal_spec = _parse_arm_signal_spec_unavailable  # type: ignore[assignment]

logger = logging.getLogger(__name__)


def _wrap_angle_rad(a: float) -> float:
    """Wrap radians to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _wrap_deg(a: float) -> float:
    """Wrap degrees to (-180, 180]."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


# ---------------------------------------------------------------------------
# Protocol constants (section 5 of the manual)
# ---------------------------------------------------------------------------

PING_INTERVAL_S: float = 2.0           # 应用层 ping（测云端往返，给断网保护用）
# websocket-client 协议层 ping：必须严格大于 ping_timeout，否则 run_forever
# 会直接抛 Ensure ping_interval > ping_timeout，WebSocket 线程起不来。
WS_PING_INTERVAL_S: float = 15.0
WS_PING_TIMEOUT_S: float = 10.0
SERVER_TIMEOUT_S: float = 45.0          # cloud declares offline after this
STATE_REPORT_INTERVAL_S: float = 1.5    # status push period (1-2 s)
LOC_TICK_S: float = 0.10                # 扫描匹配 / 伪地图 / 融合里程（建图前兜底）
NAV_STATE_PUSH_S: float = 0.40          # goto 期间加快位姿推送，网页地图才跟得上
OCC_MAP_SAVE_INTERVAL_S: float = 30.0   # 伪地图 PNG 落盘间隔
RECONNECT_BASE_S: float = 1.0           # exponential backoff base
RECONNECT_MAX_S: float = 30.0           # exponential backoff ceiling

# 云端点云实时流每帧推送的最大点数。提高可让云端图形查看器（vl/vm/vb）
# 更接近 robosense_airy 的观感（80000 点），但受 WebSocket 带宽与
# matplotlib 渲染帧率约束，8000 点是密度与流畅度的较好折中。
PC_STREAM_MAX_POINTS: int = 8000

# Safety clamp for remote `move` commands. The chassis accepts up to
# 1.3 m/s / 2.0 rad/s, but we cap remote velocity far below that so a
# buggy or malicious cloud client cannot drive the vehicle at full speed.
MAX_SAFE_LINEAR_M_S: float = 0.5
MAX_SAFE_ANGULAR_RAD_S: float = 1.0

# Remote `move` is a continuous velocity command.  A cloud operator who
# sends a single `move` (as the local mock_cloud does) must never leave the
# chassis running forever: if no new `move` arrives to refresh the command
# within this window, the agent auto-stops the chassis and clears the
# command.  A real cloud that keeps streaming moves every few seconds is
# unaffected.  This is the last line of defense against a stalled link /
# forgotten command / runaway client.
MOVE_KEEPALIVE_TIMEOUT_S: float = 5.0
MOVE_WATCHDOG_INTERVAL_S: float = 0.2
# kb / bypass_guard 遥控：单包 duration 上限。云端停键后若停车包丢失，
# 最多再走这么久，不能把 0.90s 保活当成继续冲的许可证。
TELEOP_DURATION_CAP_S: float = 0.35


class AgentState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    AUTHENTICATING = "authenticating"
    ONLINE = "online"


# ---------------------------------------------------------------------------
# Event types for immediate push (section 5, Step 5)
# ---------------------------------------------------------------------------

class EventType:
    LOW_BATTERY = "low_battery"
    OBSTACLE = "obstacle"
    ARRIVED = "arrived"
    FAULT = "fault"
    OFFLINE = "offline"
    FAILSAFE = "failsafe"


# ---------------------------------------------------------------------------
# Inbound command payload (after the common envelope)
# ---------------------------------------------------------------------------

@dataclass
class Command:
    action: str
    v: float = 0.0          # linear velocity m/s  (for move)
    w: float = 0.0          # angular velocity rad/s (for move)
    task_id: str = ""       # for task_submit
    from_: str = ""         # for task_submit
    to: str = ""           # for task_submit
    waypoints: list = field(default_factory=list)  # for task_submit
    name: str = ""          # for track_record
    track_id: str = ""      # for track_follow
    reverse: bool = False   # for track_follow: True = replay the track backwards (回程)
    duration: float = 0.0   # for move: keep this velocity for N seconds, then auto-stop (0 = rely on keep-alive watchdog)
    x: float = 0.0          # for goto: goal x (m, vehicle frame)
    y: float = 0.0          # for goto: goal y (m, vehicle frame)
    speed: float = 0.0      # for goto: 导航线速度上限 m/s（0 = 用默认 0.30）
    approach: bool = True   # for find_object: 默认进入底盘对接（对准/逼近/停稳）；False=只开到停靠点就返回
    hz: float = 0.0         # for pc_stream: 点云推送频率 Hz（>0 开流，<=0 关流）
    silent: bool = False    # for lidar_map: 云端轮询时置 True，避免控制台刷屏
    include_free: bool = False  # for lidar_map: 快照附带自由格坐标（地图模式渲染用）
    yaw_deg: float = 0.0    # pose_align / goto 终点航向（度）
    goal_yaw_set: bool = False  # True 当载荷带了 yawDeg（goto 用来区分「未指定」）
    ts_ms: int = 0          # envelope timestamp (ms); 0 = legacy cloud omitted it
    cells: list = field(default_factory=list)   # for map_upload: [[x, y, value], ...]
    resolution: float = 0.0  # for map_upload: 导入栅格分辨率 m（0=沿用现有）
    map_return: bool = False  # for map_return: 是否启用「地图优先返回」
    bypass_guard: bool = False  # kb 遥控：不过雷达守卫（m/goto/回放仍过）
    open_loop: bool = False  # 网页定时 move：不过雷达，且不受 kb 0.35s 封顶

    @classmethod
    def from_payload(cls, payload: dict) -> Command:
        if not isinstance(payload, dict):
            payload = {}

        def _to_float(value: Any, default: float = 0.0) -> float:
            try:
                f = float(value)
            except (TypeError, ValueError):
                f = default
            if f != f:  # NaN guard
                return default
            return f

        return cls(
            action=str(payload.get("action", "")),
            v=_to_float(payload.get("v")),
            w=_to_float(payload.get("w")),
            task_id=str(payload.get("taskId", "")),
            from_=str(payload.get("from", "")),
            to=str(payload.get("to", "")),
            waypoints=payload.get("waypoints", []) or [],
            name=str(payload.get("name", "")),
            track_id=str(payload.get("trackId", "")),
            reverse=bool(payload.get("reverse", False)),
            duration=_to_float(payload.get("duration")),
            x=_to_float(payload.get("x")),
            y=_to_float(payload.get("y")),
            speed=_to_float(payload.get("speed")),
            approach=bool(payload["approach"]) if "approach" in payload
                    else True,
            hz=_to_float(payload.get("hz")),
            silent=bool(payload.get("silent", False)),
            include_free=bool(payload.get("includeFree", False)),
            yaw_deg=_to_float(payload.get("yawDeg")),
            goal_yaw_set="yawDeg" in payload,
            cells=payload.get("cells", []) or [],
            resolution=_to_float(payload.get("resolution")),
            map_return=bool(payload.get("mapReturn", False)),
            bypass_guard=bool(payload.get("bypassGuard", False))
            or str(payload.get("source", "")).lower() in ("kb", "keyboard"),
            open_loop=bool(payload.get("openLoop", False)),
        )


# ---------------------------------------------------------------------------
# Car-side Agent
# ---------------------------------------------------------------------------

class BunkerMiniAgent:
    """WebSocket-to-CAN bridge agent for the BUNKER MINI 2.0.

    Usage::

        agent = BunkerMiniAgent(
            ws_url="wss://your-cloud-relay.example.com/ws",
            device_id="BUNKER-TEST01",
            bind_code="TEST-BIND-xxxx",
            # candleLight: interface="gs_usb", channel=None (auto-detect)
            #      or:    interface="candle", channel="0030001F4148570C20343133:0"
            # slcan:        interface="slcan", channel="COM3"
        )
        agent.run()   # blocking until SIGTERM / Ctrl+C
    """

    def __init__(
        self,
        ws_url: str,
        device_id: str,
        bind_code: str,
        channel: str | None = None,
        interface: str | None = None,
        *,
        track_dir: str | None = None,
        state_interval_s: float = STATE_REPORT_INTERVAL_S,
        ping_interval_s: float = PING_INTERVAL_S,
        enable_lidar: bool = True,
        lidar_host: str = "0.0.0.0",
        lidar_port: int = MSOP_PORT,
        lidar_pcap: str | None = None,
        lidar_mount_yaw_deg: float = 0.0,
        lidar_pitch_deg: float = 0.0,
        lidar_height_m: float = 0.0,
        lidar_self_mask: bool = True,
        wheelbase_m: float = 0.5,
        step_limit_m: float = 0.07,
        recon_max_duration_s: float = 120.0,
        recon_max_distance_m: float = 20.0,
        # --- 系统性搜索（初始 360° 扫描 + 周期性扫描停顿） ---
        initial_sweep: bool = True,
        scan_pause_every_s: float = 15.0,
        scan_pause_swing_deg: float = 180.0,
        # --- 建图适配（组员建图完成后接入，默认关闭不影响现有行为） ---
        pose_source: Optional[Callable[[], Optional[tuple[float, float, float]]]] = None,
        prefer_map_return: bool = False,
        # --- 动态姿态源（IMU）：不平路面实时校正点云水平基准 ---
        attitude_source: Optional[Callable[[], Optional[tuple[float, float]]]] = None,
        # --- 视觉伺服提前介入阈值（导航末段切视觉闭环） ---
        visual_handoff_m: float = 1.2,
        visual_handoff_deg: float = 15.0,
        # --- 轻量扫描匹配（本机漂移闭环，建图前兜底；pose_source 优先） ---
        enable_scan_match: bool = True,
        scan_match_blend: float = 0.35,
        # --- 机械臂抓取完成信号（arm_bridge/）：空 = 关闭（就位即返回） ---
        arm_grasp_signal: str = "",
        arm_wait_timeout_s: float = 90.0,
        # --- 实战无人值守：开机自启任务 + 拒绝遥控 ---
        auto_mission: str = "",
        auto_mission_target: str = "target",
        auto_mission_approach: bool = True,
        auto_mission_wait_lidar_s: float = 20.0,
        mission_lock: bool = False,
        local_mode: bool = False,
    ) -> None:
        self._local_mode = bool(local_mode)
        if not ws_url and not self._local_mode:
            raise ValueError("ws_url is required")
        if not device_id:
            raise ValueError("device_id is required")
        if not bind_code and not self._local_mode:
            raise ValueError("bind_code is required")

        self._ws_url = ws_url or "local://tcp"
        self._device_id = device_id
        self._bind_code = bind_code
        self._state_interval_s = state_interval_s
        self._ping_interval_s = ping_interval_s

        # CAN layer – resolved early so CLI errors surface immediately
        ch, iface = resolve_can_config(channel, interface, allow_auto_channel=True)
        self._can_channel = ch
        self._can_interface = iface
        # 用户显式指定通道时尊重选择，不再启动时探测替换；
        # 留空自动检测时，启动阶段会 probe「有底盘反馈」的通道。
        self._can_channel_explicit = bool(channel)

        # Runtime state — guarded by _lock
        self._lock = threading.Lock()
        self._state = AgentState.DISCONNECTED
        self._stop_event = threading.Event()
        self._ws: Optional[websocket.WebSocketApp] = None
        self._ws_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._state_thread: Optional[threading.Thread] = None
        self._loc_thread: Optional[threading.Thread] = None
        self._last_nav_state_push: float = 0.0
        self._occ_cell_count: int = -1

        self._controller: Optional[BunkerMiniController] = None
        self._session_id: int = 0  # incremented on each reconnect to guard stale callbacks
        self._token: str = ""
        # 最近一次连接错误（on_error 记录）。用于判断「服务器不在线」——
        # 连接被拒时云端可能随时重启，退避应保持短间隔以便快速恢复。
        self._last_conn_error: Optional[BaseException] = None

        # Tracker (trajectory record / replay)
        self._recorder: Optional[TrackRecorder] = None
        self._player: Optional[TrackPlayer] = None
        # Where recorded tracks are persisted (configurable for deployment)
        self._track_dir: str = track_dir or DEFAULT_TRACK_DIR
        # 网络延迟 / 断电保护（主线任务本地闭环，遥控走 TTL）
        self._failsafe = FailsafeMonitor(
            FailsafePolicy.from_env(),
            checkpoint_path=checkpoint_path_for(self._track_dir),
        )
        self._boot_failsafe_announced: bool = False

        # LiDAR avoidance & navigation (created in _init_lidar)
        self._enable_lidar = enable_lidar
        self._lidar_host = lidar_host
        self._lidar_port = lidar_port
        self._lidar_pcap = lidar_pcap
        self._lidar_mount_yaw_deg = lidar_mount_yaw_deg
        self._lidar_pitch_deg = lidar_pitch_deg
        self._lidar_height_m = lidar_height_m
        self._lidar_self_mask = lidar_self_mask
        self._wheelbase_m = wheelbase_m
        self._step_limit_m = step_limit_m
        self._lidar: Optional[AiryLidar] = None
        self._guard: Optional[ObstacleGuard] = None
        self._navigator: Optional[Navigator] = None
        # 2D 伪地图（occupancy grid，云端 goto 预检/伪地图绘制用）
        self._occ_grid: Optional[OccupancyGrid] = None
        self._occ_map_saved_at: float = 0.0
        # Matplotlib 俯视窗口（只读栅格/位姿；默认探测到 DISPLAY 才开）
        self._map_view = None
        self._map_view_attempted: bool = False
        # 全局 A* 路径规划器（基于伪地图，goto/find_object 沿规划航点导航）
        self._planner: Optional[GlobalPlanner] = None
        # 点云实时流：>0 表示开启，按该频率向云端推最新帧（独立线程驱动）
        self._pc_stream_hz: float = 0.0
        self._pc_stream_next_at: float = 0.0
        self._pc_stream_thread: Optional[threading.Thread] = None
        # 里程计接线只注册一次（lidar on/off 反复重建雷达时不重复叠加回调）
        self._odometer_wired: bool = False
        # Stage-3 vision: 目标检测（反射强度通路）与机械臂对接状态机
        self._detector: Optional[ReflectivityDetector] = None
        self._approach: Optional[ApproachController] = None
        # Stage-3 finale: 自主探路巡游控制器（搜索阶段驱动底盘移动）
        self._patrol: Optional[PatrolController] = None

        # Last sent command – used for continuous motion
        self._pending_command: Optional[Command] = None
        # Wall-clock of the last `move` — the watchdog auto-stops the
        # chassis if a move is not refreshed within MOVE_KEEPALIVE_TIMEOUT_S.
        self._last_move_at: float = 0.0
        # E-迁移（底盘诊断 hook）：掉回 STANDBY 自动重使能 + 指令/实测轮速
        # 对比诊断的去重时间戳。
        self._last_reenable_at: float = 0.0
        self._last_rc_recover_at: float = 0.0
        self._last_zero_warn_at: float = 0.0
        self._last_no_feedback_warn_at: float = 0.0
        # 周期重探通道去重时间戳：底盘反馈长时间缺失时，周期性尝试在其他
        # SocketCAN 网卡（如后出现的 can1）上找到底盘并热切换，避免 agent
        # 锁死在启动时探测失败的通道上（「底盘在 can1 但 agent 听 can0」）。
        self._last_can_reprobe_at: float = 0.0
        self._can_reprobe_interval_s: float = 30.0
        self._can_reprobe_lock = threading.Lock()
        self._can_probe_misses: int = 0
        self._last_gs_usb_reset_at: float = 0.0
        self._can_tx_armed: bool = False
        # Absolute deadline for a `move` with a duration (0/None = keep-alive
        # watchdog only).  The watchdog stops the chassis when now reaches it.
        self._move_deadline: Optional[float] = None
        self._teleop_inhibit_until: float = 0.0
        self._last_tcp_state_at: float = 0.0
        # TCP HID 100Hz 空闲发 (0,0)。长按中偶发一帧 0（Wi-Fi/HID 抖动）
        # 不能立刻 stop_motion，否则角速度要从静摩擦重新起步。
        self._stick_zero_n: int = 0
        self._watchdog_thread: Optional[threading.Thread] = None

        # Active transport task (task_submit, Step 5 任务流转).  A task is
        # "replay the recorded route referenced by trackId" and reports its
        # lifecycle via state.payload.task + task/arrived events.
        # Schema: {"task_id", "track_id", "status", "progress"}
        self._task: Optional[dict[str, Any]] = None

        # Event dedup – avoid spamming the same event repeatedly
        self._last_event: dict[str, float] = {}

        # find_object mission (stage-3): 搜索 → 检测 → 导航 → 对接 编排线程
        self._mission: Optional[dict] = None
        self._mission_thread: Optional[threading.Thread] = None
        self._mission_stop_event: Optional[threading.Event] = None
        self._search_angular_rad_s: float = 0.60   # 原地搜索旋转速度（无雷达时兜底）
        self._search_timeout_s: float = 20.0       # 原地搜索超时（无雷达时兜底）
        # 自主巡游搜索阶段参数
        self._recon_max_duration_s: float = recon_max_duration_s  # 探路最长时长（超时沿轨迹返回）
        self._recon_max_distance_m: float = recon_max_distance_m  # 探路最远直线距离（超距沿轨迹返回）
        # 系统性搜索：出发前 360° 扫描 + 周期扫描停顿
        self._initial_sweep_enabled = bool(initial_sweep)
        self._scan_pause_every_s = float(scan_pause_every_s)
        self._scan_pause_swing_deg = float(scan_pause_swing_deg)
        # 目标跟踪记忆：最后一次确认目标的世界系坐标（里程系）
        self._target_memory: Optional[tuple[float, float]] = None
        # 探路起点位姿（里程系），供返回前航向/位置对齐判定
        self._recon_start_pose: Optional[Pose2D] = None
        # 最近一次定格的探路轨迹：cancel 时 recon 可能已 stop() 录制，
        # 不能只看 recorder.is_recording，否则「探路中召回」会丢掉返航路径。
        self._recon_track: Optional[Track] = None
        # 任务链期间临时拉长伪地图 TTL，结束后恢复
        self._occ_ttl_saved: Optional[float] = None

        # 建图适配：外部位姿源（SLAM/视觉）与地图坐标系对齐（默认恒等）
        self._pose_source = pose_source
        self._map_alignment = MapAlignment()
        self._prefer_map_return = prefer_map_return
        # 动态姿态源（IMU）：不平路面实时校正点云水平基准（默认 None）
        self._attitude_source = attitude_source
        # 视觉伺服提前介入阈值：导航末段目标进入此距离+方位内 → 切视觉闭环
        self._visual_handoff_m = visual_handoff_m
        self._visual_handoff_deg = visual_handoff_deg
        # 轻量扫描匹配（本机漂移闭环）：在 _init_lidar 里创建
        self._scan_match_enabled = bool(enable_scan_match)
        self._scan_match_blend = float(scan_match_blend)
        self._scan_matcher: Optional["ScanMatcher"] = None
        self._scan_match_last_yaw: Optional[float] = None
        # 定位健康度（云端监控）：当前位姿来源 odom / scanmatch / external
        self._loc_source: str = "odom"
        # 最近一次定位修正统计（dx/dy/dyawDeg/at），供云端判断导航精度可信度
        self._loc_last_corr: Optional[dict] = None
        # 当前任务链的目标估计（结构化上报：方位/距离/半径/中心坐标），
        # 供云端验证视觉并在点云/地图视图叠加目标标记
        self._current_target: Optional[dict] = None

        # 机械臂抓取完成信号桥接（arm_bridge/）。默认未配置 → 关闭机械臂
        # 等待（保持旧行为：就位后立即返回）。配置后 find_object 在机械臂
        # 就位（ready）后停稳等待抓取完成信号再返回起点。
        self._arm_grasp_signal = arm_grasp_signal or ""
        self._auto_mission = (auto_mission or "").strip().lower()
        self._auto_mission_target = auto_mission_target or "target"
        self._auto_mission_approach = bool(auto_mission_approach)
        self._auto_mission_wait_lidar_s = max(1.0, float(auto_mission_wait_lidar_s))
        self._mission_lock = bool(mission_lock)
        self._last_lock_warn_at: float = 0.0
        self._teleop_tcp: Optional[TeleopTcpServer] = None
        self._auto_mission_thread: Optional[threading.Thread] = None
        self._arm_wait_timeout_s = float(arm_wait_timeout_s)
        self._arm_bridge: Optional[ArmBridge] = None
        try:
            self._arm_bridge = ArmBridge(
                parse_arm_signal_spec(self._arm_grasp_signal),
                wait_timeout_s=self._arm_wait_timeout_s,
            )
        except Exception:
            logger.exception("armfb: 机械臂信号桥接初始化失败，关闭机械臂等待")
            self._arm_bridge = None
        if self._arm_bridge is not None and self._arm_bridge.enabled:
            logger.info("armfb: 已启用机械臂抓取完成等待，通道=%s，超时=%.0fs",
                        [b.name for b in self._arm_bridge.backends],
                        self._arm_wait_timeout_s)
        elif arm_grasp_signal:
            logger.warning("armfb: 信号规格 %r 未解析出可用通道，机械臂等待关闭",
                           arm_grasp_signal)

        # Callbacks for external observation
        self._state_callbacks: list[Callable[[AgentState], None]] = []
        self._command_callbacks: list[Callable[[Command], None]] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def state(self) -> AgentState:
        with self._lock:
            return self._state

    def on_state_change(self, callback: Callable[[AgentState], None]) -> None:
        self._state_callbacks.append(callback)

    def on_command(self, callback: Callable[[Command], None]) -> None:
        self._command_callbacks.append(callback)

    def run(self) -> None:
        """Blocking entry point. Connects, authenticates, then loops forever."""
        try:
            self._init_can()
            self._init_lidar()
            self._restore_failsafe_checkpoint()
            self._spawn_auto_mission()
            if self._local_mode:
                self._serve_local()
            else:
                self._connect_and_serve()
        except KeyboardInterrupt:
            pass
        except CanConfigError as exc:
            # CAN 未配置/网卡 DOWN/适配器未插入：直接打印可执行的解决步骤，
            # 不再抛出原始 OSError traceback（"Network is down"）。
            logger.error("CAN 配置错误: %s", exc)
        except Exception:
            logger.exception("Agent fatal error")
        finally:
            self._cleanup()

    def stop(self) -> None:
        """Signal the agent to shut down gracefully (thread-safe)."""
        self._stop_event.set()
        self._close_ws()

    def _serve_local(self) -> None:
        """不连云端 WebSocket：CAN/雷达/任务在本机跑，指令走 teleop TCP。"""
        self._failsafe.reset_session()
        self._set_state(AgentState.ONLINE)
        self._announce_boot_failsafe()
        self._start_background_tasks()
        logger.info(
            "本地 TCP 模式已就绪：不连云端、不走 WebSocket。"
            "控制台: python3 run_local.py   遥操: python3 teleop_tcp_client.py"
        )
        while not self._stop_event.is_set():
            self._wait_with_map_view(0.2)
        self._stop_background_tasks(cloud_only=False)

    # ------------------------------------------------------------------
    # CAN initialization
    # ------------------------------------------------------------------

    def _init_can(self) -> None:
        # 1) 启动自动拉起所有已存在的 SocketCAN 网卡：USB-CAN 适配器插上但
        #    未 bringup（ip link set up）时，接口存在却是 DOWN，底盘永远
        #    收不到/发不出——这正是「CAN 初始化成功但小车不动」的常见原因。
        #    agent 通常以 root 运行，这里直接 ip link set up 无需 sudo。
        if self._can_interface == "socketcan":
            try:
                ensure_all_socketcan_up()
            except Exception:
                logger.debug("auto socketcan bring-up failed", exc_info=True)

        # listen-only 并行探测。USB-CAN 内核名不固定，按 sysfs + 底盘 RX 选定。
        if not self._can_channel_explicit:
            try:
                probed = probe_chassis_channel(
                    self._can_interface, timeout_s=5.0, listen_only=False)
            except Exception:
                logger.exception("[底盘诊断] CAN 通道探测异常，用默认通道")
                probed = None
            if probed and probed != self._can_channel:
                logger.warning(
                    "[底盘诊断] 探测到底盘反馈通道 %s（原默认 %s），切换之",
                    probed, self._can_channel,
                )
                self._can_channel = probed
            elif not probed:
                self._can_probe_misses += 1
                usb = []
                try:
                    usb = usb_socketcan_channels()
                except Exception:
                    usb = []
                if usb:
                    if usb[0] != self._can_channel:
                        logger.warning(
                            "[底盘诊断] 未听到底盘帧，保持 USB-CAN %s（原 %s）。"
                            "0x211 静默多半是底盘没上电，不再 usbreset",
                            usb[0], self._can_channel,
                        )
                        self._can_channel = usb[0]
                    else:
                        logger.warning(
                            "[底盘诊断] 启动探测未听到底盘帧（通道 %s）。"
                            "手册：底盘上电后 0x211 每 200ms 广播；"
                            "不软复位 USB-CAN，避免把还在的适配器打掉",
                            self._can_channel,
                        )
                else:
                    logger.warning(
                        "[底盘诊断] 没有 USB-CAN 网卡，不绑定板载 CAN。"
                        "未听到底盘帧时也不会往 Jetson mttcan 发 0x111。"
                        "请确认 candleLight 已插入后再启动 run_local"
                    )

        if self._can_interface == "socketcan":
            try:
                usb = usb_socketcan_channels()
            except Exception:
                usb = []
            flush_ch = (usb[0] if usb else None) or self._can_channel
            if flush_ch:
                logger.warning(
                    "[底盘诊断] 启动清一次 %s 发送邮箱"
                    "（gs_usb 堵死会跨进程残留，只重启 Python 清不掉）",
                    flush_ch,
                )
                try:
                    flush_socketcan_mailbox(flush_ch)
                except Exception:
                    logger.debug("启动清邮箱失败", exc_info=True)
            try:
                restore_tx_mode()
            except Exception:
                logger.debug("restore TX mode failed", exc_info=True)

        self._controller = BunkerMiniController(
            channel=self._can_channel,
            interface=self._can_interface,
            wheelbase_m=self._wheelbase_m,
        )
        self._controller.start()
        logger.info(
            "CAN layer initialized (interface=%s, channel=%s)",
            self._can_interface,
            self._can_channel,
        )
        # 先听 :9100，再等 0x211 / 切模式。否则 run_local 会空等十几秒，
        # 看起来像卡死，Ctrl+C 打在 _wait_port 上。
        self._start_teleop_tcp()

        # Wait for chassis feedback
        for _ in range(200):
            if self._controller.latest_status is not None:
                break
            time.sleep(0.05)
        else:
            logger.warning("尚未收到 0x211；运动帧保持静默，周期重探通道")

        # 未听到 0x211 时不要发 0x421：无 ACK 会立刻 ERROR-PASSIVE，把探测窗口毁掉。
        if self._controller.latest_status is None:
            logger.info("未听到底盘帧，空闲不发 0x421，避免堵死 USB-CAN 发送队列")
        else:
            self._arm_can_tx_if_heard()
            logger.info("按手册接管底盘：清失联锁 (0x441) 并保持 CAN 指令模式 (0x421)...")
            switched = self._controller.ensure_can_mode(timeout_s=2.0)
            st = self._controller.latest_status
            mode = getattr(st, "control_mode", None) if st is not None else None
            fault = getattr(st, "fault_code", 0) if st is not None else 0
            fault_other = int(fault or 0) & ~0x04
            rc_on = bool(getattr(self._controller, "remote_alive", False))
            if switched and not fault_other:
                logger.info("控制模式已切到 CAN 指令模式")
            elif switched:
                logger.warning(
                    "[底盘诊断] 已切到指令模式但仍有故障码 0x%02X —— 可能阻断运动。"
                    "检查急停、电池与驱动通讯。",
                    fault_other,
                )
            elif rc_on:
                logger.warning(
                    "[底盘诊断] 遥控器仍在线且未拨到指令档（SWB 上），"
                    "底盘保持遥控模式。请关机遥控器，或把 SWB 拨到最上。",
                )
            else:
                logger.warning(
                    "[底盘诊断] 遥控器已关但控制模式仍为 %s。"
                    "代理会持续发 0x421 并清 0x04，无需打开遥控器。",
                    mode,
                )

        # Initialize tracker tools.  The player's velocity_guard wires LiDAR
        # obstacle avoidance into track replay / task replay (aborts playback
        # on a hard stop).
        #
        # A-闭环纠偏：回放时用同一差速模型对比期望/实际位姿，逐周期修正
        # 角速度（只纠方向）；C-末端停靠：收尾低速爬行 + 停稳确认。
        self._recorder = TrackRecorder(
            self._controller,
            track_dir=self._track_dir,
            wheelbase_m=self._wheelbase_m,
            drive_mode="kb",
        )
        self._player = TrackPlayer(
            self._controller,
            track_dir=self._track_dir,
            velocity_guard=self._velocity_guard if self._enable_lidar else None,
            wheelbase_m=self._wheelbase_m,
            correction=PlaybackCorrectionConfig(wheelbase_m=self._wheelbase_m),
            dock=PlaybackDockConfig(),
            drive=self._play_drive,
        )

    def _start_teleop_tcp(self) -> None:
        """局域网 TCP 遥操：HID/move/estop/query 不走 SSH 和 WebSocket JSON。"""
        if os.environ.get("BUNKER_TELEOP_TCP", "1").lower() in ("0", "false", "no"):
            logger.info("teleop TCP 已关闭（BUNKER_TELEOP_TCP=0）")
            return
        host = os.environ.get("BUNKER_TELEOP_HOST", "0.0.0.0")
        port = int(os.environ.get("BUNKER_TELEOP_PORT", "9100"))
        token = os.environ.get("BUNKER_TELEOP_TOKEN", "bunker-teleop")
        try:
            srv = TeleopTcpServer(
                host=host,
                port=port,
                token=token,
                on_stick=self._teleop_stick,
                on_estop=self._teleop_estop,
                on_idle_stop=self._teleop_idle_stop,
                on_move=self._teleop_move,
                on_goto=self._teleop_goto,
                on_query=self._teleop_query,
                on_cmd=self._teleop_cmd,
                on_rx=self._failsafe.note_rx if self._local_mode else None,
            )
            srv.start()
            self._teleop_tcp = srv
            logger.info(
                "teleop TCP 已监听 %s:%s（笔记本本机: python teleop_from_laptop.py --host <工控机IP>）",
                host, port,
            )
        except OSError as exc:
            logger.warning("teleop TCP 端口 %s:%s 未拉起: %s", host, port, exc)

    def _teleop_stick(self, v: float, w: float) -> None:
        if self._combat_lock_blocks("move"):
            now = time.time()
            if now - getattr(self, "_last_lock_warn_at", 0.0) >= 2.0:
                logger.warning("实战锁：忽略 TCP stick（本地控制台请用 run_local，勿读 mission.conf 的 LOCK）")
                self._last_lock_warn_at = now
            return
        ctrl = self._controller
        if ctrl is None:
            return
        if abs(v) <= 1e-6 and abs(w) <= 1e-6:
            # 网页 / 笔记本 HID 100Hz 空闲也发 (0,0)。这是保活，不是停车。
            # 不能清掉定时 move，更不能 stop_motion 掐掉 goto/探路/回放。
            if self._has_local_autonomy() or self._timed_move_active():
                return
            # 长按 A/D 时 HID 偶发一帧 0：立刻停再起步就是角速度卡顿。
            # 连续 2 帧（约 20ms @ 100Hz）才当真松开。
            self._stick_zero_n += 1
            if self._stick_zero_n < 2:
                return
            with self._lock:
                self._pending_command = None
                self._move_deadline = None
            ctrl.stop_motion()
            return
        self._stick_zero_n = 0
        # 真的按了 WASD：操作员接管，取消车侧自主。
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        with self._lock:
            self._last_move_at = time.time()
            # stick 不是带 duration 的 move：禁止 +0.20s 截止，否则松键后还会冲
            self._move_deadline = None
            self._pending_command = None
        self._drive(v, w, apply_guard=False, bypass_ramp=True)

    def _timed_move_active(self) -> bool:
        with self._lock:
            pending = self._pending_command
            deadline = self._move_deadline
            last = self._last_move_at
        if pending is None or pending.action != "move":
            return False
        now = time.time()
        if deadline is not None:
            return now < deadline
        return last > 0.0 and (now - last) < MOVE_KEEPALIVE_TIMEOUT_S

    def _drive_status(self) -> dict[str, Any]:
        lidar = self._lidar
        out: dict[str, Any] = {
            "kind": "idle",
            "lidarEnabled": lidar is not None,
            "lidarReady": bool(lidar is not None and lidar.is_receiving),
        }
        nav = self._navigator
        if nav is not None and getattr(nav, "is_navigating", False):
            goal = getattr(nav, "goal", None)
            pose = getattr(nav, "pose", None)
            dist = None
            if goal is not None and pose is not None:
                dist = round(math.hypot(goal[0] - pose.x, goal[1] - pose.y), 3)
            out["kind"] = "goto"
            if goal is not None:
                out["goto"] = {
                    "x": round(float(goal[0]), 3),
                    "y": round(float(goal[1]), 3),
                    "distM": dist,
                }
            return out
        thread = self._mission_thread
        if thread is not None and thread.is_alive():
            out["kind"] = "mission"
            return out
        player = self._player
        if player is not None and getattr(player, "is_playing", False):
            out["kind"] = "replay"
            return out
        with self._lock:
            task = self._task
            pending = self._pending_command
            deadline = self._move_deadline
            last = self._last_move_at
        if isinstance(task, dict) and task.get("status") == "running":
            out["kind"] = "mission"
            return out
        if pending is not None and pending.action == "move":
            remain = None
            now = time.time()
            if deadline is not None:
                remain = round(max(0.0, deadline - now), 2)
            elif last > 0.0:
                remain = round(max(0.0, MOVE_KEEPALIVE_TIMEOUT_S - (now - last)), 2)
            out["kind"] = "move"
            out["move"] = {
                "v": round(float(pending.v), 3),
                "w": round(float(pending.w), 3),
                "remainS": remain,
                "durationS": round(float(pending.duration or 0.0), 2) or None,
                "openLoop": bool(getattr(pending, "open_loop", False)),
            }
        return out

    def _teleop_idle_stop(self) -> None:
        if self._has_local_autonomy() or self._timed_move_active():
            return
        ctrl = self._controller
        if ctrl is None:
            return
        with self._lock:
            self._pending_command = None
            self._move_deadline = None
        ctrl.stop_motion()

    def _teleop_estop(self) -> None:
        self._handle_command(Command(action="estop"))

    def _teleop_move(self, v: float, w: float, duration: float, bypass: bool) -> None:
        if self._combat_lock_blocks("move"):
            return
        self._handle_command(Command(
            action="move", v=v, w=w, duration=duration, bypass_guard=bypass,
        ))

    def _teleop_goto(self, x: float, y: float, speed: float) -> None:
        if self._combat_lock_blocks("goto"):
            return
        self._handle_command(Command(action="goto", x=x, y=y, speed=speed))

    def _teleop_query(self) -> tuple[float, float]:
        ctrl = self._controller
        if ctrl is None or ctrl.latest_motion is None:
            return 0.0, 0.0
        fb = ctrl.latest_motion
        return fb.linear_velocity_m_s, fb.angular_velocity_rad_s

    def _teleop_cmd(self, payload: dict) -> None:
        """本地 TCP 控制台下发的云端同语义指令（不经 WebSocket）。"""
        if not isinstance(payload, dict):
            return
        action = str(payload.get("action") or "")
        if action == "map_upload" and payload.get("file") and not payload.get("cells"):
            path = str(payload["file"])
            try:
                with open(path, "r", encoding="utf-8") as fp:
                    doc = json.load(fp)
            except Exception as exc:
                logger.error("map_upload 读文件失败: %s", exc)
                self._send_event("fault", f"map_upload: 读文件失败 {exc}")
                return
            payload = dict(payload)
            payload["cells"] = doc.get("cells") or []
            if doc.get("resolution"):
                try:
                    payload["resolution"] = float(doc["resolution"])
                except (TypeError, ValueError):
                    pass
        cmd = Command.from_payload(payload)
        try:
            cmd.ts_ms = int(payload.get("ts") or time.time() * 1000)
        except (TypeError, ValueError):
            cmd.ts_ms = int(time.time() * 1000)
        if self._local_mode:
            self._failsafe.note_rx()
        verdict = self._failsafe.judge(cmd.action, cmd.ts_ms or None)
        if not verdict.accept:
            self._send_event(
                EventType.FAILSAFE,
                f"拒绝过期/劣质链路指令 {cmd.action}（{verdict.reason}）",
            )
            return
        if cmd.action == "move" and cmd.bypass_guard:
            logger.debug("TCP kb move: v=%.2f w=%.2f", cmd.v, cmd.w)
        else:
            logger.info("TCP cmd: action=%s", cmd.action)
        self._handle_command(cmd)

    def _play_drive(self, v: float, w: float) -> None:
        """回放专用下发：诊断/重使能走 _drive，但不再套第二层速度斜坡。

        kb 录制采到的是底盘已经斜坡之后的 0x221；回放若再走 set_velocity，
        点按和转弯会被抹成更慢的爬升，贴线变差。

        已录轨迹 / B 回程是固定轮迹：不过雷达守卫（``apply_guard=False``）。
        探路返回等未知环境回放仍过守卫。
        """
        if abs(v) <= 1e-6 and abs(w) <= 1e-6:
            ctrl = self._controller
            if ctrl is not None:
                ctrl.stop_motion()
            return
        player = self._player
        skip_guard = bool(player is not None and player.bypass_guard)
        self._drive(v, w, bypass_ramp=True, apply_guard=not skip_guard)

    def _drive(self, v: float, w: float, *, apply_guard: bool = True,
               bypass_ramp: bool = False) -> None:
        """Central chassis velocity dispatch with diagnostics (E 迁移).

        所有底盘指令统一经此下发：
          1. 运行中控制模式掉回 STANDBY → 自动重发使能（0.5 s 去重），
             避免「指令下发但底盘不动」的隐性故障。
          2. **遥控模式（REMOTE_CONTROL）下也尝试重使能**：遥控器开着时
             拥有最高权限，会阻断 0x111 指令；但只要用户把遥控器 SWB 拨到
             「指令」档，0x421 模式设置就会生效，回放/B 键回程立即能驱动。
             这是「遥控模式录制后按 B / fb 不动」的根因修复。
          3. 下发后对比实测轮速：指令明显前进但实测≈0 → 周期性告警
             （检查急停 / 遥控档位 / 履带着地 / 电池）。
        """
        ctrl = self._controller
        if ctrl is None:
            return
        now = time.time()

        # Last-line obstacle safety: every chassis command (kb, recon sweep,
        # goto detour/backup, approach, replay) must pass the guard.  A hard
        # stop uses stop_motion() — set_velocity(0,0) would still ramp and
        # coast into the obstacle.
        guard = self._guard
        if apply_guard and guard is not None:
            v, w, blocked = guard.guard_velocity(v, w)
            if blocked:
                try:
                    ctrl.stop_motion()
                except Exception:
                    logger.exception("Obstacle hard-stop failed")
                return

        st = getattr(ctrl, "latest_status", None)
        mode = getattr(st, "control_mode", None) if st is not None else None
        if st is None:
            onboard = (
                self._can_interface == "socketcan"
                and not self._can_channel_explicit
                and not _is_usb_netdev(self._can_channel)
            )
            if now - self._last_no_feedback_warn_at >= 5.0:
                if onboard:
                    logger.warning(
                        "[底盘诊断] 当前通道 %s 是板载 CAN 且未收到 0x211，"
                        "拒绝发送 0x111，避免误打 Jetson mttcan",
                        self._can_channel,
                    )
                else:
                    logger.warning(
                        "[底盘诊断] 尚未收到 0x211，通道 %s 状态 %s；"
                        "运动帧保持静默，等底盘广播后再发，以免堵 USB-CAN。",
                        self._can_channel,
                        socketcan_operstate(self._can_channel),
                    )
                self._last_no_feedback_warn_at = now
            if onboard:
                return
        elif mode is not None and mode != ControlMode.CAN_COMMAND:
            if now - self._last_reenable_at >= 0.5:
                reason = (
                    "掉回 STANDBY" if mode == ControlMode.STANDBY
                    else "处于遥控模式（遥控器开着），尝试切换"
                )
                logger.warning(
                    "[底盘诊断] Control Mode %s，重新 enable_can_control()",
                    reason,
                )
                try:
                    if int(getattr(st, "fault_code", 0) or 0) & 0x04:
                        ctrl.clear_faults()
                    ctrl.enable_can_control()
                except Exception:
                    logger.exception("[底盘诊断] 重使能失败")
                self._last_reenable_at = now

        ctrl.set_velocity_now(v, w) if bypass_ramp else ctrl.set_velocity(v, w)

        mot = getattr(ctrl, "latest_motion", None)
        if (
            mot is not None
            and abs(v) >= 0.05
            and abs(getattr(mot, "linear_velocity_m_s", 1.0)) < 0.02
            and now - self._last_zero_warn_at >= 2.0
        ):
            mode_hint = (
                "遥控器 SWB 未拨到指令档（当前遥控模式），" if mode == ControlMode.REMOTE_CONTROL else ""
            )
            logger.warning(
                "[底盘诊断] ⚠ 已发送前进指令但实测轮速≈0：%s"
                "检查急停、履带是否着地、电池与故障码。",
                mode_hint,
            )
            self._last_zero_warn_at = now

    def _maybe_recover_can_mode(self, status: Any) -> None:
        """遥控器已关却仍报遥控模式：让控制器保持 0x421（TX 环会清幽灵锁）。"""
        ctrl = self._controller
        if ctrl is None or status is None:
            return
        if getattr(status, "control_mode", None) == ControlMode.CAN_COMMAND:
            return
        if getattr(ctrl, "remote_alive", False) and not getattr(
            ctrl, "remote_swb_command", False,
        ):
            return
        now = time.time()
        if now - self._last_rc_recover_at < 5.0:
            return
        self._last_rc_recover_at = now
        try:
            ctrl.enable_can_control()
            logger.info(
                "底盘恢复：遥控器已关，正在保持 CAN 指令模式（0x421）",
            )
        except Exception:
            logger.exception("[底盘诊断] 遥控失联恢复失败")

    def _arm_can_tx_if_heard(self) -> None:
        """First 0x211: leave listen-only so motion frames can actually leave."""
        if self._can_tx_armed:
            return
        ctrl = self._controller
        if ctrl is None or ctrl.latest_status is None:
            return
        ch = self._can_channel
        try:
            if self._can_interface == "socketcan" and socketcan_is_listen_only(ch):
                logger.info("听到 0x211，退出 listen-only 后开始发送")
                leave_listen_only(ch)
                rebuild = getattr(ctrl, "rebuild_bus", None)
                if callable(rebuild):
                    rebuild()
        except Exception:
            logger.debug("退出 listen-only 失败", exc_info=True)
        try:
            ctrl.ensure_can_mode(timeout_s=2.0)
        except Exception:
            logger.debug("ensure_can_mode 失败", exc_info=True)
        self._can_tx_armed = True

    def _maybe_reprobe_can_channel(self, now: float) -> None:
        """listen-only 重探：不再先 down/up 再探测（那会打掉 RX 窗口）。"""
        if self._can_channel_explicit:
            return
        if now - self._last_can_reprobe_at < self._can_reprobe_interval_s:
            return
        if not self._can_reprobe_lock.acquire(blocking=False):
            return
        self._last_can_reprobe_at = now
        ctrl = self._controller
        if ctrl is None:
            self._can_reprobe_lock.release()
            return
        try:
            self._reprobe_can_channel_locked(ctrl)
        finally:
            self._can_reprobe_lock.release()

    def _reprobe_can_channel_locked(self, ctrl: Any) -> None:
        """只听其它口。当前 TX 口已经有 RX 线程，再开套接字会把发送队列打爆。"""
        try:
            others = [
                ch for ch in list_socketcan_interfaces()
                if ch != self._can_channel
            ]
        except Exception:
            others = []
        probed = None
        try:
            probed = probe_chassis_channel(
                self._can_interface,
                candidates=others,
                timeout_s=2.0 if others else 0.2,
                listen_only=False,
            )
        except Exception:
            logger.debug("CAN 通道重探异常", exc_info=True)
            probed = None
        if probed and probed != self._can_channel:
            self._can_probe_misses = 0
            logger.warning(
                "[底盘诊断] 重探发现底盘在通道 %s（当前 %s）——热切换到 %s",
                probed, self._can_channel, probed,
            )
            old_channel = self._can_channel
            try:
                ctrl.switch_channel(probed, self._can_interface)
                self._can_channel = probed
                logger.info("已热切换 CAN 通道 %s → %s", old_channel, probed)
            except Exception:
                logger.exception(
                    "[底盘诊断] 热切换到 %s 失败，保持 %s", probed, old_channel,
                )
            return
        self._can_probe_misses += 1
        usb: list[str] = []
        try:
            usb = usb_socketcan_channels()
        except Exception:
            usb = []
        prefer = usb[0] if usb else None
        if prefer and prefer != self._can_channel:
            logger.warning(
                "[底盘诊断] 重探未听到底盘帧，切到 USB-CAN %s（当前 %s）",
                prefer, self._can_channel,
            )
            old_channel = self._can_channel
            try:
                ctrl.switch_channel(prefer, self._can_interface)
                self._can_channel = prefer
            except Exception:
                logger.exception(
                    "[底盘诊断] 热切换到 %s 失败，保持 %s", prefer, old_channel,
                )

    def _init_lidar(self) -> None:
        """Initial LiDAR startup (called from run())."""
        if not self._enable_lidar:
            logger.info("LiDAR disabled (enable_lidar=False) — no obstacle avoidance")
            return
        self._start_lidar()

    def _build_lidar(self):
        """Construct the LiDAR source (UDP or PCAP replay) per current config."""
        if self._lidar_pcap:
            return PcapReplaySource(
                self._lidar_pcap,
                mount_yaw_deg=self._lidar_mount_yaw_deg,
                pitch_deg=self._lidar_pitch_deg,
                lidar_height_m=self._lidar_height_m,
                self_mask=SelfMaskConfig(enabled=self._lidar_self_mask),
            )
        return AiryLidar(
            host=self._lidar_host,
            port=self._lidar_port,
            mount_yaw_deg=self._lidar_mount_yaw_deg,
            pitch_deg=self._lidar_pitch_deg,
            lidar_height_m=self._lidar_height_m,
            self_mask=SelfMaskConfig(enabled=self._lidar_self_mask),
            attitude_source=self._attitude_source,
        )

    def _start_lidar(self) -> bool:
        """(Re)create and start the LiDAR stack. Returns True when online.

        幂等：雷达已在跑且在线 → 直接返回 True，不重建。
        ``lidar on`` 云端命令也走这里：即便启动时 ``--no-lidar`` 关闭，
        操作员也能在运行中把雷达开起来。AiryLidar.stop() 会关闭 UDP
        socket、无法原地重启，因此掉线后采用「拆除 → 重建」。
        """
        existing = self._lidar
        if existing is not None and existing.is_receiving:
            return True
        # 保留里程计位姿连续性：navigator 复用，只换 guard（不重建位姿）
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        if existing is not None:
            self._teardown_lidar_source()
        try:
            lidar = self._build_lidar()
            lidar.start()
        except (LidarError, PcapError) as exc:
            logger.warning("%s — 避障降级为无传感器模式", exc)
            self._send_event("fault", f"雷达启动失败: {exc}")
            return False

        self._lidar = lidar
        # fail-safe：雷达在栈中即 require_sensor=True —— 中途掉线立即停车并
        # 上报 obstacle 事件；只有 lidar off / --no-lidar 才用透传守卫。
        self._guard = ObstacleGuard(
            lidar,
            ObstaclePolicy(step_limit_m=self._step_limit_m),
            require_sensor=True,
        )
        self._guard.on_blocked(self._on_obstacle_blocked)
        self._detector = ReflectivityDetector(mount_yaw_deg=self._lidar_mount_yaw_deg)
        self._approach = ApproachController(ApproachConfig())
        # 轻量扫描匹配（本机漂移闭环）：建图前界住里程漂移，pose_source
        # 接入绝对位姿后自动退居兜底。
        self._scan_matcher = (
            ScanMatcher(config=ScanMatchConfig())
            if self._scan_match_enabled else None
        )
        self._scan_match_last_yaw = None
        # 伪地图与全局规划器先建好，供 patrol 探索记忆 / goto 规划复用
        self._occ_grid = OccupancyGrid()
        # 建图模式：关闭 5s TTL，走过的区域持续累计进同一张栅格
        self._occ_grid.set_ttl(0)
        self._planner = GlobalPlanner(self._occ_grid)
        # 导航器必须先于 patrol 创建：探路记忆 / frontier 依赖 pose_fn。
        # 旧实现在首次启动时 nav 仍为 None，pose_fn 被设成永久空 → 探索记忆失效。
        if nav is not None:
            nav.set_guard(self._guard)
            self._navigator = nav
        else:
            nav_cfg = None
            if self._auto_mission:
                # 建图未完成：未知溶洞里用更慢的导航，把里程漂移和时间留给雷达
                nav_cfg = NavigateConfig(max_linear_m_s=0.20)
            self._navigator = Navigator(
                self._controller,
                self._guard,
                wheelbase_m=self._wheelbase_m,
                config=nav_cfg,
                drive=self._drive,
            )
            if not self._odometer_wired:
                # Feed odometry into the dead-reckoned pose estimate (once)
                self._controller.on_odometer(self._on_odometer_feedback)
                self._odometer_wired = True
        patrol_cfg = None
        if self._auto_mission:
            patrol_cfg = PatrolConfig(max_linear_m_s=0.18)
        self._patrol = PatrolController(
            self._controller,
            self._guard,
            lidar,
            config=patrol_cfg,
            pose_fn=self._patrol_pose,
            explore_grid=self._occ_grid,
            frontier_provider=self._patrol_frontier_target,
        )
        logger.info(
            "LiDAR + obstacle guard + navigator initialized "
            "(mount_yaw=%.1f°, wheelbase=%.3f m)",
            self._lidar_mount_yaw_deg,
            self._wheelbase_m,
        )
        self._maybe_start_map_view()
        return True

    def _maybe_start_map_view(self) -> None:
        """雷达栈就绪后在主线程非阻塞开 Matplotlib 窗（只读，不影响建图/WS）。"""
        if self._map_view is not None or self._map_view_attempted:
            return
        try:
            from .map_view import OccupancyMapView, map_view_should_start
        except Exception:
            logger.debug("map_view import failed", exc_info=True)
            return
        if not map_view_should_start():
            return
        self._map_view_attempted = True
        view = OccupancyMapView(
            grid_fn=lambda: self._occ_grid,
            pose_fn=self._map_view_pose,
            sectors_fn=self._map_view_sectors,
        )
        if view.start():
            self._map_view = view
            logger.info("Occupancy Matplotlib view started")
        else:
            logger.info("Occupancy Matplotlib view skipped: %s", view.last_error)

    def _pump_map_view(self) -> None:
        """主线程泵一拍地图窗口；无窗口则为空操作。"""
        view = self._map_view
        if view is None:
            return
        try:
            view.pump()
        except Exception:
            logger.debug("map view pump failed", exc_info=True)

    def _wait_with_map_view(self, timeout_s: float) -> None:
        """主线程等待，同时泵地图（超时/stop 语义与 sleep 相同）。"""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while not self._stop_event.is_set():
            self._pump_map_view()
            remain = deadline - time.monotonic()
            if remain <= 0.0:
                return
            self._stop_event.wait(timeout=min(0.05, remain))

    def _map_view_pose(self):
        """可视化只读：当前里程系 (x, y, yaw_deg)。"""
        nav = self._navigator
        if nav is None:
            return None
        p = nav.pose
        return (p.x, p.y, p.yaw_deg)

    def _map_view_sectors(self):
        """可视化只读：当前扇区极坐标，由 map_view 按 OccupancyGrid 公式投世界系。"""
        lidar = self._lidar
        if lidar is None:
            return []
        try:
            return lidar.sector_points()
        except Exception:
            return []

    def _patrol_pose(self):
        """Patrol 探索记忆用的当前里程系位姿；导航器未就绪时抛给调用方吞掉。"""
        nav = self._navigator
        if nav is None:
            raise RuntimeError("navigator unavailable")
        return nav.pose

    def _wait_lidar_online(self, timeout_s: float = 5.0) -> bool:
        """等待雷达收到首帧数据（启动后约一个帧周期才置在线）。

        直接判 ``is_receiving`` 会因首帧未到而误报失败；这里轮询等待，
        超时仍未收到帧 → 判离线。默认 5s 给雷达慢启动留余量
        （Airy 开机建链/出点云需要时间）。
        """
        deadline = time.monotonic() + timeout_s
        lidar = self._lidar
        while lidar is not None and time.monotonic() < deadline:
            if lidar.is_receiving:
                return True
            time.sleep(0.05)
        return lidar is not None and lidar.is_receiving

    def _combat_lock_blocks(self, action: str) -> bool:
        """实战锁：任务自己跑的时候，拒绝会抢方向盘的云端/键盘指令。

        急停、召回、查询、机械臂完成信号仍放行。内部 ``_handle_find_object``
        不走这条（自动开任务不是遥控）。
        本地 TCP 控制台（``--local`` / run_local.py）就是来开车的，不套这把锁。
        """
        if self._local_mode:
            return False
        if not self._mission_lock:
            return False
        allowed = {
            "estop", "cancel", "query", "grasp_done",
            "lidar_status", "lidar_map", "point_cloud", "pc_stream",
        }
        return action not in allowed

    def _spawn_auto_mission(self) -> None:
        """雷达起来后自动开 find_object，不等云端键盘。"""
        if self._auto_mission not in ("find_object", "fo", "fobj"):
            return
        if self._mission_lock:
            logger.info("实战锁已开启：任务期间忽略键盘/move 遥控")
        t = threading.Thread(
            target=self._auto_mission_loop,
            name="auto-mission",
            daemon=True,
        )
        self._auto_mission_thread = t
        t.start()

    def _auto_mission_loop(self) -> None:
        name = self._auto_mission_target
        logger.info(
            "自动任务：等待雷达在线（最长 %.0f s）后开始搜寻 '%s'",
            self._auto_mission_wait_lidar_s, name,
        )
        if not self._wait_lidar_online(self._auto_mission_wait_lidar_s):
            logger.error("自动任务失败：雷达未在线，不会自行开车")
            self._send_event("fault", "自动任务失败：雷达未在线")
            return
        if self._stop_event.is_set():
            return
        logger.info("自动任务：雷达已在线，开始 find_object '%s'", name)
        self._handle_find_object(Command(
            action="find_object",
            name=name,
            approach=self._auto_mission_approach,
        ))

    def _teardown_lidar_source(self) -> None:
        """Stop the LiDAR source and release the perception stack (lidar off).

        导航位姿（navigator）与里程计接线保留，保证再开雷达后 goto 坐标系
        不跳变；guard 保留（指向已停的雷达 → 无传感器降级模式）。
        """
        lidar = self._lidar
        self._lidar = None
        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                logger.exception("LiDAR stop error")
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        # 关闭雷达 = 显式降级为无传感器模式：换透传守卫（require_sensor=False），
        # 否则上一步创建的 fail-safe 守卫会把所有运动通路锁死。
        self._guard = ObstacleGuard(
            None,
            ObstaclePolicy(step_limit_m=self._step_limit_m),
            require_sensor=False,
        )
        if nav is not None:
            nav.set_guard(self._guard)
        self._patrol = None
        self._detector = None
        self._approach = None
        self._scan_matcher = None
        self._scan_match_last_yaw = None
        self._occ_grid = None
        self._planner = None

    # ------------------------------------------------------------------
    # WebSocket lifecycle
    # ------------------------------------------------------------------

    def _connect_and_serve(self) -> None:
        reconnect_count = 0
        # 记录本次进程生命周期内是否曾成功上线：区分「服务器不在线」
        # （从未连上 → 固定短间隔探测）与「曾经在线后断开」（指数退避）。
        ever_online = False

        while not self._stop_event.is_set():
            self._session_id += 1
            session = self._session_id

            self._set_state(AgentState.CONNECTING)
            logger.info("Connecting to %s (session %d) ...", self._ws_url, session)

            # Capture session in closures to guard against stale callbacks
            def on_open(ws: websocket.WebSocketApp) -> None:
                self._on_open(ws, session)

            def on_message(ws: websocket.WebSocketApp, raw: str) -> None:
                self._on_message(ws, raw, session)

            def on_error(ws: websocket.WebSocketApp, error: Any) -> None:
                self._on_error(ws, error)

            def on_close(ws: websocket.WebSocketApp, code: int, msg: str) -> None:
                self._on_close(ws, code, msg, session)

            ws_app = websocket.WebSocketApp(
                self._ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close,
            )
            with self._lock:
                self._ws = ws_app

            self._ws_thread = threading.Thread(
                target=self._ws.run_forever,
                kwargs={
                    "ping_interval": WS_PING_INTERVAL_S,
                    "ping_timeout": WS_PING_TIMEOUT_S,
                },
                name="ws-run",
                daemon=True,
            )
            self._ws_thread.start()

            # Wait for auth / ONLINE
            for _ in range(100):
                with self._lock:
                    state = self._state
                if state in (AgentState.ONLINE, AgentState.DISCONNECTED):
                    break
                self._wait_with_map_view(0.1)

            if state == AgentState.ONLINE:
                # Reset backoff so a future transient drop restarts at 1s
                # instead of inheriting a long delay from past disconnects.
                reconnect_count = 0
                ever_online = True
                self._last_conn_error = None
                self._start_background_tasks()
                # 主线程不再空 join：泵 Matplotlib；WS 仍在 ws-run 线程。
                while self._ws_thread.is_alive() and not self._stop_event.is_set():
                    self._pump_map_view()
                    self._ws_thread.join(timeout=0.05)
                # 心跳依赖当前 WS；定位/伪地图/看门狗在任务期间必须继续跑，
                # 不能因为 mock_cloud 抖动把本地闭环一起拆掉。
                self._stop_background_tasks(cloud_only=True)
            elif state == AgentState.DISCONNECTED:
                # 判断「服务器不在线」：连接被拒 / 解析失败等（on_error 记录）。
                # 此时云端可能随时重启，退避保持短间隔，避免云端重启后 agent
                # 仍卡在 30s 长退避里 → 表现为「云端显示没检测到车侧代理」。
                reconnect_count = self._next_reconnect_count(
                    reconnect_count, ever_online)
            else:
                # Still authenticating but ws_thread died
                reconnect_count = 1

            # Exponential backoff
            delay = min(RECONNECT_BASE_S * (2 ** (reconnect_count - 1)), RECONNECT_MAX_S)
            if not self._stop_event.is_set():
                logger.info("Reconnecting in %.1f s (attempt %d)...", delay, reconnect_count)
                self._wait_with_map_view(delay)
        # end while

    def _on_open(self, ws: websocket.WebSocketApp, session: int) -> None:
        if session != self._session_id:
            return
        logger.info("WebSocket connected (session %d), sending auth...", session)
        self._set_state(AgentState.AUTHENTICATING)
        auth_msg = {
            "type": "auth",
            "deviceId": self._device_id,
            "ts": int(time.time() * 1000),
            "token": "",
            "payload": {"bindCode": self._bind_code},
        }
        ws.send(json.dumps(auth_msg, ensure_ascii=False))

    def _on_message(self, ws: websocket.WebSocketApp, raw: str, session: int) -> None:
        if session != self._session_id:
            return
        try:
            msg: dict = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("Received non-JSON message: %s", raw[:200])
            return

        msg_type = msg.get("type", "")

        if msg_type == "auth":
            token = msg.get("token") or ""
            error = msg.get("code", 0)
            # Tolerate cloud replies that put the token inside payload.
            if not token and isinstance(msg.get("payload"), dict):
                token = msg.get("payload", {}).get("token", "")
            if error == 401 or not token:
                logger.error(
                    "Auth failed (code=%s). Check that BUNKER_DEVICE_ID and "
                    "BUNKER_BIND_CODE match what the cloud expects. %s",
                    error, msg,
                )
                self._set_state(AgentState.DISCONNECTED)
                self._close_ws()
                return
            with self._lock:
                self._token = token
            self._set_state(AgentState.ONLINE)
            self._failsafe.reset_session()
            logger.info("Authenticated OK, agent ONLINE")
            self._announce_boot_failsafe()

        elif msg_type == "cmd":
            # Envelope checks (manual §4: every post-auth message must
            # carry the deviceId and token).
            dev = msg.get("deviceId")
            if dev and dev != self._device_id:
                logger.warning("Command deviceId mismatch (%s != %s), ignoring", dev, self._device_id)
                return
            with self._lock:
                current_token = self._token
            if not msg.get("token") or msg.get("token") != current_token:
                logger.warning("Command token missing or mismatched, ignoring")
                return
            cmd = Command.from_payload(msg.get("payload", {}))
            try:
                cmd.ts_ms = int(msg.get("ts") or 0)
            except (TypeError, ValueError):
                cmd.ts_ms = 0
            self._failsafe.note_rx()
            verdict = self._failsafe.judge(cmd.action, cmd.ts_ms or None)
            if not verdict.accept:
                self._send_event(
                    EventType.FAILSAFE,
                    f"拒绝过期/劣质链路指令 {cmd.action}（{verdict.reason}）",
                )
                return
            if cmd.action == "move" and cmd.bypass_guard:
                logger.debug(
                    "Received kb move: v=%.2f w=%.2f", cmd.v, cmd.w,
                )
            else:
                logger.info(
                    "Received command: action=%s v=%.2f w=%.2f",
                    cmd.action, cmd.v, cmd.w,
                )
            self._handle_command(cmd)
            for cb in self._command_callbacks:
                try:
                    cb(cmd)
                except Exception:
                    logger.exception("Command callback error")

        elif msg_type == "pong":
            self._failsafe.note_rx()
            ts = msg.get("ts")
            try:
                sent = int(ts) if ts else 0
            except (TypeError, ValueError):
                sent = 0
            if sent > 0:
                self._failsafe.note_pong(time.time() - sent / 1000.0)
            logger.debug("Pong received")

        elif msg_type == "ping":
            self._failsafe.note_rx()
            try:
                ws.send(json.dumps({"type": "pong", "ts": msg.get("ts")}))
            except Exception:
                logger.debug("Pong reply failed", exc_info=True)

        elif msg_type == "error":
            logger.error("Cloud error: %s", msg)

        else:
            logger.debug("Unhandled message type: %s", msg_type)

    def _on_error(self, ws: websocket.WebSocketApp, error: Any) -> None:
        logger.error("WebSocket error: %s", error)
        # 记录本次连接失败原因：服务器不在线（连接被拒）时，应尽快重试，
        # 因为云端可能随时重启——见 _connect_and_serve 的退避决策。
        with self._lock:
            self._last_conn_error = error

    def _connection_refused(self) -> bool:
        """最近一次连接错误是否表明「服务器不在线」。

        连接被拒绝 / 无法解析主机 / 超时等说明云端当前不可达，应当用
        短间隔反复探测，等待云端重启后自动恢复（避免长退避导致「云端
        显示没检测到车侧代理」）。
        """
        with self._lock:
            err = self._last_conn_error
        if err is None:
            return False
        text = str(err).lower()
        refused_markers = (
            "connection refused",
            "refused",
            "timed out",
            "timeout",
            "getaddrinfo failed",
            "name or service not known",
            "name resolution",
            "111",          # ECONNREFUSED errno
            "error 111",
        )
        return any(marker in text for marker in refused_markers)

    def _next_reconnect_count(self, reconnect_count: int, ever_online: bool) -> int:
        """断开后决定下一轮重连的退避档位。

        - 服务器不在线（连接被拒/超时/解析失败）：固定回到最短间隔 1s，
          因为云端可能随时重启，需要保持频繁探测以便尽快自动恢复。
        - 曾在线后正常断开（网络抖动、云端被正常关闭）：指数退避 +1，
          但也从最小档位起步，避免长时间退避导致云端重启后迟迟不重连。
        - 从未在线且断开原因不明（如鉴权失败）：同样用最短间隔重试。
        """
        if self._connection_refused():
            return 1
        if ever_online:
            return reconnect_count + 1
        return 1

    def _on_close(self, ws: websocket.WebSocketApp, close_status_code: int, close_msg: str, session: int) -> None:
        if session != self._session_id:
            return  # stale callback from a previous connection
        # 非正常断开（对端强杀进程 / 网络中断）时 close_status_code 与
        # close_msg 均为 None，格式串里不能用 %d/%s 之外的类型占位，
        # 否则 logging 本身会抛 TypeError 刷屏（详见断线复现）。
        logger.info("WebSocket closed (code=%r, msg=%r, session=%d)",
                    close_status_code, close_msg, session)

        # Capture old state under lock, then send OFFLINE BEFORE changing
        # state (otherwise _send_event's ONLINE guard rejects it).
        with self._lock:
            old_state = self._state

        if old_state == AgentState.ONLINE:
            self._send_event_inline(EventType.OFFLINE, "Connection lost")

            # Always drop a lingering teleop `move` so the watchdog cannot
            # later stop_motion() a local autonomy loop that is still running.
            self._pending_command = None
            self._move_deadline = None

            if self._has_local_autonomy():
                # find_object / goto / replay 的传感、规划、执行都在车侧本地
                # 闭环。云端抖动只应丢掉监控通道，不能把溶洞任务掐死。
                # 遥控 teleop 仍走下面的立即停车。物理急停始终有效。
                logger.warning(
                    "Connection lost — local autonomy continues "
                    "(mission/nav/replay). Cloud monitor is down; "
                    "physical E-stop still applies."
                )
            else:
                # Teleop safety: the TX loop keeps streaming the last
                # velocity forever, so a dropped kb/move link must stop
                # the chassis immediately.
                ctrl = self._controller
                if ctrl is not None:
                    ctrl.stop_motion()
                    logger.warning(
                        "Connection lost — chassis motion stopped for safety"
                    )
                self._stop_existing_mission("connection lost")

        self._failsafe.mark_lost()
        self._set_state(AgentState.DISCONNECTED)

    def _has_local_autonomy(self) -> bool:
        """True when a car-side loop is driving the chassis without the cloud.

        kb/`move` teleop is NOT autonomy: those commands arrive over WS and
        must stop if the link drops.  Missions, goto, and track replay issue
        velocities locally and must survive a mock_cloud/SSH blip.
        """
        thread = self._mission_thread
        if thread is not None and thread.is_alive():
            return True
        nav = self._navigator
        if nav is not None and getattr(nav, "is_navigating", False):
            return True
        player = self._player
        if player is not None and getattr(player, "is_playing", False):
            return True
        task = self._task
        if isinstance(task, dict) and task.get("status") == "running":
            return True
        return False

    # ------------------------------------------------------------------
    # WS / thread helpers — all guarded by _lock for self._ws
    # ------------------------------------------------------------------

    def _close_ws(self) -> None:
        """Close the WebSocket connection safely from any thread."""
        with self._lock:
            ws = self._ws
            self._ws = None
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def _get_ws(self) -> Optional[websocket.WebSocketApp]:
        """Capture a stable reference to the WebSocket under lock."""
        with self._lock:
            return self._ws

    def _emit_tcp(self, msg: dict) -> None:
        """把状态/事件推给本地 TCP 控制台（点云大包不下发）。"""
        srv = self._teleop_tcp
        if srv is None or srv.clients <= 0:
            return
        if msg.get("type") == "event":
            ev = ""
            payload = msg.get("payload")
            if isinstance(payload, dict):
                ev = str(payload.get("event") or "")
            if ev in ("point_cloud",):
                return
            if ev == "lidar_map" and isinstance(payload, dict):
                data = payload.get("data")
                if isinstance(data, dict):
                    slim = {
                        k: data[k]
                        for k in data
                        if k not in ("cells", "free", "occupied", "blocked", "points")
                    }
                    msg = dict(msg)
                    msg["payload"] = dict(payload, data=slim)
        try:
            srv.push_event(msg)
        except Exception:
            logger.debug("TCP event push failed", exc_info=True)

    def _start_background_tasks(self) -> None:
        """Launch heartbeat, state-reporting and watchdog threads.

        Idempotent: 断线重连时定位/看门狗线程可能仍在跑（本地任务不能
        跟着 WS 一起停），只补启动已退出的线程。
        """
        def _spawn(attr: str, target, name: str) -> None:
            t = getattr(self, attr)
            if t is not None and t.is_alive():
                return
            t = threading.Thread(target=target, name=name, daemon=True)
            setattr(self, attr, t)
            t.start()

        _spawn("_heartbeat_thread", self._heartbeat_loop, "agent-hb")
        _spawn("_state_thread", self._state_report_loop, "agent-state")
        _spawn("_loc_thread", self._localization_loop, "agent-loc")
        _spawn("_watchdog_thread", self._watchdog_loop, "agent-watchdog")

    def _stop_background_tasks(self, *, cloud_only: bool = False) -> None:
        """Join background threads.

        ``cloud_only=True``：只回收心跳（它在 OFFLINE 时会自行退出）。
        定位循环与看门狗继续跑，保证 find_object/goto 在云端抖动时
        仍能更新伪地图和扫描匹配。完整停止走 agent.stop() → _cleanup。
        """
        to_join = [self._heartbeat_thread]
        self._heartbeat_thread = None
        if not cloud_only:
            to_join.extend((
                self._state_thread,
                self._loc_thread,
                self._watchdog_thread,
                self._pc_stream_thread,
            ))
            self._state_thread = None
            self._loc_thread = None
            self._watchdog_thread = None
            self._pc_stream_thread = None
        for t in to_join:
            if t and t.is_alive():
                t.join(timeout=2.0)

    def _heartbeat_loop(self) -> None:
        """Send application-level ping every ping_interval_s (section 5.1)."""
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=self._ping_interval_s)
            with self._lock:
                online = (self._state == AgentState.ONLINE)
            if not online:
                break
            if self._local_mode:
                continue
            ws = self._get_ws()
            if ws is None:
                break
            try:
                ws.send(json.dumps({"type": "ping", "ts": int(time.time() * 1000)}))
                logger.debug("Ping sent")
            except Exception:
                logger.warning("Ping failed, closing socket to reconnect")
                self._close_ws()
                break

    def _state_report_loop(self) -> None:
        """在线时向云端 / TCP 控制台推状态。

        扫描匹配与伪地图改由 ``_localization_loop`` 按 10 Hz 跑，不再绑在
        1.5 s 状态节拍上（goto 转弯时 1.5 s 一拍等于没定位）。
        """
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=self._state_interval_s)
            if self._stop_event.is_set():
                break
            with self._lock:
                online = (self._state == AgentState.ONLINE)
            if online:
                self._push_state()

    def _localization_loop(self) -> None:
        """建图前的本机定位节拍：融合里程 → 扫描匹配 → 伪地图。

        必须独立于状态上报。旧实现跟 1.5 s 推状态绑在一起，goto 转几度
        就被 3° 门控整拍跳过，车觉得自己还在原点。
        """
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=LOC_TICK_S)
            if self._stop_event.is_set():
                break
            try:
                self._sync_nav_odometry()
                self._localization_tick()
            except Exception:
                logger.debug("localization tick failed", exc_info=True)
            lidar = self._lidar
            if lidar is not None:
                try:
                    self._update_occupancy_grid(lidar, self._navigator)
                except Exception:
                    logger.debug("occupancy tick failed", exc_info=True)
            nav = self._navigator
            if nav is not None and nav.is_navigating:
                now = time.monotonic()
                if now - self._last_nav_state_push >= NAV_STATE_PUSH_S:
                    self._last_nav_state_push = now
                    try:
                        self._push_state(force=True)
                    except Exception:
                        logger.debug("nav pose push failed", exc_info=True)

    # ------------------------------------------------------------------
    # Motion watchdog
    # ------------------------------------------------------------------

    def _watchdog_loop(self) -> None:
        """Periodically enforce the move keep-alive timeout.

        Runs independently of the state-report loop so the chassis is
        stopped quickly even if state reporting stalls.
        """
        while not self._stop_event.is_set():
            self._stop_event.wait(timeout=MOVE_WATCHDOG_INTERVAL_S)
            if self._stop_event.is_set():
                break
            self._check_move_watchdog()
            self._refresh_guarded_move()
            self._check_link_failsafe()
            self._check_power_failsafe()
            self._maybe_failsafe_checkpoint()
            # 与「启动时只探一次、锁错 can0」同一条修复：无 0x211 时周期重探。
            # 不能只挂在 _drive 里——遥控被丢掉时根本不会进 _drive。
            ctrl = self._controller
            self._recover_stuck_mailbox()
            self._publish_chassis_status()
            if ctrl is not None and ctrl.latest_status is None:
                self._maybe_reprobe_can_channel(time.time())
            elif ctrl is not None:
                self._arm_can_tx_if_heard()

    def _recover_stuck_mailbox(self) -> None:
        """ENOBUFS 时由看门狗线程清一次发送邮箱，不在 TX 线程里 down/up。"""
        ctrl = self._controller
        if ctrl is None or not getattr(ctrl, "mailbox_stuck", False):
            return
        if getattr(ctrl, "latest_status", None) is None:
            # 没有 0x211 = 总线上没有对端 ACK。再 down/up 清不掉，只会打断发送。
            ctrl.note_mailbox_recovered()
            return
        channel = self._can_channel
        logger.warning("[底盘诊断] USB-CAN 发送队列已堵，清一次邮箱 %s", channel)
        try:
            flush_socketcan_mailbox(channel)
            rebuild = getattr(ctrl, "rebuild_bus", None)
            if callable(rebuild):
                rebuild()
            ctrl.note_mailbox_recovered()
        except Exception:
            logger.debug("清发送邮箱失败", exc_info=True)

    def _publish_chassis_status(self) -> None:
        ctrl = self._controller
        st = getattr(ctrl, "latest_status", None) if ctrl is not None else None
        try:
            detail = self._chassis_detail()
        except Exception:
            detail = {"heard": st is not None}
        bms = detail.get("bms") if isinstance(detail, dict) else None
        sys = detail.get("system") if isinstance(detail, dict) else None
        payload = {
            "heard": st is not None,
            "mode": (sys or {}).get("controlMode") or str(getattr(st, "control_mode", "") or ""),
            "vehicleState": (sys or {}).get("vehicleState"),
            "battery": None if not bms else (int(bms.get("socPercent") or 0) / 100.0),
            "channel": self._can_channel,
            "tx_ok": int(getattr(ctrl, "tx_ok", 0) or 0),
            "tx_fail": int(getattr(ctrl, "tx_fail", 0) or 0),
            "tx_err": str(getattr(ctrl, "last_tx_error", "") or ""),
            "can": {
                "channel": self._can_channel,
                "interface": self._can_interface,
                "feedback": st is not None,
            },
            "chassis": detail,
        }
        try:
            path = "/tmp/bunker_chassis_status"
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(payload, fp)
        except Exception:
            pass

    def _check_move_watchdog(self) -> None:
        """Auto-stop when a `move` expires (duration elapsed or no refresh)."""
        with self._lock:
            pending = self._pending_command
            if pending is None or pending.action != "move":
                return
            # 已经是停车指令：直通刹停后再清 pending。只清 pending 会让
            # TX 环继续保活斜坡残速（松键 creeping）。
            if float(pending.v) == 0.0 and float(pending.w) == 0.0:
                self._pending_command = None
                self._move_deadline = None
                ctrl = self._controller
                if ctrl is not None:
                    ctrl.stop_motion()
                return
            deadline = self._move_deadline
            now = time.time()
            elapsed = now - self._last_move_at

        if deadline is not None:
            if now < deadline:
                return  # still inside the commanded duration
            reason = f"move duration ({pending.duration:.1f}s) elapsed"
        else:
            if elapsed <= MOVE_KEEPALIVE_TIMEOUT_S:
                return
            reason = f"no move refresh for {elapsed:.0f}s"

        with self._lock:
            self._pending_command = None
            self._move_deadline = None
        ctrl = self._controller
        if ctrl is not None:
            ctrl.stop_motion()
        self._teleop_inhibit_until = time.time() + 0.4
        logger.warning("Auto-stopped chassis: %s", reason)
        self._send_event("auto_stop", f"Auto-stop: {reason}")

    def _pending_skips_guard(self, pending: Optional[Command]) -> bool:
        """kb / 开环定时 move 不过雷达守卫。"""
        if pending is None:
            return False
        return bool(pending.bypass_guard) or bool(getattr(pending, "open_loop", False))

    def _refresh_guarded_move(self) -> None:
        """遥控 move 持续期间每拍再过一次守卫。

        ``m 0.1 0 5`` 旧实现只在下发瞬间看一眼雷达，之后几秒底盘保持原速，
        人/椅靠近或车开过去都不会再刹。导航/回放/探路自带控制环，这里不抢杆。

        开环 move / kb 只在下发时绕过守卫；若这里仍按 ``bypass_guard`` 判断，
        网页「开环（不过雷达）」会在 0.2 s 看门狗拍被近场急停打成 0，
        表现为「执行中但车不动 / 只蹭一下」。开环交给 TX 环保活已下发速度。
        """
        with self._lock:
            pending = self._pending_command
            deadline = self._move_deadline
        if pending is None or pending.action != "move":
            return
        if deadline is not None and time.time() >= deadline:
            return
        if time.time() < self._teleop_inhibit_until:
            return
        if self._has_local_autonomy():
            return
        if self._pending_skips_guard(pending):
            return
        v = max(-MAX_SAFE_LINEAR_M_S, min(MAX_SAFE_LINEAR_M_S, pending.v))
        w = max(-MAX_SAFE_ANGULAR_RAD_S, min(MAX_SAFE_ANGULAR_RAD_S, pending.w))
        self._drive(v, w, apply_guard=True)

    def _check_link_failsafe(self) -> None:
        """Silence / RTT 分级：半开连接当断线；高延迟丢掉遥控。"""
        if self._local_mode:
            return
        with self._lock:
            online = self._state == AgentState.ONLINE
            pending = self._pending_command
        if not online:
            return
        link, rx_age, _rtt = self._failsafe.link_state()
        if link is LinkClass.LOST:
            logger.warning(
                "Failsafe: link lost (silence %.1fs) — closing WebSocket",
                -1.0 if rx_age is None else rx_age,
            )
            self._close_ws()
            return
        if link is LinkClass.DEGRADED and pending is not None \
                and pending.action == "move" and not self._has_local_autonomy():
            with self._lock:
                self._pending_command = None
                self._move_deadline = None
            ctrl = self._controller
            if ctrl is not None:
                ctrl.stop_motion()
            logger.warning("Failsafe: degraded link — teleop stopped")
            self._send_event(
                EventType.FAILSAFE,
                "链路延迟过大，遥控已停车（本地任务不受影响）",
            )

    def _check_power_failsafe(self) -> None:
        """进程仍活着时的欠压保护：任务中 → 本地返航；否则停车。"""
        ctrl = self._controller
        if ctrl is None:
            return
        status = ctrl.latest_status
        bms = ctrl.latest_bms
        flags = FaultFlags.from_byte(status.fault_code) if status is not None else None
        power = self._failsafe.observe_power(
            soc_percent=None if bms is None else bms.soc_percent,
            voltage_v=None if status is None else status.battery_voltage_v,
            undervoltage_fault=bool(flags and flags.battery_undervoltage_fault),
            undervoltage_warning=bool(flags and flags.battery_undervoltage_warning),
        )
        with self._lock:
            mission = self._mission
        status_s = None if not mission else str(mission.get("status") or "")
        if status_s == "returning":
            return
        if power is PowerClass.CRITICAL:
            if self._has_local_autonomy() or should_return_on_power(status_s, power):
                if self._failsafe.begin_power_return():
                    name = (mission or {}).get("target") or "target"
                    logger.warning(
                        "Failsafe: power CRITICAL during %s — abort and return",
                        status_s or "autonomy",
                    )
                    self._send_event(
                        EventType.FAILSAFE,
                        f"电量危急，中止任务并返航（原状态 {status_s or 'autonomy'}）",
                    )
                    self._handle_cancel(
                        Command(action="cancel", name=str(name)),
                        detail=f"电量危急：中止 {status_s or 'autonomy'} 并返回",
                    )
            else:
                ctrl.stop_motion()

    def _maybe_failsafe_checkpoint(self, *, force: bool = False,
                                   reason: str = "periodic") -> None:
        with self._lock:
            mission = self._mission
        status = None if not mission else mission.get("status")
        if status not in ACTIVE_MISSION_STATUSES and not force:
            return
        pose = None
        nav = self._navigator
        if nav is not None:
            p = nav.pose
            pose = {
                "x": round(float(p.x), 3),
                "y": round(float(p.y), 3),
                "yawDeg": round(float(p.yaw), 1),
            }
        track_name = None
        recon = self._recon_track
        if recon is not None:
            track_name = recon.name
        self._failsafe.maybe_save(
            {
                "schema": 1,
                "mission": None if mission is None else dict(mission),
                "pose": pose,
                "reconTrack": track_name,
                "failsafe": self._failsafe.snapshot(),
            },
            force=force,
            reason=reason,
        )

    def _restore_failsafe_checkpoint(self) -> None:
        data = self._failsafe.load_boot_checkpoint()
        if not data:
            return
        mission = data.get("mission") or {}
        status = mission.get("status")
        if status not in ACTIVE_MISSION_STATUSES:
            return
        self._mission = {
            "status": "interrupted",
            "target": mission.get("target", ""),
            "detail": "断电或进程重启：未自动续开（里程原点已丢失）",
            "lastStatus": status,
            "lastPose": data.get("pose"),
            "reconTrack": data.get("reconTrack"),
        }
        logger.warning(
            "Failsafe: boot checkpoint mission=%s — holding, not resuming motion",
            status,
        )

    def _announce_boot_failsafe(self) -> None:
        if self._boot_failsafe_announced:
            return
        mission = self._mission
        if not mission or mission.get("status") != "interrupted":
            return
        self._boot_failsafe_announced = True
        self._send_event(
            EventType.FAILSAFE,
            "检测到上次任务被断电/重启打断，未自动续开。可 c 返航或重新 fo。",
        )

    # ------------------------------------------------------------------
    # Command dispatch  (section 5.2)
    # ------------------------------------------------------------------

    def _handle_command(self, cmd: Command) -> None:
        if self._controller is None:
            logger.warning("CAN controller not ready, skipping command")
            return

        action = cmd.action

        if self._combat_lock_blocks(action):
            logger.warning("实战锁：忽略遥控/调试指令 %s", action)
            self._send_event("fault", f"实战锁：任务进行中忽略 {action}")
            return

        if action == "move":
            # A new motion command supersedes any running track playback —
            # otherwise the replay thread keeps re-issuing velocities and
            # fights the manual command.
            player = self._player
            if player is not None and player.is_playing:
                player.stop()
            self._cancel_task("manual move")
            # A manual move also cancels ongoing goto navigation.
            nav = self._navigator
            if nav is not None and nav.is_navigating:
                nav.stop()
                logger.info("Move command cancelled ongoing navigation")
            # 手动 move 取消正在进行的 find_object；无任务线程时跳过
            # （kb 刷新约 5 Hz，不必每次 join/清 TTL）。
            mt = self._mission_thread
            if mt is not None and mt.is_alive():
                self._stop_existing_mission("manual move")
            # Clamp to safe limits instead of letting an out-of-range
            # value either drive at full speed or crash the command
            # dispatch with a ValueError.
            v = max(-MAX_SAFE_LINEAR_M_S, min(MAX_SAFE_LINEAR_M_S, cmd.v))
            w = max(-MAX_SAFE_ANGULAR_RAD_S, min(MAX_SAFE_ANGULAR_RAD_S, cmd.w))
            if v != cmd.v or w != cmd.w:
                logger.warning(
                    "move clamped: requested v=%.2f w=%.2f → v=%.2f w=%.2f",
                    cmd.v, cmd.w, v, w,
                )
            # 停车必须直通：set_velocity(0,0) 只斜坡一步，看门狗接着把
            # pending 清掉后 TX 环会把残速（常见 0.03~0.04 m/s）保活到
            # 下一次非零指令——松键后「同方向极低速 creeping」。
            if abs(v) <= 1e-6 and abs(w) <= 1e-6:
                with self._lock:
                    self._pending_command = None
                    self._move_deadline = None
                ctrl = self._controller
                if ctrl is not None:
                    ctrl.stop_motion()
                return
            # LiDAR obstacle guard: slow down near obstacles, stop inside
            # the stop distance. kb 遥控带 bypass_guard，不过雷达。
            lidar = self._lidar
            open_loop = bool(getattr(cmd, "open_loop", False))
            apply_guard = not cmd.bypass_guard and not open_loop
            if apply_guard and lidar is not None and not lidar.is_receiving:
                self._send_event(
                    "fault",
                    "move: 雷达已开启但当前离线，避障会把速度打成 0。"
                    "请先点「雷达关」，或勾选「开环（不过雷达）」再试。"
                    "WASD 遥控不受此限制。goto 必须等雷达在线。",
                )
                return
            commanded_v, commanded_w = v, w
            blocked = False
            if self._guard is not None and apply_guard:
                v, w, blocked = self._guard.guard_velocity(commanded_v, commanded_w)
            duration = float(cmd.duration or 0.0)
            if cmd.bypass_guard and duration > TELEOP_DURATION_CAP_S:
                duration = TELEOP_DURATION_CAP_S
            # pending 必须记住操作员原速。若把守卫打成的 0 写进去，
            # 空地误刹一次后看门狗会按 (0,0) 把整段 m 直接结束。
            pending = Command(
                action="move",
                v=commanded_v,
                w=commanded_w,
                duration=duration,
                bypass_guard=cmd.bypass_guard,
                open_loop=open_loop,
            )
            with self._lock:
                self._pending_command = pending
                self._last_move_at = time.time()
                # A duration-bearing move auto-stops after `duration` seconds;
                # without one, the keep-alive watchdog stops it after
                # MOVE_KEEPALIVE_TIMEOUT_S without a refresh.
                self._move_deadline = (
                    time.time() + duration if duration > 0 else None
                )
            # kb / 开环：目标速度一步到位，不走 0.5 m/s² 斜坡。
            # 普通 m/goto 仍斜坡，避免导航/避障突然窜车。
            # 本拍已经 guard 过：blocked 必须 stop_motion，不能 set_velocity(0)
            # 斜坡残速。后续看门狗按 pending 原速再过守卫，误刹能恢复。
            if blocked:
                ctrl = self._controller
                if ctrl is not None:
                    ctrl.stop_motion()
            else:
                self._drive(
                    v, w,
                    apply_guard=False,
                    bypass_ramp=bool(cmd.bypass_guard) or open_loop,
                )
            if duration > 0:
                logger.info(
                    "MOVING: v=%.2f m/s w=%.2f rad/s for %.1f s%s, then auto-stop",
                    commanded_v, commanded_w, duration,
                    " (open-loop)" if open_loop else "",
                )
                self._send_event(
                    "move",
                    f"定时运动已开始：v={commanded_v:.2f} m/s  w={commanded_w:.2f} rad/s  {duration:.1f}s"
                    + ("（开环）" if open_loop else ""),
                )
            else:
                logger.info(
                    "MOVING: v=%.2f m/s w=%.2f rad/s (auto-stop in %.0f s without refresh)",
                    v, w, MOVE_KEEPALIVE_TIMEOUT_S,
                )
                self._send_event(
                    "move",
                    f"运动已开始：v={v:.2f} m/s  w={w:.2f} rad/s（无时长，需持续刷新）",
                )
            self._push_state(force=True)

        elif action == "estop":
            # Emergency stop must also halt any active track replay,
            # otherwise the playback thread re-issues velocity commands
            # and the chassis starts moving again.
            player = self._player
            if player is not None and player.is_playing:
                player.stop()
            self._cancel_task("estop")
            nav = self._navigator
            if nav is not None and nav.is_navigating:
                nav.stop()
            self._stop_existing_mission("estop")
            self._pending_command = None
            self._move_deadline = None
            self._controller.stop_motion()
            logger.info("EMERGENCY STOP executed")

        elif action == "goto":
            self._handle_goto(cmd)

        elif action == "find_object":
            self._handle_find_object(cmd)

        elif action == "go_home":
            self._handle_go_home(cmd)

        elif action == "task_submit":
            self._handle_task_submit(cmd)

        elif action == "track_record":
            self._handle_track_record(cmd)

        elif action == "track_follow":
            self._handle_track_follow(cmd)

        elif action == "track_delete":
            self._handle_track_delete(cmd)

        elif action == "query":
            self._push_state(force=True)
            try:
                self._send_event_data("chassis", self._chassis_detail(), log=False)
            except Exception:
                logger.debug("chassis event push failed", exc_info=True)

        elif action == "lidar_on":
            self._handle_lidar_on(cmd)

        elif action == "lidar_off":
            self._handle_lidar_off(cmd)

        elif action == "lidar_status":
            self._handle_lidar_status(cmd)

        elif action == "lidar_map":
            self._handle_lidar_map(cmd)

        elif action == "point_cloud":
            self._handle_point_cloud(cmd)

        elif action == "pc_stream":
            self._handle_pc_stream(cmd)

        elif action == "cancel":
            self._handle_cancel(cmd)

        elif action == "odom_reset":
            self._handle_odom_reset(cmd)

        elif action == "map_upload":
            self._handle_map_upload(cmd)

        elif action == "pose_align":
            self._handle_pose_align(cmd)

        elif action == "map_return":
            self._handle_map_return(cmd)

        elif action == "grasp_done":
            self._handle_grasp_done(cmd)

        else:
            logger.warning("Unknown action: %s", action)

    def _handle_track_record(self, cmd: Command) -> None:
        """Start or stop trajectory recording."""
        recorder = self._recorder
        if recorder is None:
            return

        if cmd.name:
            # Names come from the cloud — keep them filesystem-safe
            # (strip path separators / illegal characters).
            name = sanitize_track_name(cmd.name)
            if recorder.is_recording:
                logger.warning("Track record: already recording '%s', ignoring start", name)
                self._send_event(
                    "fault",
                    f"Track record: already recording, ignored start '{name}'",
                )
                return
            recorder.start(name)
            logger.info(
                "Track recording started: '%s' — recording only samples, it does "
                "NOT drive the chassis; drive it with move/remote to record a path",
                name,
            )
            self._send_event("track_record", f"Recording started: {name}")
        else:
            if not recorder.is_recording:
                logger.warning("Track record: not recording, ignoring stop")
                self._send_event("fault", "Track record: not recording")
                return
            try:
                track = recorder.stop()
                player = self._player
                if player:
                    # 本段录制刚停：必须按原名落盘。若改成 r1_时间戳，
                    # 云端 B 仍 follow r1，会放旧文件或找不到新轨迹。
                    # 覆盖流程已先 track_delete；同名覆盖是预期行为。
                    player.save_track(track)

                # Tell the cloud whether the recording actually contains
                # movement — a saved all-zero track is a useless/dangerous
                # replay target and the operator should know immediately.
                has_motion = any(w.v != 0.0 or w.w != 0.0 for w in track.waypoints)
                has_odo = any(
                    w.left_mm != track.waypoints[0].left_mm
                    or w.right_mm != track.waypoints[0].right_mm
                    for w in track.waypoints[1:]
                )
                if not has_motion and not has_odo:
                    self._send_event(
                        "track_record",
                        f"Track '{track.name}' saved but has NO movement data — "
                        "the chassis did not move during recording. Drive it next time.",
                    )
                else:
                    self._send_event(
                        "track_record",
                        f"Track saved: {track.name} ({track.total_duration_s:.1f}s, {len(track.waypoints)} waypoints)",
                    )
            except RuntimeError as e:
                logger.error("Track record stop failed: %s", e)
                self._send_event("fault", f"Track record stop failed: {e}")

    def _handle_track_follow(self, cmd: Command) -> None:
        """Play back a previously recorded trajectory."""
        player = self._player
        if player is None or self._controller is None:
            return

        if not cmd.track_id:
            logger.warning("Track follow: missing trackId")
            self._send_event("fault", "Track follow: missing trackId")
            return

        # The trackId arrives from the cloud — reject anything that could
        # escape the tracks directory (path traversal).  Local CLI tools
        # (play_track.py --file) still accept absolute paths.
        if "/" in cmd.track_id or "\\" in cmd.track_id or ".." in cmd.track_id:
            logger.warning("Track follow: invalid trackId rejected: %r", cmd.track_id)
            self._send_event("fault", f"Invalid trackId: {cmd.track_id}")
            return

        if player.is_playing:
            logger.warning("Track follow: already playing, stopping first")
            player.stop()

        try:
            track = player.load_track(cmd.track_id)
        except FileNotFoundError as e:
            logger.error("Track follow: %s", e)
            self._send_event("fault", f"Track not found: {cmd.track_id}")
            return
        except ValueError as e:
            logger.error("Track follow: %s", e)
            self._send_event("fault", f"Track corrupt: {cmd.track_id}")
            return

        # A manual replay supersedes any running transport task.
        self._cancel_task("track_follow")
        # And any ongoing goto navigation.
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        # And any find_object mission.
        self._stop_existing_mission("track_follow")

        # Playback drives the chassis itself — forget any lingering manual
        # move so the motion watchdog cannot auto-stop the replay mid-way.
        with self._lock:
            self._pending_command = None
            self._move_deadline = None
        # 先刹住 kb 残速 / 家具锁残留，避免回放第一拍被车头近障误判急停
        # （录制结束按 B 时常见：车还在 0.03 m/s 往前拱，守卫直接 abort）。
        if self._controller is not None:
            self._controller.stop_motion()

        direction = "reverse" if cmd.reverse else "forward"
        if cmd.reverse:
            track = track.reversed()
            logger.info("Track follow: replaying '%s' in REVERSE (back to start)", track.name)
        else:
            logger.info("Track follow: replaying '%s'", track.name)
        self._send_event("track_follow", f"Replaying: {track.name} ({direction})")
        # 已录路径是固定轮迹（B 回程尤其是「沿原路回家」死命令）：
        # 雷达守卫不得限速/急停，否则车头近处椅腿会把回程第一拍掐死。
        player.play_async(
            track,
            on_complete=lambda complete: self._on_track_complete(direction, complete),
            reverse=bool(cmd.reverse),
            bypass_guard=True,
        )

    def _on_track_complete(self, direction: str, complete: bool) -> None:
        """Called by TrackPlayer when replay finishes — notify the cloud.

        无论回放是否完整都会上报 arrived（除非被显式 stop() 打断），否则
        「回程中途卡住 → 云端收不到 arrived → 回程状态一直空等、终端卡死」。
        complete=False 表示残缺/被避障中止的回放，文案里会明确说明。

        里程缺失（时间回放）时，即使 complete=True 也**不能**声称「已返回
        起点」——时间回放没有位置反馈，小车不一定真的回到起点。文案会
        明确标注 approximate，避免误导。
        """
        had_odo = bool(self._player and self._player.last_playback_had_odo)
        if direction == "reverse":
            if not complete:
                self._send_event(
                    EventType.ARRIVED,
                    "Reverse replay interrupted — did not fully return to start",
                )
            elif not had_odo:
                self._send_event(
                    EventType.ARRIVED,
                    "Reverse replay finished by time-based fallback (odometer "
                    "unavailable) — position approximate, may not be exactly "
                    "back at start",
                )
            else:
                self._send_event(EventType.ARRIVED, "Reverse replay completed — back at start")
        else:
            if not complete:
                self._send_event(
                    EventType.ARRIVED,
                    "Track replay interrupted — did not fully cover the route",
                )
            elif not had_odo:
                self._send_event(
                    EventType.ARRIVED,
                    "Track replay finished by time-based fallback (odometer "
                    "unavailable) — route coverage approximate",
                )
            else:
                self._send_event(EventType.ARRIVED, "Track replay completed")

    def _handle_track_delete(self, cmd: Command) -> None:
        """Delete a recorded track file (manage multiple 轨迹克隆 recordings)."""
        player = self._player
        if player is None:
            return

        if not cmd.track_id:
            logger.warning("Track delete: missing trackId")
            self._send_event("fault", "Track delete: missing trackId")
            return

        # Same path-traversal guard as track_follow — the name comes from
        # the cloud and must never escape the tracks directory.
        if "/" in cmd.track_id or "\\" in cmd.track_id or ".." in cmd.track_id:
            logger.warning("Track delete: invalid trackId rejected: %r", cmd.track_id)
            self._send_event("fault", f"Invalid trackId: {cmd.track_id}")
            return

        try:
            deleted = player.delete_track(cmd.track_id)
        except FileNotFoundError as e:
            logger.error("Track delete: %s", e)
            self._send_event("fault", f"Track not found: {cmd.track_id}")
            return
        self._send_event("track_delete", f"Track deleted: {deleted}")

    # ------------------------------------------------------------------
    # LiDAR avoidance helpers  (雷达避障辅助)
    # ------------------------------------------------------------------

    def _velocity_guard(self, v: float, w: float) -> tuple[float, float, bool]:
        """Wire the current obstacle guard into track/task replay.

        TrackPlayer calls this for every waypoint velocity; on a hard stop it
        aborts the replay (chassis stopped, no `arrived` event).
        """
        guard = self._guard
        if guard is None:
            return v, w, False
        return guard.guard_velocity(v, w)

    def _on_obstacle_blocked(self, distance_m: float, reason: str) -> None:
        """Obstacle guard hard-stop callback — notify the cloud immediately."""
        self._send_event("obstacle", reason)
        logger.warning("Obstacle event: %s", reason)

    def _on_odometer_feedback(self, odometer) -> None:
        """Feed live (fused) wheel odometry into the navigator pose."""
        nav = self._navigator
        if nav is not None:
            nav.feed_odometry(odometer.left_wheel_mm, odometer.right_wheel_mm)

    def _sync_nav_odometry(self) -> None:
        """10 Hz 兜底：把融合里程推进导航位姿（0x221 回调漏了也不停）。"""
        ctrl = self._controller
        nav = self._navigator
        if ctrl is None or nav is None:
            return
        odo = getattr(ctrl, "live_odometer", None)
        if odo is None:
            odo = ctrl.latest_odometer
        if odo is None:
            return
        nav.feed_odometry(odo.left_wheel_mm, odo.right_wheel_mm)

    def _localization_tick(self) -> None:
        """周期修正航迹推算漂移（外部绝对位姿优先，本机扫描匹配兜底）。

        建图完成后由 ``pose_source`` 提供地图系位姿；本方法经
        :class:`MapAlignment.map_to_odom` 换算回里程系后覆盖 navigator 位姿
        （内部仍统一里程系）。建图前无 pose_source 时，退化为用自身雷达做
        轻量扫描匹配（:meth:`_scan_match_tick`）界住漂移。
        """
        src = self._pose_source
        nav = self._navigator
        if nav is None:
            return
        if src is not None:
            try:
                obs = src.get_pose()
            except Exception:
                logger.debug("pose_source.get_pose failed", exc_info=True)
                return
            if obs is not None:
                try:
                    x, y, yaw_deg = obs
                except (TypeError, ValueError):
                    logger.warning("pose_source 返回格式非法，应为 (x, y, yaw_deg)")
                    return
                before = nav.pose
                xo, yo, yoaw = self._map_alignment.map_to_odom(x, y, yaw_deg)
                nav.apply_external_pose(xo, yo, yoaw)
                # 定位健康度：绝对位姿源（SLAM/视觉）生效，记录本次修正量
                self._loc_source = "external"
                self._loc_last_corr = {
                    "dx": round(xo - before.x, 3),
                    "dy": round(yo - before.y, 3),
                    "dyawDeg": round(_wrap_deg(yoaw - math.degrees(before.yaw)), 2),
                    "at": time.monotonic(),
                }
                return
        # 外部绝对位姿暂不可用 → 本机轻量扫描匹配兜底（建图前漂移闭环）
        self._scan_match_tick()

    def _scan_match_tick(self) -> None:
        """轻量扫描匹配修正：把当前帧与自身参考地图对齐，界住里程漂移。

        顺序：**先匹配、后注册**（参考地图只含历史帧，避免当前帧自匹配
        把修正量偏置为 0）。门控：
          * 原地/快速旋转时不匹配（对称场景易误配）；
          * 只接受小窗口内的高分且优次分明结果；
          * 修正量按 ``blend`` 融合，单帧不突变。
        """
        matcher = self._scan_matcher
        nav = self._navigator
        lidar = self._lidar
        if matcher is None or nav is None or lidar is None \
                or not lidar.is_receiving:
            return
        try:
            sectors = lidar.sector_points()
        except AttributeError:
            sectors = []
        if not sectors:
            return
        pose = nav.pose
        # 只跳过猛打方向（约 120°/s @ 10Hz）。旧门槛 3° 绑在 1.5s 节拍上
        # 时，goto 每次转向都整拍跳过，匹配等于没跑。
        if self._scan_match_last_yaw is not None:
            d_yaw = abs(_wrap_angle_rad(pose.yaw - self._scan_match_last_yaw))
            if math.degrees(d_yaw) > 12.0:
                self._scan_match_last_yaw = pose.yaw
                return
        self._scan_match_last_yaw = pose.yaw
        corr = matcher.match(sectors, pose.x, pose.y, pose.yaw_deg)
        if corr is None:
            matcher.observe(sectors, pose.x, pose.y, pose.yaw_deg)
            return
        dx, dy, dyaw_deg = corr
        blend = self._scan_match_blend
        if blend > 0:
            nx = pose.x + dx * blend
            ny = pose.y + dy * blend
            nyaw = _wrap_angle_rad(pose.yaw + math.radians(dyaw_deg) * blend)
            nav.apply_external_pose(nx, ny, math.degrees(nyaw))
            # 定位健康度：本机扫描匹配在兜底修正（建图前漂移闭环）
            self._loc_source = "scanmatch"
            self._loc_last_corr = {
                "dx": round(dx, 3),
                "dy": round(dy, 3),
                "dyawDeg": round(dyaw_deg, 2),
                "at": time.monotonic(),
            }
            logger.debug("scan match: corr d=(%.2f, %.2f) d%.1f°",
                         dx, dy, dyaw_deg)
        else:
            self._loc_source = "odom"
        # 用校正后位姿把当前帧注册进参考地图（保持参考坐标系一致）
        pose2 = nav.pose
        matcher.observe(sectors, pose2.x, pose2.y, pose2.yaw_deg)

    # ------------------------------------------------------------------
    # Point-to-point navigation  (goto — 目标点自主导航)
    # ------------------------------------------------------------------

    def _handle_goto(self, cmd: Command) -> None:
        """Navigate the chassis to (x, y) in the odometry frame.

        目标点基于「里程系/导航坐标系」：原点 (0, 0) 为 agent 启动（导航栈
        初始化）时小车所在位置、车头方向为 yaw=0（与底盘物理上电无关——
        若 agent 未启动时小车被搬动，原点按 agent 启动瞬间的位置起算）。
        原点可被 ``odom_reset`` 指令远程重置。
        与 move / 回放 / task 互斥；完成推 ``arrived`` 事件，因障碍长时间
        无法到达则推 ``auto_stop`` 事件。
        """
        nav = self._navigator
        if nav is None:
            logger.warning("goto rejected: navigator not initialized")
            self._send_event("fault", "goto: 导航未启用（雷达未就绪）")
            return

        # 雷达栈已开启但收不到点云：fail-safe 守卫会把一切速度指令拦成停车，
        # 直接拒绝并给原因，避免「下发后小车没反应、5s 后才 auto_stop」。
        lidar = self._lidar
        if lidar is not None and not lidar.is_receiving:
            self._send_event(
                "fault",
                "goto: 雷达已开启但当前收不到点云（0 帧），无法安全导航。"
                "先排查雷达数据链路（见 lidar status 的 packets 计数），"
                "雷达在线后再试",
            )
            return

        # 雷达 2D 伪地图预检目标点：已知不可通行/被障碍占据 → 直接拒绝，
        # 避免「设计 goto 前不知道哪里不能走」导致撞台阶/岩壁。
        # 注意：伪地图是「累积 + 时效」的（OccupancyGrid 时间衰减），但小车
        # 长期未移动时旧观测不会自动刷新，可能把早期/瞬时近场点残留成
        # occupied/blocked 而误拒 goto。因此只在**实时雷达复核确认**目标
        # 方位存在明显近于目标的障碍时才拒绝；其余情况按陈旧数据放行，
        # 导航途中实时避障守卫仍会兜底停车/绕行（安全不降级）。
        grid = self._occ_grid
        if grid is not None:
            check = grid.check_target(cmd.x, cmd.y)
            state = check["state"]
            if state in ("blocked", "occupied"):
                if self._confirm_obstacle_between(cmd.x, cmd.y, nav, lidar):
                    self._send_event(
                        "fault",
                        f"goto 目标 ({cmd.x:.2f}, {cmd.y:.2f}) 前方存在实时障碍，"
                        "已拒绝下发（导航途中会被避障守卫挡住）",
                    )
                    return
                logger.warning(
                    "goto precheck: 伪地图标记目标为 %s（最近障碍 %s m），"
                    "但实时雷达目标方位无近障，按可通行放行",
                    state, check["nearestObstacle"],
                )
            elif state == "unknown":
                logger.info(
                    "GOTO: 目标 (%.2f, %.2f) 尚无雷达观测，按原计划执行",
                    cmd.x, cmd.y,
                )

        # goto 与手动 move / 轨迹回放互斥
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        self._cancel_task("goto")
        self._stop_existing_mission("goto")
        with self._lock:
            self._pending_command = None
            self._move_deadline = None

        speed = cmd.speed if cmd.speed > 0.0 else None
        if speed is not None:
            speed = min(speed, MAX_SAFE_LINEAR_M_S)
            logger.info("GOTO: 本次导航线速度上限 %.2f m/s", speed)

        # 全局路径规划：把「A* 绕障折线」切成中间航点，goto 沿航点导航，
        # 避免直线 goto 直穿已知障碍。规划失败/无地图时退化为直线导航。
        waypoints = self._plan_waypoints(cmd.x, cmd.y)

        goal_yaw = math.radians(cmd.yaw_deg) if cmd.goal_yaw_set else None
        ok = nav.goto(
            cmd.x,
            cmd.y,
            waypoints=waypoints,
            on_arrived=lambda: self._send_event(
                EventType.ARRIVED,
                f"goto arrived: ({cmd.x:.2f}, {cmd.y:.2f})",
            ),
            on_abort=lambda reason: self._send_event(
                "auto_stop", f"goto aborted: {reason}"
            ),
            speed=speed,
            replanner=self._replan_waypoints,
            goal_yaw=goal_yaw,
        )
        if not ok:
            self._send_event("fault", "goto: 已在导航中，请先停止")
            return
        n_wp = len(waypoints) if waypoints else 0
        logger.info("GOTO: navigating to (%.2f, %.2f) via %d planned waypoint(s)",
                    cmd.x, cmd.y, n_wp)
        self._send_event(
            "goto",
            f"导航已开始：目标 ({cmd.x:.2f}, {cmd.y:.2f})"
            + (f"，{n_wp} 个航点" if n_wp else "，直线"),
        )
        self._push_state(force=True)

    def _confirm_obstacle_between(self, tx: float, ty: float, nav,
                                  lidar) -> bool:
        """实时复核目标方位是否存在明显近于目标的障碍（goto 预检用）。

        伪地图预检可能被历史残留格误判，这里用**实时**雷达（滑动窗口，
        与避障守卫同一数据路径）复核目标方向：
          * 扇区障碍：目标方位 ±25° 内有回波，且距离比目标明显更近
            （≥0.3 m）→ 真障碍挡路；
          * 地形不可通行（台阶/岩壁/坑/下坠）：同方位地形剖面 blocked，
            且不可通行距离明显近于目标 → 前方过不去。
        两者都无 → 返回 False（放行，按陈旧伪地图数据处理）。
        """
        if lidar is None or nav is None:
            return False
        pose = nav.pose
        dx, dy = tx - pose.x, ty - pose.y
        target_dist = math.hypot(dx, dy)
        yaw = math.radians(pose.yaw_deg)
        siny, cosy = math.sin(yaw), math.cos(yaw)
        vx = dx * siny - dy * cosy
        vy = dx * cosy + dy * siny
        bearing = math.degrees(math.atan2(vx, vy)) % 360.0

        try:
            live = lidar.nearest_in_range(bearing, 25.0)
            if live is not None and target_dist - live >= 0.30:
                logger.info("goto precheck: 实时障碍 %.2f m 近于目标 %.2f m（方位 %.0f°）",
                            live, target_dist, bearing)
                return True
            terrain = lidar.terrain_sector(bearing, self._step_limit_m)
            if terrain.blocked:
                cands = [x for x in (terrain.obstacle_distance_m,
                                     terrain.negative_obstacle_distance_m)
                         if x is not None]
                if cands and target_dist - min(cands) >= 0.30:
                    logger.info(
                        "goto precheck: 目标方位地形不可通行 %.2f m（方位 %.0f°）",
                        min(cands), bearing)
                    return True
        except Exception:
            logger.exception("goto precheck: 实时复核异常，按放行处理")
        return False

    # ------------------------------------------------------------------
    # Object-finding mission  (find_object — 搜索 → 检测 → 导航 → 对接)
    # ------------------------------------------------------------------

    def _handle_find_object(self, cmd: Command) -> None:
        """任务链编排：自主探路 → 视觉判定目标 → 确定坐标 → 自动导航 →
        机械臂对接 → 沿探路轨迹返回起点。

        完整流程（贴合「出发探路 → 找目标 → 抓取 → 返回」实战场景）：
         1. ``recon`` 自主巡游：无目标坐标时，LiDAR 选开阔可通行方向
            低速前进，边巡游边跑反射强度检测，同时自动记录轨迹；
            找不到目标且超时/超距 → 沿轨迹原路返回并上报失败
         2. 检测到目标 → 换算成里程系坐标（``target_to_odom``）
         3. 自动导航到目标点（复用 ``Navigator.goto``）
         4. 若 ``approach=True``，进入对接状态机：对准 → 逼近 → 停稳
            → 推送 ``grasp_ready`` 事件（机械臂才可抓取）
         5. 若配置了机械臂信号通道（arm_grasp_signal），车体停稳等待
            「抓取完成」信号（holding → grasped）；未配置则直接进入返回
         6. 完成后沿探路轨迹反向返回起点（轨迹不可用则回退 go_home）
        与 move / 轨迹回放 / task / goto 互斥。
        """
        if self._detector is None or self._navigator is None or self._controller is None:
            logger.warning("find_object rejected: vision/navigation not ready")
            self._send_event("fault", "find_object: 视觉/导航未启用（雷达未就绪）")
            return
        lidar = self._lidar
        if lidar is None or not lidar.is_receiving:
            logger.warning("find_object rejected: lidar offline")
            self._send_event("fault", "find_object: 雷达离线，无法视觉检测")
            return

        # 互斥：停掉一切正在驱动底盘的运动源
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        self._cancel_task("find_object")
        nav = self._navigator
        if nav.is_navigating:
            nav.stop()
        self._stop_existing_mission("new find_object")
        with self._lock:
            self._pending_command = None
            self._move_deadline = None

        name = cmd.name or "target"
        with self._lock:
            self._mission_stop_event = threading.Event()
            self._mission = {"status": "recon", "target": name}
            self._current_target = None
        self._recon_track = None
        self._target_memory = None
        # 探路期间拉长伪地图 TTL，避免 5s 窗口把刚扫过的走廊清掉
        grid = self._occ_grid
        if grid is not None:
            self._occ_ttl_saved = grid.ttl_s
            grid.set_ttl(max(float(self._recon_max_duration_s), 60.0))
        # 清除上一轮任务的抓取信号（早到的 grasp_done 在进入等待后立即生效）
        if self._arm_bridge is not None:
            self._arm_bridge.reset()
        self._mission_thread = threading.Thread(
            target=self._find_object_loop,
            args=(name, cmd.approach),
            name="find-object",
            daemon=True,
        )
        self._mission_thread.start()
        logger.info("find_object: 开始自主探路搜寻 '%s' (approach=%s)", name, cmd.approach)
        self._send_event("find_object", f"开始自主探路，搜寻目标: {name}")

    def _find_object_loop(self, name: str, do_approach: bool) -> None:
        det = self._detector
        lidar = self._lidar
        ctrl = self._controller
        nav = self._navigator
        stop = self._mission_stop_event
        patrol = self._patrol
        if det is None or lidar is None or ctrl is None or nav is None:
            return
        track_for_return: Optional[Track] = None
        try:
            # ---- 1. 自主探路（巡游搜索）：无目标坐标时边巡游边检测，
            #          同时自动记录轨迹供返回复用 ----
            est, track_for_return = self._recon_search(
                name, det, lidar, ctrl, patrol, stop)
            if stop.is_set():
                return
            if est is None:
                self._set_mission("failed", target=name, detail="探路未找到目标")
                self._send_event("find_object",
                                 f"自主探路未发现目标 {name}，准备沿轨迹返回起点")
                self._return_home(name, track_for_return, stop)
                return

            ctrl.stop_motion()
            self._set_mission("found", target=name)
            logger.info("find_object: 检测到 %s — %s", name, est.summary())
            self._send_event("find_object", f"检测到目标 {name}: {est.summary()}")

            # ---- 2. 换算坐标并导航到停靠点（不是目标中心，避免开进物体）----
            pose = nav.pose
            cx, cy = target_to_odom(pose.x, pose.y, pose.yaw, est)
            standoff_m = 0.8
            if do_approach and self._approach is not None:
                standoff_m = max(0.3, float(self._approach.config.approach_distance_m))
            gx, gy = self._standoff_goal(pose.x, pose.y, cx, cy, standoff_m)
            # 目标结构化上报：方位/表面距离/半径估计/中心距离/里程系中心坐标。
            # 云端可据此在点云/地图视图叠加目标标记，验证视觉检测是否正确。
            self._current_target = {
                "bearingDeg": round(est.bearing_deg, 1),
                "distanceM": round(est.distance_m, 2),
                "radiusM": round(est.radius_estimate_m, 2),
                "centerDistanceM": round(est.center_distance_m, 2),
                "centerX": round(cx, 2),
                "centerY": round(cy, 2),
                "standoffX": round(gx, 2),
                "standoffY": round(gy, 2),
            }
            self._set_mission("found", target=name,
                              extra={"targetEst": self._current_target})
            self._send_event_data("target", self._current_target)
            self._set_mission("navigating", target=name, goal=(gx, gy))
            abort_reason: list[str] = [""]
            waypoints = self._plan_waypoints(gx, gy)
            ok = nav.goto(
                gx, gy,
                waypoints=waypoints,
                on_abort=lambda reason: abort_reason.__setitem__(0, reason),
                replanner=self._replan_waypoints,
            )
            if not ok:
                self._fail_and_return(
                    name, "导航无法启动（已有导航在跑？）",
                    track_for_return, stop)
                return
            # 导航途中：仅在需要对接时做视觉伺服提前介入——目标进入近距离
            # 视野即切视觉闭环。不对接时必须把 goto 走完，否则会在 1.2 m 外
            # 停车然后直接返航，等于没到达。
            handoff = False
            while not stop.is_set():
                if not nav.is_navigating:
                    break
                if do_approach:
                    est_now = self._detect_target(det, lidar, name)
                    if (
                        est_now is not None
                        and est_now.distance_m <= self._visual_handoff_m
                        and abs(_wrap_deg(est_now.bearing_deg))
                            <= self._visual_handoff_deg
                    ):
                        handoff = True
                        break
                stop.wait(0.2)
            if stop.is_set():
                return
            if handoff:
                nav.stop()
                self._send_event(
                    "find_object",
                    f"目标 {name} 已进入视觉伺服范围，提前切换对接",
                )
            elif abort_reason[0]:
                self._fail_and_return(
                    name, abort_reason[0], track_for_return, stop,
                    event_msg=f"导航失败: {abort_reason[0]}，沿轨迹返回起点")
                return

            # ---- 3. 可选机械臂对接（录制继续——轨迹延伸到对接终点）----
            if do_approach:
                self._set_mission("approaching", target=name)
                result = self._approach_loop(det, lidar, ctrl, stop, name)
                if result != "ready":
                    if result == "cancelled" or stop.is_set():
                        return
                    self._fail_and_return(
                        name, result, track_for_return, stop,
                        event_msg=f"对接失败: {result}，沿轨迹返回起点")
                    return
                self._set_mission("ready", target=name)
                self._send_event(EventType.ARRIVED,
                                 f"目标 {name} 已就位（可抓取）")
                logger.info("find_object: %s READY — 机械臂可开始抓取", name)

                # ---- 3.5 等待机械臂抓取完成（抓取完成信号反馈）----
                # 车体就位后停稳等待机械臂完成抓取，收到「抓取完成」信号
                # （云端 grasp_done / CAN 0x3A1 / GPIO 电平 / 联调文件）才返回。
                # 未配置信号通道（默认）时保持旧行为：就位后立即返回。
                grasp = self._wait_arm_grasp(stop, name)
                if grasp == "cancelled":
                    return
                if grasp == "timeout":
                    self._fail_and_return(
                        name, "等待机械臂抓取完成超时",
                        track_for_return, stop,
                        event_msg=(
                            f"等待机械臂抓取完成超时（{self._arm_wait_timeout_s:.0f}s）；"
                            "沿轨迹返回起点。若机械臂实际已抓取可重发 gd"),
                        extra_event="grasp_timeout")
                    return

            # ---- 4. 定格完整轨迹（探路+导航+对接）并沿其返回起点 ----
            full = self._finalize_recon_track()
            if full is not None:
                track_for_return = full
            self._return_home(name, track_for_return, stop)
        except Exception:
            logger.exception("find_object loop error")
            self._set_mission("failed", target=name, detail="find_object 线程异常")
        finally:
            # 中断/异常时也把半成品轨迹定格保存（供排查/后续复用）
            try:
                self._finalize_recon_track()
            except Exception:
                pass
            try:
                ctrl.stop_motion()
            except Exception:
                pass
            with self._lock:
                owns = self._mission_thread is threading.current_thread()
                if owns:
                    self._mission_thread = None
            # 仅本轮任务线程恢复 TTL，避免 join 超时后把下一轮探路的长 TTL 冲掉
            if owns:
                self._restore_occ_ttl()

    def _finalize_recon_track(self) -> Optional[Track]:
        """停止并保存探路录制，返回完整轨迹；未在录制则返回 None。"""
        recorder = self._recorder
        if recorder is None or not recorder.is_recording:
            return None
        try:
            track = recorder.stop()
            if self._player is not None:
                self._player.save_track(track)
            self._recon_track = track
            return track
        except Exception:
            logger.exception("recon: 轨迹录制停止失败")
            return None

    def _restore_occ_ttl(self) -> None:
        """任务链结束：恢复进入任务前保存的 TTL。

        建图模式基线为 0（``_start_lidar`` 已 ``set_ttl(0)``）。
        ``saved is None`` 时保持 0，避免误把累计地图拉回 5s 衰减。
        find_object 探路中仍用临时长 TTL，结束按保存值恢复（通常为 0）。
        """
        grid = self._occ_grid
        saved = self._occ_ttl_saved
        self._occ_ttl_saved = None
        if grid is None:
            return
        try:
            grid.set_ttl(0.0 if saved is None else float(saved))
        except Exception:
            logger.debug("restore occupancy TTL failed", exc_info=True)

    def _standoff_goal(self, x: float, y: float, gx: float, gy: float,
                      standoff_m: float) -> tuple[float, float]:
        """把导航终点从目标中心拉回到停靠距离，避免开进物体。

        ``(gx, gy)`` 是目标中心；返回沿「当前车位 → 中心」方向、距中心
        ``standoff_m`` 的点。已经比停靠距离更近则保持当前位置（交给对接
        状态机微调）。
        """
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist <= max(standoff_m, 1e-6):
            return x, y
        scale = (dist - standoff_m) / dist
        return x + dx * scale, y + dy * scale

    def _fail_and_return(self, name: str, detail: str,
                         track: Optional[Track], stop,
                         event_msg: Optional[str] = None,
                         extra_event: Optional[str] = None) -> None:
        """任务失败仍要返航：溶洞里停在原地等于丢车。"""
        self._set_mission("failed", target=name, detail=detail)
        msg = event_msg or detail
        if extra_event:
            self._send_event(extra_event, msg)
        self._send_event("find_object", msg)
        full = self._finalize_recon_track()
        self._return_home(name, full or track or self._recon_track, stop,
                          done_status="failed")

    def _recon_search(self, name: str, detector, lidar, ctrl,
                      patrol, stop) -> tuple[Optional[object], Optional[Track]]:
        """自主巡游搜索阶段：边巡游边检测目标，同时记录轨迹。

        返回 (目标估计 | None, 轨迹 | None)。
          * 找到目标 → 返回 (est, None)：**录制不停止**，继续贯穿后续
            导航/对接，使轨迹延伸到目标点（返回时才能从目标点原路回去）。
          * 超时/超距未找到 → 返回 (None, track)：录制在此定格保存，
            供「沿原路返回」复用。
        """
        recorder = self._recorder
        track: Optional[Track] = None
        est: Optional[object] = None

        # 记录探路起点位姿（里程系），返回前用它做航向/位置对齐判定
        if self._navigator is not None:
            p = self._navigator.pose
            self._recon_start_pose = Pose2D(p.x, p.y, p.yaw)

        # 巡游开始时自动记录探路轨迹（无需人工驾驶录制）
        try:
            if recorder is not None:
                recorder.start(f"recon_{int(time.time())}")
        except Exception:
            logger.exception("recon: 轨迹录制启动失败")

        # 每次探路必须 start()：上一轮 recon 的 finally 会 patrol.stop()，
        # 不重新 start 则 update() 恒返回 (0,0)，第二次 find_object 会原地发呆。
        if patrol is not None and hasattr(patrol, "start"):
            try:
                patrol.start()
            except Exception:
                logger.exception("recon: patrol.start 失败")

        # 0) 出发前初始 360° 扫描：目标可能在出发方向的另一侧/近处，
        #    先原地全景检测，避免一上来就背向目标走。
        if self._initial_sweep_enabled:
            est = self._initial_sweep(detector, lidar, ctrl, stop, name)
            if stop.is_set():
                return None, None
            if est is not None:
                est = self._confirm_target(detector, lidar, stop, name)
                if est is not None:
                    # 初始扫描即命中 → 巡游未启动，仍统一收尾巡游控制器
                    if patrol is not None:
                        patrol.stop()
                    self._remember_target(est)
                    return est, None

        patrol_ok = patrol is not None and lidar is not None
        recon_until = time.monotonic() + self._recon_max_duration_s
        recon_origin = self._recon_start_pose or Pose2D()
        last_detail_t = 0.0
        last_scan_pause_t = time.monotonic()

        try:
            while not stop.is_set():
                # 每帧检测目标（无论是否在移动）
                est = self._detect_target(detector, lidar, name)
                if est is not None:
                    # 停车多帧确认：连续 N 帧在**同一区域**检测到目标才算确认，
                    # 过滤扬尘/噪点造成的单帧误检。确认失败则忽略并继续巡游。
                    est = self._confirm_target(detector, lidar, stop, name)
                    if est is not None:
                        self._remember_target(est)
                        break

                # 周期性扫描停顿：移动中扫掠角/遮挡容易漏检目标，每隔
                # 一段时间停下列车原地扫过 ±swing/2，逐帧检测后再继续巡游。
                now_mono_sweep = time.monotonic()
                if now_mono_sweep - last_scan_pause_t >= self._scan_pause_every_s:
                    last_scan_pause_t = now_mono_sweep
                    est = self._scan_pause(
                        detector, lidar, ctrl, stop, name,
                        swing_deg=self._scan_pause_swing_deg)
                    if stop.is_set():
                        return None, None
                    if est is not None:
                        est = self._confirm_target(detector, lidar, stop, name)
                        if est is not None:
                            self._remember_target(est)
                            break

                # 周期更新任务详情（探索覆盖率/直线距离），供云端可视化
                now_mono = time.monotonic()
                if now_mono - last_detail_t >= 2.0:
                    last_detail_t = now_mono
                    detail_bits = []
                    if self._navigator is not None:
                        p = self._navigator.pose
                        dist = math.hypot(p.x - recon_origin.x,
                                          p.y - recon_origin.y)
                        detail_bits.append(f"直距{dist:.1f}m")
                    if patrol is not None and hasattr(patrol, "explored_cell_count"):
                        detail_bits.append(f"探{patrol.explored_cell_count}格")
                    if detail_bits:
                        self._set_mission("recon", target=name,
                                          detail="/".join(detail_bits))

                # 探路超时或直线距离超限 → 停止探路，未找到
                if now_mono > recon_until:
                    logger.info("recon: 探路超时（%.0f s），未找到目标 %s",
                                self._recon_max_duration_s, name)
                    break
                if self._navigator is not None:
                    p = self._navigator.pose
                    dist = math.hypot(p.x - recon_origin.x, p.y - recon_origin.y)
                    if dist > self._recon_max_distance_m:
                        logger.info("recon: 探路直线距离 %.1f m 超限，未找到目标 %s",
                                    dist, name)
                        break

                if patrol_ok:
                    v, w = patrol.update()
                else:
                    # 兜底：无巡游控制器时原地旋转扫描
                    v, w = 0.0, self._search_angular_rad_s
                self._drive(v, w)
                stop.wait(0.05)
        finally:
            ctrl.stop_motion()
            # 只在未找到目标时定格录制（找到后由调用方在导航/对接后定格）
            if est is None:
                track = self._finalize_recon_track()
            if patrol is not None:
                patrol.stop()
        return est, track

    def _initial_sweep(self, detector, lidar, ctrl, stop,
                       name: str = "target", omega_rad_s: float = 0.6,
                       timeout_s: float = 15.0) -> Optional[object]:
        """出发前原地旋转 360° 全景扫描目标（系统性搜索第 0 步）。

        目标可能在出发方向的另一侧/近处；不先转一圈，一上来就可能背向
        目标越走越远。旋转整圈（含 5% 余量）仍未发现 → None。发现即返回
        （调用方负责停车多帧确认）。
        """
        nav = self._navigator
        if nav is None or lidar is None or not lidar.is_receiving:
            return None
        start_yaw = nav.pose.yaw
        target_delta = 2.0 * math.pi * 1.05
        turned = 0.0
        last_yaw = start_yaw
        deadline = time.monotonic() + timeout_s
        while not stop.is_set() and time.monotonic() < deadline:
            est = self._detect_target(detector, lidar, name)
            if est is not None:
                return est
            now_yaw = nav.pose.yaw
            turned += abs(_wrap_angle_rad(now_yaw - last_yaw))
            last_yaw = now_yaw
            if turned >= target_delta:
                break
            self._drive(0.0, omega_rad_s)
            stop.wait(0.05)
        return None

    def _scan_pause(self, detector, lidar, ctrl, stop,
                    name: str = "target", swing_deg: float = 180.0,
                    omega_rad_s: float = 0.8) -> Optional[object]:
        """周期性扫描停顿：停下列车，原地扫过 swing_deg 方位区间逐帧检测。

        巡游中目标可能只在特定扫掠角被看见（遮挡/方向性反光），移动中
        容易漏检；每隔一段时间停一下、把探测扇区扫一遍再继续走。
        """
        duration = math.radians(swing_deg) / omega_rad_s
        deadline = time.monotonic() + duration
        while not stop.is_set() and time.monotonic() < deadline:
            est = self._detect_target(detector, lidar, name)
            if est is not None:
                return est
            self._drive(0.0, omega_rad_s)
            stop.wait(0.05)
        return None

    def _remember_target(self, est) -> None:
        """记录已确认目标的世界系坐标（里程系），用于跟踪记忆选择。

        相邻帧间同一目标位置连续；有了这个记忆，后续检测更倾向选「同一个」
        目标而不是远处突然闪现的亮岩/扬尘噪声。
        """
        nav = self._navigator
        if nav is None:
            return
        try:
            pose = nav.pose
            self._target_memory = target_to_odom(pose.x, pose.y, pose.yaw, est)
        except Exception:
            logger.debug("target memory update failed", exc_info=True)

    def _return_home(self, name: str, recon_track: Optional[Track],
                     stop, *, done_status: str = "done") -> None:
        """抓取/探路完成后沿探路轨迹反向返回起点。

        返回前做 C1-起点对齐：用探路起点位姿 + 轨迹末端期望位姿，对照当前
        里程系位姿——航向偏差大则先原地转到位，位置偏差大（小车不在轨迹
        终点附近）则直接回退 go_home，避免「沿轨迹但整体偏着回去」。
        轨迹播放正常完成（on_complete 触发）→ 返回成功；轨迹缺失或播放
        被障碍中止 → 回退 go_home 直线返回导航原点。
        ``done_status`` 成功返航后的终态（正常任务 ``done``；失败后返航
        仍用 ``failed``，避免云端把「导航失败但车已回家」显示成成功）。
        """
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        # 地图优先返回（建图完成 prefer_map_return=True 后启用）：用全局
        # A* 规划一条更优返回路径，绕开探路时未发现的新障碍，也避免原路
        # 绕远；规划失败则回退沿轨迹 / 直线兜底。
        if self._prefer_map_return:
            return_wps = self._plan_waypoints(0.0, 0.0)
            if return_wps is not None:
                nav = self._navigator
                if nav is not None and not nav.is_navigating:
                    self._set_mission("returning", target=name)
                    self._send_event("find_object", "地图规划返回路径，返回起点")
                    abort_reason: list[str] = [""]
                    nav.goto(0.0, 0.0, waypoints=return_wps,
                             on_abort=lambda r: abort_reason.__setitem__(0, r),
                             replanner=self._replan_waypoints)
                    while not stop.is_set() and nav.is_navigating:
                        stop.wait(0.2)
                    if stop.is_set():
                        return
                    if abort_reason[0]:
                        self._set_mission("failed", target=name,
                                          detail=f"返回失败: {abort_reason[0]}")
                        self._send_event("find_object", f"返回起点失败: {abort_reason[0]}")
                        return
                    self._set_mission(done_status, target=name)
                    self._send_event(EventType.ARRIVED, "已返回起点（任务完成）")
                    return

        player = self._player
        ctrl = self._controller
        track_ok = player is not None and ctrl is not None \
            and recon_track is not None and recon_track.waypoints
        if track_ok:
            # C1-起点对齐：只在具备公共里程基准（探路起点位姿）时启用
            if not self._align_return_heading(recon_track, stop):
                # 位置偏差过大 → 不再沿轨迹，直接 go_home 兜底
                self._send_event("find_object", "当前位置偏离探路轨迹终点，回退直线返回")
                track_ok = False
        if track_ok:
            finished: list[bool] = [False]
            try:
                reversed_track = recon_track.reversed()
                self._set_mission("returning", target=name)
                self._send_event("find_object", "开始沿探路轨迹返回起点")
                player.play_async(
                    reversed_track,
                    on_complete=lambda complete: finished.__setitem__(0, bool(complete)),
                )
                while not stop.is_set() and player.is_playing:
                    stop.wait(0.2)
                if stop.is_set():
                    return
                if finished[0]:
                    # 轨迹播放完整走完 → 已回到探路起点
                    self._set_mission(done_status, target=name)
                    self._send_event(EventType.ARRIVED,
                                     "已沿探路轨迹返回起点（任务完成）")
                    return
                # 播放被障碍中止 → 回退直线返回
                self._send_event("find_object", "轨迹返回被中断，回退直线返回起点")
            except Exception:
                logger.exception("沿轨迹返回失败，回退 go_home")
        else:
            self._send_event("find_object", "无可用轨迹，直线返回起点")

        # 兜底：直线返回导航原点 (0, 0)
        nav = self._navigator
        if nav is None or ctrl is None:
            return
        # 上一阶段（goto/对接）若未停干净，绝不能静默跳过返航
        if nav.is_navigating:
            nav.stop()
        abort_reason: list[str] = [""]
        nav.goto(0.0, 0.0, on_abort=lambda r: abort_reason.__setitem__(0, r),
                 replanner=self._replan_waypoints)
        while not stop.is_set() and nav.is_navigating:
            stop.wait(0.2)
        if stop.is_set():
            return
        if abort_reason[0]:
            self._set_mission("failed", target=name, detail=f"返回失败: {abort_reason[0]}")
            self._send_event("find_object", f"返回起点失败: {abort_reason[0]}")
            return
        self._set_mission(done_status, target=name)
        self._send_event(EventType.ARRIVED, "已返回起点（任务完成）")

    def _align_return_heading(self, recon_track: Track, stop) -> bool:
        """返回前航向/位置对齐判定（C1）。返回 False 表示位置偏差过大。

        期望返回起点位姿 = 探路起点位姿 + 轨迹末端期望位姿（相对轨迹起点）。
        把当前里程系位姿换算到同一基准后比较：
          * 位置偏差 > 0.5 m → 小车不在轨迹终点附近，沿轨迹会错位 → False
          * 航向偏差 > 20°  → 先原地转到一致再走
        """
        ctrl = self._controller
        nav = self._navigator
        if ctrl is None or nav is None:
            return True
        start = self._recon_start_pose
        if start is None:
            return True  # 无基准（无导航器）→ 不判，直接走

        end = self._track_end_pose(recon_track)
        # 轨迹末端在里程系中的期望位姿 = 起点位姿 + 轨迹相对末端位姿
        exp_x = start.x + end.x * math.cos(start.yaw) - end.y * math.sin(start.yaw)
        exp_y = start.y + end.x * math.sin(start.yaw) + end.y * math.cos(start.yaw)
        exp_yaw = _wrap_angle_rad(start.yaw + end.yaw)

        now = nav.pose
        d_pos = math.hypot(now.x - exp_x, now.y - exp_y)
        if d_pos > 0.5:
            logger.warning(
                "return: 当前位置 (%.2f, %.2f) 偏离轨迹终点 (%.2f, %.2f) 达 %.2f m，"
                "放弃沿轨迹返回", now.x, now.y, exp_x, exp_y, d_pos,
            )
            return False

        d_yaw = _wrap_angle_rad(exp_yaw - now.yaw)
        if abs(math.degrees(d_yaw)) > 20.0:
            logger.info("return: 航向偏差 %.1f°，原地转向对齐后再返回",
                        math.degrees(d_yaw))
            align_deadline = time.monotonic() + 6.0
            gain = 0.8
            while not stop.is_set() and time.monotonic() < align_deadline:
                now = nav.pose
                err = _wrap_angle_rad(exp_yaw - now.yaw)
                if abs(math.degrees(err)) <= 3.0:
                    break
                self._drive(0.0,
                            max(-0.6, min(0.6, err * gain)))
                stop.wait(0.05)
            ctrl.stop_motion()
        return True

    def _track_end_pose(self, track: Track) -> Pose2D:
        """轨迹末端相对轨迹起点的期望位姿（同一差速模型积分）。"""
        pose = OdometryPose(self._wheelbase_m)
        for wp in track.waypoints:
            pose.update(wp.left_mm, wp.right_mm)
        return Pose2D(pose.pose.x, pose.pose.y, pose.pose.yaw)

    def _select_target(self, ests: list, name: str):
        """从候选目标中选最可信的一个。

        优先语义名称匹配（DL 通路返回 ``name`` 类别）；其次**跟踪记忆**：
        若已有确认过的目标位置，优先选其 0.8m 邻域内的候选（同一目标位置
        连续，远处闪亮岩石是突发噪声）；否则按置信度 → 命中点数 → 距离近
        的次序取最优。无候选返回 None。
        """
        if not ests:
            return None
        named = [e for e in ests if getattr(e, "name", "") == name]
        pool = named or ests
        if self._target_memory is not None and self._navigator is not None:
            try:
                pose = self._navigator.pose
                near = []
                for e in pool:
                    gx, gy = target_to_odom(pose.x, pose.y, pose.yaw, e)
                    if math.hypot(gx - self._target_memory[0],
                                  gy - self._target_memory[1]) <= 0.8:
                        near.append(e)
                if near:
                    pool = near
            except Exception:
                logger.debug("target memory selection failed", exc_info=True)
        return max(
            pool,
            key=lambda e: (
                getattr(e, "confidence", 0.0),
                getattr(e, "point_count", 0),
                -getattr(e, "distance_m", 0.0),
            ),
        )

    def _detect_target(self, detector: ReflectivityDetector,
                       lidar: AiryLidar, name: str = "target"):
        """取最新帧点云，用检测器找目标；无目标返回 None。"""
        if not lidar.is_receiving:
            return None
        frame = lidar.latest_frame
        if frame is None or not frame.points:
            return None
        ests = detector.detect(frame.points)
        return self._select_target(ests, name)

    def _confirm_target(self, detector: ReflectivityDetector, lidar: AiryLidar,
                        stop, name: str = "target",
                        frames: int = 3, window_s: float = 1.2):
        """停车后多帧确认目标（防误检）。

        ``window_s`` 内累计 ``frames`` 帧都检测到目标，且距离 / 方位保持
        一致（车静止，同一目标应落在同一区域）才返回最后一次估计；
        累计帧数不足或位置跳变 → 判定误检返回 None。用于过滤扬尘 /
        单帧噪点造成的假阳性（不要求逐帧连续，短暂丢帧可容忍）。
        """
        self._drive(0.0, 0.0)
        hits = []
        t0 = time.monotonic()
        while len(hits) < frames and time.monotonic() - t0 < window_s:
            if stop.is_set():
                return None
            est = self._detect_target(detector, lidar, name)
            if est is not None:
                hits.append(est)
            time.sleep(0.05)
        if len(hits) < frames:
            return None
        d0 = hits[0].distance_m
        b0 = hits[0].bearing_deg
        for h in hits:
            if abs(h.distance_m - d0) > 0.25:
                return None
            bd = abs(((h.bearing_deg - b0 + 180.0) % 360.0) - 180.0)
            if bd > 10.0:
                return None
        logger.info("find_object: 目标多帧确认通过（%d/%d 帧）", frames, frames)
        return hits[-1]

    def _approach_loop(self, detector, lidar, ctrl, stop,
                       name: str = "target") -> str:
        """对接状态机驱动循环，返回 'ready' / 失败原因 / 'cancelled'。"""
        app = self._approach
        if app is None:
            return "approach 未初始化"
        app.start()
        try:
            while not stop.is_set():
                est = self._detect_target(detector, lidar, name)
                v, w = app.update(est)
                # 对接也必须过避障守卫：视觉伺服不能把车开进岩壁
                if self._guard is not None:
                    v, w, blocked = self._guard.guard_velocity(v, w)
                    if blocked:
                        v = 0.0
                self._drive(v, w)
                if app.state.value in ("ready", "failed", "aborted"):
                    return app.state.value
                stop.wait(app.config.update_interval_s)
        finally:
            app.stop()
            try:
                ctrl.stop_motion()
            except Exception:
                pass
        return "cancelled"

    def _wait_arm_grasp(self, stop, name: str) -> str:
        """机械臂抓取等待，返回 'grasped' / 'timeout' / 'cancelled' / 'disabled'。

        车体已就位（ready）且机械臂可抓取时调用：小车**停稳等待**机械臂完成
        抓取，收到「抓取完成」信号（云端 ``grasp_done`` / CAN / GPIO / 联调
        文件，任一通道）才返回 'grasped'；超时返回 'timeout'；任务被取消
        （cancel / estop）返回 'cancelled'；未配置信号通道返回 'disabled'
        （调用方按旧行为立即返回起点）。
        """
        arm = self._arm_bridge
        if arm is None or not arm.enabled or self._arm_wait_timeout_s <= 0:
            return "disabled"
        self._set_mission("holding", target=name)
        self._send_event("find_object",
                         f"目标 {name} 已就位，等待机械臂抓取完成…")
        try:
            arm.start()
            result = arm.wait_grasp(self._arm_wait_timeout_s, cancel=stop)
        except Exception:
            logger.exception("armfb: 等待机械臂抓取异常，按未启用处理")
            return "disabled"
        finally:
            try:
                arm.stop()
            except Exception:
                pass
        if result.value == "grasped":
            self._set_mission("grasped", target=name)
            self._send_event("grasped",
                             f"机械臂已抓取目标 {name}，开始返回起点")
            logger.info("find_object: %s GRASPED — 开始返回起点", name)
            return "grasped"
        if result.value == "cancelled":
            return "cancelled"
        return "timeout"

    def _handle_grasp_done(self, cmd: Command) -> None:
        """云端/机械臂控制器通知「抓取完成」。

        任务链处于 holding（等待抓取）时置位信号，等待立即结束并返回起点。
        未在等待（如信号早到）时信号被保留，find_object 进入等待后立即生效
        ——配合 ``_handle_find_object`` 里的 ``arm_bridge.reset()`` 语义。
        """
        arm = self._arm_bridge
        if arm is not None and arm.enabled:
            arm.notify_grasped(source="ws")
        self._send_event("grasp_done", "已收到机械臂抓取完成信号")
        logger.info("GRASP_DONE: 机械臂抓取完成信号已确认")

    # -- mission state helpers -------------------------------------------

    def _set_mission(self, status: str, target: str = "",
                     goal: Optional[tuple[float, float]] = None,
                     detail: str = "",
                     extra: Optional[dict] = None) -> None:
        m: dict = {"status": status, "target": target}
        if goal is not None:
            m["goal"] = [round(goal[0], 2), round(goal[1], 2)]
        if detail:
            m["detail"] = detail
        if extra:
            m.update(extra)
        with self._lock:
            self._mission = m
        if status in ACTIVE_MISSION_STATUSES:
            self._maybe_failsafe_checkpoint(force=True, reason=f"phase:{status}")
        elif status in ("done", "failed", "cancelled"):
            self._failsafe.clear_checkpoint()
            self._failsafe.clear_power_return()

    def _stop_existing_mission(self, reason: str) -> None:
        """Abort any running find_object mission (safe to call anytime)."""
        ev = self._mission_stop_event
        if ev is not None:
            ev.set()
        thread = self._mission_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        with self._lock:
            if self._mission is not None and self._mission.get("status") in (
                    "recon", "found", "navigating", "approaching", "holding",
                    "returning"):
                self._mission = dict(self._mission, status="cancelled",
                                     detail=reason)
            # 任务链终结 → 目标标记随之下线（避免云端叠加过期目标）
            self._current_target = None
            self._mission_thread = None
        self._restore_occ_ttl()

    def _handle_go_home(self, cmd: Command) -> None:
        """返回起点：等价 goto(0, 0)。

        这里的 (0, 0) 是**导航原点**——agent 启动（导航栈初始化）时小车
        所在位置，车头方向为 yaw=0°；不是底盘物理上电位置。若 agent 启动
        后小车被搬动，或需要换起点，可用 ``odom_reset`` 指令把原点重置到
        当前位置。
        """
        self._send_event("go_home", "开始返回导航原点 (0, 0)")
        logger.info("GO_HOME: navigating back to origin (0, 0)")
        self._handle_goto(Command(action="goto", x=0.0, y=0.0))

    # ------------------------------------------------------------------
    # Mission / localization remote control  (cancel / odom_reset /
    # map_upload / pose_align / map_return — 无人值守任务与建图接入)
    # ------------------------------------------------------------------

    def _handle_cancel(self, cmd: Command, *, detail: Optional[str] = None) -> None:
        """云端优雅中止：停止当前任务链并自动返回起点（比 estop 温和）。

        语义：停止回放/任务/导航/机械臂对接，若 find_object 已录到轨迹
        （探路/导航阶段）则定格该轨迹并沿其返回起点；无轨迹时仅停车。
        ``detail`` 供断电保护复用本路径时改写任务链说明。
        """
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        self._cancel_task("cancel")
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        with self._lock:
            self._pending_command = None
            self._move_deadline = None
            self._current_target = None
        # 让 find_object 任务链线程退出（不沿用 _stop_existing_mission 的
        # 「就地取消」，这里要保留轨迹做「返回」）
        ev = self._mission_stop_event
        if ev is not None:
            ev.set()
        thread = self._mission_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        with self._lock:
            self._mission_thread = None
        self._restore_occ_ttl()
        ctrl = self._controller
        if ctrl is not None:
            ctrl.stop_motion()

        name = cmd.name or "target"
        track = self._finalize_recon_track()
        if track is None:
            track = self._recon_track
        # 溶洞任务：cancel 必须尝试返航。无轨迹时走直线 go_home，不能停在原地。
        self._set_mission("returning", target=name,
                          detail=detail or "云端 cancel：中止任务并返回")
        stop = threading.Event()
        threading.Thread(
            target=self._return_home,
            args=(name, track, stop),
            name="cancel-return",
            daemon=True,
        ).start()
        if track is not None:
            self._send_event("find_object",
                             "已中止任务，沿已记录轨迹返回起点")
            logger.info("CANCEL: mission aborted, returning along track")
        else:
            self._send_event("find_object",
                             "已中止任务，无探路轨迹，直线返回起点")
            logger.info("CANCEL: mission aborted, returning via go_home")

    def _handle_odom_reset(self, cmd: Command) -> None:
        """远程重置里程系原点（新起点 / 小车被搬动后校正坐标系）。

        重置 navigator 位姿、伪地图、扫描匹配参考、目标记忆与任务链状态，
        使 ``goto``/``find_object`` 的导航原点 (0, 0) 从当前位置重新起算
        （换起点 / 小车被搬动后校正坐标系）。
        """
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        self._cancel_task("odom_reset")
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        self._stop_existing_mission("odom_reset")
        ctrl = self._controller
        if ctrl is not None:
            ctrl.stop_motion()
        with self._lock:
            self._pending_command = None
            self._move_deadline = None
            self._current_target = None
            self._target_memory = None
            self._recon_start_pose = None
        if nav is not None:
            nav.reset_pose()
        if self._occ_grid is not None:
            self._occ_grid.reset()
        if self._planner is not None:
            self._planner.reset_cache()
        # 扫描匹配参考地图随里程原点一起失效
        if self._scan_match_enabled:
            self._scan_matcher = ScanMatcher(config=ScanMatchConfig())
        self._scan_match_last_yaw = None
        self._loc_source = "odom"
        self._loc_last_corr = None
        self._send_event("odom_reset", "里程系原点已重置为当前位置")
        logger.info("ODOM_RESET: pose origin reset to current location")
        self._push_state(force=True)

    def _handle_map_upload(self, cmd: Command) -> None:
        """云端导入全局栅格地图（组员 SLAM 地图交付后的接入通道）。

        ``payload.cells``: [[world_x, world_y, value], ...]，value 取值
        1=自由 / 2=占用 / 3=不可通行（与 occupancy 常量一致）；可带
        ``resolution``（m）切换栅格分辨率重建网格。导入后 A* 膨胀集缓存
        强制重建，``goto`` 预检与全局规划立即可用新地图。
        """
        cells = cmd.cells
        if not cells:
            self._send_event("fault", "map_upload: cells 为空")
            return
        grid = self._occ_grid
        if grid is None:
            self._send_event("fault",
                             "map_upload: 伪地图未初始化（先执行 lidar on）")
            return
        res = cmd.resolution if cmd.resolution > 0.0 else grid.resolution_m
        if res != grid.resolution_m:
            # 分辨率不一致 → 重建栅格与规划器（旧网格数据丢弃，以导入为准）
            self._occ_grid = OccupancyGrid(resolution_m=res)
            self._planner = GlobalPlanner(self._occ_grid)
            grid = self._occ_grid
            logger.info("MAP_UPLOAD: rebuilding grid at resolution %.3f m", res)
        parsed: list[tuple[float, float, int]] = []
        valid = {FREE, OCCUPIED, BLOCKED}
        for row in cells:
            if not isinstance(row, (list, tuple)) or len(row) < 3:
                continue
            try:
                x, y, value = float(row[0]), float(row[1]), int(row[2])
            except (TypeError, ValueError):
                continue
            if value not in valid:
                continue
            parsed.append((x, y, value))
        if not parsed:
            self._send_event("fault", "map_upload: 没有合法的栅格单元")
            return
        grid.import_map(parsed)
        if self._planner is not None:
            self._planner.reset_cache()
        self._send_event_data("map_upload", {
            "imported": len(parsed),
            "resolution": res,
            "totalCells": grid.cell_count,
        })
        self._send_event("info",
                         f"地图已导入 {len(parsed)} 格（栅格共 {grid.cell_count} 格）")
        logger.info("MAP_UPLOAD: imported %d cells (resolution %.3f m, total %d)",
                    len(parsed), res, grid.cell_count)

    def _handle_pose_align(self, cmd: Command) -> None:
        """设置 地图系↔里程系 标定（建图完成后由云端下发）。

        把「地图原点在里程系中的位姿 (x, y, yawDeg)」写入
        :class:`MapAlignment`——通常把车停在地图已知点，读两坐标系位姿
        求差后下发。此后 ``pose_source`` 提供的地图系位姿会被正确换算。
        """
        self._map_alignment.origin_x = cmd.x
        self._map_alignment.origin_y = cmd.y
        self._map_alignment.origin_yaw_deg = cmd.yaw_deg
        self._send_event(
            "pose_align",
            f"地图对齐已设置: origin=({cmd.x:.2f}, {cmd.y:.2f}) "
            f"yaw={cmd.yaw_deg:.1f}°",
        )
        logger.info("POSE_ALIGN: map origin in odom frame set to "
                    "(%.2f, %.2f) yaw=%.1f°", cmd.x, cmd.y, cmd.yaw_deg)

    def _handle_map_return(self, cmd: Command) -> None:
        """运行时切换「地图优先返回」（建图完成后启用，免重启）。"""
        self._prefer_map_return = bool(cmd.map_return)
        state = "开启" if self._prefer_map_return else "关闭"
        self._send_event("map_return", f"地图优先返回已{state}")
        logger.info("MAP_RETURN: prefer_map_return=%s",
                    self._prefer_map_return)

    # ------------------------------------------------------------------
    # LiDAR remote control  (lidar_on/off/status/map — 云端雷达命令)
    # ------------------------------------------------------------------

    def _handle_lidar_on(self, cmd: Command) -> None:
        """云端单独命令「开启雷达」：启动 UDP/PCAP 数据源 + 避障/导航/伪地图。

        幂等：雷达已在线直接确认；掉线则「拆除 → 重建」。启动时即使以
        ``--no-lidar`` 关闭，操作员也可在此处运行中把雷达打开。
        判定「开启成功」按是否有数据：给首帧一个预热窗口（启动瞬间
        ``is_receiving`` 必然为 False，直接判失败是误报）。
        """
        # 与正在驱动底盘的任务互斥：雷达栈重建期间不保留任何运动源
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        self._cancel_task("lidar_on")
        self._stop_existing_mission("lidar_on")
        if not self._start_lidar():
            # 绑定失败（端口被占等）：_start_lidar 已推送带具体原因的 fault
            return
        lidar = self._lidar
        if lidar is not None and self._wait_lidar_online():
            self._send_event("lidar", "雷达已开启并在线")
        else:
            frames = lidar.frame_count if lidar else 0
            packets = lidar.packet_count if lidar else 0
            bad = lidar.bad_packet_count if lidar else 0
            if packets == 0:
                hint = (
                    f"当前收到 0 个 UDP 包 → 数据根本没到本机：用雷达配置工具把"
                    f"数据目标 IP/端口设为本机 {self._lidar_host or '静态IP'}:"
                    f"{self._lidar_port}（RoboSense 雷达是主动发包方），核对网线/网段，"
                    "关闭 Windows 防火墙对 UDP 的拦截"
                )
            elif frames == 0:
                hint = (
                    f"已收到 {packets} 包但解析不出帧（无效包 {bad}）→ 可能是"
                    "非 Airy MSOP 数据源占用了该端口，或端口填错"
                )
            else:
                hint = f"已收到 {packets} 包/解析 {frames} 帧，但最近 3 秒无新帧（雷达可能已停发）"
            self._send_event(
                "fault",
                f"雷达已启动（UDP 端口已绑定）但收不到数据：{hint}",
            )

    def _handle_lidar_off(self, cmd: Command) -> None:
        """云端单独命令「关闭雷达」：停数据源，避障降级为无传感器模式。"""
        player = self._player
        if player is not None and player.is_playing:
            player.stop()
        self._cancel_task("lidar_off")
        self._stop_existing_mission("lidar_off")
        self._teardown_lidar_source()
        self._send_event("lidar", "雷达已关闭，避障降级为无传感器模式")

    def _handle_lidar_status(self, cmd: Command) -> None:
        """雷达/避障自检：在线状态、前方障碍、地形通过性、履带侧隙、低矮碎石。"""
        lidar = self._lidar
        online = bool(lidar is not None and lidar.is_receiving)
        latest = getattr(lidar, "latest_frame", None) if lidar is not None else None
        info: dict[str, Any] = {
            "online": online,
            "source": "pcap" if self._lidar_pcap else "udp",
            "frames": lidar.frame_count if lidar else 0,
            "packets": lidar.packet_count if lidar else 0,
            "badPackets": lidar.bad_packet_count if lidar else 0,
            "pointCount": len(latest.points) if latest is not None else 0,
            "difop": bool(getattr(lidar, "using_difop_calibration", False)),
            "mount": {
                "yawDeg": round(self._lidar_mount_yaw_deg, 1),
                "pitchDeg": round(self._lidar_pitch_deg, 1),
                "heightM": self._lidar_height_m,
                "selfMask": self._lidar_self_mask,
            },
        }
        if self._guard is not None:
            if online:
                front = self._guard.forward_distance(0.2, 0.0)
                body = self._guard.body_clearance(0.2)
                near, near_kind = self._guard.near_collision()
                info["front"] = {
                    "obstacleDistance": round(front, 3) if front is not None else None,
                    "bodyDistance": round(body, 3) if body is not None else None,
                    "nearDistance": round(near, 3) if near is not None else None,
                    "nearKind": near_kind or None,
                }
                info["terrain"] = self._guard.front_terrain_summary()
            # 避障自检：给一条 (0.2, 0.0) 指令，看守卫放行多少/是否急停
            v_safe, w_safe, blocked = self._guard.guard_velocity(0.2, 0.0)
            info["avoidanceTest"] = {
                "requested": [0.2, 0.0],
                "granted": [round(v_safe, 3), round(w_safe, 3)],
                "blocked": blocked,
            }
            left, right = self._guard.track_side_clearance()
            info["tracks"] = {
                "left": round(left, 3) if left is not None else None,
                "right": round(right, 3) if right is not None else None,
            }
            low_count, low_nearest = self._guard.front_low_obstacles()
            info["lowObstacles"] = {
                "count": low_count,
                "nearest": round(low_nearest, 3) if low_nearest is not None else None,
            }
        # 离线时给出数据链路诊断，帮云端直接判断问题在哪一层
        if not online and lidar is not None:
            if lidar.packet_count == 0:
                info["diagnosis"] = (
                    "0 包到达：雷达未向本机发包。用雷达配置工具把数据目标 IP/端口"
                    "设为本机 IP:6699（RoboSense 是主动发包方），核对网线/网段/防火墙"
                )
            elif lidar.frame_count == 0:
                info["diagnosis"] = (
                    f"已收到 {lidar.packet_count} 包但解析不出帧（无效包 "
                    f"{lidar.bad_packet_count}）：端口可能是别的数据源，或非 Airy 格式"
                )
            else:
                info["diagnosis"] = (
                    f"已解析 {lidar.frame_count} 帧，但最近 3 秒无新帧：雷达可能停发"
                )
        self._send_event_data("lidar_status", info)

    def _handle_lidar_map(self, cmd: Command) -> None:
        """返回雷达 2D 伪地图（占用格 + 不可通行格 + ASCII 局部视图）。

        ``cmd.x/y`` 非零时附带目标点预检 ``targetCheck``——设计 goto 前
        先确认目标格是否可通行。
        ``cmd.include_free`` 时快照附带自由格坐标（云端地图模式渲染）；
        ``cmd.silent`` 会回写进事件数据，供云端控制台跳过逐帧打印。
        """
        grid = self._occ_grid
        lidar = self._lidar
        nav = self._navigator
        if grid is None:
            self._send_event("fault", "雷达地图未就绪（先执行 lidar on）")
            return
        if lidar is not None and nav is not None:
            self._update_occupancy_grid(lidar, nav)
        pose = nav.pose if nav is not None else Pose2D()
        data = grid.snapshot(
            pose.x,
            pose.y,
            pose.yaw_deg,
            online=bool(lidar is not None and lidar.is_receiving),
            include_free=bool(cmd.include_free),
        )
        if cmd.x or cmd.y:
            data["targetCheck"] = grid.check_target(cmd.x, cmd.y)
        with self._lock:
            target_est = self._current_target
        if target_est is not None:
            # 地图模式叠加当前任务链目标标记（世界系中心坐标 + 半径）
            data["target"] = target_est
        if cmd.silent:
            data["silent"] = True
        self._send_event_data("lidar_map", data)

    def _handle_point_cloud(self, cmd: Command) -> None:
        """返回最新一帧雷达点云快照（等间隔子采样），供云端绘制点云图。

        ``lidar on`` 在线后雷达持续出点云；本命令取最新帧发一份给云端，
        云端可渲染 ASCII 俯视密度图或保存 PNG（见 ``ascii_view``）。
        """
        data = self._collect_point_cloud_data()
        self._send_event_data("point_cloud", data)

    def _handle_pc_stream(self, cmd: Command) -> None:
        """开关点云实时流：``hz>0`` 开启并按该频率推送最新帧，``hz<=0`` 关闭。

        由独立线程 ``_pc_stream_loop`` 按真实频率推送，云端/实时查看器可据此
        持续渲染图形界面（见 ``examples/live_viewer.py``）。
        """
        hz = 0.0 if cmd.hz <= 0 else max(1.0, min(cmd.hz, 10.0))
        with self._lock:
            self._pc_stream_hz = hz
            self._pc_stream_next_at = 0.0
        if hz > 0:
            with self._lock:
                thread = self._pc_stream_thread
            if thread is None or not thread.is_alive():
                t = threading.Thread(
                    target=self._pc_stream_loop, name="agent-pcstream", daemon=True)
                with self._lock:
                    self._pc_stream_thread = t
                t.start()
            self._send_event(
                "info",
                f"点云实时流已开启：每帧最多 {PC_STREAM_MAX_POINTS} 点 @ {hz:.1f} Hz"
                "（可用 view off / pc_stream 0 关闭）",
            )
            logger.info("Point-cloud streaming ON @ %.1f Hz", hz)
        else:
            self._send_event("info", "点云实时流已关闭")
            logger.info("Point-cloud streaming OFF")

    def _pc_stream_loop(self) -> None:
        """按 ``_pc_stream_hz`` 真实频率推送点云帧；关流后进入空闲等待。"""
        while not self._stop_event.is_set():
            with self._lock:
                hz = self._pc_stream_hz
            if hz <= 0:
                self._stop_event.wait(timeout=0.5)
                continue
            self._push_point_cloud_stream()
            self._stop_event.wait(timeout=1.0 / hz)

    def _collect_point_cloud_data(self, streaming: bool = False) -> dict:
        """构建一帧点云快照数据（含位姿），供单发命令与实时流共用。

        ``streaming=True`` 表示该帧来自点云实时流（云端据此做轻量处理，
        避免对每一帧都保存 PNG/打印 ASCII）。
        """
        lidar = self._lidar
        online = bool(lidar is not None and lidar.is_receiving)
        if online and lidar is not None and hasattr(lidar, "point_cloud"):
            try:
                data = lidar.point_cloud(max_points=PC_STREAM_MAX_POINTS,
                                         max_range_m=60.0)
            except Exception:
                logger.exception("point_cloud snapshot error")
                data = {"online": False, "pointCount": 0, "points": []}
        else:
            data = {"online": online, "pointCount": 0, "points": []}
        nav = self._navigator
        pose = nav.pose if nav is not None else Pose2D()
        data["pose"] = {
            "x": round(pose.x, 2),
            "y": round(pose.y, 2),
            "yawDeg": round(pose.yaw_deg, 1),
        }
        with self._lock:
            target_est = self._current_target
        if target_est is not None:
            # 云端点云视图叠加目标标记（中心坐标 + 半径）
            data["target"] = target_est
        if streaming:
            data["streaming"] = True
        return data

    def _push_point_cloud_stream(self) -> None:
        """按 ``_pc_stream_hz`` 节流推送点云帧（仅在线且已开流时发送）。"""
        with self._lock:
            hz = self._pc_stream_hz
            if hz <= 0:
                return
            now = time.time()
            if now < self._pc_stream_next_at:
                return
            self._pc_stream_next_at = now + 1.0 / hz
        data = self._collect_point_cloud_data(streaming=True)
        if data.get("online"):
            self._send_event_data("point_cloud", data, log=False)

    def _update_occupancy_grid(self, lidar, nav) -> None:
        """把最新一帧雷达观测注册进 2D 伪地图（里程系全局累积）。

        障碍边界取自累积扇区图（``sector_points``）；「不可通行」标记
        取自地形剖面（台阶/岩壁/坑沿）。由定位环约 10 Hz 调用，地图随
        小车移动逐步展开。
        """
        grid = self._occ_grid
        if grid is None:
            return
        frame = getattr(lidar, "latest_frame", None)
        if frame is None or not lidar.is_receiving:
            return
        pose = nav.pose if nav is not None else Pose2D()
        try:
            sectors = lidar.sector_points()
        except AttributeError:
            sectors = []
        blocked: list[tuple[float, float]] = []
        try:
            for r in lidar.terrain.all_sectors(self._step_limit_m):
                if r.blocked and r.obstacle_distance_m is not None:
                    blocked.append((r.angle_deg, r.obstacle_distance_m))
        except Exception:
            logger.debug("Terrain sector fetch failed", exc_info=True)
        grid.update(pose.x, pose.y, pose.yaw_deg, sectors, blocked)
        planner = self._planner
        if planner is not None and grid.cell_count != self._occ_cell_count:
            planner.reset_cache()
            self._occ_cell_count = grid.cell_count
        now = time.monotonic()
        if now - self._occ_map_saved_at >= OCC_MAP_SAVE_INTERVAL_S:
            try:
                path = save_occupancy_png(self._occ_grid, out_dir="./maps")
                self._occ_map_saved_at = now
                if path:
                    logger.info("Occupancy map saved: %s", path)
            except Exception:
                self._occ_map_saved_at = now
                logger.debug("occupancy PNG save failed", exc_info=True)

    def _plan_waypoints(self, goal_x: float, goal_y: float) -> Optional[list[tuple[float, float]]]:
        """全局 A* 规划起点→目标，返回中间航点（不含起点与终点本身）。

        规划失败（无地图 / 目标不可达 / 异常）时返回 None，调用方退化为
        直线导航。航点列表供 ``Navigator.goto(waypoints=...)`` 沿折线执行。
        """
        planner = self._planner
        nav = self._navigator
        if planner is None or nav is None:
            return None
        try:
            pose = nav.pose
            path = planner.plan(pose.x, pose.y, goal_x, goal_y)
        except Exception:
            logger.exception("Global path planning failed — falling back to straight line")
            return None
        if not path:
            return None
        # 去掉起点；末端若与最终目标几乎重合也去掉（避免无谓的「到达-推进」循环）
        wps = [(w.x, w.y) for w in path[1:]]
        while wps:
            wx, wy = wps[-1]
            if math.hypot(wx - goal_x, wy - goal_y) < 0.3:
                wps.pop()
            else:
                break
        return wps

    def _replan_waypoints(self, goal_x: float, goal_y: float) -> Optional[list[tuple[float, float]]]:
        """导航途中重规划回调（供 ``Navigator.goto(replanner=...)``）。

        从**当前位置**重新 A* 规划到最终目标，返回中间航点；失败返回 None
        （Navigator 继续反应式绕行，最终由 stall 超时兜底）。
        """
        return self._plan_waypoints(goal_x, goal_y)

    def _patrol_frontier_target(self) -> Optional[tuple[float, float]]:
        """巡游探索的 frontier 目标：伪地图的最近未知边界（世界系）。

        建图完成后换成基于 SLAM 地图的 frontier 提取（含明确 unknown 区域），
        即可把盲探升级为主动覆盖。无地图/无导航器返回 None。
        """
        grid = self._occ_grid
        nav = self._navigator
        if grid is None or nav is None:
            return None
        pose = nav.pose
        return grid.find_frontier(pose.x, pose.y)

    # ------------------------------------------------------------------
    # Task flow  (task_submit — Step 5 任务流转)
    # ------------------------------------------------------------------

    def _handle_task_submit(self, cmd: Command) -> None:
        """Execute a point-to-point transport task by replaying a preset
        route — the natural bridge between Step 1 (轨迹克隆: record & replay)
        and Step 2 (智能派遣: the mini-program dispatches a task).

        The task is "drive the recorded track referenced by trackId" (or a
        waypoints list in Track format).  Lifecycle is reported to the cloud
        via ``state.payload.task`` and the ``task`` / ``arrived`` events.
        """
        player = self._player
        if player is None or self._controller is None:
            return

        if not cmd.task_id:
            logger.warning("Task submit: missing taskId")
            self._send_event("fault", "task_submit: missing taskId")
            return

        # 409 semantics (manual §5.4): a task is already executing — reject.
        with self._lock:
            active = self._task is not None and self._task.get("status") == "running"
        if active:
            self._send_event("fault", f"Task rejected (busy): {self._task['task_id']}")
            logger.warning("Task submit: busy with task %s, rejecting %s", self._task["task_id"], cmd.task_id)
            return

        track = None
        if cmd.track_id:
            if "/" in cmd.track_id or "\\" in cmd.track_id or ".." in cmd.track_id:
                self._send_event("fault", f"Invalid trackId: {cmd.track_id}")
                return
            try:
                track = player.load_track(cmd.track_id)
            except FileNotFoundError:
                self._send_event("fault", f"Task rejected: track not found: {cmd.track_id}")
                return
            except ValueError:
                self._send_event("fault", f"Task rejected: corrupt track: {cmd.track_id}")
                return
        elif cmd.waypoints:
            # Accept a self-contained task: waypoints in Track format
            # (t/left_mm/right_mm/v/w) — reuse the tolerant parser.
            track = Track.from_json(
                {
                    "name": f"task_{cmd.task_id}",
                    "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "total_duration_s": float(cmd.waypoints[-1].get("t", 0.0)) if cmd.waypoints else 0.0,
                    "waypoints": cmd.waypoints,
                }
            )
            if not track.waypoints:
                self._send_event("fault", f"Task rejected: no usable waypoints for {cmd.task_id}")
                return
        else:
            self._send_event("fault", "task_submit: needs trackId or waypoints")
            return

        if player.is_playing:
            logger.warning("Task submit: stopping current playback first")
            player.stop()
        # And any ongoing goto navigation.
        nav = self._navigator
        if nav is not None and nav.is_navigating:
            nav.stop()
        # And any find_object mission.
        self._stop_existing_mission("task_submit")

        with self._lock:
            self._task = {
                "task_id": cmd.task_id,
                "track_id": cmd.track_id or track.name,
                "status": "running",
                "progress": 0.0,
            }
        self._send_event("task", f"Task accepted: {cmd.task_id} (route: {track.name})")
        logger.info("Task %s started — replaying track '%s'", cmd.task_id, track.name)
        player.play_async(
            track,
            on_complete=lambda complete: self._on_task_track_complete(cmd.task_id, complete),
        )

    def _on_task_track_complete(self, task_id: str, complete: bool) -> None:
        """Mark the task as complete when its route replay finishes.

        只对完整覆盖全程的回放上报完成；残缺/被中止的回放上报 fault，避免
        云端误以为任务全程走完（与 fb 回程基准同理）。
        """
        if not complete:
            with self._lock:
                task = self._task
                if task is not None and task.get("task_id") == task_id:
                    self._task = dict(task, status="cancelled")
            self._send_event("fault", f"Task interrupted: {task_id} (route not fully covered)")
            return
        with self._lock:
            task = self._task
            if task is None or task.get("task_id") != task_id:
                return
            self._task = dict(task, status="complete", progress=1.0)
        self._send_event(EventType.ARRIVED, f"Task completed: {task_id}")

    def _cancel_task(self, reason: str) -> None:
        """Mark the running task as cancelled (manual move / estop / new replay)."""
        with self._lock:
            task = self._task
            if task is None or task.get("status") != "running":
                return
            self._task = dict(task, status="cancelled")
        self._send_event("task", f"Task cancelled: {task['task_id']} ({reason})")

    # ------------------------------------------------------------------
    # State reporting  (section 5.3 / Step 4)
    # ------------------------------------------------------------------

    def _push_state(self, force: bool = False) -> None:
        with self._lock:
            online = (self._state == AgentState.ONLINE)
        if not online:
            return
        ws = self._get_ws()
        tcp = self._teleop_tcp
        if ws is None and (tcp is None or tcp.clients <= 0):
            return

        snapshot = self._collect_snapshot()
        with self._lock:
            token = self._token

        payload: dict[str, Any] = {
            "battery": snapshot.battery,
            "speed": snapshot.speed,
            "odometer": snapshot.odometer,
            "faultCode": snapshot.fault_code,
        }
        # 底盘控制模式/车辆状态：云端据此发现「遥控器抢占 CAN 指令」。
        # 遥控器开着时（REMOTE_CONTROL）CAN 运动指令无效，状态里直接点名，
        # 避免操作员下发 move/goto 后发现小车没动却不知原因。
        if snapshot.control_mode is not None:
            try:
                mode = ControlMode(snapshot.control_mode)
            except ValueError:
                mode = snapshot.control_mode
            payload["mode"] = control_mode_label(mode)
            payload["modeCode"] = int(mode)
            payload["remoteAlive"] = bool(
                getattr(self._controller, "remote_alive", False)
            )
        if snapshot.vehicle_state is not None:
            try:
                vstate = VehicleState(snapshot.vehicle_state)
            except ValueError:
                vstate = snapshot.vehicle_state
            payload["vehicleState"] = vehicle_state_label(vstate)
            payload["vehicleStateCode"] = int(vstate)
        # CAN 链路健康度：接口、链路状态、是否收到底盘反馈帧。操作员发现
        # 「下发指令小车不动」时，这里能一眼分辨是 CAN 链路问题还是底盘模式
        # 问题——没有反馈帧 = 链路不通（接线/适配器/未 bringup），有别于
        # 遥控模式抢占（有反馈但 mode=遥控）。
        if self._controller is not None:
            link = socketcan_operstate(self._can_channel) \
                if self._can_interface == "socketcan" else "n/a"
            payload["can"] = {
                "channel": self._can_channel,
                "interface": self._can_interface,
                "link": link,
                "feedback": self._controller.latest_status is not None,
            }
        # LiDAR avoidance / navigation status (optional fields).
        nav = self._navigator
        if nav is not None:
            pose = nav.pose
            payload["pose"] = {
                "x": round(pose.x, 3),
                "y": round(pose.y, 3),
                "yaw": round(pose.yaw_deg, 2),
            }
            payload["navigating"] = nav.is_navigating
        payload["drive"] = self._drive_status()
        lidar = self._lidar
        if lidar is not None:
            front = None
            if self._guard is not None:
                front = self._guard.forward_distance(0.0, 0.0)
            payload["lidar"] = {
                "online": lidar.is_receiving,
                "frames": lidar.frame_count,
                "frontObstacle": round(front, 3) if front is not None else None,
            }
            # 前方地形通过性（台阶/岩壁/坡/坑）——月球溶洞等复杂路况下云端
            # 需要知道“能不能过去”而不只是“多远有东西”。
            if self._guard is not None:
                payload["terrain"] = self._guard.front_terrain_summary()
            # 2D 伪地图：随状态上报持续注册最新帧（地图逐步展开），并附带
            # 轻量摘要；完整地图由 `lidar_map` 命令拉取。
            if self._occ_grid is not None:
                payload["map"] = self._occ_grid.summary()
        # find_object mission 状态（searching/found/navigating/approaching/
        # ready/done/failed/cancelled），云端可实时跟踪任务链进度。
        with self._lock:
            mission = self._mission
            target_est = self._current_target
        if mission is not None:
            # 目标估计随整个任务链持续附带（导航/对接/就位阶段云端也能
            # 在地图视图叠加目标标记，而不只出现在 found 一瞬间）
            if target_est is not None and "targetEst" not in mission:
                mission = dict(mission, targetEst=target_est)
            payload["mission"] = mission
        payload["failsafe"] = self._failsafe.snapshot()
        # 定位健康度：位姿来源（odom/scanmatch/external）+ 最近一次修正量。
        # 云端据此判断「当前导航精度是否可信」（纯里程漂移无界，被扫描匹配
        # /外部 SLAM 修正后才是闭环位姿）。
        payload["localization"] = {
            "source": self._loc_source,
        }
        corr = self._loc_last_corr
        if corr is not None:
            ago = time.monotonic() - corr.get("at", 0.0)
            payload["localization"]["lastCorrection"] = {
                "dx": corr["dx"],
                "dy": corr["dy"],
                "dyawDeg": corr["dyawDeg"],
                "agoS": round(ago, 1),
            }
        # Let the cloud see whether a trajectory is being recorded right now
        # (the mock_cloud shows a ●REC tag so the operator gets immediate
        # visual confirmation that track_record actually started).
        recorder = self._recorder
        payload["recording"] = bool(recorder is not None and recorder.is_recording)
        if snapshot.motor_temp is not None:
            payload["motorTemp"] = snapshot.motor_temp
        if snapshot.gps is not None:
            payload["gps"] = snapshot.gps
        # Transport task (task_submit) — the manual lists `task` as an
        # optional state field; report it whenever a task exists so the
        # cloud / phone can follow its lifecycle without polling.
        with self._lock:
            task = self._task
        if task is not None:
            if task.get("status") == "running" and self._player is not None:
                task = dict(task, progress=self._player.progress)
            payload["task"] = {
                "taskId": task["task_id"],
                "status": task["status"],
                "progress": round(task["progress"], 3),
            }
        if snapshot.obstacle is not None:
            payload["obstacle"] = snapshot.obstacle
        payload["chassis"] = self._chassis_detail()

        msg = {
            "type": "state",
            "deviceId": self._device_id,
            "ts": int(time.time() * 1000),
            "token": token,
            "payload": payload,
        }
        if ws is not None:
            try:
                ws.send(json.dumps(msg, ensure_ascii=False))
            except websocket.WebSocketConnectionClosedException:
                # 断线窗口内发送失败是预期行为；debug 级别即可，避免每次
                # 状态上报周期都打印完整 traceback。
                logger.debug("State dropped (connection closed)")
            except Exception:
                logger.exception("Failed to send state")
        # 网页 100Hz stick 会一直刷新 stick_age。以前这里直接 return，
        # 导致 query / 底盘状态永远到不了浏览器。主动查询必须下发；
        # 空闲仍按 2s 推一帧；move/goto 执行中 0.4s 推一次，给页面倒计时。
        drip_s = 0.4 if payload.get("drive", {}).get("kind") not in (None, "idle") else 2.0
        if (
            self._local_mode
            and tcp is not None
            and tcp.stick_age_s() < 2.0
            and not force
        ):
            if time.monotonic() - self._last_tcp_state_at < drip_s:
                return
        if self._local_mode and tcp is not None:
            self._last_tcp_state_at = time.monotonic()
        self._emit_tcp(msg)

    def _send_event(self, event: str, msg_text: str) -> None:
        """Send an event immediately — requires agent to be ONLINE."""
        with self._lock:
            online = (self._state == AgentState.ONLINE)
        if not online:
            return
        self._send_event_inline(event, msg_text)

    def _send_event_inline(self, event: str, msg_text: str) -> None:
        """Send an event message immediately (no state guard — caller checks)."""

        # Dedup: don't re-send the same event type within 30 s.
        # Guarded by _lock — this method runs from multiple threads
        # (state-report loop, ws callback, playback thread).
        # ARRIVED is excluded: in round-trip mode the forward and reverse
        # replays both fire `arrived` and may finish within < 30 s of each
        # other — deduping would swallow the "back at start" signal and the
        # cloud would never learn the vehicle returned.
        now = time.time()
        # 按「事件类型 + 文案」去重：同一种 fault 刷屏仍压住，
        # 但 move:/goto: 拒绝原因不能被更早的 obstacle 吞掉。
        dedup_key = f"{event}|{msg_text}"
        if event not in (EventType.OFFLINE, EventType.ARRIVED, "move", "goto"):
            with self._lock:
                last = self._last_event.get(dedup_key, 0)
                if now - last < 30.0:
                    return
                self._last_event[dedup_key] = now

        ws = self._get_ws()
        with self._lock:
            token = self._token

        evt_msg = {
            "type": "event",
            "deviceId": self._device_id,
            "ts": int(time.time() * 1000),
            "token": token,
            "payload": {"event": event, "msg": msg_text},
        }
        self._emit_tcp(evt_msg)
        if ws is None:
            if self._local_mode:
                logger.info("Event (TCP): %s — %s", event, msg_text)
            return

        try:
            ws.send(json.dumps(evt_msg, ensure_ascii=False))
            logger.info("Event sent: %s — %s", event, msg_text)
        except websocket.WebSocketConnectionClosedException:
            # 断线窗口内发送必然失败（预期行为，如 OFFLINE 事件在 _on_close 中
            # 触发时连接已关闭）。不打 exception traceback，避免断线时刷屏。
            logger.debug("Event dropped (connection closed): %s — %s", event, msg_text)
        except Exception:
            logger.exception("Failed to send event")

    def _send_event_data(self, event: str, data: dict, *, log: bool = True) -> None:
        """Send a structured event with a JSON payload (bypasses 30 s dedup).

        ``payload.data`` 携带结构化内容（雷达状态 / 2D 伪地图 / 点云帧），
        ``payload.msg`` 为序列化副本，兼容只看 ``event/msg`` 的旧客户端。
        ``log=False`` 用于高频点云流，避免按帧刷日志。
        """
        ws = self._get_ws()
        with self._lock:
            token = self._token
        evt_msg = {
            "type": "event",
            "deviceId": self._device_id,
            "ts": int(time.time() * 1000),
            "token": token,
            "payload": {
                "event": event,
                "msg": json.dumps(data, ensure_ascii=False),
                "data": data,
            },
        }
        self._emit_tcp(evt_msg)
        if ws is None:
            if self._local_mode and log:
                logger.info("Event (TCP): %s (data)", event)
            return
        try:
            ws.send(json.dumps(evt_msg, ensure_ascii=False))
            logger.info("Event sent: %s (data %d bytes)", event, len(evt_msg["payload"]["msg"]))
        except websocket.WebSocketConnectionClosedException:
            logger.debug("Event dropped (connection closed): %s", event)
        except Exception:
            logger.exception("Failed to send event")

    def _chassis_detail(self) -> dict[str, Any]:
        """手册 3.3.2 反馈帧：0x211 / 0x221 / 0x311 / 0x361 / 0x241。"""
        ctrl = self._controller
        if ctrl is None:
            return {"heard": False}
        status = getattr(ctrl, "latest_status", None)
        motion = getattr(ctrl, "latest_motion", None)
        bms = getattr(ctrl, "latest_bms", None)
        odometer = getattr(ctrl, "latest_odometer", None)
        remote = getattr(ctrl, "latest_remote", None)
        if remote is None:
            remote = getattr(ctrl, "_latest_remote", None)

        out: dict[str, Any] = {"heard": status is not None}
        if status is not None:
            raw_fault = int(getattr(status, "fault_code", 0) or 0)
            faults = FaultFlags.from_byte(raw_fault)
            mode = getattr(status, "control_mode", None)
            vstate = getattr(status, "vehicle_state", None)
            try:
                mode_l = control_mode_label(ControlMode(mode))
                mode_c = int(mode)
            except (TypeError, ValueError):
                mode_l, mode_c = (str(mode) if mode is not None else None), None
            try:
                vstate_l = vehicle_state_label(VehicleState(vstate))
                vstate_c = int(vstate)
            except (TypeError, ValueError):
                vstate_l, vstate_c = (str(vstate) if vstate is not None else None), None
            volt = getattr(status, "battery_voltage_v", None)
            out["system"] = {
                "id": "0x211",
                "vehicleState": vstate_l,
                "vehicleStateCode": vstate_c,
                "controlMode": mode_l,
                "controlModeCode": mode_c,
                "batteryVoltageV": None if volt is None else round(float(volt), 2),
                "faultCode": raw_fault,
                "faultHex": f"0x{raw_fault:02X}",
                "faults": faults.active_items(),
                "heartbeat": getattr(status, "count", None),
            }
        if motion is not None:
            out["motion"] = {
                "id": "0x221",
                "linearMs": round(float(motion.linear_velocity_m_s), 3),
                "angularRads": round(float(motion.angular_velocity_rad_s), 3),
            }
        if odometer is not None:
            left = int(odometer.left_wheel_mm)
            right = int(odometer.right_wheel_mm)
            src = getattr(ctrl, "odometer_source", None)
            out["odometer"] = {
                "id": "0x311",
                "leftMm": left,
                "rightMm": right,
                "leftM": round(left / 1000.0, 3),
                "rightM": round(right / 1000.0, 3),
                "source": src() if callable(src) else src,
            }
        if bms is not None:
            out["bms"] = {
                "id": "0x361",
                "socPercent": int(bms.soc_percent),
                "sohPercent": int(bms.soh_percent),
                "voltageV": round(float(bms.voltage_v), 2),
                "currentA": round(float(bms.current_a), 1),
                "temperatureC": round(float(bms.temperature_c), 1),
            }
        if remote is not None:
            out["remote"] = {
                "id": "0x241",
                "alive": bool(getattr(ctrl, "remote_alive", False)),
                "swbCommand": bool(getattr(ctrl, "remote_swb_command", False)),
                "switchBits": int(remote.switch_bits),
                "rightStick": [int(remote.right_stick_lr), int(remote.right_stick_ud)],
                "leftStick": [int(remote.left_stick_lr), int(remote.left_stick_ud)],
                "knobVra": int(remote.knob_vra),
            }
        return out

    def _collect_snapshot(self) -> _StateSnapshot:
        """Collect current chassis state from the controller."""
        ctrl = self._controller
        if ctrl is None:
            return _StateSnapshot()

        status = ctrl.latest_status
        motion = ctrl.latest_motion
        bms = ctrl.latest_bms
        odometer = ctrl.latest_odometer

        battery = 0.0
        speed = 0.0
        odo = 0.0
        fault_code = 0
        motor_temp: Optional[float] = None
        control_mode: Optional[int] = None
        vehicle_state: Optional[int] = None

        if status is not None:
            raw_fault = int(status.fault_code or 0)
            control_mode = status.control_mode
            vehicle_state = status.vehicle_state
            faults = FaultFlags.from_byte(raw_fault)
            # 遥控器关掉后 0x04 会一直在：这是「没用手持遥控」，不是新故障。
            # 云端每 30 s 报一次会误导；真正要报的是欠压/急停/驱动通讯。
            items = faults.active_items(ignore=frozenset({"remote_control_lost"}))
            if items:
                self._send_event(EventType.FAULT, "; ".join(items))
            self._maybe_recover_can_mode(status)
            # 云端状态栏同样不要把 0x04 / 仅因此产生的「系统异常」当故障。
            fault_code = raw_fault & ~0x04
            if (
                fault_code == 0
                and vehicle_state == VehicleState.SYSTEM_EXCEPTION
            ):
                vehicle_state = int(VehicleState.NORMAL)

        if motion is not None:
            speed = motion.linear_velocity_m_s

        if odometer is not None:
            odo = (odometer.left_wheel_mm + odometer.right_wheel_mm) / 2000.0

        if bms is not None:
            battery = bms.soc_percent / 100.0  # doc requires 0~1
            motor_temp = bms.temperature_c
            if bms.soc_percent < 15:
                self._send_event(
                    EventType.LOW_BATTERY,
                    f"Battery SOC={bms.soc_percent}%",
                )

        return _StateSnapshot(
            battery=battery,
            speed=speed,
            odometer=odo,
            fault_code=fault_code,
            motor_temp=motor_temp,
            control_mode=control_mode,
            vehicle_state=vehicle_state,
        )

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _set_state(self, new_state: AgentState) -> None:
        callbacks: list[Callable[[AgentState], None]] = []
        with self._lock:
            if self._state == new_state:
                return
            old = self._state
            self._state = new_state
            callbacks = list(self._state_callbacks)

        logger.info("Agent state: %s → %s", old.value, new_state.value)
        for cb in callbacks:
            try:
                cb(new_state)
            except Exception:
                logger.exception("State callback error")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        self._stop_event.set()
        view = self._map_view
        if view is not None:
            try:
                view.stop()
                view.close()
            except Exception:
                logger.debug("map view stop failed", exc_info=True)
            self._map_view = None
        teleop = self._teleop_tcp
        if teleop is not None:
            try:
                teleop.stop()
            except Exception:
                logger.debug("teleop TCP stop failed", exc_info=True)
            self._teleop_tcp = None
        try:
            self._maybe_failsafe_checkpoint(force=True, reason="shutdown")
        except Exception:
            logger.debug("failsafe checkpoint on shutdown failed", exc_info=True)
        self._close_ws()
        self._stop_background_tasks(cloud_only=False)
        # 停止 find_object mission（若有）
        self._stop_existing_mission("agent shutdown")
        if self._recorder and self._recorder.is_recording:
            try:
                # Shutdown while a recording is in progress (agent killed /
                # Ctrl+C / link dropped for good): save what was sampled so
                # the work is not lost.  The file is never corrupt thanks to
                # the atomic save in TrackPlayer.save_track.
                track = self._recorder.stop()
                if self._player:
                    name = f"{track.name}_interrupted_{time.strftime('%Y%m%d_%H%M%S')}"
                    track.name = name
                    self._player.save_track(track)
                    logger.warning(
                        "Agent stopped mid-recording — saved partial track as '%s' "
                        "(%d waypoints, %.1f s)",
                        name, len(track.waypoints), track.total_duration_s,
                    )
            except Exception as exc:
                logger.warning("Could not save interrupted recording: %s", exc)
        if self._player and self._player.is_playing:
            try:
                self._player.stop()
            except Exception:
                pass
        if self._controller is not None:
            try:
                self._controller.stop_motion()
                self._controller.stop()
            except Exception:
                logger.exception("Controller stop error")
            self._controller = None
        # Shut down navigation and LiDAR last, after motion is fully stopped.
        nav = self._navigator
        if nav is not None:
            try:
                nav.stop()
            except Exception:
                logger.exception("Navigator stop error")
            self._navigator = None
        lidar = self._lidar
        if lidar is not None:
            try:
                lidar.stop()
            except Exception:
                logger.exception("LiDAR stop error")
            self._lidar = None
        logger.info("Agent cleaned up")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

@dataclass
class _StateSnapshot:
    battery: float = 0.0
    speed: float = 0.0
    odometer: float = 0.0
    fault_code: int = 0
    motor_temp: Optional[float] = None
    gps: Optional[dict] = None
    task: Optional[dict] = None
    obstacle: Optional[dict] = None
    control_mode: Optional[int] = None   # protocol.ControlMode 值
    vehicle_state: Optional[int] = None  # protocol.VehicleState 值
