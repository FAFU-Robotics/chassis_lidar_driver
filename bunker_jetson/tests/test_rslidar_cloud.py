"""rslidar_sdk 点云桥：车体系变换、套接字帧、agent 数据源选择。"""

from __future__ import annotations

import json
import socket
import subprocess
import time
from pathlib import Path

try:
    import pytest
except ImportError:  # pragma: no cover
    pytest = None  # type: ignore

import bunker_mini.agent as agent_mod
from bunker_mini.agent import BunkerMiniAgent, Command, fit_tcp_visual, slim_tcp_visual
from bunker_mini.agent import TCP_PC_MAX_POINTS, TCP_VISUAL_MAX_BYTES
from bunker_mini.lidar import msop_udp_bound
from bunker_mini.rslidar_cloud import (
    BRIDGE_SH,
    decode_cloud_payload,
    encode_cloud_frame,
    rslidar_to_vehicle,
    vehicle_points_from_sensor,
    RslidarCloudSource,
)


def _near(a: float, b: float, tol: float = 1e-6) -> None:
    assert abs(float(a) - float(b)) <= tol, (a, b)


if pytest is not None:
    @pytest.fixture
    def agent(monkeypatch) -> BunkerMiniAgent:
        monkeypatch.setattr(
            agent_mod, "resolve_can_config",
            lambda channel, interface, *, allow_auto_channel=True: ("0", "virtual"),
        )
        a = BunkerMiniAgent(
            ws_url="ws://127.0.0.1:1/test",
            device_id="TEST-01",
            bind_code="TEST-BIND-xxxx",
        )
        yield a
        a.stop()
        a._cleanup()


def test_rslidar_to_vehicle_optical_forward_is_vehicle_y():
    x, y, z = rslidar_to_vehicle(0.0, 0.0, 1.2, height_m=0.365)
    _near(x, 0.0)
    _near(y, 1.2)
    _near(z, 0.365)


def test_rslidar_to_vehicle_sensor_up_is_vehicle_z():
    x, y, z = rslidar_to_vehicle(0.0, 0.8, 0.0, height_m=0.365)
    _near(x, 0.0)
    _near(y, 0.0)
    _near(z, 1.165)


def test_rslidar_to_vehicle_sensor_plus_x_is_left():
    x, y, z = rslidar_to_vehicle(0.4, 0.0, 0.0, height_m=0.0)
    _near(x, -0.4)
    _near(y, 0.0)
    _near(z, 0.0)


def test_rslidar_to_vehicle_extra_yaw_left_90():
    x, y, z = rslidar_to_vehicle(0.0, 0.0, 1.0, height_m=0.0, extra_yaw_deg=90.0)
    _near(x, -1.0)
    _near(y, 0.0)
    _near(z, 0.0)


def test_encode_decode_roundtrip():
    pts = [(0.1, 0.2, 0.3, 12.0), (1.0, 0.0, 2.0, 200.0)]
    raw = encode_cloud_frame(pts)
    assert raw[:4] == b"RSC1"
    got = decode_cloud_payload(raw[8:])
    assert len(got) == 2
    for a, b in zip(got, pts):
        for u, v in zip(a, b):
            _near(u, v)


def test_vehicle_points_skip_nan_and_range():
    pts = vehicle_points_from_sensor(
        [(float("nan"), 0.0, 1.0, 1.0), (0.0, 0.0, 0.01, 1.0), (0.0, 0.0, 1.0, 90.0)],
        height_m=0.0,
    )
    assert len(pts) == 1
    _near(pts[0].y, 1.0)
    _near(pts[0].azimuth_deg, 0.0)


def test_cloud_source_ingest_via_unix_socket(tmp_path: Path):
    sock_path = str(tmp_path / "cloud.sock")
    src = RslidarCloudSource(
        sock_path=sock_path,
        spawn_bridge=False,
        height_m=0.0,
        extra_yaw_deg=0.0,
    )
    src.start()
    cli = None
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                cli = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                cli.connect(sock_path)
                break
            except OSError:
                if cli is not None:
                    cli.close()
                    cli = None
                time.sleep(0.05)
        assert cli is not None, "agent unix socket never accepted"
        cli.sendall(encode_cloud_frame([(0.0, 0.0, 1.0, 80.0)]))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and src.frame_count < 1:
            time.sleep(0.05)
        assert src.frame_count >= 1
        assert src.is_receiving
        dist = src.nearest_in_range(0.0, 40.0)
        assert dist is not None
        _near(dist, 1.0, 0.08)
        snap = src.point_cloud()
        assert snap["online"] is True
        assert snap["pointCount"] >= 1
    finally:
        if cli is not None:
            cli.close()
        src.stop()


def test_rslidar_cloud_bridge_sh_sources_humble():
    """Humble setup.bash 不能在 nounset 下 source，否则网页雷达桥会瞬间 exit 1。"""
    if not Path("/opt/ros/humble/setup.bash").is_file():
        return
    assert BRIDGE_SH.is_file()
    r = subprocess.run(
        ["bash", str(BRIDGE_SH), "--help"],
        capture_output=True, text=True, timeout=12,
    )
    assert r.returncode == 0, r.stderr


def test_msop_udp_bound_detects_listener():
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    port = srv.getsockname()[1]
    try:
        assert msop_udp_bound(port) is True
    finally:
        srv.close()
    assert msop_udp_bound(port) is False


def test_build_lidar_auto_ros_when_msop_busy(agent, monkeypatch):
    sentinel = object()
    monkeypatch.setattr(agent_mod, "msop_udp_bound", lambda port: True)

    def _fake_ros(**kwargs):
        assert kwargs["lidar_height_m"] == 0.0
        return sentinel

    monkeypatch.setattr(agent_mod, "RslidarCloudSource", _fake_ros)
    agent._lidar_pcap = None
    agent._lidar_source = "auto"
    assert agent._build_lidar() is sentinel


def test_build_lidar_auto_udp_when_msop_free(agent, monkeypatch):
    sentinel = object()
    monkeypatch.setattr(agent_mod, "msop_udp_bound", lambda port: False)
    monkeypatch.setattr(agent, "_build_udp_lidar", lambda: sentinel)
    agent._lidar_pcap = None
    agent._lidar_source = "auto"
    assert agent._build_lidar() is sentinel


def test_build_lidar_force_ros_even_if_port_free(agent, monkeypatch):
    sentinel = object()
    monkeypatch.setattr(agent_mod, "msop_udp_bound", lambda port: False)
    monkeypatch.setattr(agent_mod, "RslidarCloudSource", lambda **kwargs: sentinel)
    agent._lidar_pcap = None
    agent._lidar_source = "ros"
    assert agent._build_lidar() is sentinel


def test_lidar_status_source_ros(agent):
    class _L:
        is_receiving = True
        frame_count = 3
        packet_count = 3
        bad_packet_count = 0
        using_difop_calibration = False
        source = "ros"
        latest_frame = None

        def stop(self) -> None:
            pass

        def nearest_in_range(self, *args, **kwargs):
            return 1.0

    agent._lidar = _L()
    agent._guard = None
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_lidar_status(Command(action="lidar_status"))
    assert out[0][1]["source"] == "ros"


def test_slim_tcp_visual_point_cloud_xy_and_cap():
    pts = [[float(i), 1.0, 9.0, 200] for i in range(5000)]
    out = slim_tcp_visual("point_cloud", {
        "online": True, "pointCount": 5000, "points": pts, "text": "ASCII",
    })
    assert "text" not in out
    assert len(out["points"]) <= TCP_PC_MAX_POINTS
    assert out["shown"] == len(out["points"])
    assert out["pointCount"] == 5000
    assert out["points"][0] == [0.0, 1.0]


def test_slim_tcp_visual_drops_ascii_text():
    out = slim_tcp_visual("lidar_map", {
        "online": True,
        "text": "#" * 8000,
        "occupied": [[1.0, 2.0]] * 20,
        "blocked": [],
        "free": [[0.1, 0.2]] * 10,
        "occupiedCells": 20,
        "blockedCells": 0,
        "pose": {"x": 0, "y": 0, "yawDeg": 0},
    })
    assert "text" not in out
    assert out["occupied"]
    assert out["free"]


def test_fit_tcp_visual_stays_under_budget():
    fat = {
        "online": True,
        "text": "X" * 20000,
        "occupied": [[i * 0.1, 0.2] for i in range(4000)],
        "blocked": [[i * 0.1, 1.2] for i in range(4000)],
        "free": [[i * 0.1, 2.2] for i in range(8000)],
        "occupiedCells": 4000,
        "blockedCells": 4000,
        "freeCells": 8000,
    }
    slim = fit_tcp_visual("lidar_map", fat)
    blob = json.dumps(
        {"type": "event", "payload": {"event": "lidar_map", "msg": "x", "data": slim}},
        ensure_ascii=False,
    ).encode("utf-8")
    assert len(blob) <= TCP_VISUAL_MAX_BYTES
    assert "text" not in slim


def test_emit_tcp_forwards_point_cloud(agent):
    pushed = []

    class _Tcp:
        clients = 1

        def push_event(self, msg):
            pushed.append(msg)

    agent._teleop_tcp = _Tcp()
    agent._emit_tcp({
        "type": "event",
        "payload": {
            "event": "point_cloud",
            "msg": json.dumps({"points": [[1, 2, 3, 4]] * 50}),
            "data": {"online": True, "pointCount": 50, "points": [[1, 2, 3, 4]] * 50},
        },
    })
    assert pushed
    data = pushed[0]["payload"]["data"]
    assert data["points"]
    assert pushed[0]["payload"]["msg"].startswith("point_cloud")
    raw = json.dumps(pushed[0], ensure_ascii=False).encode("utf-8")
    assert len(raw) < 60_000


def test_lidar_map_empty_when_grid_missing(agent):
    agent._occ_grid = None
    agent._lidar = None
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_lidar_map(Command(action="lidar_map"))
    assert out and out[0][0] == "lidar_map"
    assert out[0][1]["ready"] is False
    assert out[0][1]["occupied"] == []


if __name__ == "__main__":
    import tempfile

    class _MP:
        def __init__(self) -> None:
            self._restore: list = []

        def setattr(self, obj, name, value) -> None:
            self._restore.append((obj, name, getattr(obj, name)))
            setattr(obj, name, value)

        def undo(self) -> None:
            for obj, name, old in reversed(self._restore):
                setattr(obj, name, old)

    test_rslidar_to_vehicle_optical_forward_is_vehicle_y()
    test_rslidar_to_vehicle_sensor_up_is_vehicle_z()
    test_rslidar_to_vehicle_sensor_plus_x_is_left()
    test_rslidar_to_vehicle_extra_yaw_left_90()
    test_encode_decode_roundtrip()
    test_vehicle_points_skip_nan_and_range()
    with tempfile.TemporaryDirectory() as td:
        test_cloud_source_ingest_via_unix_socket(Path(td))
    test_msop_udp_bound_detects_listener()
    test_rslidar_cloud_bridge_sh_sources_humble()
    test_slim_tcp_visual_point_cloud_xy_and_cap()
    test_slim_tcp_visual_drops_ascii_text()
    test_fit_tcp_visual_stays_under_budget()

    mp = _MP()
    mp.setattr(
        agent_mod, "resolve_can_config",
        lambda channel, interface, *, allow_auto_channel=True: ("0", "virtual"),
    )
    ag = BunkerMiniAgent(
        ws_url="ws://127.0.0.1:1/test",
        device_id="TEST-01",
        bind_code="TEST-BIND-xxxx",
    )
    try:
        test_build_lidar_auto_ros_when_msop_busy(ag, mp)
        test_build_lidar_auto_udp_when_msop_free(ag, mp)
        test_build_lidar_force_ros_even_if_port_free(ag, mp)
        test_lidar_status_source_ros(ag)
        test_emit_tcp_forwards_point_cloud(ag)
        test_lidar_map_empty_when_grid_missing(ag)
    finally:
        ag.stop()
        ag._cleanup()
        mp.undo()
    print("PASS test_rslidar_cloud")
