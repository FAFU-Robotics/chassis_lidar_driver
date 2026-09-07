# chassis_lidar_drivers

Local console for Bunker Mini 2.0 + RoboSense Airy on Jetson (`run_local.py`, web `:9101`).

BUNKER MINI 2.0 底盘 + RoboSense Airy 雷达项目的工作区根目录。

## 目录分工

| 路径 | 说明 | 维护 |
|------|------|------|
| `bunker_jetson/` | **项目主体**：本地控制台、导航与控制、轨迹录制/回放、雷达避障（云端通道已废弃） | 林 |
| `robosense_airy/` | Airy 雷达驱动（WRS 框架版，含 3D 视图 / ROS2 桥） | 蔡 |
| `bunker_mini/` | Airy 雷达纯 Python 模块（lidar / terrain / obstacle / vision / approach / pcap / ascii_view） | 蔡 |
| `view_lidar.py` / `live_airview.py` / `airy_ros2_bridge.py` | 雷达视图 / 3D 视图 / ROS2 桥 | 蔡 |
| `teleop_desktop.py` / `start_teleop_client.*` | 网页控制台桌面客户端（pywebview 窗口，缺库则开浏览器） | 林 |
| `teleop_client_laptop/` | 笔记本双击客户端（拷到本机后双击 `.bat`） | 林 |
| `install_local_service.sh` / `bunker-local.service` | 开机无头服务（:9100+:9101，不开雷达窗） | 林 |
| `gs_usb_kmod/` | USB-CAN 内核模块源码与构建产物 | 硬件 |
| `tracks/` | 轨迹数据 | - |
| `使用说明.md` | 工控机联调说明（主文档） | - |

## 重要说明

- **日常入口在仓库根目录**，不要走已废弃的云端：

  ```bash
  python3 run_local.py          # 本地控制台 + 网页 :9101（详见 网页遥控说明.md）
  # 详见 使用说明.md
  ```

- **云端模式已废弃，不建议调用** `mock_cloud.py`、`run_mission.py`、
  `start_agent.sh` 连 WebSocket。后续功能默认接到 `run_local.py`。

- 根目录曾是旧项目，其历史脚本已清理，车侧代码**以 `bunker_jetson/` 为准**。

- `robosense_airy/` 与 `bunker_mini/` 下的雷达相关代码由组员维护，**请勿改动**。
