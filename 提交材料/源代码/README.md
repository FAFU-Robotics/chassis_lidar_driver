# 全地形底盘与自主导航 — 提交源代码（过期快照）

**请改用仓库根目录 `lidar_nav/`。** 本目录仍写 ALIGN/DRIVE，跟当前圆弧跟线不一致，只保留作当时提交备份。

对应 PPT：
1. 全地形运动平台与三维环境感知
2. MOLA 实时定位与局部建图
3. 自主导航与避障

测试平台：AgileX Bunker Mini 2.0 + RoboSense Airy 96 线（测试底盘，不是最终比赛底盘）。

第三方定位库：[MOLA](https://github.com/MOLAorg/mola)（本包只含本项目的启动脚本与 Airy 管线配置，不含 MOLA 上游源码）。

运行请使用系统 `/usr/bin/python3`（ROS 2 Humble / Python 3.10），不要用 conda 的 Python 3.13。

---

## 目录与 PPT 对应关系

### 01_三维感知

| 文件 | 作用 |
|---|---|
| `bringup_sense.sh` | 感知栈启动：rslidar_sdk、TF `base_link→rslidar`、轮速听口、MOLA、cloud_to_grid。不发 CAN 控制 |
| `restart_mola_deskew.sh` | 仅重启 MOLA 并打开去畸变扫描发布 |

数据流：RoboSense Airy → `/rslidar_points`（约 10 Hz，XYZIRT）→ 三维点云。

### 02_MOLA定位与局部建图

| 文件 | 作用 |
|---|---|
| `start_mola_mapping.sh` | 正式建图启动（关键帧约 0.25 m / 20°） |
| `start_mola_nav.sh` | 导航里程计启动（关键帧更稀，给跟车用） |
| `lidar3d-icp-airy.yaml` | 本项目 Airy 的 MOLA-LO ICP / localmap 管线 |
| `fastdds_udp.xml` | FastDDS UDP，避免 Jetson SHM 锁失败 |
| `publish_wheel_odom.py` | 只听 CAN 轮速/转速，发布 `/wheel_odom`（不发控制） |
| `mola_pose.py` | 订阅 `/lidar_odometry/pose`，用接收时钟判断过期 |

数据流：点云匹配 → 位姿估计 → `/lidar_odometry/pose`、`/lidar_odometry/localmap_points`（统一到 `map`）。

### 03_栅格规划与避障

| 文件 | 作用 |
|---|---|
| `cloud_to_grid.py` | 点云 + MOLA pose → `/nav_demo/obstacle_grid` |
| `occupancy.py` | 二维占用栅格（自由/占用/不可通行，TTL） |
| `ros_grid.py` | ROS OccupancyGrid → 规划器接口 |
| `global_planner.py` | 官方 A*（8 连通 + 膨胀 + 视线拉直） |
| `plan_once.py` | 真实栅格上规划一次，发布 `/nav_demo/plan`，不控车 |
| `official_mods.py` | 加载 occupancy / A* / Navigator / ObstacleGuard |
| `navigator.py` | 航点跟踪（ALIGN/DRIVE） |
| `obstacle.py` | ObstacleGuard 限速/停车 |
| `run_mvp.py` | Grid → A* → Navigator + Guard，默认 DRY_RUN |
| `run_live_go.py` | 短距实车跟线（需明确允许） |
| `run_live_avoid.py` | Guard BLOCK 后 A* 重规划（需明确允许） |

数据流：环境感知 → MOLA 定位 → obstacle_grid → A* → 路径跟踪 → 安全避障。

---

## 最小复现（不控车）

```bash
# 1. 感知 + MOLA + 栅格
bash 01_三维感知/bringup_sense.sh

# 2. 只规划，不发 CAN
/usr/bin/python3 03_栅格规划与避障/plan_once.py --forward 1.8
```

依赖（工控机已装）：ROS 2 Humble、`rslidar_sdk`、`mola_lidar_odometry`。

`plan_once.py` / `run_mvp.py` 默认不发送 stick / CAN。实车脚本必须口头允许后再运行。

---

## 坐标系（请勿改正式 TF）

- 雷达安装：`base_link → rslidar`，z = 0.365 m，roll +90°，yaw +90°
- MOLA：`map → odom → base_link`（REP-105）
- 栅格 `frame_id=map`：100 占用 / 50 膨胀（可视化） / 0 自由 / −1 未知
- A* 规划膨胀 0.20 m；起点车体足迹 0.40 m 内占用格按可通行处理
