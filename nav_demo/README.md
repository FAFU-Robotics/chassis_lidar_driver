# nav_demo — 真实感知 → 栅格 → A* → Navigator dry-run

本目录只做挑战杯 Demo / PPT。**不修改**正式 MOLA / SDK / ICP / TF / YAML / `occupancy.py` / `global_planner.py` / `navigator.py` / `obstacle.py`。

**当前禁止（未获你明确允许之前）：** `stick`、CAN 发送、真实底盘运动、`run_obstacle_avoidance.py`、第二条 CAN、Nav2。

## 今天已验证的链路（dry-run）

真实 `/rslidar_points` + MOLA `/lidar_odometry/pose`
→ `cloud_to_grid.py` → `/nav_demo/obstacle_grid`
→ 官方 `GlobalPlanner`（A*）
→ 官方 `Navigator.goto` + `ObstacleGuard`
→ 打印 `DRY_RUN v= w=`
→ **不**调用 `TeleopTcpClient.stick`

底盘入口（实车阶段才会用）：`TeleopTcpClient.stick(v, w)` → 已有 `:9100`（`run_local.py --daemon --no-lidar`）。不要再开第二条 CAN。

## 启动（仅感知，不控车）

```bash
cd /home/fafu_robot/Desktop/chassis_lidar_drivers
bash nav_demo/scripts/bringup_sense.sh
```

若 MOLA 未发布去畸变扫描（`/lidar_odometry/deskewed_scan_points` publisher=0），只重启 MOLA 并带环境变量（不改 YAML）：

```bash
bash nav_demo/scripts/restart_mola_deskew.sh
```

栅格节点：

```bash
/usr/bin/python3 nav_demo/cloud_to_grid.py
```

日志：`/tmp/nav_demo_bringup/cloud_to_grid.log`

## 阶段脚本（全部不发 stick）

| 脚本 | 作用 |
|---|---|
| `cloud_to_grid.py` | 点云 + MOLA pose → `/nav_demo/obstacle_grid` |
| `plan_once.py` | 真实 Grid + 前方 1.8 m 测试目标 → 官方 A*，发布 `/nav_demo/plan` |
| `run_mvp.py` | Grid → A* → `Navigator.goto` + Guard + replan；只打印 `DRY_RUN` |
| `check_9100.py` | hello/ping `:9100`，确认 stick 代码路径，**不发送 stick** |

```bash
/usr/bin/python3 nav_demo/plan_once.py --forward 1.8
/usr/bin/python3 nav_demo/run_mvp.py --forward 1.8 --seconds 6 --speed 0.12
/usr/bin/python3 nav_demo/check_9100.py
```

终端关键字：`POSE` `GRID` `PLAN` `WAYPOINT` `GUARD` `REPLAN` `DRY_RUN`

## Topic / 坐标系

| Topic | 用途 |
|---|---|
| `/rslidar_points` | 占用栅格主输入（车体系，密） |
| `/lidar_odometry/deskewed_scan_points` | 去畸变扫描（RViz + 无原始点云时回退） |
| `/lidar_odometry/pose` | MOLA `map→base_link`。只用 `position.x/y` + 四元数 yaw，**不用 twist** |
| `/nav_demo/obstacle_grid` | `frame_id=map`；100 占用 / 50 膨胀 / 0 自由 / -1 未知 |
| `/nav_demo/plan` | A* 路径，供 PPT RViz |

坐标系（来自 `bunker_mini/rslidar_cloud.py` 的 `rslidar_to_vehicle`，不猜测）：

- 传感器：`+Z` 朝前，`+Y` 朝上，`+X` 朝左
- ROS `base_link`：`x_fwd = z_s`，`y_left = x_s`，`z_up = y_s + 0.365`
- OccupancyGrid 车体系：`right = -y_left`，`forward = x_fwd`

高度过滤：`base_link z ∈ [0.15, 1.20] m`。水平 `< 0.45 m` 的点当车体自扫丢掉。

## 栅格参数

- resolution 0.10 m，窗口半径 6.0 m
- 发布层膨胀 0.35 m（RViz 上看得到）
- A* 规划膨胀 0.20 m + 车体足迹 0.40 m 清掉（否则官方 A* 会因起点落在膨胀里直接失败）
- TTL 5 s（现有 `OccupancyGrid.update`）

## PPT RViz

只用：

```bash
bash ppt_materials/scripts/start_rviz_ppt.sh
```

配置：`ppt_materials/rviz/ppt_live.rviz`（黑底、无彩虹、Fixed Frame=`map`、ObstacleGrid、AStarPlan）。

**不要**改 `maps/mola_lo.rviz`。

## 安全

- 这些脚本不会发速度、不会 `stick`、不会 CAN TX。
- 不要同时跑 `run_obstacle_avoidance.py`（抢 UDP 6699）。
- 实车测试必须你口头允许后再做。建议 `v ≤ 0.12 m/s`，目标 1.5–2 m，急停：遥控松键 / 网页急停 / `stick(0,0)`。
