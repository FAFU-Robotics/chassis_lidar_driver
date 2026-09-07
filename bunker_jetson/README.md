# BUNKER MINI 2.0 CAN 控制

Car-side agent (CAN, lidar, tracks, navigation). Start from the repo root: `python3 run_local.py`.

> **Jetson / Linux 用户请看 [`使用说明.md`](使用说明.md)** —— 本文件保留自 Windows
> 参考项目（`bunker_mini_can_control`），部分示例为 Windows 命令。
> Jetson 日常入口在**仓库根目录** `python3 run_local.py`，不要再开本目录的
> `mock_cloud.py` WebSocket 云端。
>
> **云端模式已废弃，不建议调用。** 下文「远程接入 / 模拟云端」只是历史协议说明，
> 后续功能非必要不要再靠这条通道。

基于 [BUNKER MINI 2.0 用户手册](https://agilexsupport.yuque.com/staff-hso6mo/rg519a/bbv1zm1aqm0ifyab) 第 3.3.2 节 CAN 协议实现的车体控制代码。

## 目录结构

```
bunker-mini-can-control/            ← 项目根目录（所有命令在此目录下运行）
├── bunker_mini/                    ← Python 库
│   ├── protocol.py                 CAN 协议定义（帧编码/解码）
│   ├── controller.py               运动控制器（TX/RX 线程）
│   ├── monitor.py                  只读状态采集
│   ├── can_util.py                 设备检测 / 接口解析
│   ├── tracker.py                  轨迹录制 / 回放
│   ├── agent.py                    车侧代理（WebSocket ↔ CAN）
│   ├── lidar.py                    Airy 激光雷达驱动（UDP → 点云 → 累积 360°障碍扇区图；含 DIFOP 标定 / 自扫过滤 / 安装外参）
│   ├── pcap.py                     PCAP 离线回放（纯标准库解析，可替代 UDP 数据源）
│   ├── terrain.py                  地形剖面/通过性判定（台阶、岩壁、坡、坑、碎石区分）
│   ├── obstacle.py                 避障守卫（限速/多帧确认急停/坡面降速，叠加在所有运动通路上）
│   ├── navigator.py                目标点自主导航（里程计航迹推算 + goto + 绕障 + 倒车脱困）
│   ├── patrol.py                   自主探路巡游（LiDAR 选开阔可通行方向 + 实时避障，边巡游边记录轨迹）
│   ├── occupancy.py                雷达 2D 伪地图（射线栅格化 + 地形不可通行标记 + 里程系全局累积）
│   ├── ascii_view.py               点云/地图 ASCII 俯视图 + matplotlib PNG 保存（纯 Python，无 GUI）
│   ├── vision.py                   视觉目标检测（LiDAR 反射强度聚类 → 目标方位/距离/高度 + 里程系坐标换算）
│   └── approach.py                 机械臂对接状态机（对准 → 逼近 → 停稳 → READY，视觉伺服闭环）
├── examples/                       ← 可执行脚本
│   ├── get_robot_info.py           读取底盘信息
│   ├── keyboard_control.py         键盘实时操控
│   ├── record_track.py             轨迹录制
│   ├── play_track.py               轨迹回放
│   ├── run_agent.py                车侧代理入口
│   ├── mock_cloud.py               已废弃的模拟云端（不建议启动）
│   ├── live_viewer.py              点云/全局地图实时图形查看器（连云端 → 开流 → matplotlib 窗口渲染）
│   ├── view_lidar.py               雷达点云/障碍图可视化（验证雷达链路）
│   └── simulate_lunar_terrain.py   月球溶洞地形+高反光目标圆柱合成 MSOP 模拟器（无雷达自检 / UDP 仿真）
├── tracks/                         轨迹文件存放目录（自动创建）
└── tests/                          协议 / 雷达 / 地形 / 视觉 / 对接测试
```

## 协议摘要

| 参数 | 值 |
|------|-----|
| 标准 | CAN 2.0B |
| 波特率 | 500 Kbps |
| 字节序 | Motorola（大端） |
| 控制周期 | 20 ms（50 Hz） |
| 超时保护 | 500 ms 无指令则停车 |

### 主要 CAN ID

| ID | 方向 | 说明 |
|----|------|------|
| `0x421` | 发送 | 模式设置：`0x01` 进入 CAN 指令模式 |
| `0x111` | 发送 | 运动控制（线速度 + 角速度） |
| `0x441` | 发送 | 故障清除 |
| `0x211` | 接收 | 系统状态反馈 |
| `0x221` | 接收 | 运动状态反馈 |
| `0x311` | 接收 | 里程计 |
| `0x361` | 接收 | BMS 电池 |

## 安装

```bash
cd D:\bunker_mini_can_control\bunker-mini-can-control
pip install -r requirements.txt
pip install "python-can[gs_usb]"    # candleLight 支持（Windows 需配合 Zadig 装 WinUSB 驱动）
pip install keyboard                 # 键盘操控依赖
```

## 快速开始（Windows + candleLight）

### 1. 确认硬件连接

```
电脑 ──USB──→ candleLight ──CAN线──→ Bunker Mini 底盘
```

- candleLight 用 [Zadig](https://zadig.akeo.ie/) 安装 WinUSB 驱动（Options → List All Devices → 选 candleLight → Replace Driver）
- 设备出现在「通用串行总线设备」下是**正常**的（candleLight 不是串口设备）

### 2. 列出 CAN 设备

```bash
cd D:\bunker_mini_can_control\bunker-mini-can-control
python examples\get_robot_info.py --list-devices
```

### 3. 验证 CAN 链路

```bash
python examples\get_robot_info.py --interface gs_usb
# candleLight 也可用 --interface candle（auto-fallback 会自动切换）
```

看到 `车体状态: 正常  电池电压: ...V  故障码: 0x00` 即链路正常。

### 4. 键盘实时操控

```bash
python examples\keyboard_control.py

# 键盘驾驶 + 同步录制轨迹（见下方"轨迹克隆"）
python examples\keyboard_control.py --record warehouse_loop
```

按住走、松开停：
| 键 | 功能 |
|----|------|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左转 / 右转 |
| `R` | 停止并保存录制（`--record` 模式） |
| `+`/`=` / `-`/`_` | 增大 / 减小步长 |
| `X` / `SPACE` | 急停 |
| `Q` / `ESC` | 退出 |

限速：线速度 0.20 m/s，角速度 0.50 rad/s（保守安全值）。

## 轨迹克隆（轨迹录制 → 回放）

> **开发优先级（甲方最新确认）**：车侧接入分两步——
> **第一步 = 本节的轨迹克隆**（小车能按预设路线自主行驶）；
> **第二步 = 小程序/云端智能派遣**（`task_submit` 等）。
> 只有先完成轨迹克隆，小程序端才能做智能派遣，故当前项目以轨迹克隆为交付主线。

### 录制

两种方式任选：

```bash
# 方式一：遥控器驾驶（录制脚本只采样、不驱动，用实体遥控器开小车走路线）
#         手动驾驶小车走一遍路线，按 Enter 结束
python examples\record_track.py --name warehouse_loop

#     或定时自动停止
python examples\record_track.py --name delivery_A --duration 30

# 方式二：键盘驾驶 + 同步录制（不需要遥控器）
#         用 W/A/S/D 开小车，按 R 停止并保存，退出时自动保存
python examples\keyboard_control.py --record warehouse_loop
```

轨迹保存为 `tracks\<名称>.json`（**自适应采样**：直行匀速自动降采样省文件、转弯/加减速自动加密保留细节，
同时写入 `wheelbase / 总里程 / 最大速度 / schema 版本` 元数据）。
所有录制都会单独存为一个文件，互不覆盖（除非你重复使用同一 `--name`，后者会覆盖前者；
`keyboard_control.py --record` 多次按 R 会自动加时间戳后缀生成不同文件）。

### 回放

```bash
# 列出所有已保存的轨迹（含之前的旧录制）
python examples\play_track.py --list-tracks

# 不带参数运行也会列出全部轨迹
python examples\play_track.py

# 回放指定的一条（把小车放回录制起点）
python examples\play_track.py --name warehouse_loop

# 回放 3 次
python examples\play_track.py --name warehouse_loop --loop 3

# 回放绝对路径文件
python examples\play_track.py --file D:\bunker_mini_can_control\bunker-mini-can-control\tracks\warehouse_loop.json
```

> ⚠️ 回放自带安全限幅：线速度 ≤ 0.5 m/s、角速度 ≤ 1.0 rad/s。
> 若轨迹文件里出现异常偏大的角速度（例如修复角速度解析**之前**录制的旧轨迹），
> 回放会自动截断到安全值并打印警告；建议旧轨迹删除后重新录制。
> 回放按**里程驱动**（以左右轮累计里程 0x311 为准走到录制距离，速度只影响快慢），
> 因此路程与录制一致，不受底盘电机响应快慢影响；若里程帧异常则自动退回按时间回放。

### 录制 / 回放质量优化（A 闭环纠偏 · B 录制质量 · C 起终点对齐与停靠）

- **A. 闭环纠偏（回放）**：回放端把已录制左右轮里程用 `OdometryPose` 差速模型积成
  「期望位姿序列」，同时用真实 0x311 积「实际位姿」；每控制周期算**横向偏差 + 航向偏差**，
  只叠加修正角速度 `w`（**里程目标不变，不抢距离**），直接补偿轮滑/地面变化导致的漂移。
  对 `find_object` 的倒放返回精度是关键——返回精度上去了，任务才算真正闭环。
  依赖 wheelbase 标定精度；录制元数据里 wheelbase 与回放不一致时会打印告警。
- **B. 录制质量**：
  - **自适应采样率**：固定 100ms 改为「里程/速度变化阈值触发」——直行匀速少采样
    （文件更小、回放更顺），转弯/加减速自动加密采样（保留细节）；`stop()` 自动补尾部采样。
  - **录制后轻量平滑**：对 v/w 做 3 点滑动平均，去掉手动驾驶抖动（不改里程，回放仍按里程驱动）。
  - **元数据随文件保存**：`wheelbase / 总里程 / 最大速度 / schema_version` 写入 Track JSON，
    回放前校验（wheelbase 不一致告警，这是闭环纠偏 A 的前提与排障关键）。
- **C. 起终点对齐与末端停靠**：
  - **起点对齐**：`reversed()` 倒放返回前，用「探路起点位姿 + 轨迹末端期望位姿」对照当前
    里程系位姿——航向偏差 >20° 先原地转到位，位置偏差 >0.5 m（不在轨迹终点附近）直接
    回退 go_home，避免「沿轨迹但整体偏着回去」。
  - **末端停靠**：回放收尾段自动切低速爬行（0.05 m/s），到达里程目标后命令停车并做
    「停稳确认」（位移停滞 1 s 视为到位），避免「到达但停得离目标点差几厘米」。

### 录制/回放时长限制

**代码没有硬性时长上限**，限制来自实际资源：

| 因素 | 说明 |
|------|------|
| 采样频率 | 自适应：直行匀速自动降采样（≤0.5s/点）、转弯/加减速加密（≤50ms/点） |
| 内存/文件 | 约 10 分钟 ≈ 6000 航点 ≈ 500 KB JSON；1 小时 ≈ 3.6 万航点 ≈ 3 MB（自适应后进一步减小） |
| 回放耗时 | 按里程驱动 ≈ 录制时长（底盘响应慢会稍长） |
| 回放保护 | 若里程反馈卡死，回放会在「录制时长 ×3（最少 30s）」后自动停止并警告 |

实际建议单次录制 ≤ **30~60 分钟**；更长的路线建议分段录制、分段回放。

## 远程接入（WebSocket 车侧代理 · 已废弃，不建议调用）

```powershell
# 方式一：环境变量（PowerShell 语法；避免命令行泄漏）
$env:BUNKER_WS_URL   = "wss://<云端地址>/ws"
$env:BUNKER_DEVICE_ID = "BUNKER-TEST01"
$env:BUNKER_BIND_CODE = "<甲方分配绑定码>"
python examples\run_agent.py

# 方式二：命令行参数（不用环境变量，最简单）
python examples\run_agent.py --ws-url wss://<云端地址>/ws --device-id BUNKER-TEST01 --bind-code <绑定码>
```

> ⚠️ **PowerShell 环境变量必须用 `$env:变量 = "值"`**，不能用 cmd 的 `set 变量=值` 语法
> （PowerShell 里 `set` 是 `Set-Variable` 的别名，不会设置环境变量，代理会报
> "必须提供 --ws-url" 错误）。若确实在用 cmd.exe，才用 `set BUNKER_WS_URL=...`。

Agent 启动后：
- 连接 → 鉴权（auth + bindCode）→ ONLINE
- 每 15s 发心跳 `ping`，断线指数退避重连（1s→2s→…→30s）
- 每 1.5s 上报状态 `state`（battery/speed/odometer/faultCode）
- 接收指令 `cmd`：`move` / `estop` / `task_submit` / `track_record` / `track_follow` / `track_delete` / `query`
- 事件推送 `event`：`fault` / `low_battery` / `offline` / `arrived`（回放完成）/ `auto_stop`
- 云端 `move` 指令自动限幅：线速度 ≤ 0.5 m/s，角速度 ≤ 1.0 rad/s（安全保护）
- **`move` 自动停车保护（重要）**：`move` 是持续速度指令，若 5 秒内没有新的 `move` 续期，
  agent 会**自动停车**并发 `auto_stop` 事件。云端需持续下发 move 才能让小车持续运动；
  单条 `move` 最多跑 5 秒——即使操作员忘记停止或网络中断，小车也不会无限跑下去
- **`move` 可带时长**：协议 payload 支持 `duration` 字段（秒），
  例如 `{"action":"move","v":0.2,"w":0.1,"duration":10}` 会让小车按该速度行驶 10 秒后自动停车，
  期间无需续期；`duration` 省略或为 0 时按上述 5 秒看门狗逻辑
- **断线立即停车（重要）**：WebSocket 连接断开时 agent 立即停车并发 `offline` 事件，
  绝不会带着最后一条速度指令继续行驶
- 回放期间下发 `move` / `estop` 会**立即终止当前回放**，防止回放线程与手动指令互相打架（急停绝不会被回放"复活"）
- **`task_submit` 任务流转（Step 2 智能派遣的地基）**：下发点到点运输任务
  `{"action":"task_submit","taskId":"T001","trackId":"路线1"}` → 车侧校验轨迹存在（任务执行中再下发会被拒绝，对应手册 409"车忙"）→
  沿该轨迹自主回放 → 每 1.5s 经 `state.payload.task` 上报 `{taskId,status,progress}` → 完成推 `arrived` 事件；
  手动 `move`/`estop`/新回放会取消当前任务。也支持 `waypoints[]`（Track 格式 t/left_mm/right_mm/v/w）直接下发自包含任务
- 紧急停止：`estop`（云端命令）即时生效；若连 CAN 都失控，直接拔掉 USB 或按底盘急停按钮

## 激光雷达避障与导航（RoboSense Airy）

在轨迹克隆之上新增的两层能力，全部为纯 Python 实现（不依赖 ROS），
Windows 笔记本与 Jetson 通用：

1. **避障保护**：云端 `move`、轨迹回放、`task_submit`、`goto` 导航——所有
   会驱动底盘的通路都叠加同一道雷达安全网（`ObstacleGuard`）
2. **目标点自主导航**：`goto(x, y)` 指令让小车从当前位置自主行驶到目标点，
   途中自动避障

### 硬件与网络

```
[Airy 雷达] ──网线──→ [车侧电脑网口1: 静态 IP 192.168.1.102]
                        [网口2 / WiFi: 云端 WebSocket]
```

| 项 | 值 |
|----|-----|
| 雷达 | RoboSense Airy，96 线 3D，水平 360° / 垂直 0~90°，盲区 0.1 m，10 Hz |
| 雷达 IP / MSOP 端口 | `192.168.1.200` / UDP `6699`（电脑需配静态 IP `192.168.1.102`） |
| 数据包 | 1248 B（42 帧头 + 8×148 数据块 + 6 帧尾），距离分辨率 0.5 cm |

> 雷达 0° 不一定朝车头：用 `view_lidar.py` 观察，把车头正前方障碍出现在
> 0° 扇区，偏多少度就填到 `BUNKER_LIDAR_MOUNT_YAW`。

### 验证雷达链路（不驱动底盘）

```bash
python examples\view_lidar.py                      # 实时 360° 障碍扇区图
python examples\view_lidar.py --points             # 点云统计
python examples\view_lidar.py --mount-yaw 90       # 指定雷达安装偏置
```

看到扇区图随前方障碍移动变化即链路正常。

### 组员 Airy 代码迁移能力（A~F，已合入）

从另一位组员的 `robosense_airy` 仓库（`airy_driver.py` / `obstacle.py` /
`pcap_reader.py` / `example.py`）迁移并**按本项目架构重构**的六项能力，
全部纯 Python（不引入 numpy / panda3d / pypcap 等依赖）：

| 迁移项 | 能力 | 落地位置 |
|--------|------|----------|
| **A. DIFOP 角度标定** | 解析雷达 DIFOP 上报的出厂垂直/水平角标定（RPM/FOV/安装模式），替代硬编码表；无标定时自动降级回手册默认表 | `lidar.py`（`parse_difop_packet`，UDP 端口 7788） |
| **B. PCAP 离线回放** | 用抓包文件替代 UDP 雷达：`BUNKER_LIDAR_PCAP=<file>` 或 `--lidar-pcap`，无雷达也能跑通避障/导航/地形全链路（PCAP 支持 scapy 或纯标准库两种后端） | `pcap.py`（`PcapReplaySource`，接口与 `AiryLidar` 对齐） |
| **C. 自扫硬件过滤** | 剔除雷达安装支架/线缆区域的固定点，避免支架在扇区图/地形里形成固定误检；可 `BUNKER_LIDAR_SELF_MASK=0` 关闭 | `lidar.py`（`SelfMaskConfig` / `filter_self_hardware`） |
| **D. 履带外扩 + 低矮障碍** | 履带两侧外扩区域间隙测量（贴近刮蹭时自动限速）+ 车前低矮碎石计数（随地面起伏剔除） | `obstacle.py`（`track_side_clearance` / `count_low_obstacles`） |
| **E. 底盘诊断 hook** | 运行中控制模式掉回 STANDBY 自动重发使能；已下发前进指令但实测轮速≈0 时周期性告警（检查急停/遥控档位/着地/电池） | `agent.py`（`_drive` 统一下发口） |
| **F. 安装外参标定** | 雷达高度（`BUNKER_LIDAR_HEIGHT_M`）与俯仰（`BUNKER_LIDAR_PITCH_DEG`）参数化点云变换，地形通过性/目标高度判定拿到「离地高度」系 | `lidar.py`（`transform_point_cloud`，`view_lidar.py` 可视化验证） |

- 组员的 `example.py` 用 `numpy` 做 3D 可视化、`pypcap/pcapy-ng` 抓包，**均未迁入**
  （本仓库保持纯 Python）；仅保留其**算法能力**并复用到本项目的
  `ObstacleGuard` / `TerrainProfile` / `agent._drive` 框架里。
- 外参参数已打通到 `run_agent.py` / `view_lidar.py` / `agent.conf`：
  装好后用 `python examples\view_lidar.py --height 0.5 --pitch 0` 等观察点云是否贴合实际地形。
- PCAP 回放示例：先在有雷达的电脑上 `tcpdump/wireshark` 抓 MSOP 包存成 pcap，
  再把文件路径填给车侧代理即可离线复现当时的避障/导航表现。

### 避障规则（`ObstacleGuard`，可调）

| 前方障碍距离 | 行为 |
|--------------|------|
| < 0.3 m | 急停，推送 `event: obstacle`（需连续 3 帧确认，抗碎石/扬尘毛刺） |
| 0.3 ~ 0.8 m | 按距离线性限速（越近越慢） |
| ≥ 0.8 m | 放行 |

雷达未连线 / 掉线时避障自动降级为「无传感器」模式：不拦截指令但每 10s
打警告（方便开发阶段先跑通底盘）。

### 复杂路况加固（阶段 2，面向月球溶洞等凹凸/碎石地形）

新增 `terrain.py`：把点云按「方位扇区 × 距离格」统计高度，用**高度突变**
区分「低矮碎石（可通行，限速慢过）」与「台阶/岩壁/高堆石（不可通行，绕行）」，
并识别连续坡面（只降速不误急停）：

| 地形特征 | 判定 | 守卫行为 |
|----------|------|----------|
| 3 cm 级碎石 | 凸起高度 < 台阶限高 | 仅限速，不急停 |
| 台阶 / 岩壁 / 坑沿（高度突变 > 限高） | 不可通行 | 进入 1 m 提前减速，进入 0.3 m 急停 |
| 连续坡面（坡度 ≤ 约 26°） | 坡面 | 限速 50%，爬行通过 |
| 陡于 26° 的坡 / 墙 | 不可通行 | 绕行 |
| 方向无任何回波 | 黑洞/未知 | 保守绕行决策 |

- 障碍距离测量改为**滑动窗口累积 + p25 低分位**，消除单帧稀疏与瞬时噪点
- 急停需**连续 N 帧确认**（默认 3），碎石/扬尘不再误触发
- `goto` 导航：接近目标改用**停滞判定**（位移持续停滞视为到达，抗里程计噪声）；
  被堵死时支持**短距离倒车再转向**脱困
- 状态上报新增 `terrain{online, blocked, obstacleDistance, slope, maxHeight, unclear}`

#### 无雷达自检：月球溶洞模拟器

```bash
# 合成「碎石+大石+台阶+坑」的月球溶洞地形，直接验证通过性判定（无需雷达/网络）
python examples\simulate_lunar_terrain.py --selftest --seed 7

# 纯坡面场景
python examples\simulate_lunar_terrain.py --selftest --seed 3 --slope 18 --no-gravel --no-pit --rocks 0 --step-h 0

# 通过 UDP 发送合成 MSOP 包，配合 view_lidar.py / run_agent.py 端到端验证
python examples\simulate_lunar_terrain.py --udp 127.0.0.1:6699 --seed 7
```

`--tilt` 模拟雷达前倾安装角（默认 25°，手册推荐做法）。注意：前倾安装时点云
`z` 是雷达坐标系，地形高度判定前需用 IMU 姿态做水平系校正（自检模式已内置
模拟该校正；真实部署时阶段 3 引入 IMU 解析后自动生效）。

### 目标点导航

```json
{"action": "goto", "x": 2.0, "y": 1.5}
{"action": "goto", "x": 2.0, "y": 1.5, "speed": 0.15}   // 可选：本次导航线速度上限 m/s（现场降速，无需改代码）
```

- 坐标系：以「导航原点」为原点 (0,0)——agent 启动（导航栈初始化）时小车所在
  位置，车头方向为 yaw=0，逆时针为正（**不是底盘物理上电位置**；agent 未启动时
  小车被搬动，原点按 agent 启动瞬间的位置起算，也可用 `odom_reset` 指令重置）
- 到达判定：接近目标后「位移持续停滞 1.5 s」视为到达，完成后推送 `arrived` 事件
- **途中自动避障**：导航线程每个控制周期都过 `ObstacleGuard` —— 前方障碍限速/急停、
  地形不可通行提前绕行（`steer_away_deg` 选开阔侧）、正前方被挡原地转向、两侧都堵则
  倒车脱困、长时间无法到达（> 5s）放弃并推送 `auto_stop` 事件
- **绕行方向锁定**：持续被堵期间保持同一转向侧，避免长墙/走廊/凹槽里左右横跳
- **雷达掉线 fail-safe**：雷达在栈中（`lidar_on` 或开机即开）时，中途掉线（网线松动/
  雷达重启/端口被占）会**立即强制停车并推送 `obstacle` 事件**，绝不在无雷达保护下
  继续行驶；只有显式 `lidar_off` / `--no-lidar` 才降级为无传感器模式
- **目标点预检**：下发 `goto` 时用雷达 2D 伪地图查目标格（`map <x> <y>` 也可预检）。
  伪地图带 **5 s 时间衰减**（只有再次观测到的格保留，避免历史瞬时观测永久残留），
  预检命中 occupied/blocked 时还会用**实时雷达**在目标方位复核：确认目标方向确实存在
  明显近于目标的障碍才拒绝并推 `fault` 事件；伪地图陈旧数据则放行，由导航途中的
  实时避障守卫兜底（不会撞上，也不会被误拒）
- 与 `move` / 轨迹回放 / `task_submit` 互斥：任一指令下发都会取消正在进行的 `goto`
- 状态上报新增可选字段：`pose{x,y,yaw}`、`navigating`、`lidar{online,frames,frontObstacle}`、`terrain{...}`、`map{...}`

### 云端雷达命令与 2D 伪地图

可在云端单独控制雷达，并用雷达为小车绘制一张 2D 伪地图（**清楚标注哪里不能通行**），
方便在设计 `goto` 目标点之前先探明路况：

| 云端命令 | 作用 |
| --- | --- |
| `{"action": "lidar_on"}` | 单独开启雷达（运行中随时可打开，即使启动时 `--no-lidar` 关闭；幂等） |
| `{"action": "lidar_off"}` | 关闭雷达（避障降级为无传感器模式） |
| `{"action": "lidar_status"}` | 雷达/避障自检：在线状态、帧数、**本帧点数**、前方障碍距离、地形通过性、履带侧隙、低矮碎石、避障守卫放行测试 |
| `{"action": "lidar_map"}` | 返回 2D 伪地图快照（占用格/不可通行格坐标 + ASCII 俯视图 + 车体位姿） |
| `{"action": "lidar_map", "x":…, "y":…}` | 附带目标点预检 `targetCheck`：查询 (x,y) 附近能否通行 |
| `{"action": "lidar_map", "silent": true}` | 静默轮询帧：事件回写 `silent` 标记，云端控制台完全不打印（图只在查看器窗口） |
| `{"action": "lidar_map", "includeFree": true}` | 快照附带 `free` 自由格坐标（地图模式渲染已扫描区域） |
| `{"action": "point_cloud"}` | 返回最新一帧雷达点云快照（抽样子采样 `[x,y,z,强度]`，最多 8000 点） |
| `{"action": "pc_stream", "hz": 3}` | 开关点云实时流：`hz>0` 开启并按该频率持续推送最新帧（钳制 1~10），`hz=0` 关闭 |

- **地图来源**（`bunker_mini/occupancy.py`）：把雷达「累积 360° 障碍扇区图」沿射线
  栅格化（自由 → 占用），再把地形判定「不可通行」的扇区（台阶/岩壁/坑沿）标为
  blocked（X）。随小车运动按里程计位姿注册到全局坐标，地图逐步展开累积。
- **随状态上报持续更新**：状态报文的 `map` 字段带轻量摘要（占用/不可通行/自由格数）；
  完整地图用 `map` / `lidar_map` 拉取：优先在工控机桌面打开地图窗口，
  终端只留中文摘要。没显示器时才印文字俯视图（**上=车头**：`▲`=车头、
  `@`=车、`X`=过不去、`#`=障碍、`.`=空地、空白=没扫到）。
- **地图方向**：文本俯视图以车体为中心、按当前 yaw **旋转到车体系**——
  「车前方」永远显示在屏幕上方，转弯后自动跟着车转，不再需要脑内换算世界坐标。
- 设计 `goto` 的推荐流程：`map` 看哪里不能走 → `map <x> <y>` 预检目标格 →
  再下发 `goto <x> <y>`。
- **点云一定会出现吗？** 只要 `lidar on` 判定为「在线」，雷达就在持续出点云——
  `is_receiving=True` 的定义就是「3 秒内收到过一帧含点云的数据」，每帧几千点。
  `lidar_status` 的 `pointCount` 字段显示最近一帧点数，非 0 即有点云在流动。
  云端 `view` 会优先在工控机桌面打开点云窗口；没显示器时才印文字俯视密度图
  （上=车头，`.` 零星 / `o` 稀疏 / `O` 较密 / `#` 很密），并保存
  `pointcloud_*.png`。
- **想看实时 3D 点云窗口**（组员 `d:\robosense_airy\robosense_airy` 方案）：
  那是基于 **Panda3D + wrs 框架**的桌面 3D 窗口（`viz.py` / `example.py`），
  必须在**收到雷达数据的这台机器**上独立运行，且依赖 Panda3D 等重型图形库，
  与 WebSocket 云端链路是两套独立程序。
- **实时图形查看（推荐，无需 Panda3D）**（短命令，与既有单/双字母风格一致）：
  - 模拟云端控制台输入 `vl [hz]`（长形式 `view live`）：自动下发 `pc_stream`
    开启点云流，并在新窗口启动 `examples/live_viewer.py`——matplotlib 实时窗口
    （车头朝上 2D 俯视图，按强度着色；加 `--3d` 可看 3D）。Ctrl+C 退出时
    自动下发 `pc_stream off` 关流。
  - **`vm [hz]`（长形式 `view map`，观赏价值最高）**：世界系固定视角渲染
    2D 伪地图，小车移动时地图**逐步展开**（SLAM 式建图动画），叠加：
    小车箭头、历史轨迹线、绿色已扫描自由区、橙色障碍、红色不可通行格。
    数据来自 ~1 Hz 静默轮询 `lidar_map`（`silent`/`includeFree`），
    **云端控制台不打印任何轮询内容**——图只在查看器窗口里，终端不刷屏。
  - `vb [hz]`（`view both`）：点云 + 全局地图左右并排；`vo`（`view off`）关流。
  - 查看器窗口内按 **f** 切换跟随/固定视角（默认跟随小车居中）；命令行
    `--range <m>` 缩放、`--target <x> <y>` 叠加 goto 目标点与预检结果
    （free 绿 / occupied 橙 / blocked 红 / unknown 灰）、
    `--save-dir <目录>` 每约 30 s 自动存一张地图快照 PNG（回放建图过程）。
  - **云端终端按 Ctrl+C 时，若查看器窗口还在运行，只关闭查看器并下发
    `pc_stream off`，云端保持运行**；再次按 Ctrl+C 或输入 `quit` 才退出云端。
    车侧代理断线重连采用「服务器不可达 → 固定 1 s 探测」，云端重启后
    自动快速重连（2026-08 修复，避免出现重启云端后「没检测到车侧代理」）。
  - 也可手动启动：`python examples\live_viewer.py --ws-url ws://云端:端口 --mode map`
    （同一 deviceId/bindCode）。无 GUI/无 matplotlib 时自动回退**终端 ANSI
    动画渲染**（清屏重绘 + 颜色：`#`红/`Oo`黄/`.`绿/`@`白），非 TTY 自动降级为逐行日志。
  - `view`/`pc` 仍可拉单帧点云快照；`map` 拉单帧地图；流用 `vo` 随时关。
- **`lidar_on` 判定标准**：雷达启动后给约 5 s 首帧预热窗口（启动瞬间
  `is_receiving` 必然为 False，立即判失败是误报）。启动失败分两类，事件里会
  给出具体原因：
  - `雷达启动失败: 无法绑定雷达 UDP 端口 6699 ...` → 端口被占用（如另一个
    agent 实例仍在运行），关掉旧进程即可；
  - `雷达已启动（UDP 端口已绑定）但收不到数据` → 报文按收包计数细分诊断：
    **0 包到达**（雷达没把数据发到本机，用雷达配置工具把数据目标 IP/端口设
    为本机静态 IP:6699，RoboSense 是主动发包方）、**包到但解析不出帧**（端口
    被别的数据源占用/非 Airy 格式）、**曾有帧但 3 秒无新帧**（雷达停发）。
  - `lidar_status` 同样带 `diagnosis` 字段；可用 `python examples\view_lidar.py`
    单独确认是否收到帧。
- **`goto` 需要在线雷达**：雷达已开启但收不到点云时，`goto` 会**立即拒绝**并
  推 `fault` 事件（而不是下发后小车毫无反应）；仅 `lidar_off`/`--no-lidar`
  显式无传感器模式才允许在无雷达保护下导航。

### 配置（`agent.conf`）

```ini
BUNKER_ENABLE_LIDAR=1              # 0=关闭雷达避障/导航
# BUNKER_LIDAR_PORT=6699           # 雷达 MSOP 端口（改过按实际填）
# BUNKER_LIDAR_MOUNT_YAW=0         # 雷达 0° 相对车头偏置（度，左正）
# BUNKER_WHEELBASE=0.5             # 左右轮间距（米），导航航迹推算用，需实测标定
# BUNKER_STEP_LIMIT=0.07           # 允许的最大台阶/坑深度（米），超过判不可通行绕行
```

> ⚠️ `BUNKER_WHEELBASE` 决定 `goto` 定位精度，甲方现场务必实测两轮间距填入。
> ⚠️ `BUNKER_STEP_LIMIT` 建议按底盘离地间隙的一半设置（BUNKER MINI 2.0 约 0.07 m）。

### 视觉目标检测与任务链（阶段 3：探路 → 找目标 → 导航 → 抓取对接 → 返回）

对应任务流「出发探路 → 视觉判定目标 → 确定坐标 → 自动导航 → 机械臂抓取 →
返回起点」中**除机械臂本体**以外的全部车体环节。**无需人工驾驶、无需提前录制
轨迹**——路线未知时小车自主探路，探路轨迹自动记录，返回时沿原路回来：

```json
{"action": "find_object", "name": "目标A", "approach": true}
{"action": "go_home"}
```

- **目标检测（LiDAR 反射强度通路，`bunker_mini/vision.py`）**
  - 目标贴高反光材料（工程级反光贴纸/镀膜）后，点云反射强度显著高于低反射的
    玄武岩表面；按强度阈值过滤 → 方位聚类 → 输出车体系方位角 / 距离 / 高度。
  - 主动光源不受月球无光照影响；聚类取「最近命中点」距离、「簇内最高点」高度，
    抗地形遮挡。
  - 检测器统一实现 `TargetDetector` 接口：组员的相机 / 深度学习识别模型（YOLO 等）
    只需包一个 `detect()` 即可接入，**无需改动 find_object / approach 编排**。
    建议分工：DL 模型回答「是什么」（语义类别），反射强度通路回答「在哪」（几何定位）。
- **自主探路巡游（`bunker_mini/patrol.py`）**
  - 无目标坐标时，小车按 LiDAR 累积扇区图选「最开阔 + 地形可通行」的方向
    低速前进（默认 ≤0.25 m/s），实时避障；被堵死原地转向重试，长时间无法
    动弹自动倒车脱困。
  - 边巡游边跑反射强度检测，**同时自动记录轨迹**（无需人工驾驶录制）——
    这正是「路线未知、临场发挥」场景下的探路实现。
  - 探路超时（默认 120 s）或直线距离超限（默认 10 m）仍未找到目标 →
    沿已记录轨迹原路返回起点并上报失败。
- **find_object 编排（`agent`）**
  1. `recon` — 自主巡游探路，边巡游边检测目标、边自动记录轨迹
  2. `found` — 检测到目标，`target_to_odom(pose, est)` 换算成里程系坐标
  3. `navigating` — 自动导航到目标点（复用 goto，途中雷达避障/地形绕行）；
     **录制不停，轨迹延伸到目标点**
  4. `approaching` —（仅 `approach=true`）进入对接状态机；**录制仍继续**，
     轨迹最终定格在机械臂可抓取的位置
  5. `ready` — 推送 `arrived`，机械臂可开始抓取
  6. `holding` —（仅启用 `arm_bridge` 信号通道时）车体停稳等待机械臂
     **真正完成抓取**（此前代码在此处直接返回，未等机械臂）；收到
     「抓取完成」信号（云端 `grasp_done` / CAN 0x3A1 / GPIO / 联调文件）
     → `grasped`；超时推 `grasp_timeout` 并停车待人工处置
  7. `returning` — 抓取完成后**沿完整轨迹反向返回起点**（`Track.reversed()`，
     安全避开探路时绕过的障碍）；返回前先做航向/位置对齐判定（C1），
     轨迹不可用/被中断时回退 go_home 直线返回
  - 状态经 `state.payload.mission{status,target,goal}` 实时上报；与
    `move` / 轨迹回放 / `task_submit` / `goto` 互斥，estop/断线立即中止。
- **机械臂对接状态机（`bunker_mini/approach.py`）**
  - `ALIGNING`（对准方位）→ `APPROACHING`（边前进边微调，≤0.2 m/s）
    → `STABILIZING`（到位停稳确认）→ `READY`；目标短暂丢失原地旋转重寻，
    丢失超时/外部取消转为终态。
  - 必须在**视觉反馈下闭环**——开环导航误差 + 里程计漂移在 1 m 内可达 10 cm+，
    而机械臂工作距离通常只有 0.4~0.6 m，不闭环会抓偏。
- **自检（无雷达）**：`simulate_lunar_terrain.py` 已合成高反光目标圆柱并演示检测：

```powershell
python examples\simulate_lunar_terrain.py --selftest --seed 7     # 末尾输出反射强度检测结果与真值对比
python examples\simulate_lunar_terrain.py --selftest --no-target  # 验证无目标时不误检
```

## Jetson 部署（最终阶段：全自主导航）

> 代码与 Windows 笔记本**完全共用**（纯 Python），Jetson 上只需：
> 启用内核 CAN 网卡 + socketcan 配置 + 双网卡（雷达/云端）接线。

```bash
# 1) 启用内核 CAN 网卡（每次开机执行，或写 systemd）
#    USB-CAN 适配器（candleLight）→ can1（推荐，底盘标准接线）
bash bringup_gs_usb_can.sh
#    或板载 mttcan → can0
bash bringup_can0.sh

# 2) 一键启动（自动读 agent.conf + socketcan；启动时自动拉起所有 can* 网卡，
#    并探测「有底盘反馈帧(0x211)」的通道，USB-CAN/板载自动选对）
bash start_agent.sh
```

> **自动选口/自动拉起**：agent 启动时自动 `ip link set <can*> up type can bitrate 500000`
> （root 下免 sudo），并短听各通道的 0x211 反馈帧，选中真正连到底盘的那个口。
> 若状态行出现 `CAN=can1(down)⚠无反馈` 或 `CAN=can0⚠无反馈`，说明链路不通：
> 检查 CAN_H/CAN_L 接线、适配器供电、底盘是否上电（`ls` 雷达自检看有没有 `模式=`）。

> **USB-CAN 插上却一直 DOWN / 小车不动？** 先跑一键诊断修复脚本（需要 root）：
>
> ```bash
> sudo bash fix_candle_usb.sh
> ```
>
> 它会：① 修复 `/etc/udev/rules.d/99-gs_usb-candle.rules` 的语法错误
> （历史版本行尾多了个右括号，导致 MODE=0666 权限和「自动 ip link set up」
> 规则整体失效——这是 USB-CAN 无法使用的最常见根因）；
> ② reload udev 并拉起全部 can* 网卡；③ 逐个 candump 探测底盘反馈，
> 标出真正连底盘的口，并给出 `BUNKER_CAN_CHANNEL=xxx` 建议。

详细部署步骤（系统准备 / 双网卡配置 / systemd 自启 / 排障）见 **[DEPLOY_JETSON.md](DEPLOY_JETSON.md)**。

## 本地自测（「模拟云端」· 已废弃）

**不要再开这条。** 日常自测用仓库根目录 `python3 run_local.py`。
下面只保留旧 WebSocket 对照步骤，不建议调用。

甲方真实云端尚未建立时，旧入口可用 `mock_cloud.py` 复刻手册协议
（鉴权/心跳/状态上报/指令接收）。新功能不要再往这里靠。

```powershell
pip install websockets    # 仅模拟云端需要

# 终端 A — 启动模拟云端（保持运行，另开窗口跑终端 B）
python examples\mock_cloud.py

# 终端 B — 让车侧代理连它（注意 ws:// 而非 wss://）
$env:BUNKER_WS_URL   = "ws://127.0.0.1:9000"
$env:BUNKER_DEVICE_ID = "BUNKER-TEST01"
$env:BUNKER_BIND_CODE = "TEST-BIND-xxxx"
python examples\run_agent.py
```

在终端 A 中输入指令下发到车侧代理（代理需连上真实 CAN 底盘才能看到运动效果）。

命令都支持单/双字母简写，状态上报默认只在显著变化时打印（输入 `quiet` 可完全静默、
`verbose` 恢复每次打印），不会被刷屏：

```
m 0.2 0.1           运动 (move 0.2 0.1)，v>0 前进 / v<0 后退，w>0 左转 / w<0 右转
                    不带 t：约 5s 无续期自动停车；想持续运动需每隔几秒重发
m 0.2 0.1 10        运动 10 秒后自动停车（第三个参数 = 行驶秒数，单位秒）
kb / keyboard       键盘控制模式（不录制轨迹）：W/A/S/D 驾驶、SPACE 急停、Q 退出
                    +/= 提速，-/_ 减速（初始 v=0.10 m/s、w=0.20 rad/s，上限 v=0.50 / w=1.00）
e / s               紧急停止（estop，立即停车）
q                   立即上报一次状态（query）
lidar on / lo       云端单独命令开启雷达（运行中可随时打开）
lidar off / loff    关闭雷达（避障降级为无传感器模式）
lidar status / ls   雷达/避障自检：在线状态、前方障碍、地形通过性、履带侧隙、
                    低矮碎石、避障守卫放行测试
map / lidar map     在工控机桌面打开雷达地图窗口（绿=空地 橙=障碍 红叉=过不去）
                    没显示器才在本终端印带「前/后/左/右」的中文简图
map 2.0 1.5         目标点预检：桌面窗口标出该点，并告诉你能不能走
view / pc           在工控机桌面打开点云窗口（上=车头，颜色=反射强度）
vl [hz]             实时点云窗口（hz 默认 3；Ctrl+C 先关窗口）
vm [hz]             全局地图窗口：车走图长；窗口内按 f 切换跟随/固定
vb [hz]             点云 + 地图左右并排
vo                  关闭桌面窗口和点云流
r 路线1             键盘驾驶录制：进入后 W/A/S/D 驾驶小车走路线，
                    按 Q 停止录制并保存
                    若「路线1」已存在会先询问 y/n：y=覆盖旧记录（原记录消失），
                    n=取消录制保留旧轨迹
                    覆盖带失败回滚：先本地备份旧轨迹并等车侧删除确认，
                    网络超时/删除失败则中止并恢复旧轨迹；录制中断也能自动恢复
                    录制中按 B：保存当前轨迹并沿原路径自动返回起点，
                    返回后录制结束并自动退出键盘模式
f 路线1             进入回放模式（track_follow）：小车沿「路线1」行驶，
                    按 B 沿原路径自动返回起点；按 Q 退出回放模式，SPACE 急停
fb 路线1            往返模式（track_follow_back）：小车回放到终点后自动沿原路径
                    返回起点，往返完成自动退出回命令模式；SPACE 急停，Q 随时退出
d 路线1             删除一条已保存的轨迹（track_delete，不可恢复）
task 路线1 [T001]   下发点到点运输任务：车侧沿「路线1」自主回放，
                    状态行显示 任务=T001:running 进度，完成推 arrived 事件
t                   列出车侧全部轨迹（tracks）
p                   下发应用层心跳（ping）
verbose / smart / quiet   状态打印模式切换
auth_fail           模拟鉴权失败(401)
h                   help；x = quit 退出
```

**云端回放已录轨迹**：轨迹文件保存在车侧电脑的 `tracks\<名称>.json`，云端无需上传文件，
只要下发 `track_follow <名称>`，车侧代理就会读取本地轨迹并控制小车回放（回放完成后推 `arrived` 事件）。
录制多条轨迹就存多个文件（如 `warehouse_loop.json`、`delivery_A.json`），互不覆盖，
想回放哪条就 `track_follow` 哪个名字，终端 A 输入 `tracks` 可随时查看现有列表。

### 配置驱动启动（推荐，本地/甲方通用）

同一份代码、同一条命令，本地自测与甲方现场**只改配置文件的值**：

```powershell
# 一键启动（读取项目根目录 agent.conf），三者等价：
.\start_agent.bat                          # 双击即用，无执行策略限制
python examples\run_agent.py --config agent.conf
powershell -ExecutionPolicy Bypass -File start_agent.ps1   # 本机为 Restricted 策略时的替代
```

> 若 `.\start_agent.ps1` 报"禁止运行脚本"，是 Windows 执行策略（Restricted）所致，
> 直接用上面的 `.bat` 或 `python examples\run_agent.py --config agent.conf` 即可。

`agent.conf` 为 `KEY=VALUE` 格式，示例已带注释。**参数优先级：命令行参数 > 环境变量 > 配置文件 > 内置默认值**。

甲方现场只需在 `agent.conf` 里把 `BUNKER_WS_URL` 换成 `wss://甲方域名/ws`、`BUNKER_DEVICE_ID`/`BUNKER_BIND_CODE` 换成甲方签发值，
轨迹目录建议改成绝对路径（如 `C:\bunker_agent\tracks`），并把 `BUNKER_LOG_FILE` 填上（日志落盘）。其余代码零改动。

## 环境变量

| 变量 | 说明 |
|------|------|
| `BUNKER_WS_URL` | 云端 WebSocket 地址 |
| `BUNKER_DEVICE_ID` | 车辆唯一 ID |
| `BUNKER_BIND_CODE` | 绑定码 |
| `BUNKER_TRACK_DIR` | 轨迹文件保存目录（默认 `./tracks`） |
| `BUNKER_LOG_FILE` | 日志文件路径（留空只输出控制台） |
| `BUNKER_LOG_LEVEL` | 日志级别（DEBUG/INFO/WARNING/ERROR，默认 INFO） |
| `BUNKER_CAN_INTERFACE` | CAN 接口（gs_usb / candle / slcan / socketcan） |
| `BUNKER_CAN_CHANNEL` | CAN 通道（不设置时自动检测） |
| `BUNKER_ENABLE_LIDAR` | 是否启用雷达避障/导航（`1`/`0`，默认 `1`） |
| `BUNKER_LIDAR_PORT` | 雷达 MSOP UDP 端口（默认 6699） |
| `BUNKER_LIDAR_MOUNT_YAW` | 雷达 0° 相对车头偏置角（度，左正，默认 0） |
| `BUNKER_LIDAR_PITCH_DEG` | 雷达安装俯仰角（度，正=抬头，默认 0；F 迁移） |
| `BUNKER_LIDAR_HEIGHT_M` | 雷达光心离地高度（米，默认 0；F 迁移） |
| `BUNKER_LIDAR_SELF_MASK` | 自扫硬件过滤开关（`1`/`0`，默认 `1`；C 迁移） |
| `BUNKER_LIDAR_PCAP` | 离线回放：PCAP 抓包文件路径（替代 UDP 雷达；B 迁移） |
| `BUNKER_WHEELBASE` | 左右轮间距（米），导航航迹推算用（默认 0.5） |
| `BUNKER_RECON_MAX_DURATION` | 自主探路最长时长（秒，默认 120） |
| `BUNKER_RECON_MAX_DISTANCE` | 自主探路最远直线距离（米，默认 10） |

## 注意事项

1. 开机默认待机模式，需先发送 `0x421` 且数据为 `0x01` 才能接受速度指令。
2. 若遥控器已开启并处于遥控模式，CAN 指令会被屏蔽；需将遥控器切到指令模式。
3. 控制线程以 50 Hz 持续发送 `0x111`，避免 500 ms 超时停车。
4. CAN_H / CAN_L 需正确接入，并确认终端电阻与 500 Kbps 波特率。
5. candleLight 未插入/驱动异常时，运行脚本会给出明确的排查提示（Zadig 安装步骤）。
6. 雷达避障依赖网络：电脑网口需配静态 IP `192.168.1.102`（与雷达 `192.168.1.200`
   同网段）。**雷达掉线时 fail-safe 停车**：只要雷达处于开启状态（`lidar_on` / 开机即开），
   中途掉线会立即停车并上报 `obstacle` 事件，不会在无保护下继续行驶；只有显式
   `lidar_off` 或 `--no-lidar` 才进入无传感器模式。
7. `goto` 导航是**航迹推算**（里程计积分），长时间行驶会累积误差；`BUNKER_WHEELBASE`
   需按底盘实测值配置，甲方现场若需要绝对定位，可后续接 GPS/视觉做位姿修正。

## 风险处置结论（安全评审）

| 风险 | 级别 | 处置 |
| --- | --- | --- |
| 雷达掉线时避障静默失效 | 高危 | **已修复**：雷达在栈中即 `require_sensor=True`，掉线 → 强制停车 + `obstacle` 事件；仅 `lidar_off`/`--no-lidar` 为透传无传感器模式 |
| 无全局路径规划，绕行可能失败 | 高危 | **部分缓解**：绕行方向锁定（避免长墙/走廊左右横跳）；5s 超时仍是「安全放弃」而非「撞上」。全局寻路（接 `_occ_grid` 做 A* / 绕行后回轨迹）属中期升级，未在本版实现 |
| 航迹推算漂移 | 中危 | **标定项**：现场实测 `BUNKER_WHEELBASE`（两轮间距）填入；误差随时间累积，到达判定有停滞兜底 |
| 探测盲区（FOV 60°/侧向/后方） | 中危 | **标定+接受**：`--mount-yaw/--pitch/--height` 标定安装外参；倒车时守卫探测 180°；悬空物/细杆/低反射目标属传感器物理限制 |
| 急停确认延迟（3 帧 ≈150ms） | 中危 | **保留**：多帧确认是抗碎石/扬尘误触发的代价；确认期间已限速慢行，制动余量偏紧时请用 `goto ... speed` 主动降速 |
| 导航速度云端不可调 | 中低 | **已修复**：`goto` 支持可选 `speed`（m/s），现场降速无需改代码重启 |
| 回放限幅宽松 | 低 | **缓解**：回放/探路均过 `ObstacleGuard`，雷达掉线时由 fail-safe 兜底（见首行） |
| 低矮碎石/台阶判定依赖参数 | 低 | **已配置**：`BUNKER_STEP_LIMIT=0.07`（约离地间隙一半，默认值）已生效，可按实测调整 |
