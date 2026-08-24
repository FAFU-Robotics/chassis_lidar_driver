# 机械臂 ↔ 底盘「抓取完成」信号桥接（arm_bridge）

任务链（`find_object`）完整流程为：**自主探路 → 视觉找目标 → 导航到位 →
机械臂对接 → [新增]等待机械臂抓取完成 → 沿轨迹返回起点**。

此前代码在机械臂就位（`grasp_ready`）后**立即返回**，没有等待机械臂真正完成
抓取。本包补齐这个环节：车体就位停稳后，等待机械臂发出「抓取完成」信号，
收到信号才返回起点。

## 一、机械臂方如何反馈「抓取完成」

四种通道，**至少启用一条**即可（可同时启用多条，任一先到即生效，抗单点失效）：

| 通道 | 机械臂方需要做什么 | 优点 / 场景 |
|---|---|---|
| **云端指令 `grasp_done`**（推荐首选） | 抓取完成后，经甲方云端向底盘下发 `{"action":"grasp_done"}`；联调时在 `mock_cloud` 控制台输入 `gd` | 零硬件接线；机械臂控制器能上网/能调用云端接口即可 |
| **CAN 报文** | 并入底盘同一 socketcan 总线，发送标准帧 `0x3A1`（可配置），`data[0]=0x01` | 实时性最高；机械臂有 CAN 口时走总线最干净 |
| **GPIO 电平** | 抓取完成后把 Jetson 40-pin 指定引脚拉高（默认 pin 18，可配置高低有效） | 完全独立于网络，最抗干扰；需拉一根信号线 |
| **联调文件** | 抓取完成后 `echo grasped > /tmp/grasp.sig`（可配路径） | 无需硬件即可端到端联调；机械臂控制器是工控机时最省事 |

协议细节：

- **云端**：action 名 `grasp_done`，无附加参数。底盘收到后任务链状态
  `holding → grasped`，随后自动返回起点。
- **CAN**：帧 ID `0x3A1`（标准帧，在底盘协议 0x111~0x441 之外，不会误解析），
  数据 8 字节，第 0 字节 `0x01` 表示抓取完成。可用
  `--arm-grasp-signal "can:can1:0x3A1"` 修改通道/帧 ID。
- **GPIO**：默认 BOARD 编号 pin 18、高电平有效（`--arm-grasp-signal "gpio:18:high"`；
  低电平有效写 `low`）。边沿触发 + 0.15s 消抖确认，防止电平抖动误触发。
- **文件**：内容（忽略空白，大小写不敏感）为 `1` / `grasped` / `done` /
  `true` / `ok` 之一即视为抓取完成。

## 二、底盘侧如何启用

```bash
# 车侧代理启动时指定信号通道（逗号分隔多通道）：
python3 run_agent.py --arm-grasp-signal "ws,file:/tmp/grasp.sig" --arm-wait-timeout 90
```

或者写进 `agent.conf`：

```ini
ARM_GRASP_SIGNAL=ws,can:can1:0x3A1
ARM_WAIT_TIMEOUT=90
```

参数：

| 配置 | 说明 | 默认 |
|---|---|---|
| `ARM_GRASP_SIGNAL` / `--arm-grasp-signal` | 信号通道规格，逗号分隔；`none` 或留空 = 关闭机械臂等待（恢复旧行为：就位即返回） | 空（关闭） |
| `ARM_WAIT_TIMEOUT` / `--arm-wait-timeout` | 等待抓取完成的最长秒数 | `90` |

信号规格段格式：

```
ws                        # 云端 grasp_done 指令
file:/tmp/grasp.sig       # 联调文件
can:can1:0x3A1            # CAN 通道:帧ID（帧ID 支持 0x 十六进制）
gpio:18:high              # GPIO 引脚:有效电平（high|low）
```

## 三、行为与安全

- 就位后车体**完全停稳**等待（`state.payload.mission.status = holding`），
  不会影响机械臂作业；
- 收到信号 → `grasped` → 推 `grasped` 事件 → 沿探路轨迹返回起点；
- **超时**（超过 `ARM_WAIT_TIMEOUT`）→ 推 `grasp_timeout` 事件，
  `mission = failed`，车停在原地不动——此时由操作员判断：机械臂可能已抓成但
  信号没到（可重新下发 `gd`），也可能抓取失败（可下发 `cancel` 召回返回）；
- **取消**：等待期间云端 `cancel` / `estop` 会立即打断等待并按各自语义停车/
  返回，不会傻等满超时；
- **无信号通道**时行为与旧版完全一致（就位后立即返回），默认不改变现有行为。

## 四、联调步骤（无机械臂硬件）

```bash
# 终端 A — 模拟云端
python3 mock_cloud.py

# 终端 B — 车侧代理（启用文件信号通道，便于手动触发）
BUNKER_WS_URL=ws://127.0.0.1:9000 python3 run_agent.py \
    --arm-grasp-signal "ws,file:/tmp/grasp.sig" --arm-wait-timeout 120

# 终端 A — 下发任务（approach 参数开启机械臂对接阶段）
fo 目标A a

# 车就位（看到 mission=holding / 可抓取）后，任选一种方式模拟机械臂完成抓取：
#   方式一（云端）：终端 A 输入 gd
#   方式二（文件）：echo grasped > /tmp/grasp.sig
#   方式三（模拟脚本）：
python3 arm_bridge/simulate_arm.py --file /tmp/grasp.sig

# 预期：mission holding → grasped → returning，小车沿轨迹返回起点
```

## 五、真机接线（GPIO 通道）

1. 机械臂控制器选择一路可编程 IO，抓取完成后输出有效电平；
2. 信号线接入 Jetson 40-pin 对应引脚（默认 BOARD pin 18）与 GND；
3. `pip install Jetson.GPIO`（需 root）；指定 `--arm-grasp-signal "gpio:18"`；
4. 用 `simulate_arm.py --gpio 18` 可先行验证底盘侧通路。

## 六、代码结构

```
arm_bridge/
├── feedback.py        # 事件/结果枚举 + 后端抽象接口
├── ws_backend.py      # 云端 grasp_done 指令通道
├── can_backend.py     # CAN 0x3A1 报文通道
├── gpio_backend.py    # Jetson GPIO 电平通道
├── file_backend.py    # 联调文件通道
├── arm_bridge.py      # ArmBridge 桥接层 + 规格解析
├── simulate_arm.py    # 机械臂信号模拟器（联调用）
└── test_arm_bridge.py # 单元测试
```
