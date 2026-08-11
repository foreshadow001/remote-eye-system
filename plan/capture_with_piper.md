# capture_with_piper.cpp — 采集 + 机械臂联动

> 整合 `hdf5_multi_process.cpp` 的多进程 HDF5 采集流程 与 `test_piper_ctrl.cpp` 的机械臂 TCP 控制
> 最后更新：2026-08-10

## 概括

Master 端同时承担**相机采集**和**机械臂控制**两个角色。每次录制前，Master 先将机械臂移动到目标位置，收到实际位姿后通过独立 TCP 端口转发给 Slave，然后双方同时录制、dump 到 HDF5。Slave 不接触机械臂——仅从 Master 接收 gaze_target 后写入自己的 HDF5。

```
Master                                           Slave                    Ubuntu
  │                                                │                        │
  │─ 相机初始化 (Pylon)                               │─ 相机初始化              │
  │─ TCP 连接 Ubuntu (:49301)                        │                        │─ piper_windows_ctrl_server
  │─ 启动 Gaze Server (:49302)                       │─ TCP 连接 Master :49302 │
  │─ 双机械臂回零 (upper+lower)                        │                        │
  │                                                │                        │
  │─ [s] 等待相机 STREAMING                           │                        │
  │─ [s] MOVE_TO upper → 收到 MOVED ─────────────────────────────────────→│─ move_to()
  │─ [s] 发送 GAZE:x,y,z → Slave ──────────────────→│─ 存储 gaze            │
  │─ [s] ← GAZE_ACK ───────────────────────────────│                        │
  │  (此时 SPACE 生效)                                │                        │
  │                                                │                        │
  │─ [SPACE] 录制 (core_frames 帧)                     │─ [SPACE] 录制           │
  │─ dump2RAM 完成                                   │─ dump2RAM 完成          │
  │─ ARM + HDF5 并行:                                 │─ HDF5 写入              │
  │   ├─ HDF5 子进程写入 (含旧 gaze)                    │─ 发 HDF5_DONE → Master │
  │   └─ 移臂到下一目标 (并行)                           │                        │
  │─ 双方 HDF5 都完成                                  │                        │
  │─ GAZE 转发 → Slave                               │─ 收 GAZE → 回复 ACK    │
  │─ Sentry 握手                                     │─ Sentry 握手           │
  │  ... 循环 ...                                    │  ... 循环 ...           │
  │                                                │                        │
  │─ [ESC] 双机械臂回零 → SHUTDOWN → 退出              │─ 退出                  │─ 退出
```

---

## 一、涉及文件

（同前）

---

## 二、TCP 端口分配

| 端口 | 名称 | 用途 | 方向 |
|------|------|------|------|
| `49300` | pose port | `windows_end_monitor.py` (GET_POSE) | Windows → Ubuntu |
| `49301` | ctrl port | `piper_windows_ctrl_server.py` (MOVE_JOINTS/MOVE_TO/SHUTDOWN) | Master → Ubuntu |
| `49302` | gaze port | GAZE 转发 | Master → Slave |
| `49303` | cmd port | TRIGGER / EXIT / FAULT / **HDF5_DONE** / **GAZE_DONE** | Master ↔ Slave |
| `net_port+300` | sentry port | HDF5 Sentry 握手 | Master ↔ Slave |

**新增两个 cmd 指令**：

| 指令 | 方向 | 含义 |
|------|------|------|
| `HDF5_DONE` | Slave → Master | Slave 所有子进程完成 HDF5 写入 |
| `GAZE_DONE` | Master → Slave | Master 已发送 GAZE，Slave 可进入 sentry 握手 |

---

## 三、dump 阶段：ARM 与 HDF5 并行

### 3.1 并行阶段

```
all_done=true
  |
  +-- 并行阶段 (Master: ARM || HDF5, Slave: HDF5 only):
  |     Master:
  |       save rec_gaze = 旧值
  |       ├─ 线程1: arm_thread → idx++ → moveArmToTarget()
  |       └─ 主线程: precreate .h5 → CreateProcess → WaitForMultipleObjects
  |       arm_thread.join()
  |     Slave:
  |       save rec_gaze = 旧值
  |       precreate .h5 → CreateProcess → WaitForMultipleObjects
  |       send "HDF5_DONE" → Master (via cmd_port)
  |     UI: "ARM + HDF5 STAGE"
  |
  +-- 同步阶段 (串行, 双方都等 HDF5 完成):
  |     Master: wait Slave "HDF5_DONE"
  |     Master: send GAZE:x,y,z → Slave (via gaze_port) → wait GAZE_ACK
  |     Master: send "GAZE_DONE" → Slave (via cmd_port)
  |     Slave:  wait GAZE (from gazeClientWorker)
  |     Slave:  wait "GAZE_DONE" (from cmdWorker)
  |     UI: "SYNC STAGE"
  |
  +-- Sentry 握手 (port+300):
  |     交换 local_total → 取 min → increment sentry
  |
  +-- SPACE ready
```

### 3.2 时序（预期）


### 3.3 Slave ARM 阶段的含义

```
Master ARM time = max(arm_movement, HDF5_write) + slave_HDF5_wait(if needed)
Slave ARM time = HDF5_write + wait_for_Master_GAZE

如果 arm ~3.5s, HDF5 ~3.5s → 并行阶段 ~3.5s (两者重叠)
同步阶段: GAZE TCP ~0.1s, sentry TCP ~0.1s
总 dump 时间: ~3.5 + 0.2 = ~3.7s (之前串行 ~7s)
```


## 四、按键定义

| 按键 | 作用 | 可用条件 |
|------|------|----------|
| `s` | Start：等待相机 STREAMING → 移动机械臂到 sentry 点位 → 启用录制 | 仅 Master，相机已 STREAMING |
| `SPACE` | 触发一次录制 | `s` 已按下后 |
| `t` | 切换机械臂 (upper↔lower)，park 当前臂 | 仅 Master |
| `b` | 当前臂回零 | 仅 Master |
| `c` | 清空 piper sentry | 仅 Master |
| `ESC/q` | 双机械臂回零 → SHUTDOWN → 退出 | 任意时刻 |
# capture_with_piper — plan

> 最后更新：2026-08-11

## 六、指令耗尽处理

### 6.0 piper sentry 递增方式改进

### 现状

`moveArmToTarget()` 被两处调用：

| 调用位置 | 当前代码 |
|---------|---------|
| `s` 键（首次移动） | `moveArmToTarget()` |
| dump 入口 ARM 阶段 | `idx++; moveArmToTarget()` |

`moveArmToTarget()` 内部在跳过无解目标时已会 `idx++`，成功后不递增。

### 改进

**递增统一放在调用方，且仅在 `moveArmToTarget()` 返回 `ArmResult::OK` 后执行**：

```
新规则:
  调用 moveArmToTarget()
    → OK         → idx++ → updatePiperSentry()
    → EXHAUSTED  → idx 不变（臂完成）
    → ERROR      → idx 不变，logException
    内部跳过无解目标时已自行 idx++（逻辑不变）
```

**两处调用方修改**：

| 位置 | 修改前 | 修改后 |
|------|--------|--------|
| `s` 键 | `moveArmToTarget()` | `if(moveArmToTarget()==OK){idx++; updatePiperSentry();}` |
| dump 入口 | `idx++; moveArmToTarget()` | `if(moveArmToTarget()==OK){idx++; updatePiperSentry();}` |

### 效果

- `idx` 严格指向**已成功抵达的目标数量**，不再出现 sentry=8 越界的情况
- 臂完成时 `idx` 停在最后一个成功目标的索引，不递增到无效值
- `sentry.txt` 值始终有效

### 6.1 启动时越界检查

在渲染 capture UI（10 台相机缩略图 + enlarged 区域）**之前**，检查当前激活臂是否已耗尽。

```
程序启动 → 读取 piper sentry
  │
  ├─ sentry.upper >= targets_upper.size() ？
  │   是 → upper_done = true
  │
  ├─ sentry.lower >= targets_lower.size() ？
  │   是 → lower_done = true
  │
  ├─ 当前激活臂 (默认 upper) done？
  │   是 → 渲染 EXHAUSTED UI，不渲染 capture UI
  │         "upper inst exhausted. Press 't' to switch to lower or 'c' to clear sentry."
  │
  ├─ 按 t → 切换到 lower → 重新检查 lower
  │   若 lower 也 done → 显示 "lower inst exhausted. Press 'c' to clear sentry."
  │                            (两台主机都显示)
  │
  └─ 按 c → 清零 piper sentry → upper_done/lower_done = false → 渲染 capture UI
```

**关键**：Slave 没有 piper 状态。Master 需要在初始化阶段通过 cmd_port 将 `upper_done`/`lower_done` 状态发送给 Slave。

### 6.2 跨主机通信

在 startup sentry handshake **之后**、渲染 capture UI **之前**，Master 发送 piper 状态给 Slave：

```
Master:  sendLineRaw(g_cmd_sock, "PIPER:upper:8:done")   // 或 "PIPER:upper:3:ok"
Slave:   cmdWorker recv → 设置 g_upper_done/g_upper_idx
```

Slave 的 cmdWorker 新增 `PIPER:` 指令处理：

```
PIPER:upper:8:done  → g_upper_idx=8, g_upper_done=true
PIPER:upper:3:ok    → g_upper_idx=3, g_upper_done=false
PIPER:lower:0:ok    → g_lower_idx=0, g_lower_done=false
```

同样，录制过程中 Master 切换臂、清空 sentry 等操作也需要通过 cmd_port 同步给 Slave。

### 6.3 EXHAUSTED UI 设计

不渲染 10 台相机画面，使用全屏黑底 + 大白字 + 操作提示：

```
┌─────────────────────────────────────────┐
│                                         │
│        UPPER INST EXHAUSTED             │
│                                         │
│   Press 't' to switch to lower          │
│   Press 'c' to clear piper sentry       │
│   Press 'q' to quit                     │
│                                         │
│   Upper: 8/8 done                       │
│   Lower: 3/8 ok                         │
│                                         │
└─────────────────────────────────────────┘
```

如果两臂都耗尽：

```
│        BOTH ARMS EXHAUSTED              │
│   Press 'c' to clear piper sentry       │
│   Press 'q' to quit                     │
```

### 6.4 录制后越界检查（调用 moveArmToTarget 之前）

一次录制完成后、dump 的 ARM 阶段启动**之前**，检查 piper sentry 是否已越界。越界则跳过 ARM 阶段，HDF5 阶段照常进行。HDF5 完成后（两台主机都完成，sync 逻辑不变），UI 显示 EXHAUSTED UI，不再渲染 capture UI。

```
all_done=true
  │
  ├─ 检查: idx >= total ?
  │   否 → 正常: 启动 ARM 线程 + HDF5 并行
  │   是 → 跳过 ARM: 仅执行 HDF5 阶段
  │         HDF5 完成后 → sync（GAZE 不更新，直接用旧值）
  │         → sentry 握手 → 显示 EXHAUSTED UI
  │         → 等待用户按 t 或 c
```

**关键**：越界时 `moveArmToTarget` 不调用，所以 6.0 节的递增逻辑不触发。旧 gaze 值保持不变，HDF5 照常写入（写入最后一次成功位置的 gaze）。Slave 通过 PIPER 状态得知越界，也显示 EXHAUSTED UI。

### 6.5 moveArmToTarget() 调用后的越界检查

6.4 处理的是**调用前**已越界（跳过 ARM 阶段）。6.5 处理的是**调用后**返回结果的判断。

`moveArmToTarget()` 当前返回 `bool`，无法区分"指令用完"和"其他异常"。改为三态：

```cpp
enum class ArmResult { OK, EXHAUSTED, ERROR };
```

| 返回值 | 含义 | 触发条件 |
|--------|------|---------|
| `OK` | 成功移动到一个可解目标 | MOVED 响应 |
| `EXHAUSTED` | 所有目标尝试完毕，无可用指令 | `while(idx < total)` 循环耗尽 |
| `ERROR` | 其他异常（网络断开、硬件故障等） | TCP 超时、响应格式错误 |

#### 6.5.1 录制结束时（dump ARM 阶段）

arm 线程调用 `moveArmToTarget()`，根据返回值：

```
OK        → idx++（调用方按 6.0 递增）, 线程结束
EXHAUSTED → idx 不变, 设置 g_show_exhausted=true, 线程正常结束
ERROR     → idx 不变, logException("ERROR",...), 线程结束
```

**关键**：`EXHAUSTED` 时不立即显示 exhausted UI。ARM 线程结束后，HDF5 阶段照常进行（因为 RAM 中的数据仍需写入磁盘）。HDF5 + sync + sentry 全部完成后，主线程检查 `g_show_exhausted`，若为 true 则渲染 EXHAUSTED UI 而非 capture UI。Slave 通过 Master 在 sync 阶段后发送的 `PIPER` 状态得知越界，也显示 EXHAUSTED UI。

#### 6.5.2 按 's' 时

`s` 键调用 `moveArmToTarget()`，根据返回值：

```
OK        → idx++（调用方按 6.0 递增）, g_recording_enabled=true
EXHAUSTED → 显示 EXHAUSTED UI, g_recording_enabled=false
ERROR     → 显示错误提示, 不进入录制
```

---

## 七、M5Stack LED 状态识别

第一阶段仅在 Master 的 enlarged 区域左上角显示当前状态的颜色和名称，不涉及真实的 M5Stack 通信。

**状态判定仅在进入主循环后生效**。主循环之前（TCP 握手、双回零、相机初始化、startup sentry handshake）不设置 LED 状态——此时机械臂尚未进入工作流程。

### 7.1 状态枚举

```cpp
enum class LedState { PIPER_INIT, READY, CAPTURING, WAITING, EXHAUSTED, OVER };
LedState g_led_state = LedState::PIPER_INIT;
```

| 状态 | 颜色 | LED 效果 | 含义 |
|------|------|---------|------|
| `PIPER_INIT` | 蓝 | 常亮 | 等待按下 `s` 开始采集 |
| `READY` | 绿 | 饼图倒计时 | `s` 已按下，可采集，等待 SPACE |
| `CAPTURING` | 绿 | 呼吸灯 | 正在录制 |
| `WAITING` | 黄 | 常亮 | 录制结束，dump 进行中，等待下次 SPACE |
| `EXHAUSTED` | 红 | 常亮 | 当前臂指令用尽，等待 `t` 或 `c` |
| `OVER` | 彩 | 横向流转 | 两臂均完成，实验可结束 |

### 7.2 状态转移

```
          ┌──────────┐
          │   PIPER_INIT   │ ← 程序启动 / 按 t 切换臂后
          │   蓝     │
          └────┬─────┘
               │ [s] 成功 → moveArmToTarget() → GAZE 转发完成
               ▼
          ┌──────────┐
          │  READY   │ ← g_recording_enabled=true, 等 SPACE
          │  绿饼图   │
          └────┬─────┘
               │ [SPACE] → instantTrigger()
               ▼
          ┌──────────┐
          │CAPTURING │ ← is_recording=true
          │ 绿呼吸灯  │
          └────┬─────┘
               │ all_done=true → is_dumping=true
               ▼
          ┌──────────┐
          │ WAITING  │ ← dump (ARM+HDF5+sentry) 进行中
          │   黄     │
          └────┬─────┘
               │ is_dumping=false
               ├─── 正常 ────────────────→ READY
               ├─── g_show_exhausted ───→ EXHAUSTED
               │
          ┌───┴──────┐
          │EXHAUSTED │ ← 当前臂指令用尽
          │   红     │
          └────┬─────┘
               │ [t] → 切换臂成功 ──────→ PIPER_INIT (新臂)
               │ [c] → 清 sentry ───────→ PIPER_INIT
               │ 另一臂也 done ──────────→ OVER
               │
          ┌───┴──────┐
          │  OVER    │ ← g_upper_done && g_lower_done
          │  彩流转   │
          └──────────┘
               │ [c] → 清 sentry ───────→ PIPER_INIT
```

### 7.3 各状态的精确进入和退出条件

#### PIPER_INIT

| 进入 | 退出 |
|------|------|
| 程序主循环首次进入（`g_recording_enabled=false`） | `s` 键：`moveArmToTarget()` 返回 `ARM_OK` 且 GAZE 转发完成 |
| `t` 键：切换臂后 | |
| `c` 键：清空 sentry 后 | |
| `EXHAUSTED` + `t` 切换到非耗尽臂 | |

**条件**：`is_master_pc && !g_recording_enabled && !g_show_exhausted && !(g_upper_done && g_lower_done)`

#### READY

| 进入 | 退出 |
|------|------|
| `PIPER_INIT` + `s` 成功 | SPACE 按下（`trigger_start=true`） |

**条件**：`is_master_pc && g_recording_enabled && !is_recording && !is_dumping`

#### CAPTURING

| 进入 | 退出 |
|------|------|
| `READY` + SPACE | `all_done=true`（is_dumping 变 true） |

**条件**：`is_recording && !is_dumping`

#### WAITING

| 进入 | 退出 |
|------|------|
| `CAPTURING` + `all_done=true` | `is_dumping=false` |

**条件**：`is_dumping`

#### EXHAUSTED

| 进入 | 退出 |
|------|------|
| `WAITING` + `g_show_exhausted=true` | `t` 切换到可用臂 |
| 启动时 piper sentry 越界 | `c` 清空 sentry |
| `s` 键 `moveArmToTarget()` 返回 `ARM_EXHAUSTED` | |

**条件**：`g_show_exhausted && !(g_upper_done && g_lower_done)`

#### OVER

| 进入 | 退出 |
|------|------|
| `g_upper_done && g_lower_done`（两臂均耗尽） | `c` 清空 sentry |

**条件**：`g_upper_done && g_lower_done`

### 7.4 状态优先级

主循环中按优先级从高到低判断，确保不重叠：

```cpp
if (g_show_exhausted) {
    if (g_upper_done && g_lower_done) g_led_state = OVER;
    else                              g_led_state = EXHAUSTED;
} else if (is_dumping) {
    g_led_state = WAITING;
} else if (is_recording) {
    g_led_state = CAPTURING;
} else if (g_recording_enabled) {
    g_led_state = READY;
} else {
    g_led_state = PIPER_INIT;
}
```

### 7.5 UI 显示

在 enlarged 区域左上角显示一个彩色矩形 + 状态名：

```
┌──────────────────────────────┐
│ ┌──┐ READY                  │  ← enlarged 区域左上角
│ └──┘                        │
│  绿色方块                    │
│                             │
│     (相机画面)               │
│                             │
└──────────────────────────────┘
```
