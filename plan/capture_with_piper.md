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
