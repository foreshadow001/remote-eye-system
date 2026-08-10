# capture_with_piper.cpp — 采集 + 机械臂联动

> 整合 `hdf5_multi_process.cpp` 的多进程 HDF5 采集流程 与 `test_piper_ctrl.cpp` 的机械臂 TCP 控制
> 最后更新：2026-08-09

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
  │─ 发送下一目标 MOVE_TO → 收到 MOVED ─────────────────────────────────→│
  │─ 发送 GAZE → Slave ────────────────────────────→│─ 存储 gaze            │
  │─ ← GAZE_ACK ───────────────────────────────────│                        │
  │─ HDF5 多进程写入 (含 gaze)                        │─ HDF5 多进程写入 (含 gaze)│
  │─ Sentry 握手                                     │─ Sentry 握手           │
  │  ... 循环 ...                                    │  ... 循环 ...           │
  │                                                │                        │
  │─ [ESC] 双机械臂回零 → SHUTDOWN → 退出              │─ 退出                  │─ 退出
```

---

## 一、涉及文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `cpp_eyetracker/tests/utils/cam/capture_with_piper.cpp` | **新建** | Master 端主程序 |
| `cpp_eyetracker/tests/utils/cam/hdf5_multi_process.cpp` | **参考** | 多进程 HDF5 采集流程（复用其 dump 逻辑） |
| `cpp_eyetracker/tests/utils/piper/test_piper_ctrl.cpp` | **参考** | 机械臂 TCP 控制（复用其 arm 控制逻辑） |
| `cpp_eyetracker/tests/utils/cam/hdf5_multi_process_child.cpp` | **修改** | 子进程新增 `gaze_x gaze_y gaze_z` 参数，写入 gaze_target |
| `cpp_eyetracker/tests/utils/cam/CMakeLists.txt` | **修改** | 新增 `capture_with_piper` target |
| `piper_ros/.../piper_windows_ctrl_server.py` | **不变** | 已在 test_piper_ctrl 阶段完成 |
| `cpp_eyetracker/cfg/piper.yaml` | **修改** | 新增 `gaze_port` |
| `piper_ros/.../cfg/net.yaml` | **修改** | 新增 `gaze_port` |

---

## 二、TCP 端口分配

| 端口 | 名称 | 用途 | 方向 | 生命周期 |
|------|------|------|------|----------|
| `49300` | pose port | `windows_end_monitor.py` (GET_POSE) | Windows → Ubuntu | 持久 |
| `49301` | ctrl port | `piper_windows_ctrl_server.py` (MOVE_JOINTS/MOVE_TO/SHUTDOWN) | Master → Ubuntu | 持久 |
| **`49302`** | **gaze port** | **GAZE 转发**（仅位姿数据） | Master → Slave | 持久 |
| **`49303`** | **cmd port** | **TRIGGER（触发录制）+ EXIT（退出）** | Master → Slave | 持久 |
| `net_port + 300` | sentry port | Sentry 握手 | Master ↔ Slave | 按需（每次录制后） |

**设计理由**：GAZE 转发的时机在"机械臂移动到位后、HDF5 写入前"，而 TRIGGER 在"用户按 SPACE 时"、EXIT 在"程序退出时"。三个指令的触发时机和调用栈完全不同，分端口可以各自使用独立的阻塞 send/recv 调用，互不干扰。

---

## 三、Master→Slave TCP 协议

### 3.1 Gaze 转发（gaze_port 49302）

**唯一职责**：在机械臂移动到位后、HDF5 写入前，将 tool-in-CCS 位姿从 Master 转发到 Slave。

```
Master (Gaze Server)                    Slave (Gaze Client)
  │─ listen 0.0.0.0:49302                  │─ connect master_ip:49302
  │─ accept() ←────────────────────────────│
  │                                         │
  │  (机械臂移动到位后)                        │
  │─ send: GAZE:x,y,z\n ─────────────────→│─ recv → 存入 g_gaze_target_x/y/z
  │←────────────────────── GAZE_ACK\n ─────│─ send ACK
  │  (Master 阻塞等待 ACK 后才继续 HDF5 写入)   │
```

| 指令 | 格式 | 响应 | 时机 |
|------|------|------|------|
| `GAZE` | `GAZE:x,y,z` | `GAZE_ACK` | 每次 MOVE_TO 成功后、HDF5 写入前 |

### 3.2 录制控制（cmd_port 49303）

**职责**：触发 Slave 开始录制 + 通知 Slave 退出。

```
Master (Cmd Server)                      Slave (Cmd Client)
  │─ listen 0.0.0.0:49303                  │─ connect master_ip:49303
  │─ accept() ←────────────────────────────│
  │                                         │
  │  (用户按 SPACE)                           │
  │─ send: TRIGGER\n ─────────────────────→│─ recv → instantTrigger()
  │←────────────────────── TRIGGER_ACK\n ──│─ send ACK
  │  (Master 收到 ACK 后自己也 instantTrigger) │
  │                                         │
  │  (用户按 ESC/q)                           │
  │─ send: EXIT\n ────────────────────────→│─ recv → global_running=false
  │←────────────────────── EXIT_ACK\n ──────│─ send ACK
```

| 指令 | 格式 | 响应 | Slave 动作 |
|------|------|------|-----------|
| `TRIGGER` | `TRIGGER` | `TRIGGER_ACK` | `instantTrigger()` |
| `EXIT` | `EXIT` | `EXIT_ACK` | `global_running = false` |

**时序**：Master 先发 `TRIGGER` 并等待 `TRIGGER_ACK`，收到后自己再调用 `instantTrigger()`——保证 Master 和 Slave 几乎同时开始录制。

---

## 四、按键定义

| 按键 | 作用 | 可用条件 |
|------|------|----------|
| `s` | **Start**：等待相机 STREAMING → 移动机械臂到 sentry 点位 → 启用录制 | 仅 Master，相机已 STREAMING |
| `SPACE` | 触发一次录制 (core_frames 帧) | `s` 已按下后 |
| `t` | 切换机械臂 (upper↔lower)，park 当前臂 | 仅 Master |
| `b` | 当前臂回零（仅物理回零，不重置进度） | 仅 Master |
| `c` | 清空 piper sentry（两臂进度归零），不清空录制目录 | 仅 Master |
| `ESC/q` | 双机械臂回零 → SHUTDOWN → Master+Slave 退出 | 任意时刻 |

**注意**：Slave 端无需 `s/t/b/c`——所有机械臂按键在 Slave 端无效。

---

## 五、核心时序

### 5.1 启动阶段

```
Master                                   Slave
  │                                        │
  ├─ 加载 capture.yaml + piper.yaml         ├─ 加载 capture.yaml
  ├─ Pylon 初始化 → 相机 STREAMING            ├─ Pylon 初始化 → 相机 STREAMING
  ├─ 连接 Ubuntu :49301                     │
  ├─ 启动 Gaze Server :49302                ├─ 连接 Master :49302
  ├─ 读取 piper sentry → 恢复进度             │
  ├─ zeroArm("upper") → zeroArm("lower")    │
  └─ 进入主循环                              └─ 进入主循环
```

### 5.2 s 键按下（仅 Master）

```
[s] 按下
  │
  ├─ 检查：所有相机 status == STREAMING？
  │   否 → 显示 "Waiting for cameras..." → 返回
  │   是 → 继续
  │
  ├─ 检查：当前臂是否已完成？
  │   是 → 显示 "Arm done - press T to switch" → 返回
  │
  ├─ 从 sentry 获取当前目标点位
  ├─ 发送 MOVE_TO:<arm>:x,y,z → 阻塞等待 MOVED/ERROR
  │   ├─ MOVED → 提取实际位姿 → computeToolCcs()
  │   ├─ ERROR:no_solution → armIdx()++ → updateSentry() → 重试下一个
  │   └─ Timeout → 显示错误 → 返回
  │
  ├─ 发送 GAZE:x,y,z → Slave → 阻塞等待 GAZE_ACK
  ├─ 更新 UI：显示当前目标、实际位姿、Tool in CCS
  ├─ 标记 recording_enabled = true
  └─ 状态："Ready - press SPACE to record"
```

### 5.3 dump 阶段：ARM 阶段 + HDF5 阶段（串行）

录制完成后（all_done=true），dump 分为两个严格串行的阶段。每个阶段 Master 和 Slave 都通过 TCP 握手确认同步后再进入下一阶段。

#### ARM 阶段

**Master**：
  1. 保存当前 `g_gaze_x/y/z`（本次录制的值，供 HDF5 写入）
  2. `idx++` → `updatePiperSentry()`
  3. `moveArmToTarget()` → 阻塞等 TCP 响应（含 auto-skip 无解目标）
  4. 收到 MOVED → `computeToolCcs()` → 更新 `g_gaze_x/y/z`（下一轮的值）
  5. 通过 gaze_port 发送 `GAZE:x,y,z` → Slave
  6. 等待 Slave 回复 `GAZE_ACK`
  7. UI 显示 "ARM STAGE" + 当前 gaze

**Slave**：
  1. 保存当前 `g_gaze_x/y/z`（本次录制的值，供 HDF5 写入）
  2. 等待 Master 通过 gaze_port 发送下一轮的 `GAZE:x,y,z`
  3. 收到后存入 `g_gaze_x/y/z` → 回复 `GAZE_ACK`
  4. UI 显示 "ARM STAGE"

#### HDF5 阶段

ARM 阶段完成后，双方 gaze 已同步为下一轮的值，本次录制的 gaze 已保存。进入 HDF5 写入：

**Master + Slave 共同**：
  1. 预创建 .h5 文件（若跨 chunk）
  2. CreateProcess 启动子进程（传入**保存的**本次录制 gaze）
  3. WaitForMultipleObjects
  4. Sentry 握手 (port+300)：交换 `local_total`，取 min 对齐
  5. `g_frame_offset += core_frames` → `updateSentry()`
  6. UI 显示 "DUMPING RAM TO HDF5..."

#### 时序图

```
all_done=true
  |
  +-- ARM 阶段 (Master+Slave, serial sync):
  |     Master: save gaze -> idx++ -> moveArmToTarget -> recv MOVED
  |             -> send GAZE:x,y,z -> wait GAZE_ACK
  |     Slave:  save gaze -> recv GAZE -> send GAZE_ACK
  |     耗时: ~3.5s (arm) + TCP (<0.1s)
  |
  +-- HDF5 阶段:
  |     precreate -> CreateProcess -> WaitForMultipleObjects -> sentry handshake
  |     耗时: ~3.5s (首次 ~15s)
  |
  +-- SPACE ready
```

#### 关键：gaze 新旧分离

dump 开始时先保存旧值，arm 移动更新新值，HDF5 写旧值：

```
rec_gaze = g_gaze_load()         ← 本次录制 arm 所在位置的 gaze
moveArmToTarget()                ← g_gaze 更新为下一轮位置
GAZE 转发给 Slave                ← Slave 同步更新
CreateProcess(rec_gaze)          ← HDF5 写入本次录制的 gaze
```

ARM 阶段的 TCP 阻塞确保了 HDF5 阶段启动时双方 gaze 已同步。

```

### 5.5 移动失败自动跳过

```
当前目标无解:
  ERROR:upper:no_solution
    → armIdx()++ (跳过)
    → updateSentry()
    → 立即取下一个目标 → 发送 MOVE_TO
    → 继续 loop 直到找到可解目标 或 该臂全部尝试完毕

若该臂所有剩余目标都无解:
    → 标记该臂 done
    → 提示 "UPPER/LOWER all targets exhausted"
    → 等待用户按 t 切换臂
```

### 5.6 退出时序

```
[ESC/q] (Master)
  │
  ├─ zeroArm("upper") → 阻塞
  ├─ zeroArm("lower") → 阻塞
  ├─ 发送 SHUTDOWN → Ubuntu :49301
  ├─ 发送 EXIT → Slave (via gaze_port :49302) → 等待 EXIT_ACK
  ├─ 关闭所有相机线程
  ├─ closesocket / WSACleanup
  └─ 退出

[ESC/q] (Slave)
  │
  ├─ 关闭所有相机线程
  ├─ 关闭 gaze 客户端连接
  ├─ closesocket / WSACleanup
  └─ 退出
```

---

### 5.7 dump2RAM 结束到下一次录制就绪 —— 实测时序分析

#### Master 端实测数据（连续 7 次录制）

```
#1: arm=3.53s  launch=3.79s  wait=14.99s  sentry=2.35s  TOTAL=21.16s
#2: arm=3.88s  launch=4.14s  wait=3.07s   sentry=3.11s  TOTAL=10.33s
#3: arm=2.56s  launch=2.83s  wait=3.59s   sentry=0.08s  TOTAL=6.51s
#4: arm=3.03s  launch=3.30s  wait=3.41s   sentry=3.72s  TOTAL=10.46s
#5: arm=3.49s  launch=3.75s  wait=3.51s   sentry=3.29s  TOTAL=10.57s
#6: arm=3.79s  launch=4.04s  wait=3.45s   sentry=3.05s  TOTAL=10.55s
#7: arm=3.35s  launch=3.62s  wait=3.49s   sentry=3.56s  TOTAL=10.67s
```

#### Slave 端实测数据（同步的 7 次录制）

```
#1: arm=0.03s  launch=0.05s  wait=14.09s  sentry=7.02s  TOTAL=21.20s
#2: arm=0.00s  launch=0.04s  wait=3.34s   sentry=7.04s  TOTAL=10.43s
#3: arm=0.00s  launch=0.04s  wait=3.48s   sentry=3.02s  TOTAL=6.56s
#4: arm=0.00s  launch=0.04s  wait=3.42s   sentry=7.04s  TOTAL=10.51s
#5: arm=0.00s  launch=0.04s  wait=3.51s   sentry=7.03s  TOTAL=10.58s
#6: arm=0.00s  launch=0.04s  wait=3.48s   sentry=7.01s  TOTAL=10.54s
#7: arm=0.00s  launch=0.04s  wait=3.60s   sentry=7.03s  TOTAL=10.69s
```

#### 各阶段说明

| 阶段 | 计时含义 | Master 耗时 | Slave 耗时 | 差异原因 |
|------|---------|------------|-----------|---------|
| `arm` | 移动机械臂（Slave 无此阶段） | **~3.4s** | 0s | Master 向 Ubuntu 发 MOVE_TO，阻塞等 TCP 响应 |
| `launch` | CreateProcess 启动 10 个子进程 | **~3.7s**（！） | **~0.04s** | **Master 异常慢，待排查** |
| `wait` | WaitForMultipleObjects 等子进程写完 | #1: ~15s 后 ~3.4s | #1: ~14s 后 ~3.4s | 首次冷缓存，后续稳定 |
| `sentry` | TCP 握手 port+300 | 0~4s | **~7s（几乎恒定）** | Slave 先到握手点，阻塞等 Master |

#### Master `launch` 阶段异常分析

Master 的 `launch` 耗时 ~3.7s，Slave 仅 ~0.04s——**差距约 100 倍**。两者执行完全相同的代码路径：`for` 循环中 `CreateProcessA` 启动 10 个 `hdf5_multi_process_child.exe`。

可能原因：
1. 子进程启动后向父进程的共享内存写入 HDF5 时，与父进程的 gaze server 线程或 cmd worker 线程争抢磁盘 IO
2. Master 端磁盘（D:/E:）同时承载录制目录和子进程 exe，IO 竞争比 Slave 更严重
3. Windows Defender 实时扫描子进程 exe
4. 第一次 CreateProcess 慢（exe 冷加载），后续应缓存

**待验证**：将子进程 exe 放到与 HDF5 写入不同的物理磁盘上，观察 `launch` 时间变化。

#### 首次录制「冷启动」效应

录制 #1 的 `wait` 在两台机器上都是 ~14-15s，远高于后续的 ~3.4s。这是首次创建 `.h5` 文件时的写放大：
- 首次 `H5F_ACC_TRUNC`：分配 `raw_image` dataset（2000×2048×2448 × 1byte ≈ 10GB 预留空间）
- 后续 `H5F_ACC_RDWR`：仅 hyperslab 写入 200 帧（~1GB 实际数据）
- HDF5 首次创建需要初始化内部 B-tree 和 metadata

#### Master-Slave 握手等待时序
#### 瓶颈排序与改进预期（修复后数据，launch=0.05s）

**第一次录制（#1，需创建 .h5）**：

Master（TOTAL = 21.1s）：
  T+0.00s  启动 arm 线程（并行） + precreate(0.03s)
  T+3.58s  arm 线程 join（arm 移动完成）
  T+3.63s  CreateProcess ×10 (launch=0.05s)
  T+3.68s  子进程开始 HDF5 写入
  T+18.25s 子进程完成 (wait=14.57s) —— 首次 H5F_ACC_TRUNC，分配 10GB raw_image
  T+20.96s Sentry 握手完成 (sentry=2.71s)
  T+21.14s 录制完成

Slave（TOTAL = 21.04s）：
  T+0.00s  precreate(0.03s) → CreateProcess(0.04s)
  T+0.07s  子进程开始 HDF5 写入
  T+14.01s 子进程完成 (wait=13.94s) —— 首次 H5F_ACC_TRUNC
  T+14.01s Sentry connect() → 阻塞，等 Master 到达
  T+21.04s 握手完成 (sentry=7.03s = 空等 4.3s + TCP 2.7s)

首次瓶颈：wait ~14s（双方），HDF5 首次创建文件时分配 10GB raw_image 空间。

**后续录制（#2，已有 .h5）**：

Master（TOTAL = 10.34s）：
  T+0.00s  启动 arm 线程（并行） + precreate(0s, 跳过)
  T+3.89s  arm 线程 join（arm 移动完成）
  T+3.95s  CreateProcess ×10 (launch=0.06s)
  T+4.01s  子进程开始 HDF5 写入
  T+6.96s  子进程完成 (wait=2.95s) —— H5F_ACC_RDWR，hyperslab 写 200 帧
  T+10.17s Sentry 握手完成 (sentry=3.21s)
  T+10.34s 录制完成

Slave（TOTAL = 10.48s）：
  T+0.00s  precreate(0s) → CreateProcess(0.04s)
  T+0.04s  子进程开始 HDF5 写入
  T+3.44s  子进程完成 (wait=3.40s)
  T+3.44s  Sentry connect() → 阻塞，等 Master 到达
  T+10.47s 握手完成 (sentry=7.03s = 空等 3.5s + TCP 3.5s)

后续瓶颈：Slave 在 sentry 空等 Master 3.5s（Master 先做 arm 再写 HDF5）

**Slave sentry=7.0s 的构成**：
  Slave 到握手点：wait(3.40s) = 3.40s
  Master 到握手点：arm(3.89s) + launch(0.06s) + wait(2.95s) = 6.90s
  Slave 空等：6.90 - 3.40 = 3.50s
  TCP 往返：~3.2s
  Slave sentry = 3.50 + 3.21 ≈ 6.7~7.0s ✓

**各阶段结论（后续录制）**：

| 阶段 | Master | Slave | 说明 |
|------|--------|-------|------|
| arm | 3.5s（并行） | 0s | Ubuntu MoveIt 规划+执行 |
| precreate | 0s | 0s | 已有文件时跳过 |
| launch | 0.06s | 0.04s | CreateProcess×10，正常 ✓ |
| wait | 3.0s | 3.5s | HDF5 磁盘 IO |
| sentry | 3.2s | 7.0s | Slave 空等 Master 3.5s |
| **TOTAL** | **10.3s** | **10.5s** | |

改进方向：arm 与 HDF5 写入（wait）并行化。当前 arm 与 precreate 并行（precreate=0s，无收益）。
若 arm 与 wait 重叠：Master 到握手点 = max(arm 3.9s, HDF5 3.0s) + launch + sentry ≈ 3.9 + 0.06 + 3.2 ≈ 7.2s，
Slave 不再空等 → sentry ≈ 3.2s → TOTAL ≈ 3.5 + 0.04 + 3.2 ≈ 6.7s（提速 35%）。


### 6.1 gaze_target dataset 改为 3 列

原定义为 `(capacity, 2)` → 改为 `(capacity, 3)`，存储 `(x, y, z)`：

```cpp
// 父进程预创建 (Step 0):
hsize_t gd[2] = {(hsize_t)g_hdf5_chunk_capacity, 3};  // 原: 2 → 3
f.createDataSet("gaze_target", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, gd));
```

### 6.2 子进程新增 gaze 参数

```
argv[10] = gaze_x (double)
argv[11] = gaze_y (double)
argv[12] = gaze_z (double)
```

写入逻辑：

```cpp
double gx = atof(argv[10]), gy = atof(argv[11]), gz = atof(argv[12]);
hsize_t gz_start[2] = {(hsize_t)frame_offset, 0};
hsize_t gz_count[2] = {(hsize_t)N, 3};  // 3 列
H5::DataSpace gz_mem(2, gz_count), gz_file = gaze_ds.getSpace();
gz_file.selectHyperslab(H5S_SELECT_SET, gz_count, gz_start);
vector<double> gz_buf(N * 3);
for (int i = 0; i < N; ++i) {
    gz_buf[i * 3]     = gx;
    gz_buf[i * 3 + 1] = gy;
    gz_buf[i * 3 + 2] = gz;
}
gaze_ds.write(gz_buf.data(), H5::PredType::NATIVE_DOUBLE, gz_mem, gz_file);
```

### 6.3 注意

- `hdf5_multi_process.cpp`（父进程）的 Step 0 预创建也需要同步改为 3 列
- `test_load_hdf5_frame.cpp` 加载 gaze 时也需要适配 3 列

---

## 七、Master 端新增组件

### 7.1 Gaze Server（独立线程）

```cpp
void gazeServerWorker(int port, atomic<double>& gx, atomic<double>& gy, atomic<double>& gz) {
    SOCKET listen_sock = socket(...);
    bind(listen_sock, port);
    listen(listen_sock, 1);
    SOCKET client = accept(listen_sock);
    while (g_running) {
        // Wait for main thread to set gaze values
        // send GAZE:x,y,z → wait GAZE_ACK
    }
}
```

实际上，Gaze Server 更简单的实现：在主线程的 dump 流程中，直接向 Slave 发送 gaze（阻塞式），不需要独立线程。

### 7.2 Master 端的 PipertCP 集成

从 `test_piper_ctrl.cpp` 移植：
- `recvLine` / `sendLine`
- `zeroArm` (带 busy 标志)
- `parsePoseResponse` (解析 MOVED 响应)
- `computeToolCcs` (计算 tool 在 CCS 中的位置)
- `updateSentry` (piper sentry 读写)

### 7.3 Slave 端的 Gaze Client

Slave 启动后连接 Master 的 `gaze_port`。每次录制前接收 `GAZE:x,y,z`，存储到 `g_gaze_target`，回复 `GAZE_ACK`。

---

## 八、Slave 端修改

基于 `hdf5_multi_process.cpp`，新增/修改：

1. **Gaze Client 线程**：启动后连接 Master `master_ip:49302`，循环接收 `GAZE` / `TRIGGER` / `EXIT` 指令
2. **全局变量**：`atomic<double> g_gaze_target_x/y/z`——收到 `GAZE` 后更新，子进程启动时读取
3. **录制触发**：收到 `TRIGGER` → `instantTrigger()` → 回复 `TRIGGER_ACK`
4. **按键屏蔽**：`enable_net_sync=true` 时 Slave 端 `r/s/SPACE` 全部无效（仅 ESC/q 有效）
5. **退出**：收到 `EXIT` → `global_running = false` → 回复 `EXIT_ACK` → 退出

**注意**：Slave 不接触任何 piper 相关代码——无 `piper.hpp`、无 TCP 连接 Ubuntu、无 arm 变量。

---

## 九、已确认的设计决策

| # | 决策 | 说明 |
|---|------|------|
| 1 | gaze_target 改为 3 列 `(x, y, z)` | 父进程预创建 + 子进程写入都改为 `(capacity, 3)` |
| 2 | Master→Slave 录制触发改为 **TCP** | 通过 `gaze_port` (49302) 发送 `TRIGGER`，Slave 收到后执行 `instantTrigger()`。Slave 端所有按键无效 |
| 3 | Gaze Server 绑定 `0.0.0.0:49302` | Slave 连接 `master_ip:49302`，与现有 TCP 模式一致 |

---

## 十、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 修改 `hdf5_multi_process_child.cpp`：新增 gaze 参数 | 5min |
| 2 | 新建 `capture_with_piper.cpp`（基于 `hdf5_multi_process.cpp`）：集成 arm TCP、gaze server、新按键逻辑 | 30min |
| 3 | 修改 Slave 端（`hdf5_multi_process.cpp` 或新建 slave 版本）：gaze client 接收 | 15min |
| 4 | 新增 `gaze_port` 到 `piper.yaml` 和 `net.yaml` | 2min |
| 5 | CMakeLists 新增 target | 3min |
| 6 | 编译 + 联调 | 20min |

**总计：~75 分钟**
