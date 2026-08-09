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

### 5.3 SPACE 按下（Master 通过 TCP 触发 Slave）

```
[SPACE] (Master, recording_enabled == true)
  │
  ├─ Master: instantTrigger() → 所有相机开始录制
  ├─ Master: 通过 gaze_port 发送 TRIGGER → Slave
  │          Slave: 收到 TRIGGER → instantTrigger() → 回复 TRIGGER_ACK
  ├─ Master: 收到 TRIGGER_ACK → 状态："Recording..."
  │
  ├─ 等待 all_done (Master 和 Slave 各自的相机 dump_ready == true)
  │
  ├─ === 仅 Master ===
  │   ├─ armIdx()++ (当前录制已完成，推进 piper sentry)
  │   ├─ updateSentry() (piper sentry)
  │   │
  │   ├─ 自动移动到下一个目标：
  │   │   loop:
  │   │     ├─ 若当前臂已完成 → 停止循环
  │   │     ├─ 发送 MOVE_TO → 等待 MOVED/ERROR
  │   │     ├─ MOVED → computeToolCcs() → break
  │   │     └─ ERROR → armIdx()++ → updateSentry() → 继续 loop
  │   │
  │   ├─ 发送 GAZE:x,y,z → Slave → 等待 GAZE_ACK
  │   │
  │   └─ (此时 gaze 已为**下一次**录制准备好)
  │
  ├─ === Master + Slave ===
  │   ├─ Step 0: 预创建 HDF5 文件 (若需要)
  │   ├─ Step 1: 启动子进程 (传入 gaze_x gaze_y gaze_z)
  │   ├─ Step 2: WaitForMultipleObjects
  │   ├─ Step 3: 检查退出码 → CloseHandle(hJob)
  │   ├─ Step 4: Sentry 握手 (TCP port+300)
  │   └─ Step 5: g_frame_offset += core_frames → updateSentry (HDF5 sentry)
  │
  └─ 状态："Ready - press SPACE to record"
```

### 5.4 关键时间点

```
时间线 (一次录制):
  T0: [SPACE] → instantTrigger()
  T1: 所有相机 dump_ready (录制完成, ~1s)
  T2: Master 移动机械臂 (MOVE_TO, ~0.5-2s)
  T3: Master 转发 GAZE → Slave (TCP, ~1ms)
  T4: 子进程启动 → HDF5 写入 (~1s, 多进程并发)
  T5: Sentry 握手 (~0.1s)
  T6: 可接受下一次 SPACE

总周期: ~3-4s / 次录制
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

## 六、子进程修改（hdf5_multi_process_child.cpp）

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
