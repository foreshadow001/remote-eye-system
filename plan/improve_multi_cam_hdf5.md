# test_multi_cam_multi_host_hdf5.cpp 设计计划

> 基于 `test_multi_cam_multi_host.cpp` 复制并改造，将 raw 数据写入 HDF5 文件
> 最后更新：2026-08-02

---

## 一、目录与文件结构

### 1.1 帧数说明

`total_record_frames = core_frames + 2 × margin_frames`（如 200 + 20 = 220）。其中 margin_frames 用于跨相机 BlockID 对齐，**实际写入 HDF5 的帧数是 `core_frames`**（如 200）。对齐裁剪逻辑与现有的 `enable_intersection` 相同——取中间 core_frames 帧。

### 1.2 参与者目录（每相机独立路径）

与 `test_multi_cam_multi_host.cpp` 的 `save_dir` 一致——每台相机可指定不同的存储路径。仅一份 `sentry.h5`，放在列表第一个目录下。

```
D:/capture/P001/                 # 相机 0-4 的根目录，也是 sentry 所在
  sentry.h5                      # 唯一的写入位置追踪
  40768742/                      # 相机 SN 子目录
    0000.h5
    0001.h5
  40774056/
    0000.h5
  ...
E:/capture/P001/                 # 相机 5-9 的根目录
  40774064/
    0000.h5
  ...
```每次录制的帧按顺序**追加**到当前 chunk 文件末尾。多
次录制共享同一个 chunk 文件，直到它满 2000 帧再创建新文件。

### 1.2 配置文件 `capture.yaml`

`default.yaml` 中 `test_multi_cam` 节点整体迁移至此，不再读取 `default.yaml`。

```yaml
# capture.yaml — test_multi_cam_multi_host_hdf5.cpp 和 test_load_hdf5_frame.cpp 共用

# === 录制节点（采集脚本使用） ===
capture:
  participant_root:               # 每相机独立保存路径，sentry 放在第一个目录
    - "D:/capture/P001"
    - "D:/capture/P001"
    - "D:/capture/P001"
    - "D:/capture/P001"
    - "D:/capture/P001"
    - "E:/capture/P001"
    - "E:/capture/P001"
    - "E:/capture/P001"
    - "E:/capture/P001"
    - "E:/capture/P001"

  cam_indices:
    - "40768742"
    - "40774056"
    - "40768741"
    - "40774063"
    - "40774057"
    - "40774064"
    - "40768744"
    - "40768743"
    - "40773101"
    - "40772283"

  hardware_trigger: true
  fps: 200
  gain: 0.0
  gamma: 1.0
  exposure_time: 4000.0
  cam_width: 2448
  cam_height: 2048
  record_time: 1.0
  margin_frames_ratio: 0.05
  max_num_buffer: 30
  ui_fps: 20.0
  window_width: 1600
  window_height: 800
  enable_offset: false
  enable_intersection: false

  # 网络同步
  enable_net_sync: true
  is_master: false
  master_ip: "192.168.10.1"
  slave_ip: "192.168.10.2"
  port: 49200

  # HDF5
  hdf5_chunk_frame_capacity: 2000

# === 加载节点（验证脚本使用） ===
loader:
  participant_root: "D:/capture/P001"

  cam_indices:
    - "40768742"
    - "40774056"
    # ... 同上

  cam_width: 2448
  cam_height: 2048
```

- `test_multi_cam_multi_host_hdf5.cpp` 读取 `cfg["capture"]`
- `test_load_hdf5_frame.cpp` 读取 `cfg["loader"]`

---

## 二、HDF5 文件结构

### 2.1 数据文件 `<SN>/NNNN.h5`

每个相机独立拥有自己的 HDF5 chunk 文件。文件内三个 **extendible dataset**：

```
/NNNN.h5
  ├─ /raw_image/data       (M, H, W)  dtype=uint8      M ≤ 2000，初始 M=0
  ├─ /gaze_target/data     (M, 2)     dtype=float64    注视点 (x,y)，全部初始化为 0
  └─ /valid/data           (M,)       dtype=uint8      0 或 1，全部初始化为 0
```

- 三个 dataset 创建时**一次性预分配**到 (2000, H, W) / (2000, 2) / (2000,)，全部填 0。
- 写帧时用 hyperslab 选择写入位置 `[g_frame_offset + i, :, :]`，不需要 extend。
- 达到 2000 帧后此文件封存，创建下一个 chunk。

**为什么不用 per-frame dataset（`/raw_image/0`, `/raw_image/1`...）**：2000 个独立 dataset 会导致 HDF5 元数据膨胀，打开/遍历极慢。单个 extendible dataset 追加写入是 HDF5 的标准高效模式。

### 2.2 哨兵文件 `sentry.h5`

```
/sentry.h5
  ├─ chunk_idx      scalar int    当前 chunk 编号（0, 1, 2...）
  └─ frame_offset   scalar int    当前 chunk 内已写入帧数（0 ~ 1999）
```

### 丢帧处理

所有相机始终写入相同数量的帧（`core_frames`）。若某相机丢帧（该 BlockID 的数据缺失），写入一张全零图像，并将 `valid` 对应位置置 0。正常帧 `valid=1`。

这样每台相机每次录制写入相同的帧数，全局 `frame_offset` 总是一致的，sentry 只需两个标量。

---

## 三、句柄管理：启动时打开，全程持有

为避免反复打开/关闭 HDF5 文件的开销，**程序启动时为每台相机打开当前 chunk 文件的句柄**，存入 `CameraContext`。

### 3.1 CameraContext 新增字段

```cpp
// 全局（所有相机相同）
int g_chunk_idx = 0;
int g_frame_offset = 0;

struct CameraContext {
    // ... 现有字段 ...

    // ---- HDF5 ----
    string hdf5_dir;               // <participant_root>/<SN>/
    H5::H5File* hdf5_file;         // 当前已打开的 HDF5 文件句柄
    H5::DataSet hdf5_raw_ds;       // raw_image/data dataset
    H5::DataSet hdf5_gaze_ds;      // gaze_target/data dataset
    H5::DataSet hdf5_valid_ds;     // valid/data dataset
};
```

### 3.2 句柄生命周期

```
程序启动
  └─ initSentry() → 读取/创建 sentry.h5，得到每台相机的 chunk_idx / frame_offset
  └─ 对每台相机：
       └─ 打开（或创建） <SN>/<chunk_idx四位数>.h5
       └─ 打开/创建三个 extendible dataset
       └─ 句柄存入 ctx->hdf5_file / hdf5_raw_ds / ...

录制 #1
  └─ dumpToHdf5Worker: 用 ctx 中已有句柄写入 hyperslab
  └─ 主线程更新 g_frame_offset += core_frames
  └─ 写 sentry

录制 #2 ... 同上
  └─ 若 g_frame_offset + core_frames > 2000 → 录制前先换 chunk

程序退出
  └─ 关闭所有 hdf5_file 句柄
```

---

## 四、dumpToHdf5Worker 写入流程

```cpp
void dumpToHdf5Worker(shared_ptr<CameraContext> ctx,
                       const vector<cv::Mat>& ram_buffer,
                       int total_frames,
                       atomic<int>& finished_cams,
                       atomic<bool>& need_new_chunk,  // 输出：是否需要创建新 chunk
                       int& remaining_old,             // 输出：旧 chunk 还能写多少帧
                       int& remaining_new)             // 输出：新 chunk 需要写多少帧
```

**内部逻辑**:

```
1. 确定本次写入量 N = core_frames
2. 使用 BlockID 交集逻辑，确定每台相机哪些帧有效（valid=1）、哪些帧缺失（valid=0，写全零图）
3. 用 hyperslab 写入三个 dataset 在 `[g_frame_offset : g_frame_offset+N, :, :]` 位置
4. flush dataset
5. finished_cams++
```

> 写入位置 `g_frame_offset` ~ `g_frame_offset + core_frames` 始终在当前 chunk 范围内（录制前已检查，见第六章）。

所有相机 dump 完成后，主线程统一执行：
```cpp
g_frame_offset += core_frames;
updateSentry(participant_root);
```

> **注意**：步骤 4（跨 chunk 写入）极少发生。一次录制 core_frames 帧（如 200），chunk 容量 2000，需要
连续 9 次录制恰好踩在边界才会触发。但逻辑必须正确处理。


## 五、sentry 读写

### 5.1 启动时初始化

程序启动后，读取 `sentry.h5`。若文件不存在（首次运行），创建并初始化为 0。

```cpp
void initSentry(const string& participant_root) {
    fs::create_directories(participant_root);
    string sp = participant_root + "/sentry.h5";
    if (fs::exists(sp)) {
        H5::H5File f(sp, H5F_ACC_RDONLY);
        g_chunk_idx    = readScalarInt(f, "chunk_idx");
        g_frame_offset = readScalarInt(f, "frame_offset");
    } else {
        H5::H5File f(sp, H5F_ACC_TRUNC);
        writeScalarInt(f, "chunk_idx", 0);
        writeScalarInt(f, "frame_offset", 0);
        g_chunk_idx = 0;
        g_frame_offset = 0;
    }
}
```

然后为每台相机打开当前 chunk 文件：

```cpp
void openAllChunks(const string& participant_root,
                   vector<shared_ptr<CameraContext>>& cam_ctxs) {
    for (auto& ctx : cam_ctxs) {
        ctx->hdf5_dir = participant_root + "/" + ctx->id;
        fs::create_directories(ctx->hdf5_dir);
        ctx->hdf5_file = openOrCreateChunk(ctx->hdf5_dir, g_chunk_idx,
                                            &ctx->hdf5_raw_ds,
                                            &ctx->hdf5_gaze_ds,
                                            &ctx->hdf5_valid_ds);
    }
}
```

`openOrCreateChunk` 负责：根据 `chunk_idx` 构造文件名（`0000.h5`），若文件存在则打开已有 dataset；否则创建新文件，一次性预分配三个 dataset 到 (2000, H, W)，全部填 0。返回文件句柄，三个 dataset 句柄通过输出参数返回。

### 5.2 录制完成后更新

每次录制结束后，`g_frame_offset` 增加了 `core_frames`。若超过 2000，`g_chunk_idx` 已递增、`g_frame_offset` 已回卷（见第六章）。只需将当前值写回 sentry：

```cpp
void updateSentry(const string& participant_root) {
    string sp = participant_root + "/sentry.h5";
    H5::H5File f(sp, H5F_ACC_RDWR);
    writeScalarInt(f, "chunk_idx", g_chunk_idx);
    writeScalarInt(f, "frame_offset", g_frame_offset);
}
```

耗时 < 1ms。两次录制之间发送 SHUTDOWN 崩溃也不会丢失位点，因为更新在每次录制完成后立即执行。

### 5.3 `chunk_idx` 和 `frame_offset` 的递增规则

#### 递增时机

| 时间点 | 代码位置 | 操作 |
|--------|----------|------|
| **录制完成后** | `main()` dump 完成后 | `g_frame_offset += core_frames`；若 `>= capacity`：`g_chunk_idx++`，`g_frame_offset -= capacity`；写 `updateSentry()` |

唯一递增点，没有预检。`g_frame_offset` 始终 ≤ capacity，因为每次溢出都立即回卷。

#### 初始化规则

| 条件 | 代码位置 | 操作 |
|------|----------|------|
| **程序启动** | `main()` 中 `initSentry(g_sentry_root)` | 读 `sentry.txt`。若文件存在：加载 `chunk_idx` 和 `frame_offset`。若文件不存在（首次运行）：初始化为 `(0, 0)` 并创建文件 |
| **每轮录制开始** | `instantTrigger()` | **不重置** sentry。sentry 是跨录制持久化的全局位点，类似"文件写入指针" |
| **程序退出** | 无操作 | sentry 已在每次录制完成后写入磁盘，退出时无需额外保存 |

**Master 和 Slave 各自独立初始化**：双方都调用 `initSentry()` 读取自己本地的 `sentry.txt`。若文件不存在则从 `(0,0)` 开始。双方不交换初始化值——各自对自己的本地相机负责。

**重启动后恢复**：sentry.txt 是纯文本文件，持久化在磁盘上。程序崩溃或正常退出后重启，自动从上次的位点继续写入，不会覆盖已有数据。

#### 什么情况下递增

**每台相机的 `dump_ready` 机制**：

每台相机在 `copyWorker` 中独立计数。当该相机的 `recorded_frames` 达到 `total_record_frames`（如 220）时，该相机设置 `ctx->dump_ready = true`。注意 `total_record_frames` 包含 margin 帧（220 = 200 core + 20 margin），但最终写入 HDF5 的只有 `core_frames`（200 帧，取中间部分）。

主循环的 `all_done` 检查遍历所有相机，**所有相机**的 `dump_ready` 都为 true 时才进入 dump 阶段：

```cpp
bool all_done = true;
for (auto& ctx : cam_ctxs)
    if (!ctx->dump_ready.load()) { all_done = false; break; }
if (all_done) {
    // → dump → g_frame_offset += core_frames
}
```

因此递增条件是：

所有相机都成功写满 `ram_buffer`（`all_done == true`）→ 进入 dump 阶段。dump 阶段的完整流程：

```
all_done == true               ← 10 台相机全部 dump_ready
  │
  ├─ is_recording = false
  ├─ is_dumping = true          ← 健康检查在此阶段被跳过
  ├─ g_recording_number++
  │
  ├─ for (每个 ctx : cam_ctxs):              ← 串行，一台接一台
  │     dumpToHdf5Worker(ctx, core_frames, margin_frames)
  │        │
  │        ├─ ctx->dump_start_time = now()
  │        ├─ 打开/创建 <hdf5_dir>/<g_chunk_idx>.h5
  │        ├─ 逐帧写入 raw_image[g_frame_offset : g_frame_offset+core_frames]
  │        ├─ 写入 gaze_target（全 0）
  │        ├─ 写入 valid（全 1）
  │        ├─ H5File 析构关闭
  │        └─ ctx->dump_end_time = now()
  │
  ├─ dump_duration = now() - dump_start_time
  │
  ├─ g_frame_offset += core_frames   ← 核心：sentry 位点前移
  │     │
  │     └─ 若 g_frame_offset >= capacity:
  │           g_chunk_idx++
  │           g_frame_offset -= capacity
  │
  ├─ updateSentry(g_sentry_root)     ← 将新位点写入 sentry.txt
  │
  ├─ 若 is_master: fastUdpSend("SENTRY:<chunk_idx>:<frame_offset>")
  │
  ├─ writeReport(...)                ← 写 session log
  ├─ cout << "Done in X.Xs"
  │
  ├─ 重置 last_frame_time（防健康检查误判）
  └─ is_dumping = false             ← 恢复健康检查
```

**关键点**：`g_frame_offset += core_frames` 在**所有相机 dump 全部成功完成之后**才执行。若某台相机的 HDF5 写入中途失败（被 try-catch 捕获），该相机的 ram_buffer 数据丢失，但 dump 循环继续，`g_frame_offset` 仍然递增。该相机在 HDF5 文件中对应位置为零（数据集创建时的默认填充值），`valid` 亦为零。

**注意**：`g_frame_offset` 递增的是 `core_frames`（200），不是 `total_record_frames`（220）。margin 帧仅用于跨相机 BlockID 对齐，不写入 HDF5，不计入 sentry。

#### 什么情况下不递增

| 场景 | `g_frame_offset` | `g_chunk_idx` | 说明 |
|------|-----------------|---------------|------|
| 相机故障（FAULT） | 不递增 | 不递增 | 录制被中断，`all_done` 永远不会为 true，dump 代码块不执行 |
| 程序崩溃/强杀 | 不递增 | 不递增 | `updateSentry()` 未执行，sentry.txt 保持上一次的值 |
| 网络丢包导致 Slave 未收到 `CMD_START` | 不递增 | 不递增 | Slave 未触发录制 |

#### Master 和 Slave sentry 不一致的两种场景

Master 和 Slave 各自独立维护 sentry，各自对自己的相机负责。不一致可能发生在两个时间点：

**场景一：刚打开程序时（启动时）**

```
上一次运行：
  Master: 录制 #1(offset 0→200) → 录制 #2(offset 200→400) → updateSentry → 正常退出
  Slave:  录制 #1(offset 0→200) → 录制 #2 中途崩溃 → sentry 停在 200
  
本次启动：
  Master: initSentry() → 读 sentry.txt → (0, 400)
  Slave:  initSentry() → 读 sentry.txt → (0, 200)   ← 不一致！
```

原因：上一次运行时 Slave 在某次录制中崩溃，`updateSentry()` 未执行，sentry 落后于 Master。

**场景二：一次录制完成后（运行时）**

```
本次录制：
  Master: 10 台相机全部正常，dump 完成 → g_frame_offset += 200 → updateSentry → (0, 400)
  Slave:  某台相机 HDF5 写入失败（磁盘满/权限问题）→ 异常被 catch → dump 循环继续
          → g_frame_offset += 200 → updateSentry → (0, 400)
          
  表面上看数值相同。但如果 Slave 相机在录制期间有 BlockID 跳变（丢帧但未触发 FAULT），
  Slave 实际有效帧数可能少于 Master。两者的 frame_offset 相同但"实际数据质量"不同。
  
  更严重的情况：Slave 磁盘满导致 HDF5 创建失败（H5Fcreate failed），
  异常被 catch 后 ram_buffer 数据丢失，但 sentry 仍然前移。
  该相机的 HDF5 文件中对应位置全是零。
```

#### 统一处理方法

**比较时要综合考虑 `chunk_idx` 和 `frame_offset`**，不能只看 `frame_offset`。将两者转换为全局帧号：

```
master_total = master_ci × capacity + master_fo
slave_total  = slave_ci  × capacity + slave_fo
```

**采用较小值**（回退到落后的一方），用落后方的数据覆盖领先方的对应位置。这样可以覆盖可能无效的数据、节约存储空间：

```
若 master_total > slave_total:
    → Master 领先，回退到 Slave 位点
    → 双方都采用 (slave_ci, slave_fo)
    → 下一轮录制从该位点重新写入，覆盖 Master 之前可能无效的数据

若 slave_total > master_total:
    → Slave 领先，回退到 Master 位点
    → 双方都采用 (master_ci, master_fo)
```

**连续 3 次不一致 → ERROR + 退出**：

维护全局计数器 `g_sentry_mismatch_count`。每次 sentry 不一致时 `++`，相同时清零。达到 3 次时：

```
logException("FATAL", "sentry",
    "Sentry mismatch 3 consecutive times. Possible hardware fault. Exiting.")
global_running = false
```

防止硬件故障（如某台相机持续丢帧）导致无限循环回退。

```
Slave dump 完成后:
  ├─ g_frame_offset += core_frames
  ├─ updateSentry(local)
  └─ 等待 SENTRY 消息到达
       │
       ├─ 收到 master_ci, master_fo
       ├─ master_total = master_ci × capacity + master_fo
       ├─ slave_total  = slave_ci  × capacity + slave_fo
       │
       ├─ 若 master_total == slave_total → 正常，g_sentry_mismatch_count = 0
       │
       └─ 若不同:
             ├─ g_sentry_mismatch_count++
             ├─ 采用 min(master_total, slave_total) → 回退到落后方
             ├─ 双方都设置为较小值对应的 (chunk_idx, frame_offset)
             ├─ logException("WARN", "sentry", "mismatch #" + to_string(count)
             │     + ": Master(" + ... + ") Slave(" + ... + "), using min")
             ├─ updateSentry(local)  // 重写本地 sentry.txt
             │
             └─ 若 g_sentry_mismatch_count >= 3:
                   logException("FATAL", "sentry", "3 consecutive mismatches, exiting")
                   global_running = false
```

**为什么采用较小值**：领先方的最后一段数据可能无效（例如 Slave 某相机 HDF5 写入失败导致全零帧）。回退后重新录制，用新的有效数据覆盖。代价是浪费一段存储空间（最多 `core_frames` 帧），但保证数据质量。

#### 当前代码中 `g_frame_offset` 是 `atomic<int>`

这是早期并发 dump 线程的遗留。现在已改为串行 for 循环，`atomic` 不再必要，但保留也不影响正确性。注意 `g_frame_offset += core_frames` 在主线程中执行，`g_frame_offset` 仅在 dump 线程中**读取**（串行 for 循环中每次调 `dumpToHdf5Worker` 时读取），无竞争。

### 5.4 Master/Slave 同步时序分析

#### 当前实现

**Master**（以一次录制为例，`core_frames=200`）：

| 时间点 | 操作 | g_chunk_idx | g_frame_offset |
|--------|------|-------------|----------------|
| T0 启动 | `initSentry()` 读 `sentry.txt` | 0 | 1600 |
| T1 录制前 | chunk 空间检查：`1600+200=1800 ≤ 2000`，无需换 chunk | 0 | 1600 |
| T2 录制 | `instantTrigger()` → copyWorker 写入 ram_buffer | 0 | 1600 |
| T3 dump 后 | `g_frame_offset += 200` → `1800`；`1800 ≤ 2000`，无需换 chunk；`updateSentry()` 写 `0\n1800\n` | 0 | 1800 |
| T4 同步 | `fastUdpSend("SENTRY:0:1800")` | 0 | 1800 |

**Slave**：

| 时间点 | 操作 | g_chunk_idx | g_frame_offset |
|--------|------|-------------|----------------|
| T0 启动 | `initSentry()` 读 `sentry.txt` | 0 | 1600 |
| T1 | 收到 `CMD_START` → `instantTrigger()` | 0 | 1600 |
| T2 dump 后 | `g_frame_offset += 200` → `1800`；`updateSentry()` 写 `0\n1800\n` | 0 | 1800 |
| T3 | 收到 `SENTRY:0:1800` → 覆盖 `g_chunk_idx=0, g_frame_offset=1800`，写 `sentry.txt` | 0 | 1800 |

正常情况下 T2 和 T3 的值相同。SENTRY 消息起到"对账"作用——若 slave 因丢帧导致偏移落后，SENTRY 可修正；若 slave 的 UDP 消息丢失，自身 T2 也能正确更新。

#### 当前问题

1. **SENTRY 延迟到达**：Master 的 SENTRY 通过 UDP 发送，可能在下一次录制中途才到达 slave。这不影响正确性——slave 自身已在 T2 更新了 sentry，SENTRY 只是二次确认。但控制台日志会显示 SENTRY 出现在下一段录制的 I/O PREPARED 之后。

2. **slave 多录一次**：上一轮测试中 slave 出现了 11 次录制而 master 只有 10 次。根因是 `net_cmd_record` 被旧 SENTRY 或其他 UDP 消息意外触发。`SENTRY` 消息不应触发录制——需在 `udpListenerWorker` 中确认 `SENTRY` 不会被误当作 `CMD_START`。

#### 修改后方案：各自维护 + 握手校验

Master 和 Slave **完全独立**地按相同逻辑维护自身 `(chunk_idx, frame_offset)`，录制完成后双方均更新本地 sentry。更新完毕后，Master 发送自己的值给 Slave，Slave 与自身值比对。

> **核心原则**：Master 和 Slave 各自对自己本地的相机负责。若 Slave 相机丢帧，Slave 的 `frame_offset` 应反映真实写入量，不能被 Master 的值无声覆盖。

**修改后时序**：

| 时间点 | Master | Slave |
|--------|--------|-------|
| T0 启动 | `initSentry()` 读本地 `sentry.txt` | `initSentry()` 读本地 `sentry.txt` |
| T1 | 按 'r' → `fastUdpSend("CMD_START:...")` | 收到 `CMD_START` → `instantTrigger()` |
| T2 | 录制 + dump → `g_frame_offset += core_frames` | 录制 + dump → `g_frame_offset += core_frames` |
| T3 | 若溢出 → `g_chunk_idx++, g_frame_offset -= capacity` | 同上 |
| T4 | `updateSentry()` 写本地 | `updateSentry()` 写本地 |
| T5 | `fastUdpSend("SENTRY:<chunk_idx>:<frame_offset>")` | — |
| T6 | — | 收到 `SENTRY`，**比对**本地的 `(chunk_idx, frame_offset)` |

**T6 比对逻辑（Slave 端）**：

```
收到 SENTRY:<master_ci>:<master_fo>
    本地 slave_ci, slave_fo

若 master_ci == slave_ci && master_fo == slave_fo:
    → 正常，无需操作

若 master_ci != slave_ci || master_fo != slave_fo:
    → [WARN] 打印差异，采用 Master 值（Master 为权威），
       但保留 Slave 值记录到 session log（用于事后排查 Slave 相机故障）
       g_chunk_idx = master_ci; g_frame_offset = master_fo;
       重写本地 sentry.txt
```

**为什么差异意味着 Slave 相机故障**：
- Master 的 `frame_offset` 反映 Master 相机组的真实写入帧数
- Slave 的 `frame_offset` 反映 Slave 相机组的真实写入帧数
- 两者应始终相等。若不等，说明某一端有相机丢帧或写入失败
- 采用 Master 值保证 sentry 位点一致（下次录制从同一位置开始），但**记录差异到 session log** 用于事后审计

---

**修改后对比**：

| | 旧方案 | 新方案 |
|---|--------|--------|
| Slave sentry 维护 | 自身更新 + 被 Master 覆盖 | 自身独立更新 + Master 发来的值做比对校验 |
| 差异处理 | 静默覆盖 | 打印 WARN + 记录到 session log + 采用 Master 值 |
| 故障发现 | 无 | Slave 相机丢帧时可从 session log 发现 `frame_offset` 不一致 |

---

## 六、跨 chunk 写入的处理

**不会出现跨 chunk 场景，不做处理。**

录制开始前（`instantTrigger` 之前），主线程检查：

```cpp
if (g_frame_offset + core_frames > 2000) {
    // 当前 chunk 空间不足 → 关闭所有相机当前文件 → 创建新 chunk
    for (auto& ctx : cam_ctxs) closeChunk(ctx);
    g_chunk_idx++;
    g_frame_offset = 0;
    for (auto& ctx : cam_ctxs)
        openOrCreateChunk(ctx->hdf5_dir, g_chunk_idx, &ctx->hdf5_raw_ds, &ctx->hdf5_gaze_ds, &ctx->hdf5_valid_ds);
    updateSentry(participant_root);  // 立即持久化新位点
}
```

这样 `dumpToHdf5Worker` 始终面对一个剩余空间充足的 chunk，无需内部处理跨 chunk 逻辑，代码大幅简化。

---

## 七、需要移除的功能

| 移除项 | 原因 |
|--------|------|
| `dumpToDiskWorker` | 替换为 `dumpToHdf5Worker` |
| `convertRawToJpgWorker` | 数据存 HDF5，无需 JPG |
| `write_jpg` 及所有 JPG 相关代码路径 | 同上 |
| `save_dir` YAML 配置 | 改为 `participant_root` |
| `default.yaml` 中 `test_multi_cam` 节点 | 整体迁移至 `capture.yaml`，不再读取 `default.yaml` |
| `temp_dir`、`log_file_path`、`log_stream`（CameraContext 中） | 不再需要 raw 文件日志 |

---

## 八、保留的功能（从 `test_multi_cam_multi_host.cpp` 完整迁移）

以下功能一字不改地保留在 HDF5 版本中：

| 保留项 | 涉及代码 | 说明 |
|--------|----------|------|
| `ram_buffer` 预分配 + copyWorker 录制 | CameraContext, copyWorker(), instantTrigger() | 录制流程完全不变 |
| **全部 metrics** | CameraContext 中所有指标字段 + copyWorker 采集 + writeReport 输出 | Per-Camera 表格 + Summary 表格全部保留 |
| **会话日志** | session log (`log/capture/session_*.md`) | 启动创建，多次录制追加写入 |
| **异常处理** | `logException()` + 四级计数器 + E1-E12 检查 | 全部保留，Summary 表格输出异常计数 |
| 网络同步 | CMD_START / FAULT / SHUTDOWN / CLEAR | UDP 协议不变 |
| 网络同步按键控制 | Master/Slave 键盘屏蔽逻辑 | ESC/q/r/SPACE/c 全部保留 |
| UI 渲染 | 5×2 缩略图 + 放大视图 + 水印 + 十字线 | 不变 |
| 健康检查 | 1s 无帧 → 故障 | 不变 |
| CPU 异常处理 | `logException()` + 计数器 | 不变 |

---

## 九、测试脚本 `test_load_hdf5_frame.cpp`

加载指定参与者、指定全局帧号的画面（所有相机），转为 JPG 到输出目录。

```
test_load_hdf5_frame.exe <participant_root> <global_frame_index> <output_dir>
```

**逻辑**：
1. 从 `capture.yaml` 的 `loader` 节点读取 `participant_root`、`cam_indices`、`cam_width`、`cam_height`
2. 读取 `sentry.h5` 获取当前位点
2. 对每台相机：
   - 计算 `global_frame_index` 落在哪个 chunk（`chunk_idx = global / 2000`）
   - 计算 chunk 内偏移（`offset = global % 2000`）
   - 打开 `<SN>/<chunk_idx四位数>.h5`
   - 读取 `raw_image[offset, :, :]` → 若 mono 直接存 JPG，若 color 先 Bayer→BGR
   - 将 JPG 写入 `<output_dir>/frame_<N>_<SN>.jpg`

---

## 十、实施步骤

### 10.1 test_multi_cam_multi_host_hdf5.cpp

| 序号 | 内容 | 详细说明 | 预计 |
|------|------|----------|------|
| 1 | 复制文件 | `cp test_multi_cam_multi_host.cpp → test_multi_cam_multi_host_hdf5.cpp` | 1min |
| 2 | 添加 `#include <H5Cpp.h>` | 头文件 | 1min |
| 3 | 添加 HDF5 全局变量 | `g_chunk_idx`, `g_frame_offset`, `g_participant_root`, `g_hdf5_chunk_capacity` | 1min |
| 4 | CameraContext 新增字段 | `hdf5_dir`, `hdf5_file`, `hdf5_raw_ds`, `hdf5_gaze_ds`, `hdf5_valid_ds` | 2min |
| 5 | 添加 HDF5 辅助函数（main 之前） | `h5ReadInt`, `h5WriteInt`, `openOrCreateChunk`, `closeChunk`, `initSentry`, `updateSentry` | 15min |
| 6 | 添加 `dumpToHdf5Worker`（替换 `dumpToDiskWorker`） | ram_buffer → HDF5 hyperslab 写入 | 15min |
| 7 | 修改 `main()` 配置加载 | `Cfg cfg("capture.yaml")` + `cfg["capture"]` | 5min |
| 8 | 修改 `main()` — 移除 | `save_dirs`、`write_jpg`、`convertRawToJpgWorker`、`dumpToDiskWorker` | 5min |
| 9 | 修改 `main()` — 添加 | HDF5 init（sentry + open chunks）→ 录制前 chunk 空间检查 → dump 改为 HDF5 → 录制后 updateSentry → 退出前 close chunks | 10min |
| 10 | CMakeLists 添加 target + HDF5 依赖 | `find_package(HDF5 REQUIRED COMPONENTS CXX)` + `target_link_libraries(... ${HDF5_LIBRARIES})` | 10min |
| 11 | 编译调试 | 通过 MSVC 编译零警告 | 15min |

### 10.2 附属文件

| 序号 | 内容 | 说明 | 预计 |
|------|------|------|------|
| 12 | 创建 `capture.yaml` | `capture` + `loader` 两个节点 | 3min |
| 13 | 实现 `test_load_hdf5_frame.cpp` | 加载指定帧号 → JPG 输出 | 15min |
| 14 | 运行时验证 | 完整录制 + 加载测试 | 15min |

**总计：~2 小时**
