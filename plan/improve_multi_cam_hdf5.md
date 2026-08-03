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
