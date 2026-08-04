# hdf5_multi_process.cpp 多进程写入计划

> 解决问题：当前串行写入 10 个 HDF5 文件约 7.5 秒，通过多进程并发写入加速到 ~1.6 秒
> 基于 plan\improve_multi_cam_hdf5.md 的架构

---

## 零、概述

### 0.1 目标

将 `test_multi_cam_multi_host_hdf5.cpp` 中的串行 HDF5 写入改为多进程并发，10 台相机同时写入各自的 HDF5 文件，将 dump 耗时从 **~7.5 秒降至 ~1.6 秒**（约 4.7 倍加速）。

### 0.2 方案

- **父进程** `hdf5_multi_process.cpp`：负责录制（ram_buffer）、启动 10 个子进程、等待全部完成、更新 sentry、写 session log
- **子进程** `hdf5_multi_process_child.cpp`：每个子进程负责一台相机的 HDF5 写入，从共享内存读取 raw 数据，独立编译为 `.exe`
- **共享内存**：ram_buffer 直接分配在 Windows 命名共享内存上，copyWorker 的 `memcpy` 目标就是共享内存。子进程通过名称打开同一块共享内存，**全程零额外拷贝**。备选方案为 dump 时拷贝到共享内存（+10GB 内存，系统 40GB 可用）

### 0.3 为什么多进程而非多线程

HDF5 2.1.1 不是线程安全的——多线程同时调用 HDF5 API（哪怕操作不同文件）会导致 VOL wrapper 冲突。每个进程有独立的 HDF5 库实例，天然隔离。

### 0.4 与现有架构的关系

- **录制阶段**（copyWorker → ram_buffer）：完全不变，仅 ram_buffer 分配方式从 `cv::Mat::zeros` 改为共享内存映射
- **dump 阶段**：串行 for 循环替换为 `CreateProcess` → `WaitForMultipleObjects`
- **sentry / session log / 网络同步 / UI**：全部保留不变
- **TCP sentry 握手**：dump 完成后执行，与多进程无冲突

### 0.5 涉及文件

| 文件 | 说明 |
|------|------|
| `hdf5_multi_process.cpp` | 父进程，基于 `test_multi_cam_multi_host_hdf5.cpp` 改造 |
| `hdf5_multi_process_child.cpp` | 子进程，独立 `.exe`，负责单相机 HDF5 写入 |
| `capture.yaml` | 复用现有配置 |
| `plan/improve_multi_cam_hdf5_multiprocess.md` | 本计划 |

---

## 一、当前串行写入流程

```
main() dump 阶段:
  for (i = 0; i < 10; ++i):
    dumpToHdf5Worker(cam_ctxs[i], core_frames, margin_frames)
      │
      ├─ fs::create_directories(hdf5_dir)
      ├─ H5::H5File f(path, H5F_ACC_TRUNC or H5F_ACC_RDWR)
      │     └─ 首次创建: createDataSet raw_image(2000,H,W)
      │     └─ 已有文件: openDataSet raw_image
      ├─ for (j = 0; j < 200; ++j):  // 逐帧 hyperslab 写入
      │     raw_ds.write(ram_buffer[margin + j].data, ...)
      ├─ gaze_ds.write(...)     // 全零
      ├─ valid_ds.write(...)    // 全 1
      └─ H5File 析构 (close)
```

**耗时分解**（已有文件，200 帧）：
| 阶段 | 单相机耗时 | ×10 总耗时 |
|------|-----------|-----------|
| 打开文件 + 获取 dataset | ~0.05s | ~0.5s |
| raw_image 逐帧写入 | ~0.65s | ~6.5s |
| gaze + valid 写入 | ~0.01s | ~0.1s |
| flush + close | ~0.04s | ~0.4s |
| **合计** | **~0.75s** | **~7.5s** |

瓶颈在逐帧 hyperslab 写入（200 帧 × 5MB = 1GB 数据写入磁盘）。

---

## 二、为什么多进程可行而多线程不行

| | 多线程 | 多进程 |
|---|--------|--------|
| HDF5 线程安全 | ❌ HDF5 2.1.1 不是线程安全的（VOL wrapper 冲突） | ✅ 每个进程有独立的 HDF5 库实例 |
| 内存共享 | ✅ 同一地址空间，ram_buffer 直接访问 | ❌ 需要 IPC（共享内存/管道） |
| 进程管理 | 简单（std::thread） | 需要 CreateProcess + WaitForMultipleObjects |
| 崩溃隔离 | ❌ 一个线程崩溃，整个进程退出 | ✅ 一个子进程崩溃，父进程可检测并重试 |

结论：HDF5 的线程不安全是硬伤，多进程是唯一可行的并发方案。

---

## 三、多进程架构

### 3.1 共享内存设计（零额外拷贝）

**核心思路**：`ram_buffer` 直接分配在共享内存上，copyWorker 的 `memcpy` 写入的就是共享内存。子进程打开同一块共享内存读取，**全程零额外拷贝**。

#### 父进程改动：ram_buffer 分配在共享内存

```cpp
// 原代码：
ctx->ram_buffer[k] = cv::Mat::zeros(cam_h, cam_w, CV_8UC1);

// 新代码：分配在共享内存（含 PID 防多实例冲突）
string shm_name = "HDF5_" + to_string(GetCurrentProcessId()) + "_CAM_" + to_string(i);
size_t total_bytes = total_record_frames * cam_h * cam_w;
HANDLE hMap = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
                                  0, (DWORD)total_bytes, shm_name.c_str());
uint8_t* shm_base = (uint8_t*)MapViewOfFile(hMap, FILE_MAP_WRITE, 0, 0, total_bytes);
for (int k = 0; k < total_record_frames; ++k) {
    // cv::Mat 只包装共享内存地址，不管理生命周期
    ctx->ram_buffer[k] = cv::Mat(cam_h, cam_w, CV_8UC1, shm_base + k * cam_h * cam_w);
}
ctx->shm_handle = hMap;   // 新增：保存句柄用于清理
ctx->shm_base = shm_base;  // 新增：保存基地址用于 UnmapViewOfFile
```

> **关键**：`cv::Mat(rows, cols, type, data)` 构造函数不会接管内存所有权。当 `ram_buffer` vector 析构时，`cv::Mat` 析构函数不会释放它不拥有的数据。共享内存由父进程显式 `UnmapViewOfFile` + `CloseHandle` 清理。

#### 总内存不变

| 分配方式 | 每相机 | ×10 |
|----------|--------|-----|
| 原方案（cv::Mat::zeros） | 220 × 5MB = 1.1GB | ~11GB |
| 新方案（共享内存） | 220 × 5MB = 1.1GB | ~11GB |
| **额外拷贝** | **0** | **0** |

```
父进程 (hdf5_multi_process.cpp):
  │
  ├─ 启动阶段: ram_buffer 分配在命名共享内存 "HDF5_<PID>_CAM_0" ~ "HDF5_<PID>_CAM_9"
  │
  ├─ 录制阶段: copyWorker → memcpy → ram_buffer（即共享内存） ← 完全不变
  │
  ├─ dump 阶段:
  │     ├─ 启动 10 个子进程 (CreateProcess):
  │     │     hdf5_multi_process_child.exe <camera_index> <hdf5_dir> <chunk_idx> <frame_offset> <core_frames> <cam_h> <cam_w> <margin_frames> <shm_name>
  │     │     （父进程生成完整 shm_name 并通过命令行参数传递）
  │     │
  │     ├─ WaitForMultipleObjects(10 个进程句柄, INFINITE)
  │     │
  │     └─ 检查退出码 → 全部成功 → CloseHandle(hJob) → g_frame_offset += core_frames → updateSentry

子进程 (hdf5_multi_process_child.exe):
  │
  ├─ 解析命令行参数（含 margin_frames 和 shm_name）
  ├─ OpenFileMapping(shm_name) → MapViewOfFile(margin_frames + N 帧) → core_data = base_data + margin 偏移
  ├─ 打开 HDF5 文件 <hdf5_dir>/<chunk_idx>.h5（文件已由父进程预创建）
  ├─ 逐帧 hyperslab 写入 raw_image（从 core_data 读取，跳过 margin）
  ├─ 关闭 HDF5 文件
  └─ exit(0) 或 exit(1)
```

### 3.2 进程间通信

| 方向 | 内容 | 方式 |
|------|------|------|
| 父→子 | 相机索引、目录路径、chunk_idx、frame_offset、尺寸、margin、共享内存名称 | 命令行参数（~300 字节） |
| 父→子 | ram_buffer 数据（~1GB） | 命名共享内存 `HDF5_<PID>_CAM_<index>`，**录制时已写入，零拷贝** |
| 子→父 | 成功/失败状态 | 进程退出码（0=成功, 1=HDF5错误, 2=共享内存错误） |

> **父进程不拷贝数据**：copyWorker 已经把数据写入了共享内存。dump 阶段直接启动子进程，数据已经在共享内存中等待读取。

### 3.3 父进程预创建 HDF5 文件（dump 阶段第一步）

子进程不应负责创建文件——跨 chunk 时新文件不存在，`H5F_ACC_RDWR` 会失败。父进程在启动子进程之前，**串行预创建**所有相机的 `.h5` 文件和三个 dataset 结构。

```cpp
// Step 0: Pre-create HDF5 files for all cameras (fast serial loop, ~0.3s total)
for (auto& ctx : cam_ctxs) {
    stringstream ss; ss << ctx->hdf5_dir << "/" << setw(4) << setfill('0') << g_chunk_idx << ".h5";
    bool exists = fs::exists(ss.str());
    if (!exists) {
        H5::H5File f(ss.str(), H5F_ACC_TRUNC);
        hsize_t rd[3] = {(hsize_t)g_hdf5_chunk_capacity, (hsize_t)cam_h, (hsize_t)cam_w};
        f.createDataSet("raw_image", H5::PredType::NATIVE_UINT8, H5::DataSpace(3, rd));
        hsize_t gd[2] = {(hsize_t)g_hdf5_chunk_capacity, 2};
        f.createDataSet("gaze_target", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, gd));
        hsize_t vd[1] = {(hsize_t)g_hdf5_chunk_capacity};
        f.createDataSet("valid", H5::PredType::NATIVE_UINT8, H5::DataSpace(1, vd));
    }
}
```

> 此步骤只在跨越 chunk 容量边界时（约每 10 次录制）才真正创建新文件，其余时候 `fs::exists` 检查通过后直接跳过。耗时 < 0.3s。

### 3.4 子进程实现（hdf5_multi_process_child.cpp）

独立的 `.cpp` 文件，编译为单独的可执行文件。子进程**假设 HDF5 文件和 dataset 已被父进程创建好**，直接打开并写入。

```cpp
// hdf5_multi_process_child.cpp — standalone child process for HDF5 writing
#include <windows.h>
#include <H5Cpp.h>
#include <iostream>
#include <sstream>
#include <iomanip>
using namespace std;

int main(int argc, char* argv[]) {
    // argv[1] = camera_index   argv[2] = hdf5_dir       argv[3] = chunk_idx
    // argv[4] = frame_offset   argv[5] = core_frames     argv[6] = cam_h
    // argv[7] = cam_w          argv[8] = margin_frames   argv[9] = shm_name
    int cam_idx = atoi(argv[1]);
    string hdf5_dir = argv[2];
    int chunk_idx = atoi(argv[3]);
    int frame_offset = atoi(argv[4]);
    int N = atoi(argv[5]);
    int cam_h = atoi(argv[6]);
    int cam_w = atoi(argv[7]);
    int margin_frames = atoi(argv[8]);
    string shm_name = argv[9];

    // Open shared memory — must map margin_frames + N to reach the core data
    size_t map_size = (size_t)(margin_frames + N) * cam_h * cam_w;
    HANDLE hMap = OpenFileMappingA(FILE_MAP_READ, FALSE, shm_name.c_str());
    if (!hMap) return 2;
    uint8_t* base_data = (uint8_t*)MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, map_size);
    if (!base_data) { CloseHandle(hMap); return 2; }
    // Skip margin (pre-trigger redundancy frames) to align with core_frames
    uint8_t* core_data = base_data + margin_frames * cam_h * cam_w;

    // Write HDF5 — file and datasets already created by parent.  Write ALL three datasets
    // so parent never needs to reopen this file (avoids cross-process file lock conflict).
    try {
        stringstream ss; ss << hdf5_dir << "/" << setw(4) << setfill('0') << chunk_idx << ".h5";
        H5::H5File f(ss.str(), H5F_ACC_RDWR);
        H5::DataSet raw_ds = f.openDataSet("raw_image");
        H5::DataSet gaze_ds = f.openDataSet("gaze_target");
        H5::DataSet valid_ds = f.openDataSet("valid");

        hsize_t f_start[3] = {0,0,0}, f_count[3] = {1,(hsize_t)cam_h,(hsize_t)cam_w};
        H5::DataSpace f_mem(3, f_count);
        for (int i = 0; i < N; ++i) {
            f_start[0] = (hsize_t)(frame_offset + i);
            H5::DataSpace f_file = raw_ds.getSpace();
            f_file.selectHyperslab(H5S_SELECT_SET, f_count, f_start);
            raw_ds.write(core_data + i * cam_h * cam_w, H5::PredType::NATIVE_UINT8, f_mem, f_file);
        }
        // gaze_target (all zeros) + valid (all ones) — tiny, ~0.001s
        hsize_t gz_start[2] = {(hsize_t)frame_offset, 0}, gz_count[2] = {(hsize_t)N, 2};
        H5::DataSpace gz_mem(2, gz_count), gz_file = gaze_ds.getSpace();
        gz_file.selectHyperslab(H5S_SELECT_SET, gz_count, gz_start);
        vector<double> gz_buf(N * 2, 0.0);
        gaze_ds.write(gz_buf.data(), H5::PredType::NATIVE_DOUBLE, gz_mem, gz_file);

        hsize_t v_start[1] = {(hsize_t)frame_offset}, v_count[1] = {(hsize_t)N};
        H5::DataSpace v_mem(1, v_count), v_file = valid_ds.getSpace();
        v_file.selectHyperslab(H5S_SELECT_SET, v_count, v_start);
        vector<uint8_t> v_buf(N, 1);
        valid_ds.write(v_buf.data(), H5::PredType::NATIVE_UINT8, v_mem, v_file);
    } catch (const H5::Exception& e) {
        cerr << "HDF5 error: " << e.getCDetailMsg() << endl;
        UnmapViewOfFile(base_data); CloseHandle(hMap);
        return 1;
    }

    UnmapViewOfFile(base_data);
    CloseHandle(hMap);
    return 0;
}
```

> 子进程写入全部三个 dataset（`raw_image`、`gaze_target`、`valid`）。**父进程绝不重新打开 HDF5 文件**——避免与子进程的文件锁冲突。`gaze_target` 全零和 `valid` 全 1 的数据量极小（共 ~34KB），写入耗时 < 1ms，不影响并发性能。

### 3.5 父进程 dump 阶段完整流程

```cpp
// Step 0: Pre-create HDF5 files if needed (see 3.3)

// Step 1: Launch child processes
// Use absolute path for lpApplicationName to handle spaces in paths
char exe_path[MAX_PATH];
GetModuleFileNameA(NULL, exe_path, MAX_PATH);
string parent_dir = fs::path(exe_path).parent_path().string();
string child_exe = parent_dir + "\\hdf5_multi_process_child.exe";

HANDLE hJob = CreateJobObjectA(NULL, NULL);  // Job Object for auto-cleanup
JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli{};
jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
SetInformationJobObject(hJob, JobObjectExtendedLimitInformation, &jeli, sizeof(jeli));

vector<PROCESS_INFORMATION> procs(cam_ctxs.size());
for (size_t i = 0; i < cam_ctxs.size(); ++i) {
    auto& ctx = cam_ctxs[i];
    string shm_name = "HDF5_" + to_string(GetCurrentProcessId()) + "_CAM_" + to_string(i);
    stringstream args;
    // Fake argv[0] — Windows C runtime uses the first space-delimited token
    // as argv[0]; without this, "0" becomes argv[0] and all args shift by one.
    args << "\"hdf5_multi_process_child.exe\" "
         << i << " \"" << ctx->hdf5_dir << "\" "
         << g_chunk_idx << " " << g_frame_offset << " " << core_frames << " "
         << cam_h << " " << cam_w << " " << margin_frames << " " << shm_name;
    STARTUPINFOA si{sizeof(si)}; PROCESS_INFORMATION pi{};
    // Store in a local string — CreateProcessA may modify lpCommandLine in-place
    // (inserts \0 during parsing). args.str().c_str() returns a const pointer
    // to a temporary; casting it to LPSTR would cause an access violation.
    string cmd_line = args.str();
    CreateProcessA(child_exe.c_str(), &cmd_line[0],
                   NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi);
    CloseHandle(pi.hThread);
    AssignProcessToJobObject(hJob, pi.hProcess);  // kill on parent crash
    procs[i] = pi;
}
// NOTE: Do NOT close hJob here — JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE would
// immediately terminate all children. Close it after all children have exited (Step 3).

// Step 2: Wait for all children (child writes raw_image + gaze + valid — no parent reopening)
vector<HANDLE> handles;
for (auto& p : procs) handles.push_back(p.hProcess);
WaitForMultipleObjects((DWORD)handles.size(), handles.data(), TRUE, INFINITE);

// Step 3: Check exit codes
bool all_ok = true;
for (auto& p : procs) {
    DWORD ec; GetExitCodeProcess(p.hProcess, &ec);
    if (ec != 0) { all_ok = false; logException("ERROR", "hdf5:cam", "child failed, exit=" + to_string(ec)); }
    CloseHandle(p.hProcess);
}

CloseHandle(hJob);  // All children have exited — safe to release Job Object now

// Step 4: Update sentry (no HDF5 file access needed — children did everything)
if (all_ok) {
    g_frame_offset += core_frames;
    if (g_frame_offset >= g_hdf5_chunk_capacity) { g_chunk_idx++; g_frame_offset -= g_hdf5_chunk_capacity; }
    updateSentry(g_sentry_root);
}
```

---

## 四、开发注意事项

### 4.1 内存用量

ram_buffer 直接分配在共享内存上——`cv::Mat(rows, cols, type, data)` 包装共享内存地址，copyWorker 的 `memcpy` 目标就是共享内存。子进程打开同一块共享内存读取。**全程零额外拷贝**。

| 内存区域 | 每相机 | ×10 |
|----------|--------|-----|
| ram_buffer（在共享内存中） | 1.1GB | ~11GB |
| **总计** | | **~11GB** |

系统可用 40GB，绰绰有余。若 `cv::Mat` 包装共享内存出现兼容性问题，备选方案为 dump 时拷贝（+10GB，总计 21GB），仍在 40GB 范围内。

### 4.2 HDF5 多进程文件锁

HDF5 不支持多进程同时打开同一 `.h5` 文件。子进程写入全部三个 dataset（`raw_image` + `gaze_target` + `valid`），父进程**绝不重新打开 HDF5 文件**。每个 `.h5` 文件只被一个进程（子进程）访问，彻底消除文件锁冲突。

### 4.3 CreateProcessA 路径安全

`CreateProcessA` 的第二个参数 `lpCommandLine` 遇到路径中有空格时会解析失败。必须使用第一个参数 `lpApplicationName` 传递绝对路径，参数单独放入 `lpCommandLine`。同时 `lpCommandLine` 必须指向**可修改的缓冲区**（Windows 解析时会原地插入 `\0`），不能用临时对象的 `.c_str()`：

```cpp
// ✅ 正确：路径和参数分离 + 可修改缓冲区
string cmd_line = args.str();                               // 拥有独立内存
CreateProcessA(child_exe.c_str(), &cmd_line[0], ...);       // 可修改，不会 UB

// ❌ 错误1：路径混入 lpCommandLine，空格导致截断
CreateProcessA(NULL, (LPSTR)"C:\\New Folder\\child.exe arg1 arg2", ...);

// ❌ 错误2：(LPSTR)强转临时对象的 c_str()，修改只读内存 → Access Violation
CreateProcessA(child_exe.c_str(), (LPSTR)args.str().c_str(), ...);
```

通过 `GetModuleFileNameA(NULL, ...)` 获取父进程所在目录，拼接子进程路径。

### 4.4 错误处理

- 子进程退出码非 0 → 父进程记录异常，该相机数据可能丢失
- 子进程崩溃（`WaitForMultipleObjects` 返回 `WAIT_FAILED`）→ 父进程重试或跳过
- 共享内存创建失败 → 回退到串行模式

### 4.5 共享内存清理：Job Object 防僵尸

共享内存名称加入 PID 前缀（`HDF5_<PID>_CAM_0`）防止多实例冲突。但若父进程异常崩溃（调试期间常见），`UnmapViewOfFile` 和 `CloseHandle` 未执行，Windows 不会回收引用计数未归零的共享内存。

**隐患**：10 台相机 ≈ 11GB 共享内存。扩展到 20 台相机时 ≈ 22GB。一次崩溃残留即可耗尽 40GB 系统内存。

**解决方案**：使用 Windows **Job Object**。

```cpp
// 父进程启动时创建 Job Object
HANDLE hJob = CreateJobObjectA(NULL, NULL);
JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli{};
jeli.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
SetInformationJobObject(hJob, JobObjectExtendedLimitInformation, &jeli, sizeof(jeli));

// 每个子进程启动后立即分配进 Job
AssignProcessToJobObject(hJob, pi.hProcess);
```

`JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE` 的作用：父进程退出时（无论正常还是崩溃），Windows 内核自动终止 Job 内所有子进程，并清理它们持有的全部内核句柄（包括共享内存）。**彻底消除僵尸共享内存**。

### 4.6 共享内存命名

父进程生成完整名称（含 PID）并通过命令行参数传递给子进程，子进程直接使用接收到的名称——**绝不自行调用 `GetCurrentProcessId()`**（否则拿到的是子进程自己的 PID，无法打开父进程创建的共享内存）。命名格式：`"HDF5_" + to_string(GetCurrentProcessId()) + "_CAM_" + to_string(i)`（在父进程中调用）

### 4.7 跨 chunk 处理

与当前逻辑相同——父进程在启动子进程前检查 `g_frame_offset + core_frames > capacity`，若溢出则先更新 `g_chunk_idx`，子进程使用新的 chunk_idx。

### 4.8 HDF5 版本兼容

子进程使用与父进程相同的 HDF5 库版本（2.1.1 via vcpkg），确保文件格式一致。

---

## 五、预期性能

| 阶段 | 串行 | 多进程 | 加速比 |
|------|------|--------|--------|
| 预创建 HDF5 文件（仅跨 chunk 时） | — | ~0.3s | — |
| raw_image 写入 (10 cameras) | ~6.5s | ~0.7s | **9.3x** |
| 进程启动开销 | — | ~0.2s | — |
| sentry 更新 | ~0.01s | ~0.01s | 1x |
| **合计**（已有文件） | **~7.5s** | **~0.9s** | **~8.3x** |
| **合计**（跨 chunk，需创建新文件） | **~7.5s** | **~1.2s** | **~6.3x** |

> 共享内存无需拷贝——ram_buffer 从分配之初就在共享内存中，子进程零拷贝读取。

> 实际加速比取决于磁盘 I/O 带宽。若 10 台相机写入同一物理磁盘，磁盘带宽可能成为瓶颈（理论最大 ~500MB/s for HDD, ~3GB/s for NVMe SSD）。

---

## 六、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 创建 `hdf5_multi_process.cpp`（复制 `test_multi_cam_multi_host_hdf5.cpp`）+ `hdf5_multi_process_child.cpp` + CMakeLists | 15min |
| 2 | ram_buffer 分配改为共享内存（`cv::Mat` 包装 `MapViewOfFile` 地址） | 10min |
| 3 | 父进程 dump 阶段：预创建 HDF5 文件 + `CreateProcess`（含 Job Object）+ `WaitForMultipleObjects` | 15min |
| 4 | 父进程添加退出码检测 + 错误处理 | 5min |
| 5 | 子进程实现：从共享内存读取 → 写入 HDF5 全部三个 dataset | 10min |
| 6 | 编译调试（父进程 + 子进程） | 20min |
| 7 | 运行时验证 | 15min |

**总计：~1.5 小时**（与之前相同，步骤调整但工时不变）
