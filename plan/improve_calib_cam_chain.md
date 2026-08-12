# calib_cam_chain.cpp — 配置迁移 + 分段计时

> 当前瓶颈：特征提取阶段 (`FindCalibObject`) 对每张图每个相机串行调用，N×M 次 HALCON 调用，无计时数据。
> 目标：迁移配置到 `cam_calib.yaml` + participant_id 路径构建，分段计时为后续优化提供数据。

---

## 一、配置迁移：`default.yaml` → `cam_calib.yaml`

### 1.1 当前 `default.yaml` 的 `cam_calib` 节点

```yaml
cam_calib:
  input_folder: "D:/calib_images"                              # → 废弃，用 calib_save_dir/{pid}/pictures
  calib_plane: "C:/hitsz/apps/MVTec/multiple_cameras/calib/HG-180.cpd"
  output_folder: "D:/calib_output"                             # → 废弃，用 calib_save_dir/{pid}/output
  focus: 0.016           # m
  pixel_size_x: 2.74e-6  # m
  pixel_size_y: 2.74e-6  # m
  center_cam: "40772280" # 可选
  focus_overrides:       # 可选
    40772283: 0.075
    40772276: 0.075
```

### 1.2 新增到 `cam_calib.yaml` 的 `calib` 节点

新增以下键（与现有 `cam_indices`, `calib_save_dir` 同级）：

```yaml
calib:
  # === 已有（不变）===
  cam_indices: [...]
  calib_save_dir: "D:/calib_images"
  # ... (is_master, port, fps, etc.)

  # === 新增：标定算法参数 ===
  calib_plane: "C:/hitsz/apps/MVTec/multiple_cameras/calib/HG-180.cpd"
  focus: 0.016                       # m
  pixel_size_x: 2.74e-6              # m
  pixel_size_y: 2.74e-6              # m
  center_cam: "40772283"             # 中心相机 SN（可选），空则默认 cam 0 为参考
  focus_overrides:                   # 特定相机焦距覆盖 (m), SN → focus（可选）
    40772283: 0.075
    40772276: 0.075
```

### 1.3 路径计算规则（取代硬编码 key）

`calib_cam_chain.cpp` 不再从 YAML 读 `input_folder` / `output_folder`，而是自动计算：

```cpp
string base_dir  = calib["calib_save_dir"].as<string>();         // "D:/calib_images"
string pid       = capture["capture"]["participant_id"].as<string>();  // "P001"
string input_dir = base_dir + "/" + pid + "/pictures";           // 图片输入
string output_dir = base_dir + "/" + pid + "/output";            // XML 输出
fs::create_directories(output_dir);
```

**设计理由**：
- 图片采集已经存储到 `calib_save_dir/{pid}/pictures/`，标定链自然从这里读
- 输出 XML 放在 `calib_save_dir/{pid}/output/`，与图片同级，便于管理
- 删除 `input_folder` 和 `output_folder` 两个冗余配置键

### 1.4 代码变更：`action()` 中的 Cfg 读取

```cpp
// 旧：
Cfg cfg;  // reads default.yaml
HTuple hv_ImagePath = cfg["cam_calib"]["input_folder"].as<string>().c_str();
HTuple hv_OutputBaseDir = cfg["cam_calib"]["output_folder"].as<string>().c_str();
double focus = cfg["cam_calib"]["focus"].as<double>();

// 新：
Cfg cfg("cfg/cam_calib.yaml"); auto& calib = cfg["calib"];
Cfg cap("cfg/capture.yaml");
string pid = cap["capture"]["participant_id"].as<string>();
string base_dir = calib["calib_save_dir"].as<string>();
string input_dir = base_dir + "/" + pid + "/pictures";
string output_dir = base_dir + "/" + pid + "/output";
fs::create_directories(output_dir);

HTuple hv_ImagePath = input_dir.c_str();
HTuple hv_OutputBaseDir = output_dir.c_str();
HTuple hv_CalibObjDescr = calib["calib_plane"].as<string>().c_str();
double focus = calib["focus"].as<double>();
double pixel_size_x = calib["pixel_size_x"].as<double>();
double pixel_size_y = calib["pixel_size_y"].as<double>();
string center_cam_sn;
try { center_cam_sn = calib["center_cam"].as<string>(); } catch(...) { center_cam_sn = ""; }
```

---

## 二、分段计时

### 2.1 计时宏/辅助

在 `action()` 顶部定义计时工具：

```cpp
#include <chrono>
using namespace std::chrono;

struct StageTimer {
    steady_clock::time_point t0;
    const char* name;
    StageTimer(const char* n) : name(n), t0(steady_clock::now()) {}
    ~StageTimer() {
        double dt = duration<double>(steady_clock::now() - t0).count();
        cout << "[Timer] " << name << ": " << fixed << setprecision(2) << dt << "s" << endl;
    }
};
```

或在每个阶段直接内联：

```cpp
auto t0 = steady_clock::now();
// ... stage work ...
double dt = duration<double>(steady_clock::now() - t0).count();
cout << "[Timer] Stage N: " << dt << "s" << endl;
```

### 2.2 计时粒度

| 阶段 | 计时点 | 关键指标 |
|------|--------|---------|
| **0. Scan** | `scan_calib_image_folder()` | 目录扫描耗时 |
| **1. Init** | 所有相机初始化（读首图 + 设置参数） | 单相机平均耗时 |
| **2. Feature** | 全部 `FindCalibObject` 调用（N×M 次） | **核心瓶颈**，同时统计成功/失败次数 |
| **3. Graph** | BFS 连通性检查 | 通常 <1ms |
| **4. Calibrate** | `CalibrateCameras()` | HALCON 求解耗时 |
| **5. Rebase** | 位姿变换到中心相机 | 通常 <10ms |
| **6. Export** | 写出 N 个 XML 文件 | 文件 IO 耗时 |
| **Total** | 整体 wall-clock | — |

### 2.3 Stage 2 详细计时

特征提取是最耗时的阶段，需要子粒度数据：

```
[Timer] Stage 2 - Feature Extraction (num_images=20, num_cameras=10)
  Total:   X s
  Per img: Y ms avg, Z ms max
  Success: A / 200 (B%)
  Failed:  C images
```

### 2.4 输出示例

```
=== Multi-Camera Calibration Chain ===

[Timer] Stage 0 - Scan: 0.02s
--- Camera Init ---
[Timer] Stage 1 - Init: 1.53s (10 cameras, 0.15s avg each)
--- Feature Extraction ---
[Timer] Stage 2 - Feature: 142.30s (200 calls, 0.71s avg, 185 success, 15 failed)
--- Graph Validation ---
[Timer] Stage 3 - Graph: 0.00s
--- Calibration ---
[Timer] Stage 4 - Calibrate: 3.21s
--- Rebase ---
[Timer] Stage 5 - Rebase: 0.01s
--- Export ---
[Timer] Stage 6 - Export: 0.15s (10 XML files)
========================================
[Timer] Total: 147.22s
```

---

## 三、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 在 `cam_calib.yaml` 添加 `calib_plane`, `focus`, `pixel_size_x/y`, `center_cam`, `focus_overrides` | 3min |
| 2 | 修改 `calib_cam_chain.cpp` — 从 `cam_calib.yaml` + `capture.yaml` 读取，计算路径 | 10min |
| 3 | 添加分段计时（7 个阶段 + Stage 2 子统计） | 15min |
| 4 | 编译 + 实机测试 | 5min |
| 5 | 分析计时数据，提出优化方案 | — |

**总计：~30 分钟**

---

## 四、后续优化方向（基于计时数据）

分段计时完成后，根据 Stage 2 占总时间的比例决定优化策略：

| Stage 2 占比 | 策略 |
|-------------|------|
| > 90% | 重点优化 `FindCalibObject`：减少搜索空间、调整 alpha/sigma 参数、跳过已知失败帧 |
| 50-90% | 并行化 Stage 2（OpenMP 或线程池，每个相机一个线程） |
| < 50% | 检查其他阶段的瓶颈（Init 的 ReadImage 每次读完整图可优化） |
