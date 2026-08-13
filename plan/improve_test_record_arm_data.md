# test_record_arm_data.cpp — 配置迁移 + UI 对齐

> 目标：
> 1. 配置从 `piper.yaml` 的 `test_record_arm_data` 节点整体搬迁到新的 `calib_arm.yaml`
> 2. 图片保存目录改为 `calib_arm.yaml: calib_save_dir / capture.yaml: participant_id`
> 3. UI 与 `test_calib_images.cpp` 对齐（十字线、左下角说明、左上角 SN+模式、右上角数量）

---

## 一、配置迁移：新建 `cfg/calib_arm.yaml`

### 1.1 新文件结构

`test_record_arm_data.cpp` 不再读 `piper.yaml` 的 `test_record_arm_data` 节点，改为读 `calib_arm.yaml`：

```yaml
# calib_arm.yaml — 机械臂标定数据采集 (test_record_arm_data.cpp)

network:                              # 从 piper.yaml network 节点迁移
  ubuntu_ip: "192.168.10.4"
  port: 49300                         # windows_end_monitor.py (flange pose query)

record:                               # 从 piper.yaml test_record_arm_data 节点迁移
  cam_indices:
    - 40772278
    - 40777498
    - 40772280
    - 40772279
    - 40777497
    - 40772277
    - 40772284
  calib_save_dir: "D:/calib_arm"      # 总目录, 完整路径 = calib_save_dir/{participant_id}
                                      # participant_id 从 capture.yaml 读取
  fps: 100
  gain: 0.0
  gamma: 1.0
  exposure_time: 8000.0
  window_width: 1600
  window_height: 800
  ui_fps: 20.0
  cam_width: 2448
  cam_height: 2048
```

### 1.2 代码变更

```cpp
// 旧：
auto piper_path = ... / "cfg" / "piper.yaml";
Cfg piper_cfg(piper_path);
g_ubuntu_ip = piper_cfg["network"]["ubuntu_ip"].as<string>();
g_arm_port  = piper_cfg["network"]["port"].as<int>();
auto& rcfg = piper_cfg["test_record_arm_data"];

// 新：
fs::path cfg_dir = fs::path(__FILE__).parent_path()....parent_path() / "cfg";
Cfg arm_cfg((cfg_dir / "calib_arm.yaml").string());
Cfg cap_cfg((cfg_dir / "capture.yaml").string());

g_ubuntu_ip = arm_cfg["network"]["ubuntu_ip"].as<string>();
g_arm_port  = arm_cfg["network"]["port"].as<int>();
auto& rcfg = arm_cfg["record"];

// 保存目录 = calib_save_dir / participant_id
string pid = cap_cfg["capture"]["participant_id"].as<string>();
g_calib_save_dir = rcfg["calib_save_dir"].as<string>() + "/" + pid;
```

### 1.3 目录结构（保存目录变更的影响）

之前：`D:/calib_arm/<arm>/<SN>/calib_XX.jpg` 和 `D:/calib_arm/flange_pose_mapping_<arm>.txt`
之后：`D:/calib_arm/P001/<arm>/<SN>/calib_XX.jpg` 和 `D:/calib_arm/P001/flange_pose_mapping_<arm>.txt`

所有引用 `g_calib_save_dir` 的路径自动跟随，无需额外改动。

---

## 二、UI 对齐 test_calib_images.cpp

### 2.1 目标布局

```
┌──────────────────────┬─────────────────────────────┐
│  thumbnail grid      │  SN  [MODE: xxx]    Count: N│ ← 左上 SN+模式, 右上数量
│  (2 cols × 5 rows)   │                             │
│                      │            +               │ ← 中心十字线
│                      │                       UPPER │ ← 右下臂指示
└──────────────────────┴─────────────────────────────┘
                       [操作说明]  (左下角)
```

### 2.2 各元素规格（与 test_calib_images.cpp 一致）

| 元素 | 位置 | 样式 | 参照 |
|------|------|------|------|
| **中心十字线** | enlarged 区域正中心 `(g_right_x+g_right_w/2, g_win_h/2)` | 2 条 20px 线，`Scalar(100,100,100)`, lw=1, LINE_AA | test_calib_images.cpp L1052-1055 |
| **左下角操作说明** | `(g_right_x+10, g_win_h-45)` | `FONT_HERSHEY_SIMPLEX, 0.4, Scalar(140,140,140), 1, LINE_AA` | test_calib_images.cpp L1029-1036 |
| **第二行状态** | `(g_right_x+10, g_win_h-27)` | `FONT_HERSHEY_SIMPLEX, 0.35, Scalar(110,110,110), 1` | test_calib_images.cpp L1037-1043 |
| **右上角数量** | `(g_right_x+g_right_w-宽-15, 35)` | `FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,215,255), 2, LINE_AA` | test_calib_images.cpp L1045-1050 |
| **左右分栏线** | `x=g_left_w` 竖线 | `Scalar(60,60,60), 2` | test_calib_images.cpp L1025 |
| **左上角 SN** | `(g_right_x+10, 35)` | `FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,215,255), 2, LINE_AA` | 新增（原来 SN 显示在中间） |
| **左上角模式** | `(g_right_x+10, 60)` | `FONT_HERSHEY_SIMPLEX, 0.5`，不同模式不同颜色 | 新增 |
| **右下角臂指示** | 右对齐于 `(g_right_x+g_right_w-15, g_win_h-15)` | `FONT_HERSHEY_SIMPLEX, 1.0`，UPPER=金 `(0,215,255)` / LOWER=紫 `(200,80,255)`，lw=2 | 新增（原来在顶部居中） |

### 2.3 模式显示

```
MODE: ARM CALIB   (机械臂标定, 颜色 Scalar(200,80,255) 紫 — 与 LOWER 臂一致)
MODE: INTRINSIC   (单相机内参标定, 颜色 Scalar(0,255,0) 绿)
```

- 机械臂标定模式（当前 `!g_calib_mode` 时的拍照逻辑，包含 flange pose 记录）
- 内参标定模式（`g_calib_mode == true`）

### 2.4 UI 元素调整

- 当前顶部居中的 `UPPER`/`LOWER` 大字臂指示 → **移到右下角**（右对齐，保留原配色：UPPER 金 / LOWER 紫）
- 当前 `calib mode banner` 中置显示 → 改为左上角模式行

### 2.5 左下角操作说明文案

```
非标定模式: "[t] switch  [space] capture  [z] undo  [a] calib  [c] clear  [q] quit"
标定模式:   "[a] exit  [space] capture  [z] undo  Camera: <SN>"
第二行:      "Arm: UPPER  |  Saving to: <当前保存目标>"
```

---

## 三、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 新建 `cfg/calib_arm.yaml`（network + record 两个节点） | 5min |
| 2 | 修改 `test_record_arm_data.cpp` 配置读取 + 保存目录拼接 participant_id | 10min |
| 3 | 重写 UI 渲染段：十字线、左下角提示、左上角 SN+模式、右上角数量 | 15min |
| 4 | 移除旧的中置 arm 指示和 calib banner | 2min |
| 5 | 编译验证 | 2min |

**总计：~35 分钟**
