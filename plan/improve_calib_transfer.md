# test_calib_images.cpp — 传输速度优化

> 当前瓶颈：UDP 逐文件串行传输，每次标定采集后需 ~5 分钟传输 Slave 图片到 Master
> 目标：通过 TCP 多文件并行传输，将时间压缩到 1 分钟以内

---

## 一、新配置文件 `cfg/cam_calib.yaml`

从 `default.yaml` 的 `test_multi_cam` 节点迁移配置，新增传输相关键。

```yaml
# cam_calib.yaml — 标定图像采集 (test_calib_images.cpp)

calib:
  # === 从 default.yaml test_multi_cam 迁移 ===
  is_master: false
  master_ip: "192.168.10.1"
  slave_ip: "192.168.10.2"
  port: 49200
  enable_net_sync: true

  cam_indices:
    - "40768742"
    - "40774056"
    # ... 全部 SN

  calib_save_dir: "D:/calib_arm"     # 标定图片总目录 (完整路径 = calib_save_dir/{participant_id})
  participant_id: "P001"             # 从 capture.yaml 读取

  fps: 100
  gain: 0.0
  gamma: 1.0
  exposure_time: 8000.0
  calib_mono_exp_ext: 1.0
  window_width: 1600
  window_height: 800
  ui_fps: 20.0

  # === 新增：传输控制 ===
  test_transfer: false               # true = 跳过主循环直接测试传输
  test_transfer_recv_dir: "D:/calib_transfer_test"  # master 测试接收目录
```

**设计要点**：
- `test_transfer: true` 时，直接跳过主循环和 capture UI
- Master 和 Slave 直接进入传输流程
- Slave 扫描本地文件，Master 接收后写入 `test_transfer_recv_dir`
- 传输完成后打印统计信息并退出（避免覆盖真实接收目录）

---

## 二、传输协议改进：UDP → TCP 并行

### 2.1 实测数据

| 指标 | 值 |
|------|-----|
| 文件数 | 1850 张 JPG |
| 总大小 | 1054 MB |
| 耗时 | 260 秒 |
| 速度 | **4 MB/s** |
| 目标 | **< 60 秒** |

### 2.2 瓶颈分析

每张图 ~570KB，当前 UDP 传输每文件分 ~10 个 chunk（60000 字节/chunk），每个 chunk 之间有 `sleep_for(milliseconds(1))`：

```
1850 文件 × 10 chunk × 1ms = 18.5 秒（仅 sleep 损耗）
```

加上 UDP 串行请求-响应模式（`GET:sn_idx` → 等 DONE → 下一个文件）、无丢包重传、单线程顺序执行，实际速度仅 4 MB/s。1GbE 网络的理论带宽是 125 MB/s——利用率仅 3%。

### 2.3 TCP 方案

**核心思路**：彻底去掉 UDP。一条 TCP 连接承载文件传输 + 所有控制消息（PHOTO/UNDO/CLEAR/FAULT/SHUTDOWN/LIST/XFER）。

```
TCP 端口: net_port + 500 = 49700 (TCP 承载所有通信：控制消息 + 文件传输)

Master                                Slave
  │                                     │
  │─ TCP connect :49700 ─────────────→│─ TCP listen :49700
  │                                     │
  │─ LIST_REQ\n ──────────────────────→│─ 扫描本地 → LIST_RESP:sn1_i1,...\n
  │                                     │
  │─ XFER:sn1_i1,sn2_i2,...\n ───────→│─ 逐个读取JPEG, 连续发送
  │                                     │
  │←── FILE:sn:idx:size\n ────────────│   (文本头, size = JPEG字节数)
  │←── [size bytes JPEG data] ────────│   (紧接头之后, 二进制)
  │←── FILE:sn:idx:size\n ────────────│   下一个文件...
  │←── [JPEG data] ───────────────────│
  │←── XFER_DONE\n ───────────────────│   全部完成
  │                                     │
  │─ 统计 + 退出                         │─ 退出
```

**消息格式**：

```
# 文本控制 (换行分隔)
LIST_REQ                            Master→Slave
LIST_RESP:sn1_i1,sn2_i2,...         Slave→Master   (逗号分隔)
XFER:sn1_i1,sn2_i2,...              Master→Slave   (逗号分隔)
XFER_DONE                           Slave→Master

# 文件传输 (文本头 + 二进制体)
FILE:sn:idx:size                     Slave→Master   (如 FILE:40768742:01:580123)
<580123 bytes raw JPEG>             (紧接头之后, 精确 size 字节)
```

### 2.4 预期速度

| 环节 | 耗时估算 |
|------|---------|
| TCP 纯传输 1054MB @ 50MB/s | **~21 秒** |
| Slave 磁盘读取 1850 文件 | ~10 秒 (SSD) / ~30 秒 (HDD) |
| Master 磁盘写入 1850 文件 | ~10 秒 (SSD) / ~30 秒 (HDD) |
| **预期总计 (SSD)** | **~30-40 秒 ✓** |

1GbE 下 TCP 有效吞吐通常 50-80 MB/s。SSD 顺序读写 200+ MB/s，1850 个小文件随机读取会有开销，但 10 秒足够。即使 HDD，60 秒也足够。

### 2.5 test_transfer 模式

`test_transfer: true` 时，**完全跳过相机初始化**（不创建 Pylon、不启动相机线程）：

```
Master:
  1. 加载 cam_calib.yaml + capture.yaml（读 participant_id）
  2. TCP listen :49700, accept Slave
  3. 发送 LIST_REQ → 接收 LIST_RESP
  4. 发送 XFER:全部文件 → 接收 FILE + JPEG → 写入 test_transfer_recv_dir/{participant_id}
  5. 收到 XFER_DONE → 统计 → 退出

Slave:
  1. 加载 cam_calib.yaml + capture.yaml（读 participant_id）
  2. 实际路径 = calib_save_dir/{participant_id}
  3. TCP connect Master :49700
  4. 收到 LIST_REQ → 扫描本地目录 → 发送 LIST_RESP
  5. 收到 XFER → 连续发送全部文件
  6. 发送 XFER_DONE → 退出
```

---

## 三、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 新建 `cfg/cam_calib.yaml` | 5min |
| 2 | 复制 `test_calib_images.cpp` → `test_calib_transfer.cpp` | 1min |
| 3 | 实现 TCP send/recv 辅助函数（换行 + 二进制） | 10min |
| 4 | 实现 TCP 文件传输（Master 端接收循环 + Slave 端发送循环） | 15min |
| 5 | 实现 `test_transfer` 模式（跳过主循环） | 10min |
| 6 | 修改配置读取：从 `cam_calib.yaml` 而非 `default.yaml` | 5min |
| 7 | CMakeLists 添加 target | 2min |
| 8 | 编译 + 实机测速 | 10min |

**总计：~1 小时**

---

## 四、待确认

1. **TCP 是否足够？** 单 TCP 流的理论带宽在 1GbE 下约 100MB/s。10 台相机 × 12MB = 120MB，纯传输约 1.2 秒。加上磁盘 IO，预期 10-20 秒。如果不够可以后续加多线程。

2. **UDP 控制通道保留**：PHOTO/UNDO/CLEAR/FAULT/SHUTDOWN 仍用 UDP（延迟优先，数据量极小）。仅文件传输改用 TCP。

3. **test_transfer 模式是否需要相机初始化？** 不需要——跳过 Pylon 初始化和相机线程，直接测试纯传输速度。
