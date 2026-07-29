# test_calib_images.cpp 改进计划

> 目标文件：`cpp_eyetracker/tests/utils/cam_calib/test_calib_images.cpp`（1085 行）
> 最后更新：2026-07-29

---

## 一、现状分析

### 1.1 功能概述

| 功能 | 说明 |
|------|------|
| 相机采集 | 软件触发，所有相机同时推流，`copyWorker` 维护 `latest_frame` |
| 标定拍照 | 按 SPACE 捕获当前帧保存为 `calib_cam_<SN>_<XX>.jpg` |
| 网络同步 | Master/Slave 通过 UDP 同步拍照（PHOTO）、撤销（UNDO）、清除（CLEAR） |
| 文件传输 | 't' 键触发 `transferMissingImages()`：UDP chunk 协议，Master 向 Slave 拉取缺失的标定图片 |
| UI | 5×2 缩略图 + 放大视图，鼠标点击切换 |
| 健康监控 | 1s 无帧 → 故障 → 关所有相机 → 发 FAULT → 故障 UI → 等 ESC |
| 撤销/清除 | 'z' 撤销上一次拍照（删除本地 + 通知 Slave），'c' 清除所有本地标定图片 |

### 1.2 架构

```
main()
 ├─ UI 线程（20fps）：thumbnail grid + enlarged view
 ├─ captureWorker（per cam）：Pylon 回调 → copy_queue
 ├─ copyWorker（per cam）：copy_queue → latest_frame
 ├─ udpListenerWorker（Slave）：监听 PHOTO/UNDO/CLEAR/LIST_REQ
 └─ transferMissingImages（同步阻塞）：UDP chunk 协议传输
```

### 1.3 与 test_multi_cam_multi_host 的差异

| | test_multi_cam_multi_host | test_calib_images |
|---|---|---|
| 触发模式 | HW + SW | 仅 SW |
| 录制/采集 | 连续录制到 ram_buffer | 单帧捕获 |
| 网络协议 | CMD_START + FAULT + SHUTDOWN | PHOTO + UNDO + CLEAR + LIST_REQ/LIST_RESP + chunk 传输 |
| 指标 | 完整 metrics + session log | **无** |
| UI 提示 | 水印 + 十字线 | **无** |
| 异常处理 | logException + 计数器 | 仅 cerr/cout |

---

## 二、现有异常处理

### 2.1 按阶段

#### 阶段 1：初始化
| 异常 | 级别 | 处理 |
|------|------|------|
| WSAStartup 失败 | FATAL | `return 1` |
| 相机 open/start 失败 | ERROR | `status=ERROR_`，线程返回 |
| YAML 配置缺失 | WARN | try-catch 用默认值 |
| `save_dirs.size()` 不匹配 | FATAL | `return -1` |

#### 阶段 2：待机
| 异常 | 级别 | 处理 |
|------|------|------|
| 相机停流 1s | ERROR | 故障 → 关相机 → 发 FAULT → 故障 UI |
| 对端 FAULT | ERROR | 关相机 → 故障 UI |
| TCP connect 失败 | WARN | while 重试 |
| 帧为空 | INFO | 黑底+状态文字 |

#### 阶段 3：拍照
| 异常 | 级别 | 处理 |
|------|------|------|
| `cv::imwrite` 失败 | ERROR | **无检查** |
| Slave 断连（UDP 传输超时） | WARN | `recvSingleFile` 返回空 |

---

## 三、需要新增的功能

### 3.1 会话日志

仿照 `test_multi_cam_multi_host`，程序启动时创建 `log/capture/calib_session_<ts>.md`。

**日志内容**：
- Session 头（相机数、触发模式）
- 每次 SPACE 拍照追加一行（时间戳、index、各相机 SN）
- 'z' 撤销追加一行
- 'c' 清除追加一行
- 't' 传输追加统计行
- 故障事件追加异常记录

### 3.2 指标采集

| 指标 | 说明 | 采集时机 |
|------|------|----------|
| 拍照次数 | 累计 SPACE 次数 | 每次 SPACE +1 |
| 撤销次数 | 累计 'z' 次数 | 每次 'z' +1 |
| 传输文件数 | 't' 传输的文件数量 | transferMissingImages 结束后 |
| 传输字节数 | 't' 传输的总字节数 | 同上 |
| **传输速度** | 传输字节数 / 传输耗时 (MB/s) | 同上 |
| **传输耗时** | `transferMissingImages()` 墙上时间 | 入口出口 chrono 差值 |
| 每相机拍照数 | 各相机被 SPACE 捕获次数 | 按 latest_frame 是否为空统计 |
| 运行时长 | 从 g_ready_time 到退出 | 退出时记录 |

### 3.3 UI 增强

- **操作提示水印**（右下角）：`[SPACE] capture  [z] undo  [c] clear  [t] transfer  [ESC/q] quit`
- **状态行**：`Role: MASTER/SLAVE  |  Net Sync: ON/OFF  |  Captures: N`
- **中心十字线**（放大区域）
- **网络同步按键控制**：
  - Slave 按键无效（SPACE/'z'/'c'/'t' 全部屏蔽）
  - Master 按键有效，SPACE/'z'/'c'/'t' 通过 UDP 通知 Slave 同步执行
  - Master 按 ESC/q → 发 SHUTDOWN 给 Slave，等 200ms，两台主机同时退出
  - Slave 按 ESC/q → 忽略（仅 Master 可退出）
  - 故障状态下 ESC 仍可在 Slave 上使用

### 3.4 异常处理增强

| # | 阶段 | 异常 | 级别 | 当前 | 改进 |
|---|------|------|------|------|------|
| C1 | 1 | log/capture/ 创建失败 | ERROR | 无检查 | error_code + logException |
| C2 | 2 | UDP chunk 数据损坏 | WARN | 静默跳过 | logException |
| C3 | 3 | cv::imwrite 失败 | ERROR | 无检查 | 检查返回值 |
| C4 | 3 | Slave PHOTO 越界 | WARN | 无检查 | cmd.length() 检查 |
| C5 | 3 | UNDO/CLEAR 消息格式异常 | WARN | 无检查 | 格式校验 |
| C6 | 3 | captureWorker open/start 失败后 copyWorker 空等 | WARN | 无 | notify_all() |

### 3.5 异常日志

引入与 `test_multi_cam_multi_host` 相同的 `logException()` 函数和 FATAL/ERROR/WARN/INFO 计数器。

---

## 四、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 添加 `logException()` + 异常计数器 | 5min |
| 2 | 添加会话日志（calib_session_*.md） | 10min |
| 3 | 添加 UI 水印 + 十字线 + 网络同步按键屏蔽（ESCESC/q/SPACE/z/c/t） | 15min |
| 4 | 添加指标采集（拍照/撤销/传输统计） | 10min |
| 5 | 添加 C1-C6 异常检查 | 10min |
| 6 | 编译 + 验证 | 10min |

**总计：~60 分钟**

---

## 五、不做

- CPU 占用率监控（标定拍照是单帧操作，CPU 占用无分析价值）
- 录制相关指标（无 ram_buffer，无连续录制）
- 跨主机 BlockID 同步（软件触发，无意义）
- max_num_buffer 配置（当前硬编码 150，标定场景帧率低无需优化）
