# calib_with_HALCON.cpp — CXP-12 相机 + imaFlex 采集卡正确配置计划

> 症状：master 花屏 + `cv::resize` 崩溃 (`inv_scale_x > 0`)。Pylon Viewer 同时采集 10 台正常、无花屏。
> 结论方向：我们的采集配置与 Pylon Viewer 不一致，某些流参数设置不当。
> 硬件：Basler a2A2448-210cc/cm（ace 2 CXP-12, IMX537, 2448×2048 默认, 212fps 满速）
> 采集卡：Basler imaFlex CXP-12 Quad（4 端口 × 12.5Gbps, PCIe 3.0 x8, 1.5GB DDR4, PoCXP 17W/端口）
> Pylon 26.01

---

## 一、官方文档关键事实

| 项目 | 事实 | 来源 |
|------|------|------|
| 相机接口 | CoaXPress 2.0, HDBNC 单链路 (a2A2448-210 为单口 CXP-12 相机) | Basler a2A2448-210cc 产品页 |
| 默认分辨率 | 2448×2048（全幅 2464×2064） | 同上 |
| 最大帧率 | CXP-12 下 212fps | 同上 |
| 链路配置参数 | `CxpLinkConfiguration`: Auto / CXP12_X1 / X2 / X4 | Basler pylon API 文档 |
| 采集卡端口数 | 4 个 CXP-12 端口, 每端口 12.5Gbps | imaFlex CXP-12 Quad 产品页 |
| PCIe | 3.0 x8（典型 6.5GB/s） | 同上 |
| PoCXP | 17W/端口（a2A2448 功耗 ~5.5W, 充足） | 同上 |
| **流缓冲参数** | 官方示例: `MaxNumBuffer=16`, `MaxTransferSize=1048568`, `NumMaxQueuedUrbs=64` | docs.baslerweb.com Stream Grabber Parameters |

**⚠️ 核心疑点**：我们代码强制 `MaxNumBuffer=150`（`basler.cpp` 默认值），是官方示例值的 ~10 倍。150 × 5MB × 10 台 = 7.5GB DMA 缓冲。若驱动/采集卡的排队深度（URB 队列, 官方示例 64）或 DMA 描述符上限低于 150，驱动可能**静默复用仍在使用中的缓冲** → 缓冲被覆盖 → 花屏 + 帧元数据损坏 → `cv::resize` 崩溃。Pylon Viewer 用驱动默认缓冲数（远小于 150），所以正常。

---

## 二、诊断阶段：先收集证据（不改逻辑）

### 2.1 导出 Pylon Viewer 的参考配置

在 Pylon Viewer 中对每台相机（至少 1 台彩色 + 1 台黑白）：

1. Feature 面板筛选 `Cxp` 和 `Pixel`，截图记录：
   - `CxpLinkConfiguration`（应为 Auto）
   - `PixelFormat`（Pylon Viewer 默认值，确认是 BayerRG8 还是其他）
   - `Width` / `Height`（确认 2448×2048）
2. Stream Grabber Parameters 面板截图：
   - `MaxNumBuffer`（**关键**：Viewer 用了多少个缓冲）
   - `MaxTransferSize`、`NumMaxQueuedUrbs`
3. 运行 10 台同时采集 ≥1 分钟确认无花屏，作为基准

### 2.2 我们的程序加帧诊断（临时）

`calib_with_HALCON.cpp` 的 copyWorker 加一次性 + 异常检测打印：

```cpp
// 每台相机首帧: 分辨率 / payload / 缓冲指针
// 持续: 缓冲指针复用检测 — 若某帧 GetBuffer() 返回的地址与仍在队列中的帧相同 → 打印 OVERLAP
```

具体做法：维护 `set<void*> live_ptrs`（进队时插入、出队时删除），新帧指针已在集合中 → 打印 `[BufReuse]`——直接证实缓冲被驱动复用。

### 2.3 ★ 精细调试：采集卡事件 + TL 统计（基于官方示例 `FG_Events_GenApi_Notifications`）

官方示例揭示：imaFlex 卡有独立的 **TL (Transport Layer) 事件系统**，可以实时捕获链路层异常。示例位于 `C:\hitsz\projects\new_dataset\FG_Events_GenApi_Notifications\Grab.cpp`。

#### 2.3.1 官方示例关键机制

```cpp
// 1) 以 CXP 设备类打开采集卡接口 (每张卡一个 interface)
CInterfaceInfo info;
info.SetDeviceClass(Pylon::BaslerGenTlCxpDeviceClass);
info.SetInterfaceID("iF-CXP12-Q SN_a640042c");   // 本机每张卡的序列号
CUniversalInstantInterface instantInterface(info);
instantInterface.Open();

// 2) 通过相机访问 TL (采集卡侧) 参数
camera.GetTLParams().EventSelector.SetValue("FrameTriggerMissed");  // 帧触发丢失事件
camera.GetTLParams().EventNotification.SetValue("On");

// 3) GenApi 节点回调: 事件软计数器变化即触发
CIntegerParameter counter(camera.GetTLNodeMap(), "EventFrameTriggerMissedSoftCounter");
CallbackHandleType h = Register(counter.GetNode(), &cb, &GenapiCallbacks::membercallback);
```

#### 2.3.2 需要监控的 TL 事件/计数器（写入诊断）

| 节点 | 含义 | 花屏关联 |
|------|------|---------|
| `EventFrameTriggerMissedSoftCounter` | 触发脉冲丢失计数 | 触发链路问题 |
| `EventCustomSignalEvent0SoftCounter` | 自定义信号事件 | 链路握手/训练事件 |
| `EventFrameTransferStartSoftCounter` | 帧传输开始 | 传输吞吐监控 |

#### 2.3.3 流统计参数（Stream Grabber Parameters, 每次读取快照）

| 参数 | 含义 | 花屏关联 |
|------|------|---------|
| `BufferUnderrunCount` | 缓冲下溢 | ★ 缓冲队列耗尽 = 覆盖嫌疑 |
| `FailedBufferCount` | 失败缓冲 | ★ 缓冲损坏直接证据 |
| `FailedPacketCount` | 失败包 | 链路层错误 |
| `ResynchronizationCount` | 链路重同步 | ★ 链路训练/断链证据 |
| `MissedFrameCount` | 丢帧 | 吞吐不足 |

读取方式：`camera.GetStreamGrabberParams().BufferUnderrunCount.GetValue()`。

#### 2.3.4 诊断实施（新建独立诊断程序，不改主程序）

新建 `tests/utils/cam/diag_cxp_events.cpp`：

1. 枚举本机全部 `BaslerGenTlCxpDeviceClass` 接口，打印每张卡的 `InterfaceID` + 序列号
2. 对每台相机：
   - 注册 `FrameTriggerMissed` / 帧传输开始等事件回调（带时间戳打印）
   - 每秒打印流统计快照：Underrun / FailedBuffer / FailedPacket / Resync / MissedFrame
3. 用户同时运行 `calib_with_HALCON.cpp`（或独立开流），观察花屏出现瞬间哪个计数器跳变
4. **判定表**：
   - `FailedBufferCount` 跳变 → 缓冲损坏/覆盖 → 证实 MaxNumBuffer=150 假说
   - `ResynchronizationCount` 跳变 → 链路不稳定 → 查线缆/供电/坏卡
   - 无任何跳变但花屏 → 问题在应用层（我们的 Mat 包裹/渲染），转向实验 C/D

---

## 三、配置对齐实验（一次改一项，每项后跑 10 台 ≥10 分钟）

### 实验 A：MaxNumBuffer 150 → 10（最高优先级）

`basler.hpp` 默认值 `maxNumBuffer_ = 150` → 改为 10（或与 Pylon Viewer 记录值一致）。

```cpp
int maxNumBuffer_ = 10;  // 对齐 Pylon Viewer; CXP 流缓冲不宜过大
```

如果花屏消失 → 根因坐实。后续按需要微调（16 / 32）。

### 实验 B：显式设置 CxpLinkConfiguration = Auto

`basler.cpp::open()` 中，在打开后：

```cpp
try {
    CEnumerationPtr(nodemap.GetNode("CxpLinkConfiguration"))->FromString("Auto");
} catch (...) {}
```

排除链路协商异常（错协商到 X2/X4 或多链路模式导致数据错乱）。

### 实验 C：像素格式路径核对

- 确认相机实际 PixelFormat 与我们的假设一致（BayerRG8, 1 字节/像素）
- 若 Pylon Viewer 显示默认格式不是 BayerRG8，改为**不强制**格式、跟随相机默认，`is_mono` 按实际格式判定
- 日志打印 `open()` 后实际生效的 `PixelFormat` 字符串

### 实验 D：GrabStrategy

若 A-C 无效，尝试 `GrabStrategy_LatestImageOnly`（Viewer 的 Continuous Shot 行为类似），并减少队列深度。

---

## 四、代码改动清单

| 文件 | 改动 | 阶段 |
|------|------|------|
| `tests/utils/cam/diag_cxp_events.cpp` | **新建**: 采集卡事件 + TL 统计监控程序 | 诊断 (2.3) |
| `utils/cam/include/cam/basler.hpp` | `maxNumBuffer_` 150 → 10 | 实验 A |
| `utils/cam/src/basler.cpp` | open() 显式 `CxpLinkConfiguration=Auto` + 打印实际 PixelFormat | 实验 B/C |
| `tests/utils/cam_calib/calib_with_HALCON.cpp` | copyWorker 缓冲复用检测诊断（临时） | 诊断 (2.2) |
| `cfg/cam_calib.yaml` | 可选: `max_num_buffer` 配置键 | 实验 A |

---

## 五、验证标准

1. 10 台相机（含坏风扇卡）连续采集 ≥30 分钟无花屏、无 resize 崩溃
2. 缓冲复用检测零告警
3. 与 Pylon Viewer 的参数面板逐项一致（MaxNumBuffer / LinkConfig / PixelFormat）

---

## 六、若实验 A-D 均无效

1. 检查坏风扇卡——它可能就是花屏源（热错误帧）。对比"10 台"与"6 台"两组测试
2. 用 `pylon Viewer` 的长跑 + 我们的 6 台长跑交叉定位具体哪张卡/哪几台相机触发
3. 联系 Basler 支持：imaFlex + Pylon 26.01 + 150 buffers 的已知问题
