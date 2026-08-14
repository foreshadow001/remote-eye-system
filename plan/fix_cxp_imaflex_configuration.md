# CXP-12 相机 + imaFlex 采集卡 — 花屏/坏帧根因排查计划

> ✅ **已解决 (2026-08-14)**：根因 = **imaFlex GenTL 驱动流初始化非线程安全**。
> 10 线程并发 Open+StartGrabbing → 部分流行排序/DMA 配置错乱 → 零行掩码/图像拼接/花屏/蓝屏。
> 修复：`BaslerCamera::open()/start()` 全局互斥锁串行化 + 200ms 流建立间隔（basler.cpp）。
> 全部 14 个采集程序已编译生效；MaxNumBuffer 恢复 16。
>
> 历史记录（排查过程）保留在下方，供参考。

> 症状：master 花屏 + `cv::resize` 崩溃 (`inv_scale_x > 0`)。坏帧特征：`grab=OK`、帧高度逐帧变化（2043~2047）、payload = w×h + n×16。
> 硬件：Basler a2A2448-210cc/cm（ace 2 CXP-12, IMX537, 2448×2048 默认, 212fps 满速）
> 采集卡：Basler imaFlex CXP-12 Quad（4 端口 × 12.5Gbps, PCIe 3.0 x8, 1.5GB DDR4, PoCXP 17W/端口）× 3 张
> Pylon 26.01

---

## 〇、已完成的排查（截至 2026-08-14）

| 项目 | 结论 |
|------|------|
| 相机/流配置 | ✅ 全对（BayerRG8, PayloadSize=5013504, MaxBufferSize=5760000, CxpLinkConfiguration=Auto） |
| MaxNumBuffer 调优 | ✅ 150→10/16（150 会加重故障；配置本身无罪） |
| 回调内同步拷贝（消除缓冲延迟持有） | ✅ 已推广到全部 10 个采集程序；短跑无花屏，**长跑后花屏复发** → 非唯一根因 |
| stall 检查误杀 | ✅ 已全部移除（CXP 链路训练 >1s） |
| Pylon Viewer 短测（~1 分钟） | 正常 —— 但**未做过与故障同条件的长跑** |
| 坏风扇卡 | 拔过又插回；**当前安装状态未确认** |

## ★ 新证据指向的新假设（按优先级）

**失败模式的关键事实**：运行开始时干净 → 运行一段时间后 BadFrame 从少到多（洪水）→ 曾经 BSOD（无 WER 转储 = 硬件级故障）。同一配置不同运行结果不同。

| 优先级 | 假设 | 依据 |
|--------|------|------|
| **H1 热故障**（坏风扇卡仍在机箱内） | 干净启动 → 温度爬升 → CXP 链路错误指数增长 → 截断帧 | 渐进式失败曲线 + 已知风扇脱焊 + BSOD |
| **H2 PCIe 电源管理/ASPM** | Windows 更新重置电源计划 → PCIe 链路降功耗 → DMA 间歇截断（16 字节粒度） | 16 字节对齐截断 + 与负载/帧率无关 + 间歇性 |
| **H3 PCIe 插槽带宽/布线** | 卡插在 x4 电气插槽或与 NVMe 共享带宽 | 未验证过 |
| **H4 链路速率协商过低** | 链路训练在 CXP-6/3 而非 CXP-12（CxpLinkSpeed 节点 N/A 未读到） | 未验证过 |

---

## 一、诊断阶段 2.0：新证据收集（按 H1→H4 顺序）

### 1.1 【H1】硬件状态确认（10 分钟）

1. 确认机箱内**哪几张卡在、坏风扇卡是否在**
2. 安装 HWiNFO64 → 记录 3 张卡的 **GPU/PCIe 温度** 曲线
3. 跑 100fps × 10 台，记录 **首次 BadFrame 出现时间** 与 **温度的关系**
4. 若洪水出现时某卡温度 >85°C → H1 坐实

### 1.2 【H2】PCIe 电源管理（5 分钟，零成本）

1. 控制面板 → 电源选项 → 高性能计划 → 更改计划设置 → 更改高级电源设置
2. **PCI Express → 链接状态电源管理 → 关闭**（Windows 更新可能重置过）
3. 重跑 30 分钟对比

### 1.3 【H3】PCIe 链路宽度确认（5 分钟）

HWiNFO/GPU-Z → Bus Interface 页 → 每张 imaFlex 的 **实际链路宽度 × 速率**（应为 x8 @ Gen3）。

### 1.4 【H4】链路速率 + 精细事件（基于官方示例 FG_Events_GenApi_Notifications）

官方示例显示 imaFlex 有独立的 TL 事件系统（`camera.GetTLParams()` + GenApi 回调）：
- `EventFrameTriggerMissedSoftCounter`、`EventCustomSignalEvent0SoftCounter`
- 流统计（若节点可见）：`FailedBufferCount` / `ResynchronizationCount` / `MissedFrameCount`
- 采集卡接口侧参数：`CInterfaceInfo` + `BaslerGenTlCxpDeviceClass` 打开卡接口读取

新建 `tests/utils/cam/diag_cxp_events.cpp`：枚举 3 张卡 → 每台相机注册事件回调 + 每秒打印统计快照 → 与 calib_with_HALCON 同时运行，花屏瞬间看哪个计数器跳变。

**判定表**：
| 跳变项 | 结论 |
|--------|------|
| `FailedBufferCount` | 驱动级缓冲损坏 |
| `ResynchronizationCount` | CXP 链路重训练（线缆/供电/热） |
| 卡温度 >85°C | H1 热故障 |
| 无任何跳变 | 应用层问题，重新审视拷贝路径 |

### 1.5 Pylon Viewer 长跑对照（关键盲区）

用 Pylon Viewer 以 **100fps × 10 台连续跑 30 分钟**（与故障同条件），并**仔细检查画面底部几行**（截断帧的特征区域）：
- Viewer 也坏 → 硬件/驱动问题，与我们的代码无关
- Viewer 不坏 → 我们程序特有的问题（回到应用层排查）

---

## 二、原有配置对齐实验（保留作为后续手段）

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
