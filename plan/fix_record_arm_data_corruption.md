# test_record_arm_data.cpp — 花屏 + cv::resize 崩溃根因分析

> 症状：相机图像花屏，随后 `cv::resize` 断言失败（`inv_scale_x > 0`，即 src 宽度为 0）导致程序退出。
> Pylon Viewer 验证相机正常 → 问题在采集程序。
> 已回滚 GrabSucceeded 防护改动（暴露问题，先诊断再修复）。

---

## 一、make_shared 假设分析

### 1.1 结论：不是根因

**`test_multi_cam.cpp`（长期稳定运行的多相机采集程序）使用完全相同的模式**：

```cpp
// test_multi_cam.cpp:305 — 与 test_record_arm_data.cpp 完全一致
auto ctx = make_shared<CameraContext>(i, camera_ids[i], save_dirs[i]);
cam_ctxs.push_back(ctx);
...
ctx->copy_thread = thread(copyWorker, ctx);
```

### 1.2 make_shared 机制逐条检查

| 怀疑点 | 分析 | 判定 |
|--------|------|------|
| make_shared 分配物+控制块单块内存，导致错乱 | 单块分配是优化特性，无错乱机制；无 weak_ptr 时不延迟释放 | ✅ 安全 |
| vector 扩容移动 shared_ptr | 只移动句柄，CameraContext 对象本身在堆上从不移动 | ✅ 安全 |
| 线程持 shared_ptr 拷贝，上下文提前销毁 | 线程参数拷贝使 refcount>0，主线程 cleanup 先 join 再退出 | ✅ 安全 |
| CameraContext 被析构时线程未结束 | cleanup 已 join（`test_record_arm_data.cpp:662-667`） | ✅ 安全 |
| 全局 `cam_ctxs` 在 main 返回后析构 | 此时线程已全部 join | ✅ 安全 |

### 1.3 改成裸指针的风险

- `vector<CameraContext>` 值存储：CameraContext 含 `thread`/`mutex`，**不可拷贝不可移动** → 编译不过
- `vector<CameraContext*>` + `new`：必须手写析构/异常安全，引入 dangling/double-free 风险
- `unique_ptr`：与 shared_ptr 语义等价，不改变任何生命周期行为

**结论：改 make_shared 无意义，不做。**

---

## 二、真正的可疑点（按嫌疑排序）

### 2.1 ★★★ 像素格式回退链导致 buffer 解释错误（花屏的直接解释）

`BaslerCamera::open()` 的格式协商顺序（[basler.cpp:57-94](../cpp_eyetracker/utils/cam/src/basler.cpp#L57-L94)）：

```
1. BayerRG8 (1 字节/像素) → is_mono_ = false
2. RGB8      (3 字节/像素) → is_mono_ = false
3. Mono8     (1 字节/像素) → is_mono_ = true
```

**漏洞**：若某台相机 BayerRG8 设置失败但 RGB8 成功（ace2 彩色相机通常两者都支持，但部分型号只开放 RGB8），则：

- 缓冲区实际为 **w×h×3 字节**
- `copyWorker` 却按 `cv::Mat(h, w, CV_8UC1, buffer)` 包裹 → **只看到前 1/3 数据**
- `renderThumbnailGrid` 用 `COLOR_BayerRG2RGB` 解码 → **花屏**
- 更严重：`GetWidth()/GetHeight()` 返回传感器尺寸（正常值），但 3 字节/像素的缓冲区被 1 通道 Mat 包裹，clone 的 `w*h` 字节里混着错位的数据

另外 `is_mono` 在 `open()` 中通过**回退链判定**，而实际 PixelFormat 从未打印确认过。

### 2.2 ★★ 首帧未跳过（与 test_multi_cam 的差异）

test_multi_cam 回调有门控（[test_multi_cam.cpp:162](../cpp_eyetracker/tests/utils/cam/test_multi_cam.cpp#L162)）：

```cpp
if (!ctx->offset_initialized && state->frame_counter > 1) { ... }
if (ctx->offset_initialized) { ... push ... }
```

**第一帧不 push**（用于 offset 初始化）。test_record_arm_data 从第一帧就 push。StartGrabbing 刚返回时首帧可能为不完整/零填充缓冲。

### 2.3 ★ `is_mono` 数据竞争

```cpp
// captureWorker 线程写（一次）
ctx->is_mono = ctx->cam.isMono();
// UI 线程每帧读（无锁）
if (cam_ctxs[i]->is_mono) cvtColor(...);
```

bool 非原子读写跨线程 = 标准意义上的 UB（x86 上通常 benign，但应修复）。

### 2.4 GrabResultPtr 缓冲区生命周期（理论安全，待诊断确认）

- `OnImageGrabbed` 传 const-ref，queue push 时拷贝 → refcount+1 → 缓冲在队列期间不被回收
- `GrabStrategy_OneByOne` + `MaxNumBuffer=150` → 每帧不同缓冲，无覆盖
- 理论安全，但若某处 refcount 意外失效（如隐式转换丢失），会出现"缓冲被复用覆盖"→ 花屏 + 尺寸元数据漂移 → resize 崩溃

---

## 三、诊断计划（先暴露问题，不改逻辑）

| 步骤 | 内容 | 目的 |
|------|------|------|
| 1 | ✅ 回滚 GrabSucceeded 防护 | 暴露问题 |
| 2 | 启动时打印每台相机 `is_mono` 判定 + SN | 确认格式协商结果 |
| 3 | `copyWorker` 每帧打印 `w×h, blockID, buffer 指针地址`（先只打前 10 帧 + 尺寸变化时） | 观察尺寸是否漂移、指针是否复用 |
| 4 | render 前检测 `local_raw.cols==0` 或尺寸异常 → **打印 ERROR 而非静默** | 抓住崩溃前的证据 |
| 5 | 对比崩溃时花屏的相机 SN 与 is_mono | 锁定 2.1 假设 |

**运行一次后，根据打印结果确定修复方案（见第四节）。**

---

## 四、候选修复方案（按诊断结果选择）

### 方案 A：修复像素格式判定（对应 2.1）

- `open()` 中打印实际生效的 `PixelFormat` 字符串
- RGB8 fallback 路径：copyWorker 按 3 字节/像素处理（`CV_8UC3`），或在 open 中直接用 `converter_` 转成 BGR8 统一出口
- 在 `calib_arm.yaml` 中按 SN 显式配置 mono/color，跳过回退链

### 方案 B：对齐 test_multi_cam 的首帧门控（对应 2.2）

- 回调加 `frame_counter`，首帧丢弃

### 方案 C：is_mono 改 atomic（对应 2.3）

- `atomic<bool> is_mono`，一处改动

---

## 五、实施顺序

1. 诊断打印（第三节步骤 2-4）→ 编译
2. 实机运行 → 收集崩溃前输出
3. 按证据选方案 A/B/C → 修复
4. 复测
