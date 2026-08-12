# calib_cam_chain.cpp — 特征提取并行化方案

> 当前瓶颈：Stage 2 特征提取 ~180s（200 次 `ReadImage` + `FindCalibObject` 完全串行）
> 目标：通过并行化 ReadImage 减少磁盘 I/O 等待时间

---

## 一、HALCON 并行模型调研结论

### 1.1 关键发现

| 结论 | 来源 |
|------|------|
| `FindCalibObject` **修改共享的 `CalibDataID`**，多线程必须同步访问 | MVTec operator reference: "access to the value of this parameter must be synchronized if it is used across multiple threads" |
| `FindCalibObject` 标注为 **"Processed without parallelization"** — AOP 不会自动并行化它 | MVTec reference `set_calib_data_calib_object` |
| `ReadImage` 是 **reentrant (global scope)** — 可以在不同线程并行调用读不同文件 | MVTec parallel programming guide |
| `CalibDataID` 不可合并 — 无法多个线程各建一个模型最后合并 | HALCON 无此 API |

### 1.2 结论

**`FindCalibObject` 必须串行化**（共享 `CalibDataID`），但 **`ReadImage` 可以并行化**（每个线程读不同的文件，无共享状态）。

---

## 二、优化方案

### 2.1 方案 A：生产者-消费者管线（推荐）

**思路**：一个 I/O 线程预读下一张图，主线程消费当前图做 `FindCalibObject`。

```
I/O 线程:    ReadImage(img_1) → [ready] → ReadImage(img_3) → [ready] → ...
主线程:      FindCalibObject(img_0) → FindCalibObject(img_1) → FindCalibObject(img_2) → ...
```

**实现**：双缓冲 + `std::condition_variable`

```cpp
struct ImageSlot {
    HObject image;
    int camIdx, imgIdx;
    bool ready = false;
};

ImageSlot slots[2];  // 双缓冲
mutex mtx;
condition_variable cv;

// I/O 线程
void ioThread() {
    for (auto& task : all_tasks) {
        // 等待空槽位
        unique_lock lk(mtx);
        cv.wait(lk, [&]{ return !slots[write_idx].ready; });
        // 读取图片到槽位
        ReadImage(&slots[write_idx].image, filename);
        slots[write_idx].ready = true;
        lk.unlock();
        cv.notify_one();
    }
}

// 主线程
for (auto& task : all_tasks) {
    // 等待就绪槽位
    unique_lock lk(mtx);
    cv.wait(lk, [&]{ return slots[read_idx].ready; });
    // FindCalibObject
    FindCalibObject(slots[read_idx].image, hv_CalibDataID, ...);
    slots[read_idx].ready = false;
    lk.unlock();
    cv.notify_one();
}
```

**优点**：
- 内存占用极小（仅 2 张图）
- I/O 与计算完全重叠
- 不需要管理大量线程

**预期收益**：
- 若 ReadImage 占每次调用 30%（~270ms/900ms），则可节省 200 × 0.27s ≈ 54s（30% 提升）
- 若 ReadImage 占每次调用 50%，可节省 ~90s（50% 提升）

### 2.2 方案 B：多 I/O 线程 + 串行 FindCalibObject

**思路**：多个线程并行 ReadImage，结果放入队列，主线程串行消费。

```
Worker 1: ReadImage → enqueue → ReadImage → enqueue → ...
Worker 2: ReadImage → enqueue → ReadImage → enqueue → ...
Main:     dequeue → FindCalibObject → dequeue → FindCalibObject → ...
```

**优点**：多磁盘/多文件系统时可以进一步加速 I/O

**缺点**：
- 需要队列 + 同步开销
- 内存占用取决于队列深度
- 对于单 SSD，多线程 I/O 通常不如单线程预读

### 2.3 方案 C：HALCON AOP 调优（防御性）

```cpp
// 在 action() 开头
get_system("processor_num", &hv_cores);
set_system("parallelize_operators", "true");
set_system("thread_num", hv_cores);  // 或不设，默认用全部核心
```

`FindCalibObject` 虽标注 "Processed without parallelization"，但 `ReadImage` 等 I/O 算子可能受益于 AOP。作为基线优化，成本为零。

---

## 三、实施前必须：ReadImage vs FindCalibObject 耗时细分

并行化的收益完全取决于 ReadImage 在每次调用中的占比。需要先在 Stage 2 内部加计时：

```cpp
double t_read = 0, t_find = 0;
for (...) {
    auto tr0 = steady_clock::now();
    ReadImage(&ho_Image, ...);
    auto tr1 = steady_clock::now();
    t_read += duration<double>(tr1 - tr0).count();

    FindCalibObject(...);
    auto tf1 = steady_clock::now();
    t_find += duration<double>(tf1 - tr1).count();
}
cout << "[Timer] Stage 2 - ReadImage: " << t_read << "s (" << (t_read/dt_feat*100) << "%)" << endl;
cout << "[Timer] Stage 2 - FindCalibObject: " << t_find << "s (" << (t_find/dt_feat*100) << "%)" << endl;
```

| ReadImage 占比 | 方案 A 预期收益 | 建议 |
|---------------|---------------|------|
| < 20% | < 36s | 不值得并行化，关注 FindCalibObject 参数调优 |
| 20-40% | 36-72s | **推荐方案 A**（双缓冲管线） |
| > 40% | > 72s | **推荐方案 B**（多 I/O 线程），考虑内存限制 |

---

## 四、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | Stage 2 内部加 ReadImage / FindCalibObject 细分计时 | 5min |
| 2 | 运行一次，获取 ReadImage 占比 | 3min（实机） |
| 3 | 根据占比选择方案 A 或 B，实施并行化 | 20-30min |
| 4 | 编译 + 实机测速对比 | 5min |

---

## 五、后续优化方向（不依赖并行化）

1. **减少 ReadImage 次数**：如果多帧图片可以复用（同一相机同一图片用于多帧标定），缓存已读图片
2. **HDevelop 性能分析器**：用 HALCON 官方工具定位 `FindCalibObject` 内部耗时，调 alpha/sigma 参数减小搜索空间
3. **排除不必要参数优化**：`set_calib_data(..., 'excluded_settings', ['focus','kappa',...])` 减少 `CalibrateCameras` 求解变量数
