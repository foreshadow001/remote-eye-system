# test_multi_cam_multi_host.cpp 指标采集与监控优化计划

> 目标文件：`cpp_eyetracker/tests/utils/cam/test_multi_cam_multi_host.cpp`（1190 行）
> 最后更新：2026-07-28

---

## 一、采集阶段划分

整个程序运行周期分为 **5 个阶段**：

```
启动 ──► 初始化 ──► 待机 ──► [r] 录制 ──► 落盘 ──► 后处理 ──► 待机（循环）
        阶段1      阶段2      阶段3       阶段4     阶段5
```

| 阶段 | 名称 | 触发条件 | 结束条件 | 耗时特征 |
|------|------|----------|----------|----------|
| **1** | **初始化** | 程序启动 | 所有相机 `start()` 返回 | 一次性，~2-5秒 |
| **2** | **待机/预览** | 相机全部推流 | 用户按 'r' 或收到 CMD_START | 持续，不可控时长 |
| **3** | **录制** | `instantTrigger()` 调用 | `ram_buffer` 写满（`recorded_frames == total`） | 固定时长（如 1.2s） |
| **4** | **落盘** | 所有相机 `dump_ready` | 所有 `dumpToDiskWorker` 线程完成 | 取决于数据量，~2-10秒 |
| **5** | **后处理** | 落盘完成 | JPG 转换 + 统计报告输出 | 取决于 write_jpg 开关，~5-30秒 |

---

## 二、已有指标详解

### 2.1 单相机指标（现有 7 个，独立采集）

#### 2.1.1 last_block_id（阶段 2-3，实时）

- **定义**：该相机最近一次收到的帧的 BlockID
- **采集位置**：`copyWorker()` 每次从 `copy_queue` 出队处理完一帧后
- **计算方式**：直接读取 Pylon 回调传入的 `FrameMeta::blockID`
- **存储**：`CameraContext::last_block_id`（`atomic<int64_t>`）
- **用途**：健康监控的辅助字段，记录最后一次 BlockID 便于调试。不直接参与故障判定（故障判定用 `last_frame_time`）

#### 2.1.2 last_frame_time（阶段 2-3，实时）

- **定义**：该相机最近一次收到帧的墙上时钟时刻
- **采集位置**：`copyWorker()` 每次从 `copy_queue` 出队处理完一帧后
- **计算方式**：`std::chrono::steady_clock::now()` 直接获取当前时刻
- **存储**：`CameraContext::last_frame_time`（`atomic<steady_clock::time_point>`）
- **用途**：健康监控的核心判定依据。主循环每轮计算 `(now - last_frame_time) > 1.0s`，若为真且 `has_streamed` 为真，则判定该相机卡死，触发故障停机。选择 1.0s 作为超时阈值是因为正常帧率 200fps 下帧间隔仅 5ms，1s 意味着连续丢失约 200 帧，足以确认并非瞬时抖动

#### 2.1.3 has_streamed（阶段 2-3，实时）

- **定义**：该相机是否曾经成功收到过帧
- **采集位置**：`copyWorker()` 第一次处理帧后
- **计算方式**：布尔值，初始 `false`，首次收到帧且处理完毕后置 `true`
- **存储**：`CameraContext::has_streamed`（`atomic<bool>`）
- **用途**：区分"相机还没开始推流"和"相机已经停流"。健康检查在 `has_streamed` 为假时跳过该相机，避免启动阶段误报故障

#### 2.1.4 captured_frames（阶段 2-3，实时）

- **定义**：该相机从启动以来累计采集的帧数
- **采集位置**：`captureWorker()` 中的 Pylon 回调函数（每次相机产生一帧时就触发）
- **计算方式**：每次回调执行 `ctx->captured_frames++`
- **存储**：`CameraContext::captured_frames`（`atomic<int>`）
- **用途**：累计计数器，反映相机硬件端产生的总帧数。目前仅在回调中递增，未参与任何判定逻辑，也未在 UI 或报告中输出。属于"有采集但未使用"的指标

#### 2.1.5 recorded_frames（阶段 3，实时）

- **定义**：该相机在本次录制中已写入 `ram_buffer` 的帧数
- **采集位置**：`copyWorker()` 录制分支（`if (ctx->recording)`），每次 `memcpy` 到 `ram_buffer` 后
- **计算方式**：
  1. 读取当前序号 `seq = recorded_frames.load()`
  2. 将帧数据拷贝到 `ram_buffer[seq]`
  3. `recorded_frames.store(seq + 1)`
  4. 若 `seq + 1 == total_record_frames`，则置 `recording = false`、`dump_ready = true`
- **存储**：`CameraContext::recorded_frames`（`atomic<int>`）
- **用途**：
  - 判定录制结束：达到 `total_record_frames` 时停止录制
  - UI 显示："143/240"
  - `instantTrigger()` 中重置为 0

#### 2.1.6 后处理统计：保存帧数 total_saved（阶段 5，事后）

- **定义**：本次录制该相机实际写入磁盘的帧数
- **计算方式**：`parseLogFile()` 读取日志文件，按行数统计。一行 = 一个 `LogEntry` = 一帧
- **存储**：不存储，临时局部变量
- **不足**：需事后读文件，I/O 开销；且与内存中的 `recorded_frames` 理论上应相等，但当前未做交叉验证

#### 2.1.7 后处理统计：丢帧数 / 实际FPS / 起始BlockID（阶段 5，事后）

这三个单相机指标均在 `parseLogFile()` 返回的日志数组上计算：

- **丢帧数 `dropped_frames`**：
  ```
  遍历日志数组，i 从 1 到 n-1：
    diff = logs[i].blockID - logs[i-1].blockID
    if diff > 1: dropped_frames += (diff - 1)
  ```
  原理：正常帧的 BlockID 连续递增（差值恒为 1）。差值 > 1 说明中间有帧未被保存。**注意**：此方法低估了丢帧数——无法检测首帧之前和末帧之后的丢帧

- **实际 FPS `actual_fps`**：
  ```
  duration_s = (logs.back().timestamp - logs.front().timestamp) / 10,000,000.0
  actual_fps = (total_saved - 1) / duration_s
  ```
  Pylon 时间戳单位是 100ns（10⁻⁷秒），除以 10⁷ 转为秒。`total_saved > 1` 时才计算。得到的是**平均帧率**，不反映瞬时波动

- **起始 BlockID `start_block_id`**：
  ```
  start_block_id = logs.front().blockID
  ```
  硬件触发下所有相机的 start_block_id 应一致

**存储**：三者均为临时局部变量，不持久化。当前输出方式是临时 `cout` 逐行打印

### 2.2 全局指标（现有 1 个）

#### 2.2.1 落盘总耗时（阶段 4）

- **定义**：从所有相机 `dump_ready` 到所有 `dumpToDiskWorker` 线程完成的墙上时间
- **计算方式**：
  ```
  dump_duration = dump_end_time - dump_start_time
  ```
- **依赖的单相机指标**：无（纯粹的墙上时间测量）
- **存储**：局部变量，仅用于打印
- **当前输出**：`"[Performance] Disk Dump Finished! Time: X.XXXs"`
- **不足**：只输出总耗时，没有每台相机各自的落盘耗时，也没有计算写入带宽（MB/s）。这是一个"有总无分"的全局指标——只知道整体速度，无法定位是哪台相机慢

---

## 三、新增指标详解

### 3.1 单相机新增指标（共 9 个）

#### 3.1.1 max_queue_size — 队列压力（阶段 2-3，实时采集，事后输出）

- **定义**：该相机的 `copy_queue` 在阶段内达到的最大长度
- **采集位置**：`copyWorker()` 每次从队列出队后
- **计算方式**：
  ```
  copy_queue 出队后：
    current_size = copy_queue.size()  // 出队后剩余的数量
    if current_size > max_queue_size:
        max_queue_size = current_size
  ```
  出队后 queue 中还剩多少帧，即反映了当前积压程度。出队后还有 3 帧 → captureWorker 领先 copyWorker 3 帧
- **存储**：`CameraContext::max_queue_size`（`atomic<int>`）
- **输出**：仅在结构化报告中打印最终值，不做实时 UI 预警
- **重置**：每次 `instantTrigger()` 调用时重置为 0

#### 3.1.2 first_recorded_block_id / last_recorded_block_id — 首末帧 BlockID（阶段 3，实时）

- **定义**：本次录制该相机写入 `ram_buffer` 的第一帧和最后一帧的 BlockID
- **采集位置**：`copyWorker()` 录制分支
- **计算方式**：
  - 首帧：`seq == 0` 时直接读取 `meta.blockID` 记录
  - 末帧：每次写入时用当前 `meta.blockID` 覆盖记录
- **存储**：`CameraContext::first_recorded_block_id`（`int64_t`）、`CameraContext::last_recorded_block_id`（`int64_t`）
- **用途**：计算 BlockID 跨度 `span = last - first + 1`，与 `recorded_frames` 交叉验证丢帧数。同时也是全局同步偏移指标的原始数据来源

#### 3.1.3 first_frame_time — 首帧到达时刻（阶段 3，实时）

- **定义**：本次录制该相机首帧写入 `ram_buffer` 的墙上时钟时刻
- **采集位置**：`copyWorker()` 录制分支，`seq == 0` 时
- **计算方式**：`first_frame_time = steady_clock::now()`
- **用途**：与全局触发时刻 `global_record_start_time` 做差得到该相机的触发延迟（详见下文 trigger_latency_ms）。也是全局"最大触发延迟跨度"的原始数据来源
- **存储**：`CameraContext::first_frame_time`（`steady_clock::time_point`）

#### 3.1.4 trigger_latency_ms — 触发延迟（阶段 3，派生自 3.1.3 + 全局触发时刻）

- **定义**：从触发录制（按下 'r'）到该相机首帧数据写入 `ram_buffer` 的墙上时间差
- **数据流**：
  ```
  [主线程]                 [copyWorker 线程]
  instantTrigger()
  ├─ t0 = now()            （此即 global_record_start_time）
  ├─ 清空 copy_queue
  └─ recording = true
                                 出队第一帧
                                 seq = 0
                                 memcpy → ram_buffer[0]
                                 t1 = now()  ← first_frame_time

  trigger_latency_ms = duration(t1 - t0) 单位 ms
  ```
- **为什么在 copyWorker 中记录而不是在 Pylon 回调中**：Pylon 回调只负责推入队列，不保证帧一定被写入 ram_buffer。`instantTrigger()` 清空了队列，`copy_mtx` 锁保证清队和推入互斥，所以 `seq == 0` 的那一帧一定是触发后到达的第一帧
- **为什么用 steady_clock 而不用 Pylon 时间戳**：`t0` 是 `steady_clock::now()`，必须同一时钟域做差
- **存储**：不单独存储，由 `global_record_start_time`（已有）和 `first_frame_time`（3.1.3 新增）做差派生
- **跨相机差异原因**：硬件触发 < 1ms（仅线程调度差异）；软件触发可达 `1/fps`（各相机帧周期相位不同）

#### 3.1.5 dropped_frames — 实时丢帧计数（阶段 3，实时采集，事后输出）

- **定义**：本次录制期间累计检测到的丢帧数
- **采集位置**：`copyWorker()` 录制分支，每次写入帧时
- **计算方式**：
  ```
  若非首帧（prev_block_id != -1）：
    diff = 当前帧的 blockID - prev_block_id
    if diff > 1:
        dropped_frames += (diff - 1)
  更新 prev_block_id = 当前帧的 blockID
  ```
- **与现有阶段 5 日志解析方式（2.1.7）的对比**：
  | | 旧方式（2.1.7） | 新方式（3.1.5） |
  |---|---------------|----------------|
  | 数据来源 | 磁盘日志文件 | 内存中 copyWorker 实际处理的帧 |
  | 计算时机 | 录制完成后 | 录制期间实时累加 |
  | 理论一致性 | 应相等 | 应相等 |
  | 不一致的原因 | — | 落盘过程中某帧在内存中存在但日志漏写 |
- **存储**：`CameraContext::dropped_frames`（`atomic<int>`）
- **输出**：仅在结构化报告中打印最终值，不做实时 UI 预警
- **重置**：每次 `instantTrigger()` 调用时重置为 0

#### 3.1.6 timestamp_jitter — 时间戳抖动（阶段 5，事后）

- **定义**：相邻帧 Pylon 时间戳间隔的标准差，反映该相机帧到达的稳定性
- **计算方式**：
  ```
  第 1 步：计算所有相邻帧的时间戳间隔
    for i in 1..n-1:
        intervals[i-1] = meta_buffer[i].timestamp - meta_buffer[i-1].timestamp

  第 2 步：计算这些间隔的均值和标准差
    mean = Σ(intervals) / n
    variance = Σ((interval - mean)²) / n
    jitter = sqrt(variance)
  ```
  时间戳单位：Pylon 内部时间戳，1 单位 = 100ns（10⁻⁷秒）
- **解读**：低抖动（std < 几微秒）= 硬件触发稳定；高抖动（std > 100μs）= USB 带宽竞争、CPU 节流或驱动层中断合并
- **存储**：不存储，临时计算。需在录制完成后从 `meta_buffer` 遍历计算，不能从日志文件计算（日志文件不包含完整时间戳字段）

#### 3.1.7 per_camera_dump_time — 单相机落盘耗时（阶段 4）

- **定义**：该相机完成 `dumpToDiskWorker` 的墙上时间
- **计算方式**：
  ```
  函数入口记录 t_start = steady_clock::now()
  函数出口记录 t_end = steady_clock::now()
  duration = t_end - t_start
  ```
- **存储**：通过原子计数器或临时数组传回主线程
- **用途**：定位慢盘。若某相机落盘耗时是其他相机的 2 倍以上，说明其磁盘路径存在 I/O 瓶颈。同时也是全局落盘均衡度指标的原始数据来源

#### 3.1.8 recording_overflow_ms — 单相机 RAM 写入溢出延迟（阶段 3，实时采集，事后输出）

- **定义**：该相机的计时结束时刻（理论上应写完 RAM 的时间）与最后一帧实际写入 RAM 时刻之间的差值
- **采集位置**：
  - `t_first`：已在 3.1.3（`first_frame_time`）中记录
  - `t_last`：在 `copyWorker()` 录制分支，当 `seq + 1 == total_record_frames`（最后一帧写入 RAM）时记录
- **计算方式**：
  ```
  计时结束时刻 = t_first + total_record_frames / target_fps   （理论上应写完的时间点）
  实际完成时刻 = t_last                                         （真正写完最后一帧的时刻）
  recording_overflow_ms = t_last - 计时结束时刻                  // 单位 ms
  ```
- **解读**：
  - `≤ 0` → 正常。相机在计时结束前就写满了 RAM（帧率达标，无丢帧）
  - `> 0` → 该相机比理论慢。例如理论 1200ms 实际花了 1350ms，溢出 = +150ms。原因：录制期间发生了丢帧，每丢一帧就需要额外等一个帧周期（~5ms @ 200fps）来补足，累计导致超时
- **存储**：`CameraContext::recording_overflow_ms`（`double`）

#### 3.1.9 ram_to_disk_latency_ms — 单相机 RAM 到磁盘总延迟（阶段 3→4→5，事后输出）

- **定义**：从该相机最后一帧写入 RAM 到该相机数据全部写入硬盘的墙上时间
- **触发条件**：`write_jpg` 开启时才统计（关闭时落盘耗时极短，无统计意义）
- **采集位置**：
  - `t_ram_done`：`copyWorker()` 录制分支，最后一帧写入 RAM（`seq + 1 == total_record_frames`）时记录。与 3.1.8 的 `t_last` 是同一时刻
  - `t_disk_done`：该相机的 `dumpToDiskWorker` 结束时刻（仅 raw）；若 `write_jpg` 为 true，则为 `convertRawToJpgWorker` 结束时刻（raw + JPG）
- **计算方式**：
  ```
  ram_to_disk_latency_ms = t_disk_done - t_ram_done
  ```
- **注意**：若 `write_jpg` 为 false，报告中此列显示 `N/A`
- **存储**：`CameraContext::ram_to_disk_latency_ms`（`double`）

#### 3.2.11 max_ram_to_disk_latency — 最大 RAM 到磁盘延迟（阶段 4→5）

```
max_ram_to_disk_latency = max( ram_to_disk_latency_ms[i] )
```
- **含义**：所有相机中从 RAM 写入完成到硬盘写入完成最长的延迟。这个值就是用户按下 'r' 后，真正要等多久才能拿到全部数据
- **依赖**：每相机的 `ram_to_disk_latency_ms`（C16，新增）

### 3.2 全局新增指标（共 13 个，⭐ 为关键指标）

所有全局指标都通过对单相机指标做聚合计算得出，不直接采集原始数据。

#### 3.2.1 total_saved — 总保存帧数 ⭐（阶段 5）

这是**最重要的全局指标**，直接回答"本次录制成功了吗"。

```
total_saved = Σ recorded_frames[i]    // 所有相机的 C5 之和
total_expected = n_cams × total_record_frames
```
- **含义**：本次录制所有相机实际写入 ram_buffer 的总帧数。与预期值对比即知整体采集是否完整
- **依赖**：每相机的 `recorded_frames`（C5，已有）
- **报告输出**：Summary 区域第一行展示总数；同时每相机明细行展示各自的 saved 数，便于定位是全局性问题还是个别相机问题

#### 3.2.2 completion_rate — 整体完成率 ⭐（阶段 5）

```
completion_rate = total_saved / total_expected × 100%
```
- **含义**：将 total_saved 归一化为百分比。跨不同录制参数（帧率、时长、相机数）可横向对比
- **解读**：`> 99.5%` 优秀 / `99.0%~99.5%` 正常 / `95%~99%` 需关注 / `< 95%` 故障
- **依赖**：3.2.1（total_saved）

#### 3.2.3 total_dropped — 总丢帧数（阶段 5）

```
total_dropped = Σ dropped_frames[i]    // 所有相机的 C10（新增实时丢帧）之和
```
- **含义**：整个系统累计丢帧数。与 completion_rate 互补——completion_rate 给出整体百分比，total_dropped 给出绝对丢帧数
- **报告输出**：Summary 区域展示总数；**同时每相机明细行必须展示各自的 dropped 数**，因为只看总数无法定位问题——10 台相机丢 20 帧可能是 1 台丢了 20 帧（该相机问题），也可能是每台丢 2 帧（系统性问题）
- **依赖**：每相机的 `dropped_frames`（C10，新增）

#### 3.2.4 bottleneck_fps — 瓶颈相机帧率（阶段 5）

```
bottleneck_fps = min( actual_fps[i] )    // 所有相机中 actual_fps 的最小值
bottleneck_cam = argmin( actual_fps[i] ) // 最慢相机的编号
```
- **含义**：系统的有效帧率取决于最慢的那台相机。9 台 200fps + 1 台 150fps → 系统可用帧率 = 150fps，因为跨相机对齐时要裁剪到最小公共帧集
- **依赖**：每相机的 `actual_fps`（C11，已有，阶段 5 日志解析）

#### 3.2.5 max_sync_offset — 相机间最大同步偏移（阶段 5，仅硬件触发）

```
sync_offset[i] = first_recorded_block_id[i] - first_recorded_block_id[0]
max_sync_offset = max(sync_offset) - min(sync_offset)
```
- **含义**：以相机 0 为基准，所有相机首帧 BlockID 的最大跨度。硬件触发下理想值 = 0（所有相机 BlockID 严格一致）。非零 → 触发信号到达不一致或某相机漏收前几帧
- **适用条件**：仅硬件触发。软件触发下各相机独立运行，此指标无意义，报告中显示 `N/A`
- **依赖**：每相机的 `first_recorded_block_id`（C7，新增）

#### 3.2.6 max_trigger_latency_spread — 最大触发延迟跨度（阶段 5）

```
latency_spread = max(trigger_latency_ms[i]) - min(trigger_latency_ms[i])
```
- **含义**：最快响应和最慢响应相机的时间差。硬件触发 < 2ms，软件触发 ≤ `1/fps`
- **定位价值**：某相机延迟显著高于其他（如 10ms vs 0.5ms）→ 该相机存在 USB 线缆、驱动或调度问题。但由于实际对齐用的是 BlockID 严格逐帧匹配（而非时间戳对齐），延迟差异**不影响数据精度**，仅作为系统健康参考
- **依赖**：每相机的 `trigger_latency_ms`（C13，派生自 C9 + 全局触发时刻）

#### 3.2.7 peak_queue_pressure — 系统队列压力峰值（阶段 3）

```
peak_queue_pressure = max( max_queue_size[i] )
bottleneck_cam = argmax( max_queue_size[i] )
```
- **含义**：找出 copyWorker 最吃力的相机。某相机队列压力 > 5 而其他 ≤ 2 → 该相机所在 USB 控制器带宽不足
- **依赖**：每相机的 `max_queue_size`（C6，新增）

#### 3.2.8 max_jitter — 时间戳抖动峰值（阶段 5）

```
max_jitter = max( timestamp_jitter[i] )
worst_jitter_cam = argmax( timestamp_jitter[i] )   // 抖动最大的相机编号
```
- **含义**：帧到达最不稳定的相机及其抖动值。报告中须同时输出相机编号和抖动值，直接定位问题相机
- **依赖**：每相机的 `timestamp_jitter`（C12，新增）

#### 3.2.9 dump_bandwidth — 落盘带宽（阶段 4，周期采样取峰值）

落盘是并行的（每台相机一个线程），带宽随时间变化——启动时各线程同时开始写，带宽高；末尾只剩慢线程在写，带宽低。因此不能只看最终平均值。

**采集方式**：落盘期间用一个独立的采样线程，每隔 `50ms` 读取各相机的已写入字节数，计算瞬时带宽：

```
第 k 次采样：
  written_k = Σ (各相机此刻已写入的字节数)   // 从 dumpToDiskWorker 的进度变量读取
  instant_bw_k = (written_k - written_{k-1}) / 0.5s / (1024²)   // MB/s
  peak_bandwidth = max(instant_bw_k)        // 取所有采样周期的最大值
```

**输出**：
- `peak_bandwidth_mbps` — 峰值带宽，反映磁盘的最大写入能力
- `avg_bandwidth_mbps` — 平均带宽 = `total_bytes / dump_wall_clock_time / (1024²)`
- 两者对比：峰值接近标称速度 → 磁盘本身没问题；平均值显著低于峰值 → 落盘尾部有长尾线程拖累

- **依赖**：每相机的 `recorded_frames`（C5）+ 各 dumpToDiskWorker 实时写入进度
- **注意**：其中 `total_bytes = Σ (recorded_frames[i] × width × height × 1)` 是中间计算量，不独立成指标

#### 3.2.10 dump_balance — 落盘负载均衡度（阶段 4）

```
dump_balance = max(per_camera_dump_time[i]) / min(per_camera_dump_time[i])
```
- **含义**：各相机落盘耗时的均匀程度。比值接近 1.0 = 均衡；> 2.0 = 某磁盘路径存在瓶颈
- **依赖**：每相机的 `per_camera_dump_time`（C14，新增）

#### 3.2.12 max_recording_overflow — 最大 RAM 写入溢出（阶段 5）

```
max_overflow = max( recording_overflow_ms[i] )   // 溢出最多的相机
```
- **含义**：所有相机中，比理论计时超时最多的值。如果某相机溢出 +150ms，意味着整个录制实际上比理论多花了 150ms 才完成 RAM 写入
- **依赖**：每相机的 `recording_overflow_ms`（C15，新增）

#### 3.2.13 system_healthy — 系统健康综合判定（阶段 5）

```
system_healthy = (max_sync_offset ≤ 2)
              && (bottleneck_fps ≥ target_fps × 0.95)
              && (total_dropped ≤ total_expected × 0.01)
```
- **含义**：综合 3 个关键全局指标，给出系统是否健康的判定：
  1. 同步偏移 ≤ 2 BlockID — 硬件触发信号正常（BlockID 对齐的基础）
  2. 瓶颈帧率 ≥ 目标 95% — 帧率达标
  3. 总丢帧率 ≤ 1% — 数据完整性达标
- **三项全部满足**才判定为 `HEALTHY`，否则输出第一条不满足的原因
- **注意**：不包含 `max_trigger_latency_spread`，因为实际对齐用的是 BlockID 逐帧严格匹配，延迟差异不影响数据精度
- **依赖**：3.2.4、3.2.5、3.2.3

### 3.3 新增指标依赖关系

```
单相机新增（每台独立采集）               全局新增（汇总计算）
══════════════════════             ══════════════════════

recorded_frames (C5,已有) ────────► total_saved ⭐ (3.2.1)
                              └───► completion_rate ⭐ (3.2.2)
                              └───► dump_bandwidth (3.2.9)

dropped_frames (3.1.5) ──────────► total_dropped (3.2.3)
                                    system_healthy (3.2.13)

actual_fps (C11,已有) ───────────► bottleneck_fps (3.2.4)
                                    system_healthy (3.2.13)

first_recorded_block_id (3.1.2) ─► max_sync_offset (3.2.5)
                                    system_healthy (3.2.13)

first_frame_time (3.1.3) ────────► trigger_latency_ms (3.1.4)
+ global_record_start_time         max_trigger_latency_spread (3.2.6)

max_queue_size (3.1.1) ──────────► peak_queue_pressure (3.2.7)

timestamp_jitter (3.1.6) ────────► max_jitter (3.2.8)

per_camera_dump_time (3.1.7) ────► dump_balance (3.2.10)
                                    dump_bandwidth (3.2.9)

recording_overflow_ms (3.1.8) ───► max_recording_overflow (3.2.12)

ram_to_disk_latency_ms (3.1.9) ───► max_ram_to_disk_latency (3.2.11)
```

---

## 四、UI 实时叠加指标

队列压力和丢帧数均只做事后报告输出，不做实时 UI 预警。录制期间唯一的实时 UI 叠加是指标：

### 实时 FPS（阶段 3，放大视图底部）

- **数据来源**：`CameraContext::recorded_frames` + 录制已用时间 `elapsed_s`
- **计算方式**：`recorded_frames / elapsed_s`
- **显示条件**：`is_recording && recorded_frames > 10`（前 10 帧不显示，避免启动瞬态数值不稳定）
- **实现位置**：`renderEnlargedView()`，已用时间显示下方
- **更新频率**：同上

---

## 五、阶段与指标对应总表

> 单相机 vs 全局的分类详见[第二节](#二已有指标详解)（已有指标）和[第三节](#三新增指标详解)（新增指标）。下表仅做阶段索引。

```
                           │     单相机指标              │        全局指标
指标                  阶段 1  2  3  4  5  采集方式       │ 1  2  3  4  5
══════════════════════════════════════════════════════════╪═════════════════════
last_block_id                   ██ ██      实时·内存      │
last_frame_time                 ██ ██      实时·内存      │
has_streamed                    ██ ██      实时·内存      │
captured_frames                 ██ ██      实时·内存      │
recorded_frames                    ██      实时·内存      │
max_queue_size (新)              ██ ██      实时·内存      │
first_recorded_block_id (新)        ██      实时·内存      │
last_recorded_block_id (新)         ██      实时·内存      │
first_frame_time (新)               ██      实时·内存      │
dropped_frames (新·实时)            ██      实时·内存      │
trigger_latency_ms (新)             ██      派生·内存      │
per_camera_dump_time (新)                 ██   实时·内存    │
recording_overflow_ms (新)                ██   实时·内存    │
ram_to_disk_latency_ms (新·write_jpg时)         ██  ██  事后·内存  │
actual_fps                                  ██  事后·日志 │
timestamp_jitter (新)                       ██  事后·内存 │
──────────────────────────────────────────────────────────┼─────────────────────
total_saved ⭐ (新·全局)                                 │              ██
completion_rate ⭐ (新·全局)                              │              ██
total_dropped (新·全局)                                  │              ██
bottleneck_fps (新·全局)                                 │              ██
max_sync_offset (新·全局)                                │              ██
max_trigger_latency_spread (新·全局)                     │              ██
peak_queue_pressure (新·全局)                            │        ██
max_jitter (新·全局)                                     │              ██
dump_bandwidth 峰值+均值 (新·全局)                       │        ██ 实时采样
dump_balance (新·全局)                                   │           ██
max_recording_overflow (新·全局)                        │              ██
max_ram_to_disk_latency (新·全局·write_jpg时)           │              ██
system_healthy (新·全局)                                 │              ██
落盘总耗时 (已有·全局)                                   │           ██
──────────────────────────────────────────────────────────┼─────────────────────
UI 实时FPS                                UI·atomic读   │
结构化报告                                           文件│              ██
```


---

## 六、日志书写规范

所有指标输出必须写入文件，不依赖控制台打印。控制台仅保留启动信息、关键事件（录制开始/完成、故障）和致命错误。

### 6.1 日志目录

```
<项目根目录>/log/capture/
```

程序启动时自动创建（`std::filesystem::create_directories`）。不在 yaml 中配置，固定路径。

### 6.2 会话日志文件

**一次程序启动只创建一个日志文件**。程序启动时以启动时间戳命名，之后每次录制追加写入同一文件。多次录制之间用分隔线隔开。

| 项目 | 规范 |
|------|------|
| 文件名 | `session_<YYYYMMDD_HHMMSS>.txt`（**程序启动时刻**的时间戳） |
| 路径 | `log/capture/session_20260721_142801.txt` |
| 编码 | UTF-8（无 BOM） |
| 换行 | LF（`\n`） |
| 写入模式 | 启动时创建（或追加已有），每次录制追加写入 |

程序启动时控制台输出一行：

```
[Log] Session log: log/capture/session_20260721_142801.txt
```

### 6.3 日志文件格式

每次录制追加一个区块，以 `=== Recording #N:` 开头（N 从 1 开始递增）。每台相机标注 mono 或 color，不缩写。

```
=== Recording #1: 20260721_143052 ===
Config: 10 cams, HW trigger, 200.0 fps, 1.000s core + 0.050 margin, 240 total frames
write_jpg: true

--- Per-Camera Metrics ---
Cam,SN,Type,Saved,Dropped,FPS,QPeak,Lat_ms,FirstBlk,LastBlk,SyncOff,Jitter_us,RecOverflow_ms,RamToDisk_ms,DumpTime_ms
0,40768742,mono,240,0,200.1,2,0.8,1001,1240,0,12.5,-1.2,4520.3,4200.1
1,40774056,mono,240,1,199.8,3,1.2,1001,1240,0,15.3,3.5,4680.7,4350.2
...
9,40772283,color,238,0,198.3,2,0.9,1001,1238,-2,18.1,-0.8,N/A,3980.5
...

--- Dump Bandwidth Samples (50ms interval) ---
   0.0s     0.0 MB/s
   0.5s   612.3 MB/s
   1.0s   623.5 MB/s  <- peak
   1.5s   601.2 MB/s
   2.0s   587.4 MB/s
   2.5s   512.1 MB/s
   3.0s   445.3 MB/s
   3.5s   312.7 MB/s
   4.0s   156.2 MB/s
   4.2s    89.4 MB/s

--- Summary ---
total_expected       : 2400
total_saved          : 2398 (99.92%)
total_dropped        : 2
completion_rate      : 99.92%
bottleneck_fps       : 198.3 (cam 9: 40772283)
max_sync_offset      : 0 BlockID
max_trigger_latency_spread : 1.8 ms
peak_queue_pressure  : 3 (cam 1: 40774056)
max_jitter           : 15.3 us (cam 1: 40774056)
dump_bandwidth_peak  : 623.5 MB/s
dump_bandwidth_avg   : 557.1 MB/s
dump_balance         : 1.15
max_recording_overflow : 3.5 ms (cam 1: 40774056)
max_ram_to_disk_latency : 4680.7 ms (cam 1: 40774056)
system_healthy       : PASS
```

### 6.4 控制台输出原则

| 类型 | 输出到 | 示例 |
|------|--------|------|
| 会话日志路径 | 控制台 cout | `[Log] Session log: log/capture/session_20260721_142801.txt` |
| 启动信息 | 控制台 cout | `10 cameras, HW trigger, 240 frames.` |
| 录制开始 | 控制台 cout | `[Master] Broadcast START: 20260721_143052` |
| 录制完成 | 控制台 cout | `[Recording #1] Done in 4.2s (appended to session log)` |
| 故障告警 | 控制台 cerr | `[FAULT] Camera 40768742 stalled!` |
| 所有指标数据 | **仅文件** | 不输出到控制台 |

### 6.5 旧代码清理

以下现有的控制台输出行必须删除，改为写入会话日志文件：

- `main()` 中 `[Info] Auto-Stop Reached. Processing Disk Dump in background...`
- `main()` 中 `[Performance] Disk Dump Finished! Time: X.XXXs`
- `main()` 中 `[Info] Calculating frame drop...` 及随后的 `[OK]/[Warning]` 逐相机行
- `main()` 中 `[Info] Ready for next capture.`

录制完成后控制台仅输出：

```
[Recording #1] Done in 4.2s (appended to session log)
```

---

## 七、实施顺序

| 序号 | 步骤 | 涉及位置 | 风险 | 预计 |
|------|------|----------|------|------|
| 1 | **CameraContext 新增字段**（9 个）：`max_queue_size`、`first_recorded_block_id`、`last_recorded_block_id`、`first_frame_time`、`dropped_frames`、`prev_block_id`、`recording_end_time`、`ram_to_disk_latency_ms`、`recording_overflow_ms` | CameraContext 结构体 | 低 | 5min |
| 2 | **copyWorker() 植入采集逻辑**：队列压力追踪、首/末帧 BlockID、首帧时刻、丢帧检测、最后一帧时刻（t_last） | `copyWorker()` 录制分支 | 低 | 20min |
| 3 | **instantTrigger() 触发时间戳 + 指标重置**：清零所有 per-camera 指标 | `instantTrigger()` | 低 | 5min |
| 4 | **renderEnlargedView() 实时 FPS**：`recorded_frames / elapsed_s`，>=10 帧后显示 | `renderEnlargedView()` | 低 | 10min |
| 5 | **dumpToDiskWorker() / convertRawToJpgWorker() 单相机耗时**：入口出口 chrono 差值，写入 atomic 变量 | 两个 worker 函数 | 低 | 5min |
| 6 | **落盘带宽周期采样线程**：每 50ms 读取各线程写入进度，计算瞬时带宽，追踪峰值 | `main()` 落盘等待段 | 中 | 20min |
| 7 | **会话日志文件**：启动时创建 `log/capture/session_<ts>.txt`（追加模式），记录 `[Log] Session log: ...` | `main()` 启动段，+10 行 | 低 | 5min |
| 8 | **writeReport() 追加写入**：每次录制完成后按 6.3 格式追加写入会话日志文件（`=== Recording #N ===` 区块 + Per-Camera 表 + 带宽采样 + Summary） | `main()` 后处理段，+120 行 | 中 | 35min |
| 9 | **清理旧控制台输出**：删除旧 cout 行，录制完成改为输出 `[Recording #N] Done in X.Xs (appended to session log)` | `main()`，-35 行 | 低 | 10min |
| 10 | **编译检查**（零警告） | — | — | 5min |
| 11 | **运行时验证**：10 相机，HW 触发，`write_jpg=true`，连续 2 次录制，检查会话日志追加写入 | — | — | 15min |
| 9 | **编译检查**（零警告） | — | — | 5min |
| 10 | **运行时验证**：10 相机，HW 触发，`write_jpg=true`，1 次完整录制 | — | — | 15min |

**总预计：~2 小时 15 分钟**

---

## 八、验证清单

### 编译与基础功能
- [ ] 编译零警告
- [ ] UI 以 20fps 流畅渲染，无闪烁
- [ ] 原有功能无回退：键盘（r/空格/q）、故障检测、网络同步
- [ ] 程序启动时自动创建 `log/capture/` 目录
- [ ] 启动时控制台输出：`[Log] Session log: log/capture/session_<ts>.txt`

### 控制台输出
- [ ] 控制台**不出现**指标数据行（`[OK]`、`[Warning]`、saved/drop/FPS 统计数字）
- [ ] 每次录制完成后控制台输出：`[Recording #N] Done in X.Xs (appended to session log)`
- [ ] 故障告警仍正常输出到 cerr
- [ ] 启动信息、录制开始等关键事件正常输出

### 会话日志文件（log/capture/session_<ts>.txt）
- [ ] 编码 UTF-8 无 BOM，换行 LF
- [ ] 程序启动时创建，多次录制**追加写入同一文件**，不创建新文件
- [ ] 每次录制以 `=== Recording #N:` 开头，N 从 1 递增
- [ ] Per-Camera Metrics 表头含 `Type` 列，值为 `mono` 或 `color`（非缩写）
- [ ] 全部列完整：Cam,SN,Type,Saved,Dropped,FPS,QPeak,Lat_ms,FirstBlk,LastBlk,SyncOff,Jitter_us,RecOverflow_ms,RamToDisk_ms,DumpTime_ms
- [ ] `write_jpg=false` 时 RamToDisk_ms 显示 `N/A`
- [ ] `write_jpg=true` 时 RamToDisk_ms 有合理正值
- [ ] Dump Bandwidth Samples 区域：每 50ms 一行，含 `<- peak` 标记
- [ ] Summary 区域包含全部全局指标，值与原始数据一致

### 数据一致性
- [ ] `dropped_frames`（实时采集）与旧方式（parseLogFile 日志解析）数值一致
- [ ] 硬件触发下 `sync_offset` 全为 0
- [ ] `total_saved` = `total_record_frames * n_cams`（无丢帧时）
- [ ] 无丢帧场景 `recording_overflow_ms <= 0`
- [ ] 有丢帧场景 `recording_overflow_ms > 0`（溢出值 ~= 丢帧数 * 帧周期）

### 落盘带宽
- [ ] 报告中峰值 >= 均值
- [ ] 峰值带宽接近磁盘标称写入速度

### 全流程
- [ ] 录制 -> 落盘 -> JPG（若开启）-> 追加写入会话日志 -> 等待下次录制，完整循环正常
- [ ] 连续 2 次录制（按两次 'r'），会话日志中可见两个 `=== Recording #` 区块，N 递增
- [ ] 程序退出后会话日志文件仍保留在磁盘上

## 九、实施完成情况

> 记录时间：2026-07-28，从 `b14a2c0 Fix Summary Format` 起累计

### 已完成

| 内容 | 状态 |
|------|------|
| b14a2c0 全部功能（metrics、session log、writeReport、水印、同步键盘等） | ✅ |
| 网络同步下 Slave 键盘屏蔽（ESC/q/SPACE），Master ESC 发 SHUTDOWN 退出两台主机 | ✅ |
| UI 右下角操作提示水印 + 放大区域中心十字线 | ✅ |

### 待实施

| 内容 | 计划章节 |
|------|----------|
| max_num_buffer 从 yaml 动态加载 | — |
| 异常处理增强（E1-E12）+ 日志写入规范 | 十一 |

---


---

## 十、异常处理分析与增强

### 10.1 现有异常（按阶段，标注级别）

#### 阶段 1：初始化
| 异常 | 级别 | 处理 |
|------|------|------|
| WSAStartup 失败 | FATAL | `return 1` |
| save_dirs 数量不匹配 | FATAL | `return -1` |
| 相机 open() 失败 | ERROR | 设 status=ERROR_，线程返回 |
| 相机 start() 失败 | ERROR | 设 status=ERROR_，线程返回 |
| YAML 配置字段缺失 | WARN | try-catch 后用默认值 |
| Slave bind 失败 | ERROR | cerr 打印，线程返回 |

#### 阶段 2：待机
| 异常 | 级别 | 处理 |
|------|------|------|
| Pylon 帧抓取失败 | WARN | cerr 后跳过该帧 |
| 相机停流 1s 无帧 | ERROR | 触发故障 → 关所有相机 → 发 FAULT |
| 对端 FAULT 通知 | ERROR | 关所有相机 → 故障 UI |
| setFrameRate/setGain 等失败 | WARN | try-catch 静默吞掉 |
| 渲染帧为空 | INFO | 显示黑底+状态文字 |
| TCP connect 失败 | WARN | while 循环重试 500ms |

#### 阶段 3：录制
| 异常 | 级别 | 处理 |
|------|------|------|
| seq >= total_record_frames | WARN | if 守卫跳过 |
| copy_queue 满 | WARN | 静默丢弃（BlockID 跳变体现） |

#### 阶段 4：落盘
| 异常 | 级别 | 处理 |
|------|------|------|
| ofstream::write 失败 | ERROR | **无检查** |
| ofstream::open 失败 | ERROR | is_open() 为假时跳过 |
| create_directories 失败 | ERROR | **无检查** |

#### 阶段 5：后处理/清理
| 异常 | 级别 | 处理 |
|------|------|------|
| cv::imwrite 失败 | ERROR | **无检查** |
| BaslerCamera::close() 异常 | WARN | try-catch 静默吞掉 |
| 线程 join 失败 | WARN | joinable() 守卫 |
| g_session_log 未打开 | WARN | is_open() 守卫 |

### 10.2 新增异常（E1-E12，按阶段）

| # | 阶段 | 异常 | 级别 | 风险 |
|---|------|------|------|------|
| E1 | 1 | 所有相机 open() 失败 | ERROR | 中 |
| E2 | 1 | max_num_buffer ≤ 0 | WARN | 低 |
| E3 | 1 | log/capture/ 目录创建失败 | ERROR | 高 |
| E4 | 2 | FAULT 消息 fi 越界 | FATAL | 高 |
| E5 | 3 | CMD_START 消息越界 | WARN | 低 |
| E6 | 3 | open() 失败后 copyWorker 空转 | WARN | 低 |
| E7 | 3 | QueryPerformanceFrequency=0 | WARN | 低 |
| E8 | 4 | 磁盘满 ofstream::write 失败 | ERROR | 高 |
| E9 | 4 | create_directories 失败 | ERROR | 高 |
| E10 | 5 | 磁盘满 cv::imwrite 失败 | ERROR | 中 |
| E11 | 5 | recorded_frames=0 越界 | FATAL | 高 |
| E12 | 5 | total_expected=0 除零 | WARN | 低 |

### 10.3 按级别统计

| 级别 | 现有 | 新增 | 合计 | 说明 |
|------|------|------|------|------|
| **FATAL** | 2 | 2 | **4** | 程序无法继续（WSAStartup、配置错误、FAULT越界、meta_buffer越界） |
| **ERROR** | 7 | 5 | **12** | 功能受损但可继续（相机失败、磁盘满、目录创建失败） |
| **WARN** | 8 | 5 | **13** | 降级运行（配置缺失、丢帧、队列满、除零守卫） |
| **INFO** | 1 | 0 | **1** | 状态通知（渲染帧为空） |
| **合计** | **18** | **12** | **30** | |

### 10.4 运行时统计与报告

程序运行时维护 4 个全局计数器，每次异常发生时递增。录制结束后，在会话日志 Summary 表格末尾输出统计：

```
| exceptions_fatal | 0 | FATAL 异常计数（本次录制） |
| exceptions_error | 1 | ERROR 异常计数（本次录制） |
| exceptions_warn  | 3 | WARN 异常计数（本次录制） |
| exceptions_info  | 0 | INFO 异常计数（本次录制） |
```

每行 `> 0` 时加粗标记（`**1**`），方便快速定位问题。

---

## 十一、不做（留给后续阶段）

- 跨主机同步延迟测量（需协议扩展）
- 自动化质量门控（PASS/WARN/FAIL 阈值）
- RAM 预触（pre-touch）优化
- `copy_queue` 有界化
- 线程优先级调优
- Python 离线分析仪表盘
