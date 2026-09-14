# 传输提速计划 — 接收端 RAM 缓冲 + 单流顺序落盘

> 2026-09-13 · 链路: 采集机(master/slave, Win10) → 4090(Ubuntu) 100G DAC 直连
> 状态: **补测完成, 方案 A 定案 (盘侧实测 4.6 GB/s), 待用户批准实施**

## 1. 问题与排障结论 (全部实测)

现象: master 传输 1TB/15min (1.15 GB/s), 起始 4.0 渐降至 1.3; slave 同样衰减到 0.8。
用户目标 4 GB/s。

| 环节 | 实测 | 结论 |
|---|---|---|
| 网络 (iperf3 8流) | 36 Gbps = 4.5 GB/s | 健康 |
| 发送端程序 (send_slave 同机对照) | 3.4 GB/s 起步, 同样衰减 | 发送侧无罪 |
| 发送端盘 (2× 三星 9100 PRO 4TB) | 与 slave 同款 | 排除 |
| **4090 /data (md0, 2× Lexar ARES 8TB QLC)** | 传输中 md0 100% util、队列深 165、w_await 13ms; **rx_dropped 77万 (NIC 层仅 249) → TCP 重传 79万 → 拥塞坍缩** | **根因** |
| 温度/SMART | 58°C / 全零 | 排除热与故障 |
| **fio 单流顺序直写 1TB** | **平均 2.83 GB/s** (min 1.1 / max 6.9) | 单流顺序是 QLC 最优写模式 |

根因: **QLC 盘并发交织写惩罚**——4 连接各自直写 4 个文件, 交织下 QLC 掉速至
0.8-1.3 GB/s; 接收应用写不动 → socket 缓冲满 → 丢包 → 重传 → 拥塞窗口坍缩
(两台采集机的衰减曲线即此)。

## 2. 方案 (用户提出, 已设计未实施)

```
网络(4.5) → RAM 文件缓冲(4并发在途, 总额度上限) → 单刷盘线程顺序落盘(盘侧上限)
```

改动仅 `preprocess-server/transfer/baseline_recv_data.cpp` (协议零改动, 发送端零改动):

1. **RamFile 结构 + 就绪队列**: 每文件一块 `unique_ptr<char[]>`; 全局额度
   `g_ram_bytes` 上限默认 80GB (`--ram-gb` 可调, 机器 251GB RAM)
2. **网络线程** (handle_conn 改): GO 后先等 RAM 额度 (背压: 满 → 停收 → TCP 窗口
   收缩 → 发送端自动减速) → `recv_to_ram` 零拷贝收满 → 入队 → **立即回 OK**
   (worker 立刻发下一文件, 管线不因落盘等待堵塞 — 实测"等落盘回 OK"会把
   8 worker 压到等效 2 流)
3. **BYE = drain 屏障**: BYE 时接收端等 `g_ram_bytes == 0` (全部落盘) 再应答,
   发送端收到 BYE 应答退出即安全; 崩溃丢 RAM/未完成文件由发送端按大小校验重传自愈
3. **flush_thread** (新增单线程): pop → `.part` + fallocate → 8MB 块顺序单流写
   → rename → 释放额度 → 唤醒网络线程; 不 fsync (与现版一致)
4. status_line 加 RAM 占用; 日志分 net/disk 两行
5. Ctrl+C 直接退出 (RAM 数据靠重传自愈, 不做优雅排空)

## 3. 补测结果 (2026-09-13, 已完成 → 定案)

3TB 长跑 (远超任何 SLC 缓存), 每 5s 时间序列, 尾端累计即稳态:

| 测试 | 模式 | 3TB 全程均值 | 末端稳态 | 备注 |
|---|---|---|---|---|
| A (w2) | O_DIRECT, QD32 单流 | 3.62 GB/s | **3.6, 无衰减** | 盘物理上限参考 |
| **B (w3)** | **buffered + sysctl 调优*** | **4.39 GB/s** | **4.6, 无衰减** | **采用路径; 反超 direct** |

\* 调优参数 (实施时持久化到 /etc/sysctl.d/):
```bash
vm.dirty_background_bytes = 8589934592   # 8G: 触发后台回写
vm.dirty_bytes = 34359738368             # 32G: 硬上限, write() 在此节流
```

结论:
- **方案 A (RAM 缓冲 + buffered 单流刷盘) 盘侧实测 4.6 GB/s**, 超过 4 GB/s 目标
- B 的 sys 81.5% (单核 write 拷贝) 是新边际瓶颈, 封顶 ~4.6-5, 与网络 4.5 自然平衡,
  无需进一步优化
- **全链预期 ~4-4.5 GB/s, 5TB/机 ≈ 20 分钟** (原 1.15 GB/s → 3.5-4 倍)

小批实测 (2026-09-13, 单相机 250GB / 双盘各一台 500GB):
- 盘侧 (flush): 稳定 3-4 GB/s ✓ 达标; RAM 背压 37-75GB 波动正常; 零错误
- 聚合 2.6 GB/s: 瓶颈移至**发送侧单流 TCP 有效吞吐 ~0.7-1 GB/s**
  (含每文件连接建立/GO 往返间隙稀释) → 下一步 workers 4→8 (分桶后每盘 4 流,
  9100PRO 单盘 4 流读 8.14 GB/s 无压力)

测试命令存档 (复现用):
```bash
sudo fio --name=w2 --filename=/data/f2 --size=3000G --bs=1M --direct=1 \
  --rw=write --iodepth=32 --numjobs=1 --group_reporting --status-interval=5
sudo sysctl vm.dirty_background_bytes=8589934592 vm.dirty_bytes=34359738368
sudo fio --name=w3 --filename=/data/f3 --size=3000G --bs=1M --rw=write \
  --iodepth=32 --numjobs=1 --group_reporting --status-interval=5
# 每条跑完 rm 测试文件 + fstrim + 静置数分钟再跑下一条
```

## 4. 实施步骤 (补测确认 + 用户批准后)

1. 改 `baseline_recv_data.cpp` (按第 2 节设计)
2. 4090 编译: `g++ -O3 -std=c++17 -pthread transfer/baseline_recv_data.cpp -o transfer/build/baseline_recv_data`
3. 小批验证: slave 跑 `send_slave.exe --data-ip 10.10.2.1 --cams 40768742` (250GB)
   - status_line 稳态 GB/s (预期≈补测稳态)
   - `nstat -az | grep -i retrans` 两遍增量 (预期≈0)
   - `free -g` 确认 RSS 受控 (≤ limit + 少量)
4. 全量 2.4TB, 核对文件数/字节数与发送端汇总一致
5. 回滚预案: preprocess-server 仓库 `git checkout -- transfer/baseline_recv_data.cpp`
   (现版本已回滚至 HEAD, 生产零改动)

## 5. 待批: 细粒度额度记账 (根治网络批振荡)

现状: 额度按**整文件**预登记 (开收前占住 10GB) → RAM 只能装 ⌊limit/10⌋ 个文件 →
满后网络全停等 flush 清位 → **批式放行** (实测 RECV 成组出现, 组间 10-20s 空窗;
ram 贴 limit-10 恒定是其特征)。加大 --ram-gb 只能摊薄空窗占比, 不能消除。

改法 (接收端, ~20 行):
- 额度改为按**已收字节**动态记账: `recv_to_ram` 每收 n 字节 `g_ram_bytes += n`;
  flush 完成后 `-= size`; 额度等待谓词 `g_ram_bytes + CHUNK < limit`
  (每次放行一个 CHUNK 而不是一个文件 → 网络连续供给, 队列深度自然稳定)
- 预期: 生产者恒速化 → flush 持续有货 → 盘侧进入 fio-B 式持续写 (4+ GB/s),
  与 fio 3TB 稳态对齐
- 前提: 干净基线测试 (fstrim+静置) 先确认盘侧当前真实上限, 避免把折叠债
  误判为架构问题

## 5.5 根因修正: 额度释放粒度 (2026-09-13 定位)

阶段计时仪表 (open/write 分布/close/rename + Dirty 水位) 定位:
- open/close/rename 全程 <0.2ms — 元数据/extent/fallocate 全部排除
- 慢文件 p50 8-20ms 正常, 但 p90 50ms+, max 295-547ms, 数十个 >100ms 慢块随机分布
  = **QLC 固件折叠停顿** (fio 同样抖动 min=208 max=5778, 但无放大器时平均仍 4.4)
- Dirty 稳在 8G (background 线) — 内核节流正常
- **真正的 bug: 额度登记 512MB 细粒度, 但释放是整文件粒度** — 慢文件写盘期间
  已写入页缓存的 ~10GB 仍占用额度 → 网络停摆 (实测 net 跌至 80MB/s)
  → 占空比坍塌, 聚合只有 fio 的 1/3

修复: flush 写盘循环每累计 RAM_WINDOW (512MB) 即释放额度 + notify,
与登记侧同粒度 — QLC 抖动不再放大, 网络持续供给。
同时删除 posix_fallocate (XFS unwritten extent 转换惩罚, 历史实测 2.45 vs 3.8+)。

## 6. 测试记录 (2026-09-13)

| # | 配置 | 聚合 | 备注 |
|---|---|---|---|
| 1 | 4 流单相机(单盘), ok=落盘后回 | 2.6 | 等落盘 OK 把管线堵住 (8→2 等效流) |
| 2 | 8 流双盘, ok=落盘后回 | 1.7 | 同上更糟; 盘侧全程 3-4 健康 |
| 3 | 8 流双盘, ok=入RAM即回, ram 80 | 2.1 | ram 贴 65.4: 第 8 文件批不进; 批振荡 |
| 4 | 8 流双盘 + BYE 屏障, ram 80 | 1.9 | 批振荡 + 折叠债; BYE drain 验证 ✓ |
| 5 | 同 4, ram 120 | 1.9-2.4 | 批更大仍有空窗; disk 低谷 1.1-1.3 (连测污染) |
| 6 | 细粒度记账, ram 120 | 2.2-2.8 | ram 平滑化 ✓ 空窗缩短 ✓; 残余=盘折叠债+worker 同步 |
| 7 | 全量 8 流 (1TB 后) | 崩到 0.3-0.6 | disk 行 296/204/382 MB/s; ram 贴 119 (盘成节拍器) |

#7 排查: 与 fio-B (4.6 持续) 的代码级差异 = **posix_fallocate** —
fio 单文件直写无预分配; 接收端每文件 fallocate(10GB) → XFS unwritten extent
逐块状态转换 (历史先例: 含 fallocate 的 O_DIRECT 2.45 vs 3.8+)。
RAM 缓冲架构下预分配无必要 (整文件缓冲+rename 原子+BYE drain) → 已删除, 待重测。

## 7. 附: 本轮已落地的发送端修复 (与接收端方案独立, 均已提交)

- send_slave 扫描 ×5 重复入队 bug (capture.yaml root↔cam 一一配对语义)
- send_slave / send_ui worker 按数据盘分桶绑定 (并发读交错惩罚缓解)

## 8. v3.1: Blob + Reflink + fid 分组 (2026-09-13, 基于用户指定回滚 7193f54)

v3 架构 (热缓冲池 128×1MB → 全局块队列 → 单 flush 线程顺序写 blob →
BYE 后 FICLONERANGE 切分) 实测 12 worker 3.5 GB/s, 但 reflink 失败:
**多连接块在队列中交错, flush 按到达序写 blob → 文件边界混入他文件数据**。

### 8.1 文件边界修复 (fid 分组)

- `Chunk` 增加 `file_id`; handle_conn 在 GO 后登记 `g_file_meta[fid]=(rel,size)`
- flush: 当前文件的块直接写; **其他文件的块缓存 `pending[fid]`**;
  当前文件收到 last 块后, 把 pending 首文件的块推回队头续写
- 发送端 worker 串行发文件 (收 OK 才发下一个) → 同连接至多 1 个在途文件
  → pending 文件数 ≤ 并发连接数, 缓冲占用受池上限约束

### 8.2 可靠性修复 (与边界修复配套, 缺一即死锁/挂死)

| 修复 | 问题 | 机制 |
|------|------|------|
| 缓冲池优先槽 | 池耗尽时当前文件的线程抢不到缓冲 → last 块到不了 → 全局死锁 | `pool_release` 首个缓冲进优先槽, 仅 `g_flush_fid` 匹配者可取 |
| abandoned 集合 | short read 文件的残块卡住 flush (永远等不到 last) | `abandon_file()`: 标记+删 meta+记账; flush 丢弃残块 (队头/pending/当前三处) |
| g_open_files 屏障 | BYE 只等 g_pending==0 会漏最后一个 entry 入表竞态 | BYE 等 `g_pending==0 && g_open_files==0` |
| pending 推回过滤 | 被放弃文件是 pending 首个时, 推回后块全被丢, 剩余 pending 永不排空 → BYE 挂死 | `take_next_pending` 循环: abandoned 就地丢弃, 直到推回一个活文件或清空 |
| blob 重开 | finalize 后第二台机器传输 → fd=-1 写失败 | 文件启动时 `g_blob_fd<0` 则 `blob_open()` 新 blob; finalize 清空 entries |
| 错误计数 | short read 被 handle_conn 和 flush 各计一次 | `++g_errs` 唯一入口在 `abandon_file` (幂等) |
| 流式超时 | recv_line 的 60s SO_RCVTIMEO 残留到数据阶段 → 背压时误判 short read | GO 后 `SO_RCVTIMEO=0` |
| 空文件 | size=0 无块 → 永远无 last → g_open_files 卡死 | 直接落空文件, 不登记 fid |

### 8.3 测试要点

1. 单相机 250 文件: BLOB 行文件名正确、`finalize 250/250` reflink 成功
2. 8-12 worker: 聚合 ≥3 GB/s (对照 v3 同配置 3.5, 修复不应降速)
3. 人为中断 (传输中 Ctrl+C slave): ERR abandoned 日志出现, slave 重启后续传, BYE 正常
4. 两台机器先后传: 第二批 reflink 正常 (blob 重开验证)

## 9. v3.2: staging 分段调度 (2026-09-13, v3.1 实测定位)

### 9.1 v3.1 实测与根因

2 worker 实测: 聚合 ~1.0 GB/s (disk=1.0-1.1, 每文件 net 460-554 MB/s ×2);
**>2 worker 发送端"发不出去"** (超时断连)。累计均值 2.9 是历史包袱, 瞬时 1.0 才是真相。

根因: v3.1 要求文件在 blob 内**单段连续** → flush 一次只写"当前文件" →
写盘管线深度=1 (写 1MB → 释放 1 缓冲 → 该 worker 醒来收 1MB → 入队 → 再写):
- 单文件串行流 ~500 MB/s (往返延迟限制, 非盘速)
- 其他 worker 的块进 pending, 而 pending 只在 worker 拿到缓冲后才能增长,
  缓冲释放又只来自当前文件的写入 (500 MB/s) → 12 worker 时每个 ~45 MB/s
  → 发送端 send 长期阻塞 → 超时断连
- v3 为什么快: 到达序写所有文件的块 (12 路交织, 队列深, 盘永远有货) = 3.5 GB/s

### 9.2 v3.2 设计: 放弃单段连续, 改多段

关键: FICLONERANGE 可对同一目标文件调用多次 → 文件在 blob 内可为**多段**
(每段一次 clone, dest 偏移 = 文件内逻辑偏移; 段长/偏移 4096 对齐)。

- flush = staging 调度: 整批取队列 → 按 fid 分桶暂存 → 写某桶当且仅当:
  桶 ≥ 32MB (RUN_MIN) / 文件剩余已收齐 / staging 总量 ≥ 768MB (STAGE_HIGH,
  池将耗尽必须释放) / 队列干了 100ms (timed_out)
- 写整桶 = blob 尾部顺序追加一大段 → 盘看到密集顺序大段 (v3 同款写形态)
  且不依赖任何单一文件的网络供给 → 无饿死路径 → 优先槽机制删除
- 段记录: 同文件相邻段 blob 内连续则合并; 完成 (last 块+字节吻合) → 多段 entry
- finalize: 逐文件逐段 FICLONERANGE (dest ftruncate 到最大段尾, 全部 clone
  后截回真实大小); 段数 ≈ 文件大小/(池/并发文件数) ≈ 10GB/(1GB/12) ≈ 120/文件
- 池扩到 1024×1MB = 1GB (段更长, 段数更少; 251GB RAM 无压力; 冷缓存写 3.77 GB/s 仍高于盘)

预期: 12 worker 聚合回到 ~3-3.5 GB/s (网络 4.5 / 盘 3.3-3.7 之间);
finalize 段 clone 增加数秒。可靠性机制 (abandoned/BYE 屏障/多批次重开/空文件) 全部保留。

## 10. 收官 (2026-09-14)

| 项 | 结果 |
|---|---|
| v3.2 全量实测 | 单批 ~8-11 min (vs v1 15 min/TB = 1.15 GB/s) |
| worker 扫描 | **8 流最优**, 比 12 流快 ~2 min (12 流 staging 分桶更碎 + 发送端读交错) |
| 段粒度 | ~55 segs / 10GB 文件 (均值 ~187MB/段, clone 开销可忽略) |
| 观测说明 | BLOB 行偶发 disk 0.0 GB/s (连续数个) = 页缓存回写空窗, 正常: disk= 统计的是 md0 介质回写, write() 只进脏页; finalize 的 syncfs 保证 reflink 前数据已落盘 |
| 默认值 | transfer.yaml workers: 8 |
| 累计提升 | 1.15 → ~3.7 GB/s (3.2×) |

调试方法论沉淀 (用户总结): 瓶颈定位靠分环节仪表而非猜测 (stage 计时/diskstats/每文件 net 率);
目标机最小复现脚本隔离昂贵路径, 一次一变量快速对比; 看瞬时值不看累计均值。

## 11. send_ui 交互升级 (2026-09-14)

状态机新增 PAUSED; 握手协议命令化 (START/ABORT/QUIT + 掉线):

- **SPACE 前退出**: master CONFIG 屏 q/ESC → QUIT → slave 命令循环退出;
  slave 连接 master 加 ~2min 上限 (master 未启/已退出不再无限重试)
- **z 中断 / SPACE 续传**: master 任意传输阶段 (RUNNING/WAIT_SLAVE) 按 z →
  文件粒度温和停止 (master 本机 g_abort / slave 由 ABORT 命令触发) →
  双方进 PAUSED; SPACE 重跑全程会话 (计数重置, 接收端 SKIP 秒过已传文件) →
  自然续传。slave 由一次性 START 改为命令循环, 支持多次 START
- q (传输中) 保持原语义: 温和中止为终态 ABORTED (与 z 的可恢复中断区分)
- 注意: 握手协议变化需两台主机同时升级二进制

## 12. 并行传输模式 (2026-09-14)

串行模式的实测问题: master 2.4TB 传完紧接 slave, SLC 缓存耗尽 + 后台折叠债
竞争 → slave 段 2 GB/s 跌至 0.2 GB/s。fstrim 无删除可回收 (无效操作),
静置需 20-40min (折叠债时长)。

**方案: 两机并行**。v3.2 接收端单 blob 单写流, 连接数不影响盘写形态 →
两机 16 流对 QLC 与单机 8 流等价; 全量一次连续写完, 只经历一个 SLC 周期。
预期 4.8TB / ~3.7 GB/s ≈ 22min 全程满速。

- send_ui: SPACE 即发 "START <gen>" 令 slave 同步开传 (旧串行: 本机完成后才发);
  master 本机完成后提前切 WAIT_SLAVE (BYE 在接收端等两机全部数据落盘, 会阻塞),
  只收**同号** SLAVE_DONE (z 中断遗留旧应答按号丢弃); slave 命令循环解析
  "START <gen>" 回 "SLAVE_DONE <gen> ok=.. skip=.. fail=..", 取消 ACK 等待
- z 中断 (RUNNING): UI 线程直接发 ABORT 令 slave 同停 (不等 controller,
  否则 master 的 BYE 会挂到 slave 传完)
- 接收端: blob_finalize 加互斥锁 (双机 BYE 并发触发; 第二个 no-op);
  文件不会跨 blob (finalize 仅在 g_pending==0 && g_open_files==0 时执行,
  即无任何在途文件), 双 blob 各自 reflink, 数据零丢失
- 备份: send_ui_serial_backup.cpp (串行版)

## 13. 后续: 数据流水线定稿 (2026-09-14)

4.8TB 并行实测 53min 确认 QLC SLC 缓存上限 (~每盘 1.2TB) 后, 流程演进为
**两机并发 + slave 直写硬盘柜 + NVMe 仅中转 master** 的流水线 (总 ~40min,
NVMe 永远缓存内满速)。完整推演与部署见 `data_pipeline.md`。
