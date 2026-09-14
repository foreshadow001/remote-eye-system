# slave 链路直写模式 — baseline_recv_data --direct 设计计划

2026-09-14。slave 数据不经 NVMe 直写 ZFS 硬盘柜（`data_pipeline.md` §1）。
本文档为实施前计划（未写代码）。

## 1. 目标与实测基线（archive/raid36-hc580-research.md §5）

| 项 | 值 | 来源 |
|---|---|---|
| 池 | tank `draid2:9d:2s:35c`, recordsize=1M, lz4, atime=off | build-pool.sh |
| 挂载 | `/archive` (tank/data), 非 root 可写 (yanglinxuan) | build-pool.sh |
| **8 路直写 (dd)** | **2.61 GB/s** | 与本场景最接近的实测 |
| 16 路 rsync 持续段 | 2.90 GB/s | 用户所说"上限 3 GB/s"的出处 |
| 冷读 | 1.7 GB/s | async_read_max 已调 32 |
| ZFS 脏数据缓冲 | **zfs_dirty_data_max=32GiB** (已持久化调优) | zfs-tuning.conf |

目标: 8 worker (transfer.yaml 现值) 全量 ~2.4TB 稳态 **≈2.6-2.9 GB/s**,
背压自动限速, 无人工干预。2.4TB 预期 ~15min。

## 2. 核心决策: 缓冲池 + write() 直写, 不用 splice

**关键洞察: "大 RAM 缓存"已经存在——ZFS 自己的 32GiB dirty 层**。
write() 落 ARC 脏页 µs 级返回 → 连接跑满网络速; 32GiB 攒满后 DMU 自动
节流 write → 用户态池满 → recv 停 → TCP 窗口收缩 → 发送端减速。
**这就是"动态网络发送逻辑", 发送端零改动**——与 v3 的背压机制同构,
只是缓存放进了 ZFS（比用户态缓存更优: 零额外拷贝、自动节流、崩溃一致性由 TXG 保证）。

对比 splice（曾实测, transfer/*.c）:

| 维度 | splice (sock→pipe→file) | 缓冲池 + write() |
|---|---|---|
| 用户态路径上限 | 3.06 GB/s 单线程, 多线程无扩展 (3.2) | 5.31 GB/s (热缓存) |
| 相对盘速余量 | 3.06 vs 2.9 ≈ 0, 顶死 | 5.31 vs 2.9 ≈ 80% |
| ZFS 兼容性 | splice_write 对 ZFS write_iter 的 bvec 路径**未验证**, 可能 fallback 到拷贝 | 通用路径, 生产已验证 |
| 背压粒度 | pipe 容量 64KB-1MB, 太细 | 用户态池 + 32GiB ZFS dirty 双层 |
| CPU (双拷贝) | 省 ~6 GB/s 内存带宽 | 144 线程机器 <2%, 无意义 |
| 代码成熟度 | 从未生产使用 | v3 路径在 NVMe 上 250/250 |

**结论: 缓冲池 + write(), splice 无收益有风险。**

## 3. 架构 (baseline_recv_data.cpp 加 --direct, 单 binary 双模式)

```
handle_conn (direct):  FILE 头 → SKIP 判重(不变) → GO →
  创建 <dest>.part → 循环 { pool_acquire 1MB → recv → write(.part) → release }
  → 收满 rename 真名 → 回 OK → ++g_files / g_bytes
  short read → unlink(.part) + ERR (无 fid/staging/abandon 概念)
BYE (direct):  等 g_open_files==0 → syncfs(/archive 上的 fd) 刷 TXG
  (最坏 32GiB ÷ 2.9 GB/s ≈ 11s) → 回 ack
```

- 复用: 头解析/路径安全/SKIP/热缓冲池/status_line/BYE 框架 — 全部不动
- 不启动 flush_thread/队列 (direct 分支跳过), blob 代码零改动 (默认模式不受影响)
- OK 语义 = write 完成 (数据在 ZFS 脏页); 落盘由 BYE 的 syncfs 保证 —
  崩溃一致性: .part + rename 均为 ZFS 元数据事务, 全有或全无
- 用户态池保持 1024×1MB 共享常量: direct 模式每连接串行在途 1-2 块,
  8 连接仅占 ~16 块 — 池几乎不满, 背压实际由 ZFS dirty 层主导 (设计意图)

## 4. 改动清单 (~90 行)

1. `main`: 解析 `--direct` → `g_direct` 全局; direct 时不启动 flush 线程
2. `handle_conn`: GO 后按 g_direct 走直写分支 (新函数 `direct_recv_file`)
3. BYE 处理: g_direct → g_open_files 屏障 + syncfs; 原 blob 分支不变
4. 文档头注释与 banner 更新

## 5. 部署与验收

```bash
# 4090 (slave 链路接收端 B)
g++ -O3 -std=c++17 -pthread transfer/baseline_recv_data.cpp -o transfer/build/baseline_recv_data
./transfer/build/baseline_recv_data --bind 10.10.2.1 --port 5001 --out /archive/dataset --direct
```

slave 照常 `send_ui`/`send_slave`（server_ip_slave_link 不变, workers=8）。

验收（对照 §1 基线）:
1. 稳态速率 2.6-2.9 GB/s, 全程无下坠（ZFS 无 QLC 折叠, 应为平线）
2. 2.4TB ≈ 15min; BYE ack 延迟 ≤ ~12s (syncfs 刷脏)
3. 对账: 每相机 25 个 h5 × 相机数, `du -sb` 逻辑字节与发送端一致
   （注意 `du -sh` 显示的是 lz4 压缩后实占 ~1.3TB, 口径不同）
4. 断连重传: 传输中 Ctrl+C slave → 重启 → SKIP 续传, .part 被覆写
5. `zpool status tank` 无错误计数增长

## 6. 风险与备注

| 项 | 说明 |
|---|---|
| 每盘 116KB 碎写 | recordsize=1M / 9+2 条带所致 (research §5 已定位); 若日后要 >3 GB/s, 建 `tank/slave -o recordsize=8M` 独立数据集 — 优化杠杆, 不阻塞本轮 |
| rsync 尾段衰减 | 是 rsync 按目录分片的缺陷, 直写按文件轮转天然无此问题 |
| 池与 master 链路共存 | 两接收端进程各自独立池, 互不影响; ZFS dirty 32GiB 仅 B 进程的写入使用 |
| scrub 窗口 | 每月手动, 避开传输时段 (README 既有约定) |
| master 链路 | 完全独立 (blob 模式), 本计划不动它 |

## 7. 实施记录 (2026-09-14, 已完成)

- `baseline_recv_data.cpp` v3.3: `--direct` 开关 / `direct_recv_file()` /
  在途集合 g_active (ERR busy 防并发写 .part) / BYE syncfs 屏障 /
  disk 速率统计 direct 下聚合 sd* 整盘 (物理口径, lz4 会使其低于逻辑值) /
  direct 不启动 flush 线程; blob 模式零改动
- 配套 (计划外, 必要): send_ui + send_slave 的 `REPLY_TIMEOUT_MS`
  60s → 180s — direct 的 OK 等整文件写完 (~10GB@0.3GB/s≈33s), 60s 超时
  会触发重传并撞上在途保护; 180s 给 5 倍余量。**slave 端需重编发送器**

## 8. 正确性与性能收官 (2026-09-14 晚)

**重大根因修正**: 11:10 批次的帧偏移损坏 **不是 blob 接收链路** — 是发送端
**TransmitFile 路径本身产生数据损坏** (两次独立复现: v3.3+TransmitFile 11:10 批次、
v4 修复版+transmitfile 3.2GB/s 图片错乱; 而 user 路径 + CRC 两次 md5 全对)。
TransmitFile 内核态读发用户态无法校验 → **弃用**, 生产 = user 路径 + 块 CRC。
(blob 被冤枉, 但退役决定不变。)

**调试链** (方法论再次生效: 5b014939 版本对比 + crc_bench.c 隔离测量):
1. v4 重写时 bcrc.add 无条件进热路径 → transmitfile 0.5 (修: 条件化)
2. 查表 CRC32 串行依赖链 0.43GB/s/核 → user+CRC 1.4 (crc_bench: A=0.43/B=6.99/C=0.37/D=4.27)
3. 两端换 SSE4.2 CRC32C (函数级 target 属性免编译开关) → **3.1 GB/s 单相机**
   (= TransmitFile 时代单盘读上限, CRC 开销趋零)

**最终架构**: slave/master 串行 → direct 直写 /archive + 64MB 块 CRC32C 硬件校验
+ trailer 重传自愈。单相机 3.1 (单盘读上限), 全量预期 3.3-3.5。

## 9. 定稿 (2026-09-14 晚, 用户拍板)

**生产版本 = 当前 CRC32C 版**: 单相机 3.1 GB/s + 图片正确。
单相机 3.1 = 单盘 8 流读上限 (全量多盘历史 3.5, 接收端上限 4.27/连接);
读侧优化 (CreateFile 直读/32MB 块/深管线) 经评估收益有限, 不做。
备份: send_slave_crc32c_3.1_backup.cpp。

**两端必须同版本**: CRC32C 算法两端绑定 (单边更新会 ERR checksum)。
master 采集机需拉新代码重编后才可传输。
