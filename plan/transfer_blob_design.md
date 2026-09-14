# 传输落盘 v2 — Blob 顺序写 + Reflink 切分（完全复现 fio 模式）

> 2026-09-13 · 目标: 消除多文件回写交错导致的 QLC 写降速, 完全复现 fio 单文件
> 连续写 4.4 GB/s 的行为。

## 1. 问题定位（阶段计时仪表实测）

| 阶段 | 耗时 | 结论 |
|---|---|---|
| open / close / rename | <0.2ms | 排除 |
| fallocate | 已删除 | 排除 |
| write p50 | 8-20ms | 页缓存吸收, 本身不慢 |
| **write p90 / max** | **50ms / 550ms, 随机分布** | **QLC 固件停顿 + 回写交错放大** |
| Dirty 水位 | 稳在 8G | 内核节流正常 |

与 fio 的核心差异: **页缓存中多文件脏页共存, 回写下发的 IO 在不同文件的
extent 间跳变**。fio 单文件连续 append, 脏页永远是同一段连续 LBA,
回写永远大块连续 → QLC FTL 顺序写最优。

RAM 缓冲已就位 (网络不停摆), 但盘的写速率因回写跳变降至 0.3-1 GB/s。

## 2. 方案: Blob + Reflink 切分

### 2.1 传输阶段（复现 fio）

```
网络 → RAM (整文件缓冲, 额度背压, 细粒度窗口)
     → flush_thread 顺序 append 到单一 blob 文件
       /data/incoming/<participant>.blob
     → 记录每个 h5 在 blob 中的 [offset, size)
```

- blob 是**唯一**被写入的文件 → 脏页永远是 blob 的一段连续 LBA → 回写连续
- write 模式与 fio `--rw=write --bs=8M` 完全一致
- 文件位置映射在内存中: `vector<BlobEntry{rel, offset, size}>`

### 2.2 切分阶段（BYE 后, Reflink 元数据操作）

```c
// 对每个 h5:
int dst = open(rel, O_CREAT|O_WRONLY, 0644);
fallocate(dst, 0, size);                    // 预分配目标文件空间
ioctl(dst, FICLONERANGE, &range);           // 从 blob [offset, size) 克隆
close(dst); rename(.part → 最终名);
// 最后: unlink(blob)
```

- `FICLONERANGE` = XFS reflink（块级 COW 引用, **不拷贝数据**, 每文件 <10ms）
- blob 与目标文件共享数据块, 磁盘空间不 double
- unlink(blob) 后块归属目标文件
- 250 个文件的切分总计 <3 秒

### 2.3 协议变化

- FILE/GO/SKIP/OK/BYE 不变, 发送端零改动
- **BYE 语义增强**: 等 blob 全部写完 + reflink 切分完成后才应答
- SKIP 检查：仍查最终文件（切分后的）；传输中间查 blob 的 offset 记录
  （同一发送会话内重传的文件, 通过 RAM 中的映射表判重, 不读盘）

## 3. 已知约束

- **XFS reflink**: `xfs_info /data | grep reflink` 确认=1（Ubuntu 默认 XFS 开启）
- **同一文件系统**: blob 与目标文件都在 /data ✓
- **磁盘空间**: blob 峰值 = participant 总量（~2.5TB/机）+ 目标文件共享块
  （reflink 后 unlink 前, 共享不额外占用; unlink 后归属目标）
- **崩溃恢复**: blob + 内存映射丢失 → 发送端按大小校验重传（最终文件
  不存在/不完整 → 重传）→ 自愈 ✓
- **多个 participant 同时传**: 当前场景一次一个; 多个时 blob 按 participant 命名

## 4. 改动范围

仅 `baseline_recv_data.cpp`（接收端, 单文件）:

| # | 改动 | 量 |
|---|---|---|
| 1 | flush_thread 改写: open blob 一次, 所有文件 append + 记录 offset | ~40 行 |
| 2 | 切分函数: reflink_ficlone_range + 遍历映射表 + unlink blob | ~50 行 |
| 3 | BYE drain: 等 blob 写完 + 切分完再应答 | ~10 行 |
| 4 | SKIP 逻辑: 传输中查内存映射, 不读盘 | ~10 行 |
| 5 | 清理: 阶段计时仪表保留（验证用） | 0 |

## 5. 验证

1. **fio 基线对照**: 清空+trim+静置后先跑 fio（确认 4.4 仍在）, 紧接跑传输
2. **小批**: 单相机 250GB, 看聚合速率（预期 ~3.5-4.5 GB/s）
3. **切分正确性**: 切分后每文件 md5 抽查 + 文件数/字节数与发送端一致
4. **全量**: 2.4TB, 预期 ~10 分钟
5. **xfs reflink 确认**: `xfs_info /data | grep reflink` = 1

## 6. 备选: fdatasync 串行化（若 reflink 不可用）

每文件写完后 `fdatasync(fd)` 等回写完成再写下一文件:
- 保证页缓存任一时刻只有一个文件的脏页 → 回写连续
- 代价: 文件间同步等待, 吞吐 = 回写速率 (仍应 ~3-4 GB/s)
- 实现更简单 (~5 行), 但不是真正的"连续 append"模式
