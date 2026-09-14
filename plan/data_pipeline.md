# 数据流水线 — 两机并发传输 + NVMe 处理中转 + ZFS 归档

2026-09-14 定稿（v2, 修正柜文件系统为 ZFS + 串行归档约束）。
承接 `transfer_ram_buffer.md` 的传输结论 (v3.2 / QLC SLC 上限)。

## 1. 分流原则与数据布局

| 主机 | 数据用途 | 路径 | 途经 NVMe |
|------|---------|------|----------|
| master | 需 4090 处理 | master ─口1→ 接收端A → `/data/dataset` (NVMe, XFS, blob+reflink) → 处理 → rsync 归档 → `/archive/dataset` | 是 (中转) |
| slave | 无需处理 | slave ─口2→ 接收端B → **直写** `/archive/dataset` (ZFS tank) | 否 |

- 路径布局对齐: 两路最终都落在 `/archive/dataset/capture/{P}/{cam}/*.h5`,
  与 `archive/bench-archive.sh` 的 `DST=/archive, SRC=/data/dataset` 布局一致
- NVMe 每轮只承载 master ~2.4TB = 每盘 1.2TB ≈ SLC 缓存内 → 传输永远满速
- ZFS lz4 对 h5 实测 ~1.8x → 4.8TB 逻辑量实占 ~2.7TB

## 2. 时序 (硬约束: slave 全部落柜后才开始归档 master)

```
t=0    两机并发开传 (send_ui 并行模式)
       master → NVMe @3.7 GB/s          slave → ZFS @2-3 GB/s (待实测)
t≈11   master 传完 → 立即开始处理 (读 NVMe; 与 slave 写 ZFS 资源不相交, 不必等)
t≈16-19 slave 传完 (BYE → ZFS sync 落 TXG)
       ├─ 处理可能仍在跑 (读 NVMe 不冲突, 归档等它完成)
       └─ 处理完 + slave 完 → 开始归档: bench-archive.sh PART=P00x
t≈35-40 归档完 (2.4TB @ ~2.09 GB/s 实测均值) → --check 对账
       → NVMe: rm -rf /data/dataset/* && fstrim (TRIM 取消折叠债, 无需静置)
       → 下一轮立即可开
```

瓶颈: 全流程地板 = max(网络+NVMe 链 11min, ZFS 写入 2.4TB, 归档 rsync 2.4TB)
≈ **35-40 min/轮**。对比 4.8TB 全落 NVMe 的 53min (后半程 0.1-0.4 GB/s)。

## 3. 接收端 B 的架构决策: 直写模式 (不做 blob)

> 详细设计（splice vs 缓冲池论证、ZFS dirty 层背压分析、验收指标）见
> `transfer_direct_mode.md`（2026-09-14 计划, 实施以此为准）。

事实链:
- `/archive` 是 ZFS (OpenZFS on Ubuntu 24.04), 非 XFS — FICLONERANGE
  依赖 2.2+ block cloning 且对齐语义是 recordsize (非 4096), blob 段布局大概率 EINVAL
- blob+reflink 架构是为 QLC 并发交织惩罚发明的; ZFS/draid2 36 盘无此问题
- → **接收端 B 用 v1 式直写**: 每连接顺序写自己的文件, 无队列/flush/pending

改动 (baseline_recv_data.cpp 增加 `--direct` 运行时开关, 单 binary 双模式):
- direct 模式: handle_conn 收到 GO 后直接 `open(.part) → write 循环 → rename`,
  复用现有热缓冲池 (1MB 块) / SKIP 判重 / 头解析 / 路径安全检查
- BYE 屏障: direct 模式下等所有在途文件写完 + `syncfs` (刷 ZFS TXG) 再回 ack
- 非 direct (默认) 行为完全不变 — 接收端 A 零改动
- 可选记录性验证: `test_ficlone` 在 /archive 跑一次, 确认 block cloning 不可用
  (仅留档, 不作为方案依赖)

## 4. 部署

```bash
# 4090: 接收端 A (master 链路 → NVMe, 现有 v3.2 binary)
./transfer/build/baseline_recv_data --bind 10.10.1.1 --port 5001 --out /data/dataset

# 4090: 接收端 B (slave 链路 → ZFS, direct 模式)
./transfer/build/baseline_recv_data --bind 10.10.2.1 --port 5001 --out /archive/dataset --direct
```

发送端零改动: transfer.yaml 本就按角色分流 (master_link/slave_link),
send_ui 并行模式 SPACE 即两机同传。

## 5. 脚本适配项 (archive/bench-archive.sh)

1. `--check` 口径: 双主机数据分居两侧 — master 的相机目录在 SRC/DST 双侧比对;
   slave 的目录仅存在于 DST。需改为按相机目录粒度: SRC 有的比对两侧,
   SRC 没有的 (slave 目录) 只统计数量供人工对发送端 summary
2. 归档范围: bench-archive.sh 只动 master 的数据 (SRC=/data/dataset) — 正确, 不改
3. 小项 (calib/IR/map.json) 均由 master 采集 → 在 NVMe → rsync 正常覆盖

## 6. 风险与上线校验

| 项 | 风险/未知 | 动作 |
|----|----------|------|
| ZFS 网络直写速度 | 无实测 (rsync 2.09 是 NVMe 读+rsync 开销的合成值) | 上线首日记录接收端 B status 速率; 若 <2 GB/s 再调 (recordsize/zil) |
| 8 流并发写 ZFS | draid2 36 盘吸收, 预期无交织惩罚 | 同上, 首日观测 |
| 归档期 NVMe 读 | 折叠债期 2-3.5 GB/s ≥ rsync 需求 | 首轮记录 bench 输出速率 |
| ZFS ARC 内存 | 与 staging 池 (1GB) 共存, 251GB 充裕 | 无动作 |
| 池容量 | 每轮 +2.7TB 实占, 523TiB 池 | 按采集周期轮换; 每月 scrub 避开传输窗口 (README 已有) |
| 崩溃一致性 | slave 半传文件 | .part + rename 原子; 发送端 SKIP 续传 |

## 7. 单轮操作清单

1. 4090: 起接收端 A、B (§4 两条命令)
2. master: send_ui SPACE → 两机同传
3. master 传完 → 跑处理 (不必等 slave)
4. slave 传输完成 (接收端 B BYE ack) + 处理完成 → `PART=P001 bash archive/bench-archive.sh`
5. `--check` 对账 + `zpool status tank` 尾三行
6. `bash archive/rm-participant.sh P001 nvme go` (或 rm + fstrim)
7. 下一轮开始 (NVMe 已就绪, 无需静置)

## 8. 实施顺序 (代码, 待批准后)

1. baseline_recv_data.cpp 加 `--direct` (handle_conn 直写分支 + BYE syncfs, ~80 行)
2. bench-archive.sh `--check` 按相机目录粒度适配 (~20 行)
3. 4090 编译 + test_ficlone 在 /archive 留档验证
4. 首轮全流程试跑 (单人), 按 §6 表回填实测数字到本文档
