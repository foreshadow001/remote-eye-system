# 传输 V2 终版 — 保持 RAID0, 调优抬顶 + 顺序读/写管道

> **2026-08-29 终局: V2 实测失败, 生产定格 baseline (3.6-3.8 GB/s)。**
> 见 §五 终局数据。下文为设计过程存档。

## 五、V2 终局（实测否决）

| 运行 | 速率 |
|------|------|
| V2 初版 (8 流 + 逐块 fadvise) | 1.7 GB/s |
| V2 移除 fadvise | 1.7 GB/s (无变化, 假设排除) |
| 对照: 4 流用户态 (--tx user, 文件并行) | 3.0 GB/s |
| 对照: baseline (TransmitFile 内核态, 文件并行) | **3.6-3.8 GB/s** |

**判决**: 用户态 send() 总吞吐随流数不升反降 (4 流 3.0 → 8 流 1.7),
本机 (Win10+E810) 用户态发送总天花板 1.7-3.0, 低于内核态 TransmitFile 一倍。
而 TransmitFile 的形态约束 (每连接独立顺序文件) 正是 baseline 的文件并行架构
—— **baseline 即本硬件栈的实测最优解**。

XFS 调优 (allocsize=1G 等) 将接收端缓冲写从 4.5 抬到 4.9 GB/s, 保留生效。

**生产配置 (定格)**: baseline_recv_data + send_slave (默认 4 流, TransmitFile);
5TB 四盘串行 ≈ 20-23 分钟。V2 代码 (send_v2/recv_v2) 存档不进生产。


## 一、两步路径

**第一步 — 免代码调优, 抬高 RAID0 写天花板 (4.5 → 预期 ~6)**

1. XFS 挂载参数 (`/etc/fstab` 的 /data 行): `allocsize=1G,logbsize=256k,largeio,swalloc`
2. 脏页/网络积压 (并入现有 /etc/sysctl.d/99-100g.conf):
   `vm.dirty_ratio=60`, `vm.dirty_background_ratio=30`
   (netdev_max_backlog 已设 250000, 无需重复)
3. 验收: `dd if=/dev/zero of=/data/t.bin bs=1M count=30000 status=progress`
   缓冲写应从 4.5 升至 ~6 GB/s

**第二步 — V2 管道 (单读线程 → 内存环队列 → K 发送线程 / K 收线程 → 重排 → 单写线程)**

```
发送端:  1 读盘线程 (单句柄顺序读, 5.46 GB/s)
         → 16 槽 × 8MB 环队列 (带 seq)
         → K 个发送线程 (普通 send(), 各持一条持久连接)
接收端:  K 个收线程 → 按 seq 入重排 map (天然有界: 发送端 16 槽在途)
         → 1 写线程 顺序 write + posix_fadvise(DONTNEED)
```

**参数修正 (相对三审稿)**: K=4 不够 — 用户态 send() 单流实测硬顶
~0.75-0.8 GB/s (iperf 0.79 与 --tx user 每流 750MB/s 互证), K=4 只有 3.0。
**默认 K=8** (8×0.75≈6.0 ≥ 写上限), --streams 可调, 实测定值。

## 二、协议 (V2)

```
conn0:  "FILE <sid> <rel> <size> <K>\n"  →  "GO <sid>" / "SKIP" / "ERR ..."
conn1..K-1: "JOIN <sid>\n"               →  "JOINED"
任意:   "DATA <sid> <seq> <len>\n" + <len 字节>
conn0:  "END <sid>\n"                    →  "OK <size>" / "ERR ..."
BYE
```

- 发送端收到 GO + 全部 JOINED 才开始流数据 (避免 DATA 无会话可归属)
- 会话内 seq 从 0 单调; 重排天然有界 (发送端环 16 槽限制在途块)
- 容错: 会话/应答超时 30s (100G 专线 TCP 重传 ms 级, 裕量充足);
  超时按整文件重传 (粒度同 baseline)
- SKIP 续传/原子落位/路径安全: 同 baseline 语义

## 三、O_DIRECT 复活奇招 (备选, 缓冲写路径不达标时)

`xfs_io -c "extsize 16m" <接收目录>` + 接收端 O_DIRECT (不 fallocate) —
XFS 直接分配大块 Written extent, 避开 Unwritten 串行转换 (此前 2.45 的死因)。
预估 5.5-6.0 GB/s。recv_v2 预留 --direct 开关。

## 四、实施

1. 4090: 第一步调优 → dd 验收 (目标 ≥5.5)
2. send_v2.cpp (Windows) / recv_v2.cpp (Ubuntu) — 新文件, baseline 不动
3. 实测 K=6/8 定值; 达标 (≥4.5 稳定) 替换生产用法
