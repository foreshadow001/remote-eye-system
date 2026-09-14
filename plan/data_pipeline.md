# 数据流水线 — 双机串行直写硬盘柜 (v3 定稿)

2026-09-14 v3。**架构简化定稿：抛弃 NVMe 中继，master/slave 串行 direct 直写
ZFS 归档柜，处理也从柜读。** v1/v2 的 NVMe 中转方案 (blob+reflink+rsync 归档+
fstrim 循环) 全部退役。

## 1. 定稿依据 (实测)

| 事实 | 来源 |
|---|---|
| 柜 direct 直写 8 流 3.0-3.1 GB/s, 全程平线无下坠 | 2026-09-14 slave 单相机/全量实测 |
| 柜比 NVMe 稳定 (无 QLC 折叠/SLC 缓存周期) | 同上 + transfer_ram_buffer.md §10 |
| 冷读 1.7 GB/s 够处理用 | raid36-hc580-research.md §5 |
| NVMe blob 链路出现**内容损坏** (11:10 批次, 第0帧正常/第1000帧整体偏移, 采集端已排除) | 用户核验; blob 多段 reflink 路径嫌疑, 未定位根因即退役 |
| 2.4TB/台 @3GB/s ≈ 13min, 串行双机 ≈ 27min | direct 实测推算 |

## 2. 架构 (v3)

```
master ──口1(10.10.1.1)──┐
                         ├─→ 接收端 (单进程, direct) ──→ /archive (ZFS tank)
slave  ──口2(10.10.2.1)──┘         64MB 块 CRC32 校验       = 最终归档形态
                                                         (无中转/无 rsync/无 fstrim)
处理: 直接从 /archive 读 (冷读 1.7 GB/s)
NVMe /data: 退役 (blob 模式代码保留在 baseline_recv_data, 生产不用)
```

- **串行**: 一台传完再传另一台 (master 先, 附件随行; 或反之) — 柜无双流
- 附件 (xml/IR/arm_pose/map.json) 同走 direct + FORCE, 落 `/archive/calib/...`
  与根目录 → **bench-archive.sh 归档步骤退役** (--check 仍可作物料核对)
- 布局: `/archive/capture/{P}/{SN}/*.h5`, 两机相机目录汇合 (send_slave relpath
  统一 capture/ 前缀后与 rm-participant.sh 兼容)
- 正确性: 传输层由 **64MB 块 CRC32 trailer 协议**保障 (不符 → ERR → 自动重传
  自愈); 端到端抽验用双端 md5 (见 §4)

## 3. 部署

```bash
# 4090 唯一接收端 (direct, 双口都可达)
./transfer/build/baseline_recv_data --bind 0.0.0.0 --port 5001 --out /archive --direct
```

```powershell
# 两台采集机各自 (角色/链路/附件由 capture.yaml is_master 自动决定)
send_slave.exe                  # master: →10.10.1.1, h5+附件
send_slave.exe                  # slave : →10.10.2.1, 仅 h5
```

## 4. 正确性验收 (每轮)

1. 传输中: 接收端 `ERR checksum block N` 即管线问题实锤 (CRC 给出损坏块
   的文件内偏移); 自动重传后 `fail=0` 为通过
2. 传完: 双端 md5 全量对账
   ```bash
   # 4090
   cd /archive/capture/P001/<SN> && md5sum *.h5 | sort > /tmp/recv.md5
   # 采集机 (git-bash), 同一相机目录
   md5sum *.h5 | sort > /tmp/src.md5 && diff /tmp/src.md5 /tmp/recv.md5
   ```
3. 抽验: `h5dump -H <file>` 维度 + 读第 1000 帧目视

## 5. 历史决策记录 (供追溯)

- v1 (2026-09-14 早): master 落 NVMe 处理后 rsync 归档, slave 直写柜 —
  因 blob 链路内容损坏 + 柜性能超预期而简化
- v2: 两机并行 (NVMe+柜双流) — 53min 实测暴露 QLC SLC 上限; 且 blob 数据损坏
- blob 模式多段 reflink 的损坏根因**未定位** (CRC 协议上线后可低成本复测定位,
  但生产已不再依赖该路径)
- 传输优化全程: `transfer_ram_buffer.md`; direct 模式设计: `transfer_direct_mode.md`
