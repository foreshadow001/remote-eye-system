# transfer — 采集端传输工具集 (Windows)

把采集主机（master / slave）上一名参与者的 h5 数据经 100G 直连发送到
数据处理主机（4090, Ubuntu, [preprocess-server](../../../..
/../../preprocess-server)）。配套调研与实测结论见
[plan/transfer_100g.md](../../../plan/transfer_100g.md)、
[plan/transfer_perf_report.md](../../../plan/transfer_perf_report.md)、
[plan/transfer_seq_design.md](../../../plan/transfer_seq_design.md)。

## 程序一览

| 程序 | 状态 | 说明 |
|------|------|------|
| `send_slave` | **生产** | 独立发送器（无握手），TransmitFile 内核态读发重叠，默认 4 流 |
| `send_data` | 备用 | master↔slave 握手串行版（slave 先传，slave 失败则 master 不传） |
| `baseline_send_slave.cpp` | 快照 | 历史最优配置的 send_slave 存档 |
| `send_v2` | 存档 | V2 管道实验（1 顺序读 + K 用户态发送线程），实测 1.7 GB/s 已否决 |

## 配置

- `cfg/capture.yaml`：`participant_id`（发送谁）、`participant_root`（扫哪些盘）
- `cfg/transfer.yaml`：`server_ip_master_link` / `server_ip_slave_link`（4090 两口）、
  `server_port`（5001）、`workers`（并发流数，**4 为实测最优**）

## 用法

```powershell
# build 目录下
# slave 生产用法 (participant 读 capture.yaml, 默认 D:/E: 两盘全扫):
send_slave.exe --participant P001 --roots D:/capture

# 只传指定相机:
send_slave.exe --participant P001 --roots D:/capture --cams 40768741 40768742

# master (指向 4090 口1):
send_slave.exe --data-ip 10.10.1.1 --participant P001 --roots D:/capture

# 诊断: 用户态发送对照 (默认 TransmitFile):
send_slave.exe --tx user ...

# 四盘串行全量: 逐盘切换 roots/participant
```

## 协议（与 baseline_recv_data 配对）

```
"FILE <relpath> <size> <sha256|0>\n" + <size 字节内容>
  → "OK <size>" | "SKIP"(对端已有同大小文件, 断点续传) | "ERR <原因>"
"BYE\n" 结束
```

特性：文件级 SKIP 断点续传、失败重试 3 次、`.part` 原子落位（接收端）、
头行与数据同包残留正确处理（曾致 short read 4055B 的坑已修）。

## 性能基线（2026-08-29 实测）

- 单采集机单盘：**3.6-3.9 GB/s**（冷态），4.4 热态上沿
- 单相机 250GB ≈ 65s；5TB 四盘串行 ≈ 21 分钟
- 关键结论：用户态 send() 总天花板 1.7-3.0 GB/s（随流数负增长），
  必须用内核态 TransmitFile；单流 TCP 上限 ~0.8-1.9 GB/s
