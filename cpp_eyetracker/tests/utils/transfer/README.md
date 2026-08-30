# transfer — 采集端传输工具集 (Windows)

把采集主机（master / slave）上一名参与者的采集数据经 100G 直连发送到
数据处理主机（4090, Ubuntu, [preprocess-server](../../../../..
/../preprocess-server)，接收端以 `--out /data/dataset` 启动）。
配套调研与实测结论见
[plan/transfer_100g.md](../../../plan/transfer_100g.md)、
[plan/transfer_perf_report.md](../../../plan/transfer_perf_report.md)、
[plan/transfer_seq_design.md](../../../plan/transfer_seq_design.md)。

## 程序

| 程序 | 说明 |
|------|------|
| `send_ui` | **生产（交互式）**：配置屏确认 → SPACE 开始；角色自动（capture.yaml `is_master`）；D:/E: 图像逐盘串行；master 追加内外参 XML / IR 位置 / day_participant_map.json 阶段 |
| `send_slave` | **生产（CLI）**：同一引擎的命令行版，适合脚本化/重测 |

历史版本（send_data 串行握手版、send_v2 分条实验、baseline 快照、Python 接收端）
已清理，见 git 历史与上述报告。

## 落位布局（发送端 relpath 决定；接收端默认输出根 `/data/dataset`，无需参数）

```
/data/dataset/
├── capture/{P}/          h5 图像 (slave: 阶段 1-2; master 同)
├── calib/cams/{P}/       相机内外参 xml (master 阶段 3, {calib_save_dir}/{P}/output)
├── calib/IR/D{n}.txt     红外发射器位置 (master 阶段 4, cfg/IR/{day_id}.txt)
└── day_participant_map.json   (master 阶段 5)
```

## 配置

- `cfg/transfer.yaml`：`participant_id`、`server_ip_master_link` / `server_ip_slave_link`、
  `server_port`（5001）、`workers`（**4 为实测最优**）
- `cfg/capture.yaml`：`is_master`（角色）；`participant_root`（send_slave 默认扫盘）
- master 附加源：`cfg/cam_calib.yaml`（calib_save_dir）、`cfg/calib_arm.yaml`（record.day_id）

## 用法

```powershell
# 交互式 (推荐; master/slave 通用):
send_ui.exe                                  # 配置屏核对后 SPACE 开始

# CLI:
send_slave.exe --participant P001 --roots D:/capture E:/capture    # 注意: 两盘并发混读
send_slave.exe --participant P001 --roots D:/capture               # 串行需分两次跑
send_slave.exe --cams 40768741 40768742 ...                        # 只传指定相机
```

## 协议（与 baseline_recv_data 配对）

```
"FILE <relpath> <size> <sha256|0>\n" + <size 字节内容>
  → "OK <size>" | "SKIP"(对端已有同大小文件, 断点续传) | "ERR <原因>"
"BYE\n" 结束
```

特性：文件级 SKIP 断点续传、失败重试 3 次、`.part` 原子落位（接收端）、
头行与数据同包残留正确处理。

## 性能基线（2026-08-29/30 实测）

- 单采集机（两盘串行 2.5TB）：slave ~9 min（待复核 SKIP），master 11 min
- 单相机 250GB ≈ 65-70s（3.6-3.9 GB/s）
- 关键结论：用户态 send() 总天花板 1.7-3.0 GB/s（随流数负增长），
  必须用内核态 TransmitFile；单流 TCP 上限 ~0.8-1.9 GB/s
