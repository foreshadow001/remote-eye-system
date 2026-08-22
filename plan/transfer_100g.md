# 100G 数据传输方案 — 采集主机 (Win10 ×2) → 数据处理主机 (Ubuntu 24.04)

> 目标：把一名参与者（20 相机 × 25 h5 × 10GB ≈ 5TB）从 master/slave 通过
> 100G DAC 直连转移到处理主机。理论下限 400s，工程目标 **10–20 分钟**。

## 一、拓扑与 IP 规划

```
master (Win10, LRES1019PF 单口)  ←— 100G DAC —→  处理主机 口1 (10.10.1.1/24)  ┐
slave  (Win10, LRES1019PF 单口)  ←— 100G DAC —→  处理主机 口2 (10.10.2.1/24)  ┘ 两口合计 100G
```

| 设备 | 网卡 | IP | MTU |
|------|------|-----|-----|
| 处理主机 口1 (enp?) | LRES1014PF-2QSFP28 (E810-C) | 10.10.1.1/24 | 9000 |
| 处理主机 口2 (enp?) | 同上 | 10.10.2.1/24 | 9000 |
| master | LRES1019PF-QSFP28 (E810-C) | 10.10.1.2/24 | 9000 |
| slave（本机） | LRES1019PF-QSFP28 (E810-C) | 10.10.2.2/24 | 9000 |

两条独立 /24 点对点，无网关无路由。相机 SN 全局唯一（每台 10 个、两台共 20 个），
接收端目录无冲突：`/data/{P001}/{SN}/NNNN.h5`。

## 二、数据处理主机配置（Ubuntu 24.04 server）

### 2.1 网卡驱动（Intel E810 → ice 驱动）

Ubuntu 24.04（内核 6.8）**自带 in-tree ice 驱动**，E810-C 基本连通开箱即用。
先验证，再决定是否装最新版：

```bash
lspci | grep -i E810                 # 识别两颗 E810-C
ls /sys/module/ice                   # 驱动已加载?
ip link                              # 应出现两个 100G 接口 (enp…)
sudo apt install linux-firmware      # ice DDP 固件包 (缺它会降级/报 DDP 不匹配)
dmesg | grep ice                     # 关注 "DDP package" 与链路状态
```

若要最新驱动（可选，修复与新特性）：
- 官方驱动下载：[Intel Network Adapter Driver for 800 Series Devices under Linux](https://www.intel.com/content/www/us/en/download/19630/intel-network-adapter-driver-for-800-series-devices-under-linux.html)
- 官方源码仓库：[intel/ethernet-linux-ice](https://github.com/intel/ethernet-linux-ice/)（`cd src && make install`）
- 官方安装指引：[How to Build and Install Linux Drivers for 800 Series](https://www.intel.com/content/www/us/en/support/articles/000092128/ethernet-products/800-series-network-adapters-up-to-200gbe.html)
- 建议 DKMS 化，避免内核升级后失效；固件（NVM）更新用 [E810 NVM Update Utility](https://www.intel.cn/content/www/cn/zh/products/sku/192558/intel-ethernet-network-adapter-e810cqda2/downloads.html)（同芯片原厂卡页面，可选）
- 卡官方规格：[LR-LINK LRES1014PF-2QSFP28](https://www.lr-link.com/tc/products-detail/id/2349)（PCIe 4.0 x16，**必须插 x16 槽**否则带宽不足）

### 2.2 静态 IP + 巨帧（netplan）

`/etc/netplan/01-100g.yaml`（接口名以 `ip link` 实测为准）：

```yaml
network:
  version: 2
  ethernets:
    enp1s0:            # 口1 ← master
      mtu: 9000
      addresses: [10.10.1.1/24]
    enp2s0:            # 口2 ← slave
      mtu: 9000
      addresses: [10.10.2.1/24]
```

`sudo netplan apply`。**两端 MTU 必须一致**（Windows 端也要 9000），不一致
表现为巨包黑洞、速度掉到 KB 级或卡死。

### 2.3 内核 TCP 调优（100G 参考配方，来自 [ESnet Fasterdata](https://fasterdata.es.net/host-tuning/linux/100g-tuning/)）

```bash
# /etc/sysctl.d/99-100g.conf
net.core.rmem_max  = 134217728        # 128MB
net.core.wmem_max  = 134217728
net.ipv4.tcp_rmem  = 4096 87380 134217728
net.ipv4.tcp_wmem  = 4096 65536 134217728
net.core.netdev_max_backlog = 250000
```

```bash
sudo sysctl --system
sudo cpupower frequency-set -g performance     # CPU 满频
sudo ethtool -G <iface> rx 4096 tx 4096        # 收发环缓冲拉满 (先 ethtool -g 看上限)
```

### 2.4 磁盘（两块 8TB M.2，尚未格式化）

容量核算：单参与者 5TB < 16TB；RAID0 后单挂载点最简单，顺序写 ~2×7GB/s
远超网络需求。参考：[DigitalOcean mdadm on Ubuntu](https://www.digitalocean.com/community/tutorials/how-to-create-raid-arrays-with-mdadm-on-ubuntu)、
[Ubuntu 社区 SoftwareRAID](https://help.ubuntu.com/community/Installation/SoftwareRAID)、
[RHEL 管理 RAID 官方文档](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux/9/html/managing_storage_devices/managing-raid_managing-storage-devices)。

```bash
lsblk                                    # 确认两块 nvme (各 8TB, 无分区)
sudo mdadm --create /dev/md0 --level=0 --raid-devices=2 \
     --chunk=256K /dev/nvme0n1 /dev/nvme1n1
sudo mkfs.xfs -d su=256k,sw=2 -L DATA /dev/md0    # 条带对齐 (RAID0 专用)
sudo mkdir /data
echo '/dev/md0 /data xfs defaults,noatime 0 2' | sudo tee -a /etc/fstab
sudo mount -a
sudo mdadm --detail --scan | sudo tee -a /etc/mdadm/mdadm.conf   # 重启后阵列自动装配
sudo systemctl enable fstrim.timer
```

> 备选：不做 RAID，两盘分别挂 `/data1 /data2` 按 master/slave 分流——
> 少一层软件 RAID，故障域更小；缺点是目录分散。默认推荐 RAID0。

### 2.5 网络与磁盘验收

```bash
# 处理主机做 iperf3 服务端 (双口各起一个)
iperf3 -s -p 5201 &   iperf3 -s -p 5202 &
# Windows 端 (装 iperf3): 每口预期 ≥ 60-90 Gbps (多流 -P 8)
iperf3 -c 10.10.1.1 -P 8 -t 30 -w 64M
# 磁盘顺序写验收 (预期 > 5 GB/s)
fio --name=t --filename=/data/t.bin --size=50G --bs=1M --rw=write --direct=1 --runtime=30
```

## 三、采集端配置（master / slave，驱动已装）

仅三件事（管理员 PowerShell，两台各配自己的 IP）：

```powershell
# 静态 IP (无网关) —— master 用 10.10.1.2/24, slave 用 10.10.2.2/24
New-NetIPAddress -InterfaceIndex <idx> -IPAddress 10.10.1.2 -PrefixLength 24
# 巨帧 9000
Set-NetAdapterAdvancedProperty -Name "<网卡名>" -RegistryKeyword "*JumboPacket" -RegistryValue 9000
# 防火墙放行发送脚本 (出站默认允许; 如入站测试被拦再放行 5201)
```

注意：D:\capture、E:\capture 已加入 Defender 排除（上次排查时配置），
读 5TB 数据不会再触发实时扫描。

## 四、传输实现（定稿：C++ 发送端 + Python 接收端）

**架构**（用户指定，比双机并行更可靠 —— 串行传输，逐台验证）：

```
[slave C++] ──握手(现有 192.168.10.x 网, :50100)──> [master C++]
     │  1. slave → master: "READY <n> <bytes>"
     │  2. master → slave: "START"
     │  3. slave 经 100G 口2 (10.10.2.1) 传给处理主机
     │  4. slave → master: "SLAVE_DONE <ok> <skip> <fail>"
     │  5. master → slave: "ACK"
     └─ 6. master 经 100G 口1 (10.10.1.1) 传输
```

- **发送端 send_data.exe（C++，master/slave 同一程序）**：角色自动取
  capture.yaml `is_master`；与项目内收发/重试风格一致（无限重试握手）
- **接收端 recv_data.py（Python，Ubuntu）**：不变（已实现并回环自测）
- 数据协议不变：`FILE <rel> <size> 0\n` + 字节流 → `OK/SKIP/ERR`，
  4 工作线程 × 4MB 块，SKIP 断点续传
- 配置：`cfg/transfer.yaml`（server 口1/口2 IP、端口、握手端口、workers）；
  participant 从 capture.yaml
- CLI 覆盖参数用于回环测试（--role/--handshake-ip/--data-ip）
- 传输完成后**默认不删**源数据；sha256 关闭（大小校验 + 抽查兜底）

## 五、验证清单

1. `iperf3 -P 8` 每口 ≥ 60Gbps；不达标查 MTU 一致性 / 插槽 x16 / 环缓冲
2. fio 顺序写 ≥ 5GB/s
3. 传输后：文件数 = 20 相机 × 25 = 500；目录数 20；总字节与源端一致
4. 抽查 3 个 h5（不同相机）：`h5dump -H` 数据集维度 [2000,2048,2448] 完整
5. sender 汇总无 FAIL；receiver 日志无 ERR

## 六、风险与坑

| 风险 | 对策 |
|------|------|
| 两端 MTU 不一致（巨包黑洞，速度 KB 级） | 配置后 `ping -f -l 8972 <对端>` 验证巨帧通 |
| LRES1014PF 插在 x8 槽 | 100G 需 PCIe4.0 x16；`lspci -vv` 看 LnkSta 宽度 |
| in-tree 驱动旧、E810 链路不稳（DDP 不匹配） | dmesg 检查；必要时装 Intel 最新 ice + DDP |
| Win10 TCP 单流跑不满 | 4 并发流已规划；iperf3 实测定流数 |
| 断电/中断 | SKIP 断点续传；fallocate 防残缺文件被误判完整（大小不符即重传） |
| /data 写满（16TB 仅容 3 个参与者） | 脚本收到 ERR disk-full 即停；处理完及时清理 |

## 七、待确认

1. 脚本存放位置：建议 `cpp_eyetracker/tests/utils/transfer/`（send_data.py / recv_data.py）
2. 传输完成后是否自动/手动删除采集端源数据（默认不删，人工确认后删）
3. sha256 全量校验默认关闭是否可接受（有大小 + 抽查兜底）
