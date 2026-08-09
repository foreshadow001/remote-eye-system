# test_piper_ctrl.cpp — 单臂 TCP 控制测试

> 基于 `tests/utils/piper/get_piper_pose.cpp` 改造，实现对 Piper 机械臂的 TCP 指令控制和位姿反馈
> 最后更新：2026-08-08

## 概括

在 Windows 端新建 `test_piper_ctrl.cpp`，读取 `cfg/gaze_target/P001/piper_upper.txt` 和 `piper_lower.txt` 中的目标点位，通过 TCP 逐点发送至 Ubuntu 端。**启动时 upper 和 lower 均回零**，用户按空格依次执行当前臂的所有点位，按 `t` 切换机械臂。全部执行完毕后提示完成，按 ESC/q 退出。Ubuntu 端新建 `piper_windows_ctrl_server.py`，接收指令后调用 `PiperArmController` 驱动机械臂运动，运动完成后从 ROS `/end_pose` 主题获取实际到达位姿，回传 Windows 端显示。

```
Windows (test_piper_ctrl.cpp)              Ubuntu (piper_windows_ctrl_server.py)
  │                                              │
  │─ 读取 cfg/gaze_target/P001/piper_upper.txt     │
  │─ 读取 cfg/gaze_target/P001/piper_lower.txt     │
  │─ TCP connect ───────────────────────────────→│─ 监听 :49301
  │─ MOVE_JOINTS:upper 回零 ────────────────────→│─ ctrl.move_to_joints(...)
  │←────────────────────── MOVED:upper:... ──────│
  │─ MOVE_JOINTS:lower 回零 ────────────────────→│─ ctrl.move_to_joints(...)
  │←────────────────────── MOVED:lower:... ──────│
  │                                              │
  │─ [空格] MOVE_TO:<arm>:x,y,z ────────────────→│─ ctrl.move_to(x,y,z)
  │←────────────────────── MOVED:<arm>:... ──────│─ 查询 /end_pose
  │  (OpenCV 显示指令位姿 vs 实际位姿)              │
  │  ... 循环直至当前 arm 所有点位执行完毕 ...        │
  │                                              │
  │─ [t] 切换到 lower arm ───────────────────────→│─ (同上流程)
  │  ... lower 全部点位执行完毕 → 提示完成           │
  │                                              │
  │─ [ESC] SHUTDOWN ────────────────────────────→│─ rospy.signal_shutdown()
```

- **Windows 端**：纯单线程阻塞模式（send → recv → 显示 → 等按键），无需摄像头或 HDF5
- **Ubuntu 端**：独立脚本，不修改现有的 `windows_end_monitor.py`；使用独立端口 `49301` 避免冲突
- **协议**：换行分隔纯文本，格式为 `MOVED:arm:x,y,z,qx,qy,qz,qw,α,β,γ`
- **双机械臂**：启动时 upper 和 lower 都回零，按 `t` 切换，各自独立执行本臂的全部点位

---

## 一、目标

创建 `test_piper_ctrl.cpp`，从 Windows 端通过 TCP 向 Ubuntu 端的 Piper 机械臂发送运动指令，接收实际到达位姿，在 OpenCV 窗口中展示。这是后续将真实 `gaze_target` 写入 HDF5 文件的第一步。

---

## 二、涉及文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `cpp_eyetracker/tests/utils/piper/test_piper_ctrl.cpp` | **新建** | Windows TCP 客户端 + OpenCV UI |
| `cpp_eyetracker/tests/utils/piper/CMakeLists.txt` | **修改** | 添加 `test_piper_ctrl` target |
| `cpp_eyetracker/tests/utils/CMakeLists.txt` | **修改** | 取消注释 `add_subdirectory(piper)` |
| `piper_ros/src/piper_moveit/moveit_ctrl/scripts/piper_windows_ctrl_server.py` | **新建** | Ubuntu 端独立 TCP 服务脚本，处理运动指令 |
| `plan/improve_piper_ctrl.md` | **新建** | 本计划 |

---

## 三、现有代码基础

`tests/utils/piper/get_piper_pose.cpp`（~230 行）提供了可直接复用的模式：

| 组件 | 说明 |
|------|------|
| Winsock 头文件块 | `WIN32_LEAN_AND_MEAN` → `winsock2.h` → `ws2tcpip.h` → `#pragma comment(lib, "ws2_32.lib")` |
| `recvLine(SOCKET, string&, timeout_ms)` | 从 TCP 流读取一行（以 `\n` 结尾），带 `SO_RCVTIMEO` + deadline 循环 |
| `parsePoseResponse()` | 解析 `POSE:upper:x,y,z,qx,qy,qz,qw,alpha,beta,gamma` 格式字符串 |
| `drawUI()` | OpenCV 窗口，`cv::putText` 状态展示，`waitKey` 事件轮询 |
| `main()` | `WSAStartup` → Cfg 加载 → `thread(tcpWorker)` → `drawUI()` → `WSACleanup` |
| 配置文件路径 | `fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg" / "piper.yaml"` |

CMake 依赖：仅 `eyetracker::cfg` + `${OpenCV_LIBS}`，无 HDF5、无 Pylon。

---

## 四、TCP 协议设计

采用换行分隔的文本协议，与现有 `GET_POSE` 风格一致。

### 4.1 Windows → Ubuntu

```
MOVE_JOINTS:upper:0.0,0.0,0.0,0.0,0.0,0.0    # 回零：6 个关节角 (rad)
MOVE_TO:upper:0.3,0.1,0.2                      # 笛卡尔移动：x, y, z (m)
SHUTDOWN                                        # 终止服务端
```

### 4.2 Ubuntu → Windows

```
MOVED:upper:0.299,0.101,0.198,0.123,0.456,0.789,0.012,45.0,30.0,10.0
       ^arm  ^x     ^y     ^z     ^qx   ^qy   ^qz   ^qw   ^alpha ^beta ^gamma

ERROR:upper:no_solution
      ^arm  ^reason
```

### 4.3 单点交互时序

```
Windows                         Ubuntu
  |-- MOVE_TO:upper:x,y,z ----->|
  |                              |-- PiperArmController.move_to(x, y, z)
  |                              |-- 读取 /end_pose 主题获取实际位姿
  |<---- MOVED:upper:... -------|
  |-- 显示结果，等待按键         |
  |-- MOVE_TO:upper:x2,y2,z2 -->|
  ...                           ...
  |-- SHUTDOWN ---------------->|
  |                              |-- rospy.signal_shutdown()
```

---

## 五、Ubuntu 端新建：`piper_windows_ctrl_server.py`

新建独立脚本，不复用 `windows_end_monitor.py` 的 `PoseServer`。自行监听 TCP 端口（与 `net.yaml` 中 `windows_end_monitor` 使用不同端口，或共用同一端口但责任分离），接收 Windows 发来的运动指令，调用 `PiperArmController` 执行，返回实际到达位姿。

### 5.1 依赖

```python
import socket
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from piper_arm_controller import PiperArmController
```

- `PiperArmController` 提供 `move_to_joints()` 和 `move_to()`，已在服务模式下验证过可用性
- 位姿反馈通过 ROS 主题 `/piper_upper/end_pose`（类型 `PoseStamped`）获取，不依赖 `ArmPoseCache`

### 5.2 结构与流程

```
main():
  1. rospy.init_node("piper_ctrl_server")
  2. 读取 cfg/net.yaml → ip, port_ctrl（用独立端口，如 49301）
  3. 读取 cfg/piper_upper.yaml → 创建 PiperArmController.from_yaml("upper", use_service=True)
  4. 创建 TCP server socket，bind + listen
  5. 循环 accept：
       - 收到连接 → 进入 _handle_client(conn)
       - 逐行读取指令
       - MOVE_JOINTS → ctrl.move_to_joints(...) → 查 /end_pose → 响应 MOVED
       - MOVE_TO    → ctrl.move_to(...)       → 查 /end_pose → 响应 MOVED
       - SHUTDOWN   → 响应 SHUTDOWN_ACK → 关闭连接 → rospy.signal_shutdown()
       - 未知指令   → 响应 ERROR
  6. rospy.spin()
```

### 5.3 获取实际位姿

不依赖 `ArmPoseCache`（避免与 `windows_end_monitor.py` 耦合）。直接从 ROS 主题读取：

```python
def _get_current_pose(self):
    """从 /end_pose 主题获取当前末端位姿 (return Pose or None)"""
    topic = f"{self.can_port}/end_pose" if self.can_port else "end_pose"
    try:
        msg = rospy.wait_for_message(topic, PoseStamped, timeout=1.0)
        return msg.pose
    except rospy.ROSException:
        return None
```

`PiperArmController.move_to()` 内部也是用同样的方式读取 `/end_pose` 验证运动。此处复读一次以获取最新值格式化响应。

### 5.4 位姿格式化

参照 `windows_end_monitor.py` 中的 `_zxz_from_quaternion()` 和 `format_response()` 逻辑，将 `geometry_msgs/Pose` 格式化为：

```
MOVED:upper:x,y,z,qx,qy,qz,qw,alpha,beta,gamma
```

### 5.5 端口选择

为避免与现有 `windows_end_monitor.py`（端口 49300）冲突，使用独立端口。在 `net.yaml` 中新增字段：

```yaml
network:
  ip: "192.168.10.4"
  port: 49300              # windows_end_monitor.py 用
  ctrl_port: 49301         # piper_windows_ctrl_server.py 用（新增）
```

同时在 Windows 端 `piper.yaml` 的 `network` 节同步新增 `ctrl_port: 49301`。

### 5.6 SHUTDOWN 处理

收到 SHUTDOWN 后：
1. 发送 `SHUTDOWN_ACK` 确认
2. 关闭客户端连接
3. 调用 `rospy.signal_shutdown("Windows requested shutdown")` 退出 ROS

---

## 六、Windows 端新建：`test_piper_ctrl.cpp`

### 6.1 与 `get_piper_pose.cpp` 的关键差异

| 项 | get_piper_pose | test_piper_ctrl |
|----|---------------|-----------------|
| 通信模式 | 轮询 `GET_POSE`（每 200ms） | 顺序发送指令，等待响应 |
| 线程模型 | `tcpWorker` 独立线程 | 单线程：send → recv → 显示 → 等按键 |
| 指令集 | `GET_POSE` 只读 | `MOVE_JOINTS` / `MOVE_TO` / `SHUTDOWN` |
| 数据来源 | 无（被动查询） | `cfg/gaze_target/P001/piper_upper.txt` |

### 6.2 结构

```
main():
  1. WSAStartup
  2. 加载 cfg/piper.yaml → ubuntu_ip, ctrl_port
  3. 加载 cfg/capture.yaml → participant_id
  4. 加载 cfg/gaze_target/<pid>/piper_upper.txt → targets_upper
  5. 加载 cfg/gaze_target/<pid>/piper_lower.txt → targets_lower
  6. 读取 cfg/gaze_target/<pid>/sentry.txt → 恢复进度 (upper_idx, lower_idx)
  7. socket() + connect() 到 ubuntu_ip:ctrl_port
  8. Upper 回零: MOVE_JOINTS:upper → 等待 MOVED → 记录进度
  9. Lower 回零: MOVE_JOINTS:lower → 等待 MOVED → 记录进度
  10. 进入 drawUI 循环 (单线程):
      - 当前臂: "upper" (初始)，arm_idx = sentry 恢复的值
      - 渲染状态：臂名称、进度 N/M、指令/实际位姿、距离
      - [空格] → 发送 MOVE_TO → 接收响应：
          MOVED → arm_idx++ → updateSentry()
          ERROR:no_solution → arm_idx++ (跳过) → updateSentry()
          → 若当前臂全部完成 → 提示 "UPPER/LOWER DONE"
      - [t]   → 切换臂 → 目标臂回零 → 恢复该臂进度 → updateSentry()
      - [R]   → 当前臂重新回零
      - 两个臂都完成 → "ALL DONE"
      - [ESC]/[q] → 双回零 → SHUTDOWN → 退出
  11. closesocket, WSACleanup
```

### 6.3 TCP 辅助函数（复用自 get_piper_pose.cpp）

```cpp
// 发送一行（自动追加 \n）
bool sendLine(SOCKET sock, const string& msg) {
    string data = msg + "\n";
    return send(sock, data.c_str(), (int)data.length(), 0) > 0;
}

// 接收一行（复用 get_piper_pose.cpp 的 recvLine 实现）
bool recvLine(SOCKET sock, string& line, int timeout_ms = 10000);
```

### 6.4 解析 MOVED 响应

```
"MOVED:upper:0.299,0.101,0.198,0.123,0.456,0.789,0.012,45.0,30.0,10.0"
```

按 `:` 分割 → 取第三段 → 按 `,` 分割 → 得 10 个 double：
`[x, y, z, qx, qy, qz, qw, alpha, beta, gamma]`

复用 `get_piper_pose.cpp` 中的 `parsePoseResponse()`，只需将前缀从 `"POSE:"` 改为 `"MOVED:"`，或新增一个通用的 `parseMovedResponse()`。

### 6.5 OpenCV UI 布局

```
┌─────────────────────────────────────────────────────┐
│  Piper Upper Arm Control                            │
│                                                     │
│  Target:  #3 / 8                                    │
│  Command: (0.300, -0.100, 0.200)                    │
│  Actual:  (0.299, -0.101, 0.198)                    │
│  Z-X-Z':  α=44.5° β=30.1° γ=0.0°                   │
│  Status:  OK  (dist=0.002m)                         │
│                                                     │
│  [SPACE] next target  [R] re-zero  [ESC/Q] quit     │
└─────────────────────────────────────────────────────┘
```

纯文字叠加在空白画布上，不需要缩略图。

### 6.6 配置文件加载

```cpp
namespace fs = std::filesystem;
auto yp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
           / "cfg" / "piper.yaml").string();
Cfg cfg(yp);
string ubuntu_ip = cfg["network"]["ubuntu_ip"].as<string>();
int    ctrl_port = cfg["network"]["ctrl_port"].as<int>();
```

### 6.7 目标点位文件加载

文件位于 `cfg/gaze_target/P001/`，每个臂一个 txt 文件，每行一个 `"x, y, z"` 点位。启动时加载全部行，依次执行。

```
cfg/gaze_target/P001/piper_upper.txt:
  0.3, 0.1, 0.2
  0.3, -0.1, 0.2
  ...（8 个点）

cfg/gaze_target/P001/piper_lower.txt:
  0.3, 0.1, 0.2
  ...（8 个点）
```

```cpp
auto loadTargets = [](const string& path) -> vector<array<double,3>> {
    vector<array<double,3>> out;
    ifstream in(path);
    string line;
    while (getline(in, line)) {
        stringstream ss(line); string token;
        array<double,3> pt{};
        for (int i = 0; i < 3; ++i) {
            if (!getline(ss, token, ',')) break;
            try { pt[i] = stod(token); } catch (...) { break; }
        }
        out.push_back(pt);
    }
    return out;
};
auto targets_upper = loadTargets("cfg/gaze_target/P001/piper_upper.txt");
auto targets_lower = loadTargets("cfg/gaze_target/P001/piper_lower.txt");
```

### 6.8 sentry 维护机制

sentry 由两个独立变量组成：`upper_idx` 和 `lower_idx`，分别记录 **upper 臂**和 **lower 臂**各自已成功执行的目标点位数量（即下一帧的索引）。例如 `upper_idx=3` 表示 upper 臂已完成前 3 个点位，下一次按空格将从第 4 个（index=3）开始执行。

两个臂的进度完全独立——切换臂时各自保留自己的索引，互不影响。

#### 6.8.1 存储格式

sentry 以纯文本文件 `sentry.txt` 存储在 `cfg/gaze_target/<participant_id>/` 目录下。格式为两行 `key:value`：

```
upper:<已完成帧数>
lower:<已完成帧数>
```

选择纯文本的原因：读写极其轻量（每次点位完成后写一次，约 20 字节），无需 HDF5 的复杂性。

#### 6.8.2 启动时初始化

程序启动后，读取 `sentry.txt`。若文件不存在（首次运行），两个臂都从索引 0 开始：

```cpp
string sentry_path = gaze_dir + "/sentry.txt";
int upper_idx = 0, lower_idx = 0;
ifstream sf(sentry_path);
if (sf) {
    string line;
    while (getline(sf, line)) {
        if (line.rfind("upper:", 0) == 0) upper_idx = stoi(line.substr(6));
        if (line.rfind("lower:", 0) == 0) lower_idx = stoi(line.substr(6));
    }
}
```

若 `upper_idx >= targets_upper.size()`，则该臂标记为已完成（`upper_done = true`）。同理处理 lower。

#### 6.8.3 更新时机

`updateSentry()` 在以下**每一个事件后立即调用**，确保异常退出不丢进度：

| 事件 | 对 sentry 的操作 | 代码路径 |
|------|-----------------|----------|
| **MOVE_TO 成功** | `armIdx()++` 然后 `updateSentry()` | SPACE 键 → 收到 `MOVED:` 响应 |
| **MOVE_TO 无解** | `armIdx()++`（跳过此点）然后 `updateSentry()` | SPACE 键 → 收到 `ERROR:no_solution` |
| **MOVE_JOINTS 成功** | 不改变索引，但 `updateSentry()` 确保持久化 | `zeroArm()` 内部、`t` 切换臂、`R` 回零、`ESC` 退出前 |
| **按 C 清除** | `upper_idx=0, lower_idx=0` 然后 `updateSentry()` | `c` 键 |
| **按 R 回零** | 仅 `zeroArm()`，**不改变索引**，不写 sentry | `r` 键 |
| **全部完成** | 标记 `armDone()=true` 然后 `updateSentry()` | SPACE 键 → 当前臂无下一目标 |

**注意**：`updateSentry()` 始终写入 `upper_idx` 和 `lower_idx` 的当前值，不论当前激活的是哪个臂。例如当前在 upper 臂执行，`lower_idx` 保持上次的值不变，一并写入 sentry。

#### 6.8.4 upper 臂的 `upper_idx` 维护

| 条件 | 操作 |
|------|------|
| **程序启动** | 从 `sentry.txt` 读取 `upper:N`；若文件不存在则 `upper_idx=0` |
| **按下空格（upper 激活）** | `MOVE_TO:upper:x,y,z` → 成功后 `upper_idx++` → `updateSentry()`；若无解 `upper_idx++`（跳过）→ `updateSentry()` |
| **按下 R（upper 激活）** | `zeroArm("upper")` → 仅物理回零，`upper_idx` 和 sentry **不变** |
| **按下 T 切换到 lower** | `zeroArm("upper")`（park 当前臂）→ 切换到 lower，`upper_idx` **保持不变** |
| **按下 T 从 lower 切回 upper** | 从 sentry 恢复 `upper_idx`（已有值），继续执行 |
| **按下 C** | `upper_idx=0` → `updateSentry()` |
| **upper 全部完成** | `upper_done=true`，`upper_idx` 不再递增 |
| **程序崩溃/强杀** | `updateSentry()` 已在上次事件后执行，进度保留到最近一次成功写入 |

#### 6.8.5 lower 臂的 `lower_idx` 维护

与 upper 完全对称的逻辑：

| 条件 | 操作 |
|------|------|
| **程序启动** | 从 `sentry.txt` 读取 `lower:N`；若文件不存在则 `lower_idx=0` |
| **按下空格（lower 激活）** | `MOVE_TO:lower:x,y,z` → 成功后 `lower_idx++` → `updateSentry()`；若无解 `lower_idx++`（跳过）→ `updateSentry()` |
| **按下 R（lower 激活）** | `zeroArm("lower")` → 仅物理回零，`lower_idx` 和 sentry **不变** |
| **按下 T 切换到 upper** | `zeroArm("lower")`（park 当前臂）→ 切换到 upper，`lower_idx` **保持不变** |
| **按下 T 从 upper 切回 lower** | 从 sentry 恢复 `lower_idx`（已有值），继续执行 |
| **按下 C** | `lower_idx=0` → `updateSentry()` |
| **lower 全部完成** | `lower_done=true`，`lower_idx` 不再递增 |
| **程序崩溃/强杀** | `updateSentry()` 已在上次事件后执行，进度保留到最近一次成功写入 |

#### 6.8.6 切换臂时的回零策略

按下 `t` 时，**先对当前臂执行 MOVE_JOINTS 回零（park），再切换到目标臂**。无论回零是否成功都强制切换（超时也不阻塞）：

```
[t] 按下
  → 若目标臂已完成 → 跳过
  → zeroArm(当前臂)   ← park 当前臂，放回安全位置
  → arm = 目标臂
  → 目标臂的 armIdx 保持 sentry 中记录的值（不受切换影响）
```

设计理由：离开一个臂时将其归零，防止机械臂停留在上次运动的目标位置（可能不安全）。

#### 6.8.7 退出时的回零策略

按下 `ESC/q` 时，**无论是否全部完成，两个臂都执行回零**，然后再发送 SHUTDOWN：

```
[ESC] 按下
  → zeroArm("upper") → 打印 OK/FAIL
  → zeroArm("lower") → 打印 OK/FAIL
  → sendLine("SHUTDOWN")
  → 接收 SHUTDOWN_ACK
  → 退出
```

回零在 SHUTDOWN 之前执行，确保两臂都回到安全零位后再关闭 Ubuntu 端服务。

#### 6.8.8 异常恢复

| 场景 | `upper_idx` | `lower_idx` | 说明 |
|------|------------|------------|------|
| 程序正常退出 | 保持 | 保持 | `updateSentry()` 已在最后事件时写入 |
| 程序崩溃/强杀 | 保持 | 保持 | sentry 在上次事件时已写入，最多丢失**最后一次**操作 |
| 网络断开 | 保持 | 保持 | Windows 端退出不会回滚 sentry |
| Ubuntu 端崩溃 | 保持 | 保持 | TCP 超时 → `zeroArm` 失败 → Windows 端状态不变 |
| 按 C 清除 | 归零 | 归零 | 手动重置，相当于首次运行 |


#### 6.8.7 退出时的回零策略

按下  时，**无论是否全部完成，两个臂都执行回零**，然后再发送 SHUTDOWN：



回零在 SHUTDOWN 之前执行，确保两臂都回到安全零位后再关闭 Ubuntu 端服务。

#### 6.8.8 异常恢复

| 场景 |  |  | 说明 |
|------|------------|------------|------|
| 程序正常退出 | 保持 | 保持 |  已在最后事件时写入 |
| 程序崩溃/强杀 | 保持 | 保持 | sentry 在上次事件时已写入，最多丢失**最后一次**操作 |
| 网络断开 | 保持 | 保持 | Windows 端退出不会回滚 sentry |
| Ubuntu 端崩溃 | 保持 | 保持 | TCP 超时 →  失败 → Windows 端状态不变 |
| 按 C 清除 | 归零 | 归零 | 手动重置，相当于首次运行 |

---

### 6.9 末端工具在 CCS 中的位置显示

#### 坐标系链

从 TCP 收到的 flange 位姿到末端工具在相机坐标系 (CCS) 中的位姿，经过两层变换：

```
Flange (来自 /end_pose)
  │
  ├─ T_tool_in_flange: tool.translation + tool.rotation_zxz
  │     R_offset = zxzToQuat(tool_rot_zxz)
  │     p_tool_arm = p_flange + R_flange * R_offset * tool_trans
  │     R_tool_arm = R_flange * R_offset
  │
  └─ 末端工具在 Arm Base 坐标系中 (tool_in_arm)
       │
       ├─ T_arm_in_ccs: arm_in_ccs.translation + arm_in_ccs.rotation_zxz
       │     R_arm_ccs = zxzToQuat(arm_rot_zxz)
       │     p_tool_ccs = arm_trans + R_arm_ccs * p_tool_arm
       │     R_tool_ccs = R_arm_ccs * R_tool_arm
       │
       └─ 末端工具在 CCS 中 (返回值)
```

即 `T_tool_ccs = T_arm_in_ccs * T_flange * T_tool_in_flange`。

#### 数学实现

复用 `eyetracker::piper` 库中已有的 `armToolToCamPose()`（`utils/piper/src/piper.cpp:102-109`）：

```cpp
Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg);
```

**注意**：`piper` 库中 `Quat` 的字段顺序是 `{x, y, z, w}`（w 在最后），而 ROS `/end_pose` 和 TCP 响应中的四元数顺序是 `(qx, qy, qz, qw)`。传入 `armToolToCamPose` 前无需重排——两者的 x/y/z/w 字段顺序恰好一致。

#### 配置读取

从 `piper.yaml` 读取两个臂的变换参数（启动时加载，切换臂时自动切换）：

```yaml
arms:
  upper:
    tool:
      translation: [0.0, 0.0, 0.02]                  # 米, 法兰盘坐标系
      rotation_zxz: [0.0, 0.0, 0.0]                 # 度, 内旋 Z-X-Z''
    arm_in_ccs:
      translation: [-0.0091, 0.0224, -0.1684]        # 米
      rotation_zxz: [0.0948, 89.4077, 86.6415]      # 度, 内旋 Z-X-Z''
  lower:
    tool: ...
    arm_in_ccs: ...
```

```cpp
struct ArmTransform { Pt3 tool_t, tool_r, ccs_t, ccs_r; };
ArmTransform xf_upper, xf_lower;

// 加载 (以 upper 为例):
xf_upper.tool_t = readPt3(cfg["arms"]["upper"]["tool"]["translation"]);
xf_upper.tool_r = readPt3(cfg["arms"]["upper"]["tool"]["rotation_zxz"]);
xf_upper.ccs_t  = readPt3(cfg["arms"]["upper"]["arm_in_ccs"]["translation"]);
xf_upper.ccs_r  = readPt3(cfg["arms"]["upper"]["arm_in_ccs"]["rotation_zxz"]);
```

#### 计算与显示

每次收到 `MOVED` 响应后立即计算：

```cpp
// flange pose 来自 MOVED 响应解析
Pose flange{{mp.x, mp.y, mp.z}, {mp.qx, mp.qy, mp.qz, mp.qw}};
auto& xf = (arm == "upper") ? xf_upper : xf_lower;
Pose tool_ccs = armToolToCamPose(flange, xf.tool_t, xf.tool_r, xf.ccs_t, xf.ccs_r);
// 显示: tool_ccs.pos.x, tool_ccs.pos.y, tool_ccs.pos.z
```

UI 在现有 flange 位姿行之后新增一行：

```
Tool in CCS (m): [ 0.1234,  0.0567,  0.8901]
```

#### CMake 依赖

`test_piper_ctrl` 当前只链接 `eyetracker::cfg` + `OpenCV`。需新增 `eyetracker::piper`：

```cmake
target_link_libraries(test_piper_ctrl
    PRIVATE
        eyetracker::piper
        eyetracker::cfg
        ${OpenCV_LIBS}
)
```

---

## 七、CMake 改造

### 7.1 `tests/utils/CMakeLists.txt`

```cmake
add_subdirectory(cam)
# add_subdirectory(cam_calib)
add_subdirectory(piper)          # ← 取消注释
# add_subdirectory(glint_detection)
```

### 7.2 `tests/utils/piper/CMakeLists.txt`

在 `get_piper_pose` target 之后追加：

```cmake
add_executable(test_piper_ctrl
    test_piper_ctrl.cpp
)

target_link_libraries(test_piper_ctrl
    PRIVATE
        eyetracker::cfg
        ${OpenCV_LIBS}
)

message(STATUS "test_piper_ctrl will be built")
```

无需 HDF5、Pylon、Ceres。`ws2_32.lib` 由源文件中的 `#pragma comment` 处理。

---

## 八、验证步骤

### 8.1 Ubuntu 端

```bash
# 终端 1：启动 MoveIt + piper 驱动
roslaunch piper_no_gripper_moveit demo.launch can_port:=piper_upper

# 终端 2：启动运动控制服务（新增脚本）
rosrun moveit_ctrl piper_windows_ctrl_server.py
```

> `windows_end_monitor.py` 无需运行——`piper_windows_ctrl_server.py` 是独立的，直接读取 `/piper_upper/end_pose` 主题获取位姿。

### 8.2 Windows 端

```powershell
cd cpp_eyetracker
cmake --preset vs2022-vcpkg -B build
cmake --build build --config Release --target test_piper_ctrl
.\build\tests\utils\piper\Release\test_piper_ctrl.exe
```

### 8.3 测试流程

1. Ubuntu 日志输出 `"piper_ctrl_server listening on 192.168.10.4:49301"`
2. Windows 连接后自动发送 `MOVE_JOINTS` → 机械臂回零，响应 `MOVED:upper:...`
3. Windows 显示 `"Command (0.300, 0.100, 0.200)"`
4. 按空格 → Windows 发送 `MOVE_TO:upper:0.3,0.1,0.2`
5. Ubuntu：机械臂运动，响应 `MOVED:upper:0.299,...`
6. Windows：显示实际位置与目标距离
7. 按 ESC → Windows 发送 `SHUTDOWN` → Ubuntu 响应 `SHUTDOWN_ACK` 并退出

---

## 九、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | 新建 `piper_windows_ctrl_server.py`：独立 TCP 服务，集成 `PiperArmController`，处理 `MOVE_JOINTS`/`MOVE_TO`/`SHUTDOWN`，从 `/end_pose` 读取位姿 | 15min |
| 2 | 在 `net.yaml` 和 `piper.yaml` 中新增 `ctrl_port: 49301` | 2min |
| 3 | 创建 `test_piper_ctrl.cpp`：基于 `get_piper_pose.cpp` 改造为单线程指令模式 | 15min |
| 4 | 取消注释 `tests/utils/CMakeLists.txt` 中的 `add_subdirectory(piper)` + 在 `tests/utils/piper/CMakeLists.txt` 中添加 target | 3min |
| 5 | 编译调试 | 10min |
| 6 | 联调验证（需 Ubuntu 端运行） | 15min |

**总计：~60 分钟**
