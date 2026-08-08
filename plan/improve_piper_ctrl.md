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

### 6.8 进度维护与异常恢复

#### sentry.txt

每次执行完一个点位后，将两个臂的进度写入 `cfg/gaze_target/<participant_id>/sentry.txt`。格式：

```
upper:<已完成帧数>
lower:<已完成帧数>
```

其中数字表示该臂**已成功执行的帧数**（即下一帧的索引）。例如 `upper:3` 表示 upper 已完成前 3 个点位，下次从第 4 个（index=3）开始。

#### 启动时恢复

程序启动时读取 sentry.txt：
- 文件不存在 → 两个臂都从 0 开始
- 文件存在 → 加载进度，从上次中断处继续

```cpp
int upper_idx = 0, lower_idx = 0;
ifstream sf(gaze_dir + "/sentry.txt");
if (sf) {
    string line;
    while (getline(sf, line)) {
        if (line.rfind("upper:", 0) == 0) upper_idx = stoi(line.substr(6));
        if (line.rfind("lower:", 0) == 0) lower_idx = stoi(line.substr(6));
    }
}
arm_idx = (arm == "upper") ? upper_idx : lower_idx;
```

#### 每次写入后更新

```cpp
auto updateSentry = [&]() {
    ofstream sf(gaze_dir + "/sentry.txt");
    sf << "upper:" << upper_idx << "\nlower:" << lower_idx << "\n";
};
```

在每次 MOVE_TO 成功后、MOVE_JOINTS 成功后调用 `updateSentry()`。

#### 切换臂时回零

按下 `t` 切换到另一臂后：
1. 若目标臂未完成 → 先对该臂执行 `MOVE_JOINTS` 回零
2. 回零成功 → 从 sentry 记录的索引恢复该臂的进度
3. 回零失败 → 保持当前臂，提示用户

```
[t] 按下
  → 若目标臂已完成 → 跳过
  → zeroArm(target_arm) → 等待完成
  → 切换到目标臂，arm_idx = saved_idx
  → updateSentry()
```

#### 无解情况处理

当 Ubuntu 返回 `ERROR:<arm>:no_solution` 时，跳过当前点位，自动递增索引，更新 sentry：

```
recvLine 返回 "ERROR:upper:no_solution"
  → 打印警告
  → arm_idx++  (跳过此点)
  → updateSentry()
  → 状态：SKIPPED — upper #N
```

不阻塞等待用户确认，自动继续下一个点位。

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
