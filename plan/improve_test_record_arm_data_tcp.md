# test_record_arm_data.cpp — TCP 通信改造 (Master ↔ Ubuntu piper_ctrl)

> 现状：`test_record_arm_data.cpp` 用裸 `send/recv` 直连 `windows_end_monitor.py` (port 49300)，
> 仅发 `GET_POSE:<arm>` 查询位姿。无握手、命令/响应无结构化封装。
>
> 目标：改为与 `piper_windows_ctrl_server.py` (ctrl_port 49301) 通信，
> Ubuntu 端新增 GET_POSE 请求；初始化时先握手，然后初始化相机。
> （无回零需求）

---

## 一、Ubuntu 端：`piper_windows_ctrl_server.py` 新增命令

### 1.1 新增 `READY` 握手 + `GET_POSE:<arm>` 位姿查询

```python
def _process(self, cmd):
    if cmd.strip() == "READY":
        # 握手：客户端确认服务端就绪
        return "ACK"

    elif cmd.startswith("GET_POSE:"):
        # "GET_POSE:upper" → 查询当前 flange 位姿（不移动）
        _, arm = cmd.split(":", 1)
        if arm not in self._ctrls:
            return f"ERROR:{arm}:unknown arm (available: {list(self._ctrls.keys())})"
        ctrl = self._ctrls[arm]
        pose = self._get_current_flange_pose(ctrl.can_port)
        if pose is None:
            return f"ERROR:{arm}:no pose data"
        x, y, z, qx, qy, qz, qw = pose
        alpha, beta, gamma = _zxz_from_quaternion(qx, qy, qz, qw)
        return (f"POSE:{arm}:"
                f"{x:.6f},{y:.6f},{z:.6f},"
                f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},"
                f"{alpha:.4f},{beta:.4f},{gamma:.4f}")

    # ... existing MOVE_JOINTS / MOVE_TO / SHUTDOWN unchanged
```

要点：
- `POSE:` 与 `MOVED:` 都是 10 个逗号分隔值 `x,y,z,qx,qy,qz,qw,alpha,beta,gamma`（与 `windows_end_monitor.py` 的格式兼容，旧解析逻辑可复用）
- `READY → ACK` 与 test_calib_images 的握手语义一致（客户端发 READY，服务端回 ACK）

### 1.2 文档字符串更新

```
Supported commands:
    READY                                                   # handshake → ACK
    MOVE_JOINTS:<arm>:<j1>,...,<j6>                         # joint-space move (rad)
    MOVE_TO:<arm>:<x>,<y>,<z>                               # Cartesian move (m)
    GET_POSE:<arm>                                          # query current flange pose
    SHUTDOWN                                                # exit server
```

---

## 二、Windows 端：`test_record_arm_data.cpp`

### 2.1 配置变更 (`calib_arm.yaml`)

```yaml
network:
  ubuntu_ip: "192.168.10.4"
  ctrl_port: 49301                       # piper_windows_ctrl_server.py（原 port: 49300 废弃）
```

代码：`g_arm_port = arm_cfg["network"]["ctrl_port"].as<int>();`

### 2.2 TCP 辅助函数（从 test_piper_ctrl.cpp 内联）

```cpp
bool recvLine(SOCKET sock, string& line, int timeout_ms);
bool sendLine(SOCKET sock, const string& msg);   // send + "\n"
struct ArmPose { double x,y,z, qx,qy,qz,qw, alpha,beta,gamma; bool valid; };
bool parsePoseResponse(const string& resp, string& arm, ArmPose& pose);
```

`parsePoseResponse` 同时接受 `MOVED:` 和 `POSE:` 前缀（两者格式一致）。

### 2.3 替换 `queryFlangePose()`

```cpp
// 旧：裸 send/recv，GET_POSE 到 49300
string queryFlangePose(const string& arm);

// 新：结构化 sendLine/recvLine
ArmPose queryFlangePose(const string& arm) {
    lock_guard<mutex> lock(g_arm_mtx);
    if (g_arm_sock == INVALID_SOCKET) return {};
    sendLine(g_arm_sock, "GET_POSE:" + arm);
    string resp; 
    if (!recvLine(g_arm_sock, resp, 3000)) { cerr << "[Arm] GET_POSE timeout" << endl; return {}; }
    string resp_arm; ArmPose pose;
    if (parsePoseResponse(resp, resp_arm, pose)) return pose;
    cerr << "[Arm] GET_POSE error: " << resp << endl;
    return {};
}
```

调用点（SPACE 拍照后）改为使用 `ArmPose` 结构，简化解析逻辑。

### 2.4 初始化顺序（核心改动）

当前顺序：读配置 → 打印配置 → `create_directories` → 连接 Ubuntu → 建 mapping 文件 → 创建相机上下文 → 启动相机线程 → UI

**新顺序**：

```
1. 读配置 (calib_arm.yaml + capture.yaml)
2. 打印配置
3. create_directories + mapping 文件
4. ── TCP 连接 Ubuntu (ctrl_port) ──
5. 握手：sendLine("READY") → recvLine == "ACK" ?
   ├─ 失败 → 打印错误，继续（无 arm 功能）
6. ── 初始化相机（PylonInitialize + 相机线程）──
7. UI 循环
```

### 2.5 退出时

保持现状（断开 socket），**无回零**。

---

## 三、实施步骤

| 序号 | 内容 | 预计 |
|------|------|------|
| 1 | Ubuntu: `piper_windows_ctrl_server.py` 添加 READY + GET_POSE 命令 | 10min |
| 2 | `calib_arm.yaml`: `port: 49300` → `ctrl_port: 49301` | 1min |
| 3 | `test_record_arm_data.cpp`: 内联 recvLine/sendLine/parsePoseResponse | 10min |
| 4 | 替换 queryFlangePose + 调整初始化顺序（握手 → 相机初始化） | 10min |
| 5 | 编译 + 实机验证（Ubuntu 端先启动 ctrl server） | 10min |

**总计：~40 分钟**

---

## 四、验证清单

1. Ubuntu 启动 `piper_windows_ctrl_server.py`
2. Windows 启动 `test_record_arm_data.exe`：
   - 控制台打印 handshake OK
   - 握手完成后相机开始初始化
3. SPACE 拍照：GET_POSE 返回 POSE:...，mapping 文件写入位姿
4. 对比之前 49300 端口协议：POSE 格式一致，mapping 文件格式不变
