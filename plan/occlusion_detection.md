# 遮挡检测计划 v3 — capture_with_occlusion_detection.cpp（臂入画面判定 + valid=0，无 CSV）

## 0. 与 v2 的区别

v2 只输出 CSV 供事后查看。v3 按新要求把判定结果**写进数据集**：
h5 写盘前判定"本次录制对应 gaze target"的遮挡情况，被遮挡相机的
**valid 数据集整段置 0（该目标全部 core_frames=100 帧）**；每录制只判定一次
（臂静止时），判定用的位姿**缓存**（h5 阶段臂已开始移往下一目标）。

**判定口径（v3 定稿）：机械臂进入相机画面即遮挡** —— 把臂网格经内参外参投影，
任一三角形落在该相机图像范围内（且在近平面前方）→ 该相机遮挡。
不再使用"眼→光心精确视线、碰到即遮挡"；**眼位不再参与判定**，
遮挡标志由 内参 K + 外参 R|T + 臂 FK 完全决定。该口径偏保守：
"臂挡住眼"必然属于"臂进入画面"（眼≈画面中心），反之未必 ——
臂只出现在画面边缘、眼区通畅时也计为遮挡。

**坐标系事实（已在 viz_gaze_coverage.py 验证）：**
- 相机标定 XML 的参考系 = `arm_in_ccs` 的 CCS = 40772280（center_cam）相机系，全部同系；
- URDF 链：joint1..6 全部绕 z 轴（revolute），origin xyz/rpy 已知；link1..6 + base_link +
  gripper_base 均有 STL 网格（`piper_description/meshes/*.STL`，scale=1）；
- 关节角来源：ROS 话题 `{can_port}/joint_states_actual`（rad），控制服务器可查询。

## 1. 关键时序（capture_with_M5Stack 现有流程）

```
moveArmToTarget(#N) → ARM_OK   ← 臂静止在目标 #N（判定时机！）
  → READY → SPACE → 录 100 帧(入 RAM)
  → h5 阶段: 子进程写盘 #N 的 100 帧 ∥ arm 线程 moveArmToTarget(#N+1)
             (rec_gaze 在此之前已按同样思路缓存 — line ~1484)
```

- **判定时机**：ARM_OK 后、SPACE 前（臂静止，另一臂停在上一目标）。
  GET_JOINTS × 2 → FK → 20 相机遮挡标志 → **缓存** `g_occ_flags`（属于 #N）。
  **判定同步完成才进入 READY**（遮挡检验阻塞在 ARM_OK → READY 之间，
  失败/停用则直接放行）。
  每录制一次判定，覆盖该目标全部 100 帧（"抽取一帧"代表整段，臂静止故成立）。
  判定内容：两臂（按当前关节角构型）是否进入各相机画面。
- **应用时机**：h5 阶段启动子进程时，把该相机 occluded 标志作为命令行参数传入；
  子进程写 valid 时整段写 0。此时无需再算几何，直接用缓存。
- **跨机同步**：Master 算全部 20 相机（纯几何，无需图像）；ARM_OK 后立即经
  cmd 通道发送 `OCCL:sn1,sn2,...`（被遮挡 SN 列表），Slave 缓存并在自己的
  h5 阶段同样应用。发送远早于 Slave 写盘，无竞态。

## 2. 输入（全部运行时可得）

| 数据 | 来源 | 说明 |
|---|---|---|
| 关节角×6×2 臂 | 控制服务器（新增 `GET_JOINTS` 命令） | 每次判定查询一次（臂静止） |
| URDF | `piper_ros/.../piper_description/urdf/piper_description.urdf` | pugixml 解析（项目已有依赖） |
| STL 网格 | `piper_ros/.../piper_description/meshes/*.STL` | 二进制/ASCII STL 解析器（~50 行） |
| 相机内参 | `{SN}_Data.xml` 的 `InternalParameters/RawData` | `area_scan_division Focus Kappa Sx Sy Cx Cy W H` → K=[[f/Sx,0,Cx],[0,f/Sy,Cy],[0,0,1]]，像面 W×H |
| 相机外参 | 同 XML `ExternalParameters` | gba：`R=Rx(α)Ry(β)Rz(γ)`，`p_ref=R·p_cam+T` |
| 臂基座位姿 | `cfg/arm_pose/{day}.yaml` 的 `arm_in_ccs`（脚本已加载） | 复用现有 zxz 加载代码 |

（末端理想化为**不装任何工具**：遮挡体 = 两臂 URDF 连杆本身，无工装模型；
眼位不参与本判定口径。）

## 3. 算法

```
每次判定（ARM_OK 后，臂静止）:
  1) GET_JOINTS upper/lower → j[6] × 2（连同时间戳一起缓存）
  2) FK: URDF 链 × 关节角 → 各 link 网格顶点 → 世界系（经 arm_in_ccs，
     复用 utils/piper 的 zxzToQuat/composePoses）
  3) 对每相机 i（内参 K_i, 外参 R_i|T_i）:
       像素投影 p_uv = K_i · (R_i⁻¹(p_w − T_i))
       臂任一三角形与"画面范围"相交（三角形先经近平面 z≥ε 裁剪
       ——Sutherland-Hodgman，只保留镜头前方部分；裁剪后多边形任一
       顶点落图内 / 任一边横跨画面边界 / 画面中心在多边形内）
       → occluded[i] = 1
       （P001 教训：曾把"横跨相机平面"一律保守算入画，而臂网格几乎
       横跨每台环形相机的 z≈0 平面 → 17/20 相机全目标误报；现裁剪后
       判定，视场外的跨越不再计入）
  4) 缓存 occluded[]（本目标）→ UI 显示 → Master 发 OCCL 给 Slave
     （判定结果只进 valid 数据集与 UI/报告，不写 CSV）
```

- 实现：先做 link 级快速剔除（link 包围盒四角投影出图 → 整 link 跳过），
  再逐三角形；顶点投影 20 相机 × 2 臂 × ~3 万顶点 ≈ 百万次乘法，毫秒级。
- 规模：2 臂 × 7 link × 数千三角形 × 20 相机，在 READY 等待期执行，
  不占录制/写盘热路径。
- gripper 手指（link7/8，joint7/8）默认**不含**（末端装的是定位工装），常量开关可启用。

## 4. 修改位置与内容

### A. 新文件 `tests/utils/cam/capture_with_occlusion_detection.cpp`（复制 capture_with_M5Stack.cpp）

| # | 位置 | 内容 |
|---|---|---|
| 1 | 文件头 | 更名注释；include pugixml；遮挡开关常量（如 link 膨胀余量 `OCC_MARGIN`） |
| 2 | Piper 区块后新增「Occlusion detection」区块 | `UrdfModel`（joint 链+link 网格）+ `loadUrdf()`/`loadStl()`；`fkLinks(joints)→link 世界系位姿`；`loadCamXmls(dir)`（内参 K + 外参 R\|T 解析）；`triInFrame()`（顶点投影 + 视锥平面测试）；`queryJoints(arm)`（GET_JOINTS 协议）；`runOcclusionCheck()`（臂入画面判定 + 缓存 `g_occ_flags`） |
| 3 | main() 初始化（加载 arm_pose 之后） | 加载 XML/URDF、启动自检（第 5 节）；失败仅警告并停用遮挡（valid 恒写 1，不阻断采集） |
| 4 | `moveArmToTarget()` ARM_OK 分支（返回前） | 调 `runOcclusionCheck()`；Master 额外 `sendLineRaw(cmd_sock, "OCCL:...")` 同步 Slave |
| 5 | cmdWorker（Slave 侧） | 新增 `OCCL:` 消息解析 → 缓存到本机 `g_occ_flags` |
| 6 | h5 阶段子进程参数（line ~1540 args 构造） | 追加 `argv[13] = 本相机 occluded(0/1)`（Master 用自身判定，Slave 用 OCCL 缓存） |
| 7 | UI canvas（READY 画面） | 一行：`Occluded: 3/20 — SN1 SN2 SN3`（红字） |
| 8 | Session report | 汇总每相机被遮挡目标数（20 行表，控制台输出） |

### B. `tests/utils/cam/hdf5_multi_process_child.cpp`（约 3 行）

- argv 新增 `occluded`（0/1）；
- valid 写入：`vector<uint8_t> v_buf(N, occluded ? 0 : 1);`
  （gaze_target 仍照常写 —— 数据集结构不变，仅 valid 置 0）。

### C. `piper_ros/.../scripts/piper_windows_ctrl_server.py`（约 15 行）

- 新增命令 `GET_JOINTS:<arm>`：`wait_for_message(f"{can_port}/joint_states_actual", JointState)`
  → 回复 `JOINTS:<arm>:j1,...,j6`（rad）；命令表注释同步。

### D. `tests/utils/cam/CMakeLists.txt`（3 行）

- 新 target `capture_with_occlusion_detection`（链接 pugixml）。

### E. `tests/utils/cam/test_load_hdf5_frame.cpp` — 遮挡提示（人工验证）

valid=0 即遮挡标记（数据集自带，无需读 CSV）：

| # | 位置 | 内容 |
|---|---|---|
| 1 | `loadFrame()` | 已读 valid（现状 `!valid → N/A`）— 保留数据，新增 `c.occluded = !valid` |
| 2 | render() 缩略图格 | `occluded` 相机：红色边框 + 左上角红色 `OCC` 角标（区别于"未写入"的黑色 N/A） |
| 3 | render() 放大视图 | Frame 信息行追加 `Arm/Target`（帧号→目标映射，复用 `armRecorded()` 口径）与 `VALID/INVALID(occluded)` 徽标 |

验证口径：浏览到某目标帧 → 标红相机画面里应能看到机械臂本体挡在眼区前方；
未标相机眼区应通畅。

## 5. 一致性自检（关键验证，启动时 + 首目标各一次）

1. **FK ↔ MOVED 对账**：URDF FK 的 flange 位姿（世界系）与服务器 `MOVED`
   回报（经 armToolToCamPose）位置差 < 1 cm，否则告警停用遮挡（valid 恒 1）；
2. **FK 工具尖 ↔ 目标点**：FK 定位工装尖 vs 当前 gaze target（CCS）距离 < 1 cm；
3. 全零关节角下遮挡数应为 0 或极少（臂折叠于基座附近，远离各相机视锥）。

两条对账同时通过才启用遮挡输出，保证 URDF/FK/坐标系链整体正确。

## 6. 已知局限

- **口径偏保守**：臂出现在画面任何角落即遮挡，即使眼区视野通畅；
  反之"臂挡眼"必然伴随"臂入画面"，不会漏报真正的视线遮挡；
- 只判定"到位后静止"状态（录制只发生在静止期，与判定一致）；
- 末端理想化为无工具；线缆/支架等其他遮挡物不在模型内；
- 关节角精度 = 编码器反馈精度（远小于网格尺寸，可忽略）；
- 若遮挡功能因自检失败停用，valid 全 1（与现状一致，安全回退）。

## 7. 实施顺序

1. C：服务器加 GET_JOINTS（先打通数据源，手工 telnet 验证）；
2. D：CMake 占位 + 复制源文件；
3. A-2/A-3：URDF/STL/XML 加载 + FK + 眼位 + 自检对账（第 5 节）；
4. B：子进程 occluded 参数 + valid 写 0；
5. A-4..A-8：判定接入 ARM_OK + OCCL 同步 + 子进程传参 + UI + 报告；
6. E：viewer 遮挡提示；
7. 联调：空跑一轮（臂+服务器即可）核对 UI 与 valid；
   正式采集后用 E 浏览实拍帧，人工抽验遮挡标记与画面一致。

## 后续排查 (2026-09-14): 下方相机漏检下臂遮挡 — arm_pose z 钳制

**症状**: 40768743 / 40772277 (人脸下方相机) 漏检遮挡 (0013.h5 frame1800
下臂横挡半脸但 valid=1); 其余 18 台正常。

**排查链** (viz_occ_overlay.py 投影叠加 + 亮区统计 + 10 机三角化 + 扰动扫描):
1. 图像实测遮挡区 v≈849-1507, 模型下臂投影最深 v=-90 (画面外) — 垂直偏差 ~1600px
2. 相机外参排除: 人脸亮区质心多机三角化, 问题相机视线残差与正常机同量级
3. FK/上臂排除 (与其他相机共享); 扰动扫描命中: lower arm_pose **z+0.10m**
   单轴修正 → 投影 v=+1064 落入遮挡区, 且对照相机无误报
4. **根因 (用户确认)**: lower `arm_in_ccs.translation.z` 的标定搜索范围被限制在
   [-0.30, -0.10], 结果被钳制在边界 -0.100 (整齐圆数即边界值的信号);
   真实值 ≈ 0

**选择性分布的解释**: 下臂位于人脸下方, 只有朝上看的下方相机画面里可见 —
z 偏 10cm 只影响这两台; 其余 18 台看不见下臂, 故"90% 正常"。

**修复**: 放开 z 搜索范围, 重跑 lower 臂手眼标定 (test_record_arm_data 下臂流程)。
**教训**: ① 标定输出的整齐圆数 = 被钳制/手填的信号, 一眼可疑; ② 选择性失效
分布指向"只有部分传感器可见的量" (此处=下臂); ③ lower 的 zxz b≈90.5° 贴万向锁
奇异区, 重标定时留意欧拉角数值稳定性。

诊断工具: `tests/utils/cam/viz_occ_overlay.py` (投影叠加, 分类统计命中/边距带/
跨近平面, 支持扰动对比)。

## 录制顺序可翻转 (2026-09-19, capture.cpp)

**需求**: 平衡系统误差, 支持从 lower 开始录制 (lower250→upper250 或
upper250→lower250 两种纯两段式顺序; 交错段不在工况内出现)。

**实现** (capture.cpp = capture_with_occlusion_detection.cpp 副本, 新 CMake target):
- `armRecorded()` 对称翻转: `(a == g_first_arm) ? min(tot, 配额) : max(0, tot-配额)`
  — OVER/右上角/断点续录/z 回退全部调用者自动一致, 调用点零改动
- 顺序自证 `detectFirstArm()`: chunk0 `occ_meta[0][0]` (occ_arm 0/1) 即第一录
  的臂; 数据为空 → 当前 g_arm。**零新增持久状态, 完全依赖 h5**
- init 按 t 切臂 (原有能力, 守卫未动): 空数据时切换即定下顺序
  (`h5FramesWritten()==0` → `g_first_arm=新臂`); 已有数据则顺序不变 (历史事实)
- 右上角两行加 `>` 前缀标记当前活动臂

**边界** (与旧版一致): 中途未录满切臂后继续录 = 交错段, h5 帧区间映射失准
— 调试用; 正式录制前 z 回退或清盘恢复两段式。

测试: num_targets_per_arm=5 验证 lower-first 全流程 + 断点续录顺序自证。

## 录制顺序翻转 — 收官 (2026-09-20)

lower-first 双机验证通过。最终机制 (三层, h5 事实最高权威):
1. master 主动同步: `PIPER:first:<arm>` 消息 (随 syncPiperToSlave 全时机携带)
2. slave 自愈: 第一录落盘后 detectFirstArm() 读本地 chunk0 occ_arm,
   一次即定 (g_first_checked) — 不依赖 master 部署版本/消息到达
3. 优先级: sync 的 first 是 master 预判 (可能基于空数据), 不覆盖已自证的
   h5 事实

排障记录: 首轮 slave 仍 UPPER 递增 = master 未部署新 exe (不发 first 消息)。
教训: 双机系统改协议/加消息时, 两台必须同时重编 — 本次新增的 slave 自愈层
已把此类部署错位变成无害。
