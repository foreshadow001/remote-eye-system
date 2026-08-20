# gen_gaze_target.py — Gaze Target 随机生成脚本

## Context

采集流程需要每台机械臂一组 gaze target 点（受试者注视点），由 Windows 端程序
（capture_with_M5Stack / capture_with_LED / capture_with_piper / test_piper_ctrl）
读取并逐点移动机械臂。目前这些点需要手工生成。本脚本按以下要求自动生成：

1. 随机生成（均匀分布）；
2. 点在**安全可达范围**内 —— 以 workspace_analyzer.py 输出的可达点阵为真值，
   编写一个判断函数过滤不可达点；
3. 第二阶段重排顺序，使相邻两个目标距离 ≤ max_distance（0.2 m）；
4. 默认每臂 **500** 个点，实际可能只使用前 250 个 —— 必须保证**前 250 与
   后 250 分布大致一致**；
5. 生成完毕后可视化。

## 已确认的事实（探索结论）

| 事实 | 结论 |
|------|------|
| 可达点阵格式 | `reachable_range/points_piper_{arm}*.txt`，空格分隔 `x y z`（3 位小数），格距 0.05 m（upper 共 1166 点，lower 文件同样存在） |
| 输出消费格式 | C++ `loadTgts` 按 **逗号** 分隔解析 → 输出必须为 `x,y,z` 每行一点 |
| 输出路径 | `cpp_eyetracker/cfg/gaze_target/{participant_id}/piper_upper.txt` + `piper_lower.txt` |
| 消费方 | capture_with_M5Stack.cpp / capture_with_LED.cpp / capture_with_piper.cpp / test_piper_ctrl.cpp 均读该目录 |
| sentry.txt | 同目录，行格式 `upper:N` / `lower:N`（进度恢复用；capture_with_M5Stack.cpp:423 写出格式确认） |
| sampling box | **复用** `workspace_analysis.sampling_box`（per-arm yaml 各自读取，不改 yaml） |
| 可达点阵生成 | workspace_analyzer.py `save_results()`；文件名 `n3` = 3 个障碍物（`n{len(obstacles)}`），与格距无关 |
| 坐标空间 | 机械臂基座坐标系，与 workspace_analyzer 点阵同一空间（无需变换） |
| Python 环境 | 本机 Anaconda：python 3.13.9 / numpy 2.3.5 / PyYAML 6.0.3 ✓；可视化须 `MPLBACKEND=TkAgg`（Qt 后端在本机不可用，DEV_GUIDE 约定） |

## 文件

| 文件 | 操作 |
|------|------|
| `cpp_eyetracker/tests/utils/piper/gen_gaze_target.py` | **新建**（约 200 行，与 viz_piper_chain.py 同目录惯例） |
| `cpp_eyetracker/tests/utils/piper/viz_gaze_target.py` | **新建**（约 150 行，独立可运行，gen 脚本结束前自动拉起） |

不修改任何 C++ 代码与 yaml。依赖：仅标准库 + numpy + PyYAML + matplotlib（均已安装）。

## 设计

### CLI

```
python gen_gaze_target.py [--participant P001] [--num 500]
                          [--edge 0.025] [--max-dist 0.2] [--seed <int>]
                          [--arms upper lower] [--no-viz]
```

- 默认生成 upper + lower 两臂（各自的 yaml + 点阵文件都必须存在）
- `--seed` 默认随机并打印实际种子（可复现）；确定性 seed 便于重跑
- 默认生成完毕后自动启动 `viz_gaze_target.py`（阻塞至窗口关闭）；`--no-viz` 跳过
- 路径推导（不硬编码盘符）：
  - repo 根 = `Path(__file__).resolve().parents[4]`（remote-eye-system）
  - piper_ros = `parents[5] / "piper_ros"`（new_dataset 下）
  - 输出目录 = `parents[3]`（cpp_eyetracker）`/ cfg / gaze_target / {participant_id}`

### 流程

**Step 0 — 加载（per arm）**
- 读 `.../scripts/cfg/piper_{arm}.yaml` 的 `workspace_analysis.sampling_box`
- glob `.../scripts/reachable_range/points_piper_{arm}*`；多个匹配取 **mtime 最新** 并警告；不存在 → 报错退出
- 加载点阵为 numpy 数组（N×3）
- **推断格距 r**：点阵坐标最小正差值（当前数据为 0.05）

**Step 1 — 随机采样 + 可达性过滤（方案 A：腐蚀判据）**
- gen box = sampling_box 每侧内缩 `edge`（0.025）—— 只防采样盒边界，
  可达区域内部的不规则边界由下面的腐蚀判据处理
- 各轴均匀随机采样候选点，接受条件 `is_reachable(p)`，拒绝则重采
- **预处理（点阵腐蚀）**：仅保留"内部格点"——6 个轴向相邻格点全部存在且可达
  的格点（坐标取 3 位小数四舍五入为字典键做邻域查询）；任何邻域含不可达/缺失
  格点的边界格点全部剔除。腐蚀宽度 ≈ 1 格 = 5 cm
- `is_reachable(p)`：`min_dist(p, 内部格点集) ≤ r·√3/2 + 1e-3`
  - 不误杀：可达区域内部点的所在胞体 8 个顶点必为内部格点，胞内点到顶点
    最大距离恰为胞体对角线之半 `r·√3/2 ≈ 0.043`（r=0.05）
  - 不误收：距真实可达边界 < ~1 格的点，其最近格点必为边界格点（已被剔除），
    判据拒绝 → 原方案在边界外 ~4.3 cm 的误收壳层被消除
- 实现：numpy 向量化（分批广播求 min 距离；点阵仅 ~1k 点，无 scipy 依赖）
- 重试上限 `200 × num`，超出则报错并打印接受率（提示可达区域过小/num 过大；
  薄于 2 格的区域会被整体拒绝并由此显式暴露）
- 残余风险兜底：采集端对 MOVE_TO 失败已有 `ERROR:no_solution` 跳过处理
- 一条连续采样流采样 `num` 个点，**按接受顺序切分**：
  前半 = 前 `num//2` 个，后半 = 剩余

**Step 2 — 排序（相邻距离 ≤ max_dist）—— 前后两半分布一致的核心逻辑**

1. **分布一致在采样端保证**：前后两半来自同一条 i.i.d. 均匀采样流
   （拒绝采样无记忆性，两半独立同分布）—— 任何截断偏置在源头即不存在，
   排序阶段永不跨半重新划分点；
2. **两组各自独立排序，共用同一套简单递归（chain_pts）**：
   - 小盒（包围盒对角线 ≤ max_dist）：盒内任意两点都满足 → 任意序；
   - 否则在链首/尾两点坐标的中点二分（保证两者分居两侧），交界取两侧的
     **最近交叉点对**（须 ≤ max_dist，且不与链首尾重合），两侧递归，
     每侧链的首尾由交界点对确定；
   - 归纳保证：相邻点要么在小盒内（≤ 对角线 ≤ max_dist），要么是检验过
     的交界对（≤ max_dist）。确定性、与区域几何形状无关
3. **桥接**：文件顺序 = 组1链 + 组2链。组2起点 = 组2中离组1终点最近的点
   （须 ≤ max_dist）；
4. **失败兜底**：任一交界或桥接超限 → 整批重采样（最多 10 次，upper 实测
   第 2 次成功、lower 第 1 次成功，adjacent max = 0.172 / 0.189 m）；
   耗尽则报错。

结果：使用前 250（组1）与使用后 250（组2）在统计意义上是同一均匀分布；
文件内相邻距离约束在链内与桥接处全部满足。

**Step 3 — 写出 + 自检**
- `piper_{arm}.txt`：每行 `"{x:.4f},{y:.4f},{z:.4f}"`（逗号分隔，匹配 C++ 解析）
- sentry.txt：读取现有文件，**仅将本次生成的臂重置为 0**（`upper:0` / `lower:0`），
  未生成臂的行保持原值（避免误重置另一臂进度）
- 自检并打印：点数 = num、所有点 `is_reachable`、所有相邻距离 ≤ max_dist
  （含桥接处）、相邻距离 min/mean/max、**前后两半各自的质心与标准差**
  （人工对比两半分布是否一致）

**Step 4 — 可视化（viz_gaze_target.py）**
- gen 结束前自动拉起（`--no-viz` 跳过），`MPLBACKEND=TkAgg`（本机 Qt 后端不可用）
- 每臂一幅 3D 图（1×2 布局，标题含 participant + arm + 相邻距离 min/mean/max）：
  - 灰色小点：可达点阵（背景真值）
  - 蓝色点：前半 250，红色点：后半 250（直观对比两半分布）
  - 灰色折线：链顺序连线（含桥接处）
  - 青色线框：内缩后的 gen box
- 独立可运行：`python viz_gaze_target.py --participant P001`（只读 txt + 点阵）

## 边界与错误处理

| 情况 | 行为 |
|------|------|
| yaml / 点阵文件缺失 | 报错列出缺失文件，fail-fast |
| 多个 points 文件匹配 | 取 mtime 最新 + 警告 |
| 接受率过低（重试耗尽） | 报错 + 接受率统计 |
| 重排无法满足 0.2 m（含桥接） | 报错 + 建议参数 |
| 重复点（极小距离） | 无害（对约束更有利），不处理 |
| matplotlib 缺失 | viz 降级为控制台提示（不阻塞生成流程） |

## 验证

1. 小规模冒烟：`python gen_gaze_target.py --participant P001 --num 20 --seed 42 --no-viz`
   → 检查两个 txt 的行数、逗号格式、脚本自检输出（含两半质心/标准差对比）
2. 全量：`--num 500`（默认），确认自检全部通过、相邻距离 max ≤ 0.2（含桥接）、
   两半质心/标准差大致一致；viz 窗口弹出且两半颜色分布无明显差异
3. 消费端联动：运行 `test_piper_ctrl.exe`，确认输出
   `Loaded: upper=500 pts, lower=500 pts`
4. sentry.txt 检查：已存在时另一臂的行保持不变
