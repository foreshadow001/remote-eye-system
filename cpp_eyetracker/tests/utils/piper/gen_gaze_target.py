#!/usr/bin/env python3
# ================== gen_gaze_target ==================
# 随机生成 gaze target 点 (每臂默认 500 个, 前后两半分布一致)
# 输入:
#   piper_ros/.../scripts/cfg/piper_{arm}.yaml          -> workspace_analysis.sampling_box
#   piper_ros/.../scripts/reachable_range/points_piper_{arm}*.txt  (workspace_analyzer.py 输出)
# 输出:
#   cpp_eyetracker/cfg/gaze_target/{participant_id}/piper_{arm}.txt  (逗号分隔 x,y,z, 匹配 C++ loadTgts)
#   同目录 sentry.txt (仅重置本次生成的臂)
# 判据: 点阵腐蚀 (仅内部格点) + min_dist <= r*sqrt(3)/2
# 排序: 两组各自 chain_pts (小盒任意序 / 二分递归 + 最近交界点对),
#   组2起点 = 离组1终点最近的点; 失败整批重采样
# =================================================================

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

sys.setrecursionlimit(10000)   # 递归桥接链: 不平衡分裂时深度可接近点数

SCRIPT_DIR = Path(__file__).resolve().parent      # cpp_eyetracker/tests/utils/piper
CPP_DIR = SCRIPT_DIR.parents[2]                   # cpp_eyetracker (parents[0]=utils, [1]=tests)
PIPER_SCRIPTS = (SCRIPT_DIR.parents[4] / "piper_ros"
                 / "src" / "piper_moveit" / "moveit_ctrl" / "scripts")
OUT_ROOT = CPP_DIR / "cfg" / "gaze_target"

_MIN_DIST_CHUNK = 2048         # 分批广播的候选点块大小
_MAX_SAMPLE_ATTEMPTS = 10      # 排序/桥接失败时的整批重采样次数


def fatal(msg):
    print(f"[Error] {msg}", file=sys.stderr)
    sys.exit(1)


def default_participant():
    """participant id 从 capture.yaml 读取 (与采集端 test_piper_ctrl / capture_* 一致)."""
    cap = CPP_DIR / "cfg" / "capture.yaml"
    if not cap.exists():
        fatal(f"missing config: {cap}")
    try:
        return yaml.safe_load(cap.read_text(encoding="utf-8"))["capture"]["participant_id"]
    except Exception as e:
        fatal(f"cannot read participant_id from {cap}: {e}")


def infer_resolution(pts):
    """点阵格距: 各轴唯一坐标的最小正差值 (文件名不含格距信息, 从数据推断)."""
    r = float("inf")
    for i in range(3):
        d = np.diff(np.unique(pts[:, i]))
        d = d[d > 0]
        if len(d):
            r = min(r, d.min())
    return r


def load_arm(arm):
    """加载 {arm} 臂的 sampling box 与可达点阵, 返回 (box, points, r)."""
    yaml_path = PIPER_SCRIPTS / "cfg" / f"piper_{arm}.yaml"
    if not yaml_path.exists():
        fatal(f"missing yaml: {yaml_path}")
    box = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))["workspace_analysis"]["sampling_box"]

    rng_dir = PIPER_SCRIPTS / "reachable_range"
    matches = sorted(rng_dir.glob(f"points_piper_{arm}*.txt"),
                     key=lambda p: p.stat().st_mtime)
    if not matches:
        fatal(f"no reachable points file for arm '{arm}' in {rng_dir}")
    if len(matches) > 1:
        print(f"[Warn] {len(matches)} matches for '{arm}', using newest: {matches[-1].name}")
    pts = np.loadtxt(matches[-1])
    if pts.ndim != 2 or pts.shape[1] != 3:
        fatal(f"bad points file format: {matches[-1]}")
    r = infer_resolution(pts)
    print(f"[Load] {arm}: lattice={len(pts)} pts, r={r:.3f} m, file={matches[-1].name}")
    return box, pts, r


def interior_lattice(pts, r):
    """点阵腐蚀: 仅保留 6 个轴向相邻格点全部存在且可达的内部格点 (腐蚀宽度 ~1 格).

    3 位小数取整作字典键: 点阵由 "%.3f" 写入, 键两侧同一取整规则保证一致.
    """
    reach = {tuple(np.round(p, 3)) for p in pts}
    offsets = [(-r, 0, 0), (r, 0, 0), (0, -r, 0), (0, r, 0), (0, 0, -r), (0, 0, r)]
    interior = [v for v in pts
                if all(tuple(np.round(v + o, 3)) in reach for o in offsets)]
    if not interior:
        fatal("erosion left no interior lattice points (region thinner than 2 cells?)")
    print(f"[Erode] interior lattice: {len(interior)} / {len(pts)} pts")
    return np.array(interior)


def min_dists(cands, lattice):
    """候选点 (Mx3) 到点阵 (Kx3) 的最近距离, 分批广播避免内存峰值."""
    out = np.empty(len(cands))
    for i in range(0, len(cands), _MIN_DIST_CHUNK):
        c = cands[i:i + _MIN_DIST_CHUNK]
        out[i:i + _MIN_DIST_CHUNK] = np.sqrt(
            ((c[:, None, :] - lattice[None, :, :]) ** 2).sum(-1)).min(1)
    return out


def sample_points(box, interior, r, n, rng, label):
    """拒绝采样 n 个可达点 (i.i.d. 均匀于 gen box), 返回按接受顺序排列的点集."""
    lo = np.array([box["x"][0], box["y"][0], box["z"][0]])
    hi = np.array([box["x"][1], box["y"][1], box["z"][1]])
    tol = r * np.sqrt(3) / 2 + 1e-3   # 胞体对角线之半: 不误杀可达区域内部点
    pts, attempts, accepted = [], 0, 0
    cap = 200 * n
    while len(pts) < n:
        cands = rng.uniform(lo, hi, size=(min(4096, n - len(pts)), 3))
        d = min_dists(cands, interior)
        for c, dist in zip(cands, d):
            attempts += 1
            if attempts > cap:
                fatal(f"accept rate too low ({accepted}/{attempts}) for '{label}' — "
                      f"reachable region too small? reduce --num or check lattice")
            if dist <= tol:
                pts.append(c)
                accepted += 1
                if len(pts) >= n:
                    break
    print(f"[Sample] {label}: {n} pts, accept rate {accepted / attempts * 100:.1f}%")
    return np.array(pts)


def chain_pts(pts, start, end, max_dist):
    """返回 pts 的索引排列: 首=start, 尾=end, 相邻距离 <= max_dist.

    归纳构造 (两组共用同一逻辑):
    - 小盒 (包围盒对角线 <= max_dist): 盒内任意两点都满足 -> 任意序;
    - 否则在 start/end 坐标的中点二分 (保证两者分居两侧), 交界取最近交叉
      点对 (须 <= max_dist 且不与链首尾重合); 两侧递归, 首尾由交界对确定.
    交界不可行抛 ValueError (调用方整批重采样).
    """
    n = len(pts)
    if n == 1:
        return [start]
    ext = pts.max(0) - pts.min(0)
    if np.sqrt((ext ** 2).sum()) <= max_dist:   # 小盒: 任意序
        return [start] + [i for i in range(n) if i != start and i != end] + [end]

    # 分裂轴: start/end 坐标不同的轴 (优先跨度大的), 中点分裂保证两者异侧
    cand = [i for i in range(3) if pts[start][i] != pts[end][i]]
    axis = max(cand, key=lambda i: ext[i])
    split_val = (pts[start][axis] + pts[end][axis]) / 2.0
    li = np.flatnonzero(pts[:, axis] < split_val)    # start 侧
    ri = np.flatnonzero(pts[:, axis] >= split_val)   # end 侧
    L, R = pts[li], pts[ri]
    d_cross = np.sqrt(((L[:, None, :] - R[None, :, :]) ** 2).sum(-1))

    def pos(arr, v):
        return int(np.where(arr == v)[0][0])

    s_in_L = start in li
    s_pos = pos(li, start) if s_in_L else pos(ri, start)
    e_pos = pos(ri, end) if s_in_L else pos(li, end)

    # 最近交叉点对作交界 (避开与链首尾重合, 否则两侧链首尾同点)
    k = None
    for kk in np.argsort(d_cross.ravel()):
        a, b = int(kk) // len(ri), int(kk) % len(ri)
        l_s, l_e = (s_pos, a) if s_in_L else (a, e_pos)   # L 链首尾
        r_s, r_e = (b, e_pos) if s_in_L else (s_pos, b)   # R 链首尾
        if len(li) >= 2 and l_s == l_e:
            continue
        if len(ri) >= 2 and r_s == r_e:
            continue
        if d_cross[a, b] <= max_dist:
            k = int(kk)
            break
    if k is None:
        raise ValueError("bridge exceeds max_dist")
    a, b = k // len(ri), k % len(ri)
    if s_in_L:
        return ([li[i] for i in chain_pts(L, s_pos, a, max_dist)]
                + [ri[i] for i in chain_pts(R, b, e_pos, max_dist)])
    return ([ri[i] for i in chain_pts(R, s_pos, b, max_dist)]
            + [li[i] for i in chain_pts(L, a, e_pos, max_dist)])


def chain_two_halves(a, b, max_dist):
    """两组独立排序 (同一套 chain_pts) + 桥接; 失败返回 None -> 整批重采样.

    组1: 任意首尾; 组2: 起点 = 组2中离组1终点最近的点 (须 <= max_dist).
    """
    try:
        a_chain = a[chain_pts(a, 0, len(a) - 1, max_dist)]
        a_end = a_chain[-1]
        d_b = np.sqrt(((b - a_end) ** 2).sum(1))
        j = int(d_b.argmin())
        if d_b[j] > max_dist:
            return None
        b_end = (len(b) - 1) if j != len(b) - 1 else 0
        b_chain = b[chain_pts(b, j, b_end, max_dist)]
    except ValueError:
        return None
    full = np.vstack([a_chain, b_chain])
    d = np.linalg.norm(np.diff(full, axis=0), axis=1)
    if (d > max_dist + 1e-9).any():   # 防御性复核
        return None
    return a_chain, b_chain


def self_check(pts, interior, r, max_dist, label):
    """写出前自检: 全部可达 / 相邻距离 (含桥接) <= max_dist / 两半分布统计."""
    tol = r * np.sqrt(3) / 2 + 1e-3
    d_lat = min_dists(pts, interior)
    if not (d_lat <= tol).all():
        fatal(f"self-check failed: unreachable point in '{label}' output")
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    if not (d <= max_dist + 1e-9).all():
        fatal(f"self-check failed: adjacent distance exceeds {max_dist} in '{label}' output")
    half = len(pts) // 2
    for name, seg in (("1st half", pts[:half]), ("2nd half", pts[half:])):
        c, s = seg.mean(0), seg.std(0)
        print(f"[Check] {label} {name}: centroid=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) "
              f"std=({s[0]:.3f},{s[1]:.3f},{s[2]:.3f})")
    print(f"[Check] {label}: {len(pts)} pts, adjacent dist min/mean/max = "
          f"{d.min():.3f}/{d.mean():.3f}/{d.max():.3f} m, bridge = {d[half - 1]:.3f} m")


def update_sentry(out_dir, generated_arms):
    """仅将本次生成的臂重置为 0, 未生成臂的进度行保持原值."""
    sentry = out_dir / "sentry.txt"
    lines = {}
    if sentry.exists():
        for line in sentry.read_text(encoding="utf-8").splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                lines[k.strip()] = v.strip()
    for arm in generated_arms:
        lines[arm] = "0"
    order = ["upper", "lower"] + sorted(k for k in lines if k not in ("upper", "lower"))
    sentry.write_text("\n".join(f"{k}:{lines[k]}" for k in order if k in lines) + "\n", encoding="utf-8")
    print(f"[Sentry] reset arms {generated_arms} -> {sentry}")


def main():
    ap = argparse.ArgumentParser(description="Randomly generate reachable gaze targets per arm")
    ap.add_argument("--participant", default=None,
                    help="participant id (default: read from cfg/capture.yaml)")
    ap.add_argument("--num", type=int, default=500, help="targets per arm")
    ap.add_argument("--edge", type=float, default=0.025, help="gen box inset (m)")
    ap.add_argument("--max-dist", type=float, default=0.2,
                    help="max adjacent target distance after reorder (m)")
    ap.add_argument("--seed", type=int, default=123, help="RNG seed")
    ap.add_argument("--arms", nargs="+", default=["upper", "lower"],
                    help="arms to generate (default: upper lower)")
    ap.add_argument("--no-viz", action="store_true", help="skip visualization")
    args = ap.parse_args()

    args.participant = args.participant or default_participant()
    rng = np.random.default_rng(args.seed)
    print(f"=== gen_gaze_target: participant={args.participant} num={args.num} "
          f"edge={args.edge} max_dist={args.max_dist} seed={args.seed} arms={args.arms}")

    out_dir = OUT_ROOT / args.participant
    out_dir.mkdir(parents=True, exist_ok=True)

    generated = []
    for arm in args.arms:
        box, lattice, r = load_arm(arm)
        interior = interior_lattice(lattice, r)
        lo = np.array([box["x"][0], box["y"][0], box["z"][0]]) + args.edge
        hi = np.array([box["x"][1], box["y"][1], box["z"][1]]) - args.edge
        if (hi <= lo).any():
            fatal(f"arm '{arm}': gen box empty after edge inset (edge={args.edge})")
        gen_box = {"x": [lo[0], hi[0]], "y": [lo[1], hi[1]], "z": [lo[2], hi[2]]}

        # 同一条 i.i.d. 采样流按接受顺序切分 -> 两半独立同分布 (截断使用无偏)
        # 排序/桥接失败则整批重采样 (最多 _MAX_SAMPLE_ATTEMPTS 次)
        full = None
        for attempt in range(1, _MAX_SAMPLE_ATTEMPTS + 1):
            pts = sample_points(gen_box, interior, r, args.num, rng, f"{arm}")
            a, b = pts[:args.num // 2], pts[args.num // 2:]
            res = chain_two_halves(a, b, args.max_dist)
            if res is not None:
                a_ord, b_ord = res
                full = np.vstack([a_ord, b_ord])
                break
            print(f"[Chain] {arm} attempt {attempt}: ordering/bridge failed — resampling")
        else:
            fatal(f"cannot order '{arm}' targets under {args.max_dist} m "
                  f"after {_MAX_SAMPLE_ATTEMPTS} attempts")

        self_check(full, interior, r, args.max_dist, arm)   # 自检通过才写出

        out_path = out_dir / f"piper_{arm}.txt"
        with open(out_path, "w") as f:
            for p in full:
                f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}\n")
        print(f"[Write] {out_path} ({len(full)} pts)")
        generated.append(arm)

    update_sentry(out_dir, generated)

    if not args.no_viz:
        viz = SCRIPT_DIR / "viz_gaze_target.py"
        try:
            subprocess.run([sys.executable, str(viz), "--participant", args.participant,
                            "--edge", str(args.edge), "--arms"] + generated)
        except Exception as e:
            print(f"[Warn] viz launch failed: {e}")

    print("=== done ===")


if __name__ == "__main__":
    main()
