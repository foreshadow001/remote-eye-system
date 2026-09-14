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
# 排序: 两组各自贪心最近邻游走 (多起点重试, 全败转瓶颈最小插入修复),
#   组2起点 = 离组1终点最近的点; 仍失败整批重采样
# =================================================================

import argparse
import random
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent      # cpp_eyetracker/tests/utils/piper
CPP_DIR = SCRIPT_DIR.parents[2]                   # cpp_eyetracker (parents[0]=utils, [1]=tests)
PIPER_SCRIPTS = (SCRIPT_DIR.parents[4] / "piper_ros"
                 / "src" / "piper_moveit" / "moveit_ctrl" / "scripts")
OUT_ROOT = CPP_DIR / "cfg" / "gaze_target"

_MIN_DIST_CHUNK = 2048         # 分批广播的候选点块大小
_MAX_SAMPLE_ATTEMPTS = 10      # 排序/桥接失败时的整批重采样次数
_NUM_CHAIN_STARTS = 40         # 组1贪心游走的随机起点数 (全败转插入修复)


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


def nn_walk(D, start, max_dist):
    """从 start 出发的贪心最近邻游走; 任一步 > max_dist 返回 None."""
    n = D.shape[0]
    unv = np.ones(n, bool)
    unv[start] = False
    order, cur = [start], start
    while unv.any():
        d = D[cur].copy()
        d[~unv] = np.inf
        j = int(d.argmin())
        if d[j] > max_dist:
            return None
        unv[j] = False
        order.append(j)
        cur = j
    return order


def insert_repair(path, D, rem, max_dist):
    """剩余点逐个插入瓶颈代价最小的边 (代价 = max(d(u,x), d(x,v))).

    单点贪心游走会把已消耗走廊两侧的叶尖困死, 插入可绕过已用点借道.
    """
    while rem:
        best = None                          # (代价, 点, 插入位置)
        for x in rem:
            c = np.maximum(D[x, path[:-1]], D[x, path[1:]])
            p = int(c.argmin())
            if best is None or c[p] < best[0]:
                best = (c[p], x, p)
        if best[0] > max_dist:
            return None
        _, x, p = best
        path.insert(p + 1, x)
        rem.remove(x)
    return path


def chain_order(pts, start, max_dist, rng):
    """返回 pts 的索引序: 相邻距离 <= max_dist; start 为起点 (None 则任意).

    贪心最近邻对起点敏感 (薄走廊区域从中间出发会困死叶尖), 故多起点重试;
    全败时以「起点+距起点最近点」为骨架做瓶颈插入修复.
    不可行返回 None (调用方整批重采样).
    """
    D = np.sqrt(((pts[:, None, :] - pts[None, :, :]) ** 2).sum(-1))
    n = len(pts)
    starts = ([start] if start is not None
              else rng.permutation(n))[:_NUM_CHAIN_STARTS]
    for s in starts:
        order = nn_walk(D, int(s), max_dist)
        if order is not None:
            return order
    s0 = int(starts[0])
    d0 = D[s0].copy()
    d0[s0] = np.inf
    s1 = int(d0.argmin())
    if D[s0, s1] > max_dist:
        return None
    rem = [i for i in range(n) if i not in (s0, s1)]
    return insert_repair([s0, s1], D, rem, max_dist)


def chain_two_halves(a, b, max_dist, rng):
    """两组独立排序 + 桥接; 失败返回 None -> 整批重采样.

    组1: 起点任意 (多起点贪心); 组2: 起点 = 组2中离组1终点最近的点 (须 <= max_dist).
    """
    a_order = chain_order(a, None, max_dist, rng)
    if a_order is None:
        return None
    a_end = a_order[-1]
    d_b = np.sqrt(((b - a[a_end]) ** 2).sum(1))
    j = int(d_b.argmin())
    if d_b[j] > max_dist:
        return None
    b_order = chain_order(b, j, max_dist, rng)
    if b_order is None:
        return None
    a_chain, b_chain = a[a_order], b[b_order]
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
    ap.add_argument("--edge", type=float, default=0.002, help="gen box inset (m)")
    ap.add_argument("--max-dist", type=float, default=0.2,
                    help="max adjacent target distance after reorder (m)")
    ap.add_argument("--seed", type=int, default=None,
                    help="RNG seed (default: random, printed for reproducibility)")
    ap.add_argument("--arms", nargs="+", default=["upper", "lower"],
                    help="arms to generate (default: upper lower)")
    ap.add_argument("--no-viz", action="store_true", help="skip visualization")
    args = ap.parse_args()

    args.participant = args.participant or default_participant()
    if args.seed is None:
        args.seed = random.randrange(1 << 32)
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
            res = chain_two_halves(a, b, args.max_dist, rng)
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
