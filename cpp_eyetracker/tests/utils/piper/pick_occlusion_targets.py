#!/usr/bin/env python3
# ================== pick_occlusion_targets ==================
# 为每臂挑选最可能造成遮挡的 gaze target (遮挡检测压力测试用, 仅测试)。
#
# 输入/输出: cfg/gaze_target/{participant}/piper_{upper,lower}.txt (臂基座系)
# 运行: 先备份原文件为 piper_{arm}.txt.bak (同目录; 已有 .bak 则保留最早的
#   原始集, 重复运行结果不变), 再从原始集挑选覆盖原文件, 并重置 sentry。
# --recover: 用 .bak 恢复原文件并重置 sentry (.bak 保留, 可重复恢复)。
#
# 挑选准则 (各维在臂内 min-max 归一化后加权求和, 按得分降序取 Top-N;
# 不做相邻距离/顺序约束 — 测试用途, 臂按得分顺序逐点运动):
#   1. x 尽可能大 (离相机远, 臂伸入场景更深)   权重 1.0 (两臂)
#   2. y 尽可能大 (横向极端)                    权重 0.5 (两臂)
#   3. upper 着重越过中心相机: z 越低越好       权重 0.5 (仅 upper, 取 -z)
# =================================================================
import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent      # cpp_eyetracker/tests/utils/piper
CPP_DIR = SCRIPT_DIR.parents[2]                   # cpp_eyetracker
OUT_ROOT = CPP_DIR / "cfg" / "gaze_target"

W_X = 1.0          # x 项权重 (两臂)
W_Y = 0.5          # y 项权重 (两臂)
W_Z_UPPER = 0.5    # upper 的 -z 项权重 (越过中心相机)


def fatal(msg):
    print(f"[Error] {msg}", file=sys.stderr)
    sys.exit(1)


def default_participant():
    cap = CPP_DIR / "cfg" / "capture.yaml"
    try:
        return yaml.safe_load(cap.read_text(encoding="utf-8"))["capture"]["participant_id"]
    except Exception as e:
        fatal(f"cannot read participant_id from {cap}: {e}")


def norm01(v):
    """min-max 归一化到 [0,1]; 常数列归零。"""
    lo, hi = v.min(), v.max()
    return np.zeros_like(v) if hi - lo < 1e-12 else (v - lo) / (hi - lo)


def scores(pts, arm):
    """遮挡倾向得分: x 大 + y 大 (+ upper: z 低)。"""
    s = W_X * norm01(pts[:, 0]) + W_Y * norm01(pts[:, 1])
    if arm == "upper":
        s = s + W_Z_UPPER * norm01(-pts[:, 2])
    return s


def update_sentry(out_dir, arms):
    """仅将本次处理的臂重置为 0 (与 gen_gaze_target 一致)。"""
    sentry = out_dir / "sentry.txt"
    lines = {}
    if sentry.exists():
        for line in sentry.read_text(encoding="utf-8").splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                lines[k.strip()] = v.strip()
    for arm in arms:
        lines[arm] = "0"
    order = ["upper", "lower"] + sorted(k for k in lines if k not in ("upper", "lower"))
    sentry.write_text("\n".join(f"{k}:{lines[k]}" for k in order if k in lines) + "\n",
                      encoding="utf-8")
    print(f"[Sentry] reset arms {arms} -> {sentry}")


def main():
    ap = argparse.ArgumentParser(description="Pick occlusion-prone gaze targets per arm")
    ap.add_argument("--participant", default=None,
                    help="participant id (default: read from cfg/capture.yaml)")
    ap.add_argument("--num", type=int, default=20, help="targets per arm (default 20)")
    ap.add_argument("--arms", nargs="+", default=["upper", "lower"],
                    help="arms to process (default: upper lower)")
    ap.add_argument("--recover", action="store_true",
                    help="restore original targets from .bak and reset sentry")
    args = ap.parse_args()
    participant = args.participant or default_participant()

    out_dir = OUT_ROOT / participant
    if not out_dir.is_dir():
        fatal(f"missing gaze target dir: {out_dir}")

    if args.recover:
        for arm in args.arms:
            src, dst = out_dir / f"piper_{arm}.txt.bak", out_dir / f"piper_{arm}.txt"
            if not src.exists():
                print(f"[Recover] no backup for {arm}, skipped")
                continue
            shutil.copy2(src, dst)
            n = len(np.loadtxt(dst, delimiter=","))
            print(f"[Recover] {arm}: restored {n} targets from {src.name} (backup kept)")
        update_sentry(out_dir, args.arms)
        print("=== done (recovered) ===")
        return

    print(f"=== pick_occlusion_targets: participant={participant} "
          f"num={args.num} arms={args.arms} "
          f"weights: x={W_X} y={W_Y} z_upper={W_Z_UPPER} ===")
    for arm in args.arms:
        txt = out_dir / f"piper_{arm}.txt"
        bak = out_dir / f"piper_{arm}.txt.bak"
        if not txt.exists():
            fatal(f"missing {txt}")
        # 备份原始集 (已有 .bak 保留最早版本; 挑选始终基于原始集, 重复运行不变)
        if not bak.exists():
            shutil.copy2(txt, bak)
            print(f"[Backup] {txt.name} -> {bak.name}")
        else:
            print(f"[Backup] {bak.name} exists (original kept)")
        pts = np.loadtxt(bak, delimiter=",")
        if len(pts) < args.num:
            fatal(f"{arm}: only {len(pts)} targets < --num {args.num}")

        s = scores(pts, arm)
        pick = np.argsort(s)[::-1][:args.num]     # 按得分降序, 不做顺序/距离约束
        sel = pts[pick]
        with open(txt, "w") as f:
            for p in sel:
                f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}\n")
        print(f"[Pick] {arm}: {args.num}/{len(pts)} targets, "
              f"x[{sel[:,0].min():.2f},{sel[:,0].max():.2f}] "
              f"y[{sel[:,1].min():.2f},{sel[:,1].max():.2f}] "
              f"z[{sel[:,2].min():.2f},{sel[:,2].max():.2f}]")
        for i, (j, p) in enumerate(zip(pick, sel)):
            print(f"    #{i+1:2d}  x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}  (orig #{j+1}, score {s[j]:.3f})")

    update_sentry(out_dir, args.arms)
    print("=== done ===")


if __name__ == "__main__":
    main()
