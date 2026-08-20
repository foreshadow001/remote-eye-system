#!/usr/bin/env python3
# ================== viz_gaze_target ==================
# 可视化 gaze target 生成结果 (3D):
#   浅灰小点 = 可达点阵真值;  蓝色 = 前半;  红色 = 后半 (对比两半分布)
#   灰色折线 = 链序连线;  橙色粗线 = 两半桥接段;  青色线框 = 内缩后 gen box
# 必须 MPLBACKEND=TkAgg (本机 Qt 后端不可用, DEV_GUIDE 约定)
# =================================================================

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "TkAgg")

import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))   # 允许 standalone 运行时导入同目录 gen_gaze_target

from gen_gaze_target import OUT_ROOT, default_participant, load_arm  # noqa: E402


def load_targets(participant, arm):
    p = OUT_ROOT / participant / f"piper_{arm}.txt"
    if not p.exists():
        print(f"[Skip] no targets: {p}")
        return None
    pts = np.array([[float(v) for v in line.split(",")]
                    for line in p.read_text(encoding="utf-8").splitlines() if line.strip()])
    if pts.ndim != 2 or pts.shape[1] != 3:
        print(f"[Skip] bad format: {p}")
        return None
    return pts


def draw_box(ax, box, color="cyan"):
    x0, x1 = box["x"]
    y0, y1 = box["y"]
    z0, z1 = box["z"]
    corners = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0),
             (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for i, j in edges:
        ax.plot(*corners[[i, j]].T, color=color, lw=0.8, alpha=0.8)


def main():
    ap = argparse.ArgumentParser(description="Visualize generated gaze targets")
    ap.add_argument("--participant", default=None,
                    help="participant id (default: read from cfg/capture.yaml)")
    ap.add_argument("--arms", nargs="+", default=["upper", "lower"])
    ap.add_argument("--edge", type=float, default=0.025, help="gen box inset (m)")
    args = ap.parse_args()
    args.participant = args.participant or default_participant()

    arms = [a for a in args.arms if load_targets(args.participant, a) is not None]
    if not arms:
        print("[Error] nothing to visualize")
        sys.exit(1)

    fig = plt.figure(figsize=(7 * len(arms), 6))
    for k, arm in enumerate(arms):
        box, lattice, _ = load_arm(arm)
        pts = load_targets(args.participant, arm)
        half = len(pts) // 2
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)

        ax = fig.add_subplot(1, len(arms), k + 1, projection="3d")
        ax.scatter(lattice[:, 0], lattice[:, 1], lattice[:, 2],
                   c="lightgray", s=2, alpha=0.35, label="reachable lattice")
        ax.scatter(pts[:half, 0], pts[:half, 1], pts[:half, 2],
                   c="royalblue", s=12, label=f"1st half ({half})")
        ax.scatter(pts[half:, 0], pts[half:, 1], pts[half:, 2],
                   c="crimson", s=12, label=f"2nd half ({len(pts) - half})")
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color="gray", lw=0.8, alpha=0.7)
        ax.plot(pts[half - 1:half + 1, 0], pts[half - 1:half + 1, 1],
                pts[half - 1:half + 1, 2], color="orange", lw=2.0, label="bridge")
        lo = np.array([box["x"][0], box["y"][0], box["z"][0]]) + args.edge
        hi = np.array([box["x"][1], box["y"][1], box["z"][1]]) - args.edge
        draw_box(ax, {"x": [lo[0], hi[0]], "y": [lo[1], hi[1]], "z": [lo[2], hi[2]]})
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        ax.set_title(f"{args.participant} — piper_{arm} (N={len(pts)})\n"
                     f"adjacent: min {d.min():.3f} / mean {d.mean():.3f} / max {d.max():.3f} m"
                     f"   bridge {d[half - 1]:.3f} m", fontsize=10)
        ax.legend(fontsize=8, loc="upper right")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
