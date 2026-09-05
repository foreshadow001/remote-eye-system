#!/usr/bin/env python3
# ================== viz_gaze_coverage ==================
# 被试视线方向 (头部坐标系) 的 pitch×yaw 分布热力图 + 世界系 3D 分布。
#
# 输入:
#   piper_ros cfg/piper_{upper,lower}.yaml 的 eye_position  (头部/双眼原点)
#   cfg/gaze_target/{P}/piper_{upper,lower}.txt             (目标点, 臂基座系)
#   cfg/capture.yaml 的 participant_id
#
# 世界坐标系 (由臂基座系 (b) 定义):
#   原点 = piper upper 基座原点;  x_w = -y_b,  y_w = -z_b,  z_w = +x_b
#   piper lower 基座原点 = (0, dz, 0), dz = 两臂 eye_position 的 z 值之差
#   (动态计算, 保证两臂眼位在世界系中重合);  单位 m
#
# 头部坐标系 (HCS): 与世界系同向 (被试头不动), 原点 = 双眼中点。
# 视线方向 = (target_world - eye_world) 归一化;  pitch/yaw 依次绕 xyz:
#   pitch = asin(-v_y),  yaw = atan2(-v_x, -v_z)   [normalization.vector_to_angles]
#   (参考 ZHANG2015 官方约定: 前向 -z, 下 +y, 右 +x)
#
# 展示: 两个独立窗口 (世界系 3D + pitch×yaw 密度图), 默认不保存文件。
# 配色/格式参考 gaze_distribution.py: turbo 色带 + 深蓝底、白色网格
# 30°主/15°细、方格等比、横轴 yaw 纵轴 pitch、LogNorm 密度。
# 密度图: 每个样本点按 KDE 局部密度着色 (浅蓝→红, 不做格子填充);
# 色柱截去 turbo 深蓝端从浅蓝起步, 峰值 1/3 处饱和 (同参考脚本), 全 ±120°。
# =================================================================
import os
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.stats import gaussian_kde

os.environ.setdefault("MPLBACKEND", "TkAgg")
import matplotlib
matplotlib.use(os.environ.get("MPLBACKEND", "TkAgg"), force=True)
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, LinearSegmentedColormap

SCRIPT_DIR = Path(__file__).resolve().parent          # .../cpp_eyetracker/tests/utils/piper
CPP_DIR = SCRIPT_DIR.parents[2]                       # cpp_eyetracker (parents[0]=utils, [1]=tests)
PIPER_CFG = (SCRIPT_DIR.parents[4] / "piper_ros"        # parents[4] = new_dataset
             / "src" / "piper_moveit" / "moveit_ctrl" / "scripts" / "cfg")
LIM = 120                                             # 显示 ±120°
BG = '#101d4a'


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def b2w(p):
    """臂基座系 -> 世界系 (无平移): x_w=-y_b, y_w=-z_b, z_w=+x_b"""
    p = np.asarray(p, dtype=float)
    return np.stack([-p[:, 1], -p[:, 2], p[:, 0]], axis=1)


def base_to_world(p_upper, p_lower, dz):
    """臂基座系 -> 世界系; lower 原点平移 (0, dz, 0), dz = eye z 值之差 → 眼位重合"""
    return b2w(p_upper), b2w(p_lower) + np.array([0.0, dz, 0.0])


def angles_from(v):
    """方向向量 -> (pitch, yaw) 度; 与 normalization.vector_to_angles 一致"""
    v = v / np.linalg.norm(v, axis=1, keepdims=True)
    pitch = np.degrees(np.arcsin(np.clip(-v[:, 1], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(-v[:, 0], -v[:, 2]))
    return pitch, yaw


def load_targets(participant):
    out = {}
    for arm in ("upper", "lower"):
        p = CPP_DIR / "cfg" / "gaze_target" / participant / f"piper_{arm}.txt"
        out[arm] = np.loadtxt(p, delimiter=",")
    return out


def main():
    cap = load_yaml(CPP_DIR / "cfg" / "capture.yaml")["capture"]
    participant = cap["participant_id"]
    eye_u = np.array(load_yaml(PIPER_CFG / "piper_upper.yaml")["arm"]["eye_position"])
    eye_l = np.array(load_yaml(PIPER_CFG / "piper_lower.yaml")["arm"]["eye_position"])
    print(f"participant={participant}  eye upper(b)={eye_u}  lower(b)={eye_l}")

    dz = float(eye_l[2] - eye_u[2])            # 两眼位 z 值之差 = lower 基座世界 y 偏移
    print(f"lower base world offset = (0, {dz:.2f}, 0)")
    tgt = load_targets(participant)
    tw_u, tw_l = base_to_world(tgt["upper"], tgt["lower"], dz)
    eye_w = b2w(eye_u.reshape(1, 3))[0]        # 眼位 (世界系, 两臂重合)

    # 世界系 3D: 目标点 + 视线射线 (eye -> target); 两臂眼位重合, 单一原点
    all_tw = np.vstack([tw_u, tw_l])
    all_dirs = all_tw - eye_w

    # HCS pitch/yaw (头不动 → 头部系与世界系同向, 仅原点移到双眼中点:
    # 方向向量平移不变, 直接取世界系方向)
    pitch, yaw = angles_from(all_dirs)
    n = len(pitch)
    m = (np.abs(pitch) <= LIM) & (np.abs(yaw) <= LIM)
    print(f"samples={n}  in-range={m.sum()}"
          f"  pitch[{pitch.min():.1f},{pitch.max():.1f}]"
          f"  yaw[{yaw.min():.1f},{yaw.max():.1f}]")

    # ---- 窗口 1: 世界系 3D 分布;  窗口 2: pitch×yaw 密度图 ----
    cmap = LinearSegmentedColormap.from_list(           # turbo 截去深蓝端: 色柱从浅蓝起
        'turbo_lb', matplotlib.colormaps['turbo'](np.linspace(0.22, 1.0, 256)))

    fig3 = plt.figure(figsize=(10, 9))
    fig3.suptitle(f"{participant} — gaze targets in world frame "
                  f"({n} targets, 2 arms x 500)", fontsize=13)
    ax3 = fig3.add_subplot(1, 1, 1, projection="3d")
    for tw, c, lb in ((tw_u, "crimson", "targets upper"),
                      (tw_l, "darkorange", "targets lower")):
        ax3.scatter(tw[:, 0], tw[:, 1], tw[:, 2], c=c, s=6, alpha=0.5, label=lb)
        seg = np.stack([np.repeat(eye_w[None], len(tw), axis=0), tw], axis=1)
        ax3.plot(seg[:, :, 0].T, seg[:, :, 1].T, seg[:, :, 2].T,
                 color=c, lw=0.3, alpha=0.25)
    ax3.scatter(*eye_w, c="black", s=80, marker="X", label="eye")
    # 世界系三轴 (RGB = XYZ)
    L = 0.25
    for d, c, lb in (((L, 0, 0), "red", "x"),
                     ((0, L, 0), "green", "y"),
                     ((0, 0, L), "blue", "z")):
        ax3.quiver(0, 0, 0, *d, color=c, lw=1.8, arrow_length_ratio=0.12)
        ax3.text(d[0] * 1.2, d[1] * 1.2, d[2] * 1.2, lb,
                 color=c, fontsize=11, fontweight="bold")
    ax3.set_xlabel("x (m)")
    ax3.set_ylabel("y (m)")
    ax3.set_zlabel("z (m)")
    ax3.set_title("origin = upper base")
    ax3.legend(fontsize=8)
    ax3.view_init(elev=90, azim=90)            # 初始视角: z 出屏, x 向左, y 向下

    fig = plt.figure(figsize=(9, 9))
    fig.suptitle(f"{participant} — gaze direction in head frame", fontsize=13)
    ax = fig.add_subplot(1, 1, 1)
    ax.set_facecolor(BG)
    # 每个样本点按 KDE 局部密度着色 (无格子填充): 低→高 = 浅蓝→红
    xy = np.vstack([yaw[m], pitch[m]])
    dens = gaussian_kde(xy)(xy) * 100.0                # % 每平方度
    o = np.argsort(dens)                               # 密点最后画, 顶层可见
    vmin = np.quantile(dens, 0.05)                     # 底端浅蓝 (不融进背景)
    vmax = dens.max() / 3.0                            # 峰值 1/3 处饱和 (同参考脚本)
    sc = ax.scatter(xy[0, o], xy[1, o], s=5, c=dens[o], cmap=cmap,
                    norm=LogNorm(vmin=vmin, vmax=vmax), linewidths=0)
    ax.set_aspect("equal")
    ax.set_xlim(-LIM, LIM)
    ax.set_ylim(-LIM, LIM)
    ax.set_axisbelow(False)
    ax.set_xticks(np.arange(-LIM, LIM + 1, 30))
    ax.set_yticks(np.arange(-LIM, LIM + 1, 30))
    ax.grid(which="major", color="white", lw=0.5)
    ax.set_xticks(np.arange(-LIM, LIM + 1, 15), minor=True)
    ax.set_yticks(np.arange(-LIM, LIM + 1, 15), minor=True)
    ax.grid(which="minor", color="white", lw=0.2, alpha=0.5)
    ax.tick_params(which="minor", length=0)
    ax.set_xlabel("Yaw (deg)")
    ax.set_ylabel("Pitch (deg)")
    fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.03,
                 label="density (% per deg$^2$)")

    fig3.tight_layout()
    fig.tight_layout()
    plt.show()                                   # 两窗口同时显示, 不保存文件


if __name__ == "__main__":
    main()
