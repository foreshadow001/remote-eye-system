#!/usr/bin/env python3
# ================== viz_gaze_coverage ==================
# 被试视线方向 (世界系) pitch×yaw 分布热力图 + 世界系 3D 分布。
#
# 世界系 = 中心相机 (cam_calib.yaml center_cam, 40772280) 相机坐标系:
#   标定链 XML 外参均以中心相机为参考 (其自身位姿 = 单位), arm_pose 手眼
#   结果的 CCS 亦为同一参考系 -> 相机光轴 / 臂目标点 / 眼位天然同系。
# 相机系约定: x 右, y 下, z 向前 (出镜头指向被试)。
#
# 输入:
#   cfg/capture.yaml                      -> participant_id
#   cfg/day_participant_map.json          -> participant -> day_id
#   {calib_save_dir}/{day}/output/{SN}_Data.xml  相机位姿 (gba: R=Rx(a)Ry(b)Rz(g),
#                                              p_ref = R·p_cam + T, 单位 m)
#   cfg/arm_pose/{day}.yaml               -> arm_in_ccs (zxz: R=Rz(a)Rx(b)Rz(g),
#                                              p_ccs = R·p_arm + T)
#   cfg/gaze_target/{P}/piper_{upper,lower}.txt  目标点 (臂基座系, m)
# 眼位 = 全部相机光轴 (过 T_i, 方向 R_i·e_z) 的最小二乘交点 (不手动指定)。
#
# pitch/yaw (依次绕 xyz, 与 normalization.vector_to_angles 同式):
#   pitch = asin(-v_y),  yaw = atan2(-v_x, -v_z)
#
# 展示: 两个独立窗口 (世界系 3D + pitch×yaw 密度图), 默认不保存文件。
# 密度图: 每个样本点按 KDE 局部密度着色 (浅蓝→红, 无格子填充);
# 色柱截去 turbo 深蓝端从浅蓝起步, 峰值 1/3 处饱和, 全 ±120° 显示,
# 白色网格 30°主/15°细、方格等比、横轴 yaw 纵轴 pitch、LogNorm 密度。
# =================================================================
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from scipy.stats import gaussian_kde

os.environ.setdefault("MPLBACKEND", "TkAgg")
import matplotlib
matplotlib.use(os.environ.get("MPLBACKEND", "TkAgg"), force=True)
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, LogNorm

SCRIPT_DIR = Path(__file__).resolve().parent          # .../cpp_eyetracker/tests/utils/piper
CPP_DIR = SCRIPT_DIR.parents[2]                       # cpp_eyetracker (parents[0]=utils, [1]=tests)
CFG_DIR = CPP_DIR / "cfg"
LIM = 120                                             # 显示 ±120°
BG = '#101d4a'


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def fatal(msg):
    print(f"[Error] {msg}", file=sys.stderr)
    sys.exit(1)


# ---------------- 旋转构造 (两种欧拉约定, 角度均为 deg) ----------------
def _Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def gba_R(a, b, g):
    """HALCON XML OrderOfRotation=gba: R = Rx(Alpha)·Ry(Beta)·Rz(Gamma)"""
    a, b, g = np.radians([a, b, g])
    return _Rx(a) @ _Ry(b) @ _Rz(g)


def zxz_R(a, b, g):
    """piper yaml rotation_zxz (与 piper.hpp zxzToQuat 一致): R = Rz(a)·Rx(b)·Rz(g)"""
    a, b, g = np.radians([a, b, g])
    return _Rz(a) @ _Rx(b) @ _Rz(g)


# ---------------- 输入加载 ----------------
def find_day(participant):
    """day_participant_map.json: {day: [participants]} -> participant 的 day (多日取最新)."""
    m = json.loads((CFG_DIR / "day_participant_map.json").read_text(encoding="utf-8"))
    days = [d for d, ps in m.items() if participant in ps]
    if not days:
        fatal(f"participant {participant} not found in day_participant_map.json")
    return sorted(days)[-1]


def load_cam_poses(xml_dir):
    """{SN}_Data.xml -> {SN: (R, T)}; 位姿为相机 -> 参考系 (中心相机系)."""
    poses = {}
    for f in sorted(Path(xml_dir).glob("*_Data.xml")):
        ext = ET.parse(f).getroot().find("ExternalParameters")
        T = np.array([float(ext.find(f"Translation/{k}").text) for k in "XYZ"])
        a, b, g = (float(ext.find(f"Rotation/{k}").text)
                   for k in ("Alpha", "Beta", "Gamma"))
        poses[f.name.removesuffix("_Data.xml")] = (gba_R(a, b, g), T)
    return poses


def axes_intersection(poses):
    """全部相机光轴 (过 T_i, 方向 R_i·e_z) 的最小二乘交点 + 各轴距残差."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    resid = []
    for R, T in poses.values():
        d = R @ np.array([0.0, 0.0, 1.0])
        P = np.eye(3) - np.outer(d, d)
        A += P
        b += P @ T
    eye = np.linalg.solve(A, b)
    for R, T in poses.values():
        d = R @ np.array([0.0, 0.0, 1.0])
        resid.append(np.linalg.norm(np.cross(eye - T, d)))
    return eye, np.array(resid)


def angles_from(v):
    """方向向量 -> (pitch, yaw) 度; 与 normalization.vector_to_angles 一致"""
    v = v / np.linalg.norm(v, axis=1, keepdims=True)
    pitch = np.degrees(np.arcsin(np.clip(-v[:, 1], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(-v[:, 0], -v[:, 2]))
    return pitch, yaw


def main():
    P = load_yaml(CFG_DIR / "capture.yaml")["capture"]["participant_id"]
    day = find_day(P)
    cal = load_yaml(CFG_DIR / "cam_calib.yaml")["calib"]
    center_sn = cal.get("center_cam") or "40772280"

    # 相机位姿 XML: 优先参与者内参输出, 否则当日标定链输出
    root = Path(cal["calib_save_dir"])
    xml_dir = next((d for d in (root / P / "output", root / day / "output")
                    if d.is_dir()), None)
    if xml_dir is None:
        fatal(f"no calib XML dir under {root}/{{{P},{day}}}/output")
    poses = load_cam_poses(xml_dir)
    if center_sn not in poses:
        fatal(f"center cam {center_sn} XML missing in {xml_dir}")

    eye, resid = axes_intersection(poses)
    print(f"participant={P}  day={day}  world={center_sn} cam frame  ({len(poses)} cams)")
    print(f"eye (world) = {np.round(eye, 4)} m  |eye| = {np.linalg.norm(eye):.3f} m")
    print(f"optical-axis residual (eye to axis): "
          f"min/mean/max = {resid.min()*1e3:.1f}/{resid.mean()*1e3:.1f}/{resid.max()*1e3:.1f} mm")

    # 目标点: 臂基座系 -> CCS (= 中心相机系) 经 arm_in_ccs
    arm_y = load_yaml(CFG_DIR / "arm_pose" / f"{day}.yaml")["arms"]
    tgt = {}
    for arm in ("upper", "lower"):
        p = np.loadtxt(CFG_DIR / "gaze_target" / P / f"piper_{arm}.txt", delimiter=",")
        cc = arm_y[arm]["arm_in_ccs"]
        R = zxz_R(*cc["rotation_zxz"])
        t = np.array(cc["translation"], dtype=float)
        tgt[arm] = p @ R.T + t

    tw_u, tw_l = tgt["upper"], tgt["lower"]
    all_tw = np.vstack([tw_u, tw_l])
    all_dirs = all_tw - eye
    pitch, yaw = angles_from(all_dirs)
    n = len(pitch)
    m = (np.abs(pitch) <= LIM) & (np.abs(yaw) <= LIM)
    print(f"samples={n}  in-range={m.sum()}"
          f"  pitch[{pitch.min():.1f},{pitch.max():.1f}]"
          f"  yaw[{yaw.min():.1f},{yaw.max():.1f}]")

    # ---- 窗口 1: 世界系 (中心相机系) 3D;  窗口 2: pitch×yaw 密度图 ----
    cmap = LinearSegmentedColormap.from_list(           # turbo 截去深蓝端: 色柱从浅蓝起
        'turbo_lb', matplotlib.colormaps['turbo'](np.linspace(0.22, 1.0, 256)))

    fig3 = plt.figure(figsize=(10, 9))
    fig3.suptitle(f"{P} — gaze targets in world frame ({center_sn} camera, "
                  f"origin = its optical center)", fontsize=13)
    ax3 = fig3.add_subplot(1, 1, 1, projection="3d")
    for tw, c, lb in ((tw_u, "crimson", "targets upper"),
                      (tw_l, "darkorange", "targets lower")):
        ax3.scatter(tw[:, 0], tw[:, 1], tw[:, 2], c=c, s=6, alpha=0.5, label=lb)
        seg = np.stack([np.repeat(eye[None], len(tw), axis=0), tw], axis=1)
        ax3.plot(seg[:, :, 0].T, seg[:, :, 1].T, seg[:, :, 2].T,
                 color=c, lw=0.3, alpha=0.25)
    ax3.scatter(*eye, c="black", s=80, marker="X", label="eye (axis cross)")
    ax3.text(eye[0] + 0.02, eye[1], eye[2],
             f"eye ({eye[0]:.3f}, {eye[1]:.3f}, {eye[2]:.3f}) m",
             fontsize=9, fontweight="bold")
    # 世界系三轴 (RGB = XYZ), 原点 = 中心相机光心
    L = 0.1
    for d, c, lb in (((L, 0, 0), "red", "x"),
                     ((0, L, 0), "green", "y"),
                     ((0, 0, L), "blue", "z")):
        ax3.quiver(0, 0, 0, *d, color=c, lw=1.8, arrow_length_ratio=0.12)
        ax3.text(d[0] * 1.2, d[1] * 1.2, d[2] * 1.2, lb,
                 color=c, fontsize=11, fontweight="bold")
    ax3.set_xlabel("x (m)")
    ax3.set_ylabel("y (m)")
    ax3.set_zlabel("z (m)")
    ax3.set_title(f"day={day}, {len(poses)}-cam axis cross")
    ax3.legend(fontsize=8)
    ax3.view_init(elev=90, azim=90)            # 初始视角: z 出屏, x 向左, y 向下

    fig = plt.figure(figsize=(9, 9))
    fig.suptitle(f"{P} — gaze direction in world frame", fontsize=13)
    ax = fig.add_subplot(1, 1, 1)
    ax.set_facecolor(BG)
    # 每个样本点按 KDE 局部密度着色 (无格子填充): 低→高 = 浅蓝→红
    xy = np.vstack([yaw[m], pitch[m]])
    dens = gaussian_kde(xy)(xy) * 100.0                # % 每平方度
    o = np.argsort(dens)                               # 密点最后画, 顶层可见
    vmin = np.quantile(dens, 0.05)                     # 底端浅蓝 (不融进背景)
    vmax = dens.max() / 3.0                            # 峰值 1/3 处饱和
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
