#!/usr/bin/env python3
# viz_occ_overlay.py — 遮挡检测投影可视化: 臂网格三角形叠加到实际帧
#
# 用途: 定位"图像有明显遮挡但判定为 OK"的相机 — 直接看投影臂与图像中
# 实际遮挡物的偏差 (模型缺件 / 投影偏移 / 只进边距带)。
# 分类: 绿=与内缩 200px 矩形相交 (判定遮挡) / 黄=入画面但只在边距带 /
#       红框=内缩矩形 (EDGE_MARGIN 口径)。
# 依赖: 复用 replay_occlusion.py 的全部几何函数 (与其保持逐条一致)。
#
# 用法: python viz_occ_overlay.py --cams 40768743 --rec 19 --frame 50
import argparse, glob, json, os, sys
import numpy as np
import h5py
import cv2
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from replay_occlusion import (load_urdf_chain, load_arm_meshes, load_cam, fk_world_tris,
                              EDGE_MARGIN, NEAR_EPS)

REPO = HERE.removesuffix(os.sep + "tests" + os.sep + "utils" + os.sep + "cam")
CFG_DIR = os.path.join(REPO, "cfg")


def project_tri_uv(cam, Vw):
    """世界系三角形 → 相机系, 返回 (uv[n,3,2], z[n,3], 全前向掩码)"""
    Pc = (Vw - cam["T"]) @ cam["R"]
    front = Pc[..., 2] >= NEAR_EPS
    z = np.where(front, Pc[..., 2], 1.0)                  # 防除零
    u = cam["fx"] * Pc[..., 0] / z + cam["cx"]
    v = cam["fy"] * Pc[..., 1] / z + cam["cy"]
    return np.stack([u, v], axis=-1), Pc[..., 2], front.all(axis=-1)


def tri_rect_overlap(uv, x0, y0, x1, y1):
    """三角形与矩形粗 overlap (bbox 相交 + 任一顶点入内) — 足够可视化分类"""
    return ((uv[..., 0].max(-1) >= x0) & (uv[..., 0].min(-1) <= x1)
            & (uv[..., 1].max(-1) >= y0) & (uv[..., 1].min(-1) <= y1)
            & (((uv[..., 0] >= x0) & (uv[..., 0] <= x1)
                & (uv[..., 1] >= y0) & (uv[..., 1] <= y1)).any(axis=-1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", nargs="+", required=True)
    ap.add_argument("--rec", type=int, required=True, help="h5 文件号 (19 → 0019.h5)")
    ap.add_argument("--frame", type=int, default=50, help="occ_meta 帧索引")
    ap.add_argument("--scale", type=float, default=0.25)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(os.path.join(CFG_DIR, "capture.yaml"), encoding="utf-8"))
    pid = cfg["capture"]["participant_id"]
    roots = [r + "/" + pid for r in cfg["loader"]["participant_root"]]
    cams_cfg = cfg["loader"]["cam_indices"]
    save_dir = yaml.safe_load(open(os.path.join(CFG_DIR, "cam_calib.yaml"),
                                    encoding="utf-8"))["calib"]["calib_save_dir"]
    day = json.load(open(os.path.join(CFG_DIR, "day_participant_map.json"), encoding="utf-8"))
    days = sorted(d for d, ps in day.items() if pid in ps)
    xml_dir = os.path.join(save_dir, days[-1], "output")

    chain = load_urdf_chain(os.path.join(
        os.path.dirname(os.path.dirname(REPO)), "piper_ros", "src", "piper_description",
        "urdf", "piper_description.urdf"))
    meshes = load_arm_meshes()
    print(f"[setup] 网格顶点 {sum(m.shape[0] for m in meshes) * 3}, XML: {xml_dir}")

    for sn in args.cams:
        # 找该相机的 h5 (在任一 root 下)
        h5p = next((p for r in roots for p in glob.glob(os.path.join(r, sn, f"{args.rec:04d}.h5"))), None)
        if h5p is None:
            print(f"[{sn}] 找不到 {args.rec:04d}.h5"); continue
        cam = load_cam(sn, xml_dir)
        with h5py.File(h5p, "r") as f:
            M = f["occ_meta"][args.frame]
            V = int(f["valid"][args.frame])
            img = f["raw_image"][args.frame]
        print(f"[{sn}] {os.path.basename(h5p)} frame{args.frame}: valid={V} (0=遮挡)")

        Vw = np.concatenate([fk_world_tris(chain, meshes, M[30:36], M[18:24]),
                             fk_world_tris(chain, meshes, M[36:42], M[24:30])])
        uv, z, all_front = project_tri_uv(cam, Vw)
        x0, y0 = EDGE_MARGIN, EDGE_MARGIN
        x1, y1 = cam["w"] - EDGE_MARGIN, cam["h"] - EDGE_MARGIN
        # 分类 (只统计全前向三角形; 与 replay 口径一致)
        hit = np.zeros(len(uv), dtype=bool)          # 绿: 判遮挡
        band = np.zeros(len(uv), dtype=bool)         # 黄: 只进边距带
        m = all_front.copy()
        hit[m] = tri_rect_overlap(uv[m], x0, y0, x1, y1)
        mb = m & ~hit
        band[mb] = tri_rect_overlap(uv[mb], 0, 0, cam["w"], cam["h"])
        n_cross = int((~all_front).sum())
        print(f"  三角形 {len(uv)}: 遮挡命中 {hit.sum()}, 仅边距带 {band.sum()}, "
              f"跨近平面 {n_cross}; z 范围 [{z.min():.3f}, {z.max():.3f}] m")

        # 前向三角形投影整体范围 (诊断整体偏移)
        if all_front.any():
            fu, fv = uv[all_front, :, 0], uv[all_front, :, 1]
            print(f"  前向投影范围: u [{fu.min():.0f}, {fu.max():.0f}] "
                  f"v [{fv.min():.0f}, {fv.max():.0f}]  (画面 {cam['w']}x{cam['h']}, "
                  f"内缩矩形 [{x0:.0f},{y0:.0f}]-[{x1:.0f},{y1:.0f}])")

        # 跨近平面: S-H 裁剪后投影, 橙色; 统计裁剪后命中
        cross_hit = cross_band = 0
        Pc = (Vw - cam["T"]) @ cam["R"]
        for i in np.where(~all_front)[0]:
            from replay_occlusion import _clip_near_sh
            Q = _clip_near_sh(Pc[i])
            if Q is None:
                continue
            uu = cam["fx"] * Q[:, 0] / Q[:, 2] + cam["cx"]
            vv = cam["fy"] * Q[:, 1] / Q[:, 2] + cam["cy"]
            in_full = ((uu >= 0) & (uu <= cam["w"]) & (vv >= 0) & (vv <= cam["h"])).any()
            if not in_full:
                continue
            in_margin = ((uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1)).any()
            cross_hit += in_margin
            cross_band += not in_margin
        print(f"  跨近平面裁剪后: 命中内缩矩形 {cross_hit}, 仅边距带 {cross_band}")

        # 叠加绘制
        s = args.scale
        vis = cv2.resize(img, (int(cam["w"] * s), int(cam["h"] * s)))
        if len(vis.shape) == 2:
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(vis, (int(x0 * s), int(y0 * s)), (int(x1 * s), int(y1 * s)), (0, 0, 255), 2)
        for mask, color in ((band, (0, 255, 255)), (hit, (0, 255, 0))):
            idx = np.where(mask)[0]
            for i in idx[:: max(1, len(idx) // 4000)]:        # 抽样 ≤4000 个三角形
                pts = (uv[i] * s).astype(np.int32)
                cv2.polylines(vis, [pts.reshape(-1, 1, 2)], True, color, 1, cv2.LINE_AA)
        out = os.path.join(HERE, f"overlay_{sn}_{args.rec:04d}_{args.frame}.png")
        cv2.imwrite(out, vis)
        print(f"  → {out}  (绿=判定遮挡, 黄=仅边距带, 红框=内缩{int(EDGE_MARGIN)}px)")


if __name__ == "__main__":
    main()
