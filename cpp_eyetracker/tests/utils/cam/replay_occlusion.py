#!/usr/bin/env python3
# replay_occlusion.py — 离线复现遮挡检测 (与 capture_with_occlusion_detection.cpp 逐条对齐)
#
# 数据源 (h5, 每相机): occ_meta [N,42] (运行时真值, 不依赖本地 yaml)
#   [0]arm [1]status [2:4]check_err [4:18]flange [18:30]arm_pose [30:42]joints
# 几何链 (与 C++ 一致):
#   FK:      t += R@origin_xyz;  R = R@Rpy(rpy)@Rz(q)     (rpy = Rz(y)·Ry(p)·Rx(r))
#   世界系:  p_w = Rw·(R·v + t) + ccs_t                   (zxz: Rz(a)·Rx(b)·Rz(g), 度)
#   相机系:  p_cam = R^T·(p_w − T)                        (gba: Rx(a)·Ry(b)·Rz(g), 度)
#   内参:    fx = focus/sx;  u = fx·x/z + cx
#   判定:    近平面 z≥1e-4 Sutherland-Hodgman 裁剪 → 与内缩 200px 矩形:
#           顶点入矩形 / 边相交(Liang-Barsky) / 画面中心在多边形内
#   网格:    base_link + link1..6 (gripper_base 剔除, link7/8 不含)
#
# 用法:
#   python replay_occlusion.py                    # 全部录制 × 本机全部相机 (capture.yaml)
#   python replay_occlusion.py --rec 5 100 200    # 指定全局录号
#   python replay_occlusion.py --cams 40774056    # 指定相机
#   python replay_occlusion.py -v                 # 逐录打印
import argparse, glob, json, math, os, sys
import numpy as np
import h5py
import xml.etree.ElementTree as ET
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = HERE.removesuffix(os.sep + "tests" + os.sep + "utils" + os.sep + "cam")   # cpp_eyetracker
PROJ = os.path.dirname(os.path.dirname(REPO))              # new_dataset (与 C++ cfg 上三级一致)
CFG_DIR = os.path.join(REPO, "cfg")
URDF_PATH = os.path.join(PROJ, "piper_ros", "src", "piper_description",
                         "urdf", "piper_description.urdf")
MESH_DIR = os.path.join(PROJ, "piper_ros", "src", "piper_description", "meshes")
EDGE_MARGIN = 200.0          # kOccEdgeMargin
NEAR_EPS = 1e-4              # 近平面 0.1mm
CAP = 2000                   # hdf5_chunk_frame_capacity


# ---------------- 基础旋转 ----------------
def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


# ---------------- URDF / STL ----------------
def load_urdf_chain(path):
    """返回 [(link_name, origin_xyz, origin_R)] 链: joint1..6 → link1..6 (gripper_base 不含)"""
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])])
        r, p, y = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        joints[j.get("name")] = (parent, child, xyz, _rz(y) @ _ry(p) @ _rx(r))
    chain, cur = [], "base_link"
    for _ in range(6):
        jn = next(n for n, v in joints.items()
                  if v[0] == cur and v[1].startswith("link"))     # joint1..6 → linkN
        _, child, xyz, R = joints[jn]
        chain.append((child, xyz, R))
        cur = child
    return chain


def load_stl(path):
    """二进制 STL → [n,3,3] (向量化读; ASCII STL 不支持——piper 网格均为二进制)"""
    with open(path, "rb") as f:
        data = f.read()
    n = int.from_bytes(data[80:84], "little")
    raw = np.frombuffer(data[84:], dtype=np.uint8, count=n * 50).reshape(n, 50)[:, 12:48].copy()
    return raw.view("<f4").reshape(n, 3, 3).astype(np.float64)


def load_arm_meshes():
    """base_link + link1..6 的 [k,3,3] 网格列表 (与 C++ push_link 顺序一致)"""
    root = ET.parse(URDF_PATH).getroot()
    meshes = {}
    for l in root.findall("link"):
        m = l.find("visual/geometry/mesh")
        if m is not None:
            meshes[l.get("name")] = os.path.join(MESH_DIR, m.get("filename").split("/")[-1])
    out = [load_stl(meshes["base_link"])]
    for i in range(1, 7):
        out.append(load_stl(meshes[f"link{i}"]))
    return out


# ---------------- 相机 XML ----------------
def load_cam(sn, xml_dir):
    x = ET.parse(os.path.join(xml_dir, f"{sn}_Data.xml")).getroot()
    raw = x.find("InternalParameters/RawData").text.split()
    foc, sx, sy, cx, cy = float(raw[1]), float(raw[3]), float(raw[4]), float(raw[5]), float(raw[6])
    w, h = int(float(raw[7])), int(float(raw[8]))
    e = x.find("ExternalParameters")
    T = np.array([float(e.find(f"Translation/{k}").text) for k in "XYZ"])
    a, b, g = (math.radians(float(e.find(f"Rotation/{k}").text)) for k in ("Alpha", "Beta", "Gamma"))
    return dict(sn=sn, fx=foc / sx, fy=foc / sy, cx=cx, cy=cy, w=w, h=h,
                R=_rx(a) @ _ry(b) @ _rz(g), T=T)


# ---------------- FK → 世界系三角形 ----------------
def fk_world_tris(chain, meshes, q, arm_pose):
    """两段变换与 C++ occFkWorld 一致; 返回 [n,3,3] 世界系顶点三角形"""
    a, b, g = (math.radians(v) for v in arm_pose[3:6])         # upper zxz
    Rw = _rz(a) @ _rx(b) @ _rz(g)
    tw = np.array(arm_pose[0:3])
    R, t, qi, out = np.eye(3), np.zeros(3), 0, []
    for (name, xyz, R0), mesh in zip(chain, meshes[1:]):       # link1..6 (meshes[0]=base)
        t = t + R @ xyz
        R = R @ R0 @ _rz(q[qi]); qi += 1
        out.append(mesh @ R.T + t)                              # [n,3,3] link 系
    out.insert(0, meshes[0])                                    # base_link: 臂基座系原样
    Vw = np.concatenate(out)                                    # 臂基座系全部顶点
    return (Vw @ Rw.T + tw)                                     # → 世界(CCS)


# ---------------- 判定 (向量化; 算法与 occTriInFrame 逐条对齐) ----------------
def seg_hits_rect(u0, v0, u1, v1, x0, y0, x1, y1):
    """Liang-Barsky 向量化 (与 occSegHitsRect 标量版逐条对应), 输入 [n] 数组"""
    n = len(u0)
    t0, t1 = np.zeros(n), np.ones(n)
    alive = np.ones(n, dtype=bool)
    du, dv = u1 - u0, v1 - v0
    for p_, q_ in ((-du, u0 - x0), (du, x1 - u0), (-dv, v0 - y0), (dv, y1 - v0)):
        r = np.full(n, np.inf)
        nz = p_ != 0
        r[nz] = q_[nz] / p_[nz]
        alive &= ~(~nz & (q_ < 0))                      # p==0 且 q<0 → 平行于边且在外
        neg, pos = nz & (p_ < 0), nz & (p_ > 0)
        alive[neg] &= ~(r[neg] > t1[neg])
        upd = neg & (r > t0); t0[upd] = r[upd]
        alive[pos] &= ~(r[pos] < t0[pos])
        upd = pos & (r < t1); t1[upd] = r[upd]
    return alive


def _clip_near_sh(P):
    """Sutherland-Hodgman 单平面 z>=eps 裁剪 (与 C++ occTriInFrame 逐行对应), 返回 [m,3]"""
    eps = NEAR_EPS
    Q = []
    for a in range(3):
        b = (a + 1) % 3
        ain, bin_ = P[a][2] >= eps, P[b][2] >= eps
        if ain:
            Q.append(P[a])
        if ain != bin_:
            t = (eps - P[a][2]) / (P[b][2] - P[a][2])
            Q.append(P[a] + t * (P[b] - P[a]))
    return np.array(Q) if len(Q) >= 3 else None


def replay_cam_occlusion(cam, Vw):
    """True = 复现判定为遮挡; 三重测试与 C++ occTriInFrame 一致"""
    Pc = (Vw - cam["T"]) @ cam["R"]                              # [n,3,3] 相机系
    z = Pc[..., 2]
    front = z >= NEAR_EPS
    all_front = front.all(axis=-1)
    all_back = ~front.any(axis=-1)
    cross_plane = ~all_front & ~all_back
    occl = np.zeros(len(Vw), dtype=bool)
    x0, y0 = EDGE_MARGIN, EDGE_MARGIN
    x1, y1 = cam["w"] - EDGE_MARGIN, cam["h"] - EDGE_MARGIN

    if all_front.any():
        idx = np.where(all_front)[0]
        P = Pc[idx]
        u = cam["fx"] * P[..., 0] / P[..., 2] + cam["cx"]
        v = cam["fy"] * P[..., 1] / P[..., 2] + cam["cy"]
        # 粗筛: 三角形 uv 包围盒与矩形相交 (三重判定的必要条件)
        cand = ((u.max(-1) >= x0) & (u.min(-1) <= x1)
                & (v.max(-1) >= y0) & (v.min(-1) <= y1))
        if cand.any():
            k = idx[cand]; uu, vv = u[cand], v[cand]
            # (1) 任一顶点入内矩形
            hit = ((uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1)).any(axis=-1)
            # (2) 任一边与矩形相交 (Liang-Barsky)
            if not hit.all():
                for a in range(3):
                    b = (a + 1) % 3
                    rest = ~hit
                    if not rest.any():
                        break
                    hit[rest] |= seg_hits_rect(uu[rest, a], vv[rest, a], uu[rest, b], vv[rest, b],
                                               x0, y0, x1, y1)
            # (3) 画面中心在三角形内 (叉积同号: 三边全 >=0 或全 <=0)
            if not hit.all():
                rest = ~hit
                px, py = cam["w"] / 2, cam["h"] / 2
                crs = np.stack([((uu[rest, (a + 1) % 3] - uu[rest, a]) * (py - vv[rest, a])
                                - (vv[rest, (a + 1) % 3] - vv[rest, a]) * (px - uu[rest, a]))
                               for a in range(3)], axis=-1)
                hit[rest] |= np.all(crs >= 0, axis=-1) | np.all(crs <= 0, axis=-1)
            occl[k] |= hit

    # 跨近平面三角形 (<0.5%): 与 C++ 一致 — S-H 裁剪后三项测试 (Python 循环, 数量小)
    px, py = cam["w"] / 2, cam["h"] / 2
    for i in np.where(cross_plane)[0]:
        Q = _clip_near_sh(Pc[i])
        if Q is None:
            continue
        uu = cam["fx"] * Q[:, 0] / Q[:, 2] + cam["cx"]
        vv = cam["fy"] * Q[:, 1] / Q[:, 2] + cam["cy"]
        n = len(Q)
        # bbox 快速排除 (三项判定的必要条件)
        if not (uu.max() >= x0 and uu.min() <= x1 and vv.max() >= y0 and vv.min() <= y1):
            continue
        if ((uu >= x0) & (uu <= x1) & (vv >= y0) & (vv <= y1)).any():     # (1)
            occl[i] = True; continue
        hit = False
        for a in range(n):                                                # (2)
            b = (a + 1) % n
            if seg_hits_rect(np.array([uu[a]]), np.array([vv[a]]),
                             np.array([uu[b]]), np.array([vv[b]]), x0, y0, x1, y1)[0]:
                hit = True; break
        if not hit:                                                       # (3)
            crs = [(uu[(a + 1) % n] - uu[a]) * (py - vv[a])
                   - (vv[(a + 1) % n] - vv[a]) * (px - uu[a]) for a in range(n)]
            hit = all(c >= 0 for c in crs) or all(c <= 0 for c in crs)
        if hit:
            occl[i] = True
    return occl.any()


# ---------------- 主流程 ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", type=int, nargs="*", default=None, help="全局录号 (缺省=全部)")
    ap.add_argument("--cams", nargs="*", default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(os.path.join(CFG_DIR, "capture.yaml"), encoding="utf-8"))
    pid = cfg["capture"]["participant_id"]
    roots = [r + "/" + pid for r in cfg["loader"]["participant_root"]]
    cams_cfg = cfg["loader"]["cam_indices"]
    cams_sel = args.cams or cams_cfg
    save_dir = yaml.safe_load(open(os.path.join(CFG_DIR, "cam_calib.yaml"), encoding="utf-8"))["calib"]["calib_save_dir"]
    day = json.load(open(os.path.join(CFG_DIR, "day_participant_map.json"), encoding="utf-8"))
    days = sorted(d for d, ps in day.items() if pid in ps)
    xml_dir = os.path.join(save_dir, days[-1], "output")

    chain = load_urdf_chain(URDF_PATH)
    meshes = load_arm_meshes()
    print(f"[setup] URDF 链 {len(chain)} 关节, 网格顶点 "
          f"{sum(m.shape[0] for m in meshes) * 3}, 相机 XML: {xml_dir}")

    cams = {sn: load_cam(sn, xml_dir) for sn in cams_sel}

    # ---- 预载: 每录 occ_meta (全局一致, 取首个相机) + 每相机 valid (录制级取中帧) ----
    meta, valid = {}, {}                       # meta[g] = 42 列; valid[sn][g] = 0/1
    n_skip = 0
    for sn, root in zip(cams_cfg, roots):
        if sn not in cams:
            continue
        for h5p in sorted(glob.glob(os.path.join(root, sn, "*.h5"))):
            ci = int(os.path.basename(h5p)[0:4])
            with h5py.File(h5p, "r") as f:
                if "occ_meta" not in f:
                    print(f"[{sn}] chunk{ci}: 无 occ_meta (旧格式, 跳过)"); n_skip += 1; continue
                M = f["occ_meta"][:]; V = f["valid"][:]; G = f["gaze_target"][:]
            for r in range(len(M) // 100):
                g, s = ci * (CAP // 100) + r, r * 100 + 50
                if abs(G[s][2]) < 1e-9:        # 未写区域
                    continue
                valid.setdefault(sn, {})[g] = int(V[s])
                if sn not in meta:
                    meta[g] = M[s].copy()
    recs = sorted(meta)
    if args.rec is not None:
        recs = [g for g in recs if g in args.rec]
    print(f"[data] 已写录制 {len(recs)} 个, {len(valid)} 台相机")

    # ---- 重放: 每录 FK 一次 (两臂), 全部相机复用 ----
    n_match = n_diff = 0
    diffs = []
    for gi, g in enumerate(recs):
        m = meta[g]
        Vw = np.concatenate([fk_world_tris(chain, meshes, m[30:36], m[18:24]),
                             fk_world_tris(chain, meshes, m[36:42], m[24:30])])
        for sn, cam in cams.items():
            if g not in valid.get(sn, {}):
                continue
            rep = replay_cam_occlusion(cam, Vw)
            h5v = valid[sn][g] == 0
            if rep == h5v:
                n_match += 1
            else:
                n_diff += 1
                diffs.append((g, sn, "h5=OCC" if h5v else "h5=OK", "rep=OCC" if rep else "rep=OK"))
            if args.verbose:
                print(f"[rec {g:3d}] {sn}: h5={'OCC' if h5v else 'ok'} rep={'OCC' if rep else 'ok'}"
                      f"{'  <== 不一致' if rep != h5v else ''}")
        if not args.verbose and (gi + 1) % 100 == 0:
            print(f"  ... {gi + 1}/{len(recs)} 录 (不一致 {n_diff})")
    print(f"\n===== 汇总: 一致 {n_match}, 不一致 {n_diff}, 跳过 {n_skip} =====")
    for g, sn, a, b in diffs[:40]:
        print(f"  rec{g:3d} {sn}: {a} vs {b}")
    if len(diffs) > 40:
        print(f"  ... 共 {len(diffs)} 条")
    sys.exit(0 if n_diff == 0 else 1)


if __name__ == "__main__":
    main()
