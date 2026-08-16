#!/usr/bin/env python3
"""3D viz Piper chain — Cam · Arm · Flange · Calib(est+GT).  ← → keys, sidebar stats."""
import sys, math, numpy as np
from pathlib import Path
import yaml

import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import proj3d

# ---------------------------------------------------------------------------
def quat_to_rot(qx, qy, qz, qw):
    r00=1-2*qy*qy-2*qz*qz; r01=2*qx*qy-2*qz*qw; r02=2*qx*qz+2*qy*qw
    r10=2*qx*qy+2*qz*qw; r11=1-2*qx*qx-2*qz*qz; r12=2*qy*qz-2*qx*qw
    r20=2*qx*qz-2*qy*qw; r21=2*qy*qz+2*qx*qw; r22=1-2*qx*qx-2*qy*qy
    return np.array([[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]])

COLORS = {"X":(1,0.15,0.15),"Y":(0.15,1,0.15),"Z":(0.25,0.25,1)}

def draw_axes(ax, pos, quat, length=0.04, lw=2.5, alpha=1.0):
    R = quat_to_rot(*quat)
    for i, c in enumerate(COLORS.values()):
        ax.quiver(pos[0],pos[1],pos[2], R[0,i]*length,R[1,i]*length,R[2,i]*length,
                  color=c, linewidth=lw, arrow_length_ratio=0.25, alpha=alpha)

def draw_text(ax, pos, text, color="#2B2B2B", dx=14, dy=0):
    # 悬浮信息: 投影到屏幕后按像素偏移锚定 (dx/dy: 显示像素),
    # 锚点(坐标系原点)始终位于方框外, 且各标签可用 dy 上下错开
    x2, y2, _ = proj3d.proj_transform(pos[0], pos[1], pos[2], ax.get_proj())
    ax.text2D(x2 + dx, y2 + dy, text, fontsize=9, color=color,
              bbox=dict(boxstyle="round,pad=0.15",fc="#FAFAF2",ec="#9E9E9E",lw=0.6,alpha=0.92))

# ---------------------------------------------------------------------------
def load(path):
    lines = Path(path).read_text().strip().splitlines()
    arm_ccs = None; frames = []
    for ln in lines:
        if ln.startswith("# arm_in_ccs:"):
            v = list(map(float, ln.split()[2:]))
            arm_ccs = (np.array(v[:3]), np.array(v[3:7]))
        elif not ln.startswith("#"):
            v = list(map(float, ln.split()[1:]))
            fp,fq = np.array(v[0:3]), np.array(v[3:7])
            cp,cq = np.array(v[7:10]),np.array(v[10:14])
            bp,bq = np.array(v[14:17]),np.array(v[17:21])
            idx = int(ln[:2])
            valid = not math.isnan(bp[0])
            frames.append((idx, fp,fq, cp,cq, bp,bq, valid))
    return arm_ccs, frames

def find_piper_yaml():
    cand = Path(__file__).resolve().parent
    for _ in range(4):
        cand = cand.parent
        t = cand / "cfg" / "piper.yaml"
        if t.exists(): return t
    return None

# ---------------------------------------------------------------------------
class App:
    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
        self.arm = "upper"
        self.frames = {}     # arm → [valid frames]
        self.stats = {}      # arm → stats dict
        self.arm_ccs = {}
        self.n = 0
        self.i = 0           # current frame index
        self.AL = 0.05

        self.fig = plt.figure(figsize=(12.5, 9))
        self.ax = self.fig.add_axes([0.05, 0.08, 0.62, 0.88], projection="3d")
        self.ax_side = self.fig.add_axes([0.69, 0.06, 0.30, 0.90])
        self.ax_side.axis("off")
        plt.subplots_adjust(bottom=0.12)

        # 圆角按钮 (UPPER / LOWER)
        self.btn_patches = {}
        for i, name in enumerate(("upper", "lower")):
            axb = plt.axes([0.15 + i * 0.16, 0.015, 0.13, 0.045])
            axb.axis("off")
            axb.set_xlim(0, 1); axb.set_ylim(0, 1)
            # 圆角矩形按钮: pad=0 保证圆弧不被裁剪
            patch = mpatches.FancyBboxPatch((0, 0), 1, 1, transform=axb.transAxes,
                                            boxstyle="round,pad=0,rounding_size=0.35",
                                            fc="#37474F", ec="none")
            patch.set_clip_on(False)
            axb.add_patch(patch)
            lab = axb.text(0.5, 0.5, name.upper(), transform=axb.transAxes,
                           ha="center", va="center", fontsize=10, color="white",
                           family="monospace", fontweight="bold")
            self.btn_patches[name] = (patch, lab, axb)
        self.fig.canvas.mpl_connect("button_press_event", self.on_btn_click)

        # pre-load both arms + print stats
        for arm in ("upper","lower"):
            self.load_arm_data(arm)
        self._print_all_stats()
        self.switch_arm("upper")

    def load_arm_data(self, arm):
        path = self.data_dir / f"chain_viz_{arm}.txt"
        if not path.exists():
            # chain 文件缺失 (save_piper_chain 尚未运行): 填默认值, 避免后续 KeyError
            print(f"[Warn] {path} not found — run save_piper_chain first.")
            self.frames[arm] = []
            self.arm_ccs[arm] = None
            self.stats[arm] = dict(n_valid=0, n_total=0,
                                   pos_mm=np.array([0]), rot_deg=np.array([0]))
            return
        ccs, all_f = load(path)
        self.arm_ccs[arm] = ccs
        valid_f = [f for f in all_f if f[-1]]
        self.frames[arm] = valid_f
        # compute stats
        errs_p, errs_r = [], []
        for _, fp,fq, cp,cq, bp,bq, _ in valid_f:
            errs_p.append(np.linalg.norm(cp-bp)*1000)
            R_err = quat_to_rot(*cq).T @ quat_to_rot(*bq)
            tr = np.trace(R_err); angle = math.acos(max(-1,min(1,(tr-1)/2)))*180/math.pi
            if not math.isnan(angle): errs_r.append(angle)
        self.stats[arm] = dict(
            n_valid=len(valid_f), n_total=len(all_f),
            pos_mm=np.array(errs_p) if errs_p else np.array([0]),
            rot_deg=np.array(errs_r) if errs_r else np.array([0]),
        )

    def _print_all_stats(self):
        print("\n" + "="*60)
        for arm in ("upper","lower"):
            s = self.stats[arm]
            if s["n_valid"] == 0: continue
            print(f"  {arm.upper():6s}  valid={s['n_valid']}/{s['n_total']}  "
                  f"pos(mm): mean={s['pos_mm'].mean():.2f} median={np.median(s['pos_mm']):.2f} "
                  f"max={s['pos_mm'].max():.2f} std={s['pos_mm'].std():.2f}  |  "
                  f"rot(deg): mean={s['rot_deg'].mean():.2f} median={np.median(s['rot_deg']):.2f} "
                  f"max={s['rot_deg'].max():.2f} std={s['rot_deg'].std():.2f}")
        print("="*60 + "\n  ← → keys to step. Close window to exit.\n")

    def on_btn_click(self, event):
        for name, (_, _, axb) in self.btn_patches.items():
            if event.inaxes == axb and name != self.arm:
                self.switch_arm(name)
                return

    def _update_btn_colors(self):
        for name, (patch, lab, _) in self.btn_patches.items():
            active = (name == self.arm)
            patch.set_facecolor("#00838F" if active else "#37474F")
            lab.set_color("white" if active else "#AFAFAF")

    def switch_arm(self, arm):
        self.arm = arm
        self.n = len(self.frames[arm])
        self.i = 0
        self._update_btn_colors()
        self.update()
        self.fig.canvas.draw_idle()

    def update(self):
        self.ax.clear()
        self.ax_side.clear(); self.ax_side.axis("off")
        if self.n == 0: return

        idx, fp,fq, cp,cq, bp,bq, _ = self.frames[self.arm][self.i]
        ap, aq = self.arm_ccs[self.arm]
        err_mm = np.linalg.norm(cp-bp)*1000
        R_err = quat_to_rot(*cq).T @ quat_to_rot(*bq)
        tr = np.trace(R_err); err_deg = math.acos(max(-1,min(1,(tr-1)/2)))*180/math.pi

        # 3D axes (悬浮框屏幕锚定: 锚点在框外, 标签间按像素上下错开)
        draw_axes(self.ax, np.zeros(3), (0,0,0,1), self.AL, lw=2.5, alpha=0.9)
        draw_text(self.ax, np.zeros(3), "Cam", dx=16, dy=20)
        draw_axes(self.ax, ap, aq, self.AL, lw=2.5, alpha=0.8)
        draw_text(self.ax, ap, "Arm", dx=16, dy=20)
        self.ax.plot([0,ap[0]],[0,ap[1]],[0,ap[2]],"gray",lw=0.6,ls="--")
        draw_axes(self.ax, fp, fq, self.AL*0.8, lw=2, alpha=0.8)
        draw_text(self.ax, fp, f"Flange [{idx:02d}]", dx=16, dy=-30)
        self.ax.plot([ap[0],fp[0]],[ap[1],fp[1]],[ap[2],fp[2]],"cyan",lw=0.6,ls="--")
        draw_axes(self.ax, cp, cq, self.AL*0.8, lw=2, alpha=0.85)
        self.ax.plot([fp[0],cp[0]],[fp[1],cp[1]],[fp[2],cp[2]],"orange",lw=0.6,ls="--")
        draw_axes(self.ax, bp, bq, self.AL*0.8, lw=2.5, alpha=0.9)
        self.ax.plot([cp[0],bp[0]],[cp[1],bp[1]],[cp[2],bp[2]],"red",lw=2)

        # Calib(est)/Board(GT)/Δ 三框上下分开: 分别投影到 cp / bp / 中点, 像素级错开
        draw_text(self.ax, cp, "Calib(est)", dx=16, dy=34)
        draw_text(self.ax, bp, "Board(GT)", dx=16, dy=2)
        mid = (cp + bp) / 2
        draw_text(self.ax, mid, f"Δ{err_mm:.1f}mm", color="#C62828", dx=16, dy=-30)

        pts = [np.zeros(3),ap,fp,cp,bp]; pts = np.array(pts)
        lo,hi = pts.min(0),pts.max(0); pad = max((hi-lo).max()*0.25,0.06)
        mid = (lo+hi)/2; r = (hi-lo).max()/2+pad
        self.ax.set_xlim(mid[0]-r,mid[0]+r); self.ax.set_ylim(mid[1]-r,mid[1]+r)
        self.ax.set_zlim(mid[2]-r,mid[2]+r)
        self.ax.set_xlabel("X (m)"); self.ax.set_ylabel("Y (m)"); self.ax.set_zlabel("Z (m)")
        self.ax.set_title(f"[{self.arm.upper()}] Frame {idx:02d}  ({self.i+1}/{self.n})",fontsize=13)
        # 初始视角: 指向相机坐标系 z 轴负方向, x 轴向左, y 轴向下 (与 viz_calib_chain 一致)
        self.ax.view_init(elev=90, azim=90)

        # sidebar (统计面板: 圆角面板 + 深色标题栏 + 两列对齐 + 分隔线)
        s = self.stats[self.arm]
        self.ax_side.set_xlim(0, 1); self.ax_side.set_ylim(0, 1)
        panel = mpatches.FancyBboxPatch((0, 0), 1, 1, transform=self.ax_side.transAxes,
                                        boxstyle="round,pad=0.015,rounding_size=0.03",
                                        fc="#F8F8F4", ec="#B0B0B0", lw=1.2, zorder=0)
        self.ax_side.add_patch(panel)
        hdr = mpatches.FancyBboxPatch((0.03, 0.905), 0.94, 0.08, transform=self.ax_side.transAxes,
                                      boxstyle="round,pad=0.004,rounding_size=0.025",
                                      fc="#37474F", ec="none", zorder=1)
        self.ax_side.add_patch(hdr)
        self.ax_side.text(0.5, 0.945, f"{self.arm.upper()}  |  Frame {idx:02d}  ({self.i+1}/{self.n})",
                          transform=self.ax_side.transAxes, ha="center", va="center",
                          fontsize=11, color="white", family="monospace", fontweight="bold")

        GOOD_POS, GOOD_ROT = 10.0, 5.0   # 误差着色阈值 (mm / deg)
        def hdr_txt(y, txt, ac="#00838F"):
            # 节标题: 左侧彩色圆角强调条 + 深灰加粗文字
            bar = mpatches.FancyBboxPatch((0.055, y - 0.014), 0.010, 0.028,
                                          transform=self.ax_side.transAxes,
                                          boxstyle="round,pad=0,rounding_size=0.5",
                                          fc=ac, ec="none", zorder=2)
            self.ax_side.add_patch(bar)
            self.ax_side.text(0.085, y, txt, transform=self.ax_side.transAxes,
                              fontsize=10, color="#37474F", family="monospace",
                              fontweight="bold", ha="left", va="center")
        def pair(y, label, value, vc="#2B2B2B"):
            self.ax_side.text(0.07, y, label, transform=self.ax_side.transAxes,
                              fontsize=9.5, color="#2B2B2B", family="monospace",
                              ha="left", va="center")
            self.ax_side.text(0.93, y, value, transform=self.ax_side.transAxes,
                              fontsize=9.5, color=vc, family="monospace",
                              fontweight="bold", ha="right", va="center")
        def sub(y, txt):
            self.ax_side.text(0.07, y, txt, transform=self.ax_side.transAxes,
                              fontsize=9, color="#555555", family="monospace",
                              ha="left", va="center")
        def sep(y):
            self.ax_side.plot([0.07, 0.93], [y, y], transform=self.ax_side.transAxes,
                              color="#D0D0D0", lw=0.8, zorder=2)
        def frac_bar(y, frac):
            self.ax_side.add_patch(mpatches.FancyBboxPatch((0.07, y), 0.86, 0.014,
                                  transform=self.ax_side.transAxes,
                                  boxstyle="round,pad=0,rounding_size=0.5",
                                  fc="#E4E4DE", ec="none", zorder=2))
            if frac > 0:
                self.ax_side.add_patch(mpatches.FancyBboxPatch((0.07, y), 0.86 * min(frac, 1.0), 0.014,
                                      transform=self.ax_side.transAxes,
                                      boxstyle="round,pad=0,rounding_size=0.5",
                                      fc="#00838F", ec="none", zorder=3))

        y = 0.855
        hdr_txt(y, "THIS FRAME", "#00838F"); y -= 0.055
        pair(y, "dPos", f"{err_mm:6.2f} mm", "#2E7D32" if err_mm < GOOD_POS else "#C62828"); y -= 0.052
        pair(y, "dRot", f"{err_deg:6.2f} deg", "#2E7D32" if err_deg < GOOD_ROT else "#C62828"); y -= 0.062
        sep(y + 0.031)
        hdr_txt(y, "ALL FRAMES", "#E65100"); y -= 0.055
        pair(y, "valid", f"{s['n_valid']:2d} / {s['n_total']:2d}"); y -= 0.042
        frac_bar(y + 0.021, s['n_valid'] / s['n_total'] if s['n_total'] else 0.0); y -= 0.042
        sub(y, "pos (mm)"); y -= 0.042
        pair(y, "mean", f"{s['pos_mm'].mean():6.2f}"); y -= 0.038
        pair(y, "med", f"{np.median(s['pos_mm']):6.2f}"); y -= 0.038
        pair(y, "max", f"{s['pos_mm'].max():6.2f}"); y -= 0.038
        pair(y, "std", f"{s['pos_mm'].std():6.2f}"); y -= 0.048
        sub(y, "rot (deg)"); y -= 0.042
        pair(y, "mean", f"{s['rot_deg'].mean():6.2f}"); y -= 0.038
        pair(y, "med", f"{np.median(s['rot_deg']):6.2f}"); y -= 0.038
        pair(y, "max", f"{s['rot_deg'].max():6.2f}"); y -= 0.038
        pair(y, "std", f"{s['rot_deg'].std():6.2f}"); y -= 0.056
        sep(y + 0.028)
        self.ax_side.text(0.5, y, "<-  ->  step frame", transform=self.ax_side.transAxes,
                          fontsize=8.5, color="#888888", family="monospace",
                          ha="center", va="center")

        self.fig.canvas.draw_idle()

    def on_key(self, e):
        if e.key == "right" and self.i < self.n-1:
            self.i += 1; self.update()
        elif e.key == "left" and self.i > 0:
            self.i -= 1; self.update()

# ---------------------------------------------------------------------------
def main():
    yaml_path = find_piper_yaml()
    if yaml_path is None: print("Cannot find piper.yaml"); sys.exit(1)
    with open(yaml_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    # day_id 来自 calib_arm.yaml (与 save_piper_chain/test_record_arm_data 一致)
    day_id = "D001"
    arm_cfg_path = yaml_path.parent / "calib_arm.yaml"
    if arm_cfg_path.exists():
        with open(arm_cfg_path, encoding="utf-8") as f:
            arm_cfg = yaml.safe_load(f)
        day_id = arm_cfg["record"]["day_id"]
    data_dir = Path(cfg["test_record_arm_data"]["calib_save_dir"]) / day_id
    app = App(data_dir)
    app.fig.canvas.mpl_connect("key_press_event", app.on_key)
    plt.show()

if __name__=="__main__": main()
