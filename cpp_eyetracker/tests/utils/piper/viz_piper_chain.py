#!/usr/bin/env python3
"""3D viz Piper chain — Cam · Arm · Flange · Calib(est+GT).  ← → keys, sidebar stats."""
import sys, math, numpy as np
from pathlib import Path
import yaml

import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

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

def draw_text(ax, pos, text, color="white"):
    ax.text(pos[0],pos[1],pos[2],text,fontsize=9,color=color,
            bbox=dict(boxstyle="round,pad=0.1",fc="black",ec="none",alpha=0.65))

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

        # buttons
        for i, name in enumerate(("upper","lower")):
            axb = plt.axes([0.15 + i*0.14, 0.02, 0.12, 0.04])
            btn = Button(axb, name.upper(), color="0.2", hovercolor="0.5")
            btn.label.set_color("white")
            btn.on_clicked(self.make_switch(name))
            setattr(self, f"btn_{name}", btn)

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

    def make_switch(self, name):
        def handler(event):
            if name != self.arm:
                self.switch_arm(name)
        return handler

    def switch_arm(self, arm):
        self.arm = arm
        self.n = len(self.frames[arm])
        self.i = 0
        for an in ("upper","lower"):
            btn = getattr(self, f"btn_{an}")
            btn.color = "0.4" if an==arm else "0.2"
            btn.label.set_color("lime" if an==arm else "white")
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

        # 3D axes
        draw_axes(self.ax, np.zeros(3), (0,0,0,1), self.AL, lw=2.5, alpha=0.9)
        draw_text(self.ax, np.zeros(3), "  Cam")
        draw_axes(self.ax, ap, aq, self.AL, lw=2.5, alpha=0.8)
        draw_text(self.ax, ap, "  Arm")
        self.ax.plot([0,ap[0]],[0,ap[1]],[0,ap[2]],"gray",lw=0.6,ls="--")
        draw_axes(self.ax, fp, fq, self.AL*0.8, lw=2, alpha=0.8)
        draw_text(self.ax, fp, f"  Flange [{idx:02d}]")
        self.ax.plot([ap[0],fp[0]],[ap[1],fp[1]],[ap[2],fp[2]],"cyan",lw=0.6,ls="--")
        draw_axes(self.ax, cp, cq, self.AL*0.8, lw=2, alpha=0.85)
        draw_text(self.ax, cp, "  Calib(est)")
        self.ax.plot([fp[0],cp[0]],[fp[1],cp[1]],[fp[2],cp[2]],"orange",lw=0.6,ls="--")
        draw_axes(self.ax, bp, bq, self.AL*0.8, lw=2.5, alpha=0.9)
        draw_text(self.ax, bp, "  Board(GT)")
        self.ax.plot([cp[0],bp[0]],[cp[1],bp[1]],[cp[2],bp[2]],"red",lw=2)
        mid = (cp+bp)/2; self.ax.text(mid[0],mid[1],mid[2],f"Δ{err_mm:.1f}mm",fontsize=8,color="red")

        pts = [np.zeros(3),ap,fp,cp,bp]; pts = np.array(pts)
        lo,hi = pts.min(0),pts.max(0); pad = max((hi-lo).max()*0.25,0.06)
        mid = (lo+hi)/2; r = (hi-lo).max()/2+pad
        self.ax.set_xlim(mid[0]-r,mid[0]+r); self.ax.set_ylim(mid[1]-r,mid[1]+r)
        self.ax.set_zlim(mid[2]-r,mid[2]+r)
        self.ax.set_xlabel("X (m)"); self.ax.set_ylabel("Y (m)"); self.ax.set_zlabel("Z (m)")
        self.ax.set_title(f"[{self.arm.upper()}] Frame {idx:02d}  ({self.i+1}/{self.n})",fontsize=13)
        self.ax.view_init(elev=-90, azim=-90)

        # sidebar
        s = self.stats[self.arm]
        rows = [
            (f"=== {self.arm.upper()} ===", True),
            (f"Frame: {idx:02d}  ({self.i+1}/{self.n})", False),
            ("", False),
            ("This frame:", True),
            (f"  Δpos  {err_mm:7.2f} mm", False),
            (f"  Δrot  {err_deg:7.2f} deg", False),
            ("", False),
            ("All frames:", True),
            (f"  Valid {s['n_valid']}/{s['n_total']}", False),
            ("  Pos (mm):", False),
            (f"    mean {s['pos_mm'].mean():7.2f}", False),
            (f"    med  {np.median(s['pos_mm']):7.2f}", False),
            (f"    max  {s['pos_mm'].max():7.2f}", False),
            (f"    std  {s['pos_mm'].std():7.2f}", False),
            ("  Rot (deg):", False),
            (f"    mean {s['rot_deg'].mean():7.2f}", False),
            (f"    med  {np.median(s['rot_deg']):7.2f}", False),
            (f"    max  {s['rot_deg'].max():7.2f}", False),
            (f"    std  {s['rot_deg'].std():7.2f}", False),
        ]
        y = 0.96; dy = 0.046
        for txt, bold in rows:
            self.ax_side.text(0.02, y, txt, transform=self.ax_side.transAxes,
                              fontsize=10, color="black", family="monospace",
                              fontweight="bold" if bold else "normal",
                              verticalalignment="top")
            y -= dy

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
