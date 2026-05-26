#!/usr/bin/env python3
"""3D visualization of Piper calibration chain — all 4 coordinate frames in CCS.

Camera · Arm base · Flange · Calib board (predicted + ground truth).
Reads ``chain_viz_<arm>.txt`` from ``save_piper_chain.cpp``.
"""
import sys, math, numpy as np
from pathlib import Path
import yaml

import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ---------------------------------------------------------------------------
def quat_to_rot(qx, qy, qz, qw):
    r00=1-2*qy*qy-2*qz*qz; r01=2*qx*qy-2*qz*qw; r02=2*qx*qz+2*qy*qw
    r10=2*qx*qy+2*qz*qw; r11=1-2*qx*qx-2*qz*qz; r12=2*qy*qz-2*qx*qw
    r20=2*qx*qz-2*qy*qw; r21=2*qy*qz+2*qx*qw; r22=1-2*qx*qx-2*qy*qy
    return np.array([[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]])

COLORS = {"X":(1.0,0.15,0.15), "Y":(0.15,1.0,0.15), "Z":(0.25,0.25,1.0)}

def draw_axes(ax, pos, quat, length=0.04, lw=2.5, alpha=1.0):
    """Draw X(red) Y(green) Z(blue) arrows."""
    R = quat_to_rot(*quat)
    for i, (name, c) in enumerate(COLORS.items()):
        end = pos + R[:,i] * length
        ax.quiver(pos[0], pos[1], pos[2],
                  R[0,i]*length, R[1,i]*length, R[2,i]*length,
                  color=c, linewidth=lw, arrow_length_ratio=0.25, alpha=alpha)

def draw_text(ax, pos, text, color="white"):
    ax.text(pos[0], pos[1], pos[2], text, fontsize=9, color=color,
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
            # flange_ccs, calib_ccs, board_gt (each 7)
            fp,fq = np.array(v[0:3]),  np.array(v[3:7])
            cp,cq = np.array(v[7:10]), np.array(v[10:14])
            bp,bq = np.array(v[14:17]),np.array(v[17:21])
            frames.append((int(ln[:2]), fp,fq, cp,cq, bp,bq))
    return arm_ccs, frames

# ---------------------------------------------------------------------------
def find_piper_yaml():
    """Locate piper.yaml relative to this script (4 levels up from tests/utils/piper)."""
    cand = Path(__file__).resolve().parent  # tests/utils/piper/
    for _ in range(4):
        cand = cand.parent
        test = cand / "cfg" / "piper.yaml"
        if test.exists(): return test
    return None

def main():
    arm = sys.argv[1] if len(sys.argv) > 1 else "upper"

    yaml_path = find_piper_yaml()
    if yaml_path is None:
        print("Cannot find piper.yaml"); sys.exit(1)

    import yaml
    with open(yaml_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    data_dir = cfg["test_record_arm_data"]["calib_save_dir"]
    path = Path(data_dir) / f"chain_viz_{arm}.txt"
    if not path.exists():
        print(f"Not found: {path}\nRun save_piper_chain.exe first.")
        sys.exit(1)

    arm_ccs, frames = load(path)
    if not frames: print("No data"); return
    n = len(frames)
    print(f"Loaded {n} frames. ← → keys / slider. Close window to exit.")

    ap, aq = arm_ccs
    AL = 0.05   # axis length

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(bottom=0.12)

    ax_slider = plt.axes([0.15,0.02,0.7,0.03])
    slider = Slider(ax_slider, "Frame", 0, n-1, valinit=0, valfmt="%d")

    ax_status = plt.axes([0.15,0.06,0.7,0.03]); ax_status.axis("off")
    stxt = ax_status.text(0.5,0.5,"",transform=ax_status.transAxes,ha="center",va="center",
                          fontsize=10,color="white")

    def update(i):
        ax.clear()
        idx, fp,fq, cp,cq, bp,bq = frames[i]

        # ── Camera (origin) ──
        draw_axes(ax, np.zeros(3), (0,0,0,1), AL, lw=2.5, alpha=0.9)
        draw_text(ax, np.zeros(3), "  Cam")

        # ── Arm base ──
        draw_axes(ax, ap, aq, AL, lw=2.5, alpha=0.8)
        draw_text(ax, ap, "  Arm")
        ax.plot([0,ap[0]],[0,ap[1]],[0,ap[2]],"gray",lw=0.6,ls="--")

        # ── Flange ──
        draw_axes(ax, fp, fq, AL*0.8, lw=2, alpha=0.8)
        draw_text(ax, fp, "  Flange")
        ax.plot([ap[0],fp[0]],[ap[1],fp[1]],[ap[2],fp[2]],"cyan",lw=0.6,ls="--")

        # ── Calib board (predicted) ──
        draw_axes(ax, cp, cq, AL*0.8, lw=2, alpha=0.85)
        draw_text(ax, cp, "  Calib(est)")
        ax.plot([fp[0],cp[0]],[fp[1],cp[1]],[fp[2],cp[2]],"orange",lw=0.6,ls="--")

        # ── Board ground truth ──
        if not math.isnan(bp[0]):
            draw_axes(ax, bp, bq, AL*0.8, lw=2.5, alpha=0.9)
            draw_text(ax, bp, "  Board(GT)")
            # error
            ax.plot([cp[0],bp[0]],[cp[1],bp[1]],[cp[2],bp[2]],"red",lw=2)
            err_mm = np.linalg.norm(cp - bp)*1000
            mid = (cp+bp)/2; ax.text(mid[0],mid[1],mid[2],f"Δ{err_mm:.1f}mm",fontsize=8,color="red")
            stxt.set_text(f"Frame {idx:02d} ({i+1}/{n})  |  "
                          f"Cam → Arm → Flange → Calib(est) → Board(GT)  "
                          f"  Error: {err_mm:.1f} mm")
        else:
            stxt.set_text(f"Frame {idx:02d} ({i+1}/{n})  |  "
                          f"Cam → Arm → Flange → Calib(est)  [no GT]")

        # bounds
        pts = [np.zeros(3), ap, fp, cp]
        if not math.isnan(bp[0]): pts.append(bp)
        pts = np.array(pts); lo,hi = pts.min(axis=0), pts.max(axis=0)
        pad = max((hi-lo).max()*0.25, 0.06)
        mid = (lo+hi)/2; r = (hi-lo).max()/2 + pad
        ax.set_xlim(mid[0]-r,mid[0]+r); ax.set_ylim(mid[1]-r,mid[1]+r); ax.set_zlim(mid[2]-r,mid[2]+r)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title(f"Frame {idx:02d}", fontsize=13)
        fig.canvas.draw_idle()

    def on_slider(v): update(int(round(v)))
    def on_key(e):
        c = int(round(slider.val))
        if e.key in ("right","up") and c<n-1: slider.set_val(c+1)
        elif e.key in ("left","down") and c>0: slider.set_val(c-1)

    slider.on_changed(on_slider)
    fig.canvas.mpl_connect("key_press_event", on_key)
    update(0); plt.show()

if __name__=="__main__": main()
