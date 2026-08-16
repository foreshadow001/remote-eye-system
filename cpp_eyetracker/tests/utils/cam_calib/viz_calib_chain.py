#!/usr/bin/env python3
"""
viz_calib_chain.py — 3D visualization of multi-camera calibration extrinsics.

Reads CameraData XML files from calib_save_dir/{viz_day_id}/output/
and plots all camera poses in the center_cam coordinate system.
Each camera is shown with its XYZ axes and position label.

Config:
  - cam_calib.yaml:   calib_save_dir, viz_day_id, center_cam
                     (day_id is the fallback if viz_day_id is empty)
  - optional argv[1]:  directory ID override (day_id or participant_id)
  - MPLBACKEND:        forced to TkAgg (default Qt backend unavailable on this machine)
"""

import os
os.environ.setdefault("MPLBACKEND", "TkAgg")  # 本机默认 Qt 后端不可用, 强制 TkAgg (与 DEV_GUIDE 一致)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import xml.etree.ElementTree as ET
import yaml
import sys
import glob
import re
from pathlib import Path


# ==================== Path helpers ====================

def find_config_dir():
    """Find cfg/ relative to this script's location."""
    return Path(__file__).resolve().parent.parent.parent.parent / "cfg"


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# ==================== XML parsing ====================

def parse_camera_xml(xml_path):
    """Parse a CameraData XML file. Returns dict with SN, internal_params_raw, pose (x,y,z,a,b,g)."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    data = {}

    # Internal params
    ip_node = root.find('.//InternalParameters')
    if ip_node is not None:
        raw = ip_node.find('RawData')
        if raw is not None and raw.text:
            data['internal_raw'] = raw.text.strip()

    # External params
    ep_node = root.find('.//ExternalParameters')
    if ep_node is not None:
        # Translation
        trans = ep_node.find('Translation')
        if trans is not None:
            data['tx'] = float(trans.find('X').text)
            data['ty'] = float(trans.find('Y').text)
            data['tz'] = float(trans.find('Z').text)

        # Rotation
        rot = ep_node.find('Rotation')
        if rot is not None:
            data['alpha'] = float(rot.find('Alpha').text)
            data['beta']  = float(rot.find('Beta').text)
            data['gamma'] = float(rot.find('Gamma').text)

    return data


# ==================== Rotation conversion ====================

def euler_gba_to_matrix(alpha_deg, beta_deg, gamma_deg):
    """
    HALCON 'gba' rotation: R = Rz(alpha) * Ry(beta) * Rz(gamma), angles in degrees.
    Returns 3x3 rotation matrix.
    """
    a = np.radians(alpha_deg)
    b = np.radians(beta_deg)
    g = np.radians(gamma_deg)

    def Rz(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def Ry(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    return Rz(a) @ Ry(b) @ Rz(g)


# ==================== Visualization ====================

def draw_camera_axes(ax, pos, R, size=0.03, pick_radius=5):
    """
    Draw XYZ axes at position `pos` (in meters), oriented by rotation matrix `R`.
    size: axis length in meters.
    Returns list of Line2D artists with camera metadata attached.
    """
    origin = np.array(pos)
    x_axis = origin + R @ np.array([size, 0, 0])
    y_axis = origin + R @ np.array([0, size, 0])
    z_axis = origin + R @ np.array([0, 0, size])

    lines = []
    lx, = ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]],
                   c='r', lw=1.5, picker=pick_radius)
    ly, = ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]],
                   c='g', lw=1.5, picker=pick_radius)
    lz, = ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]],
                   c='b', lw=1.5, picker=pick_radius)
    lines.extend([lx, ly, lz])
    return lines


def draw_camera_fov(ax, pos, R, cam_size=0.01, fov_len=0.02):
    """Draw a small camera pyramid showing the viewing direction (-Z is forward in HALCON)."""
    origin = np.array(pos)
    # Camera frustum corners in camera-local coords
    s = cam_size
    f = fov_len
    corners_local = np.array([
        [ s,  s, 0], [ s, -s, 0], [-s, -s, 0], [-s,  s, 0],  # sensor corners
        [ 0,  0,  f],  # focal point (HALCON: +Z = forward out of lens)
    ])
    corners_world = np.array([origin + R @ c for c in corners_local])

    # Draw lines from sensor corners to focal point
    for i in range(4):
        ax.plot([corners_world[i][0], corners_world[4][0]],
                [corners_world[i][1], corners_world[4][1]],
                [corners_world[i][2], corners_world[4][2]],
                c='gray', lw=0.5, alpha=0.5)
    # Draw sensor rectangle
    for i in range(4):
        j = (i + 1) % 4
        ax.plot([corners_world[i][0], corners_world[j][0]],
                [corners_world[i][1], corners_world[j][1]],
                [corners_world[i][2], corners_world[j][2]],
                c='gray', lw=0.5, alpha=0.5)


def main():
    # Load config
    cfg_dir = find_config_dir()
    calib_cfg = load_yaml(cfg_dir / "cam_calib.yaml")

    calib = calib_cfg.get("calib", {})
    base_dir  = calib.get("calib_save_dir", "D:/calib_images")
    center_sn = calib.get("center_cam", "")
    # day_id 由 cam_calib.yaml 自带 (D001 系列), viz_day_id 留空时回退到它
    # 命令行参数可覆盖 (calib_with_HALCON 按 v 传入 day_id / participant_id):
    #   python viz_calib_chain.py D001
    base_did  = calib.get("day_id", "D001")
    viz_did   = sys.argv[1] if len(sys.argv) > 1 else (calib.get("viz_day_id", "") or base_did)

    # Locate XML files
    xml_dir = os.path.join(base_dir, viz_did, "output")
    if not os.path.isdir(xml_dir):
        print(f"[ERROR] Output directory not found: {xml_dir}")
        sys.exit(1)

    xml_files = sorted(glob.glob(os.path.join(xml_dir, "*_Data.xml")))
    if not xml_files:
        print(f"[ERROR] No *_Data.xml files found in {xml_dir}")
        sys.exit(1)

    print(f"Found {len(xml_files)} camera XML files in {xml_dir}")
    print(f"Center camera SN: {center_sn}")

    # Parse all cameras
    cameras = []
    for xf in xml_files:
        sn = Path(xf).stem.replace("_Data", "")
        data = parse_camera_xml(xf)
        data['sn'] = sn
        cameras.append(data)

    # Separate center camera
    center_cam = None
    others = []
    for c in cameras:
        if c['sn'] == center_sn:
            center_cam = c
        else:
            others.append(c)

    if not center_cam:
        print(f"[WARN] Center camera {center_sn} not found, using camera 0 as reference")
        center_cam = cameras[0]
        others = cameras[1:]

    print(f"Center: {center_cam['sn']} at ({center_cam['tx']:.3f}, {center_cam['ty']:.3f}, {center_cam['tz']:.3f}) m")
    print(f"Other cameras: {len(others)}")

    # ==================== 3D Plot ====================
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Build mapping: Line2D artist → camera data (for pick events)
    artist_to_cam = {}  # id(line) → {sn, tx, ty, tz}

    # Compute position range first (used to scale axes)
    all_positions = np.array([[0, 0, 0]] + [[c['tx'], c['ty'], c['tz']] for c in others])
    pos_range = np.max(np.ptp(all_positions, axis=0)) if len(others) > 0 else 1.0

    # Draw center camera axes = world coordinate system (thicker, longer)
    origin = np.array([0.0, 0.0, 0.0])
    wsize = max(pos_range * 0.08, 0.08)
    ax.plot([0, wsize], [0, 0], [0, 0], c='r', lw=2.5)  # X axis
    ax.plot([0, 0], [0, wsize], [0, 0], c='g', lw=2.5)  # Y axis
    ax.plot([0, 0], [0, 0], [0, wsize], c='b', lw=2.5)  # Z axis
    ax.text(wsize * 1.1, 0, 0, "X", color='r', fontsize=11, fontweight='bold')
    ax.text(0, wsize * 1.1, 0, "Y", color='g', fontsize=11, fontweight='bold')
    ax.text(0, 0, wsize * 1.1, "Z", color='b', fontsize=11, fontweight='bold')
    # Center camera dot
    ax.scatter(*origin, c='k', marker='o', s=60, zorder=5)
    # Register for picking
    center_line_ref = ax.plot([0, wsize], [0, 0], [0, 0], c='r', lw=2.5, picker=8)[0]
    artist_to_cam[id(center_line_ref)] = {'sn': center_cam['sn'], 'tx': 0.0, 'ty': 0.0, 'tz': 0.0, 'is_center': True}
    draw_camera_fov(ax, (0, 0, 0), np.eye(3), cam_size=wsize * 0.3, fov_len=wsize * 0.5)

    # Initial view: from -Z direction, X-left, Y-down
    ax.view_init(elev=90, azim=90)

    # Axis size for other cameras
    axis_size = max(pos_range * 0.05, 0.02)

    # Draw other cameras (no text labels)
    for c in others:
        pos = np.array([c['tx'], c['ty'], c['tz']])
        R = euler_gba_to_matrix(c['alpha'], c['beta'], c['gamma'])
        cam_lines = draw_camera_axes(ax, pos, R, size=axis_size, pick_radius=8)
        for line in cam_lines:
            artist_to_cam[id(line)] = {'sn': c['sn'], 'tx': c['tx'], 'ty': c['ty'], 'tz': c['tz'], 'is_center': False}
        draw_camera_fov(ax, pos, R, cam_size=axis_size * 0.5, fov_len=axis_size * 0.8)

    # Floating annotation (initially off-screen)
    annot = ax.text2D(0, 0, "", fontsize=9, ha='center', va='bottom',
                      bbox=dict(boxstyle='round,pad=0.4', facecolor='lightyellow', edgecolor='black', alpha=0.95),
                      transform=ax.transAxes, zorder=100)
    annot.set_visible(False)

    def on_pick(event):
        """Show XYZ label for the clicked camera."""
        cam = None
        # event.artist might be a single Line2D; check our mapping
        lid = id(event.artist)
        if lid in artist_to_cam:
            cam = artist_to_cam[lid]
        else:
            # Check if any of the line collection elements match
            for art in (event.artist if hasattr(event.artist, '__iter__') else [event.artist]):
                if id(art) in artist_to_cam:
                    cam = artist_to_cam[id(art)]
                    break

        if cam is None:
            annot.set_visible(False)
            fig.canvas.draw_idle()
            return

        if cam.get('is_center'):
            text = f"{cam['sn']} (reference)\n(0, 0, 0) m"
        else:
            text = f"{cam['sn']}\nx={cam['tx']:.4f}  y={cam['ty']:.4f}  z={cam['tz']:.4f} m"

        annot.set_text(text)
        annot.set_visible(True)
        # Position annotation in upper-left corner of the axes
        annot.set_position((0.02, 0.98))
        annot.set_va('top')
        annot.set_ha('left')
        fig.canvas.draw_idle()

    def on_click_outside(event):
        """Hide annotation when clicking on empty space."""
        if event.inaxes != ax:
            return
        # Check if the click hit any artist
        contained = False
        for artist in ax.get_children():
            if hasattr(artist, 'contains') and artist.contains(event)[0]:
                contained = True
                break
        if not contained:
            annot.set_visible(False)
            fig.canvas.draw_idle()

    # ==================== Plot settings ====================
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Camera Extrinsics — {len(cameras)} cameras in "{center_cam["sn"]}" frame\n{viz_did}\nClick a camera to see XYZ', fontsize=12)

    mid = np.mean(all_positions, axis=0)
    rng = max(np.max(np.ptp(all_positions, axis=0)) * 0.6, 0.5)
    ax.set_xlim(mid[0] - rng, mid[0] + rng)
    ax.set_ylim(mid[1] - rng, mid[1] + rng)
    ax.set_zlim(mid[2] - rng, mid[2] + rng)

    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass

    # Event handlers
    fig.canvas.mpl_connect('pick_event', on_pick)
    fig.canvas.mpl_connect('button_press_event', on_click_outside)

    # Keyboard shortcuts
    print("\nKeyboard / mouse:")
    print("  Click camera axes → show XYZ")
    print("  Click empty space  → hide label")
    print("  t / f / s          → top / front / side view")
    print("  r                  → reset view")
    print("  q                  → quit")

    def on_key(event):
        if event.key == 't':
            ax.view_init(elev=90, azim=-90)
        elif event.key == 'f':
            ax.view_init(elev=0, azim=-90)
        elif event.key == 's':
            ax.view_init(elev=0, azim=0)
        elif event.key == 'r':
            ax.view_init(elev=90, azim=90)
        elif event.key == 'q':
            plt.close()
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key)

    print(f"\nCamera count: {len(cameras)}")
    print(f"Positions range: X [{all_positions[:,0].min():.3f}, {all_positions[:,0].max():.3f}] m")
    print(f"                 Y [{all_positions[:,1].min():.3f}, {all_positions[:,1].max():.3f}] m")
    print(f"                 Z [{all_positions[:,2].min():.3f}, {all_positions[:,2].max():.3f}] m")
    plt.show()


if __name__ == "__main__":
    main()
