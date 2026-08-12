#!/usr/bin/env python3
"""
viz_calib_chain.py — 3D visualization of multi-camera calibration extrinsics.

Reads CameraData XML files from calib_save_dir/{viz_participant_id}/output/
and plots all camera poses in the center_cam coordinate system.
Each camera is shown with its XYZ axes and position label.

Config:
  - cam_calib.yaml:   calib_save_dir, viz_participant_id, center_cam
  - capture.yaml:     participant_id (fallback if viz_participant_id empty)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import xml.etree.ElementTree as ET
import yaml
import os
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

def draw_camera_axes(ax, pos, R, size=0.03, label=None):
    """
    Draw XYZ axes at position `pos` (in meters), oriented by rotation matrix `R`.
    size: axis length in meters.
    """
    origin = np.array(pos)
    x_axis = origin + R @ np.array([size, 0, 0])
    y_axis = origin + R @ np.array([0, size, 0])
    z_axis = origin + R @ np.array([0, 0, size])

    ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]],
            c='r', lw=1.5)
    ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]],
            c='g', lw=1.5)
    ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]],
            c='b', lw=1.5)

    # Position label
    if label:
        ax.text(origin[0], origin[1], origin[2] + size * 0.3, label,
                fontsize=7, ha='center', va='bottom',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))


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
    cap_cfg   = load_yaml(cfg_dir / "capture.yaml")

    calib = calib_cfg.get("calib", {})
    base_dir  = calib.get("calib_save_dir", "D:/calib_images")
    center_sn = calib.get("center_cam", "")
    viz_pid   = calib.get("viz_participant_id", "")
    if not viz_pid:
        viz_pid = cap_cfg.get("capture", {}).get("participant_id", "P001")

    # Locate XML files
    xml_dir = os.path.join(base_dir, viz_pid, "output")
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

    # Draw center camera (reference, at origin or its own pose)
    # Note: poses are already in center_cam coordinate system.
    # The center camera should be at (0,0,0) with identity rotation.
    # But due to numerical errors it might be near-zero. Use its actual pose.
    draw_camera_axes(ax, (0, 0, 0), np.eye(3), size=0.04, label=f"{center_cam['sn']}\n(reference)")

    # Draw center camera FOV pyramid
    # For center camera, it's at origin, rotation is identity (Z forward in HALCON convention)
    draw_camera_fov(ax, (0, 0, 0), np.eye(3), cam_size=0.015, fov_len=0.03)

    # Draw all other cameras
    # Scale from meters to something visible
    # Actually, positions are in meters. Let's check the range.
    all_positions = []
    for c in others:
        all_positions.append([c['tx'], c['ty'], c['tz']])

    all_positions = np.array(all_positions)
    if len(all_positions) > 0:
        pos_range = np.max(np.ptp(all_positions, axis=0))
    else:
        pos_range = 1.0

    axis_size = max(pos_range * 0.05, 0.02)  # 5% of range, min 2cm

    for c in others:
        pos = np.array([c['tx'], c['ty'], c['tz']])
        R = euler_gba_to_matrix(c['alpha'], c['beta'], c['gamma'])

        # Simple position label
        label = f"{c['sn']}\n({c['tx']:.3f}, {c['ty']:.3f}, {c['tz']:.3f})"
        draw_camera_axes(ax, pos, R, size=axis_size, label=label)
        draw_camera_fov(ax, pos, R, cam_size=axis_size * 0.5, fov_len=axis_size * 0.8)

    # ==================== Plot settings ====================
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Camera Extrinsics — {len(cameras)} cameras in "{center_cam["sn"]}" frame\n{viz_pid}', fontsize=12)

    # Auto-range with some margin
    all_pts = [[0,0,0]]
    for c in cameras:
        all_pts.append([c['tx'], c['ty'], c['tz']])
    all_pts = np.array(all_pts)
    mid = np.mean(all_pts, axis=0)
    rng = max(np.max(np.ptp(all_pts, axis=0)) * 0.6, 0.5)  # at least 0.5m
    ax.set_xlim(mid[0] - rng, mid[0] + rng)
    ax.set_ylim(mid[1] - rng, mid[1] + rng)
    ax.set_zlim(mid[2] - rng, mid[2] + rng)

    # Equal aspect ratio (best effort in 3D)
    try:
        ax.set_box_aspect([1, 1, 1])
    except Exception:
        pass

    # Toggle between views with keyboard
    print("\nKeyboard shortcuts:")
    print("  t / f / s : top / front / side view")
    print("  r         : reset to default 3D view")
    print("  q         : quit")

    def on_key(event):
        if event.key == 't':
            ax.view_init(elev=90, azim=-90)
        elif event.key == 'f':
            ax.view_init(elev=0, azim=-90)
        elif event.key == 's':
            ax.view_init(elev=0, azim=0)
        elif event.key == 'r':
            ax.view_init(elev=20, azim=-60)
        elif event.key == 'q':
            plt.close()
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key)

    print(f"\nCamera count: {len(cameras)}")
    print(f"Positions range: X [{all_pts[:,0].min():.3f}, {all_pts[:,0].max():.3f}] m")
    print(f"                 Y [{all_pts[:,1].min():.3f}, {all_pts[:,1].max():.3f}] m")
    print(f"                 Z [{all_pts[:,2].min():.3f}, {all_pts[:,2].max():.3f}] m")
    plt.show()


if __name__ == "__main__":
    main()
