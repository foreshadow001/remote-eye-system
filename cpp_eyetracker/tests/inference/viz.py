#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# --- Configuration ---
TXT_LEFT = 'D:/ylx/calib_inference_result_left.txt'
TXT_RIGHT = 'D:/ylx/calib_inference_result_right.txt'

# Camera position from C++ cfg["cam_pos"]
ACTUAL_CAMERA_POSITION = np.array([29.0, -31, 17]) 

SCREEN_W = 59.5
SCREEN_H = 33.6

def screen_rect():
    """Defines the 4 corners of the screen + back to start for plotting."""
    return np.array([
        [0,        -SCREEN_H, 0],
        [SCREEN_W, -SCREEN_H, 0],
        [SCREEN_W, 0,         0],
        [0,        0,         0],
        [0,        -SCREEN_H, 0]
    ])

def load_single_file(filepath):
    """Parses a single inference result file."""
    data = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                if line.startswith('#'): continue
                parts = line.strip().split()
                if len(parts) != 15: continue
                nums = list(map(float, parts))
                data.append({
                    'cornea_cam': np.array(nums[0:3]),
                    'opt_axis':   np.array(nums[3:6]),
                    'vis_axis':   np.array(nums[6:9]),
                    'gt':         np.array(nums[9:12]),
                    'pred':       np.array(nums[12:15])
                })
    except FileNotFoundError:
        print(f"[Error] File not found: {filepath}")
    return data

def load_paired_data():
    """Loads left and right data and pairs them by index."""
    left_list = load_single_file(TXT_LEFT)
    right_list = load_single_file(TXT_RIGHT)

    if len(left_list) != len(right_list):
        print(f"[Warn] Line count mismatch: Left={len(left_list)}, Right={len(right_list)}. Truncating to shorter.")
    
    n = min(len(left_list), len(right_list))
    paired_data = []
    
    for i in range(n):
        # We assume line N in left file corresponds to line N in right file
        # Check if GT matches to be sure
        dist_gt = np.linalg.norm(left_list[i]['gt'] - right_list[i]['gt'])
        if dist_gt > 1e-3:
            print(f"[Warn] Frame {i} GT mismatch between files!")
            
        paired_data.append({
            'left': left_list[i],
            'right': right_list[i],
            'gt': left_list[i]['gt'] # Shared GT
        })
        
    return paired_data

def pick_frame(n_frames):
    while True:
        try:
            val = input(f'Please input frame index (0~{n_frames-1}): ')
            if not val: return 0 # Default to 0 if enter
            idx = int(val)
            if 0 <= idx < n_frames: return idx
        except ValueError: pass
        print('Invalid input, please try again.')

def intersect_z0(c, d):
    """Calculates intersection of ray (origin c, direction d) with plane Z=0."""
    if abs(d[2]) < 1e-6:
        return None, False
    t = -c[2] / d[2]
    inter = c + t * d
    # Check if inside screen boundaries
    in_screen = (0 <= inter[0] <= SCREEN_W) and (-SCREEN_H <= inter[1] <= 0)
    return inter, in_screen

def plot_3d_paired(frame_idx, frame_data):
    fig = plt.figure(f'3D Visualization Frame {frame_idx}', figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # --- 1. Draw Environment ---
    # Screen
    scr = screen_rect()
    ax.plot(scr[:, 0], scr[:, 1], scr[:, 2], color='black', lw=2, label='Screen')
    
    # Camera
    ax.scatter(*ACTUAL_CAMERA_POSITION, color='black', s=50, marker='^', label='Camera')
    # Camera Optical Axis (Visualization only)
    ax.plot(
        [ACTUAL_CAMERA_POSITION[0], ACTUAL_CAMERA_POSITION[0]],
        [ACTUAL_CAMERA_POSITION[1], ACTUAL_CAMERA_POSITION[1] + 20],
        [ACTUAL_CAMERA_POSITION[2], ACTUAL_CAMERA_POSITION[2] + 35],
        color='gray', linestyle='--', lw=1
    )

    # --- 2. Draw Eyes Helper ---
    def draw_eye(eye_info, cornea_color, label_prefix):
        c_wcs = eye_info['cornea_cam']
        o_dir = eye_info['opt_axis']
        v_dir = eye_info['vis_axis']
        pred  = eye_info['pred']

        # Draw Cornea
        ax.scatter(*c_wcs, color=cornea_color, s=40, label=f'{label_prefix} Cornea')

        # Calculate Intersections
        opt_hit, opt_valid = intersect_z0(c_wcs, o_dir)
        vis_hit, vis_valid = intersect_z0(c_wcs, v_dir)

        # Draw Optical Axis (Red)
        if opt_hit is not None:
            ax.plot([c_wcs[0], opt_hit[0]], [c_wcs[1], opt_hit[1]], [c_wcs[2], opt_hit[2]],
                    color='red', lw=1, alpha=0.6)

        # Draw Visual Axis (Cyan/Blue)
        if vis_hit is not None:
            ax.plot([c_wcs[0], vis_hit[0]], [c_wcs[1], vis_hit[1]], [c_wcs[2], vis_hit[2]],
                    color='cyan', lw=2, alpha=0.8)
            
        # Draw Prediction on Screen
        ax.scatter(*pred, color=cornea_color, s=50, marker='x', lw=2, label=f'{label_prefix} Pred')

    #  
    # Draw Left Eye (Blue theme)
    draw_eye(frame_data['left'], 'blue', 'Left')
    
    # Draw Right Eye (Green theme)
    draw_eye(frame_data['right'], 'green', 'Right')

    # --- 3. Draw Ground Truth ---
    gt = frame_data['gt']
    ax.scatter(*gt, color='red', s=80, marker='o', edgecolors='black', label='GT Gaze')

    # --- Settings ---
    ax.set_xlim(-SCREEN_W * 0.5, SCREEN_W * 1.5)
    ax.set_ylim(-SCREEN_H * 2.0, SCREEN_H * 0.5)
    ax.set_zlim(0, 50)
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    
    # Invert Y/Z visual aspect to match reality (Screen is usually Y-down or Camera is Z-back)
    # But usually 3D plots are easier to read with standard axes.
    # We maintain aspect ratio roughly
    ax.set_box_aspect([2, 1.5, 1]) 
    
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))
    plt.title(f'Binocular Gaze Visualization - Frame {frame_idx}')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    print("Loading data...")
    data = load_paired_data()
    
    if not data:
        print("No valid data found in specified files.")
        exit()
        
    print(f"Loaded {len(data)} frames.")
    
    # 2. Plot specific 3D frame
    idx = pick_frame(len(data))
    plot_3d_paired(idx, data[idx])