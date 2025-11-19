#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

TXT = 'D:/users/Projects/new_dataset/calib_inference_result.txt'
actual_camera_position = np.array([29.0, -31, 17])
SCREEN_W = 59.5
SCREEN_H = 33.6
ARROW_LEN = 4

def screen_rect():
    return np.array([
        [0,        -SCREEN_H, 0],
        [SCREEN_W, -SCREEN_H, 0],
        [SCREEN_W, 0,         0],
        [0,        0,         0],
        [0,        -SCREEN_H, 0]
    ])

def load():
    data = []
    with open(TXT) as f:
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
    return data

def pick_frame(data):
    n = len(data)
    while True:
        try:
            idx = int(input(f'please input frame index (0~{n-1}):'))
            if 0 <= idx < n: return idx
        except ValueError: pass
        print('invalid input, please try again')

def plot_3d_single(frame):
    fig = plt.figure('3D visualization', figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # screen
    scr = screen_rect()
    ax.plot(scr[:, 0], scr[:, 1], scr[:, 2], color='black', lw=1, label='Screen')

    # camera
    ax.scatter(*actual_camera_position, color='red', s=20, marker='^', label='Camera')
    # draw the camera's optical axis
    ax.plot(
        [actual_camera_position[0], actual_camera_position[0]],
        [actual_camera_position[1], actual_camera_position[1] + 40],
        [actual_camera_position[2], actual_camera_position[2] + 40 * 1.732],
        color='yellow', lw=2, label='Camera Optical axis'
    )

    # data
    c_wcs = frame['cornea_cam']
    o_dir = frame['opt_axis']
    v_dir = frame['vis_axis']
    gt = frame['gt']
    pred = frame['pred']

    def intersect_z0(c, d):
        if abs(d[2]) < 1e-12:
            return None, False
        t = -c[2] / d[2]
        inter = c + t * d
        in_screen = (0 <= inter[0] <= SCREEN_W) and (-SCREEN_H <= inter[1] <= 0)
        in_screen = True
        return inter, in_screen

    opt_hit, opt_valid = intersect_z0(c_wcs, o_dir)
    vis_hit, vis_valid = intersect_z0(c_wcs, v_dir)

    # optical axis
    if opt_hit is not None:
        ax.plot([c_wcs[0], opt_hit[0]], [c_wcs[1], opt_hit[1]], [c_wcs[2], opt_hit[2]],
                color='red', lw=2, label='Optical axis')
        if opt_valid:
            ax.scatter(*opt_hit, color='pink', s=20, marker='o', label='Optical hit')

    # visual axis
    if vis_hit is not None:
        ax.plot([c_wcs[0], vis_hit[0]], [c_wcs[1], vis_hit[1]], [c_wcs[2], vis_hit[2]],
                color='cyan', lw=2, label='Visual axis')
        if vis_valid:
            ax.scatter(*vis_hit, color='lime', s=20, marker='x', label='Visual hit')
            
    # other elements
    ax.scatter(*c_wcs, color='orange', s=20, label='Cornea center')
    ax.scatter(*gt, color='red', s=20, marker='o', label='GT gaze')
    ax.scatter(*pred, color='blue', s=20, marker='x', lw=3, label='Pred gaze')

    # coordinate range
    ax.set_xlim(- SCREEN_W, 2 * SCREEN_W)
    ax.set_ylim(- 2 * SCREEN_H, SCREEN_H)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')  # type: ignore
    ax.set_box_aspect([SCREEN_W, SCREEN_H, 40])  # type: ignore
    ax.legend()
    ax.set_title(f'3D Eye Model — Frame {idx}')
    plt.tight_layout()
    plt.show()

def plot_2d_distribution(data):
    plt.figure('2D calibration result', figsize=(6, 4))
    ax = plt.gca()
    bucket = defaultdict(list)
    for fr in data:
        bucket[tuple(fr['gt'][:2])].append(fr['pred'][:2])

    # calculate the mean and standard deviation distance loss
    xy = [fr['gt'][:2] for fr in data]
    preds = [fr['pred'][:2] for fr in data]
    mean_dist = np.mean(np.linalg.norm(preds - np.array(xy), axis=1))
    std_dist = np.std(np.linalg.norm(preds - np.array(xy), axis=1))

    # show the statistics in the bottom-left corner
    ax.text(0.05, 0.05, f'Mean distance loss: {mean_dist:.2f}\n'
                         f'Std distance loss: {std_dist:.2f}',
            transform=ax.transAxes, fontsize=12)

    cmap = plt.cm.tab10 # type: ignore
    for i, (gt_xy, preds) in enumerate(bucket.items()):
        color = cmap(i % cmap.N)
        preds = np.array(preds)
        ax.scatter(preds[:, 0], preds[:, 1], c=[color], s=25, alpha=0.7)
        ax.scatter(*gt_xy, c='black', s=50, marker='o', zorder=3)

    ax.set_xlim(0, SCREEN_W)
    ax.set_ylim(0, -SCREEN_H)
    ax.invert_yaxis()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Pred vs GT — same-color = same-GT')
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    data = load()
    if not data:
        print('data is empty')
        exit()
    idx = pick_frame(data)
    plot_3d_single(data[idx])
    plot_2d_distribution(data)