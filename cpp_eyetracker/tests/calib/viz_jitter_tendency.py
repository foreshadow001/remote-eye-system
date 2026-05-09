import os
import re
import csv
import numpy as np
import matplotlib.pyplot as plt

# ==================== 路径配置 ====================
BASE_DIR = "D:/hitsz/projects/new_dataset/eyetracker_test/ylx"

# ==================== 列索引常量 ====================
IDX_CORNEA = 0
IDX_OPT    = 3
IDX_VIS    = 6
IDX_GT     = 9
IDX_PRED   = 12

# ==================== 数学运算辅助函数 ====================
def normalize(v):
    n = np.linalg.norm(v)
    return v if n == 0 else v / n

def closest_points_on_two_lines(O1, D1, O2, D2):
    W0 = O1 - O2
    a = np.dot(D1, D1)
    b = np.dot(D1, D2)
    c = np.dot(D2, D2)
    d = np.dot(D1, W0)
    e = np.dot(D2, W0)
    denom = a * c - b * b
    
    if denom < 1e-6:
        return O1, O2
        
    s = (b * e - c * d) / denom
    t = (a * e - b * d) / denom
    
    P1 = O1 + s * D1
    P2 = O2 + t * D2
    return P1, P2

def calc_binocular_errors(row_l, row_r):
    cornea_l = row_l[IDX_CORNEA : IDX_CORNEA+3]
    vis_l    = normalize(row_l[IDX_VIS : IDX_VIS+3])
    gt       = row_l[IDX_GT : IDX_GT+3]
    
    cornea_r = row_r[IDX_CORNEA : IDX_CORNEA+3]
    vis_r    = normalize(row_r[IDX_VIS : IDX_VIS+3])
    
    P1, P2 = closest_points_on_two_lines(cornea_l, vis_l, cornea_r, vis_r)
    P_pred = (P1 + P2) / 2.0
    
    dist_err = np.linalg.norm(gt - P_pred)
    
    gt_vec_l = normalize(gt - cornea_l)
    ang_l = np.degrees(np.arccos(np.clip(np.dot(vis_l, gt_vec_l), -1.0, 1.0)))
    
    gt_vec_r = normalize(gt - cornea_r)
    ang_r = np.degrees(np.arccos(np.clip(np.dot(vis_r, gt_vec_r), -1.0, 1.0)))
    
    return dist_err, ang_l, ang_r

# ==================== 核心指标计算 ====================
def calculate_metrics(left_data, right_data):
    num_frames = min(len(left_data), len(right_data))
    if num_frames < 2: return None

    all_dist_errs, all_ang_l, all_ang_r = [], [], []
    all_vis_l, all_vis_r, all_gts = [], [], []

    for i in range(num_frames):
        dist_err, ang_l, ang_r = calc_binocular_errors(left_data[i], right_data[i])
        all_dist_errs.append(dist_err)
        all_ang_l.append(ang_l)
        all_ang_r.append(ang_r)
        all_vis_l.append(normalize(left_data[i][IDX_VIS : IDX_VIS+3]))
        all_vis_r.append(normalize(right_data[i][IDX_VIS : IDX_VIS+3]))
        all_gts.append(left_data[i][IDX_GT : IDX_GT+3])

    all_gts = np.array(all_gts)
    unique_gts, inverse_indices = np.unique(np.round(all_gts, decimals=3), axis=0, return_inverse=True)
    num_points = len(unique_gts)

    point_accuracies_dist = []
    point_precisions_sd_l, point_precisions_sd_r = [], []
    point_jitters_l, point_jitters_r = [], []

    for i in range(num_points):
        indices = np.where(inverse_indices == i)[0]
        if len(indices) < 2: continue
        
        point_accuracies_dist.append(np.mean([all_dist_errs[idx] for idx in indices]))
        point_precisions_sd_l.append(np.std([all_ang_l[idx] for idx in indices]))
        point_precisions_sd_r.append(np.std([all_ang_r[idx] for idx in indices]))
        
        def calc_point_jitter(vec_list):
            diffs = []
            for k in range(1, len(vec_list)):
                if indices[k] - indices[k-1] == 1:
                    v1, v2 = vec_list[k], vec_list[k-1]
                    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
                    diffs.append(np.degrees(np.arccos(dot))**2)
            return np.sqrt(np.mean(diffs)) if diffs else 0.0

        point_jitters_l.append(calc_point_jitter([all_vis_l[idx] for idx in indices]))
        point_jitters_r.append(calc_point_jitter([all_vis_r[idx] for idx in indices]))

    return {
        'acc_dist': np.mean(point_accuracies_dist),
        'acc_ang_l': np.mean(all_ang_l),
        'acc_ang_r': np.mean(all_ang_r),
        'prec_sd_l': np.mean(point_precisions_sd_l),
        'prec_sd_r': np.mean(point_precisions_sd_r),
        'prec_jit_l': np.mean(point_jitters_l),
        'prec_jit_r': np.mean(point_jitters_r)
    }

# ==================== 文件解析与绘图 ====================
def main():
    if not os.path.exists(BASE_DIR):
        print(f"Directory not found: {BASE_DIR}")
        return

    pupil_results = {}  
    glint_results = {}  

    re_pj = re.compile(r'calib_result_left_pj_([0-9.]+)_([0-9.]+)\.txt')
    re_gj = re.compile(r'calib_result_left_gj_([0-9.]+)_([0-9.]+)\.txt')
    re_nj = re.compile(r'calib_result_left_no_jitter\.txt')

    print("Scanning directory for results...")
    for file in os.listdir(BASE_DIR):
        if not file.startswith('calib_result_left_'):
            continue
            
        left_path = os.path.join(BASE_DIR, file)
        right_path = left_path.replace('_left_', '_right_')
        
        if not os.path.exists(right_path):
            print(f"Missing right data for {file}")
            continue
            
        try:
            left_data = np.loadtxt(left_path, comments='#')
            right_data = np.loadtxt(right_path, comments='#')
        except Exception as e:
            print(f"Error loading {file}: {e}")
            continue

        metrics = calculate_metrics(left_data, right_data)
        if not metrics:
            continue

        if re_nj.match(file):
            print(f"Loaded: Baseline (No Jitter)")
            pupil_results[0.0] = metrics
            glint_results[0.0] = metrics
        elif match := re_pj.match(file):
            val = float(match.group(1))
            print(f"Loaded: Pupil Jitter {val}")
            pupil_results[val] = metrics
        elif match := re_gj.match(file):
            val = float(match.group(1))
            print(f"Loaded: Glint Jitter {val}")
            glint_results[val] = metrics

    if not pupil_results and not glint_results:
        print("No valid jitter results found!")
        return

    # 按 Jitter 大小排序
    pupil_sorted = sorted(pupil_results.items())
    glint_sorted = sorted(glint_results.items())

    # ==================== 导出 CSV ====================
    csv_path = os.path.join(BASE_DIR, 'jitter_metrics_summary.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        # 写入表头
        writer.writerow([
            'Jitter_Type', 'Std_Dev_Pixel', 
            'Acc_Dist_cm', 'Acc_Ang_L_deg', 'Acc_Ang_R_deg', 
            'Prec_SD_L_deg', 'Prec_SD_R_deg', 
            'Prec_Jit_L_deg', 'Prec_Jit_R_deg'
        ])

        # 辅助写入函数
        def write_row(j_type, val, m):
            writer.writerow([
                j_type, val,
                f"{m['acc_dist']:.4f}",
                f"{m['acc_ang_l']:.4f}", f"{m['acc_ang_r']:.4f}",
                f"{m['prec_sd_l']:.4f}", f"{m['prec_sd_r']:.4f}",
                f"{m['prec_jit_l']:.4f}", f"{m['prec_jit_r']:.4f}"
            ])

        # 写入 Baseline (Std = 0.0)
        if 0.0 in pupil_results:
            write_row('Baseline', 0.0, pupil_results[0.0])

        # 写入 Pupil 数据
        for val, m in pupil_sorted:
            if val == 0.0: continue
            write_row('Pupil', val, m)

        # 写入 Glint 数据
        for val, m in glint_sorted:
            if val == 0.0: continue
            write_row('Glint', val, m)

    print(f"\n[SUCCESS] Saved quantitative metrics to: {csv_path}")

    # ==================== 提取数据画图 ====================
    def unpack(sorted_items):
        x = [item[0] for item in sorted_items]
        dist = [item[1]['acc_dist'] for item in sorted_items]
        ang_l = [item[1]['acc_ang_l'] for item in sorted_items]
        ang_r = [item[1]['acc_ang_r'] for item in sorted_items]
        sd_l = [item[1]['prec_sd_l'] for item in sorted_items]
        sd_r = [item[1]['prec_sd_r'] for item in sorted_items]
        jit_l = [item[1]['prec_jit_l'] for item in sorted_items]
        jit_r = [item[1]['prec_jit_r'] for item in sorted_items]
        return x, dist, ang_l, ang_r, sd_l, sd_r, jit_l, jit_r

    px, pdist, pal, par, psdl, psdr, pjl, pjr = unpack(pupil_sorted)
    gx, gdist, gal, gar, gsdl, gsdr, gjl, gjr = unpack(glint_sorted)

    fig, axes = plt.subplots(4, 2, figsize=(14, 16))
    fig.suptitle('Gaze Estimation Jitter Analysis', fontsize=16)

    def plot_metric(ax, x, y1, y2, title, ylabel, xlabel):
        if x:
            if y2 is None:
                ax.plot(x, y1, 'k-o', linewidth=2)
            else:
                ax.plot(x, y1, 'b-o', label='Left Eye', linewidth=2)
                ax.plot(x, y2, 'r-s', label='Right Eye', linewidth=2)
                ax.legend()
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            ax.set_xlabel(xlabel)
            ax.grid(True, linestyle='--', alpha=0.6)

    # Pupil 列
    plot_metric(axes[0, 0], px, pdist, None, 'Global Distance Error', 'Error (cm)', 'Pupil Jitter Std (px)')
    plot_metric(axes[1, 0], px, pal, par, 'Angular Accuracy', 'Error (Deg)', 'Pupil Jitter Std (px)')
    plot_metric(axes[2, 0], px, psdl, psdr, 'Precision (SD)', 'SD (Deg)', 'Pupil Jitter Std (px)')
    plot_metric(axes[3, 0], px, pjl, pjr, 'Precision (RMS Jitter)', 'Jitter (Deg)', 'Pupil Jitter Std (px)')

    # Glint 列
    plot_metric(axes[0, 1], gx, gdist, None, 'Global Distance Error', 'Error (cm)', 'Glint Jitter Std (px)')
    plot_metric(axes[1, 1], gx, gal, gar, 'Angular Accuracy', 'Error (Deg)', 'Glint Jitter Std (px)')
    plot_metric(axes[2, 1], gx, gsdl, gsdr, 'Precision (SD)', 'SD (Deg)', 'Glint Jitter Std (px)')
    plot_metric(axes[3, 1], gx, gjl, gjr, 'Precision (RMS Jitter)', 'Jitter (Deg)', 'Glint Jitter Std (px)')

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    png_path = os.path.join(BASE_DIR, 'jitter_analysis_results.png')
    plt.savefig(png_path, dpi=300)
    print(f"[SUCCESS] Saved plot to: {png_path}")
    plt.show()

if __name__ == '__main__':
    main()