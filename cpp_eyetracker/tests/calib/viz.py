import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider
import yaml
import os

# ==================== 路径配置 ====================
BASE_DIR = "D:/hitsz/projects/new_dataset/debug_gaze"
CONFIG_PATH = "D:/hitsz/projects/new_dataset/remote-eye-system/cpp_eyetracker/cfg/default.yaml"
LEFT_DATA_PATH = os.path.join(BASE_DIR, "calib_inference_result_left_single.txt")
RIGHT_DATA_PATH = os.path.join(BASE_DIR, "calib_inference_result_right_single.txt")

# ==================== 列索引常量 ====================
# 对应 C++ 输出: cornea(3) opt(3) vis(3) gt(3) pred(3)
IDX_CORNEA = 0
IDX_OPT    = 3
IDX_VIS    = 6  # 视轴方向向量
IDX_GT     = 9
IDX_PRED   = 12

class GazeVisualizer:
    def __init__(self, left_file, right_file, config_file):
        self.load_config(config_file)
        self.left_data = self.load_data(left_file)
        self.right_data = self.load_data(right_file)
        
        self.num_frames = min(len(self.left_data), len(self.right_data))
        if self.num_frames == 0:
            raise ValueError("No data found.")
        print(f"Total Frames: {self.num_frames}")

        # 启动时计算并打印总体平均误差
        self.print_summary_stats()

        self.calc_screen_geometry()

        # 初始化绘图
        self.fig = plt.figure(figsize=(15, 9))
        plt.subplots_adjust(bottom=0.15)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.dynamic_artists =[]
        self.draw_static_scene()
        self.update_frame(0)

        # 进度条
        ax_slider = plt.axes([0.2, 0.05, 0.60, 0.03], facecolor='lightgray')
        self.slider = Slider(ax_slider, 'Frame', 0, self.num_frames - 1, valinit=0, valstep=1)
        self.slider.on_changed(self.update_frame)

    def load_config(self, path):
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                self.cfg = yaml.safe_load(f)
            print(f"Config loaded: {path}")
        else:
            print("[WARN] Config not found, using defaults.")
            self.cfg = {
                'screen_width_cm': 59.5, 'screen_height_cm': 33.6,
                'screen_center': [0, 0, 0],
                'screen_horizontal_direction': [-1, 0, 0], 'screen_vertical_direction':[0, 1, 0]
            }

    def load_data(self, path):
        if not os.path.exists(path):
            return[]
        try:
            return np.loadtxt(path, comments='#')
        except Exception as e:
            print(f"[ERR] Load failed {path}: {e}")
            return[]

    def normalize(self, v):
        n = np.linalg.norm(v)
        return v if n == 0 else v / n

    def closest_points_on_two_lines(self, O1, D1, O2, D2):
        """
        计算两条3D射线之间最接近的两个点
        """
        W0 = O1 - O2
        a = np.dot(D1, D1)
        b = np.dot(D1, D2)
        c = np.dot(D2, D2)
        d = np.dot(D1, W0)
        e = np.dot(D2, W0)
        denom = a * c - b * b
        
        if denom < 1e-6:
            # 视线几乎平行，直接返回原点
            return O1, O2
            
        s = (b * e - c * d) / denom
        t = (a * e - b * d) / denom
        
        P1 = O1 + s * D1
        P2 = O2 + t * D2
        return P1, P2

    def calc_binocular_errors(self, row_l, row_r):
        """
        计算全局距离误差与双眼角度误差
        """
        cornea_l = row_l[IDX_CORNEA : IDX_CORNEA+3]
        vis_l    = self.normalize(row_l[IDX_VIS : IDX_VIS+3])
        gt       = row_l[IDX_GT : IDX_GT+3]  # 左眼右眼的 GT 是一致的
        
        cornea_r = row_r[IDX_CORNEA : IDX_CORNEA+3]
        vis_r    = self.normalize(row_r[IDX_VIS : IDX_VIS+3])
        
        # 1. 计算双眼视线相交点(或最接近点)的中点，作为全局预测点 P_pred
        P1, P2 = self.closest_points_on_two_lines(cornea_l, vis_l, cornea_r, vis_r)
        P_pred = (P1 + P2) / 2.0
        
        # 2. 全局距离误差
        dist_err = np.linalg.norm(gt - P_pred)
        
        # 3. 角度误差
        gt_vec_l = self.normalize(gt - cornea_l)
        ang_l = np.degrees(np.arccos(np.clip(np.dot(vis_l, gt_vec_l), -1.0, 1.0)))
        
        gt_vec_r = self.normalize(gt - cornea_r)
        ang_r = np.degrees(np.arccos(np.clip(np.dot(vis_r, gt_vec_r), -1.0, 1.0)))
        
        return dist_err, ang_l, ang_r, P_pred

    def print_summary_stats(self):
        """打印全局平均误差数据"""
        dist_list, ang_l_list, ang_r_list = [], [],[]

        for i in range(self.num_frames):
            dist_err, ang_l, ang_r, _ = self.calc_binocular_errors(self.left_data[i], self.right_data[i])
            dist_list.append(dist_err)
            ang_l_list.append(ang_l)
            ang_r_list.append(ang_r)

        print("\n" + "="*50)
        print("          GLOBAL ERROR SUMMARY")
        print("="*50)
        print(f"Mean Global Dist Loss : {np.mean(dist_list):.2f} cm")
        print(f"Mean Left Angular Loss: {np.mean(ang_l_list):.2f}°")
        print(f"Mean Right Angular Loss:{np.mean(ang_r_list):.2f}°")
        print("="*50 + "\n")

    def calc_screen_geometry(self):
        center = np.array(self.cfg['screen_center'])
        w = float(self.cfg['screen_width_cm'])
        h = float(self.cfg['screen_height_cm'])
        vec_x = self.normalize(np.array(self.cfg['screen_horizontal_direction']))
        vec_y = self.normalize(np.array(self.cfg['screen_vertical_direction']))
        
        self.screen_normal = self.normalize(np.cross(vec_x, vec_y))
        self.screen_center = center
        
        half_w = 0.5 * w * vec_x
        half_h = 0.5 * h * vec_y
        self.screen_corners =[
            center - half_w - half_h, center + half_w - half_h,
            center + half_w + half_h, center - half_w + half_h
        ]

    def intersect_ray_plane(self, ray_origin, ray_dir):
        denom = np.dot(ray_dir, self.screen_normal)
        if abs(denom) < 1e-6: return None
        t = np.dot(self.screen_center - ray_origin, self.screen_normal) / denom
        if t < 0: return None
        return ray_origin + t * ray_dir

    def draw_static_scene(self):
        """绘制静态场景：屏幕 + 相机坐标轴"""
        # 1. 画屏幕面
        verts = [self.screen_corners]
        poly = Poly3DCollection(verts, alpha=0.1, facecolors='gray', edgecolors='k')
        self.ax.add_collection3d(poly)
        
        # 2. 画相机坐标系 (RGB -> XYZ)
        origin = np.array([0, 0, 0])
        axis_len = 5.0 # 坐标轴长度 (cm)
        
        # X轴 (Red)
        self.ax.plot([origin[0], origin[0]+axis_len], [origin[1], origin[1]], [origin[2], origin[2]], 
                     c='r', lw=1, linestyle='--', label='Cam X')
        self.ax.text(origin[0]+axis_len, origin[1], origin[2], "X", color='r')
        
        # Y轴 (Green)
        self.ax.plot([origin[0], origin[0]], [origin[1], origin[1]+axis_len], [origin[2], origin[2]], 
                     c='g', lw=1, linestyle='--', label='Cam Y')
        self.ax.text(origin[0], origin[1]+axis_len, origin[2], "Y", color='g')
        
        # Z轴 (Blue)
        self.ax.plot([origin[0], origin[0]], [origin[1], origin[1]], [origin[2], origin[2]+axis_len], 
                     c='b', lw=1, linestyle='--', label='Cam Z')
        self.ax.text(origin[0], origin[1], origin[2]+axis_len, "Z", color='b')
        
        # 标记原点
        self.ax.scatter(*origin, c='k', marker='o', s=50)

        # 设置显示范围和标签
        self.ax.set_xlabel('X (cm)'); self.ax.set_ylabel('Y (cm)'); self.ax.set_zlabel('Z (cm)')
        
        # 比例控制
        pts = np.vstack(self.screen_corners + [[0,0,0]])
        mid = np.mean(pts, axis=0)
        rng = 30.0
        self.ax.set_xlim(mid[0]-rng, mid[0]+rng)
        self.ax.set_ylim(mid[1]-rng, mid[1]+rng)
        self.ax.set_zlim(mid[2]-rng, mid[2]+rng)
        
        # 初始视角
        self.ax.view_init(elev=90, azim=90)

    def draw_ray(self, origin, direction, color, style, label=None):
        """通用射线绘制函数"""
        hit_pt = self.intersect_ray_plane(origin, direction)
        arts =[]
        if hit_pt is not None:
            # 画线到交点
            line, = self.ax.plot(
                [origin[0], hit_pt[0]], [origin[1], hit_pt[1]], [origin[2], hit_pt[2]],
                c=color, ls=style, lw=(2 if style=='-' else 1), label=label
            )
            pt = self.ax.scatter(*hit_pt, c=color, marker='.', s=20)
            arts.extend([line, pt])
        else:
            # 未击中屏幕，画一段
            end = origin + direction * 25.0
            line, = self.ax.plot([origin[0], end[0]], [origin[1], end[1]], [origin[2], end[2]],
                c=color, ls=style, lw=1, alpha=0.5, label=label
            )
            arts.append(line)
        return arts

    def draw_eye_data(self, row, c_vis, c_opt, lbl):
        # 提取当前眼数据
        cornea = row[IDX_CORNEA : IDX_CORNEA+3]
        opt_vec = self.normalize(row[IDX_OPT : IDX_OPT+3])
        vis_vec = self.normalize(row[IDX_VIS : IDX_VIS+3])
        
        arts =[]
        arts.append(self.ax.scatter(*cornea, c=c_vis, s=15))
        arts.extend(self.draw_ray(cornea, vis_vec, c_vis, '-', f'{lbl} Vis Axis'))
        arts.extend(self.draw_ray(cornea, opt_vec, c_opt, '--', f'{lbl} Opt Axis'))

        return arts

    def update_frame(self, val):
        idx = int(val)
        for a in self.dynamic_artists: a.remove()
        self.dynamic_artists.clear()
        
        row_l = self.left_data[idx]
        row_r = self.right_data[idx]
        
        # 1. 绘制左右眼射线和角膜
        al = self.draw_eye_data(row_l, 'blue', 'cyan', 'L')
        ar = self.draw_eye_data(row_r, 'red', 'magenta', 'R')
        self.dynamic_artists.extend(al + ar)
        
        # 2. 计算误差与双眼交点(预测点)
        dist_err, ang_l, ang_r, P_pred = self.calc_binocular_errors(row_l, row_r)
        
        # 3. 绘制 GT 与 Binocular Pred 点
        gt = row_l[IDX_GT : IDX_GT+3]
        pgt = self.ax.scatter(*gt, c='green', s=60, marker='o', label='GT')
        ppred = self.ax.scatter(*P_pred, c='orange', marker='X', s=150, lw=2, label='Binocular Pred')
        
        self.dynamic_artists.extend([pgt, ppred])
        
        # 更新标题显示
        title = (f"Frame {idx} | Global Dist Err: {dist_err:.2f} cm | "
                 f"Ang Err (L: {ang_l:.2f}°, R: {ang_r:.2f}°)")
        self.ax.set_title(title, fontsize=11)
        
        self.fig.canvas.draw_idle()

    def show(self):
        from matplotlib.lines import Line2D
        legs = [
            Line2D([0], [0], color='blue', lw=2, label='Left Vis Ray'),
            Line2D([0], [0], color='cyan', lw=1, ls='--', label='Left Opt Ray'),
            Line2D([0], [0], color='red', lw=2, label='Right Vis Ray'),
            Line2D([0], [0], color='magenta', lw=1, ls='--', label='Right Opt Ray'),
            Line2D([0], [0], marker='X', color='orange', ls='None', ms=10, label='Binocular Pred'),
            Line2D([0], [0], marker='o', color='green', ls='None', label='GT')
        ]
        self.ax.legend(handles=legs, loc='upper right', fontsize='small')
        plt.show()

if __name__ == "__main__":
    if os.path.exists(LEFT_DATA_PATH) and os.path.exists(RIGHT_DATA_PATH):
        viz = GazeVisualizer(LEFT_DATA_PATH, RIGHT_DATA_PATH, CONFIG_PATH)
        viz.show()
    else:
        print("Data file not found. Ensure both left and right data paths exist.")