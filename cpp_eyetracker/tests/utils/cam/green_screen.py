import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ================= 配置参数 =================
# 相机参数
NUM_CAMERAS = 9
FOCAL_LENGTH = 16e-3        # 16mm 焦距 (单位: 米)
RES_W = 2448                # 分辨率 宽
RES_H = 2048                # 分辨率 高
PIXEL_SIZE = 2.74e-6        # 像元尺寸 2.74um (单位: 米)

# 传感器物理尺寸
SENSOR_W = RES_W * PIXEL_SIZE
SENSOR_H = RES_H * PIXEL_SIZE

# 阵列结构参数
R_SPHERE = 1.1              # 球面半径 1.1m
H_CENTER = 1.25             # 球心离地高度 1.25m (用于可视化参照)

# 绿幕位置参数
# 解读：假设绿幕是作为背景，放置在球心对侧。
# 如果中间相机在 Z = +1.1, 朝向 Z = 0
# 我们假设绿幕平面位于 Z = -1.5 (即距离球心1.5m处，作为背景墙)
# 如果您原本的意思是绿幕在物理相机机身后的墙上(比如做反光背景)，请修改此值为 +2.6
GREEN_SCREEN_Z_WORLD = - 0.25

# 角度分布 (度)
# 中间一排: 纬度 0, 经度 -45, 0, 45
# 上面一排: 纬度 45, 经度 -45, 0, 45
# 下面一排: 纬度 -45, 经度 -45, 0, 45
LATS = [0, 0, 0, 45, 45, 45, -45, -45, -45] # Elevation (Phi)
LONS = [-45, 0, 45, -45, 0, 45, -45, 0, 45] # Azimuth (Theta)

# ================= 数学工具函数 =================

def spherical_to_cartesian(r, lat_deg, lon_deg):
    """将球坐标转换为笛卡尔坐标 (Y轴向上, Z轴向前)"""
    # 注意：这里我们定义 Z轴为正前方(中间相机位置)，Y轴为正上方
    # 数学惯例调整：让(0,0)对应(0,0,r)
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    
    # 假设中间相机(0,0)位于 Z轴正方向
    x = r * np.sin(lon) * np.cos(lat)
    y = r * np.sin(lat)
    z = r * np.cos(lon) * np.cos(lat)
    return np.array([x, y, z])

def look_at_matrix(camera_pos, target_pos, up_vector=np.array([0, 1, 0])):
    """计算相机的旋转矩阵 (World -> Camera) 的逆，即 Camera -> World"""
    z_axis = camera_pos - target_pos # 相机看向 target，Z轴指向相机后方
    z_axis = z_axis / np.linalg.norm(z_axis)
    
    x_axis = np.cross(up_vector, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    y_axis = np.cross(z_axis, x_axis)
    
    # 构建旋转矩阵 (列向量为基向量)
    R = np.column_stack((x_axis, y_axis, z_axis))
    return R

def ray_plane_intersection(ray_origin, ray_dir, plane_z):
    """计算射线与 Z = plane_z 平面的交点"""
    # Ray: P = O + t * D
    # P.z = plane_z => O.z + t * D.z = plane_z => t = (plane_z - O.z) / D.z
    
    if abs(ray_dir[2]) < 1e-6: # 平行于平面
        return None
        
    t = (plane_z - ray_origin[2]) / ray_dir[2]
    
    if t < 0: # 交点在射线反方向 (相机背后)
        return None
        
    intersection = ray_origin + t * ray_dir
    return intersection

# ================= 主逻辑 =================

def main():
    # 1. 计算相机内参 (视场角)
    # 在单位距离平面(Z=1)上的成像范围的一半
    half_w_at_1m = (SENSOR_W / 2) / FOCAL_LENGTH
    half_h_at_1m = (SENSOR_H / 2) / FOCAL_LENGTH
    
    # 传感器四个角的射线向量 (在相机局部坐标系下，相机看向 -Z)
    # 顺序：左上, 右上, 右下, 左下
    # 注意：在相机坐标系中，看向 -Z，所以 Z = -1
    local_rays = [
        np.array([-half_w_at_1m, half_h_at_1m, -1]), # Top-Left
        np.array([ half_w_at_1m, half_h_at_1m, -1]), # Top-Right
        np.array([ half_w_at_1m, -half_h_at_1m, -1]),# Bottom-Right
        np.array([-half_w_at_1m, -half_h_at_1m, -1]) # Bottom-Left
    ]
    
    # 归一化射线
    local_rays = [v / np.linalg.norm(v) for v in local_rays]
    
    cameras_pos = []
    all_intersections = []
    
    # 准备绘图
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    print(f"{'Camera ID':<10} | {'Position (x,y,z)':<25}")
    print("-" * 40)

    # 2. 遍历所有相机
    for i in range(NUM_CAMERAS):
        # 计算位置
        pos = spherical_to_cartesian(R_SPHERE, LATS[i], LONS[i])
        cameras_pos.append(pos)
        
        print(f"Cam {i+1:<6} | {pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}")
        
        # 计算旋转 (看向原点)
        R_matrix = look_at_matrix(pos, np.array([0, 0, 0]))
        
        # 绘制相机位置
        ax.scatter(pos[0], pos[1], pos[2], c='red', marker='o', s=50)
        ax.text(pos[0], pos[1], pos[2], f"C{i+1}", fontsize=8)
        
        # 投射视锥体射线
        frustum_points = []
        intersections_for_this_cam = []
        
        for local_ray in local_rays:
            # 将射线转到世界坐标系
            world_ray_dir = R_matrix @ local_ray
            
            # 计算与绿幕平面的交点
            hit = ray_plane_intersection(pos, world_ray_dir, GREEN_SCREEN_Z_WORLD)
            
            # 绘制视锥体连线 (只画一小段示意，避免乱)
            ax.plot([pos[0], pos[0] + world_ray_dir[0]*0.5],
                    [pos[1], pos[1] + world_ray_dir[1]*0.5],
                    [pos[2], pos[2] + world_ray_dir[2]*0.5], 'k-', alpha=0.3, linewidth=0.5)
            
            if hit is not None:
                all_intersections.append(hit)
                intersections_for_this_cam.append(hit)
                # 绘制从相机到幕布的线
                ax.plot([pos[0], hit[0]], [pos[1], hit[1]], [pos[2], hit[2]], 'b--', alpha=0.1)

        # 绘制该相机在绿幕上的投影框
        if len(intersections_for_this_cam) == 4:
            xs = [p[0] for p in intersections_for_this_cam]
            ys = [p[1] for p in intersections_for_this_cam]
            zs = [p[2] for p in intersections_for_this_cam]
            # 闭合矩形
            xs.append(xs[0])
            ys.append(ys[0])
            zs.append(zs[0])
            ax.plot(xs, ys, zs, 'c-', alpha=0.5, linewidth=1)

    # 3. 计算绿幕尺寸
    if not all_intersections:
        print("\n错误：视锥体与平面没有交点。请检查绿幕位置设置。")
        return

    all_hits = np.array(all_intersections)
    min_x, max_x = np.min(all_hits[:, 0]), np.max(all_hits[:, 0])
    min_y, max_y = np.min(all_hits[:, 1]), np.max(all_hits[:, 1])
    
    width = max_x - min_x
    height = max_y - min_y
    
    print("\n" + "="*40)
    print(" 计算结果 ")
    print("="*40)
    print(f"绿幕平面位置 Z: {GREEN_SCREEN_Z_WORLD} m")
    print(f"所需绿幕覆盖范围 X: [{min_x:.3f}, {max_x:.3f}]")
    print(f"所需绿幕覆盖范围 Y: [{min_y:.3f}, {max_y:.3f}]")
    print(f"建议绿幕尺寸 (宽 x 高): {width:.3f} m x {height:.3f} m")
    print("="*40)

    # 4. 绘制绿幕平面矩形
    rect_x = [min_x, max_x, max_x, min_x, min_x]
    rect_y = [min_y, min_y, max_y, max_y, min_y]
    rect_z = [GREEN_SCREEN_Z_WORLD] * 5
    
    # 画出绿幕边界
    ax.plot(rect_x, rect_y, rect_z, 'g-', linewidth=2, label='Required Green Screen')
    # 填充绿幕区域
    verts = [list(zip(rect_x, rect_y, rect_z))]
    poly = Poly3DCollection(verts, alpha=0.3, facecolors='g')
    ax.add_collection3d(poly)
    
    # 辅助绘图：球心
    ax.scatter(0, 0, 0, c='black', marker='x', s=100, label='Subject Center (0,0,0)')
    
    # 设置坐标轴
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'9-Camera Array & Green Screen FOV Coverage\nScreen Size: {width:.2f}m x {height:.2f}m')
    
    # 调整视角便于观察
    ax.view_init(elev=20, azim=45)
    
    # 使得比例一致 (Matplotlib 3D 默认比例往往不对)
    max_range = np.array([max_x-min_x, max_y-min_y, 4.0]).max() / 2.0
    mid_x = (max_x+min_x) * 0.5
    mid_y = (max_y+min_y) * 0.5
    mid_z = (R_SPHERE + GREEN_SCREEN_Z_WORLD) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(GREEN_SCREEN_Z_WORLD - 1, R_SPHERE + 1)
    
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()