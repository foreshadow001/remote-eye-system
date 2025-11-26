#include <cmath>
#include <limits>

#include "utils/intersection.hpp"

namespace gazeestimation
{

Vec3
PoGToWCS(
    Vec2 PoG,
    const Cfg& cfg
)
{
    const double screen_pixel_size_x = cfg["screen_width_cm"].as<double>() / cfg["screen_width_px"].as<double>();
    const double screen_pixel_size_y = cfg["screen_height_cm"].as<double>() / cfg["screen_height_px"].as<double>();
    return make_vec3(
        PoG[0] * screen_pixel_size_x,
        - PoG[1] * screen_pixel_size_y,
        0.0
    );
}

// 假设 Vec3 支持 + - * 浮点数，dot(a,b) 和 a.normalized() 等
Vec3
computeGazeIntersection(
    const Vec3& origin_l,   // 左眼角膜中心 (m)
    const Vec3& dir_l_in,   // 左眼视轴方向（不一定归一）
    const Vec3& origin_r,   // 右眼角膜中心 (m)
    const Vec3& dir_r_in,   // 右眼视轴方向（不一定归一）
    bool& is_valid          // 输出：是否有效
)
{
    // 参数化：阈值（可调整）
    const double EPS_DENOM = 1e-9;   // 判定平行的阈值
    const double MAX_SEPARATION = 0.03; // 两最近点允许的最大分离（m），例如 3cm
    const double MIN_FORWARD = -0.05;   // 若你希望 s,t 必须是“向前”，可以设为0；带点容差

    // 先把方向归一化（重要）
    Vec3 dir_l = dir_l_in;
    Vec3 dir_r = dir_r_in;
    double nl = dir_l.norm();
    double nr = dir_r.norm();
    if (nl <= 1e-12 || nr <= 1e-12) {
        is_valid = false;
        return (origin_l + origin_r) * 0.5; // 返回中点作为占位
    }
    dir_l = dir_l / nl;
    dir_r = dir_r / nr;

    // w0 = P_L - P_R (与你原式一致)
    Vec3 w0 = origin_l - origin_r;

    double a = dot(dir_l, dir_l); // 理论上 1
    double b = dot(dir_l, dir_r);
    double c = dot(dir_r, dir_r); // 理论上 1
    double d = dot(dir_l, w0);
    double e = dot(dir_r, w0);

    double denom = a * c - b * b;
    if (std::abs(denom) < EPS_DENOM) {
        // 近似平行：不能可靠求最近点
        is_valid = false;
        // 返回左右眼中点投影在某个默认远处（不要返回巨大的 1000 单位）
        // 这里返回两眼中心点前方 1 m 作为占位（但标记无效）
        Vec3 mid_eye = (origin_l + origin_r) * 0.5;
        Vec3 forward = (dir_l + dir_r) * 0.5;
        if (forward.norm() < 1e-6) forward = Vec3(0,0,1); // 兜底
        forward = forward / forward.norm();
        return mid_eye + forward * 1.0; // 1 m 前方
    }

    // 两条线的最近点参数
    double s = (b * e - c * d) / denom;
    double t = (a * e - b * d) / denom;

    // 计算最近点与它们之间的分离
    Vec3 point_on_l = origin_l + dir_l * s;
    Vec3 point_on_r = origin_r + dir_r * t;
    double separation = (point_on_l - point_on_r).norm();

    // 合理性检查：
    // - 如果两最近点相距太远 => 无效
    // - 可选：如果 s 或 t 明显为负（在头部后方），认为无效（取决于你定义）
    bool s_t_forward_ok = (s >= MIN_FORWARD) && (t >= MIN_FORWARD);
    if (separation > MAX_SEPARATION || !s_t_forward_ok) {
        is_valid = false;
    } else {
        is_valid = true;
    }

    // 返回中点（即两最近点的中点）
    Vec3 gaze_point = (point_on_l + point_on_r) * 0.5;
    return gaze_point;
}


Vec3
rayPlaneIntersection(
    const Vec3& ray_origin,
    const Vec3& ray_direction,
    const Cfg& cfg
)
{
    Vec3 screen_normal = cfg["screen_vertical_direction"].as<Vec3>().cross(cfg["screen_horizontal_direction"].as<Vec3>());
    screen_normal.normalize();
    Vec3 screen_center = cfg["screen_center"].as<Vec3>();
    Vec3 ray_dir = ray_direction;
    ray_dir.normalize();

    double denominator = dot(screen_normal, ray_dir);

    if (std::abs(denominator) < 1e-6) {
        return Vec3(std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity());
    }

    double t = dot(screen_normal, screen_center - ray_origin) / denominator;

    Vec3 intersection_point = ray_origin + ray_dir * t;

    return intersection_point;
}

Vec2
pointToScreenCoordinate(
    const Vec3& intersection_point,
    const Cfg& cfg
)
{
    Vec3 screen_u = cfg["screen_horizontal_direction"].as<Vec3>();
    screen_u.normalize();
    Vec3 screen_v = cfg["screen_vertical_direction"].as<Vec3>();
    screen_v.normalize();

    Vec3 relative_point = intersection_point - cfg["screen_center"].as<Vec3>();

    double u_distance = dot(relative_point, screen_u);
    double v_distance = dot(relative_point, screen_v);

    double pixel_scale_u = cfg["screen_width_cm"].as<double>() / cfg["screen_width_px"].as<double>();
    double pixel_scale_v = cfg["screen_height_cm"].as<double>() / cfg["screen_height_px"].as<double>();

    double u_pixel = u_distance / pixel_scale_u;
    double v_pixel = v_distance / pixel_scale_v;

    double PoG_x = u_pixel + (cfg["screen_width_px"].as<double>() / 2.0);
    double PoG_y = v_pixel + (cfg["screen_height_px"].as<double>() / 2.0);

    return make_vec2(PoG_x, PoG_y);
}

Vec2
rayToScreenCoordinate(
    const Vec3& ray_origin,
    const Vec3& ray_direction,
    const Cfg& cfg
)
{
    Vec3 intersection_point = rayPlaneIntersection(ray_origin, ray_direction, cfg);
    
    if (std::isinf(intersection_point[0]) ||
        std::isinf(intersection_point[1]) ||
        std::isinf(intersection_point[2])) {
        return make_vec2(std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity());
    }

    Vec2 PoG = pointToScreenCoordinate(intersection_point, cfg);
    return PoG;




}

Vec3
screenToWCS(
    const Vec2& screen_point,
    const Cfg& cfg
)
{
    const double screen_pixel_size_x = cfg["screen_width_cm"].as<double>() / cfg["screen_width_px"].as<double>();
    const double screen_pixel_size_y = cfg["screen_height_cm"].as<double>() / cfg["screen_height_px"].as<double>();
    
    double u_pixel = screen_point[0] - (cfg["screen_width_px"].as<double>() / 2.0);
    double v_pixel = screen_point[1] - (cfg["screen_height_px"].as<double>() / 2.0);

    double u_distance = u_pixel * screen_pixel_size_x;
    double v_distance = v_pixel * screen_pixel_size_y;

    Vec3 screen_u = cfg["screen_horizontal_direction"].as<Vec3>();
    screen_u.normalize();
    Vec3 screen_v = cfg["screen_vertical_direction"].as<Vec3>();
    screen_v.normalize();

    Vec3 wcs_point = cfg["screen_center"].as<Vec3>() + screen_u * u_distance + screen_v * v_distance;

    return wcs_point;
}

}