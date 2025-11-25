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

Vec3 computeGazeIntersection(
    const Vec3& origin_l,   // 左眼角膜中心
    const Vec3& dir_l,      // 左眼视轴单位向量
    const Vec3& origin_r,   // 右眼角膜中心
    const Vec3& dir_r,      // 右眼视轴单位向量
    bool& is_valid          // [输出] 结果是否有效
)
{
    // 1. 计算起始点之间的向量 w0 = P_L - P_R
    Vec3 w0 = origin_l - origin_r;

    // 2. 计算点积系数
    // a = dir_l · dir_l (如果由于归一化，通常为 1)
    // b = dir_l · dir_r
    // c = dir_r · dir_r (如果由于归一化，通常为 1)
    // d = dir_l · w0
    // e = dir_r · w0
    float a = dot(dir_l, dir_l);
    float b = dot(dir_l, dir_r);
    float c = dot(dir_r, dir_r);
    float d = dot(dir_l, w0);
    float e = dot(dir_r, w0);

    // 3. 计算分母 (ac - b^2)
    float denom = a * c - b * b;

    // 4. 检查平行情况
    // 如果 denom 非常接近 0，说明视线平行（注视无穷远）或重合
    if (std::abs(denom) < 1e-6f) {
        is_valid = false; 
        // 平行时通常无法计算交点，可以返回一个默认的远距离点，或者保持无效
        // 这里简单的取左眼视线前方 1米处作为 fallback
        return origin_l + dir_l * 1000.0f; 
    }

    // 5. 计算两条射线上的最近点参数 s (左眼) 和 t (右眼)
    // s = (be - cd) / denom
    // t = (ae - bd) / denom
    float s = (b * e - c * d) / denom;
    float t = (a * e - b * d) / denom;
    
    // 6. 额外的合理性检查 (可选)
    // 如果 s 或 t 为负数，说明交点在脑袋后方，这是不合理的
    if (s < 0 || t < 0) {
        is_valid = false;
    }

    // 7. 计算两条射线上距离最近的两个点
    Vec3 point_on_l = origin_l + dir_l * s;
    Vec3 point_on_r = origin_r + dir_r * t;

    // 8. 返回这两个点的中点作为最终注视点
    Vec3 gaze_point = (point_on_l + point_on_r) * 0.5f;

    return gaze_point + make_vec3(29.0, -31.0, 17.0);
}

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