#pragma once
#include "core/math_types.hpp"
#include "cfg/config.hpp"

namespace gazeestimation
{

Vec3
PoGToWCS(
    Vec2 PoG,
    const Cfg& cfg
);

Vec3
computeGazeIntersection(
    const Vec3& origin_l,   // 左眼角膜中心
    const Vec3& dir_l,      // 左眼视轴单位向量
    const Vec3& origin_r,   // 右眼角膜中心
    const Vec3& dir_r,      // 右眼视轴单位向量
    bool& is_valid          // [输出] 结果是否有效
);

Vec3
rayPlaneIntersection(
    const Vec3& ray_origin,
    const Vec3& ray_direction,
    const Cfg& cfg
);

Vec3
screenToWCS(
    const Vec2& screen_point,
    const Cfg& cfg
);

}