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

}