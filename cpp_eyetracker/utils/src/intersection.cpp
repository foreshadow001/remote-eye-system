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

}