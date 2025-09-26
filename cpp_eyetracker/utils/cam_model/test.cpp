#include <iostream>

#include "pinhole_camera_model.hpp"

using namespace gazeestimation;

int main() {
    PinholeCameraModel cam;

    int x;
    int y;
    Vec3 wcs;

    cam.set_camera_angles(0.2, 0.1, 0.3);
    cam.principal_point_x = 800;
    cam.principal_point_y = 600;
    cam.pixel_size_cm_x = 0.01;
    cam.pixel_size_cm_y = 0.01;
    cam.effective_focal_length_cm = 0.01;
    cam.position = make_vec3(500, 100, 200);

    x = 100;
    y = 200;
    wcs = cam.ics_to_wcs(make_vec2(x, y));
    std::cout << "wcs: " << wcs.transpose() << std::endl;

    return 0;
}