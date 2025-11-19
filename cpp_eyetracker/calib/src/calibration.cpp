#include "calib/calibration.hpp"
#include "utils/shared_calculations.hpp"

namespace gazeestimation
{

Vec3
result_processor(
    const DefaultGazeEstimationResult& result,
    const Vec3& actual_cam_pos
)
{
    return calculatePointOfInterest(
        result.cornea_center,
        result.visual_axis_unit,
        - actual_cam_pos[2]
    ) + actual_cam_pos;
}

EyeAndCameraParameters
variables_calibration_applicator(
    EyeAndCameraParameters params,
    double const* const* variables
)
{
    params.alpha = variables[0][0];
    params.beta = variables[1][0];
    params.R = variables[2][0];
    params.K = variables[3][0];
    // params.cameras[0].set_camera_angle_y(variables[4][0]);
    // params.cameras[0].set_camera_angle_z(variables[5][0]);
    return params;
}

const double* const* const
vecvec_to_pointer_pointer(
    std::vector<std::vector<double>>& a
)
{
    std::vector<double*> tmp;
    for (unsigned int i = 0; i < a.size(); i++)
        tmp.push_back(&a[i][0]);
    return &tmp[0];
}

}