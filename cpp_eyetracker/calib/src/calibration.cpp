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
        result.left.cornea_center,
        result.left.visual_axis_unit,
        - actual_cam_pos[2]
    ) + actual_cam_pos;
}

EyeAndCameraParameters
variables_calibration_applicator(
    EyeAndCameraParameters params,
    double const* const* variables
)
{
    params.left.alpha = variables[0][0];
    params.left.beta = variables[1][0];
    params.left.R = variables[2][0];
    params.left.K = variables[3][0];
    
    params.right.alpha = variables[4][0];
    params.right.beta = variables[5][0];
    params.right.R = variables[6][0];
    params.right.K = variables[7][0];
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