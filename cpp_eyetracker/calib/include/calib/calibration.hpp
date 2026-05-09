// calib/calibration.hpp

#pragma once

#include <vector>
#include <utility>

#include "core/math_types.hpp"
#include "utils/gaze_estimation_types.hpp"
#include "inference/one_camera_spherical.hpp"
#include "cfg/config.hpp"

namespace gazeestimation {

class Calibration
{
public:
    // 使用具体类型的别名使声明简短
    typedef std::vector<std::pair<PupilCenterGlintInputs, Vec3>> CalibrationDataMap;
    typedef std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>> SingleEyeCalibrationDataMap;

    std::vector<std::vector<double>> calibrate(
        GazeTracker& estimation,
        EyeAndCameraParameters& parameters,
        CalibrationDataMap& data,
        std::vector<std::vector<double>> initial_values,
        std::vector<std::vector<std::pair<double, double>>> bounds);

    std::vector<std::vector<double>> calibrate(
        GazeTracker& estimation,
        SingleEyeAndCameraParameters& parameters,
        SingleEyeCalibrationDataMap& data,
        std::vector<std::vector<double>> initial_values,
        std::vector<std::vector<std::pair<double, double>>> bounds);

    std::vector<std::vector<double>> calibrateScreen(
        GazeTracker& estimation,
        SingleEyeAndCameraParameters& parameters,
        SingleEyeCalibrationDataMap& data,
        std::vector<std::vector<double>> initial_values,
        std::vector<std::vector<std::pair<double, double>>> bounds);
};

Vec3
result_processor(
    const DefaultGazeEstimationResult& result,
    const Vec3& actual_cam_pos
);

EyeAndCameraParameters
variables_calibration_applicator(
    EyeAndCameraParameters params,
    double const* const* variables
);

SingleEyeAndCameraParameters
variables_calibration_applicator(
    SingleEyeAndCameraParameters params,
    double const* const* variables
);

const double* const* const
vecvec_to_pointer_pointer(
    std::vector<std::vector<double>>& a
);

} // namespace gazeestimation