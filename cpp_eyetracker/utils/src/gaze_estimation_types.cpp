// utils/src/gaze_estimation_types.cpp

#include "utils/gaze_estimation_types.hpp"
#include <iostream>

namespace gazeestimation
{

DefaultGazeEstimationResult::DefaultGazeEstimationResult():
	is_valid(false),
	is_error(false),
	cornea_center(0, 0, 0),
	visual_axis_unit(0, 0, 0),
	optical_axis_unit(0, 0, 0),
	error("") { }

DefaultGazeEstimationResult DefaultGazeEstimationResult::make_error(std::string error)
{
	DefaultGazeEstimationResult res;
	res.is_valid = false;
	res.is_error = true;
	res.error = error;
	return res;
}

EyeAndCameraParameters::EyeAndCameraParameters(const std::string& left_or_right)
{
    Cfg cfg;

    // 获取节点
    auto eye = cfg["calib_init_values"];

    // ----------- 眼球参数 -----------
    alpha = eye["alpha"].as<double>();
    if (left_or_right == "right") alpha = - alpha;

    beta = eye["beta"].as<double>();
    R    = eye["R"].as<double>();
    K    = eye["K"].as<double>();
    n1   = eye["n1"].as<double>();
    n2   = eye["n2"].as<double>();
    D    = eye["D"].as<double>();
	eye_cam_dist_init = eye["eye_cam_dist_init"].as<double>();

	light_positions = cfg["lights_pos"].as<std::vector<Vec3>>();
    
}

} // namespace gazeestimation
