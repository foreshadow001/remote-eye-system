// utils/src/gaze_estimation_types.cpp

#include "utils/gaze_estimation_types.hpp"

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

} // namespace gazeestimation
