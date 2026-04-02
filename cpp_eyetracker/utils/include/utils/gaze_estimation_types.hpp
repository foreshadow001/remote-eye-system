// utils/include/utils/gaze_estimation_types.hpp

#pragma once
#include <vector>
#include <string>

#include "cfg/config.hpp"
#include "core/math_types.hpp"
#include "cam_model/pinhole_camera_model.hpp"

namespace gazeestimation{

class DefaultGazeEstimationResult
{
public:
	DefaultGazeEstimationResult();

	struct GazeEstimationResult
	{
		Vec3 cornea_center;
		Vec3 visual_axis_unit;
		Vec3 optical_axis_unit;
	};

	GazeEstimationResult left;
	GazeEstimationResult right;

	bool is_valid = false;
	bool is_error = false;
	Vec3 gaze_point;
	std::string error;

	static DefaultGazeEstimationResult make_error(std::string error);
};

class DefaultSingleEyeGazeEstimationResult
{
public:
	DefaultSingleEyeGazeEstimationResult();

	Vec3 cornea_center;
	Vec3 visual_axis_unit;
	Vec3 optical_axis_unit;

	bool is_valid = false;
	bool is_error = false;
	std::string error;

	static DefaultSingleEyeGazeEstimationResult make_error(std::string error);
};

// --- 以下为具体的输入输出数据结构 ---
struct SingleEyePupilCenterGlintInput
{
	std::vector<Vec2> glints;
	Vec2 pupil_center;
};

struct SingleEyePupilCenterGlintInputs
{
	std::vector<SingleEyePupilCenterGlintInput> data;
};

struct PupilCenterGlintInput
{
	SingleEyePupilCenterGlintInput left;
	SingleEyePupilCenterGlintInput right;
    bool is_valid = false;
};

struct PupilCenterGlintInputs
{
	std::vector<PupilCenterGlintInput> data;
};

class EyeAndCameraParameters {
public:
    EyeAndCameraParameters();

    struct EyeParams {
        double alpha = 0.0;
        double beta = 0.0;
        double R = 0.0;
        double K = 0.0;
        double n1 = 0.0;
        double n2 = 0.0;
        double D = 0.0;
    };

    EyeParams left;
    EyeParams right;

    double eye_cam_dist_init = 0.0;

    std::vector<PinholeCameraModel> cameras;
    std::vector<Vec3> light_positions;
};

class SingleEyeAndCameraParameters {
public:
    SingleEyeAndCameraParameters(std::string which_eye);

	double alpha = 0.0;
	double beta = 0.0;
	double R = 0.0;
	double K = 0.0;
	double n1 = 0.0;
	double n2 = 0.0;
	double D = 0.0;

    double eye_cam_dist_init = 0.0;

    std::vector<PinholeCameraModel> cameras;
    std::vector<Vec3> light_positions;
};

} // namespace gazeestimation