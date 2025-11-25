// utils/include/utils/gaze_estimation_types.hpp

#pragma once
#include <vector>

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

	bool is_valid;
	bool is_error;
	Vec3 gaze_point;
	std::string error;

	static DefaultGazeEstimationResult make_error(std::string error);
};


template <class Parameters, class InputData, class GazeEstimationResult>
class GazeEstimationMethod
{
public:
	virtual ~GazeEstimationMethod();
	virtual GazeEstimationResult estimate(const InputData& data,
                                          const Parameters& parameters) const = 0;
};

template <class Parameters, class InputData, class GazeEstimationResult>
GazeEstimationMethod<Parameters, InputData, GazeEstimationResult>::~GazeEstimationMethod() {}

template <class CalibratedParameters>
class CalibrationMethod
{
public:
	virtual ~CalibrationMethod();
};

template <class CalibratedParameters>
CalibrationMethod<CalibratedParameters>::~CalibrationMethod() {}

struct SingleEyePupilCenterGlintInput
{
	std::vector<Vec2> glints;
	Vec2 pupil_center;
};

// the input data for the gaze estimation method
struct PupilCenterGlintInput
{
	SingleEyePupilCenterGlintInput left;
	SingleEyePupilCenterGlintInput right;
    bool is_valid;
};

struct PupilCenterGlintInputs
{
	std::vector<PupilCenterGlintInput> data;
};

class EyeAndCameraParameters {
public:
    EyeAndCameraParameters();

    // ---------- 左右眼参数 ----------
    struct EyeParams {
        double alpha = 0.0;
        double beta = 0.0;
        double R = 0.0;
        double K = 0.0;
        double n1 = 0.0;
        double n2 = 0.0;
        double D = 0.0;
    };

    EyeParams left;   // 左眼参数
    EyeParams right;  // 右眼参数

    double eye_cam_dist_init = 0.0;

    std::vector<PinholeCameraModel> cameras;
    std::vector<Vec3> light_positions;
};

}
