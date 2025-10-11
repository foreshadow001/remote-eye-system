// utils/include/utils/gaze_estimation_types.hpp

#pragma once
#include <vector>

#include "utils/math_types.hpp"
#include "cam_model/pinhole_camera_model.hpp"

namespace gazeestimation{

class DefaultGazeEstimationResult
{
public:
	bool is_valid;
	bool is_error;
	Vec3 cornea_center;
	Vec3 visual_axis_unit;
	Vec3 optical_axis_unit;
	std::string error;

	explicit DefaultGazeEstimationResult();
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

// the input data for the gaze estimation method
struct PupilCenterGlintInput
{
	std::vector<Vec2> glints;	
	Vec2 pupil_center;
    bool is_valid;
};

struct PupilCenterGlintInputs
{
	std::vector<PupilCenterGlintInput> data;
};

struct EyeAndCameraParameters
{
	// eye parameters
	double alpha;
	double beta;
	double R; // R in cm
	double K; // K in cm
	double n1;
	double n2;
	double D; // D in cm

	std::vector<PinholeCameraModel> cameras;

	// lights
	std::vector<Vec3> light_positions; // light positions (ordered!, this is important for glint association)

	double eye_cam_dist_init; // initial guess for the eye-camera distance
};

}
