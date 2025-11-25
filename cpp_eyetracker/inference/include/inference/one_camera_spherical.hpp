#pragma once

#include <functional>
#include <memory> // for std::unique_ptr

#include "core/math_types.hpp"
#include "utils/gaze_estimation_types.hpp"
#include "utils/intersection.hpp"

namespace gazeestimation
{

class Vec3MedianFilter
{
public:
	explicit Vec3MedianFilter(unsigned int size);
	Vec3 newSample(gazeestimation::Vec3 new_sample);
	void reset();

private:
	std::list<gazeestimation::Vec3> data;
	unsigned int size;
};

class GazeTracker:
public GazeEstimationMethod<EyeAndCameraParameters, PupilCenterGlintInputs, DefaultGazeEstimationResult>
{
public:
    typedef std::function<Vec3(Vec3)> Vec3Filter;
    
    GazeTracker(); 

    DefaultGazeEstimationResult
    estimate(const PupilCenterGlintInputs& input,
             const EyeAndCameraParameters& eye_cam_params
            ) const override;

    void setCorneaCenterFilter(Vec3Filter filter);
    void setPupilCenterFilter(Vec3Filter filter);

private:
    // 内部初始化函数，用于读取配置
    void loadSettingsFromConfig();

    bool use_chen_noise_reduction = false;
    
    Vec3Filter cornea_center_filter;
    Vec3Filter pupil_center_filter;

    std::unique_ptr<gazeestimation::Vec3MedianFilter> ptr_cc_filter;
    std::unique_ptr<gazeestimation::Vec3MedianFilter> ptr_pc_filter;
};

Vec3
calculateCorneaCenterWCS(
    const std::vector<Vec3>* const glints,
    const std::vector<Vec3>* const lights,
    const Vec3& cam_pos,
    double R,
    double eye_cam_dist_init
);

class DistanceBetweenCorneasFunctor
{
private:
    const std::vector<Vec3>* const glints_;
    const std::vector<Vec3>* const lights_;
    double R_;
    const Vec3 cam_pos_;

public:
    DistanceBetweenCorneasFunctor(
        const std::vector<Vec3>* const glints,
        const std::vector<Vec3>* const lights,
        double r,
        Vec3 cam_pos);

    bool operator()(double const* const* variables, double* residual) const;
};

Vec3
solveCorneaCenter(
    std::vector<Vec2> glints,
    PinholeCameraModel camera,
    std::vector<Vec3> light_positions,
    double R,
    double eye_cam_dist_init
);

Vec3
geometricMedian3(const std::list<Vec3>& points);

} // namespace gazeestimation
