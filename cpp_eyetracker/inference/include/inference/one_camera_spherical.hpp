#include "utils/gaze_estimation_types.hpp"

namespace gazeestimation
{

class GazeTracker:
public GazeEstimationMethod<EyeAndCameraParameters, PupilCenterGlintInputs, DefaultGazeEstimationResult>
{
public:
    typedef std::function<Vec3(Vec3)> Vec3Filter;

    GazeTracker() = default;
    explicit GazeTracker(bool use_chen_noise_reduction);

    DefaultGazeEstimationResult
    estimate(const PupilCenterGlintInputs& input,
             const EyeAndCameraParameters& params) const override;

    void setCorneaCenterFilter(Vec3Filter filter);
    void setPupilCenterFilter(Vec3Filter filter);

private:
    bool use_chen_noise_reduction = false;
    Vec3Filter cornea_center_filter;
    Vec3Filter pupil_center_filter;
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
    const EyeAndCameraParameters& eye_cam_params
);


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

Vec3
geometricMedian3(const std::list<Vec3>& points);

}
