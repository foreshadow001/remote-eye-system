/* TODO: 
❌ 1. handle two glints case
❌ 2. set the filter for pupil center and cornea center
*/

#include <iostream>
#include <memory>
#include <ceres/ceres.h>

#include "utils/gaze_estimation_types.hpp"
#include "inference/one_camera_spherical.hpp"
#include "cam_model/pinhole_camera_model.hpp"
#include "utils/shared_calculations.hpp"

using namespace gazeestimation;

namespace gazeestimation
{

void
GazeTracker::setCorneaCenterFilter(Vec3Filter filter)
{
    cornea_center_filter = filter;
}

void
GazeTracker::setPupilCenterFilter(Vec3Filter filter)
{
    pupil_center_filter = filter;
}

GazeTracker::GazeTracker(bool use_chen_noise_reduction):
use_chen_noise_reduction(use_chen_noise_reduction) {}

DefaultGazeEstimationResult
GazeTracker::estimate(
    const PupilCenterGlintInputs& glints_pupil_center,
    const EyeAndCameraParameters& eye_cam_params
) const
/*
Estimate gaze based on pupil center and glints.

Args:
    glints_pupil_center: a struct containing pupil center and glints information.
    eye_cam_params: a struct containing eye and camera parameters,
                    including light positions, camera intrinsics and extrinsics.

Returns:
    A struct containing gaze estimation result.

Raises:
    std::exception: if the input data is invalid or the method cannot handle the input.
*/
{
    if (glints_pupil_center.data.size() != 1)
    {
        throw std::exception("This method must have exactly one pair of pupil center/glint.");
    }
    
    if (glints_pupil_center.data[0].glints.size() < 2)
    {
        throw std::exception("This method must have at least two glints.");
    }

    if (eye_cam_params.cameras.size() != 1)
    {
        throw std::exception("this method can only handle a single camera");
    }

    int valid_glints = 0;
    for(const auto& glint : glints_pupil_center.data[0].glints)
    {
        if (isGlintValid(glint))
            valid_glints++;
    }

    if (valid_glints < 2) throw std::exception("not enough valid glints to estimate gaze");

    const PinholeCameraModel camera = eye_cam_params.cameras[0];
    const PupilCenterGlintInput glints_pupil_center_data = glints_pupil_center.data[0];

    Vec3 cornea_center = solveCorneaCenter(
        glints_pupil_center_data.glints,
        eye_cam_params
    );

    if(cornea_center_filter)
    {
        cornea_center = cornea_center_filter(cornea_center);
    }

    Vec3 pupil_image_wcs = camera.ics_to_wcs(glints_pupil_center_data.pupil_center);

    if(pupil_center_filter)
    {
        pupil_image_wcs = pupil_center_filter(pupil_image_wcs);
    }

    const Vec3 optic_axis_unit = calculateOpticalAxisUnit(
        pupil_image_wcs,
        eye_cam_params.cameras[0].position,
        cornea_center,
        eye_cam_params.R,
        eye_cam_params.K,
        eye_cam_params.n1,
        eye_cam_params.n2,
        use_chen_noise_reduction
    );

    const Vec3 visual_axis_unit = calculateVisualAxisUnit(
        optic_axis_unit,
        eye_cam_params.alpha,
        eye_cam_params.beta
    );

    DefaultGazeEstimationResult result;
    result.is_valid = true;
    result.cornea_center = cornea_center;
    result.optical_axis_unit = optic_axis_unit;
    result.visual_axis_unit = visual_axis_unit;

    return result;

} // estimate

Vec3
solveCorneaCenter(
    std::vector<Vec2> glints,
    const EyeAndCameraParameters& eye_cam_params
)
/*
Estimate the center of cornea based on pupil center and glints.

Args:
    glints: a vector of glint positions in ICS.
    eye_cam_params: a struct containing eye and camera parameters,
                    including light positions, camera intrinsics and extrinsics.

Returns:
    The center of cornea in WCS.
*/
{
    std::vector<Vec3> glints_image_wcs;
    std::vector<Vec3> selected_lights;

    // filter out invalid glints
    for(int i = 0; i < glints.size(); i++)
    {
        if (!isGlintValid(glints[i])) continue;
        glints_image_wcs.push_back(eye_cam_params.cameras[0].ics_to_wcs(glints[i]));
        selected_lights.push_back(eye_cam_params.light_positions[i]);
    }

    Vec3 cornea_center = calculateCorneaCenterWCS(
        &glints_image_wcs,
        &selected_lights,
        eye_cam_params.cameras[0].position,
        eye_cam_params.R,
        eye_cam_params.eye_cam_dist_init
    );

    return cornea_center;
}

Vec3
calculateCorneaCenterWCS(
    const std::vector<Vec3>* const glints,
    const std::vector<Vec3>* const lights,
    const Vec3& cam_pos,
    double R,
    double eye_cam_dist_init
)
/*
Calculate the center of cornea in WCS.

Args:
    glints: a vector of glint positions in WCS.
    lights: a vector of light positions in WCS.
    cam_pos: the camera position in WCS.
    R: the radius of the cornea.
    eye_cam_dist_init: initial guess for the eye-camera distance.

Returns:
    The center of cornea in WCS.
*/
{
    auto cornea_dists = new DistanceBetweenCorneasFunctor(
        glints, lights, R, cam_pos
    );

    auto cost_function = new 
    ceres::DynamicNumericDiffCostFunction<DistanceBetweenCorneasFunctor, ceres::FORWARD>(
        cornea_dists);

    // create a ceres problem
    ceres::Problem problem;
    
    // one parameter (the eye-camera distance) to be optimized for each glint
    for (unsigned int i = 0; i < glints->size(); i++)
    {
        cost_function->AddParameterBlock(1);
    }

    // set the number of residuals
    cost_function->SetNumResiduals(
        3 * 0.5 * (glints->size() * glints->size() - glints->size())
    );

    // set the initial guess for the eye-camera distance
    std::unique_ptr<double[]> reflex_cam_dists(new double[glints->size()]);
    std::vector<double*> variables;

    for (unsigned int i = 0; i < glints->size(); i++) {
        reflex_cam_dists[i] = eye_cam_dist_init;
        variables.push_back(&reflex_cam_dists[i]);
    }

    // add the cost function to the problem
    problem.AddResidualBlock(cost_function, nullptr, variables);

    // set the bounds for the eye-camera distance
    for (unsigned int i = 0; i < glints->size(); i++) {
        problem.SetParameterLowerBound(variables[i], 0, 5);
        problem.SetParameterUpperBound(variables[i], 0, 500);
    }

    // set the options for the optimization
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1e3;
    options.minimizer_progress_to_stdout = false;

    // run the optimization
    ceres::Solver::Summary summary;

    try {
        ceres::Solve(options, &problem, &summary);
    } catch (const std::exception& e) {
        std::cerr << "[CC] EXC during Solve(): " << e.what() << std::endl;
        return make_vec3(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN());
    } catch (...) {
        std::cerr << "[CC] unknown EXC during Solve()" << std::endl;
        return make_vec3(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN());
    }

    Vec3 cornea_total = make_vec3(0, 0, 0);
    for (unsigned int i = 0; i < glints->size(); i++)
    {
        Vec3 reflex = calculateReflex(variables[i][0], cam_pos, (*glints)[i]);
        Vec3 cornea = reflexToCorneaCenter(reflex, (*lights)[i], cam_pos, R);
        cornea_total += cornea;
    }

    Vec3 cornea_avg = cornea_total / static_cast<double>(glints->size());

    // delete cost_function;
    // delete cornea_dists;

    return cornea_avg;
} // calculateCorneaCenterWCS

DistanceBetweenCorneasFunctor::DistanceBetweenCorneasFunctor(
    const std::vector<Vec3>* const glints,
    const std::vector<Vec3>* const lights,
    double r,
    Vec3 cam_pos)
    : glints_(glints), lights_(lights), R_(r), cam_pos_(cam_pos) {}

bool
DistanceBetweenCorneasFunctor::operator()(
    double const* const* variables,
    double* residual
) const
{
    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> cornea_centers;

    for (unsigned int i = 0; i < glints_->size(); i++)
    {
        double reflex_cam_dist = variables[i][0];

        Vec3 reflex = calculateReflex(reflex_cam_dist, cam_pos_, (*glints_)[i]);
        Vec3 cornea_center = reflexToCorneaCenter(
            reflex, (*lights_)[i], cam_pos_, R_
        );

        cornea_centers.push_back(cornea_center);
    }

    size_t index = 0;
    for(int i = 0; i < glints_->size(); i++)
    {
        for(int j = 0; j < i; j++)
        {
            Vec3 diff = cornea_centers[i] - cornea_centers[j];
            residual[index++] = diff[0];
            residual[index++] = diff[1];
            residual[index++] = diff[2];
        }
    }

    return true;
}

Vec3MedianFilter::Vec3MedianFilter(unsigned int size):
data(), size(size) {}

Vec3
Vec3MedianFilter::newSample(gazeestimation::Vec3 new_sample)
{
    // 参数设定
    const double jump_threshold = 5.0;  // 大位移阈值，可根据 glint 像素大小调整
    const double epsilon = 1e-6;        // 数值安全防护

    // ---- 1️⃣ 空输入保护 ----
    if (data.empty()) {
        data.push_back(new_sample);
        return new_sample;
    }

    // ---- 2️⃣ 检测跳变 ----
    const Vec3 last_output = geometricMedian3(data);
    double dist = length_vec3(new_sample - last_output);

    if (dist > jump_threshold) {
        // 大位移：重置窗口
        data.clear();
        data.push_back(new_sample);
        return new_sample;
    }

    // ---- 3️⃣ 正常更新 ----
    data.push_back(new_sample);
    while (data.size() > size) {
        data.pop_front();
    }

    // ---- 4️⃣ 样本太少时直接返回 ----
    if (data.size() < 3)
        return new_sample;

    // ---- 5️⃣ 计算几何中值 ----
    Vec3 median = geometricMedian3(data);

    // ---- 6️⃣ 稳定性增强：少量指数平滑 ----
    const double alpha = 0.2; // 响应率，可调节
    Vec3 smoothed = alpha * median + (1.0 - alpha) * last_output;

    return smoothed;
}

// Geometric Median through Weiszfeld's algorithm
Vec3
geometricMedian3(const std::list<Vec3>& points)
{
	std::unique_ptr<double> weights(new double[points.size()]);
	for (int i = 0; i < points.size(); i++)
		weights.get()[i] = 1;

	const int max_iterations = 150;
	int iteration_index = 0;

	Vec3 last_result(-2, -2, 0);

	Vec3 result = (*points.begin())+Vec3(1,1,1);

	const double epsilon = 0.0005;

	while (squared_length(Vec3(last_result - result)) > epsilon * epsilon)
	{
		last_result = result;
		result = Vec3(0, 0, 0);

		double weights_sum = 0;

		int i = 0;
		for(const auto& point : points)
		{
			const Vec3 diff = last_result - point;


			if (length_vec3(diff) < 0.000001)
			{
				weights.get()[i] = 0;
			}
			else
			{
				weights.get()[i] = 1 / length_vec3(diff);
			}
			weights_sum += weights.get()[i];
			result += weights.get()[i] * point;
			
			i++;
		}

		result /= weights_sum;

		iteration_index++;

		if (iteration_index > max_iterations) break;

    }

	return result;
}

void Vec3MedianFilter::reset()
{
	data.clear();
}

} // namespace gazeestimation