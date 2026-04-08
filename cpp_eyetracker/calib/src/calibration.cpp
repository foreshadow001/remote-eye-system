// calib/calibration.cpp

#include "calib/calibration.hpp"
#include "utils/shared_calculations.hpp"

#include <iostream>
#include <cassert>
#include <ceres/ceres.h>

namespace gazeestimation
{

// 私有仿函数类，用于提供给 Ceres 自动求导计算残差
class CalibrationErrorFunctor
{
private:
    GazeTracker* const gaze_estimation;
    const Calibration::CalibrationDataMap* const data;
    const EyeAndCameraParameters parameters;

public:
    CalibrationErrorFunctor(
        GazeTracker* const gaze_estimation,
        const Calibration::CalibrationDataMap* const data,
        const EyeAndCameraParameters parameters
    ) :
        gaze_estimation(gaze_estimation),
        data(data),
        parameters(parameters)
    {
    }

    bool operator()(double const* const* variables, double* residual) const {
        auto our_parameters = variables_calibration_applicator(parameters, variables);
        int index = 0;
        
        for (auto It = data->begin(); It != data->end(); ++It)
        {
            PupilCenterGlintInputs data_in = (*It).first;
            Vec3 truth = (*It).second;

            DefaultGazeEstimationResult result = gaze_estimation->estimate(data_in, our_parameters);
            const Vec3 estimate = result.gaze_point;
            const Vec3 diff = truth - estimate;
                            
            residual[index++] = diff[0];
            residual[index++] = diff[1];
            residual[index++] = diff[2];
        }

        return true;
    }
};

// 私有仿函数类，用于提供给 Ceres 自动求导计算残差
class SingleEyeCalibrationErrorFunctor
{
private:
    GazeTracker* const gaze_estimation;
    const Calibration::SingleEyeCalibrationDataMap* const data;
    const SingleEyeAndCameraParameters parameters;

public:
    SingleEyeCalibrationErrorFunctor(
        GazeTracker* const gaze_estimation,
        const Calibration::SingleEyeCalibrationDataMap* const data,
        const SingleEyeAndCameraParameters parameters
    ) :
        gaze_estimation(gaze_estimation),
        data(data),
        parameters(parameters)
    {
    }

    bool operator()(double const* const* variables, double* residual) const {
        auto our_parameters = variables_calibration_applicator(parameters, variables);
        int index = 0;
        
        for (auto It = data->begin(); It != data->end(); ++It)
        {
            SingleEyePupilCenterGlintInputs data_in = (*It).first;
            Vec3 truth = (*It).second;

            try {
                DefaultSingleEyeGazeEstimationResult result = gaze_estimation->estimate(data_in, our_parameters);
                
                Vec3 C = result.cornea_center;     // 视线起点 (角膜中心)
                Vec3 V = result.visual_axis_unit;  // 视线方向 (单位向量)
                
                // 真值点到视线起点的向量
                Vec3 diff = truth - C;
                // diff在视线方向上的投影长度
                double dot = diff[0] * V[0] + diff[1] * V[1] + diff[2] * V[2];
                
                // 垂足向量 (误差向量)
                residual[index++] = diff[0] - dot * V[0];
                residual[index++] = diff[1] - dot * V[1];
                residual[index++] = diff[2] - dot * V[2];
            } catch (...) {
                // 如果参数探索导致推断过程异常(比如计算出NaN)，返回false让Ceres重试
                return false;
            }
        }

        return true;
    }
};

std::vector<std::vector<double>> Calibration::calibrate(
    GazeTracker& estimation,
    EyeAndCameraParameters& parameters,
    CalibrationDataMap& data,
    std::vector<std::vector<double>> initial_values,
    std::vector<std::vector<std::pair<double, double>>> bounds)
{
    assert(bounds.size() == initial_values.size());

    ceres::Problem problem;

    auto cost_function = new ceres::DynamicNumericDiffCostFunction<CalibrationErrorFunctor, ceres::CENTRAL>
        (new CalibrationErrorFunctor(&estimation, &data, parameters));

    for (size_t i = 0; i < initial_values.size(); i++)
    {
        cost_function->AddParameterBlock(initial_values[i].size());
    }

    cost_function->SetNumResiduals(data.size() * 3);

    std::vector<double*> variables;

    for(size_t i = 0; i < initial_values.size(); i++)
    {
        double* element = new double[initial_values[i].size()];
        for(size_t j = 0; j < initial_values[i].size(); j++)
        {
            element[j] = initial_values[i][j];
        }
        variables.push_back(element);
    }

    problem.AddResidualBlock(cost_function, nullptr, variables);

    for(size_t i = 0; i < initial_values.size(); i++)
    {
        for (size_t j = 0; j < bounds[i].size(); j++) {
            double* element = variables[i];
            problem.SetParameterLowerBound(element, j, bounds[i][j].first);
            problem.SetParameterUpperBound(element, j, bounds[i][j].second);
        }
    }
    
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1e4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::vector<std::vector<double>> result;

    for(size_t i = 0; i < variables.size(); i++)
    {
        std::vector<double> this_variable;
        for(size_t j = 0; j < initial_values[i].size(); j++)
        {
            this_variable.push_back(variables[i][j]);
        }
        result.push_back(this_variable);
        
        delete[] variables[i]; // 防止内存泄漏
    }
    return result;
}

std::vector<std::vector<double>> Calibration::calibrate(
    GazeTracker& estimation,
    SingleEyeAndCameraParameters& parameters,
    SingleEyeCalibrationDataMap& data,
    std::vector<std::vector<double>> initial_values,
    std::vector<std::vector<std::pair<double, double>>> bounds)
{
    assert(bounds.size() == initial_values.size());

    ceres::Problem problem;

    auto cost_function = new ceres::DynamicNumericDiffCostFunction<SingleEyeCalibrationErrorFunctor, ceres::CENTRAL>
        (new SingleEyeCalibrationErrorFunctor(&estimation, &data, parameters));

    for (size_t i = 0; i < initial_values.size(); i++)
    {
        cost_function->AddParameterBlock(initial_values[i].size());
    }

    cost_function->SetNumResiduals(data.size() * 3);

    std::vector<double*> variables;

    for(size_t i = 0; i < initial_values.size(); i++)
    {
        double* element = new double[initial_values[i].size()];
        for(size_t j = 0; j < initial_values[i].size(); j++)
        {
            element[j] = initial_values[i][j];
        }
        variables.push_back(element);
    }

    problem.AddResidualBlock(cost_function, nullptr, variables);

    for(size_t i = 0; i < initial_values.size(); i++)
    {
        for (size_t j = 0; j < bounds[i].size(); j++) {
            double* element = variables[i];
            problem.SetParameterLowerBound(element, j, bounds[i][j].first);
            problem.SetParameterUpperBound(element, j, bounds[i][j].second);
        }
    }
    
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 1e4;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::vector<std::vector<double>> result;

    for(size_t i = 0; i < variables.size(); i++)
    {
        std::vector<double> this_variable;
        for(size_t j = 0; j < initial_values[i].size(); j++)
        {
            this_variable.push_back(variables[i][j]);
        }
        result.push_back(this_variable);
        
        delete[] variables[i]; // 防止内存泄漏
    }
    return result;
}


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

SingleEyeAndCameraParameters
variables_calibration_applicator(
    SingleEyeAndCameraParameters params,
    double const* const* variables
)
{
    params.alpha = variables[0][0];
    params.beta = variables[1][0];
    params.R = variables[2][0];
    params.K = variables[3][0];
    
    return params;
}

const double* const* const
vecvec_to_pointer_pointer(
    std::vector<std::vector<double>>& a
)
{
    std::vector<double*> tmp;
    for (size_t i = 0; i < a.size(); i++)
        tmp.push_back(&a[i][0]);
    
    static std::vector<double*> static_tmp;
    static_tmp = tmp;
    return &static_tmp[0];
}

} // namespace gazeestimation