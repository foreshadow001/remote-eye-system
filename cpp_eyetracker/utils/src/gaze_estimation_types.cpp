// utils/src/gaze_estimation_types.cpp

#include "utils/gaze_estimation_types.hpp"
#include <iostream>

namespace gazeestimation
{

DefaultGazeEstimationResult::DefaultGazeEstimationResult():
    left{Vec3(0,0,0), Vec3(0,0,0), Vec3(0,0,0)},
    right{Vec3(0,0,0), Vec3(0,0,0), Vec3(0,0,0)},
    is_valid(false),
    is_error(false),
    gaze_point(0,0,0),
    error("") {}

DefaultGazeEstimationResult DefaultGazeEstimationResult::make_error(std::string error)
{
	DefaultGazeEstimationResult res;
	res.is_valid = false;
	res.is_error = true;
	res.error = error;
	return res;
}

EyeAndCameraParameters::EyeAndCameraParameters()
{
    Cfg cfg;

    // 加载校准初始值节点
    auto calib = cfg["calib_init_values"];

    // ────────────────────────────────
    // 左眼参数
    // ────────────────────────────────
    left.alpha = deg_to_rad(calib["left"]["alpha"].as<double>());
    left.beta  = deg_to_rad(calib["left"]["beta"].as<double>());
    left.R     = calib["left"]["R"].as<double>();
    left.K     = calib["left"]["K"].as<double>();
    left.n1    = calib["left"]["n1"].as<double>();
    left.n2    = calib["left"]["n2"].as<double>();
    left.D     = calib["left"]["D"].as<double>();

    // ────────────────────────────────
    // 右眼参数
    // ────────────────────────────────
    right.alpha = deg_to_rad(calib["right"]["alpha"].as<double>());
    right.beta  = deg_to_rad(calib["right"]["beta"].as<double>());
    right.R     = calib["right"]["R"].as<double>();
    right.K     = calib["right"]["K"].as<double>();
    right.n1    = calib["right"]["n1"].as<double>();
    right.n2    = calib["right"]["n2"].as<double>();
    right.D     = calib["right"]["D"].as<double>();
    
    // 公共的 eye_cam_dist_init
    eye_cam_dist_init = calib["eye_cam_dist_init"].as<double>();

    // 1. 获取相机 XML 文件路径列表
    std::vector<std::string> xml_paths;
    try {
        xml_paths = cfg["cam_xml_path"].as<std::vector<std::string>>();
    } catch (...) {
        std::cerr << "[Warning] 'cam_xml_path' not found or invalid in config. No cameras loaded." << std::endl;
    }

    // 2. 预分配内存以提高性能
    if (!xml_paths.empty()) {
        cameras.reserve(xml_paths.size());
    }

    // 3. 循环初始化相机
    for (size_t i = 0; i < xml_paths.size(); ++i) {
        cameras.emplace_back(static_cast<int>(i));
        
        std::cout << "[Info] Loaded camera index: " << i << std::endl;
    }

	light_positions = cfg["lights_pos"].as<std::vector<Vec3>>();
    
}

} // namespace gazeestimation
