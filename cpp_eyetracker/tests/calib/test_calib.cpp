#include <windows.h>
#include <string>
#include <regex>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>

#include "cfg/config.hpp"
#include "utils/gaze_estimation_types.hpp"
#include "core/math_types.hpp"
#include "utils/visualize.hpp"
#include "utils/intersection.hpp"
#include "glint_detection/detect_glint.h"
#include "pupil_center/localize_pupil.h"
#include "inference/one_camera_spherical.hpp"
#include "utils/shared_calculations.hpp"
#include "calib/calibration.hpp"

using namespace gazeestimation;
using namespace visualization;
namespace fs = std::filesystem;

using OurCalibrationType = GenericCalibration<
    EyeAndCameraParameters,
    PupilCenterGlintInputs,
    DefaultGazeEstimationResult>;

int main() {
    std::cout << "Loading config file..." << std::endl;
    Cfg cfg; // load config file
    GazeTracker left_gazetracker = gazeestimation::GazeTracker(); // create left gazetracker

    // set filters for pupil center and cornea center
	// Vec3MedianFilter filter_le_pupil(5), filter_le_coc(5);
	// left_gazetracker.setCorneaCenterFilter(std::bind(&Vec3MedianFilter::newSample, &filter_le_coc, std::placeholders::_1));
	// left_gazetracker.setPupilCenterFilter(std::bind(&Vec3MedianFilter::newSample, &filter_le_pupil, std::placeholders::_1));

    PinholeCameraModel camera;
    camera.principal_point_x = 400;
    camera.principal_point_y = 300;
    camera.pixel_size_cm_x = 4.8 * 1e-4;
    camera.pixel_size_cm_y = 4.8 * 1e-4;
    camera.effective_focal_length_cm = 1.2;
    camera.position = make_vec3(0, 0, 0);
    camera.set_camera_angles(-deg_to_rad(31.5), 0, 0);

    EyeAndCameraParameters left_parameters("left");
    left_parameters.cameras.push_back(camera);

    std::vector<std::pair<PupilCenterGlintInputs, Vec3>> calibrate_against_left;

    std::regex re(R"((?:\\|/)?([^\\/_]+)_(\d+)_(\d+)_([\d]+)\.avi$)");

    for (const auto& entry : fs::directory_iterator(cfg["input_dir"].as<std::string>())) {
        if (entry.path().extension() != ".avi") continue;
        std::string video_path = entry.path().string();

        std::smatch match;
        if (!std::regex_search(video_path, match, re)) {
            std::cerr << "[WARN] Filename format invalid: " << video_path << std::endl;
            continue;
        }

        double pog_x = std::stod(match[3]);
        double pog_y = std::stod(match[4]);
        Vec2 true_pog = make_vec2(pog_x, pog_y);

        cv::VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] Cannot open video: " << video_path << std::endl;
            continue;
        }

        int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        int start_frame = total_frames * 0.20;
        int end_frame = total_frames * 0.80;

        std::cout << "Extracting from " << video_path << " ..." << std::endl;

        for (int i = start_frame; i < start_frame + 50; i++) {
            cap.set(cv::CAP_PROP_POS_FRAMES, i);
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) continue;

            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            auto [leftEyeGlints, rightEyeGlints, processed_img_left, processed_img_right] =
                glintdetection::searchForGlints(frame, 50.0);
            if (leftEyeGlints.size() != 3) continue;

            auto [leftPupilCenter, leftEyeImage, leftRadius] =
                pupilcenter::localizePupilCenter(gray, leftEyeGlints);

            PupilCenterGlintInputs inputs_left;
            PupilCenterGlintInput input_left;
            for (const auto& g : leftEyeGlints)
                input_left.glints.push_back(make_vec2(g.x, g.y));
            input_left.pupil_center = make_vec2(leftPupilCenter.x, leftPupilCenter.y);
            inputs_left.data.push_back(input_left);

            Vec3 target = PoGToWCS(true_pog, cfg);
            calibrate_against_left.push_back(std::make_pair(inputs_left, target));

			// --- 将检测结果转换为 OpenCV 点 ----
			std::vector<cv::Point2d> gl_pts;
			gl_pts.reserve(leftEyeGlints.size());
			for (const auto& g : leftEyeGlints) {
				gl_pts.emplace_back(g.x, g.y);
			}
			cv::Point2d pupil_pt(leftPupilCenter.x, leftPupilCenter.y);

			// --- 生成可视化图像 ---
			cv::Mat vis = visualizeGlintsAndPupil(frame, gl_pts, pupil_pt, leftRadius);

			// --- 准备输出目录：父目录 / "<video_stem>_viz" ---
			fs::path video_path_obj = entry.path(); // 你在外面有 entry
			fs::path parent_dir = video_path_obj.parent_path();
			std::string stem = video_path_obj.stem().string();
			fs::path out_dir = parent_dir / (stem + "_viz");
			if (!fs::exists(out_dir)) {
				try {
					fs::create_directories(out_dir);
				} catch (const std::exception& e) {
					std::cerr << "[WARN] 无法创建输出目录: " << out_dir << " , " << e.what() << std::endl;
				}
			}

			// --- 保存图片，命名为 <video_stem>_frame_<frameIdx>.png ---
			std::ostringstream fname;
			fname << out_dir.string() << "/" << stem << "_frame_" << i << ".png";
			if (!cv::imwrite(fname.str(), vis)) {
				std::cerr << "[WARN] 保存可视化图片失败: " << fname.str() << std::endl;
			}

        }
    }

    std::cout << "\nTotal samples collected: "
              << calibrate_against_left.size() << std::endl;

    if (calibrate_against_left.empty()) {
        std::cerr << "[ERROR] No valid frames extracted. Abort." << std::endl;
        return -1;
    }

    // === 一次全局标定 ===
    std::cout << "\nStarting global calibration..." << std::endl;

    std::vector<std::vector<double>> initial_values_left = {
        {left_parameters.alpha}, {left_parameters.beta},
        {left_parameters.R}, {left_parameters.K}
        // {left_parameters.cameras[0].camera_angle_y()},
        // {left_parameters.cameras[0].camera_angle_z()}
    };

    std::vector<std::vector<std::pair<double, double>>> bounds = {
        { {deg_to_rad(cfg["calib_values_bounds"]["alpha"].asVector<double>()[0]), deg_to_rad(cfg["calib_values_bounds"]["alpha"].asVector<double>()[1])} },
        { {deg_to_rad(cfg["calib_values_bounds"]["beta"].asVector<double>()[0]),  deg_to_rad(cfg["calib_values_bounds"]["beta"].asVector<double>()[1])} },
        { {cfg["calib_values_bounds"]["R"].asVector<double>()[0],  cfg["calib_values_bounds"]["R"].asVector<double>()[1]} },
        { {cfg["calib_values_bounds"]["K"].asVector<double>()[0],  cfg["calib_values_bounds"]["K"].asVector<double>()[1]} }
        // { {deg_to_rad(-3),  deg_to_rad(3)} },
        // { {deg_to_rad(-3),  deg_to_rad(3)} }
    };

    OurCalibrationType calibration;
    auto calibration_result_left = calibration.calibrate(
        left_gazetracker, left_parameters,
        variables_calibration_applicator,
        std::bind(result_processor, std::placeholders::_1, cfg["cam_pos"].as<Vec3>()),
        calibrate_against_left,
        initial_values_left,
        bounds
    );

    left_parameters = variables_calibration_applicator(
        left_parameters,
        vecvec_to_pointer_pointer(calibration_result_left)
    );

    std::cout << "\n=== Global Calibration Result ===\n";
    std::cout << "Alpha: "
              << rad_to_deg(left_parameters.alpha) << " deg" << std::endl;
    std::cout << "Beta: "
              << rad_to_deg(left_parameters.beta) << " deg" << std::endl;
    std::cout << "R: " << left_parameters.R << std::endl;
    std::cout << "K: " << left_parameters.K << std::endl;
    std::cout << "\nCalibration finished successfully.\n";

    std::ofstream fout("D:/ylx/calib_inference_result.txt");
    if (!fout) {
        std::cerr << "[ERROR] Cannot open file: calib_inference_result.txt\n";
        return -1;
    }
    // 表头
    fout << "# cornea_x cornea_y cornea_z "
         << "opt_x opt_y opt_z "
         << "vis_x vis_y vis_z "
         << "gt_x gt_y gt_z "
         << "pred_x pred_y pred_z\n";

    for (const auto& [inputs, gt_wcs] : calibrate_against_left)
    {
        try {
            DefaultGazeEstimationResult res =
                left_gazetracker.estimate(inputs, left_parameters);

            Vec3 pred_wcs = result_processor(res, cfg["cam_pos"].as<Vec3>());
			res.cornea_center += cfg["cam_pos"].as<Vec3>();

            // 输出一行，保留 6 位小数
            fout << std::fixed << std::setprecision(6)
                 << res.cornea_center[0] << ' ' << res.cornea_center[1] << ' ' << res.cornea_center[2] << ' '
                 << res.optical_axis_unit[0] << ' ' << res.optical_axis_unit[1] << ' ' << res.optical_axis_unit[2] << ' '
                 << res.visual_axis_unit[0] << ' ' << res.visual_axis_unit[1] << ' ' << res.visual_axis_unit[2] << ' '
                 << gt_wcs[0] << ' ' << gt_wcs[1] << ' ' << gt_wcs[2] << ' '
                 << pred_wcs[0] << ' ' << pred_wcs[1] << ' ' << pred_wcs[2] << '\n';
        }
        catch (const std::exception& e) {
            std::cerr << "[WARN] failed to estimate gaze for input: " << e.what() << '\n';
            continue;
        }
    }
    fout.close();
    std::cout << "\ninference result saved to calib_inference_result.txt\n";

    return 0;
}
