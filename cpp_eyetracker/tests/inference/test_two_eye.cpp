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
#include "utils/utils.hpp"
#include "utils/intersection.hpp"

using namespace gazeestimation;
using namespace visualization;
namespace fs = std::filesystem;

using OurCalibrationType = GenericCalibration<
    EyeAndCameraParameters,
    PupilCenterGlintInputs,
    DefaultGazeEstimationResult>;

int main() {
    std::cout << "Loading config file..." << std::endl;
    Cfg cfg;
    GazeTracker gazetracker;
    EyeAndCameraParameters parameters;

    std::vector<std::pair<PupilCenterGlintInputs, Vec3>> calibrate_against;

    std::regex re(R"((?:\\|/)?([^\\/_]+)_(\d+)_(\d+)_([\d]+)\.avi$)");

    int counter = 0;

    for (const auto& entry : fs::directory_iterator(cfg["input_dir"].as<std::string>())) {
        if (entry.path().extension() != ".avi") continue;

        if (counter > 10) break;

        std::string video_path = entry.path().string();

        std::smatch match;
        if (!std::regex_search(video_path, match, re)) {
            std::cerr << "[WARN] Filename format invalid: " << video_path << std::endl;
            continue;
        }

        double pog_x = std::stod(match[3]);
        double pog_y = std::stod(match[4]);
        Vec2 true_pog = make_vec2(pog_x, pog_y);
        // Vec3 target = PoGToWCS(true_pog, cfg);
        Vec3 target = screenToWCS(true_pog, cfg);
        std::cout << "target WCS: " << target.transpose() << std::endl;

        cv::VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] Cannot open video: " << video_path << std::endl;
            continue;
        }

        int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        int start_frame = total_frames * 0.20;
        int end_frame = total_frames * 0.80;

        std::cout << "Extracting from " << video_path << " ..." << std::endl;

        for (int i = start_frame; i < start_frame + 1; i++) {
            cap.set(cv::CAP_PROP_POS_FRAMES, i);
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) continue;

            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            auto [leftEyeGlints, rightEyeGlints, img_debug] =
                glintdetection::searchForGlints(frame, 50.0);
            if (leftEyeGlints.size() != 3) continue;
            if (rightEyeGlints.size() != 3) continue;

            auto [leftPupilCenter, leftEyeImage, leftRadius] =
                pupilcenter::localizePupilCenter(gray, leftEyeGlints);
            auto [rightPupilCenter, rightEyeImage, rightRadius] =
                pupilcenter::localizePupilCenter(gray, rightEyeGlints);

            PupilCenterGlintInputs inputs;
            PupilCenterGlintInput input;
            for (const auto& g : leftEyeGlints)
                input.left.glints.push_back(make_vec2(g.x, g.y));
            input.left.pupil_center = make_vec2(leftPupilCenter.x, leftPupilCenter.y);

            for (const auto& g : rightEyeGlints)
                input.right.glints.push_back(make_vec2(g.x, g.y));
            input.right.pupil_center = make_vec2(rightPupilCenter.x, rightPupilCenter.y);

            inputs.data.push_back(input);
            calibrate_against.push_back(std::make_pair(inputs, target));

			// --- 将检测结果转换为 OpenCV 点 ----
			std::vector<cv::Point2d> gl_pts;
			gl_pts.reserve(leftEyeGlints.size() + rightEyeGlints.size());
			for (const auto& g : leftEyeGlints) {
				gl_pts.emplace_back(g.x, g.y);
			}
            for (const auto& g : rightEyeGlints) {
                gl_pts.emplace_back(g.x, g.y);
			}
			cv::Point2d pupil_left(leftPupilCenter.x, leftPupilCenter.y);
            cv::Point2d pupil_right(rightPupilCenter.x, rightPupilCenter.y);

			// --- 生成可视化图像 ---
			cv::Mat vis = visualizeGlintsAndPupil(frame, gl_pts, pupil_left, leftRadius);
            vis = visualizeGlintsAndPupil(vis, gl_pts, pupil_right, rightRadius);

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
        counter++;
    }

    std::cout << "\nTotal samples collected: "
              << calibrate_against.size() << std::endl;


    std::ofstream fout("D:/ylx/inference_result_left.txt");
    if (!fout) {
        std::cerr << "[ERROR] Cannot open file: calib_inference_result_left.txt\n";
        return -1;
    }
    // 表头
    fout << "# cornea_x cornea_y cornea_z "
         << "opt_x opt_y opt_z "
         << "vis_x vis_y vis_z "
         << "gt_x gt_y gt_z "
         << "pred_x pred_y pred_z\n";

    for (const auto& [inputs, gt_wcs] : calibrate_against)
    {
        try {
            DefaultGazeEstimationResult res = gazetracker.estimate(inputs, parameters);

            Vec3 pred_wcs = res.gaze_point;

            // 输出一行，保留 6 位小数
            fout << std::fixed << std::setprecision(6)
                 << res.left.cornea_center[0] << ' ' << res.left.cornea_center[1] << ' ' << res.left.cornea_center[2] << ' '
                 << res.left.optical_axis_unit[0] << ' ' << res.left.optical_axis_unit[1] << ' ' << res.left.optical_axis_unit[2] << ' '
                 << res.left.visual_axis_unit[0] << ' ' << res.left.visual_axis_unit[1] << ' ' << res.left.visual_axis_unit[2] << ' '
                 << gt_wcs[0] << ' ' << gt_wcs[1] << ' ' << gt_wcs[2] << ' '
                 << pred_wcs[0] << ' ' << pred_wcs[1] << ' ' << pred_wcs[2] << '\n';
        }
        catch (const std::exception& e) {
            std::cerr << "[WARN] failed to estimate gaze for input: " << e.what() << '\n';
            continue;
        }
    }
    fout.close();
    std::cout << "\ninference result saved to calib_inference_result_left.txt\n";

    std::ofstream fout_right("D:/ylx/inference_result_right.txt");
    if (!fout_right) {
        std::cerr << "[ERROR] Cannot open file: inference_result_right.txt\n";
        return -1;
    }
    
    fout_right << "# cornea_x cornea_y cornea_z "
              << "opt_x opt_y opt_z "
              << "vis_x vis_y vis_z "
              << "gt_x gt_y gt_z "
              << "pred_x pred_y pred_z\n";

    for (const auto& [inputs, gt_wcs] : calibrate_against)
    {
        try {
            DefaultGazeEstimationResult res = gazetracker.estimate(inputs, parameters);
            Vec3 pred_wcs = res.gaze_point;

            fout_right << std::fixed << std::setprecision(6)
                       << res.right.cornea_center[0] << ' ' << res.right.cornea_center[1] << ' ' << res.right.cornea_center[2] << ' '
                       << res.right.optical_axis_unit[0] << ' ' << res.right.optical_axis_unit[1] << ' ' << res.right.optical_axis_unit[2] << ' '
                       << res.right.visual_axis_unit[0] << ' ' << res.right.visual_axis_unit[1] << ' ' << res.right.visual_axis_unit[2] << ' '
                       << gt_wcs[0] << ' ' << gt_wcs[1] << ' ' << gt_wcs[2] << ' '
                       << pred_wcs[0] << ' ' << pred_wcs[1] << ' ' << pred_wcs[2] << '\n';
        }
        catch (const std::exception& e) {
            std::cerr << "[WARN] failed to estimate gaze for input: " << e.what() << '\n';
            continue;
        }
    }
    fout_right.close();
    std::cout << "\ninference result saved to inference_result_right.txt\n";

    return 0;
}
