#include <windows.h>
#include <string>
#include <regex>
#include <iostream>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>

#include "utils/gaze_estimation_types.hpp"
#include "utils/math_types.hpp"
#include "glint_detection/detect_glint.h"
#include "pupil_center/localize_pupil.h"
#include "inference/one_camera_spherical.hpp"
#include "utils/shared_calculations.hpp"
#include "calib/calibration.hpp"

using namespace gazeestimation;
namespace fs = std::filesystem;

using OurCalibrationType = GenericCalibration<
    EyeAndCameraParameters,
    PupilCenterGlintInputs,
    DefaultGazeEstimationResult>;


// 在文件顶部加上：#include <sstream>

cv::Mat visualizeGlintsAndPupil(
    const cv::Mat& frame,
    const std::vector<cv::Point2d>& glints,   // 期望 size == 3
    const cv::Point2d& pupil_center,
	const float radius)
{
    cv::Mat vis;
    if (frame.channels() == 1) cv::cvtColor(frame, vis, cv::COLOR_GRAY2BGR);
    else vis = frame.clone();

    // 如果 glints 数量为 3，就连成三角形并标出
    if (glints.size() == 3) {
        std::vector<cv::Point> pts;
        pts.reserve(3);
        for (const auto& g : glints)
            pts.emplace_back(cv::Point(cvRound(g.x), cvRound(g.y)));

        // 画三角线
        const cv::Scalar triColor(0, 255, 0); // 绿色
        cv::polylines(vis, pts, true, triColor, 1, cv::LINE_AA);

    } else {
        // 如果没有 3 个 glint，也尝试把已有的点画出来
        for (const auto& g : glints)
            cv::circle(vis, cv::Point(cvRound(g.x), cvRound(g.y)), 3, cv::Scalar(0,200,200), cv::FILLED, cv::LINE_AA);
    }

    // 在瞳孔中心画十字
    const int crossLen = 6;
    const cv::Point center(cvRound(pupil_center.x), cvRound(pupil_center.y));
    const cv::Scalar crossColor(0, 0, 255); // 红色
    cv::line(vis, center + cv::Point(-crossLen, 0), center + cv::Point(crossLen, 0), crossColor, 1, cv::LINE_AA);
    cv::line(vis, center + cv::Point(0, -crossLen), center + cv::Point(0, crossLen), crossColor, 1, cv::LINE_AA);
    // 可选：画一个半透明圆环表示不确定区域（这里用实线）
    cv::circle(vis, center, cvRound(radius), crossColor, 1, cv::LINE_AA);

    // 可选：输出坐标文本（你可以注释掉）
    std::ostringstream oss;
    oss << "(" << center.x << "," << center.y << ")" << " " << cvRound(radius);
    cv::putText(vis, oss.str(), center + cv::Point(8, -18),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1, cv::LINE_AA);

    return vis;
}


// ================== 辅助函数 ==================

EyeAndCameraParameters six_variable_calibration_applicator(
    EyeAndCameraParameters params, double const* const* variables)
{
    params.alpha = variables[0][0];
    params.beta = variables[1][0];
    params.R = variables[2][0];
    params.K = variables[3][0];
    params.cameras[0].set_camera_angle_y(variables[4][0]);
    params.cameras[0].set_camera_angle_z(variables[5][0]);
    return params;
}

Vec3 result_processor(const DefaultGazeEstimationResult& result,
                      double z_shift, const Vec3& wcs_offset)
{
    return calculatePointOfInterest(result.cornea_center,
                                    result.visual_axis_unit,
                                    z_shift) - wcs_offset;
}

const double* const* const vecvec_to_pointer_pointer(
    std::vector<std::vector<double>>& a)
{
    std::vector<double*> tmp;
    for (unsigned int i = 0; i < a.size(); i++)
        tmp.push_back(&a[i][0]);
    return &tmp[0];
}


// ================== 主函数 ==================

int main() {
    std::string folder_path = "D:/users/projects/new_dataset/calib_records";

    // === 通用设置 ===
    GazeTracker left_gazetracker = gazeestimation::GazeTracker();
	Vec3MedianFilter filter_le_pupil(5), filter_le_coc(5);
	// left_gazetracker.setCorneaCenterFilter(std::bind(&Vec3MedianFilter::newSample, &filter_le_coc, std::placeholders::_1));
	// left_gazetracker.setPupilCenterFilter(std::bind(&Vec3MedianFilter::newSample, &filter_le_pupil, std::placeholders::_1));

    const Vec3 actual_camera_position = make_vec3(29.0, -31, 17);
    const Vec3 wcs_offset = -actual_camera_position;

    PinholeCameraModel camera;
    camera.principal_point_x = 400;
    camera.principal_point_y = 300;
    camera.pixel_size_cm_x = 4.8 * 1e-4;
    camera.pixel_size_cm_y = 4.8 * 1e-4;
    camera.effective_focal_length_cm = 1.2;//0.0119144;
    camera.position = actual_camera_position + wcs_offset;
    camera.set_camera_angles(-deg_to_rad(31.5), 0, 0);

    std::vector<Vec3> lights = {
        actual_camera_position + make_vec3(21, 3, 0) + wcs_offset,
        actual_camera_position + make_vec3(-25, 3, 1) + wcs_offset,
        actual_camera_position + make_vec3(0, -8, 6) + wcs_offset
    };

    EyeAndCameraParameters left_parameters;
    left_parameters.alpha = deg_to_rad(-5);
    left_parameters.beta = deg_to_rad(-1.5);
    left_parameters.R = 0.78;
    left_parameters.K = 0.42;
    left_parameters.n1 = 1.3375;
    left_parameters.n2 = 1;
    left_parameters.D = 0.53;
    left_parameters.eye_cam_dist_init = 40;
    left_parameters.cameras.push_back(camera);
    left_parameters.light_positions = lights;

    // 屏幕信息
    const double display_surface_size_cm_x = 59.5;
    const double display_surface_size_cm_y = 33.6;
    const double screen_resolution_x = 2560;
    const double screen_resolution_y = 1440;
    const double screen_pixel_size_x = display_surface_size_cm_x / screen_resolution_x;
    const double screen_pixel_size_y = display_surface_size_cm_y / screen_resolution_y;
    const double z_shift = -actual_camera_position[2];

    // === 收集所有视频的标定样本 ===
    std::vector<std::pair<PupilCenterGlintInputs, Vec3>> calibrate_against_left;

    std::regex re(R"((?:\\|/)?([^\\/_]+)_(\d+)_(\d+)_([\d]+)\.avi$)");

    for (const auto& entry : fs::directory_iterator(folder_path)) {
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

            Vec3 target = make_vec3(true_pog[0] * screen_pixel_size_x,
                                    -true_pog[1] * screen_pixel_size_y, 0);
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
        {left_parameters.R}, {left_parameters.K},
        {left_parameters.cameras[0].camera_angle_y()},
        {left_parameters.cameras[0].camera_angle_z()}
    };

    std::vector<std::vector<std::pair<double, double>>> bounds = {
        { {deg_to_rad(-6), deg_to_rad(6)} },
        { {deg_to_rad(-5),  deg_to_rad(5)} },
        { {0.4, 1.2} },
        { {0.2, 0.8} },
        { {deg_to_rad(-3),  deg_to_rad(3)} },
        { {deg_to_rad(-3),  deg_to_rad(3)} }
    };

    OurCalibrationType calibration;
    auto calibration_result_left = calibration.calibrate(
        left_gazetracker, left_parameters,
        six_variable_calibration_applicator,
        std::bind(result_processor, std::placeholders::_1, z_shift, wcs_offset),
        calibrate_against_left,
        initial_values_left,
        bounds
    );

    left_parameters = six_variable_calibration_applicator(
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
    std::cout << "CamAy: "
              << rad_to_deg(left_parameters.cameras[0].camera_angle_y()) << " deg" << std::endl;
    std::cout << "CamAz: "
              << rad_to_deg(left_parameters.cameras[0].camera_angle_z()) << " deg" << std::endl;

    std::cout << "\nCalibration finished successfully.\n";

    // 如果前面已经定义过 z_shift，就千万别再定义一次
    // const double z_shift = -actual_camera_position[2];   // <-- 删掉或注释掉

    std::ofstream fout("D:/users/projects/new_dataset/calib_inference_result.txt");
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

            Vec3 pred_wcs = result_processor(res, z_shift, wcs_offset);
			res.cornea_center += actual_camera_position;

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
