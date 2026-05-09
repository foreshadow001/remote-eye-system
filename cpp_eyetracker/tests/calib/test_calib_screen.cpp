#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <filesystem>
#include <ceres/ceres.h>

#include "cfg/config.hpp"
#include "utils/gaze_estimation_types.hpp"
#include "core/math_types.hpp"
#include "inference/one_camera_spherical.hpp"
#include "utils/shared_calculations.hpp"
#include "calib/calibration.hpp"
#include "utils/utils.hpp"
#include "utils/intersection.hpp"

using namespace gazeestimation;
namespace fs = std::filesystem;

int main() {
    std::cout << "Loading config file for Single Eye Calibration..." << std::endl;
    Cfg cfg;
    
    // 初始化左右眼的追踪器和参数
    GazeTracker gazetracker_left;
    GazeTracker gazetracker_right;

    Calibration calibration_left;
    Calibration calibration_right;
    
    SingleEyeAndCameraParameters params_left("left");
    SingleEyeAndCameraParameters params_right("right");

    std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>> calib_data_left;
    std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>> calib_data_right;

    std::string record_dir = cfg["test_calib"]["record_dir"].as<std::string>();
    int max_images_per_folder = cfg["test_calib"]["num_images_per_folder"].as<int>();
    std::string output_dir = cfg["test_calib"]["output_dir"].as<std::string>();

    // 1. 自动读取屏幕标定点映射 (video_point_mapping.csv)
    std::map<std::string, Vec2> point_mapping;
    std::string mapping_file = record_dir + "/video_point_mapping.csv";
    std::ifstream map_in(mapping_file);
    if (!map_in.is_open()) {
        std::cerr << "[ERROR] Cannot open mapping file: " << mapping_file << std::endl;
        return -1;
    }

    std::string line;
    while (std::getline(map_in, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string batch_timestr, point_idx_str, x_str, y_str;
        std::getline(ss, batch_timestr, ',');
        std::getline(ss, point_idx_str, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        
        if (!x_str.empty() && !y_str.empty()) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            point_mapping[batch_timestr] = make_vec2(x, y);
        }
    }
    map_in.close();
    std::cout << "[INFO] Loaded " << point_mapping.size() << " screen target mappings." << std::endl;

    // 2. 遍历每个 record_xxx 文件夹，解析 glint_pupil_data.csv
    for (const auto& entry : fs::directory_iterator(record_dir)) {
        if (!entry.is_directory()) continue;
        
        std::string dirname = entry.path().filename().string();
        
        // 排除不符合命名规则的文件夹或 raw 文件夹
        if (dirname.rfind("record_", 0) != 0 || dirname.find("raw") != std::string::npos) {
            continue;
        }

        // 提取 batch_timestr 并查找 target WCS
        std::string timestr = dirname.substr(7);
        if (point_mapping.find(timestr) == point_mapping.end()) {
            std::cerr << "[WARN] No mapping found for folder: " << dirname << std::endl;
            continue;
        }

        Vec2 true_pog = point_mapping[timestr];
        Vec3 target = screenToWCS(true_pog, cfg);
        std::cout << "Folder: " << dirname << " -> target WCS: " << target[0] << ", " << target[1] << ", " << target[2] << std::endl;

        std::string data_csv_path = (entry.path() / "glint_pupil_data.csv").string();
        std::ifstream data_in(data_csv_path);
        if (!data_in.is_open()) {
            std::cerr << "[WARN] Cannot open data file: " << data_csv_path << std::endl;
            continue;
        }

        // 跳过表头
        std::getline(data_in, line);

        int img_count_left = 0;
        int img_count_right = 0;
        
        while (std::getline(data_in, line)) {
            if (line.empty()) continue;

            std::stringstream ss(line);
            std::vector<std::string> tokens;
            std::string token;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() < 17) continue;

            std::string filepath = tokens[0];

            if (filepath.find("cam_0") == std::string::npos) {
                continue;
            }

            // 解析坐标
            double lp_x = std::stod(tokens[1]);
            double lp_y = std::stod(tokens[2]);
            double rp_x = std::stod(tokens[3]);
            double rp_y = std::stod(tokens[4]);

            double lg_l_x = std::stod(tokens[5]);
            double lg_l_y = std::stod(tokens[6]);
            double lg_r_x = std::stod(tokens[7]);
            double lg_r_y = std::stod(tokens[8]);
            double lg_m_x = std::stod(tokens[9]);
            double lg_m_y = std::stod(tokens[10]);

            double rg_l_x = std::stod(tokens[11]);
            double rg_l_y = std::stod(tokens[12]);
            double rg_r_x = std::stod(tokens[13]);
            double rg_r_y = std::stod(tokens[14]);
            double rg_m_x = std::stod(tokens[15]);
            double rg_m_y = std::stod(tokens[16]);

            // 检查检测有效性
            bool left_valid = (lg_l_x != 0 || lg_l_y != 0 || lg_r_x != 0 || lg_r_y != 0);
            bool right_valid = (rg_l_x != 0 || rg_l_y != 0 || rg_r_x != 0 || rg_r_y != 0);

            // 左眼填充
            if (left_valid && img_count_left < max_images_per_folder) {
                SingleEyePupilCenterGlintInputs inputs_l;
                SingleEyePupilCenterGlintInput input_l;
                input_l.pupil_center = make_vec2(lp_x, lp_y);
                input_l.glints.push_back(make_vec2(lg_l_x, lg_l_y));
                input_l.glints.push_back(make_vec2(lg_r_x, lg_r_y));
                input_l.glints.push_back(make_vec2(lg_m_x, lg_m_y));
                inputs_l.data.push_back(input_l);
                calib_data_left.push_back(std::make_pair(inputs_l, target));
                img_count_left++;
            }

            // 右眼填充
            if (right_valid && img_count_right < max_images_per_folder) {
                SingleEyePupilCenterGlintInputs inputs_r;
                SingleEyePupilCenterGlintInput input_r;
                input_r.pupil_center = make_vec2(rp_x, rp_y);
                input_r.glints.push_back(make_vec2(rg_l_x, rg_l_y));
                input_r.glints.push_back(make_vec2(rg_r_x, rg_r_y));
                input_r.glints.push_back(make_vec2(rg_m_x, rg_m_y));
                inputs_r.data.push_back(input_r);
                calib_data_right.push_back(std::make_pair(inputs_r, target));
                img_count_right++;
            }

            // 当双眼都已经提取满所需数量后，退出当前文件夹的数据读取
            if (img_count_left >= max_images_per_folder && img_count_right >= max_images_per_folder) {
                break;
            }
        }
        data_in.close();
        std::cout << "  -> Extracted Left: " << img_count_left << " | Right: " << img_count_right << " valid inputs from cam_0." << std::endl;
    }

    std::cout << "\nTotal samples collected -> Left: " << calib_data_left.size() << " | Right: " << calib_data_right.size() << std::endl;

    // 获取约束边界配置
    auto bounds_config = cfg["calib_values_bounds"];
    std::vector<std::vector<std::pair<double, double>>> bounds = {
        { {deg_to_rad(bounds_config["alpha"].as<std::vector<double>>()[0]), deg_to_rad(bounds_config["alpha"].as<std::vector<double>>()[1])} },
        { {deg_to_rad(bounds_config["beta"].as<std::vector<double>>()[0]),  deg_to_rad(bounds_config["beta"].as<std::vector<double>>()[1])} },
        { {bounds_config["R"].as<std::vector<double>>()[0],  bounds_config["R"].as<std::vector<double>>()[1]} },
        { {bounds_config["K"].as<std::vector<double>>()[0],  bounds_config["K"].as<std::vector<double>>()[1]} }
    };

    // ================== 3. 左眼独立标定 ==================
    if (!calib_data_left.empty()) {
        std::cout << "\nStarting left eye calibration..." << std::endl;
        std::vector<std::vector<double>> init_left = {
            {params_left.alpha}, {params_left.beta}, {params_left.R}, {params_left.K}
        };

        auto calibration_result_left = calibration_left.calibrateScreen(gazetracker_left, params_left, calib_data_left, init_left, bounds);

        params_left = variables_calibration_applicator(
            params_left,
            vecvec_to_pointer_pointer(calibration_result_left)
        );

        std::cout << "\n=== Left Eye Calibration Result ===\n";
        std::cout << "Alpha: " << rad_to_deg(params_left.alpha) << " deg\n";
        std::cout << "Beta: " << rad_to_deg(params_left.beta) << " deg\n";
        std::cout << "R: " << params_left.R << "\n";
        std::cout << "K: " << params_left.K << "\n";

        // 输出左眼推理结果 (单眼没有预测的交叉点，无需 pred_x/y/z)
        std::ofstream fout_left(output_dir + "/calib_inference_result_left_single_screen.txt");
        if (fout_left) {
            fout_left << "# cornea_x cornea_y cornea_z "
                      << "opt_x opt_y opt_z "
                      << "vis_x vis_y vis_z "
                      << "gt_x gt_y gt_z\n";

            for (const auto&[inputs, gt_wcs] : calib_data_left) {
                try {
                    DefaultSingleEyeGazeEstimationResult res = gazetracker_left.estimate(inputs, params_left);

                    fout_left << std::fixed << std::setprecision(6)
                              << res.cornea_center[0] << ' ' << res.cornea_center[1] << ' ' << res.cornea_center[2] << ' '
                              << res.optical_axis_unit[0] << ' ' << res.optical_axis_unit[1] << ' ' << res.optical_axis_unit[2] << ' '
                              << res.visual_axis_unit[0] << ' ' << res.visual_axis_unit[1] << ' ' << res.visual_axis_unit[2] << ' '
                              << gt_wcs[0] << ' ' << gt_wcs[1] << ' ' << gt_wcs[2] << '\n';
                }
                catch (const std::exception& e) {
                    continue;
                }
            }
            fout_left.close();
            std::cout << "\nLeft eye inference result saved to " << output_dir << "/calib_inference_result_left_single_screen.txt\n";
        }
    } else {
        std::cerr << "[WARN] No valid frames extracted for left eye." << std::endl;
    }

    // ================== 4. 右眼独立标定 ==================
    if (!calib_data_right.empty()) {
        std::cout << "\nStarting right eye calibration..." << std::endl;
        std::vector<std::vector<double>> init_right = {
            {params_right.alpha}, {params_right.beta}, {params_right.R}, {params_right.K}
        };

        auto calibration_result_right = calibration_right.calibrateScreen(gazetracker_right, params_right, calib_data_right, init_right, bounds);

        params_right = variables_calibration_applicator(
            params_right,
            vecvec_to_pointer_pointer(calibration_result_right)
        );

        std::cout << "\n=== Right Eye Calibration Result ===\n";
        std::cout << "Alpha: " << rad_to_deg(params_right.alpha) << " deg\n";
        std::cout << "Beta: " << rad_to_deg(params_right.beta) << " deg\n";
        std::cout << "R: " << params_right.R << "\n";
        std::cout << "K: " << params_right.K << "\n";

        // 输出右眼推理结果
        std::ofstream fout_right(output_dir + "/calib_inference_result_right_single_screen.txt");
        if (fout_right) {
            fout_right << "# cornea_x cornea_y cornea_z "
                       << "opt_x opt_y opt_z "
                       << "vis_x vis_y vis_z "
                       << "gt_x gt_y gt_z\n";

            for (const auto&[inputs, gt_wcs] : calib_data_right) {
                try {
                    DefaultSingleEyeGazeEstimationResult res = gazetracker_right.estimate(inputs, params_right);

                    fout_right << std::fixed << std::setprecision(6)
                               << res.cornea_center[0] << ' ' << res.cornea_center[1] << ' ' << res.cornea_center[2] << ' '
                               << res.optical_axis_unit[0] << ' ' << res.optical_axis_unit[1] << ' ' << res.optical_axis_unit[2] << ' '
                               << res.visual_axis_unit[0] << ' ' << res.visual_axis_unit[1] << ' ' << res.visual_axis_unit[2] << ' '
                               << gt_wcs[0] << ' ' << gt_wcs[1] << ' ' << gt_wcs[2] << '\n';
                }
                catch (const std::exception& e) {
                    continue;
                }
            }
            fout_right.close();
            std::cout << "\nRight eye inference result saved to " << output_dir << "/calib_inference_result_right_single_screen.txt\n";
        }
    } else {
        std::cerr << "[WARN] No valid frames extracted for right eye." << std::endl;
    }

    return 0;
}