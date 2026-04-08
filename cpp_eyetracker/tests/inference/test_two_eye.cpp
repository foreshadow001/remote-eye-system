#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <filesystem>

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
    std::cout << "Loading config file..." << std::endl;
    Cfg cfg;
    GazeTracker gazetracker;
    EyeAndCameraParameters parameters;

    std::vector<std::pair<PupilCenterGlintInputs, Vec3>> calibrate_against;

    // 获取数据根目录 (优先采用 test_glint 中的 record_dir)
    std::string record_dir = cfg["test_two_eye"]["record_dir"].as<std::string>();
    std::string output_dir = cfg["test_two_eye"]["output_dir"].as<std::string>();

    // 获取每个文件夹采样的图片数量，默认 10 张
    int max_images_per_folder = cfg["test_two_eye"]["num_images_per_folder"].as<int>();

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
        std::string timestr = dirname.substr(7); // "record_" 占前 7 个字符
        if (point_mapping.find(timestr) == point_mapping.end()) {
            std::cerr << "[WARN] No mapping found for folder: " << dirname << std::endl;
            continue;
        }

        Vec2 true_pog = point_mapping[timestr];
        Vec3 target = screenToWCS(true_pog, cfg);
        std::cout << "Folder: " << dirname << " -> target WCS: " << target.transpose() << std::endl;

        // 打开该目录下的数据记录表
        std::string data_csv_path = (entry.path() / "glint_pupil_data.csv").string();
        std::ifstream data_in(data_csv_path);
        if (!data_in.is_open()) {
            std::cerr << "[WARN] Cannot open data file: " << data_csv_path << std::endl;
            continue;
        }

        // 跳过表头
        std::getline(data_in, line);

        int img_count = 0;
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

            // =========== REQUIREMENT 1: 仅采用 cam_0 ===========
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

            // =========== 剔除检测失败(填充0)的帧 ===========
            if (lg_l_x == 0 && lg_l_y == 0 && lg_r_x == 0 && lg_r_y == 0) continue;
            if (rg_l_x == 0 && rg_l_y == 0 && rg_r_x == 0 && rg_r_y == 0) continue;

            // 构造网络推断所需输入
            PupilCenterGlintInputs inputs;
            PupilCenterGlintInput input;

            // 左眼填充
            input.left.pupil_center = make_vec2(lp_x, lp_y);
            input.left.glints.push_back(make_vec2(lg_l_x, lg_l_y));
            input.left.glints.push_back(make_vec2(lg_r_x, lg_r_y));
            input.left.glints.push_back(make_vec2(lg_m_x, lg_m_y));

            // 右眼填充
            input.right.pupil_center = make_vec2(rp_x, rp_y);
            input.right.glints.push_back(make_vec2(rg_l_x, rg_l_y));
            input.right.glints.push_back(make_vec2(rg_r_x, rg_r_y));
            input.right.glints.push_back(make_vec2(rg_m_x, rg_m_y));

            inputs.data.push_back(input);
            calibrate_against.push_back(std::make_pair(inputs, target));

            img_count++;

            // =========== REQUIREMENT 2: 达到指定读取数量后跳出当前文件夹 ===========
            if (img_count >= max_images_per_folder) {
                break;
            }
        }
        data_in.close();
        std::cout << "  -> Extracted " << img_count << " valid inputs from cam_0." << std::endl;
    }

    std::cout << "\nTotal samples collected: " << calibrate_against.size() << std::endl;

    // =========== 保持后续输出写入 txt 逻辑不变 ===========
    std::ofstream fout(output_dir + "/inference_result_left.txt");
    if (!fout) {
        std::cerr << "[ERROR] Cannot open file: " << output_dir << "/inference_result_left.txt\n";
        return -1;
    }
    // 表头
    fout << "# cornea_x cornea_y cornea_z "
         << "opt_x opt_y opt_z "
         << "vis_x vis_y vis_z "
         << "gt_x gt_y gt_z "
         << "pred_x pred_y pred_z\n";

    for (const auto&[inputs, gt_wcs] : calibrate_against)
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
    std::cout << "\ninference result saved to " << output_dir << "/inference_result_left.txt\n";


    std::ofstream fout_right(output_dir + "/inference_result_right.txt");
    if (!fout_right) {
        std::cerr << "[ERROR] Cannot open file: " << output_dir <<"/inference_result_right.txt\n";
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
    std::cout << "\ninference result saved to " << output_dir << "/inference_result_right.txt\n";

    return 0;
}