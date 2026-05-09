#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <filesystem>
#include <random>
#include <vector>
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

// 定义实验类型
enum class JitterType { PUPIL, GLINT };

// 辅助函数：格式化浮点数，用于生成文件名
std::string format_float(double val, int precision = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << val;
    return out.str();
}

// 核心功能：执行一次完整的“加噪 -> 标定 -> 推理保存”流程
void run_calibration_session(
    const std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>>& clean_data_left,
    const std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>>& clean_data_right,
    double std_val,
    int num_noise_samples,
    JitterType type,
    const Cfg& cfg,
    const std::string& output_dir
) {
    // 构造文件名后缀
    std::string type_str = (type == JitterType::PUPIL) ? "pj" : "gj";
    std::string filename_suffix = "_" + type_str + "_" + format_float(std_val) + "_" + format_float(std_val);
    
    // std为0则作为基准文件(无噪声)
    if (std_val == 0.0) {
        filename_suffix = "_no_jitter";
    }

    std::cout << "\n============================================================" << std::endl;
    std::cout << ">>> Running Session: " << (type == JitterType::PUPIL ? "Pupil" : "Glint") 
              << " Jitter Std = " << std_val << " <<<" << std::endl;
    std::cout << "============================================================" << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, std_val);

    // 1. 准备噪声数据副本 (针对每个样本生成 num_noise_samples 个随机副本)
    auto prepare_noisy_data = [&](const std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>>& clean_data) {
        if (std_val <= 0.0) return clean_data; // 0直接返回干净数据(不扩增)
        
        std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>> noisy_data;
        noisy_data.reserve(clean_data.size() * num_noise_samples);
        
        for (const auto& item : clean_data) {
            // 对当前干净样本，生成 N 个加了不同噪声的副本
            for (int i = 0; i < num_noise_samples; ++i) {
                auto noisy_item = item; // 深拷贝
                for (auto& input : noisy_item.first.data) {
                    if (type == JitterType::PUPIL) {
                        input.pupil_center[0] += dist(gen);
                        input.pupil_center[1] += dist(gen);
                    } else { // GLINT
                        for (auto& g : input.glints) {
                            g[0] += dist(gen);
                            g[1] += dist(gen);
                        }
                    }
                }
                noisy_data.push_back(noisy_item);
            }
        }
        return noisy_data;
    };

    auto calib_data_left = prepare_noisy_data(clean_data_left);
    auto calib_data_right = prepare_noisy_data(clean_data_right);

    // 打印数据量变化信息
    if (std_val > 0.0) {
        std::cout << "Data augmented: " << clean_data_left.size() << " -> " << calib_data_left.size() 
                  << " samples per eye (" << num_noise_samples << " noises/sample)" << std::endl;
    }

    // 2. 获取约束边界配置
    auto bounds_config = cfg["calib_values_bounds"];
    std::vector<std::vector<std::pair<double, double>>> bounds = {
        { {deg_to_rad(bounds_config["alpha"].as<std::vector<double>>()[0]), deg_to_rad(bounds_config["alpha"].as<std::vector<double>>()[1])} },
        { {deg_to_rad(bounds_config["beta"].as<std::vector<double>>()[0]),  deg_to_rad(bounds_config["beta"].as<std::vector<double>>()[1])} },
        { {bounds_config["R"].as<std::vector<double>>()[0],  bounds_config["R"].as<std::vector<double>>()[1]} },
        { {bounds_config["K"].as<std::vector<double>>()[0],  bounds_config["K"].as<std::vector<double>>()[1]} }
    };

    // 单眼标定和推理闭包函数
    auto run_eye_calib = [&](std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>>& data, std::string side) {
        if (data.empty()) {
            std::cerr << "[WARN] No valid frames for " << side << " eye." << std::endl;
            return;
        }

        // 每次Session必须实例化新的Tracker和Params
        GazeTracker tracker;
        Calibration calib;
        SingleEyeAndCameraParameters params(side);

        std::vector<std::vector<double>> init = {{params.alpha}, {params.beta}, {params.R}, {params.K}};
        
        std::cout << "Starting " << side << " eye calibration..." << std::endl;
        auto res_vec = calib.calibrate(tracker, params, data, init, bounds);
        params = variables_calibration_applicator(params, vecvec_to_pointer_pointer(res_vec));

        std::cout << "--- " << side << " Eye Result ---" << std::endl;
        std::cout << "Alpha: " << rad_to_deg(params.alpha) << " deg, Beta: " << rad_to_deg(params.beta) 
                  << " deg, R: " << params.R << ", K: " << params.K << std::endl;

        std::string out_path = output_dir + "/calib_result_" + side + filename_suffix + ".txt";
        std::ofstream fout(out_path);
        if (!fout) {
            std::cerr << "[ERROR] Cannot open output file: " << out_path << std::endl;
            return;
        }

        fout << "# cornea_x cornea_y cornea_z opt_x opt_y opt_z vis_x vis_y vis_z gt_x gt_y gt_z\n";
        for (const auto& [inputs, gt] : data) {
            try {
                auto res = tracker.estimate(inputs, params);
                fout << std::fixed << std::setprecision(6)
                     << res.cornea_center[0] << ' ' << res.cornea_center[1] << ' ' << res.cornea_center[2] << ' '
                     << res.optical_axis_unit[0] << ' ' << res.optical_axis_unit[1] << ' ' << res.optical_axis_unit[2] << ' '
                     << res.visual_axis_unit[0] << ' ' << res.visual_axis_unit[1] << ' ' << res.visual_axis_unit[2] << ' '
                     << gt[0] << ' ' << gt[1] << ' ' << gt[2] << '\n';
            } catch (const std::exception& e) {
                continue;
            }
        }
        std::cout << "Inference saved to: " << out_path << std::endl;
    };

    run_eye_calib(calib_data_left, "left");
    run_eye_calib(calib_data_right, "right");
}

int main() {
    std::cout << "Loading config file for Calibration Jitter Ablation Study..." << std::endl;
    Cfg cfg;
    
    std::string record_dir = cfg["test_jitter"]["record_dir"].as<std::string>();
    int max_images_per_folder = cfg["test_jitter"]["num_images_per_folder"].as<int>();
    std::string output_dir = cfg["test_jitter"]["output_dir"].as<std::string>();
    
    // 读取每个样本生成多少个噪声的配置
    int num_noise_samples = cfg["test_jitter"]["num_noise_samples"].as<int>();
    if (num_noise_samples < 1) num_noise_samples = 1;

    std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>> clean_data_left;
    std::vector<std::pair<SingleEyePupilCenterGlintInputs, Vec3>> clean_data_right;

    // ==========================================================
    // 阶段 1：自动读取映射表和干净数据 (仅执行一次，避免重复IO)
    // ==========================================================
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

    std::cout << "\n[INFO] Extracting clean features from data..." << std::endl;
    for (const auto& entry : fs::directory_iterator(record_dir)) {
        if (!entry.is_directory()) continue;
        
        std::string dirname = entry.path().filename().string();
        if (dirname.rfind("record_", 0) != 0 || dirname.find("raw") != std::string::npos) continue;

        std::string timestr = dirname.substr(7);
        if (point_mapping.find(timestr) == point_mapping.end()) {
            std::cerr << "[WARN] No mapping found for folder: " << dirname << std::endl;
            continue;
        }

        Vec2 true_pog = point_mapping[timestr];
        Vec3 target = screenToWCS(true_pog, cfg);

        std::string data_csv_path = (entry.path() / "glint_pupil_data.csv").string();
        std::ifstream data_in(data_csv_path);
        if (!data_in.is_open()) continue;

        std::getline(data_in, line); // 跳过表头
        int img_count_left = 0, img_count_right = 0;
        
        while (std::getline(data_in, line)) {
            if (line.empty()) continue;

            std::stringstream ss(line);
            std::vector<std::string> tokens;
            std::string token;
            while (std::getline(ss, token, ',')) tokens.push_back(token);
            if (tokens.size() < 17) continue;

            if (tokens[0].find("cam_0") == std::string::npos) continue;

            double lp_x = std::stod(tokens[1]), lp_y = std::stod(tokens[2]);
            double rp_x = std::stod(tokens[3]), rp_y = std::stod(tokens[4]);
            double lg_l_x = std::stod(tokens[5]), lg_l_y = std::stod(tokens[6]);
            double lg_r_x = std::stod(tokens[7]), lg_r_y = std::stod(tokens[8]);
            double lg_m_x = std::stod(tokens[9]), lg_m_y = std::stod(tokens[10]);
            double rg_l_x = std::stod(tokens[11]), rg_l_y = std::stod(tokens[12]);
            double rg_r_x = std::stod(tokens[13]), rg_r_y = std::stod(tokens[14]);
            double rg_m_x = std::stod(tokens[15]), rg_m_y = std::stod(tokens[16]);

            bool left_valid = (lg_l_x != 0 || lg_l_y != 0 || lg_r_x != 0 || lg_r_y != 0);
            bool right_valid = (rg_l_x != 0 || rg_l_y != 0 || rg_r_x != 0 || rg_r_y != 0);

            if (left_valid && img_count_left < max_images_per_folder) {
                SingleEyePupilCenterGlintInputs inputs_l;
                SingleEyePupilCenterGlintInput input_l;
                input_l.pupil_center = make_vec2(lp_x, lp_y);
                input_l.glints.push_back(make_vec2(lg_l_x, lg_l_y));
                input_l.glints.push_back(make_vec2(lg_r_x, lg_r_y));
                input_l.glints.push_back(make_vec2(lg_m_x, lg_m_y));
                inputs_l.data.push_back(input_l);
                clean_data_left.push_back(std::make_pair(inputs_l, target));
                img_count_left++;
            }

            if (right_valid && img_count_right < max_images_per_folder) {
                SingleEyePupilCenterGlintInputs inputs_r;
                SingleEyePupilCenterGlintInput input_r;
                input_r.pupil_center = make_vec2(rp_x, rp_y);
                input_r.glints.push_back(make_vec2(rg_l_x, rg_l_y));
                input_r.glints.push_back(make_vec2(rg_r_x, rg_r_y));
                input_r.glints.push_back(make_vec2(rg_m_x, rg_m_y));
                inputs_r.data.push_back(input_r);
                clean_data_right.push_back(std::make_pair(inputs_r, target));
                img_count_right++;
            }

            if (img_count_left >= max_images_per_folder && img_count_right >= max_images_per_folder) break;
        }
        data_in.close();
        std::cout << "  -> Extracted " << dirname << " | L: " << img_count_left << ", R: " << img_count_right << std::endl;
    }

    std::cout << "\n[INFO] Total clean samples -> Left: " << clean_data_left.size() << " | Right: " << clean_data_right.size() << std::endl;

    // ==========================================================
    // 阶段 2：读取遍历参数并自动化执行消融实验
    // ==========================================================
    double p_start = cfg["test_jitter"]["pupil_jitter_start"].as<double>();
    double p_step  = cfg["test_jitter"]["pupil_jitter_step"].as<double>();
    double p_end   = cfg["test_jitter"]["pupil_jitter_end"].as<double>();

    double g_start = cfg["test_jitter"]["glint_jitter_start"].as<double>();
    double g_step  = cfg["test_jitter"]["glint_jitter_step"].as<double>();
    double g_end   = cfg["test_jitter"]["glint_jitter_end"].as<double>();

    // 基准测试 (0.0 噪声)
    if (p_start > 0 && g_start > 0) {
        run_calibration_session(clean_data_left, clean_data_right, 0.0, num_noise_samples, JitterType::PUPIL, cfg, output_dir);
    }

    // Pupil Jitter 循环
    for (double s = p_start; s <= p_end + 1e-6; s += p_step) {
        run_calibration_session(clean_data_left, clean_data_right, s, num_noise_samples, JitterType::PUPIL, cfg, output_dir);
    }

    // Glint Jitter 循环
    for (double s = g_start; s <= g_end + 1e-6; s += g_step) {
        if (s == 0.0) continue; 
        run_calibration_session(clean_data_left, clean_data_right, s, num_noise_samples, JitterType::GLINT, cfg, output_dir);
    }

    std::cout << "\n[SUCCESS] All ablation experiments completed!" << std::endl;
    return 0;
}