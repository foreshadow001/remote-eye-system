#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"
#include "logger/logger.hpp"
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <vector>
#include <string>

using namespace glintdetection;
using namespace visualization;

// 单个 record 文件夹的处理函数 (单线程顺序执行)
bool processRecordDirectory(
    const std::filesystem::path& target_dir, 
    const std::string& param_type, 
    int max_images, 
    bool is_collecting) 
{
    try {
        // 每个文件夹使用独立的检测器实例，避免内部状态冲突
        GlintDetector glint_detector(param_type);
        glint_detector.setLocalDebug(false);
        glint_detector.setDebugTime(false);
        glint_detector.setViz(false);
        glint_detector.setVizThreshold(false);
        if (is_collecting) glint_detector.setIsCollecting(true);

        std::filesystem::path csv_path = target_dir / "glint_pupil_data.csv";
        
        // 1. 使用 error_code 尝试删除旧文件，避免直接崩溃
        std::error_code ec;
        if (std::filesystem::exists(csv_path)) {
            if (!std::filesystem::remove(csv_path, ec)) {
                Logger::error() << "CRITICAL: Cannot delete existing CSV (File may be locked/opened): " 
                                << csv_path.string() << " | Error: " << ec.message();
                return false; // 明确返回失败
            }
        }

        std::ofstream record_csv(csv_path);
        if (!record_csv.is_open()) {
            Logger::error() << "CRITICAL: Failed to create CSV file: " << csv_path.string() 
                            << " - Check permissions or if the folder is read-only.";
            return false;
        }
        // 写入表头
        record_csv << "filepath,lp_x,lp_y,rp_x,rp_y,"
                << "lg_l_x,lg_l_y,lg_r_x,lg_r_y,lg_m_x,lg_m_y,"
                << "rg_l_x,rg_l_y,rg_r_x,rg_r_y,rg_m_x,rg_m_y\n";

        int idx = 0;
        Logger::info() << "Processing folder: " << target_dir.filename().string();

        // 递归遍历该特定 record 文件夹下的所有文件 (例如 cam_0, cam_1 子目录里的图片)
        for (const auto& sub_dir : std::filesystem::directory_iterator(target_dir)) {
            // 过滤掉 target_dir 根目录下的普通文件，只处理文件夹
            if (!sub_dir.is_directory()) continue;

            for (const auto& entry : std::filesystem::directory_iterator(sub_dir.path())) {
                if (!entry.is_regular_file()) continue;

                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(),[](unsigned char c){ return std::tolower(c); });

                if (ext != ".jpg" && ext != ".jpeg" && ext != ".png" && ext != ".bmp") {
                    continue;
                }

                std::string filepath = entry.path().string();
                glint_detector.setImageName(entry.path().stem().string());

                cv::Mat img = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
                if (img.empty()) {
                    Logger::error() << "Failed to read image: " << filepath;
                    continue;
                }

                // 核心检测调用
                auto[leftEyeGlintsList, rightEyeGlintsList, leftPupilCenter, rightPupilCenter] = glint_detector.detect(img);

                // 获取相对路径 (例如: cam_0/000000.jpg)
                std::string rel_path = std::filesystem::relative(entry.path(), target_dir).generic_string();

                record_csv << rel_path << ","
                        << leftPupilCenter.x << "," << leftPupilCenter.y << ","
                        << rightPupilCenter.x << "," << rightPupilCenter.y << ",";

                // 保存左眼 glint geometry
                if (!leftEyeGlintsList.empty() && leftEyeGlintsList[0].size() == 3) {
                    record_csv << leftEyeGlintsList[0][0].x << "," << leftEyeGlintsList[0][0].y << ","
                            << leftEyeGlintsList[0][1].x << "," << leftEyeGlintsList[0][1].y << ","
                            << leftEyeGlintsList[0][2].x << "," << leftEyeGlintsList[0][2].y << ",";
                } else {
                    record_csv << "0,0,0,0,0,0,";
                }

                // 保存右眼 glint geometry
                if (!rightEyeGlintsList.empty() && rightEyeGlintsList[0].size() == 3) {
                    record_csv << rightEyeGlintsList[0][0].x << "," << rightEyeGlintsList[0][0].y << ","
                            << rightEyeGlintsList[0][1].x << "," << rightEyeGlintsList[0][1].y << ","
                            << rightEyeGlintsList[0][2].x << "," << rightEyeGlintsList[0][2].y << "\n";
                } else {
                    record_csv << "0,0,0,0,0,0\n";
                }

                idx++;
                // 抽样打印进度，防止刷屏卡顿
                if (idx % 1000 == 0) {
                    Logger::info() << "  -> Processed " << idx << " images...";
                }

                if (idx >= max_images) break;
            }
        }

        record_csv.close();
        Logger::info() << ">>> Completed folder[" << target_dir.filename().string() << "] with " << idx << " images.";
        return true;
    } catch (const std::exception& e) {
        Logger::error() << "Unexpected exception in processRecordDirectory: " << e.what();
        return false;
    }
}

int main() {
    Cfg cfg;
    bool use_record = cfg["test_glint"]["use_record"].as<bool>();
    std::string param_type = cfg["test_glint"]["param_type"].as<std::string>();
    bool is_collecting = cfg["test_glint"]["is_collecting"].as<bool>();
    int max_images = cfg["test_glint"]["num_images"].as<int>();

    // ====================================================================
    // 模式 1：Record 单线程批量处理模式
    // ====================================================================
    if (use_record) {
        std::string record_dir = cfg["test_glint"]["record_dir"].as<std::string>();
        
        if (!std::filesystem::exists(record_dir) || !std::filesystem::is_directory(record_dir)) {
            Logger::error() << "Record root directory does not exist: " << record_dir;
            return -1;
        }

        // 1. 扫描满足条件的所有子文件夹
        std::vector<std::filesystem::path> valid_record_dirs;
        for (const auto& entry : std::filesystem::directory_iterator(record_dir)) {
            if (entry.is_directory()) {
                std::string dirname = entry.path().filename().string();
                
                // 必须以 "record" 开头，且不包含 "raw" 字符串
                if (dirname.rfind("record", 0) == 0 && dirname.find("raw") == std::string::npos) {
                    valid_record_dirs.push_back(entry.path());
                }
            }
        }

        if (valid_record_dirs.empty()) {
            Logger::error() << "No valid record directories found in: " << record_dir;
            return 0;
        }

        Logger::info() << "Found " << valid_record_dirs.size() << " valid record directories. Starting sequential processing...";

        int success_count = 0;
        for (const auto& dir : valid_record_dirs) {
            if (processRecordDirectory(dir, param_type, max_images, is_collecting)) {
                success_count++;
            } else {
                Logger::error() << "Failed to process directory: " << dir.string();
                // 如果你想在遇到任何错误时彻底停止，可以在这里 return -1;
            }
        }

        Logger::info() << "Batch process finished. Successfully processed " 
                       << success_count << " / " << valid_record_dirs.size() << " directories.";
        
        return (success_count == valid_record_dirs.size()) ? 0 : -1;
    }

    // ====================================================================
    // 模式 2：普通单文件夹处理模式（保留可视化、调试与阈值输出等）
    // ====================================================================
    GlintDetector glint_detector(param_type);
    glint_detector.setLocalDebug(cfg["test_glint"]["local_debug"].as<bool>());
    glint_detector.setDebugTime(cfg["test_glint"]["debug_time"].as<bool>());
    glint_detector.setViz(cfg["test_glint"]["viz"].as<bool>());
    glint_detector.setVizThreshold(cfg["test_glint"]["viz_threshold"].as<bool>());
    if (is_collecting) glint_detector.setIsCollecting(true);

    std::string input_folder = cfg["test_glint"]["input_folder"].as<std::string>();
    std::string threshold_output_folder = input_folder + "/threshold_output";
    std::string debug_img_output_folder = input_folder + "/debug_img";

    std::filesystem::create_directories(threshold_output_folder);
    std::filesystem::create_directories(debug_img_output_folder);

    std::vector<std::string> sub_dirs = {
        "/0_binary_pupil", "/1_glass_reflection", "/2_frame_reflection",
        "/3_exclusion_mask", "/4_res", "/5_jitter_pupil", "/6_jitter_glint", 
        "/7_sr_left", "/8_sr_right"
    };

    if (!std::filesystem::exists(input_folder) || !std::filesystem::is_directory(input_folder)) {
        Logger::error() << "Input folder does not exist or is not a folder: " << input_folder;
        return -1;
    }

    Logger::info() << "Start processing images in normal mode...";

    int idx = 0;
    for (const auto& entry : std::filesystem::directory_iterator(input_folder)) {
        if (!entry.is_regular_file()) continue;

        std::string filename = entry.path().filename().string();
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),[](unsigned char c){ return std::tolower(c); });

        if (ext != ".jpg" && ext != ".jpeg" && ext != ".png" && ext != ".bmp") {
            continue;
        }

        std::string filepath = entry.path().string();
        glint_detector.setImageName(entry.path().stem().string());

        cv::Mat img = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            Logger::error() << "Failed to read image: " << filepath;
            continue;
        }

        auto[leftEyeGlintsList, rightEyeGlintsList, leftPupilCenter, rightPupilCenter] = glint_detector.detect(img);

        // 收集调试信息
        cv::Mat threshold_output = glint_detector.getThresholdOutput();
        std::vector<cv::Mat> debug_imgs = glint_detector.getDebugImgs();
        if (!debug_imgs.empty()) {
            for (int i = 0; i < debug_imgs.size(); i++) {
                if (debug_imgs[i].empty()) continue;
                std::string save_path = debug_img_output_folder + sub_dirs[i];
                std::filesystem::create_directories(save_path);
                cv::imwrite(save_path + "/" + filename, debug_imgs[i]);
            }
        }
        cv::imwrite(threshold_output_folder + "/" + filename, threshold_output);

        Logger::info() << "Saved to: " << threshold_output_folder + "/" + filename
                       << " | num of glints: " << leftEyeGlintsList.size() << " | " << rightEyeGlintsList.size();

        idx++;
        if (idx >= max_images) break;
    }

    Logger::info() << "Processed " << idx << " images in normal mode.";
    return 0;
}