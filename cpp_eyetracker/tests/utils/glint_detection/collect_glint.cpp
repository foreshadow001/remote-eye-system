#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"
#include "logger/logger.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace glintdetection;
namespace fs = std::filesystem;

// 数据结构定义：用于保存序列化参数
struct PupilMeta {
    double roi_min_val;
    double darkness;
    double area;
    int contour_points;
    float major_axis;
    float axis_ratio;
    double solidity;
    double fit_ratio;
    float avg_residual;
    bool is_valid = false;
};

struct GlintMeta {
    double lr_y;
    double lr_x;
    double lm_x_ratio;
    double lm_y_ratio;
    double rm_x_ratio;
    double rm_y_ratio;
    double max_exclusion_radius_ratio;
    double bg_brightness;
    bool is_valid = false;
};

// 辅助函数：将参数保存为 YAML 格式
void saveMetaData(const std::string& filepath, const std::vector<PupilMeta>& pupils, const std::vector<GlintMeta>& glints) {
    cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
    
    fs << "Pupils" << "[";
    for (const auto& p : pupils) {
        fs << "{:" 
           << "roi_min_val" << p.roi_min_val
           << "darkness" << p.darkness
           << "area" << p.area
           << "contour_points" << p.contour_points
           << "major_axis" << p.major_axis
           << "axis_ratio" << p.axis_ratio
           << "solidity" << p.solidity
           << "fit_ratio" << p.fit_ratio
           << "avg_residual" << p.avg_residual
           << "is_valid" << p.is_valid
           << "}";
    }
    fs << "]";

    fs << "Glints" << "[";
    for (const auto& g : glints) {
        fs << "{:"
           << "lr_y" << g.lr_y
           << "lr_x" << g.lr_x
           << "lm_x_ratio" << g.lm_x_ratio
           << "lm_y_ratio" << g.lm_y_ratio
           << "rm_x_ratio" << g.rm_x_ratio
           << "rm_y_ratio" << g.rm_y_ratio
           << "max_exclusion_radius_ratio" << g.max_exclusion_radius_ratio
           << "bg_brightness" << g.bg_brightness
           << "is_valid" << g.is_valid
           << "}";
    }
    fs << "]";
    fs.release();
}

// ================== 从录制文件提取中值帧 ==================
void prepareRecordData(const std::string& save_dir, const std::string& collect_input_dir) {
    if (!fs::exists(save_dir)) {
        Logger::error() << "Source save_dir does not exist: " << save_dir;
        return;
    }

    if (fs::exists(collect_input_dir)) return;

    if (!fs::exists(collect_input_dir)) {
        fs::create_directories(collect_input_dir);
    }

    for (const auto& entry : fs::directory_iterator(save_dir)) {
        if (!entry.is_directory()) continue;
        
        std::string rec_name = entry.path().filename().string();
        if (rec_name.find("record_") != 0) continue; 
        
        std::string timestamp = rec_name.substr(7); 

        for (const auto& cam_entry : fs::directory_iterator(entry.path())) {
            if (!cam_entry.is_directory()) continue;
            
            std::string cam_name = cam_entry.path().filename().string();
            if (cam_name.find("cam_") != 0) continue;

            std::vector<fs::path> jpg_files;
            for (const auto& file_entry : fs::directory_iterator(cam_entry.path())) {
                if (file_entry.is_regular_file() && file_entry.path().extension() == ".jpg") {
                    jpg_files.push_back(file_entry.path());
                }
            }

            if (jpg_files.empty()) continue;

            std::sort(jpg_files.begin(), jpg_files.end());

            size_t mid_idx = jpg_files.size() / 2;
            fs::path mid_file = jpg_files[mid_idx];
            std::string frame_str = mid_file.stem().string();

            std::string new_filename = timestamp + "_" + cam_name + "_" + frame_str + ".jpg";
            fs::path dest_path = fs::path(collect_input_dir) / new_filename;

            if (!fs::exists(dest_path)) {
                try {
                    fs::copy_file(mid_file, dest_path, fs::copy_options::overwrite_existing);
                    // Logger::info() << "Extracted middle frame: " << new_filename;
                } catch (const fs::filesystem_error& e) {
                    Logger::error() << "Failed to copy file: " << e.what();
                }
            }
        }
    }
}
// ===============================================================

// 第一阶段：处理图像，提取特征并保存可视化图
void processAndSaveData(const std::string& input_folder, const std::string& output_folder, int max_images) {
    GlintDetector glint_detector("relaxed");
    glint_detector.setViz(false);
    glint_detector.setVizThreshold(false);
    glint_detector.setLocalDebug(false);
    glint_detector.setIsCollecting(true);

    if (!fs::exists(output_folder)) fs::create_directories(output_folder);

    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = FindFirstFile(search_path.c_str(), &fd);
    
    if (hFind == INVALID_HANDLE_VALUE) {
        Logger::error() << "Input folder does not exist or empty: " << input_folder;
        return;
    }

    int count = 0;

    do {
        std::string filename = fd.cFileName;
        if (filename == "." || filename == "..") continue;
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;

        std::string filepath = input_folder + "\\" + filename;
        std::string filename_no_ext = filename.substr(0, filename.find_last_of('.'));
        glint_detector.setImageName(filename_no_ext);

        std::string current_img_out_dir = output_folder + "\\" + filename_no_ext;
        std::string metadata_path = current_img_out_dir + "\\metadata.yml";

        if (fs::exists(metadata_path)) {
            Logger::info() << "Metadata already exists, skipping extraction for: " << filename;
            if (++count >= max_images) break; 
            continue; 
        }

        cv::Mat img = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;

        glint_detector.detect(img);

        std::string pupil_dir = current_img_out_dir + "\\pupil";
        std::string glint_dir = current_img_out_dir + "\\glint";
        fs::create_directories(pupil_dir);
        fs::create_directories(glint_dir);

        cv::Mat bgr_img;
        cv::cvtColor(img, bgr_img, cv::COLOR_GRAY2BGR);

        std::vector<PupilMeta> pupil_metas;
        std::vector<GlintMeta> glint_metas;

        // 1. 处理 Pupils
        const auto& pupils = glint_detector.getPupils();
        for (size_t i = 0; i < pupils.size(); ++i) {
            const auto& p = pupils[i];
            PupilMeta pm = {p.roi_min_val, p.darkness, p.area, p.contour_points, 
                            p.major_axis, p.axis_ratio, p.solidity, p.fit_ratio, p.avg_residual, false};
            pupil_metas.push_back(pm);

            cv::Mat viz = bgr_img.clone();
            cv::ellipse(viz, p.rr, cv::Scalar(0, 255, 0), 2);
            cv::circle(viz, p.rr.center, 2, cv::Scalar(0, 0, 255), -1);
            cv::imwrite(pupil_dir + "\\" + std::to_string(i) + ".png", viz);
        }

        // 2. 处理 Glint Geometries
        const auto& geometries = glint_detector.getGlintGeometries();
        for (size_t i = 0; i < geometries.size(); ++i) {
            const auto& g = geometries[i];
            
            double lr_x = std::abs(g.l_pt.x - g.r_pt.x);
            double lr_y = std::abs(g.l_pt.y - g.r_pt.y);
            double lm_x = g.m_pt.x - g.l_pt.x;
            double lm_y = g.m_pt.y - g.l_pt.y;
            double rm_x = g.r_pt.x - g.m_pt.x;
            double rm_y = g.m_pt.y - g.r_pt.y;

            if (g.l_pt.y < g.r_pt.y) {
                std::swap(lm_x, rm_x);
                std::swap(lm_y, rm_y);
            }

            double max_dist_ratio = 0.0;
            if (g.linked_pupil.major_axis > 0) {
                double d1 = cv::norm(g.l_pt - g.linked_pupil.rr.center);
                double d2 = cv::norm(g.r_pt - g.linked_pupil.rr.center);
                double d3 = cv::norm(g.m_pt - g.linked_pupil.rr.center);
                max_dist_ratio = std::max({d1, d2, d3}) / (g.linked_pupil.major_axis * 0.5);
            }

            GlintMeta gm = {
                lr_y, lr_x,
                lr_x > 0 ? lm_x / lr_x : 0, lr_x > 0 ? lm_y / lr_x : 0,
                lr_x > 0 ? rm_x / lr_x : 0, lr_x > 0 ? rm_y / lr_x : 0,
                max_dist_ratio, g.bg_brightness, false
            };
            glint_metas.push_back(gm);

            cv::Mat viz = bgr_img.clone();
            // --- 修复：根据要求调整为绘制 1 个像素宽度的纯绿线，且不画点 ---
            cv::line(viz, g.l_pt, g.r_pt, cv::Scalar(0, 255, 0), 1);
            cv::line(viz, g.l_pt, g.m_pt, cv::Scalar(0, 255, 0), 1);
            cv::line(viz, g.r_pt, g.m_pt, cv::Scalar(0, 255, 0), 1);

            if (g.linked_pupil.major_axis > 0) {
                cv::circle(viz, g.linked_pupil.rr.center, 3, cv::Scalar(0, 0, 255), -1);
            }
            
            cv::imwrite(glint_dir + "\\" + std::to_string(i) + ".png", viz);
        }

        saveMetaData(metadata_path, pupil_metas, glint_metas);
        
        // Logger::info() << "Processed and saved: " << filename;
        
        if (++count >= max_images) break;

    } while (FindNextFile(hFind, &fd) != 0);
    FindClose(hFind);
}

// 第二阶段：交互式人工筛选
void manualReviewData(const std::string& output_folder) {
    std::string win_name = "Data Collector Review";
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    bool exit_early = false;

    // 先收集所有有效的检测目录以便计算总数展示进度
    std::vector<fs::path> valid_directories;
    for (const auto& entry : fs::directory_iterator(output_folder)) {
        if (!entry.is_directory()) continue;
        std::string yml_path = entry.path().string() + "\\metadata.yml";
        if (fs::exists(yml_path)) {
            valid_directories.push_back(entry.path());
        }
    }

    int total_images = valid_directories.size();

    for (int idx = 0; idx < total_images; ++idx) {
        const auto& img_path = valid_directories[idx];
        std::string img_folder = img_path.string();
        std::string img_name = img_path.filename().string();
        std::string yml_path = img_folder + "\\metadata.yml";

        // 1. 读取现有的 Metadata
        cv::FileStorage fs_read(yml_path, cv::FileStorage::READ);
        cv::FileNode pupil_nodes = fs_read["Pupils"];
        cv::FileNode glint_nodes = fs_read["Glints"];

        std::vector<PupilMeta> pupils;
        for (auto it = pupil_nodes.begin(); it != pupil_nodes.end(); ++it) {
            PupilMeta pm;
            pm.roi_min_val    = (double)(*it)["roi_min_val"];
            pm.darkness       = (double)(*it)["darkness"];
            pm.area           = (double)(*it)["area"];
            pm.contour_points = (int)(*it)["contour_points"];
            pm.major_axis     = (float)(*it)["major_axis"];
            pm.axis_ratio     = (float)(*it)["axis_ratio"];
            pm.solidity       = (double)(*it)["solidity"];
            pm.fit_ratio      = (double)(*it)["fit_ratio"];
            pm.avg_residual   = (float)(*it)["avg_residual"];
            pm.is_valid       = (int)(*it)["is_valid"] != 0; 
            pupils.push_back(pm);
        }

        std::vector<GlintMeta> glints;
        for (auto it = glint_nodes.begin(); it != glint_nodes.end(); ++it) {
            GlintMeta gm;
            gm.lr_y           = (double)(*it)["lr_y"];
            gm.lr_x           = (double)(*it)["lr_x"];
            gm.lm_x_ratio     = (double)(*it)["lm_x_ratio"];
            gm.lm_y_ratio     = (double)(*it)["lm_y_ratio"];
            gm.rm_x_ratio     = (double)(*it)["rm_x_ratio"];
            gm.rm_y_ratio     = (double)(*it)["rm_y_ratio"];
            gm.max_exclusion_radius_ratio = (double)(*it)["max_exclusion_radius_ratio"];
            gm.bg_brightness  = (double)(*it)["bg_brightness"];
            gm.is_valid       = (int)(*it)["is_valid"] != 0; 
            glints.push_back(gm);
        }
        fs_read.release(); 

        // 2. 加载图片路径
        std::vector<std::string> pupil_imgs, glint_imgs;
        for (size_t i = 0; i < pupils.size(); ++i) {
            pupil_imgs.push_back(img_folder + "\\pupil\\" + std::to_string(i) + ".png");
        }
        for (size_t i = 0; i < glints.size(); ++i) {
            glint_imgs.push_back(img_folder + "\\glint\\" + std::to_string(i) + ".png");
        }

        int mode = 0; // 0 = Pupil, 1 = Glint
        int p_idx = 0, g_idx = 0;
        bool next_image = false;

        while (!next_image) {
            int current_total = (mode == 0) ? pupil_imgs.size() : glint_imgs.size();
            int current_idx = (mode == 0) ? p_idx : g_idx;
            
            cv::Mat display_img;
            if (current_total > 0) {
                std::string path = (mode == 0) ? pupil_imgs[current_idx] : glint_imgs[current_idx];
                display_img = cv::imread(path);
                
                if (display_img.empty()) {
                    display_img = cv::Mat::zeros(600, 800, CV_8UC3);
                    cv::putText(display_img, "IMAGE NOT FOUND", cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                }

                bool is_selected = (mode == 0) ? pupils[current_idx].is_valid : glints[current_idx].is_valid;
                
                // --- 显示总体进度和文件名 ---
                std::string progress_str = "Progress: " + std::to_string(idx + 1) + " / " + std::to_string(total_images) + " | File: " + img_name;
                cv::putText(display_img, progress_str, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                // --- 显示类别内部状态 ---
                std::string mode_str = (mode == 0) ? "MODE: PUPIL" : "MODE: GLINT";
                cv::putText(display_img, mode_str + "[" + std::to_string(current_idx+1) + "/" + std::to_string(current_total) + "]", 
                            cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
                
                if (is_selected) {
                    cv::rectangle(display_img, cv::Point(0,0), cv::Point(display_img.cols-1, display_img.rows-1), cv::Scalar(0, 255, 0), 10);
                    cv::putText(display_img, "SELECTED", cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 3);
                } else {
                    cv::putText(display_img, "REJECTED (Space to Keep)", cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                }
            } else {
                display_img = cv::Mat::zeros(600, 800, CV_8UC3);
                cv::putText(display_img, "Progress: " + std::to_string(idx + 1) + " / " + std::to_string(total_images) + " | File: " + img_name, 
                            cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                cv::putText(display_img, "NO DATA IN THIS CATEGORY", cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            }

            // ========== 新增：右侧侧边栏参数显示逻辑 ==========
            int panel_width = 350; // 右侧面板宽度
            cv::Mat canvas = cv::Mat::zeros(display_img.rows, display_img.cols + panel_width, CV_8UC3);
            display_img.copyTo(canvas(cv::Rect(0, 0, display_img.cols, display_img.rows)));

            if (current_total > 0) {
                int text_x = display_img.cols + 20;
                int start_y = 50;
                int step_y = 35;

                cv::putText(canvas, "--- Parameters ---", cv::Point(text_x, start_y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                start_y += step_y + 10;

                auto putKV = [&](const std::string& k, double v) {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "%s: %.3f", k.c_str(), v);
                    cv::putText(canvas, buf, cv::Point(text_x, start_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
                    start_y += step_y;
                };

                if (mode == 0) {
                    const auto& pm = pupils[current_idx];
                    putKV("roi_min_val", pm.roi_min_val);
                    putKV("darkness", pm.darkness);
                    putKV("area", pm.area);
                    putKV("contour_points", (double)pm.contour_points);
                    putKV("major_axis", pm.major_axis);
                    putKV("axis_ratio", pm.axis_ratio);
                    putKV("solidity", pm.solidity);
                    putKV("fit_ratio", pm.fit_ratio);
                    putKV("avg_residual", pm.avg_residual);
                } else {
                    const auto& gm = glints[current_idx];
                    putKV("lr_y", gm.lr_y);
                    putKV("lr_x", gm.lr_x);
                    // 排除要求外的 xy ratio
                    putKV("max_excl_rad_ratio", gm.max_exclusion_radius_ratio);
                    putKV("bg_brightness", gm.bg_brightness);
                }
            }

            cv::imshow(win_name, canvas); // 替换原来的 cv::imshow(win_name, display_img);
            // ===================================================

            int key = cv::waitKeyEx(0);

            switch (key) {
                case ' ': 
                    if (current_total > 0) {
                        if (mode == 0) pupils[current_idx].is_valid = !pupils[current_idx].is_valid;
                        else glints[current_idx].is_valid = !glints[current_idx].is_valid;
                    }
                    break;
                // 2. 将 Tab 替换为 上/下 方向键
                case 2490368: // Windows 上方向键
                case 2621440: // Windows 下方向键
                case 65362:   // Linux/Mac 上方向键
                case 65364:   // Linux/Mac 下方向键
                    mode = 1 - mode;
                    break;
                // 3. 将 'a' 替换为 左方向键
                case 2424832: // Windows 左方向键
                case 65361:   // Linux/Mac 左方向键
                    if (mode == 0 && p_idx > 0) p_idx--;
                    if (mode == 1 && g_idx > 0) g_idx--;
                    break;
                // 4. 将 'd' 替换为 右方向键
                case 2555904: // Windows 右方向键
                case 65363:   // Linux/Mac 右方向键
                    if (mode == 0 && p_idx < (int)pupil_imgs.size() - 1) p_idx++;
                    if (mode == 1 && g_idx < (int)glint_imgs.size() - 1) g_idx++;
                    break;
                // ========== 新增：Backspace 退回上一张图片 ==========
                case 8: // Backspace 键值
                    next_image = true;
                    if (idx > 0) {
                        idx -= 2; // -1表示回退，再-1抵消for循环结尾的++idx
                    } else {
                        idx = -1; // 已经是第一张时，保持在第一张 (抵消++idx后为0)
                    }
                    break;
                // ==================================================
                case 13: // Enter
                    next_image = true;
                    break;
                case 27: // Esc
                    next_image = true;
                    exit_early = true;
                    break;
            }
        }

        saveMetaData(yml_path, pupils, glints);
        if (exit_early) return;
    }
}

int main() {
    Cfg cfg;

    bool use_record = false;
    try {
        use_record = cfg["collect_glint"]["use_record"].as<bool>();
    } catch (...) {
        Logger::info() << "use_record not found in config. Defaulting to false.";
    }

    std::string input_folder = cfg["collect_glint"]["input_folder"].as<std::string>();
    std::string output_folder = cfg["collect_glint"]["output_folder"].as<std::string>();
    int max_images = cfg["collect_glint"]["num_images"].as<int>();

    // 如果启用了从录制中抽取图片
    if (use_record) {
        Logger::info() << "use_record is enabled. Preparing data from test_multi_cam's save_dir...";
        std::string save_dir = cfg["collect_glint"]["record_dir"].as<std::string>();
        
        // --- 修复：覆盖原本的输入和输出路径，将其重定向到提取文件夹中 ---
        input_folder = save_dir + "\\collect_input";
        output_folder = save_dir + "\\collect_output";
        
        // 执行自动拷贝和重命名过程
        prepareRecordData(save_dir, input_folder);
    }
    
    // 步骤 1：以宽松参数运行检测，生成可视化切片和包含原始特征的 YAML 元数据文件
    Logger::info() << "Phase 1: Starting Data Collection and Visualization Generation from " << input_folder;
    processAndSaveData(input_folder, output_folder, max_images);
    
    // 步骤 2：启动 OpenCV GUI 监听键盘，进行人工复核
    Logger::info() << "Phase 2: Starting Manual Review Interface...";
    Logger::info() << "Controls: [Space] Toggle Selection | [Up/Down] Switch Category | [Left/Right] Prev/Next in Category | [Enter] Next Image";
    manualReviewData(output_folder);
    
    Logger::info() << "Data collection and review completed.";
    return 0;
}