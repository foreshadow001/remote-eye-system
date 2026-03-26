#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"
#include "logger/logger.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

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

// 第一阶段：处理图像，提取特征并保存可视化图
void processAndSaveData(Cfg& cfg) {
    GlintDetector glint_detector("inference");

    std::string input_folder = cfg["collect_glint"]["input_folder"].as<std::string>();
    std::string output_folder = cfg["collect_glint"]["output_folder"].as<std::string>();

    if (!fs::exists(output_folder)) fs::create_directories(output_folder);

    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = FindFirstFile(search_path.c_str(), &fd);
    
    if (hFind == INVALID_HANDLE_VALUE) {
        Logger::error() << "Input folder does not exist: " << input_folder;
        return;
    }

    int count = 0;
    int max_images = cfg["collect_glint"]["num_images"].as<int>();

    do {
        std::string filename = fd.cFileName;
        if (filename == "." || filename == "..") continue;
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;

        std::string filepath = input_folder + "\\" + filename;
        std::string filename_no_ext = filename.substr(0, filename.find_last_of('.'));

        std::string current_img_out_dir = output_folder + "\\" + filename_no_ext;
        std::string metadata_path = current_img_out_dir + "\\metadata.yml";

        // 【新增逻辑】：如果 metadata.yml 已经存在，说明之前处理过，直接跳过提取阶段
        if (fs::exists(metadata_path)) {
            Logger::info() << "Metadata already exists, skipping extraction for: " << filename;
            if (++count >= max_images) break; 
            continue; 
        }

        cv::Mat img = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;

        // 运行检测
        glint_detector.detect(img);

        // 创建当前图像的专属目录
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
                max_dist_ratio = std::max({d1, d2, d3}) / g.linked_pupil.major_axis;
            }

            GlintMeta gm = {
                lr_y, lr_x,
                lr_x > 0 ? lm_x / lr_x : 0, lr_x > 0 ? lm_y / lr_x : 0,
                lr_x > 0 ? rm_x / lr_x : 0, lr_x > 0 ? rm_y / lr_x : 0,
                max_dist_ratio, g.bg_brightness, false
            };
            glint_metas.push_back(gm);

            cv::Mat viz = bgr_img.clone();
            cv::line(viz, g.l_pt, g.r_pt, cv::Scalar(0, 255, 255), 2);
            cv::line(viz, g.l_pt, g.m_pt, cv::Scalar(0, 255, 255), 2);
            cv::line(viz, g.r_pt, g.m_pt, cv::Scalar(0, 255, 255), 2);
            cv::circle(viz, g.l_pt, 3, cv::Scalar(0, 0, 255), -1);
            cv::circle(viz, g.r_pt, 3, cv::Scalar(0, 0, 255), -1);
            cv::circle(viz, g.m_pt, 3, cv::Scalar(255, 0, 0), -1);
            cv::imwrite(glint_dir + "\\" + std::to_string(i) + ".png", viz);
        }

        saveMetaData(metadata_path, pupil_metas, glint_metas);
        
        Logger::info() << "Processed and saved: " << filename;
        
        if (++count >= max_images) break;

    } while (FindNextFile(hFind, &fd) != 0);
    FindClose(hFind);
}

// 第二阶段：交互式人工筛选
void manualReviewData(Cfg& cfg) {
    std::string output_folder = cfg["collect_glint"]["output_folder"].as<std::string>();
    std::string win_name = "Data Collector Review";
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    bool exit_early = false;

    for (const auto& entry : fs::directory_iterator(output_folder)) {
        if (!entry.is_directory()) continue;
        
        std::string img_folder = entry.path().string();
        std::string yml_path = img_folder + "\\metadata.yml";
        if (!fs::exists(yml_path)) continue;

        // 1. 读取现有的 Metadata 到结构体中，同时提取已有的 is_valid 状态
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
            pm.is_valid       = (int)(*it)["is_valid"] != 0; // 加载原有状态
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
            gm.is_valid       = (int)(*it)["is_valid"] != 0; // 加载原有状态
            glints.push_back(gm);
        }
        fs_read.release(); // 释放对 metadata.yml 的占用，允许后续重新写入

        // 2. 加载图片路径（用序号强制对齐，修复 fs::directory_iterator 顺序错乱导致的选中错位）
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
                
                // 绘制交互状态 UI
                std::string mode_str = (mode == 0) ? "MODE: PUPIL" : "MODE: GLINT";
                cv::putText(display_img, mode_str + "[" + std::to_string(current_idx+1) + "/" + std::to_string(current_total) + "]", 
                            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 0), 2);
                
                if (is_selected) {
                    cv::rectangle(display_img, cv::Point(0,0), cv::Point(display_img.cols-1, display_img.rows-1), cv::Scalar(0, 255, 0), 10);
                    cv::putText(display_img, "SELECTED", cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 3);
                } else {
                    cv::putText(display_img, "REJECTED (Space to Keep)", cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                }
            } else {
                display_img = cv::Mat::zeros(600, 800, CV_8UC3);
                cv::putText(display_img, "NO DATA IN THIS CATEGORY", cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            }

            cv::imshow(win_name, display_img);
            int key = cv::waitKey(0);

            switch (key) {
                case ' ': // Space: Toggle selection
                    if (current_total > 0) {
                        if (mode == 0) pupils[current_idx].is_valid = !pupils[current_idx].is_valid;
                        else glints[current_idx].is_valid = !glints[current_idx].is_valid;
                    }
                    break;
                case '\t': // Tab: Switch mode
                    mode = 1 - mode;
                    break;
                case 'a': // A / Left Arrow (Alternative to navigate inside category)
                    if (mode == 0 && p_idx > 0) p_idx--;
                    if (mode == 1 && g_idx > 0) g_idx--;
                    break;
                case 'd': // D / Right Arrow
                    if (mode == 0 && p_idx < (int)pupil_imgs.size() - 1) p_idx++;
                    if (mode == 1 && g_idx < (int)glint_imgs.size() - 1) g_idx++;
                    break;
                case 13: // Enter: Next Image
                    next_image = true;
                    break;
                case 27: // Esc: Exit review early
                    next_image = true;
                    exit_early = true;
                    break;
            }
        }

        // 3. 复用现有的 saveMetaData 进行保存，替代原来复杂且容易格式损坏的节点重写
        saveMetaData(yml_path, pupils, glints);

        if (exit_early) return;
    }
}

int main() {
    Cfg cfg;
    
    // 步骤 1：以宽松参数运行检测，生成可视化切片和包含原始特征的 YAML 元数据文件
    Logger::info() << "Phase 1: Starting Data Collection and Visualization Generation...";
    processAndSaveData(cfg);
    
    // 步骤 2：启动 OpenCV GUI 监听键盘，进行人工复核
    Logger::info() << "Phase 2: Starting Manual Review Interface...";
    Logger::info() << "Controls: [Space] Toggle Selection | [Tab] Switch Category | [A/D] Prev/Next in Category | [Enter] Next Image";
    manualReviewData(cfg);
    
    Logger::info() << "Data collection and review completed.";
    return 0;
}