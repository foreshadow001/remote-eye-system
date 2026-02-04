/**
 * calculate_params.cpp
 * * 功能：
 * 1. 统计 Glint 区域像素强度，计算稳健平均值，更新 filter 阈值。
 * 2. 统计几何参数 (LR, LM, RM)，根据 YAML 配置的区间进行分类统计。
 * 3. 生成高清可视化图表：
 * - geometric_stats.png: 包含全局分布及所有分类子项的详细分布 (动态排版)。
 * - glint_stats.png: 包含不同 Level 菱形区域的像素均值分布。
 * 4. 自动更新 YAML 配置文件。
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <cmath>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include "cfg/config.hpp"
// #include "utils/shared_calculations.hpp"

using namespace std;
using namespace gazeestimation;

// --- 数据结构 ---

struct Record {
    double lx, ly, rx, ry, mx, my;
};

struct RangeResult {
    double rec_min;
    double rec_max;
    double data_min;
    double data_max;
};

// --- 辅助函数 ---

vector<string> split(const string& s, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) tokens.push_back(token);
    return tokens;
}

/**
 * 计算推荐区间 (Side2Side / Side2Mid)
 */
RangeResult processData(const vector<double>& vec, double ext_min, double ext_max, bool is_ratio) {
    RangeResult r = {0.0, 0.0, 0.0, 0.0};
    if (vec.empty()) return r;

    r.data_min = *min_element(vec.begin(), vec.end());
    r.data_max = *max_element(vec.begin(), vec.end());

    double rec_min = r.data_min - ext_min;
    double rec_max = r.data_max + ext_max;

    // 施加边界约束
    r.rec_min = std::max(0.0, rec_min);
    if (is_ratio) {
        r.rec_max = std::min(1.0, rec_max);
    } else {
        r.rec_max = rec_max;
    }
    return r;
}

/**
 * 计算旋转45度的菱形区域稳健平均值 (Trimmed Diamond Average)
 */
static inline double calculateDiamondAverage(const cv::Mat& img, double cx, double cy, int level, int k) {
    if (img.empty()) return 0.0;

    int center_x = std::round(cx);
    int center_y = std::round(cy);
    int max_dist = level - 1;

    int x0 = std::max(0, center_x - max_dist);
    int y0 = std::max(0, center_y - max_dist);
    int x1 = std::min(img.cols - 1, center_x + max_dist);
    int y1 = std::min(img.rows - 1, center_y + max_dist);

    std::vector<uchar> pixels;
    pixels.reserve((max_dist + 1) * (max_dist + 1) * 2);

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            int dist = std::abs(x - center_x) + std::abs(y - center_y);
            if (dist <= max_dist) {
                pixels.push_back(img.at<uchar>(y, x));
            }
        }
    }

    int n = pixels.size();
    if (n <= 2 * k) return 0.0; 

    std::sort(pixels.begin(), pixels.end());

    double sum = 0.0;
    for (int i = k; i < n - k; ++i) {
        sum += pixels[i];
    }

    return sum / (n - 2 * k);
}

// --- 可视化函数 (Requested Version) ---

/**
 * 绘制分布图 (高清分辨率设置)
 */
void visualize(cv::Mat& canvas, const vector<double>& values, const string& title, 
               const cv::Rect& roi, const cv::Scalar& color, const RangeResult& rr) {
    if (values.empty()) return;

    // --- 高清绘图参数 ---
    double fontScale = 1.1; // 更大的字体
    int thickness = 3;      // 更粗的线条
    int pointSize = 5;      // 更大的数据点
    int lineThickness = 4;  // 推荐线粗细

    // 绘制背景
    cv::rectangle(canvas, roi, cv::Scalar(250, 250, 250), -1);
    cv::rectangle(canvas, roi, cv::Scalar(200, 200, 200), 2);

    // 标题
    cv::putText(canvas, title, cv::Point(roi.x + 15, roi.y + 45), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0,0,0), thickness);

    // 数据范围文本 (底部)
    char buf[100];
    sprintf(buf, "Data Range: %.2f ~ %.2f", rr.data_min, rr.data_max);
    cv::putText(canvas, buf, cv::Point(roi.x + 15, roi.y + roi.height - 15), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale * 0.7, cv::Scalar(80,80,80), 2);

    // Y轴映射 Lambda
    double plot_min = std::min(rr.data_min, rr.rec_min);
    double plot_max = std::max(rr.data_max, rr.rec_max);
    
    // 特殊处理 Glint 显示范围 (0~255) 以免范围过小显示异常
    if (title.find("Outline") != string::npos) {
        plot_min = 0;
        plot_max = std::max(260.0, plot_max);
    }

    double range = plot_max - plot_min;
    if (range < 1e-5) range = 1.0; 

    // 稍微扩宽上下视野 (上下各留 10%)
    plot_min -= range * 0.1;
    plot_max += range * 0.1;

    auto get_y = [&](double v) {
        double ratio = (v - plot_min) / (plot_max - plot_min);
        // 上下留出足够的边距用于显示文字 (Top: ~60px, Bottom: ~40px)
        return roi.y + roi.height - (int)(ratio * (roi.height - 100)) - 40; 
    };

    // 绘制散点
    for (size_t i = 0; i < values.size(); ++i) {
        int y = get_y(values[i]);
        // X轴均匀分布，左右留边
        int x = roi.x + 25 + (i * (double)(roi.width - 50) / values.size());
        cv::circle(canvas, cv::Point(x, y), pointSize, color, -1);
    }

    // 绘制推荐线 (绿线Min, 红线Max)
    int y_min = get_y(rr.rec_min);
    int y_max = get_y(rr.rec_max);

    // Min Line & Text
    cv::line(canvas, cv::Point(roi.x, y_min), cv::Point(roi.x+roi.width, y_min), cv::Scalar(0,200,0), lineThickness);
    sprintf(buf, "Min: %.2f", rr.rec_min);
    cv::putText(canvas, buf, cv::Point(roi.x+roi.width-180, y_min-8), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale*0.7, cv::Scalar(0,180,0), 2);

    // Max Line & Text
    cv::line(canvas, cv::Point(roi.x, y_max), cv::Point(roi.x+roi.width, y_max), cv::Scalar(0,0,255), lineThickness);
    sprintf(buf, "Max: %.2f", rr.rec_max);
    cv::putText(canvas, buf, cv::Point(roi.x+roi.width-180, y_max-8), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale*0.7, cv::Scalar(0,0,200), 2);
}

// --- Main ---

int main() {
    Cfg cfg;

    // 1. 读取配置参数
    string output_folder = cfg["collect_glint"]["output_folder"].as<string>();
    string csv_path = output_folder + "\\glint_data.csv";

    // 扩展参数 (Side2Side)
    auto relaxed = cfg["relaxed_glint_hyperparameter"];
    double ext_x_min = relaxed["horizontal_pair"]["extension_x_min"].as<double>();
    double ext_x_max = relaxed["horizontal_pair"]["extension_x_max"].as<double>();
    double ext_y_min = relaxed["horizontal_pair"]["extension_y_min"].as<double>();
    double ext_y_max = relaxed["horizontal_pair"]["extension_y_max"].as<double>();

    // 扩展参数 (Side2Mid)
    double ext_lmx_min = relaxed["middle_point"]["extension_lm_x/lr_x_min"].as<double>();
    double ext_lmx_max = relaxed["middle_point"]["extension_lm_x/lr_x_max"].as<double>();
    double ext_lmy_min = relaxed["middle_point"]["extension_lm_y/lr_x_min"].as<double>();
    double ext_lmy_max = relaxed["middle_point"]["extension_lm_y/lr_x_max"].as<double>();
    double ext_rmx_min = relaxed["middle_point"]["extension_rm_x/lr_x_min"].as<double>();
    double ext_rmx_max = relaxed["middle_point"]["extension_rm_x/lr_x_max"].as<double>();
    double ext_rmy_min = relaxed["middle_point"]["extension_rm_y/lr_x_min"].as<double>();
    double ext_rmy_max = relaxed["middle_point"]["extension_rm_y/lr_x_max"].as<double>();

    // 扩展参数 (Glint Pixel Extension)
    double ext_outline_2 = relaxed["glint_filter"]["extension_outline_2_value_average_max"].as<double>();
    double ext_outline_3 = relaxed["glint_filter"]["extension_outline_3_value_average_max"].as<double>();
    double ext_outline_4 = relaxed["glint_filter"]["extension_outline_4_value_average_max"].as<double>();

    // 获取条件区间列表
    auto raw_conditions = relaxed["middle_point"]["conditions"].as<vector<vector<double>>>();
    vector<pair<double, double>> ranges;
    for(const auto& row : raw_conditions) {
        if(row.size() >= 2) ranges.push_back({row[0], row[1]});
    }

    // 2. 准备数据容器
    vector<double> all_lr_x, all_lr_y;
    map<int, vector<double>> lmx_stats, lmy_stats, rmx_stats, rmy_stats;
    
    vector<double> outline2_values; // Level 2 (3x3)
    vector<double> outline3_values; // Level 3 (5x5)
    vector<double> outline4_values; // Level 4 (7x7)

    double max_avg_val_level2 = 0.0; 
    double max_avg_val_level3 = 0.0; 
    double max_avg_val_level4 = 0.0; 

    // 3. 读取 CSV 并处理
    ifstream file(csv_path);
    if (!file.is_open()) {
        cerr << "Cannot open CSV: " << csv_path << endl;
        return -1;
    }
    string line;
    getline(file, line); // header

    string last_img_path = "";
    cv::Mat current_img;

    int line_count = 0;
    while (getline(file, line)) {
        if (line.empty()) continue;
        auto tokens = split(line, ',');
        if (tokens.size() < 9) continue;

        Record rec;
        string img_path_str;
        try {
            rec.lx = stod(tokens[2]); rec.ly = stod(tokens[3]);
            rec.rx = stod(tokens[4]); rec.ry = stod(tokens[5]);
            rec.mx = stod(tokens[6]); rec.my = stod(tokens[7]);
            img_path_str = tokens[8];
        } catch (...) { continue; }
        
        line_count++;

        // --- Glint Pixel Logic ---
        img_path_str.erase(0, img_path_str.find_first_not_of(" \r\n\t"));
        img_path_str.erase(img_path_str.find_last_not_of(" \r\n\t") + 1);

        if (img_path_str != last_img_path) {
            current_img = cv::imread(img_path_str, cv::IMREAD_GRAYSCALE);
            last_img_path = img_path_str;
        }

        if (!current_img.empty()) {
            vector<pair<double, double>> points = { {rec.lx, rec.ly}, {rec.rx, rec.ry}, {rec.mx, rec.my} };
            for (const auto& p : points) {
                if (p.first > 0 && p.second > 0) {
                    double val_2 = calculateDiamondAverage(current_img, p.first, p.second, 2, 1);
                    if (val_2 > max_avg_val_level2) max_avg_val_level2 = val_2;
                    outline2_values.push_back(val_2);

                    double val_3 = calculateDiamondAverage(current_img, p.first, p.second, 3, 2);
                    if (val_3 > max_avg_val_level3) max_avg_val_level3 = val_3;
                    outline3_values.push_back(val_3);

                    double val_4 = calculateDiamondAverage(current_img, p.first, p.second, 4, 3);
                    if (val_4 > max_avg_val_level4) max_avg_val_level4 = val_4;
                    outline4_values.push_back(val_4);
                }
            }
        }

        // --- Geometric Logic ---
        double lr_x = std::abs(rec.lx - rec.rx);
        double lr_y = std::abs(rec.ly - rec.ry);
        
        all_lr_x.push_back(lr_x);
        all_lr_y.push_back(lr_y);

        double lm_x = rec.mx - rec.lx;
        double lm_y = rec.my - rec.ly;
        double rm_x = rec.rx - rec.mx;
        double rm_y = rec.my - rec.ry;

        if (rec.ly < rec.ry) {
            std::swap(lm_x, rm_x);
            std::swap(lm_y, rm_y);
        }

        int category = -1;
        for (int i = 0; i < ranges.size(); ++i) {
            double r_min = ranges[i].first;
            double r_max = ranges[i].second;
            bool match = false;
            if (i == ranges.size() - 1) {
                if (lr_y >= r_min && lr_y <= r_max) match = true;
            } else {
                if (lr_y >= r_min && lr_y < r_max) match = true;
            }
            if (match) {
                category = i;
                break;
            }
        }

        if (category != -1 && lr_x > 1e-3) {
            lmx_stats[category].push_back(lm_x / lr_x);
            lmy_stats[category].push_back(lm_y / lr_x);
            rmx_stats[category].push_back(rm_x / lr_x);
            rmy_stats[category].push_back(rm_y / lr_x);
        }
    }
    file.close();

    // ---------------------------------------------------------
    // 可视化阶段
    // ---------------------------------------------------------

    // 布局参数
    int row_h = 500;
    int col_w = 600;
    int num_categories = ranges.size();
    
    // --- 1. Geometric Stats (包含所有类别) ---
    // 总高度 = (Global 1行 + Category N行) * 行高
    cv::Mat canvas_geo((num_categories + 1) * row_h, col_w * 4, CV_8UC3, cv::Scalar(255, 255, 255));

    // A. 绘制 Global Stats (占据第0行)
    // Global LR_X 占前两列 (1200宽), Global LR_Y 占后两列 (1200宽)
    RangeResult rr_lr_x = processData(all_lr_x, ext_x_min, ext_x_max, false);
    RangeResult rr_lr_y = processData(all_lr_y, ext_y_min, ext_y_max, false);

    visualize(canvas_geo, all_lr_x, "Horizontal pair lr_x", 
              cv::Rect(0, 0, col_w * 2, row_h), cv::Scalar(200, 100, 100), rr_lr_x);
    visualize(canvas_geo, all_lr_y, "lr_y", 
              cv::Rect(col_w * 2, 0, col_w * 2, row_h), cv::Scalar(100, 200, 100), rr_lr_y);

    // B. 绘制 Category Stats (占据第1到N行)
    vector<vector<double>> output_conditions;

    for (int i = 0; i < num_categories; ++i) {
        RangeResult lmx = processData(lmx_stats[i], ext_lmx_min, ext_lmx_max, true);
        RangeResult lmy = processData(lmy_stats[i], ext_lmy_min, ext_lmy_max, true);
        RangeResult rmx = processData(rmx_stats[i], ext_rmx_min, ext_rmx_max, true);
        RangeResult rmy = processData(rmy_stats[i], ext_rmy_min, ext_rmy_max, true);

        // 收集参数用于 YAML
        output_conditions.push_back({
            ranges[i].first, ranges[i].second,
            lmx.rec_min, lmx.rec_max,
            lmy.rec_min, lmy.rec_max,
            rmx.rec_min, rmx.rec_max,
            rmy.rec_min, rmy.rec_max
        });

        // 绘制图表
        // 计算当前行的Y坐标偏移
        int current_y = (i + 1) * row_h;
        stringstream title_suffix;
        title_suffix << " [range: " << (int)ranges[i].first << "-" << (int)ranges[i].second << "]";

        // 每个Category一行，4列
        visualize(canvas_geo, lmx_stats[i], "lm_x/lr_x " + title_suffix.str(), 
                  cv::Rect(0 * col_w, current_y, col_w, row_h), cv::Scalar(150, 150, 50), lmx);
        
        visualize(canvas_geo, lmy_stats[i], "lm_y/lr_x " + title_suffix.str(), 
                  cv::Rect(1 * col_w, current_y, col_w, row_h), cv::Scalar(150, 150, 50), lmy);
        
        visualize(canvas_geo, rmx_stats[i], "rm_x/lr_x " + title_suffix.str(), 
                  cv::Rect(2 * col_w, current_y, col_w, row_h), cv::Scalar(50, 150, 150), rmx);
        
        visualize(canvas_geo, rmy_stats[i], "rm_y/lr_x " + title_suffix.str(), 
                  cv::Rect(3 * col_w, current_y, col_w, row_h), cv::Scalar(50, 150, 150), rmy);
    }

    // 保存 Geometric 图
    string geo_save_path = output_folder + "\\geometric_stats.png";
    cv::imwrite(geo_save_path, canvas_geo);
    cout << "Saved HD Geometric Stats (All Categories) to: " << geo_save_path << endl;

    // --- 2. Glint Pixel Stats ---
    
    // 计算最终阈值
    auto calc_final_val = [](double data_max, double ext) {
        return std::min(255.0, data_max + ext);
    };
    double final_outline_2 = calc_final_val(max_avg_val_level2, ext_outline_2);
    double final_outline_3 = calc_final_val(max_avg_val_level3, ext_outline_3);
    double final_outline_4 = calc_final_val(max_avg_val_level4, ext_outline_4);

    // 辅助 lambda: 构造 RangeResult 
    auto make_glint_rr = [](const vector<double>& vec, double rec_max) {
        RangeResult r;
        if (vec.empty()) { r.data_min=0; r.data_max=0; }
        else {
            r.data_min = *min_element(vec.begin(), vec.end());
            r.data_max = *max_element(vec.begin(), vec.end());
        }
        r.rec_min = 0; 
        r.rec_max = rec_max;
        return r;
    };

    // Glint 布局：单行，3列，每列宽1000，高1000
    int g_w = 1000;
    int g_h = 1000;
    cv::Mat canvas_glint(g_h, g_w * 3, CV_8UC3, cv::Scalar(255, 255, 255));

    visualize(canvas_glint, outline2_values, "Outline 2 (Level 2)", 
              cv::Rect(0, 0, g_w, g_h), cv::Scalar(100, 100, 255), 
              make_glint_rr(outline2_values, final_outline_2));

    visualize(canvas_glint, outline3_values, "Outline 3 (Level 3)", 
              cv::Rect(g_w, 0, g_w, g_h), cv::Scalar(100, 255, 100), 
              make_glint_rr(outline3_values, final_outline_3));

    visualize(canvas_glint, outline4_values, "Outline 4 (Level 4)", 
              cv::Rect(g_w * 2, 0, g_w, g_h), cv::Scalar(255, 100, 100), 
              make_glint_rr(outline4_values, final_outline_4));

    // 保存 Glint 图
    string glint_save_path = output_folder + "\\glint_stats.png";
    cv::imwrite(glint_save_path, canvas_glint);
    cout << "Saved HD Glint Stats to: " << glint_save_path << endl;

    // ---------------------------------------------------------
    // 写入 YAML
    // ---------------------------------------------------------

    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_x_min", rr_lr_x.rec_min);
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_x_max", rr_lr_x.rec_max);
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_y_min", rr_lr_y.rec_min);
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_y_max", rr_lr_y.rec_max);
    cfg.setVector2D("glint_hyperparameter.middle_point.conditions", output_conditions);

    cfg.setScalar("glint_hyperparameter.glint_filter.outline_2_value_average_max", final_outline_2);
    cfg.setScalar("glint_hyperparameter.glint_filter.outline_3_value_average_max", final_outline_3);
    cfg.setScalar("glint_hyperparameter.glint_filter.outline_4_value_average_max", final_outline_4);

    cfg.save();
    cout << "YAML updated successfully." << endl;

    cv::Mat geo_preview;
    cv::resize(canvas_geo, geo_preview, cv::Size(), 0.5, 0.5);
    cv::imshow("Geometric Stats (Preview)", geo_preview);

    cv::Mat glint_preview;
    cv::resize(canvas_glint, glint_preview, cv::Size(), 0.5, 0.5);
    cv::imshow("Glint Stats (Preview)", glint_preview);

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}