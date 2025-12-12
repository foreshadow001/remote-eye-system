/**
 * calculate_params.cpp
 * 1. 读取 YAML 配置获取宽松范围定义和分类条件。
 * 2. 读取 CSV，根据 YAML 定义的区间动态收集样本数据。
 * 3. 计算 Side2Side 和 Side2Mid 推荐超参数 (Min-ext, Max+ext)。
 * 4. 可视化散点分布 (高清分辨率)。
 * 5. 将结果写回 YAML 文件。
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

using namespace std;

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
 * 计算推荐区间
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
    // 文字位置调整到线沿上方
    cv::putText(canvas, buf, cv::Point(roi.x+roi.width-180, y_min-8), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale*0.7, cv::Scalar(0,180,0), 2);

    // Max Line & Text
    cv::line(canvas, cv::Point(roi.x, y_max), cv::Point(roi.x+roi.width, y_max), cv::Scalar(0,0,255), lineThickness);
    sprintf(buf, "Max: %.2f", rr.rec_max);
    // 文字位置调整到线沿上方
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
    double ext_x_min = relaxed["horizontal_pair"]["extention_x_min"].as<double>();
    double ext_x_max = relaxed["horizontal_pair"]["extention_x_max"].as<double>();
    double ext_y_min = relaxed["horizontal_pair"]["extention_y_min"].as<double>();
    double ext_y_max = relaxed["horizontal_pair"]["extention_y_max"].as<double>();

    // 扩展参数 (Side2Mid)
    double ext_lmx_min = relaxed["middle_point"]["extention_lm_x/lr_x_min"].as<double>();
    double ext_lmx_max = relaxed["middle_point"]["extention_lm_x/lr_x_max"].as<double>();
    double ext_lmy_min = relaxed["middle_point"]["extention_lm_y/lr_x_min"].as<double>();
    double ext_lmy_max = relaxed["middle_point"]["extention_lm_y/lr_x_max"].as<double>();
    double ext_rmx_min = relaxed["middle_point"]["extention_rm_x/lr_x_min"].as<double>();
    double ext_rmx_max = relaxed["middle_point"]["extention_rm_x/lr_x_max"].as<double>();
    double ext_rmy_min = relaxed["middle_point"]["extention_rm_y/lr_x_min"].as<double>();
    double ext_rmy_max = relaxed["middle_point"]["extention_rm_y/lr_x_max"].as<double>();

    // 获取条件区间列表
    auto raw_conditions = relaxed["middle_point"]["conditions"].as<vector<vector<double>>>();
    int num_categories = raw_conditions.size();
    
    vector<pair<double, double>> ranges;
    for(const auto& row : raw_conditions) {
        if(row.size() >= 2) ranges.push_back({row[0], row[1]});
    }

    // 2. 准备数据容器
    vector<double> all_lr_x, all_lr_y;
    map<int, vector<double>> lmx_stats, lmy_stats, rmx_stats, rmy_stats;

    // 3. 读取 CSV
    ifstream file(csv_path);
    if (!file.is_open()) {
        cerr << "Cannot open CSV: " << csv_path << endl;
        return -1;
    }
    string line;
    getline(file, line); // header

    while (getline(file, line)) {
        if (line.empty()) continue;
        auto tokens = split(line, ',');
        if (tokens.size() < 8) continue;

        Record rec;
        try {
            rec.lx = stod(tokens[2]); rec.ly = stod(tokens[3]);
            rec.rx = stod(tokens[4]); rec.ry = stod(tokens[5]);
            rec.mx = stod(tokens[6]); rec.my = stod(tokens[7]);
        } catch (...) { continue; }

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

        // 动态分类逻辑
        int category = -1;
        for (int i = 0; i < ranges.size(); ++i) {
            double r_min = ranges[i].first;
            double r_max = ranges[i].second;
            bool match = false;
            // 最后一组包含上界
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

    // 4. 计算与可视化准备 (恢复到高清分辨率)
    int row_h = 500; // 更高的高度
    int col_w = 600; // 更宽的宽度
    cv::Mat viz = cv::Mat::zeros(row_h * (num_categories + 1), col_w * 4, CV_8UC3);
    cv::bitwise_not(viz, viz); // 白底

    cout << "Processing " << num_categories << " categories with HD visualization..." << endl;

    // --- Step A: Side2Side (Global) ---
    RangeResult rr_lr_x = processData(all_lr_x, ext_x_min, ext_x_max, false);
    RangeResult rr_lr_y = processData(all_lr_y, ext_y_min, ext_y_max, false);
    
    // 绘制 Global (占用第0行，前两列，宽度拉伸)
    visualize(viz, all_lr_x, "Global lr_x (Absolute Distance)", cv::Rect(0, 0, col_w*2, row_h), cv::Scalar(180,130,70), rr_lr_x);
    visualize(viz, all_lr_y, "Global lr_y (Absolute Distance)", cv::Rect(col_w*2, 0, col_w*2, row_h), cv::Scalar(180,130,70), rr_lr_y);

    // --- Step B: Side2Mid (Per Category) ---
    vector<vector<double>> output_conditions;

    for (int i = 0; i < num_categories; ++i) {
        RangeResult lmx = processData(lmx_stats[i], ext_lmx_min, ext_lmx_max, true);
        RangeResult lmy = processData(lmy_stats[i], ext_lmy_min, ext_lmy_max, true);
        RangeResult rmx = processData(rmx_stats[i], ext_rmx_min, ext_rmx_max, true);
        RangeResult rmy = processData(rmy_stats[i], ext_rmy_min, ext_rmy_max, true);

        output_conditions.push_back({
            ranges[i].first, ranges[i].second,
            lmx.rec_min, lmx.rec_max,
            lmy.rec_min, lmy.rec_max,
            rmx.rec_min, rmx.rec_max,
            rmy.rec_min, rmy.rec_max
        });

        // 可视化
        int r = i + 1;
        string suffix = " [Range: " + to_string((int)ranges[i].first) + "-" + to_string((int)ranges[i].second) + "]";
        
        visualize(viz, lmx_stats[i], "lm_x/lr_x" + suffix, cv::Rect(0*col_w, r*row_h, col_w, row_h), cv::Scalar(200,100,0), lmx);
        visualize(viz, lmy_stats[i], "lm_y/lr_x" + suffix, cv::Rect(1*col_w, r*row_h, col_w, row_h), cv::Scalar(200,100,0), lmy);
        visualize(viz, rmx_stats[i], "rm_x/lr_x" + suffix, cv::Rect(2*col_w, r*row_h, col_w, row_h), cv::Scalar(200,100,0), rmx);
        visualize(viz, rmy_stats[i], "rm_y/lr_x" + suffix, cv::Rect(3*col_w, r*row_h, col_w, row_h), cv::Scalar(200,100,0), rmy);
    }

    // 5. 保存结果
    string img_path = output_folder + "\\hyperparameter_distribution_hd.png";
    cv::imwrite(img_path, viz);
    cout << "HD Visualization saved to: " << img_path << endl;

    // 写入 YAML
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_x_min", rr_lr_x.rec_min);
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_x_max", rr_lr_x.rec_max);
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_y_min", rr_lr_y.rec_min);
    cfg.setScalar("glint_hyperparameter.horizontal_pair.lr_y_max", rr_lr_y.rec_max);
    cfg.setVector2D("glint_hyperparameter.middle_point.conditions", output_conditions);
    cfg.save();
    cout << "YAML updated successfully." << endl;

    // 预览 (缩放显示)
    cv::Mat preview;
    cv::resize(viz, preview, cv::Size(), 0.5, 0.5); // 缩小4倍预览
    cv::imshow("HD Results Preview", preview);
    cv::waitKey(0);

    return 0;
}