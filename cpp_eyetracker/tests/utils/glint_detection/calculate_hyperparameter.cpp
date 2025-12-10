/**
 * calculate_params.cpp
 * 读取CSV，计算lr_x/lr_y不同区间的比例分布，绘制可视化图并输出推荐参数。
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "cfg/config.hpp"

using namespace std;

struct Record {
    string img;
    string side;
    double lx, ly, rx, ry, mx, my;
};

// 用于存储计算后的比率
struct Ratios {
    double lm_x_r, lm_y_r;
    double rm_x_r, rm_y_r;
};

// 简单的字符串分割
vector<string> split(const string& s, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// 绘制分布图的辅助函数
void drawDistribution(cv::Mat& canvas, const vector<double>& values, string title, cv::Rect roi, cv::Scalar color) {
    if (values.empty()) return;

    double min_v = *min_element(values.begin(), values.end());
    double max_v = *max_element(values.begin(), values.end());
    
    // 稍微扩大绘图范围
    double plot_min = min_v - 0.1;
    double plot_max = max_v + 0.1;

    // 绘制背景框
    cv::rectangle(canvas, roi, cv::Scalar(240, 240, 240), -1);
    cv::rectangle(canvas, roi, cv::Scalar(200, 200, 200), 1);

    // 绘制标题和范围
    cv::putText(canvas, title, cv::Point(roi.x + 5, roi.y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1);
    char buf[100];
    sprintf(buf, "Min:%.3f Max:%.3f", min_v, max_v);
    cv::putText(canvas, buf, cv::Point(roi.x + 5, roi.y + roi.height - 5), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0,0,0), 1);

    // 绘制散点
    int x_step = roi.width / (values.size() + 1);
    if (x_step < 1) x_step = 1; // 防止重叠过多

    for (size_t i = 0; i < values.size(); ++i) {
        double val = values[i];
        // 归一化到 ROI 高度 (注意 Y 轴向下)
        int y = roi.y + roi.height - (int)((val - plot_min) / (plot_max - plot_min) * (roi.height - 30)) - 15;
        int x = roi.x + (i * (double)roi.width / values.size()); // 均匀分布
        
        cv::circle(canvas, cv::Point(x, y), 2, color, -1);
    }
    
    // 绘制推荐线 (Min - 0.05, Max + 0.05)
    int y_rec_min = roi.y + roi.height - (int)((min_v - 0.05 - plot_min) / (plot_max - plot_min) * (roi.height - 30)) - 15;
    int y_rec_max = roi.y + roi.height - (int)((max_v + 0.05 - plot_min) / (plot_max - plot_min) * (roi.height - 30)) - 15;
    
    cv::line(canvas, cv::Point(roi.x, y_rec_min), cv::Point(roi.x+roi.width, y_rec_min), cv::Scalar(0, 255, 0), 1); // 下限绿线
    cv::line(canvas, cv::Point(roi.x, y_rec_max), cv::Point(roi.x+roi.width, y_rec_max), cv::Scalar(0, 0, 255), 1); // 上限红线
}

int main() {
    Cfg cfg;

    string output_folder = cfg["collect_glint"]["output_folder"].as<string>();

    string csv_path = output_folder + "\\" + "glint_data.csv"; // 假设这是修正后的文件
    ifstream file(csv_path);
    if (!file.is_open()) {
        cerr << "Could not open " << csv_path << endl;
        return -1;
    }

    string line;
    getline(file, line); // 跳过表头

    // 存储三个类别的比率数据
    // Key 0: lr_y < 2
    // Key 1: 2 <= lr_y < 3
    // Key 2: 3 <= lr_y <= 5
    map<int, vector<double>> lmx_stats, lmy_stats, rmx_stats, rmy_stats;

    while (getline(file, line)) {
        if (line.empty()) continue;
        auto tokens = split(line, ',');
        if (tokens.size() < 8) continue;

        Record rec;
        rec.lx = stod(tokens[2]); rec.ly = stod(tokens[3]);
        rec.rx = stod(tokens[4]); rec.ry = stod(tokens[5]);
        rec.mx = stod(tokens[6]); rec.my = stod(tokens[7]);

        // 计算基础距离
        double lr_x = std::abs(rec.lx - rec.rx);
        double lr_y = std::abs(rec.ly - rec.ry);
        
        double lm_x = rec.mx - rec.lx;
        double lm_y = rec.my - rec.ly;
        double rm_x = rec.rx - rec.mx;
        double rm_y = rec.my - rec.ry;

        // ========= 核心逻辑复现：标准化处理 =========
        // "for simplicity, we only consider the case where left is lower than right"
        // 原代码逻辑：如果 left.y < right.y (左点在图像上方)，则交换 lm 和 rm 的定义
        if (rec.ly < rec.ry) {
            double temp_lm_x = lm_x;
            double temp_lm_y = lm_y;
            lm_x = rm_x;
            lm_y = rm_y;
            rm_x = temp_lm_x;
            rm_y = temp_lm_y;
        }

        // 分类
        int category = -1;
        if (lr_y < 2) category = 0;
        else if (lr_y >= 2 && lr_y < 3) category = 1;
        else if (lr_y >= 3 && lr_y <= 5) category = 2;

        if (category != -1 && lr_x > 0) {
            lmx_stats[category].push_back(lm_x / lr_x);
            lmy_stats[category].push_back(lm_y / lr_x); // 注意原代码中是除以 lr_x
            rmx_stats[category].push_back(rm_x / lr_x);
            rmy_stats[category].push_back(rm_y / lr_x);
        }
    }

    // ========= 可视化与输出 =========
    // 创建画布: 3行 (categories), 4列 (params)
    int row_h = 200, col_w = 300;
    cv::Mat viz = cv::Mat::zeros(row_h * 3, col_w * 4, CV_8UC3);
    cv::bitwise_not(viz, viz); // 白底

    string cat_names[] = {"lr_y < 2", "2 <= lr_y < 3", "3 <= lr_y <= 5"};

    cout << "Recommended Hyperparameters (Min-0.05, Max+0.05):\n" << endl;

    for (int i = 0; i < 3; i++) {
        if (lmx_stats[i].empty()) continue;

        cout << "Category [" << cat_names[i] << "]:" << endl;

        // 辅助宏用于处理绘制和输出
        auto process = [&](vector<double>& vec, string name, int col_idx) {
            if (vec.empty()) return;
            double min_v = *min_element(vec.begin(), vec.end());
            double max_v = *max_element(vec.begin(), vec.end());
            
            cout << "  " << name << ": [" << (min_v - 0.05) << ", " << (max_v + 0.05) << "]" 
                 << " (Actual: " << min_v << " to " << max_v << ")" << endl;

            cv::Rect roi(col_idx * col_w, i * row_h, col_w - 10, row_h - 10);
            drawDistribution(viz, vec, name + " (" + cat_names[i] + ")", roi, cv::Scalar(255, 0, 0));
        };

        process(lmx_stats[i], "lm_x / lr_x", 0);
        process(lmy_stats[i], "lm_y / lr_x", 1);
        process(rmx_stats[i], "rm_x / lr_x", 2);
        process(rmy_stats[i], "rm_y / lr_x", 3);

        cout << endl;
    }

    cv::imwrite(output_folder + "\\hyperparameter_distribution.png", viz);
    cout << "Distribution chart saved to '" << output_folder + "\\hyperparameter_distribution.png'" << endl;
    
    // 显示图片 (可选)
    // cv::imshow("Distributions", viz);
    // cv::waitKey(0);

    return 0;
}