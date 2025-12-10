/**
 * calculate_params.cpp
 * 读取CSV，计算 side2side (lr_x, lr_y) 和 side2mid 的各项超参数分布
 * 绘制可视化图并输出推荐参数。
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <cmath>
#include <iomanip> // 用于控制输出精度
#include <opencv2/opencv.hpp>
#include "cfg/config.hpp"

using namespace std;

struct Record {
    string img;
    string side;
    double lx, ly, rx, ry, mx, my;
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
    
    // 稍微扩大绘图范围，防止点画在边界上
    double plot_min = min_v - (max_v - min_v) * 0.1;
    double plot_max = max_v + (max_v - min_v) * 0.1;
    if (plot_min == plot_max) { plot_min -= 1.0; plot_max += 1.0; } // 防止单一值导致除零

    // 绘制背景框
    cv::rectangle(canvas, roi, cv::Scalar(245, 245, 245), -1);
    cv::rectangle(canvas, roi, cv::Scalar(200, 200, 200), 1);

    // 绘制标题
    cv::putText(canvas, title, cv::Point(roi.x + 5, roi.y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,0,0), 1, cv::LINE_AA);

    // 绘制范围文字 (Min/Max)
    char buf[100];
    sprintf(buf, "Min:%.2f Max:%.2f", min_v, max_v);
    cv::putText(canvas, buf, cv::Point(roi.x + 5, roi.y + roi.height - 8), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(50,50,50), 1, cv::LINE_AA);

    // 绘制散点
    for (size_t i = 0; i < values.size(); ++i) {
        double val = values[i];
        // 归一化 Y 轴 (数值越大，Y坐标越小，因为图像原点在左上)
        // 留出上下 25 像素的 margin
        int plot_h = roi.height - 50;
        int y = roi.y + roi.height - 25 - (int)((val - plot_min) / (plot_max - plot_min) * plot_h);
        
        // X 轴均匀分布
        int x = roi.x + 10 + (int)(i * (double)(roi.width - 20) / values.size()); 
        
        cv::circle(canvas, cv::Point(x, y), 2, color, -1);
    }
    
    // 绘制推荐线 (Min - 0.05, Max + 0.05)
    auto calc_y = [&](double v) {
        return roi.y + roi.height - 25 - (int)((v - plot_min) / (plot_max - plot_min) * (roi.height - 50));
    };

    int y_rec_min = calc_y(min_v - 0.05);
    int y_rec_max = calc_y(max_v + 0.05);
    
    // 限制绘制范围在 ROI 内
    y_rec_min = std::clamp(y_rec_min, roi.y, roi.y + roi.height);
    y_rec_max = std::clamp(y_rec_max, roi.y, roi.y + roi.height);

    cv::line(canvas, cv::Point(roi.x, y_rec_min), cv::Point(roi.x+roi.width, y_rec_min), cv::Scalar(0, 180, 0), 1); // 下限绿线
    cv::line(canvas, cv::Point(roi.x, y_rec_max), cv::Point(roi.x+roi.width, y_rec_max), cv::Scalar(0, 0, 200), 1); // 上限红线
}

int main() {
    Cfg cfg; // 确保命名空间正确

    string output_folder = cfg["collect_glint"]["output_folder"].as<string>(); // 注意：原代码通常是 test_glint
    string csv_path = output_folder + "\\glint_data.csv"; // 读取脚本一生成的文件

    ifstream file(csv_path);
    if (!file.is_open()) {
        cerr << "Could not open " << csv_path << endl;
        return -1;
    }

    string line;
    getline(file, line); // 跳过表头

    // 存储 Side2Mid 的比率数据 (按类别)
    map<int, vector<double>> lmx_stats, lmy_stats, rmx_stats, rmy_stats;
    
    // 存储 Side2Side 的绝对距离数据 (全局)
    vector<double> global_lr_x, global_lr_y;

    while (getline(file, line)) {
        if (line.empty()) continue;
        auto tokens = split(line, ',');
        if (tokens.size() < 8) continue;

        Record rec;
        rec.lx = stod(tokens[2]); rec.ly = stod(tokens[3]);
        rec.rx = stod(tokens[4]); rec.ry = stod(tokens[5]);
        rec.mx = stod(tokens[6]); rec.my = stod(tokens[7]);

        // 1. 计算 Side2Side 基础距离
        double lr_x = std::abs(rec.lx - rec.rx);
        double lr_y = std::abs(rec.ly - rec.ry);

        // 收集全局 Side2Side 数据
        global_lr_x.push_back(lr_x);
        global_lr_y.push_back(lr_y);

        // 2. 计算 Side2Mid 相对数据
        double lm_x = rec.mx - rec.lx;
        double lm_y = rec.my - rec.ly;
        double rm_x = rec.rx - rec.mx;
        double rm_y = rec.my - rec.ry;

        // "for simplicity, we only consider the case where left is lower than right"
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
        // 注意：如果你的样本里 lr_y > 5，这里 category 为 -1，不会计入 side2mid 统计，但会计入 global_lr_y 统计

        if (category != -1 && lr_x > 0) {
            lmx_stats[category].push_back(lm_x / lr_x);
            lmy_stats[category].push_back(lm_y / lr_x);
            rmx_stats[category].push_back(rm_x / lr_x);
            rmy_stats[category].push_back(rm_y / lr_x);
        }
    }

    // ========= 可视化与输出 =========
    // 创建画布: 4行 (1行全局 + 3行类别), 4列 (params)
    int row_h = 220; 
    int col_w = 320;
    cv::Mat viz = cv::Mat::zeros(row_h * 4, col_w * 4, CV_8UC3);
    cv::bitwise_not(viz, viz); // 白底

    cout << std::fixed << std::setprecision(4);
    cout << "========================================================" << endl;
    cout << "           Recommended Hyperparameters Result           " << endl;
    cout << "          (Logic: Min - 0.05, Max + 0.05)               " << endl;
    cout << "========================================================" << endl << endl;

    // --- 1. 处理 Side2Side (第 0 行) ---
    cout << "[Global] Side2Side Parameters (Horizontal Pair):" << endl;
    
    // 辅助 Lambda：处理并绘制
    auto process_and_draw = [&](vector<double>& vec, string name, int row, int col, cv::Scalar color) {
        if (vec.empty()) return;
        double min_v = *min_element(vec.begin(), vec.end());
        double max_v = *max_element(vec.begin(), vec.end());
        
        cout << "  " << name << ": [" << (min_v - 0.05) << ", " << (max_v + 0.05) << "]" 
             << " (Raw: " << min_v << " ~ " << max_v << ")" << endl;

        cv::Rect roi(col * col_w, row * row_h, col_w - 10, row_h - 10);
        drawDistribution(viz, vec, name, roi, color);
    };

    // 绘制 lr_x (放在第0行，第1列)
    process_and_draw(global_lr_x, "lr_x (Dist)", 0, 1, cv::Scalar(0, 100, 0));
    // 绘制 lr_y (放在第0行，第2列)
    process_and_draw(global_lr_y, "lr_y (Dist)", 0, 2, cv::Scalar(0, 100, 0));
    
    cout << endl;

    // --- 2. 处理 Side2Mid (第 1-3 行) ---
    string cat_names[] = {"lr_y < 2", "2 <= lr_y < 3", "3 <= lr_y <= 5"};

    for (int i = 0; i < 3; i++) {
        if (lmx_stats[i].empty()) continue;

        cout << "[Category] " << cat_names[i] << " :" << endl;

        // 绘制在 i+1 行 (因为第0行被占用了)
        int current_row = i + 1;
        cv::Scalar color(200, 0, 0); // 蓝色

        process_and_draw(lmx_stats[i], "lm_x / lr_x", current_row, 0, color);
        process_and_draw(lmy_stats[i], "lm_y / lr_x", current_row, 1, color);
        process_and_draw(rmx_stats[i], "rm_x / lr_x", current_row, 2, color);
        process_and_draw(rmy_stats[i], "rm_y / lr_x", current_row, 3, color);

        cout << endl;
    }

    // 保存结果
    // 确保 output_folder 存在，否则保存到当前目录
    string save_path = "hyperparameter_distribution.png";
    if (!output_folder.empty()) {
         save_path = output_folder + "\\hyperparameter_distribution.png";
    }
    
    cv::imwrite(save_path, viz);
    cout << "Chart saved to: " << save_path << endl;

    // 显示 (可选)
    cv::namedWindow("Distributions", cv::WINDOW_NORMAL);
    cv::resizeWindow("Distributions", 1200, 800);
    cv::imshow("Distributions", viz);
    cv::waitKey(0);

    return 0;
}