/**
 * calculate_hyperparameter.cpp
 * 功能：
 * 1. 遍历收集目录，读取所有 is_valid == true 的 metadata.yml 数据。
 * 2. 统计 Pupil 和 Glint 各项参数的数据边界 (Data Min/Max)。
 * 3. 使用硬编码的容差 (Extensions) 计算推荐边界 (Recommended Min/Max)。
 * 4. 生成 3 张高清分布图：pupil_stats.png, glint_global_stats.png, glint_ratio_stats.png
 * 5. 自动将结果更新至 recommended_specific_hyperparameter 并保存 YAML。
 */

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <cmath>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "cfg/config.hpp"
#include "logger/logger.hpp"

using namespace std;
namespace fs = std::filesystem;

// --- 数据结构 ---
struct RangeResult {
    double rec_min;
    double rec_max;
    double data_min;
    double data_max;
};

// --- 辅助计算函数 ---
RangeResult processData(const vector<double>& vec, double ext_min, double ext_max, double abs_min = -1e9, double abs_max = 1e9) {
    RangeResult r = {0.0, 0.0, 0.0, 0.0};
    if (vec.empty()) return r;

    r.data_min = *min_element(vec.begin(), vec.end());
    r.data_max = *max_element(vec.begin(), vec.end());

    double rec_min = r.data_min - ext_min;
    double rec_max = r.data_max + ext_max;

    r.rec_min = std::max(abs_min, rec_min);
    r.rec_max = std::min(abs_max, rec_max);
    return r;
}

// --- 可视化函数 ---
void visualize(cv::Mat& canvas, const vector<double>& values, const string& title, 
               const cv::Rect& roi, const cv::Scalar& color, const RangeResult& rr) {
    if (values.empty()) return;

    double fontScale = 1.0; 
    int thickness = 2;      
    int pointSize = 4;      
    int lineThickness = 3;  

    cv::rectangle(canvas, roi, cv::Scalar(245, 245, 245), -1);
    cv::rectangle(canvas, roi, cv::Scalar(200, 200, 200), 2);

    cv::putText(canvas, title, cv::Point(roi.x + 15, roi.y + 35), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0,0,0), thickness);

    char buf[100];
    sprintf(buf, "Data: %.2f~%.2f", rr.data_min, rr.data_max);
    cv::putText(canvas, buf, cv::Point(roi.x + 15, roi.y + roi.height - 15), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale * 0.7, cv::Scalar(80,80,80), 2);

    double plot_min = std::min(rr.data_min, rr.rec_min);
    double plot_max = std::max(rr.data_max, rr.rec_max);
    
    double range = plot_max - plot_min;
    if (range < 1e-5) range = 1.0; 

    plot_min -= range * 0.1;
    plot_max += range * 0.1;

    auto get_y = [&](double v) {
        double ratio = (v - plot_min) / (plot_max - plot_min);
        return roi.y + roi.height - (int)(ratio * (roi.height - 80)) - 30; 
    };

    for (size_t i = 0; i < values.size(); ++i) {
        int y = get_y(values[i]);
        int x = roi.x + 20 + (i * (double)(roi.width - 40) / values.size());
        cv::circle(canvas, cv::Point(x, y), pointSize, color, -1);
    }

    int y_min = get_y(rr.rec_min);
    int y_max = get_y(rr.rec_max);

    cv::line(canvas, cv::Point(roi.x, y_min), cv::Point(roi.x+roi.width, y_min), cv::Scalar(0,200,0), lineThickness);
    sprintf(buf, "Min: %.2f", rr.rec_min);
    cv::putText(canvas, buf, cv::Point(roi.x+roi.width-150, y_min-8), cv::FONT_HERSHEY_SIMPLEX, fontScale*0.6, cv::Scalar(0,180,0), 2);

    cv::line(canvas, cv::Point(roi.x, y_max), cv::Point(roi.x+roi.width, y_max), cv::Scalar(0,0,255), lineThickness);
    sprintf(buf, "Max: %.2f", rr.rec_max);
    cv::putText(canvas, buf, cv::Point(roi.x+roi.width-150, y_max-8), cv::FONT_HERSHEY_SIMPLEX, fontScale*0.6, cv::Scalar(0,0,200), 2);
}

int main() {
    Cfg cfg;
    string output_folder = cfg["collect_glint"]["output_folder"].as<string>();

    // ==========================================
    // 1. 硬编码扩展容差 (Extensions) - 代替从 YAML 读取
    // ==========================================
    CfgNode ext = cfg["recommended_specific_hyperparameter"]["ext"];
    // Pupil
    const double ext_area       = 10.0;
    const double ext_points     = 0.0;
    const double ext_axis       = 2.0;
    const double ext_ratio      = 0.1;
    const double ext_solidity   = 0.05;
    const double ext_fit_ratio  = 0.05;
    const double ext_residual   = 0.05;
    const double ext_darkness   = 5.0;
    const double ext_offset     = 5.0; // 新增：自适应阈值偏移量的容差

    // Glint Global
    const double ext_lr_x       = 1.5;
    const double ext_lr_y       = 0.5;
    const double ext_bg_bright  = 5.0;
    const double ext_exclusion  = 0.2;

    // Glint Ratio (Middle point)
    const double ext_ratio_xy   = 0.05; 

    // 获取分组边界条件 (保持原逻辑：通过 lr_y 进行分段)
    auto raw_conditions = cfg["relaxed_specific_hyperparameter"]["glint"]["middle_point"]["conditions"].as<vector<vector<double>>>();
    vector<pair<double, double>> ranges;
    for(const auto& row : raw_conditions) {
        if(row.size() >= 2) ranges.push_back({row[0], row[1]});
    }

    // ==========================================
    // 2. 数据容器
    // ==========================================
    vector<double> p_areas, p_points, p_axes, p_ratios, p_solidities, p_fits, p_residuals, p_darknesses, p_offsets;
    vector<double> g_lr_xs, g_lr_ys, g_brights, g_exclusions;
    
    map<int, vector<double>> lmx_stats, lmy_stats, rmx_stats, rmy_stats;

    // ==========================================
    // 3. 遍历读取 metadata.yml
    // ==========================================
    for (const auto& entry : fs::directory_iterator(output_folder)) {
        if (!entry.is_directory()) continue;
        string yml_path = entry.path().string() + "\\metadata.yml";
        if (!fs::exists(yml_path)) continue;

        cv::FileStorage fs(yml_path, cv::FileStorage::READ);
        if (!fs.isOpened()) continue;

        cv::FileNode p_nodes = fs["Pupils"];
        for (auto it = p_nodes.begin(); it != p_nodes.end(); ++it) {
            if ((int)(*it)["is_valid"] != 0) {
                p_areas.push_back((double)(*it)["area"]);
                p_points.push_back((double)(int)(*it)["contour_points"]);
                p_axes.push_back((double)(float)(*it)["major_axis"]);
                p_ratios.push_back((double)(float)(*it)["axis_ratio"]);
                p_solidities.push_back((double)(*it)["solidity"]);
                p_fits.push_back((double)(*it)["fit_ratio"]);
                p_residuals.push_back((double)(float)(*it)["avg_residual"]);
                p_darknesses.push_back((double)(*it)["darkness"]);
            }
        }

        cv::FileNode g_nodes = fs["Glints"];
        for (auto it = g_nodes.begin(); it != g_nodes.end(); ++it) {
            if ((int)(*it)["is_valid"] != 0) {
                double lr_y = (double)(*it)["lr_y"];
                double lr_x = (double)(*it)["lr_x"];
                
                g_lr_ys.push_back(lr_y);
                g_lr_xs.push_back(lr_x);
                g_brights.push_back((double)(*it)["bg_brightness"]);
                g_exclusions.push_back((double)(*it)["max_exclusion_radius_ratio"]);

                // 寻找匹配的分组
                int category = -1;
                for (int i = 0; i < ranges.size(); ++i) {
                    if (lr_y >= ranges[i].first && lr_y < ranges[i].second + 1e-5) {
                        category = i; break;
                    }
                }

                if (category != -1) {
                    lmx_stats[category].push_back((double)(*it)["lm_x_ratio"]);
                    lmy_stats[category].push_back((double)(*it)["lm_y_ratio"]);
                    rmx_stats[category].push_back((double)(*it)["rm_x_ratio"]);
                    rmy_stats[category].push_back((double)(*it)["rm_y_ratio"]);
                }
            }
        }
        fs.release();
    }

    if (p_areas.empty() && g_lr_xs.empty()) {
        Logger::error() << "No valid data found in the output folder! Please annotate some data first.";
        return -1;
    }

    // ==========================================
    // 4. 计算与可视化
    // ==========================================
    int row_h = 400, col_w = 400;

    // --- A. Pupil 统计 ---
    cv::Mat canvas_pupil(row_h * 2, col_w * 4, CV_8UC3, cv::Scalar(255, 255, 255));
    auto rr_area = processData(p_areas, ext_area, ext_area, 0.0);
    auto rr_pts  = processData(p_points, ext_points, 999, 5.0); // 最小轮廓点保底5
    auto rr_axis = processData(p_axes, 0, ext_axis, 0.0);       // 长轴只关心最大值
    auto rr_rat  = processData(p_ratios, 0, ext_ratio, 0.0);
    auto rr_sol  = processData(p_solidities, ext_solidity, 0, 0.0, 1.0);
    auto rr_fit  = processData(p_fits, ext_fit_ratio, 0, 0.0, 1.0);
    auto rr_res  = processData(p_residuals, 0, ext_residual, 0.0);
    auto rr_dark = processData(p_darknesses, 0, ext_darkness, 0.0, 255.0);

    visualize(canvas_pupil, p_areas, "Area", cv::Rect(0*col_w, 0, col_w, row_h), cv::Scalar(200, 100, 100), rr_area);
    visualize(canvas_pupil, p_points, "Contour Pts", cv::Rect(1*col_w, 0, col_w, row_h), cv::Scalar(100, 200, 100), rr_pts);
    visualize(canvas_pupil, p_axes, "Major Axis", cv::Rect(2*col_w, 0, col_w, row_h), cv::Scalar(100, 100, 200), rr_axis);
    visualize(canvas_pupil, p_ratios, "Axis Ratio", cv::Rect(3*col_w, 0, col_w, row_h), cv::Scalar(150, 150, 50), rr_rat);
    
    visualize(canvas_pupil, p_solidities, "Solidity", cv::Rect(0*col_w, row_h, col_w, row_h), cv::Scalar(50, 150, 150), rr_sol);
    visualize(canvas_pupil, p_fits, "Fit Ratio", cv::Rect(1*col_w, row_h, col_w, row_h), cv::Scalar(150, 50, 150), rr_fit);
    visualize(canvas_pupil, p_residuals, "Avg Residual", cv::Rect(2*col_w, row_h, col_w, row_h), cv::Scalar(200, 150, 100), rr_res);
    visualize(canvas_pupil, p_darknesses, "Darkness", cv::Rect(3*col_w, row_h, col_w, row_h), cv::Scalar(100, 150, 200), rr_dark);

    cv::imwrite(output_folder + "\\pupil_stats.png", canvas_pupil);

    // --- B. Glint Global 统计 (保持不变) ---
    cv::Mat canvas_g_global(row_h, col_w * 4, CV_8UC3, cv::Scalar(255, 255, 255));
    auto rr_lr_y = processData(g_lr_ys, ext_lr_y, ext_lr_y, 0.0);
    auto rr_lr_x = processData(g_lr_xs, ext_lr_x, ext_lr_x, 0.0);
    auto rr_bri  = processData(g_brights, 0, ext_bg_bright, 0.0, 255.0);
    auto rr_excl = processData(g_exclusions, 0, ext_exclusion, 0.0);

    visualize(canvas_g_global, g_lr_ys, "LR_Y", cv::Rect(0*col_w, 0, col_w, row_h), cv::Scalar(200, 100, 100), rr_lr_y);
    visualize(canvas_g_global, g_lr_xs, "LR_X", cv::Rect(1*col_w, 0, col_w, row_h), cv::Scalar(100, 200, 100), rr_lr_x);
    visualize(canvas_g_global, g_brights, "BG Brightness", cv::Rect(2*col_w, 0, col_w, row_h), cv::Scalar(100, 100, 200), rr_bri);
    visualize(canvas_g_global, g_exclusions, "Exclusion Ratio", cv::Rect(3*col_w, 0, col_w, row_h), cv::Scalar(150, 150, 50), rr_excl);

    cv::imwrite(output_folder + "\\glint_global_stats.png", canvas_g_global);

    // --- C. Glint Ratio 分组统计 (保持不变) ---
    int num_categories = ranges.size();
    cv::Mat canvas_g_ratio(num_categories * row_h, col_w * 4, CV_8UC3, cv::Scalar(255, 255, 255));
    vector<vector<double>> output_conditions;

    for (int i = 0; i < num_categories; ++i) {
        auto lmx = processData(lmx_stats[i], ext_ratio_xy, ext_ratio_xy, 0.0, 1.0);
        auto lmy = processData(lmy_stats[i], ext_ratio_xy, ext_ratio_xy, 0.0, 1.0);
        auto rmx = processData(rmx_stats[i], ext_ratio_xy, ext_ratio_xy, 0.0, 1.0);
        auto rmy = processData(rmy_stats[i], ext_ratio_xy, ext_ratio_xy, 0.0, 1.0);

        output_conditions.push_back({
            ranges[i].first, ranges[i].second,
            lmx.rec_min, lmx.rec_max, lmy.rec_min, lmy.rec_max,
            rmx.rec_min, rmx.rec_max, rmy.rec_min, rmy.rec_max
        });

        int y_off = i * row_h;
        stringstream title_suffix;
        title_suffix << " [" << (int)ranges[i].first << "~" << (int)ranges[i].second << "]";

        visualize(canvas_g_ratio, lmx_stats[i], "LM_X" + title_suffix.str(), cv::Rect(0*col_w, y_off, col_w, row_h), cv::Scalar(150, 50, 150), lmx);
        visualize(canvas_g_ratio, lmy_stats[i], "LM_Y" + title_suffix.str(), cv::Rect(1*col_w, y_off, col_w, row_h), cv::Scalar(150, 50, 150), lmy);
        visualize(canvas_g_ratio, rmx_stats[i], "RM_X" + title_suffix.str(), cv::Rect(2*col_w, y_off, col_w, row_h), cv::Scalar(50, 150, 150), rmx);
        visualize(canvas_g_ratio, rmy_stats[i], "RM_Y" + title_suffix.str(), cv::Rect(3*col_w, y_off, col_w, row_h), cv::Scalar(50, 150, 150), rmy);
    }
    
    cv::imwrite(output_folder + "\\glint_ratio_stats.png", canvas_g_ratio);

    // ==========================================
    // 5. 将计算结果更新至 YAML 并保存
    // ==========================================
    string prefix_pupil = "recommended_specific_hyperparameter.pupil.searchPupilInROI.";
    cfg.setScalar<int>(prefix_pupil + "kMinPupilArea", rr_area.rec_min);
    cfg.setScalar<int>(prefix_pupil + "kMaxPupilArea", rr_area.rec_max);
    cfg.setScalar<int>(prefix_pupil + "kMinPupilContourPoints", (int)rr_pts.rec_min);
    cfg.setScalar<double>(prefix_pupil + "kMaxPupilAxis", rr_axis.rec_max);
    cfg.setScalar<double>(prefix_pupil + "kMaxAxisRatio", rr_rat.rec_max);
    cfg.setScalar<double>(prefix_pupil + "kMinSolidity", rr_sol.rec_min);
    cfg.setScalar<double>(prefix_pupil + "kMinFitRatio", rr_fit.rec_min);
    cfg.setScalar<double>(prefix_pupil + "kMaxAvgResidual", rr_res.rec_max);
    cfg.setScalar<double>(prefix_pupil + "kMaxDarkness", rr_dark.rec_max);
    // cfg.setScalar(prefix_pupil + "kAdaptiveThreshOffset", rr_offset.rec_max);
    // cfg.setScalar(prefix_pupil + "kAdaptiveThreshMax", rr_dark.rec_max);

    string prefix_glint = "recommended_specific_hyperparameter.glint.";
    cfg.setScalar<double>(prefix_glint + "isPupilNearby.kExclusionRadiusRatio", rr_excl.rec_max);
    cfg.setScalar<double>(prefix_glint + "checkAndPushGlintGeometry.kBrightnessThreshold", rr_bri.rec_max);
    cfg.setScalar<double>(prefix_glint + "horizontal_pair.lr_y_min", rr_lr_y.rec_min);
    cfg.setScalar<double>(prefix_glint + "horizontal_pair.lr_y_max", rr_lr_y.rec_max);
    cfg.setScalar<double>(prefix_glint + "horizontal_pair.lr_x_min", rr_lr_x.rec_min);
    cfg.setScalar<double>(prefix_glint + "horizontal_pair.lr_x_max", rr_lr_x.rec_max);
    cfg.setVector2D<double>(prefix_glint + "middle_point.conditions", output_conditions);

    cfg.save();
    Logger::info() << "Hyperparameters updated and saved successfully to config.yaml";

    cout << "Finished! All stats images saved to: " << output_folder << endl;
    return 0;
}