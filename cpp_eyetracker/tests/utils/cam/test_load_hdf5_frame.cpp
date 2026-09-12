// test_load_hdf5_frame.cpp — interactive HDF5 frame viewer (5x2 grid + enlarged, arrow keys navigate)
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <H5Cpp.h>

#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;

static int g_win_w = 1600, g_win_h = 800;
static int g_left_w, g_right_x, g_right_w, g_thumb_w, g_thumb_h;
static int g_enlarged = -1;
static int g_cam_w = 2448, g_cam_h = 2048;
static int g_capacity = 2000;

struct CamInfo {
    string sn;
    string root;       // per-camera participant_root
    int chunk_idx;
    int frame_offset;  // within chunk
    cv::Mat raw;       // current frame data
    uint8_t valid;
    bool occluded;     // valid=0 (已写入区域) = 遮挡判定置 0
    bool loaded;
    double gt[3] = {0, 0, 0};   // gaze_target (40772280 相机系)
    int occ_status = -1;        // 判定状态 (-1=旧文件无数据集; 0正常 1停用 2查询失败 3下发失败 4对账失配)
    double occ_err[2] = {0, 0}; // 双臂对账误差 mm (NaN=未对账)
};

static vector<CamInfo> g_cams;
static int g_global_frame = 0;  // current global frame index
static int g_max_frame = 0;     // upper bound (from sentry)
static double g_gain = 1.0;     // 显示增益 (画面实际很暗; [L]加亮 [D]变暗)
static string g_sentry_root;
static int g_core_frames = 100;          // = ceil(fps×record_time)
static int64_t g_frames_per_arm = 25000; // = num_targets_per_arm × core_frames

// 帧号 → (臂, 目标序号) — 复用 capture_with_M5Stack armRecorded() 口径
static void frameToTarget(int64_t frame, string& arm, int& target_idx) {
    if (frame < g_frames_per_arm) { arm = "upper"; target_idx = (int)(frame / g_core_frames); }
    else { arm = "lower"; target_idx = (int)((frame - g_frames_per_arm) / g_core_frames); }
}

static void updateLayout() {
    g_left_w = g_win_h * 2 / 5;
    g_right_x = g_left_w;
    g_right_w = g_win_w - g_left_w;
    g_thumb_w = g_left_w / 2;
    g_thumb_h = g_win_h / 5;
}

static void onMouse(int ev, int x, int y, int, void*) {
    if (ev != cv::EVENT_LBUTTONDOWN || x >= g_left_w) return;
    int col = x / g_thumb_w, row = y / g_thumb_h;
    int idx = row * 2 + col;
    if (idx >= 0 && idx < (int)g_cams.size())
        g_enlarged = (g_enlarged == idx) ? -1 : idx;
}

static bool loadFrame(int global_idx) {
    bool any_loaded = false;
    for (auto& c : g_cams) {
        c.chunk_idx = global_idx / g_capacity;
        c.frame_offset = global_idx % g_capacity;
        c.loaded = false;

        stringstream ss;
        ss << c.root << "/" << c.sn << "/" << setw(4) << setfill('0') << c.chunk_idx << ".h5";
        if (!fs::exists(ss.str())) continue;

        try {
            H5::H5File f(ss.str(), H5F_ACC_RDONLY);
            H5::DataSet raw_ds = f.openDataSet("raw_image");
            H5::DataSet valid_ds = f.openDataSet("valid");
            H5::DataSet gt_ds; bool has_gt = false;
            try { gt_ds = f.openDataSet("gaze_target"); has_gt = true; } catch (const H5::Exception&) {}

            hsize_t v_start[1] = {(hsize_t)c.frame_offset}, v_count[1] = {1};
            H5::DataSpace v_mem(1, v_count);
            H5::DataSpace v_file = valid_ds.getSpace();
            v_file.selectHyperslab(H5S_SELECT_SET, v_count, v_start);
            valid_ds.read(&c.valid, H5::PredType::NATIVE_UINT8, v_mem, v_file);
            // valid=0 且在已写入范围内 → 遮挡判定置 0 (仍加载图像供查看);
            // 未写入区域 (>= sentry) 不在本函数出现 (调用方以 g_max_frame 限界)
            c.occluded = !c.valid;

            c.raw = cv::Mat(g_cam_h, g_cam_w, CV_8UC1);
            hsize_t r_start[3] = {(hsize_t)c.frame_offset, 0, 0};
            hsize_t r_count[3] = {1, (hsize_t)g_cam_h, (hsize_t)g_cam_w};
            H5::DataSpace r_mem(3, r_count);
            H5::DataSpace r_file = raw_ds.getSpace();
            r_file.selectHyperslab(H5S_SELECT_SET, r_count, r_start);
            raw_ds.read(c.raw.data, H5::PredType::NATIVE_UINT8, r_mem, r_file);
            // gaze_target: 该帧录制对应的定位工装尖 (40772280 相机系, 每录恒定)
            // 旧数据可能缺该数据集 → 独立 try, 不影响图像加载 (显示 0)
            if (has_gt) {
                try {
                    hsize_t g_start[2] = {(hsize_t)c.frame_offset, 0};
                    hsize_t g_count[2] = {1, 3};
                    H5::DataSpace g_mem(2, g_count);
                    H5::DataSpace g_file = gt_ds.getSpace();
                    g_file.selectHyperslab(H5S_SELECT_SET, g_count, g_start);
                    gt_ds.read(c.gt, H5::PredType::NATIVE_DOUBLE, g_mem, g_file);
                } catch (const H5::Exception&) {}
            }
            // occ_status / occ_check_err: 判定状态与双臂对账误差 (旧文件缺 → 保持 -1)
            try {
                H5::DataSet st_ds = f.openDataSet("occ_status");
                hsize_t s_start[1] = {(hsize_t)c.frame_offset}, s_count[1] = {1};
                H5::DataSpace s_mem(1, s_count), s_file = st_ds.getSpace();
                s_file.selectHyperslab(H5S_SELECT_SET, s_count, s_start);
                uint8_t st = 0; st_ds.read(&st, H5::PredType::NATIVE_UINT8, s_mem, s_file);
                c.occ_status = (int)st;
                H5::DataSet er_ds = f.openDataSet("occ_check_err");
                hsize_t e_start[2] = {(hsize_t)c.frame_offset, 0}, e_count[2] = {1, 2};
                H5::DataSpace e_mem(2, e_count), e_file = er_ds.getSpace();
                e_file.selectHyperslab(H5S_SELECT_SET, e_count, e_start);
                float ev[2]; er_ds.read(ev, H5::PredType::NATIVE_FLOAT, e_mem, e_file);
                c.occ_err[0] = ev[0]; c.occ_err[1] = ev[1];
            } catch (const H5::Exception&) {}
            c.loaded = true;
            any_loaded = true;
        } catch (const H5::Exception&) {}
    }
    return any_loaded;
}

static void render(cv::Mat& canvas) {
    canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
    int n = (int)g_cams.size();

    // --- Left panel: 5x2 thumbnails (match recording UI layout) ---
    for (int i = 0; i < 10; ++i) {
        int r = i / 2, c = i % 2;
        cv::Rect roi(c * g_thumb_w, r * g_thumb_h, g_thumb_w, g_thumb_h);

        if (i < n) {
            cv::Mat cell;
            if (g_cams[i].loaded) {
                cv::Mat bgr;
                // 彩色相机实际排列为 BayerBG (按 RG 解读会 R/B 互换: 黄→蓝)
                try { cv::cvtColor(g_cams[i].raw, bgr, cv::COLOR_BayerBG2BGR); }
                catch (...) { cv::cvtColor(g_cams[i].raw, bgr, cv::COLOR_GRAY2BGR); }
                double sc = min((double)g_thumb_w / bgr.cols, (double)g_thumb_h / bgr.rows);
                int dw = (int)(bgr.cols * sc), dh = (int)(bgr.rows * sc);
                cv::Mat rz; cv::resize(bgr, rz, {dw, dh});
                if (g_gain != 1.0) rz.convertTo(rz, -1, g_gain, 0);   // 显示增益
                cell = cv::Mat(g_thumb_h, g_thumb_w, CV_8UC3, cv::Scalar(0,0,0));
                rz.copyTo(cell(cv::Rect((g_thumb_w-dw)/2, (g_thumb_h-dh)/2, dw, dh)));
            } else {
                cell = cv::Mat(g_thumb_h, g_thumb_w, CV_8UC3, cv::Scalar(0,0,0));
                cv::putText(cell, g_cams[i].sn + " (N/A)", {4, g_thumb_h-20},
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0,0,255), 1);
            }
            // SN label at bottom center (被遮挡相机: 红色)
            int bl; cv::Size ts = cv::getTextSize(g_cams[i].sn, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &bl);
            cv::Scalar sn_col = g_cams[i].occluded ? cv::Scalar(0,0,255) : cv::Scalar(255,255,255);
            cv::putText(cell, g_cams[i].sn, {(g_thumb_w - ts.width)/2, g_thumb_h - 5},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 3);
            cv::putText(cell, g_cams[i].sn, {(g_thumb_w - ts.width)/2, g_thumb_h - 5},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, sn_col, 1);
            // OCC 角标 (左上角, 红): 本目标该相机被遮挡 (valid=0)
            if (g_cams[i].occluded) {
                cv::putText(cell, "OCC", {4, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(0,0,0), 3);
                cv::putText(cell, "OCC", {4, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(0,0,255), 1);
            }
            cell.copyTo(canvas(roi));
            if (i == g_enlarged) cv::rectangle(canvas, roi, {0,255,0}, 2);   // 选中: 绿框
            if (g_cams[i].occluded) cv::rectangle(canvas, roi, {0,0,255}, 2); // 遮挡: 红框 (覆盖选中框)
        } else {
            canvas(roi) = cv::Scalar(0,0,0);
        }
    }

    // --- Right panel: enlarged view ---
    cv::Rect right(g_right_x, 0, g_right_w, g_win_h);
    if (g_enlarged >= 0 && g_enlarged < n && g_cams[g_enlarged].loaded) {
        cv::Mat bgr;
        try { cv::cvtColor(g_cams[g_enlarged].raw, bgr, cv::COLOR_BayerBG2BGR); }   // BayerBG (见缩略图处注释)
        catch (...) { cv::cvtColor(g_cams[g_enlarged].raw, bgr, cv::COLOR_GRAY2BGR); }
        double sc = min((double)g_right_w / bgr.cols, (double)g_win_h / bgr.rows);
        int dw = (int)(bgr.cols * sc), dh = (int)(bgr.rows * sc);
        cv::Mat rz; cv::resize(bgr, rz, {dw, dh});
        if (g_gain != 1.0) rz.convertTo(rz, -1, g_gain, 0);           // 显示增益
        canvas(right) = cv::Scalar(0,0,0);
        rz.copyTo(canvas(cv::Rect(g_right_x + (g_right_w-dw)/2, (g_win_h-dh)/2, dw, dh)));

        // Chunk & offset (top-right) + Frame info (top-left) — draw on canvas AFTER copyTo
        auto& ec = g_cams[g_enlarged];
        int off_x = g_right_x + (g_right_w-dw)/2, off_y = (g_win_h-dh)/2;
        string cfo = "Chunk:" + to_string(ec.chunk_idx) + "  Offset:" + to_string(ec.frame_offset);
        int bl; cv::Size cs = cv::getTextSize(cfo, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &bl);
        cv::putText(canvas, cfo, {off_x + dw - cs.width - 10, off_y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 3);
        cv::putText(canvas, cfo, {off_x + dw - cs.width - 10, off_y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);

        // Frame 信息行: 帧号 + 臂/目标 (armRecorded 口径) + SN + VALID/OCCLUDED + 判定状态
        string arm; int tgt_i = 0;
        frameToTarget(g_global_frame, arm, tgt_i);
        static const char* kOccStName[] = {"OK", "DISABLED", "QUERY_FAIL", "DELIVERY_FAIL", "MISMATCH"};
        string fi = "Frame:" + to_string(g_global_frame) + "  " + arm + " #" + to_string(tgt_i + 1)
                    + "  " + g_cams[g_enlarged].sn
                    + (g_cams[g_enlarged].occluded ? "  OCCLUDED" : "  VALID");
        {   // 判定状态徽标 (≠0 黄色提示; -1 旧文件) + 双臂对账误差
            auto& ec2 = g_cams[g_enlarged];
            if (ec2.occ_status > 0)
                fi += string("  [") + kOccStName[min(ec2.occ_status, 4)] + "]";
            else if (ec2.occ_status < 0)
                fi += "  [legacy]";
            if (ec2.occ_status >= 0 && ec2.occ_err[0] == ec2.occ_err[0]) {   // 非 NaN
                char eb[80]; snprintf(eb, sizeof(eb), "  chk U:%.1f L:%.1fmm", ec2.occ_err[0], ec2.occ_err[1]);
                fi += eb;
            }
        }
        cv::Scalar fi_col = g_cams[g_enlarged].occluded ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0);
        cv::putText(canvas, fi, {off_x + 10, off_y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 3);
        cv::putText(canvas, fi, {off_x + 10, off_y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, fi_col, 2);

        // gaze target 坐标 (本相机系; 每录恒定, 显示当前帧所属录制的值)
        { auto& ec = g_cams[g_enlarged];
          char gb[160];
          snprintf(gb, sizeof(gb), "Gaze target (cam frame): [%.4f, %.4f, %.4f] m",
                   ec.gt[0], ec.gt[1], ec.gt[2]);
          cv::putText(canvas, gb, {off_x + 10, off_y + 62},
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 3);
          cv::putText(canvas, gb, {off_x + 10, off_y + 62},
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200,200,0), 2); }
    } else {
        canvas(right) = cv::Scalar(0,0,0);
    }

    // Separator line
    cv::line(canvas, {g_left_w, 0}, {g_left_w, g_win_h}, {60,60,60}, 2);

    // Watermark + crosshair
    int hx = g_right_x + 10, hy = g_win_h - 25;
    { // gaze target 摘要 (未放大时也可见; 取首个已加载相机 — 各相机同值)
      const CamInfo* gc = nullptr;
      for (auto& c : g_cams) if (c.loaded) { gc = &c; break; }
      if (gc) {
          char gb[160];
          snprintf(gb, sizeof(gb), "GT(cam): [%.3f, %.3f, %.3f] m",
                   gc->gt[0], gc->gt[1], gc->gt[2]);
          cv::putText(canvas, gb, {hx, hy - 20}, cv::FONT_HERSHEY_SIMPLEX, 0.35,
                      cv::Scalar(200,200,0), 1, cv::LINE_AA);
      } }
    cv::putText(canvas, "Frame " + to_string(g_global_frame) + "/" + to_string(g_max_frame)
                + "  [<-][->] +/-1  [W][S] +/-100  [L] brighter  [D] darker"
                + "  gain x" + ([](){ char b[16]; snprintf(b,sizeof(b),"%.2f",g_gain); return string(b); })(),
                {hx, hy}, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(140,140,140), 1);
    int cx = g_right_x + g_right_w/2, cy = g_win_h/2;
    cv::line(canvas, {cx-20, cy}, {cx+20, cy}, {100,100,100}, 1);
    cv::line(canvas, {cx, cy-20}, {cx, cy+20}, {100,100,100}, 1);
}

int main(int argc, char* argv[]) {
    vector<string> roots;
    vector<string> sns;
    try {
        auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg").string();
        Cfg cfg(cfg_dir+"/capture.yaml");
        // Append participant_id to each root
        string participant_id;
        try { participant_id = cfg["capture"]["participant_id"].as<string>(); } catch (...) { participant_id = "P001"; }
        auto& loader = cfg["loader"];
        roots = loader["participant_root"].as<vector<string>>();
        for (auto& r : roots) r += "/" + participant_id;
        sns = loader["cam_indices"].as<vector<string>>();
        g_cam_w = loader["cam_width"].as<int>();
        g_cam_h = loader["cam_height"].as<int>();
        // 帧号 → 目标映射参数 (与 capture_with_M5Stack 同口径)
        try {
            auto& c = cfg["capture"];
            double fps = c["fps"].as<double>(), rt = c["record_time"].as<double>();
            int ntpa = c["num_targets_per_arm"].as<int>();
            g_core_frames = (int)ceil(fps * rt);
            g_frames_per_arm = (int64_t)ntpa * g_core_frames;
        } catch (...) {}   // 缺键保持默认 (100/25000)
    } catch (...) {
        cerr << "Cannot read cfg/capture.yaml loader node." << endl;
        return 1;
    }

    g_sentry_root = roots[0];
    if (sns.size() != roots.size()) {
        cerr << "cam_indices and participant_root must have same length" << endl;
        return 1;
    }

    // Read sentry for max frame count
    string sp = g_sentry_root + "/sentry.txt";
    if (fs::exists(sp)) {
        ifstream in(sp);
        int ci, fo; in >> ci >> fo;
        g_max_frame = ci * g_capacity + fo;
    }
    if (g_max_frame == 0) { cerr << "No data found (sentry frame count is 0)." << endl; return 1; }
    cout << "Max frame: " << g_max_frame << " (chunk=" << (g_max_frame/g_capacity) << " offset=" << (g_max_frame%g_capacity) << ")" << endl;

    for (size_t i = 0; i < sns.size(); ++i) {
        CamInfo ci;
        ci.sn = sns[i];
        ci.root = roots[i];
        g_cams.push_back(ci);
    }

    cout << g_cams.size() << " cameras loaded. Starting at frame 0." << endl;

    cv::namedWindow("HDF5 Frame Viewer", cv::WINDOW_NORMAL);
    cv::resizeWindow("HDF5 Frame Viewer", g_win_w, g_win_h);
    updateLayout();
    cv::setMouseCallback("HDF5 Frame Viewer", onMouse);

    loadFrame(g_global_frame);

    while (true) {
        cv::Mat canvas;
        render(canvas);
        cv::imshow("HDF5 Frame Viewer", canvas);

        int key = cv::waitKeyEx(30);  // 30ms poll — responsive to mouse clicks
        if (key < 0) continue;
        if (key == 'q' || key == 27) break;
        int prev = g_global_frame;
        if (key == 2424832 || key == 'a')       g_global_frame = max(0, g_global_frame - 1);    // LEFT/A: -1
        else if (key == 2555904)                g_global_frame = min(g_max_frame - 1, g_global_frame + 1);   // RIGHT: +1
        else if (key == 2490368 || key == 'w')  g_global_frame = min(g_max_frame - 1, g_global_frame + 100); // UP/W: +100
        else if (key == 2621440 || key == 's')  g_global_frame = max(0, g_global_frame - 100);               // DOWN/S: -100
        else if (key == 'l' || key == 'L')      g_gain = min(32.0, g_gain * 1.25);              // L: 加亮
        else if (key == 'd' || key == 'D')      g_gain = max(0.05, g_gain / 1.25);              // D: 变暗
        if (g_global_frame != prev) loadFrame(g_global_frame);
    }

    cv::destroyAllWindows();
    return 0;
}
