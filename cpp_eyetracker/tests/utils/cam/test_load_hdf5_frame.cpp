// test_load_hdf5_frame.cpp — interactive HDF5 frame viewer (5x2 grid + enlarged, arrow keys navigate)
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <string>
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
    bool loaded;
};

static vector<CamInfo> g_cams;
static int g_global_frame = 0;  // current global frame index
static int g_max_frame = 0;     // upper bound (from sentry)
static string g_sentry_root;

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

            hsize_t v_start[1] = {(hsize_t)c.frame_offset}, v_count[1] = {1};
            H5::DataSpace v_mem(1, v_count);
            H5::DataSpace v_file = valid_ds.getSpace();
            v_file.selectHyperslab(H5S_SELECT_SET, v_count, v_start);
            valid_ds.read(&c.valid, H5::PredType::NATIVE_UINT8, v_mem, v_file);
            if (!c.valid) continue;

            c.raw = cv::Mat(g_cam_h, g_cam_w, CV_8UC1);
            hsize_t r_start[3] = {(hsize_t)c.frame_offset, 0, 0};
            hsize_t r_count[3] = {1, (hsize_t)g_cam_h, (hsize_t)g_cam_w};
            H5::DataSpace r_mem(3, r_count);
            H5::DataSpace r_file = raw_ds.getSpace();
            r_file.selectHyperslab(H5S_SELECT_SET, r_count, r_start);
            raw_ds.read(c.raw.data, H5::PredType::NATIVE_UINT8, r_mem, r_file);
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
                try { cv::cvtColor(g_cams[i].raw, bgr, cv::COLOR_BayerRG2BGR); }
                catch (...) { cv::cvtColor(g_cams[i].raw, bgr, cv::COLOR_GRAY2BGR); }
                double sc = min((double)g_thumb_w / bgr.cols, (double)g_thumb_h / bgr.rows);
                int dw = (int)(bgr.cols * sc), dh = (int)(bgr.rows * sc);
                cv::Mat rz; cv::resize(bgr, rz, {dw, dh});
                cell = cv::Mat(g_thumb_h, g_thumb_w, CV_8UC3, cv::Scalar(0,0,0));
                rz.copyTo(cell(cv::Rect((g_thumb_w-dw)/2, (g_thumb_h-dh)/2, dw, dh)));
            } else {
                cell = cv::Mat(g_thumb_h, g_thumb_w, CV_8UC3, cv::Scalar(0,0,0));
                cv::putText(cell, g_cams[i].sn + " (N/A)", {4, g_thumb_h-20},
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0,0,255), 1);
            }
            // SN label at bottom center
            int bl; cv::Size ts = cv::getTextSize(g_cams[i].sn, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &bl);
            cv::putText(cell, g_cams[i].sn, {(g_thumb_w - ts.width)/2, g_thumb_h - 5},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 3);
            cv::putText(cell, g_cams[i].sn, {(g_thumb_w - ts.width)/2, g_thumb_h - 5},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
            cell.copyTo(canvas(roi));
            if (i == g_enlarged) cv::rectangle(canvas, roi, {0,255,0}, 2);
        } else {
            canvas(roi) = cv::Scalar(0,0,0);
        }
    }

    // --- Right panel: enlarged view ---
    cv::Rect right(g_right_x, 0, g_right_w, g_win_h);
    if (g_enlarged >= 0 && g_enlarged < n && g_cams[g_enlarged].loaded) {
        cv::Mat bgr;
        try { cv::cvtColor(g_cams[g_enlarged].raw, bgr, cv::COLOR_BayerRG2BGR); }
        catch (...) { cv::cvtColor(g_cams[g_enlarged].raw, bgr, cv::COLOR_GRAY2BGR); }
        double sc = min((double)g_right_w / bgr.cols, (double)g_win_h / bgr.rows);
        int dw = (int)(bgr.cols * sc), dh = (int)(bgr.rows * sc);
        cv::Mat rz; cv::resize(bgr, rz, {dw, dh});
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

        string fi = "Frame:" + to_string(g_global_frame) + "  " + g_cams[g_enlarged].sn;
        cv::putText(canvas, fi, {off_x + 10, off_y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 3);
        cv::putText(canvas, fi, {off_x + 10, off_y + 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
    } else {
        canvas(right) = cv::Scalar(0,0,0);
    }

    // Separator line
    cv::line(canvas, {g_left_w, 0}, {g_left_w, g_win_h}, {60,60,60}, 2);

    // Watermark + crosshair
    int hx = g_right_x + 10, hy = g_win_h - 25;
    cv::putText(canvas, "Frame " + to_string(g_global_frame) + "/" + to_string(g_max_frame) + "  [A][D] +/-1  [W][S] +/-100  [ESC] quit",
                {hx, hy}, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(140,140,140), 1);
    int cx = g_right_x + g_right_w/2, cy = g_win_h/2;
    cv::line(canvas, {cx-20, cy}, {cx+20, cy}, {100,100,100}, 1);
    cv::line(canvas, {cx, cy-20}, {cx, cy+20}, {100,100,100}, 1);
}

int main(int argc, char* argv[]) {
    vector<string> roots;
    vector<string> sns;
    try {
        Cfg cfg("cfg/capture.yaml");
        // Append participant_id to each root
        string participant_id;
        try { participant_id = cfg["capture"]["participant_id"].as<string>(); } catch (...) { participant_id = "P001"; }
        auto& loader = cfg["loader"];
        roots = loader["participant_root"].as<vector<string>>();
        for (auto& r : roots) r += "/" + participant_id;
        sns = loader["cam_indices"].as<vector<string>>();
        g_cam_w = loader["cam_width"].as<int>();
        g_cam_h = loader["cam_height"].as<int>();
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
        else if (key == 2555904 || key == 'd')  g_global_frame = min(g_max_frame - 1, g_global_frame + 1);   // RIGHT/D: +1
        else if (key == 2490368 || key == 'w')  g_global_frame = min(g_max_frame - 1, g_global_frame + 100); // UP/W: +100
        else if (key == 2621440 || key == 's')  g_global_frame = max(0, g_global_frame - 100);               // DOWN/S: -100
        if (g_global_frame != prev) loadFrame(g_global_frame);
    }

    cv::destroyAllWindows();
    return 0;
}
