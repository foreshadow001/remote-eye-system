// ================== 网络与系统核心头文件 (必须放在最前面) ==================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <pthread.h>
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define closesocket close
#endif

// ================== 标准库与第三方库 ==================
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <functional>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== 全局状态 ==================
atomic<bool> global_running{true};

// ================== TCP 通信 ==================
SOCKET g_arm_sock = INVALID_SOCKET;
string g_ubuntu_ip;
int g_arm_port = 0;
mutex g_arm_mtx;
string g_current_arm = "upper";  // 't' to toggle

// ================== 相机上下文 (同 test_calib_images) ==================
struct CameraContext {
    string sn;
    BaslerCamera cam{""};
    bool is_mono = true;

    thread capture_thread;
    thread copy_thread;
    atomic<bool> running{true};

    cv::Mat latest_frame;
    FrameMeta latest_meta;
    mutex frame_mtx;

    queue<pair<Pylon::CBaslerUniversalGrabResultPtr, FrameMeta>> copy_queue;
    mutex copy_mtx;
    condition_variable copy_cv;

    explicit CameraContext(string cam_sn) : sn(cam_sn), cam(cam_sn) {}
};

vector<shared_ptr<CameraContext>> cam_ctxs;
atomic<int> g_enlarged_cam{-1};

// UI layout
int g_win_w = 1224, g_win_h = 1024;
int g_left_w = 0, g_right_x = 0, g_right_w = 0;
int g_thumb_w = 0, g_thumb_h = 0;
string g_calib_save_dir;
string g_mapping_file;

// ================== TCP 辅助 ==================

bool connectToArmServer() {
    g_arm_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_arm_sock == INVALID_SOCKET) return false;

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(g_arm_port);
    inet_pton(AF_INET, g_ubuntu_ip.c_str(), &server.sin_addr);

    if (connect(g_arm_sock, (sockaddr*)&server, sizeof(server)) != 0) {
        closesocket(g_arm_sock);
        g_arm_sock = INVALID_SOCKET;
        return false;
    }
    return true;
}

void disconnectArmServer() {
    if (g_arm_sock != INVALID_SOCKET) {
        closesocket(g_arm_sock);
        g_arm_sock = INVALID_SOCKET;
    }
}

// 向 Ubuntu 查询 flange 位姿。返回 POSE 响应字符串，失败返回空
string queryFlangePose(const string& arm) {
    lock_guard<mutex> lock(g_arm_mtx);
    if (g_arm_sock == INVALID_SOCKET) return {};

    string cmd = "GET_POSE:" + arm + "\n";
    if (send(g_arm_sock, cmd.c_str(), (int)cmd.length(), 0) <= 0) return {};

#ifdef _WIN32
    DWORD timeout = 3000;
    setsockopt(g_arm_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif
    char buf[512];
    int n = recv(g_arm_sock, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return {};

    buf[n] = '\0';
    string resp(buf);
    // 去掉尾随换行
    while (!resp.empty() && (resp.back() == '\n' || resp.back() == '\r'))
        resp.pop_back();
    return resp;
}

// ================== UI 布局 ==================
void updateLayout() {
    g_left_w = g_win_h * 2 / 5;
    g_right_x = g_left_w;
    g_right_w = g_win_w - g_left_w;
    g_thumb_w = g_left_w / 2;
    g_thumb_h = g_win_h / 5;
}

void onMouse(int event, int x, int y, int, void*) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    if (x < g_left_w) {
        int col = x / g_thumb_w;
        int row = y / g_thumb_h;
        int idx = row * 2 + col;
        int n = static_cast<int>(cam_ctxs.size());
        if (idx >= 0 && idx < n) {
            int prev = g_enlarged_cam.load();
            g_enlarged_cam.store((prev == idx) ? -1 : idx);
        }
    }
}

// ================== UI 渲染 ==================
void renderThumbnailGrid(cv::Mat& canvas, int selected_idx) {
    int n = static_cast<int>(cam_ctxs.size());
    for (int i = 0; i < 10; ++i) {
        int row = i / 2, col = i % 2;
        int x = col * g_thumb_w, y = row * g_thumb_h;
        cv::Rect roi(x, y, g_thumb_w, g_thumb_h);

        if (i < n) {
            cv::Mat local_raw;
            {
                lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);
                local_raw = cam_ctxs[i]->latest_frame;
            }
            cv::Mat cell;
            if (!local_raw.empty()) {
                if (cam_ctxs[i]->is_mono) cv::cvtColor(local_raw, cell, cv::COLOR_GRAY2RGB);
                else cv::cvtColor(local_raw, cell, cv::COLOR_BayerRG2RGB);
                double scale = min(static_cast<double>(g_thumb_w) / cell.cols,
                                   static_cast<double>(g_thumb_h) / cell.rows);
                int dw = static_cast<int>(cell.cols * scale), dh = static_cast<int>(cell.rows * scale);
                cv::Mat resized;
                cv::resize(cell, resized, cv::Size(dw, dh));
                cell = cv::Mat::zeros(g_thumb_h, g_thumb_w, CV_8UC3);
                int ox = (g_thumb_w - dw) / 2, oy = (g_thumb_h - dh) / 2;
                resized.copyTo(cell(cv::Rect(ox, oy, dw, dh)));
            } else {
                cell = cv::Mat::zeros(g_thumb_h, g_thumb_w, CV_8UC3);
            }

            string label = cam_ctxs[i]->sn;
            int baseline = 0;
            double font_scale = 0.45;
            cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline);
            cv::Point org((g_thumb_w - ts.width) / 2, g_thumb_h - 6);
            cv::putText(cell, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(0, 0, 0), 3);
            cv::putText(cell, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(255, 255, 255), 1);

            cell.copyTo(canvas(roi));
            if (i == selected_idx)
                cv::rectangle(canvas, roi, cv::Scalar(0, 255, 0), 2);
        } else {
            canvas(roi) = cv::Scalar(0, 0, 0);
        }
    }
}

void renderEnlargedView(cv::Mat& canvas, int cam_idx) {
    cv::Rect right_roi(g_right_x, 0, g_right_w, g_win_h);
    if (cam_idx < 0 || cam_idx >= static_cast<int>(cam_ctxs.size())) {
        canvas(right_roi) = cv::Scalar(0, 0, 0);
        return;
    }

    cv::Mat local_raw;
    {
        lock_guard<mutex> lock(cam_ctxs[cam_idx]->frame_mtx);
        local_raw = cam_ctxs[cam_idx]->latest_frame;
    }

    if (local_raw.empty()) { canvas(right_roi) = cv::Scalar(0, 0, 0); return; }

    cv::Mat img;
    if (cam_ctxs[cam_idx]->is_mono) cv::cvtColor(local_raw, img, cv::COLOR_GRAY2RGB);
    else cv::cvtColor(local_raw, img, cv::COLOR_BayerRG2RGB);

    double scale = min(static_cast<double>(g_right_w) / img.cols,
                       static_cast<double>(g_win_h) / img.rows);
    int dst_w = static_cast<int>(img.cols * scale), dst_h = static_cast<int>(img.rows * scale);
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(dst_w, dst_h));

    int off_x = g_right_x + (g_right_w - dst_w) / 2, off_y = (g_win_h - dst_h) / 2;
    resized.copyTo(canvas(cv::Rect(off_x, off_y, dst_w, dst_h)));
}

// ================== copyWorker (同 test_calib_images) ==================
void copyWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        pair<Pylon::CBaslerUniversalGrabResultPtr, FrameMeta> task;
        {
            unique_lock<mutex> lock(ctx->copy_mtx);
            ctx->copy_cv.wait(lock, [&]{ return !ctx->copy_queue.empty() || !ctx->running; });
            if (!ctx->running && ctx->copy_queue.empty()) break;
            task = ctx->copy_queue.front();
            ctx->copy_queue.pop();
        }

        cv::Mat temp(task.first->GetHeight(), task.first->GetWidth(), CV_8UC1, task.first->GetBuffer());
        cv::Mat clone_img = temp.clone();
        {
            lock_guard<mutex> lock(ctx->frame_mtx);
            ctx->latest_frame = clone_img;
            ctx->latest_meta = task.second;
        }
    }
}

// ================== captureWorker (同 test_calib_images) ==================
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time) {
    if (!ctx->cam.open(TriggerMode::Software)) {
        cerr << "[Error] Failed to open camera " << ctx->sn << endl;
        return;
    }

    ctx->is_mono = ctx->cam.isMono();

    try {
        ctx->cam.setFrameRate(fps);
        ctx->cam.setGain(gain);
        ctx->cam.setGamma(gamma);
        ctx->cam.setExposureTime(exp_time);
    } catch (...) {}

    ctx->cam.setFrameCallback([ctx](const Pylon::CBaslerUniversalGrabResultPtr& ptr, FrameMeta meta) {
        lock_guard<mutex> lock(ctx->copy_mtx);
        if (ctx->copy_queue.size() < 2) {
            ctx->copy_queue.push({ptr, meta});
            ctx->copy_cv.notify_one();
        }
    });

    if (!ctx->cam.start()) {
        cerr << "[Error] Failed to start camera " << ctx->sn << endl;
        return;
    }

    while (ctx->running) this_thread::sleep_for(chrono::milliseconds(50));
    ctx->cam.close();
}

// ================== 辅助函数 ==================
int getNextCalibCounter(const string& save_dir) {
    int max_counter = -1;
    if (!fs::exists(save_dir)) return 0;
    for (auto& e : fs::directory_iterator(save_dir)) {
        if (e.path().extension() == ".jpg") {
            try {
                string stem = e.path().stem().string();
                size_t last_underscore = stem.find_last_of('_');
                if (last_underscore != string::npos) {
                    max_counter = max(max_counter, stoi(stem.substr(last_underscore + 1)));
                }
            } catch (...) {}
        }
    }
    return max_counter + 1;
}

// ================== 主函数 ==================
int main() {
    cout << "=== [TEST] Record Arm Data (Camera + Flange Pose) ===" << endl;

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cerr << "[System ERROR] WSAStartup failed." << endl;
        return -1;
    }
#endif

    // --- 读取 piper.yaml ---
    namespace fs = std::filesystem;
    auto piper_path = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                       / "cfg" / "piper.yaml").string();

    Cfg piper_cfg(piper_path);

    // 网络配置
    g_ubuntu_ip = piper_cfg["network"]["ubuntu_ip"].as<string>();
    g_arm_port  = piper_cfg["network"]["port"].as<int>();

    // 相机配置
    auto& rcfg = piper_cfg["test_record_arm_data"];
    vector<string> camera_ids = rcfg["cam_indices"].as<vector<string>>();
    g_calib_save_dir = rcfg["calib_save_dir"].as<string>();

    double target_fps = rcfg["fps"].as<double>();
    double gain_val   = rcfg["gain"].as<double>();
    double gamma_val  = rcfg["gamma"].as<double>();
    double exp_time   = rcfg["exposure_time"].as<double>();

    g_win_w = rcfg["window_width"].as<int>();
    g_win_h = rcfg["window_height"].as<int>();
    double ui_fps = rcfg["ui_fps"].as<double>();

    Pylon::PylonInitialize();

    // --- 打印配置 ---
    cout << "\n--- Record Arm Data Configuration ---" << endl;
    cout << "Ubuntu IP : " << g_ubuntu_ip << ":" << g_arm_port << endl;
    cout << "Current arm: " << g_current_arm << "  [t] to switch" << endl;
    cout << "Save dir   : " << g_calib_save_dir << endl;
    cout << "Cameras    : " << camera_ids.size() << endl;
    for (size_t i = 0; i < camera_ids.size(); ++i)
        cout << "  " << i << ": SN=" << camera_ids[i] << endl;
    cout << "------------------------------------\n" << endl;

    fs::create_directories(g_calib_save_dir);

    // --- 连接 Ubuntu arm server ---
    cout << "[Arm] Connecting to " << g_ubuntu_ip << ":" << g_arm_port << " ... " << flush;
    if (!connectToArmServer()) {
        cerr << "FAILED. Arm pose query will be unavailable." << endl;
    } else {
        cout << "OK" << endl;
    }

    // --- 位姿映射文件 ---
    g_mapping_file = g_calib_save_dir + "/flange_pose_mapping.txt";
    {
        bool file_exists = fs::exists(g_mapping_file);
        ofstream mf(g_mapping_file, ios::app);
        if (!file_exists) {
            mf << "# index arm x y z qx qy qz qw alpha beta gamma\n";
        }
    }
    cout << "[Mapping] " << g_mapping_file << endl;

    // --- 创建相机上下文 ---
    for (size_t i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(camera_ids[i]);
        cam_ctxs.push_back(ctx);
    }

    // --- 启动相机线程 ---
    for (auto& ctx : cam_ctxs) {
        ctx->running = true;
        ctx->copy_thread = thread(copyWorker, ctx);
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain_val, gamma_val, exp_time);
    }

    // --- UI 窗口 ---
    cv::namedWindow("Record Arm Data", cv::WINDOW_NORMAL);
    cv::resizeWindow("Record Arm Data", g_win_w, g_win_h);
    updateLayout();
    cv::setMouseCallback("Record Arm Data", onMouse);

    auto ui_interval = chrono::milliseconds(static_cast<int>(1000.0 / ui_fps));
    auto last_ui_time = chrono::steady_clock::now() - ui_interval;

    cout << "\n=== Ready ===" << endl;
    cout << "  SPACE - Capture photo + flange pose" << endl;
    cout << "  t     - Switch arm (upper/lower), current: " << g_current_arm << endl;
    cout << "  Q/ESC - Quit\n" << endl;

    while (global_running) {
        auto current_time = chrono::steady_clock::now();
        bool need_ui_update = (current_time - last_ui_time) >= ui_interval;

        // ===== 1. UI 渲染 =====
        if (need_ui_update) {
            cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
            int sel = g_enlarged_cam.load();
            renderThumbnailGrid(canvas, sel);
            renderEnlargedView(canvas, sel);

            // Right panel: show current arm + pose overlay
            cv::Rect right_roi(g_right_x, 0, g_right_w, g_win_h);
            int tx = g_right_x + 10, ty = g_win_h - 60;
            cv::putText(canvas, "Arm: " + g_current_arm,
                        cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            ty += 22;
            cv::putText(canvas, "[t] switch  [space] capture  [q] quit",
                        cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

            cv::imshow("Record Arm Data", canvas);
            last_ui_time = current_time;
        }

        // ===== 2. 键盘事件 =====
        char key = static_cast<char>(cv::waitKey(1));

        if (key == 'q' || key == 27) {
            global_running = false;
        }
        else if (key == 't' || key == 'T') {
            g_current_arm = (g_current_arm == "upper") ? "lower" : "upper";
            cout << "[Arm] Switched to: " << g_current_arm << endl;
        }
        else if (key == ' ') {
            int counter = getNextCalibCounter(g_calib_save_dir);
            stringstream ss;
            ss << setw(2) << setfill('0') << counter;
            string idx_str = ss.str();

            cout << "\n[Capture] Index " << idx_str << " | Arm: " << g_current_arm << endl;

            // 1. 保存所有相机照片
            for (auto& ctx : cam_ctxs) {
                cv::Mat snapshot;
                {
                    lock_guard<mutex> lock(ctx->frame_mtx);
                    snapshot = ctx->latest_frame.clone();
                }
                if (!snapshot.empty()) {
                    cv::Mat out_img;
                    if (ctx->is_mono) out_img = snapshot.clone();
                    else cv::cvtColor(snapshot, out_img, cv::COLOR_BayerRG2RGB);
                    string fn = g_calib_save_dir + "/calib_cam_" + ctx->sn + "_" + idx_str + ".jpg";
                    cv::imwrite(fn, out_img);
                    cout << "  -> " << fn << endl;
                }
            }

            // 2. 查询当前机械臂 flange 位姿
            string resp = queryFlangePose(g_current_arm);
            if (!resp.empty() && resp.rfind("POSE:", 0) == 0) {
                // 格式: POSE:<arm>:<x>,<y>,<z>,<qx>,<qy>,<qz>,<qw>,<alpha>,<beta>,<gamma>
                string data = resp.substr(5);  // after "POSE:"
                size_t p = data.find(':');
                string arm_name = data.substr(0, p);
                string vals = data.substr(p + 1);

                vector<double> nums;
                stringstream vss(vals);
                string token;
                while (getline(vss, token, ','))
                    nums.push_back(stod(token));

                if (nums.size() == 10) {
                    cout << fixed << setprecision(4);
                    cout << "  Pose  XYZ (m):      [" << nums[0] << ", " << nums[1] << ", " << nums[2] << "]" << endl;
                    cout << "        Quat (wxyz):   [" << nums[6] << ", " << nums[3] << ", " << nums[4] << ", " << nums[5] << "]" << endl;
                    cout << fixed << setprecision(2);
                    cout << "        Euler ZXZ'':   [" << nums[7] << ", " << nums[8] << ", " << nums[9] << "] deg" << endl;

                    // 写入映射文件
                    ofstream mf(g_mapping_file, ios::app);
                    mf << fixed << setprecision(6);
                    mf << idx_str << " " << arm_name;
                    for (double v : nums) mf << " " << v;
                    mf << endl;
                    cout << "  -> saved to " << g_mapping_file << endl;
                }
            } else if (resp.empty()) {
                cerr << "  [Warn] No response from arm server. "
                     << "Pose NOT recorded." << endl;
            } else {
                cerr << "  [Warn] Arm server: " << resp << endl;
            }
        }
    }

    // ================== 清理 ==================
    cout << "[System] Shutting down..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->copy_cv.notify_all();
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
    }

    disconnectArmServer();
    cv::destroyAllWindows();
    Pylon::PylonTerminate();

#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
