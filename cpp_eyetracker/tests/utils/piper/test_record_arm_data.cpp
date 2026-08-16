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

// ================== Resolve 子进程 (r 键触发) ==================
PROCESS_INFORMATION g_resolve_pi{};

// ================== 手眼标定流水线 (h 键: 依次运行三个程序) ==================
atomic<bool> g_pipeline_running{false};
atomic<int> g_pipeline_step{0};                        // 1~3 运行中; 0 等待/未运行; -1 失败; -2 完成
string g_pipeline_fail_msg;                            // 失败信息 (UI 显示)
chrono::steady_clock::time_point g_pipeline_start;     // 流水线启动时刻 (UI 计时)

// 运行同目录下的 exe 并等待结束, 返回退出码 (启动失败返回 -1)
int runExeWait(const string& exe_name, const string& display) {
    char self_path[MAX_PATH];
    GetModuleFileNameA(NULL, self_path, MAX_PATH);
    fs::path exe = fs::path(self_path).parent_path() / exe_name;
    if (!fs::exists(exe)) {
        cerr << "[Pipeline] Not found: " << exe.string() << endl;
        return -1;
    }
    STARTUPINFOA si{}; si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    if (!CreateProcessA(exe.string().c_str(), NULL, NULL, NULL, FALSE, 0,
                        NULL, NULL, &si, &pi)) {
        cerr << "[Pipeline] Launch failed (error " << GetLastError() << "): " << exe.string() << endl;
        return -1;
    }
    cout << "[Pipeline] Running " << display << " ..." << endl;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD ec = 0; GetExitCodeProcess(pi.hProcess, &ec);
    CloseHandle(pi.hProcess); CloseHandle(pi.hThread);
    cout << "[Pipeline] " << display << " finished (exit code " << ec << ")" << endl;
    return static_cast<int>(ec);
}

// 后台线程: resolve_calib_board_pose → test_piper_hand_eye_calib → save_piper_chain
void runHandEyePipeline() {
    cout << "\n[Pipeline] Hand-eye pipeline started (resolve -> hand_eye -> save_chain)..." << endl;
    vector<pair<string,string>> steps = {
        {"resolve_calib_board_pose.exe", "resolve_calib_board_pose"},
        {"test_piper_hand_eye_calib.exe", "test_piper_hand_eye_calib"},
        {"save_piper_chain.exe", "save_piper_chain"},
    };
    for (size_t i = 0; i < steps.size(); ++i) {
        g_pipeline_step.store(static_cast<int>(i) + 1);
        int ec = runExeWait(steps[i].first, steps[i].second);
        if (ec != 0) {
            g_pipeline_fail_msg = "Step " + to_string(i + 1) + "/" + to_string(steps.size())
                                + ": " + steps[i].second + " (exit " + to_string(ec) + ")";
            cerr << "[Pipeline] " << steps[i].second << " failed (exit " << ec << "). Stopping." << endl;
            g_pipeline_step.store(-1);   // UI 显示失败画面 (保持 running=true, 按任意键返回)
            return;
        }
        // 每个程序之间间隔几秒 (等待文件落盘/资源释放)
        if (i + 1 < steps.size()) {
            g_pipeline_step.store(0);   // 等待期间 UI 不显示步骤
            cout << "[Pipeline] Waiting 3s before next step..." << endl;
            this_thread::sleep_for(chrono::seconds(3));
        }
    }
    g_pipeline_step.store(-2);   // UI 显示完成画面 3 秒后自动返回
    cout << "[Pipeline] Done.\n" << endl;
    this_thread::sleep_for(chrono::seconds(3));
    g_pipeline_running.store(false);
    g_pipeline_step.store(0);
}

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

    queue<pair<cv::Mat, FrameMeta>> copy_queue;  // [Fix] 队列存已拷贝 Mat, 回调内同步拷贝
    mutex copy_mtx;
    condition_variable copy_cv;

    explicit CameraContext(string cam_sn) : sn(cam_sn), cam(cam_sn) {}
};

vector<shared_ptr<CameraContext>> cam_ctxs;
atomic<int> g_enlarged_cam{-1};
int g_last_capture_index = -1;
string g_last_capture_arm;
bool g_calib_mode = false;      // 内参标定模式 ('i' 切换)
string g_calib_cam_sn;          // 标定模式下的相机 SN

// UI layout
int g_win_w = 1224, g_win_h = 1024;
int g_left_w = 0, g_right_x = 0, g_right_w = 0;
int g_thumb_w = 0, g_thumb_h = 0;
string g_calib_save_dir;
string g_mapping_file_upper;
string g_mapping_file_lower;
string g_handeye_output;   // 手眼标定输出 = cfg/arm_pose/{day_id}.yaml
string g_cam_xml_dir;      // 相机标定 XML 加载目录 = cam_calib.yaml: calib_save_dir/{day_id}/output

const string& currentMappingFile() {
    return (g_current_arm == "upper") ? g_mapping_file_upper : g_mapping_file_lower;
}

// ================== TCP 辅助 ==================

bool recvLine(SOCKET sock, string& line, int timeout_ms = 10000) {
#ifdef _WIN32
    DWORD to = timeout_ms;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
#endif
    char buf[256];
    string acc;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);

    while (chrono::steady_clock::now() < deadline) {
        int n = recv(sock, buf, sizeof(buf) - 1, 0);
        if (n <= 0) return false;
        buf[n] = '\0';
        acc += buf;
        size_t nl = acc.find('\n');
        if (nl != string::npos) {
            line = acc.substr(0, nl);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
    }
    return false;
}

bool sendLine(SOCKET sock, const string& msg) {
    string data = msg + "\n";
    return send(sock, data.c_str(), (int)data.length(), 0) > 0;
}

// Parse "MOVED:arm:x,y,z,qx,qy,qz,qw,alpha,beta,gamma" or "POSE:arm:..."
// (shared format — POSE and MOVED both carry 10 comma-separated values)
struct ArmPose { double x,y,z, qx,qy,qz,qw, alpha,beta,gamma; bool valid=false; };

bool parsePoseResponse(const string& resp, string& arm, ArmPose& pose) {
    if (resp.rfind("MOVED:", 0) != 0 && resp.rfind("POSE:", 0) != 0) return false;
    size_t colon1 = resp.find(':');
    size_t colon2 = resp.find(':', colon1 + 1);
    if (colon1 == string::npos || colon2 == string::npos) return false;
    arm = resp.substr(colon1 + 1, colon2 - colon1 - 1);
    string vals = resp.substr(colon2 + 1);
    vector<double> nums;
    stringstream ss(vals); string token;
    while (getline(ss, token, ',')) {
        try { nums.push_back(stod(token)); } catch (...) { return false; }
    }
    if (nums.size() != 10) return false;
    pose.x=nums[0]; pose.y=nums[1]; pose.z=nums[2];
    pose.qx=nums[3]; pose.qy=nums[4]; pose.qz=nums[5]; pose.qw=nums[6];
    pose.alpha=nums[7]; pose.beta=nums[8]; pose.gamma=nums[9];
    pose.valid = true;
    return true;
}

bool connectToArmServer() {
    // 重试连接 (Ubuntu 端可能后启动, 参照 test_calib_images 的重试逻辑)
    int retry = 0;
    while (global_running) {
        g_arm_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (g_arm_sock == INVALID_SOCKET) return false;

        sockaddr_in server{};
        server.sin_family = AF_INET;
        server.sin_port = htons(g_arm_port);
        inet_pton(AF_INET, g_ubuntu_ip.c_str(), &server.sin_addr);

        if (connect(g_arm_sock, (sockaddr*)&server, sizeof(server)) == 0) {
            // 已连接, 尝试握手
            sendLine(g_arm_sock, "READY");
            string hl;
            if (recvLine(g_arm_sock, hl, 10000) && hl == "ACK") {
                cout << "[Arm] Handshake OK." << endl;
                return true;
            }
            cerr << "[Arm] Handshake fail: " << (hl.empty() ? "no response" : hl) << endl;
            closesocket(g_arm_sock);
            g_arm_sock = INVALID_SOCKET;
        } else {
            closesocket(g_arm_sock);
            g_arm_sock = INVALID_SOCKET;
        }

        if (++retry % 10 == 1)
            cerr << "[Arm] Connect retry #" << retry << " (" << g_ubuntu_ip << ":" << g_arm_port << ")..." << endl;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    return false;
}

void disconnectArmServer() {
    if (g_arm_sock != INVALID_SOCKET) {
        closesocket(g_arm_sock);
        g_arm_sock = INVALID_SOCKET;
    }
}

// 向 Ubuntu 查询 flange 位姿 (GET_POSE on ctrl_port 49301)
ArmPose queryFlangePose(const string& arm) {
    lock_guard<mutex> lock(g_arm_mtx);
    if (g_arm_sock == INVALID_SOCKET) return {};

    if (!sendLine(g_arm_sock, "GET_POSE:" + arm)) return {};

    string resp;
    if (!recvLine(g_arm_sock, resp, 3000)) {
        cerr << "[Arm] GET_POSE timeout for " << arm << endl;
        return {};
    }

    string resp_arm; ArmPose pose;
    if (parsePoseResponse(resp, resp_arm, pose)) return pose;
    cerr << "[Arm] GET_POSE error: " << resp << endl;
    return {};
}

// ================== UI 布局 ==================
int countJpgs(const string& dir);   // 前向声明 (onMouse 使用, 定义在辅助函数区)
string lastJpgIn(const string& dir);

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
            if (prev != idx) {
                // 切换到不同相机 → 保持标定模式, 标定对象跟随切换
                if (g_calib_mode) {
                    g_calib_cam_sn = cam_ctxs[idx]->sn;
                    string calib_dir = g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn;
                    fs::create_directories(calib_dir);
                    cout << "[Calib Mode] Camera switched to " << g_calib_cam_sn
                         << "  (existing: " << countJpgs(calib_dir) << ")" << endl;
                }
                g_enlarged_cam.store(idx);
            } else {
                g_enlarged_cam.store(-1);  // deselect
            }
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
        pair<cv::Mat, FrameMeta> task;
        {
            unique_lock<mutex> lock(ctx->copy_mtx);
            ctx->copy_cv.wait(lock, [&]{ return !ctx->copy_queue.empty() || !ctx->running; });
            if (!ctx->running && ctx->copy_queue.empty()) break;
            task = ctx->copy_queue.front();
            ctx->copy_queue.pop();
        }

        {
            lock_guard<mutex> lock(ctx->frame_mtx);
            ctx->latest_frame = task.first;
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
        // [Fix] 回调内同步拷贝: 采集卡缓冲只在回调期间被持有
        cv::Mat temp(ptr->GetHeight(), ptr->GetWidth(), CV_8UC1, ptr->GetBuffer());
        cv::Mat clone_img = temp.clone();
        lock_guard<mutex> lock(ctx->copy_mtx);
        if (ctx->copy_queue.size() < 2) {
            ctx->copy_queue.push({clone_img, meta});
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

// 统计目录内 jpg 数量 (计数器显示用)
int countJpgs(const string& dir) {
    int n = 0;
    if (!fs::exists(dir)) return 0;
    for (auto& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".jpg") n++;
    return n;
}

// 目录内索引最大的 jpg 完整路径 (撤回用)
string lastJpgIn(const string& dir) {
    string best; int mx = -1;
    if (!fs::exists(dir)) return best;
    for (auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() != ".jpg") continue;
        string s = e.path().stem().string();
        size_t u = s.find_last_of('_');
        if (u == string::npos) continue;
        try { int v = stoi(s.substr(u + 1)); if (v > mx) { mx = v; best = e.path().string(); } }
        catch (...) {}
    }
    return best;
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

    // --- 读取 calib_arm.yaml ---
    namespace fs = std::filesystem;
    fs::path cfg_dir = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg";
    Cfg arm_cfg((cfg_dir / "calib_arm.yaml").string());

    // 网络配置
    g_ubuntu_ip = arm_cfg["network"]["ubuntu_ip"].as<string>();
    g_arm_port  = arm_cfg["network"]["ctrl_port"].as<int>();

    // 相机配置
    auto& rcfg = arm_cfg["record"];
    vector<string> camera_ids = rcfg["cam_indices"].as<vector<string>>();
    string day_id = rcfg["day_id"].as<string>();
    g_calib_save_dir = rcfg["calib_save_dir"].as<string>() + "/" + day_id;
    g_handeye_output = "cfg/arm_pose/" + day_id + ".yaml";   // test_piper_hand_eye_calib 的输出 (显示用, 从 cfg 开始)

    // 相机标定 XML 目录 (与 resolve_calib_board_pose.cpp 的计算一致)
    {
        Cfg cam_cfg((cfg_dir / "cam_calib.yaml").string());
        g_cam_xml_dir = cam_cfg["calib"]["calib_save_dir"].as<string>() + "/" + day_id + "/output";
    }

    double target_fps = rcfg["fps"].as<double>();
    double gain_val   = rcfg["gain"].as<double>();
    double gamma_val  = rcfg["gamma"].as<double>();
    double exp_time   = rcfg["exposure_time"].as<double>();

    g_win_w = rcfg["window_width"].as<int>();
    g_win_h = rcfg["window_height"].as<int>();
    double ui_fps = rcfg["ui_fps"].as<double>();

    // --- 打印配置 ---
    cout << "\n--- Record Arm Data Configuration ---" << endl;
    cout << "Ubuntu IP : " << g_ubuntu_ip << ":" << g_arm_port << endl;
    cout << "Current arm: " << g_current_arm << "  [t] to switch" << endl;
    cout << "Save dir   : " << g_calib_save_dir << endl;
    cout << "Hand-eye out: " << g_handeye_output << endl;
    cout << "Cam XML dir : " << g_cam_xml_dir << endl;
    cout << "Cameras    : " << camera_ids.size() << endl;
    for (size_t i = 0; i < camera_ids.size(); ++i)
        cout << "  " << i << ": SN=" << camera_ids[i] << endl;
    cout << "------------------------------------\n" << endl;

    fs::create_directories(g_calib_save_dir);

    // --- 连接 Ubuntu arm server (带重试握手) ---
    cout << "[Arm] Connecting to " << g_ubuntu_ip << ":" << g_arm_port << " ... " << flush;
    if (!connectToArmServer()) {
        cerr << "FAILED. Arm pose query will be unavailable." << endl;
    }

    // --- 位姿映射文件 (upper / lower 分别存储) ---
    g_mapping_file_upper = g_calib_save_dir + "/flange_pose_mapping_upper.txt";
    g_mapping_file_lower = g_calib_save_dir + "/flange_pose_mapping_lower.txt";
    for (auto& mf : {g_mapping_file_upper, g_mapping_file_lower}) {
        bool exists = fs::exists(mf);
        ofstream f(mf, ios::app);
        if (!exists) f << "# index arm x y z qx qy qz qw alpha beta gamma\n";
        cout << "[Mapping] " << mf << endl;
    }

    // --- 初始化相机 ---
    cout << "[Camera] Initializing cameras..." << endl;
    Pylon::PylonInitialize();

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
    cout << "  r     - Launch resolve_calib_board_pose.exe" << endl;
    cout << "  h     - Hand-eye pipeline (resolve -> hand_eye -> save_chain)" << endl;
    cout << "  v     - Launch viz_piper_chain.py" << endl;
    cout << "  Q/ESC - Quit\n" << endl;

    while (global_running) {
        auto current_time = chrono::steady_clock::now();
        bool need_ui_update = (current_time - last_ui_time) >= ui_interval;

        // ===== 1. UI 渲染 =====
        if (need_ui_update) {
            cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
            if (g_pipeline_running.load()) {
                // 流水线运行中: 全屏状态文字, 不渲染采集 UI (降低 CPU)
                int cx = g_win_w / 2, cy = g_win_h / 2;
                auto put = [&](int y, const string& t, double s, cv::Scalar c) {
                    int bl = 0;
                    cv::Size sz = cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX, s, 2, &bl);
                    cv::putText(canvas, t, cv::Point(cx - sz.width / 2, y),
                                cv::FONT_HERSHEY_SIMPLEX, s, c, 2, cv::LINE_AA);
                };
                int step = g_pipeline_step.load();
                const char* names[] = {"", "resolve_calib_board_pose", "test_piper_hand_eye_calib", "save_piper_chain"};
                if (step == -1) {
                    // 失败: 保持显示直到按任意键返回
                    put(cy - 80, "PIPELINE FAILED", 1.2, cv::Scalar(0, 0, 255));
                    put(cy - 20, g_pipeline_fail_msg, 0.7, cv::Scalar(255, 255, 255));
                    put(cy + 30, "Check console output for details", 0.5, cv::Scalar(140, 140, 140));
                    put(g_win_h - 60, "[any key] return  [ESC/q] quit", 0.5, cv::Scalar(140, 140, 140));
                } else if (step == -2) {
                    // 完成: 显示 3 秒后自动返回
                    put(cy - 40, "PIPELINE COMPLETE", 1.2, cv::Scalar(0, 255, 0));
                    put(g_win_h - 60, "Returning to capture UI...", 0.5, cv::Scalar(140, 140, 140));
                } else {
                    put(cy - 80, "PIPELINE RUNNING...", 1.2, cv::Scalar(0, 215, 255));
                    if (step >= 1 && step <= 3) {
                        char sb[128];
                        snprintf(sb, sizeof(sb), "Step %d/3: %s", step, names[step]);
                        put(cy - 20, sb, 0.8, cv::Scalar(220, 220, 220));
                    }
                    char et[64];
                    snprintf(et, sizeof(et), "Elapsed: %.1fs",
                             chrono::duration<double>(current_time - g_pipeline_start).count());
                    put(cy + 30, et, 0.6, cv::Scalar(140, 140, 140));
                    put(g_win_h - 60, "[ESC/q] quit (pipeline continues in background)", 0.5, cv::Scalar(140, 140, 140));
                }
            } else {
            int sel = g_enlarged_cam.load();
            renderThumbnailGrid(canvas, sel);
            renderEnlargedView(canvas, sel);
            cv::line(canvas, cv::Point(g_left_w, 0), cv::Point(g_left_w, g_win_h),
                     cv::Scalar(60, 60, 60), 2);

            // Center crosshair in enlarged area
            int cx = g_right_x + g_right_w / 2, cy = g_win_h / 2, cl = 20;
            cv::line(canvas, cv::Point(cx - cl, cy), cv::Point(cx + cl, cy), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
            cv::line(canvas, cv::Point(cx, cy - cl), cv::Point(cx, cy + cl), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

            // Top-left: SN + capture mode
            string sn_text = (sel >= 0 && sel < (int)cam_ctxs.size()) ? cam_ctxs[sel]->sn : "NO SELECTION";
            cv::putText(canvas, sn_text, cv::Point(g_right_x + 10, 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);
            string mode_text = g_calib_mode ? "MODE: INTRINSIC" : "MODE: ARM CALIB";
            cv::Scalar mode_color = g_calib_mode ? cv::Scalar(0, 255, 0) : cv::Scalar(200, 80, 255);
            cv::putText(canvas, mode_text, cv::Point(g_right_x + 10, 62),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 2, cv::LINE_AA);

            // Top-right: 直接统计图片数量 (两种模式均以磁盘为准)
            int capture_count = g_calib_mode
                ? countJpgs(g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn)
                : countJpgs(g_calib_save_dir);
            string cnt = "Captures: " + to_string(capture_count);
            cv::Size cnt_sz = cv::getTextSize(cnt, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, 0);
            cv::putText(canvas, cnt,
                        cv::Point(g_right_x + g_right_w - cnt_sz.width - 15, 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);

            // Bottom-right: 目录水印 (右对齐, 位于 arm 指示上方)
            string io_xml = "XML: " + g_cam_xml_dir;
            cv::Size xml_sz = cv::getTextSize(io_xml, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, 0);
            cv::putText(canvas, io_xml,
                        cv::Point(g_right_x + g_right_w - xml_sz.width - 15, g_win_h - 95),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
            string io_in = "In : " + g_calib_save_dir;
            cv::Size in_sz = cv::getTextSize(io_in, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, 0);
            cv::putText(canvas, io_in,
                        cv::Point(g_right_x + g_right_w - in_sz.width - 15, g_win_h - 75),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
            string io_out = "Out: " + g_handeye_output;
            cv::Size out_sz = cv::getTextSize(io_out, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, 0);
            cv::putText(canvas, io_out,
                        cv::Point(g_right_x + g_right_w - out_sz.width - 15, g_win_h - 55),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);

            // Bottom-right: arm indicator
            string arm_label = (g_current_arm == "upper") ? "UPPER" : "LOWER";
            cv::Scalar arm_color = (g_current_arm == "upper")
                                   ? cv::Scalar(0, 215, 255)    // gold
                                   : cv::Scalar(200, 80, 255);   // purple
            cv::Size arm_sz = cv::getTextSize(arm_label, cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0);
            cv::putText(canvas, arm_label,
                        cv::Point(g_right_x + g_right_w - arm_sz.width - 15, g_win_h - 15),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, arm_color, 2, cv::LINE_AA);

            // Bottom-left hints
            int hx = g_right_x + 10, hy = g_win_h - 45;
            string hints = g_calib_mode
                ? "[i] exit  [space] capture  [z] undo  Camera: " + g_calib_cam_sn
                : "[t] switch  [space] capture  [z] undo  [i] calib  [r] resolve  [h] pipeline  [v] viz  [c] clear  [q] quit";
            cv::putText(canvas, hints, cv::Point(hx, hy),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
            hy += 18;
            string saving = g_calib_mode
                ? ("-> " + g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn + "/calib_XX.jpg")
                : ("Arm: " + g_current_arm + "  |  Saving to: flange_pose_mapping_" + g_current_arm + ".txt");
            cv::putText(canvas, saving, cv::Point(hx, hy),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
            }  // end if (g_pipeline_running)

            cv::imshow("Record Arm Data", canvas);
            last_ui_time = current_time;
        }

        // ===== 2. 键盘事件 =====
        char key = static_cast<char>(cv::waitKey(1));

        if (key == 'q' || key == 27) {
            // 通知 Ubuntu 端退出
            if (g_arm_sock != INVALID_SOCKET) {
                lock_guard<mutex> lock(g_arm_mtx);
                sendLine(g_arm_sock, "SHUTDOWN");
                string ack;
                if (recvLine(g_arm_sock, ack, 2000)) cout << "[Arm] Server: " << ack << endl;
            }
            global_running = false;
        }
        else if (g_pipeline_running.load()) {
            // 流水线期间禁用按键; 失败画面按任意键返回采集界面
            if (g_pipeline_step.load() == -1) {
                g_pipeline_running.store(false);
                g_pipeline_step.store(0);
                cout << "[Pipeline] Failure screen dismissed." << endl;
            }
        }
        else if (key == 't' || key == 'T') {
            g_current_arm = (g_current_arm == "upper") ? "lower" : "upper";
            cout << "[Arm] Switched to: " << g_current_arm << endl;
        }
        else if (key == 'r' || key == 'R') {
            // 触发 resolve_calib_board_pose.exe (与自身同目录)
            if (g_resolve_pi.hProcess) {
                DWORD ec = 0;
                if (GetExitCodeProcess(g_resolve_pi.hProcess, &ec) && ec == STILL_ACTIVE) {
                    cout << "[Resolve] Already running. Please wait." << endl;
                } else {
                    CloseHandle(g_resolve_pi.hProcess); CloseHandle(g_resolve_pi.hThread);
                    ZeroMemory(&g_resolve_pi, sizeof(g_resolve_pi));
                }
            }
            if (!g_resolve_pi.hProcess) {
                char self_path[MAX_PATH];
                GetModuleFileNameA(NULL, self_path, MAX_PATH);
                fs::path exe = fs::path(self_path).parent_path() / "resolve_calib_board_pose.exe";
                if (!fs::exists(exe)) {
                    cerr << "[Resolve] Not found: " << exe.string() << endl;
                } else {
                    STARTUPINFOA si{}; si.cb = sizeof(si);
                    if (CreateProcessA(exe.string().c_str(), NULL, NULL, NULL, FALSE, 0,
                                       NULL, NULL, &si, &g_resolve_pi)) {
                        cout << "[Resolve] Launched resolve_calib_board_pose.exe" << endl;
                    } else {
                        cerr << "[Resolve] Launch failed (error " << GetLastError() << "): "
                             << exe.string() << endl;
                    }
                }
            }
        }
        else if (key == 'h' || key == 'H') {
            // 依次运行 resolve → hand_eye → save_chain (后台线程, UI 显示全屏状态)
            g_pipeline_running.store(true);
            g_pipeline_step.store(0);
            g_pipeline_start = chrono::steady_clock::now();
            thread(runHandEyePipeline).detach();
        }
        else if (key == 'v' || key == 'V') {
            // 启动 viz_piper_chain.py (脚本内已强制 TkAgg, 此处再设环境变量双保险)
            SetEnvironmentVariableA("MPLBACKEND", "TkAgg");
            fs::path script = fs::path(__FILE__).parent_path() / "viz_piper_chain.py";
            string cmd = "python \"" + script.string() + "\"";
            STARTUPINFOA si{}; si.cb = sizeof(si);
            PROCESS_INFORMATION pi{};
            if (CreateProcessA(NULL, cmd.data(), NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi)) {
                CloseHandle(pi.hProcess); CloseHandle(pi.hThread);
                cout << "[Viz] Launched viz_piper_chain.py" << endl;
            } else {
                cerr << "[Viz] Launch failed (error " << GetLastError() << "): " << cmd << endl;
            }
        }
        else if (key == 'z' || key == 'Z') {
            if (g_calib_mode) {
                // 撤回索引最大的一张 (以磁盘为准)
                string calib_dir = g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn;
                string del = lastJpgIn(calib_dir);
                if (del.empty()) { cout << "[Undo] No calib photos to undo." << endl; }
                else { fs::remove(del); cout << "[Undo] Removed " << del << endl; }
            } else if (g_last_capture_index < 0) {
                cout << "[Undo] No previous capture to undo." << endl;
            } else {
                stringstream ss; ss << setw(2) << setfill('0') << g_last_capture_index;
                string idx_str = ss.str();
                cout << "\n[Undo] Deleting capture index " << idx_str
                     << " (arm: " << g_last_capture_arm << ")" << endl;
                if (fs::exists(g_calib_save_dir)) {
                    for (auto& e : fs::directory_iterator(g_calib_save_dir)) {
                        string stem = e.path().stem().string();
                        if (stem.rfind("calib_cam_", 0) == 0 && stem.length() >= 2
                            && stem.substr(stem.length() - 2) == idx_str)
                            fs::remove(e.path());
                    }
                }
                string mf_path = (g_last_capture_arm == "upper")
                    ? g_mapping_file_upper : g_mapping_file_lower;
                if (fs::exists(mf_path)) {
                    vector<string> lines;
                    { ifstream in(mf_path); string line;
                      while (getline(in, line))
                          if (!line.empty() && line[0] != '#') lines.push_back(line); }
                    while (!lines.empty() && lines.back()[0] == '#') lines.pop_back();
                    if (!lines.empty()) lines.pop_back();
                    { ofstream out(mf_path);
                      out << "# index arm x y z qx qy qz qw alpha beta gamma\n";
                      for (auto& l : lines) out << l << "\n"; }
                }
                g_last_capture_index--;
                cout << "[Undo] Done.\n" << endl;
            }
        }
        else if (key == 'i' && !g_calib_mode) {
            int cam_idx = g_enlarged_cam.load();
            if (cam_idx < 0 || cam_idx >= (int)cam_ctxs.size()) {
                cout << "[Calib] No camera selected. Click a thumbnail first." << endl;
            } else {
                g_calib_mode = true;
                g_calib_cam_sn = cam_ctxs[cam_idx]->sn;
                string calib_dir = g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn;
                fs::create_directories(calib_dir);
                cout << "\n[Calib Mode] ON — Camera: " << g_calib_cam_sn
                     << "  Arm: " << g_current_arm
                     << "  Existing: " << countJpgs(calib_dir)
                     << "  (i=exit, space=capture, z=undo)" << endl;
            }
        }
        else if (key == 'i' && g_calib_mode) {
            g_calib_mode = false;
            cout << "[Calib Mode] OFF — " << countJpgs(g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn)
                 << " photos captured for " << g_calib_cam_sn << endl;
        }
        else if (key == 'c') {
            cout << "\n[Clear] Removing all photos and mapping files..." << endl;
            int removed = 0;
            if (fs::exists(g_calib_save_dir)) {
                try {
                    for (auto& e : fs::recursive_directory_iterator(g_calib_save_dir)) {
                        string ext = e.path().extension().string();
                        if (ext == ".jpg" || ext == ".txt") {
                            fs::remove(e.path());
                            removed++;
                        }
                    }
                    // 删除空子目录
                    for (auto& e : fs::directory_iterator(g_calib_save_dir)) {
                        if (e.is_directory()) {
                            error_code ec;
                            fs::remove_all(e.path(), ec);
                        }
                    }
                } catch (const fs::filesystem_error& e) {
                    cerr << "[Clear] Error: " << e.what() << endl;
                }
            }
            g_last_capture_index = -1;
            cout << "[Clear] Removed " << removed << " files. Counters reset.\n" << endl;
        }
        else if (key == ' ') {
            if (g_calib_mode) {
                // 内参标定模式: 保存到 save_dir/<arm>/<SN>/calib_<NN>.jpg (索引由文件夹扫描决定)
                string calib_dir = g_calib_save_dir + "/" + g_current_arm + "/" + g_calib_cam_sn;
                fs::create_directories(calib_dir);
                int counter = getNextCalibCounter(calib_dir);
                stringstream ss; ss << setw(2) << setfill('0') << counter;
                int cam_idx = -1;
                for (int i = 0; i < (int)cam_ctxs.size(); ++i)
                    if (cam_ctxs[i]->sn == g_calib_cam_sn) { cam_idx = i; break; }
                if (cam_idx < 0) { cout << "[Calib] Camera not found!" << endl; }
                else {
                    cv::Mat snapshot;
                    { lock_guard<mutex> lock(cam_ctxs[cam_idx]->frame_mtx);
                      snapshot = cam_ctxs[cam_idx]->latest_frame.clone(); }
                    if (!snapshot.empty()) {
                        cv::Mat out_img;
                        if (cam_ctxs[cam_idx]->is_mono) out_img = snapshot.clone();
                        else cv::cvtColor(snapshot, out_img, cv::COLOR_BayerRG2RGB);
                        string fn = calib_dir + "/calib_" + ss.str() + ".jpg";
                        cv::imwrite(fn, out_img);
                        cout << "  [Calib " << ss.str() << "] -> " << fn << endl;
                    }
                }
            } else {
                int counter = getNextCalibCounter(g_calib_save_dir);
                stringstream ss;
                ss << setw(2) << setfill('0') << counter;
                string idx_str = ss.str();

                g_last_capture_index = counter;
                g_last_capture_arm = g_current_arm;
                cout << "\n[Capture] Index " << idx_str << " | Arm: " << g_current_arm << endl;

                int cam_idx = g_enlarged_cam.load();
                if (cam_idx < 0 || cam_idx >= (int)cam_ctxs.size()) {
                    cout << "  [Warn] No camera selected. Click a thumbnail to enlarge it first." << endl;
                } else {
                    auto& ctx = cam_ctxs[cam_idx];
                    cv::Mat snapshot;
                    { lock_guard<mutex> lock(ctx->frame_mtx);
                      snapshot = ctx->latest_frame.clone(); }
                    if (!snapshot.empty()) {
                        cv::Mat out_img;
                        if (ctx->is_mono) out_img = snapshot.clone();
                        else cv::cvtColor(snapshot, out_img, cv::COLOR_BayerRG2RGB);
                        string fn = g_calib_save_dir + "/calib_cam_" + ctx->sn + "_" + idx_str + ".jpg";
                        cv::imwrite(fn, out_img);
                        cout << "  -> " << fn << " (cam " << cam_idx << ": " << ctx->sn << ")" << endl;
                    }
                }

                // 查询当前机械臂 flange 位姿
                ArmPose pose = queryFlangePose(g_current_arm);
                if (pose.valid) {
                    cout << fixed << setprecision(4);
                    cout << "  Pose  XYZ (m):      [" << pose.x << ", " << pose.y << ", " << pose.z << "]" << endl;
                    cout << "        Quat (wxyz):   [" << pose.qw << ", " << pose.qx << ", " << pose.qy << ", " << pose.qz << "]" << endl;
                    cout << fixed << setprecision(2);
                    cout << "        Euler ZXZ'':   [" << pose.alpha << ", " << pose.beta << ", " << pose.gamma << "] deg" << endl;
                    string mf_path = currentMappingFile();
                    ofstream mf(mf_path, ios::app);
                    mf << fixed << setprecision(6);
                    mf << idx_str << " " << g_current_arm
                       << " " << pose.x << " " << pose.y << " " << pose.z
                       << " " << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw
                       << " " << pose.alpha << " " << pose.beta << " " << pose.gamma << endl;
                    cout << "  -> saved to " << mf_path << endl;
                } else {
                    cerr << "  [Warn] No response from arm server. Pose NOT recorded." << endl;
                }
            }  // end else (!g_calib_mode)
        }  // end SPACE handler
    }  // end while (global_running)

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
