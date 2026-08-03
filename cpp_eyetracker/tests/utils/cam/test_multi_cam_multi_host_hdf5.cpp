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
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <iomanip>
#include <cmath>
#include <functional>
#include <fstream>
#include <algorithm> // [新增] 用于 std::min / std::max
#include <system_error>
#include <pylon/PylonIncludes.h>
#include <H5Cpp.h>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== 全局网络同步标志 ==================
atomic<bool> global_running{true};
atomic<bool> net_cmd_record{false};

// [新增] 极速触发相关的全局变量
string shared_record_timestr = "";
std::chrono::steady_clock::time_point global_record_start_time;
SOCKET master_udp_sock = INVALID_SOCKET;
sockaddr_in slave_udp_addr{};
SOCKET g_fault_sock = INVALID_SOCKET;
sockaddr_in g_peer_fault_addr{};
bool g_use_hw_trigger = false;

// ------------------------------------------------------------------
// [重要修改] 将原代码中的 "相机通用功能" (包括 CameraContext 结构体和 cam_ctxs 变量) 
// 全部剪切并移动到这里！(必须在 UDP 线程函数之上)
// ------------------------------------------------------------------
enum class CamStatus { INIT, OPENED, WAITING_TRIGGER, STREAMING, ERROR_ };

struct LogEntry { string filename; int64_t blockID; int64_t timestamp; int width; int height; };

struct CameraContext {
    int index;
    string id;
    string save_base_dir; 
    BaslerCamera cam{""};

    bool is_mono = true;
    
    thread capture_thread;
    thread copy_thread; 
    atomic<bool> running{true};
    atomic<bool> recording{false};

    cv::Mat latest_frame;
    FrameMeta latest_meta;
    mutex frame_mtx; 

    vector<cv::Mat> ram_buffer;
    vector<FrameMeta> meta_buffer;
    int total_record_frames = 0;
    atomic<bool> dump_ready{false};

    queue<pair<Pylon::CBaslerUniversalGrabResultPtr, FrameMeta>> copy_queue;
    mutex copy_mtx;
    condition_variable copy_cv;

    string temp_dir;      
    string log_file_path;
    ofstream log_stream;

    atomic<int> captured_frames{0};
    atomic<int> recorded_frames{0}; 

    atomic<CamStatus> status{CamStatus::INIT};
    string status_msg = "Initializing";

    int64_t frame_offset = 0;
    bool offset_initialized = false;
    static atomic<int64_t> master_first_id;
    static atomic<bool> master_set;

    // Health monitoring
    atomic<int64_t> last_block_id{-1};
    atomic<chrono::steady_clock::time_point> last_frame_time{chrono::steady_clock::now()};
    atomic<bool> has_streamed{false};

    // ---- Metrics (new) ----
    atomic<int> max_queue_size{0};
    int64_t first_recorded_block_id = -1;
    int64_t last_recorded_block_id = -1;
    chrono::steady_clock::time_point first_frame_time;
    atomic<int> dropped_frames{0};
    int64_t prev_block_id = -1;
    chrono::steady_clock::time_point recording_end_time;  // t_last: when last frame written to RAM
    double recover2ram_s = 0.0;         // (t_last - t_first) - theoretical, unit: seconds
    double wait4disk_s = 0.0;           // dump_start - recording_end_time, unit: seconds
    double ram2disk_s = 0.0;            // dump_end - dump_start, unit: seconds
    chrono::steady_clock::time_point dump_start_time;
    chrono::steady_clock::time_point dump_end_time;
    chrono::steady_clock::time_point jpg_start_time;
    chrono::steady_clock::time_point jpg_end_time;

    // ---- HDF5 ----
    string hdf5_dir;
    H5::H5File* hdf5_file = nullptr;
    H5::DataSet hdf5_raw_ds;
    H5::DataSet hdf5_gaze_ds;
    H5::DataSet hdf5_valid_ds;

    CameraContext(int idx, string cam_id, string save_dir) : index(idx), id(cam_id), save_base_dir(save_dir), cam(cam_id) {}
};

atomic<int64_t> CameraContext::master_first_id(-1);
atomic<bool> CameraContext::master_set(false);

vector<shared_ptr<CameraContext>> cam_ctxs;

// ================== 故障状态 ==================
atomic<bool> g_fault_active{false};
atomic<int> g_faulty_cam{-1};
atomic<bool> g_fault_on_master{false};
chrono::steady_clock::time_point g_ready_time;
chrono::steady_clock::time_point g_fault_time;

// ================== HDF5 全局 ==================
int g_chunk_idx = 0;
atomic<int> g_frame_offset{0};
vector<string> g_participant_roots;   // per-camera, same length as cam_ctxs
string g_sentry_root;                // = g_participant_roots[0]
int g_hdf5_chunk_capacity = 2000;

// ================== 会话日志 ==================
string g_session_log_path;
int g_recording_number = 0;
ofstream g_session_log;

// ================== 异常计数器 ==================
atomic<int> g_exc_fatal{0}, g_exc_error{0}, g_exc_warn{0}, g_exc_info{0};

void logException(const string& level, const string& source, const string& msg) {
    auto t = chrono::system_clock::now();
    auto tt = chrono::system_clock::to_time_t(t);
    auto ms = chrono::duration_cast<chrono::milliseconds>(t.time_since_epoch()) % 1000;
    char tb[16]; strftime(tb, sizeof(tb), "%H:%M:%S", localtime(&tt));
    string ts = string(tb) + "." + to_string(ms.count() / 100) + to_string((ms.count() / 10) % 10) + to_string(ms.count() % 10);

    string line = "> **[" + level + "]** `" + ts + "` | " + source + " | " + msg;

    if (level == "FATAL") { g_exc_fatal++; cerr << line << endl; }
    else if (level == "ERROR") { g_exc_error++; cerr << line << endl; }
    else if (level == "WARN") { g_exc_warn++; cout << line << endl; }
    else { g_exc_info++; cout << line << endl; }

    if (g_session_log.is_open()) g_session_log << line << "\n" << flush;
}

// ================== 跨主机同步偏移 ==================
int64_t g_peer_first_block_id = -1;  // other host's first BlockID (from TCP exchange)

// ================== UI 布局全局变量 ==================
atomic<int> g_enlarged_cam{-1};
int g_win_w = 1224, g_win_h = 1024;
int g_left_w = 0, g_right_x = 0, g_right_w = 0;
int g_thumb_w = 0, g_thumb_h = 0;

// ================== [新增] 核心：极速零延迟触发函数 ==================
void instantTrigger() {
    global_record_start_time = std::chrono::steady_clock::now();
    g_exc_fatal = 0; g_exc_error = 0; g_exc_warn = 0; g_exc_info = 0;
    for (auto& ctx : cam_ctxs) {
        {
            lock_guard<mutex> lock(ctx->copy_mtx);
            while (!ctx->copy_queue.empty()) ctx->copy_queue.pop();
        }
        ctx->recorded_frames.store(0, std::memory_order_relaxed);
        ctx->dump_ready.store(false, std::memory_order_relaxed);
        ctx->recording.store(true, std::memory_order_release);

        // ---- reset per-camera metrics ----
        ctx->max_queue_size = 0;
        ctx->first_recorded_block_id = -1;
        ctx->last_recorded_block_id = -1;
        ctx->dropped_frames = 0;
        ctx->prev_block_id = -1;
        ctx->recover2ram_s = 0.0;
        ctx->wait4disk_s = 0.0;
        ctx->ram2disk_s = 0.0;
    }
}

// ================== 网络发送模块 (Master) ==================
// [修改] 替换为内联的长连接发送函数，消除 socket 创建开销
inline void fastUdpSend(const string& msg) {
    if (master_udp_sock != INVALID_SOCKET) {
        sendto(master_udp_sock, msg.c_str(), msg.length(), 0, (sockaddr*)&slave_udp_addr, sizeof(slave_udp_addr));
    }
}

// ================== 网络同步与协商模块 (TCP 可靠传输) ==================
bool syncGlobalBlockIDTCP(bool is_master, const string& master_ip, int port, int64_t& local_start, int64_t& local_end) {
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) return false;

    int64_t my_range[2] = { local_start, local_end };
    int64_t other_range[2] = { 0, 0 };

    if (is_master) {
        // Master 监听 port + 1 端口，等待 Slave 连接
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port + 1);

        ::bind(sock, (sockaddr*)&server_addr, sizeof(server_addr));
        listen(sock, 1);
        
        cout << "[Net Sync] Master is dumping faster. Waiting for Slave blockID..." << endl;
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        SOCKET client_sock = accept(sock, (sockaddr*)&client_addr, &client_len);
        
        // 1. 接收 Slave 的边界
        recv(client_sock, (char*)other_range, sizeof(other_range), 0);
        g_peer_first_block_id = other_range[0];  // cross-host sync offset

        // 2. Master 计算真正的全局交集
        my_range[0] = std::max(my_range[0], other_range[0]);
        my_range[1] = std::min(my_range[1], other_range[1]);
        
        // 3. 将结果发回给 Slave
        send(client_sock, (const char*)my_range, sizeof(my_range), 0);
        
        closesocket(client_sock);
        local_start = my_range[0];
        local_end = my_range[1];
    } else {
        // Slave 主动连接 Master
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port + 1);
        inet_pton(AF_INET, master_ip.c_str(), &server_addr.sin_addr);

        cout << "[Net Sync] Slave connecting to Master to sync blockID..." << endl;
        // 循环等待，直到 Master 准备好接收
        while (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
        }
        
        // 1. 发送自己的边界给 Master
        send(sock, (const char*)my_range, sizeof(my_range), 0);
        // 2. 接收 Master 计算好的全局边界
        recv(sock, (char*)other_range, sizeof(other_range), 0);
        g_peer_first_block_id = other_range[0];  // cross-host sync offset (intersection start)

        local_start = other_range[0];
        local_end = other_range[1];
    }
    closesocket(sock);
    return true;
}

// ================== 网络监听模块 (Slave - 提权至最高优先级) ==================
void udpListenerWorker(const string& bind_ip, int port) {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#else
    pthread_t this_thread = pthread_self();
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(this_thread, SCHED_FIFO, &params);
#endif

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) return;

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&optval, sizeof(optval));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, bind_ip.c_str(), &server_addr.sin_addr);

    if (::bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        cerr << "[Slave ERROR] Bind failed on " << bind_ip << ":" << port << endl;
        closesocket(sock);
        return;
    }

#ifdef _WIN32
    DWORD timeout = 100; // 100ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif

    char buffer[256];
    cout << "[Net Sync] Priority UDP Listener Bound to " << bind_ip << ":" << port << endl;

    while (global_running) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int bytes = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&client_addr, &client_len);
        
        if (bytes > 0) {
            buffer[bytes] = '\0';
            string cmd(buffer);
            // [修改] 识别带时间戳的指令，瞬间开火
            if (cmd.rfind("CMD_START:", 0) == 0) {
                if (cmd.length() <= 10) { logException("WARN", "slave", "CMD_START truncated"); continue; }
                instantTrigger();
                shared_record_timestr = cmd.substr(10);
                net_cmd_record = true;
            }
            else if (cmd.rfind("FAULT:", 0) == 0 && !g_fault_active.load()) {
                if (cmd.length() <= 7) continue;
                char hf = cmd[6]; int fi = stoi(cmd.substr(7));
                if (fi < 0 || fi >= (int)cam_ctxs.size()) {
                    logException("WARN", "slave", "FAULT cam idx out of bounds: " + to_string(fi));
                    continue;
                }
                cout << "[Slave] Fault from MASTER: cam " << fi << endl;
                g_fault_time = chrono::steady_clock::now();
                g_fault_active.store(true); g_faulty_cam.store(fi); g_fault_on_master.store(true);
                for (auto& ctx : cam_ctxs) {
                    ctx->running = false;
                    ctx->copy_cv.notify_all();
                }
                for (auto& ctx : cam_ctxs) {
                    if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
                    if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
                }
                cout << "[Fault] All cameras stopped. Press ESC to exit." << endl;
            }
            else if (cmd == "SHUTDOWN") {
                cout << "[Slave] Received SHUTDOWN from master." << endl;
                global_running = false;
            }
        } 
    }
    closesocket(sock);
}

// ================== 相机通用功能 ==================
int getNextCalibCounter(const std::string& save_dir) {
    int max_counter = -1;
    if (!fs::exists(save_dir)) return 0;
    for (auto& e : fs::directory_iterator(save_dir)) {
        if (e.path().extension() == ".jpg") {
            try {
                string stem = e.path().stem().string();
                size_t last_underscore = stem.find_last_of('_');
                if (last_underscore != string::npos) {
                     string num_part = stem.substr(last_underscore + 1);
                     max_counter = max(max_counter, stoi(num_part));
                }
            } catch (...) {}
        }
    }
    return max_counter + 1;
}

// ================== UI 布局计算 ==================
void updateLayout() {
    g_left_w = g_win_h * 2 / 5;
    g_right_x = g_left_w;
    g_right_w = g_win_w - g_left_w;
    g_thumb_w = g_left_w / 2;
    g_thumb_h = g_win_h / 5;
}

// ================== 鼠标回调 ==================
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

// ================== UI 渲染函数 ==================
void renderThumbnailGrid(cv::Mat& canvas, int selected_idx, bool is_recording,
                         const chrono::steady_clock::time_point& record_start_time, int total_record_frames) {
    int n = static_cast<int>(cam_ctxs.size());
    for (int i = 0; i < 10; ++i) {
        int row = i / 2;
        int col = i % 2;
        int x = col * g_thumb_w;
        int y = row * g_thumb_h;
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
                int dw = static_cast<int>(cell.cols * scale);
                int dh = static_cast<int>(cell.rows * scale);
                cv::Mat resized;
                cv::resize(cell, resized, cv::Size(dw, dh));
                cell = cv::Mat::zeros(g_thumb_h, g_thumb_w, CV_8UC3);
                int ox = (g_thumb_w - dw) / 2;
                int oy = (g_thumb_h - dh) / 2;
                resized.copyTo(cell(cv::Rect(ox, oy, dw, dh)));
            } else {
                cell = cv::Mat::zeros(g_thumb_h, g_thumb_w, CV_8UC3);
                int baseline = 0;
                cv::Size textSize = cv::getTextSize(cam_ctxs[i]->status_msg, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                cv::Point textOrg((g_thumb_w - textSize.width) / 2, (g_thumb_h + textSize.height) / 2);
                cv::putText(cell, cam_ctxs[i]->status_msg, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }

            // Recording indicator on cell
            if (is_recording) {
                int radius = 6;
                cv::Point center(g_thumb_w - radius * 2, radius * 2);
                cv::circle(cell, center, radius, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                int current_frame = cam_ctxs[i]->recorded_frames.load(std::memory_order_relaxed);
                cv::putText(cell, to_string(current_frame) + "/" + to_string(total_record_frames),
                            cv::Point(4, 14), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 2);
                cv::putText(cell, to_string(current_frame) + "/" + to_string(total_record_frames),
                            cv::Point(4, 14), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 0), 1);
            }

            // SN label
            string label = cam_ctxs[i]->id;
            int baseline = 0;
            double font_scale = 0.4;
            cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline);
            cv::Point org((g_thumb_w - ts.width) / 2, g_thumb_h - 5);
            cv::putText(cell, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(0, 0, 0), 3);
            cv::putText(cell, label, org, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(255, 255, 255), 1);

            cell.copyTo(canvas(roi));

            // Green border when selected
            if (i == selected_idx) {
                cv::rectangle(canvas, roi, cv::Scalar(0, 255, 0), 2);
            }
        } else {
            canvas(roi) = cv::Scalar(0, 0, 0);
        }
    }
}

void renderEnlargedView(cv::Mat& canvas, int cam_idx, bool is_recording,
                        const chrono::steady_clock::time_point& record_start_time, int total_record_frames) {
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

    if (local_raw.empty()) {
        canvas(right_roi) = cv::Scalar(0, 0, 0);
        return;
    }

    cv::Mat img;
    if (cam_ctxs[cam_idx]->is_mono) cv::cvtColor(local_raw, img, cv::COLOR_GRAY2RGB);
    else cv::cvtColor(local_raw, img, cv::COLOR_BayerRG2RGB);

    double scale = min(static_cast<double>(g_right_w) / img.cols,
                       static_cast<double>(g_win_h) / img.rows);
    int dst_w = static_cast<int>(img.cols * scale);
    int dst_h = static_cast<int>(img.rows * scale);

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(dst_w, dst_h));

    int off_x = g_right_x + (g_right_w - dst_w) / 2;
    int off_y = (g_win_h - dst_h) / 2;

    // Recording info on enlarged view
    if (is_recording) {
        int radius = 14;
        cv::Point center(off_x + dst_w - radius * 2, off_y + radius * 2);
        cv::circle(resized, cv::Point(dst_w - radius * 2, radius * 2), radius, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::putText(resized, "REC", cv::Point(dst_w - radius * 10, radius * 3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

        int current_frame = cam_ctxs[cam_idx]->recorded_frames.load(std::memory_order_relaxed);
        double elapsed_s = chrono::duration<double>(chrono::steady_clock::now() - record_start_time).count();
        char buf[64]; snprintf(buf, sizeof(buf), "%.1fs", elapsed_s);

        cv::putText(resized, "Frame: " + to_string(current_frame) + "/" + to_string(total_record_frames),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
        cv::putText(resized, "Frame: " + to_string(current_frame) + "/" + to_string(total_record_frames),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(resized, string(buf), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
        cv::putText(resized, string(buf), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Live FPS (show after 10+ frames to avoid startup jitter)
        if (current_frame > 10 && elapsed_s > 0.01) {
            double live_fps = current_frame / elapsed_s;
            char fps_buf[32]; snprintf(fps_buf, sizeof(fps_buf), "%.1f fps", live_fps);
            cv::putText(resized, string(fps_buf), cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
            cv::putText(resized, string(fps_buf), cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
    }

    canvas(right_roi) = cv::Scalar(0, 0, 0);
    resized.copyTo(canvas(cv::Rect(off_x, off_y, dst_w, dst_h)));
}

void showFaultOverlay(int faulty_cam, bool is_hw) {
    cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
    int cx = g_win_w / 2, y = g_win_h / 2 - 80;
    auto put = [&](int y, const string& text, double s, cv::Scalar c) {
        int bl; cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, s, 2, &bl);
        cv::putText(canvas, text, cv::Point(cx - sz.width / 2, y), cv::FONT_HERSHEY_SIMPLEX, s, c, 2);
    };
    put(y, "CAMERA FAULT DETECTED", 1.0, cv::Scalar(0, 0, 255));
    y += 40;
    string ci = "Camera: " + (faulty_cam >= 0 && faulty_cam < (int)cam_ctxs.size() ? cam_ctxs[faulty_cam]->id : "?")
                + " (index " + to_string(faulty_cam) + ")";
    put(y, ci, 0.7, cv::Scalar(255, 255, 255));
    y += 30;
    put(y, string("Host: ") + (g_fault_on_master.load() ? "MASTER" : "SLAVE"), 0.7, cv::Scalar(255, 255, 255));
    y += 30;
    auto uptime_s = chrono::duration<double>(g_fault_time - g_ready_time).count();
    int h = (int)uptime_s / 3600, m = ((int)uptime_s % 3600) / 60;
    char ubuf[64]; snprintf(ubuf, sizeof(ubuf), "Uptime: %dh %dm", h, m);
    put(y, string(ubuf), 0.7, cv::Scalar(255, 255, 255));
    y += 40;
    put(y, "All cameras stopped. Press ESC to exit both hosts.", 0.6, cv::Scalar(0, 255, 255));
    cv::imshow("Multi-Cam Preview", canvas);
    cv::waitKey(1);
}

// ================== 后台异步拷贝线程 ==================
void copyWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        pair<Pylon::CBaslerUniversalGrabResultPtr, FrameMeta> task;
        {
            unique_lock<mutex> lock(ctx->copy_mtx);
            ctx->copy_cv.wait(lock, [&]{ return !ctx->copy_queue.empty() || !ctx->running; });
            if (!ctx->running && ctx->copy_queue.empty()) break;
            // ---- queue pressure: record BEFORE pop (total queue length) ----
            int qs = (int)ctx->copy_queue.size();
            if (qs > ctx->max_queue_size.load()) ctx->max_queue_size = qs;
            task = ctx->copy_queue.front();
            ctx->copy_queue.pop();
        }

        if (ctx->recording) {
            int seq = ctx->recorded_frames.load(std::memory_order_relaxed);
            if (seq < ctx->total_record_frames) {
                void* pBuffer = task.first->GetBuffer();
                size_t payload_size = task.first->GetWidth() * task.first->GetHeight();

                memcpy(ctx->ram_buffer[seq].data, pBuffer, payload_size);
                ctx->meta_buffer[seq] = task.second;

                {
                    lock_guard<mutex> lock(ctx->frame_mtx);
                    ctx->latest_frame = ctx->ram_buffer[seq];
                    ctx->latest_meta = task.second;
                }

                // ---- metrics: first frame ----
                if (seq == 0) {
                    ctx->first_recorded_block_id = task.second.blockID;
                    ctx->first_frame_time = chrono::steady_clock::now();
                }
                // ---- metrics: last frame + drop detection ----
                ctx->last_recorded_block_id = task.second.blockID;
                if (ctx->prev_block_id != -1) {
                    int64_t diff = task.second.blockID - ctx->prev_block_id;
                    if (diff > 1) ctx->dropped_frames += (int)(diff - 1);
                }
                ctx->prev_block_id = task.second.blockID;

                int next_seq = seq + 1;
                ctx->recorded_frames.store(next_seq, std::memory_order_relaxed);

                if (next_seq == ctx->total_record_frames) {
                    ctx->recording = false;
                    ctx->dump_ready = true;
                    ctx->recording_end_time = chrono::steady_clock::now();  // t_last
                }
            }
            ctx->last_block_id.store(task.second.blockID, memory_order_relaxed);
            ctx->last_frame_time.store(chrono::steady_clock::now(), memory_order_relaxed);
            ctx->has_streamed.store(true, memory_order_relaxed);
        } else {
            cv::Mat temp(task.first->GetHeight(), task.first->GetWidth(), CV_8UC1, task.first->GetBuffer());
            cv::Mat clone_img = temp.clone();
            {
                lock_guard<mutex> lock(ctx->frame_mtx);
                ctx->latest_frame = clone_img;
                ctx->latest_meta = task.second;
            }
            ctx->last_block_id.store(task.second.blockID, memory_order_relaxed);
            ctx->last_frame_time.store(chrono::steady_clock::now(), memory_order_relaxed);
            ctx->has_streamed.store(true, memory_order_relaxed);
        }
    }
}

// ================== Pylon 回调触发线程 ==================
// [修改] 增加 enable_offset 参数
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time, bool use_hw_trigger, bool enable_offset) {
    TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
    if (!ctx->cam.open(mode)) { ctx->status = CamStatus::ERROR_; ctx->copy_cv.notify_all(); return; }

    ctx->is_mono = ctx->cam.isMono();

    try {
        if (!use_hw_trigger) ctx->cam.setFrameRate(fps);  
        ctx->cam.setGain(gain);                           
        ctx->cam.setGamma(gamma);                         
        ctx->cam.setExposureTime(exp_time);               
    } catch (...) {}

    struct GrabState { int64_t frame_counter = 0; };
    auto state = make_shared<GrabState>();

    // [修改] 将 enable_offset 捕获进 lambda
    ctx->cam.setFrameCallback([ctx, state, use_hw_trigger, enable_offset](const Pylon::CBaslerUniversalGrabResultPtr& ptr, FrameMeta meta) {
        state->frame_counter++;
        ctx->status = CamStatus::STREAMING;

        if (!ctx->offset_initialized && state->frame_counter > 1) {
            // [修改] 如果是硬件触发 或者 软件触发但未开启偏移补偿，直接将偏移置0
            if (use_hw_trigger || !enable_offset) {
                ctx->frame_offset = 0;
                ctx->offset_initialized = true;
            } else {
                // 原有逻辑：软件触发下利用全局变量寻找第一帧差值
                if (!CameraContext::master_set.exchange(true)) {
                    CameraContext::master_first_id = meta.blockID;
                    ctx->frame_offset = 0;
                    ctx->offset_initialized = true;
                } else {
                    int retry = 0;
                    while (CameraContext::master_first_id == -1 && retry < 100) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        retry++;
                    }
                    if (CameraContext::master_first_id != -1) {
                        ctx->frame_offset = meta.blockID - CameraContext::master_first_id.load();
                        ctx->offset_initialized = true;
                    }
                }
            }
        }

        if (ctx->offset_initialized) {
            // 对于硬件触发(offset=0)，blockID自然保持严格一致
            meta.blockID = meta.blockID - ctx->frame_offset; 
            ctx->captured_frames++;

            lock_guard<mutex> lock(ctx->copy_mtx);
            if (ctx->recording) {
                if ((int)ctx->copy_queue.size() < ctx->total_record_frames) {
                    ctx->copy_queue.push({ptr, meta});
                    ctx->copy_cv.notify_one();
                }
                // else: queue full, frame silently dropped (will show as BlockID gap)
            } else {
                if (ctx->copy_queue.size() < 2) {
                    ctx->copy_queue.push({ptr, meta});
                    ctx->copy_cv.notify_one();
                }
            }
        }
    });

    if (!ctx->cam.start()) { ctx->status = CamStatus::ERROR_; ctx->copy_cv.notify_all(); return; }
    ctx->status_msg = use_hw_trigger ? "HW WAITING" : "STREAMING";

    while (ctx->running) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ctx->cam.close();
}

void dumpToHdf5Worker(shared_ptr<CameraContext> ctx, int core_frames, int margin_frames,
                       atomic<int>& finished_cams) {
    cout << "[DEBUG-HDF5] cam" << ctx->index << " dumpToHdf5Worker START" << endl;
    ctx->dump_start_time = chrono::steady_clock::now();
    int N = core_frames;

    if (ctx->ram_buffer.empty() || N <= 0) {
        cout << "[DEBUG-HDF5] cam" << ctx->index << " ram_buffer EMPTY, skipping" << endl;
        ctx->dump_end_time = chrono::steady_clock::now();
        finished_cams++;
        return;
    }

    try {
        int cam_h = ctx->ram_buffer[0].rows;
        int cam_w = ctx->ram_buffer[0].cols;
        cout << "[DEBUG-HDF5] cam" << ctx->index << " writing " << N << " frames offset=" << g_frame_offset << " (per-frame, zero-copy)" << endl;
        auto t1 = chrono::steady_clock::now();

        // Write raw_image frame-by-frame directly from ram_buffer — no extra buffer
        hsize_t f_start[3], f_count[3];
        f_start[1] = 0; f_start[2] = 0;
        f_count[0] = 1; f_count[1] = (hsize_t)cam_h; f_count[2] = (hsize_t)cam_w;
        H5::DataSpace f_mem(3, f_count);
        H5::DataSpace f_file = ctx->hdf5_raw_ds.getSpace();
        for (int i = 0; i < N; ++i) {
            int src_idx = margin_frames + i;
            f_start[0] = (hsize_t)(g_frame_offset + i);
            f_file.selectHyperslab(H5S_SELECT_SET, f_count, f_start);
            ctx->hdf5_raw_ds.write(ctx->ram_buffer[src_idx].data, H5::PredType::NATIVE_UINT8, f_mem, f_file);
        }
        auto t2 = chrono::steady_clock::now();
        cout << "[DEBUG-HDF5] cam" << ctx->index << " raw_image (" << N << " frames) done in " << chrono::duration<double>(t2-t1).count() << "s" << endl;

        // Write gaze_target and valid as single hyperslabs (small data, negligible memory)
        hsize_t gz_start[2] = {(hsize_t)g_frame_offset, 0};
        hsize_t gz_count[2] = {(hsize_t)N, 2};
        H5::DataSpace gz_mem(2, gz_count);
        H5::DataSpace gz_file = ctx->hdf5_gaze_ds.getSpace();
        gz_file.selectHyperslab(H5S_SELECT_SET, gz_count, gz_start);
        vector<double> gz_buf(N * 2, 0.0);
        ctx->hdf5_gaze_ds.write(gz_buf.data(), H5::PredType::NATIVE_DOUBLE, gz_mem, gz_file);

        hsize_t v_start[1] = {(hsize_t)g_frame_offset};
        hsize_t v_count[1] = {(hsize_t)N};
        H5::DataSpace v_mem(1, v_count);
        H5::DataSpace v_file = ctx->hdf5_valid_ds.getSpace();
        v_file.selectHyperslab(H5S_SELECT_SET, v_count, v_start);
        vector<uint8_t> v_buf(N, 1);
        ctx->hdf5_valid_ds.write(v_buf.data(), H5::PredType::NATIVE_UINT8, v_mem, v_file);

        auto t3 = chrono::steady_clock::now();
        cout << "[DEBUG-HDF5] cam" << ctx->index << " ALL done in " << chrono::duration<double>(t3-t1).count() << "s" << endl;
    } catch (const H5::Exception& e) {
        cout << "[DEBUG-HDF5] cam" << ctx->index << " EXCEPTION: " << e.getCDetailMsg() << endl;
        logException("ERROR", "hdf5:cam" + to_string(ctx->index),
                     string("HDF5 write failed: ") + e.getCDetailMsg());
    }

    ctx->dump_end_time = chrono::steady_clock::now();
    finished_cams++;
}

vector<LogEntry> parseLogFile(const string& log_path) {
    vector<LogEntry> entries;
    ifstream infile(log_path);
    string line;
    while (getline(infile, line)) {
        stringstream ss(line); string segment; vector<string> parts;
        while (getline(ss, segment, ',')) parts.push_back(segment);
        if (parts.size() >= 6) entries.push_back({parts[0], stoll(parts[1]), stoll(parts[2]), stoi(parts[4]), stoi(parts[5])});
    }
    return entries;
}

void convertRawToJpgWorker(string temp_raw_dir, string out_jpg_dir, vector<LogEntry> valid_entries, atomic<int>& global_processed, bool is_mono, chrono::steady_clock::time_point* jpg_start = nullptr, chrono::steady_clock::time_point* jpg_end = nullptr) {
    if (jpg_start) *jpg_start = chrono::steady_clock::now();
    if (valid_entries.empty()) { if (jpg_end) *jpg_end = chrono::steady_clock::now(); return; }
    fs::create_directories(out_jpg_dir);
    for (const auto& entry : valid_entries) {
        string raw_path = temp_raw_dir + "/" + entry.filename;
        cv::Mat raw_img(entry.height, entry.width, CV_8UC1);
        ifstream in_raw(raw_path, ios::binary);
        if (in_raw) {
            in_raw.read(reinterpret_cast<char*>(raw_img.data), raw_img.total() * raw_img.elemSize());
            in_raw.close();

            cv::Mat final_img;
            if (is_mono) {
                final_img = raw_img.clone();
            } else {
                cv::cvtColor(raw_img, final_img, cv::COLOR_BayerRG2RGB);
            }

            string jpg_filename = to_string(entry.blockID) + ".jpg";
            if (!cv::imwrite(out_jpg_dir + "/" + jpg_filename, final_img))
                logException("ERROR", "jpg", "cv::imwrite failed: " + out_jpg_dir + "/" + jpg_filename);
        }
        global_processed++;
    }
    if (jpg_end) *jpg_end = chrono::steady_clock::now();
}

// ================== HDF5 辅助 ==================
static void h5WriteInt(H5::H5File& f, const string& name, int v) {
    H5::DataSpace ds(H5S_SCALAR);
    auto d = f.createDataSet(name, H5::PredType::NATIVE_INT, ds);
    d.write(&v, H5::PredType::NATIVE_INT);
}
static int h5ReadInt(H5::H5File& f, const string& name) {
    auto d = f.openDataSet(name);
    int v; d.read(&v, H5::PredType::NATIVE_INT);
    return v;
}

static void openOrCreateChunk(shared_ptr<CameraContext> ctx, int cam_h, int cam_w, int cap) {
    stringstream ss; ss << ctx->hdf5_dir << "/" << setw(4) << setfill('0') << g_chunk_idx << ".h5";
    string path = ss.str();
    bool exists = fs::exists(path);

    auto create = [&]() {
        ctx->hdf5_file = new H5::H5File(path, H5F_ACC_TRUNC);
        hsize_t rd[3] = {(hsize_t)cap, (hsize_t)cam_h, (hsize_t)cam_w};
        ctx->hdf5_raw_ds = ctx->hdf5_file->createDataSet("raw_image", H5::PredType::NATIVE_UINT8, H5::DataSpace(3, rd));
        hsize_t gd[2] = {(hsize_t)cap, 2};
        ctx->hdf5_gaze_ds = ctx->hdf5_file->createDataSet("gaze_target", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, gd));
        hsize_t vd[1] = {(hsize_t)cap};
        ctx->hdf5_valid_ds = ctx->hdf5_file->createDataSet("valid", H5::PredType::NATIVE_UINT8, H5::DataSpace(1, vd));
    };

    if (exists) {
        try {
            ctx->hdf5_file = new H5::H5File(path, H5F_ACC_RDWR);
            ctx->hdf5_raw_ds = ctx->hdf5_file->openDataSet("raw_image");
            ctx->hdf5_gaze_ds = ctx->hdf5_file->openDataSet("gaze_target");
            ctx->hdf5_valid_ds = ctx->hdf5_file->openDataSet("valid");
        } catch (const H5::Exception&) {
            delete ctx->hdf5_file;
            fs::remove(path);
            create();
        }
    } else {
        create();
    }
}

static void closeChunk(shared_ptr<CameraContext> ctx) {
    if (!ctx->hdf5_file) return;
    ctx->hdf5_raw_ds.close();
    ctx->hdf5_gaze_ds.close();
    ctx->hdf5_valid_ds.close();
    ctx->hdf5_file->close();
    delete ctx->hdf5_file;
    ctx->hdf5_file = nullptr;
}

static void initSentry(const string& root) {
    fs::create_directories(root);
    string sp = root + "/sentry.h5";
    if (fs::exists(sp)) {
        H5::H5File f(sp, H5F_ACC_RDONLY);
        g_chunk_idx = h5ReadInt(f, "chunk_idx");
        g_frame_offset = h5ReadInt(f, "frame_offset");
    } else {
        H5::H5File f(sp, H5F_ACC_TRUNC);
        h5WriteInt(f, "chunk_idx", 0);
        h5WriteInt(f, "frame_offset", 0);
    }
}

static void updateSentry(const string& root) {
    string sp = root + "/sentry.h5";
    H5::H5File f(sp, H5F_ACC_RDWR);
    f.unlink("chunk_idx");
    f.unlink("frame_offset");
    h5WriteInt(f, "chunk_idx", g_chunk_idx);
    h5WriteInt(f, "frame_offset", g_frame_offset);
}

// ================== 结构化报告写入 ==================
void writeReport(const string& timestr, int rec_num, double target_fps, int total_frames,
                 double dump_wall_s, double jpg_wall_s, bool write_jpg, bool hw_trigger) {
    if (!g_session_log.is_open()) return;

    g_session_log << "\n---\n\n"
                  << "## Recording #" << rec_num << ": " << timestr << "\n\n"
                  << "- **Cameras**: " << cam_ctxs.size() << "\n"
                  << "- **Trigger**: " << (hw_trigger ? "HW" : "SW") << "\n"
                  << "- **Target FPS**: " << fixed << setprecision(1) << target_fps << "\n"
                  << "- **Total frames**: " << total_frames << "\n"
                  << "- **write_jpg**: " << (write_jpg ? "true" : "false") << "\n\n";

    // --- Per-Camera Metrics (Markdown table) ---
    g_session_log << "### Per-Camera Metrics\n\n";

    double theoretical_s = total_frames / target_fps;  // seconds

    if (write_jpg) {
        g_session_log << "| # | SN | Type | Saved | Drop | FPS | QPeak | Lat(ms) | FirstBlk | LastBlk | SyncOff | Jitter(us) | RecOver2RAM(s) | Wait4Disk(s) | Ram2Disk(s) | RecOver2Disk(s) | Wait4JPG(s) | JPGTime(s) | RecOver2JPG(s) |\n";
        g_session_log << "|---|-----|------|-------|------|-----|-------|---------|----------|----------|---------|------------|----------------|--------------|-------------|-----------------|--------------|------------|-----------------|\n";
        g_session_log << "|   |     | mono/color | 实际保存帧数 | BlockID跳变丢帧 | 平均帧率 | 队列峰值/总帧数 | 首帧触发延迟 | 录制首帧BlockID | 录制末帧BlockID | 跨主机BlockID差(本机首帧-对端首帧) | Pylon时间戳间隔标准差(us) | RAM写完−理论完成 | RAM写完→DISK开始 | DISK开始→DISK结束 | ①+②+③ | DISK完成→JPG开始 | JPG耗时 | ①②③+④+⑤ |\n";
    } else {
        g_session_log << "| # | SN | Type | Saved | Drop | FPS | QPeak | Lat(ms) | FirstBlk | LastBlk | SyncOff | Jitter(us) | RecOver2RAM(s) | Wait4Disk(s) | Ram2Disk(s) | RecOver2Disk(s) |\n";
        g_session_log << "|---|-----|------|-------|------|-----|-------|---------|----------|----------|---------|------------|----------------|--------------|-------------|-----------------|\n";
        g_session_log << "|   |     | mono/color | 实际保存帧数 | BlockID跳变丢帧 | 平均帧率 | 队列峰值/总帧数 | 首帧触发延迟 | 录制首帧BlockID | 录制末帧BlockID | 跨主机BlockID差(本机首帧-对端首帧) | Pylon时间戳间隔标准差(us) | RAM写完−理论完成 | RAM写完→DISK开始 | DISK开始→DISK结束 | ①+②+③ |\n";
    }

    int64_t ref_first_blk = -1;
    if (!cam_ctxs.empty()) ref_first_blk = cam_ctxs[0]->first_recorded_block_id;

    for (auto& ctx : cam_ctxs) {
        int saved = ctx->recorded_frames.load();
        int dropped = ctx->dropped_frames.load();
        double fps = 0.0, jitter_us = 0.0;

        if (saved > 1) {
            double dur_s = (ctx->meta_buffer[saved-1].timestamp - ctx->meta_buffer[0].timestamp) / 10000000.0;
            if (dur_s > 0) fps = (saved - 1) / dur_s;
            double sum = 0, sum_sq = 0; int n = saved - 1;
            for (int k = 1; k < saved; ++k) {
                double dt = (ctx->meta_buffer[k].timestamp - ctx->meta_buffer[k-1].timestamp) / 10.0;
                sum += dt; sum_sq += dt * dt;
            }
            if (n > 0) { double mean = sum / n; double var = sum_sq / n - mean * mean; if (var > 0) jitter_us = sqrt(var); }
        }

        double lat_ms = 0.0;
        if (ctx->first_frame_time.time_since_epoch().count() > 0)
            lat_ms = chrono::duration<double, milli>(ctx->first_frame_time - global_record_start_time).count();

        // RecOver2RAM(s) = actual RAM duration - theoretical
        double actual_ram_s = chrono::duration<double>(ctx->recording_end_time - ctx->first_frame_time).count();
        ctx->recover2ram_s = actual_ram_s - theoretical_s;

        // Wait4Disk(s) = dump_start - recording_end_time (how long this camera waited before disk write started)
        ctx->wait4disk_s = chrono::duration<double>(ctx->dump_start_time - ctx->recording_end_time).count();

        // Ram2Disk(s) = dump_end - dump_start (actual disk write duration)
        ctx->ram2disk_s = chrono::duration<double>(ctx->dump_end_time - ctx->dump_start_time).count();

        int64_t sync_off = (hw_trigger && g_peer_first_block_id >= 0 && ctx->first_recorded_block_id >= 0)
                         ? (ctx->first_recorded_block_id - g_peer_first_block_id) : 0;
        string sync_str = hw_trigger ? to_string(sync_off) : "N/A";

        auto fmt3 = [] (double v) { ostringstream oss; oss << fixed << setprecision(3) << v; return oss.str(); };

        double rec_over2disk = ctx->recover2ram_s + ctx->wait4disk_s + ctx->ram2disk_s;

        g_session_log << "| " << ctx->index << " | " << ctx->id << " | "
                      << (ctx->is_mono ? "mono" : "color") << " | "
                      << saved << " | " << dropped << " | "
                      << fixed << setprecision(1) << fps << " | "
                      << ctx->max_queue_size.load() << "/" << total_frames << " | "
                      << fixed << setprecision(1) << lat_ms << " | "
                      << ctx->first_recorded_block_id << " | " << ctx->last_recorded_block_id << " | "
                      << sync_str << " | "
                      << fixed << setprecision(1) << jitter_us << " | "
                      << fmt3(ctx->recover2ram_s) << " | "
                      << fmt3(ctx->wait4disk_s) << " | "
                      << fmt3(ctx->ram2disk_s) << " | "
                      << fmt3(rec_over2disk) << " | ";

        if (write_jpg) {
            double wait4jpg = 0, jpg_time = 0;
            if (ctx->jpg_start_time.time_since_epoch().count() > 0 && ctx->dump_end_time.time_since_epoch().count() > 0)
                wait4jpg = chrono::duration<double>(ctx->jpg_start_time - ctx->dump_end_time).count();
            if (ctx->jpg_end_time.time_since_epoch().count() > 0 && ctx->jpg_start_time.time_since_epoch().count() > 0)
                jpg_time = chrono::duration<double>(ctx->jpg_end_time - ctx->jpg_start_time).count();
            double rec_over2jpg = rec_over2disk + wait4jpg + jpg_time;
            g_session_log << fmt3(wait4jpg) << " | " << fmt3(jpg_time) << " | " << fmt3(rec_over2jpg) << " |\n";
        } else {
            g_session_log << "\n";
        }
        g_session_log << defaultfloat;
    }
    g_session_log << "\n";

    // --- Summary (Markdown table) ---
    g_session_log << "\n### Summary\n\n";
    int total_expected = (int)cam_ctxs.size() * total_frames;
    int total_saved = 0, total_dropped = 0;
    double min_fps = 1e9; int min_fps_cam = -1;
    int peak_q = 0, peak_q_cam = -1;
    double max_jitter = 0; int max_jitter_cam = -1;
    double max_ram2disk = 0; int max_ram2disk_cam = -1;
    double max_total = 0; int max_total_cam = -1;
    int64_t max_sync = 0;

    for (auto& ctx : cam_ctxs) {
        int s = ctx->recorded_frames.load();
        int d = ctx->dropped_frames.load();
        total_saved += s; total_dropped += d;
        if (s > 1) {
            double dur = (ctx->meta_buffer[s-1].timestamp - ctx->meta_buffer[0].timestamp) / 10000000.0;
            double f = dur > 0 ? (s - 1) / dur : 0;
            if (f < min_fps) { min_fps = f; min_fps_cam = ctx->index; }
        }
        int q = ctx->max_queue_size.load();
        if (q > peak_q) { peak_q = q; peak_q_cam = ctx->index; }
        if (ctx->ram2disk_s > max_ram2disk) { max_ram2disk = ctx->ram2disk_s; max_ram2disk_cam = ctx->index; }
        double total_s = ctx->recover2ram_s + ctx->wait4disk_s + ctx->ram2disk_s;
        if (write_jpg && ctx->jpg_start_time.time_since_epoch().count() > 0 && ctx->jpg_end_time.time_since_epoch().count() > 0)
            total_s += chrono::duration<double>(ctx->jpg_end_time - ctx->dump_end_time).count();
        if (total_s > max_total) { max_total = total_s; max_total_cam = ctx->index; }

        if (s > 1) {
            double sum = 0, sum_sq = 0; int n = s - 1;
            for (int k = 1; k < s; ++k) { double dt = (ctx->meta_buffer[k].timestamp - ctx->meta_buffer[k-1].timestamp) / 10.0; sum += dt; sum_sq += dt * dt; }
            if (n > 0) { double mean = sum / n; double j = sqrt(sum_sq / n - mean * mean); if (j > max_jitter) { max_jitter = j; max_jitter_cam = ctx->index; } }
        }
        if (hw_trigger && g_peer_first_block_id >= 0 && ctx->first_recorded_block_id >= 0) {
            int64_t off = ctx->first_recorded_block_id - g_peer_first_block_id;
            if (llabs(off) > max_sync) max_sync = llabs(off);
        }
    }

    double completion = total_expected > 0 ? 100.0 * total_saved / total_expected : 0;
    double min_lat = 1e9, max_lat = 0;
    for (auto& ctx : cam_ctxs) {
        if (ctx->first_frame_time.time_since_epoch().count() > 0) {
            double lat = chrono::duration<double, milli>(ctx->first_frame_time - global_record_start_time).count();
            if (lat < min_lat) min_lat = lat; if (lat > max_lat) max_lat = lat;
        }
    }
    bool healthy = (max_sync <= 2) && (min_fps >= target_fps * 0.95) && (total_dropped <= total_expected * 0.01);

    g_session_log << "| 指标 | 值 | 说明 |\n";
    g_session_log << "|------|-----|------|\n";
    g_session_log << "| total_expected | " << total_expected << " | 预期总帧数 |\n";
    g_session_log << "| total_saved | " << total_saved << " (" << fixed << setprecision(1) << completion << "%) | 实际保存总帧数 |\n";
    g_session_log << "| total_dropped | " << total_dropped << " | 累计丢帧数 |\n";
    g_session_log << "| bottleneck_fps | " << fixed << setprecision(1) << min_fps << " | 最慢相机 (cam " << min_fps_cam << ": " << (min_fps_cam >= 0 ? cam_ctxs[min_fps_cam]->id : "?") << ") |\n";
    g_session_log << "| max_sync_offset | " << max_sync << " BlockID | 跨主机 BlockID 差" << (hw_trigger ? "" : " (N/A)") << " |\n";
    g_session_log << "| trigger_latency_spread | " << fixed << setprecision(1) << (max_lat - min_lat) << " ms | 最快/最慢相机首帧延迟差 |\n";
    g_session_log << "| peak_queue_pressure | " << peak_q << " | 队列峰值 (cam " << peak_q_cam << ": " << (peak_q_cam >= 0 ? cam_ctxs[peak_q_cam]->id : "?") << ") |\n";
    g_session_log << "| max_jitter | " << fixed << setprecision(1) << max_jitter << " us | 时间戳抖动峰值 (cam " << max_jitter_cam << ": " << (max_jitter_cam >= 0 ? cam_ctxs[max_jitter_cam]->id : "?") << ") |\n";
    g_session_log << "| max_ram2disk | " << fixed << setprecision(3) << max_ram2disk << " s | DISK写入最长 (cam " << max_ram2disk_cam << ": " << (max_ram2disk_cam >= 0 ? cam_ctxs[max_ram2disk_cam]->id : "?") << ") |\n";
    g_session_log << "| max_end_to_end | " << fixed << setprecision(3) << max_total << " s | 端到端最长 (cam " << max_total_cam << ": " << (max_total_cam >= 0 ? cam_ctxs[max_total_cam]->id : "?") << ") |\n";
    g_session_log << "| system_healthy | **" << (healthy ? "PASS" : "FAIL") << "** | sync≤2 && fps≥95% && drop≤1% |\n";
    int ef = g_exc_fatal.load(), ee = g_exc_error.load(), ew = g_exc_warn.load(), ei = g_exc_info.load();
    auto b = [](int v) { return v > 0 ? "**" + to_string(v) + "**" : to_string(v); };
    g_session_log << "| exceptions_fatal | " << b(ef) << " | FATAL 异常计数 |\n";
    g_session_log << "| exceptions_error | " << b(ee) << " | ERROR 异常计数 |\n";
    g_session_log << "| exceptions_warn  | " << b(ew) << " | WARN 异常计数 |\n";
    g_session_log << "| exceptions_info  | " << b(ei) << " | INFO 异常计数 |\n";
    g_session_log << defaultfloat << flush;
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Sync Network Node) ===" << endl;
    
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cerr << "[System ERROR] WSAStartup failed." << endl;
        return 1;
    }
#endif

    H5::Exception::dontPrint();
    Cfg cfg("cfg/capture.yaml");
    auto& cap = cfg["capture"];
    Pylon::PylonInitialize();

    g_participant_roots = cap["participant_root"].as<vector<string>>();
    g_sentry_root = g_participant_roots[0];
    try { g_hdf5_chunk_capacity = cap["hdf5_chunk_frame_capacity"].as<int>(); } catch (...) {}

    bool is_master_pc = cap["is_master"].as<bool>();
    string master_ip  = cap["master_ip"].as<string>();
    string slave_ip   = cap["slave_ip"].as<string>();
    int net_port      = cap["port"].as<int>();

    vector<string> camera_ids = cap["cam_indices"].as<vector<string>>();
    bool use_hw_trigger = cap["hardware_trigger"].as<bool>();
    
    // [新增] 读取对齐相关控制开关
    // 请确保您的 yaml/json 配置文件中含有 enable_offset 和 enable_intersection 这两项布尔值
    bool enable_offset = true; 
    bool enable_intersection = true;
    bool enable_net_sync = true; // [新增]
    try {
        enable_offset = cap["enable_offset"].as<bool>();
        enable_intersection = cap["enable_intersection"].as<bool>();
        enable_net_sync = cap["enable_net_sync"].as<bool>(); // [新增]
    } catch (...) {
        cout << "[WARN] Sync configs missing, defaulting to TRUE" << endl;
    }

    double target_fps = cap["fps"].as<double>();
    double gain = cap["gain"].as<double>();
    double gamma = cap["gamma"].as<double>();
    double exp_time = cap["exposure_time"].as<double>();
    g_win_w = cap["window_width"].as<int>();
    g_win_h = cap["window_height"].as<int>();

    double record_time = cap["record_time"].as<double>();
    int cam_w = cap["cam_width"].as<int>();
    int cam_h = cap["cam_height"].as<int>();

    int core_frames = static_cast<int>(std::ceil(target_fps * record_time));
    double margin_frames_ratio = cap["margin_frames_ratio"].as<double>();
    int margin_frames = static_cast<int>(std::ceil(core_frames * margin_frames_ratio));
    int total_record_frames = core_frames + 2 * margin_frames;

    cout << "\n--- Network Sync Configuration ---" << endl;
    cout << "Role             : " << (is_master_pc ? "MASTER (Sender)" : "SLAVE (Receiver)") << endl;
    cout << "Master IP        : " << master_ip << endl;
    cout << "Slave IP         : " << slave_ip << endl;
    cout << "Port             : " << net_port << endl;
    cout << "HW Trigger       : " << (use_hw_trigger ? "ON" : "OFF") << endl;
    cout << "SW Offset Init   : " << (enable_offset ? "ON" : "OFF") << endl;
    cout << "Intersection Crop: " << (enable_intersection ? "ON" : "OFF") << endl;
    cout << "Net Sync         : " << (enable_net_sync ? "ON" : "OFF (Local Mode)") << endl;
    cout << "----------------------------------\n" << endl;

    g_use_hw_trigger = use_hw_trigger;

    // [新增] Master 预先建立 UDP Socket (常驻内存)
    if (is_master_pc && enable_net_sync) {
        master_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        slave_udp_addr.sin_family = AF_INET;
        slave_udp_addr.sin_port = htons(net_port);
        inet_pton(AF_INET, slave_ip.c_str(), &slave_udp_addr.sin_addr);
    }

    // 故障通信 socket (port + 200)
    if (enable_net_sync) {
        g_fault_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        sockaddr_in fb{}; fb.sin_family = AF_INET; fb.sin_port = htons(net_port + 200);
        inet_pton(AF_INET, (is_master_pc ? master_ip : slave_ip).c_str(), &fb.sin_addr);
        ::bind(g_fault_sock, (sockaddr*)&fb, sizeof(fb));
        g_peer_fault_addr.sin_family = AF_INET; g_peer_fault_addr.sin_port = htons(net_port + 200);
        inet_pton(AF_INET, (is_master_pc ? slave_ip : master_ip).c_str(), &g_peer_fault_addr.sin_addr);
    }

    // --- 启动网络监听器 (仅Slave) ---
    thread listener_thread;
    if (enable_net_sync && !is_master_pc) { // [修改] 加入 enable_net_sync 判断
        listener_thread = thread(udpListenerWorker, slave_ip, net_port);
    }

    for (int i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(i, camera_ids[i], g_participant_roots[i]);
        cam_ctxs.push_back(ctx);
    }

    cout << "[System] Pre-allocating contiguous RAM blocks for " << total_record_frames << " frames..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->total_record_frames = total_record_frames;
        ctx->ram_buffer.resize(total_record_frames);
        ctx->meta_buffer.resize(total_record_frames);
        for (int k = 0; k < total_record_frames; ++k) {
            ctx->ram_buffer[k] = cv::Mat::zeros(cam_h, cam_w, CV_8UC1);
        }
    }
    cout << "[System] OS Memory array reserved. Pipeline ready.\n" << endl;

    CameraContext::master_set.store(false);
    CameraContext::master_first_id.store(-1);
    
    for (auto& ctx : cam_ctxs) {
        ctx->running = true;
        ctx->dump_ready = false;
        ctx->offset_initialized = false;
        ctx->copy_thread = thread(copyWorker, ctx);
        // [修改] 传递 enable_offset 参数
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain, gamma, exp_time, use_hw_trigger, enable_offset);
    }

    cv::namedWindow("Multi-Cam Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Cam Preview", g_win_w, g_win_h);
    updateLayout();
    cv::setMouseCallback("Multi-Cam Preview", onMouse);

    // --- Session log ---
    {
        auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
        char tb[64]; strftime(tb, sizeof(tb), "%Y%m%d_%H%M%S", localtime(&t));
        error_code ec;
        fs::create_directories("log/capture", ec);
        if (ec) {
            logException("ERROR", "init", "Cannot create log/capture/: " + ec.message());
        } else {
            g_session_log_path = string("log/capture/session_") + tb + ".md";
            g_session_log.open(g_session_log_path, ios::out | ios::app);
        }
        if (g_session_log.is_open())
            g_session_log << "# Session: " << tb << "\n\n"
                          << "- **Cameras**: " << camera_ids.size() << "\n"
                          << "- **Trigger**: " << (use_hw_trigger ? "HW" : "SW") << "\n"
                          << "- **Target FPS**: " << target_fps << "\n"
                          << "- **Storage**: HDF5\n"
                          << "\n---\n" << flush;
        cout << "[Log] Session log: " << g_session_log_path << endl;
    }

    // HDF5: init sentry + open all camera chunks
    initSentry(g_sentry_root);
    cout << "[HDF5] Sentry: chunk=" << g_chunk_idx << " offset=" << g_frame_offset << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->hdf5_dir = g_participant_roots[ctx->index] + "/" + ctx->id;
        fs::create_directories(ctx->hdf5_dir);
        openOrCreateChunk(ctx, cam_h, cam_w, g_hdf5_chunk_capacity);
    }
    cout << "[HDF5] All camera chunks opened." << endl;

    if (is_master_pc) cout << "Press 'r' to START REC across ALL nodes, 'space' to photo, 'q' to quit.\n";
    else cout << "Waiting for Master trigger... Press 'space' to photo, 'q' to quit.\n";

    bool is_recording = false;
    atomic<bool> is_dumping{false}; 
    string current_record_timestr;
    std::chrono::steady_clock::time_point record_start_time;

    double target_ui_fps = cap["ui_fps"].as<double>(); 
    auto ui_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / target_ui_fps));
    auto last_ui_time = std::chrono::steady_clock::now() - ui_interval; 

    g_ready_time = std::chrono::steady_clock::now();

    while (global_running) {
        // E1: check if no camera has started streaming within 30s
        static bool startup_checked = false;
        if (!startup_checked && chrono::duration<double>(chrono::steady_clock::now() - g_ready_time).count() > 30.0) {
            startup_checked = true;
            bool any_streaming = false;
            for (auto& ctx : cam_ctxs) if (ctx->has_streamed.load()) { any_streaming = true; break; }
            if (!any_streaming) logException("ERROR", "startup", "No camera streaming after 30s");
        }
        auto current_time = std::chrono::steady_clock::now();
        bool need_ui_update = (current_time - last_ui_time) >= ui_interval;

        // ===== 0. 非阻塞故障/SHUTDOWN 消息轮询 =====
        if (enable_net_sync && g_fault_sock != INVALID_SOCKET) {
            fd_set readfds; FD_ZERO(&readfds); FD_SET(g_fault_sock, &readfds);
            timeval tv = {0, 0};
            if (select(0, &readfds, NULL, NULL, &tv) > 0) {
                char poll_buf[64];
                sockaddr_in sender; socklen_t slen = sizeof(sender);
                int nb = recvfrom(g_fault_sock, poll_buf, sizeof(poll_buf) - 1, 0, (sockaddr*)&sender, &slen);
                if (nb > 0) {
                    poll_buf[nb] = '\0'; string pm(poll_buf);
                    if (pm.rfind("FAULT:", 0) == 0 && !g_fault_active.load()) {
                        if (pm.length() <= 7) continue;
                        char hf = pm[6]; int fi = stoi(pm.substr(7));
                        if (fi < 0 || fi >= (int)cam_ctxs.size()) {
                            logException("WARN", "main", "FAULT cam idx out of bounds: " + to_string(fi));
                            continue;
                        }
                        cout << "[Fault] Received from " << (hf == 'M' ? "MASTER" : "SLAVE")
                             << ": cam " << fi << endl;
                        g_fault_time = std::chrono::steady_clock::now();
                        g_fault_active.store(true); g_faulty_cam.store(fi); g_fault_on_master.store(hf == 'M');
                        for (auto& ctx : cam_ctxs) {
                            ctx->running = false;
                            ctx->copy_cv.notify_all();
                        }
                        for (auto& ctx : cam_ctxs) {
                            if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
                            if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
                        }
                        is_recording = false;
                        cout << "[Fault] All cameras stopped. Press ESC to exit." << endl;
                    }
                    else if (pm == "SHUTDOWN") {
                        cout << "[System] Received SHUTDOWN from peer. Exiting." << endl;
                        global_running = false;
                    }
                }
            }
        }

        // ===== 1. 相机健康检查 =====
        if (!g_fault_active.load() && !is_dumping.load()) {
            auto now = std::chrono::steady_clock::now();
            for (size_t i = 0; i < cam_ctxs.size(); ++i) {
                if (!cam_ctxs[i]->has_streamed.load()) continue;
                if (std::chrono::duration<double>(now - cam_ctxs[i]->last_frame_time.load()).count() > 1.0) {
                    cerr << "\n[FAULT] Camera " << cam_ctxs[i]->id
                         << " (index " << i << ") stalled!" << endl;
                    g_fault_time = std::chrono::steady_clock::now();
                    g_fault_active.store(true); g_faulty_cam.store((int)i);
                    g_fault_on_master.store(is_master_pc);
                    if (enable_net_sync && g_fault_sock != INVALID_SOCKET) {
                        string fm = "FAULT:" + string(is_master_pc ? "M" : "S") + to_string(i);
                        sendto(g_fault_sock, fm.c_str(), (int)fm.length(), 0,
                               (sockaddr*)&g_peer_fault_addr, sizeof(g_peer_fault_addr));
                    }
                    // Close all cameras
                    for (auto& ctx : cam_ctxs) {
                        ctx->running = false;
                        ctx->copy_cv.notify_all();
                    }
                    for (auto& ctx : cam_ctxs) {
                        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
                        if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
                    }
                    is_recording = false;
                    cout << "[Fault] All cameras stopped. Press ESC to exit both hosts." << endl;
                    break;
                }
            }
        }

        // ===== 2. 解耦 UI 渲染模块 =====
        if (need_ui_update && !is_dumping) {
            if (g_fault_active.load()) {
                showFaultOverlay(g_faulty_cam.load(), g_use_hw_trigger);
            } else {
                cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
                int sel = g_enlarged_cam.load();
                renderThumbnailGrid(canvas, sel, is_recording, record_start_time, total_record_frames);
                renderEnlargedView(canvas, sel, is_recording, record_start_time, total_record_frames);
                cv::line(canvas, cv::Point(g_left_w, 0), cv::Point(g_left_w, g_win_h),
                         cv::Scalar(60, 60, 60), 2);

                // Watermark hints in enlarged area (bottom-left of right panel)
                int hx = g_right_x + 10, hy = g_win_h - 45;
                string hints;
                if (is_recording) hints = "[REC] Recording in progress...";
                else if (is_dumping) hints = "[DUMP] Writing to disk...";
                else if (enable_net_sync && !is_master_pc) hints = "[r][space][q] disabled (Slave)  |  Waiting for Master...";
                else hints = "[r] record  [space] photo  [ESC/q] quit";
                cv::putText(canvas, hints, cv::Point(hx, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
                hy += 18;
                string status = enable_net_sync
                    ? (is_master_pc ? "Role: MASTER  |  Net Sync: ON" : "Role: SLAVE  |  Net Sync: ON")
                    : "Role: STANDALONE  |  Net Sync: OFF";
                cv::putText(canvas, status, cv::Point(hx, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);

                // Crosshair at center of enlarged area
                int cx = g_right_x + g_right_w / 2;
                int cy = g_win_h / 2;
                int cl = 20;
                cv::line(canvas, cv::Point(cx - cl, cy), cv::Point(cx + cl, cy), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
                cv::line(canvas, cv::Point(cx, cy - cl), cv::Point(cx, cy + cl), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

                cv::imshow("Multi-Cam Preview", canvas);
            }
            last_ui_time = current_time;
        }

        // ===== 2. 落盘等待逻辑 =====
        if (is_recording && !is_dumping) {
            bool all_done = true;
            for (auto& ctx : cam_ctxs) if (!ctx->dump_ready.load()) { all_done = false; break; }

            if (all_done) {
                is_recording = false;
                is_dumping = true;
                g_recording_number++;

                auto dump_start_time = std::chrono::steady_clock::now();
                atomic<int> finished_cams{0};
                vector<thread> dump_threads;

                for (auto& ctx : cam_ctxs) dump_threads.emplace_back(dumpToHdf5Worker, ctx, core_frames, margin_frames, std::ref(finished_cams));

                cout << "[DEBUG-HDF5] Dump wait loop START, waiting for " << cam_ctxs.size() << " cameras" << endl;
                while (finished_cams < cam_ctxs.size()) {
                    if (finished_cams.load() % 2 == 0)
                        cout << "[DEBUG-HDF5] Dump progress: " << finished_cams.load() << "/" << cam_ctxs.size() << endl;
                    cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                    cv::putText(loading, "DUMPING RAM TO HDF5... (" + to_string(finished_cams.load()) + "/" + to_string(cam_ctxs.size()) + ")", cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                    cv::imshow("Multi-Cam Preview", loading);
                    cv::waitKey(50);
                }
                cout << "[DEBUG-HDF5] All dumps done, joining threads..." << endl;
                for (auto& t : dump_threads) if (t.joinable()) t.join();
                cout << "[DEBUG-HDF5] All threads joined" << endl;

                double dump_duration = chrono::duration<double>(chrono::steady_clock::now() - dump_start_time).count();

                cout << "[DEBUG-HDF5] Updating sentry, frame_offset " << g_frame_offset << " -> " << (g_frame_offset + core_frames) << endl;
                // Update HDF5 sentry
                g_frame_offset += core_frames;
                if (g_frame_offset >= g_hdf5_chunk_capacity) {
                    for (auto& ctx : cam_ctxs) closeChunk(ctx);
                    g_chunk_idx++;
                    g_frame_offset -= g_hdf5_chunk_capacity;
                    for (auto& ctx : cam_ctxs) openOrCreateChunk(ctx, cam_h, cam_w, g_hdf5_chunk_capacity);
                }
                updateSentry(g_sentry_root);

                double jpg_duration = 0.0;
                writeReport(current_record_timestr, g_recording_number, target_fps, total_record_frames,
                            dump_duration, jpg_duration, false, use_hw_trigger);

                cout << "[Recording #" << g_recording_number << "] Done in " << fixed << setprecision(1)
                     << dump_duration << "s (HDF5, appended to session log)" << endl;

                is_dumping = false;
                while (cv::waitKey(1) >= 0);
                last_ui_time = chrono::steady_clock::now();
            }
        }

        // ===== 3. 事件轮询 (1000Hz 极速响应) =====
        char key = (char)cv::waitKey(1);
        bool trigger_start = false;

        if (key == 'q' || key == 27) {
            if (enable_net_sync) {
                if (is_master_pc) {
                    if (g_fault_sock != INVALID_SOCKET) {
                        string sm = "SHUTDOWN";
                        sendto(g_fault_sock, sm.c_str(), (int)sm.length(), 0,
                               (sockaddr*)&g_peer_fault_addr, sizeof(g_peer_fault_addr));
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                    global_running = false;
                }
                // Slave: ignore ESC/q when net sync on (except during fault)
                if (!is_master_pc && g_fault_active.load()) global_running = false;
            } else {
                global_running = false;
            }
        } else if (g_fault_active.load()) {
            // Block all keys except ESC during fault
        } else if (key == 'r' && !is_recording && !is_dumping) {
            auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            char buf[64]; strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
            shared_record_timestr = string(buf);

            if (enable_net_sync) {
                if (is_master_pc) {
                    cout << "\n[Master UI] 'r' pressed. Broadcast START to Slave..." << endl;
                    fastUdpSend("CMD_START:" + shared_record_timestr); // 极速发送
                    instantTrigger(); // Master 本地瞬间开火
                    trigger_start = true;
                } else {
                    cout << "\n[Warning] Net Sync is ON. Please press 'r' on the Master PC." << endl;
                }
            } else {
                cout << "\n[Local UI] 'r' pressed. Local capture starting..." << endl;
                instantTrigger();
                trigger_start = true;
            }
        } else if (key == ' ') {
            if (enable_net_sync && !is_master_pc) {
                // Slave: ignore SPACE when net sync is on
            } else if (!is_recording && !is_dumping) {
                int counter = 0;
                if (!cam_ctxs.empty()) counter = getNextCalibCounter(cam_ctxs[0]->save_base_dir);
                std::stringstream ss; ss << std::setw(2) << std::setfill('0') << counter;
                string calib_str = ss.str();
                cout << "\n[Photo] Capturing calibration set " << calib_str << endl;

                for (auto& ctx : cam_ctxs) {
                    cv::Mat snapshot;
                    { lock_guard<mutex> lock(ctx->frame_mtx); snapshot = ctx->latest_frame.clone(); }
                    if (!snapshot.empty()) {
                        cv::Mat out_snapshot;
                        if (ctx->is_mono) {
                            out_snapshot = snapshot.clone(); 
                        } else {
                            cv::cvtColor(snapshot, out_snapshot, cv::COLOR_BayerRG2RGB);
                        }
                        string fn = ctx->save_base_dir + "/calib_cam_" + ctx->id + "_" + calib_str + ".jpg";
                        cv::imwrite(fn, out_snapshot);
                        cout << "  -> Saved " << fn << endl;
                    }
                }
            }
        }

        // ===== 4. 目录创建与 IO 准备 (完全移出关键路径) =====
        // 这里同时处理 Master(靠 trigger_start) 和 Slave(靠 net_cmd_record) 的 IO 请求
        if ((trigger_start || net_cmd_record.exchange(false)) && !is_recording && !is_dumping) {
            
            // 同步主线程状态与网络线程状态
            current_record_timestr = shared_record_timestr;
            record_start_time = global_record_start_time;

            cout << "[Info] RAM CAPTURE STARTED. PREPARING I/O ASYNC..." << endl;

            // HDF5: no temp_raw directories needed — data goes directly to HDF5

            // HDF5: check chunk space before recording
            if (g_frame_offset + core_frames > g_hdf5_chunk_capacity) {
                for (auto& ctx : cam_ctxs) closeChunk(ctx);
                g_chunk_idx++;
                g_frame_offset = 0;
                for (auto& ctx : cam_ctxs) openOrCreateChunk(ctx, cam_h, cam_w, g_hdf5_chunk_capacity);
                updateSentry(g_sentry_root);
                cout << "[HDF5] New chunk: " << g_chunk_idx << endl;
            }

            is_recording = true; // 更新主线程 UI 状态
            cout << "[Info] I/O PREPARED FOR: " << current_record_timestr << endl;
        }
    }

    cout << "[System] Shutting down threads..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->copy_cv.notify_all();
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
    }
    
    if (listener_thread.joinable()) {
        listener_thread.join();
    }

    if (g_session_log.is_open()) g_session_log.close();
    for (auto& ctx : cam_ctxs) closeChunk(ctx);
    cv::destroyAllWindows();
    Pylon::PylonTerminate();
    if (master_udp_sock != INVALID_SOCKET) closesocket(master_udp_sock);
    if (g_fault_sock != INVALID_SOCKET) closesocket(g_fault_sock);

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}