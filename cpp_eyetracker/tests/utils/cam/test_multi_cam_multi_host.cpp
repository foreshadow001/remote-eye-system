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
#include <pylon/PylonIncludes.h>

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

// ================== UI 布局全局变量 ==================
atomic<int> g_enlarged_cam{-1};
int g_win_w = 1224, g_win_h = 1024;
int g_left_w = 0, g_right_x = 0, g_right_w = 0;
int g_thumb_w = 0, g_thumb_h = 0;

// ================== [新增] 核心：极速零延迟触发函数 ==================
void instantTrigger() {
    global_record_start_time = std::chrono::steady_clock::now();
    for (auto& ctx : cam_ctxs) {
        {
            // 瞬间清空残余队列
            lock_guard<mutex> lock(ctx->copy_mtx);
            while (!ctx->copy_queue.empty()) ctx->copy_queue.pop();
        }
        ctx->recorded_frames.store(0, std::memory_order_relaxed);
        ctx->dump_ready.store(false, std::memory_order_relaxed);
        // 核心：直接开启内存拷贝，彻底绕过 UI 线程！
        ctx->recording.store(true, std::memory_order_release); 
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
                instantTrigger();
                shared_record_timestr = cmd.substr(10);
                net_cmd_record = true;
            }
            else if (cmd.rfind("FAULT:", 0) == 0 && !g_fault_active.load()) {
                char hf = cmd[6]; int fi = stoi(cmd.substr(7));
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

                int next_seq = seq + 1;
                ctx->recorded_frames.store(next_seq, std::memory_order_relaxed);

                if (next_seq == ctx->total_record_frames) {
                    ctx->recording = false;
                    ctx->dump_ready = true;
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
    if (!ctx->cam.open(mode)) { ctx->status = CamStatus::ERROR_; return; }

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
                ctx->copy_queue.push({ptr, meta});
                ctx->copy_cv.notify_one();
            } else {
                if (ctx->copy_queue.size() < 2) {
                    ctx->copy_queue.push({ptr, meta});
                    ctx->copy_cv.notify_one();
                }
            }
        }
    });

    if (!ctx->cam.start()) { ctx->status = CamStatus::ERROR_; return; }
    ctx->status_msg = use_hw_trigger ? "HW WAITING" : "STREAMING";

    while (ctx->running) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ctx->cam.close();
}

void dumpToDiskWorker(shared_ptr<CameraContext> ctx, int write_delay_ms, atomic<int>& finished_cams) {
    ctx->log_stream.open(ctx->log_file_path);
    int frames_to_dump = ctx->recorded_frames.load();
    
    for (int i = 0; i < frames_to_dump; ++i) {
        cv::Mat& img = ctx->ram_buffer[i];
        FrameMeta& meta = ctx->meta_buffer[i];
        
        // [修改] 使用 meta.blockID 作为 raw 的文件名
        string filename = ctx->temp_dir + "/" + to_string(meta.blockID) + ".raw";
        
        std::ofstream out_raw(filename, std::ios::binary);
        if (out_raw) out_raw.write(reinterpret_cast<const char*>(img.data), img.total() * img.elemSize());

        if (ctx->log_stream.is_open()) {
            ctx->log_stream << fs::path(filename).filename().string() << "," 
                            << meta.blockID << "," << meta.timestamp << ","
                            << meta.blockID << "," << img.cols << "," << img.rows << "\n";
        }
        if (write_delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(write_delay_ms));
    }
    if (ctx->log_stream.is_open()) ctx->log_stream.close();
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

void convertRawToJpgWorker(string temp_raw_dir, string out_jpg_dir, vector<LogEntry> valid_entries, atomic<int>& global_processed, bool is_mono) {
    if (valid_entries.empty()) return;
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
            cv::imwrite(out_jpg_dir + "/" + jpg_filename, final_img);
        }
        global_processed++;
    }
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

    Cfg cfg;
    Pylon::PylonInitialize();
    
    // --- 网络参数注入 ---
    bool is_master_pc = cfg["test_multi_cam"]["is_master"].as<bool>();
    string master_ip  = cfg["test_multi_cam"]["master_ip"].as<string>();
    string slave_ip   = cfg["test_multi_cam"]["slave_ip"].as<string>();
    int net_port      = cfg["test_multi_cam"]["port"].as<int>();
    
    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();
    bool use_hw_trigger = cfg["test_multi_cam"]["hardware_trigger"].as<bool>();
    
    // [新增] 读取对齐相关控制开关
    // 请确保您的 yaml/json 配置文件中含有 enable_offset 和 enable_intersection 这两项布尔值
    bool enable_offset = true; 
    bool enable_intersection = true;
    bool enable_net_sync = true; // [新增]
    try {
        enable_offset = cfg["test_multi_cam"]["enable_offset"].as<bool>();
        enable_intersection = cfg["test_multi_cam"]["enable_intersection"].as<bool>();
        enable_net_sync = cfg["test_multi_cam"]["enable_net_sync"].as<bool>(); // [新增]
    } catch (...) {
        cout << "[WARN] Sync configs missing, defaulting to TRUE" << endl;
    }

    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time = cfg["test_multi_cam"]["exposure_time"].as<double>();
    vector<string> save_dirs = cfg["test_multi_cam"]["save_dir"].as<vector<string>>();
    
    g_win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    g_win_h = cfg["test_multi_cam"]["window_height"].as<int>();
    bool write_jpg = cfg["test_multi_cam"]["write_jpg"].as<bool>(); 
    
    double record_time = cfg["test_multi_cam"]["record_time"].as<double>(); 
    int write_delay_ms = cfg["test_multi_cam"]["write_delay_ms"].as<int>();
    int cam_w = cfg["test_multi_cam"]["cam_width"].as<int>();
    int cam_h = cfg["test_multi_cam"]["cam_height"].as<int>();

    int core_frames = static_cast<int>(std::ceil(target_fps * record_time));
    double margin_frames_ratio = cfg["test_multi_cam"]["margin_frames_ratio"].as<double>();
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

    if (save_dirs.size() != camera_ids.size()) { cerr << "Mismatch config." << endl; return -1; }
    for (const auto& dir : save_dirs) std::filesystem::create_directories(dir);

    for (int i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(i, camera_ids[i], save_dirs[i]);
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

    if (is_master_pc) cout << "Press 'r' to START REC across ALL nodes, 'space' to photo, 'q' to quit.\n";
    else cout << "Waiting for Master trigger... Press 'space' to photo, 'q' to quit.\n";

    bool is_recording = false;
    atomic<bool> is_dumping{false}; 
    string current_record_timestr;
    std::chrono::steady_clock::time_point record_start_time;

    double target_ui_fps = cfg["test_multi_cam"]["ui_fps"].as<double>(); 
    auto ui_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / target_ui_fps));
    auto last_ui_time = std::chrono::steady_clock::now() - ui_interval; 

    g_ready_time = std::chrono::steady_clock::now();

    while (global_running) {
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
                        char hf = pm[6]; int fi = stoi(pm.substr(7));
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
                
                cout << "\n[Info] Auto-Stop Reached. Processing Disk Dump in background..." << endl;
                
                auto dump_start_time = std::chrono::steady_clock::now();
                atomic<int> finished_cams{0};
                vector<thread> dump_threads;

                for (auto& ctx : cam_ctxs) dump_threads.emplace_back(dumpToDiskWorker, ctx, write_delay_ms, std::ref(finished_cams));

                while (finished_cams < cam_ctxs.size()) {
                    cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                    cv::putText(loading, "DUMPING RAM TO DISK... (" + to_string(finished_cams.load()) + "/" + to_string(cam_ctxs.size()) + ")", cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                    cv::imshow("Multi-Cam Preview", loading);
                    cv::waitKey(50); 
                }
                for (auto& t : dump_threads) if (t.joinable()) t.join();

                double dump_duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - dump_start_time).count();
                cout << "\n=================================================" << endl;
                cout << "[Performance] Disk Dump Finished! Time: " << fixed << setprecision(3) << dump_duration << "s" << endl;
                cout << "=================================================" << endl;

                cout << "\n[Info] Calculating frame drop and actual FPS statistics..." << endl;
                vector<vector<LogEntry>> all_logs;
                vector<vector<LogEntry>> core_sliced_logs; 
                
                // 【需替换的部分：统计与报告打印】
                for (int i = 0; i < cam_ctxs.size(); ++i) {
                    auto logs = parseLogFile(cam_ctxs[i]->log_file_path);
                    all_logs.push_back(logs);
                    
                    int total_saved = logs.size();
                    int dropped_frames = 0;
                    double actual_fps = 0.0;
                    double duration_s = 0.0;
                    int64_t start_block_id = logs.empty() ? -1 : logs.front().blockID; // 获取起始 blockID
                    
                    if (total_saved > 1) {
                        for (size_t k = 1; k < logs.size(); ++k) {
                            int64_t diff = logs[k].blockID - logs[k-1].blockID;
                            if (diff > 1) dropped_frames += (diff - 1);
                        }
                        duration_s = static_cast<double>(logs.back().timestamp - logs.front().timestamp) / 10000000.0; 
                        if (duration_s > 0) actual_fps = (total_saved - 1) / duration_s;
                    }
                    
                    char report_buf[256];
                    if (dropped_frames > 0) {
                        snprintf(report_buf, sizeof(report_buf), "[Warning] Cam %s | StartID: %6lld | Saved: %4d | Drop: %d | Actual FPS: %.4f", 
                                 cam_ctxs[i]->id.c_str(), start_block_id, total_saved, dropped_frames, actual_fps);
                    } else {
                        snprintf(report_buf, sizeof(report_buf), "[OK]      Cam %s | StartID: %6lld | Saved: %4d | Drop: 0 | Actual FPS: %.4f", 
                                 cam_ctxs[i]->id.c_str(), start_block_id, total_saved, actual_fps);
                    }
                    cout << report_buf << endl;
                }

                if (write_jpg) {
                    cout << "\n[Info] Generating Sync JPGs for Core Frames..." << endl;
                    
                    vector<vector<LogEntry>> final_entries_lists(cam_ctxs.size()); 
                    int total_frames_all_cams = 0;

                    if (use_hw_trigger || enable_intersection) {
                        int64_t global_start_idx = 0;
                        int64_t global_end_idx = 9999999999LL; 

                        // 1. 在完整的 raw 数据集 (all_logs) 上寻找【本机】交集边界
                        for (const auto& logs : all_logs) {
                            if (logs.empty()) continue;
                            if (logs.front().blockID > global_start_idx) global_start_idx = logs.front().blockID;
                            if (logs.back().blockID < global_end_idx) global_end_idx = logs.back().blockID;
                        }

                        // ============ [新增] 跨主机 BlockID 交换 ============
                        if (enable_net_sync) {
                            cout << "[Alignment] Local Range: [" << global_start_idx << ", " << global_end_idx << "]. Syncing with " << (is_master_pc ? "Slave" : "Master") << "..." << endl;
                            syncGlobalBlockIDTCP(is_master_pc, master_ip, net_port, global_start_idx, global_end_idx);
                            cout << "[Alignment] Final Global Intersection Range: [" << global_start_idx << ", " << global_end_idx << "]" << endl;
                        } else {
                            cout << "[Alignment] Net Sync OFF. Using Local Intersection Range: [" << global_start_idx << ", " << global_end_idx << "]" << endl;
                        }
                        // ====================================================
                        
                        // 2. 先取出交集，再从中截取 core_frames
                        for (int i = 0; i < cam_ctxs.size(); ++i) {
                            vector<LogEntry> intersected;
                            for (const auto& entry : all_logs[i]) {
                                if (entry.blockID >= global_start_idx && entry.blockID <= global_end_idx) {
                                    intersected.push_back(entry); 
                                }
                            } 

                            // 3. 从交集中取最中间的 core_frames
                            if (intersected.size() >= core_frames) {
                                int start_offset = (intersected.size() - core_frames) / 2;
                                final_entries_lists[i] = vector<LogEntry>(intersected.begin() + start_offset, intersected.begin() + start_offset + core_frames);
                            } else {
                                cout << "[Warning] Cam " << cam_ctxs[i]->index << " intersected frames (" << intersected.size() << ") < core_frames (" << core_frames << ")." << endl;
                                final_entries_lists[i] = intersected; // 数量不够则全量保留
                            }
                            total_frames_all_cams += final_entries_lists[i].size(); 
                        }
                    } else {
                        // 如果不开启对齐，直接在每台相机自己的 raw 数组中切取中间的 core_frames
                        for (int i = 0; i < cam_ctxs.size(); ++i) {
                            if (all_logs[i].size() >= core_frames) {
                                int start_offset = (all_logs[i].size() - core_frames) / 2;
                                final_entries_lists[i] = vector<LogEntry>(all_logs[i].begin() + start_offset, all_logs[i].begin() + start_offset + core_frames);
                            } else {
                                final_entries_lists[i] = all_logs[i];
                            }
                            total_frames_all_cams += final_entries_lists[i].size();
                        }
                    }

                    if (total_frames_all_cams > 0) {
                        std::atomic<int> global_processed{0};
                        
                        auto draw_progress_ui = [&]() {
                            cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                            int processed = global_processed.load();
                            cv::putText(loading, "Saving JPGs... " + to_string(processed) + "/" + to_string(total_frames_all_cams), cv::Point(50, 180), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                            float ratio = total_frames_all_cams > 0 ? (std::min)(1.0f, (float)processed / total_frames_all_cams) : 0.0f;
                            cv::rectangle(loading, cv::Point(50, 220), cv::Point(550, 240), cv::Scalar(255, 255, 255), 1);
                            if (ratio > 0) cv::rectangle(loading, cv::Point(50, 220), cv::Point(50 + (int)(500 * ratio), 240), cv::Scalar(0, 255, 0), -1);
                            cv::imshow("Multi-Cam Preview", loading);
                            cv::waitKey(1);
                        };
                        
                        vector<thread> compile_threads;
                        for (int i = 0; i < cam_ctxs.size(); ++i) {
                            string out_jpg_dir = cam_ctxs[i]->save_base_dir + "/record_" + current_record_timestr + "/" + cam_ctxs[i]->id;
                            compile_threads.emplace_back(
                                convertRawToJpgWorker, 
                                cam_ctxs[i]->temp_dir, 
                                out_jpg_dir, 
                                final_entries_lists[i], 
                                std::ref(global_processed), 
                                cam_ctxs[i]->is_mono
                            );
                        }
                        
                        int last_processed = 0, same_count = 0;  
                        while (true) {
                            draw_progress_ui();
                            int current_processed = global_processed.load();
                            if (current_processed >= total_frames_all_cams) break;
                            if (current_processed == last_processed) {
                                if (++same_count > 100) break;
                            } else { same_count = 0; last_processed = current_processed; }
                            this_thread::sleep_for(chrono::milliseconds(50));
                        }
                        for (auto& t : compile_threads) if (t.joinable()) t.join();
                        global_processed = total_frames_all_cams;  
                        draw_progress_ui();
                        cv::waitKey(500);
                    }
                }

                cout << "\n[Info] Ready for next capture." << endl;
                is_dumping = false;
                
                while (cv::waitKey(1) >= 0); 
                last_ui_time = std::chrono::steady_clock::now(); 
            }
        }

        // ===== 3. 事件轮询 (1000Hz 极速响应) =====
        char key = (char)cv::waitKey(1);
        bool trigger_start = false;

        if (key == 'q' || key == 27) {
            if (g_fault_active.load() && enable_net_sync && g_fault_sock != INVALID_SOCKET) {
                string sm = "SHUTDOWN";
                sendto(g_fault_sock, sm.c_str(), (int)sm.length(), 0,
                       (sockaddr*)&g_peer_fault_addr, sizeof(g_peer_fault_addr));
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            global_running = false;
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
            if (!is_recording && !is_dumping) { 
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

            // 慢慢建文件夹，完全不影响底层的图像拷贝
            for (auto& ctx : cam_ctxs) {
                string batch_raw = ctx->save_base_dir + "/temp_raw_" + current_record_timestr + "/" + ctx->id;
                fs::create_directories(batch_raw);
                ctx->temp_dir = batch_raw;
                ctx->log_file_path = ctx->save_base_dir + "/record_" + current_record_timestr + "_" + ctx->id + ".txt";
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

    cv::destroyAllWindows();
    Pylon::PylonTerminate();
    if (master_udp_sock != INVALID_SOCKET) closesocket(master_udp_sock);
    if (g_fault_sock != INVALID_SOCKET) closesocket(g_fault_sock);

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}