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
#include <functional>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== 全局状态 ==================
atomic<bool> global_running{true};

// ================== UDP 全局句柄 ==================
SOCKET g_udp_sock = INVALID_SOCKET;
sockaddr_in g_master_addr{};
sockaddr_in g_slave_addr{};
int g_net_port = 0;
string g_calib_save_dir;
bool g_enable_net_sync = false;
bool g_is_master = false;

// ================== UDP 文件传输协议 ==================
struct FileTransferHeader {
    uint32_t file_index;
    uint32_t total_chunks;
    uint32_t chunk_index;
    uint32_t data_size;
    char sn[32];
};

static const size_t MAX_CHUNK_DATA = 60000;

inline void fastUdpSend(const sockaddr_in& target, const string& msg) {
    if (g_udp_sock != INVALID_SOCKET) {
        sendto(g_udp_sock, msg.c_str(), static_cast<int>(msg.length()), 0,
               (const sockaddr*)&target, sizeof(target));
    }
}

void sendImageViaUdp(const sockaddr_in& target, const vector<uint8_t>& data,
                     uint32_t file_index, const string& sn) {
    uint32_t total = static_cast<uint32_t>((data.size() + MAX_CHUNK_DATA - 1) / MAX_CHUNK_DATA);
    cout << "  [Send] SN=" << sn << " idx=" << file_index
         << ": " << data.size() << " bytes, " << total << " chunks" << endl;

    for (uint32_t ci = 0; ci < total; ++ci) {
        size_t offset = ci * MAX_CHUNK_DATA;
        size_t chunk_sz = min(MAX_CHUNK_DATA, data.size() - offset);

        vector<uint8_t> packet(sizeof(FileTransferHeader) + chunk_sz);
        auto* hdr = reinterpret_cast<FileTransferHeader*>(packet.data());
        hdr->file_index = file_index;
        hdr->total_chunks = total;
        hdr->chunk_index = ci;
        hdr->data_size = static_cast<uint32_t>(chunk_sz);
        memset(hdr->sn, 0, sizeof(hdr->sn));
        strncpy(hdr->sn, sn.c_str(), sizeof(hdr->sn) - 1);
        memcpy(packet.data() + sizeof(FileTransferHeader), data.data() + offset, chunk_sz);

        sendto(g_udp_sock, reinterpret_cast<const char*>(packet.data()),
               static_cast<int>(packet.size()), 0, (const sockaddr*)&target, sizeof(target));
        this_thread::sleep_for(chrono::microseconds(200));
    }
}

vector<uint8_t> recvImageViaUdp(uint32_t expected_index, string* out_sn = nullptr,
                                 uint32_t* out_index = nullptr, int timeout_ms = 10000) {
#ifdef _WIN32
    DWORD timeout = min(timeout_ms, 2000);
    setsockopt(g_udp_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif
    map<uint32_t, vector<uint8_t>> chunk_map;
    uint32_t total_chunks = 0;
    string current_sn;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);

    vector<uint8_t> recv_buf(65536);

    while (chrono::steady_clock::now() < deadline) {
        sockaddr_in sender;
        socklen_t sender_len = sizeof(sender);
        int bytes = recvfrom(g_udp_sock, reinterpret_cast<char*>(recv_buf.data()),
                             static_cast<int>(recv_buf.size()), 0, (sockaddr*)&sender, &sender_len);

        if (bytes < static_cast<int>(sizeof(FileTransferHeader))) continue;

        auto* hdr = reinterpret_cast<FileTransferHeader*>(recv_buf.data());
        if (expected_index != UINT32_MAX && hdr->file_index != expected_index) continue;

        // 通配模式下：按 SN 隔离，防止多相机相同 file_index 的 chunk 互相覆盖
        if (expected_index == UINT32_MAX && out_sn) {
            string chunk_sn(hdr->sn);
            if (current_sn.empty()) {
                current_sn = chunk_sn;
                *out_sn = chunk_sn;
            } else if (chunk_sn != current_sn) {
                continue; // 跳过其他相机的 chunk，留给下次 recvImageViaUdp
            }
        } else if (out_sn && out_sn->empty()) {
            char tmp[32];
            strncpy(tmp, hdr->sn, sizeof(hdr->sn));
            tmp[sizeof(hdr->sn) - 1] = '\0';
            *out_sn = tmp;
        }

        if (out_index && total_chunks == 0) *out_index = hdr->file_index;
        if (total_chunks == 0) total_chunks = hdr->total_chunks;

        chunk_map[hdr->chunk_index] = vector<uint8_t>(
            recv_buf.data() + sizeof(FileTransferHeader),
            recv_buf.data() + sizeof(FileTransferHeader) + hdr->data_size);

        if (total_chunks > 0 && chunk_map.size() == total_chunks) {
            vector<uint8_t> result;
            for (uint32_t i = 0; i < total_chunks; ++i) {
                auto it = chunk_map.find(i);
                if (it == chunk_map.end()) return {};
                result.insert(result.end(), it->second.begin(), it->second.end());
            }
            return result;
        }
    }
    return {};
}

// ================== 相机上下文 ==================
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

// ================== 传输状态机 (非阻塞) ==================
struct TransferState {
    bool active = false;
    int received = 0;
    int expected_total = 0;
    size_t total_bytes = 0;
    int consecutive_empty = 0;
    chrono::steady_clock::time_point start_time;
    chrono::steady_clock::time_point deadline;
};
TransferState xfer;

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
                    string num_part = stem.substr(last_underscore + 1);
                    max_counter = max(max_counter, stoi(num_part));
                }
            } catch (...) {}
        }
    }
    return max_counter + 1;
}

// ================== 后台拷贝线程 ==================
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

// ================== 相机采集线程 (仅软件触发) ==================
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

// ================== 网络监听线程 (Slave - 最高优先级) ==================
void udpListenerWorker(const string& bind_ip, int port) {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#endif

    SOCKET listen_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (listen_sock == INVALID_SOCKET) return;

    int optval = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&optval, sizeof(optval));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, bind_ip.c_str(), &server_addr.sin_addr);

    if (::bind(listen_sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        cerr << "[Slave ERROR] Bind failed on " << bind_ip << ":" << port << endl;
        closesocket(listen_sock);
        return;
    }

#ifdef _WIN32
    DWORD timeout = 100;
    setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif

    char buffer[256];
    cout << "[Net Sync] Slave UDP listener on " << bind_ip << ":" << port << endl;

    while (global_running) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int bytes = recvfrom(listen_sock, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&client_addr, &client_len);

        if (bytes > 0) {
            buffer[bytes] = '\0';
            string cmd(buffer);

            if (cmd.rfind("PHOTO:", 0) == 0) {
                int img_idx = stoi(cmd.substr(6));
                stringstream ss;
                ss << setw(2) << setfill('0') << img_idx;
                cout << "[Slave] PHOTO " << img_idx << endl;

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
                        string fn = g_calib_save_dir + "/calib_cam_" + ctx->sn + "_" + ss.str() + ".jpg";
                        cv::imwrite(fn, out_img);
                        cout << "  -> Saved " << fn << endl;
                    }
                }
            }
            else if (cmd.rfind("XFER:", 0) == 0) {
                size_t comma = cmd.find(',', 5);
                if (comma != string::npos) {
                    int start_idx = stoi(cmd.substr(5, comma - 5));
                    int end_idx = stoi(cmd.substr(comma + 1));
                    cout << "[Slave] Transfer request: " << start_idx << ".." << end_idx << endl;

                    for (auto& ctx : cam_ctxs) {
                        for (int i = start_idx; i <= end_idx; ++i) {
                            stringstream ss;
                            ss << setw(2) << setfill('0') << i;
                            string fn = g_calib_save_dir + "/calib_cam_" + ctx->sn + "_" + ss.str() + ".jpg";
                            if (!fs::exists(fn)) continue;

                            ifstream in(fn, ios::binary | ios::ate);
                            if (!in) continue;
                            size_t fsize = in.tellg();
                            in.seekg(0, ios::beg);
                            vector<uint8_t> jpeg_data(fsize);
                            in.read(reinterpret_cast<char*>(jpeg_data.data()), fsize);
                            in.close();

                            sendImageViaUdp(client_addr, jpeg_data, i, ctx->sn);
                        }
                    }
                    fastUdpSend(client_addr, "XFER_DONE");
                    cout << "[Slave] Transfer done." << endl;
                }
            }
        }
    }
    closesocket(listen_sock);
}

// ================== 主函数 ==================
int main() {
    cout << "=== [TEST] Calibration Image Capture (Sync Dual-Host) ===" << endl;

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cerr << "[System ERROR] WSAStartup failed." << endl;
        return -1;
    }
#endif

    Cfg cfg;
    Pylon::PylonInitialize();

    // --- 配置读取 ---
    g_is_master = cfg["test_multi_cam"]["is_master"].as<bool>();
    string master_ip = cfg["test_multi_cam"]["master_ip"].as<string>();
    string slave_ip  = cfg["test_multi_cam"]["slave_ip"].as<string>();
    g_net_port       = cfg["test_multi_cam"]["port"].as<int>();
    g_enable_net_sync = cfg["test_multi_cam"]["enable_net_sync"].as<bool>();

    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();
    g_calib_save_dir = cfg["test_multi_cam"]["calib_save_dir"].as<string>();

    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain_val   = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma_val  = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time   = cfg["test_multi_cam"]["exposure_time"].as<double>();

    int win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    int win_h = cfg["test_multi_cam"]["window_height"].as<int>();
    double ui_fps = cfg["test_multi_cam"]["ui_fps"].as<double>();

    cout << "\n--- Calibration Capture Configuration ---" << endl;
    cout << "Role      : " << (g_is_master ? "MASTER" : "SLAVE") << endl;
    cout << "Net Sync  : " << (g_enable_net_sync ? "ON" : "OFF") << endl;
    if (g_enable_net_sync) {
        cout << "Master IP : " << master_ip << endl;
        cout << "Slave IP  : " << slave_ip << endl;
        cout << "Port      : " << g_net_port << endl;
    }
    cout << "Trigger   : Software" << endl;
    cout << "Save dir  : " << g_calib_save_dir << endl;
    cout << "Cameras   : " << camera_ids.size() << endl;
    for (size_t i = 0; i < camera_ids.size(); ++i)
        cout << "  " << i << ": SN=" << camera_ids[i] << endl;
    cout << "-----------------------------------------\n" << endl;

    // === 目录创建 ===
    fs::create_directories(g_calib_save_dir);

    // === 网络初始化 (仅在 enable_net_sync 时) ===
    thread listener_thread;
    if (g_enable_net_sync) {
        g_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (g_udp_sock == INVALID_SOCKET) {
            cerr << "Failed to create UDP socket" << endl;
            return -1;
        }

        int optval = 1;
        setsockopt(g_udp_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&optval, sizeof(optval));

        g_master_addr.sin_family = AF_INET;
        g_master_addr.sin_port = htons(g_net_port);
        inet_pton(AF_INET, master_ip.c_str(), &g_master_addr.sin_addr);

        g_slave_addr.sin_family = AF_INET;
        g_slave_addr.sin_port = htons(g_net_port);
        inet_pton(AF_INET, slave_ip.c_str(), &g_slave_addr.sin_addr);

        // 绑定文件传输端口 (port + 100)
        {
            sockaddr_in bind_addr{};
            bind_addr.sin_family = AF_INET;
            bind_addr.sin_port = htons(g_net_port + 100);
            if (g_is_master) {
                inet_pton(AF_INET, master_ip.c_str(), &bind_addr.sin_addr);
            } else {
                inet_pton(AF_INET, slave_ip.c_str(), &bind_addr.sin_addr);
            }
            if (::bind(g_udp_sock, (sockaddr*)&bind_addr, sizeof(bind_addr)) == -1) {
                cerr << "Warning: File transfer port bind failed" << endl;
            }
        }

        // Slave: 启动命令监听线程
        if (!g_is_master) {
            listener_thread = thread(udpListenerWorker, slave_ip, g_net_port);
        }
    }

    // === 创建相机上下文 ===
    for (size_t i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(camera_ids[i]);
        cam_ctxs.push_back(ctx);
    }

    // === 启动相机线程 ===
    for (auto& ctx : cam_ctxs) {
        ctx->running = true;
        ctx->copy_thread = thread(copyWorker, ctx);
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain_val, gamma_val, exp_time);
    }

    // === 预览窗口 ===
    cv::namedWindow("Calib Capture", cv::WINDOW_NORMAL);
    cv::resizeWindow("Calib Capture", win_w, win_h);

    auto ui_interval = chrono::milliseconds(static_cast<int>(1000.0 / ui_fps));
    auto last_ui_time = chrono::steady_clock::now() - ui_interval;

    cout << "\n=== Ready ===" << endl;
    cout << "  SPACE - Take one calibration photo";
    if (g_enable_net_sync) cout << " (synced)";
    cout << endl;
    if (g_is_master && g_enable_net_sync) cout << "  T     - Pull slave images" << endl;
    cout << "  Q/ESC - Quit\n" << endl;

    while (global_running) {
        auto current_time = chrono::steady_clock::now();
        bool need_ui_update = (current_time - last_ui_time) >= ui_interval;

        // ===== 1. UI 渲染 (按 ui_fps 控制) =====
        if (need_ui_update) {
            int n_cams = static_cast<int>(cam_ctxs.size());
            int grid_rows = 1, grid_cols = 1;
            if (n_cams <= 1) { grid_rows = 1; grid_cols = 1; }
            else if (n_cams <= 4) { grid_rows = 2; grid_cols = 2; }
            else if (n_cams <= 9) { grid_rows = 3; grid_cols = 3; }
            else { grid_rows = 4; grid_cols = (n_cams + 3) / 4; }

            int cell_w = win_w / grid_cols;
            int cell_h = win_h / grid_rows;
            cv::Mat canvas = cv::Mat::zeros(win_h, win_w, CV_8UC3);

            for (size_t i = 0; i < cam_ctxs.size(); ++i) {
                cv::Mat local_raw;
                {
                    lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);
                    if (!cam_ctxs[i]->latest_frame.empty())
                        local_raw = cam_ctxs[i]->latest_frame.clone();
                }

                cv::Mat cell;
                if (!local_raw.empty()) {
                    cv::Mat color;
                    if (cam_ctxs[i]->is_mono) cv::cvtColor(local_raw, color, cv::COLOR_GRAY2RGB);
                    else cv::cvtColor(local_raw, color, cv::COLOR_BayerRG2RGB);
                    cv::resize(color, cell, cv::Size(cell_w, cell_h));
                } else {
                    cell = cv::Mat::zeros(cell_h, cell_w, CV_8UC3);
                }

                int r = static_cast<int>(i) / grid_cols;
                int c = static_cast<int>(i) % grid_cols;
                cell.copyTo(canvas(cv::Rect(c * cell_w, r * cell_h, cell_w, cell_h)));
            }

            // 传输进度条
            if (xfer.active && xfer.expected_total > 0) {
                int bar_h = 30;
                int bar_y = canvas.rows - bar_h - 10;
                int bar_x = 20;
                int bar_w = canvas.cols - 40;
                float ratio = static_cast<float>(xfer.received) / xfer.expected_total;
                cv::rectangle(canvas, cv::Point(bar_x, bar_y), cv::Point(bar_x + bar_w, bar_y + bar_h), cv::Scalar(80, 80, 80), -1);
                cv::rectangle(canvas, cv::Point(bar_x, bar_y), cv::Point(bar_x + static_cast<int>(bar_w * ratio), bar_y + bar_h), cv::Scalar(0, 200, 0), -1);
                cv::rectangle(canvas, cv::Point(bar_x, bar_y), cv::Point(bar_x + bar_w, bar_y + bar_h), cv::Scalar(255, 255, 255), 1);
                string prog_text = "XFER " + to_string(xfer.received) + "/" + to_string(xfer.expected_total);
                auto elapsed = chrono::duration<double>(chrono::steady_clock::now() - xfer.start_time).count();
                if (elapsed > 0.1) {
                    double speed_mbps = (xfer.total_bytes / 1048576.0) / elapsed;
                    stringstream ss_speed;
                    ss_speed << fixed << setprecision(1) << speed_mbps;
                    prog_text += "  " + ss_speed.str() + " MB/s";
                }
                cv::putText(canvas, prog_text, cv::Point(bar_x + 10, bar_y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }

            cv::imshow("Calib Capture", canvas);
            last_ui_time = current_time;
        }

        // ===== 2. 传输状态机 (每次循环尝试收一帧) =====
        if (xfer.active) {
            if (xfer.received < xfer.expected_total
                && chrono::steady_clock::now() < xfer.deadline
                && xfer.consecutive_empty < 6) {
                // 强制下一轮 UI 立即刷新，确保进度条可见
                last_ui_time = chrono::steady_clock::now() - ui_interval;

                string sn;
                uint32_t file_idx = 0;
                vector<uint8_t> jpeg_data = recvImageViaUdp(UINT32_MAX, &sn, &file_idx, 500);
                if (!jpeg_data.empty() && !sn.empty()) {
                    stringstream ss;
                    ss << setw(2) << setfill('0') << file_idx;
                    string out_fn = g_calib_save_dir + "/calib_cam_" + sn + "_" + ss.str() + ".jpg";
                    ofstream out(out_fn, ios::binary);
                    out.write(reinterpret_cast<const char*>(jpeg_data.data()), jpeg_data.size());
                    xfer.received++;
                    xfer.total_bytes += jpeg_data.size();
                    xfer.consecutive_empty = 0;
                    cout << "  -> Received " << out_fn << " (" << jpeg_data.size() << " bytes)" << endl;
                } else {
                    xfer.consecutive_empty++;
                }
            } else {
                // 传输完成或超时
                if (xfer.consecutive_empty >= 6) {
                    cout << "  Early finish: no data from slave for " << xfer.consecutive_empty
                         << " rounds." << endl;
                }
                auto elapsed = chrono::duration<double>(chrono::steady_clock::now() - xfer.start_time);
                double speed_mbps = elapsed.count() > 0 ? (xfer.total_bytes / 1048576.0) / elapsed.count() : 0;
                cout << "\n=== Transfer Complete ===" << endl;
                cout << "Files    : " << xfer.received << " / " << xfer.expected_total << endl;
                cout << "Data     : " << (xfer.total_bytes / 1048576.0) << " MB" << endl;
                cout << "Time     : " << fixed << setprecision(2) << elapsed.count() << " s" << endl;
                cout << "Speed    : " << fixed << setprecision(2) << speed_mbps << " MB/s" << endl;
                xfer.active = false;
            }
        }

        // ===== 3. 键盘事件 (1ms 轮询 — 极速响应) =====
        char key = static_cast<char>(cv::waitKey(1));

        if (key == 'q' || key == 27) {
            global_running = false;
        }
        else if (key == ' ') {
            if (g_enable_net_sync && !g_is_master) {
                cout << "[Slave] Waiting for master SPACE..." << endl;
            } else {
                int counter = getNextCalibCounter(g_calib_save_dir);
                stringstream ss;
                ss << setw(2) << setfill('0') << counter;

                // 通知 slave
                if (g_enable_net_sync && g_is_master) {
                    fastUdpSend(g_slave_addr, "PHOTO:" + to_string(counter));
                }

                cout << "\n[Photo] Capturing index " << counter << endl;
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
                        string fn = g_calib_save_dir + "/calib_cam_" + ctx->sn + "_" + ss.str() + ".jpg";
                        cv::imwrite(fn, out_img);
                        cout << "  -> Saved " << fn << endl;
                    }
                }
            }
        }
        else if ((key == 't' || key == 'T') && g_is_master && g_enable_net_sync && !xfer.active) {
            int max_idx = getNextCalibCounter(g_calib_save_dir) - 1;
            if (max_idx < 0) {
                cout << "No local images found. Capture first." << endl;
            } else {
                xfer.active = true;
                xfer.received = 0;
                xfer.expected_total = static_cast<int>(cam_ctxs.size()) * (max_idx + 1);
                xfer.total_bytes = 0;
                xfer.consecutive_empty = 0;
                xfer.start_time = chrono::steady_clock::now();
                xfer.deadline = xfer.start_time + chrono::seconds(120);

                fastUdpSend(g_slave_addr, "XFER:0," + to_string(max_idx));
                cout << "\n=== Transferring " << xfer.expected_total << " Slave Images ===" << endl;
            }
        }
    }

    // === 清理 ===
    cout << "[System] Shutting down..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->copy_cv.notify_all();
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
    }
    if (listener_thread.joinable()) listener_thread.join();

    cv::destroyAllWindows();
    Pylon::PylonTerminate();
    if (g_udp_sock != INVALID_SOCKET) closesocket(g_udp_sock);

#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
