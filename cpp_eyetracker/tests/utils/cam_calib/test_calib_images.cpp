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

// ================== UDP 全局句柄 ==================
SOCKET g_udp_sock = INVALID_SOCKET;
sockaddr_in g_master_addr{};
sockaddr_in g_slave_addr{};
int g_net_port = 0;
string g_calib_save_dir;
bool g_enable_net_sync = false;
bool g_is_master = false;
atomic<bool> g_xfer_active{false};
int g_win_w = 1224;
int g_win_h = 1024;

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
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

// --- 可靠的 UDP 接收辅助函数 ---

void drainSocket(int timeout_ms) {
#ifdef _WIN32
    DWORD timeout = timeout_ms;
    setsockopt(g_udp_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif
    vector<char> buf(65536);
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int bytes = recvfrom(g_udp_sock, buf.data(), static_cast<int>(buf.size()), 0, nullptr, nullptr);
        if (bytes <= 0) break;
    }
}

// 接收短字符串响应（如 LIST_RESP, DONE, CLEAR_DONE 等控制消息）
string recvStringResponse(int timeout_ms) {
#ifdef _WIN32
    DWORD timeout = min(timeout_ms, 2000);
    setsockopt(g_udp_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif
    vector<char> buf(65536);
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        sockaddr_in sender;
        socklen_t sender_len = sizeof(sender);
        int bytes = recvfrom(g_udp_sock, buf.data(), static_cast<int>(buf.size()) - 1, 0,
                             (sockaddr*)&sender, &sender_len);
        if (bytes > 0) {
            buf[bytes] = '\0';
            string data(buf.data(), bytes);
            // 只接受已知协议前缀的控制消息；跳过意外到达的 chunk 数据
            if (data.rfind("LIST_RESP:", 0) == 0 ||
                data.rfind("DONE:", 0) == 0 ||
                data.rfind("CLEAR_DONE", 0) == 0) {
                return data;
            }
        }
    }
    return {};
}

// 接收单个文件的 chunk 传输（阻塞，直到组装完成或超时）
vector<uint8_t> recvSingleFile(const string& expected_sn, uint32_t expected_idx, int timeout_ms) {
#ifdef _WIN32
    DWORD timeout = min(timeout_ms, 2000);
    setsockopt(g_udp_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif
    map<uint32_t, vector<uint8_t>> chunk_map;
    uint32_t total_chunks = 0;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    vector<uint8_t> recv_buf(65536);

    while (chrono::steady_clock::now() < deadline) {
        sockaddr_in sender;
        socklen_t sender_len = sizeof(sender);
        int bytes = recvfrom(g_udp_sock, reinterpret_cast<char*>(recv_buf.data()),
                             static_cast<int>(recv_buf.size()), 0, (sockaddr*)&sender, &sender_len);

        if (bytes < static_cast<int>(sizeof(FileTransferHeader))) {
            // 检查是否有 DONE 短消息
            if (bytes > 0) {
                string msg(reinterpret_cast<char*>(recv_buf.data()), bytes);
                if (msg == "DONE:" + expected_sn + "_" + to_string(expected_idx)) break;
            }
            continue;
        }

        auto* hdr = reinterpret_cast<FileTransferHeader*>(recv_buf.data());
        string chunk_sn(hdr->sn);
        if (chunk_sn != expected_sn || hdr->file_index != expected_idx) continue;

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
    // 超时：如果有完整 chunk 集合就组装，否则返回空
    if (total_chunks > 0 && chunk_map.size() == total_chunks) {
        vector<uint8_t> result;
        for (uint32_t i = 0; i < total_chunks; ++i) {
            auto it = chunk_map.find(i);
            if (it == chunk_map.end()) return {};
            result.insert(result.end(), it->second.begin(), it->second.end());
        }
        return result;
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
atomic<int> g_enlarged_cam{-1};

// UI layout constants (recomputed each frame)
int g_left_w = 0, g_right_x = 0, g_right_w = 0;
int g_thumb_w = 0, g_thumb_h = 0;

// 扫描本地文件夹，返回 "SN_idx" 的集合
set<string> scanLocalCalibFiles() {
    set<string> files;
    if (!fs::exists(g_calib_save_dir)) return files;
    for (auto& e : fs::directory_iterator(g_calib_save_dir)) {
        if (e.path().extension() == ".jpg") {
            string stem = e.path().stem().string();
            if (stem.rfind("calib_cam_", 0) == 0)
                files.insert(stem.substr(10)); // 去掉 "calib_cam_" 前缀
        }
    }
    return files;
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
void renderThumbnailGrid(cv::Mat& canvas, int selected_idx) {
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
            }

            // SN label drawn on cell
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

            // Green border when selected
            if (i == selected_idx) {
                cv::rectangle(canvas, roi, cv::Scalar(0, 255, 0), 2);
            }
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
    resized.copyTo(canvas(cv::Rect(off_x, off_y, dst_w, dst_h)));
}

void showTransferringOverlay() {
    cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
    string text = "Transferring...";
    int baseline = 0;
    cv::Size text_sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.5, 3, &baseline);
    cv::Point text_org((g_win_w - text_sz.width) / 2, (g_win_h + text_sz.height) / 2);
    cv::putText(canvas, text, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 255, 255), 3);
    cv::imshow("Calib Capture", canvas);
    cv::waitKey(1);
}

// 同步传输缺失的标定图片（阻塞调用，每次一张）
void transferMissingImages() {
    g_xfer_active = true;
    showTransferringOverlay();

    cout << "\n=== Querying Slave File List ===" << endl;

    drainSocket(100);
    fastUdpSend(g_slave_addr, "LIST");

    string resp = recvStringResponse(3000);
    if (resp.empty() || resp.rfind("LIST_RESP:", 0) != 0) {
        cout << "[Error] No response from slave." << endl;
        g_xfer_active = false;
        return;
    }

    // 解析 slave 文件列表
    set<string> slave_files;
    stringstream ss(resp.substr(10)); // 去掉 "LIST_RESP:"
    string token;
    while (getline(ss, token, ',')) {
        if (!token.empty()) slave_files.insert(token);
    }
    cout << "Slave has " << slave_files.size() << " files." << endl;

    set<string> local_files = scanLocalCalibFiles();
    cout << "Master has " << local_files.size() << " files." << endl;

    // 找缺失
    vector<string> missing;
    for (auto& f : slave_files)
        if (!local_files.count(f)) missing.push_back(f);

    if (missing.empty()) {
        cout << "All files in sync. Nothing to transfer.\n" << endl;
        g_xfer_active = false;
        return;
    }

    cout << "Transferring " << missing.size() << " missing files:\n" << endl;

    // 通知 slave 进入传输状态
    fastUdpSend(g_slave_addr, "XFER_INFO:" + to_string(missing.size()));

    int received = 0;
    size_t total_bytes = 0;
    auto start_time = chrono::steady_clock::now();

    for (size_t i = 0; i < missing.size(); ++i) {
        const string& sn_idx = missing[i];
        size_t us = sn_idx.rfind('_');
        string sn = sn_idx.substr(0, us);
        uint32_t idx = static_cast<uint32_t>(stoi(sn_idx.substr(us + 1)));

        cout << "[" << (i + 1) << "/" << missing.size() << "] "
             << sn << " idx=" << idx << " ... " << flush;

        showTransferringOverlay();

        fastUdpSend(g_slave_addr, "GET:" + sn_idx);

        vector<uint8_t> jpeg_data = recvSingleFile(sn, idx, 10000);
        if (!jpeg_data.empty()) {
            stringstream fss;
            fss << setw(2) << setfill('0') << idx;
            string fn = g_calib_save_dir + "/calib_cam_" + sn + "_" + fss.str() + ".jpg";
            ofstream out(fn, ios::binary);
            out.write(reinterpret_cast<const char*>(jpeg_data.data()), jpeg_data.size());
            received++;
            total_bytes += jpeg_data.size();
            cout << "OK (" << jpeg_data.size() << " bytes)" << endl;
        } else {
            cout << "FAILED" << endl;
        }
    }

    // 通知 slave 传输结束
    fastUdpSend(g_slave_addr, "XFER_END");

    auto elapsed = chrono::duration<double>(chrono::steady_clock::now() - start_time);
    double speed_mbps = elapsed.count() > 0 ? (total_bytes / 1048576.0) / elapsed.count() : 0;
    cout << "\n=== Transfer Complete ===" << endl;
    cout << "Files: " << received << " / " << missing.size() << endl;
    cout << "Data : " << fixed << setprecision(2) << (total_bytes / 1048576.0) << " MB" << endl;
    cout << "Time : " << fixed << setprecision(2) << elapsed.count() << " s" << endl;
    cout << "Speed: " << fixed << setprecision(2) << speed_mbps << " MB/s\n" << endl;

    g_xfer_active = false;
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

    char buffer[4096];
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
            else if (cmd == "LIST") {
                cout << "[Slave] LIST request" << endl;
                stringstream file_list;
                if (fs::exists(g_calib_save_dir)) {
                    for (auto& e : fs::directory_iterator(g_calib_save_dir)) {
                        if (e.path().extension() == ".jpg") {
                            string stem = e.path().stem().string();
                            if (stem.rfind("calib_cam_", 0) == 0)
                                file_list << stem.substr(10) << ",";
                        }
                    }
                }
                string resp = "LIST_RESP:" + file_list.str();
                fastUdpSend(client_addr, resp);
                cout << "[Slave] Sent file list" << endl;
            }
            else if (cmd.rfind("GET:", 0) == 0) {
                string sn_idx = cmd.substr(4); // "SN_idx"
                size_t us = sn_idx.rfind('_');
                if (us != string::npos) {
                    string sn = sn_idx.substr(0, us);
                    int idx = stoi(sn_idx.substr(us + 1));
                    stringstream ss;
                    ss << setw(2) << setfill('0') << idx;
                    string fn = g_calib_save_dir + "/calib_cam_" + sn + "_" + ss.str() + ".jpg";

                    if (fs::exists(fn)) {
                        ifstream in(fn, ios::binary | ios::ate);
                        if (in) {
                            size_t fsize = in.tellg();
                            in.seekg(0, ios::beg);
                            vector<uint8_t> jpeg_data(fsize);
                            in.read(reinterpret_cast<char*>(jpeg_data.data()), fsize);
                            in.close();
                            sendImageViaUdp(client_addr, jpeg_data, idx, sn);
                            cout << "[Slave] Sent " << fn << endl;
                        }
                    }
                    // 无论文件是否存在都发 DONE，让 master 确认传输结束
                    fastUdpSend(client_addr, "DONE:" + sn_idx);
                }
            }
            else if (cmd == "CLEAR") {
                cout << "[Slave] CLEAR — removing all calibration photos" << endl;
                if (fs::exists(g_calib_save_dir)) {
                    for (auto& e : fs::directory_iterator(g_calib_save_dir)) {
                        if (e.path().extension() == ".jpg")
                            fs::remove(e.path());
                    }
                }
                fastUdpSend(client_addr, "CLEAR_DONE");
                cout << "[Slave] Cleared." << endl;
            }
            else if (cmd.rfind("XFER_INFO:", 0) == 0) {
                cout << "[Slave] Transfer started" << endl;
                g_xfer_active = true;
            }
            else if (cmd == "XFER_END") {
                cout << "[Slave] Transfer ended" << endl;
                g_xfer_active = false;
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

    g_win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    g_win_h = cfg["test_multi_cam"]["window_height"].as<int>();
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

        // 增大 UDP 接收缓冲区，防止高速多 chunk 传输时内核丢包
        {
            int rcvbuf = 8 * 1024 * 1024; // 8 MB
            setsockopt(g_udp_sock, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf, sizeof(rcvbuf));
        }

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
    cv::resizeWindow("Calib Capture", g_win_w, g_win_h);
    updateLayout();
    cv::setMouseCallback("Calib Capture", onMouse);

    auto ui_interval = chrono::milliseconds(static_cast<int>(1000.0 / ui_fps));
    auto last_ui_time = chrono::steady_clock::now() - ui_interval;

    cout << "\n=== Ready ===" << endl;
    cout << "  SPACE - Take one calibration photo";
    if (g_enable_net_sync) cout << " (synced)";
    cout << endl;
    if (g_is_master && g_enable_net_sync) {
        cout << "  T     - Pull missing slave images" << endl;
        cout << "  C     - Clear all calibration photos (both hosts)" << endl;
    }
    cout << "  Q/ESC - Quit\n" << endl;

    while (global_running) {
        auto current_time = chrono::steady_clock::now();
        bool need_ui_update = (current_time - last_ui_time) >= ui_interval;

        // ===== 1. UI 渲染 (按 ui_fps 控制) =====
        if (need_ui_update) {
            if (!g_is_master && g_xfer_active) {
                showTransferringOverlay();
            } else {
                cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
                int sel = g_enlarged_cam.load();
                renderThumbnailGrid(canvas, sel);
                renderEnlargedView(canvas, sel);
                // Vertical separator
                cv::line(canvas, cv::Point(g_left_w, 0), cv::Point(g_left_w, g_win_h),
                         cv::Scalar(60, 60, 60), 2);
                cv::imshow("Calib Capture", canvas);
            }
            last_ui_time = current_time;
        }

        // ===== 2. 键盘事件 (1ms 轮询 — 极速响应) =====
        char key = static_cast<char>(cv::waitKey(1));

        if (key == 'q' || key == 27) {
            global_running = false;
        }
        else if (key == ' ') {
            if (g_xfer_active) {
                // Block SPACE during transfer on both hosts
            } else if (g_enable_net_sync && !g_is_master) {
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
        else if ((key == 't' || key == 'T') && g_is_master && g_enable_net_sync && !g_xfer_active) {
            // 传输缺失的 slave 图片（同步阻塞调用）
            transferMissingImages();
            // 传输期间 UI 被阻塞，返回后刷新 UI
            last_ui_time = chrono::steady_clock::now() - ui_interval;
        }
        else if ((key == 'c' || key == 'C') && g_is_master && g_enable_net_sync && !g_xfer_active) {
            cout << "\n[Clear] Removing local calibration photos..." << endl;
            if (fs::exists(g_calib_save_dir)) {
                for (auto& e : fs::directory_iterator(g_calib_save_dir)) {
                    if (e.path().extension() == ".jpg")
                        fs::remove(e.path());
                }
            }
            cout << "[Clear] Local photos removed." << endl;

            fastUdpSend(g_slave_addr, "CLEAR");
            this_thread::sleep_for(chrono::milliseconds(200));
            drainSocket(200);
            cout << "[Clear] Slave photos should be removed.\n" << endl;
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
