// ================== send_ui ==================
// 交互式串行传输 (Windows, master/slave 自动识别): OpenCV UI 确认配置 →
// SPACE 开始 → 逐阶段串行发送到处理主机 (baseline_recv_data --out /data/dataset)。
//
// 阶段 (slave: 1-2; master: 1-5):
//   1/2. D:/capture、E:/capture 的 h5 → /data/dataset/capture/{P}/
//   3. 相机内外参 XML ({calib_save_dir}/{P}/output) → /data/dataset/calib/cams/{P}/
//   4. 红外发射器位置 cfg/IR/{day_id}.txt → /data/dataset/calib/IR/{day_id}.txt
//   5. cfg/day_participant_map.json → /data/dataset/day_participant_map.json
//
// 配置: transfer.yaml (participant_id/链路/流数), capture.yaml (is_master),
//       cam_calib.yaml (calib_save_dir), calib_arm.yaml (record.day_id)
// 按键: SPACE=开始, q/ESC=退出 (传输中 q=当前文件完成后温和中止)
// 引擎: TransmitFile 内核态读发重叠, 默认 4 流, SKIP 断点续传。
// =================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #include <mswsock.h>              // TransmitFile: 内核级 读盘+发送 重叠
    #pragma comment(lib, "ws2_32.lib")
    #pragma comment(lib, "Mswsock.lib")
#else
    #error "sender is Windows-only (collection hosts)"
#endif

#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstdlib>

#include "cfg/config.hpp"

using namespace std;
namespace fs = std::filesystem;
using namespace gazeestimation;

static const int CHUNK = 8 * 1024 * 1024;
static const int SOCK_BUF = 32 * 1024 * 1024;
static const int RETRIES = 3;
static const int REPLY_TIMEOUT_MS = 60000;

// ================== 配置 ==================
struct UiCfg {
    string server_ip;                 // 按 capture.yaml is_master 选 master/slave 链路
    int server_port = 5001;
    int workers = 4;
    string participant;
    bool is_master = false;
    vector<string> roots;             // 图像盘串行顺序: D:/capture, E:/capture
    string cfg_dir;                   // cpp_eyetracker/cfg (xml/IR/map 源)
    string xml_dir;                   // {calib_save_dir}/{P}/output (master)
    string ir_file;                   // cfg/IR/{day_id}.txt (master)
    string map_file;                  // cfg/day_participant_map.json (master)
    string master_ip;                 // 握手用 (capture.yaml network.master_ip)
    int handshake_port = 50100;       // transfer.yaml
};
static UiCfg g_cfg;

// ================== 传输状态 ==================
struct Job { fs::path path; string rel; uint64_t size; bool force = false; };   // force: 覆盖式 (标定附件)
struct PhasePlan { string label; vector<Job> jobs; uint64_t bytes = 0; };
static vector<PhasePlan> g_plans;

enum class Phase { CONFIG, RUNNING, WAIT_SLAVE, DONE, ABORTED };
static atomic<int> g_phase{(int)Phase::CONFIG};
static atomic<bool> g_abort{false};
static atomic<int> g_done_files{0}, g_skip{0}, g_fail{0};
static atomic<uint64_t> g_bytes{0};
static atomic<int> g_plan_idx{0};
static uint64_t g_total_bytes = 0;
static int g_total_files = 0;
static atomic<uint64_t> g_t0_us{0}, g_t_end_us{0};
static string g_last_fail;

// master↔slave 握手 (走现有 192.168.10.x 网): master SPACE 后先传自己,
// 完成后 START 令 slave 传输, 收 SLAVE_DONE 回 ACK — 全程 slave 按键无效
static SOCKET g_hs = INVALID_SOCKET;
static atomic<bool> g_slave_ready{false};
static string g_slave_summary;

// ================== 日志 (时间戳) ==================
static string ts() {
    char b[32];
    auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
    strftime(b, sizeof(b), "%H:%M:%S", localtime(&t));
    return string("[") + b + "] ";
}
#define LOG(msg) (cout << ts() << msg << endl)

// ================== TCP 基础 (同 send_slave) ==================
static bool sendLine(SOCKET s, const string& msg) {
    string d = msg + "\n";
    return send(s, d.data(), (int)d.size(), 0) == (int)d.size();
}
static bool recvLine(SOCKET s, string& line, int timeout_ms) {
    DWORD to = timeout_ms;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
    char buf[4096]; string acc;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int n = recv(s, buf, sizeof(buf), 0);
        if (n <= 0) return false;
        acc.append(buf, n);
        size_t nl = acc.find('\n');
        if (nl != string::npos) {
            line = acc.substr(0, nl);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
    }
    return false;
}
static SOCKET connectTo(const string& ip, int port) {
    int retry = 0;
    while (true) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        sockaddr_in sa{};
        sa.sin_family = AF_INET; sa.sin_port = htons((u_short)port);
        inet_pton(AF_INET, ip.c_str(), &sa.sin_addr);
        if (connect(s, (sockaddr*)&sa, sizeof(sa)) == 0) return s;
        closesocket(s);
        ++retry;
        cout << ts() << "[Net] connect " << ip << ":" << port
             << " failed (retry #" << retry << ", receiver running?)" << endl;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
}
static SOCKET openStream() {
    SOCKET s = connectTo(g_cfg.server_ip, g_cfg.server_port);
    int one = 1, buf = SOCK_BUF;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char*)&buf, sizeof(buf));
    return s;
}

// ================== 传输引擎 (TransmitFile) ==================
static int sendOne(SOCKET s, const Job& j) {
    HANDLE hf = CreateFileA(j.path.string().c_str(), GENERIC_READ, FILE_SHARE_READ,
                            NULL, OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, NULL);
    if (hf == INVALID_HANDLE_VALUE) return -1;
    // FILE   = 大小一致则 SKIP (断点续传, 图像用)
    // FORCE  = 总是覆盖写入 (标定附件: xml / IR / map)
    string hdr = string(j.force ? "FORCE " : "FILE ") + j.rel + " "
               + to_string(j.size) + " 0";
    if (!sendLine(s, hdr)) { CloseHandle(hf); return -1; }
    uint64_t remaining = j.size;
    while (remaining > 0) {
        DWORD chunk = (DWORD)min<uint64_t>(remaining, 1ull << 30);
        if (!TransmitFile(s, hf, chunk, 0, NULL, NULL, 0)) { CloseHandle(hf); return -1; }
        remaining -= chunk;
    }
    CloseHandle(hf);
    string reply;
    if (!recvLine(s, reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("OK", 0) == 0) return 0;
    if (reply.rfind("SKIP", 0) == 0) return 1;
    return -1;
}

static void workerLoop(vector<Job>* jobs, atomic<size_t>* next) {
    while (!g_abort.load()) {
        size_t i = next->fetch_add(1);
        if (i >= jobs->size()) return;
        const Job& j = (*jobs)[i];
        int r = -1;
        for (int attempt = 1; attempt <= RETRIES; ++attempt) {
            SOCKET s = openStream();
            r = sendOne(s, j);
            closesocket(s);
            if (r >= 0) break;
            if (attempt == RETRIES) break;
            this_thread::sleep_for(chrono::seconds(1));
        }
        g_done_files++;
        if (r == 0) g_bytes += j.size;
        else if (r == 1) g_skip++;
        else { g_fail++; g_last_fail = j.rel; }
    }
}

static void transferController() {
    g_t0_us.store(chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count());
    for (size_t d = 0; d < g_plans.size(); ++d) {
        if (g_abort.load()) break;
        g_plan_idx = (int)d;
        atomic<size_t> next{0};
        vector<thread> ths;
        for (int i = 0; i < g_cfg.workers; ++i)
            ths.emplace_back(workerLoop, &g_plans[d].jobs, &next);
        for (auto& t : ths) t.join();
    }
    g_t_end_us.store(chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count());

    // master: 本机完成后令 slave 传输, 等 SLAVE_DONE
    if (g_cfg.is_master && g_hs != INVALID_SOCKET) {
        sendLine(g_hs, g_abort.load() ? "ABORT" : "START");
        if (!g_abort.load()) {
            g_phase.store((int)Phase::WAIT_SLAVE);
            cout << ts() << "[HS] Waiting for slave to finish..." << endl;
            string line;
            if (recvLine(g_hs, line, 4 * 3600 * 1000) &&
                line.rfind("SLAVE_DONE", 0) == 0) {
                g_slave_summary = line.substr(10);
                cout << ts() << "[HS] Slave done:" << g_slave_summary << endl;
            } else {
                g_slave_summary = " (no response)";
                cout << ts() << "[HS] Slave no SLAVE_DONE response!" << endl;
                g_fail += 1;                          // 对端异常计入失败
            }
            sendLine(g_hs, "ACK");
        }
        closesocket(g_hs);
        g_hs = INVALID_SOCKET;
    }
    g_phase.store((int)(g_abort.load() ? Phase::ABORTED : Phase::DONE));
}

// master: 监听握手, 等 slave READY (SPACE 需 g_slave_ready)
static void masterHandshakeListen() {
    SOCKET lst = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int one = 1;
    setsockopt(lst, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons((u_short)g_cfg.handshake_port);
    sa.sin_addr.s_addr = INADDR_ANY;
    if (::bind(lst, (sockaddr*)&sa, sizeof(sa)) != 0 || listen(lst, 1) != 0) {
        cerr << ts() << "[HS] master bind/listen failed: " << WSAGetLastError() << endl;
        return;
    }
    cout << ts() << "[HS] Master waiting for slave handshake on :" << g_cfg.handshake_port << endl;
    SOCKET conn = accept(lst, nullptr, nullptr);
    closesocket(lst);
    if (conn == INVALID_SOCKET) return;
    string line;
    if (recvLine(conn, line, 600000) && line.rfind("READY", 0) == 0) {
        cout << ts() << "[HS] Slave: " << line << endl;
        g_hs = conn;
        g_slave_ready.store(true);
    } else {
        cerr << ts() << "[HS] bad READY: " << line << endl;
        closesocket(conn);
    }
}

// slave 看门狗: 传输期间监测握手 socket — master 发来 QUIT/ABORT 或连接关闭
// (master 退出/崩溃) 一律中止当前传输 (文件粒度温和停止)
static void slaveQuitWatcher(SOCKET hs) {
    while ((Phase)g_phase.load() == Phase::RUNNING) {
        fd_set rf; FD_ZERO(&rf); FD_SET(hs, &rf);
        timeval tv{2, 0};
        if (select(0, &rf, nullptr, nullptr, &tv) > 0) {
            g_abort = true;
            cout << ts() << "[HS] master signalled quit / disconnected — aborting" << endl;
            return;
        }
    }
}

// slave: 连 master → READY → 等 START (master SPACE 且本机完成后才来) → 传输
// 完成后回 SLAVE_DONE, 等 ACK; slave 全程无有效按键
static void slaveHandshakeAndRun() {
    cout << ts() << "[HS] Slave connecting to master " << g_cfg.master_ip
         << ":" << g_cfg.handshake_port << " ..." << endl;
    SOCKET s = connectTo(g_cfg.master_ip, g_cfg.handshake_port);
    sendLine(s, "READY " + to_string(g_total_files) + " " + to_string(g_total_bytes));
    string cmd;
    if (!recvLine(s, cmd, 2 * 3600 * 1000) || cmd != "START") {
        cerr << ts() << "[HS] master cmd: " << cmd << " — aborted" << endl;
        closesocket(s);
        g_phase.store((int)Phase::ABORTED);
        return;
    }
    cout << ts() << "[HS] Master ordered START — slave transferring." << endl;
    g_phase.store((int)Phase::RUNNING);               // UI 切进度屏
    thread(slaveQuitWatcher, s).detach();             // 监测 master QUIT/掉线
    transferController();                             // 结束时置 DONE/ABORTED
    int ok = g_done_files.load() - g_skip.load() - g_fail.load();
    sendLine(s, "SLAVE_DONE " + to_string(ok) + " "
                  + to_string(g_skip.load()) + " " + to_string(g_fail.load()));
    string ack;
    recvLine(s, ack, 60000);                          // ACK (尽力而为)
    closesocket(s);
}

// ================== 阶段构建 ==================
static void addImagePhase(const string& root) {
    PhasePlan plan;
    plan.label = root + "  (h5 -> capture/" + g_cfg.participant + ")";
    fs::path base = fs::path(root) / g_cfg.participant;
    if (fs::exists(base)) {
        for (auto& e : fs::recursive_directory_iterator(base)) {
            if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
            string rel = "capture/" + g_cfg.participant + "/"
                       + e.path().parent_path().filename().string()
                       + "/" + e.path().filename().string();
            uint64_t sz = (uint64_t)e.file_size();
            plan.jobs.push_back({e.path(), rel, sz});
            plan.bytes += sz;
        }
        sort(plan.jobs.begin(), plan.jobs.end(),
             [](const Job& a, const Job& b) { return a.rel < b.rel; });
    } else {
        cout << ts() << "[Warn] missing " << base.string() << endl;
    }
    g_total_files += (int)plan.jobs.size();
    g_total_bytes += plan.bytes;
    g_plans.push_back(move(plan));
}

static void addFilePhase(const string& src, const string& rel, const string& label) {
    PhasePlan plan;
    plan.label = label;
    error_code ec;
    if (fs::exists(src, ec) && fs::is_regular_file(src, ec)) {
        uint64_t sz = (uint64_t)fs::file_size(src, ec);
        plan.jobs.push_back({fs::path(src), rel, sz, /*force=*/true});
        plan.bytes = sz;
    } else {
        cout << ts() << "[Warn] missing (phase skipped): " << src << endl;
        return;                                  // 不加入 (配置屏不显示缺失项)
    }
    g_total_files += 1;
    g_total_bytes += plan.bytes;
    g_plans.push_back(move(plan));
}

static void addXmlPhase() {
    PhasePlan plan;
    plan.label = g_cfg.xml_dir + "  (xml -> calib/cams/" + g_cfg.participant + ")";
    if (fs::exists(g_cfg.xml_dir)) {
        for (auto& e : fs::recursive_directory_iterator(g_cfg.xml_dir)) {
            if (!e.is_regular_file() || e.path().extension() != ".xml") continue;
            fs::path sub = fs::relative(e.path(), g_cfg.xml_dir);
            string rel = "calib/cams/" + g_cfg.participant + "/" + sub.generic_string();
            uint64_t sz = (uint64_t)e.file_size();
            plan.jobs.push_back({e.path(), rel, sz, /*force=*/true});
            plan.bytes += sz;
        }
        sort(plan.jobs.begin(), plan.jobs.end(),
             [](const Job& a, const Job& b) { return a.rel < b.rel; });
    } else {
        cout << ts() << "[Warn] missing (phase skipped): " << g_cfg.xml_dir << endl;
        return;
    }
    g_total_files += (int)plan.jobs.size();
    g_total_bytes += plan.bytes;
    g_plans.push_back(move(plan));
}

// ================== UI ==================
static void drawConfig(cv::Mat& cv) {
    cv = cv::Mat::zeros(600, 900, CV_8UC3);
    int y = 44;
    auto put = [&](const string& s, double sc, cv::Scalar c, int dy = 30) {
        cv::putText(cv, s, {40, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, 1, cv::LINE_AA);
        y += dy;
    };
    put("100G Transfer - Configuration", 0.85, {0, 215, 255}, 46);
    put("Role        : " + string(g_cfg.is_master ? "MASTER" : "SLAVE"), 0.55, {255, 255, 255});
    put("Participant : " + g_cfg.participant, 0.55, {255, 255, 255});
    put("Server      : " + g_cfg.server_ip + ":" + to_string(g_cfg.server_port), 0.55, {255, 255, 255});
    put("Streams     : " + to_string(g_cfg.workers) + "  (TransmitFile)", 0.55, {255, 255, 255});
    y += 8;
    put("Serial phases:", 0.55, {200, 200, 200}, 32);
    for (auto& p : g_plans) {
        ostringstream os;
        os << "  " << p.label << "   " << p.jobs.size() << " files  "
           << fixed << setprecision(2) << (double)p.bytes / 1e9 << " GB";
        put(os.str(), 0.45, {0, 255, 200}, 26);
    }
    y += 10;
    ostringstream tot;
    tot << "Total: " << g_total_files << " files, " << fixed << setprecision(2)
        << (double)g_total_bytes / 1e12 << " TB";
    put(tot.str(), 0.55, {0, 215, 255}, 52);
    if (g_cfg.is_master) {
        if (g_slave_ready.load())
            cv::putText(cv, "[SPACE] Start (master first, slave follows)      [q/ESC] Quit",
                        {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 0}, 2, cv::LINE_AA);
        else
            cv::putText(cv, "Waiting for slave handshake...      [q/ESC] Quit",
                        {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 200, 255}, 2, cv::LINE_AA);
    } else {
        cv::putText(cv, "Waiting for master command (no keys) - master SPACE starts sequence",
                    {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 200, 255}, 2, cv::LINE_AA);
    }
}

static void drawProgress(cv::Mat& cv) {
    cv = cv::Mat::zeros(600, 900, CV_8UC3);
    Phase ph = (Phase)g_phase.load();
    int y = 64;
    cv::putText(cv, ph == Phase::RUNNING ? "Transferring..." :
                ph == Phase::WAIT_SLAVE ? "Master done - slave transferring..." :
                (ph == Phase::DONE ? "DONE" : "ABORTED"),
                {40, y}, cv::FONT_HERSHEY_DUPLEX, 0.9, {0, 215, 255}, 2, cv::LINE_AA);
    y += 52;
    int idx = min(g_plan_idx.load(), (int)g_plans.size() - 1);
    ostringstream phs;
    phs << "Phase " << idx + 1 << "/" << g_plans.size() << "  " << g_plans[idx].label;
    cv::putText(cv, phs.str(), {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
    y += 44;
    double frac = g_total_bytes ? (double)g_bytes.load() / (double)g_total_bytes : 0;
    cv::rectangle(cv, {40, y}, {860, y + 36}, {80, 80, 80}, 1);
    cv::rectangle(cv, {40, y}, {40 + (int)(820 * min(frac, 1.0)), y + 36}, {0, 200, 0}, -1);
    char pct[16]; snprintf(pct, sizeof(pct), "%.1f%%", frac * 100);
    cv::putText(cv, pct, {425, y + 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1, cv::LINE_AA);
    y += 70;
    uint64_t t0 = g_t0_us.load();
    uint64_t tnow = g_t_end_us.load() ? g_t_end_us.load()
                  : chrono::duration_cast<chrono::microseconds>(
                        chrono::steady_clock::now().time_since_epoch()).count();
    double el = max((double)(tnow - t0) / 1e6, 1e-6);
    double rate = g_bytes.load() / el / 1e9;
    double eta = rate > 0.01 ? (double)(g_total_bytes - g_bytes.load()) / (rate * 1e9) : 0;
    char b[160];
    snprintf(b, sizeof(b), "Files : %d / %d    (skip %d, fail %d)",
             g_done_files.load(), g_total_files, g_skip.load(), g_fail.load());
    cv::putText(cv, b, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1, cv::LINE_AA); y += 34;
    snprintf(b, sizeof(b), "Data  : %.2f / %.2f TB", (double)g_bytes.load() / 1e12,
             (double)g_total_bytes / 1e12);
    cv::putText(cv, b, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 200}, 1, cv::LINE_AA); y += 34;
    snprintf(b, sizeof(b), "Rate  : %.2f GB/s    Elapsed : %.1f min    ETA : %.1f min",
             rate, el / 60, eta / 60);
    cv::putText(cv, b, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 200}, 1, cv::LINE_AA); y += 50;
    if (g_fail.load()) {
        cv::putText(cv, "Last fail: " + g_last_fail, {40, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255}, 1, cv::LINE_AA); y += 34;
    }
    if (ph == Phase::DONE && g_cfg.is_master && !g_slave_summary.empty()) {
        cv::putText(cv, "Slave:" + g_slave_summary, {40, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 200}, 1, cv::LINE_AA); y += 30;
    }
    string hint = (ph == Phase::RUNNING) ? "[q] Abort after current file"
                : (ph == Phase::WAIT_SLAVE) ? "[q] Abort wait (slave continues)"
                : "[SPACE/q] Exit";
    cv::putText(cv, hint, {40, 560}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {150, 150, 150}, 1, cv::LINE_AA);
}

// ================== main ==================
int main(int argc, char** argv) {
    WSAData wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
    string data_ip_override, participant_override;
    int port_override = 0, workers_override = 0;
    vector<string> roots_override;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto nextval = [&](string& d) { if (i + 1 < argc) d = argv[++i]; };
        if (a == "--data-ip") nextval(data_ip_override);
        else if (a == "--port") port_override = atoi(argv[++i]);
        else if (a == "--participant") nextval(participant_override);
        else if (a == "--workers") workers_override = atoi(argv[++i]);
        else if (a == "--roots") {
            while (i + 1 < argc && string(argv[i + 1]).rfind("--", 0) != 0)
                roots_override.push_back(argv[++i]);
        }
        else { cerr << "unknown arg " << a << endl; return 1; }
    }

    g_cfg.cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path()
                     .parent_path() / "cfg").string();
    try {
        Cfg cap(g_cfg.cfg_dir + "/capture.yaml");     // 主机角色 + 握手目标
        g_cfg.is_master = cap["capture"]["is_master"].as<bool>();
        g_cfg.master_ip = cap["capture"]["master_ip"].as<string>();
        Cfg xf(g_cfg.cfg_dir + "/transfer.yaml"); auto& t = xf["transfer"];
        g_cfg.participant = participant_override.empty()
                            ? t["participant_id"].as<string>() : participant_override;
        string link = g_cfg.is_master ? t["server_ip_master_link"].as<string>()
                                      : t["server_ip_slave_link"].as<string>();
        g_cfg.server_ip = data_ip_override.empty() ? link : data_ip_override;
        g_cfg.server_port = port_override ? port_override : t["server_port"].as<int>();
        g_cfg.workers = workers_override ? workers_override : t["workers"].as<int>();
        try { g_cfg.handshake_port = t["handshake_port"].as<int>(); } catch (...) {}
    } catch (const exception& e) {
        cerr << ts() << "[Error] config: " << e.what() << endl; return 1;
    }
    if (roots_override.empty()) g_cfg.roots = {"D:/capture", "E:/capture"};
    else g_cfg.roots = roots_override;

    // master 附加源 (xml / IR / map); 缺失则跳过该阶段并告警
    if (g_cfg.is_master) {
        string calib_save, calib_part, day_id;
        try {
            Cfg cc(g_cfg.cfg_dir + "/cam_calib.yaml");
            calib_save = cc["calib"]["calib_save_dir"].as<string>();
            try { calib_part = cc["calib"]["participant_id"].as<string>(); }
            catch (...) { calib_part = "P001"; }
        } catch (...) {}
        try {
            Cfg arm(g_cfg.cfg_dir + "/calib_arm.yaml");
            day_id = arm["record"]["day_id"].as<string>();
        } catch (...) {}
        if (!calib_save.empty())
            g_cfg.xml_dir = (fs::path(calib_save) / calib_part / "output").string();
        if (!day_id.empty())
            g_cfg.ir_file = g_cfg.cfg_dir + "/IR/" + day_id + ".txt";
        g_cfg.map_file = g_cfg.cfg_dir + "/day_participant_map.json";
    }

    // 构建阶段
    addImagePhase(g_cfg.roots[0]);
    if (g_cfg.roots.size() > 1) addImagePhase(g_cfg.roots[1]);
    if (g_cfg.is_master) {
        if (!g_cfg.xml_dir.empty()) addXmlPhase();
        if (!g_cfg.ir_file.empty())
            addFilePhase(g_cfg.ir_file, "calib/IR/" + fs::path(g_cfg.ir_file).filename().generic_string(),
                         "IR positions (calib/IR)");
        if (!g_cfg.map_file.empty())
            addFilePhase(g_cfg.map_file, "day_participant_map.json",
                         "day_participant_map.json");
    }
    if (!g_total_files) {
        cerr << ts() << "[Error] no files found for " << g_cfg.participant << endl; return 1;
    }

    // 握手协作: master 监听等 READY; slave 连上后挂等 START (按键全失效)
    if (g_cfg.is_master) thread(masterHandshakeListen).detach();
    else thread(slaveHandshakeAndRun).detach();

    cv::namedWindow("send_ui", cv::WINDOW_NORMAL);
    cv::resizeWindow("send_ui", 900, 600);
    cv::Mat canvas;
    thread ctl;
    while (true) {
        Phase ph = (Phase)g_phase.load();
        if (ph == Phase::CONFIG) drawConfig(canvas);
        else drawProgress(canvas);
        cv::imshow("send_ui", canvas);
        int key = cv::waitKey(ph == Phase::RUNNING ? 30 : 50);
        if (!g_cfg.is_master) {
            // slave: 按键全失效; 结束态显示 5s 后自动退出
            if (ph == Phase::DONE || ph == Phase::ABORTED) {
                static auto done_at = chrono::steady_clock::now();
                if (chrono::steady_clock::now() - done_at > chrono::seconds(5)) break;
            }
            continue;
        }
        if (ph == Phase::CONFIG) {
            if (key == ' ' && g_slave_ready.load()) {   // 需 slave 已握手
                g_phase.store((int)Phase::RUNNING);
                ctl = thread(transferController);
            } else if (key == 'q' || key == 27) {       // 通知 slave 一并退出
                if (g_hs != INVALID_SOCKET) {
                    sendLine(g_hs, "QUIT");
                    closesocket(g_hs); g_hs = INVALID_SOCKET;
                } else {
                    cout << ts() << "[HS] slave not connected — it will keep waiting" << endl;
                }
                break;
            }
        } else if (ph == Phase::RUNNING) {
            if (key == 'q' || key == 27) g_abort = true;   // 完成后 controller 发 ABORT
        } else if (ph == Phase::WAIT_SLAVE) {
            if ((key == 'q' || key == 27) && g_hs != INVALID_SOCKET) {
                sendLine(g_hs, "QUIT");                    // slave 看门狗捕获后中止
                closesocket(g_hs); g_hs = INVALID_SOCKET;
                break;
            }
        } else {
            if (key >= 0) break;
        }
    }
    g_abort = true;
    if (ctl.joinable()) ctl.join();
    cv::destroyAllWindows();
    WSACleanup();
    return g_fail.load() ? 1 : 0;
}
