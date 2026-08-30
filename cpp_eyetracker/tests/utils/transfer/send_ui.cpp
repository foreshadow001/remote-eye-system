// ================== send_ui ==================
// 交互式串行传输 (Windows, slave): OpenCV UI 确认配置 → SPACE 开始 →
// D:/capture 与 E:/capture 逐盘串行发送到处理主机 (baseline_recv_data)。
// participant 默认读 cfg/transfer.yaml 的 participant_id (可用 --participant 覆盖)。
// 按键: SPACE=开始传输, q/ESC=退出 (传输中 q=在当前文件完成后中止)
// 引擎与 send_slave 相同: TransmitFile 内核态读发重叠, 默认 4 流。
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
    string server_ip;                 // 按 capture.yaml is_master 自动选 master/slave 链路
    int server_port = 5001;
    int workers = 4;
    string participant;
    bool is_master = false;
    vector<string> roots;             // 串行顺序: D:/capture, E:/capture
};
static UiCfg g_cfg;

// ================== 传输状态 ==================
struct Job { fs::path path; string rel; uint64_t size; };
struct DiskPlan { string root; vector<Job> jobs; uint64_t bytes = 0; };
static vector<DiskPlan> g_plans;

enum class Phase { CONFIG, RUNNING, DONE, ABORTED };
static atomic<int> g_phase{(int)Phase::CONFIG};
static atomic<bool> g_abort{false};
static atomic<int> g_done_files{0}, g_skip{0}, g_fail{0};
static atomic<uint64_t> g_bytes{0};
static atomic<int> g_disk_idx{0};                 // 当前盘 (RUNNING)
static uint64_t g_total_bytes = 0;
static int g_total_files = 0;
static atomic<uint64_t> g_t0_us{0}, g_t_end_us{0};
static string g_last_fail;

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
        if (++retry % 10 == 1) cerr << "[Net] connect retry #" << retry << endl;
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

// ================== 传输引擎 (TransmitFile, 同 send_slave) ==================
static int sendOne(SOCKET s, const Job& j) {
    HANDLE hf = CreateFileA(j.path.string().c_str(), GENERIC_READ, FILE_SHARE_READ,
                            NULL, OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, NULL);
    if (hf == INVALID_HANDLE_VALUE) return -1;
    if (!sendLine(s, "FILE " + j.rel + " " + to_string(j.size) + " 0")) { CloseHandle(hf); return -1; }
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
        g_disk_idx = (int)d;
        atomic<size_t> next{0};
        vector<thread> ths;
        for (int i = 0; i < g_cfg.workers; ++i)
            ths.emplace_back(workerLoop, &g_plans[d].jobs, &next);
        for (auto& t : ths) t.join();
    }
    g_t_end_us.store(chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count());
    g_phase.store((int)(g_abort.load() ? Phase::ABORTED : Phase::DONE));
}

// ================== UI ==================
static void drawConfig(cv::Mat& cv) {
    cv = cv::Mat::zeros(560, 860, CV_8UC3);
    int y = 50;
    auto put = [&](const string& s, double sc, cv::Scalar c, int dy = 34) {
        cv::putText(cv, s, {40, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, 1, cv::LINE_AA);
        y += dy;
    };
    put("100G Transfer - Configuration", 0.9, {0, 215, 255}, 50);
    put("Role        : " + string(g_cfg.is_master ? "MASTER" : "SLAVE")
        + "   (from capture.yaml is_master)", 0.6, {255, 255, 255});
    put("Participant : " + g_cfg.participant, 0.6, {255, 255, 255});
    put("Server      : " + g_cfg.server_ip + ":" + to_string(g_cfg.server_port), 0.6, {255, 255, 255});
    put("Streams     : " + to_string(g_cfg.workers) + "  (TransmitFile)", 0.6, {255, 255, 255});
    put("Serial disk plan:", 0.6, {200, 200, 200}, 44);
    for (auto& p : g_plans) {
        ostringstream os;
        os << "  " << p.root << "   " << p.jobs.size() << " files   "
           << fixed << setprecision(2) << (double)p.bytes / 1e12 << " TB";
        put(os.str(), 0.55, {0, 255, 200});
    }
    y += 16;
    ostringstream tot;
    tot << "Total: " << g_total_files << " files, " << fixed << setprecision(2)
        << (double)g_total_bytes / 1e12 << " TB";
    put(tot.str(), 0.6, {0, 215, 255}, 60);
    cv::putText(cv, "[SPACE] Start transfer      [q/ESC] Quit",
                {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2, cv::LINE_AA);
}

static void drawProgress(cv::Mat& cv) {
    cv = cv::Mat::zeros(560, 860, CV_8UC3);
    Phase ph = (Phase)g_phase.load();
    int y = 60;
    cv::putText(cv, ph == Phase::RUNNING ? "Transferring..." :
                (ph == Phase::DONE ? "DONE" : "ABORTED"),
                {40, y}, cv::FONT_HERSHEY_DUPLEX, 0.9, {0, 215, 255}, 2, cv::LINE_AA);
    y += 50;
    // 当前盘
    ostringstream dsk;
    dsk << "Disk " << min(g_disk_idx.load() + 1, (int)g_plans.size()) << "/" << g_plans.size()
        << "  " << g_plans[min((size_t)g_disk_idx.load(), g_plans.size() - 1)].root;
    cv::putText(cv, dsk.str(), {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1, cv::LINE_AA);
    y += 44;
    // 进度条
    double frac = g_total_bytes ? (double)g_bytes.load() / (double)g_total_bytes : 0;
    cv::rectangle(cv, {40, y}, {820, y + 36}, {80, 80, 80}, 1);
    cv::rectangle(cv, {40, y}, {40 + (int)(780 * min(frac, 1.0)), y + 36}, {0, 200, 0}, -1);
    char pct[16]; snprintf(pct, sizeof(pct), "%.1f%%", frac * 100);
    cv::putText(cv, pct, {410, y + 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1, cv::LINE_AA);
    y += 70;
    // 数字
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
    string hint = (ph == Phase::RUNNING) ? "[q] Abort after current file"
                                         : "[SPACE/q] Exit";
    cv::putText(cv, hint, {40, 520}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {150, 150, 150}, 1, cv::LINE_AA);
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

    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path()
                    .parent_path() / "cfg").string();
    try {
        Cfg cap(cfg_dir + "/capture.yaml");    // 主机角色 (每台机器自己的 capture.yaml 声明)
        g_cfg.is_master = cap["capture"]["is_master"].as<bool>();
        Cfg xf(cfg_dir + "/transfer.yaml"); auto& t = xf["transfer"];
        g_cfg.participant = participant_override.empty()
                            ? t["participant_id"].as<string>() : participant_override;
        // master → 口1 (10.10.1.1), slave → 口2 (10.10.2.1); --data-ip 覆盖
        string link = g_cfg.is_master ? t["server_ip_master_link"].as<string>()
                                      : t["server_ip_slave_link"].as<string>();
        g_cfg.server_ip = data_ip_override.empty() ? link : data_ip_override;
        g_cfg.server_port = port_override ? port_override : t["server_port"].as<int>();
        g_cfg.workers = workers_override ? workers_override : t["workers"].as<int>();
    } catch (const exception& e) {
        cerr << "[Error] config: " << e.what() << endl; return 1;
    }
    if (roots_override.empty()) g_cfg.roots = {"D:/capture", "E:/capture"};
    else g_cfg.roots = roots_override;

    // 扫描各盘 (串行顺序)
    for (auto& root : g_cfg.roots) {
        DiskPlan plan; plan.root = root;
        fs::path base = fs::path(root) / g_cfg.participant;
        if (fs::exists(base)) {
            for (auto& e : fs::recursive_directory_iterator(base)) {
                if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
                string rel = g_cfg.participant + "/" + e.path().parent_path().filename().string()
                           + "/" + e.path().filename().string();
                uint64_t sz = (uint64_t)e.file_size();
                plan.jobs.push_back({e.path(), rel, sz});
                plan.bytes += sz;
            }
        }
        sort(plan.jobs.begin(), plan.jobs.end(),
             [](const Job& a, const Job& b) { return a.rel < b.rel; });
        g_total_files += (int)plan.jobs.size();
        g_total_bytes += plan.bytes;
        g_plans.push_back(move(plan));
    }
    if (!g_total_files) {
        cerr << "[Error] no h5 found for " << g_cfg.participant
             << " under roots" << endl; return 1;
    }

    cv::namedWindow("send_ui", cv::WINDOW_NORMAL);
    cv::resizeWindow("send_ui", 860, 560);
    cv::Mat canvas;
    thread ctl;
    while (true) {
        Phase ph = (Phase)g_phase.load();
        if (ph == Phase::CONFIG) drawConfig(canvas);
        else drawProgress(canvas);
        cv::imshow("send_ui", canvas);
        int key = cv::waitKey(ph == Phase::RUNNING ? 30 : 50);
        if (ph == Phase::CONFIG) {
            if (key == ' ') {
                g_phase.store((int)Phase::RUNNING);
                ctl = thread(transferController);
            } else if (key == 'q' || key == 27) break;
        } else if (ph == Phase::RUNNING) {
            if (key == 'q' || key == 27) g_abort = true;   // 文件粒度温和中止
        } else {                                            // DONE / ABORTED
            if (key >= 0) break;
        }
    }
    g_abort = true;
    if (ctl.joinable()) ctl.join();
    cv::destroyAllWindows();
    WSACleanup();
    return g_fail.load() ? 1 : 0;
}
