// ================== send_slave ==================
// slave (Windows) 独立发送器 — 100G 直连传给数据处理主机 (4090, recv_data.py)。
// 阶段一版本: 不做 master 握手 (后续并入 send_data 的串行流程)。
// 协议: "FILE <rel> <size> 0\n" + <size 字节> → 应答 "OK <n>"/"SKIP"/"ERR ..."
// SKIP = 对端已有同大小文件 (断点续传)。失败重试 3 次。
// 配置: cfg/capture.yaml (participant/participant_root) + cfg/transfer.yaml (slave 链路)
// CLI 覆盖: --data-ip <ip> --port <p> --participant <id> --roots <dir>... --workers <n>
//           --cams <SN> [<SN> ...] (只传指定相机目录, 不给则传 participant 下全部)
// =================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX                  // 防 windows.h 的 min/max 宏破坏 std::min
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #include <mswsock.h>              // TransmitFile: 内核级 读盘+发送 重叠
    #pragma comment(lib, "ws2_32.lib")
    #pragma comment(lib, "Mswsock.lib")
#else
    #error "sender is Windows-only (collection hosts)"
#endif

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
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

// ================== 配置 ==================
static string g_server_ip, g_participant;
static int g_server_port = 5001, g_workers = 4;
static vector<string> g_roots;
static vector<string> g_cams;              // 空 = 全部相机

static const int CHUNK = 8 * 1024 * 1024;     // 8MB 流式块 (与 C++ 接收端一致)
static const int SOCK_BUF = 32 * 1024 * 1024;
static const int RETRIES = 3;
static const int REPLY_TIMEOUT_MS = 60000;

// ================== TCP 基础 ==================
bool sendLine(SOCKET s, const string& msg) {
    string d = msg + "\n";
    return send(s, d.data(), (int)d.size(), 0) == (int)d.size();
}

bool recvLine(SOCKET s, string& line, int timeout_ms) {
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

// 连接 (无限重试, 与项目内 connectArm 风格一致)
SOCKET connectTo(const string& ip, int port) {
    int retry = 0;
    while (true) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        sockaddr_in sa{};
        sa.sin_family = AF_INET; sa.sin_port = htons((u_short)port);
        inet_pton(AF_INET, ip.c_str(), &sa.sin_addr);
        if (connect(s, (sockaddr*)&sa, sizeof(sa)) == 0) return s;
        closesocket(s);
        if (++retry % 10 == 1)
            cerr << "[Net] connect " << ip << ":" << port << " retry #" << retry << endl;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
}

// ================== 文件清单与进度 ==================
struct Job { fs::path path; string rel; uint64_t size; };

vector<Job> scanFiles() {
    vector<Job> jobs;
    auto cam_wanted = [&](const string& sn) {
        return g_cams.empty() || find(g_cams.begin(), g_cams.end(), sn) != g_cams.end();
    };
    for (auto& root : g_roots) {
        fs::path base = fs::path(root) / g_participant;
        if (!fs::exists(base)) { cout << "[Scan] skip missing " << base.string() << endl; continue; }
        for (auto& e : fs::recursive_directory_iterator(base)) {
            if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
            string sn = e.path().parent_path().filename().string();
            if (!cam_wanted(sn)) continue;
            string rel = g_participant + "/" + sn + "/" + e.path().filename().string();
            jobs.push_back({e.path(), rel, (uint64_t)e.file_size()});
        }
    }
    return jobs;
}

struct Progress {
    atomic<int> done{0}, skip{0}, fail{0};
    atomic<uint64_t> bytes{0};
    uint64_t total_bytes = 0; int total_files = 0;
    chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
    mutex print_mtx;
    void line() {
        double el = chrono::duration<double>(chrono::steady_clock::now() - t0).count();
        double rate = bytes.load() / max(el, 1e-6) / 1e9;
        double eta = rate > 0.01 ? (double)(total_bytes - bytes.load()) / (rate * 1e9) : 0;
        lock_guard<mutex> lk(print_mtx);
        printf("\r%d/%d files  %.2f/%.2f TB  %.1f GB/s  ETA %.1f min  (skip %d, fail %d)",
               done.load(), total_files, (double)bytes.load() / 1e12, (double)total_bytes / 1e12,
               rate, eta / 60.0, skip.load(), fail.load());
        fflush(stdout);
    }
} g_prog;

// ================== 数据通道 (与 recv_data 协议对应) ==================
// TransmitFile: 内核级 读盘+发送 重叠 (用户态串行 read+send 会把单流压到
// ~350MB/s; 内核重叠后单流逼近线速), 每次调用上限 DWORD, 大文件分段
// 返回 0=OK 1=SKIP -1=失败
int sendOne(SOCKET s, const Job& j) {
    HANDLE hf = CreateFileA(j.path.string().c_str(), GENERIC_READ, FILE_SHARE_READ,
                            NULL, OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, NULL);
    if (hf == INVALID_HANDLE_VALUE) return -1;
    if (!sendLine(s, "FILE " + j.rel + " " + to_string(j.size) + " 0")) { CloseHandle(hf); return -1; }
    uint64_t remaining = j.size;
    while (remaining > 0) {
        DWORD chunk = (DWORD)min<uint64_t>(remaining, 1ull << 30);   // 1GB/段
        if (!TransmitFile(s, hf, chunk, 0, NULL, NULL, 0)) { CloseHandle(hf); return -1; }
        remaining -= chunk;                                           // 文件指针随发送推进
    }
    CloseHandle(hf);
    string reply;
    if (!recvLine(s, reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("OK", 0) == 0) return 0;
    if (reply.rfind("SKIP", 0) == 0) return 1;
    cerr << "\n[Recv] " << j.rel << ": " << reply << endl;
    return -1;
}

void workerMain(const vector<Job>* jobs, atomic<size_t>* next) {
    while (true) {
        size_t i = next->fetch_add(1);
        if (i >= jobs->size()) return;
        const Job& j = (*jobs)[i];
        for (int attempt = 1; attempt <= RETRIES; ++attempt) {
            SOCKET s = connectTo(g_server_ip, g_server_port);
            int one = 1, buf = SOCK_BUF;
            setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
            setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char*)&buf, sizeof(buf));
            int r = sendOne(s, j);
            closesocket(s);
            if (r == 0) { g_prog.done++; g_prog.bytes += j.size; break; }
            if (r == 1) { g_prog.done++; g_prog.skip++; break; }
            if (attempt == RETRIES) {
                g_prog.done++; g_prog.fail++;
                cerr << "\n[Fail] " << j.rel << endl;
            } else {
                this_thread::sleep_for(chrono::seconds(1));
            }
        }
        g_prog.line();
    }
}

// ================== main ==================
int main(int argc, char** argv) {
    WSAData wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
    string data_ip_override, participant_override;
    int port_override = 0, workers_override = 0;
    vector<string> roots_override;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto nextval = [&](string& dst) { if (i + 1 < argc) dst = argv[++i]; };
        if (a == "--data-ip") nextval(data_ip_override);
        else if (a == "--port") port_override = atoi(argv[++i]);
        else if (a == "--participant") nextval(participant_override);
        else if (a == "--workers") workers_override = atoi(argv[++i]);
        else if (a == "--roots") {
            while (i + 1 < argc && string(argv[i + 1]).rfind("--", 0) != 0)
                roots_override.push_back(argv[++i]);
        }
        else if (a == "--cams") {
            while (i + 1 < argc && string(argv[i + 1]).rfind("--", 0) != 0)
                g_cams.push_back(argv[++i]);
        }
        else { cerr << "unknown arg " << a << endl; return 1; }
    }

    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path()
                    .parent_path() / "cfg").string();
    try {
        Cfg cap(cfg_dir + "/capture.yaml"); auto& c = cap["capture"];
        g_participant = participant_override.empty()
                        ? c["participant_id"].as<string>() : participant_override;
        if (roots_override.empty()) g_roots = c["participant_root"].as<vector<string>>();
        else g_roots = roots_override;
        Cfg xf(cfg_dir + "/transfer.yaml"); auto& t = xf["transfer"];
        g_server_ip = data_ip_override.empty() ? t["server_ip_slave_link"].as<string>()
                                               : data_ip_override;
        g_server_port = port_override ? port_override : t["server_port"].as<int>();
        g_workers = workers_override ? workers_override : t["workers"].as<int>();
    } catch (const exception& e) {
        cerr << "[Error] config: " << e.what() << endl; return 1;
    }

    vector<Job> jobs = scanFiles();
    if (jobs.empty()) {
        cerr << "[Error] no h5 under roots for " << g_participant << endl;
        return 1;
    }
    g_prog.total_files = (int)jobs.size();
    for (auto& j : jobs) g_prog.total_bytes += j.size;
    cout << "=== send_slave: " << jobs.size() << " files, " << fixed << setprecision(2)
         << g_prog.total_bytes / 1e12 << " TB -> " << g_server_ip << ":" << g_server_port
         << " (" << g_workers << " streams) ===" << endl;

    atomic<size_t> next{0};
    vector<thread> ths;
    for (int i = 0; i < g_workers; ++i) ths.emplace_back(workerMain, &jobs, &next);
    for (auto& t : ths) t.join();
    g_prog.line(); cout << endl;
    double el = chrono::duration<double>(chrono::steady_clock::now() - g_prog.t0).count();
    cout << "=== done in " << fixed << setprecision(1) << el / 60 << " min: ok="
         << g_prog.done - g_prog.skip - g_prog.fail << " skip=" << g_prog.skip
         << " fail=" << g_prog.fail << " ===" << endl;
    WSACleanup();
    return g_prog.fail.load() ? 1 : 0;
}
