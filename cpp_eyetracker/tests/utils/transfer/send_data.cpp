// ================== send_data ==================
// 采集端 (Windows, master/slave 同一程序) — 100G 直连发送器, 与处理主机
// recv_data.py (Ubuntu) 配对。串行流程 (更可靠):
//   1. slave → master 握手 (现有 192.168.10.x 网): "READY <n> <bytes>"
//   2. master → "START" → slave 先传 (处理主机口2)
//   3. slave 完成 → "SLAVE_DONE <ok> <skip> <fail>" → master "ACK"
//   4. master 再传 (处理主机口1)
// 数据协议: "FILE <rel> <size> 0\n" + <size 字节> → 应答 "OK <n>"/"SKIP"/"ERR ..."
// SKIP = 对端已有同大小文件 (断点续传)。sha256 关闭 (大小校验+抽查兜底)。
// 配置: cfg/transfer.yaml (链路) + cfg/capture.yaml (participant/is_master/master_ip)
// 回环测试覆盖: --role master|slave --handshake-ip <ip> --data-ip <ip> --port <p>
//               --roots <dir> [<dir> ...] (覆盖 capture.yaml 的 participant_root, 止于下一 -- 参数)
// =================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX                  // 防 windows.h 的 min/max 宏破坏 std::min
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
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

#include "cfg/config.hpp"

using namespace std;
namespace fs = std::filesystem;
using namespace gazeestimation;

// ================== 配置 ==================
struct XferCfg {
    string server_ip_master_link, server_ip_slave_link;
    int server_port = 5001;
    int handshake_port = 50100;
    int workers = 4;
    bool is_master = false;
    string master_ip;                  // capture.yaml (192.168.10.x, 握手用)
    string participant;
    vector<string> roots;
};
XferCfg g_cfg;

static const int    CHUNK = 4 * 1024 * 1024;
static const int    SOCK_BUF = 16 * 1024 * 1024;
static const int    RETRIES = 3;
static const int    REPLY_TIMEOUT_MS = 60000;

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
    for (auto& root : g_cfg.roots) {
        fs::path base = fs::path(root) / g_cfg.participant;
        if (!fs::exists(base)) { cout << "[Scan] skip missing " << base.string() << endl; continue; }
        for (auto& e : fs::recursive_directory_iterator(base)) {
            if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
            string rel = g_cfg.participant + "/" + e.path().parent_path().filename().string()
                       + "/" + e.path().filename().string();
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

// ================== 数据通道 (与 recv_data.py 协议对应) ==================
// 返回 0=OK 1=SKIP -1=失败
int sendOne(SOCKET s, const Job& j) {
    ifstream in(j.path, ios::binary);
    if (!in) return -1;
    string hdr = "FILE " + j.rel + " " + to_string(j.size) + " 0\n";
    if (!sendLine(s, hdr.substr(0, hdr.size() - 1))) return -1;   // sendLine 自带 \n
    vector<char> buf(CHUNK);
    uint64_t sent = 0;
    while (sent < j.size) {
        int want = (int)min<uint64_t>(CHUNK, j.size - sent);
        in.read(buf.data(), want);
        int got = (int)in.gcount();
        if (got <= 0) return -1;
        int off = 0;
        while (off < got) {
            int n = send(s, buf.data() + off, got - off, 0);
            if (n <= 0) return -1;
            off += n;
        }
        sent += got;
    }
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
            SOCKET s = connectTo(g_cfg.is_master ? g_cfg.server_ip_master_link
                                                 : g_cfg.server_ip_slave_link,
                                 g_cfg.server_port);
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

// 传输本机全部文件; 返回失败数
int runDataTransfer() {
    vector<Job> jobs = scanFiles();
    if (jobs.empty()) {
        cerr << "[Error] no h5 under roots for " << g_cfg.participant << endl;
        return 1;
    }
    g_prog.total_files = (int)jobs.size();
    for (auto& j : jobs) g_prog.total_bytes += j.size;
    cout << "[Xfer] " << (g_cfg.is_master ? "MASTER" : "SLAVE") << ": "
         << jobs.size() << " files, " << fixed << setprecision(2)
         << g_prog.total_bytes / 1e12 << " TB -> "
         << (g_cfg.is_master ? g_cfg.server_ip_master_link : g_cfg.server_ip_slave_link)
         << ":" << g_cfg.server_port << " (" << g_cfg.workers << " streams)" << endl;

    atomic<size_t> next{0};
    vector<thread> ths;
    for (int i = 0; i < g_cfg.workers; ++i) ths.emplace_back(workerMain, &jobs, &next);
    for (auto& t : ths) t.join();
    g_prog.line(); cout << endl;
    double el = chrono::duration<double>(chrono::steady_clock::now() - g_prog.t0).count();
    cout << "[Xfer] done in " << fixed << setprecision(1) << el / 60 << " min: ok="
         << g_prog.done - g_prog.skip - g_prog.fail << " skip=" << g_prog.skip
         << " fail=" << g_prog.fail << endl;
    return g_prog.fail.load();
}

// ================== master↔slave 握手 ==================
// master: 等 slave READY → START → 等 SLAVE_DONE → ACK → 自己传
int runMaster(const string& listen_ip) {
    SOCKET lst = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int opt = 1;
    setsockopt(lst, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
    sockaddr_in sa{};
    sa.sin_family = AF_INET; sa.sin_port = htons((u_short)g_cfg.handshake_port);
    inet_pton(AF_INET, listen_ip.c_str(), &sa.sin_addr);
    if (::bind(lst, (sockaddr*)&sa, sizeof(sa)) != 0) {   // ::bind 防 std::bind 名字冲突
        // 绑定指定 IP 失败 (如回环测试) → 退化为 0.0.0.0
        sa.sin_addr.s_addr = INADDR_ANY;
        if (::bind(lst, (sockaddr*)&sa, sizeof(sa)) != 0) {
            cerr << "[HS] bind failed: " << WSAGetLastError() << endl;
            return 1;
        }
    }
    listen(lst, 1);
    cout << "[HS] Master waiting for slave handshake on " << listen_ip
         << ":" << g_cfg.handshake_port << " ..." << endl;
    SOCKET s = accept(lst, nullptr, nullptr);
    closesocket(lst);
    if (s == INVALID_SOCKET) { cerr << "[HS] accept failed" << endl; return 1; }

    string line;                                  // 1) READY
    if (!recvLine(s, line, 600000) || line.rfind("READY", 0) != 0) {
        cerr << "[HS] bad READY: " << line << endl; closesocket(s); return 1;
    }
    cout << "[HS] Slave: " << line << " — ordering slave to transfer first." << endl;
    sendLine(s, "START");                         // 2) START

    int slave_fail = -1;                          // 4) SLAVE_DONE (slave 传完才回)
    while (true) {
        if (!recvLine(s, line, 3600000)) {        // 最长 1h
            cerr << "[HS] slave connection lost before SLAVE_DONE" << endl;
            closesocket(s); return 1;
        }
        if (line.rfind("SLAVE_DONE", 0) == 0) {
            istringstream iss(line.substr(10));
            int ok = 0, skip = 0, fail = 0;
            iss >> ok >> skip >> fail;
            cout << "\n[HS] Slave done: ok=" << ok << " skip=" << skip << " fail=" << fail << endl;
            slave_fail = fail;
            break;
        }
    }
    sendLine(s, "ACK");                           // 5) ACK
    closesocket(s);
    if (slave_fail > 0) {
        cerr << "[HS] slave had " << slave_fail << " failures — master NOT transferring. "
             << "Re-run after fixing slave." << endl;
        return 1;
    }
    cout << "[HS] Master transferring now..." << endl;   // 6)
    return runDataTransfer();
}

// slave: 连 master → READY → 等 START → 传输 → SLAVE_DONE → 等 ACK
int runSlave(const string& master_ip) {
    cout << "[HS] Slave connecting to master " << master_ip << ":" << g_cfg.handshake_port << " ..." << endl;
    SOCKET s = connectTo(master_ip, g_cfg.handshake_port);
    {   // 1) READY
        vector<Job> jobs = scanFiles();
        uint64_t tot = 0; for (auto& j : jobs) tot += j.size;
        sendLine(s, "READY " + to_string(jobs.size()) + " " + to_string(tot));
    }
    string line;                                  // 2) START
    if (!recvLine(s, line, 600000) || line != "START") {
        cerr << "[HS] bad START: " << line << endl; closesocket(s); return 1;
    }
    cout << "[HS] Master ordered START — slave transferring first." << endl;
    int fail = runDataTransfer();                 // 3)
    sendLine(s, "SLAVE_DONE " + to_string(g_prog.done - g_prog.skip - g_prog.fail)
              + " " + to_string(g_prog.skip.load()) + " " + to_string(fail));   // 4)
    recvLine(s, line, 60000);                     // 5) ACK
    closesocket(s);
    return fail;
}

// ================== main ==================
int main(int argc, char** argv) {
    WSAData wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
    string role_override, hs_ip_override, data_ip_override;
    int port_override = 0;
    vector<string> roots_override;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto nextval = [&](string& dst) { if (i + 1 < argc) dst = argv[++i]; };
        if (a == "--role") nextval(role_override);
        else if (a == "--handshake-ip") nextval(hs_ip_override);
        else if (a == "--data-ip") nextval(data_ip_override);
        else if (a == "--port") port_override = atoi(argv[++i]);
        else if (a == "--roots") {                 // 收多个值, 直到下一个 -- 参数
            while (i + 1 < argc && string(argv[i + 1]).rfind("--", 0) != 0)
                roots_override.push_back(argv[++i]);
        }
        else { cerr << "unknown arg " << a << endl; return 1; }
    }

    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path()
                    .parent_path() / "cfg").string();
    try {
        Cfg cap(cfg_dir + "/capture.yaml"); auto& c = cap["capture"];
        g_cfg.is_master = c["is_master"].as<bool>();
        g_cfg.master_ip = c["master_ip"].as<string>();
        g_cfg.participant = c["participant_id"].as<string>();
        g_cfg.roots = c["participant_root"].as<vector<string>>();
        Cfg xf(cfg_dir + "/transfer.yaml"); auto& t = xf["transfer"];
        g_cfg.server_ip_master_link = t["server_ip_master_link"].as<string>();
        g_cfg.server_ip_slave_link = t["server_ip_slave_link"].as<string>();
        g_cfg.server_port = t["server_port"].as<int>();
        g_cfg.handshake_port = t["handshake_port"].as<int>();
        g_cfg.workers = t["workers"].as<int>();
    } catch (const exception& e) {
        cerr << "[Error] config: " << e.what() << endl; return 1;
    }
    if (role_override == "master") g_cfg.is_master = true;
    else if (role_override == "slave") g_cfg.is_master = false;
    if (!roots_override.empty()) g_cfg.roots = roots_override;
    if (port_override) g_cfg.server_port = port_override;
    if (data_ip_override.size()) {   // 回环测试: 两口都指测试 IP
        g_cfg.server_ip_master_link = data_ip_override;
        g_cfg.server_ip_slave_link = data_ip_override;
    }

    cout << "=== send_data: " << (g_cfg.is_master ? "MASTER" : "SLAVE")
         << "  participant=" << g_cfg.participant << "  server="
         << (g_cfg.is_master ? g_cfg.server_ip_master_link : g_cfg.server_ip_slave_link)
         << ":" << g_cfg.server_port << " ===" << endl;

    int rc;
    if (g_cfg.is_master) {
        rc = runMaster(hs_ip_override.empty() ? g_cfg.master_ip : hs_ip_override);
    } else {
        rc = runSlave(hs_ip_override.empty() ? g_cfg.master_ip : hs_ip_override);
    }
    WSACleanup();
    return rc;
}
