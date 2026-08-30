// ================== send_v2 ==================
// V2 管道发送端 (Windows): 1 顺序读线程 → 16 槽×8MB 环队列 → K 个 send() 发送线程。
// 与 recv_v2.cpp (Ubuntu) 配对, 协议见 plan/transfer_seq_design.md §二。
// K 默认 8: 用户态 send() 单流硬顶 ~0.75-0.8 GB/s, 8×0.75≈6.0 盖住写盘上限。
// CLI 与 send_slave 一致, 另有 --streams K。
// =================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #error "sender is Windows-only (collection hosts)"
#endif

#include <atomic>
#include <chrono>
#include <condition_variable>
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

static string g_server_ip, g_participant;
static int g_server_port = 5001, g_streams = 8;
static vector<string> g_roots;
static vector<string> g_cams;

static const int    CHUNK = 8 * 1024 * 1024;
static const int    SLOTS = 16;                 // 环队列槽 (128MB 在途上限)
static const int    SOCK_BUF = 16 * 1024 * 1024;
static const int    RETRIES = 3;
static const int    REPLY_TIMEOUT_MS = 30000;   // 容错: 会话/应答 30s

// ================== TCP 基础 ==================
static bool sendLine(SOCKET s, const string& msg) {
    string d = msg + "\n";
    return send(s, d.data(), (int)d.size(), 0) == (int)d.size();
}
static bool sendAll(SOCKET s, const char* p, int n) {
    int off = 0;
    while (off < n) {
        int r = send(s, p + off, n - off, 0);
        if (r <= 0) return false;
        off += r;
    }
    return true;
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

// ================== 文件清单与进度 ==================
struct Job { fs::path path; string rel; uint64_t size; };

static vector<Job> scanFiles() {
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
            jobs.push_back({e.path(), g_participant + "/" + sn + "/" + e.path().filename().string(),
                            (uint64_t)e.file_size()});
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

// ================== 环队列 (读线程生产, 发送线程消费) ==================
struct Ring {
    struct Slot { vector<char> buf; uint64_t len = 0, seq = 0; atomic<int> st{0}; };  // 0空 1满
    vector<Slot> slots;
    mutex mtx;
    condition_variable cv_empty, cv_full;
    atomic<bool> read_done{false};
    atomic<uint64_t> next_push{0};
    Ring() : slots(SLOTS) { for (auto& s : slots) s.buf.resize(CHUNK); }
};

// ================== 单文件会话 ==================
struct Session {
    // 发送端: 读线程 + K 发送线程共享 ring; 主线程控制文件顺序
    Ring ring;
    atomic<int> senders_idle{0};
    atomic<bool> aborted{false};
};

static void readerThread(Session& sess, const Job& j) {
    ifstream in(j.path, ios::binary);
    if (!in) { sess.aborted = true; sess.ring.read_done = true; sess.ring.cv_full.notify_all(); return; }
    uint64_t seq = 0, total = 0;
    while (total < j.size) {
        Ring::Slot* slot = nullptr;
        // 找空槽 (轮询+等待)
        {
            unique_lock<mutex> lk(sess.ring.mtx);
            sess.ring.cv_empty.wait(lk, [&] {
                for (auto& s : sess.ring.slots) if (s.st.load() == 0) return true;
                return sess.ring.read_done.load();
            });
            if (sess.ring.read_done.load()) return;
            for (auto& s : sess.ring.slots)
                if (s.st.load() == 0) { slot = &s; break; }
            if (!slot) continue;
            // 标记为填充中: 先置 -1 防止发送线程误取
            slot->st.store(-1);
        }
        int want = (int)min<uint64_t>(CHUNK, j.size - total);
        in.read(slot->buf.data(), want);
        int got = (int)in.gcount();
        if (got <= 0) {
            lock_guard<mutex> lk(sess.ring.mtx);
            slot->st.store(0);
            sess.aborted = true;
            sess.ring.read_done = true;
            sess.ring.cv_full.notify_all();
            return;
        }
        slot->len = (uint64_t)got;
        slot->seq = seq++;
        total += (uint64_t)got;
        slot->st.store(1);                       // 发布
        sess.ring.cv_full.notify_one();
    }
    lock_guard<mutex> lk(sess.ring.mtx);
    sess.ring.read_done = true;
    sess.ring.cv_full.notify_all();
}

static void senderThread(Session& sess, SOCKET sock, uint64_t file_size, int my_id) {
    while (true) {
        Ring::Slot* slot = nullptr;
        {
            unique_lock<mutex> lk(sess.ring.mtx);
            sess.ring.cv_full.wait(lk, [&] {
                for (auto& s : sess.ring.slots) if (s.st.load() == 1) return true;
                return sess.ring.read_done.load();
            });
            if (sess.aborted.load()) return;
            for (auto& s : sess.ring.slots)
                if (s.st.load() == 1) { slot = &s; break; }
            if (!slot) {
                if (sess.ring.read_done.load()) return;
                continue;
            }
            slot->st.store(-1);                  // 认领
        }
        // 发 DATA 头 + 载荷
        char hdr[64];
        int hn = snprintf(hdr, sizeof(hdr), "DATA %llu %llu\n",
                          (unsigned long long)slot->seq, (unsigned long long)slot->len);
        bool ok = sendAll(sock, hdr, hn) && sendAll(sock, slot->buf.data(), (int)slot->len);
        {
            lock_guard<mutex> lk(sess.ring.mtx);
            slot->st.store(0);                   // 归还槽位
        }
        sess.ring.cv_empty.notify_one();
        if (!ok) { sess.aborted = true; sess.ring.read_done = true;
                   sess.ring.cv_full.notify_all(); sess.ring.cv_empty.notify_all(); return; }
        g_prog.bytes += slot->len;
        g_prog.line();
    }
}

// ================== 单文件传输 (返回 0=OK 1=SKIP -1=FAIL) ==================
static int sendFile(vector<SOCKET>& socks, int sid, const Job& j) {
    // 1) FILE 握手 (conn0)
    if (!sendLine(socks[0], "FILE " + to_string(sid) + " " + j.rel + " " +
                            to_string(j.size) + " " + to_string((int)socks.size())))
        return -1;
    string reply;
    if (!recvLine(socks[0], reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("SKIP", 0) == 0) return 1;
    if (reply.rfind("GO", 0) != 0) { cerr << "\n[Recv] " << j.rel << ": " << reply << endl; return -1; }

    // 2) JOIN 其余连接 (等待确认, 保证 DATA 到达时会话已完整)
    for (size_t i = 1; i < socks.size(); ++i) {
        if (!sendLine(socks[i], "JOIN " + to_string(sid))) return -1;
        string jr;
        if (!recvLine(socks[i], jr, REPLY_TIMEOUT_MS) || jr != "JOINED") return -1;
    }

    // 3) 读线程 + 发送线程
    Session sess;
    thread rd(readerThread, ref(sess), cref(j));
    vector<thread> snds;
    for (size_t i = 0; i < socks.size(); ++i)
        snds.emplace_back(senderThread, ref(sess), socks[i], j.size, (int)i);
    rd.join();
    for (auto& t : snds) t.join();
    if (sess.aborted.load()) return -1;

    // 4) END + 等终应答
    if (!sendLine(socks[0], "END " + to_string(sid))) return -1;
    if (!recvLine(socks[0], reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("OK", 0) == 0) return 0;
    cerr << "\n[Recv] " << j.rel << ": " << reply << endl;
    return -1;
}

// ================== main ==================
int main(int argc, char** argv) {
    WSAData wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
    string data_ip_override, participant_override;
    int port_override = 0, streams_override = 0;
    vector<string> roots_override;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto nextval = [&](string& d) { if (i + 1 < argc) d = argv[++i]; };
        if (a == "--data-ip") nextval(data_ip_override);
        else if (a == "--port") port_override = atoi(argv[++i]);
        else if (a == "--participant") nextval(participant_override);
        else if (a == "--streams") streams_override = atoi(argv[++i]);
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
        g_streams = streams_override ? streams_override : t["workers"].as<int>() * 2;  // V2 默认 8
    } catch (const exception& e) {
        cerr << "[Error] config: " << e.what() << endl; return 1;
    }

    vector<Job> jobs = scanFiles();
    if (jobs.empty()) { cerr << "[Error] no h5 under roots for " << g_participant << endl; return 1; }
    g_prog.total_files = (int)jobs.size();
    for (auto& j : jobs) g_prog.total_bytes += j.size;
    cout << "=== send_v2: " << jobs.size() << " files, " << fixed << setprecision(2)
         << g_prog.total_bytes / 1e12 << " TB -> " << g_server_ip << ":" << g_server_port
         << " (" << g_streams << " streams, 1 sequential reader) ===" << endl;

    // K 条持久连接
    vector<SOCKET> socks;
    for (int i = 0; i < g_streams; ++i) {
        SOCKET s = connectTo(g_server_ip, g_server_port);
        int one = 1, buf = SOCK_BUF;
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char*)&buf, sizeof(buf));
        socks.push_back(s);
    }

    int sid = 0, fails = 0;
    for (auto& j : jobs) {
        int r = -1;
        for (int attempt = 1; attempt <= RETRIES; ++attempt) {
            r = sendFile(socks, sid, j);
            if (r >= 0) break;
            if (attempt == RETRIES) break;
            this_thread::sleep_for(chrono::seconds(1));
            // 失败重连 (连接可能已坏)
            for (auto& s : socks) { closesocket(s); }
            for (auto& s : socks) {
                s = connectTo(g_server_ip, g_server_port);
                int one = 1, buf = SOCK_BUF;
                setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
                setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char*)&buf, sizeof(buf));
            }
        }
        ++sid;
        g_prog.done++;
        if (r == 0) { /* ok */ }
        else if (r == 1) g_prog.skip++;
        else { g_prog.fail++; fails++; cerr << "\n[Fail] " << j.rel << endl; }
        g_prog.line();
    }
    for (auto& s : socks) { sendLine(s, "BYE"); closesocket(s); }
    g_prog.line(); cout << endl;
    double el = chrono::duration<double>(chrono::steady_clock::now() - g_prog.t0).count();
    cout << "=== done in " << fixed << setprecision(1) << el / 60 << " min: ok="
         << g_prog.done - g_prog.skip - g_prog.fail << " skip=" << g_prog.skip
         << " fail=" << g_prog.fail << " ===" << endl;
    WSACleanup();
    return fails ? 1 : 0;
}
