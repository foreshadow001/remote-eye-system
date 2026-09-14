// ================== send_slave ==================
// 采集机 (Windows) 独立发送器 — 100G 直连传给数据处理主机 (4090, baseline_recv_data)。
// 无握手依赖, master/slave 均可用 (角色由 capture.yaml is_master 决定链路与附件):
//   slave: 相机 h5 → 10.10.2.1 (slave 链路, direct 模式直写归档柜)
//   master: 相机 h5 + 标定附件 (xml/IR/arm_pose/map.json, FORCE 覆盖) → 10.10.1.1
// 协议: "FILE|FORCE <rel> <size> 0\n" + <size 字节> → 应答 "OK"/"SKIP"/"ERR ..."
// FILE = 大小一致 SKIP (断点续传); FORCE = 覆盖重写 (附件)。失败重试 3 次。
// 配置: cfg/capture.yaml (participant/roots/is_master) + cfg/transfer.yaml (链路/流数)
//       master 附件另读 cam_calib.yaml (calib_save_dir) + calib_arm.yaml (day_id)
// CLI 覆盖: --data-ip <ip> --port <p> --participant <id> --roots <dir>... --workers <n>
//           --cams <SN> [<SN> ...] (只传指定相机目录, 附件不受此过滤)
//           --tx transmitfile|user (默认 user: 边读边算 64MB 块 CRC32,
//               数据后发 "CRC32 c1 c2 ..." 行供接收端逐块校验, 不符自动重传;
//               transmitfile = 内核态对照路径, 无内容校验)
// =================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX                  // 防 windows.h 的 min/max 宏破坏 std::min
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #include <mswsock.h>              // TransmitFile: 内核级 读盘+发送 重叠
    #include <intrin.h>               // _mm_crc32_u64 (SSE4.2 硬件 CRC32C)
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
static vector<string> g_cam_ids;           // capture.yaml cam_indices (与 root 一一配对)
static vector<string> g_cams;              // 空 = 全部相机
static bool g_tx_user = true;              // 默认用户态路径: 唯一能边读边算 CRC 的路径
                                           // (--tx transmitfile = 无校验对照)
static bool g_is_master = false;           // capture.yaml is_master → 附加标定/arm_pose

// ---- CRC32C 硬件校验 (SSE4.2, 与 preprocess-server/transfer/baseline_recv_data.cpp
// 保持同步) — 64MB 块校验。查表版实测 0.43GB/s/核 (串行依赖链, 聚合被压到
// 1.4 GB/s), 硬件版 7GB/s/核 (对照 crc_bench.c)。初值直传累计, 无 pre/post xor。
static const size_t CRC_BLK = 64ull << 20;
static uint32_t crc32c_hw(uint32_t crc, const void* p, size_t n) {
    const uint8_t* b = (const uint8_t*)p;
    while (n >= 8) { crc = (uint32_t)_mm_crc32_u64(crc, *(const uint64_t*)b); b += 8; n -= 8; }
    if (n >= 4) { crc = _mm_crc32_u32(crc, *(const uint32_t*)b); b += 4; n -= 4; }
    while (n--) crc = _mm_crc32_u8(crc, *b++);
    return crc;
}

static const int CHUNK = 8 * 1024 * 1024;     // 8MB 流式块 (与 C++ 接收端一致)
static const int SOCK_BUF = 32 * 1024 * 1024;
static const int RETRIES = 3;
static const int REPLY_TIMEOUT_MS = 180000;   // direct 接收端 OK=整文件写完 (~10GB@0.3GB/s≈33s), 3 倍余量防超时重传并发写 .part

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
struct Job { fs::path path; string rel; uint64_t size; bool force = false; };

vector<Job> scanFiles() {
    vector<Job> jobs;
    auto cam_wanted = [&](const string& sn) {
        return g_cams.empty() || find(g_cams.begin(), g_cams.end(), sn) != g_cams.end();
    };
    // capture.yaml 语义: participant_root[i] ↔ cam_indices[i] 一一配对 (loader 同口径)。
    // root 列表含重复盘符 (5×D + 5×E), 整目录扫会 ×5 重复入队 → 总量虚标/重复传输
    if (!g_cam_ids.empty()) {
        for (size_t i = 0; i < g_cam_ids.size(); ++i) {
            const string& sn = g_cam_ids[i];
            if (!cam_wanted(sn)) continue;
            fs::path dir = fs::path(g_roots[i % g_roots.size()]) / g_participant / sn;
            if (!fs::exists(dir)) { cout << "[Scan] skip missing " << dir.string() << endl; continue; }
            for (auto& e : fs::directory_iterator(dir)) {
                if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
                string rel = "capture/" + g_participant + "/" + sn + "/" + e.path().filename().string();
                jobs.push_back({e.path(), rel, (uint64_t)e.file_size()});
            }
        }
    } else {
        // 兜底: cam_indices 缺失 → root 去重后整扫
        vector<string> uniq;
        for (auto& r : g_roots)
            if (find(uniq.begin(), uniq.end(), r) == uniq.end()) uniq.push_back(r);
        for (auto& root : uniq) {
            fs::path base = fs::path(root) / g_participant;
            if (!fs::exists(base)) { cout << "[Scan] skip missing " << base.string() << endl; continue; }
            for (auto& e : fs::recursive_directory_iterator(base)) {
                if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
                string sn = e.path().parent_path().filename().string();
                if (!cam_wanted(sn)) continue;
                string rel = "capture/" + g_participant + "/" + sn + "/" + e.path().filename().string();
                jobs.push_back({e.path(), rel, (uint64_t)e.file_size()});
            }
        }
    }
    return jobs;
}

// master 附加数据 (capture.yaml is_master=true 时自动附带, 与 send_ui 阶段 3-5 对齐):
//   1. 相机标定 XML  {calib_save_dir}/{P}/output/**.xml → calib/cams/{P}/...
//   2. IR 位置       cfg/IR/{day_id}.txt                 → calib/IR/
//   3. arm pose      cfg/arm_pose/{day_id}.yaml          → calib/arm_pose/  (按天共享,
//      与 calib/IR 同级; rm-participant 的 */P00x 扫描不会误删)
//   4. 参与者映射    cfg/day_participant_map.json        → 根目录
// 全部 FORCE 覆盖语义 (小文件, 每次传输重写); 缺失项告警跳过不报错
static void addMasterExtras(const string& cfg_dir, vector<Job>& jobs) {
    size_t before = jobs.size();
    string calib_save, calib_part, day_id;
    try {
        Cfg cc(cfg_dir + "/cam_calib.yaml");
        calib_save = cc["calib"]["calib_save_dir"].as<string>();
        try { calib_part = cc["calib"]["participant_id"].as<string>(); }
        catch (...) { calib_part = g_participant; }
    } catch (...) {}
    try {
        Cfg arm(cfg_dir + "/calib_arm.yaml");
        day_id = arm["record"]["day_id"].as<string>();
    } catch (...) {}

    if (!calib_save.empty()) {
        fs::path xml_dir = fs::path(calib_save) / calib_part / "output";
        if (fs::exists(xml_dir)) {
            for (auto& e : fs::recursive_directory_iterator(xml_dir)) {
                if (!e.is_regular_file() || e.path().extension() != ".xml") continue;
                fs::path sub = fs::relative(e.path(), xml_dir);
                jobs.push_back({e.path(), "calib/cams/" + calib_part + "/"
                                + sub.generic_string(), (uint64_t)e.file_size(), true});
            }
        } else cout << "[Scan] skip missing " << xml_dir.string() << endl;
    }
    auto add1 = [&](const string& src, const string& rel) {
        error_code ec;
        if (fs::exists(src, ec) && fs::is_regular_file(src, ec))
            jobs.push_back({fs::path(src), rel, (uint64_t)fs::file_size(src, ec), true});
        else cout << "[Scan] skip missing " << src << endl;
    };
    if (!day_id.empty()) {
        add1(cfg_dir + "/IR/" + day_id + ".txt", "calib/IR/" + day_id + ".txt");
        add1(cfg_dir + "/arm_pose/" + day_id + ".yaml", "calib/arm_pose/" + day_id + ".yaml");
    }
    add1(cfg_dir + "/day_participant_map.json", "day_participant_map.json");
    cout << "[Scan] +" << (jobs.size() - before) << " master extras (xml/IR/arm_pose/map)" << endl;
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
        printf("\r%d/%d files  %.2f/%.2f TB  %.1f GB/s  elapsed %dm%02ds  ETA %.1f min  (skip %d, fail %d)",
               done.load(), total_files, (double)bytes.load() / 1e12, (double)total_bytes / 1e12,
               rate, (int)el / 60, (int)el % 60, eta / 60.0, skip.load(), fail.load());
        fflush(stdout);
    }
} g_prog;

// ================== 数据通道 (与 baseline_recv_data 两段式协议对应) ==================
// FILE 头 → 等 GO/SKIP → (GO 时) TransmitFile 数据 → 等 OK
// 先应答后发数据: 接收端对已存在文件回 SKIP 后不再读数据流,
// 若发送端先行灌数据会 TCP 背压死锁 (大文件 SKIP 场景)
// TransmitFile: 内核级 读盘+发送 重叠, 每次调用上限 DWORD, 大文件分段
// 返回 0=OK 1=SKIP -1=失败
int sendOne(SOCKET s, const Job& j) {
    // FILE = 大小一致 SKIP (h5 续传); FORCE = 覆盖重写 (标定附件每次必更新)
    if (!sendLine(s, string(j.force ? "FORCE " : "FILE ") + j.rel + " "
                   + to_string(j.size) + " 0")) return -1;
    string reply;
    if (!recvLine(s, reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("SKIP", 0) == 0) return 1;
    if (reply.rfind("GO", 0) != 0) {
        cerr << "\n[Recv] " << j.rel << ": " << reply << endl;
        return -1;
    }
    HANDLE hf = CreateFileA(j.path.string().c_str(), GENERIC_READ, FILE_SHARE_READ,
                            NULL, OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, NULL);
    if (hf == INVALID_HANDLE_VALUE) return -1;
    uint64_t remaining = j.size;
    while (remaining > 0) {
        DWORD chunk = (DWORD)min<uint64_t>(remaining, 1ull << 30);   // 1GB/段
        if (!TransmitFile(s, hf, chunk, 0, NULL, NULL, 0)) { CloseHandle(hf); return -1; }
        remaining -= chunk;                                           // 文件指针随发送推进
    }
    CloseHandle(hf);
    if (!recvLine(s, reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("OK", 0) == 0) return 0;
    cerr << "\n[Recv] " << j.rel << ": " << reply << endl;
    return -1;
}

// ---- 用户态重叠读发 (默认): 每文件一个读线程填 3 槽缓冲, 发送线程送出 ----
// 边读边算 64MB 块 CRC32, 数据流后追加 "CRC32 c1,c2,..." 行 —
// 接收端逐块比对, 不符回 ERR checksum → 本函数 -1 → worker 重试 (自愈)
int sendOneUser(SOCKET s, const Job& j) {
    ifstream in(j.path, ios::binary);
    if (!in) return -1;
    if (!sendLine(s, string(j.force ? "FORCE " : "FILE ") + j.rel + " "
                   + to_string(j.size) + " crc32")) return -1;
    string preply;
    if (!recvLine(s, preply, REPLY_TIMEOUT_MS)) return -1;   // 先应答后数据 (SKIP 不读流)
    if (preply.rfind("SKIP", 0) == 0) return 1;
    if (preply.rfind("GO", 0) != 0) {
        cerr << "\n[Recv] " << j.rel << ": " << preply << endl; return -1;
    }

    struct Slot { vector<char> buf; uint64_t len = 0; };
    vector<Slot> slots(3, Slot{vector<char>(CHUNK), 0});
    mutex mtx;
    condition_variable cv_empty, cv_full;
    size_t rd = 0, wr = 0, count = 0;      // 环形槽
    bool read_done = false, read_fail = false;
    uint64_t total_read = 0;

    // 块 CRC 累计 (reader 线程私有; 64MB 边界跨 8MB 槽切割, 文件尾封口)
    uint32_t rcrc = 0; size_t rfill = 0;
    vector<uint32_t> rcrcs;
    auto feed = [&](const char* p, int n) {
        while (n > 0) {
            size_t take = min((size_t)n, CRC_BLK - rfill);
            rcrc = crc32c_hw(rcrc, p, take);
            rfill += take; p += take; n -= (int)take;
            if (rfill == CRC_BLK) { rcrcs.push_back(rcrc); rcrc = 0; rfill = 0; }
        }
    };

    thread reader([&]() {
        while (true) {
            unique_lock<mutex> lk(mtx);
            cv_empty.wait(lk, [&] { return count < slots.size() || read_done; });
            if (read_done) return;
            lk.unlock();
            in.read(slots[wr].buf.data(), CHUNK);
            int got = (int)in.gcount();
            if (got > 0) feed(slots[wr].buf.data(), got);
            lk.lock();
            if (got <= 0) {
                read_fail = total_read < j.size;
                if (rfill > 0) { rcrcs.push_back(rcrc); rcrc = 0; rfill = 0; }  // 尾块
                read_done = true;
            } else {
                slots[wr].len = (uint64_t)got; total_read += (uint64_t)got;
                wr = (wr + 1) % slots.size(); ++count;
            }
            cv_full.notify_one();
            if (read_done) return;
        }
    });

    bool send_fail = false;
    while (true) {
        unique_lock<mutex> lk(mtx);
        cv_full.wait(lk, [&] { return count > 0 || read_done; });
        if (count == 0) break;                    // 读尽
        Slot& sl = slots[rd];
        lk.unlock();
        const char* p = sl.buf.data();
        uint64_t off = 0;
        while (off < sl.len) {
            int n = send(s, p + off, (int)min<uint64_t>(sl.len - off, 1u << 30), 0);
            if (n <= 0) { send_fail = true; break; }
            off += (uint64_t)n;
        }
        lk.lock();
        rd = (rd + 1) % slots.size(); --count;
        cv_empty.notify_one();
        if (send_fail) break;
    }
    { unique_lock<mutex> lk(mtx); read_done = true; }
    cv_empty.notify_all();
    reader.join();
    if (read_fail || send_fail || total_read != j.size) return -1;

    // trailer: 块 CRC 清单 (接收端比对后回 OK / ERR checksum)
    { ostringstream os; os << "CRC32";
      for (uint32_t c : rcrcs) os << " " << hex << c;
      if (!sendLine(s, os.str())) return -1; }

    string reply;
    if (!recvLine(s, reply, REPLY_TIMEOUT_MS)) return -1;
    if (reply.rfind("OK", 0) == 0) return 0;
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
            int r = g_tx_user ? sendOneUser(s, j) : sendOne(s, j);
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
        else if (a == "--tx") {                  // transmitfile (默认) | user (重叠读发对照)
            string mode; nextval(mode); g_tx_user = (mode == "user");
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
        try { g_cam_ids = c["cam_indices"].as<vector<string>>(); } catch (...) {}
        try { g_is_master = c["is_master"].as<bool>(); } catch (...) {}
        Cfg xf(cfg_dir + "/transfer.yaml"); auto& t = xf["transfer"];
        // 链路按角色自动选: master → 口1 (10.10.1.1), slave → 口2 (10.10.2.1)
        g_server_ip = data_ip_override.empty()
            ? t[g_is_master ? "server_ip_master_link" : "server_ip_slave_link"].as<string>()
            : data_ip_override;
        g_server_port = port_override ? port_override : t["server_port"].as<int>();
        g_workers = workers_override ? workers_override : t["workers"].as<int>();
    } catch (const exception& e) {
        cerr << "[Error] config: " << e.what() << endl; return 1;
    }

    vector<Job> jobs = scanFiles();
    if (g_is_master) addMasterExtras(cfg_dir, jobs);
    if (jobs.empty()) {
        cerr << "[Error] no h5 under roots for " << g_participant << endl;
        return 1;
    }
    g_prog.total_files = (int)jobs.size();
    for (auto& j : jobs) g_prog.total_bytes += j.size;
    cout << "=== send_slave: " << jobs.size() << " files, " << fixed << setprecision(2)
         << g_prog.total_bytes / 1e12 << " TB -> " << g_server_ip << ":" << g_server_port
         << " (" << g_workers << " streams) ===" << endl;

    // 按数据盘分桶 + worker 绑定盘 (每盘流数 = workers/盘数): 4 流跨盘交错取队列时
    // 会交错打在同一块盘上 (单盘主要读惩罚, master 实测 4.0→1.3 GB/s 衰减);
    // 每盘固定流数做连续顺序读, 交错惩罚减半
    vector<vector<Job>> buckets;                  // 按盘符 (如 "D:" / "E:")
    for (auto& j : jobs) {
        string drv = j.path.root_name().string();
        auto it = find_if(buckets.begin(), buckets.end(),
                          [&](const vector<Job>& b) { return !b.empty() && b[0].path.root_name().string() == drv; });
        if (it != buckets.end()) it->push_back(j);
        else buckets.push_back({j});
    }
    int nb = (int)buckets.size();
    vector<atomic<size_t>> nexts(nb);
    vector<thread> ths;
    for (int b = 0; b < nb; ++b) {
        int nw = g_workers / nb + (b < g_workers % nb ? 1 : 0);
        for (int i = 0; i < nw; ++i) ths.emplace_back(workerMain, &buckets[b], &nexts[b]);
    }
    for (auto& t : ths) t.join();

    // 退出屏障: BYE 让接收端等 RAM 全部落盘后应答 — 本进程退出即数据全在盘上
    // (无此屏障时接收端 OK=已入RAM, 发送端退出后盘还在写)
    {
        SOCKET s = connectTo(g_server_ip, g_server_port);
        if (s != INVALID_SOCKET) {
            sendLine(s, "BYE");
            string reply;
            if (recvLine(s, reply, 4 * 3600 * 1000) && reply == "BYE")
                cout << "[Recv] all data flushed to disk" << endl;
            else
                cerr << "[Recv] WARN: no BYE ack (receiver draining?)" << endl;
            closesocket(s);
        }
    }
    g_prog.line(); cout << endl;
    double el = chrono::duration<double>(chrono::steady_clock::now() - g_prog.t0).count();
    cout << "=== done in " << fixed << setprecision(1) << el / 60 << " min: ok="
         << g_prog.done - g_prog.skip - g_prog.fail << " skip=" << g_prog.skip
         << " fail=" << g_prog.fail << " ===" << endl;
    WSACleanup();
    return g_prog.fail.load() ? 1 : 0;
}
