// ================== send_ui ==================
// 交互式串行传输 (Windows, master/slave 自动识别): OpenCV UI 确认配置 →
// SPACE 开始 → master 先传, 完成后令 slave 传输 (接收端 baseline_recv_data
// 直写 ZFS 归档柜, 见 plan/data_pipeline.md)。
//
// 阶段 (slave: 1-2; master: 1-6):
//   1/2. D:/capture、E:/capture 的 h5 → /data/dataset/capture/{P}/
//   3. 相机内外参 XML ({calib_save_dir}/{P}/output) → /data/dataset/calib/cams/{P}/
//   4. 红外发射器位置 cfg/IR/{day_id}.txt → /data/dataset/calib/IR/{day_id}.txt
//   5. arm pose cfg/arm_pose/{day_id}.yaml → /data/dataset/calib/arm_pose/{day_id}.yaml
//   6. cfg/day_participant_map.json → /data/dataset/day_participant_map.json
//
// 配置: capture.yaml (participant_id/is_master/master_ip + participant_root↔cam_indices
//       动态配对加载, 两盘按字节量加权分流), transfer.yaml (链路/流数),
//       cam_calib.yaml (calib_save_dir), calib_arm.yaml (record.day_id)
// 按键 (仅 master 有效, slave 全程由 master 控制):
//   SPACE = 开始 / 中断后续传 (SKIP 断点, 已传文件秒过)
//   z     = 中断 (文件粒度温和停止, 两机联动; SPACE 续传)
//   q/ESC = 退出 (任意时刻含 SPACE 前; 两机联动退出)
//   传输中 q = 当前文件完成后温和中止 (终态, 非中断)
// 引擎: 用户态读发管线 (reader/sender 双线程 3×8MB 槽) + 64MB 块 CRC32C
//       硬件校验 trailer (TransmitFile 已弃用 — 数据流损坏, 见 plan/transfer_direct_mode.md §8)。
// =================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #include <mswsock.h>              // (TransmitFile 已弃用, 保留头无副作用)
    #include <intrin.h>               // _mm_crc32_u64 (SSE4.2 硬件 CRC32C)
    #pragma comment(lib, "ws2_32.lib")
    #pragma comment(lib, "Mswsock.lib")
#else
    #error "sender is Windows-only (collection hosts)"
#endif

#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
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
static const int REPLY_TIMEOUT_MS = 180000;   // direct 接收端 OK=整文件写完 (~33s), 3 倍余量防超时重传并发写 .part

// ---- CRC32C 硬件校验 (SSE4.2, 与 send_slave.cpp / baseline_recv_data.cpp 同步) —
// 64MB 块。查表版 0.43GB/s/核 (串行依赖链, 曾把聚合压到 1.4 GB/s), 硬件版
// 7GB/s/核 (对照 crc_bench.c)。初值直传累计, 无 pre/post xor。
static const size_t CRC_BLK = 64ull << 20;
static uint32_t crc32c_hw(uint32_t crc, const void* p, size_t n) {
    const uint8_t* b = (const uint8_t*)p;
    while (n >= 8) { crc = (uint32_t)_mm_crc32_u64(crc, *(const uint64_t*)b); b += 8; n -= 8; }
    if (n >= 4) { crc = _mm_crc32_u32(crc, *(const uint32_t*)b); b += 4; n -= 4; }
    while (n--) crc = _mm_crc32_u8(crc, *b++);
    return crc;
}

// ================== 配置 ==================
struct UiCfg {
    string server_ip;                 // 按 capture.yaml is_master 选 master/slave 链路
    int server_port = 5001;
    int workers = 4;
    string participant;
    bool is_master = false;
    vector<string> roots;             // 图像盘串行顺序: D:/capture, E:/capture
    vector<string> cam_ids;           // capture.yaml cam_indices (与 root 一一配对)
    string cfg_dir;                   // cpp_eyetracker/cfg (xml/IR/map 源)
    string xml_dir;                   // {calib_save_dir}/{P}/output (master)
    string ir_file;                   // cfg/IR/{day_id}.txt (master)
    string arm_pose_file;             // cfg/arm_pose/{day_id}.yaml (master, 按天共享)
    string map_file;                  // cfg/day_participant_map.json (master)
    string master_ip;                 // 握手用 (capture.yaml network.master_ip)
    int handshake_port = 50100;       // transfer.yaml
};
static UiCfg g_cfg;

// ================== 传输状态 ==================
struct Job { fs::path path; string rel; uint64_t size; bool force = false; };   // force: 覆盖式 (标定附件)
struct PhasePlan { string label; vector<Job> jobs; uint64_t bytes = 0; };
static vector<PhasePlan> g_plans;

enum class Phase { CONFIG, RUNNING, WAIT_SLAVE, PAUSED, DONE, ABORTED };
static atomic<int> g_phase{(int)Phase::CONFIG};
static atomic<bool> g_abort{false};
static atomic<bool> g_pause{false};             // z 中断: 文件粒度停止, SPACE 续传
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
static atomic<bool> g_master_connected{false};   // slave 视角: 已连上 master
static atomic<bool> g_exit{false};               // slave: 收到 master 退出信号 → 结程
static string g_slave_summary;
static atomic<bool> g_conn_fail{false};          // 数据连接失败 (UI 红字提示)
static string g_conn_target;
static atomic<bool> g_4090_ok{false};            // 4090 数据链路已连通至少一次
static atomic<uint64_t> g_skip_bytes{0};         // SKIP 文件字节 (进度条计入)
static atomic<uint64_t> g_wait_t0_us{0};         // WAIT_SLAVE 起始 (显示等待时长)

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
static SOCKET connectTo(const string& ip, int port, int max_retry = 0) {  // 0=无限 (数据链路)
    int retry = 0;
    while (true) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        sockaddr_in sa{};
        sa.sin_family = AF_INET; sa.sin_port = htons((u_short)port);
        inet_pton(AF_INET, ip.c_str(), &sa.sin_addr);
        if (connect(s, (sockaddr*)&sa, sizeof(sa)) == 0) {
            g_conn_fail.store(false);                 // 恢复: 清除告警
            return s;
        }
        closesocket(s);
        ++retry;
        g_conn_target = ip + ":" + to_string(port);
        g_conn_fail.store(true);                      // UI 红字: 环境问题一眼可见
        if (max_retry && retry >= max_retry) return INVALID_SOCKET;
        if (retry == 1 || retry % 10 == 0)
            cout << ts() << "[Net] connect " << ip << ":" << port
                 << " failed (retry #" << retry << ", peer running?)" << endl;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
}
static SOCKET openStream() {
    SOCKET s = connectTo(g_cfg.server_ip, g_cfg.server_port);
    g_4090_ok.store(true);                           // UI: 4090 链路 OK
    int one = 1, buf = SOCK_BUF;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char*)&buf, sizeof(buf));
    return s;
}

// ================== 传输引擎 (用户态读发管线 + 块 CRC32C, 同 send_slave 定稿版) ====
// TransmitFile 已弃用: 两次独立复现数据流损坏 (帧错位, 大小校验拦不住)。
// reader/sender 双线程 3×8MB 槽重叠; 边读边算 64MB 块 CRC32C,
// 数据流后追加 "CRC32 c1 c2 ..." trailer — 接收端逐块比对, 不符 ERR → 重试自愈。
// 先应答后发数据: 接收端对已存在文件回 SKIP 后不再读数据流,
// 若发送端先行灌数据会 TCP 背压死锁 (10GB 大文件 SKIP 场景)
static int sendOne(SOCKET s, const Job& j) {
    // FILE   = 大小一致则 SKIP (断点续传, 图像用)
    // FORCE  = 总是覆盖写入 (标定附件: xml / IR / arm_pose / map)
    ifstream in(j.path, ios::binary);
    if (!in) return -1;
    if (!sendLine(s, string(j.force ? "FORCE " : "FILE ") + j.rel + " "
                   + to_string(j.size) + " crc32")) return -1;
    string preply;
    if (!recvLine(s, preply, REPLY_TIMEOUT_MS)) return -1;   // 先应答后数据
    if (preply.rfind("SKIP", 0) == 0) return 1;
    if (preply.rfind("GO", 0) != 0) {
        cerr << ts() << "[Recv] " << j.rel << ": " << preply << endl; return -1;
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
    cerr << ts() << "[Recv] " << j.rel << ": " << reply << endl;
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
        else if (r == 1) { g_skip++; g_skip_bytes += j.size; }   // 进度条计入 SKIP
        else { g_fail++; g_last_fail = j.rel; }
    }
}

static void transferController() {
    // 会话重置 (z 中断后续传): 已传文件由接收端 SKIP 重新计入,
    // 进度条口径 = 本会话 sent + skip, 天然覆盖历史进度
    g_abort.store(false);
    g_done_files.store(0); g_skip.store(0); g_fail.store(0);
    g_bytes.store(0); g_skip_bytes.store(0); g_plan_idx.store(0);
    g_t0_us.store(chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count());
    for (size_t d = 0; d < g_plans.size(); ++d) {
        if (g_abort.load()) break;
        g_plan_idx = (int)d;
        // 按数据盘分桶 + worker 绑定盘: 单一队列会让 4 流交错取到同一块盘的文件
        // (单盘主要读惩罚, master 实测 4.0→1.3 GB/s 衰减); 每盘固定流数连续顺序读
        vector<vector<Job>> buckets;
        for (auto& j : g_plans[d].jobs) {
            string drv = j.path.root_name().string();
            auto it = find_if(buckets.begin(), buckets.end(),
                              [&](const vector<Job>& b) { return !b.empty() && b[0].path.root_name().string() == drv; });
            if (it != buckets.end()) it->push_back(j);
            else buckets.push_back({j});
        }
        int nb = (int)buckets.size();
        // 流数按桶字节量加权 → 各盘同时收尾, 全程双盘并行读。
        // (固定均分 + 桶文件量不均时, 轻盘先空 → 剩余时间全打一块盘)
        vector<uint64_t> bbytes(nb, 0);
        for (int b = 0; b < nb; ++b)
            for (auto& j : buckets[b]) bbytes[b] += j.size;
        uint64_t tot = 0;
        for (auto v : bbytes) tot += v;
        vector<int> nw(nb, 1);
        if (tot > 0 && nb <= g_cfg.workers) {
            for (int b = 0; b < nb; ++b)
                nw[b] = 1 + (int)((uint64_t)(g_cfg.workers - nb) * bbytes[b] / tot);
            int sum = 0;
            for (int v : nw) sum += v;
            nw[(int)(max_element(bbytes.begin(), bbytes.end()) - bbytes.begin())]
                += g_cfg.workers - sum;              // 取整余数归最大桶
        }
        vector<atomic<size_t>> nexts(nb);
        vector<thread> ths;
        for (int b = 0; b < nb; ++b) {
            for (int i = 0; i < nw[b]; ++i)
                ths.emplace_back(workerLoop, &buckets[b], &nexts[b]);
        }
        for (auto& t : ths) t.join();
    }
    g_t_end_us.store(chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count());

    // 退出屏障: BYE 让接收端等 RAM 全部落盘后应答 — 数据全在盘上才算传完
    // (无此屏障时接收端 OK=已入RAM, 本机退出后盘还在写)
    {
        SOCKET s = connectTo(g_cfg.server_ip, g_cfg.server_port);
        if (s != INVALID_SOCKET) {
            sendLine(s, "BYE");
            string reply;
            if (recvLine(s, reply, 4 * 3600 * 1000) && reply == "BYE")
                cout << ts() << "[Recv] all data flushed to disk" << endl;
            else
                cout << ts() << "[Recv] WARN: no BYE ack (receiver draining?)" << endl;
            closesocket(s);
        }
    }

    // master: 本机完成后令 slave 传输, 等 SLAVE_DONE
    if (g_cfg.is_master && g_hs != INVALID_SOCKET) {
        if (g_abort.load() && !g_pause.load()) {
            // q 温和中止 (终态): 通知 slave 停止待命; 握手保留至 master 退出发 QUIT
            sendLine(g_hs, "ABORT");
            cout << ts() << "[HS] ABORT sent to slave (standing by)" << endl;
        } else if (!g_abort.load()) {
            sendLine(g_hs, "START");
            cout << ts() << "[HS] START sent to slave" << endl;
            g_wait_t0_us.store(chrono::duration_cast<chrono::microseconds>(
                chrono::steady_clock::now().time_since_epoch()).count());
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
            // 不关闭 g_hs: 留给 master 退出 (ESC/q) 发 QUIT 或 z 中断后续传再发 START
        }
        // z 中断 (pause): 本机停在文件边界, slave 尚未启动, 无需发令;
        //   WAIT_SLAVE 期间的中断由 UI 线程直接发 ABORT, 此处收到 SLAVE_DONE 后同路返回
    }
    g_phase.store((int)(g_pause.load() ? Phase::PAUSED :
                        g_abort.load() ? Phase::ABORTED : Phase::DONE));
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
            // select 返回后再核验: 传输可能恰在等待间隙结束, 此刻到达的是
            // SLAVE_DONE 的 ACK (不是退出信号) — 不算中止
            if ((Phase)g_phase.load() != Phase::RUNNING) return;
            g_abort = true;
            cout << ts() << "[HS] master signalled quit / disconnected — aborting" << endl;
            return;
        }
    }
}

// slave: 连 master (上限 ~2min, master 未启/已退出则自行退出) → READY →
// 命令循环: START=传输(→SLAVE_DONE→ACK→待命), ABORT=待命(master 中止/暂停),
// QUIT/掉线=退出 — 支持 master 多次 START (z 中断后 SPACE 续传)
static void slaveHandshakeAndRun() {
    cout << ts() << "[HS] Slave connecting to master " << g_cfg.master_ip
         << ":" << g_cfg.handshake_port << " ..." << endl;
    SOCKET s = connectTo(g_cfg.master_ip, g_cfg.handshake_port, 240);
    if (s == INVALID_SOCKET) {                        // master 未启动或已退出
        cerr << ts() << "[HS] cannot reach master — exiting" << endl;
        g_phase.store((int)Phase::ABORTED);
        g_exit.store(true);
        return;
    }
    sendLine(s, "READY " + to_string(g_total_files) + " " + to_string(g_total_bytes));
    g_master_connected.store(true);                   // UI: 已连上, 等 START
    for (;;) {
        string cmd;
        if (!recvLine(s, cmd, 2 * 3600 * 1000)) {     // master 掉线 (含已退出)
            cout << ts() << "[HS] master disconnected — exiting" << endl;
            g_exit.store(true);
            return;
        }
        if (cmd == "QUIT") {
            cout << ts() << "[HS] master QUIT — exiting" << endl;
            g_exit.store(true);
            return;
        }
        if (cmd == "ABORT") {                         // master 温和中止/暂停 → 待命
            cout << ts() << "[HS] master ABORT — standing by" << endl;
            continue;
        }
        if (cmd != "START") continue;

        cout << ts() << "[HS] Master ordered START — slave transferring." << endl;
        g_phase.store((int)Phase::RUNNING);           // UI 切进度屏
        thread(slaveQuitWatcher, s).detach();         // 监测 master 中止/掉线
        transferController();                         // 结束时置 PAUSED/DONE/ABORTED
        int ok = g_done_files.load() - g_skip.load() - g_fail.load();
        cout << ts() << "[HS] Sending SLAVE_DONE ok=" << ok << " skip="
             << g_skip.load() << " fail=" << g_fail.load() << endl;
        sendLine(s, "SLAVE_DONE " + to_string(ok) + " "
                      + to_string(g_skip.load()) + " " + to_string(g_fail.load()));
        string ack;
        recvLine(s, ack, 60000);                      // ACK (尽力而为)
        g_phase.store((int)Phase::PAUSED);            // 待命: master 决定续传 (START) 或退出 (QUIT)
    }
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

// 按配对逐相机扫描 (capture.yaml participant_root[i] ↔ cam_indices[i], 与
// send_slave.scanFiles / loader 同口径); phase 仍按数据盘组织 (配置屏按盘展示)。
// 兜底: cam_indices 缺失 → root 去重后整目录扫 (旧行为)。
static void addImagePhases() {
    if (!g_cfg.cam_ids.empty()) {
        map<string, PhasePlan> by_root;               // 盘符 → phase
        for (size_t i = 0; i < g_cfg.cam_ids.size(); ++i) {
            const string& sn = g_cfg.cam_ids[i];
            fs::path dir = fs::path(g_cfg.roots[i % g_cfg.roots.size()])
                         / g_cfg.participant / sn;
            if (!fs::exists(dir)) {
                cout << ts() << "[Warn] missing " << dir.string() << endl;
                continue;
            }
            PhasePlan& plan = by_root[dir.root_name().string()];
            for (auto& e : fs::directory_iterator(dir)) {
                if (!e.is_regular_file() || e.path().extension() != ".h5") continue;
                string rel = "capture/" + g_cfg.participant + "/" + sn
                           + "/" + e.path().filename().string();
                plan.jobs.push_back({e.path(), rel, (uint64_t)e.file_size()});
                plan.bytes += (uint64_t)e.file_size();
            }
        }
        for (auto& [drv, plan] : by_root) {
            sort(plan.jobs.begin(), plan.jobs.end(),
                 [](const Job& a, const Job& b) { return a.rel < b.rel; });
            plan.label = drv + "/capture  (h5 -> capture/" + g_cfg.participant + ")";
            g_total_files += (int)plan.jobs.size();
            g_total_bytes += plan.bytes;
            g_plans.push_back(move(plan));
        }
        return;
    }
    vector<string> uniq;                              // 兜底: root 去重整扫
    for (auto& r : g_cfg.roots)
        if (find(uniq.begin(), uniq.end(), r) == uniq.end()) uniq.push_back(r);
    for (auto& root : uniq) addImagePhase(root);
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
// 两行链路状态: 对端采集主机 (握手) + 4090 (数据)
static void drawLinks(cv::Mat& cv, int& y) {
    string peer = g_cfg.is_master
        ? (g_slave_ready.load() ? "Slave      : Connected" : "Slave      : Not connected - waiting...")
        : (g_master_connected.load() ? "Master     : Connected" : "Master     : Connecting...");
    cv::Scalar pc = (g_cfg.is_master ? g_slave_ready.load() : g_master_connected.load())
                    ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 165, 255};
    cv::putText(cv, peer, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, pc, 1, cv::LINE_AA);
    y += 30;
    string srv;
    cv::Scalar sc;
    if (g_conn_fail.load()) { srv = "4090       : " + g_conn_target + "  FAIL (retrying)"; sc = {0, 0, 255}; }
    else if (g_4090_ok.load()) { srv = "4090       : " + g_cfg.server_ip + ":" + to_string(g_cfg.server_port) + "  OK"; sc = {0, 255, 0}; }
    else { srv = "4090       : " + g_cfg.server_ip + ":" + to_string(g_cfg.server_port) + "  idle"; sc = {150, 150, 150}; }
    cv::putText(cv, srv, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, sc, 1, cv::LINE_AA);
    y += 34;
}

static void drawConfig(cv::Mat& cv) {
    cv = cv::Mat::zeros(600, 900, CV_8UC3);
    int y = 44;
    auto put = [&](const string& s, double sc, cv::Scalar c, int dy = 30) {
        cv::putText(cv, s, {40, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, 1, cv::LINE_AA);
        y += dy;
    };
    put("100G Transfer - Configuration", 0.85, {0, 215, 255}, 46);
    put("Role        : " + string(g_cfg.is_master ? "MASTER" : "SLAVE"), 0.55, {255, 255, 255});
    drawLinks(cv, y);                                 // 对端主机 + 4090 链路状态
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
                ph == Phase::PAUSED ? "PAUSED - interrupted (SPACE resumes)" :
                (ph == Phase::DONE ? "DONE" : "ABORTED"),
                {40, y}, cv::FONT_HERSHEY_DUPLEX, 0.9, {0, 215, 255}, 2, cv::LINE_AA);
    y += 52;
    int idx = min(g_plan_idx.load(), (int)g_plans.size() - 1);
    ostringstream phs;
    phs << "Phase " << idx + 1 << "/" << g_plans.size() << "  " << g_plans[idx].label;
    cv::putText(cv, phs.str(), {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
    y += 44;
    drawLinks(cv, y);                                 // 对端主机 + 4090 链路状态
    // WAIT_SLAVE: 显示已等待秒数
    if (ph == Phase::WAIT_SLAVE && g_wait_t0_us.load()) {
        uint64_t tnow = chrono::duration_cast<chrono::microseconds>(
            chrono::steady_clock::now().time_since_epoch()).count();
        char wb[64]; snprintf(wb, sizeof(wb), "Waiting %d s", (int)((tnow - g_wait_t0_us.load()) / 1000000));
        cv::putText(cv, wb, {460, 64}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
    }
    double frac = g_total_bytes
        ? (double)(g_bytes.load() + g_skip_bytes.load()) / (double)g_total_bytes : 0;
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
    snprintf(b, sizeof(b), "Data  : %.2f / %.2f TB  (skipped %.2f TB counted)",
             (double)(g_bytes.load() + g_skip_bytes.load()) / 1e12,
             (double)g_total_bytes / 1e12, (double)g_skip_bytes.load() / 1e12);
    cv::putText(cv, b, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 200}, 1, cv::LINE_AA); y += 34;
    snprintf(b, sizeof(b), "Rate  : %.2f GB/s    Elapsed : %.1f min    ETA : %.1f min",
             rate, el / 60, eta / 60);
    cv::putText(cv, b, {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 200}, 1, cv::LINE_AA); y += 50;
    if (g_fail.load()) {
        cv::putText(cv, "Last fail: " + g_last_fail, {40, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255}, 1, cv::LINE_AA); y += 34;
    }
    if (g_conn_fail.load()) {
        cv::putText(cv, "Cannot connect " + g_conn_target + " — is receiver running?",
                    {40, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 255}, 2, cv::LINE_AA); y += 34;
    }
    if (ph == Phase::DONE && g_cfg.is_master && !g_slave_summary.empty()) {
        cv::putText(cv, "Slave:" + g_slave_summary, {40, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 200}, 1, cv::LINE_AA); y += 30;
    }
    string hint;
    if (ph == Phase::RUNNING)
        hint = g_cfg.is_master ? "[z] Pause (SPACE resumes)   [q] Abort after current file"
                               : "Controlled by master";
    else if (ph == Phase::WAIT_SLAVE)
        hint = "[z] Pause (slave stops at file boundary)   [q] Abort wait (slave continues)";
    else if (ph == Phase::PAUSED)
        hint = g_cfg.is_master ? "[SPACE] Resume   [q/ESC] Quit (quits slave too)"
                               : (g_abort.load() ? "Paused - waiting for master command..."
                                                 : "Done - waiting for master to exit...");
    else if (!g_cfg.is_master)
        hint = "Waiting for master to exit...";
    else
        hint = "[ESC/q] Exit (quits slave too)";
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
        Cfg cap(g_cfg.cfg_dir + "/capture.yaml");     // 主机角色 + 握手目标 + participant
        g_cfg.is_master = cap["capture"]["is_master"].as<bool>();
        g_cfg.master_ip = cap["capture"]["master_ip"].as<string>();
        g_cfg.participant = participant_override.empty()
                            ? cap["capture"]["participant_id"].as<string>() : participant_override;
        // 动态加载存储配置: participant_root[i] ↔ cam_indices[i] 一一配对
        // (与 send_slave.scanFiles / loader 同口径; 不再硬编码盘符)
        if (roots_override.empty())
            g_cfg.roots = cap["capture"]["participant_root"].as<vector<string>>();
        else g_cfg.roots = roots_override;
        try { g_cfg.cam_ids = cap["capture"]["cam_indices"].as<vector<string>>(); } catch (...) {}
        Cfg xf(g_cfg.cfg_dir + "/transfer.yaml"); auto& t = xf["transfer"];
        string link = g_cfg.is_master ? t["server_ip_master_link"].as<string>()
                                      : t["server_ip_slave_link"].as<string>();
        g_cfg.server_ip = data_ip_override.empty() ? link : data_ip_override;
        g_cfg.server_port = port_override ? port_override : t["server_port"].as<int>();
        g_cfg.workers = workers_override ? workers_override : t["workers"].as<int>();
        try { g_cfg.handshake_port = t["handshake_port"].as<int>(); } catch (...) {}
    } catch (const exception& e) {
        cerr << ts() << "[Error] config: " << e.what() << endl; return 1;
    }

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
        if (!day_id.empty()) {
            g_cfg.ir_file = g_cfg.cfg_dir + "/IR/" + day_id + ".txt";
            g_cfg.arm_pose_file = g_cfg.cfg_dir + "/arm_pose/" + day_id + ".yaml";
        }
        g_cfg.map_file = g_cfg.cfg_dir + "/day_participant_map.json";
    }

    // 构建阶段 (按 capture.yaml 配对逐相机扫描)
    addImagePhases();
    if (g_cfg.is_master) {
        if (!g_cfg.xml_dir.empty()) addXmlPhase();
        if (!g_cfg.ir_file.empty())
            addFilePhase(g_cfg.ir_file, "calib/IR/" + fs::path(g_cfg.ir_file).filename().generic_string(),
                         "IR positions (calib/IR)");
        if (!g_cfg.arm_pose_file.empty())
            addFilePhase(g_cfg.arm_pose_file,
                         "calib/arm_pose/" + fs::path(g_cfg.arm_pose_file).filename().generic_string(),
                         "arm pose (calib/arm_pose, 按天共享)");
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
        if (g_exit.load()) break;                      // slave: master 已退出
        Phase ph = (Phase)g_phase.load();
        if (ph == Phase::CONFIG) drawConfig(canvas);
        else drawProgress(canvas);
        cv::imshow("send_ui", canvas);
        int key = cv::waitKey(ph == Phase::RUNNING ? 30 : 50);
        if (!g_cfg.is_master) continue;                 // slave: 按键全失效, 由 master 控制退出
        if (ph == Phase::CONFIG) {
            if (key == ' ' && g_slave_ready.load()) {   // 需 slave 已握手
                g_phase.store((int)Phase::RUNNING);
                ctl = thread(transferController);
            } else if (key == 'q' || key == 27) {       // 通知 slave 一并退出
                if (g_hs != INVALID_SOCKET) {
                    sendLine(g_hs, "QUIT");
                    closesocket(g_hs); g_hs = INVALID_SOCKET;
                } else {
                    cout << ts() << "[HS] slave not connected — "
                         << "if it starts later it gives up after ~2min" << endl;
                }
                break;
            }
        } else if (ph == Phase::RUNNING) {
            if (key == 'z') {                    // 中断: 文件粒度温和停止 → PAUSED
                g_pause.store(true);
                g_abort.store(true);
            } else if (key == 'q' || key == 27) { // 温和中止 (终态)
                g_abort = true;
            }
        } else if (ph == Phase::WAIT_SLAVE) {
            if (key == 'z' && g_hs != INVALID_SOCKET) {   // 中断 slave 传输 (文件粒度)
                g_pause.store(true);
                sendLine(g_hs, "ABORT");                  // slave 看门狗捕获后停止
            } else if ((key == 'q' || key == 27) && g_hs != INVALID_SOCKET) {
                sendLine(g_hs, "QUIT");                   // slave 看门狗捕获后中止
                closesocket(g_hs); g_hs = INVALID_SOCKET;
                break;
            }
        } else if (ph == Phase::PAUSED) {
            if (key == ' ' ) {                            // 续传: 重跑全程 (SKIP 秒过)
                if (ctl.joinable()) ctl.join();           // 上一会话 controller 已结束
                g_pause.store(false);
                g_phase.store((int)Phase::RUNNING);
                ctl = thread(transferController);
            } else if (key == 'q' || key == 27) {
                if (g_hs != INVALID_SOCKET) {
                    sendLine(g_hs, "QUIT");
                    closesocket(g_hs); g_hs = INVALID_SOCKET;
                }
                break;
            }
        } else {
            if (key == 'q' || key == 27) break;         // DONE/ABORTED: 仅 ESC/q 退出
        }
    }
    g_abort = true;
    if (ctl.joinable()) ctl.join();
    // master 退出前通知 slave 一并结束 (握手 socket 在 ACK 后保留至此)
    if (g_cfg.is_master && g_hs != INVALID_SOCKET) {
        sendLine(g_hs, "QUIT");
        closesocket(g_hs);
        g_hs = INVALID_SOCKET;
    }
    cv::destroyAllWindows();
    WSACleanup();
    return g_fail.load() ? 1 : 0;
}
