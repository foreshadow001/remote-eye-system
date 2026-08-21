// capture_with_M5Stack.cpp — Multi-cam HDF5 recording + Piper arm control + M5Stack 状态灯
// Master: camera capture + arm control + gaze forwarding to Slave + M5Stack LED 状态同步
// Slave:  camera capture only, receives gaze from Master via TCP
// LED 状态 → 图案映射见 plan/m5stack_atom_matrix.md
// ====================================================================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>  /* ... omitted for brevity ... */
#endif

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
#include <algorithm>
#include <system_error>
#include <pylon/PylonIncludes.h>
#include <H5Cpp.h>

#include "cam/basler.hpp"
#include "cfg/config.hpp"
#include "piper/piper.hpp"
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== 控制台时间戳 (每行前缀 [YYYY-MM-DD HH:MM:SS]) ==================
string nowTimestamp() {
    auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
    tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    char buf[32];
    strftime(buf, sizeof(buf), "[%Y-%m-%d %H:%M:%S] ", &tmv);
    return string(buf);
}

// 自定义 streambuf: 每个换行后的行首自动插入时间戳
class TimestampBuf : public std::streambuf {
    std::streambuf* dst_;
    bool at_line_start_ = true;
    mutex mtx_;  // 序列化输出, 防止多线程行交错
public:
    explicit TimestampBuf(std::streambuf* dst) : dst_(dst) {}
protected:
    int_type overflow(int_type c) override {
        if (c == traits_type::eof()) return c;
        lock_guard<mutex> lk(mtx_);
        if (at_line_start_) {
            string ts = nowTimestamp();
            for (char ch : ts) dst_->sputc(ch);
            at_line_start_ = false;
        }
        if (c == '\n') at_line_start_ = true;
        return dst_->sputc((char)c);
    }
    int sync() override {
        lock_guard<mutex> lk(mtx_);
        return dst_->pubsync();
    }
};

// ================== UI / Recording globals ==================
atomic<bool> global_running{true};
atomic<bool> net_cmd_record{false};
string shared_record_timestr;
chrono::steady_clock::time_point global_record_start_time;
bool g_use_hw_trigger = false;

// ================== TCP command channel (cmd port, replaces UDP) ==================
SOCKET g_cmd_listen_sock = INVALID_SOCKET;
SOCKET g_cmd_sock = INVALID_SOCKET;
mutex g_cmd_send_mtx;

// ================== Gaze forwarding (gaze port) ==================
SOCKET g_gaze_listen_sock = INVALID_SOCKET;
SOCKET g_gaze_sock = INVALID_SOCKET;
mutex g_gaze_send_mtx;
atomic<double> g_gaze_x{0}, g_gaze_y{0}, g_gaze_z{0};
atomic<bool> g_gaze_ready{false};
atomic<bool> g_gaze_need_send{false};
atomic<bool> g_gaze_connected{false};
atomic<bool> g_slave_hdf5_done{false};
atomic<bool> g_gaze_done{false};
bool g_show_exhausted = false;           // true = render EXHAUSTED UI after dump

// LED state (Master only, effective after main loop starts)
enum class LedState { PIPER_INIT, READY, CAPTURING, WAITING, EXHAUSTED, OVER };
LedState g_led_state = LedState::PIPER_INIT;

void drawLedIndicator(cv::Mat& canvas) {
    cv::Scalar color; string text;
    switch (g_led_state) {
        case LedState::PIPER_INIT: color={255,0,0};   text="PIPER INIT"; break;  // BGR: RGB 蓝 0000ff
        case LedState::READY:      color={0,255,0};   text="READY (breath)"; break;   // 25 全绿呼吸
        case LedState::CAPTURING:  color={0,255,0};   text="CAPTURING (cross)"; break; // 绿白十字常亮
        case LedState::WAITING:    color={0,128,255}; text="WAITING"; break;    // BGR: RGB 橙 ff8000
        case LedState::EXHAUSTED:  color={0,0,255};   text="EXHAUSTED"; break;  // BGR: RGB 红 ff0000
        case LedState::OVER:       color={255,0,255}; text="OVER"; break;
    }
    int cw=canvas.cols, ch=canvas.rows;
    int bx=cw-160, by=ch-35;
    cv::rectangle(canvas, cv::Rect(bx, by, 20, 20), color, -1);
    cv::putText(canvas, text, cv::Point(bx+26, by+15), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}

// ================== M5Stack 状态灯 (Atom Matrix 5x5, 串口) ==================
HANDLE g_led_upper = INVALID_HANDLE_VALUE;   // upper M5Stack 串口
HANDLE g_led_lower = INVALID_HANDLE_VALUE;   // lower M5Stack 串口

bool openLedSerial(HANDLE& h, const string& port, int baud) {
    if (h != INVALID_HANDLE_VALUE) { CloseHandle(h); h = INVALID_HANDLE_VALUE; }
    string path = "\\\\.\\" + port;
    HANDLE hh = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hh == INVALID_HANDLE_VALUE) {
        cerr << "[M5Stack] Cannot open " << port << " (error " << GetLastError() << ")" << endl;
        return false;
    }
    DCB dcb{}; dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hh, &dcb)) { CloseHandle(hh); return false; }
    dcb.BaudRate = baud; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(hh, &dcb)) { CloseHandle(hh); return false; }
    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = 50; to.ReadTotalTimeoutConstant = 50; to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 300; to.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(hh, &to);
    h = hh;
    cout << "[M5Stack] Opened " << port << " @ " << baud << endl;
    return true;
}

void sendLedCmd(HANDLE h, const string& cmd) {
    if (h == INVALID_HANDLE_VALUE) return;
    string line = cmd + "\n";
    DWORD written = 0;
    if (!WriteFile(h, line.c_str(), (DWORD)line.size(), &written, NULL) || written != line.size())
        cerr << "[M5Stack] Write failed." << endl;
}

extern string g_arm;   // 当前机械臂 ("upper"/"lower"), 定义在下方 Piper arm state 区

// 状态 → LED 图案命令 (每次切换仅一条指令; 呼吸动画由固件 MODE BREATH 实现)
string pixCrossCmd() {
    // 十字: 外臂+中心 (2,10,12,14,22) = 白; 内 3x3 小十字 (7,11,13,17) = 绿; 其余黑
    string px[25]; for (auto& v : px) v = "000000";
    for (int i : {7, 11, 13, 17}) px[i] = "00ff00";
    for (int i : {2, 10, 12, 14, 22}) px[i] = "ffffff";
    string cmd = "PIX"; for (auto& v : px) cmd += " " + v; return cmd;
}
string ledPatternCmd(LedState s) {
    switch (s) {
        case LedState::PIPER_INIT: return "MODE ALL 0000ff";      // 25 全亮蓝
        case LedState::READY:      return "MODE BREATH 00ff00";   // 25 全绿呼吸 (固件动画)
        case LedState::CAPTURING:  return pixCrossCmd();          // 绿白十字常亮
        case LedState::WAITING:    return "MODE ALL ff8000";      // 25 全亮橙
        case LedState::EXHAUSTED:  return "MODE ALL ff0000";      // 25 全亮红
        case LedState::OVER:       return "MODE FLOW";            // 斜向彩流
        default: return "";
    }
}

// 每臂各自的 M5Stack LED 状态 (两块设备独立显示各自机械臂的状态)
LedState g_led_state_upper = LedState::PIPER_INIT;
LedState g_led_state_lower = LedState::PIPER_INIT;

void sendLedPatternTo(HANDLE h, LedState s) {
    string cmd = ledPatternCmd(s);
    if (cmd.empty()) return;
    sendLedCmd(h, cmd);
}

// 每臂成功录制数 / OVER 标志 (运行期独立计数, 跨切换保持, 重启归零)
int g_recorded_upper = 0, g_recorded_lower = 0;
bool g_over_upper = false, g_over_lower = false;

int& recordedOf() { return (g_arm == "upper") ? g_recorded_upper : g_recorded_lower; }
bool& overOf()    { return (g_arm == "upper") ? g_over_upper : g_over_lower; }

// 设置全局状态 (UI 指示器), 只更新"当前臂"对应的设备
void setLedState(LedState s) {
    if (g_led_state == s) return;
    g_led_state = s;
    LedState& arm_st = (g_arm == "upper") ? g_led_state_upper : g_led_state_lower;
    HANDLE arm_dev = (g_arm == "upper") ? g_led_upper : g_led_lower;
    if (arm_st != s) {
        arm_st = s;
        cout << "[M5Stack] " << g_arm << " -> " << ledPatternCmd(s) << endl;
        sendLedPatternTo(arm_dev, s);
    }
}

// ================== Piper arm state (Master only) ==================
SOCKET g_piper_sock = INVALID_SOCKET;
string g_arm = "upper";
int g_upper_idx = 0, g_lower_idx = 0;
bool g_upper_done = false, g_lower_done = false;
bool g_recording_enabled = false;

// ================== AutoMove / OVER (capture.yaml) ==================
bool g_enable_auto_move = false;          // READY 超时自动跳过
double g_click_window = 0.5;              // READY 等待 SPACE 的最大时长 (s)
int g_num_targets_per_arm = 0;            // 每臂成功录制数上限 (0 = 不限制)
bool g_show_over = false;                 // 当前臂的 OVER 显示标志 (随切换加载)
atomic<int64_t> g_ready_since_us{0};      // READY 起始时刻 (us since epoch, 跨线程)

// ================== Capture delay (CAPTURING → 延迟录制) ==================
double g_capture_delay = 0.5;              // SPACE 后延迟录制的秒数 (capture.yaml)
atomic<bool> g_delay_pending{false};       // delay 等待中 (LED 显示 CAPTURING)
atomic<int64_t> g_delay_start_us{0};       // delay 起点
atomic<int64_t> g_delay_deadline_us{0};    // delay 终点
atomic<bool> g_trigger_pending{false};     // delay 结束 → 待触发录制
double g_last_delay_s = 0.0;               // 最近一次实际 delay 时长 (日志)

int64_t nowUs() {
    return chrono::duration_cast<chrono::microseconds>(
        chrono::steady_clock::now().time_since_epoch()).count();
}

// OVER 判断 (初版): 每次程序运行, 两臂从零开始独立计数, 不与任何 sentry 比较
// (h5 sentry 无法区分机械臂, piper sentry 包含跳过目标, 均不作为判据)
atomic<bool> g_init_ok{false};     // Master→Slave: all connections ready
atomic<bool> g_piper_busy{false};
struct ArmTransform { Pt3 tool_t, tool_r, ccs_t, ccs_r; };
ArmTransform g_xf_upper, g_xf_lower;
string g_arm_pose_yml;   // 加载的 arm pose yaml (cfg/... 相对路径, UI 水印显示)
string g_participant_id; // participant_id (UI 水印显示)
vector<array<double,3>> g_targets_upper, g_targets_lower;
string g_gaze_dir;
struct ArmPose { double x,y,z, qx,qy,qz,qw, alpha,beta,gamma; bool valid=false; };
ArmPose g_last_piper_pose;
Pt3 g_tool_ccs_pos{};
bool g_tool_ccs_valid = false;

// ================== CameraContext ==================
enum class CamStatus { INIT, OPENED, WAITING_TRIGGER, STREAMING, ERROR_ };
struct LogEntry { string filename; int64_t blockID; int64_t timestamp; int width; int height; };

struct CameraContext {
    int index; string id; string save_base_dir; BaslerCamera cam{""};
    bool is_mono = true;
    thread capture_thread, copy_thread;
    atomic<bool> running{true}, recording{false};
    cv::Mat latest_frame; FrameMeta latest_meta; mutex frame_mtx;
    vector<cv::Mat> ram_buffer; vector<FrameMeta> meta_buffer;
    int total_record_frames = 0; atomic<bool> dump_ready{false};
    queue<pair<cv::Mat, FrameMeta>> copy_queue;  // [Fix] 队列存已拷贝 Mat, 回调内同步拷贝
    mutex copy_mtx; condition_variable copy_cv;
    string temp_dir, log_file_path; ofstream log_stream;
    atomic<int> captured_frames{0}, recorded_frames{0};
    atomic<CamStatus> status{CamStatus::INIT}; string status_msg = "Initializing";
    int64_t frame_offset = 0; bool offset_initialized = false;
    static atomic<int64_t> master_first_id; static atomic<bool> master_set;
    atomic<int64_t> last_block_id{-1};
    atomic<chrono::steady_clock::time_point> last_frame_time{chrono::steady_clock::now()};
    atomic<bool> has_streamed{false};
    atomic<int> max_queue_size{0};
    int64_t first_recorded_block_id=-1, last_recorded_block_id=-1;
    chrono::steady_clock::time_point first_frame_time;
    atomic<int> dropped_frames{0}; int64_t prev_block_id = -1;
    chrono::steady_clock::time_point recording_end_time, dump_start_time, dump_end_time;
    chrono::steady_clock::time_point jpg_start_time, jpg_end_time;
    double recover2ram_s=0, wait4disk_s=0, ram2disk_s=0;
    string hdf5_dir;
    HANDLE shm_handle = NULL; uint8_t* shm_base = nullptr;
    CameraContext(int idx, string cam_id, string save_dir)
        : index(idx), id(cam_id), save_base_dir(save_dir), cam(cam_id) {}
};
atomic<int64_t> CameraContext::master_first_id(-1);
atomic<bool> CameraContext::master_set(false);
vector<shared_ptr<CameraContext>> cam_ctxs;

// ================== Global state ==================
atomic<bool> g_fault_active{false}; atomic<int> g_faulty_cam{-1};
atomic<bool> g_fault_on_master{false};
chrono::steady_clock::time_point g_ready_time, g_fault_time;
int g_chunk_idx = 0; atomic<int> g_frame_offset{0};
vector<string> g_participant_roots; string g_sentry_root;
int g_hdf5_chunk_capacity = 2000;
int g_cam_w = 0, g_cam_h = 0;          // 相机分辨率 (main 加载, precreateSerial 用)

// ================== i 键: 预创建 HDF5 (双机握手 + 串行创建 + 进度 UI) ==================
atomic<bool> g_precreating{false};      // 创建状态: 全屏进度 UI, 屏蔽所有按键
atomic<int>  g_pre_phase{0};            // 1=握手检查 2=创建中
atomic<int>  g_pre_total{0}, g_pre_done{0};
atomic<bool> g_pre_local_done{false};   // 本机创建完成
atomic<bool> g_pre_peer_clear{false};   // master: slave 检查通过 (无 h5)
atomic<bool> g_pre_peer_reject{false};  // master: slave 已有 h5 → 放弃
atomic<bool> g_pre_peer_done{false};    // master: slave 创建完成
thread g_pre_thread;                    // 本机创建线程 (cleanup 时 join)
chrono::steady_clock::time_point g_pre_t0;   // 握手阶段起始 (超时保护)
int g_sentry_mismatch_count = 0, g_consecutive_faults = 0;
atomic<bool> g_syncing{false};
string g_session_log_path; int g_recording_number = 0; ofstream g_session_log;
// Timing metrics (per-recording, updated after each dump)
double g_arm_stage_s = 0;       // ARM stage wall time
double g_master_hdf5_s = 0;    // Master HDF5 write (hdf5 phase in par)
double g_slave_hdf5_s = 0;     // Slave HDF5 write (hdf5 phase in par)
double g_wait_slave_hdf5_s = 0; // Wait for Slave HDF5_DONE
double g_gaze_forward_s = 0;    // GAZE forward + GAZE_DONE
double g_sentry_sync_s = 0;     // Sentry handshake
double g_recording_end_to_end_s = 0; // Total from SPACE to SPACE-ready (delay + record + arm&h5 + sync)
atomic<int64_t> g_e2e_start_us{0};   // SPACE 按下时刻 (End-to-end 起点; slave 由 TRIGGER 兜底)
atomic<int> g_exc_fatal{0}, g_exc_error{0}, g_exc_warn{0}, g_exc_info{0};
int64_t g_peer_first_block_id = -1;
atomic<int> g_enlarged_cam{-1};
int g_win_w=1224, g_win_h=1024, g_left_w=0, g_right_x=0, g_right_w=0;
int g_thumb_w=0, g_thumb_h=0;
bool g_is_master = true;
string g_master_ip;

void logException(const string& level, const string& source, const string& msg) {
    auto t = chrono::system_clock::now();
    auto tt = chrono::system_clock::to_time_t(t);
    auto ms = chrono::duration_cast<chrono::milliseconds>(t.time_since_epoch()) % 1000;
    char tb[16]; strftime(tb, sizeof(tb), "%H:%M:%S", localtime(&tt));
    string ts = string(tb)+"."+to_string(ms.count()/100)+to_string((ms.count()/10)%10)+to_string(ms.count()%10);
    string line = "> **["+level+"]** `"+ts+"` | "+source+" | "+msg;
    if (level=="FATAL") { g_exc_fatal++; cerr << line << endl; }
    else if (level=="ERROR") { g_exc_error++; cerr << line << endl; }
    else if (level=="WARN") { g_exc_warn++; cout << line << endl; }
    else { g_exc_info++; cout << line << endl; }
    if (g_session_log.is_open()) g_session_log << line << "\n" << flush;
}

// ================== TCP helpers ==================
bool recvLine(SOCKET sock, string& line, int timeout_ms = 3000) {
    DWORD to = timeout_ms;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
    char buf[256]; string acc;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int n = recv(sock, buf, sizeof(buf)-1, 0);
        if (n <= 0) return false;
        buf[n] = '\0'; acc += buf;
        size_t nl = acc.find('\n');
        if (nl != string::npos) { line = acc.substr(0,nl);
            if (!line.empty() && line.back()=='\r') line.pop_back(); return true; }
    }
    return false;
}

// 带状态返回的 recvLine: 0=收到一行, 1=超时(无数据), 2=连接关闭/错误
int recvLineStatus(SOCKET sock, string& line, int timeout_ms = 3000) {
    DWORD to = timeout_ms;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
    char buf[256]; string acc;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int n = recv(sock, buf, sizeof(buf)-1, 0);
        if (n == 0) return 2;                     // 对端正常关闭
        if (n < 0) {
#ifdef _WIN32
            int e = WSAGetLastError();
            if (e == WSAETIMEDOUT) return 1;      // 空闲超时, 不是断连
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1;
#endif
            return 2;                              // 其他错误视为断连
        }
        buf[n] = '\0'; acc += buf;
        size_t nl = acc.find('\n');
        if (nl != string::npos) { line = acc.substr(0,nl);
            if (!line.empty() && line.back()=='\r') line.pop_back(); return 0; }
    }
    return 1;
}
bool sendLineRaw(SOCKET sock, const string& msg) {
    string data = msg + "\n";
    return send(sock, data.c_str(), (int)data.length(), 0) > 0;
}
bool sendLine(SOCKET sock, const string& msg) {
    lock_guard<mutex> lk(g_cmd_send_mtx);
    return sendLineRaw(sock, msg);
}

// ================== Piper: parse MOVED response ==================
bool parsePoseResponse(const string& resp, string& arm, ArmPose& pose) {
    if (resp.rfind("MOVED:",0)!=0 && resp.rfind("POSE:",0)!=0) return false;
    size_t c1=resp.find(':'), c2=resp.find(':',c1+1);
    if (c1==string::npos||c2==string::npos) return false;
    arm = resp.substr(c1+1,c2-c1-1);
    string vals=resp.substr(c2+1); vector<double> nums;
    stringstream ss(vals); string token;
    while(getline(ss,token,',')) { try{nums.push_back(stod(token));}catch(...){return false;} }
    if(nums.size()!=10) return false;
    pose.x=nums[0];pose.y=nums[1];pose.z=nums[2];
    pose.qx=nums[3];pose.qy=nums[4];pose.qz=nums[5];pose.qw=nums[6];
    pose.alpha=nums[7];pose.beta=nums[8];pose.gamma=nums[9];
    pose.valid=true; return true;
}

// ================== Piper: compute tool in CCS ==================
void computeToolCcs(const string& arm_name) {
    auto& xf = (arm_name=="upper") ? g_xf_upper : g_xf_lower;
    Pose flange{{g_last_piper_pose.x,g_last_piper_pose.y,g_last_piper_pose.z},
                {g_last_piper_pose.qx,g_last_piper_pose.qy,g_last_piper_pose.qz,g_last_piper_pose.qw}};
    Pose tccs = armToolToCamPose(flange, xf.tool_t, xf.tool_r, xf.ccs_t, xf.ccs_r);
    g_tool_ccs_pos = tccs.pos;
    g_tool_ccs_valid = true;
}

// ================== Piper: zero arm ==================
bool zeroArm(const string& arm_name) {
    g_piper_busy = true;
    cout << "[Piper] Zeroing " << arm_name << "..." << endl;
    string cmd = "MOVE_JOINTS:" + arm_name + ":0.0,0.0,0.0,0.0,0.0,0.0";
    if (!sendLineRaw(g_piper_sock, cmd)) { g_piper_busy = false; return false; }
    string resp, resp_arm; ArmPose pose;
    if (recvLine(g_piper_sock, resp, 30000)) {
        if (parsePoseResponse(resp, resp_arm, pose)) {
            g_last_piper_pose = pose;
            computeToolCcs(arm_name);
            cout << "[Piper] " << arm_name << " zeroed" << endl;
            g_piper_busy = false; return true;
        }
        cerr << "[Piper] " << arm_name << " zero FAIL: " << resp << endl;
    } else { cerr << "[Piper] " << arm_name << " zero timeout" << endl; }
    g_piper_busy = false; return false;
}

// ================== Piper: update sentry ==================
void updatePiperSentry() {
    ofstream sf(g_gaze_dir + "/sentry.txt");
    sf << "upper:" << g_upper_idx << "\nlower:" << g_lower_idx << "\n";
}
// ================== Piper: sync state to Slave via cmd_port ==================
void syncPiperToSlave(bool send_init_ok=false) {
    if(g_cmd_sock==INVALID_SOCKET) return;
    char buf[64];
    snprintf(buf,sizeof(buf),"PIPER:active:%s",g_arm.c_str());
    sendLineRaw(g_cmd_sock,buf); this_thread::sleep_for(chrono::milliseconds(50));
    snprintf(buf,sizeof(buf),"PIPER:upper:%d:%s",g_upper_idx,g_upper_done?"done":"ok");
    sendLineRaw(g_cmd_sock,buf); this_thread::sleep_for(chrono::milliseconds(50));
    snprintf(buf,sizeof(buf),"PIPER:lower:%d:%s",g_lower_idx,g_lower_done?"done":"ok");
    sendLineRaw(g_cmd_sock,buf); this_thread::sleep_for(chrono::milliseconds(50));
    if(send_init_ok) {
        sendLineRaw(g_cmd_sock,"INIT_OK"); this_thread::sleep_for(chrono::milliseconds(50));
        cout<<"[Init] INIT_OK sent to Slave."<<endl;
    }
}

// ================== Gaze server (Master) ==================
void gazeServerWorker(int gaze_port) {
    g_gaze_listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_gaze_listen_sock == INVALID_SOCKET) return;
    int opt=1; setsockopt(g_gaze_listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
    sockaddr_in sa{}; sa.sin_family=AF_INET; sa.sin_port=htons(gaze_port);
    sa.sin_addr.s_addr=INADDR_ANY;
    ::bind(g_gaze_listen_sock, (sockaddr*)&sa, sizeof(sa));
    listen(g_gaze_listen_sock, 1);
    {DWORD to=500;setsockopt(g_gaze_listen_sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&to,sizeof(to));}  // accept 可中断
    cout << "[Gaze] Master listening TCP ::" << gaze_port << endl;
    while (global_running) {
        sockaddr_in ca; socklen_t cl=sizeof(ca);
        SOCKET cs = accept(g_gaze_listen_sock, (sockaddr*)&ca, &cl);
        if (cs == INVALID_SOCKET) { if (!global_running) break; continue; }  // 超时轮询, 退出时结束
        g_gaze_sock = cs; g_gaze_connected = true;
        cout << "[Gaze] Slave connected." << endl;
        // Loop: wait for main thread to signal new gaze data, then send
        while (global_running) {
            this_thread::sleep_for(chrono::milliseconds(50));
            if (g_gaze_need_send.exchange(false)) {
                char buf[128];
                snprintf(buf,sizeof(buf),"GAZE:%.6f,%.6f,%.6f\n", g_gaze_x.load(), g_gaze_y.load(), g_gaze_z.load());
                lock_guard<mutex> lk(g_gaze_send_mtx);
                if (send(g_gaze_sock, buf, (int)strlen(buf), 0) <= 0) {cerr<<"[Gaze] Send failed - exiting."<<endl;global_running=false;break;}
                string ack; recvLine(g_gaze_sock, ack, 5000);
                cout << "[Gaze] Sent ("<<g_gaze_x<<","<<g_gaze_y<<","<<g_gaze_z<<") ack="<<ack<<endl;
            }
        }
        g_gaze_connected = false; closesocket(g_gaze_sock); g_gaze_sock = INVALID_SOCKET;
    }
}

// ================== Gaze client (Slave) ==================
void gazeClientWorker(const string& master_ip, int gaze_port) {
    while (global_running) {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) { this_thread::sleep_for(chrono::seconds(2)); continue; }
        sockaddr_in sa{}; sa.sin_family=AF_INET; sa.sin_port=htons(gaze_port);
        inet_pton(AF_INET, master_ip.c_str(), &sa.sin_addr);
        if (connect(sock, (sockaddr*)&sa, sizeof(sa)) != 0) {
            closesocket(sock); this_thread::sleep_for(chrono::seconds(2)); continue;
        }
        g_gaze_connected = true;
        cout << "[Gaze] Slave connected to Master." << endl;
        while (global_running) {
            string line;
            int st = recvLineStatus(sock, line, 300000);
            if (st == 2) { cerr<<"[Gaze] Connection closed by Master - reconnecting."<<endl; break; }  // 真断连 → 重连, 不退出
            if (st == 1) continue;  // 空闲超时 → 继续等待
            if (line.rfind("GAZE:",0) == 0) {
                double gx,gy,gz; sscanf_s(line.c_str()+5, "%lf,%lf,%lf", &gx, &gy, &gz);
                g_gaze_x=gx; g_gaze_y=gy; g_gaze_z=gz;
                g_gaze_ready = true;
                g_gaze_need_send = true;  // signal main thread ARM stage loop
                sendLineRaw(sock, "GAZE_ACK");
            }
        }
        g_gaze_connected = false; closesocket(sock);
        if (!global_running) break;
        this_thread::sleep_for(chrono::seconds(1));
    }
}

// ================== Piper: arm result enum ==================
enum class ArmResult { ARM_OK, ARM_EXHAUSTED, ARM_ERROR };

// ================== Piper: move arm to target (auto-skip on failure) ==================
ArmResult moveArmToTarget() {
    auto& tgt = (g_arm=="upper") ? g_targets_upper : g_targets_lower;
    int& idx = (g_arm=="upper") ? g_upper_idx : g_lower_idx;
    int total = (int)tgt.size();
    while (idx < total) {
        auto& pt = tgt[idx];
        char cmd[128]; snprintf(cmd,sizeof(cmd),"MOVE_TO:%s:%.6f,%.6f,%.6f", g_arm.c_str(), pt[0], pt[1], pt[2]);
        cout << "[Piper] Moving " << g_arm << " #" << (idx+1) << "/" << total << endl;
        g_piper_busy = true;
        if (!sendLineRaw(g_piper_sock, cmd)) { g_piper_busy = false; return ArmResult::ARM_ERROR; }
        string resp, resp_arm; ArmPose pose;
        if (!recvLine(g_piper_sock, resp, 60000)) { g_piper_busy = false; return ArmResult::ARM_ERROR; }
        if (parsePoseResponse(resp, resp_arm, pose)) {
            g_last_piper_pose = pose;
            computeToolCcs(g_arm);
            g_gaze_x = g_tool_ccs_pos.x; g_gaze_y = g_tool_ccs_pos.y; g_gaze_z = g_tool_ccs_pos.z;
            g_gaze_ready = true; g_gaze_need_send = true;
            g_piper_busy = false;
            g_ready_since_us.store(nowUs());   // AutoMove: READY 计时起点
            return ArmResult::ARM_OK;
        }
        if (resp.rfind("ERROR:",0) == 0) {
            cout << "[Piper] SKIPPED #" << (idx+1) << " (no solution)" << endl;
            idx++; updatePiperSentry();
            continue;
        }
        cerr << "[Piper] Bad response: " << resp << endl;
        g_piper_busy = false; return ArmResult::ARM_ERROR;
    }
    bool& done = (g_arm=="upper") ? g_upper_done : g_lower_done;
    done = true; updatePiperSentry();
    cout << "[Piper] " << g_arm << " all targets exhausted!" << endl;
    g_piper_busy = false;
    return ArmResult::ARM_EXHAUSTED;
}

// ================== Camera: instantTrigger ==================
void instantTrigger() {
    global_record_start_time = chrono::steady_clock::now();
    g_exc_fatal=0; g_exc_error=0; g_exc_warn=0; g_exc_info=0;
    for (auto& ctx : cam_ctxs) {
        { lock_guard<mutex> lock(ctx->copy_mtx); while(!ctx->copy_queue.empty()) ctx->copy_queue.pop(); }
        ctx->recorded_frames.store(0, memory_order_relaxed);
        ctx->dump_ready.store(false, memory_order_relaxed);
        ctx->recording.store(true, memory_order_release);
        ctx->max_queue_size=0; ctx->first_recorded_block_id=-1; ctx->last_recorded_block_id=-1;
        ctx->dropped_frames=0; ctx->prev_block_id=-1;
        ctx->recover2ram_s=0; ctx->wait4disk_s=0; ctx->ram2disk_s=0;
    }
}

// ================== Camera: copyWorker ==================
void copyWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        pair<cv::Mat, FrameMeta> task;
        { unique_lock<mutex> lock(ctx->copy_mtx);
          ctx->copy_cv.wait(lock,[&]{return !ctx->copy_queue.empty()||!ctx->running;});
          if (!ctx->running && ctx->copy_queue.empty()) break;
          task=ctx->copy_queue.front(); ctx->copy_queue.pop(); }
        // [Fix] 队列里的 Mat 已在回调内同步拷贝, 这里只做消费
        { lock_guard<mutex> lock(ctx->frame_mtx); ctx->latest_frame=task.first; ctx->latest_meta=task.second; }
    }
}

// ================== Camera: captureWorker ==================
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma,
                   double exp_time, bool use_hw_trigger, bool enable_offset) {
    TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
    if (!ctx->cam.open(mode)) { ctx->status=CamStatus::ERROR_; ctx->copy_cv.notify_all(); return; }
    ctx->is_mono = ctx->cam.isMono();
    try { if(!use_hw_trigger) ctx->cam.setFrameRate(fps);
          ctx->cam.setGain(gain); ctx->cam.setGamma(gamma); ctx->cam.setExposureTime(exp_time); } catch(...){}
    struct GrabState { int64_t frame_counter=0; };
    auto state=make_shared<GrabState>();
    ctx->cam.setFrameCallback([ctx,state,use_hw_trigger,enable_offset](const Pylon::CBaslerUniversalGrabResultPtr& ptr, FrameMeta meta){
        state->frame_counter++; ctx->status=CamStatus::STREAMING;
        if(!ctx->offset_initialized && state->frame_counter>1){
            if(use_hw_trigger||!enable_offset){ctx->frame_offset=0;ctx->offset_initialized=true;}
            else{if(!CameraContext::master_set.exchange(true)){CameraContext::master_first_id=meta.blockID;ctx->frame_offset=0;ctx->offset_initialized=true;}
            else{int retry=0;while(CameraContext::master_first_id==-1&&retry<100){this_thread::sleep_for(chrono::milliseconds(10));retry++;}
            if(CameraContext::master_first_id!=-1){ctx->frame_offset=meta.blockID-CameraContext::master_first_id.load();ctx->offset_initialized=true;}}}
        }
        if(ctx->offset_initialized){
            meta.blockID=meta.blockID-ctx->frame_offset;ctx->captured_frames++;
            if (ctx->recording) {
                int seq = ctx->recorded_frames.load(std::memory_order_relaxed);
                if (seq < ctx->total_record_frames) {
                    void* pBuffer = ptr->GetBuffer();
                    size_t payload_size = ptr->GetWidth() * ptr->GetHeight();
                    // 溢出防护 (保留)
                    if (payload_size <= ctx->ram_buffer[seq].total() * ctx->ram_buffer[seq].elemSize()) {
                        memcpy(ctx->ram_buffer[seq].data, pBuffer, payload_size);
                        ctx->meta_buffer[seq] = meta;
                        if (seq == 0) { ctx->first_recorded_block_id = meta.blockID; ctx->first_frame_time = chrono::steady_clock::now(); }
                        ctx->last_recorded_block_id = meta.blockID;
                        if (ctx->prev_block_id != -1) { int64_t diff = meta.blockID - ctx->prev_block_id; if (diff > 1) ctx->dropped_frames += (int)(diff - 1); }
                        ctx->prev_block_id = meta.blockID;
                        int next_seq = seq + 1;
                        ctx->recorded_frames.store(next_seq, std::memory_order_relaxed);
                        { lock_guard<mutex> lock(ctx->frame_mtx); ctx->latest_frame = ctx->ram_buffer[seq]; ctx->latest_meta = meta; }
                        if (next_seq == ctx->total_record_frames) {
                            ctx->recording = false;
                            ctx->dump_ready = true;
                            ctx->recording_end_time = chrono::steady_clock::now();
                        }
                    }
                }
                ctx->last_block_id.store(meta.blockID, memory_order_relaxed);
                ctx->last_frame_time.store(chrono::steady_clock::now(), memory_order_relaxed);
                ctx->has_streamed.store(true, memory_order_relaxed);
            } else {
                // [Fix] 回调内同步拷贝
                cv::Mat temp(ptr->GetHeight(), ptr->GetWidth(), CV_8UC1, ptr->GetBuffer());
                cv::Mat clone_img = temp.clone();
                lock_guard<mutex> lock(ctx->copy_mtx);
                if (ctx->copy_queue.size() < 2) { ctx->copy_queue.push({clone_img, meta}); ctx->copy_cv.notify_one(); }
                ctx->last_block_id.store(meta.blockID, memory_order_relaxed);
                ctx->last_frame_time.store(chrono::steady_clock::now(), memory_order_relaxed);
                ctx->has_streamed.store(true, memory_order_relaxed);
            }
        }
    });
    if(!ctx->cam.start()){ctx->status=CamStatus::ERROR_;ctx->copy_cv.notify_all();return;}
    ctx->status_msg=use_hw_trigger?"HW WAITING":"STREAMING";
    while(ctx->running) this_thread::sleep_for(chrono::milliseconds(50));
    ctx->cam.close();
}

// ================== UI (from hdf5_multi_process.cpp) ==================
int getNextCalibCounter(const std::string& save_dir) {
    int max_counter = -1;
    if (!fs::exists(save_dir)) return 0;
    for (auto& e : fs::directory_iterator(save_dir)) {
        if (e.path().extension() == ".jpg") {
            try { string stem = e.path().stem().string();
                size_t last_underscore = stem.find_last_of('_');
                if (last_underscore != string::npos) max_counter = max(max_counter, stoi(stem.substr(last_underscore + 1)));
            } catch (...) {}
        }
    }
    return max_counter + 1;
}
void updateLayout() {
    g_left_w=g_win_h*2/5; g_right_x=g_left_w; g_right_w=g_win_w-g_left_w;
    g_thumb_w=g_left_w/2; g_thumb_h=g_win_h/5;
}
void onMouse(int event, int x, int y, int, void*) {
    if(event!=cv::EVENT_LBUTTONDOWN||x>=g_left_w) return;
    int col=x/g_thumb_w, row=y/g_thumb_h, idx=row*2+col;
    int n=(int)cam_ctxs.size();
    if(idx>=0&&idx<n){int prev=g_enlarged_cam.load();g_enlarged_cam.store((prev==idx)?-1:idx);}
}
void renderThumbnailGrid(cv::Mat& canvas, int selected_idx, bool is_recording,
                         const chrono::steady_clock::time_point& record_start_time, int total_record_frames) {
    int n=(int)cam_ctxs.size();
    for(int i=0;i<10;++i){int row=i/2,col=i%2;int x=col*g_thumb_w,y=row*g_thumb_h;cv::Rect roi(x,y,g_thumb_w,g_thumb_h);
        if(i<n){cv::Mat local_raw;{lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);local_raw=cam_ctxs[i]->latest_frame;}
            cv::Mat cell;if(!local_raw.empty()){if(cam_ctxs[i]->is_mono)cv::cvtColor(local_raw,cell,cv::COLOR_GRAY2RGB);else cv::cvtColor(local_raw,cell,cv::COLOR_BayerRG2RGB);
                double scale=min((double)g_thumb_w/cell.cols,(double)g_thumb_h/cell.rows);int dw=(int)(cell.cols*scale),dh=(int)(cell.rows*scale);
                cv::Mat resized;cv::resize(cell,resized,cv::Size(dw,dh));cell=cv::Mat::zeros(g_thumb_h,g_thumb_w,CV_8UC3);
                int ox=(g_thumb_w-dw)/2,oy=(g_thumb_h-dh)/2;resized.copyTo(cell(cv::Rect(ox,oy,dw,dh)));}
            else{cell=cv::Mat::zeros(g_thumb_h,g_thumb_w,CV_8UC3);int bl=0;cv::Size ts=cv::getTextSize(cam_ctxs[i]->status_msg,cv::FONT_HERSHEY_SIMPLEX,0.5,1,&bl);
                cv::putText(cell,cam_ctxs[i]->status_msg,cv::Point((g_thumb_w-ts.width)/2,(g_thumb_h+ts.height)/2),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255),1);}
            if(is_recording){int r=6;cv::circle(cell,cv::Point(g_thumb_w-r*2,r*2),r,cv::Scalar(0,0,255),-1,cv::LINE_AA);
                int cf=cam_ctxs[i]->recorded_frames.load(memory_order_relaxed);cv::putText(cell,to_string(cf)+"/"+to_string(total_record_frames),cv::Point(4,14),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(0,0,0),2);
                cv::putText(cell,to_string(cf)+"/"+to_string(total_record_frames),cv::Point(4,14),cv::FONT_HERSHEY_SIMPLEX,0.35,cv::Scalar(0,255,0),1);}
            string label=cam_ctxs[i]->id;int bl=0;cv::Size ts=cv::getTextSize(label,cv::FONT_HERSHEY_SIMPLEX,0.4,1,&bl);
            cv::putText(cell,label,cv::Point((g_thumb_w-ts.width)/2,g_thumb_h-5),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,0),3);
            cv::putText(cell,label,cv::Point((g_thumb_w-ts.width)/2,g_thumb_h-5),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255,255,255),1);
            cell.copyTo(canvas(roi));if(i==selected_idx)cv::rectangle(canvas,roi,cv::Scalar(0,255,0),2);}
        else{canvas(roi)=cv::Scalar(0,0,0);}
    }
}
// ================== i 键: 预创建 HDF5 (检查/创建, 由创建线程调用) ==================
// 任一相机目录下已存在 .h5 → true (双机握手时双方各自检查)
bool anyLocalH5() {
    for (auto& ctx : cam_ctxs) {
        if (!fs::exists(ctx->hdf5_dir)) continue;
        try {
            for (auto& e : fs::directory_iterator(ctx->hdf5_dir))
                if (e.path().extension() == ".h5") return true;
        } catch (...) {}   // 目录不可访问等异常 → 视为无 h5, 由后续创建失败暴露
    }
    return false;
}

// 数据集创建属性: 创建时即分配全部存储空间 (文件扩展到满尺寸),
// 且不写填充值 (避免逐字节清零 9.7GB; 未写区域由 valid 数据集标记)
H5::DSetCreatPropList allocEarlyPl() {
    H5::DSetCreatPropList pl;
    pl.setAllocTime(H5D_ALLOC_TIME_EARLY);
    pl.setFillTime(H5D_FILL_TIME_NEVER);
    return pl;
}

// 串行创建 25×N 个 h5 (H5 文件必须逐个创建, 不可并行); 进度写入 g_pre_done/g_pre_total
void precreateSerial() {
    int created = 0;
    g_pre_total = (int)cam_ctxs.size() * 25;
    g_pre_done = 0;
    for (auto& ctx : cam_ctxs)
        for (int ci = 0; ci < 25; ++ci) {
            stringstream pss; pss << ctx->hdf5_dir << "/" << setw(4) << setfill('0') << ci << ".h5";
            if (!fs::exists(pss.str())) {   // 双端预检保证为空, 此处仅防御
                try {
                    H5::H5File f(pss.str(), H5F_ACC_TRUNC);
                    H5::DSetCreatPropList pl = allocEarlyPl();
                    // 顺序关键: 小数据集必须先创建 (文件布局在 raw_image 之前) —
                    // 否则首写 gaze/valid (位于 10GB raw 区之后) 越过 NTFS 有效数据长度,
                    // close 时触发整个空洞的同步零填充 (~115ms/GB, 首录尖峰根因)
                    hsize_t gd[2] = {(hsize_t)g_hdf5_chunk_capacity, 3};
                    f.createDataSet("gaze_target", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, gd), pl);
                    hsize_t vd[1] = {(hsize_t)g_hdf5_chunk_capacity};
                    f.createDataSet("valid", H5::PredType::NATIVE_UINT8, H5::DataSpace(1, vd), pl);
                    hsize_t rd[3] = {(hsize_t)g_hdf5_chunk_capacity, (hsize_t)g_cam_h, (hsize_t)g_cam_w};
                    f.createDataSet("raw_image", H5::PredType::NATIVE_UINT8, H5::DataSpace(3, rd), pl);
                    created++;
                } catch (const H5::Exception& e) {
                    logException("ERROR", "hdf5:precreate", e.getCDetailMsg());
                } catch (...) {
                    logException("ERROR", "hdf5:precreate", "unknown exception");
                }
            }
            g_pre_done++;
        }
    cout << "[HDF5] Pre-create: " << created << " files created." << endl;
}

void renderEnlargedView(cv::Mat& canvas, int cam_idx, bool is_recording,
                        const chrono::steady_clock::time_point& record_start_time, int total_record_frames) {
    cv::Rect right_roi(g_right_x,0,g_right_w,g_win_h);if(cam_idx<0||cam_idx>=(int)cam_ctxs.size()){canvas(right_roi)=cv::Scalar(0,0,0);return;}
    cv::Mat local_raw;{lock_guard<mutex> lock(cam_ctxs[cam_idx]->frame_mtx);local_raw=cam_ctxs[cam_idx]->latest_frame;}
    if(local_raw.empty()){canvas(right_roi)=cv::Scalar(0,0,0);return;}
    cv::Mat img;if(cam_ctxs[cam_idx]->is_mono)cv::cvtColor(local_raw,img,cv::COLOR_GRAY2RGB);else cv::cvtColor(local_raw,img,cv::COLOR_BayerRG2RGB);
    double scale=min((double)g_right_w/img.cols,(double)g_win_h/img.rows);int dw=(int)(img.cols*scale),dh=(int)(img.rows*scale);
    cv::Mat resized;cv::resize(img,resized,cv::Size(dw,dh));int off_x=g_right_x+(g_right_w-dw)/2,off_y=(g_win_h-dh)/2;
    if(is_recording){int r=14;cv::circle(resized,cv::Point(dw-r*2,r*2),r,cv::Scalar(0,0,255),-1,cv::LINE_AA);
        cv::putText(resized,"REC",cv::Point(dw-r*10,r*3),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,0,255),2);
        int cf=cam_ctxs[cam_idx]->recorded_frames.load(memory_order_relaxed);double es=chrono::duration<double>(chrono::steady_clock::now()-record_start_time).count();char b[64];
        snprintf(b,sizeof(b),"%.1fs",es);cv::putText(resized,"Frame: "+to_string(cf)+"/"+to_string(total_record_frames),cv::Point(10,30),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,0),3);
        cv::putText(resized,"Frame: "+to_string(cf)+"/"+to_string(total_record_frames),cv::Point(10,30),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,0),2);
        cv::putText(resized,string(b),cv::Point(10,60),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,0),3);cv::putText(resized,string(b),cv::Point(10,60),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,0),2);
        if(cf>10&&es>0.01){double fps=cf/es;char fb[32];snprintf(fb,sizeof(fb),"%.1f fps",fps);cv::putText(resized,string(fb),cv::Point(10,90),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,0),3);cv::putText(resized,string(fb),cv::Point(10,90),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,0),2);}}
    canvas(right_roi)=cv::Scalar(0,0,0);resized.copyTo(canvas(cv::Rect(off_x,off_y,dw,dh)));
}
void showFaultOverlay(int faulty_cam, bool is_hw) {
    cv::Mat canvas=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);int cx=g_win_w/2,y=g_win_h/2-80;
    auto put=[&](int y,const string& t,double s,cv::Scalar c){int bl;cv::Size sz=cv::getTextSize(t,cv::FONT_HERSHEY_SIMPLEX,s,2,&bl);cv::putText(canvas,t,cv::Point(cx-sz.width/2,y),cv::FONT_HERSHEY_SIMPLEX,s,c,2);};
    put(y,"CAMERA FAULT DETECTED",1.0,cv::Scalar(0,0,255));y+=40;
    string ci="Camera: "+(faulty_cam>=0&&faulty_cam<(int)cam_ctxs.size()?cam_ctxs[faulty_cam]->id:"?")+" (index "+to_string(faulty_cam)+")";put(y,ci,0.7,cv::Scalar(255,255,255));y+=30;
    put(y,string("Host: ")+(g_fault_on_master.load()?"MASTER":"SLAVE"),0.7,cv::Scalar(255,255,255));y+=30;
    auto uptime_s=chrono::duration<double>(g_fault_time-g_ready_time).count();int h=(int)uptime_s/3600,m=((int)uptime_s%3600)/60;char ub[64];snprintf(ub,sizeof(ub),"Uptime: %dh %dm",h,m);put(y,string(ub),0.7,cv::Scalar(255,255,255));y+=40;
    put(y,"All cameras stopped. Press ESC to exit both hosts.",0.6,cv::Scalar(0,255,255));cv::imshow("Multi-Cam Preview",canvas);cv::waitKey(1);
}

// ================== HDF5 Sentry ==================
static void initSentry(const string& root) {
    fs::create_directories(root);
    string sp=root+"/sentry.txt";
    if(fs::exists(sp)){ifstream in(sp);int fo;in>>g_chunk_idx>>fo;g_frame_offset=fo;}
    else{g_chunk_idx=0;g_frame_offset=0;ofstream out(sp);out<<"0\n0\n";}
}
static void updateSentry(const string& root) {
    string sp=root+"/sentry.txt"; ofstream out(sp);
    out<<g_chunk_idx<<"\n"<<g_frame_offset<<"\n";
}

// ================== Cmd worker (TCP command channel) ==================
void cmdWorker(bool is_master, const string& master_ip, int cmd_port) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
    if (is_master) {
        g_cmd_listen_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); if(g_cmd_listen_sock==INVALID_SOCKET) return;
        int opt=1;setsockopt(g_cmd_listen_sock,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
        sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(cmd_port);sa.sin_addr.s_addr=INADDR_ANY;
        ::bind(g_cmd_listen_sock,(sockaddr*)&sa,sizeof(sa));listen(g_cmd_listen_sock,1);
        {DWORD to=500;setsockopt(g_cmd_listen_sock,SOL_SOCKET,SO_RCVTIMEO,(const char*)&to,sizeof(to));}  // accept 可中断
        cout<<"[Cmd] Master listening TCP ::"<<cmd_port<<endl;
        while(global_running){sockaddr_in ca;socklen_t cl=sizeof(ca);
            g_cmd_sock=accept(g_cmd_listen_sock,(sockaddr*)&ca,&cl);
            if(g_cmd_sock==INVALID_SOCKET){if(!global_running)break;continue;}  // 超时轮询, 退出时结束
            cout<<"[Cmd] Slave connected. Handshaking..."<<endl;
            string hl; if(recvLine(g_cmd_sock,hl,10000)&&hl=="READY"){sendLineRaw(g_cmd_sock,"ACK");cout<<"[Cmd] Handshake OK."<<endl;}
            else{cerr<<"[Cmd] Handshake FAILED (recv:'"<<hl<<"'). Reconnecting..."<<endl;closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;continue;}
            while(global_running){
                string line;
                int st=recvLineStatus(g_cmd_sock,line,500);
                if(st==2){cerr<<"[Cmd] Slave disconnected - re-accepting."<<endl;break;}  // 真断连 → 重新 accept
                if(st==1)continue;  // 500ms 轮询超时
                if(line.rfind("HDF5_DONE:",0)==0){g_slave_hdf5_done=true;g_slave_hdf5_s=atof(line.c_str()+10);cout<<"[Cmd] Slave HDF5 done ("<<g_slave_hdf5_s<<"s)."<<endl;}
                else if(line=="PRECHECK_CLEAR"){g_pre_peer_clear=true;cout<<"[Cmd] Slave pre-check clear (no h5)."<<endl;}
                else if(line=="PRECHECK_BLOCKED"){g_pre_peer_reject=true;cout<<"[Cmd] Slave has existing h5 — pre-create aborted."<<endl;}
                else if(line=="PRECREATE_DONE"){g_pre_peer_done=true;cout<<"[Cmd] Slave pre-create done."<<endl;}
                else if(line.rfind("FAULT:",0)==0&&!g_fault_active.load()){
                    if(line.length()<=7) continue;
                    char hf=line[6]; int fi=stoi(line.substr(7));
                    if(fi<0||fi>=(int)cam_ctxs.size()) continue;
                    cout<<"[Fault] Received from SLAVE: cam "<<fi<<endl;
                    g_fault_time=chrono::steady_clock::now();
                    g_fault_active.store(true);g_faulty_cam.store(fi);g_fault_on_master.store(false);
                    for(auto& c:cam_ctxs){c->running=false;c->copy_cv.notify_all();}
                    for(auto& c:cam_ctxs){if(c->capture_thread.joinable())c->capture_thread.join();if(c->copy_thread.joinable())c->copy_thread.join();}
                    cout<<"[Fault] All cameras stopped. Press ESC to exit."<<endl;
                }
            }
            closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;
        }
    } else {
        while(global_running){
            g_cmd_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP); if(g_cmd_sock==INVALID_SOCKET){this_thread::sleep_for(chrono::seconds(2));continue;}
            sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(cmd_port);inet_pton(AF_INET,master_ip.c_str(),&sa.sin_addr);
            if(connect(g_cmd_sock,(sockaddr*)&sa,sizeof(sa))!=0){closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;this_thread::sleep_for(chrono::seconds(2));continue;}
            cout<<"[Cmd] Slave connected. Handshaking..."<<endl;
            if(!sendLineRaw(g_cmd_sock,"READY")){closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;continue;}
            string hl; if(!recvLine(g_cmd_sock,hl,10000)||hl!="ACK"){closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;continue;}
            cout<<"[Cmd] Handshake OK."<<endl;
            while(global_running){
                string line;
                int st=recvLineStatus(g_cmd_sock,line,300000);
                if(st==2){cerr<<"[Cmd] Connection closed by Master - reconnecting."<<endl;break;}  // 真断连 → 重连, 不退出
                if(st==1)continue;  // 空闲超时 → 继续等待
                if(line=="INIT_OK"){g_init_ok=true;cout<<"[Cmd] Received INIT_OK from Master."<<endl;}
                else if(line.rfind("PIPER:",0)==0){
                    size_t c2=line.find(':',6); if(c2==string::npos) continue;
                    string an=line.substr(6,c2-6);
                    if(an=="active"){ g_arm=line.substr(c2+1); cout<<"[Cmd] PIPER active="<<g_arm<<endl; }
                    else {
                        int n=stoi(line.substr(c2+1,line.find(':',c2+1)-c2-1));
                        string st=line.substr(line.find_last_of(':')+1); bool d=(st=="done");
                        if(an=="upper"){g_upper_idx=n;g_upper_done=d;} else {g_lower_idx=n;g_lower_done=d;}
                        cout<<"[Cmd] PIPER: "<<an<<" idx="<<n<<" "<<(d?"done":"ok")<<endl;
                    }
                    // Recompute exhausted based on current active arm
                    bool& cd=(g_arm=="upper")?g_upper_done:g_lower_done; g_show_exhausted=cd;
                }
                else if(line=="GAZE_DONE"){g_gaze_done=true;cout<<"[Cmd] Received GAZE_DONE from Master."<<endl;}
                else if(line=="PRECHECK_REQ"){   // master 请求预检 → 回复本机是否有 h5
                    bool has=anyLocalH5();
                    sendLineRaw(g_cmd_sock,has?"PRECHECK_BLOCKED":"PRECHECK_CLEAR");
                    cout<<"[Cmd] PRECHECK_REQ -> "<<(has?"BLOCKED":"CLEAR")<<endl;}
                else if(line=="PRECREATE_BEGIN"){   // master 令开始创建 → 独立线程, 完成后回报
                    g_pre_phase=2;g_precreating=true;g_pre_local_done=false;
                    if(g_pre_thread.joinable())g_pre_thread.join();
                    g_pre_thread=thread([]{precreateSerial();g_pre_local_done=true;sendLineRaw(g_cmd_sock,"PRECREATE_DONE");});
                    cout<<"[Cmd] Pre-create started on slave."<<endl;}
                else if(line=="TRIGGER"){instantTrigger();net_cmd_record=true;}
                else if(line.rfind("FAULT:",0)==0&&!g_fault_active.load()){
                    if(line.length()<=7) continue;
                    char hf=line[6]; int fi=stoi(line.substr(7));
                    if(fi<0||fi>=(int)cam_ctxs.size()) continue;
                    cout<<"[Slave] Fault from MASTER: cam "<<fi<<endl;
                    g_fault_time=chrono::steady_clock::now();
                    g_fault_active.store(true);g_faulty_cam.store(fi);g_fault_on_master.store(true);
                    for(auto& c:cam_ctxs){c->running=false;c->copy_cv.notify_all();}
                    for(auto& c:cam_ctxs){if(c->capture_thread.joinable())c->capture_thread.join();if(c->copy_thread.joinable())c->copy_thread.join();}
                    cout<<"[Fault] All cameras stopped. Press ESC to exit."<<endl;
                }
                else if(line=="EXIT"){cout<<"[Slave] Received EXIT."<<endl;global_running=false;}
            }
            closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;
            if(!global_running) break;
            this_thread::sleep_for(chrono::seconds(1));
        }
    }
    if(g_cmd_listen_sock!=INVALID_SOCKET){closesocket(g_cmd_listen_sock);g_cmd_listen_sock=INVALID_SOCKET;}
    if(g_cmd_sock!=INVALID_SOCKET){closesocket(g_cmd_sock);g_cmd_sock=INVALID_SOCKET;}
}

// ================== Session report ==================
void writeReport(const string& timestr, int rec_num, int total_frames, bool hw_trigger) {
    if (!g_session_log.is_open()) return;
    g_session_log << "\n---\n\n"
                  << "## Recording #" << rec_num << ": " << timestr << "\n\n"
                  << "- **Cameras**: " << cam_ctxs.size() << "\n"
                  << "- **Trigger**: " << (hw_trigger ? "HW" : "SW") << "\n"
                  << "- **Total frames**: " << total_frames << "\n\n";

    // Per-Camera Metrics
    g_session_log << "### Per-Camera Metrics\n\n";
    g_session_log << "| # | SN | Type | Saved | Drop | FPS | QPeak | Lat(ms) | Rec2RAM(s) | Delay(s) |"
                  << " ArmStage(s) | M-HDF5(s) | S-HDF5(s) |\n";
    g_session_log << "|---|-----|------|-------|------|-----|-------|---------|------------|----------|"
                  << "------------|-----------|-----------|\n";
    g_session_log << "|   |     | mono/color | 实际保存帧数 | BlockID跳变丢帧 | 平均帧率 | 队列峰值/总帧数 | 首帧触发延迟 | RAM写完-理论完成 | SPACE→触发延迟 |"
                  << " 并行阶段墙钟 | Master HDF5写入 | Slave HDF5写入 |\n";

    double theoretical_s = total_frames / 200.0;
    for (auto& ctx : cam_ctxs) {
        int saved = ctx->recorded_frames.load();
        int dropped = ctx->dropped_frames.load();
        double fps = 0.0;
        if (saved > 1) {
            double dur_s = (ctx->meta_buffer[saved-1].timestamp - ctx->meta_buffer[0].timestamp) / 10000000.0;
            if (dur_s > 0) fps = (saved - 1) / dur_s;
        }
        double actual_ram_s = chrono::duration<double>(ctx->recording_end_time - ctx->first_frame_time).count();
        ctx->recover2ram_s = actual_ram_s - theoretical_s;
        double lat_ms = ctx->first_frame_time.time_since_epoch().count() > 0
            ? chrono::duration<double,milli>(ctx->first_frame_time - global_record_start_time).count() : 0.0;

        // S-HDF5: slave side has direct measurement; master gets it from HDF5_DONE message
        double show_shdf5 = g_slave_hdf5_s > 0 ? g_slave_hdf5_s : 0.0;
        double show_mhdf5 = g_master_hdf5_s > 0 ? g_master_hdf5_s : 0.0;

        g_session_log << "| " << ctx->index << " | " << ctx->id << " | "
                      << (ctx->is_mono?"mono":"color") << " | "
                      << saved << " | " << dropped << " | "
                      << fixed << setprecision(1) << fps << " | "
                      << ctx->max_queue_size.load() << "/" << total_frames << " | "
                      << fixed << setprecision(1) << lat_ms << " | "
                      << fixed << setprecision(3) << ctx->recover2ram_s << " | "
                      << fixed << setprecision(3) << g_last_delay_s << " | "
                      << g_arm_stage_s << " | " << show_mhdf5 << " | " << show_shdf5 << " |\n";
    }

    // Summary
    g_session_log << "\n### Summary\n\n";
    g_session_log << "| Metric | Value | Note |\n";
    g_session_log << "|--------|-------|------|\n";
    g_session_log << "| ARM stage | " << fixed << setprecision(2) << g_arm_stage_s << " s | Par. wall clock (arm || HDF5) |\n";
    g_session_log << "| Delay | " << fixed << setprecision(3) << g_last_delay_s << " s | SPACE→录制触发 (受试者聚焦十字中心) |\n";
    g_session_log << "| Master HDF5 write | " << g_master_hdf5_s << " s | WaitForMultipleObjects |\n";
    g_session_log << "| Slave HDF5 write | " << g_slave_hdf5_s << " s | WaitForMultipleObjects (via TCP) |\n";
    double max_hdf5 = max(g_master_hdf5_s, g_slave_hdf5_s);
    g_session_log << "| Max HDF5 write | " << max_hdf5 << " s | max(Master, Slave) |\n";
    g_session_log << "| Wait Slave HDF5_DONE | " << g_wait_slave_hdf5_s << " s | Master idle wait for Slave |\n";
    g_session_log << "| GAZE forward | " << g_gaze_forward_s << " s | GAZE tx + GAZE_ACK + GAZE_DONE |\n";
    g_session_log << "| Sentry sync | " << g_sentry_sync_s << " s | TCP handshake port+300 |\n";
    g_session_log << "| End-to-end | " << g_recording_end_to_end_s << " s | SPACE→SPACE-ready (delay+record+arm&h5+sync) |\n";
    int ef=g_exc_fatal.load(), ee=g_exc_error.load(), ew=g_exc_warn.load(), ei=g_exc_info.load();
    auto b=[](int v){return v>0?"**"+to_string(v)+"**":to_string(v);};
    g_session_log << "| exceptions_fatal | "<<b(ef)<<" | FATAL count |\n";
    g_session_log << "| exceptions_error | "<<b(ee)<<" | ERROR count |\n";
    g_session_log << "| exceptions_warn  | "<<b(ew)<<" | WARN count |\n";
    g_session_log << "| exceptions_info  | "<<b(ei)<<" | INFO count |\n";
    g_session_log << defaultfloat << flush;
}

// ================== main ==================
// participant_id → day_id (cfg/day_participant_map.json; JSON 是 YAML 子集)
string findDayForParticipant(const string& json_path, const string& participant) {
    try {
        YAML::Node root = YAML::LoadFile(json_path);
        for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
            string day = it->first.as<string>();
            for (size_t i = 0; i < it->second.size(); ++i)
                if (it->second[i].as<string>() == participant) return day;
        }
    } catch (const std::exception& e) {
        cerr << "[Piper] Failed to parse " << json_path << ": " << e.what() << endl;
    }
    return "";
}

int main() {
    _putenv("HDF5_USE_FILE_LOCKING=FALSE");
    // 安装时间戳输出 (所有 cout/cerr 行自动加 [YYYY-MM-DD HH:MM:SS] 前缀)
    static TimestampBuf tsb_out(cout.rdbuf());
    static TimestampBuf tsb_err(cerr.rdbuf());
    cout.rdbuf(&tsb_out);
    cerr.rdbuf(&tsb_err);
    cout<<"=== [TEST] Multi-Basler Camera Tool (Sync Network Node + Piper) ==="<<endl;
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2),&wsa);
#endif
    // ---- Load config ----
    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg").string();
    Cfg cfg_cap(cfg_dir+"/capture.yaml");
    auto& cap=cfg_cap["capture"];
    Pylon::PylonInitialize();
    g_participant_roots=cap["participant_root"].as<vector<string>>();
    string participant_id; try{participant_id=cap["participant_id"].as<string>();}catch(...){participant_id="P001";}
    g_participant_id=participant_id;
    for(auto& r:g_participant_roots) r+="/"+participant_id;
    g_sentry_root=g_participant_roots[0];
    try{g_hdf5_chunk_capacity=cap["hdf5_chunk_frame_capacity"].as<int>();}catch(...){}
    g_is_master=cap["is_master"].as<bool>();
    g_master_ip=cap["master_ip"].as<string>();
    string slave_ip=cap["slave_ip"].as<string>();
    int net_port=cap["port"].as<int>();
    vector<string> camera_ids=cap["cam_indices"].as<vector<string>>();
    bool use_hw_trigger=cap["hardware_trigger"].as<bool>();
    bool enable_offset=true, enable_intersection=true, enable_net_sync=true;
    try{enable_offset=cap["enable_offset"].as<bool>();enable_intersection=cap["enable_intersection"].as<bool>();
        enable_net_sync=cap["enable_net_sync"].as<bool>();}catch(...){}
    // AutoMove / OVER 配置
    try{g_enable_auto_move=cap["enable_auto_move"].as<bool>();}catch(...){}
    try{g_click_window=cap["click_window"].as<double>();}catch(...){}
    try{g_num_targets_per_arm=cap["num_targets_per_arm"].as<int>();}catch(...){}
    try{g_capture_delay=cap["capture_delay"].as<double>();}catch(...){}
    double target_fps=cap["fps"].as<double>(), gain=cap["gain"].as<double>();
    double gammav=cap["gamma"].as<double>(), exp_time=cap["exposure_time"].as<double>();
    g_win_w=cap["window_width"].as<int>(); g_win_h=cap["window_height"].as<int>();
    double record_time=cap["record_time"].as<double>();
    int cam_w=cap["cam_width"].as<int>(), cam_h=cap["cam_height"].as<int>();
    g_cam_w=cam_w; g_cam_h=cam_h;
    int core_frames=(int)ceil(target_fps*record_time);
    double margin_ratio=cap["margin_frames_ratio"].as<double>();
    int margin_frames=(int)ceil(core_frames*margin_ratio);
    int total_record_frames=core_frames+2*margin_frames;
    bool is_master_pc=g_is_master;
    g_use_hw_trigger=use_hw_trigger;

    // ---- 配置打印: 输入/输出目录 ----
    cout<<"\n--- Data Directories ---"<<endl;
    cout<<"Participant ID   : "<<participant_id<<endl;
    for(size_t i=0;i<g_participant_roots.size();++i)
        cout<<"ParticipantRoot["<<i<<"]: "<<g_participant_roots[i]<<endl;
    cout<<"Sentry root      : "<<g_sentry_root<<endl;
    cout<<"HDF5 chunk cap   : "<<g_hdf5_chunk_capacity<<endl;
    cout<<"Cameras          : "<<camera_ids.size()<<endl;
    cout<<"----------------------------------\n"<<endl;

    cout<<"\n--- Network Sync Configuration ---"<<endl;
    cout<<"Role             : "<<(is_master_pc?"MASTER (Sender)":"SLAVE (Receiver)")<<endl;
    cout<<"Master IP        : "<<g_master_ip<<endl;
    cout<<"Slave IP         : "<<slave_ip<<endl;
    cout<<"Port             : "<<net_port<<endl;
    cout<<"HW Trigger       : "<<(use_hw_trigger?"ON":"OFF")<<endl;
    cout<<"SW Offset Init   : "<<(enable_offset?"ON":"OFF")<<endl;
    cout<<"Intersection Crop: "<<(enable_intersection?"ON":"OFF")<<endl;
    cout<<"Net Sync         : "<<(enable_net_sync?"ON":"OFF (Local Mode)")<<endl;
    cout<<"Auto Move        : "<<(g_enable_auto_move?"ON":"OFF");
    if(g_enable_auto_move) cout<<" (click window "<<g_click_window<<"s)";
    cout<<endl;
    cout<<"Targets per arm  : "<<(g_num_targets_per_arm>0?to_string(g_num_targets_per_arm):"unlimited")<<endl;
    cout<<"Capture Delay    : "<<g_capture_delay<<"s"<<endl;
    cout<<"----------------------------------\n"<<endl;

    // ---- Piper config (Master only) ----
    string ubuntu_ip; int ctrl_port=49301, gaze_port=49302;
    if (is_master_pc) {
        Cfg cfg_piper(cfg_dir+"/piper.yaml");
        ubuntu_ip=cfg_piper["network"]["ubuntu_ip"].as<string>();
        ctrl_port=cfg_piper["network"]["ctrl_port"].as<int>();
        try{gaze_port=cfg_piper["network"]["gaze_port"].as<int>();}catch(...){gaze_port=49302;}
        g_gaze_dir="cfg/gaze_target/"+participant_id;
        auto readPt3=[](const CfgNode& n)->Pt3{return{n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};};
        // 位姿从 cfg/arm_pose/{day_id}.yaml 加载 (day_id 由 participant_id 经 day_participant_map.json 映射, 不再是 piper.yaml)
        string day_id=findDayForParticipant(cfg_dir+"/day_participant_map.json",participant_id);
        string arm_pose_yml=cfg_dir+"/arm_pose/"+day_id+".yaml";
        if(!day_id.empty()&&fs::exists(arm_pose_yml)){
            g_arm_pose_yml="cfg/arm_pose/"+day_id+".yaml";   // UI 水印用相对路径 (从 cfg 开始)
            Cfg cfg_pose(arm_pose_yml);
            for (auto& an:{"upper","lower"}) {
                try{auto& a=cfg_pose["arms"][an];auto& tl=a["tool"];auto& cc=a["arm_in_ccs"];
                    auto& xf=(an==string("upper"))?g_xf_upper:g_xf_lower;
                    xf.tool_t=readPt3(tl["translation"]);xf.tool_r=readPt3(tl["rotation_zxz"]);
                    xf.ccs_t=readPt3(cc["translation"]);xf.ccs_r=readPt3(cc["rotation_zxz"]);}
                catch(...){cerr<<"[Piper] WARN: cannot load "<<an<<" transform from "<<arm_pose_yml<<endl;}
            }
            cout<<"[Piper] Arm transforms loaded from "<<arm_pose_yml<<" (day "<<day_id<<")"<<endl;
        }else{
            cerr<<"[Piper] WARN: no arm_pose yaml for participant "<<participant_id
                <<" — check cfg/day_participant_map.json. Transforms NOT loaded."<<endl;
        }
        auto loadTgts=[&](const string& path)->vector<array<double,3>>{
            vector<array<double,3>> out; ifstream in(path); string line;
            while(getline(in,line)){if(line.empty())continue;stringstream ss(line);string token;array<double,3>pt{};
                for(int i=0;i<3&&getline(ss,token,',');++i)try{pt[i]=stod(token);}catch(...){break;}out.push_back(pt);}return out;};
        g_targets_upper=loadTgts(g_gaze_dir+"/piper_upper.txt");
        g_targets_lower=loadTgts(g_gaze_dir+"/piper_lower.txt");
        string sp=g_gaze_dir+"/sentry.txt";
        if (ifstream sf(sp);sf) {string line;while(getline(sf,line)){
            if(line.rfind("upper:",0)==0)g_upper_idx=stoi(line.substr(6));
            if(line.rfind("lower:",0)==0)g_lower_idx=stoi(line.substr(6));}}
        g_upper_done=(g_upper_idx>=(int)g_targets_upper.size());
        g_lower_done=(g_lower_idx>=(int)g_targets_lower.size());
        cout<<"[Piper] Targets: upper="<<g_targets_upper.size()<<" idx="<<g_upper_idx
            <<(g_upper_done?" DONE":"")<<" lower="<<g_targets_lower.size()<<" idx="<<g_lower_idx
            <<(g_lower_done?" DONE":"")<<endl;

        // ---- M5Stack 状态灯串口 (cfg/M5Stack.yaml: upper/lower COM) ----
        try {
            Cfg cfg_led(cfg_dir+"/M5Stack.yaml");
            string led_upper=cfg_led["ports"]["upper"].as<string>();
            string led_lower=cfg_led["ports"]["lower"].as<string>();
            int led_baud=115200;
            try{led_baud=cfg_led["serial"]["baud_rate"].as<int>();}catch(...){}
            openLedSerial(g_led_upper, led_upper, led_baud);
            openLedSerial(g_led_lower, led_lower, led_baud);
            // 初始状态同步: 两块设备各自 INIT
            sendLedPatternTo(g_led_upper, LedState::PIPER_INIT);
            sendLedPatternTo(g_led_lower, LedState::PIPER_INIT);
        } catch (...) {
            cerr<<"[M5Stack] cfg/M5Stack.yaml load failed — LED disabled."<<endl;
        }
    }

    // ====== TCP handshakes (sequential, one port at a time, before camera init) ======
    thread cmd_thread, gaze_thread;
    atomic<bool> cmd_ready{false}, gaze_ready{false};

    if (enable_net_sync) {
        int cmd_port=net_port+400;
        // 1. Cmd channel
        cout<<"[Cmd] Starting TCP command channel on port "<<cmd_port<<"..."<<endl;
        cmd_thread=thread(cmdWorker, is_master_pc, g_master_ip, cmd_port);
        while (!cmd_ready.load() && global_running) {
            if (g_cmd_sock != INVALID_SOCKET) cmd_ready = true;
            else this_thread::sleep_for(chrono::milliseconds(100));
        }
        if (!cmd_ready) { cerr<<"[Cmd] FATAL: Command channel failed."<<endl;
            if(is_master_pc){global_running=false;if(cmd_thread.joinable())cmd_thread.join();}return 1; }
        cout<<"[Cmd] Command channel established."<<endl;

        // 2. Gaze channel
        cout<<"[Gaze] Starting gaze channel on port "<<gaze_port<<"..."<<endl;
        if (is_master_pc) gaze_thread=thread(gazeServerWorker, gaze_port);
        else gaze_thread=thread(gazeClientWorker, g_master_ip, gaze_port);
        while (!gaze_ready.load() && global_running) {
            if (g_gaze_connected.load()) gaze_ready = true;
            if (!gaze_ready) this_thread::sleep_for(chrono::milliseconds(100));
        }
        if (!gaze_ready) { cerr<<"[Gaze] FATAL: Gaze channel failed."<<endl;
            if(is_master_pc&&g_cmd_sock!=INVALID_SOCKET)sendLineRaw(g_cmd_sock,"EXIT");
            global_running=false;if(cmd_thread.joinable())cmd_thread.join();return 1; }
        cout<<"[Gaze] Gaze channel established."<<endl;
    }

    // 3. Piper connection (Master only, with retry)
    if (is_master_pc) {
        cout<<"[Piper] Connecting to Ubuntu "<<ubuntu_ip<<":"<<ctrl_port<<"..."<<endl;
        bool piper_ok = false;
        for (int retry = 0; retry < 30 && global_running; ++retry) {
            g_piper_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (g_piper_sock == INVALID_SOCKET) { this_thread::sleep_for(chrono::seconds(2)); continue; }
            sockaddr_in sa{}; sa.sin_family = AF_INET; sa.sin_port = htons(ctrl_port);
            inet_pton(AF_INET, ubuntu_ip.c_str(), &sa.sin_addr);
            if (connect(g_piper_sock, (sockaddr*)&sa, sizeof(sa)) == 0) {
                piper_ok = true; break;
            }
            cerr << "[Piper] Connect attempt " << (retry+1) << "/30 failed, retrying..." << endl;
            closesocket(g_piper_sock); g_piper_sock = INVALID_SOCKET;
            this_thread::sleep_for(chrono::seconds(2));
        }
        if (!piper_ok) {
            cerr << "[Piper] FATAL: Cannot connect to Ubuntu after 30 attempts." << endl;
            if (enable_net_sync && g_cmd_sock != INVALID_SOCKET)
                sendLineRaw(g_cmd_sock, "EXIT");  // tell Slave to exit
            return 1;
        }
        cout<<"[Piper] Connected to Ubuntu."<<endl;
        cout<<"[Piper] Zeroing both arms..."<<endl;
        if(!zeroArm("upper")) cerr<<"[Piper] WARN: upper zero FAIL"<<endl;
        if(!zeroArm("lower")) cerr<<"[Piper] WARN: lower zero FAIL"<<endl;
        cout<<"[Piper] Arm initialization complete."<<endl;
    }

    // 6.1 Startup exhaustion check
    if(is_master_pc && ((g_arm=="upper" && g_upper_done) || (g_arm=="lower" && g_lower_done))){
        g_show_exhausted = true;
        cout<<"[Piper] WARNING: "<<g_arm<<" exhausted at startup."<<endl;
    }

    // ====== Master signals Slave: all connections ready ======
    if (enable_net_sync && is_master_pc) {
        syncPiperToSlave(true);  // sends PIPER + INIT_OK atomically
    }
    if (enable_net_sync && !is_master_pc) {
        cout<<"[Init] Waiting for Master INIT_OK signal..."<<endl;
        auto t0=chrono::steady_clock::now();
        while (global_running && !g_init_ok.load()){
            this_thread::sleep_for(chrono::milliseconds(100));
            if(chrono::duration<double>(chrono::steady_clock::now()-t0).count()>30.0){
                cerr<<"[Init] Timeout waiting for INIT_OK (30s)."<<endl; break;
            }
        }
        if (!g_init_ok.load()) { cerr<<"[Init] Never received INIT_OK."<<endl; return 1; }
    }
    cout<<"[Init] All 3 hosts ready. Starting camera initialization..."<<endl;

    // ---- Camera init ----
    for (int i=0;i<(int)camera_ids.size();++i)
        cam_ctxs.push_back(make_shared<CameraContext>(i,camera_ids[i],g_participant_roots[i]));
    cout<<"[System] Pre-allocating shared memory for "<<total_record_frames<<" frames..."<<endl;
    for (auto& ctx:cam_ctxs) {
        ctx->total_record_frames=total_record_frames;ctx->ram_buffer.resize(total_record_frames);ctx->meta_buffer.resize(total_record_frames);
        string shm_name="HDF5_"+to_string(GetCurrentProcessId())+"_CAM_"+to_string(ctx->index);
        size_t total_bytes=(size_t)total_record_frames*(size_t)cam_h*(size_t)cam_w;
        HANDLE hMap=CreateFileMappingA(INVALID_HANDLE_VALUE,NULL,PAGE_READWRITE,0,(DWORD)total_bytes,shm_name.c_str());
        if(!hMap){logException("FATAL","shm","CreateFileMapping failed");return 1;}
        uint8_t* shm_base=(uint8_t*)MapViewOfFile(hMap,FILE_MAP_WRITE,0,0,total_bytes);
        if(!shm_base){logException("FATAL","shm","MapViewOfFile failed");CloseHandle(hMap);return 1;}
        memset(shm_base,0,total_bytes);
        for(int k=0;k<total_record_frames;++k)ctx->ram_buffer[k]=cv::Mat(cam_h,cam_w,CV_8UC1,shm_base+(size_t)k*(size_t)cam_h*(size_t)cam_w);
        ctx->shm_handle=hMap;ctx->shm_base=shm_base;
    }
    cout<<"[System] Shared memory allocated.\n"<<endl;
    CameraContext::master_set.store(false);CameraContext::master_first_id.store(-1);
    for (auto& ctx:cam_ctxs){ctx->running=true;ctx->dump_ready=false;ctx->offset_initialized=false;
        ctx->copy_thread=thread(copyWorker,ctx);
        ctx->capture_thread=thread(captureWorker,ctx,target_fps,gain,gammav,exp_time,use_hw_trigger,enable_offset);}

    // ---- UI init ----
    cv::namedWindow("Multi-Cam Preview",cv::WINDOW_NORMAL);cv::resizeWindow("Multi-Cam Preview",g_win_w,g_win_h);
    updateLayout();cv::setMouseCallback("Multi-Cam Preview",onMouse);
    {auto t=chrono::system_clock::to_time_t(chrono::system_clock::now());char tb[64];strftime(tb,sizeof(tb),"%Y%m%d_%H%M%S",localtime(&t));
        error_code ec;fs::create_directories("log/capture",ec);
        g_session_log_path=string("log/capture/session_")+tb+".md";g_session_log.open(g_session_log_path,ios::out|ios::app);
        if(g_session_log.is_open())g_session_log<<"# Session: "<<tb<<"\n\n- **Cameras**: "<<camera_ids.size()
            <<"\n- **Trigger**: "<<(use_hw_trigger?"HW":"SW")<<"\n- **Target FPS**: "<<target_fps
            <<"\n- **Storage**: HDF5+Piper\n\n---\n"<<flush;}
    initSentry(g_sentry_root);
    cout<<"[HDF5] Sentry (local): chunk="<<g_chunk_idx<<" offset="<<g_frame_offset<<endl;
    for (auto& ctx:cam_ctxs){ctx->hdf5_dir=g_participant_roots[ctx->index]+"/"+ctx->id;fs::create_directories(ctx->hdf5_dir);}

    // ====== Startup sentry handshake ======
    if (enable_net_sync) {
        cv::namedWindow("Multi-Cam Preview",cv::WINDOW_NORMAL);
        cv::resizeWindow("Multi-Cam Preview",g_win_w,g_win_h);
        cv::Mat loading=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
        cv::putText(loading,"Startup sentry handshake in progress...",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,255,255),2);
        if(is_master_pc) drawLedIndicator(loading);
        cv::imshow("Multi-Cam Preview",loading);cv::waitKey(1);
    }
    if (enable_net_sync) {
        int handshake_port=net_port+300;
        auto local_ci=g_chunk_idx; auto local_fo=g_frame_offset.load();
        int64_t local_total=(int64_t)local_ci*g_hdf5_chunk_capacity+local_fo;
        cout<<"[Sentry] Startup handshake: local="<<local_total<<" (chunk="<<local_ci<<" offset="<<local_fo<<")"<<endl;
        if (is_master_pc) {
            SOCKET hs=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);int opt=1;setsockopt(hs,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
            sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(handshake_port);sa.sin_addr.s_addr=INADDR_ANY;
            ::bind(hs,(sockaddr*)&sa,sizeof(sa));listen(hs,1);
            cout<<"[Sentry] Master waiting for Slave startup handshake on port "<<handshake_port<<"..."<<endl;
            sockaddr_in ca;socklen_t cl=sizeof(ca);SOCKET cs=accept(hs,(sockaddr*)&ca,&cl);
            if(cs!=INVALID_SOCKET){int64_t peer_buf[2];recv(cs,(char*)peer_buf,sizeof(peer_buf),0);int64_t peer_total=peer_buf[0];
                int64_t send_buf[2]={local_total,local_total};send(cs,(const char*)send_buf,sizeof(send_buf),0);closesocket(cs);
                if(peer_total!=local_total){int peer_ci=(int)(peer_total/g_hdf5_chunk_capacity),peer_fo=(int)(peer_total%g_hdf5_chunk_capacity);
                    logException("WARN","sentry","Startup mismatch: Master("+to_string(local_ci)+","+to_string(local_fo)
                        +") Slave("+to_string(peer_ci)+","+to_string(peer_fo)+") diff="+to_string(llabs(local_total-peer_total))+" - using min");
                    if(peer_total<local_total){g_chunk_idx=peer_ci;g_frame_offset=peer_fo;updateSentry(g_sentry_root);}}
                cout<<"[Sentry] Startup handshake done. Synced to: chunk="<<g_chunk_idx<<" offset="<<g_frame_offset<<endl;}
            closesocket(hs);
        } else {
            cout<<"[Sentry] Slave connecting to Master for startup handshake on port "<<handshake_port<<"..."<<endl;
            SOCKET hs=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(handshake_port);
            inet_pton(AF_INET,g_master_ip.c_str(),&sa.sin_addr);
            while(connect(hs,(sockaddr*)&sa,sizeof(sa))==-1&&global_running) this_thread::sleep_for(chrono::milliseconds(100));
            if(hs!=INVALID_SOCKET){int64_t send_buf[2]={local_total,local_total};send(hs,(const char*)send_buf,sizeof(send_buf),0);
                int64_t peer_buf[2];recv(hs,(char*)peer_buf,sizeof(peer_buf),0);int64_t peer_total=peer_buf[0];closesocket(hs);
                if(peer_total!=local_total){int peer_ci=(int)(peer_total/g_hdf5_chunk_capacity),peer_fo=(int)(peer_total%g_hdf5_chunk_capacity);
                    logException("WARN","sentry","Startup mismatch: Master("+to_string(peer_ci)+","+to_string(peer_fo)
                        +") Slave("+to_string(local_ci)+","+to_string(local_fo)+") diff="+to_string(llabs(local_total-peer_total))+" - using min");
                    if(peer_total<local_total){g_chunk_idx=peer_ci;g_frame_offset=peer_fo;updateSentry(g_sentry_root);}}
                cout<<"[Sentry] Startup handshake done. Synced to: chunk="<<g_chunk_idx<<" offset="<<g_frame_offset<<endl;}
        }
    }
    cout<<"[HDF5] Ready."<<endl;
    if(is_master_pc)cout<<"[s] Start session  [SPACE] Record  [b] Re-zero  [c] Clear piper sentry  [t] Switch arm  [ESC/q] Quit\n";
    else cout<<"Waiting for Master... [ESC/q] to quit.\n";

    // ================== MAIN LOOP ==================
    bool is_recording=false; atomic<bool> is_dumping{false};
    string current_record_timestr; chrono::steady_clock::time_point record_start_time;
    double target_ui_fps=cap["ui_fps"].as<double>();
    auto ui_interval=chrono::milliseconds((int)(1000.0/target_ui_fps));
    auto last_ui_time=chrono::steady_clock::now()-ui_interval;
    g_ready_time=chrono::steady_clock::now();

    while (global_running) {
        auto current_time=chrono::steady_clock::now();
        bool need_ui_update=(current_time-last_ui_time)>=ui_interval;

        // ===== LED state update (Master only; 状态变化时同步到 M5Stack) =====
        if (is_master_pc) {
            if (g_show_over) {
                setLedState(LedState::OVER);
            } else if (g_show_exhausted) {
                setLedState(LedState::EXHAUSTED);
            } else if (is_dumping) {
                setLedState(LedState::WAITING);
            } else if (g_delay_pending.load()) {
                setLedState(LedState::CAPTURING);
            } else if (is_recording) {
                setLedState(LedState::CAPTURING);
            } else if (g_recording_enabled) {
                setLedState(LedState::READY);
            } else {
                setLedState(LedState::PIPER_INIT);
            }
        }

        // ===== UI render (from hdf5_multi_process.cpp) =====
        if (need_ui_update && !is_dumping) {
            if (g_fault_active.load()) {
                showFaultOverlay(g_faulty_cam.load(), g_use_hw_trigger);
            } else if (g_show_over) {
                // OVER UI — 当前臂成功录制数达到 num_targets_per_arm
                cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
                string arm_name = g_arm; for(auto& c:arm_name) c=toupper(c);
                string msg = arm_name + " OVER";
                cv::putText(canvas, msg, cv::Point(g_win_w/4, g_win_h/2-40), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(255,0,255), 2);
                char buf[64];
                snprintf(buf,sizeof(buf),"%d / %d targets recorded", recordedOf(), g_num_targets_per_arm);
                cv::putText(canvas,buf,cv::Point(g_win_w/4,g_win_h/2+10),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255),1);
                string hints = "Press 'b' to reset or 't' to switch arm | 'q' to quit";
                cv::putText(canvas,hints,cv::Point(g_win_w/4,g_win_h/2+50),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,255),1);
                if(is_master_pc) drawLedIndicator(canvas);
                cv::imshow("Multi-Cam Preview", canvas);
            } else if (g_show_exhausted) {
                // EXHAUSTED UI — no camera rendering
                cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
                string arm_name = g_arm; for(auto& c:arm_name) c=toupper(c);
                string msg = arm_name + " INST EXHAUSTED";
                if(g_upper_done && g_lower_done) msg = "BOTH ARMS EXHAUSTED";
                cv::putText(canvas, msg, cv::Point(g_win_w/4, g_win_h/2-40), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0,0,255), 2);
                char buf[64];
                snprintf(buf,sizeof(buf),"Upper: %d/%d %s | Lower: %d/%d %s",
                    g_upper_idx,(int)g_targets_upper.size(),g_upper_done?"DONE":"ok",
                    g_lower_idx,(int)g_targets_lower.size(),g_lower_done?"DONE":"ok");
                cv::putText(canvas,buf,cv::Point(g_win_w/4,g_win_h/2+10),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255),1);
                string hints = "Press 't' to switch arm | 'c' to clear sentry | 'q' to quit";
                if(g_upper_done&&g_lower_done) hints = "Press 'c' to clear sentry | 'q' to quit";
                cv::putText(canvas,hints,cv::Point(g_win_w/4,g_win_h/2+50),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,255),1);
                if(is_master_pc) drawLedIndicator(canvas);
                cv::imshow("Multi-Cam Preview", canvas);
            } else {
                cv::Mat canvas = cv::Mat::zeros(g_win_h, g_win_w, CV_8UC3);
                int sel = g_enlarged_cam.load();
                renderThumbnailGrid(canvas, sel, is_recording, record_start_time, total_record_frames);
                renderEnlargedView(canvas, sel, is_recording, record_start_time, total_record_frames);
                cv::line(canvas, cv::Point(g_left_w, 0), cv::Point(g_left_w, g_win_h), cv::Scalar(60, 60, 60), 2);

                // LED state indicator (Master, top-left of canvas)
                if (is_master_pc) drawLedIndicator(canvas);

                // 右上角 (LED 指示下方): HDF5 chunk/offset + 每臂本次写入进度 (各占一行)
                {
                    string quota = (g_num_targets_per_arm > 0) ? to_string(g_num_targets_per_arm) : "-";
                    char lb[96]; int bl = 0;
                    snprintf(lb, sizeof(lb), "chunk: %d   offset: %d", g_chunk_idx, g_frame_offset.load());
                    cv::Size wsz = cv::getTextSize(lb, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &bl);
                    cv::putText(canvas, lb, cv::Point(g_win_w - wsz.width - 15, 60),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);
                    snprintf(lb, sizeof(lb), "UPPER: %d/%s", g_recorded_upper, quota.c_str());
                    wsz = cv::getTextSize(lb, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &bl);
                    cv::putText(canvas, lb, cv::Point(g_win_w - wsz.width - 15, 84),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 200), 2, cv::LINE_AA);
                    snprintf(lb, sizeof(lb), "LOWER: %d/%s", g_recorded_lower, quota.c_str());
                    wsz = cv::getTextSize(lb, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &bl);
                    cv::putText(canvas, lb, cv::Point(g_win_w - wsz.width - 15, 108),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 200), 2, cv::LINE_AA);
                }

                // Watermark hints (bottom-left of right panel)
                int hx = g_right_x + 10, hy = g_win_h - 60;
                string hints;
                if (g_delay_pending.load()) hints = "[DELAY] Focus on the cross...";
                else if (is_recording) hints = "[REC] Recording in progress...";
                else if (is_dumping) hints = "[DUMP] Writing to disk...";
                else if (g_syncing.load()) hints = "Syncing sentry - please wait...";
                else if (enable_net_sync && !is_master_pc) hints = "[s][space] disabled (Slave) | Waiting for Master...";
                else if (is_master_pc) {
                    if (!g_recording_enabled) hints = "[s] Start session  [SPACE/b/c/t]  [ESC/q] quit";
                    else hints = "[SPACE] Record  [t] Switch arm  [b] Zero  [c] Clear  [ESC/q] quit";
                }
                cv::putText(canvas, hints, cv::Point(hx, hy), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
                hy += 18;
                string role = enable_net_sync ? (is_master_pc ? "Role: MASTER | Net Sync: ON" : "Role: SLAVE | Net Sync: ON") : "Role: STANDALONE | Net Sync: OFF";
                cv::putText(canvas, role, cv::Point(hx, hy), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
                // Piper status line (Master only)
                if (is_master_pc) {
                    hy += 16; char pb[128];
                    snprintf(pb, sizeof(pb), "Arm: %s | U:%d/%d%s | L:%d/%d%s | %s",
                        g_arm.c_str(), g_upper_idx, (int)g_targets_upper.size(), g_upper_done?" DONE":"",
                        g_lower_idx, (int)g_targets_lower.size(), g_lower_done?" DONE":"",
                        g_piper_busy?"BUSY":(g_recording_enabled?"Ready":""));
                    cv::putText(canvas, pb, cv::Point(hx, hy), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
                    if (g_tool_ccs_valid) {
                        hy += 16; snprintf(pb, sizeof(pb), "Tool CCS: [%.4f %.4f %.4f]  Gaze: [%.4f %.4f %.4f]",
                            g_tool_ccs_pos.x, g_tool_ccs_pos.y, g_tool_ccs_pos.z,
                            g_gaze_x.load(), g_gaze_y.load(), g_gaze_z.load());
                        cv::putText(canvas, pb, cv::Point(hx, hy), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 200), 1, cv::LINE_AA);
                    }
                }

                // Bottom-right 水印: participant + arm pose (位于状态行上方, 右对齐)
                string wm = "Participant: " + g_participant_id;
                cv::Size wsz = cv::getTextSize(wm, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, 0);
                cv::putText(canvas, wm, cv::Point(g_right_x + g_right_w - wsz.width - 10, g_win_h - 95),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
                if (is_master_pc && !g_arm_pose_yml.empty()) {
                    string ap = "Arm pose: " + g_arm_pose_yml;
                    cv::Size apsz = cv::getTextSize(ap, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, 0);
                    cv::putText(canvas, ap, cv::Point(g_right_x + g_right_w - apsz.width - 10, g_win_h - 75),
                                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
                }

                // Crosshair at center of enlarged area
                int cx = g_right_x + g_right_w / 2, cy = g_win_h / 2, cl = 20;
                cv::line(canvas, cv::Point(cx - cl, cy), cv::Point(cx + cl, cy), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
                cv::line(canvas, cv::Point(cx, cy - cl), cv::Point(cx, cy + cl), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

                cv::imshow("Multi-Cam Preview", canvas);
            }
            last_ui_time = current_time;
        }

        // ---- Dump wait logic ----
        if (is_recording&&!is_dumping) {
            bool all_done=true;
            for(auto& ctx:cam_ctxs) if(!ctx->dump_ready.load()){all_done=false;break;}
            if (all_done) {
                g_consecutive_faults=0; is_recording=false; is_dumping=true; g_recording_number++;
                if(is_master_pc) setLedState(LedState::WAITING);  // update before loading screens
                // OVER 检查: 本臂成功录制计数达到 num_targets_per_arm (运行期独立计数)
                // → 在提前移动机械臂之前进入 OVER
                if (is_master_pc && g_num_targets_per_arm > 0) {
                    recordedOf()++;
                    cout << "[Piper] " << g_arm << " recorded " << recordedOf()
                         << "/" << g_num_targets_per_arm << endl;
                    if (recordedOf() >= g_num_targets_per_arm) {
                        overOf() = true;
                        g_show_over = true;
                        g_recording_enabled = false;
                        cout << "[Piper] " << g_arm << " reached " << g_num_targets_per_arm
                             << " recordings — OVER." << endl;
                    }
                }
                auto t0=chrono::steady_clock::now();

                // Save old gaze (recording #N position) before arm overwrites it
                double rec_gaze_x = g_gaze_x.load();
                double rec_gaze_y = g_gaze_y.load();
                double rec_gaze_z = g_gaze_z.load();

                // ====== PARALLEL: ARM (thread) + HDF5 (main thread) ======
                { cv::Mat loading=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
                  cv::putText(loading,"ARM + HDF5 STAGE ("+to_string(cam_ctxs.size())+" cameras)",
                              cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,1.0,cv::Scalar(0,200,255),2);
                  char gb[128];snprintf(gb,sizeof(gb),"Gaze (rec #%d): [%.4f, %.4f, %.4f]",g_recording_number,rec_gaze_x,rec_gaze_y,rec_gaze_z);
                  cv::putText(loading,gb,cv::Point(g_win_w/4,g_win_h/2+40),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,255,0),1);
                  if(is_master_pc) drawLedIndicator(loading);
                  cv::imshow("Multi-Cam Preview",loading);cv::waitKey(1); }

                auto t_par0=chrono::steady_clock::now();

                // 6.4: Check exhaustion BEFORE launching arm thread
                if(is_master_pc && ((g_arm=="upper" && g_upper_idx >= (int)g_targets_upper.size()) ||
                                    (g_arm=="lower" && g_lower_idx >= (int)g_targets_lower.size()))) {
                    g_show_exhausted = true;
                    cout<<"[Piper] "<<g_arm<<" targets exhausted (pre-check). Skipping ARM."<<endl;
                }
                // Master: start arm thread (parallel with HDF5); OVER 时不移动
                thread arm_thread;
                if (is_master_pc && !g_show_exhausted && !g_show_over) {
                    cout<<"[Piper] Moving to next target..."<<endl;
                    arm_thread=thread([&](){
                        ArmResult ar = moveArmToTarget();
                        if(ar==ArmResult::ARM_EXHAUSTED){
                            g_show_exhausted = true;
                        } else {
                            logException("ERROR","piper","arm move failed");
                        }
                    });
                }

                // Step 0: Pre-create HDF5 files (小数据集先创建 — 布局须在 raw_image 前, 见 precreateSerial 注释)
                for (auto& ctx:cam_ctxs){ctx->dump_start_time=chrono::steady_clock::now();
                    stringstream pss;pss<<ctx->hdf5_dir<<"/"<<setw(4)<<setfill('0')<<g_chunk_idx<<".h5";
                    if(!fs::exists(pss.str())){try{H5::H5File f(pss.str(),H5F_ACC_TRUNC);
                        H5::DSetCreatPropList pl=allocEarlyPl();
                        hsize_t gd[2]={(hsize_t)g_hdf5_chunk_capacity,3};
                        f.createDataSet("gaze_target",H5::PredType::NATIVE_DOUBLE,H5::DataSpace(2,gd),pl);
                        hsize_t vd[1]={(hsize_t)g_hdf5_chunk_capacity};
                        f.createDataSet("valid",H5::PredType::NATIVE_UINT8,H5::DataSpace(1,vd),pl);
                        hsize_t rd[3]={(hsize_t)g_hdf5_chunk_capacity,(hsize_t)cam_h,(hsize_t)cam_w};
                        f.createDataSet("raw_image",H5::PredType::NATIVE_UINT8,H5::DataSpace(3,rd),pl);}
                        catch(const H5::Exception& e){logException("ERROR","hdf5:precreate",e.getCDetailMsg());}}}
                auto t_pre=chrono::steady_clock::now();

                // Step 1: Launch child processes
                char exe_path[MAX_PATH];GetModuleFileNameA(NULL,exe_path,MAX_PATH);
                string parent_dir=fs::path(exe_path).parent_path().string();
                string child_exe=parent_dir+"\\hdf5_multi_process_child.exe";
                HANDLE hJob=CreateJobObjectA(NULL,NULL); if(hJob){JOBOBJECT_EXTENDED_LIMIT_INFORMATION jeli{};jeli.BasicLimitInformation.LimitFlags=JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;SetInformationJobObject(hJob,JobObjectExtendedLimitInformation,&jeli,sizeof(jeli));}
                vector<PROCESS_INFORMATION> procs(cam_ctxs.size());
                for (size_t i=0;i<cam_ctxs.size();++i){auto& ctx=cam_ctxs[i];
                    string shm_name="HDF5_"+to_string(GetCurrentProcessId())+"_CAM_"+to_string(i);
                    stringstream args; args<<"\"hdf5_multi_process_child.exe\" "<<i<<" \""<<ctx->hdf5_dir<<"\" "<<g_chunk_idx
                        <<" "<<g_frame_offset<<" "<<core_frames<<" "<<cam_h<<" "<<cam_w<<" "<<margin_frames<<" "<<shm_name
                        <<" "<<rec_gaze_x<<" "<<rec_gaze_y<<" "<<rec_gaze_z;
                    STARTUPINFOA si{sizeof(si)};PROCESS_INFORMATION pi{};
                    string cmd_line=args.str();
                    if(CreateProcessA(child_exe.c_str(),&cmd_line[0],NULL,NULL,FALSE,0,NULL,NULL,&si,&pi)){
                        CloseHandle(pi.hThread);if(hJob)AssignProcessToJobObject(hJob,pi.hProcess);procs[i]=pi;}
                    else{logException("ERROR","hdf5:proc","CreateProcess failed cam "+to_string(i));procs[i].hProcess=NULL;}}
                auto t_launch=chrono::steady_clock::now();

                // Step 2-3: Wait for children + cleanup
                vector<HANDLE> handles;for(auto&p:procs)if(p.hProcess)handles.push_back(p.hProcess);
                if(!handles.empty())WaitForMultipleObjects((DWORD)handles.size(),handles.data(),TRUE,INFINITE);
                auto t_hdf5_done=chrono::steady_clock::now();
                bool all_ok=true;
                for(auto&p:procs){if(!p.hProcess){all_ok=false;continue;}DWORD ec;if(GetExitCodeProcess(p.hProcess,&ec)&&ec!=0){all_ok=false;logException("ERROR","hdf5:cam","child exit "+to_string(ec));}CloseHandle(p.hProcess);}
                if(hJob)CloseHandle(hJob);
                for(auto& ctx:cam_ctxs)ctx->dump_end_time=chrono::steady_clock::now();

                // Join arm thread (if Master)
                if(arm_thread.joinable()) arm_thread.join();
                auto t_arm_done=chrono::steady_clock::now();

                // ====== SYNC: Wait Slave HDF5 → GAZE forward → GAZE_DONE ======
                if (is_master_pc) {
                    // (1) Wait for Slave HDF5_DONE
                    auto t_wait0=chrono::steady_clock::now();
                    cout<<"[Sync] Waiting for Slave HDF5_DONE..."<<endl;
                    while(global_running && !g_slave_hdf5_done.load())
                        this_thread::sleep_for(chrono::milliseconds(50));
                    g_slave_hdf5_done = false;
                    auto t_wait1=chrono::steady_clock::now();
                    g_wait_slave_hdf5_s = chrono::duration<double>(t_wait1-t_wait0).count();
                    // (2)+(3) Forward gaze + GAZE_DONE
                    cout<<"[Gaze] Forwarding to Slave: ("<<g_gaze_x<<","<<g_gaze_y<<","<<g_gaze_z<<")"<<endl;
                    char gbuf[128]; snprintf(gbuf,sizeof(gbuf),"GAZE:%.6f,%.6f,%.6f",g_gaze_x.load(),g_gaze_y.load(),g_gaze_z.load());
                    { lock_guard<mutex> lk(g_gaze_send_mtx);
                      send(g_gaze_sock, (string(gbuf)+"\n").c_str(), (int)strlen(gbuf)+1, 0); }
                    string ack; recvLine(g_gaze_sock, ack, 5000);
                    cout<<"[Gaze] Slave ACK: "<<ack<<endl;
                    sendLineRaw(g_cmd_sock, "GAZE_DONE");
                    auto t_gaze_done=chrono::steady_clock::now();
                    g_gaze_forward_s = chrono::duration<double>(t_gaze_done-t_wait1).count();
                } else {
                    // Signal Master: HDF5 done + timing
                    char hbuf[64]; snprintf(hbuf,sizeof(hbuf),"HDF5_DONE:%.3f",g_slave_hdf5_s);
                    sendLineRaw(g_cmd_sock, hbuf);
                    // Wait for gaze from Master
                    g_gaze_need_send = false;
                    cout<<"[Sync] Waiting for Master GAZE..."<<endl;
                    while(global_running && !g_gaze_need_send.load())
                        this_thread::sleep_for(chrono::milliseconds(50));
                    g_gaze_need_send = false;
                    // Wait for GAZE_DONE from Master (cmdWorker sets g_gaze_done)
                    cout<<"[Sync] Waiting for Master GAZE_DONE..."<<endl;
                    while(global_running && !g_gaze_done.load())
                        this_thread::sleep_for(chrono::milliseconds(50));
                    g_gaze_done = false;
                }
                auto t_sync_done=chrono::steady_clock::now();

                // Step 4: Post-dump sentry handshake
                auto t_sentry0=chrono::steady_clock::now();
                if (enable_net_sync) {
                    g_syncing=true;
                    int handshake_port=net_port+300;
                    auto local_ci=g_chunk_idx; auto local_fo=g_frame_offset.load();
                    int64_t local_total=(int64_t)local_ci*g_hdf5_chunk_capacity+local_fo;
                    if (is_master_pc) {
                        SOCKET hs=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);int opt=1;setsockopt(hs,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
                        sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(handshake_port);sa.sin_addr.s_addr=INADDR_ANY;
                        ::bind(hs,(sockaddr*)&sa,sizeof(sa));listen(hs,1);sockaddr_in ca;socklen_t cl=sizeof(ca);SOCKET cs=accept(hs,(sockaddr*)&ca,&cl);
                        if(cs!=INVALID_SOCKET){int64_t peer_buf[2];recv(cs,(char*)peer_buf,sizeof(peer_buf),0);int64_t peer_total=peer_buf[0];
                            int64_t send_buf[2]={local_total,local_total};send(cs,(const char*)send_buf,sizeof(send_buf),0);closesocket(cs);
                            int64_t peer_val=peer_total,local_val=local_total;
                            cout<<"[Sentry] Post-dump handshake: Local="<<local_val<<" Peer="<<peer_val<<endl;
                            if(peer_val!=local_val){g_sentry_mismatch_count++;int peer_ci=(int)(peer_val/g_hdf5_chunk_capacity),peer_fo=(int)(peer_val%g_hdf5_chunk_capacity);
                                if(peer_val<local_val){g_chunk_idx=peer_ci;g_frame_offset=peer_fo;}
                                logException("WARN","sentry","Mismatch #"+to_string(g_sentry_mismatch_count)+" diff="+to_string(llabs(local_val-peer_val))+" - using min");
                                if(g_sentry_mismatch_count>=3){logException("FATAL","sentry","3 consecutive mismatches - exiting.");global_running=false;}}
                            else{g_sentry_mismatch_count=0;}}
                        closesocket(hs);
                    } else {
                        SOCKET hs=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);sockaddr_in sa{};sa.sin_family=AF_INET;sa.sin_port=htons(handshake_port);
                        inet_pton(AF_INET,g_master_ip.c_str(),&sa.sin_addr);
                        while(connect(hs,(sockaddr*)&sa,sizeof(sa))==-1&&global_running){this_thread::sleep_for(chrono::milliseconds(100));cv::waitKey(1);}
                        if(hs!=INVALID_SOCKET){int64_t send_buf[2]={local_total,local_total};send(hs,(const char*)send_buf,sizeof(send_buf),0);
                            int64_t peer_buf[2];recv(hs,(char*)peer_buf,sizeof(peer_buf),0);int64_t peer_total=peer_buf[0];closesocket(hs);
                            int64_t peer_val=peer_total,local_val=local_total;
                            cout<<"[Sentry] Post-dump handshake: Local="<<local_val<<" Peer="<<peer_val<<endl;
                            if(peer_val!=local_val){g_sentry_mismatch_count++;int peer_ci=(int)(peer_val/g_hdf5_chunk_capacity),peer_fo=(int)(peer_val%g_hdf5_chunk_capacity);
                                if(peer_val<local_val){g_chunk_idx=peer_ci;g_frame_offset=peer_fo;}
                                logException("WARN","sentry","Mismatch #"+to_string(g_sentry_mismatch_count)+" diff="+to_string(llabs(local_val-peer_val))+" - using min");
                                if(g_sentry_mismatch_count>=3){logException("FATAL","sentry","3 consecutive mismatches - exiting.");global_running=false;}}
                            else{g_sentry_mismatch_count=0;}}
                    }
                    g_syncing=false;
                }
                auto t_sentry1=chrono::steady_clock::now();

                // Step 5: Increment sentry AFTER handshake
                cout<<"[DEBUG-HDF5] frame_offset "<<g_frame_offset<<" -> "<<(g_frame_offset+core_frames)<<endl;
                if(all_ok){g_frame_offset+=core_frames;if(g_frame_offset>=g_hdf5_chunk_capacity){g_chunk_idx++;g_frame_offset-=g_hdf5_chunk_capacity;}updateSentry(g_sentry_root);}
                else{logException("WARN","hdf5","Child failures - sentry NOT updated");}

                // ---- Per-phase timing ----
                auto d_par=chrono::duration<double>(t_arm_done-t_par0).count();
                auto d_pre=chrono::duration<double>(t_pre-t_par0).count();
                auto d_launch=chrono::duration<double>(t_launch-t_pre).count();
                auto d_wait=chrono::duration<double>(t_hdf5_done-t_launch).count();
                auto d_arm_wait=chrono::duration<double>(t_arm_done-t_hdf5_done).count();
                auto d_sync=chrono::duration<double>(t_sync_done-t_arm_done).count();
                auto d_sentry=chrono::duration<double>(t_sentry1-t_sentry0).count();
                auto d_total=chrono::duration<double>(t_sentry1-t0).count();
                // End-to-end = SPACE → SPACE-ready: capture_delay + 录制(含margin) + arm&h5 + sync/sentry
                auto e2e_start=chrono::steady_clock::time_point(chrono::microseconds(g_e2e_start_us.load()));
                g_recording_end_to_end_s = chrono::duration<double>(t_sentry1-e2e_start).count();
                g_e2e_start_us.store(0);
                // Store for report
                g_arm_stage_s = d_par;
                if(is_master_pc) g_master_hdf5_s = d_wait; else g_slave_hdf5_s = d_wait;
                g_sentry_sync_s = d_sentry;
                double d_sync_total = d_sync; // keep for console output
                cout<<"[Timing] par="<<fixed<<setprecision(2)<<d_par
                    <<"s (pre="<<d_pre<<" launch="<<d_launch
                    <<" hdf5="<<d_wait<<" armExtra="<<d_arm_wait
                    <<") waitSlave="<<g_wait_slave_hdf5_s<<" gazeFwd="<<g_gaze_forward_s
                    <<" sentry="<<d_sentry<<" TOTAL="<<d_total<<"s"<<endl;
                writeReport(current_record_timestr.empty()?"-":current_record_timestr, g_recording_number, total_record_frames, use_hw_trigger);
                if(is_master_pc && g_show_exhausted) syncPiperToSlave();
                cout<<"[Recording #"<<g_recording_number<<"] Done in "<<fixed<<setprecision(1)<<d_total<<"s\n";
                is_dumping=false;while(cv::waitKey(1)>=0);last_ui_time=chrono::steady_clock::now();
        }}

        // ---- AutoMove: READY 超时未按 SPACE → 跳过本次 capture+dump, 直接移动机械臂 ----
        if (is_master_pc && g_enable_auto_move && g_recording_enabled
            && !is_recording && !is_dumping && !g_piper_busy
            && !g_show_exhausted && !g_show_over) {
            auto ready_tp = chrono::steady_clock::time_point(chrono::microseconds(g_ready_since_us.load()));
            double waited = chrono::duration<double>(chrono::steady_clock::now() - ready_tp).count();
            if (waited > g_click_window) {
                cout << "[AutoMove] No SPACE within " << fixed << setprecision(1) << g_click_window
                     << "s — skipping this capture, moving " << g_arm << " to next target..." << endl;
                int& idx = (g_arm=="upper") ? g_upper_idx : g_lower_idx;
                idx++; updatePiperSentry();          // 目标被跳过 (不录制, 仍消耗)
                ArmResult ar = moveArmToTarget();
                if (ar == ArmResult::ARM_OK) {
                    cout << "[AutoMove] " << g_arm << " at next target. Waiting for SPACE..." << endl;
                } else if (ar == ArmResult::ARM_EXHAUSTED) {
                    g_show_exhausted = true;
                } else {
                    cout << "[AutoMove] Arm ERROR." << endl;
                }
            }
        }

        // ---- Capture delay: 等待期结束 → 真正开始录制 ----
        if (g_delay_pending.load() && nowUs() >= g_delay_deadline_us.load()) {
            g_delay_pending.store(false);
            g_last_delay_s = (nowUs() - g_delay_start_us.load()) / 1e6;
            instantTrigger();
            g_trigger_pending.store(true);
            if(enable_net_sync&&g_cmd_sock!=INVALID_SOCKET) sendLineRaw(g_cmd_sock,"TRIGGER");
            logException("INFO","delay",to_string(g_last_delay_s)+"s");
            cout<<"[Delay] "<<g_last_delay_s<<"s elapsed - recording started."<<endl;
        }

        // ---- HDF5 预创建状态 (i 键): 状态机 + 全屏进度, 屏蔽所有按键 ----
        if(g_precreating.load()){
            if(g_pre_phase==1){
                // 握手阶段: 等待 slave 预检回复 (15s 超时保护, 防止断连卡死)
                if(g_pre_peer_reject.load()||
                   chrono::duration<double>(chrono::steady_clock::now()-g_pre_t0).count()>15.0){
                    g_precreating=false;
                    cout<<(g_pre_peer_reject.load()?"[HDF5] Slave has existing h5 — aborted."
                                                   :"[HDF5] Slave pre-check timeout — aborted.")<<endl;
                }else if(g_pre_peer_clear.load()){
                    // 双端均无 h5 → 下令双方同时开始创建
                    g_pre_peer_done=false;g_pre_local_done=false;
                    sendLineRaw(g_cmd_sock,"PRECREATE_BEGIN");
                    if(g_pre_thread.joinable())g_pre_thread.join();
                    g_pre_thread=thread([]{precreateSerial();g_pre_local_done=true;});
                    g_pre_phase=2;
                    cout<<"[HDF5] Pre-create started on master + slave."<<endl;
                }
            }else if(g_pre_phase==2){
                // 创建阶段: slave 本机完成即退出 (PRECREATE_DONE 已作为确认发给 master);
                //           master 需等本机 + slave 双方完成
                bool peer_ok=(!enable_net_sync)||(!is_master_pc)||g_pre_peer_done.load();
                if(g_pre_local_done.load()&&peer_ok){
                    g_precreating=false;
                    cout<<"[HDF5] Pre-create complete on both hosts."<<endl;
                }
            }
            if(g_precreating.load()){
                cv::Mat pc=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
                string ph=(g_pre_phase==1)?string("Checking slave cameras..."):
                    (g_pre_local_done.load()?string("Waiting for slave to finish..."):string("Creating HDF5 files (serial)..."));
                cv::putText(pc,"PRE-CREATING HDF5 FILES",cv::Point(g_win_w/4,g_win_h/2-70),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,200,255),2);
                cv::putText(pc,ph,cv::Point(g_win_w/4,g_win_h/2-20),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255),1);
                int tot=g_pre_total.load(),dn=g_pre_done.load();
                if(tot>0){
                    int bw=g_win_w/2,bx=(g_win_w-bw)/2,by=g_win_h/2+20;
                    cv::rectangle(pc,cv::Rect(bx,by,bw,30),cv::Scalar(80,80,80),1);
                    int pw=(int)(bw*(double)dn/tot);
                    if(pw>0)cv::rectangle(pc,cv::Rect(bx,by,pw,30),cv::Scalar(0,255,0),-1);
                    char pb[64];snprintf(pb,sizeof(pb),"%d / %d files",dn,tot);
                    cv::putText(pc,pb,cv::Point(bx+bw/2-60,by+21),cv::FONT_HERSHEY_SIMPLEX,0.55,cv::Scalar(255,255,255),1);
                }
                cv::putText(pc,"Do not press any key - this may take a while",cv::Point(g_win_w/4,g_win_h/2+90),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(150,150,150),1);
                cv::imshow("Multi-Cam Preview",pc);
                while(cv::waitKey(1)>=0){}   // 创建期间吞掉所有按键 (含 ESC, 中断会产生残缺 h5)
                goto next_iter;
            }
        }

        // ---- Key handling ----
        char key=(char)cv::waitKey(1); bool trigger_start=false;
        if(g_syncing.load()){}
        else if(key=='q'||key==27){
            if(is_master_pc){
                cout<<"\nExiting - zeroing both arms..."<<endl;
                zeroArm("upper"); zeroArm("lower");
                sendLineRaw(g_piper_sock,"SHUTDOWN"); string ack; recvLine(g_piper_sock,ack,2000);
                if(enable_net_sync&&g_cmd_sock!=INVALID_SOCKET){sendLineRaw(g_cmd_sock,"EXIT");this_thread::sleep_for(chrono::milliseconds(200));}
                global_running=false;
            }else{if(g_fault_active.load())global_running=false;}
        }else if(g_fault_active.load()){}
        else if(g_delay_pending.load()){/* delay 等待期间仅 ESC/q 可用 */}
        // ---- Master-only keys ----
        else if(is_master_pc&&key=='s'&&!is_recording&&!is_dumping&&!g_piper_busy&&!g_show_over){
            bool all_streaming=true;
            for(auto& ctx:cam_ctxs) if(ctx->status.load()!=CamStatus::STREAMING){all_streaming=false;break;}
            if(!all_streaming){cout<<"[s] Waiting for cameras to be STREAMING..."<<endl;goto next_iter;}
            if((g_arm=="upper"&&g_upper_done)||(g_arm=="lower"&&g_lower_done)){
                cout<<"[s] "<<g_arm<<" done - press T to switch arm"<<endl;goto next_iter;}
            cout<<"[s] Starting session - moving "<<g_arm<<" to sentry target..."<<endl;
            { cv::Mat loading=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
              cv::putText(loading,"Waiting for arm to get in position...",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,200,255),2);
              if(is_master_pc) drawLedIndicator(loading);
              cv::putText(loading,"Arm: "+g_arm,cv::Point(g_win_w/4,g_win_h/2+40),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255),1);
              cv::imshow("Multi-Cam Preview",loading);cv::waitKey(1); }
            ArmResult ar = moveArmToTarget();
            if(ar==ArmResult::ARM_OK){
                this_thread::sleep_for(chrono::milliseconds(200));g_recording_enabled=true;
                cout<<"[s] Session started. SPACE to record."<<endl;
            } else if(ar==ArmResult::ARM_EXHAUSTED){
                g_show_exhausted=true; g_recording_enabled=false;
                cout<<"[s] "<<g_arm<<" exhausted. Press t or c."<<endl;
            } else { cout<<"[s] Arm ERROR."<<endl; }
        }
        else if(is_master_pc&&key==' '&&!is_recording&&!is_dumping&&!g_piper_busy&&!g_delay_pending){
            if(g_recording_enabled){
                g_e2e_start_us.store(nowUs());   // End-to-end 起点: SPACE 按下时刻
                if(g_capture_delay>0.0){
                    // 进入 CAPTURING, 延迟 capture_delay 后再真正录制 (受试者聚焦十字中心)
                    setLedState(LedState::CAPTURING);
                    g_delay_start_us.store(nowUs());
                    g_delay_deadline_us.store(g_delay_start_us.load()+(int64_t)(g_capture_delay*1e6));
                    g_delay_pending.store(true);
                    cout<<"[Delay] Capturing - waiting "<<g_capture_delay<<"s before recording..."<<endl;
                }else{
                    instantTrigger(); trigger_start=true;
                    if(enable_net_sync&&g_cmd_sock!=INVALID_SOCKET) sendLineRaw(g_cmd_sock,"TRIGGER");
                    cout<<"[Recording] Started."<<endl;
                }
            } else {
                cout<<"[SPACE] Recording not enabled. Press [s] first."<<endl;
                if(g_upper_done&&g_lower_done) cout<<"[SPACE] Both arms DONE. Press [c] to clear sentry."<<endl;
            }
        }
        else if(is_master_pc&&(key=='b'||key=='B')&&!g_piper_busy&&!is_recording&&!is_dumping){
            g_recording_enabled=false;               // 回零后需按 s 才能录制
            g_show_exhausted=false;
            setLedState(LedState::WAITING);          // 立刻进入 WAITING 状态
            { cv::Mat loading=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
              cv::putText(loading,"Zeroing arm...",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,200,255),2);
              drawLedIndicator(loading);
              cv::imshow("Multi-Cam Preview",loading);cv::waitKey(1); }
            zeroArm(g_arm);
            // 回零 = 重置本臂 OVER 标志与录制计数 (运行期独立计数)
            overOf()=false; recordedOf()=0; g_show_over=false;
            setLedState(LedState::PIPER_INIT);       // 回零完成 → INIT (按 s 开始录制)
            cout<<"[Piper] "<<g_arm<<" zeroed. Press [s] to start session."<<endl;
        }
        else if(is_master_pc&&(key=='c'||key=='C')&&!g_piper_busy&&!is_recording&&!is_dumping&&!g_show_over){
            // Clear only current arm's piper sentry (OVER 状态下禁用, 只能 b/t/q)
            if(g_arm=="upper"){g_upper_idx=0;g_upper_done=false;}
            else{g_lower_idx=0;g_lower_done=false;}
            g_show_exhausted=false;
            overOf()=false; recordedOf()=0;
            g_recording_enabled=false; updatePiperSentry();
            syncPiperToSlave();
            cout<<"[Piper] "<<g_arm<<" sentry cleared."<<endl;
        }
        else if(is_master_pc&&(key=='i'||key=='I')&&!g_piper_busy&&!is_recording&&!is_dumping&&!g_precreating.load()){
            // 提前创建 25×N 个 h5: 双机握手预检 → 各自串行创建 → 握手退出
            if(anyLocalH5()){cout<<"[HDF5] Existing h5 on master — pre-create skipped."<<endl;}
            else if(!enable_net_sync){
                // 单机模式: 直接创建 (无握手)
                g_pre_phase=2;g_precreating=true;g_pre_local_done=false;
                if(g_pre_thread.joinable())g_pre_thread.join();
                g_pre_thread=thread([]{precreateSerial();g_pre_local_done=true;});
            }else{
                // 双机模式: 先握手检查 slave (master 本机已确认无 h5)
                g_pre_peer_clear=false;g_pre_peer_reject=false;
                g_precreating=true;g_pre_phase=1;g_pre_t0=chrono::steady_clock::now();
                sendLineRaw(g_cmd_sock,"PRECHECK_REQ");
                cout<<"[HDF5] Master clear. Pre-checking slave..."<<endl;
            }
        }
        else if(is_master_pc&&(key=='t'||key=='T')&&!g_piper_busy&&!is_recording&&!is_dumping){
            string new_arm=(g_arm=="upper")?"lower":"upper";
            bool nd=(new_arm=="upper")?g_upper_done:g_lower_done;
            if(nd){cout<<"[t] "<<new_arm<<" already done."<<endl;goto next_iter;}
            g_recording_enabled=false;               // 切换后需按 s 才能录制
            g_show_exhausted=false;
            setLedState(LedState::WAITING);          // 立刻进入 WAITING 状态
            { cv::Mat loading=cv::Mat::zeros(g_win_h,g_win_w,CV_8UC3);
              cv::putText(loading,"Zeroing arm...",cv::Point(g_win_w/4,g_win_h/2),cv::FONT_HERSHEY_DUPLEX,0.9,cv::Scalar(0,200,255),2);
              drawLedIndicator(loading);
              cv::imshow("Multi-Cam Preview",loading);cv::waitKey(1); }
            zeroArm(g_arm);
            // 原来的机械臂 → INIT (其设备回到 INIT 图案; 即使已 OVER, 不在工作状态即显示蓝色)
            if (g_arm == "upper") {
                g_led_state_upper = LedState::PIPER_INIT;
                sendLedPatternTo(g_led_upper, LedState::PIPER_INIT);
            } else {
                g_led_state_lower = LedState::PIPER_INIT;
                sendLedPatternTo(g_led_lower, LedState::PIPER_INIT);
            }
            g_arm=new_arm;
            // 加载新臂的 OVER 状态 (每臂独立标志, 切多少次都保持)
            g_show_over = overOf();
            if (g_show_over) {
                setLedState(LedState::OVER);         // 新臂已 OVER → 彩流
            } else {
                setLedState(LedState::PIPER_INIT);   // 新臂 → INIT (按 s 开始录制)
            }
            syncPiperToSlave();
            cout<<"[t] Switched to "<<g_arm<<" (press 's' to start session)"<<endl;
        }

        bool trig_fired = trigger_start || g_trigger_pending.exchange(false);
        if((trig_fired||net_cmd_record.exchange(false))&&!is_recording&&!is_dumping){
            if(is_master_pc&&trig_fired){
                // 录制开始: 当前目标指令已被使用, 此时才推进 sentry (按 t/b 放弃时不推进)
                int& idx=(g_arm=="upper")?g_upper_idx:g_lower_idx;
                idx++; updatePiperSentry();
            }
            // 吞掉快速连按/自动重复的残留按键 (此时它们已无效), 防止 READY 下 SPACE 二次触发
            while(cv::waitKey(1)>=0){}
            current_record_timestr=shared_record_timestr;
            record_start_time=global_record_start_time;
            // End-to-end 起点兜底 (slave TRIGGER / 未走 SPACE 路径): 触发时刻
            if(g_e2e_start_us.load()==0) g_e2e_start_us.store(nowUs());
            is_recording=true; cout<<"[Info] Recording in progress..."<<endl;
        }
        next_iter:;
    }

    // ---- Cleanup ----
    cout<<"[System] Shutting down..."<<endl;
    if(g_pre_thread.joinable())g_pre_thread.join();   // HDF5 预创建线程
    for(auto& ctx:cam_ctxs){ctx->running=false;ctx->copy_cv.notify_all();}
    // 退出前熄灭两个 M5Stack
    sendLedCmd(g_led_upper,"CLEAR");
    sendLedCmd(g_led_lower,"CLEAR");
    this_thread::sleep_for(chrono::milliseconds(50));   // 等待串口写出
    if(g_led_upper!=INVALID_HANDLE_VALUE){CloseHandle(g_led_upper);g_led_upper=INVALID_HANDLE_VALUE;}
    if(g_led_lower!=INVALID_HANDLE_VALUE){CloseHandle(g_led_lower);g_led_lower=INVALID_HANDLE_VALUE;}
    if(g_piper_sock!=INVALID_SOCKET) closesocket(g_piper_sock);
    if(g_gaze_sock!=INVALID_SOCKET) closesocket(g_gaze_sock);
    if(g_gaze_listen_sock!=INVALID_SOCKET) closesocket(g_gaze_listen_sock);
    global_running=false;
    if(g_cmd_sock!=INVALID_SOCKET) closesocket(g_cmd_sock);
    if(g_cmd_listen_sock!=INVALID_SOCKET) closesocket(g_cmd_listen_sock);  // 打断 accept
    for(auto& ctx:cam_ctxs){
        if(ctx->capture_thread.joinable())ctx->capture_thread.join();
        if(ctx->copy_thread.joinable())ctx->copy_thread.join();
    }
    if(cmd_thread.joinable()) cmd_thread.join();
    if(gaze_thread.joinable()) gaze_thread.join();
    if(g_session_log.is_open()) g_session_log.close();
    cv::destroyAllWindows(); Pylon::PylonTerminate();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
