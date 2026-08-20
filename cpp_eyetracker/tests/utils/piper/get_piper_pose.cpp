// ================== get_piper_pose ==================
// 实时获取 Piper 机械臂 flange 位姿 (XYZ + 四元数 + Z-X-Z'' 欧拉角)
// + flange / locating tool 在 CCS 中的位置 (arm_in_ccs 从 arm_pose yaml 读取,
//   locating tool 偏移从 piper.yaml 读取, 计算同 capture_with_M5Stack)
// + r 进入 IR 发射器位置记录模式: SPACE 记录/覆盖 locating tool CCS, ENTER 切换左右中,
//   s 保存 (三个全部记录后) 到 cfg/IR/{calib_arm.yaml: day_id}.txt (三行: 左/右/中)
// 用法: t 切换 upper/lower, g 打印位姿, r IR 记录, q 退出
// =================================================================
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
    #define SOCKET int
    #define INVALID_SOCKET -1
    #define closesocket close
#endif

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <iomanip>

#include "cfg/config.hpp"
#include "piper/piper.hpp"

using namespace std;

struct FlangePose {
    double x=0, y=0, z=0;
    double qx=0, qy=0, qz=0, qw=1.0;
    double alpha=0, beta=0, gamma=0;
    bool valid = false;
};

atomic<bool> g_running{true};
mutex g_pose_mtx;
FlangePose g_upper_pose;
FlangePose g_lower_pose;
string g_current_arm = "upper";
string g_status = "Disconnected";
string g_ubuntu_ip;
int g_port = 0;

// locating tool 在法兰盘坐标系中的位姿 + 臂基座在 CCS 中的位姿 (piper.yaml)
struct ArmXf {
    Pt3 loc_t, loc_r, ccs_t, ccs_r;
};
ArmXf g_xf_upper, g_xf_lower;

// IR 发射器位置记录模式: r 进入, SPACE 记录 locating tool CCS, ENTER 切换左右中
bool g_ir_mode = false;
int  g_ir_slot = 0;                    // 0=left 1=right 2=center
Pt3  g_ir_val[3];
bool g_ir_rec[3] = {false, false, false};
string g_ir_path;                      // cfg/IR/{day_id}.txt (绝对路径, 写入用)
string g_ir_path_disp;                 // UI 水印: 从 cfg 开始的相对路径
static const char* kIrSlotName[3] = {"LEFT", "RIGHT", "CENTER"};

void writeIrFile() {
    std::filesystem::create_directories(std::filesystem::path(g_ir_path).parent_path());
    ofstream out(g_ir_path);
    for (int i = 0; i < 3; ++i)
        out << fixed << setprecision(4)
            << g_ir_val[i].x << "," << g_ir_val[i].y << "," << g_ir_val[i].z << "\n";
    out.close();
    cout << "[IR] Written " << g_ir_path << " (LEFT/RIGHT/CENTER)" << endl;
}

// ================== TCP ==================

// 从 TCP 流中读取完整一行 (以 \n 结尾)
bool recvLine(SOCKET sock, string& line, int timeout_ms = 3000) {
#ifdef _WIN32
    DWORD to = timeout_ms;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
#endif
    char buf[256];
    string acc;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);

    while (chrono::steady_clock::now() < deadline) {
        int n = recv(sock, buf, sizeof(buf) - 1, 0);
        if (n <= 0) return false;
        buf[n] = '\0';
        acc += buf;
        size_t nl = acc.find('\n');
        if (nl != string::npos) {
            line = acc.substr(0, nl);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
    }
    return false;
}

bool parsePoseResponse(const string& resp, string& arm, FlangePose& pose) {
    if (resp.rfind("POSE:", 0) != 0) return false;
    string data = resp.substr(5);
    size_t p1 = data.find(':');
    if (p1 == string::npos) return false;
    arm = data.substr(0, p1);
    string vals = data.substr(p1 + 1);
    vector<double> nums;
    stringstream ss(vals); string token;
    while (getline(ss, token, ',')) {
        try { nums.push_back(stod(token)); } catch (...) { return false; }
    }
    if (nums.size() != 10) return false;
    pose.x=nums[0]; pose.y=nums[1]; pose.z=nums[2];
    pose.qx=nums[3]; pose.qy=nums[4]; pose.qz=nums[5]; pose.qw=nums[6];
    pose.alpha=nums[7]; pose.beta=nums[8]; pose.gamma=nums[9];
    pose.valid = true;
    return true;
}

void tcpWorker() {
    while (g_running) {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) { this_thread::sleep_for(chrono::seconds(2)); continue; }

        sockaddr_in server{};
        server.sin_family = AF_INET;
        server.sin_port = htons(g_port);
        inet_pton(AF_INET, g_ubuntu_ip.c_str(), &server.sin_addr);

        g_status = "Connecting...";
        if (connect(sock, (sockaddr*)&server, sizeof(server)) != 0) {
            g_status = "Connect failed, retry...";
            closesocket(sock);
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }
        g_status = "Connected";

        while (g_running) {
            string cmd = "GET_POSE:" + g_current_arm + "\n";
            if (send(sock, cmd.c_str(), (int)cmd.length(), 0) <= 0) break;

            string line;
            if (!recvLine(sock, line, 3000)) break;

            string arm;
            FlangePose pose;
            if (parsePoseResponse(line, arm, pose)) {
                lock_guard<mutex> lock(g_pose_mtx);
                if (arm == "upper") g_upper_pose = pose;
                else if (arm == "lower") g_lower_pose = pose;
            }
            this_thread::sleep_for(chrono::milliseconds(200));
        }

        g_status = "Disconnected";
        closesocket(sock);
        this_thread::sleep_for(chrono::seconds(1));
    }
}

// ================== UI ==================

void drawUI() {
    cv::namedWindow("Piper Arm Pose", cv::WINDOW_NORMAL);
    cv::resizeWindow("Piper Arm Pose", 640, 480);

    while (g_running) {
        cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC3);
        int y = 30;
        auto put = [&](const string& s, cv::Scalar c = {255,255,255}) {
            cv::putText(canvas, s, {20,y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, c, 1, cv::LINE_AA);
            y += 24;
        };

        cv::putText(canvas, "Piper Arm Pose Monitor", {160,y}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,255}, 2, cv::LINE_AA);
        y += 40;

        string st = g_status;
        put("Status: " + st, st == "Connected" ? cv::Scalar(0,255,0) : cv::Scalar(0,165,255));

        cv::Scalar ac = (g_current_arm=="upper") ? cv::Scalar(0,215,255) : cv::Scalar(200,80,255);
        put("Arm: " + g_current_arm + "  [t]switch  [g]print  [q]quit", ac);
        y += 10;

        FlangePose p; string dn;
        { lock_guard<mutex> lk(g_pose_mtx);
          if (g_current_arm=="upper"){p=g_upper_pose;dn="UPPER";}
          else{p=g_lower_pose;dn="LOWER";} }

        cv::putText(canvas, "--- " + dn + " FLANGE ---", {20,y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {200,200,200}, 1, cv::LINE_AA);
        y += 28;

        if (p.valid) {
            char b[128];
            snprintf(b,sizeof(b),"XYZ (m):      [%.4f, %.4f, %.4f]", p.x, p.y, p.z); put(b);
            snprintf(b,sizeof(b),"Quat (wxyz):  [%.4f, %.4f, %.4f, %.4f]", p.qw,p.qx,p.qy,p.qz); put(b);
            snprintf(b,sizeof(b),"Euler ZXZ'':  [%.2f, %.2f, %.2f] deg", p.alpha,p.beta,p.gamma);
            put(b, {0,255,0});
        } else { put("No pose data"); }

        // Flange in CCS (arm_in_ccs 变换; 零工具偏移复用 armToolToCamPose)
        cv::putText(canvas, "--- " + dn + " FLANGE (CCS) ---", {20,y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {200,200,200}, 1, cv::LINE_AA);
        y += 28;
        if (p.valid) {
            const auto& xf = (g_current_arm=="upper") ? g_xf_upper : g_xf_lower;
            Pose fl_ccs = armToolToCamPose(Pose{{p.x,p.y,p.z},{p.qx,p.qy,p.qz,p.qw}},
                                           {0,0,0}, {0,0,0}, xf.ccs_t, xf.ccs_r);
            char b[128];
            snprintf(b,sizeof(b),"XYZ (m):      [%.4f, %.4f, %.4f]",
                     fl_ccs.pos.x, fl_ccs.pos.y, fl_ccs.pos.z);
            put(b, {0,255,0});
        } else { put("No pose data"); }

        // Locating tool in CCS (变换同 capture_with_M5Stack: flange -> tool -> CCS)
        cv::putText(canvas, "--- " + dn + " LOCATING TOOL (CCS) ---", {20,y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {200,200,200}, 1, cv::LINE_AA);
        y += 28;
        if (p.valid) {
            const auto& xf = (g_current_arm=="upper") ? g_xf_upper : g_xf_lower;
            Pose lt_ccs = armToolToCamPose(Pose{{p.x,p.y,p.z},{p.qx,p.qy,p.qz,p.qw}},
                                           xf.loc_t, xf.loc_r, xf.ccs_t, xf.ccs_r);
            char b[128];
            snprintf(b,sizeof(b),"XYZ (m):      [%.4f, %.4f, %.4f]",
                     lt_ccs.pos.x, lt_ccs.pos.y, lt_ccs.pos.z);
            put(b, {0,255,0});
        } else { put("No pose data"); }

        // IR 记录模式状态 (r 进入/退出; SPACE 记录/覆盖, ENTER 切换, s 保存)
        if (g_ir_mode) {
            cv::putText(canvas, "--- IR RECORDER (" + g_ir_path_disp + ") ---", {20,y},
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, {0,200,255}, 1, cv::LINE_AA);
            y += 24;
            for (int i = 0; i < 3; ++i) {
                char b[160];
                if (g_ir_rec[i])
                    snprintf(b,sizeof(b),"%s: [%.4f, %.4f, %.4f]", kIrSlotName[i],
                             g_ir_val[i].x, g_ir_val[i].y, g_ir_val[i].z);
                else
                    snprintf(b,sizeof(b),"%s: [--]", kIrSlotName[i]);
                cv::Scalar c = (i == g_ir_slot) ? cv::Scalar(0,255,255)          // 当前槽高亮
                             : (g_ir_rec[i] ? cv::Scalar(0,255,0) : cv::Scalar(120,120,120));
                cv::putText(canvas, b, {40,y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
                y += 22;
            }
            cv::putText(canvas, "[SPACE] record/overwrite   [ENTER] switch   [s] save   [r] exit",
                        {20,y}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {150,150,150}, 1, cv::LINE_AA);
            y += 18;
        }

        y = 450;
        cv::putText(canvas, "[t] Switch arm   [g] Print pose   [r] IR record   [q/ESC] Quit",
                    {20,y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {150,150,150}, 1);

        cv::imshow("Piper Arm Pose", canvas);
        char key = (char)cv::waitKey(50);
        if (key=='q'||key==27) g_running = false;
        else if (key=='t'||key=='T') g_current_arm = (g_current_arm=="upper")?"lower":"upper";
        else if (key=='r'||key=='R') {
            if (g_ir_mode) {
                g_ir_mode = false;
                cout << "[IR] Recording mode OFF." << endl;
            } else {
                g_ir_mode = true;
                g_ir_slot = 0;
                for (int i = 0; i < 3; ++i) g_ir_rec[i] = false;
                cout << "[IR] Recording mode ON. Slot: LEFT."
                     << " SPACE=record  ENTER=switch  s=save  r=exit" << endl;
            }
        }
        else if (key=='s'||key=='S') {
            if (g_ir_mode) {
                if (g_ir_rec[0] && g_ir_rec[1] && g_ir_rec[2]) writeIrFile();
                else cout << "[IR] Not all three recorded yet — cannot save." << endl;
            }
        }
        else if (key==13) {   // ENTER: 切换左/右/中
            if (g_ir_mode) {
                g_ir_slot = (g_ir_slot + 1) % 3;
                cout << "[IR] Slot: " << kIrSlotName[g_ir_slot] << endl;
            }
        }
        else if (key==' ') {
            if (g_ir_mode) {
                if (!p.valid) {
                    cout << "[IR] No pose data — cannot record." << endl;
                } else {
                    const auto& xf = (g_current_arm=="upper") ? g_xf_upper : g_xf_lower;
                    Pose lt_ccs = armToolToCamPose(Pose{{p.x,p.y,p.z},{p.qx,p.qy,p.qz,p.qw}},
                                                   xf.loc_t, xf.loc_r, xf.ccs_t, xf.ccs_r);
                    g_ir_val[g_ir_slot] = lt_ccs.pos;
                    g_ir_rec[g_ir_slot] = true;
                    cout << fixed << setprecision(4);
                    cout << "[IR] " << kIrSlotName[g_ir_slot] << " recorded: ["
                         << lt_ccs.pos.x << ", " << lt_ccs.pos.y << ", " << lt_ccs.pos.z << "]" << endl;
                }
            }
        }
        else if (key=='g'||key=='G') {
            lock_guard<mutex> lk(g_pose_mtx);
            const auto& pp = (g_current_arm=="upper")?g_upper_pose:g_lower_pose;
            cout << "\n=== " << g_current_arm << " FLANGE ===" << endl;
            if (pp.valid) {
                cout << fixed << setprecision(4);
                cout << "XYZ (m):       ["<<pp.x<<", "<<pp.y<<", "<<pp.z<<"]"<<endl;
                cout << "Quat (wxyz):   ["<<pp.qw<<", "<<pp.qx<<", "<<pp.qy<<", "<<pp.qz<<"]"<<endl;
                cout << fixed << setprecision(2);
                cout << "Euler ZXZ'' (deg): ["<<pp.alpha<<", "<<pp.beta<<", "<<pp.gamma<<"]"<<endl;
                const auto& xf = (g_current_arm=="upper") ? g_xf_upper : g_xf_lower;
                Pose fl_ccs = armToolToCamPose(Pose{{pp.x,pp.y,pp.z},{pp.qx,pp.qy,pp.qz,pp.qw}},
                                               {0,0,0}, {0,0,0}, xf.ccs_t, xf.ccs_r);
                cout << "Flange CCS (m):  ["<<fl_ccs.pos.x<<", "<<fl_ccs.pos.y<<", "<<fl_ccs.pos.z<<"]"<<endl;
                Pose lt_ccs = armToolToCamPose(Pose{{pp.x,pp.y,pp.z},{pp.qx,pp.qy,pp.qz,pp.qw}},
                                               xf.loc_t, xf.loc_r, xf.ccs_t, xf.ccs_r);
                cout << fixed << setprecision(4);
                cout << "Locating tool CCS (m): ["<<lt_ccs.pos.x<<", "<<lt_ccs.pos.y<<", "<<lt_ccs.pos.z<<"]\n"<<endl;
            } else { cout << "No pose data\n" << endl; }
        }
    }
}

// 由 participant_id 经 day_participant_map.json 映射到 day_id (与 capture_with_M5Stack 一致)
string findDayForParticipant(const string& json_path, const string& participant) {
    try {
        YAML::Node root = YAML::LoadFile(json_path);
        for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
            string day = it->first.as<string>();
            for (size_t i = 0; i < it->second.size(); ++i)
                if (it->second[i].as<string>() == participant) return day;
        }
    } catch (const std::exception& e) {
        cerr << "[Pose] Failed to parse " << json_path << ": " << e.what() << endl;
    }
    return "";
}

int main() {
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
#endif
    namespace fs = std::filesystem;
    auto pp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg"/"piper.yaml").string();
    Cfg cfg(pp);
    g_ubuntu_ip = cfg["network"]["ubuntu_ip"].as<string>();
    g_port      = cfg["network"]["ctrl_port"].as<int>();   // piper_windows_ctrl_server.py (49301)

    string cfg_dir = (fs::path(pp).parent_path()).string();
    auto readPt3 = [](const CfgNode& n) -> Pt3 {
        return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
    };

    // locating tool 偏移: piper.yaml (法兰盘坐标系)
    for (auto& an : {"upper", "lower"}) {
        try {
            auto& lt = cfg["arms"][an]["locating_tool"];
            auto& xf = (an == string("upper")) ? g_xf_upper : g_xf_lower;
            xf.loc_t = readPt3(lt["translation"]);
            xf.loc_r = readPt3(lt["rotation_zxz"]);
        } catch (const exception& e) {
            cerr << "[Pose] WARN: cannot load " << an << " locating_tool: " << e.what() << endl;
        }
    }

    // arm pose (arm_in_ccs): 与 capture_with_M5Stack 读取方式一致 —
    // participant_id (capture.yaml) -> day_id (day_participant_map.json) -> cfg/arm_pose/{day_id}.yaml
    Cfg cfg_capture(cfg_dir + "/capture.yaml");
    string participant_id = cfg_capture["capture"]["participant_id"].as<string>();
    string day_id = findDayForParticipant(cfg_dir + "/day_participant_map.json", participant_id);
    string arm_pose_yml = cfg_dir + "/arm_pose/" + day_id + ".yaml";
    if (!day_id.empty() && fs::exists(arm_pose_yml)) {
        Cfg cfg_pose(arm_pose_yml);
        for (auto& an : {"upper", "lower"}) {
            try {
                auto& cc = cfg_pose["arms"][an]["arm_in_ccs"];
                auto& xf = (an == string("upper")) ? g_xf_upper : g_xf_lower;
                xf.ccs_t = readPt3(cc["translation"]);
                xf.ccs_r = readPt3(cc["rotation_zxz"]);
            } catch (...) {
                cerr << "[Pose] WARN: cannot load " << an << " transform from " << arm_pose_yml << endl;
            }
        }
        cout << "[Pose] Arm transforms loaded from " << arm_pose_yml << " (day " << day_id << ")" << endl;
    } else {
        cerr << "[Pose] WARN: no arm_pose yaml for participant " << participant_id
             << " — check cfg/day_participant_map.json. Transforms NOT loaded." << endl;
    }

    // IR 记录输出路径: cfg/IR/{day_id}.txt (day_id 来自 calib_arm.yaml)
    try {
        Cfg arm_cfg(cfg_dir + "/calib_arm.yaml");
        string day_id = arm_cfg["record"]["day_id"].as<string>();
        g_ir_path = cfg_dir + "/IR/" + day_id + ".txt";
        g_ir_path_disp = "cfg/IR/" + day_id + ".txt";   // UI 从 cfg 开始显示
    } catch (const exception& e) {
        cerr << "[IR] WARN: cannot read calib_arm.yaml: " << e.what()
             << " — IR recording disabled." << endl;
    }

    cout << "=== Piper Arm Pose ===" << endl;
    cout << "Server: " << g_ubuntu_ip << ":" << g_port << endl;
    cout << "[t] switch arm  [g] print  [r] IR record  [q] quit\n" << endl;

    thread t(tcpWorker);
    drawUI();
    g_running = false;
    if (t.joinable()) t.join();
    cv::destroyAllWindows();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
