// ================== get_piper_tool_pose ==================
// 获取 Piper 机械臂末端工具位置 (相机坐标系 CCS)
// 配置: calib_arm.yaml (network + record.day_id) + cfg/arm_pose/{day_id}.yaml (手眼标定结果)
// 按键: g 获取当前臂末端工具位姿, t 切换 upper/lower, q 退出
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
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <thread>

#include "cfg/config.hpp"
#include "piper/piper.hpp"

using namespace std;
using namespace gazeestimation;
namespace fs = std::filesystem;

// ================== 全局 ==================
SOCKET g_sock = INVALID_SOCKET;
string g_current_arm = "upper";
string g_status = "Disconnected";
string g_ubuntu_ip;
int g_port = 49301;
string g_arm_pose_path;                 // cfg/arm_pose/{day_id}.yaml
PiperToCam* g_converter = nullptr;

struct ToolResult {
    bool valid = false;
    Pose flange;                        // 法兰位姿 (臂基坐标系)
    PoseZxz tool;                       // 末端工具位姿 (CCS)
    double fa = 0, fb = 0, fg = 0;      // 法兰 Z-X-Z'' 角 (deg, 直接来自 POSE 响应)
};
ToolResult g_upper_res, g_lower_res;

// ================== TCP ==================
bool recvLine(SOCKET sock, string& line, int timeout_ms = 3000) {
#ifdef _WIN32
    DWORD to = timeout_ms;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
#endif
    char buf[256]; string acc;
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeout_ms);
    while (chrono::steady_clock::now() < deadline) {
        int n = recv(sock, buf, sizeof(buf) - 1, 0);
        if (n <= 0) return false;
        buf[n] = '\0'; acc += buf;
        size_t nl = acc.find('\n');
        if (nl != string::npos) {
            line = acc.substr(0, nl);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return true;
        }
    }
    return false;
}

bool sendLine(SOCKET sock, const string& msg) {
    string data = msg + "\n";
    return send(sock, data.c_str(), (int)data.length(), 0) > 0;
}

// 解析 "POSE:arm:x,y,z,qx,qy,qz,qw,alpha,beta,gamma" (10 个数值)
bool parsePoseResponse(const string& resp, string& arm, Pose& flange,
                       double& a, double& b, double& g) {
    if (resp.rfind("POSE:", 0) != 0) return false;
    string data = resp.substr(5);
    size_t p1 = data.find(':');
    if (p1 == string::npos) return false;
    arm = data.substr(0, p1);
    vector<double> nums;
    stringstream ss(data.substr(p1 + 1)); string token;
    while (getline(ss, token, ',')) {
        try { nums.push_back(stod(token)); } catch (...) { return false; }
    }
    if (nums.size() != 10) return false;
    flange.pos = {nums[0], nums[1], nums[2]};
    flange.quat = {nums[3], nums[4], nums[5], nums[6]};
    a = nums[7]; b = nums[8]; g = nums[9];
    return true;
}

// 连接 + READY/ACK 握手 (带重试, 参照 test_record_arm_data)
bool connectArm() {
    if (g_sock != INVALID_SOCKET) { closesocket(g_sock); g_sock = INVALID_SOCKET; }
    for (int retry = 0; retry < 3; ++retry) {
        g_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (g_sock == INVALID_SOCKET) return false;
        sockaddr_in server{};
        server.sin_family = AF_INET;
        server.sin_port = htons(g_port);
        inet_pton(AF_INET, g_ubuntu_ip.c_str(), &server.sin_addr);
        if (connect(g_sock, (sockaddr*)&server, sizeof(server)) != 0) {
            closesocket(g_sock); g_sock = INVALID_SOCKET;
            this_thread::sleep_for(chrono::milliseconds(500));
            continue;
        }
        sendLine(g_sock, "READY");
        string hl;
        if (recvLine(g_sock, hl, 3000) && hl == "ACK") {
            g_status = "Connected";
            return true;
        }
        closesocket(g_sock); g_sock = INVALID_SOCKET;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    g_status = "Connect failed";
    return false;
}

// ================== 查询 ==================
void queryTool() {
    if (g_sock == INVALID_SOCKET && !connectArm()) return;
    if (!sendLine(g_sock, "GET_POSE:" + g_current_arm)) {
        g_status = "Send failed - reconnecting...";
        if (!connectArm()) return;
        if (!sendLine(g_sock, "GET_POSE:" + g_current_arm)) { g_status = "Send failed"; return; }
    }
    string resp;
    if (!recvLine(g_sock, resp, 3000)) { g_status = "Query timeout"; return; }

    string arm; Pose flange; double a, b, g;
    if (!parsePoseResponse(resp, arm, flange, a, b, g)) {
        g_status = "Bad response: " + resp;
        return;
    }

    ToolResult res;
    res.valid = true;
    res.flange = flange;
    res.fa = a; res.fb = b; res.fg = g;
    res.tool = g_converter->convertZxz(g_current_arm, flange);   // 末端工具在 CCS 的位姿
    if (g_current_arm == "upper") g_upper_res = res;
    else g_lower_res = res;

    cout << "\n=== " << g_current_arm << " TOOL POSE (CCS) ===" << endl;
    cout << fixed << setprecision(4);
    cout << "Flange XYZ (m):  [" << flange.pos.x << ", " << flange.pos.y << ", " << flange.pos.z << "]" << endl;
    cout << "Tool   XYZ (m):  [" << res.tool.pos.x << ", " << res.tool.pos.y << ", " << res.tool.pos.z << "]" << endl;
    cout << fixed << setprecision(2);
    cout << "Tool   ZXZ (deg):[" << res.tool.alpha << ", " << res.tool.beta << ", " << res.tool.gamma << "]" << endl;
    cout << "------------------------------------------\n" << endl;
    g_status = "OK (tool queried)";
}

// ================== UI ==================
void drawUI() {
    cv::namedWindow("Piper Tool Pose", cv::WINDOW_NORMAL);
    cv::resizeWindow("Piper Tool Pose", 700, 430);
    while (true) {
        cv::Mat canvas = cv::Mat::zeros(430, 700, CV_8UC3);
        int y = 30;
        auto put = [&](const string& s, cv::Scalar c = {255, 255, 255}, double sc = 0.55) {
            cv::putText(canvas, s, {20, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, 1, cv::LINE_AA);
            y += 24;
        };

        cv::putText(canvas, "Piper Arm Tool Pose (CCS)", {170, y}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2, cv::LINE_AA);
        y += 40;

        put("Status: " + g_status, g_status == "Connected" ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255));
        cv::Scalar ac = (g_current_arm == "upper") ? cv::Scalar(0, 215, 255) : cv::Scalar(200, 80, 255);
        put("Arm: " + g_current_arm, ac);
        put("Pose: " + g_arm_pose_path, {150, 150, 150}, 0.4);
        y += 10;

        const ToolResult& p = (g_current_arm == "upper") ? g_upper_res : g_lower_res;
        string dn = (g_current_arm == "upper") ? "UPPER" : "LOWER";
        cv::putText(canvas, "--- " + dn + " ---", {20, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {200, 200, 200}, 1, cv::LINE_AA);
        y += 28;

        if (p.valid) {
            char b[128];
            snprintf(b, sizeof(b), "Flange XYZ (m):  [%.4f, %.4f, %.4f]", p.flange.pos.x, p.flange.pos.y, p.flange.pos.z); put(b);
            snprintf(b, sizeof(b), "Tool   XYZ (m):  [%.4f, %.4f, %.4f]", p.tool.pos.x, p.tool.pos.y, p.tool.pos.z);
            put(b, {0, 255, 0});
            snprintf(b, sizeof(b), "Tool   ZXZ(deg): [%.2f, %.2f, %.2f]", p.tool.alpha, p.tool.beta, p.tool.gamma);
            put(b, {0, 255, 0});
        } else {
            put("No pose data - press [g] to query");
        }

        y = 400;
        cv::putText(canvas, "[g] Query tool pose   [t] Switch arm   [q/ESC] Quit",
                    {20, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {150, 150, 150}, 1);

        cv::imshow("Piper Tool Pose", canvas);
        char key = (char)cv::waitKey(30);
        if (key == 'q' || key == 27) break;
        else if (key == 't' || key == 'T') {
            g_current_arm = (g_current_arm == "upper") ? "lower" : "upper";
            cout << "[Arm] Switched to " << g_current_arm << endl;
        }
        else if (key == 'g' || key == 'G') { queryTool(); }
    }
}

int main() {
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg").string();
    Cfg arm_cfg(cfg_dir + "/calib_arm.yaml");
    g_ubuntu_ip = arm_cfg["network"]["ubuntu_ip"].as<string>();
    g_port = arm_cfg["network"]["ctrl_port"].as<int>();
    string day_id = arm_cfg["record"]["day_id"].as<string>();
    g_arm_pose_path = cfg_dir + "/arm_pose/" + day_id + ".yaml";

    cout << "=== Piper Arm Tool Pose (CCS) ===" << endl;
    cout << "Server : " << g_ubuntu_ip << ":" << g_port << endl;
    cout << "Pose   : " << g_arm_pose_path << endl;

    if (!fs::exists(g_arm_pose_path)) {
        cerr << "[Fatal] Not found: " << g_arm_pose_path
             << " — run test_piper_hand_eye_calib first." << endl;
        return 1;
    }
    try {
        g_converter = new PiperToCam(g_arm_pose_path);
    } catch (const std::exception& e) {
        cerr << "[Fatal] Failed to load arm pose: " << e.what() << endl;
        return 1;
    }
    cout << "Keys   : [g] query tool pose  [t] switch arm  [q] quit\n" << endl;

    drawUI();

    delete g_converter;
    if (g_sock != INVALID_SOCKET) closesocket(g_sock);
    cv::destroyAllWindows();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
