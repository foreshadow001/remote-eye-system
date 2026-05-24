// ================== get_piper_pose ==================
// 实时获取 Piper 机械臂 flange 位姿 (XYZ + 四元数 + Z-X-Z'' 欧拉角)
// 用法: t 切换 upper/lower, g 打印位姿, q 退出
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
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <iomanip>

#include "cfg/config.hpp"

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
            cout << "[TCP] send: '" << cmd.substr(0, cmd.length()-1) << "'" << endl;
            if (send(sock, cmd.c_str(), (int)cmd.length(), 0) <= 0) {
                cout << "[TCP] send FAILED" << endl;
                break;
            }

            string line;
            if (!recvLine(sock, line, 3000)) break;

            string arm;
            FlangePose pose;
            cout << "[TCP] raw recv: '" << line << "'" << endl;
            if (parsePoseResponse(line, arm, pose)) {
                lock_guard<mutex> lock(g_pose_mtx);
                if (arm == "upper") g_upper_pose = pose;
                else if (arm == "lower") g_lower_pose = pose;
                cout << "[TCP] updated " << arm << " pose: "
                     << pose.x << "," << pose.y << "," << pose.z
                     << " | euler " << pose.alpha << "," << pose.beta << "," << pose.gamma << endl;
            } else {
                cout << "[TCP] parse FAILED for line: '" << line << "'" << endl;
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
    cv::resizeWindow("Piper Arm Pose", 640, 400);

    while (g_running) {
        cv::Mat canvas = cv::Mat::zeros(400, 640, CV_8UC3);
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

        y = 370;
        cv::putText(canvas, "[t] Switch arm   [g] Print pose   [q/ESC] Quit",
                    {20,y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {150,150,150}, 1);

        cv::imshow("Piper Arm Pose", canvas);
        char key = (char)cv::waitKey(50);
        if (key=='q'||key==27) g_running = false;
        else if (key=='t'||key=='T') g_current_arm = (g_current_arm=="upper")?"lower":"upper";
        else if (key=='g'||key=='G') {
            lock_guard<mutex> lk(g_pose_mtx);
            const auto& pp = (g_current_arm=="upper")?g_upper_pose:g_lower_pose;
            cout << "\n=== " << g_current_arm << " FLANGE ===" << endl;
            if (pp.valid) {
                cout << fixed << setprecision(4);
                cout << "XYZ (m):       ["<<pp.x<<", "<<pp.y<<", "<<pp.z<<"]"<<endl;
                cout << "Quat (wxyz):   ["<<pp.qw<<", "<<pp.qx<<", "<<pp.qy<<", "<<pp.qz<<"]"<<endl;
                cout << fixed << setprecision(2);
                cout << "Euler ZXZ'' (deg): ["<<pp.alpha<<", "<<pp.beta<<", "<<pp.gamma<<"]\n"<<endl;
            } else { cout << "No pose data\n" << endl; }
        }
    }
}

int main() {
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
#endif
    namespace fs = std::filesystem;
    auto pp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg"/"piper.yaml").string();
    Cfg cfg(pp);
    g_ubuntu_ip = cfg["network"]["ubuntu_ip"].as<string>();
    g_port      = cfg["network"]["port"].as<int>();

    cout << "=== Piper Arm Pose ===" << endl;
    cout << "Server: " << g_ubuntu_ip << ":" << g_port << endl;
    cout << "[t] switch arm  [g] print  [q] quit\n" << endl;

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
