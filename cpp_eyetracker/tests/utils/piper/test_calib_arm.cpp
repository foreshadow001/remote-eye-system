// ================== 网络头文件 (必须最前) ==================
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
#include <cmath>

#include "cfg/config.hpp"

using namespace std;

// ================== 数据结构 ==================

struct FlangePose {
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1.0;
    double alpha = 0, beta = 0, gamma = 0;  // Z-X-Z' Euler (degrees)
    bool valid = false;
};

// ================== 全局状态 ==================
atomic<bool> g_running{true};
mutex g_pose_mtx;
FlangePose g_upper_pose;
FlangePose g_lower_pose;
string g_current_arm = "upper";
string g_status = "Connecting...";
string g_ubuntu_ip;
int g_port = 0;

// ================== TCP 通信 ==================

bool parsePoseResponse(const string& resp, string& arm, FlangePose& pose) {
    // Format: POSE:<arm>:<x>,<y>,<z>,<qx>,<qy>,<qz>,<qw>,<alpha>,<beta>,<gamma>
    if (resp.rfind("POSE:", 0) != 0) return false;

    string data = resp.substr(5);  // after "POSE:"
    size_t p1 = data.find(':');
    if (p1 == string::npos) return false;

    arm = data.substr(0, p1);
    string vals = data.substr(p1 + 1);

    vector<double> nums;
    stringstream ss(vals);
    string token;
    while (getline(ss, token, ',')) {
        try { nums.push_back(stod(token)); } catch (...) { return false; }
    }
    if (nums.size() != 10) return false;

    pose.x = nums[0]; pose.y = nums[1]; pose.z = nums[2];
    pose.qx = nums[3]; pose.qy = nums[4]; pose.qz = nums[5]; pose.qw = nums[6];
    pose.alpha = nums[7]; pose.beta = nums[8]; pose.gamma = nums[9];
    pose.valid = true;
    return true;
}

void tcpClientWorker() {
    while (g_running) {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) {
            g_status = "Socket creation failed";
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }

        sockaddr_in server{};
        server.sin_family = AF_INET;
        server.sin_port = htons(g_port);
        inet_pton(AF_INET, g_ubuntu_ip.c_str(), &server.sin_addr);

        g_status = "Connecting...";
        if (connect(sock, (sockaddr*)&server, sizeof(server)) != 0) {
            g_status = "Connection failed, retrying...";
            closesocket(sock);
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }

        g_status = "Connected";
        char buf[1024];
        string acc;

        while (g_running) {
            // Send GET_POSE for current arm
            string cmd = "GET_POSE:" + g_current_arm + "\n";
            if (send(sock, cmd.c_str(), (int)cmd.length(), 0) <= 0) break;

            // Read response (block with 3s timeout)
#ifdef _WIN32
            DWORD timeout = 3000;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#endif
            int n = recv(sock, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;

            buf[n] = '\0';
            acc += string(buf);

            // Process complete lines
            size_t nl;
            while ((nl = acc.find('\n')) != string::npos) {
                string line = acc.substr(0, nl);
                acc = acc.substr(nl + 1);

                string arm;
                FlangePose pose;
                if (parsePoseResponse(line, arm, pose)) {
                    lock_guard<mutex> lock(g_pose_mtx);
                    if (arm == "upper") g_upper_pose = pose;
                    else if (arm == "lower") g_lower_pose = pose;
                }
            }
            this_thread::sleep_for(chrono::milliseconds(500));
        }

        g_status = "Disconnected, reconnecting...";
        closesocket(sock);
        this_thread::sleep_for(chrono::seconds(1));
    }
}

// ================== UI 渲染 ==================

void drawUI() {
    cv::namedWindow("Piper Arm Pose", cv::WINDOW_NORMAL);
    cv::resizeWindow("Piper Arm Pose", 640, 400);

    while (g_running) {
        cv::Mat canvas = cv::Mat::zeros(400, 640, CV_8UC3);

        int y = 30;
        auto put = [&](const string& text, cv::Scalar color = cv::Scalar(255, 255, 255)) {
            cv::putText(canvas, text, cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX,
                        0.55, color, 1, cv::LINE_AA);
            y += 24;
        };

        // Title
        cv::putText(canvas, "Piper Arm Pose Monitor", cv::Point(160, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        y += 40;

        // Status
        string status = g_status;
        cv::Scalar statusColor = (status == "Connected") ? cv::Scalar(0, 255, 0)
                                 : cv::Scalar(0, 165, 255);
        put("Status: " + status, statusColor);

        // Current arm
        cv::Scalar armColor = (g_current_arm == "upper") ? cv::Scalar(255, 200, 0)
                               : cv::Scalar(200, 100, 255);
        put("Current arm: " + g_current_arm + "  [t: switch  g: query  q: quit]", armColor);
        y += 10;

        // Display current arm's pose
        FlangePose pose;
        string dispArm;
        {
            lock_guard<mutex> lock(g_pose_mtx);
            if (g_current_arm == "upper") { pose = g_upper_pose; dispArm = "UPPER"; }
            else { pose = g_lower_pose; dispArm = "LOWER"; }
        }

        cv::putText(canvas, "--- " + dispArm + " FLANGE POSE ---",
                    cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        y += 28;

        if (pose.valid) {
            char buf[128];
            snprintf(buf, sizeof(buf), "XYZ (m):     [%.4f, %.4f, %.4f]", pose.x, pose.y, pose.z);
            put(string(buf));
            snprintf(buf, sizeof(buf), "Quat (wxyz): [%.4f, %.4f, %.4f, %.4f]",
                     pose.qw, pose.qx, pose.qy, pose.qz);
            put(string(buf));
            snprintf(buf, sizeof(buf), "Euler ZXZ'': [%.2f, %.2f, %.2f] deg",
                     pose.alpha, pose.beta, pose.gamma);
            put(string(buf), cv::Scalar(0, 255, 0));
        } else {
            put("No pose data yet");
        }

        // Controls hint
        y = 370;
        cv::putText(canvas, "[t] Switch arm   [g] Query pose   [q/ESC] Quit",
                    cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 150), 1);

        cv::imshow("Piper Arm Pose", canvas);

        char key = (char)cv::waitKey(50);
        if (key == 'q' || key == 27) {
            g_running = false;
        } else if (key == 't' || key == 'T') {
            g_current_arm = (g_current_arm == "upper") ? "lower" : "upper";
        } else if (key == 'g' || key == 'G') {
            // Force immediate query + print to console
            {
                lock_guard<mutex> lock(g_pose_mtx);
                const FlangePose& p = (g_current_arm == "upper") ? g_upper_pose : g_lower_pose;
                cout << "\n=== " << g_current_arm << " FLANGE POSE ===" << endl;
                if (p.valid) {
                    cout << fixed << setprecision(4);
                    cout << "Position (m):      ["
                         << p.x << ", " << p.y << ", " << p.z << "]" << endl;
                    cout << "Quaternion (wxyz): ["
                         << p.qw << ", " << p.qx << ", " << p.qy << ", " << p.qz << "]" << endl;
                    cout << fixed << setprecision(2);
                    cout << "Euler Z-X-Z' (deg): [" << p.alpha << ", "
                         << p.beta << ", " << p.gamma << "]" << endl;
                } else {
                    cout << "No pose data yet" << endl;
                }
                cout << endl;
            }
        }
    }
}

// ================== main ==================

int main() {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    // piper.yaml sits next to default.yaml in cfg/
    namespace fs = std::filesystem;
    auto piper_path = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                       / "cfg" / "piper.yaml").string();
    Cfg cfg(piper_path);
    try {
        g_ubuntu_ip = cfg["network"]["ubuntu_ip"].as<string>();
        g_port = cfg["network"]["port"].as<int>();
    } catch (...) {
        cerr << "Failed to read piper network config from: " << piper_path << endl;
        return 1;
    }

    cout << "=== Piper Arm Pose Monitor ===" << endl;
    cout << "Ubuntu IP: " << g_ubuntu_ip << ":" << g_port << endl;
    cout << "Controls: [t] Switch arm  [g] Query pose  [q] Quit" << endl;
    cout << "Auto-queries every 500ms for live display" << endl;
    cout << "===============================\n" << endl;

    thread net_thread(tcpClientWorker);
    drawUI();

    g_running = false;
    if (net_thread.joinable()) net_thread.join();

    cv::destroyAllWindows();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
