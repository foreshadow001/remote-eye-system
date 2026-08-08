// ================== test_piper_ctrl.cpp ==================
// Single-arm Piper control via TCP to Ubuntu (piper_windows_ctrl_server.py).
// Reads first line of gaze_target/piper_upper.txt, sends zero-return,
// then MOVE_TO, receives actual flange pose, displays in OpenCV window.
//
// Keys: [SPACE] send MOVE_TO   [R] re-zero   [ESC]/[q] SHUTDOWN + quit
// ====================================================================

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
#include <array>
#include <iomanip>
#include <chrono>
#include <cmath>

#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;

// ------------------------------------------------------------------
// TCP helpers (inlined from get_piper_pose.cpp)
// ------------------------------------------------------------------

bool recvLine(SOCKET sock, string& line, int timeout_ms = 10000) {
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

bool sendLine(SOCKET sock, const string& msg) {
    string data = msg + "\n";
    return send(sock, data.c_str(), (int)data.length(), 0) > 0;
}

// Parse "MOVED:arm:x,y,z,qx,qy,qz,qw,alpha,beta,gamma" or "POSE:arm:..."
// (shared format — POSE and MOVED both carry 10 comma-separated values)
struct ArmPose { double x,y,z, qx,qy,qz,qw, alpha,beta,gamma; bool valid=false; };

bool parsePoseResponse(const string& resp, string& arm, ArmPose& pose) {
    if (resp.rfind("MOVED:", 0) != 0 && resp.rfind("POSE:", 0) != 0) return false;
    size_t colon1 = resp.find(':');
    size_t colon2 = resp.find(':', colon1 + 1);
    if (colon1 == string::npos || colon2 == string::npos) return false;
    arm = resp.substr(colon1 + 1, colon2 - colon1 - 1);
    string vals = resp.substr(colon2 + 1);
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

// ------------------------------------------------------------------
// Config & target
// ------------------------------------------------------------------

int main() {
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    // --- Load config ---
    auto yp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
               / "cfg" / "piper.yaml").string();
    Cfg cfg(yp);
    string ubuntu_ip = cfg["network"]["ubuntu_ip"].as<string>();
    int    ctrl_port = cfg["network"]["ctrl_port"].as<int>();

    cout << "=== Piper Arm Control ===" << endl;
    cout << "Server: " << ubuntu_ip << ":" << ctrl_port << endl;

    // --- Load first target from gaze_target/piper_upper.txt ---
    double cmd_x = 0, cmd_y = 0, cmd_z = 0;
    bool has_target = false;
    {
        ifstream in("gaze_target/piper_upper.txt");
        if (!in) {
            cerr << "ERROR: Cannot open gaze_target/piper_upper.txt" << endl;
            return 1;
        }
        string line;
        if (getline(in, line)) {
            stringstream ss(line);
            string token;
            array<double, 3> pt;
            for (int i = 0; i < 3; ++i) {
                if (!getline(ss, token, ',')) break;
                try { pt[i] = stod(token); } catch (...) { break; }
            }
            cmd_x = pt[0]; cmd_y = pt[1]; cmd_z = pt[2];
            has_target = true;
            cout << "Target: (" << cmd_x << ", " << cmd_y << ", " << cmd_z << ")" << endl;
        }
        if (!has_target) {
            cerr << "ERROR: No target points in piper_upper.txt" << endl;
            return 1;
        }
    }

    // --- Connect TCP ---
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        cerr << "ERROR: socket() failed" << endl;
        return 1;
    }
    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port   = htons(ctrl_port);
    inet_pton(AF_INET, ubuntu_ip.c_str(), &server.sin_addr);

    cout << "Connecting to " << ubuntu_ip << ":" << ctrl_port << "..." << endl;
    if (connect(sock, (sockaddr*)&server, sizeof(server)) != 0) {
        cerr << "ERROR: connect() failed (error " << WSAGetLastError() << ")" << endl;
        closesocket(sock);
        return 1;
    }
    cout << "Connected." << endl;

    // --- State ---
    string status     = "Connected";
    string arm        = "upper";
    ArmPose last_pose;
    bool   zeroed     = false;
    bool   move_sent  = false;
    string last_response;

    // --- Send MOVE_JOINTS (zero-return) ---
    cout << "Sending zero-return..." << endl;
    status = "Zeroing...";
    if (!sendLine(sock, "MOVE_JOINTS:upper:0.0,0.0,0.0,0.0,0.0,0.0")) {
        cerr << "ERROR: send MOVE_JOINTS failed" << endl;
        closesocket(sock);
        return 1;
    }

    string resp;
    string resp_arm;
    if (recvLine(sock, resp, 30000)) {
        last_response = resp;
        if (parsePoseResponse(resp, resp_arm, last_pose)) {
            zeroed = true;
            status = "Zeroed";
            cout << "Zero-return OK: (" << last_pose.x << ", " << last_pose.y << ", " << last_pose.z << ")" << endl;
        } else {
            status = "Zero FAIL: " + resp;
            cerr << status << endl;
        }
    } else {
        status = "Zero timeout";
        cerr << status << endl;
    }

    // ==================================================================
    // OpenCV UI loop
    // ==================================================================
    cv::namedWindow("Piper Arm Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Piper Arm Control", 640, 420);

    bool running = true;
    while (running) {
        cv::Mat canvas = cv::Mat::zeros(420, 640, CV_8UC3);
        int y = 25;
        auto put = [&](const string& s, cv::Scalar c = {255,255,255}) {
            cv::putText(canvas, s, {20, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
            y += 22;
        };

        // Title
        cv::putText(canvas, "Piper Upper Arm Control", {160, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2, cv::LINE_AA);
        y += 35;

        // Status
        cv::Scalar sc = (status == "Zeroed" || status.rfind("MOVED", 0) == 0)
                            ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);
        put("Status: " + status, sc);

        // Target
        char b[128];
        snprintf(b, sizeof(b), "Command (x,y,z): [%.3f, %.3f, %.3f] m", cmd_x, cmd_y, cmd_z);
        put(b, {255, 200, 0});
        y += 8;

        // Actual pose
        if (last_pose.valid) {
            snprintf(b, sizeof(b), "Actual  (x,y,z): [%.4f, %.4f, %.4f] m",
                     last_pose.x, last_pose.y, last_pose.z);
            put(b);
            snprintf(b, sizeof(b), "Quat (x,y,z,w): [%.4f, %.4f, %.4f, %.4f]",
                     last_pose.qx, last_pose.qy, last_pose.qz, last_pose.qw);
            put(b);
            snprintf(b, sizeof(b), "Euler Z-X-Z'' : a=%.2f  b=%.2f  g=%.2f deg",
                     last_pose.alpha, last_pose.beta, last_pose.gamma);
            put(b, {0, 255, 0});

            // Distance from command
            double dx = last_pose.x - cmd_x;
            double dy = last_pose.y - cmd_y;
            double dz = last_pose.z - cmd_z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            snprintf(b, sizeof(b), "Dist to target: %.4f m  %s",
                     dist, dist < 0.02 ? "OK" : "OFF");
            put(b, dist < 0.02 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255));
        } else {
            put("No pose data yet");
        }
        y += 5;

        // Last raw response
        if (!last_response.empty()) {
            put("Last resp: " + last_response.substr(0, min((size_t)70, last_response.size())),
                {150, 150, 150});
        }

        // Key hints
        y = 395;
        cv::putText(canvas, "[SPACE] Send MOVE_TO   [R] Re-zero   [ESC]/[q] Quit",
                    {20, y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {140, 140, 140}, 1);

        cv::imshow("Piper Arm Control", canvas);
        char key = (char)cv::waitKey(30);

        if (key == 'q' || key == 27) {  // q or ESC
            cout << "Sending SHUTDOWN..." << endl;
            sendLine(sock, "SHUTDOWN");
            // read SHUTDOWN_ACK
            string ack;
            recvLine(sock, ack, 2000);
            cout << "Server response: " << ack << endl;
            running = false;
        }
        else if (key == ' ' && zeroed && !move_sent) {  // SPACE: send MOVE_TO
            char cmd[128];
            snprintf(cmd, sizeof(cmd), "MOVE_TO:upper:%.6f,%.6f,%.6f", cmd_x, cmd_y, cmd_z);
            cout << "Sending: " << cmd << endl;
            status = "Moving...";
            if (!sendLine(sock, cmd)) {
                status = "Send FAIL";
                cerr << status << endl;
            } else {
                string mr;
                string mr_arm;
                if (recvLine(sock, mr, 60000)) {   // movement may take up to 60s
                    last_response = mr;
                    ArmPose mp;
                    if (parsePoseResponse(mr, mr_arm, mp)) {
                        last_pose = mp;
                        status = "OK — MOVED";
                        move_sent = true;
                        cout << "MOVED: (" << mp.x << ", " << mp.y << ", " << mp.z << ")" << endl;
                    } else {
                        status = "Bad response: " + mr;
                        cerr << status << endl;
                    }
                } else {
                    status = "Timeout";
                    cerr << status << endl;
                }
            }
        }
        else if (key == 'r' || key == 'R') {  // Re-zero
            cout << "Re-sending zero-return..." << endl;
            status = "Zeroing...";
            sendLine(sock, "MOVE_JOINTS:upper:0.0,0.0,0.0,0.0,0.0,0.0");
            string zr;
            string zr_arm;
            if (recvLine(sock, zr, 30000)) {
                last_response = zr;
                if (parsePoseResponse(zr, zr_arm, last_pose)) {
                    zeroed = true;
                    move_sent = false;
                    status = "Zeroed";
                    cout << "Re-zero OK" << endl;
                } else {
                    status = "Re-zero FAIL: " + zr;
                }
            } else {
                status = "Re-zero timeout";
            }
        }
    }

    closesocket(sock);
    cv::destroyAllWindows();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
