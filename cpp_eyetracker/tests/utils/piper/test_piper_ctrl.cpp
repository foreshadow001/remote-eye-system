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
// (shared format - POSE and MOVED both carry 10 comma-separated values)
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
    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                    / "cfg").string();
    Cfg cfg_piper(cfg_dir + "/piper.yaml");
    string ubuntu_ip = cfg_piper["network"]["ubuntu_ip"].as<string>();
    int    ctrl_port = cfg_piper["network"]["ctrl_port"].as<int>();

    // Read participant ID from capture.yaml to locate gaze_target directory
    Cfg cfg_capture(cfg_dir + "/capture.yaml");
    string participant_id = cfg_capture["capture"]["participant_id"].as<string>();
    string gaze_dir = "cfg/gaze_target/" + participant_id;

    cout << "=== Piper Arm Control ===" << endl;
    cout << "Server: " << ubuntu_ip << ":" << ctrl_port << endl;
    cout << "Participant: " << participant_id << endl;

    // --- Helper: load all targets from a file ---
    auto loadTargets = [](const string& path) -> vector<array<double,3>> {
        vector<array<double,3>> out;
        ifstream in(path);
        if (!in) {
            cerr << "ERROR: Cannot open " << path << endl;
            return out;
        }
        string line;
        while (getline(in, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            string token;
            array<double,3> pt{};
            for (int i = 0; i < 3; ++i) {
                if (!getline(ss, token, ',')) break;
                try { pt[i] = stod(token); } catch (...) { break; }
            }
            out.push_back(pt);
        }
        return out;
    };

    // --- Load targets for both arms ---
    auto targets_upper = loadTargets(gaze_dir + "/piper_upper.txt");
    auto targets_lower = loadTargets(gaze_dir + "/piper_lower.txt");
    if (targets_upper.empty() && targets_lower.empty()) {
        cerr << "ERROR: No target points loaded." << endl;
        return 1;
    }
    cout << "Loaded: upper=" << targets_upper.size()
         << " pts, lower=" << targets_lower.size() << " pts" << endl;

    // --- Restore progress from sentry ---
    int upper_idx = 0, lower_idx = 0;
    string sentry_path = gaze_dir + "/sentry.txt";
    {
        ifstream sf(sentry_path);
        if (sf) {
            string line;
            while (getline(sf, line)) {
                if (line.rfind("upper:", 0) == 0) upper_idx = stoi(line.substr(6));
                if (line.rfind("lower:", 0) == 0) lower_idx = stoi(line.substr(6));
            }
            cout << "Restored progress: upper=" << upper_idx
                 << " lower=" << lower_idx << endl;
        }
    }
    auto updateSentry = [&]() {
        ofstream sf(sentry_path);
        sf << "upper:" << upper_idx << "\nlower:" << lower_idx << "\n";
    };

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
    string status        = "Connected";
    string arm           = "upper";
    ArmPose last_pose;
    string last_response;
    bool   upper_done    = (upper_idx >= (int)targets_upper.size());
    bool   lower_done    = (lower_idx >= (int)targets_lower.size());
    bool   all_done      = upper_done && lower_done;

    // Helper: current arm index (write-through to sentry)
    auto armIdx = [&]() -> int& { return (arm=="upper") ? upper_idx : lower_idx; };
    auto armDone = [&]() -> bool& { return (arm=="upper") ? upper_done : lower_done; };
    auto armTotal = [&]() -> int { return (arm=="upper") ? (int)targets_upper.size()
                                                         : (int)targets_lower.size(); };

    // Helper: send MOVE_JOINTS for an arm, wait for response, update sentry
    auto zeroArm = [&](const string& arm_name) -> bool {
        cout << "Zeroing " << arm_name << "..." << endl;
        status = "Zeroing " + arm_name + "...";
        string cmd = "MOVE_JOINTS:" + arm_name + ":0.0,0.0,0.0,0.0,0.0,0.0";
        if (!sendLine(sock, cmd)) {
            cerr << "ERROR: send MOVE_JOINTS failed for " << arm_name << endl;
            return false;
        }
        string resp, resp_arm;
        ArmPose pose;
        if (recvLine(sock, resp, 30000)) {
            last_response = resp;
            if (parsePoseResponse(resp, resp_arm, pose)) {
                last_pose = pose;
                cout << arm_name << " zeroed: (" << pose.x << ", " << pose.y << ", " << pose.z << ")" << endl;
                updateSentry();
                return true;
            } else {
                cerr << arm_name << " zero FAIL: " << resp << endl;
            }
        } else {
            cerr << arm_name << " zero timeout" << endl;
        }
        return false;
    };

    // --- Zero both arms (non-fatal - UI opens regardless) ---
    if (!zeroArm("upper")) {
        status = "Upper zero FAIL - press R to retry";
        cerr << status << endl;
    }
    if (!zeroArm("lower")) {
        status = "Lower zero FAIL - press R to retry";
        cerr << status << endl;
    }
    if (status.find("Zeroed") == string::npos && status.find("FAIL") != string::npos) {
        // neither arm zeroed successfully, keep the last failure status
    } else {
        status = "Zeroed - upper";
    }

    // Helper: get current target
    auto getCurrentTarget = [&]() -> const array<double,3>* {
        int idx = armIdx();
        auto& tgt = (arm == "upper") ? targets_upper : targets_lower;
        if (idx >= 0 && idx < (int)tgt.size()) return &tgt[idx];
        return nullptr;
    };

    // ==================================================================
    // OpenCV UI loop
    // ==================================================================
    cv::namedWindow("Piper Arm Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Piper Arm Control", 640, 460);

    bool running = true;
    while (running) {
        cv::Mat canvas = cv::Mat::zeros(460, 640, CV_8UC3);
        int y = 25;
        auto put = [&](const string& s, cv::Scalar c = {255,255,255}) {
            cv::putText(canvas, s, {20, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv::LINE_AA);
            y += 22;
        };

        // Title
        string title = "Piper Arm Control - " + arm + (arm=="upper"?" (UPPER)":" (LOWER)");
        cv::putText(canvas, title, {140, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2, cv::LINE_AA);
        y += 32;

        // Status
        cv::Scalar sc = all_done ? cv::Scalar(0,255,0) :
                        ((arm=="upper"&&upper_done)||(arm=="lower"&&lower_done))
                            ? cv::Scalar(0,255,200) : cv::Scalar(0,200,255);
        put("Status: " + status, sc);

        // Current arm + index
        int tgt_count = (arm=="upper") ? (int)targets_upper.size() : (int)targets_lower.size();
        char b[128];
        snprintf(b,sizeof(b),"Arm: %s  |  Upper done: %s  |  Lower done: %s",
                 arm.c_str(), upper_done?"YES":"no", lower_done?"YES":"no");
        put(b, {200,200,200});

        // Current target
        auto* tgt = getCurrentTarget();
        if (tgt) {
            snprintf(b,sizeof(b),"Target #%d/%d: (%.3f, %.3f, %.3f) m",
                     armIdx()+1, tgt_count, (*tgt)[0], (*tgt)[1], (*tgt)[2]);
            put(b, {255,200,0});
        } else if ((arm=="upper"&&upper_done)||(arm=="lower"&&lower_done)) {
            put("Target: ALL DONE for " + arm, {0,255,0});
        }
        y += 6;

        // Actual pose
        if (last_pose.valid) {
            snprintf(b,sizeof(b),"Actual (x,y,z): [%.4f, %.4f, %.4f] m",
                     last_pose.x, last_pose.y, last_pose.z); put(b);
            snprintf(b,sizeof(b),"Quat (x,y,z,w): [%.4f, %.4f, %.4f, %.4f]",
                     last_pose.qx,last_pose.qy,last_pose.qz,last_pose.qw); put(b);
            snprintf(b,sizeof(b),"Euler Z-X-Z'': a=%.2f  b=%.2f  g=%.2f deg",
                     last_pose.alpha,last_pose.beta,last_pose.gamma);
            put(b,{0,255,0});

            if (tgt) {
                double dx=last_pose.x-(*tgt)[0], dy=last_pose.y-(*tgt)[1], dz=last_pose.z-(*tgt)[2];
                double dist=sqrt(dx*dx+dy*dy+dz*dz);
                snprintf(b,sizeof(b),"Dist to target: %.4f m  %s",dist,dist<0.02?"OK":"OFF");
                put(b,dist<0.02?cv::Scalar(0,255,0):cv::Scalar(0,165,255));
            }
        } else {
            put("No pose data yet");
        }
        y += 4;

        // All done banner
        if (all_done) {
            cv::putText(canvas, "*** ALL DONE - Press ESC/q to exit ***",
                        {60, y+10}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);
            y += 40;
        }

        // Last raw response (truncated)
        if (!last_response.empty())
            put("Last TCP resp: "+last_response.substr(0,min((size_t)60,last_response.size())),{150,150,150});

        // Key hints
        y = 442;
        string hints = all_done ? "[ESC/q] Quit"
                      : "[SPACE] Next  [T] Switch arm  [R] Re-zero  [ESC/q] Quit";
        cv::putText(canvas, hints, {20,y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {140,140,140}, 1);

        cv::imshow("Piper Arm Control", canvas);
        char key = (char)cv::waitKey(30);

        if (key == 'q' || key == 27) {
            // Park both arms before exit (always, regardless of all_done)
            cout << "\nExiting - zeroing both arms..." << endl;
            status = "Exit: zeroing upper...";
            bool u_ok = zeroArm("upper");
            cout << "  upper zero: " << (u_ok ? "OK" : "FAIL") << endl;
            status = "Exit: zeroing lower...";
            bool l_ok = zeroArm("lower");
            cout << "  lower zero: " << (l_ok ? "OK" : "FAIL") << endl;
            // Send SHUTDOWN
            cout << "Sending SHUTDOWN..." << endl;
            status = "Exiting...";
            sendLine(sock, "SHUTDOWN");
            string ack; recvLine(sock, ack, 2000);
            cout << "Server: " << ack << endl;
            running = false;
        }
        else if (key == 't' || key == 'T') {
            // Switch arm - zero current arm first (park it), then switch
            if (all_done) continue;
            string new_arm = (arm == "upper") ? "lower" : "upper";
            bool& nd = (new_arm == "upper") ? upper_done : lower_done;
            if (nd) {
                status = new_arm + " already done - skipped";
                cout << status << endl;
                continue;
            }
            // Park current arm (the one being left) before switching
            cout << "Parking " << arm << " before switching to " << new_arm << "..." << endl;
            status = "Parking " + arm + "...";
            zeroArm(arm);
            // Switch (allow forced switch even if zero times out)
            arm = new_arm;
            status = "Switched to " + arm;
            cout << "Switched to " << arm << " (index=" << armIdx() << ")" << endl;
        }
        else if (key == ' ' && !all_done) {
            // Check if current arm is already done
            if (armDone()) {
                status = arm + " already done - press T to switch";
                continue;
            }
            auto* ct = getCurrentTarget();
            if (!ct) {
                // Mark this arm as done
                armDone() = true;
                if (arm == "upper") upper_done = true;
                else                lower_done = true;
                updateSentry();
                status = arm + " DONE!";
                cout << "\n=== " << arm << " COMPLETED ===" << endl;
                if (upper_done && lower_done) {
                    all_done = true;
                    status = "ALL DONE!";
                    cout << "=== ALL TARGETS COMPLETED ===" << endl;
                }
                continue;
            }
            // Send MOVE_TO
            char cmd[128];
            snprintf(cmd,sizeof(cmd),"MOVE_TO:%s:%.6f,%.6f,%.6f",
                     arm.c_str(), (*ct)[0], (*ct)[1], (*ct)[2]);
            int cur_idx = armIdx();
            cout << "Sending [" << arm << " #" << (cur_idx+1) << "/" << armTotal()
                 << "]: " << cmd << endl;
            status = "Moving " + arm + " #" + to_string(cur_idx+1) + "...";
            if (!sendLine(sock, cmd)) {
                status = "Send FAIL";
            } else {
                string mr, mr_arm;
                if (recvLine(sock, mr, 60000)) {
                    last_response = mr;
                    ArmPose mp;
                    if (parsePoseResponse(mr, mr_arm, mp)) {
                        last_pose = mp;
                        status = "OK - " + arm + " #" + to_string(cur_idx+1);
                        armIdx()++;
                        updateSentry();
                        cout << arm << " #" << cur_idx+1 << " MOVED: ("
                             << mp.x << ", " << mp.y << ", " << mp.z << ")"
                             << "  progress saved" << endl;
                    } else if (mr.rfind("ERROR:", 0) == 0) {
                        // No solution - skip this target, save progress
                        armIdx()++;
                        updateSentry();
                        status = "SKIPPED - " + arm + " #" + to_string(cur_idx+1)
                               + " (no solution)";
                        cout << "  SKIPPED (no solution). Progress saved." << endl;
                    } else {
                        status = "Bad resp: " + mr.substr(0,40);
                    }
                } else {
                    status = "Timeout";
                }
            }
        }
        else if (key == 'r' || key == 'R') {
            // Re-zero current arm - reset its progress
            zeroArm(arm);
            armIdx() = 0;
            armDone() = false;
            all_done = false;
            updateSentry();
            status = "Re-zeroed " + arm;
        }
    }

    closesocket(sock);
    cv::destroyAllWindows();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
