// 将机械臂 flange 位姿 (TCP 获取) 转换到相机坐标系 (center cam)
// 变换链与 end_pose_monitor.py 完全一致
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <chrono>
#include <filesystem>
#include <vector>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
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

#include "piper/piper.hpp"
#include "cfg/config.hpp"

using namespace std;
using namespace gazeestimation;

atomic<bool> g_running{true};
string g_arm = "upper";
string g_ip;
int g_port = 0;

// 从 TCP 读一行
bool recvLine(SOCKET s, string& line, int ms = 3000) {
#ifdef _WIN32
    DWORD to = ms; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
#endif
    char buf[256]; string acc;
    auto dl = chrono::steady_clock::now() + chrono::milliseconds(ms);
    while (chrono::steady_clock::now() < dl) {
        int n = recv(s, buf, sizeof(buf)-1, 0);
        if (n <= 0) return false;
        buf[n] = 0; acc += buf;
        size_t nl = acc.find('\n');
        if (nl != string::npos) { line = acc.substr(0, nl); if (!line.empty()&&line.back()=='\r') line.pop_back(); return true; }
    }
    return false;
}

int main() {
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
#endif
    namespace fs = std::filesystem;
    auto yaml_path = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                      / "cfg" / "piper.yaml").string();
    PiperToCam p2c(yaml_path);

    Cfg cfg(yaml_path);
    g_ip   = cfg["network"]["ubuntu_ip"].as<string>();
    g_port = cfg["network"]["port"].as<int>();

    cout << fixed << setprecision(4);
    cout << "=== Piper Flange → Center Cam Pose ===\n" << endl;
    cout << "Server: " << g_ip << ":" << g_port << endl;
    cout << "Loaded arms:";
    for (auto& a : p2c.arms()) cout << " " << a;
    cout << "\nKeys: t=switch arm  g=query+print  q=quit\n" << endl;

    while (g_running) {
        // Connect
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        sockaddr_in sv{}; sv.sin_family=AF_INET; sv.sin_port=htons(g_port);
        inet_pton(AF_INET, g_ip.c_str(), &sv.sin_addr);
        if (connect(s, (sockaddr*)&sv, sizeof(sv)) != 0) {
            cerr << "Connect failed, retry..." << endl;
            closesocket(s);
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }
        cout << "Connected." << endl;

        while (g_running) {
            this_thread::sleep_for(chrono::milliseconds(100));

            // Check keypress
            cout << "\n[t]switch [g]query [q]quit  arm=" << g_arm << " > " << flush;
            string input;
            getline(cin, input);
            if (input == "q") { g_running = false; break; }
            if (input == "t") { g_arm = (g_arm=="upper")?"lower":"upper"; cout << "Switched to " << g_arm << endl; continue; }
            if (input != "g") continue;

            // Send GET_POSE
            string cmd = "GET_POSE:" + g_arm + "\n";
            if (send(s, cmd.c_str(), (int)cmd.length(), 0) <= 0) { cout << "Send failed." << endl; break; }

            string resp;
            if (!recvLine(s, resp) || resp.rfind("POSE:",0)!=0) { cout << "No pose." << endl; continue; }

            // Parse: POSE:arm:x,y,z,qx,qy,qz,qw,alpha,beta,gamma
            string data = resp.substr(5);
            size_t pp = data.find(':'); string an = data.substr(0,pp);
            string vs = data.substr(pp+1);
            vector<double> nv; stringstream vss(vs); string tok;
            while (getline(vss,tok,',')) nv.push_back(stod(tok));
            if (nv.size()<10) continue;

            Pose flange;
            flange.pos    = {nv[0], nv[1], nv[2]};
            flange.quat   = {nv[3], nv[4], nv[5], nv[6]};

            // Transform via PiperToCam
            Pose tool_in_ccs = p2c.convert(g_arm, flange);

            cout << "\n=== " << g_arm << " FLANGE (arm frame) ===" << endl;
            cout << "Pos (m):       ["<<flange.pos.x<<", "<<flange.pos.y<<", "<<flange.pos.z<<"]" << endl;
            cout << "Quat (wxyz):   ["<<flange.quat.w<<", "<<flange.quat.x<<", "<<flange.quat.y<<", "<<flange.quat.z<<"]" << endl;

            cout << "\n=== " << g_arm << " TOOL in CCS (center cam frame) ===" << endl;
            cout << "Pos (m):       ["<<tool_in_ccs.pos.x<<", "<<tool_in_ccs.pos.y<<", "<<tool_in_ccs.pos.z<<"]" << endl;
            cout << "Quat (wxyz):   ["<<tool_in_ccs.quat.w<<", "<<tool_in_ccs.quat.x<<", "<<tool_in_ccs.quat.y<<", "<<tool_in_ccs.quat.z<<"]" << endl;
        }
        closesocket(s);
    }
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
