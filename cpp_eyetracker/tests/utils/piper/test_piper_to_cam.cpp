// 将机械臂 flange 位姿 (TCP 获取) 转换到相机坐标系 (center cam)
// 变换链与 end_pose_monitor.py 完全一致
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
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
    Cfg cfg((fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg"/"piper.yaml").string());
    g_ip   = cfg["network"]["ubuntu_ip"].as<string>();
    g_port = cfg["network"]["port"].as<int>();

    // 读取 arm 配置
    auto loadArm = [&](const string& arm) {
        Pt3 t_t{0,0,0.02}, t_r{0,0,0};
        Pt3 a_t{0,0,0},  a_r{0,90,90};
        try {
            auto& a = cfg["arms"][arm];
            auto& tool = a["tool"];
            auto& ccs  = a["arm_in_ccs"];
            auto& tt = tool["translation"];  t_t = {tt[0].as<double>(), tt[1].as<double>(), tt[2].as<double>()};
            auto& tr = tool["rotation_zxz"]; t_r = {tr[0].as<double>(), tr[1].as<double>(), tr[2].as<double>()};
            auto& at = ccs["translation"];   a_t = {at[0].as<double>(), at[1].as<double>(), at[2].as<double>()};
            auto& ar = ccs["rotation_zxz"];  a_r = {ar[0].as<double>(), ar[1].as<double>(), ar[2].as<double>()};
        } catch (...) { cerr << "[Warn] Using default arm config for " << arm << endl; }
        return make_tuple(t_t, t_r, a_t, a_r);
    };

    auto [ut_t, ut_r, ua_t, ua_r] = loadArm("upper");
    auto [lt_t, lt_r, la_t, la_r] = loadArm("lower");

    cout << fixed << setprecision(4);
    cout << "=== Piper Flange → Center Cam Pose ===\n" << endl;
    cout << "Server: " << g_ip << ":" << g_port << endl;
    cout << "Upper arm_in_ccs: t=(" << ua_t.x<<","<<ua_t.y<<","<<ua_t.z<<") r_zxz=("<<ua_r.x<<","<<ua_r.y<<","<<ua_r.z<<")deg" << endl;
    cout << "Lower arm_in_ccs: t=(" << la_t.x<<","<<la_t.y<<","<<la_t.z<<") r_zxz=("<<la_r.x<<","<<la_r.y<<","<<la_r.z<<")deg" << endl;
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

            // Transform
            Pt3 t_t, t_r, a_t, a_r;
            if (g_arm=="upper") { t_t=ut_t; t_r=ut_r; a_t=ua_t; a_r=ua_r; }
            else                { t_t=lt_t; t_r=lt_r; a_t=la_t; a_r=la_r; }

            Pose tool_in_ccs = armToolToCamPose(flange, t_t, t_r, a_t, a_r);

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
