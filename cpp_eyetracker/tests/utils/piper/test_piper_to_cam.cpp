// 机械臂 flange → tool-in-CCS 位姿, 输出四元数 + ZXZ'' 两种格式
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
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <chrono>
#include <filesystem>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

#include "piper/piper.hpp"
#include "cfg/config.hpp"

using namespace std;
using namespace gazeestimation;

atomic<bool> g_running{true};
string g_arm = "upper";
string g_ip;
int g_port = 0;
mutex g_mtx;
Pose    g_flange_quat;
Pose    g_tool_quat;
PoseZxz g_tool_zxz;
string  g_status = "Disconnected";

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

void tcpWorker() {
    while (g_running) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        sockaddr_in sv{}; sv.sin_family = AF_INET; sv.sin_port = htons(g_port);
        inet_pton(AF_INET, g_ip.c_str(), &sv.sin_addr);
        g_status = "Connecting...";
        if (connect(s, (sockaddr*)&sv, sizeof(sv)) != 0) {
            g_status = "Connect failed";
            closesocket(s);
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }
        g_status = "Connected";
        while (g_running) {
            string cmd = "GET_POSE:" + g_arm + "\n";
            if (send(s, cmd.c_str(), (int)cmd.length(), 0) <= 0) break;
            string resp;
            if (!recvLine(s, resp) || resp.rfind("POSE:",0)!=0) break;
            string data = resp.substr(5);
            size_t pp = data.find(':'); string an = data.substr(0,pp);
            string vs = data.substr(pp+1);
            vector<double> nv; stringstream vss(vs); string tok;
            while (getline(vss,tok,',')) nv.push_back(stod(tok));
            if (nv.size()<10) continue;
            Pose flange{{nv[0],nv[1],nv[2]},{nv[3],nv[4],nv[5],nv[6]}};
            lock_guard<mutex> lk(g_mtx);
            g_flange_quat = flange;
            this_thread::sleep_for(chrono::milliseconds(200));
        }
        g_status = "Disconnected";
        closesocket(s);
        this_thread::sleep_for(chrono::seconds(1));
    }
}

int main() {
#ifdef _WIN32
    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
#endif
    namespace fs = std::filesystem;
    auto yp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"cfg"/"piper.yaml").string();
    PiperToCam p2c(yp);
    Cfg cfg(yp);
    g_ip = cfg["network"]["ubuntu_ip"].as<string>();
    g_port = cfg["network"]["ctrl_port"].as<int>();   // piper_windows_ctrl_server.py (49301)

    cout << "=== Piper Flange -> Tool in CCS ===" << endl;
    cout << "Server: " << g_ip << ":" << g_port << endl;
    cout << "Arms:";
    for (auto& a : p2c.arms()) cout << " " << a;
    cout << "\n[t]switch  [g]print  [q]quit\n" << endl;

    thread tcp(tcpWorker);

    cv::namedWindow("Piper Arm → CCS", cv::WINDOW_NORMAL);
    cv::resizeWindow("Piper Arm → CCS", 640, 480);

    while (g_running) {
        cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC3);
        string arm_label = (g_arm=="upper")?"UPPER":"LOWER";
        cv::Scalar ac = (g_arm=="upper")?cv::Scalar(0,215,255):cv::Scalar(200,80,255);
        cv::putText(canvas, arm_label + "  " + g_status, {15,25}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    (g_status=="Connected"?cv::Scalar(0,255,0):ac), 2, cv::LINE_AA);

        Pose fl;
        { lock_guard<mutex> lk(g_mtx); fl = g_flange_quat; }
        Pose tq; PoseZxz tz;
        try { tq = p2c.convert(g_arm, fl); tz = p2c.convertZxz(g_arm, fl); }
        catch (...) {}

        int y = 55;
        auto put = [&](const string& s, cv::Scalar c={255,255,255}) {
            cv::putText(canvas, s, {15,y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv::LINE_AA);
            y += 22;
        };

        char b[256];
        put("--- FLANGE (arm frame) ---", {200,200,200});
        snprintf(b,sizeof(b),"XYZ:         [%.4f, %.4f, %.4f] m", fl.pos.x,fl.pos.y,fl.pos.z); put(b);
        snprintf(b,sizeof(b),"Quat (wxyz):  [%.4f, %.4f, %.4f, %.4f]", fl.quat.w,fl.quat.x,fl.quat.y,fl.quat.z); put(b);
        y += 6;
        put("--- TOOL in CCS (center cam frame) ---", {200,200,200});
        snprintf(b,sizeof(b),"Quat XYZ:     [%.4f, %.4f, %.4f] m", tq.pos.x,tq.pos.y,tq.pos.z); put(b);
        snprintf(b,sizeof(b),"Quat wxyz:    [%.4f, %.4f, %.4f, %.4f]", tq.quat.w,tq.quat.x,tq.quat.y,tq.quat.z); put(b);
        put("Z-X-Z'' Euler:", {0,255,0});
        snprintf(b,sizeof(b),"  alpha,beta,gamma: [%.2f, %.2f, %.2f] deg", tz.alpha,tz.beta,tz.gamma); put(b);

        y = 450;
        cv::putText(canvas, "[t]switch  [g]print  [q]quit", {15,y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {150,150,150}, 1);

        cv::imshow("Piper Arm -> CCS", canvas);
        char key = (char)cv::waitKey(50);
        if (key=='q'||key==27) g_running = false;
        else if (key=='t'||key=='T') g_arm = (g_arm=="upper")?"lower":"upper";
        else if (key=='g'||key=='G') {
            cout << "\n=== " << g_arm << " ===" << endl;
            cout << fixed << setprecision(4);
            cout << "FLANGE pos: ["<<fl.pos.x<<", "<<fl.pos.y<<", "<<fl.pos.z<<"]" << endl;
            cout << "FLANGE quat:[w="<<fl.quat.w<<", x="<<fl.quat.x<<", y="<<fl.quat.y<<", z="<<fl.quat.z<<"]"<<endl;
            cout << "TOOL_CCS quat: ["<<tq.pos.x<<", "<<tq.pos.y<<", "<<tq.pos.z
                 <<"]  wxyz=["<<tq.quat.w<<","<<tq.quat.x<<","<<tq.quat.y<<","<<tq.quat.z<<"]"<<endl;
            cout << fixed << setprecision(2);
            cout << "TOOL_CCS ZXZ'': ["<<tz.alpha<<", "<<tz.beta<<", "<<tz.gamma<<"] deg\n"<<endl;
        }
    }
    g_running = false;
    if (tcp.joinable()) tcp.join();
    cv::destroyAllWindows();
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
