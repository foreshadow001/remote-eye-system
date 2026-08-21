// test_m5stack.cpp — M5Stack Atom Matrix (5x5 RGB LED) 串口灯光控制
// 't' 切换 upper/lower 设备 (COM 口), 's' 发送随机颜色, UI 实时预览 5x5 LED 阵列
// 串口协议 (ASCII, 换行结尾): PIX <rrggbb>*25 / ALL <rrggbb> / CLEAR / BRIGHT <0-255>
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <vector>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>

#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== 全局状态 ==================
string g_cur_dev = "upper";                        // upper / lower
HANDLE g_serial = INVALID_HANDLE_VALUE;            // 当前串口句柄
string g_open_port;
vector<array<int,3>> g_grid(25, {0,0,0});          // 5x5 预览 (最近一次发送的颜色)
string g_status = "";

// ================== 串口 ==================
// 尝试打开 COM 口 (8N1). 成功则替换当前连接, 失败保持原连接不变
bool tryOpenSerial(const string& port, int baud) {
    string path = "\\\\.\\" + port;
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        cerr << "[Serial] Cannot open " << port << " (error " << GetLastError() << ")" << endl;
        return false;
    }
    DCB dcb{}; dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) { CloseHandle(h); cerr << "[Serial] GetCommState failed." << endl; return false; }
    dcb.BaudRate = baud; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); cerr << "[Serial] SetCommState failed." << endl; return false; }
    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = 50; to.ReadTotalTimeoutConstant = 50; to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 500; to.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h, &to);
    if (g_serial != INVALID_HANDLE_VALUE) CloseHandle(g_serial);
    g_serial = h; g_open_port = port;
    cout << "[Serial] Opened " << port << " @ " << baud << endl;
    return true;
}

void closeSerial() {
    if (g_serial != INVALID_HANDLE_VALUE) { CloseHandle(g_serial); g_serial = INVALID_HANDLE_VALUE; g_open_port.clear(); }
}

bool sendSerial(const string& cmd) {
    if (g_serial == INVALID_HANDLE_VALUE) { g_status = "Serial not open."; return false; }
    string line = cmd + "\n";
    DWORD written = 0;
    if (!WriteFile(g_serial, line.c_str(), (DWORD)line.size(), &written, NULL) || written != line.size()) {
        cerr << "[Serial] Write failed." << endl;
        return false;
    }
    cout << "[Serial] -> " << g_open_port << ": " << cmd << endl;
    return true;
}

// ================== 状态灯 (capture_with_M5Stack 的 LedState) ==================
// kind: ALL = 25 颗全亮;  BREATH = 25 颗全亮呼吸 (固件动画);
//       CROSS = 绿白十字常亮 (PIX);  FLOW = 斜向彩流
struct LedPattern { string name; string kind; array<int,3> rgb; string desc; };
vector<LedPattern> g_states = {
    {"PIPER_INIT", "ALL",       {  0,  0,255}, "all 25 blue"},
    {"READY",      "BREATH",    {  0,255,  0}, "all 25 green breathing"},
    {"CAPTURING",  "CROSS",     {  0,255,  0}, "green-white steady cross"},
    {"WAITING",    "ALL",       {255,128,  0}, "all 25 orange"},
    {"EXHAUSTED",  "ALL",       {255,  0,  0}, "all 25 red"},
    {"OVER",       "FLOW",      {255,  0,255}, "diagonal rainbow flow"},
};
int g_state_idx = -1;   // -1 = 尚未发送

// 十字 = 中心行 (10..14) + 中心列 (2,7,17,22), 共 9 颗 (亮度相同, 与固件一致)
const int CROSS_IDX[9] = {2, 7, 10, 11, 12, 13, 14, 17, 22};

void hsvToRgb(double h, double s, double v, int& r, int& g, int& b) {
    double c = v * s;
    double x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    double m = v - c;
    double rp = 0, gp = 0, bp = 0;
    if (h < 60) { rp = c; gp = x; }
    else if (h < 120) { rp = x; gp = c; }
    else if (h < 180) { gp = c; bp = x; }
    else if (h < 240) { gp = x; bp = c; }
    else if (h < 300) { rp = x; bp = c; }
    else { rp = c; bp = x; }
    r = (int)((rp + m) * 255); g = (int)((gp + m) * 255); b = (int)((bp + m) * 255);
}

// 十字 PIX 命令: 外臂+中心 (2,10,12,14,22) = 白, 内 3x3 小十字 (7,11,13,17) = rgb, 其余黑
string pixCrossCmd(const array<int,3>& rgb) {
    char hexbuf[8];
    snprintf(hexbuf, sizeof(hexbuf), "%02x%02x%02x", rgb[0], rgb[1], rgb[2]);
    string px[25]; for (auto& v : px) v = "000000";
    for (int i : {7, 11, 13, 17}) px[i] = hexbuf;
    for (int i : {2, 10, 12, 14, 22}) px[i] = "ffffff";
    string cmd = "PIX"; for (auto& v : px) cmd += " " + v; return cmd;
}

// 按 s 循环发送状态图案 (每次切换仅一条指令; 呼吸由固件 MODE BREATH 动画)
void sendNextState() {
    g_state_idx = (g_state_idx + 1) % (int)g_states.size();
    auto& st = g_states[g_state_idx];
    bool ok = false;
    char hexbuf[8];
    snprintf(hexbuf, sizeof(hexbuf), "%02x%02x%02x", st.rgb[0], st.rgb[1], st.rgb[2]);
    if (st.kind == "FLOW") ok = sendSerial("MODE FLOW");
    else if (st.kind == "BREATH") ok = sendSerial(string("MODE BREATH ") + hexbuf);
    else if (st.kind == "CROSS") ok = sendSerial(pixCrossCmd(st.rgb));
    else ok = sendSerial(string("MODE ALL ") + hexbuf);
    g_status = ok ? ("State: " + st.name + " — " + st.desc) : "Send failed.";
    // 更新 5x5 预览
    for (int i = 0; i < 25; ++i) g_grid[i] = {0, 0, 0};
    if (st.kind == "FLOW") {
        // 斜向彩流静态快照: 每条反对角线 (x+y) 一个色相, 偏移 40° (与固件一致)
        for (int i = 0; i < 25; ++i) {
            int x = i % 5, y = i / 5;
            int r, g, b; hsvToRgb(((x + y) * 40) % 360, 1.0, 1.0, r, g, b);
            g_grid[i] = {r, g, b};
        }
    } else if (st.kind == "CROSS") {
        // 外臂+中心 = 白, 内 3x3 小十字 = rgb (与 PIX 命令一致)
        for (int j = 0; j < 9; ++j) {
            int i = CROSS_IDX[j];
            g_grid[i] = (i == 2 || i == 10 || i == 12 || i == 14 || i == 22) ? array<int,3>{255, 255, 255} : st.rgb;
        }
    } else {
        for (int i = 0; i < 25; ++i) g_grid[i] = st.rgb;
    }
}

// ================== UI ==================
void render(cv::Mat& canvas) {
    canvas = cv::Mat::zeros(420, 760, CV_8UC3);
    // 左侧: 5x5 LED 网格预览
    int cell = 56, gap = 8, x0 = 30, y0 = 60;
    for (int i = 0; i < 25; ++i) {
        int row = i / 5, col = i % 5;
        cv::Rect r(x0 + col * (cell + gap), y0 + row * (cell + gap), cell, cell);
        cv::rectangle(canvas, r, cv::Scalar(70, 70, 70), 1);
        cv::rectangle(canvas, r, cv::Scalar(g_grid[i][2], g_grid[i][1], g_grid[i][0]), -1);   // BGR
    }
    // 右侧: 设备信息 + 提示
    int tx = x0 + 5 * (cell + gap) + 15, ty = 90;
    cv::putText(canvas, "M5Stack Atom Matrix", cv::Point(tx, ty), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 215, 255), 2, cv::LINE_AA); ty += 40;
    string dev = g_cur_dev; for (auto& c : dev) c = (char)toupper((unsigned char)c);
    cv::putText(canvas, "Device : " + dev, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2); ty += 30;
    string port = (g_serial != INVALID_HANDLE_VALUE) ? g_open_port : "(closed)";
    cv::putText(canvas, "Port   : " + port, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2); ty += 40;
    cv::putText(canvas, "[t] switch  [s] state  [ESC/q] quit", cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(140, 140, 140), 1, cv::LINE_AA); ty += 30;
    cv::putText(canvas, g_status, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

int main() {
    cout << "=== M5Stack Atom Matrix LED Control ===" << endl;
    auto cfg_dir = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg").string();
    Cfg cfg(cfg_dir + "/M5Stack.yaml");
    string port_upper = cfg["ports"]["upper"].as<string>();
    string port_lower = cfg["ports"]["lower"].as<string>();
    int baud = 115200; try { baud = cfg["serial"]["baud_rate"].as<int>(); } catch (...) {}
    cout << "Upper: " << port_upper << "  Lower: " << port_lower << "  Baud: " << baud << endl;

    // 默认打开 upper
    if (tryOpenSerial(port_upper, baud)) g_status = "Connected to " + g_open_port;
    else g_status = "Upper not connected. Press [t] to try " + port_lower + ".";

    cv::namedWindow("M5Stack LED Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("M5Stack LED Control", 760, 420);
    while (true) {
        cv::Mat canvas; render(canvas); cv::imshow("M5Stack LED Control", canvas);
        char key = (char)cv::waitKey(30);
        if (key == 27 || key == 'q') break;
        else if (key == 't' || key == 'T') {
            string next = (g_cur_dev == "upper") ? "lower" : "upper";
            string next_port = (next == "upper") ? port_upper : port_lower;
            if (tryOpenSerial(next_port, baud)) { g_cur_dev = next; g_status = "Connected to " + g_open_port; }
            else g_status = "Cannot open " + next_port + " (keeping " + g_open_port + ")";
        }
        else if (key == 's' || key == 'S') { sendNextState(); }
    }
    sendSerial("CLEAR");   // 退出前熄灯
    closeSerial();
    cv::destroyAllWindows();
    return 0;
}
