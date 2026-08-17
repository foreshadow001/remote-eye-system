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
#include <random>
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

// ================== 颜色 ==================
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

// 随机 HSV → 鲜艳颜色 (随机 RGB 会偏灰)
void sendRandom() {
    random_device rd; mt19937 gen(rd());
    uniform_real_distribution<double> dh(0.0, 360.0);
    string cmd = "PIX";
    for (int i = 0; i < 25; ++i) {
        int r, g, b; hsvToRgb(dh(gen), 1.0, 1.0, r, g, b);
        g_grid[i] = {r, g, b};
        char hexbuf[8]; snprintf(hexbuf, sizeof(hexbuf), " %02x%02x%02x", r, g, b);
        cmd += hexbuf;
    }
    g_status = sendSerial(cmd) ? "Sent 25 random colors." : "Send failed.";
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
    cv::putText(canvas, "[t] switch  [s] random  [ESC/q] quit", cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(140, 140, 140), 1, cv::LINE_AA); ty += 30;
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
        else if (key == 's' || key == 'S') { sendRandom(); }
    }
    closeSerial();
    cv::destroyAllWindows();
    return 0;
}
