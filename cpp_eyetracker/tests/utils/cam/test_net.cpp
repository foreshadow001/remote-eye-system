// ================== 1. 网络与系统核心头文件 (必须放在最前面) ==================
#ifdef _WIN32
    // 核心宏：阻止 windows.h 自动包含旧版 winsock.h 和其他不常用的 API
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

// ================== 2. 标准库 ==================
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

// ================== 3. 第三方库 ==================
#include <opencv2/opencv.hpp>
#include "cfg/config.hpp" 

using namespace std;

// 全局网络控制标志 (供主线程状态机使用)
atomic<bool> net_cmd_record{false};
atomic<bool> net_cmd_stop{false};
atomic<bool> running{true};

// ================== 网络发送模块 (Master 专用) ==================
void sendUdpCommand(const string& target_ip, int port, const string& msg) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        cerr << "[Network] Failed to create socket." << endl;
        return;
    }

    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr);

    int ret = sendto(sock, msg.c_str(), msg.length(), 0, (sockaddr*)&target_addr, sizeof(target_addr));
    if (ret == -1) {
        cerr << "[Network] Failed to send UDP packet." << endl;
    }
    closesocket(sock);
}

// ================== 网络监听模块 (Slave 专用) ==================
void udpListenerWorker(int port) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) return;

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡上的该端口

    if (::bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        cerr << "[Network] Bind failed on port " << port << ". Port might be in use." << endl;
        closesocket(sock);
        return;
    }

    // 设置非阻塞超时，以便优雅退出线程
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    char buffer[256];
    cout << "[Network] Slave thread listening on UDP port " << port << "..." << endl;

    while (running) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int bytes = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&client_addr, &client_len);
        
        if (bytes > 0) {
            buffer[bytes] = '\0';
            string cmd(buffer);
            
            // 收到指令，触发原子变量
            if (cmd == "CMD_START") {
                net_cmd_record = true;
            } else if (cmd == "CMD_STOP") {
                net_cmd_stop = true;
            }
        }
    }
    closesocket(sock);
    cout << "[Network] Slave thread exited." << endl;
}

// ================== 主程序 ==================
int main() {
    cout << "=== [TEST] UDP Network Synchronization ===" << endl;

#ifdef _WIN32
    // Windows 必须初始化 Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cerr << "WSAStartup failed." << endl;
        return 1;
    }
#endif

    // 1. 读取配置
    Cfg cfg;
    bool is_master_pc  = cfg["test_net"]["is_master"].as<bool>();
    string master_ip   = cfg["test_net"]["master_ip"].as<string>();
    string slave_ip    = cfg["test_net"]["slave_ip"].as<string>();
    int port           = cfg["test_net"]["port"].as<int>();

    cout << "Role: " << (is_master_pc ? "MASTER (Sender)" : "SLAVE (Receiver)") << endl;

    // 2. 角色分支初始化
    thread listener_thread;
    if (!is_master_pc) {
        // 从机启动监听线程
        listener_thread = thread(udpListenerWorker, port);
    }

    // 3. 准备 OpenCV 模拟 UI
    cv::namedWindow("Net Sync Test UI", cv::WINDOW_NORMAL);
    cv::resizeWindow("Net Sync Test UI", 400, 300);
    bool is_recording = false;

    // 4. 事件主循环 (完全模拟相机的循环结构)
    while (running) {
        // --- A. 渲染 UI 状态 ---
        cv::Mat canvas = cv::Mat::zeros(300, 400, CV_8UC3);
        string status_text = is_recording ? "STATUS: ARMED & RECORDING" : "STATUS: IDLE";
        cv::Scalar color = is_recording ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        
        cv::putText(canvas, status_text, cv::Point(20, 150), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        if (is_master_pc) {
            cv::putText(canvas, "Press 'r' to Send START", cv::Point(20, 200), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
            cv::putText(canvas, "Press 's' to Send STOP",  cv::Point(20, 230), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
        } else {
            cv::putText(canvas, "Waiting for network commands...", cv::Point(20, 200), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
        }
        cv::imshow("Net Sync Test UI", canvas);

        // --- B. 事件与按键捕获 ---
        char key = (char)cv::waitKey(50); // 50ms 刷新率
        if (key == 'q' || key == 27) {
            running = false;
        }

        // --- C. 状态机统一触发逻辑 ---
        bool trigger_start = false;
        bool trigger_stop = false;

        if (is_master_pc) {
            // Master 由键盘接管触发，并发送网络包
            if (key == 'r' && !is_recording) {
                cout << "\n[Master] Keyboard 'r' pressed. Sending START cmd..." << endl;
                sendUdpCommand(slave_ip, port, "CMD_START");
                trigger_start = true;
            } 
            else if (key == 's' && is_recording) {
                cout << "\n[Master] Keyboard 's' pressed. Sending STOP cmd..." << endl;
                sendUdpCommand(slave_ip, port, "CMD_STOP");
                trigger_stop = true;
            }
        } else {
            // Slave 由网络标志位接管触发
            if (net_cmd_record) {
                cout << "\n[Slave] Net signal received. Triggering START..." << endl;
                trigger_start = true;
                net_cmd_record = false; // 消费信号
            }
            if (net_cmd_stop) {
                cout << "\n[Slave] Net signal received. Triggering STOP..." << endl;
                trigger_stop = true;
                net_cmd_stop = false;   // 消费信号
            }
        }

        // --- D. 实际执行业务逻辑 ---
        if (trigger_start && !is_recording) {
            is_recording = true;
            // TODO: 在集成时，这里将放置创建目录、打开日志流、设置 ctx->recording = true 等操作
            cout << "[System] >>> RECORDING QUEUES ARMED. Waiting for HW Trigger! <<<" << endl;
        }

        if (trigger_stop && is_recording) {
            is_recording = false;
            // TODO: 在集成时，这里将放置刷新写入队列、关闭日志流等操作
            cout << "[System] >>> RECORDING STOPPED. Writers flushed. <<<" << endl;
        }
    }

    // 5. 退出清理
    cout << "[System] Shutting down..." << endl;
    if (listener_thread.joinable()) {
        listener_thread.join();
    }
    cv::destroyAllWindows();

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}