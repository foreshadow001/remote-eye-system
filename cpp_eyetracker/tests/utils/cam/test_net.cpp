// ================== 1. 网络与系统核心头文件 (必须放在最前面) ==================
#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN // 核心修复：防止 winsock.h 冲突
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
#include "cfg/config.hpp" // 请确保你的项目能找到它

using namespace std;

// ================== 全局标志位 ==================
atomic<bool> net_cmd_record{false};
atomic<bool> net_cmd_stop{false};
atomic<bool> running{true};

// ================== 网络发送模块 (Master 专用) ==================
void sendUdpCommand(const string& target_ip, int port, const string& msg) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        cerr << "[Master ERROR] Failed to create socket." << endl;
        return;
    }

    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr);

    cout << "[Master] Sending '" << msg << "' to " << target_ip << ":" << port << "..." << endl;
    
    int ret = sendto(sock, msg.c_str(), msg.length(), 0, (sockaddr*)&target_addr, sizeof(target_addr));
    
    if (ret == -1) {
#ifdef _WIN32
        cerr << "[Master ERROR] sendto failed. Error code: " << WSAGetLastError() << endl;
#else
        cerr << "[Master ERROR] sendto failed." << endl;
#endif
    } else {
        cout << "[Master] Packet sent successfully (" << ret << " bytes)." << endl;
    }
    
    closesocket(sock);
}

// ================== 网络监听模块 (Slave 专用) ==================
void udpListenerWorker(int port) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        cerr << "[Slave ERROR] Failed to create socket." << endl;
        return;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听本地所有网卡的 8888 端口

    // 修复 1：使用 ::bind 避免和 std::bind 冲突
    if (::bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
#ifdef _WIN32
        cerr << "[Slave ERROR] Bind failed on port " << port << ". Error code: " << WSAGetLastError() << endl;
#else
        cerr << "[Slave ERROR] Bind failed on port " << port << "." << endl;
#endif
        closesocket(sock);
        return;
    }

    // 修复 2：Windows 和 Linux 超时参数的平台差异
#ifdef _WIN32
    DWORD timeout = 100; // Windows 以毫秒为单位
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) == -1) {
        cerr << "[Slave WARNING] setsockopt timeout failed." << endl;
    }
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // Linux 为 100ms
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) == -1) {
        cerr << "[Slave WARNING] setsockopt timeout failed." << endl;
    }
#endif

    char buffer[256];
    cout << "[Slave] Thread successfully listening on UDP port " << port << "..." << endl;

    while (running) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        // 由于设置了超时，recvfrom 不会死锁卡死主线程退出了
        int bytes = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&client_addr, &client_len);
        
        if (bytes > 0) {
            buffer[bytes] = '\0';
            string cmd(buffer);
            
            // 打印客户端信息，用于核实是 PC A 发来的包
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
            cout << "\n[Slave] Received " << bytes << " bytes from " << client_ip << ":" << ntohs(client_addr.sin_port) << " -> " << cmd << endl;

            if (cmd == "CMD_START") {
                net_cmd_record = true;
            } else if (cmd == "CMD_STOP") {
                net_cmd_stop = true;
            }
        }
    }
    closesocket(sock);
    cout << "[Slave] Thread exited cleanly." << endl;
}

// ================== 主程序 ==================
int main() {
    cout << "=== [TEST] UDP Network Synchronization ===" << endl;

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        cerr << "[System ERROR] WSAStartup failed." << endl;
        return 1;
    }
#endif

    // 1. 读取配置
    Cfg cfg;
    bool is_master_pc = true;
    string master_ip = "192.168.10.1";
    string slave_ip = "192.168.10.2";
    int port = 8888;

    try {
        is_master_pc  = cfg["test_net"]["is_master"].as<bool>();
        master_ip     = cfg["test_net"]["master_ip"].as<string>();
        slave_ip      = cfg["test_net"]["slave_ip"].as<string>();
        port          = cfg["test_net"]["port"].as<int>();
    } catch (const std::exception& e) {
        cerr << "[System ERROR] Error reading config: " << e.what() << endl;
    }

    cout << "\n--- Current Configuration ---" << endl;
    cout << "Role     : " << (is_master_pc ? "MASTER (Sender)" : "SLAVE (Receiver)") << endl;
    cout << "Master IP: " << master_ip << endl;
    cout << "Slave IP : " << slave_ip << endl;
    cout << "Port     : " << port << endl;
    cout << "-----------------------------\n" << endl;

    // 2. 角色分支初始化
    thread listener_thread;
    if (!is_master_pc) {
        listener_thread = thread(udpListenerWorker, port);
    }

    // 3. 准备 UI
    cv::namedWindow("Net Sync Test UI", cv::WINDOW_NORMAL);
    cv::resizeWindow("Net Sync Test UI", 400, 300);
    bool is_recording = false;

    // 4. 事件主循环
    while (running) {
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

        char key = (char)cv::waitKey(50);
        if (key == 'q' || key == 27) {
            running = false;
            cout << "[System] Shutdown signal received." << endl;
            break;
        }

        bool trigger_start = false;
        bool trigger_stop = false;

        if (is_master_pc) {
            if (key == 'r' && !is_recording) {
                cout << "\n[UI] 'r' pressed. Sending START command..." << endl;
                sendUdpCommand(slave_ip, port, "CMD_START");
                trigger_start = true;
            } 
            else if (key == 's' && is_recording) {
                cout << "\n[UI] 's' pressed. Sending STOP command..." << endl;
                sendUdpCommand(slave_ip, port, "CMD_STOP");
                trigger_stop = true;
            }
        } else {
            if (net_cmd_record) {
                cout << "[UI] Network START command processed." << endl;
                trigger_start = true;
                net_cmd_record = false;
            }
            if (net_cmd_stop) {
                cout << "[UI] Network STOP command processed." << endl;
                trigger_stop = true;
                net_cmd_stop = false;
            }
        }

        if (trigger_start && !is_recording) {
            is_recording = true;
            cout << "[System] >>> QUEUES ARMED. Waiting for HW Trigger! <<<" << endl;
        }

        if (trigger_stop && is_recording) {
            is_recording = false;
            cout << "[System] >>> STOPPED. <<<" << endl;
        }
    }

    // 5. 退出清理
    if (listener_thread.joinable()) {
        listener_thread.join();
    }
    cv::destroyAllWindows();

#ifdef _WIN32
    WSACleanup();
#endif

    cout << "[System] Exited cleanly." << endl;
    return 0;
}