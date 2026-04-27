// ================== 1. 网络与系统核心头文件 (必须放在最前面) ==================
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

// ================== 全局标志位 ==================
atomic<bool> net_cmd_record{false};
atomic<bool> net_cmd_stop{false};
atomic<bool> running{true};

// ================== 网络发送模块 (Master 专用) ==================
// 新增参数 local_ip：强制指定发送端网卡
void sendUdpCommand(const string& local_ip, const string& target_ip, int port, const string& msg) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        cerr << "[Master ERROR] Failed to create socket." << endl;
        return;
    }

    // 核心修复 1：强制 Master 绑定到直连相机的网卡 (192.168.10.1)
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = 0; // 0 表示让系统随机分配一个发送端口
    inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr);

    if (::bind(sock, (sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
#ifdef _WIN32
        cerr << "[Master WARNING] Failed to bind local interface " << local_ip << ". Error: " << WSAGetLastError() << endl;
#else
        cerr << "[Master WARNING] Failed to bind local interface " << local_ip << "." << endl;
#endif
    } else {
        cout << "[Master Debug] Successfully bound to local NIC: " << local_ip << endl;
    }

    // 设定目标地址
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
// 新增参数 bind_ip：强制指定监听端网卡
void udpListenerWorker(const string& bind_ip, int port) {
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        cerr << "[Slave ERROR] Failed to create socket." << endl;
        return;
    }

    // 允许端口重用 (防止上次没退干净导致端口被占)
    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&optval, sizeof(optval));

    // 核心修复 2：抛弃 INADDR_ANY，严格绑定到直连网卡 (192.168.10.2)
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, bind_ip.c_str(), &server_addr.sin_addr);

    if (::bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
#ifdef _WIN32
        cerr << "[Slave ERROR] Bind failed on " << bind_ip << ":" << port << ". Error code: " << WSAGetLastError() << endl;
#else
        cerr << "[Slave ERROR] Bind failed on " << bind_ip << ":" << port << "." << endl;
#endif
        closesocket(sock);
        return;
    }

    // 设置超时机制，保证能够安全退出
#ifdef _WIN32
    DWORD timeout = 100; // Windows 毫秒
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // Linux 100毫秒
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif

    char buffer[256];
    cout << "[Slave] Thread successfully strictly bound to " << bind_ip << ":" << port << "..." << endl;

    while (running) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        int bytes = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&client_addr, &client_len);
        
        if (bytes > 0) {
            buffer[bytes] = '\0';
            string cmd(buffer);
            
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
            cout << "\n[Slave] Received " << bytes << " bytes from " << client_ip << ":" << ntohs(client_addr.sin_port) << " -> " << cmd << endl;

            if (cmd == "CMD_START") {
                net_cmd_record = true;
            } else if (cmd == "CMD_STOP") {
                net_cmd_stop = true;
            }
        } 
        else if (bytes < 0) {
            // 核心修复 3：捕获非超时的异常错误
#ifdef _WIN32
            int err = WSAGetLastError();
            if (err != WSAETIMEDOUT && err != WSAEWOULDBLOCK) {
                cerr << "[Slave ERROR] recvfrom exception. Code: " << err << endl;
            }
#endif
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
        // 传入当前 Slave 的网卡 IP 进行精准绑定
        listener_thread = thread(udpListenerWorker, slave_ip, port);
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
                // 传入 master_ip 和 slave_ip，强制通过指定网卡发包
                sendUdpCommand(master_ip, slave_ip, port, "CMD_START");
                trigger_start = true;
            } 
            else if (key == 's' && is_recording) {
                cout << "\n[UI] 's' pressed. Sending STOP command..." << endl;
                sendUdpCommand(master_ip, slave_ip, port, "CMD_STOP");
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