#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <atomic>
#include <pylon/PylonIncludes.h>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

using namespace std;
using namespace gazeestimation;

int main() {
    cout << "=== [TEST] Single Basler Camera BlockID Verification ===" << endl;

    Pylon::PylonInitialize();
    Cfg cfg;

    // 1. 从配置中读取参数
    string sn = cfg["test_single_cam"]["sn"].as<string>();
    bool use_hw_trigger = cfg["test_single_cam"]["hardware_trigger"].as<bool>();
    double target_fps = cfg["test_single_cam"]["fps"].as<double>();
    double gain = cfg["test_single_cam"]["gain"].as<double>();
    double gamma = cfg["test_single_cam"]["gamma"].as<double>();
    double exp_time = cfg["test_single_cam"]["exposure_time"].as<double>();

    // 2. 初始化相机对象
    BaslerCamera cam(sn);
    
    cv::Mat latest_frame;
    FrameMeta latest_meta;
    mutex frame_mtx;
    atomic<bool> is_grabbing{false};

    // 3. 设置回调函数
    cam.setFrameCallback([&](const cv::Mat& frame, FrameMeta meta) {
        lock_guard<mutex> lock(frame_mtx);
        frame.copyTo(latest_frame);
        latest_meta = meta;

        // 仅在控制台打印前 5 帧，观察 BlockID 是否置零 (通常从 1 开始)
        if (meta.blockID <= 5) {
            cout << "[Logger] SN: " << sn 
                 << " | Grabbed! BlockID: " << meta.blockID 
                 << " | Timestamp: " << meta.timestamp << endl;
        }
    });

    // 封装启动相机的逻辑
    auto start_camera = [&]() {
        cout << "\n---> Initializing and Starting Camera..." << endl;
        TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
        
        if (!cam.open(mode)) {
            cerr << "[Error] Failed to open camera SN: " << sn << endl;
            return false;
        }

        // 配置参数
        if (!use_hw_trigger) cam.setFrameRate(target_fps);
        cam.setGain(gain);
        cam.setGamma(gamma);
        cam.setExposureTime(exp_time);

        if (cam.start()) {
            is_grabbing = true;
            cout << "---> Camera started successfully! Waiting for frames..." << endl;
            return true;
        }
        return false;
    };

    // 封装停止相机的逻辑
    auto stop_camera = [&]() {
        cout << "\n---> Stopping and Closing Camera..." << endl;
        cam.close(); 
        is_grabbing = false;
        
        // 清空画面
        lock_guard<mutex> lock(frame_mtx);
        latest_frame.release(); 
    };

    // 初次启动相机
    start_camera();

    cv::namedWindow("Single Cam Test", cv::WINDOW_NORMAL);
    cv::resizeWindow("Single Cam Test", 800, 600);

    cout << "\n=======================================================" << endl;
    cout << " Controls:" << endl;
    cout << "  [ r ] - Restart Camera (Test if BlockID resets)" << endl;
    cout << "  [ q ] - Quit" << endl;
    cout << "=======================================================\n" << endl;

    bool running = true;
    while (running) {
        cv::Mat display;
        FrameMeta meta_copy;
        
        // 极速抢锁拿画面和元数据
        {
            lock_guard<mutex> lock(frame_mtx);
            if (!latest_frame.empty()) {
                display = latest_frame.clone();
                meta_copy = latest_meta;
            }
        }

        // 可视化逻辑
        if (!display.empty()) {
            if (display.type() == CV_8UC1) {
                cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
            }
            
            // 在画面左上角显著标出 BlockID
            string text = "BlockID: " + to_string(meta_copy.blockID);
            cv::putText(display, text, cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 0), 5); // 黑底
            cv::putText(display, text, cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 3); // 绿字
            
            cv::imshow("Single Cam Test", display);
        } else {
            // 没有画面时显示黑屏提示
            cv::Mat empty = cv::Mat::zeros(600, 800, CV_8UC3);
            string msg = is_grabbing ? "Waiting for trigger..." : "Camera is STOPPED.";
            cv::Scalar color = is_grabbing ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
            cv::putText(empty, msg, cv::Point(50, 300), cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
            cv::imshow("Single Cam Test", empty);
        }

        // 按键处理
        char key = (char)cv::waitKey(30);
        if (key == 'q' || key == 27) {
            running = false;
        } else if (key == 'r') {
            if (is_grabbing) {
                stop_camera();
                // 停顿一小会儿模拟重启
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            start_camera();
        }
    }

    stop_camera();
    cv::destroyAllWindows();
    Pylon::PylonTerminate();
    
    return 0;
}