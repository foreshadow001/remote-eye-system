#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;

using namespace std;
using namespace gazeestimation;

int main() {
    cout << "=== [TEST] Basler Camera Debug Tool (Optimized) ===" << endl;

    Cfg cfg;
    BaslerCamera cam(cfg["test_one_cam"]["cam_index"].as<std::string>());

    // 1. 打开相机
    if (!cam.open()) {
        cerr << "[test_cam] Failed to open Basler camera.\n";
        return -1;
    }

    // 2. 设置参数
    double target_fps = cfg["test_one_cam"]["fps"].as<double>();
    cam.setFrameRate(target_fps);
    cam.setGain(cfg["test_one_cam"]["gain"].as<double>());
    cam.setGamma(cfg["test_one_cam"]["gamma"].as<double>());
    cam.setExposureTime(cfg["test_one_cam"]["exposure_time"].as<double>());

    cout << "[test_cam] Camera initialized successfully." << endl;
    cout << "Press 'r' to start recording, 's' to stop, 'q' to quit.\n";

    // 3. 创建预览窗口
    cv::namedWindow("Basler Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow(
        "Basler Preview",
        cfg["test_one_cam"]["frame_width"].as<int>(),
        cfg["test_one_cam"]["frame_height"].as<int>()
    );

    // 4. 准备目录
    string save_dir = cfg["test_one_cam"]["save_dir"].as<std::string>();
    try {
        if (!fs::exists(save_dir)) fs::create_directories(save_dir);
    } catch (const std::exception& e) {
        cerr << "[test_cam] Failed to create directory: " << e.what() << endl;
        return -1;
    }

    bool recording = false;
    bool running = true;
    string current_filename;

    // --- 性能统计变量 ---
    long long frame_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();
    int fps_counter = 0;
    
    // 【关键优化】显示间隔：每隔 20 帧才刷新一次界面
    // 200FPS下，这意味着界面每 0.1秒刷新一次，完全足够人眼观察，且极大降低CPU负担
    const int DISPLAY_INTERVAL = 20; 

    while (running) {
        // A. 抓图 (这是最耗时且必须优先保证的操作)
        cv::Mat frame = cam.grabFrame();
        if (frame.empty()) {
            cerr << "[test_cam] Empty frame captured.\n";
            continue; 
        }

        // B. 录制 (如果开启)
        if (recording) {
            cam.writeFrame(frame);
        }

        // C. FPS 统计 (用于检查是否真的达到了 200 FPS)
        fps_counter++;
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - last_time;
        if (diff.count() >= 1.0) {
            std::cout << "\r[Status] Real FPS: " << fps_counter 
                      << " | Target: " << target_fps 
                      << " | Rec: " << (recording ? "ON " : "OFF") << "   " << std::flush;
            fps_counter = 0;
            last_time = now;
        }

        // D. 界面显示与按键 (降频处理)
        frame_count++;
        if (frame_count % DISPLAY_INTERVAL == 0) {
            cv::imshow("Basler Preview", frame);

            // 只有在刷新界面时才检测按键，这会引入微小的按键延迟(0.1s)，但为了FPS这是值得的
            char key = (char)cv::waitKey(1); 
            
            switch (key) {
                case 'r': // 开始录制
                    if (!recording) {
                        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                        char buf[64];
                        strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                        current_filename = save_dir + "/record_" + string(buf) + ".avi";
                        
                        // 开启录制
                        cam.startRecording(current_filename, target_fps);
                        recording = true;
                        cout << "\n[test_cam] Start recording -> " << current_filename << endl;
                    }
                    break;

                case 's': // 停止录制
                    if (recording) {
                        cam.stopRecording();
                        recording = false;
                        cout << "\n[test_cam] Recording stopped. Saved -> " << current_filename << endl;
                    }
                    break;

                case 'q':
                case 27: // ESC
                    running = false;
                    break;

                case ' ': // 拍照
                    {
                        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                        char buf[64];
                        strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                        string photo_filename = save_dir + "/photo_" + string(buf) + ".jpg";
                        cv::imwrite(photo_filename, frame);
                        cout << "\n[test_cam] Photo saved -> " << photo_filename << endl;
                    }
                    break;
            }
        }
    }

    if (recording) {
        cam.stopRecording();
    }

    cam.close();
    cv::destroyAllWindows();

    cout << "\n[test_cam] Program ended.\n";
    return 0;
}