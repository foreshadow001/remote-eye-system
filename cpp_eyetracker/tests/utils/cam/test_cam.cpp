#include "cam/basler.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>

namespace fs = std::filesystem;

using namespace std;
using namespace gazeestimation;

int main() {
    cout << "=== [TEST] Basler Camera Debug Tool ===" << endl;

    BaslerCamera cam;

    // 打开相机
    if (!cam.open()) {
        cerr << "[test_cam] Failed to open Basler camera.\n";
        return -1;
    }

    // 设置参数
    cam.setFrameRate(100.0);   // 帧率
    cam.setGain(3.0);         // 增益
    cam.setGamma(1.1);        // Gamma
    cam.setExposureTime(10000); // 可选曝光设置，单位微秒

    cout << "[test_cam] Camera initialized successfully." << endl;
    cout << "Press 'r' to start recording, 's' to stop, 'q' to quit.\n";

    // 创建窗口
    cv::namedWindow("Basler Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Basler Preview", 800, 600);

    // 创建保存目录
    string save_dir = "D:/users/projects/new_dataset/test_videos";
    try {
        fs::create_directories(save_dir);
    } catch (const std::exception& e) {
        cerr << "[test_cam] Failed to create directory: " << e.what() << endl;
        return -1;
    }

    bool recording = false;
    bool running = true;
    string current_filename;

    while (running) {
        // 抓取一帧
        cv::Mat frame = cam.grabFrame();
        if (frame.empty()) {
            cerr << "[test_cam]  Empty frame captured.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 显示实时预览
        cv::imshow("Basler Preview", frame);

        // 处理按键
        char key = (char)cv::waitKey(1);
        switch (key) {
            case 'r': // 开始录制
                if (!recording) {
                    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    char buf[64];
                    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                    current_filename = save_dir + "/record_" + string(buf) + ".mp4";
                    cam.startRecording(current_filename);
                    recording = true;
                    cout << "[test_cam] Start recording -> " << current_filename << endl;
                }
                break;

            case 's': // 停止录制
                if (recording) {
                    cam.stopRecording();
                    recording = false;
                    cout << "[test_cam] Saved -> " << current_filename << endl;
                }
                break;

            case 'q':
            case 27:  // ESC
                running = false;
                break;

            default:
                break;

            case ' ': // 拍照保存
                {
                    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    char buf[64];
                    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                    string photo_filename = save_dir + "/photo_" + string(buf) + ".jpg";
                    if (!frame.empty()) {
                        cv::imwrite(photo_filename, frame);
                        cout << "[test_cam] Photo saved -> " << photo_filename << endl;
                    }
                }
                break;
        }

        // 如果正在录制，则写入帧
        if (recording) {
            cam.writeFrame(frame);
        }
    }

    // 退出时清理
    if (recording) {
        cam.stopRecording();
    }

    cam.close();
    cv::destroyAllWindows();

    cout << "[test_cam] Program ended.\n";
    return 0;
}
