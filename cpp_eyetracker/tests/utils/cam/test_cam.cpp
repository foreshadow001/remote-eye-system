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
    cout << "=== [TEST] Basler Camera Debug Tool ===" << endl;

    Cfg cfg;
    BaslerCamera cam(cfg["test_one_cam"]["cam_index"].as<std::string>());

    // 打开相机
    if (!cam.open()) {
        cerr << "[test_cam] Failed to open Basler camera.\n";
        return -1;
    }

    // 设置参数
    cam.setFrameRate(cfg["test_one_cam"]["fps"].as<double>());   // 帧率
    cam.setGain(cfg["test_one_cam"]["gain"].as<double>());         // 增益
    cam.setGamma(cfg["test_one_cam"]["gamma"].as<double>());        // Gamma
    cam.setExposureTime(cfg["test_one_cam"]["exposure_time"].as<double>()); // 可选曝光设置，单位微秒

    cout << "[test_cam] Camera initialized successfully." << endl;
    cout << "Press 'r' to start recording, 's' to stop, 'q' to quit.\n";

    // 创建窗口
    cv::namedWindow("Basler Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow(
        "Basler Preview",
        cfg["test_one_cam"]["frame_width"].as<int>(),
        cfg["test_one_cam"]["frame_height"].as<int>()
    );

    // 创建保存目录
    string save_dir = cfg["test_one_cam"]["save_dir"].as<std::string>();
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
                    current_filename = save_dir + "/record_" + string(buf) + ".avi";
                    cam.startRecording(current_filename, cfg["test_one_cam"]["fps"].as<double>());
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
