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
        cfg["test_one_cam"]["window_width"].as<int>(),
        cfg["test_one_cam"]["window_height"].as<int>()
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

    // ------------------ 性能统计变量 ------------------
    double t_grab = 0.0;
    double t_write = 0.0;
    double t_ui = 0.0;
    double t_waitkey = 0.0;
    double t_total = 0.0;
    int stat_frames = 0;
    auto stat_last = chrono::steady_clock::now();
    long long loop_count = 0;

    const int DISPLAY_INTERVAL = 20; 

    while (running) {
        auto t_loop_begin = chrono::steady_clock::now();

        // A. 抓图 (这是最耗时且必须优先保证的操作)
        cv::Mat frame;
        auto t_grab_begin = chrono::steady_clock::now();
        cam.grabFrame(frame);
        auto t_grab_end = chrono::steady_clock::now();
        t_grab += ms(t_grab_begin, t_grab_end);

        if (frame.empty()) {
            cerr << "[test_cam] Empty frame captured.\n";
            continue; 
        }

        // B. 录制 (如果开启)
        if (recording) {
            auto t_write_begin = chrono::steady_clock::now();
            cam.writeFrame(frame);
            auto t_write_end = chrono::steady_clock::now();
            t_write += ms(t_write_begin, t_write_end);
        }

        // C. 界面显示与按键 (降频处理)
        if (loop_count % DISPLAY_INTERVAL == 0) {
            auto t_ui_begin = chrono::steady_clock::now();
            cv::imshow("Basler Preview", frame);
            auto t_ui_end = chrono::steady_clock::now();
            t_ui += ms(t_ui_begin, t_ui_end);

            // 只有在刷新界面时才检测按键，这会引入微小的按键延迟(0.1s)，但为了FPS这是值得的
            auto t_waitkey_begin = chrono::steady_clock::now();
            char key = (char)cv::waitKey(1); 
            auto t_waitkey_end = chrono::steady_clock::now();
            t_waitkey += ms(t_waitkey_begin, t_waitkey_end);
            
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

        auto t_loop_end = chrono::steady_clock::now();
        t_total += ms(t_loop_begin, t_loop_end);

        stat_frames++;
        loop_count++;

        // -------- 每秒输出一次统计 --------
        auto now = chrono::steady_clock::now();
        if (chrono::duration<double>(now - stat_last).count() >= 1.0) {
            double inv = 1.0 / stat_frames;
            cout << fixed << setprecision(3)
                 << "\r[PROFILE]"
                 << " FPS=" << stat_frames
                 << " | grab=" << t_grab * inv << "ms"
                 << " | write=" << t_write * inv << "ms"
                 << " | ui=" << t_ui * inv << "ms"
                 << " | waitKey=" << t_waitkey * inv << "ms"
                 << " | total=" << t_total * inv << "ms"
                 << std::flush;

            t_grab = t_write = t_ui = t_waitkey = t_total = 0.0;
            stat_frames = 0;
            stat_last = now;
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