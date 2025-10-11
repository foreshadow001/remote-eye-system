#include "cam/basler.hpp"
#include "set_image/create_image.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <thread>

namespace fs = std::filesystem;
using namespace std;
using namespace cv;
using namespace gazeestimation;

// --- 主程序 ---
int main() {
    cout << "=== [TEST] Calibration Image Display & Recording ===" << endl;

    string image_dir = "D:/users/projects/new_dataset/calibration_images";
    string save_dir  = "D:/users/projects/new_dataset/calib_records";

    try {
        fs::create_directories(save_dir);
    } catch (const std::exception& e) {
        cerr << "[test_record_calib] Failed to create save directory: " << e.what() << endl;
        return -1;
    }

    // 初始化相机
    BaslerCamera cam;
    if (!cam.open()) {
        cerr << "[test_record_calib] Failed to open Basler camera." << endl;
        return -1;
    }
    cam.setFrameRate(50.0);
    cam.setGain(3.0);
    cam.setGamma(1.1);
    cam.setExposureTime(20000.0);

    // 初始化标定图片
    int index = 0;
    string bg = "dark";
    cv::Mat calib_img;
    cv::Point2f pog;
    if (!loadCalibrationImage(index, bg, image_dir, calib_img, pog, false)) {
        cerr << "[test_record_calib] Failed to load first image." << endl;
        return -1;
    }

    // 全屏显示第一次标定图
    showImageFullscreenCapture("Calibration", calib_img);

    // 录制控制变量
    bool recording = false;
    bool running = true;
    string current_video_path;

    cout << "[test_record_calib] Controls: n=next, r=start rec, s=stop, space=snapshot, q=quit\n";

    while (running) {
        // 获取相机帧
        cv::Mat frame = cam.grabFrame();
        if (frame.empty()) {
            this_thread::sleep_for(chrono::milliseconds(5));
            continue;
        }

        // 在左上角显示提示文字（当前图编号 / PoG）
        cv::Mat display_frame = frame.clone();
        cv::putText(display_frame, 
                    "Calib #" + to_string(index) + 
                    "  (" + to_string((int)pog.x) + "," + to_string((int)pog.y) + ")", 
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 
                    1.0, cv::Scalar(0, 255, 0), 2);

        // 显示预览窗口
        cv::imshow("Basler Preview", display_frame);

        // 非阻塞按键检测
        char key = (char)cv::waitKey(1);
        switch (key) {
            case 'q':
            case 'Q':
            case 27: // ESC
                running = false;
                break;

            case 'n':
            case 'N': {
                index++;
                if (!loadCalibrationImage(index, bg, image_dir, calib_img, pog, false)) {
                    cout << "[test_record_calib] No more images. Restart.\n";
                    index = 0;
                    loadCalibrationImage(index, bg, image_dir, calib_img, pog, false);
                }
                showImageFullscreenCapture("Calibration", calib_img);
                cout << "[test_record_calib] Switched to image " << index 
                     << " (PoG=" << pog.x << "," << pog.y << ")\n";
                break;
            }

            case 'r':
            case 'R':
                if (!recording) {
                    char buf[256];
                    snprintf(buf, sizeof(buf), "%s/video_%02d_%d_%d.avi",
                             save_dir.c_str(), index, (int)pog.x, (int)pog.y);
                    current_video_path = buf;
                    cout << "[test_record_calib] Start recording -> " << current_video_path << endl;
                    cam.startRecording(current_video_path, 50.0);
                    recording = true;
                }
                break;

            case 's':
            case 'S':
                if (recording) {
                    cam.stopRecording();
                    recording = false;
                    cout << "[test_record_calib] Video saved -> " << current_video_path << endl;
                }
                break;

            case ' ':
                {
                    char buf[256];
                    snprintf(buf, sizeof(buf), "%s/image_%02d_%d_%d.jpg",
                             save_dir.c_str(), index, (int)pog.x, (int)pog.y);
                    string photo_path = buf;
                    cv::imwrite(photo_path, frame);
                    cout << "[test_record_calib] Snapshot saved -> " << photo_path << endl;
                }
                break;

            default:
                break;
        }

        // 写入视频帧（如果正在录制）
        if (recording) {
            cam.writeFrame(frame);
        }
    }

    // 清理资源
    if (recording)
        cam.stopRecording();

    cam.close();
    cv::destroyAllWindows();
    cout << "[test_record_calib] Finished.\n";
    return 0;
}
