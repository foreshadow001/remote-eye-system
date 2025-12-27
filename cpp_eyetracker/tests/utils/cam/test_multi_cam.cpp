#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// 函数声明
int getNextCalibCounter(const std::string& save_dir);

// 辅助函数：获取当前时间字符串
std::string getCurrentTimeString() {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char buf[64];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
    return std::string(buf);
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Optimized) ===" << endl;

    Cfg cfg;
    
    // 1. 读取序列号列表
    std::vector<std::string> cam_serials = cfg["test_multi_cam"]["cam_indices"].as<std::vector<std::string>>();

    if (cam_serials.empty()) {
        cerr << "[MultiCam] No camera serials found in config.\n";
        return -1;
    }

    int num_cams = cam_serials.size();
    cout << "[MultiCam] Found " << num_cams << " cameras defined in config.\n";

    // 2. 初始化相机对象列表
    std::vector<std::unique_ptr<BaslerCamera>> cameras;
    
    // 读取通用参数
    double target_fps = cfg["test_multi_cam"]["fps"].as<double>(); // 目标帧率
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exposure = cfg["test_multi_cam"]["exposure_time"].as<double>();
    string save_dir = cfg["test_multi_cam"]["save_dir"].as<std::string>();

    // 创建保存目录
    try {
        if (!fs::exists(save_dir)) fs::create_directories(save_dir);
    } catch (const std::exception& e) {
        cerr << "[MultiCam] Failed to create directory: " << e.what() << endl;
        return -1;
    }

    // 启动所有相机
    for (const auto& sn : cam_serials) {
        auto cam = std::make_unique<BaslerCamera>(sn);
        if (cam->open()) {
            cam->setFrameRate(target_fps);
            cam->setGain(gain);
            cam->setGamma(gamma);
            cam->setExposureTime(exposure);
            cameras.push_back(std::move(cam));
        } else {
            cerr << "[MultiCam] ⚠️ Failed to open camera SN: " << sn << ". Skipping.\n";
        }
    }

    if (cameras.empty()) {
        cerr << "[MultiCam] ❌ No cameras available. Exiting.\n";
        return -1;
    }

    num_cams = cameras.size();

    // 3. 计算布局 (智能网格)
    const int WIN_WIDTH = cfg["test_multi_cam"]["window_width"].as<int>();
    const int WIN_HEIGHT = cfg["test_multi_cam"]["window_height"].as<int>();
    
    int cols = (int)std::ceil(std::sqrt(num_cams));
    int rows = (int)std::ceil((double)num_cams / cols);
    int cell_w = WIN_WIDTH / cols;
    int cell_h = WIN_HEIGHT / rows;

    cv::namedWindow("Multi-Camera Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Camera Preview", WIN_WIDTH, WIN_HEIGHT);

    cout << "[MultiCam] Layout: " << rows << "x" << cols << " grid." << endl;
    cout << "[Controls] SPACE: Snapshot | 'r': Start Record | 's': Stop Record | 'q': Quit" << endl;

    bool recording = false;
    bool running = true;
    int calib_counter = 0;

    // --- 性能优化变量 ---
    // 关键设置：每隔 20 帧才刷新一次界面
    // 如果你在 200FPS 运行，界面每 0.1s 刷新一次，非常流畅且节省大量 CPU
    const int DISPLAY_INTERVAL = 40; 
    long long frame_loop_count = 0;

    // FPS 统计
    auto last_time = std::chrono::high_resolution_clock::now();
    int fps_counter = 0;

    // 主循环
    while (running) {
        // 创建画布 (只在需要显示的时候才创建，这里为了逻辑简单，我们每次循环都声明，但只在显示帧操作)
        // 为了性能，我们将 canvas 的绘制逻辑放入 if 块中
        
        bool do_update_ui = (frame_loop_count % DISPLAY_INTERVAL == 0);
        cv::Mat canvas;
        if (do_update_ui) {
            canvas = cv::Mat::zeros(WIN_HEIGHT, WIN_WIDTH, CV_8UC3);
        }

        // 遍历所有相机
        for (int i = 0; i < num_cams; ++i) {
            // A. 获取原始帧 (必须每帧都做，最优先)
            cv::Mat frame = cameras[i]->grabFrame();
            
            // B. 处理录制 (必须每帧都做，第二优先)
            if (recording) {
                cameras[i]->writeFrame(frame);
            }

            // C. 准备显示 (仅在 UI 刷新帧进行)
            if (do_update_ui && !frame.empty()) {
                int r = i / cols;
                int c = i % cols;
                
                // 定义 ROI
                cv::Rect roi(c * cell_w, r * cell_h, cell_w, cell_h);
                
                // 缩放
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(cell_w, cell_h));
                
                // 边界保护
                if (roi.x + roi.width > canvas.cols) roi.width = canvas.cols - roi.x;
                if (roi.y + roi.height > canvas.rows) roi.height = canvas.rows - roi.y;
                
                resized_frame.copyTo(canvas(roi));

                // 绘制文字
                std::string label = "SN: " + cameras[i]->getSerialNumber();
                cv::putText(canvas(roi), label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4);
                cv::putText(canvas(roi), label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                
                if (recording) {
                     cv::circle(canvas(roi), cv::Point(canvas(roi).cols - 30, 30), 10, cv::Scalar(0, 0, 255), -1);
                }
            }
        }

        // D. 统计 FPS
        fps_counter++;
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = now - last_time;
        if (diff.count() >= 1.0) {
            // 在控制台刷新显示真实帧率
            std::cout << "\r[MultiCam] Real FPS: " << fps_counter 
                      << " | Target: " << target_fps 
                      << " | Rec: " << (recording ? "ON " : "OFF") << "   " << std::flush;
            fps_counter = 0;
            last_time = now;
        }

        // E. 界面刷新与按键响应 (低频执行)
        if (do_update_ui) {
            cv::imshow("Multi-Camera Preview", canvas);

            // 按键检测放在这里，减少阻塞
            char key = (char)cv::waitKey(1);
            
            // --- 拍照 (SPACE) ---
            if (key == ' ') {
                calib_counter = getNextCalibCounter(save_dir);
                string timeStr = getCurrentTimeString();
                cout << "\n[MultiCam] 📸 Snapshot at " << timeStr << endl;
                
                // 注意：这里为了逻辑简单，再次调用 grabFrame 可能会导致极短的卡顿。
                // 如果需要极其严格的同步，建议缓存上面的 frame 变量。
                // 这里沿用你的逻辑，重新抓一帧专门用于保存高质量图片。
                for (int i = 0; i < num_cams; ++i) {
                    cv::Mat shot = cameras[i]->grabFrame(); 
                    if (!shot.empty()) {
                        std::stringstream ss;
                        ss << std::setw(2) << std::setfill('0') << calib_counter;
                        std::string calib_str = ss.str();
                        std::string fn = save_dir + "/calib_cam_" + std::to_string(i) + "_" + calib_str + ".jpg";
                        cv::imwrite(fn, shot);
                        cout << "  -> Saved: " << fn << endl;
                    }
                }
            }
            
            // --- 开始录像 (r) ---
            else if (key == 'r' && !recording) {
                string timeStr = getCurrentTimeString();
                cout << "\n[MultiCam] 🎬 Start Recording..." << endl;
                for (int i = 0; i < num_cams; ++i) {
                    string fn = save_dir + "/video_" + timeStr + "_cam_" + cameras[i]->getSerialNumber(); 
                    // 【注意】这里传入的是 target_fps。
                    // 如果 Real FPS 只有 50，但这里传入 200，播放时就会快进。
                    // 如果你希望播放速度正常，可以将 target_fps 改为 fps_counter (上一秒的真实帧率)
                    cameras[i]->startRecording(fn, target_fps);
                }
                recording = true;
            }
            
            // --- 停止录像 (s) ---
            else if (key == 's' && recording) {
                cout << "\n[MultiCam] ⏹️ Stop Recording." << endl;
                for (auto& cam : cameras) {
                    cam->stopRecording();
                }
                recording = false;
            }
            
            // --- 退出 (q / ESC) ---
            else if (key == 'q' || key == 27) {
                running = false;
            }
        }

        frame_loop_count++;
    }

    // 清理资源
    if (recording) {
        for (auto& cam : cameras) cam->stopRecording();
    }
    
    for (auto& cam : cameras) {
        cam->close();
    }
    
    cv::destroyAllWindows();
    cout << "\n[MultiCam] Program ended.\n";
    return 0;
}

// 辅助函数实现保持不变
int getNextCalibCounter(const std::string& save_dir) {
    int max_counter = -1;
    if (!fs::exists(save_dir)) return 0; // 目录不存在则从0开始

    for (const auto& entry : fs::directory_iterator(save_dir)) {
        const auto& path = entry.path();
        if (path.extension() == ".jpg" && path.stem().string().find("calib_cam_") == 0) {
            std::string filename = path.stem().string();
            std::string prefix = "calib_cam_";
            size_t pos1 = filename.find(prefix);
            if (pos1 != std::string::npos) {
                std::string suffix = filename.substr(pos1 + prefix.length());
                size_t pos2 = suffix.find('_');
                if (pos2 != std::string::npos) {
                    std::string counter_str = suffix.substr(pos2 + 1);
                    try {
                        int counter = std::stoi(counter_str);
                        if (counter > max_counter) max_counter = counter;
                    } catch (...) { continue; }
                }
            }
        }
    }
    return max_counter + 1;
}