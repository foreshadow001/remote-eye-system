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

int getNextCalibCounter(const std::string& save_dir);

// 辅助函数：获取当前时间字符串
std::string getCurrentTimeString() {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char buf[64];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
    return std::string(buf);
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool ===" << endl;

    Cfg cfg;
    
    // 1. 读取序列号列表 (假设配置中存储为 int 数组，如果是 string 数组请相应修改)
    // 这里兼容处理：先读成 int，转 string。如果你的 config 库直接支持 string vector 更好。
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
    double fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exposure = cfg["test_multi_cam"]["exposure_time"].as<double>();
    string save_dir = cfg["test_multi_cam"]["save_dir"].as<std::string>();

    // 创建保存目录
    try {
        fs::create_directories(save_dir);
    } catch (const std::exception& e) {
        cerr << "[MultiCam] Failed to create directory: " << e.what() << endl;
        return -1;
    }

    // 启动所有相机
    for (const auto& sn : cam_serials) {
        auto cam = std::make_unique<BaslerCamera>(sn);
        if (cam->open()) {
            cam->setFrameRate(fps);
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

    // 更新实际可用的相机数量
    num_cams = cameras.size();

    // 3. 计算布局 (智能网格)
    // 目标总窗口大小
    const int WIN_WIDTH = cfg["test_multi_cam"]["window_width"].as<int>();
    const int WIN_HEIGHT = cfg["test_multi_cam"]["window_height"].as<int>();
    
    // 计算行列数: ceil(sqrt(N))
    int cols = (int)std::ceil(std::sqrt(num_cams));
    int rows = (int)std::ceil((double)num_cams / cols);
    
    // 计算每个子画面的大小
    int cell_w = WIN_WIDTH / cols;
    int cell_h = WIN_HEIGHT / rows;

    cv::namedWindow("Multi-Camera Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Camera Preview", WIN_WIDTH, WIN_HEIGHT);

    cout << "[MultiCam] Layout: " << rows << "x" << cols << " grid." << endl;
    cout << "[Controls] SPACE: Snapshot | 'r': Start Record | 's': Stop Record | 'q': Quit" << endl;

    bool recording = false;
    bool running = true;

    int calib_counter = 0;

    // 主循环
    while (running) {
        // 创建画布 (黑色背景)
        cv::Mat canvas = cv::Mat::zeros(WIN_HEIGHT, WIN_WIDTH, CV_8UC3);
        
        // 遍历所有相机
        for (int i = 0; i < num_cams; ++i) {
            // 1. 获取原始帧
            cv::Mat frame = cameras[i]->grabFrame();
            
            // 2. 处理录制 (使用原始帧，保证录像质量不被缩放影响)
            if (recording) {
                cameras[i]->writeFrame(frame);
            }

            // 3. 准备显示
            if (!frame.empty()) {
                // 计算当前相机在网格中的位置
                int r = i / cols;
                int c = i % cols;
                
                // 定义画布上的 ROI (Region of Interest)
                cv::Rect roi(c * cell_w, r * cell_h, cell_w, cell_h);
                
                // 缩放并拷贝到画布
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(cell_w, cell_h));
                
                // 防止最后一行可能出现的像素越界 (简单保护)
                if (roi.x + roi.width > canvas.cols) roi.width = canvas.cols - roi.x;
                if (roi.y + roi.height > canvas.rows) roi.height = canvas.rows - roi.y;
                
                resized_frame.copyTo(canvas(roi));

                // 绘制序列号文字 (左上角)
                std::string label = "SN: " + cameras[i]->getSerialNumber();
                // 黑色描边，白色字体，确保可见
                cv::putText(canvas(roi), label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 4);
                cv::putText(canvas(roi), label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                
                // 如果正在录制，加个红点提示
                if (recording) {
                     cv::circle(canvas(roi), cv::Point(canvas(roi).cols - 30, 30), 10, cv::Scalar(0, 0, 255), -1);
                }
            }
        }

        cv::imshow("Multi-Camera Preview", canvas);

        // 按键处理
        char key = (char)cv::waitKey(1);
        
        // --- 拍照 (SPACE) ---
        if (key == ' ') {
            calib_counter = getNextCalibCounter(save_dir);
            string timeStr = getCurrentTimeString();
            cout << "[MultiCam] 📸 Snapshot at " << timeStr << endl;
            for (int i = 0; i < num_cams; ++i) {
                // 为了获得高质量图片，我们这里重新 grab 一帧，或者在循环里缓存上一帧均可。
                // 简单起见，我们利用Basler对象的buffer机制，或者在显示逻辑里缓存。
                // 但为了代码简洁，我们在 grabFrame 时并没有保存原始大图到 vector。
                // 修正策略：在上面的循环中，我们已经 grab 过了。因为 grab 是破坏性的（流式），
                // 我们无法再次 grab 同一时刻。
                // 实际工程中应该把上面循环里的 frame 存到一个 vector<Mat> 中，然后再画图。
                // 下面为了演示逻辑，我们做一次“同步触发抓拍”（虽然会有微小延时，但逻辑最简单）
                // 注意：这会导致预览卡顿一帧。
                cv::Mat shot = cameras[i]->grabFrame(); 
                if (!shot.empty()) {
                    // 创建一个stringstream来格式化calib_counter
                    std::stringstream ss;
                    ss << std::setw(2) << std::setfill('0') << calib_counter;
                    std::string calib_str = ss.str();  // 将格式化后的结果转换为字符串

                    // 拼接字符串
                    std::string fn = save_dir + "/calib_cam_" + std::to_string(i) + "_" + calib_str + ".jpg";
                    cv::imwrite(fn, shot);
                    cout << "  -> Saved: " << fn << endl;
                }
            }
        }
        
        // --- 开始录像 (r) ---
        else if (key == 'r' && !recording) {
            string timeStr = getCurrentTimeString();
            cout << "[MultiCam] 🎬 Start Recording..." << endl;
            for (int i = 0; i < num_cams; ++i) {
                string fn = save_dir + "/video_" + timeStr + "_cam_" + cameras[i]->getSerialNumber(); 
                // 后缀 .avi 会在 startRecording 内部自动添加或检查
                cameras[i]->startRecording(fn, fps);
            }
            recording = true;
        }
        
        // --- 停止录像 (s) ---
        else if (key == 's' && recording) {
            cout << "[MultiCam] ⏹️ Stop Recording." << endl;
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

    // 清理资源
    if (recording) {
        for (auto& cam : cameras) cam->stopRecording();
    }
    
    for (auto& cam : cameras) {
        cam->close();
    }
    
    cv::destroyAllWindows();
    cout << "[MultiCam] Program ended.\n";
    return 0;
}

int getNextCalibCounter(const std::string& save_dir) {
    int max_counter = -1;  // 如果没有找到任何文件，计数器应该从 0 开始

    // 遍历文件夹中的所有文件
    for (const auto& entry : fs::directory_iterator(save_dir)) {
        const auto& path = entry.path();
        
        // 只处理以 "calib_cam_" 开头且以 ".jpg" 结尾的文件
        if (path.extension() == ".jpg" && path.stem().string().find("calib_cam_") == 0) {
            std::string filename = path.stem().string();  // 获取文件名，不包含扩展名

            // 使用正则表达式从文件名中提取计数值
            std::string prefix = "calib_cam_";
            size_t pos1 = filename.find(prefix);
            if (pos1 != std::string::npos) {
                std::string suffix = filename.substr(pos1 + prefix.length());  // 获取计数部分
                
                // 提取计数的数字部分
                size_t pos2 = suffix.find('_');
                if (pos2 != std::string::npos) {
                    std::string counter_str = suffix.substr(pos2 + 1);  // 获取后缀部分
                    try {
                        int counter = std::stoi(counter_str);  // 转换为整数
                        if (counter > max_counter) {
                            max_counter = counter;
                        }
                    } catch (const std::invalid_argument&) {
                        // 如果提取过程中出错，则跳过该文件
                        continue;
                    }
                }
            }
        }
    }

    return max_counter + 1;  // 返回下一个计数值
}