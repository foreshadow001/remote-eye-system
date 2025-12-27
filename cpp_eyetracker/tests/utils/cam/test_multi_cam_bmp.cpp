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
#include <queue>
#include <mutex>
#include <condition_variable>
#include <future>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// --- 异步写盘管理类 ---
class AsyncFileWriter {
public:
    AsyncFileWriter() : stop(false) {
        worker = std::thread(&AsyncFileWriter::processQueue, this);
    }

    ~AsyncFileWriter() {
        {
            std::lock_guard<std::mutex> lock(mtx);
            stop = true;
        }
        cv_var.notify_all();
        if (worker.joinable()) worker.join();
    }

    // 异步保存图片
    void saveImage(const std::string& path, const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mtx);
        tasks.push({path, frame.clone(), TaskType::IMAGE});
        cv_var.notify_one();
    }

    // 注意：视频录制通常需要序列化写入，建议在 BaslerCamera 类内部
    // 维护一个异步队列，或者这里仅处理单张静态图。
    // 为了代码简洁，此示例重点演示图片的异步保存。

private:
    enum class TaskType { IMAGE };
    struct WriteTask {
        std::string path;
        cv::Mat frame;
        TaskType type;
    };

    std::queue<WriteTask> tasks;
    std::mutex mtx;
    std::condition_variable cv_var;
    std::thread worker;
    bool stop;

    void processQueue() {
        while (true) {
            WriteTask task;
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv_var.wait(lock, [this] { return stop || !tasks.empty(); });
                if (stop && tasks.empty()) break;
                task = std::move(tasks.front());
                tasks.pop();
            }
            if (!task.frame.empty()) {
                cv::imwrite(task.path, task.frame);
            }
        }
    }
};

// --- 辅助函数 ---
int getNextCalibCounter(const std::string& save_dir);
std::string getCurrentTimeString() {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char buf[64];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
    return std::string(buf);
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Async-Write) ===" << endl;

    Cfg cfg;
    std::vector<std::string> cam_serials = cfg["test_multi_cam"]["cam_indices"].as<std::vector<std::string>>();
    if (cam_serials.empty()) return -1;

    // 参数读取
    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exposure = cfg["test_multi_cam"]["exposure_time"].as<double>();
    string save_dir = cfg["test_multi_cam"]["save_dir"].as<std::string>();

    if (!fs::exists(save_dir)) fs::create_directories(save_dir);

    // 初始化相机与异步写处理器
    std::vector<std::unique_ptr<BaslerCamera>> cameras;
    AsyncFileWriter asyncWriter;

    for (const auto& sn : cam_serials) {
        auto cam = std::make_unique<BaslerCamera>(sn);
        if (cam->open()) {
            cam->setFrameRate(target_fps);
            cam->setGain(gain);
            cam->setGamma(gamma);
            cam->setExposureTime(exposure);
            cameras.push_back(std::move(cam));
        }
    }

    if (cameras.empty()) return -1;

    int num_cams = cameras.size();
    const int WIN_WIDTH = cfg["test_multi_cam"]["window_width"].as<int>();
    const int WIN_HEIGHT = cfg["test_multi_cam"]["window_height"].as<int>();
    
    int cols = (int)std::ceil(std::sqrt(num_cams));
    int rows = (int)std::ceil((double)num_cams / cols);
    int cell_w = WIN_WIDTH / cols;
    int cell_h = WIN_HEIGHT / rows;

    cv::namedWindow("Multi-Camera Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Camera Preview", WIN_WIDTH, WIN_HEIGHT);

    bool recording = false;
    bool running = true;
    const int DISPLAY_INTERVAL = 40; 
    long long frame_loop_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();
    int fps_counter = 0;

    // --- 主循环 ---
    while (running) {
        bool do_update_ui = (frame_loop_count % DISPLAY_INTERVAL == 0);
        cv::Mat canvas;
        if (do_update_ui) canvas = cv::Mat::zeros(WIN_HEIGHT, WIN_WIDTH, CV_8UC3);

        for (int i = 0; i < num_cams; ++i) {
            // 1. 同步抓图（保证每一帧都处理到）
            cv::Mat frame = cameras[i]->grabFrame();
            if (frame.empty()) continue;

            // 2. 视频录制（BaslerCamera::writeFrame 内部若支持异步则更好）
            if (recording) {
                cameras[i]->writeFrame(frame);
            }

            // 3. UI 渲染
            if (do_update_ui) {
                int r = i / cols;
                int c = i % cols;
                cv::Rect roi(c * cell_w, r * cell_h, cell_w, cell_h);
                
                cv::Mat resized;
                cv::resize(frame, resized, cv::Size(cell_w, cell_h));
                resized.copyTo(canvas(roi));

                string label = "SN: " + cameras[i]->getSerialNumber();
                cv::putText(canvas(roi), label, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                if (recording) cv::circle(canvas(roi), cv::Point(cell_w - 20, 20), 8, cv::Scalar(0, 0, 255), -1);
            }
        }

        // 4. FPS 统计
        fps_counter++;
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - last_time).count() >= 1.0) {
            std::cout << "\r[MultiCam] FPS: " << fps_counter << " | Rec: " << (recording ? "ON" : "OFF") << std::flush;
            fps_counter = 0;
            last_time = now;
        }

        // 5. 事件处理
        if (do_update_ui) {
            cv::imshow("Multi-Camera Preview", canvas);
            char key = (char)cv::waitKey(1);

            if (key == ' ') { // 异步截图
                int calib_id = getNextCalibCounter(save_dir);
                for (int i = 0; i < num_cams; ++i) {
                    cv::Mat shot = cameras[i]->grabFrame(); // 再次抓取当前高质量帧
                    std::stringstream ss;
                    ss << save_dir << "/calib_cam_" << i << "_" << std::setw(2) << std::setfill('0') << calib_id << ".jpg";
                    asyncWriter.saveImage(ss.str(), shot); // 扔进后台队列，主线程立即继续
                }
                cout << "\n[System] Async Snapshots Triggered." << endl;
            } 
            else if (key == 'r' && !recording) {
                string ts = getCurrentTimeString();
                for (int i = 0; i < num_cams; ++i) {
                    string fn = save_dir + "/video_" + ts + "_cam_" + cameras[i]->getSerialNumber();
                    cameras[i]->startRecording(fn, target_fps);
                }
                recording = true;
            } 
            else if (key == 's' && recording) {
                for (auto& cam : cameras) cam->stopRecording();
                recording = false;
                cout << "\n[System] Recording Stopped." << endl;
            } 
            else if (key == 'q' || key == 27) running = false;
        }
        frame_loop_count++;
    }

    // 清理
    cv::destroyAllWindows();
    return 0;
}

// 辅助函数：扫描目录获取最新编号
int getNextCalibCounter(const std::string& save_dir) {
    int max_counter = -1;
    if (!fs::exists(save_dir)) return 0;
    for (const auto& entry : fs::directory_iterator(save_dir)) {
        std::string filename = entry.path().stem().string();
        if (filename.find("calib_cam_") == 0) {
            size_t last_underscore = filename.find_last_of('_');
            if (last_underscore != std::string::npos) {
                try {
                    int c = std::stoi(filename.substr(last_underscore + 1));
                    if (c > max_counter) max_counter = c;
                } catch (...) {}
            }
        }
    }
    return max_counter + 1;
}