#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <vector>
#include <mutex>
#include <queue>
#include <atomic>
#include <iomanip>
#include <cmath>
#include <functional>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// ================== 辅助函数：获取 Calib 计数器 ==================
int getNextCalibCounter(const std::string& save_dir) {
    int max_counter = -1;
    if (!fs::exists(save_dir)) return 0;
    for (auto& e : fs::directory_iterator(save_dir)) {
        if (e.path().extension() == ".jpg") {
            try {
                string stem = e.path().stem().string();
                size_t last_underscore = stem.find_last_of('_');
                if (last_underscore != string::npos) {
                     string num_part = stem.substr(last_underscore + 1);
                     max_counter = max(max_counter, stoi(num_part));
                }
            } catch (...) {}
        }
    }
    return max_counter + 1;
}

// ================== 相机工作上下文 ==================
struct CameraContext {
    int index;
    string id;
    BaslerCamera* cam = nullptr;

    // 线程控制
    thread capture_thread;
    thread writer_thread;
    atomic<bool> running{true};
    atomic<bool> recording{false};

    // 数据交互
    cv::Mat latest_frame;
    mutex frame_mtx;

    // 录制相关 (Temp存储)
    string temp_dir;
    queue<pair<string, cv::Mat>> write_queue;
    mutex queue_mtx;
    condition_variable queue_cv;

    // 统计
    atomic<int> captured_frames{0};
    atomic<double> fps{0.0};
    atomic<double> grab_time_ms{0.0};
    atomic<double> write_time_ms{0.0};
    atomic<double> ui_time_ms{0.0};

    CameraContext(int idx, string cam_id) : index(idx), id(cam_id) {}
};

// ================== 全局变量 ==================
vector<shared_ptr<CameraContext>> cam_ctxs;

// ================== 写入线程逻辑 ==================
void writerWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        pair<string, cv::Mat> task;
        {
            unique_lock<mutex> lock(ctx->queue_mtx);
            ctx->queue_cv.wait(lock, [&] { return !ctx->write_queue.empty() || !ctx->running; });

            if (!ctx->running && ctx->write_queue.empty()) break;

            if (!ctx->write_queue.empty()) {
                task = ctx->write_queue.front();
                ctx->write_queue.pop();
            } else {
                continue;
            }
        }

        auto start = chrono::steady_clock::now();
        if (!task.second.empty()) {
            cv::imwrite(task.first, task.second);
        }
        auto end = chrono::steady_clock::now();
        ctx->write_time_ms.store(chrono::duration<double, std::milli>(end - start).count());
    }
}

// ================== 采集线程逻辑 ==================
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time) {
    ctx->cam = new BaslerCamera(ctx->id);
    if (!ctx->cam->open()) {
        cerr << "[Error] Cam " << ctx->index << " (" << ctx->id << ") failed to open.\n";
        return;
    }

    ctx->cam->setFrameRate(fps);
    ctx->cam->setGain(gain);
    ctx->cam->setGamma(gamma);
    ctx->cam->setExposureTime(exp_time);

    cout << "[Info] Cam " << ctx->index << " started.\n";

    int frame_seq = 0;
    auto last_fps_time = chrono::steady_clock::now();
    int frames_in_sec = 0;

    while (ctx->running) {
        auto grab_start = chrono::steady_clock::now();
        cv::Mat frame;
        ctx->cam->grabFrame(frame);
        auto grab_end = chrono::steady_clock::now();

        double grab_time = chrono::duration<double, std::milli>(grab_end - grab_start).count();
        ctx->grab_time_ms.store(grab_time);

        if (frame.empty()) continue;

        {
            lock_guard<mutex> lock(ctx->frame_mtx);
            frame.copyTo(ctx->latest_frame);
        }
        ctx->captured_frames++;
        frames_in_sec++;

        if (ctx->recording) {
            std::stringstream ss;
            ss << ctx->temp_dir << "/" << std::setw(6) << std::setfill('0') << frame_seq++ << ".jpg";

            auto write_start = chrono::steady_clock::now();
            {
                lock_guard<mutex> lock(ctx->queue_mtx);
                ctx->write_queue.push({ss.str(), frame.clone()});
            }
            ctx->queue_cv.notify_one();
            auto write_end = chrono::steady_clock::now();
            ctx->write_time_ms.store(chrono::duration<double, std::milli>(write_end - write_start).count());
        } else {
            frame_seq = 0;
        }

        auto now = chrono::steady_clock::now();
        if (chrono::duration<double>(now - last_fps_time).count() >= 1.0) {
            ctx->fps.store(frames_in_sec / chrono::duration<double>(now - last_fps_time).count());
            frames_in_sec = 0;
            last_fps_time = now;
        }
    }

    ctx->cam->close();
    delete ctx->cam;
}

// ================== 视频合成逻辑 ==================
void compileVideo(shared_ptr<CameraContext> ctx, string final_video_path, double fps,
                  std::atomic<int>& global_processed) {
    vector<string> images;
    try {
        for (const auto& entry : fs::directory_iterator(ctx->temp_dir)) {
            if (entry.path().extension() == ".jpg") {
                images.push_back(entry.path().string());
            }
        }
    } catch (...) {
        cerr << "[Error] Failed to read directory: " << ctx->temp_dir << endl;
        return;
    }

    sort(images.begin(), images.end());

    if (images.empty()) {
        cerr << "[Warning] No frames captured for Cam " << ctx->index << endl;
        return;
    }

    cv::Mat first = cv::imread(images[0]);
    if (first.empty()) {
        cerr << "[Error] Failed to read first image: " << images[0] << endl;
        return;
    }

    cv::VideoWriter writer(final_video_path, cv::VideoWriter::fourcc('M','J','P','G'), fps, first.size());
    if (!writer.isOpened()) {
        cerr << "[Error] Could not open video writer for " << final_video_path << endl;
        return;
    }

    for (size_t i = 0; i < images.size(); ++i) {
        cv::Mat img = cv::imread(images[i]);
        if (!img.empty()) {
            writer.write(img);
        }
        global_processed++;
    }

    writer.release();
    cout << "[Done] Saved " << final_video_path << " (" << images.size() << " frames)" << endl;
}

// ================== 主函数 ==================
int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Optimized) ===" << endl;
    Cfg cfg;

    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();
    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time = cfg["test_multi_cam"]["exposure_time"].as<double>();
    string save_base_dir = cfg["test_multi_cam"]["save_dir"].as<std::string>();
    int win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    int win_h = cfg["test_multi_cam"]["window_height"].as<int>();

    for (int i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(i, camera_ids[i]);
        cam_ctxs.push_back(ctx);
        ctx->writer_thread = thread(writerWorker, ctx);
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain, gamma, exp_time);
    }

    cv::namedWindow("Multi-Cam Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Cam Preview", win_w, win_h);

    cout << "Cameras initialized. Press 'r' to record, 's' to stop/save, 'space' to photo, 'q' to quit.\n";

    bool is_recording = false;
    bool running = true;
    string current_record_timestr;
    auto last_stats_time = chrono::steady_clock::now();

    while (running) {
        int n_cams = cam_ctxs.size();
        int grid_rows = (n_cams == 1) ? 1 : (n_cams <= 4 ? 2 : 3);
        int grid_cols = (n_cams == 1) ? 1 : (n_cams <= 4 ? 2 : 3);
        int cell_w = win_w / grid_cols;
        int cell_h = win_h / grid_rows;
        cv::Mat canvas = cv::Mat::zeros(win_h, win_w, CV_8UC3);
        int valid_rows = 0;

        auto ui_start = chrono::steady_clock::now();
        for (int i = 0; i < n_cams; ++i) {
            cv::Mat img;
            {
                lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);
                if (!cam_ctxs[i]->latest_frame.empty()) {
                    cv::resize(cam_ctxs[i]->latest_frame, img, cv::Size(cell_w, cell_h));
                }
            }
            if (!img.empty()) {
                if (img.type() == CV_8UC1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
                if (is_recording) {
                    int radius = 10;
                    cv::Point center(img.cols - radius * 2, radius * 2);
                    cv::circle(img, center, radius, cv::Scalar(0,0,255), -1, cv::LINE_AA);
                    cv::putText(img, "REC", cv::Point(img.cols - radius * 7, radius * 2.5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);
                }
                int r = i / grid_cols;
                int c = i % grid_cols;
                img.copyTo(canvas(cv::Rect(c*cell_w, r*cell_h, cell_w, cell_h)));
                valid_rows = max(valid_rows, r+1);
            }
        }
        auto ui_end = chrono::steady_clock::now();
        double ui_time = chrono::duration<double, std::milli>(ui_end - ui_start).count();
        for (auto& ctx : cam_ctxs) ctx->ui_time_ms.store(ui_time);

        for (int i = 0; i < n_cams; ++i) {
            auto& ctx = cam_ctxs[i];
            double total_ms = ctx->grab_time_ms.load() + ctx->ui_time_ms.load() + ctx->write_time_ms.load();
            stringstream ss_stats;
            ss_stats << fixed << setprecision(2)
                        << "FPS:" << ctx->fps.load()
                        << " T(ms):" << total_ms
                        << " G(ms):" << ctx->grab_time_ms.load()
                        << " U(ms):" << ctx->ui_time_ms.load()
                        << " W(ms):" << ctx->write_time_ms.load();

            int r = i / grid_cols;
            int c = i % grid_cols;
            cv::Mat roi = canvas(cv::Rect(c*cell_w, r*cell_h, cell_w, cell_h));
            cv::putText(roi, ss_stats.str(), cv::Point(5, cell_h-10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
        }

        if (valid_rows > 0 && valid_rows < grid_rows)
            cv::imshow("Multi-Cam Preview", canvas(cv::Rect(0,0,win_w, valid_rows*cell_h)));
        else
            cv::imshow("Multi-Cam Preview", canvas);

        char key = (char)cv::waitKey(20);
        if (key == 'q' || key == 27) running = false;
        else if (key == 'r') {
            if (!is_recording) {
                auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
                char buf[64];
                strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                current_record_timestr = string(buf);
                cout << "\n[Info] Start Recording Batch: " << current_record_timestr << endl;
                for (auto& ctx : cam_ctxs) {
                    string batch_temp = save_base_dir + "/temp_" + current_record_timestr + "/cam_" + to_string(ctx->index);
                    fs::create_directories(batch_temp);
                    ctx->temp_dir = batch_temp;
                    ctx->recording = true;
                }
                is_recording = true;
            }
        }
        else if (key == 's') {
            if (is_recording) {
                cout << "\n[Info] Stopping recording... waiting for buffers to flush." << endl;
                is_recording = false;
                
                // 1. 停止采集标志
                for (auto& ctx : cam_ctxs) ctx->recording = false;
                
                // 2. 等待队列写完
                cv::Mat wait_img = cv::Mat::zeros(400, 600, CV_8UC3);
                cv::putText(wait_img, "Flushing Write Queue...", cv::Point(50, 200),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                cv::imshow("Multi-Cam Preview", wait_img);
                cv::waitKey(1);
                
                for (auto& ctx : cam_ctxs) {
                    unique_lock<mutex> lk(ctx->queue_mtx);
                    ctx->queue_cv.wait(lk, [&]{ return ctx->write_queue.empty(); });
                }
                
                // 3. 计算总帧数 (用于进度条)
                int total_frames_all_cams = 0;
                for (auto& ctx : cam_ctxs) {
                    try {
                        for (auto& p : fs::directory_iterator(ctx->temp_dir)) {
                            if (p.path().extension() == ".jpg") total_frames_all_cams++;
                        }
                    } catch (...) {}
                }
                
                // 4. 创建原子计数器用于进度跟踪
                std::atomic<int> global_processed{0};
                
                // 5. 定义 UI 绘制函数 (Lambda)
                auto draw_progress_ui = [&]() {
                    cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                    
                    int processed = global_processed.load();
                    string text = "Processing Videos... " + to_string(processed) + "/" + to_string(total_frames_all_cams);
                    cv::putText(loading, text, cv::Point(50, 180),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                    
                    // 2. 进度条参数
                    int bar_x = 50, bar_y = 220;
                    int bar_w = 500, bar_h = 20;
                    float ratio = 0.0f;
                    if (total_frames_all_cams > 0) {
                        ratio = (float)processed / total_frames_all_cams;
                        if (ratio > 1.0f) ratio = 1.0f;
                    }
                    
                    // 3. 画空心矩形框 (边框)
                    cv::rectangle(loading, cv::Point(bar_x, bar_y),
                                cv::Point(bar_x + bar_w, bar_y + bar_h),
                                cv::Scalar(255, 255, 255), 1);
                    
                    // 4. 画实心矩形 (进度) - 绿色
                    if (ratio > 0) {
                        cv::rectangle(loading, cv::Point(bar_x, bar_y),
                                    cv::Point(bar_x + (int)(bar_w * ratio), bar_y + bar_h),
                                    cv::Scalar(0, 255, 0), -1);
                    }
                    
                    // 5. 显示并刷新
                    cv::imshow("Multi-Cam Preview", loading);
                    cv::waitKey(1);
                };
                
                // 6. 并行启动所有视频合成线程
                vector<thread> compile_threads;
                for (auto& ctx : cam_ctxs) {
                    string video_fn = save_base_dir + "/record_" + current_record_timestr + "_cam_" + to_string(ctx->index) + ".avi";
                    
                    compile_threads.emplace_back(
                        compileVideo, ctx, video_fn, target_fps,
                        std::ref(global_processed)
                    );
                }
                
                // 7. 主线程更新进度条，直到所有合成完成
                // 使用一个更简单的方法：检查进度是否达到总帧数
                int last_processed = 0;
                int same_count = 0;  // 用于检测进度是否停滞
                
                while (true) {
                    // 绘制进度条
                    draw_progress_ui();
                    
                    int current_processed = global_processed.load();
                    
                    // 如果进度达到总帧数，跳出循环
                    if (current_processed >= total_frames_all_cams) {
                        break;
                    }
                    
                    // 检测进度是否停滞（连续5次检查进度不变）
                    if (current_processed == last_processed) {
                        same_count++;
                        if (same_count > 10) {  // 10*50ms = 0.5秒无进展，认为完成
                            cout << "[Info] Progress appears to have stalled, assuming completion." << endl;
                            break;
                        }
                    } else {
                        same_count = 0;
                        last_processed = current_processed;
                    }
                    
                    // 短暂休眠
                    this_thread::sleep_for(chrono::milliseconds(50));
                }
                
                // 8. 等待所有线程正式结束
                for (auto& t : compile_threads) {
                    if (t.joinable()) {
                        t.join();
                    }
                }
                
                // 9. 确保显示100%
                global_processed = total_frames_all_cams;  // 强制设为100%
                draw_progress_ui();
                
                // 10. 清理 Temp 目录
                string batch_temp_root = save_base_dir + "/temp_" + current_record_timestr;
                try {
                    fs::remove_all(batch_temp_root);
                    cout << "[Info] Cleaned up temp directory: " << batch_temp_root << endl;
                } catch (exception& e) {
                    cerr << "[Error] Failed to delete temp dir: " << e.what() << endl;
                }
                
                // 恢复正常画面前的短暂延时
                cv::waitKey(500);
            }
        }
        else if (key == ' ') {
            int counter = getNextCalibCounter(save_base_dir);
            stringstream ss; ss << setw(2) << setfill('0') << counter; string calib_str = ss.str();
            cout << "\n[Photo] Capturing calibration set " << calib_str << endl;
            for (auto& ctx : cam_ctxs) {
                cv::Mat snapshot;
                { lock_guard<mutex> lock(ctx->frame_mtx); snapshot = ctx->latest_frame.clone(); }
                if (!snapshot.empty()) {
                    string fn = save_base_dir + "/calib_cam_" + to_string(ctx->index) + "_" + calib_str + ".jpg";
                    cv::imwrite(fn, snapshot);
                    cout << "  -> Saved " << fn << endl;
                }
            }
        }
    }

    cout << "[System] Shutting down threads..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->queue_cv.notify_all();
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->writer_thread.joinable()) ctx->writer_thread.join();
    }
    cv::destroyAllWindows();
    return 0;
}
