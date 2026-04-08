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
#include <atomic>
#include <fstream>
#include <pylon/PylonIncludes.h>

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
                // 文件名格式约定: calib_cam_X_YY.jpg
                // 需要解析最后的 YY。简单起见，假设文件名结构固定，或者查找最后一个 '_'
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

enum class CamStatus {
    INIT,
    OPENED,
    WAITING_TRIGGER,
    STREAMING,
    ERROR_
};

// ================== 辅助结构 ==================
struct FrameTask {
    string filename;
    cv::Mat image;
    FrameMeta meta; // 携带元数据
};

struct LogEntry {
    string filename;
    int64_t blockID;
    int64_t timestamp;
    int width;   // 新增：图像宽度
    int height;  // 新增：图像高度
};

// ================== 相机工作上下文 ==================
struct CameraContext {
    int index;
    string id;
    BaslerCamera cam{""};
    
    // 线程控制
    thread capture_thread;
    thread writer_thread;
    atomic<bool> running{true};
    atomic<bool> recording{false};

    // 数据交互
    cv::Mat latest_frame;
    FrameMeta latest_meta;
    mutex frame_mtx; 
    
    // 录制相关 (Temp存储)
    string temp_dir;      // 用于存 .raw
    string log_file_path;
    ofstream log_stream;

    queue<FrameTask> write_queue; 
    mutex queue_mtx;
    atomic<bool> is_writing{false};
    condition_variable queue_cv;
    
    atomic<int> captured_frames{0};
    atomic<int> recorded_frames{0}; // <--- 新增：专门记录当前录制期间保存的帧数

    atomic<CamStatus> status{CamStatus::INIT};
    string status_msg = "Cam is initializing";

    int64_t frame_offset = 0;
    bool offset_initialized = false;
    static atomic<int64_t> master_first_id; // 静态变量，记录第一台准备好的相机的ID
    static atomic<bool> master_set;

    // 新增：用于写入的追踪
    int64_t last_recorded_aligned_id = -1;

    CameraContext(int idx, string cam_id) : index(idx), id(cam_id), cam(cam_id) {}
};

atomic<int64_t> CameraContext::master_first_id(-1);
atomic<bool> CameraContext::master_set(false);

// ================== 全局变量 ==================
vector<shared_ptr<CameraContext>> cam_ctxs;

// ================== 写入线程逻辑 ==================
void writerWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        FrameTask task;
        {
            unique_lock<mutex> lock(ctx->queue_mtx);
            ctx->queue_cv.wait(lock, [&] { return !ctx->write_queue.empty() || !ctx->running; });
            if (!ctx->running && ctx->write_queue.empty()) break;

            task = ctx->write_queue.front();
            ctx->write_queue.pop();
            ctx->is_writing = true; // <--- 标记：我已经把任务拿出来，正在干活！
        }

        if (!task.image.empty()) {
            std::ofstream out_raw(task.filename, std::ios::binary);
            if (out_raw) {
                out_raw.write(reinterpret_cast<const char*>(task.image.data), task.image.total() * task.image.elemSize());
            }
        }

        if (ctx->log_stream.is_open()) {
            int64_t aligned_id = task.meta.blockID - ctx->frame_offset;
            ctx->log_stream << fs::path(task.filename).filename().string() << "," 
                            << aligned_id << "," << task.meta.timestamp << ","
                            << task.meta.blockID << "," << task.image.cols << "," << task.image.rows << "\n";
        }

        // <--- 干完活了，解除标记，如果队列空了则通知主线程
        ctx->is_writing = false; 
        if (ctx->write_queue.empty()) {
            ctx->queue_cv.notify_all();
        }
    }
}

// ================== 采集线程逻辑 (修改为回调驱动) ==================
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time, bool use_hw_trigger) {
    TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
    
    if (!ctx->cam.open(mode)) {
        ctx->status = CamStatus::ERROR_;
        ctx->status_msg = "OPEN FAILED";
        return;
    }

    cout << "[Step 2] Setting Params Cam " << ctx->index << "..." << endl;
    try {
        if (!use_hw_trigger) ctx->cam.setFrameRate(fps);  
        ctx->cam.setGain(gain);                           
        ctx->cam.setGamma(gamma);                         
        ctx->cam.setExposureTime(exp_time);               
    } catch (...) {}

    // 独立计数器，给 lambda 回调使用
    struct GrabState {
        int frame_seq = 0;
        int64_t frame_counter = 0;
    };
    auto state = make_shared<GrabState>();

    // ============ 设置回调函数 ============
    ctx->cam.setFrameCallback([ctx, state](const cv::Mat& frame, FrameMeta meta) {
        state->frame_counter++;
        ctx->status = CamStatus::STREAMING;

        if (!ctx->offset_initialized && state->frame_counter > 1) {
            if (!CameraContext::master_set.exchange(true)) {
                CameraContext::master_first_id = meta.blockID;
                ctx->frame_offset = 0;
                ctx->offset_initialized = true;
                cout << "[Sync] Cam " << ctx->index << " is Master. Base ID: " << meta.blockID << endl;
            } else {
                int retry = 0;
                while (CameraContext::master_first_id == -1 && retry < 100) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    retry++;
                }
                if (CameraContext::master_first_id != -1) {
                    ctx->frame_offset = meta.blockID - CameraContext::master_first_id.load();
                    ctx->offset_initialized = true;
                    cout << "[Sync] Cam " << ctx->index << " Linked. Offset: " << ctx->frame_offset << endl;
                }
            }
        }

        if (ctx->offset_initialized) {
            meta.blockID = meta.blockID - ctx->frame_offset; 
            
            lock_guard<mutex> lock(ctx->frame_mtx);
            frame.copyTo(ctx->latest_frame); 
            ctx->latest_meta = meta;
            ctx->captured_frames++;
        }

        if (ctx->recording) {
            ctx->recorded_frames = state->frame_seq; // <--- 新增：同步当前录制帧数给主线程

            std::stringstream ss;
            ss << ctx->temp_dir << "/" << std::setw(6) << std::setfill('0') << state->frame_seq++ << ".raw";
            
            {
                lock_guard<mutex> lock(ctx->queue_mtx);
                if (ctx->write_queue.size() < 800) {
                    ctx->write_queue.push({ss.str(), frame.clone(), meta});
                } else {
                    // 硬盘写入太慢导致积压，主动丢帧保全系统
                    cout << "[Warning] Cam " << ctx->index << " Write Queue FULL! Dropping frame." << endl;
                }
            }
            ctx->queue_cv.notify_one();
        } else {
            state->frame_seq = 0; 
            ctx->recorded_frames = 0; // <--- 新增：非录制状态清零
        }
    });
    // =====================================

    cout << "[Step 3] Starting Grab Stream (Callback Mode) Cam " << ctx->index << "..." << endl;
    if (!ctx->cam.start()) {
        cerr << "[Error] Cam " << ctx->index << " failed to start grabbing." << endl;
        return;
    }

    ctx->status = CamStatus::OPENED;
    ctx->status_msg = use_hw_trigger ? "HW WAITING" : "STREAMING";

    cout << "[Step 4] Entering Wait Loop Cam " << ctx->index << endl;

    // 因为采集由 Pylon 后台线程负责，所以我们的主挂起线程只需要保持存活即可。
    while (ctx->running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 清理
    ctx->cam.close();

    cout << "[End] Thread " << ctx->index << " exited." << endl;
}

// ================== 同步与图片保存逻辑 ==================

vector<LogEntry> parseLogFile(const string& log_path) {
    vector<LogEntry> entries;
    ifstream infile(log_path);
    string line;
    while (getline(infile, line)) {
        stringstream ss(line);
        string segment;
        vector<string> parts;
        while (getline(ss, segment, ',')) parts.push_back(segment);
        
        // 现在有 6 个字段了
        if (parts.size() >= 6) {
            LogEntry entry;
            entry.filename = parts[0];
            entry.blockID = stoll(parts[1]);
            entry.timestamp = stoll(parts[2]);
            entry.width = stoi(parts[4]);
            entry.height = stoi(parts[5]);
            entries.push_back(entry);
        }
    }
    return entries;
}

// ============== RAW 转换存 JPG 逻辑 ==============
void convertRawToJpgWorker(string temp_raw_dir, string out_jpg_dir, vector<LogEntry> valid_entries, atomic<int>& global_processed) {
    if (valid_entries.empty()) return;
    
    // 创建 JPG 目标目录
    fs::create_directories(out_jpg_dir);

    for (const auto& entry : valid_entries) {
        string raw_path = temp_raw_dir + "/" + entry.filename;
        cv::Mat raw_img(entry.height, entry.width, CV_8UC1);
        ifstream in_raw(raw_path, ios::binary);
        if (in_raw) {
            in_raw.read(reinterpret_cast<char*>(raw_img.data), raw_img.total());
            in_raw.close();
            
            // 替换扩展名 .raw 为 .jpg
            string jpg_filename = entry.filename;
            size_t dot_pos = jpg_filename.find_last_of('.');
            if (dot_pos != string::npos) {
                jpg_filename = jpg_filename.substr(0, dot_pos) + ".jpg";
            } else {
                jpg_filename += ".jpg";
            }
            
            // 写入 JPG
            string jpg_path = out_jpg_dir + "/" + jpg_filename;
            cv::imwrite(jpg_path, raw_img);
        }
        global_processed++;
    }
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Optimized) ===" << endl;
    
    Cfg cfg;
    Pylon::PylonInitialize();
    
    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();

    bool use_hw_trigger = cfg["test_multi_cam"]["hardware_trigger"].as<bool>();
    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time = cfg["test_multi_cam"]["exposure_time"].as<double>();
    
    string save_base_dir = cfg["test_multi_cam"]["save_dir"].as<std::string>();
    int win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    int win_h = cfg["test_multi_cam"]["window_height"].as<int>();
    bool write_jpg = cfg["test_multi_cam"]["write_jpg"].as<bool>(); // <--- 改为 write_jpg
    bool debug_time  = cfg["test_multi_cam"]["debug_time"].as<bool>();

    std::filesystem::path save_folder_path(save_base_dir);
    std::filesystem::create_directories(save_folder_path);

    // --- 1. 启动线程 ---
    for (int i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(i, camera_ids[i]);
        cam_ctxs.push_back(ctx);

        // 启动写入线程
        ctx->writer_thread = thread(writerWorker, ctx);
        // 启动采集线程
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain, gamma, exp_time, use_hw_trigger);
    }

    // --- 2. 准备显示窗口 ---
    cv::namedWindow("Multi-Cam Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Cam Preview", win_w, win_h); // 初始大小，后面会自动根据内容调整

    cout << "Cameras initialized. Press 'r' to record, 's' to stop/save, 'space' to photo, 'q' to quit.\n";

    bool is_recording = false;
    bool running = true;
    string current_record_timestr;
    std::chrono::steady_clock::time_point record_start_time;

    while (running) {
        // --- A. 生成拼图 ---
        int n_cams = cam_ctxs.size();
        int grid_rows = 1, grid_cols = 1;
        
        if (n_cams == 1) { grid_rows = 1; grid_cols = 1; }
        else if (n_cams <= 4) { grid_rows = 2; grid_cols = 2; }
        else { grid_rows = 3; grid_cols = 3; }

        int cell_w = win_w / grid_cols;
        int cell_h = win_h / grid_rows;
        
        // 创建画布
        cv::Mat canvas = cv::Mat::zeros(win_h, win_w, CV_8UC3);
        int valid_rows = 0; // 记录实际使用了多少行

        for (int i = 0; i < n_cams; ++i) {
            cv::Mat img;
            cv::Mat local_raw; // 新增一个局部变量，用于接管锁内的数据

            // 1. 极速抢锁：做深拷贝并抓取状态
            {
                lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);
                if (!cam_ctxs[i]->latest_frame.empty()) {
                    local_raw = cam_ctxs[i]->latest_frame.clone(); 
                }
            } 

            // 2. 锁外处理：慢慢做 resize 和转色
            if (!local_raw.empty()) {
                cv::Mat small_raw;
                cv::resize(local_raw, small_raw, cv::Size(cell_w, cell_h));
                cv::cvtColor(small_raw, img, cv::COLOR_GRAY2BGR);
            }

            if (img.empty()) {
                img = cv::Mat::zeros(cell_h, cell_w, CV_8UC3);

                int baseline = 0;
                cv::Size textSize = cv::getTextSize(cam_ctxs[i]->status_msg, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
                cv::Point textOrg((cell_w - textSize.width) / 2, (cell_h + textSize.height) / 2);
                cv::putText(img, cam_ctxs[i]->status_msg, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            } else {
                if (img.type() == CV_8UC1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

                if (is_recording) {
                    // --- 右上角画红点 (原代码) ---
                    int radius = 10;
                    cv::Point center(img.cols - radius * 2, radius * 2);
                    cv::circle(img, center, radius, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                    cv::putText(img, "REC", cv::Point(img.cols - radius * 7, radius * 2.5), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                    // ==========================================
                    // --- 新增：左上角仅在录制时显示的相对帧数与时间 ---
                    // ==========================================
                    int current_frame = cam_ctxs[i]->recorded_frames.load();
                    // 计算经过的时间 (秒)
                    double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - record_start_time).count();
                    
                    // 格式化时间，保留一位小数
                    char time_buf[32];
                    snprintf(time_buf, sizeof(time_buf), "Time: %.1fs", elapsed_s);
                    
                    string text_frame = "Frame: " + to_string(current_frame);
                    string text_time = string(time_buf);
                    
                    // 画黑底绿字（先画粗黑边，再画细绿字，确保在强光或纯白背景下依然清晰可见）
                    cv::putText(img, text_frame, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
                    cv::putText(img, text_frame, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    
                    cv::putText(img, text_time, cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
                    cv::putText(img, text_time, cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                }
            }

            int r = i / grid_cols;
            int c = i % grid_cols;
            
            // 将带红点的图片拷贝到画布
            img.copyTo(canvas(cv::Rect(c * cell_w, r * cell_h, cell_w, cell_h)));
            
            if (r + 1 > valid_rows) valid_rows = r + 1;
        }

        // --- B. 裁剪空行 (规则2) ---
        // 如果计算出的 grid_rows 是2，但只画了第一行 (valid_rows=1)，则裁剪
        if (valid_rows > 0 && valid_rows < grid_rows) {
            cv::Mat cropped = canvas(cv::Rect(0, 0, win_w, valid_rows * cell_h));
            // 保持比例显示在窗口中，或者调整窗口大小
            cv::resizeWindow("Multi-Cam Preview", cropped.cols, cropped.rows);
            cv::imshow("Multi-Cam Preview", cropped);
        } else {
            cv::imshow("Multi-Cam Preview", canvas);
        }

        // --- C. 按键处理 ---
        char key = (char)cv::waitKey(50); // 20ms refresh

        if (key == 'q' || key == 27) {
            running = false;
        }
        else if (key == 'r') {
            if (!is_recording) {
                auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                char buf[64];
                strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                current_record_timestr = string(buf);

                // <--- 新增：记录开始录制的系统基准时间
                record_start_time = std::chrono::steady_clock::now(); 

                cout << "\n[Info] Start Recording Batch: " << current_record_timestr << endl;

                for (auto& ctx : cam_ctxs) {
                    // 创建 raw 文件夹
                    string batch_raw = save_base_dir + "/temp_raw_" + current_record_timestr + "/cam_" + to_string(ctx->index);
                    fs::create_directories(batch_raw);
                    ctx->temp_dir = batch_raw;
                    
                    ctx->log_file_path = save_base_dir + "/record_" + current_record_timestr + "_cam_" + to_string(ctx->index) + ".txt";
                    ctx->log_stream.open(ctx->log_file_path);
                    
                    ctx->recording = true; 
                }
                is_recording = true;
            }
        }
        else if (key == 's') {
            if (is_recording) {
                cout << "\n[Info] Stopping recording... waiting for queue flush." << endl;
                is_recording = false;

                // 1. 停止标志
                for (auto& ctx : cam_ctxs) ctx->recording = false;

                // 2. 显示等待提示
                cv::Mat wait_img = cv::Mat::zeros(400, 600, CV_8UC3);
                cv::putText(wait_img, "Flushing Write Queue...", cv::Point(50, 200),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                cv::imshow("Multi-Cam Preview", wait_img);
                cv::waitKey(1);

                // 3. 等待队列清空 & 关闭日志
                for (auto& ctx : cam_ctxs) {
                    {
                        unique_lock<mutex> lk(ctx->queue_mtx);
                        ctx->queue_cv.wait(lk, [&]{ return ctx->write_queue.empty() && !ctx->is_writing.load(); });
                    }
                    if (ctx->log_stream.is_open()) ctx->log_stream.close();
                }

                // =========================================================
                // 4. 解析日志并统计丢帧及实际物理帧率
                // =========================================================
                cout << "\n[Info] Calculating frame drop and actual FPS statistics..." << endl;
                vector<vector<LogEntry>> all_logs;
                
                for (int i = 0; i < cam_ctxs.size(); ++i) {
                    auto logs = parseLogFile(cam_ctxs[i]->log_file_path);
                    all_logs.push_back(logs);
                    
                    int total_saved = logs.size();
                    int dropped_frames = 0;
                    double actual_fps = 0.0;
                    double duration_s = 0.0;
                    
                    if (total_saved > 1) {
                        for (size_t k = 1; k < logs.size(); ++k) {
                            int64_t diff = logs[k].blockID - logs[k-1].blockID;
                            if (diff > 1) dropped_frames += (diff - 1);
                        }
                        
                        // Basler 的 timestamp 默认是纳秒 (ns) 级别的系统 tick
                        // 真实耗时 = 最后一帧时间戳 - 第一帧时间戳
                        int64_t duration_ns = logs.back().timestamp - logs.front().timestamp;
                        duration_s = static_cast<double>(duration_ns) / 10000000.0; 
                        
                        if (duration_s > 0) {
                            // 实际帧率 = (总帧数 - 1) / 总耗时
                            actual_fps = (total_saved - 1) / duration_s;
                        }
                    }
                    
                    // 打印详细红黑榜，带格式化对齐
                    char report_buf[256];
                    if (dropped_frames > 0) {
                        snprintf(report_buf, sizeof(report_buf), 
                                 "[Warning] Cam %d | Saved: %4d | Dropped: %d | Time: %.2fs | Actual FPS: %.4f", 
                                 cam_ctxs[i]->index, total_saved, dropped_frames, duration_s, actual_fps);
                    } else {
                        snprintf(report_buf, sizeof(report_buf), 
                                 "[OK]      Cam %d | Saved: %4d | Dropped: 0 | Time: %.2fs | Actual FPS: %.4f", 
                                 cam_ctxs[i]->index, total_saved, duration_s, actual_fps);
                    }
                    cout << report_buf << endl;
                }

                // =========================================================
                // 5. 根据配置决定是否进行后处理保存 JPG
                // =========================================================
                if (write_jpg) {
                    cout << "\n[Info] Analyzing timestamps for JPG synchronization..." << endl;

                    int64_t max_start_time = -1;
                    int64_t min_end_time = 9223372036854775807LL; 
                    bool use_block_id_sync = true; 
                    int64_t global_start_idx = 0;
                    int64_t global_end_idx = 9999999999LL; 

                    // 计算时间窗口交集
                    for (auto& logs : all_logs) {
                        if (logs.empty()) continue;
                        if (logs.front().blockID > global_start_idx) global_start_idx = logs.front().blockID;
                        if (logs.back().blockID < global_end_idx) global_end_idx = logs.back().blockID;
                        if (logs.front().timestamp > max_start_time) max_start_time = logs.front().timestamp;
                        if (logs.back().timestamp < min_end_time) min_end_time = logs.back().timestamp;
                    }

                    vector<vector<LogEntry>> final_entries_lists(cam_ctxs.size()); 
                    int total_frames_all_cams = 0;

                    for (int i = 0; i < cam_ctxs.size(); ++i) {
                        for (const auto& entry : all_logs[i]) {
                            bool keep = false;
                            if (use_block_id_sync) {
                                if (entry.blockID >= global_start_idx && entry.blockID <= global_end_idx) keep = true;
                            } else {
                                if (entry.timestamp >= max_start_time && entry.timestamp <= min_end_time) keep = true;
                            }
                            if (keep) final_entries_lists[i].push_back(entry); 
                        } 
                        total_frames_all_cams += final_entries_lists[i].size(); 
                    } 
                    
                    std::atomic<int> global_processed{0};
                    
                    auto draw_progress_ui = [&]() {
                        cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                        int processed = global_processed.load();
                        string text = "Saving JPGs... " + to_string(processed) + "/" + to_string(total_frames_all_cams);
                        cv::putText(loading, text, cv::Point(50, 180), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                        
                        int bar_x = 50, bar_y = 220, bar_w = 500, bar_h = 20;
                        float ratio = total_frames_all_cams > 0 ? (std::min)(1.0f, (float)processed / total_frames_all_cams) : 0.0f;
                        
                        cv::rectangle(loading, cv::Point(bar_x, bar_y), cv::Point(bar_x + bar_w, bar_y + bar_h), cv::Scalar(255, 255, 255), 1);
                        if (ratio > 0) {
                            cv::rectangle(loading, cv::Point(bar_x, bar_y), cv::Point(bar_x + (int)(bar_w * ratio), bar_y + bar_h), cv::Scalar(0, 255, 0), -1);
                        }
                        cv::imshow("Multi-Cam Preview", loading);
                        cv::waitKey(1);
                    };
                    
                    vector<thread> compile_threads;
                    for (int i = 0; i < cam_ctxs.size(); ++i) {
                        string out_jpg_dir = save_base_dir + "/record_" + current_record_timestr + "/cam_" + to_string(cam_ctxs[i]->index);
                        compile_threads.emplace_back(
                            convertRawToJpgWorker, 
                            cam_ctxs[i]->temp_dir,      
                            out_jpg_dir,     
                            final_entries_lists[i],     
                            std::ref(global_processed)                 
                        );
                    }
                    
                    int last_processed = 0;
                    int same_count = 0;  
                    
                    while (true) {
                        draw_progress_ui();
                        int current_processed = global_processed.load();
                        if (current_processed >= total_frames_all_cams) break;
                        
                        if (current_processed == last_processed) {
                            same_count++;
                            if (same_count > 100) {  
                                cout << "[Warning] Disk IO stalled or progress finished." << endl;
                                break;
                            }
                        } else {
                            same_count = 0;
                            last_processed = current_processed;
                        }
                        this_thread::sleep_for(chrono::milliseconds(50));
                    }
                    
                    for (auto& t : compile_threads) {
                        if (t.joinable()) t.join();
                    }
                    
                    global_processed = total_frames_all_cams;  
                    draw_progress_ui();
                    cv::waitKey(500);
                } 
                else {
                    cout << "\n[Info] write_jpg is disabled. Process complete without JPG conversion." << endl;
                    cv::waitKey(200); 
                }
            }
        }
        else if (key == ' ') { // 拍照
            // 获取计数器 (扫描整个目录)
            int counter = getNextCalibCounter(save_base_dir);
            
            std::stringstream ss;
            ss << std::setw(2) << std::setfill('0') << counter;
            string calib_str = ss.str();
            
            cout << "\n[Photo] Capturing calibration set " << calib_str << endl;

            for (auto& ctx : cam_ctxs) {
                cv::Mat snapshot;
                {
                    lock_guard<mutex> lock(ctx->frame_mtx);
                    snapshot = ctx->latest_frame.clone();
                }
                
                if (!snapshot.empty()) {
                    string fn = save_base_dir + "/calib_cam_" + to_string(ctx->index) + "_" + calib_str + ".jpg";
                    cv::imwrite(fn, snapshot);
                    cout << "  -> Saved " << fn << endl;
                }
            }
        }
    }

    // --- 3. 清理资源 ---
    cout << "[System] Shutting down threads..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->queue_cv.notify_all(); // 唤醒写线程以便退出
        
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->writer_thread.joinable()) ctx->writer_thread.join();
    }
    
    cv::destroyAllWindows();
    Pylon::PylonTerminate();
    return 0;
}