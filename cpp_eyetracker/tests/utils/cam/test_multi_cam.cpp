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
};

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
    FrameMeta latest_meta;
    mutex frame_mtx; 
    
    // 录制相关 (Temp存储)
    string temp_dir;
    string log_file_path; // txt 文件路径
    ofstream log_stream;  // 文件流

    queue<FrameTask> write_queue; 
    mutex queue_mtx;
    condition_variable queue_cv;
    
    atomic<int> captured_frames{0};

    atomic<CamStatus> status{CamStatus::INIT};
    string status_msg = "Cam is initializing";

    int64_t frame_offset = 0;
    bool offset_initialized = false;
    static atomic<int64_t> master_first_id; // 静态变量，记录第一台准备好的相机的ID
    static atomic<bool> master_set;

    // 新增：用于视频写入的追踪
    int64_t last_recorded_aligned_id = -1;

    CameraContext(int idx, string cam_id) : index(idx), id(cam_id) {}
};

atomic<int64_t> CameraContext::master_first_id(-1);
atomic<bool> CameraContext::master_set(false);

// ================== 全局变量 ==================
vector<shared_ptr<CameraContext>> cam_ctxs;

// ================== 写入线程逻辑 ==================
// 职责：写图片 + 写TXT日志
void writerWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        FrameTask task;
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
        
        // 1. 写入图片
        if (!task.image.empty()) {
            cv::imwrite(task.filename, task.image);
        }

        // 2. 写入日志 (追加模式)
        // 格式: filename, blockID, timestamp
        int64_t aligned_id = task.meta.blockID - ctx->frame_offset;
        if (ctx->log_stream.is_open()) {
            ctx->log_stream << fs::path(task.filename).filename().string() << "," 
                            << aligned_id << "," 
                            << task.meta.timestamp << ","
                            << task.meta.blockID << "\n";

        }
    }
}

// ================== 采集线程逻辑 ==================
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time, bool use_hw_trigger) {
    // 1. 根据配置决定触发模式
    TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
    
    ctx->cam = new BaslerCamera(ctx->id);
    if (!ctx->cam->open(mode)) {
        ctx->status = CamStatus::ERROR_;
        ctx->status_msg = "OPEN FAILED";
        return;
    }

    cout << "[Step 2] Setting Params Cam " << ctx->index << "..." << endl;
    try {
        // 只有软触发才设置FPS，硬触发通常由外部信号决定，强行设置可能会报错
        if (!use_hw_trigger) {
            ctx->cam->setFrameRate(fps);
        }
        ctx->cam->setGain(gain);
        ctx->cam->setGamma(gamma);
        ctx->cam->setExposureTime(exp_time);
    } catch (const std::exception& e) {
        cerr << "[Param Error] " << e.what() << endl;
        // 参数设置失败不一定要退出，可以继续尝试
    }
    
    cout << "[Step 3] Starting Grab Stream Cam " << ctx->index << "..." << endl;
    if (!ctx->cam->start()) {
        cerr << "[Error] Cam " << ctx->index << " failed to start grabbing." << endl;
        return;
    }

    ctx->status = CamStatus::OPENED;
    ctx->status_msg = use_hw_trigger ? "HW WAITING" : "STREAMING";

    cout << "[Step 4] Entering Loop Cam " << ctx->index << endl;

    int frame_seq = 0;
    int64_t frame_counter = 0;

    while (ctx->running) {
        cv::Mat frame;
        FrameMeta meta;
        
        try{
            auto ret = ctx->cam->grabFrame(frame, meta);

            // 使用新的 grabFrame 获取元数据
            if (ret == GrabResult::OK) {
                frame_counter++;
                ctx->status = CamStatus::STREAMING;

                if (!ctx->offset_initialized && frame_counter > 1) {
                    // 尝试成为 Master
                    if (!CameraContext::master_set.exchange(true)) {
                        // 我抢到了 Master 位置
                        CameraContext::master_first_id = meta.blockID;
                        ctx->frame_offset = 0;
                        ctx->offset_initialized = true;
                        cout << "[Sync] Cam " << ctx->index << " is Master. Base ID: " << meta.blockID << endl;
                    } else {
                        // 我是 Slave，我需要等待 Master 把基准 ID 写进去
                        // 虽然 exchange 保证了 master_set 为 true，但写 ID 可能慢几微秒
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

                // 3. 最终对齐逻辑
                if (ctx->offset_initialized) {
                    int64_t aligned_id = meta.blockID - ctx->frame_offset;
                    
                    // 将对齐后的 ID 存入 metadata，方便后续存图或处理
                    meta.blockID = aligned_id; 

                    lock_guard<mutex> lock(ctx->frame_mtx);
                    frame.copyTo(ctx->latest_frame);
                    ctx->latest_meta = meta;
                    ctx->captured_frames++;
                }

                // 录制逻辑
                if (ctx->recording) {
                    // 如果是新的一次录制开始，frame_seq 会在主线程置零，这里累加
                    // 生成文件名
                    std::stringstream ss;
                    ss << ctx->temp_dir << "/" << std::setw(6) << std::setfill('0') << frame_seq++ << ".jpg";
                    
                    {
                        lock_guard<mutex> lock(ctx->queue_mtx);
                        ctx->write_queue.push({ss.str(), frame.clone(), meta});
                    }
                    ctx->queue_cv.notify_one();
                } else {
                    frame_seq = 0; 
                }
            }
            else if (ret == GrabResult::TIMEOUT) {
                ctx->status = CamStatus::WAITING_TRIGGER;
                ctx->status_msg = "HW TRIGGER - WAITING FOR TTL";
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 稍微长一点防止CPU占满
                continue;  // ⚠️ 重要：不要 break
            }
            else {
                ctx->status = CamStatus::ERROR_;
                ctx->status_msg = "GRAB ERROR";
                break;
            }
        }
        catch (const std::exception& e) {
            cerr << "!!! [CRASH] Thread " << ctx->index << " died with exception: " << e.what() << endl;
        }
        catch (...) {
            cerr << "!!! [CRASH] Thread " << ctx->index << " died with UNKNOWN exception." << endl;
        }
    }

    // 清理
    if (ctx->cam) {
        // 确保停止采集
        // ctx->cam->stop(); // 如果你有 stop 函数
        ctx->cam->close();
        delete ctx->cam;
        ctx->cam = nullptr;
    }
    cout << "[End] Thread " << ctx->index << " exited." << endl;
}

// ================== 同步与视频合成逻辑 ==================

// 解析日志文件
vector<LogEntry> parseLogFile(const string& log_path) {
    vector<LogEntry> entries;
    ifstream infile(log_path);
    string line;
    while (getline(infile, line)) {
        stringstream ss(line);
        string segment;
        vector<string> parts;
        while (getline(ss, segment, ',')) parts.push_back(segment);
        
        if (parts.size() >= 3) {
            LogEntry entry;
            entry.filename = parts[0];
            entry.blockID = stoll(parts[1]);
            entry.timestamp = stoll(parts[2]);
            entries.push_back(entry);
        }
    }
    return entries;
}

// 视频合成线程函数 (现在接收过滤后的文件列表)
void compileVideoWorker(string temp_dir, vector<string> valid_files, string final_video_path, double fps, atomic<int>& global_processed) {
    if (valid_files.empty()) return;

    // 读取第一帧确定尺寸
    string first_path = temp_dir + "/" + valid_files[0];
    cv::Mat first = cv::imread(first_path);
    if (first.empty()) return;

    cv::VideoWriter writer(final_video_path, cv::VideoWriter::fourcc('M','J','P','G'), fps, first.size());
    if (!writer.isOpened()) {
        cerr << "[Error] Cannot open writer: " << final_video_path << endl;
        return;
    }

    for (const auto& fn : valid_files) {
        string full_path = temp_dir + "/" + fn;
        cv::Mat img = cv::imread(full_path);
        if (!img.empty()) {
            writer.write(img);
        }
        global_processed++;
    }
    writer.release();
    cout << "[Done] Saved " << final_video_path << " (" << valid_files.size() << " frames)" << endl;
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Optimized) ===" << endl;
    
    Cfg cfg;
    Pylon::PylonInitialize();
    
    // --- 0. 配置读取 (假设 Config 中有相机列表，这里为了演示手动构建或从 string 读取) ---
    // 假设 cfg["cameras"] 是一个包含相机ID的数组，或者我们在代码里指定
    // 这里为了通用，假设我们读取一个字符串列表，或者你可以修改此处
    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();

    // !!! 新增：是否使用硬件触发 !!!
    bool use_hw_trigger = cfg["test_multi_cam"]["hardware_trigger"].as<bool>();

    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time = cfg["test_multi_cam"]["exposure_time"].as<double>();
    
    string save_base_dir = cfg["test_multi_cam"]["save_dir"].as<std::string>();
    int win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    int win_h = cfg["test_multi_cam"]["window_height"].as<int>();

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
            {
                lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);
                if (!cam_ctxs[i]->latest_frame.empty()) {
                    // 缩放
                    cv::resize(cam_ctxs[i]->latest_frame, img, cv::Size(cell_w, cell_h));
                }
            }

            if (img.empty()) {
                img = cv::Mat::zeros(cell_h, cell_w, CV_8UC3);

                int baseline = 0;
                cv::Size textSize = cv::getTextSize(cam_ctxs[i]->status_msg, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
                cv::Point textOrg((cell_w - textSize.width) / 2, (cell_h + textSize.height) / 2);
                cv::putText(img, cam_ctxs[i]->status_msg, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            } else {
                if (img.type() == CV_8UC1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

                // --- 新增：如果是录制状态，在右上角画红点 ---
                if (is_recording) {
                    int radius = 10;
                    // 右上角偏移量
                    cv::Point center(img.cols - radius * 2, radius * 2);
                    
                    // 画实心红点 (BGR: 0, 0, 255)
                    cv::circle(img, center, radius, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                    
                    // 可选：添加 "REC" 文字
                    cv::putText(img, "REC", cv::Point(img.cols - radius * 7, radius * 2.5), 
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
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
            // 这里我们更新显示的 Mat，不改变 Window 大小本身（或者你可以 resizeWindow）
            cv::imshow("Multi-Cam Preview", cropped);
        } else {
            cv::imshow("Multi-Cam Preview", canvas);
        }

        // --- C. 按键处理 ---
        char key = (char)cv::waitKey(20); // 20ms refresh

        if (key == 'q' || key == 27) {
            running = false;
        }
        else if (key == 'r') {
            if (!is_recording) {
                // 开始录制
                auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                char buf[64];
                strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                current_record_timestr = string(buf);

                cout << "\n[Info] Start Recording Batch: " << current_record_timestr << endl;

                for (auto& ctx : cam_ctxs) {
                    string batch_temp = save_base_dir + "/temp_" + current_record_timestr + "/cam_" + to_string(ctx->index);
                    fs::create_directories(batch_temp);
                    ctx->temp_dir = batch_temp;
                    
                    // 打开日志文件
                    ctx->log_file_path = save_base_dir + "/record_" + current_record_timestr + "_cam_" + to_string(ctx->index) + ".txt";
                    ctx->log_stream.open(ctx->log_file_path);
                    if(!ctx->log_stream.is_open()) cerr << "Failed to create log: " << ctx->log_file_path << endl;

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

                // 2. 等待队列写完
                cv::Mat wait_img = cv::Mat::zeros(400, 600, CV_8UC3);
                cv::putText(wait_img, "Flushing Write Queue...", cv::Point(50, 200),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                cv::imshow("Multi-Cam Preview", wait_img);
                cv::waitKey(1);

                // 2. 等待队列清空 & 关闭日志
                for (auto& ctx : cam_ctxs) {
                    {
                        unique_lock<mutex> lk(ctx->queue_mtx);
                        ctx->queue_cv.wait(lk, [&]{ return ctx->write_queue.empty(); });
                    }
                    if (ctx->log_stream.is_open()) ctx->log_stream.close();
                }

                // ================== 核心同步逻辑 ==================
                cout << "[Info] Analyzing timestamps for synchronization..." << endl;
                
                // 3. 读取所有日志
                vector<vector<LogEntry>> all_logs;
                for (auto& ctx : cam_ctxs) {
                    all_logs.push_back(parseLogFile(ctx->log_file_path));
                }

                // 4. 计算时间窗口交集 (Intersection)
                // 假设硬件触发下，BlockID 是相对可靠的，或者使用 Timestamp
                // 这里使用 Timestamp 寻找最大开始时间和最小结束时间
                int64_t max_start_time = -1;
                int64_t min_end_time = -1;
                
                // 注意：Basler Timestamp 是开机计时的 tick。不同相机之间如果不是 PTP 同步，基准不同。
                // 如果是硬件触发，BlockID 更加可靠（假设所有相机同时收到触发信号并开始计数）。
                // 这里我们提供两种策略，优先使用 BlockID 进行对齐（因为硬件触发保证了帧的一一对应）。
                
                bool use_block_id_sync = true; // 硬件触发推荐 true
                
                int64_t global_start_idx = 0;
                int64_t global_end_idx = 9999999999; // 找最小的结束ID

                if (use_block_id_sync) {
                    // 找到所有相机中记录到的 *最大* 的起始 BlockID (作为公共起跑线)
                    for (auto& logs : all_logs) {
                        if (logs.empty()) continue;
                        if (logs.front().blockID > global_start_idx) global_start_idx = logs.front().blockID;
                    }
                    // 找到所有相机中记录到的 *最小* 的结束 BlockID (作为公共终点线)
                    for (auto& logs : all_logs) {
                        if (logs.empty()) continue;
                        if (logs.back().blockID < global_end_idx) global_end_idx = logs.back().blockID;
                    }
                }

                cout << "Sync Range (BlockID): " << global_start_idx << " to " << global_end_idx << endl;

                // 5. 生成过滤后的文件列表
                vector<vector<string>> final_file_lists(cam_ctxs.size());
                int total_frames_all_cams = 0;

                for (int i = 0; i < cam_ctxs.size(); ++i) {
                    for (const auto& entry : all_logs[i]) {
                        bool keep = false;
                        if (use_block_id_sync) {
                            if (entry.blockID >= global_start_idx && entry.blockID <= global_end_idx) keep = true;
                        } else {
                            // Timestamp 逻辑类似，略
                        }

                        if (keep) {
                            final_file_lists[i].push_back(entry.filename);
                        }
                    }
                    total_frames_all_cams += final_file_lists[i].size();
                    cout << "Cam " << i << ": Raw=" << all_logs[i].size() << ", Valid=" << final_file_lists[i].size() << endl;
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

                for (int i = 0; i < cam_ctxs.size(); ++i) {
                    string video_fn = save_base_dir + "/record_" + current_record_timestr + "_cam_" + to_string(cam_ctxs[i]->index) + ".avi";
                    compile_threads.emplace_back(
                        compileVideoWorker, 
                        cam_ctxs[i]->temp_dir, 
                        final_file_lists[i], 
                        video_fn, 
                        target_fps, 
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