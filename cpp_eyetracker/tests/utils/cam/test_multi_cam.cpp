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
#include <fstream>
#include <pylon/PylonIncludes.h>

#include "cam/basler.hpp"
#include "cfg/config.hpp"

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

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

enum class CamStatus { INIT, OPENED, WAITING_TRIGGER, STREAMING, ERROR_ };

struct LogEntry { string filename; int64_t blockID; int64_t timestamp; int width; int height; };

struct CameraContext {
    int index;
    string id;
    string save_base_dir; 
    BaslerCamera cam{""};

    bool is_mono = true;
    
    thread capture_thread;
    thread copy_thread; 
    atomic<bool> running{true};
    atomic<bool> recording{false};

    cv::Mat latest_frame;
    FrameMeta latest_meta;
    mutex frame_mtx; 

    vector<cv::Mat> ram_buffer;
    vector<FrameMeta> meta_buffer;
    int total_record_frames = 0;
    atomic<bool> dump_ready{false};

    queue<pair<Pylon::CBaslerUniversalGrabResultPtr, FrameMeta>> copy_queue;
    mutex copy_mtx;
    condition_variable copy_cv;

    string temp_dir;      
    string log_file_path;
    ofstream log_stream;

    atomic<int> captured_frames{0};
    atomic<int> recorded_frames{0}; 

    atomic<CamStatus> status{CamStatus::INIT};
    string status_msg = "Initializing";

    int64_t frame_offset = 0;
    bool offset_initialized = false;
    static atomic<int64_t> master_first_id; 
    static atomic<bool> master_set;

    CameraContext(int idx, string cam_id, string save_dir) : index(idx), id(cam_id), save_base_dir(save_dir), cam(cam_id) {}
};

atomic<int64_t> CameraContext::master_first_id(-1);
atomic<bool> CameraContext::master_set(false);

vector<shared_ptr<CameraContext>> cam_ctxs;

// ================== 后台异步拷贝线程 ==================
void copyWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        pair<Pylon::CBaslerUniversalGrabResultPtr, FrameMeta> task;
        {
            unique_lock<mutex> lock(ctx->copy_mtx);
            ctx->copy_cv.wait(lock, [&]{ return !ctx->copy_queue.empty() || !ctx->running; });
            if (!ctx->running && ctx->copy_queue.empty()) break;
            task = ctx->copy_queue.front();
            ctx->copy_queue.pop();
        }

        if (ctx->recording) {
            int seq = ctx->recorded_frames.load(std::memory_order_relaxed);
            if (seq < ctx->total_record_frames) {
                void* pBuffer = task.first->GetBuffer();
                size_t payload_size = task.first->GetWidth() * task.first->GetHeight();
                
                memcpy(ctx->ram_buffer[seq].data, pBuffer, payload_size);
                ctx->meta_buffer[seq] = task.second;
                
                {
                    lock_guard<mutex> lock(ctx->frame_mtx);
                    ctx->latest_frame = ctx->ram_buffer[seq]; 
                    ctx->latest_meta = task.second;
                }

                int next_seq = seq + 1;
                ctx->recorded_frames.store(next_seq, std::memory_order_relaxed);

                if (next_seq == ctx->total_record_frames) {
                    ctx->recording = false;
                    ctx->dump_ready = true;
                }
            }
        } else {
            cv::Mat temp(task.first->GetHeight(), task.first->GetWidth(), CV_8UC1, task.first->GetBuffer());
            cv::Mat clone_img = temp.clone(); 
            {
                lock_guard<mutex> lock(ctx->frame_mtx);
                ctx->latest_frame = clone_img;
                ctx->latest_meta = task.second;
            }
        }
    }
}

// ================== Pylon 回调触发线程 ==================
void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time, bool use_hw_trigger) {
    TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
    if (!ctx->cam.open(mode)) { ctx->status = CamStatus::ERROR_; return; }

    ctx->is_mono = ctx->cam.isMono();

    try {
        if (!use_hw_trigger) ctx->cam.setFrameRate(fps);  
        ctx->cam.setGain(gain);                           
        ctx->cam.setGamma(gamma);                         
        ctx->cam.setExposureTime(exp_time);               
    } catch (...) {}

    struct GrabState { int64_t frame_counter = 0; };
    auto state = make_shared<GrabState>();

    ctx->cam.setFrameCallback([ctx, state](const Pylon::CBaslerUniversalGrabResultPtr& ptr, FrameMeta meta) {
        state->frame_counter++;
        ctx->status = CamStatus::STREAMING;

        if (!ctx->offset_initialized && state->frame_counter > 1) {
            if (!CameraContext::master_set.exchange(true)) {
                CameraContext::master_first_id = meta.blockID;
                ctx->frame_offset = 0;
                ctx->offset_initialized = true;
            } else {
                int retry = 0;
                while (CameraContext::master_first_id == -1 && retry < 100) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    retry++;
                }
                if (CameraContext::master_first_id != -1) {
                    ctx->frame_offset = meta.blockID - CameraContext::master_first_id.load();
                    ctx->offset_initialized = true;
                }
            }
        }

        if (ctx->offset_initialized) {
            meta.blockID = meta.blockID - ctx->frame_offset; 
            ctx->captured_frames++;

            lock_guard<mutex> lock(ctx->copy_mtx);
            if (ctx->recording) {
                ctx->copy_queue.push({ptr, meta});
                ctx->copy_cv.notify_one();
            } else {
                if (ctx->copy_queue.size() < 2) {
                    ctx->copy_queue.push({ptr, meta});
                    ctx->copy_cv.notify_one();
                }
            }
        }
    });

    if (!ctx->cam.start()) { ctx->status = CamStatus::ERROR_; return; }
    ctx->status_msg = use_hw_trigger ? "HW WAITING" : "STREAMING";

    while (ctx->running) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ctx->cam.close();
}

void dumpToDiskWorker(shared_ptr<CameraContext> ctx, int write_delay_ms, atomic<int>& finished_cams) {
    ctx->log_stream.open(ctx->log_file_path);
    int frames_to_dump = ctx->recorded_frames.load();
    
    for (int i = 0; i < frames_to_dump; ++i) {
        std::stringstream ss;
        ss << ctx->temp_dir << "/" << std::setw(6) << std::setfill('0') << i << ".raw";
        string filename = ss.str();
        
        cv::Mat& img = ctx->ram_buffer[i];
        FrameMeta& meta = ctx->meta_buffer[i];
        
        std::ofstream out_raw(filename, std::ios::binary);
        if (out_raw) out_raw.write(reinterpret_cast<const char*>(img.data), img.total() * img.elemSize());

        if (ctx->log_stream.is_open()) {
            ctx->log_stream << fs::path(filename).filename().string() << "," 
                            << meta.blockID << "," << meta.timestamp << ","
                            << meta.blockID << "," << img.cols << "," << img.rows << "\n";
        }
        if (write_delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(write_delay_ms));
    }
    if (ctx->log_stream.is_open()) ctx->log_stream.close();
    finished_cams++;
}

vector<LogEntry> parseLogFile(const string& log_path) {
    vector<LogEntry> entries;
    ifstream infile(log_path);
    string line;
    while (getline(infile, line)) {
        stringstream ss(line); string segment; vector<string> parts;
        while (getline(ss, segment, ',')) parts.push_back(segment);
        if (parts.size() >= 6) entries.push_back({parts[0], stoll(parts[1]), stoll(parts[2]), stoi(parts[4]), stoi(parts[5])});
    }
    return entries;
}

// 增加 is_mono 参数
void convertRawToJpgWorker(string temp_raw_dir, string out_jpg_dir, vector<LogEntry> valid_entries, atomic<int>& global_processed, bool is_mono) {
    if (valid_entries.empty()) return;
    fs::create_directories(out_jpg_dir);
    for (const auto& entry : valid_entries) {
        string raw_path = temp_raw_dir + "/" + entry.filename;
        cv::Mat raw_img(entry.height, entry.width, CV_8UC1);
        ifstream in_raw(raw_path, ios::binary);
        if (in_raw) {
            in_raw.read(reinterpret_cast<char*>(raw_img.data), raw_img.total() * raw_img.elemSize());
            in_raw.close();
            
            // ===== 新增彩色解算逻辑 =====
            cv::Mat final_img;
            if (is_mono) {
                final_img = raw_img;
            } else {
                cv::cvtColor(raw_img, final_img, cv::COLOR_BayerRG2RGB);
            }
            // ============================

            string jpg_filename = entry.filename;
            size_t dot_pos = jpg_filename.find_last_of('.');
            if (dot_pos != string::npos) jpg_filename = jpg_filename.substr(0, dot_pos) + ".jpg";
            else jpg_filename += ".jpg";
            
            cv::imwrite(out_jpg_dir + "/" + jpg_filename, final_img); // 写入 final_img
        }
        global_processed++;
    }
}

int main() {
    cout << "=== [TEST] Multi-Basler Camera Tool (Async Pipeline) ===" << endl;
    
    Cfg cfg;
    Pylon::PylonInitialize();
    
    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();
    bool use_hw_trigger = cfg["test_multi_cam"]["hardware_trigger"].as<bool>();
    double target_fps = cfg["test_multi_cam"]["fps"].as<double>();
    double gain = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time = cfg["test_multi_cam"]["exposure_time"].as<double>();
    vector<string> save_dirs = cfg["test_multi_cam"]["save_dir"].as<vector<string>>();
    
    int win_w = cfg["test_multi_cam"]["window_width"].as<int>();
    int win_h = cfg["test_multi_cam"]["window_height"].as<int>();
    bool write_jpg = cfg["test_multi_cam"]["write_jpg"].as<bool>(); 
    
    double record_time = cfg["test_multi_cam"]["record_time"].as<double>(); 
    int write_delay_ms = cfg["test_multi_cam"]["write_delay_ms"].as<int>();
    int cam_w = cfg["test_multi_cam"]["cam_width"].as<int>();
    int cam_h = cfg["test_multi_cam"]["cam_height"].as<int>();

    int core_frames = static_cast<int>(std::ceil(target_fps * record_time));
    int margin_frames = static_cast<int>(std::ceil(core_frames * 0.1));
    int total_record_frames = core_frames + 2 * margin_frames;

    if (save_dirs.size() != camera_ids.size()) { cerr << "Mismatch config." << endl; return -1; }
    for (const auto& dir : save_dirs) std::filesystem::create_directories(dir);

    for (int i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(i, camera_ids[i], save_dirs[i]);
        cam_ctxs.push_back(ctx);
    }

    cout << "[System] Pre-allocating contiguous RAM blocks for " << total_record_frames << " frames..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->total_record_frames = total_record_frames;
        ctx->ram_buffer.resize(total_record_frames);
        ctx->meta_buffer.resize(total_record_frames);
        for (int k = 0; k < total_record_frames; ++k) {
            ctx->ram_buffer[k] = cv::Mat::zeros(cam_h, cam_w, CV_8UC1);
        }
    }
    cout << "[System] OS Memory array reserved. Pipeline ready.\n" << endl;

    CameraContext::master_set.store(false);
    CameraContext::master_first_id.store(-1);
    
    for (auto& ctx : cam_ctxs) {
        ctx->running = true;
        ctx->dump_ready = false;
        ctx->offset_initialized = false;
        ctx->copy_thread = thread(copyWorker, ctx);
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain, gamma, exp_time, use_hw_trigger);
    }

    cv::namedWindow("Multi-Cam Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Multi-Cam Preview", win_w, win_h);
    cout << "Press 'r' to record, 's' to force stop, 'space' to photo, 'q' to quit.\n";

    bool is_recording = false;
    atomic<bool> is_dumping{false}; 
    bool running = true;
    string current_record_timestr;
    std::chrono::steady_clock::time_point record_start_time;

    while (running) {
        int n_cams = cam_ctxs.size();
        int grid_rows = 1, grid_cols = 1;
        if (n_cams == 1) { grid_rows = 1; grid_cols = 1; }
        else if (n_cams <= 4) { grid_rows = 2; grid_cols = 2; }
        else if (n_cams <= 9) { grid_rows = 3; grid_cols = 3; }
        else { grid_rows = 4; grid_cols = (n_cams + 3) / 4; }

        int cell_w = win_w / grid_cols;
        int cell_h = win_h / grid_rows;
        cv::Mat canvas = cv::Mat::zeros(win_h, win_w, CV_8UC3);
        int valid_rows = 0; 

        for (int i = 0; i < n_cams; ++i) {
            cv::Mat img, local_raw;
            {
                lock_guard<mutex> lock(cam_ctxs[i]->frame_mtx);
                if (!cam_ctxs[i]->latest_frame.empty()) local_raw = cam_ctxs[i]->latest_frame.clone(); 
            }

            if (!local_raw.empty()) {
                cv::Mat color_full;
                
                if (cam_ctxs[i]->is_mono) {
                    cv::cvtColor(local_raw, color_full, cv::COLOR_GRAY2RGB);
                } else {
                    cv::cvtColor(local_raw, color_full, cv::COLOR_BayerRG2RGB); 
                }
                
                cv::resize(color_full, img, cv::Size(cell_w, cell_h));
            }

            if (img.empty()) {
                img = cv::Mat::zeros(cell_h, cell_w, CV_8UC3);
                int baseline = 0;
                cv::Size textSize = cv::getTextSize(cam_ctxs[i]->status_msg, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
                cv::Point textOrg((cell_w - textSize.width) / 2, (cell_h + textSize.height) / 2);
                cv::putText(img, cam_ctxs[i]->status_msg, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            } else {
                if (is_recording) {
                    int radius = 10;
                    cv::Point center(img.cols - radius * 2, radius * 2);
                    cv::circle(img, center, radius, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                    cv::putText(img, "REC", cv::Point(img.cols - radius * 7, radius * 2.5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                    int current_frame = cam_ctxs[i]->recorded_frames.load(std::memory_order_relaxed);
                    double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - record_start_time).count();
                    char time_buf[32]; snprintf(time_buf, sizeof(time_buf), "Time: %.1fs", elapsed_s);
                    
                    cv::putText(img, "Frame: " + to_string(current_frame) + "/" + to_string(total_record_frames), cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
                    cv::putText(img, "Frame: " + to_string(current_frame) + "/" + to_string(total_record_frames), cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    cv::putText(img, time_buf, cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
                    cv::putText(img, time_buf, cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                }
            }
            int r = i / grid_cols; int c = i % grid_cols;
            img.copyTo(canvas(cv::Rect(c * cell_w, r * cell_h, cell_w, cell_h)));
            if (r + 1 > valid_rows) valid_rows = r + 1;
        }

        if (valid_rows > 0 && valid_rows < grid_rows) {
            cv::Mat cropped = canvas(cv::Rect(0, 0, win_w, valid_rows * cell_h));
            cv::resizeWindow("Multi-Cam Preview", cropped.cols, cropped.rows);
            cv::imshow("Multi-Cam Preview", cropped);
        } else cv::imshow("Multi-Cam Preview", canvas);

        if (is_recording && !is_dumping) {
            bool all_done = true;
            for (auto& ctx : cam_ctxs) if (!ctx->dump_ready.load()) { all_done = false; break; }

            if (all_done) {
                is_recording = false;
                is_dumping = true;
                
                cout << "\n[Info] Processing Disk Dump in background. Cameras remain fully active..." << endl;
                
                auto dump_start_time = std::chrono::steady_clock::now();
                atomic<int> finished_cams{0};
                vector<thread> dump_threads;

                for (auto& ctx : cam_ctxs) dump_threads.emplace_back(dumpToDiskWorker, ctx, write_delay_ms, std::ref(finished_cams));

                while (finished_cams < cam_ctxs.size()) {
                    cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                    cv::putText(loading, "DUMPING RAM TO DISK... (" + to_string(finished_cams.load()) + "/" + to_string(cam_ctxs.size()) + ")", cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                    cv::imshow("Multi-Cam Preview", loading);
                    cv::waitKey(50); 
                }
                for (auto& t : dump_threads) if (t.joinable()) t.join();

                double dump_duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - dump_start_time).count();
                cout << "\n=================================================" << endl;
                cout << "[Performance] Disk Dump Finished! Time: " << fixed << setprecision(3) << dump_duration << "s" << endl;
                cout << "=================================================" << endl;

                cout << "\n[Info] Calculating frame drop and actual FPS statistics..." << endl;
                vector<vector<LogEntry>> all_logs;
                vector<vector<LogEntry>> core_sliced_logs; 
                
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
                        duration_s = static_cast<double>(logs.back().timestamp - logs.front().timestamp) / 10000000.0; 
                        if (duration_s > 0) actual_fps = (total_saved - 1) / duration_s;
                    }
                    
                    char report_buf[256];
                    if (dropped_frames > 0) {
                        snprintf(report_buf, sizeof(report_buf), "[Warning] Cam %d | Saved: %4d | Dropped: %d | Time: %.2fs | Actual FPS: %.4f", 
                                 cam_ctxs[i]->index, total_saved, dropped_frames, duration_s, actual_fps);
                    } else {
                        snprintf(report_buf, sizeof(report_buf), "[OK]      Cam %d | Saved: %4d | Dropped: 0 | Time: %.2fs | Actual FPS: %.4f", 
                                 cam_ctxs[i]->index, total_saved, duration_s, actual_fps);
                    }
                    cout << report_buf << endl;

                    if (logs.size() > 2 * margin_frames) core_sliced_logs.push_back(vector<LogEntry>(logs.begin() + margin_frames, logs.end() - margin_frames));
                    else core_sliced_logs.push_back(logs);
                }

                if (write_jpg) {
                    cout << "\n[Info] Generating Sync JPGs for Core Frames..." << endl;
                    int64_t max_start_time = -1;
                    int64_t min_end_time = 9223372036854775807LL; 
                    int64_t global_start_idx = 0;
                    int64_t global_end_idx = 9999999999LL; 

                    for (auto& logs : core_sliced_logs) {
                        if (logs.empty()) continue;
                        if (logs.front().blockID > global_start_idx) global_start_idx = logs.front().blockID;
                        if (logs.back().blockID < global_end_idx) global_end_idx = logs.back().blockID;
                        if (logs.front().timestamp > max_start_time) max_start_time = logs.front().timestamp;
                        if (logs.back().timestamp < min_end_time) min_end_time = logs.back().timestamp;
                    }

                    vector<vector<LogEntry>> final_entries_lists(cam_ctxs.size()); 
                    int total_frames_all_cams = 0;

                    for (int i = 0; i < cam_ctxs.size(); ++i) {
                        for (const auto& entry : core_sliced_logs[i]) {
                            if (entry.blockID >= global_start_idx && entry.blockID <= global_end_idx) final_entries_lists[i].push_back(entry); 
                        } 
                        total_frames_all_cams += final_entries_lists[i].size(); 
                    } 

                    if (total_frames_all_cams > 0) {
                        std::atomic<int> global_processed{0};
                        
                        auto draw_progress_ui = [&]() {
                            cv::Mat loading = cv::Mat::zeros(400, 600, CV_8UC3);
                            int processed = global_processed.load();
                            cv::putText(loading, "Saving JPGs... " + to_string(processed) + "/" + to_string(total_frames_all_cams), cv::Point(50, 180), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                            float ratio = total_frames_all_cams > 0 ? (std::min)(1.0f, (float)processed / total_frames_all_cams) : 0.0f;
                            cv::rectangle(loading, cv::Point(50, 220), cv::Point(550, 240), cv::Scalar(255, 255, 255), 1);
                            if (ratio > 0) cv::rectangle(loading, cv::Point(50, 220), cv::Point(50 + (int)(500 * ratio), 240), cv::Scalar(0, 255, 0), -1);
                            cv::imshow("Multi-Cam Preview", loading);
                            cv::waitKey(1);
                        };
                        
                        vector<thread> compile_threads;
                        for (int i = 0; i < cam_ctxs.size(); ++i) {
                            string out_jpg_dir = cam_ctxs[i]->save_base_dir + "/record_" + current_record_timestr + "/cam_" + to_string(cam_ctxs[i]->index);
                            compile_threads.emplace_back(
                                convertRawToJpgWorker, 
                                cam_ctxs[i]->temp_dir, 
                                out_jpg_dir, 
                                final_entries_lists[i], 
                                std::ref(global_processed), 
                                cam_ctxs[i]->is_mono
                            );
                        }
                        
                        int last_processed = 0, same_count = 0;  
                        while (true) {
                            draw_progress_ui();
                            int current_processed = global_processed.load();
                            if (current_processed >= total_frames_all_cams) break;
                            if (current_processed == last_processed) {
                                if (++same_count > 100) break;
                            } else { same_count = 0; last_processed = current_processed; }
                            this_thread::sleep_for(chrono::milliseconds(50));
                        }
                        for (auto& t : compile_threads) if (t.joinable()) t.join();
                        global_processed = total_frames_all_cams;  
                        draw_progress_ui();
                        cv::waitKey(500);
                    }
                }

                cout << "\n[Info] Ready for next capture." << endl;
                is_dumping = false;
                while (cv::waitKey(1) >= 0); 
            }
        }

        char key = (char)cv::waitKey(50); 
        if (key == 'q' || key == 27) running = false;
        else if (key == 'r') {
            if (!is_recording && !is_dumping) { 
                auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                char buf[64]; strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
                current_record_timestr = string(buf);

                record_start_time = std::chrono::steady_clock::now(); 
                cout << "\n[Info] ASYNC CAPTURE STARTED: " << current_record_timestr << endl;

                for (auto& ctx : cam_ctxs) {
                    string batch_raw = ctx->save_base_dir + "/temp_raw_" + current_record_timestr + "/cam_" + to_string(ctx->index);
                    fs::create_directories(batch_raw);
                    ctx->temp_dir = batch_raw;
                    ctx->log_file_path = ctx->save_base_dir + "/record_" + current_record_timestr + "_cam_" + to_string(ctx->index) + ".txt";
                    
                    {
                        lock_guard<mutex> lock(ctx->copy_mtx);
                        while (!ctx->copy_queue.empty()) ctx->copy_queue.pop();
                    }

                    ctx->recorded_frames = 0; 
                    ctx->dump_ready = false;  
                    ctx->recording = true; 
                }
                is_recording = true;
            }
        }
        else if (key == 's') { 
            if (is_recording) {
                cout << "\n[Info] Manual stop requested. Terminating capture early..." << endl;
                for (auto& ctx : cam_ctxs) { ctx->recording = false; ctx->dump_ready = true; }
            }
        }
        else if (key == ' ') { 
            if (!is_recording && !is_dumping) { 
                int counter = 0;
                if (!cam_ctxs.empty()) counter = getNextCalibCounter(cam_ctxs[0]->save_base_dir);
                std::stringstream ss; ss << std::setw(2) << std::setfill('0') << counter;
                string calib_str = ss.str();
                cout << "\n[Photo] Capturing calibration set " << calib_str << endl;

                for (auto& ctx : cam_ctxs) {
                    cv::Mat snapshot;
                    { lock_guard<mutex> lock(ctx->frame_mtx); snapshot = ctx->latest_frame.clone(); }
                    if (!snapshot.empty()) {
                        cv::Mat out_snapshot;
                        if (ctx->is_mono) {
                            out_snapshot = snapshot;
                        } else {
                            cv::cvtColor(snapshot, out_snapshot, cv::COLOR_BayerRG2RGB);
                        }
                        string fn = ctx->save_base_dir + "/calib_cam_" + to_string(ctx->index) + "_" + calib_str + ".jpg";
                        cv::imwrite(fn, out_snapshot);
                        cout << "  -> Saved " << fn << endl;
                    }
                }
            }
        }
    }

    cout << "[System] Shutting down threads..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->copy_cv.notify_all();
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->copy_thread.joinable()) ctx->copy_thread.join();
    }
    
    cv::destroyAllWindows();
    Pylon::PylonTerminate();
    return 0;
}