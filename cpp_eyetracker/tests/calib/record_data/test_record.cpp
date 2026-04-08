#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#ifdef _MSC_VER
#pragma comment(lib, "user32.lib")
#endif

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
#include <algorithm>
#include <sstream>
#include <unordered_set> // 新增，用于存储有效时间戳
#include <pylon/PylonIncludes.h>

#include "cam/basler.hpp"
#include "cfg/config.hpp"
#include "logger/logger.hpp" 

namespace fs = std::filesystem;
using namespace std;
using namespace gazeestimation;

// =========================================================================
// ========================== 1. 标定点图像生成逻辑 ==========================
// =========================================================================

std::pair<int, int> getScreenResolution() {
    HMODULE user32 = LoadLibraryA("user32.dll");
    if (user32) {
        typedef BOOL(WINAPI *SetProcDPIAware_t)();
        auto p = (SetProcDPIAware_t)GetProcAddress(user32, "SetProcessDPIAware");
        if (p) p();
        FreeLibrary(user32);
    }
    int w = GetSystemMetrics(SM_CXSCREEN);
    int h = GetSystemMetrics(SM_CYSCREEN);
    return {w > 0 ? w : 1920, h > 0 ? h : 1080};
}

void drawMarkerPattern(cv::Mat& img, const cv::Point& center, const std::string& bg, int scale) {
    int cross_half = 10 * scale;     
    int circle_r   = 8 * scale;      
    int dot_r      = 4 * scale;      
    int thick      = std::max(1, scale);

    cv::Scalar cross_color = (bg == "dark") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 128, 0);
    cv::Scalar circle_color = (bg == "dark") ? cv::Scalar(255, 255, 0) : cv::Scalar(128, 128, 0); 
    cv::Scalar dot_color(0, 0, 255); 

    // cv::line(img, cv::Point(center.x - cross_half, center.y), cv::Point(center.x + cross_half, center.y), cross_color, thick, cv::LINE_AA);
    // cv::line(img, cv::Point(center.x, center.y - cross_half), cv::Point(center.x, center.y + cross_half), cross_color, thick, cv::LINE_AA);
    // cv::circle(img, center, circle_r, circle_color, thick, cv::LINE_AA);
    cv::circle(img, center, dot_r, dot_color, cv::FILLED, cv::LINE_AA);
}

bool generateCalibrationImages(int rows, int cols, int margin_x, int margin_y,
                               const std::string& save_dir, int width, int height, int marker_scale) {
    if (rows <= 0 || cols <= 0) return false;
    if (width <= 0 || height <= 0) {
        auto [w, h] = getScreenResolution();
        width = w; height = h;
    }
    fs::create_directories(save_dir);
    std::vector<std::string> bgs = {"dark", "light"};
    for (const auto& bg : bgs) {
        int index = 0;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                double x = (cols == 1) ? width / 2.0 : margin_x + c * (double)(width - 2 * margin_x) / (cols - 1);
                double y = (rows == 1) ? height / 2.0 : margin_y + r * (double)(height - 2 * margin_y) / (rows - 1);

                cv::Mat img(height, width, CV_8UC3, bg == "dark" ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255));
                drawMarkerPattern(img, cv::Point(cvRound(x), cvRound(y)), bg, marker_scale);

                char filename[256];
                snprintf(filename, sizeof(filename), "%s_%02d_%d_%d.jpg", bg.c_str(), index++, cvRound(x), cvRound(y));
                cv::imwrite((fs::path(save_dir) / filename).string(), img);
            }
        }
    }
    return true;
}

struct CalibPoint {
    int idx;
    int x;
    int y;
    std::string filepath;
};

// =========================================================================
// ======================= 2. 相机录制与处理核心结构 =========================
// =========================================================================

enum class CamStatus { INIT, OPENED, WAITING_TRIGGER, STREAMING, ERROR_ };

struct FrameTask {
    string filename;
    cv::Mat image;
    FrameMeta meta; 
};

struct LogEntry {
    string filename;
    int64_t blockID;
    int64_t timestamp;
    int width;   
    int height;  
};

struct CameraContext {
    int index;
    string id;
    BaslerCamera cam{""};
    
    thread capture_thread;
    thread writer_thread;
    atomic<bool> running{true};
    atomic<bool> recording{false};
    atomic<int> target_frames{0}; 

    cv::Mat latest_frame;
    FrameMeta latest_meta;
    mutex frame_mtx; 
    
    string temp_dir;      
    string log_file_path;
    ofstream log_stream;

    queue<FrameTask> write_queue; 
    mutex queue_mtx;
    atomic<bool> is_writing{false};
    condition_variable queue_cv;
    
    atomic<int> captured_frames{0};
    atomic<int> recorded_frames{0};

    atomic<CamStatus> status{CamStatus::INIT};
    string status_msg = "Cam is initializing";

    int64_t frame_offset = 0;
    bool offset_initialized = false;
    static atomic<int64_t> master_first_id; 
    static atomic<bool> master_set;

    CameraContext(int idx, string cam_id) : index(idx), id(cam_id), cam(cam_id) {}
};

atomic<int64_t> CameraContext::master_first_id(-1);
atomic<bool> CameraContext::master_set(false);
vector<shared_ptr<CameraContext>> cam_ctxs;

// CSV 修改更新函数：如果已存在相同的点位ID，直接覆盖；否则追加。
void updateMappingCSV(const string& filepath, const string& timestr, int point_idx, int x, int y) {
    vector<string> lines;
    ifstream in(filepath);
    bool found = false;
    if (in.is_open()) {
        string line;
        while (getline(in, line)) {
            if (line.empty()) continue;
            stringstream ss(line);
            string t_str, idx_str;
            getline(ss, t_str, ',');
            getline(ss, idx_str, ',');
            
            if (idx_str.empty()) { 
                lines.push_back(line); 
                continue; 
            }
            try {
                if (stoi(idx_str) == point_idx) {
                    // 找到相同点位，替换行内容
                    stringstream newline;
                    newline << timestr << "," << point_idx << "," << x << "," << y;
                    lines.push_back(newline.str());
                    found = true;
                } else {
                    lines.push_back(line);
                }
            } catch(...) {
                lines.push_back(line); 
            }
        }
        in.close();
    }
    
    if (!found) {
        stringstream newline;
        newline << timestr << "," << point_idx << "," << x << "," << y;
        lines.push_back(newline.str());
    }
    
    ofstream out(filepath, ios::trunc);
    for (const auto& l : lines) {
        out << l << "\n";
    }
}

// ================== 相机工作线程与转码逻辑 ==================

void writerWorker(shared_ptr<CameraContext> ctx) {
    while (ctx->running) {
        FrameTask task;
        {
            unique_lock<mutex> lock(ctx->queue_mtx);
            ctx->queue_cv.wait(lock, [&] { return !ctx->write_queue.empty() || !ctx->running; });
            if (!ctx->running && ctx->write_queue.empty()) break;
            task = ctx->write_queue.front();
            ctx->write_queue.pop();
            ctx->is_writing = true; 
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

        ctx->is_writing = false; 
        if (ctx->write_queue.empty()) {
            ctx->queue_cv.notify_all();
        }
    }
}

void captureWorker(shared_ptr<CameraContext> ctx, double fps, double gain, double gamma, double exp_time, bool use_hw_trigger) {
    TriggerMode mode = use_hw_trigger ? TriggerMode::Hardware : TriggerMode::Software;
    if (!ctx->cam.open(mode)) {  
        ctx->status = CamStatus::ERROR_;
        ctx->status_msg = "OPEN FAILED";
        return;
    }

    try {
        if (!use_hw_trigger) ctx->cam.setFrameRate(fps);  
        ctx->cam.setGain(gain);                           
        ctx->cam.setGamma(gamma);                         
        ctx->cam.setExposureTime(exp_time);               
    } catch (...) {}

    struct GrabState {
        int frame_seq = 0;
        int64_t frame_counter = 0;
    };
    auto state = make_shared<GrabState>();

    ctx->cam.setFrameCallback([ctx, state](const cv::Mat& frame, FrameMeta meta) {
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
            lock_guard<mutex> lock(ctx->frame_mtx);
            frame.copyTo(ctx->latest_frame); 
            ctx->latest_meta = meta;
            ctx->captured_frames++;
        }

        if (ctx->recording) {
            if (state->frame_seq < ctx->target_frames.load()) {
                std::stringstream ss;
                ss << ctx->temp_dir << "/" << std::setw(6) << std::setfill('0') << state->frame_seq << ".raw";
                
                {
                    lock_guard<mutex> lock(ctx->queue_mtx);
                    if (ctx->write_queue.size() < 800) {
                        ctx->write_queue.push({ss.str(), frame.clone(), meta});
                    }
                }
                ctx->queue_cv.notify_one();
                
                state->frame_seq++;
                ctx->recorded_frames = state->frame_seq; 
            }
        } else {
            state->frame_seq = 0; 
            ctx->recorded_frames = 0;
        }
    });

    if (!ctx->cam.start()) return;
    ctx->status = CamStatus::OPENED;
    ctx->status_msg = use_hw_trigger ? "HW WAITING" : "STREAMING";

    while (ctx->running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ctx->cam.close();
}

vector<LogEntry> parseLogFile(const string& log_path) {
    vector<LogEntry> entries;
    ifstream infile(log_path);
    string line;
    while (getline(infile, line)) {
        stringstream ss(line);
        string segment;
        vector<string> parts;
        while (getline(ss, segment, ',')) parts.push_back(segment);
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
    fs::create_directories(out_jpg_dir);
    for (const auto& entry : valid_entries) {
        string raw_path = temp_raw_dir + "/" + entry.filename;
        cv::Mat raw_img(entry.height, entry.width, CV_8UC1);
        ifstream in_raw(raw_path, ios::binary);
        if (in_raw) {
            in_raw.read(reinterpret_cast<char*>(raw_img.data), raw_img.total());
            in_raw.close();
            
            string jpg_filename = entry.filename;
            size_t dot_pos = jpg_filename.find_last_of('.');
            if (dot_pos != string::npos) {
                jpg_filename = jpg_filename.substr(0, dot_pos) + ".jpg";
            } else {
                jpg_filename += ".jpg";
            }
            
            string jpg_path = out_jpg_dir + "/" + jpg_filename;
            cv::imwrite(jpg_path, raw_img);
        }
        global_processed++;
    }
}


// =========================================================================
// ============================= 3. 主函数 =================================
// =========================================================================

int main() {
    cout << "=== Record with Calibration Points Tool ===" << endl;
    
    Cfg cfg;
    Pylon::PylonInitialize();
    
    // 读取相机配置
    vector<string> camera_ids = cfg["test_multi_cam"]["cam_indices"].as<vector<string>>();
    bool use_hw_trigger = cfg["test_multi_cam"]["hardware_trigger"].as<bool>();
    double target_fps   = cfg["test_multi_cam"]["fps"].as<double>();
    double gain         = cfg["test_multi_cam"]["gain"].as<double>();
    double gamma        = cfg["test_multi_cam"]["gamma"].as<double>();
    double exp_time     = cfg["test_multi_cam"]["exposure_time"].as<double>();
    string save_base_dir = cfg["test_record"]["save_dir"].as<std::string>();
    
    // 读取行为配置
    bool write_jpg     = cfg["test_record"]["write_jpg"].as<bool>();
    int target_frames  = cfg["test_record"]["target_frames"].as<int>(); 
    bool remove_false  = cfg["test_record"]["remove_false"].as<bool>(); // 是否清理多余的文件

    // 读取标定点配置
    int rows = cfg["calib_points"]["rows"].as<int>();
    int cols = cfg["calib_points"]["cols"].as<int>();
    int margin_x = cfg["calib_points"]["margin_x"].as<int>();
    int margin_y = cfg["calib_points"]["margin_y"].as<int>();
    std::string calib_img_dir = cfg["calib_points"]["save_dir"].as<std::string>();
    int marker_scale = cfg["calib_points"]["marker_scale"].as<int>();
    std::string theme = cfg["calib_points"]["theme"].as<std::string>();

    std::filesystem::path save_folder_path(save_base_dir);
    std::filesystem::create_directories(save_folder_path);

    // 1. 生成或验证标定图片
    bool ok = generateCalibrationImages(rows, cols, margin_x, margin_y, calib_img_dir, 0, 0, marker_scale);
    if (!ok) {
        cerr << "[Error] Failed to generate calibration images!" << endl;
        return -1;
    }

    // 解析出需要的标定点列表
    std::vector<CalibPoint> calib_points;
    for (const auto& entry : fs::directory_iterator(calib_img_dir)) {
        if (!entry.is_regular_file()) continue;
        std::string filename = entry.path().filename().string();
        if (filename.rfind(theme + "_", 0) == 0) { 
            int idx = 0, x = 0, y = 0;
            if (sscanf(filename.c_str(), "%*[^_]_%d_%d_%d.jpg", &idx, &x, &y) == 3) {
                calib_points.push_back({idx, x, y, entry.path().string()});
            }
        }
    }
    
    std::sort(calib_points.begin(), calib_points.end(),[](const CalibPoint& a, const CalibPoint& b){
        return a.idx < b.idx;
    });

    if(calib_points.empty()){
        cerr << "[Error] No calibration points found for theme: " << theme << endl;
        return -1;
    }

    // 2. 启动相机线程
    for (int i = 0; i < camera_ids.size(); ++i) {
        auto ctx = make_shared<CameraContext>(i, camera_ids[i]);
        cam_ctxs.push_back(ctx);
        ctx->writer_thread = thread(writerWorker, ctx);
        ctx->capture_thread = thread(captureWorker, ctx, target_fps, gain, gamma, exp_time, use_hw_trigger);
    }

    // 3. 准备全屏窗口显示点位
    const string win_name = "Calibration Window";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::setWindowProperty(win_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    cout << "Ready. Controls:\n"
         << "  [Backspace] Previous Point\n"
         << "  [Enter] Next Point\n"
         << "  [Space] Start Record (Auto Stop at " << target_frames << " frames)\n"
         << "  [q / ESC] Quit\n";

    bool is_recording = false;
    bool running = true;
    int current_img_idx = 0;
    string current_record_timestr;
    std::chrono::steady_clock::time_point record_start_time;

    string mapping_file_path = save_base_dir + "/video_point_mapping.csv";

    // ================== 主循环 ==================
    while (running) {
        cv::Mat display_img = cv::imread(calib_points[current_img_idx].filepath);
        if (display_img.empty()) {
            display_img = cv::Mat::zeros(1080, 1920, CV_8UC3); 
        }

        if (is_recording) {
            double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - record_start_time).count();
            int max_recorded = 0;
            for (auto& ctx : cam_ctxs) max_recorded = std::max(max_recorded, ctx->recorded_frames.load());

            char time_buf[128];
            snprintf(time_buf, sizeof(time_buf), "REC - %.1fs [%d/%d]", elapsed_s, max_recorded, target_frames);
            cv::putText(display_img, time_buf, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2);
        } else {
            string info = "Point " + to_string(current_img_idx + 1) + " / " + to_string(calib_points.size()) + " | Ready";
            cv::putText(display_img, info, cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(100,200,100), 2);
        }

        cv::imshow(win_name, display_img);

        char key = (char)cv::waitKey(50); 

        if (key == 'q' || key == 27) {
            running = false;
        }
        else if (key == 8 && !is_recording) { 
            if (current_img_idx > 0) current_img_idx--;
        }
        else if ((key == 13 || key == 10) && !is_recording) {  
            if (current_img_idx < calib_points.size() - 1) current_img_idx++;
        }
        else if (key == ' ' && !is_recording) { 
            auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            char buf[64];
            strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&t));
            current_record_timestr = string(buf);
            record_start_time = std::chrono::steady_clock::now(); 

            cout << "\n[Info] Start Recording Batch: " << current_record_timestr 
                 << " for Point Index: " << calib_points[current_img_idx].idx << endl;

            for (auto& ctx : cam_ctxs) {
                string batch_raw = save_base_dir + "/temp_raw_" + current_record_timestr + "/cam_" + to_string(ctx->index);
                fs::create_directories(batch_raw);
                ctx->temp_dir = batch_raw;
                
                ctx->log_file_path = save_base_dir + "/record_" + current_record_timestr + "_cam_" + to_string(ctx->index) + ".txt";
                ctx->log_stream.open(ctx->log_file_path);
                
                ctx->target_frames = target_frames; 
                ctx->recording = true; 
            }
            is_recording = true;
        }

        // 自动停止检测
        if (is_recording) {
            bool all_finished = true;
            for (auto& ctx : cam_ctxs) {
                if (ctx->recorded_frames < target_frames) {
                    all_finished = false; 
                    break;
                }
            }

            if (all_finished) {
                cout << "\n[Info] Target frames reached. Stopping recording... waiting for queue flush." << endl;
                is_recording = false;

                for (auto& ctx : cam_ctxs) ctx->recording = false;

                cv::Mat wait_img = display_img.clone();
                cv::putText(wait_img, "Flushing Write Queue...", cv::Point(30, 90), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
                cv::imshow(win_name, wait_img);
                cv::waitKey(1);

                for (auto& ctx : cam_ctxs) {
                    {
                        unique_lock<mutex> lk(ctx->queue_mtx);
                        ctx->queue_cv.wait(lk, [&]{ return ctx->write_queue.empty() && !ctx->is_writing.load(); });
                    }
                    if (ctx->log_stream.is_open()) ctx->log_stream.close();
                }

                vector<vector<LogEntry>> all_logs;
                for (int i = 0; i < cam_ctxs.size(); ++i) {
                    auto logs = parseLogFile(cam_ctxs[i]->log_file_path);
                    all_logs.push_back(logs);
                    // （日志打印省略细节）
                    cout << "[Cam " << cam_ctxs[i]->index << "] Saved: " << logs.size() << endl;
                }

                // 更新映射文件（如果有重复录制直接覆盖）
                updateMappingCSV(mapping_file_path, current_record_timestr, 
                                 calib_points[current_img_idx].idx, 
                                 calib_points[current_img_idx].x, 
                                 calib_points[current_img_idx].y);

                if (write_jpg) {
                    cout << "\n[Info] Analyzing timestamps for JPG synchronization..." << endl;
                    int64_t global_start_idx = 0;
                    int64_t global_end_idx = 9999999999LL; 

                    for (auto& logs : all_logs) {
                        if (logs.empty()) continue;
                        if (logs.front().blockID > global_start_idx) global_start_idx = logs.front().blockID;
                        if (logs.back().blockID < global_end_idx) global_end_idx = logs.back().blockID;
                    }

                    vector<vector<LogEntry>> final_entries_lists(cam_ctxs.size()); 
                    int total_frames_all_cams = 0;

                    for (int i = 0; i < cam_ctxs.size(); ++i) {
                        for (const auto& entry : all_logs[i]) {
                            if (entry.blockID >= global_start_idx && entry.blockID <= global_end_idx) {
                                final_entries_lists[i].push_back(entry);
                            }
                        } 
                        total_frames_all_cams += final_entries_lists[i].size(); 
                    } 
                    
                    std::atomic<int> global_processed{0};
                    auto draw_progress_ui = [&]() {
                        cv::Mat loading = display_img.clone(); 
                        int processed = global_processed.load();
                        string text = "Saving JPGs... " + to_string(processed) + "/" + to_string(total_frames_all_cams);
                        cv::putText(loading, text, cv::Point(30, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                        
                        int bar_x = 30, bar_y = 110, bar_w = 500, bar_h = 20;
                        float ratio = total_frames_all_cams > 0 ? (std::min)(1.0f, (float)processed / total_frames_all_cams) : 0.0f;
                        cv::rectangle(loading, cv::Point(bar_x, bar_y), cv::Point(bar_x + bar_w, bar_y + bar_h), cv::Scalar(255, 255, 255), 1);
                        if (ratio > 0) cv::rectangle(loading, cv::Point(bar_x, bar_y), cv::Point(bar_x + (int)(bar_w * ratio), bar_y + bar_h), cv::Scalar(0, 255, 0), -1);
                        
                        cv::imshow(win_name, loading);
                        cv::waitKey(1);
                    };
                    
                    vector<thread> compile_threads;
                    for (int i = 0; i < cam_ctxs.size(); ++i) {
                        string out_jpg_dir = save_base_dir + "/record_" + current_record_timestr + "/cam_" + to_string(cam_ctxs[i]->index);
                        compile_threads.emplace_back(convertRawToJpgWorker, cam_ctxs[i]->temp_dir, out_jpg_dir, final_entries_lists[i], std::ref(global_processed));
                    }
                    
                    int last_processed = 0, same_count = 0;  
                    while (true) {
                        draw_progress_ui();
                        int current_processed = global_processed.load();
                        if (current_processed >= total_frames_all_cams) break;
                        if (current_processed == last_processed) {
                            if (++same_count > 100) break;
                        } else {
                            same_count = 0;
                            last_processed = current_processed;
                        }
                        this_thread::sleep_for(chrono::milliseconds(50));
                    }
                    
                    for (auto& t : compile_threads) if (t.joinable()) t.join();
                    global_processed = total_frames_all_cams;  
                    draw_progress_ui();
                    cv::waitKey(200);
                }
            }
        }
    }

    // 4. 清理资源关闭线程
    cout << "\n[System] Shutting down threads..." << endl;
    for (auto& ctx : cam_ctxs) {
        ctx->running = false;
        ctx->queue_cv.notify_all(); 
        if (ctx->capture_thread.joinable()) ctx->capture_thread.join();
        if (ctx->writer_thread.joinable()) ctx->writer_thread.join();
    }
    
    cv::destroyAllWindows();
    Pylon::PylonTerminate();

    // =========================================================================
    // 5. 根据 remove_false 策略，退出前清理因覆盖而产生的多余/废弃数据文件夹
    // =========================================================================
    if (remove_false) {
        cout << "[System] Checking and cleaning up overwritten/obsolete records..." << endl;
        
        // 读取最终映射表，获取所有有效的 timestr
        std::unordered_set<std::string> valid_timestrs;
        ifstream in(mapping_file_path);
        if (in.is_open()) {
            string line;
            while (getline(in, line)) {
                if (line.empty()) continue;
                stringstream ss(line);
                string t_str;
                getline(ss, t_str, ',');
                if (!t_str.empty()) valid_timestrs.insert(t_str);
            }
            in.close();
        }

        // 遍历目录比对删除废弃文件
        if (fs::exists(save_folder_path)) {
            for (const auto& entry : fs::directory_iterator(save_folder_path)) {
                string filename = entry.path().filename().string();
                string t_str = "";

                // 判断是否是我们生成的日志/图片目录
                if (filename.rfind("temp_raw_", 0) == 0 && filename.length() >= 24) {
                    t_str = filename.substr(9, 15); // 从 "temp_raw_YYYYMMDD_HHMMSS" 中提取
                } else if (filename.rfind("record_", 0) == 0 && filename.length() >= 22) {
                    t_str = filename.substr(7, 15); // 从 "record_YYYYMMDD_HHMMSS..." 中提取
                }

                // 校验提取出的字符串是否符合时间戳格式(长度15，中间带下划线)
                if (!t_str.empty() && t_str.length() == 15 && t_str[8] == '_') {
                    // 如果不在最终映射表里，说明是作废数据，删掉
                    if (valid_timestrs.find(t_str) == valid_timestrs.end()) {
                        try {
                            fs::remove_all(entry.path());
                            cout << "  Deleted unused record: " << filename << endl;
                        } catch (const std::exception& e) {
                            cerr << "  [Error] Failed to delete " << filename << ": " << e.what() << endl;
                        }
                    }
                }
            }
        }
        cout << "[System] Cleanup complete." << endl;
    }

    return 0;
}