#pragma once

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <chrono>

namespace gazeestimation {

// 定义触发模式枚举
enum class TriggerMode {
    Software,   // 自由采集 / 软触发
    Hardware    // 外部硬件触发 (Line1)
};

enum class GrabResult {
    OK,             // 拿到一帧
    TIMEOUT,        // 等待触发（正常）
    ERROR_          // 真正错误
};

// 定义帧元数据结构
struct FrameMeta {
    int64_t blockID = 0;      // 相机内部帧计数器
    int64_t timestamp = 0;    // 相机内部时间戳 (ns)
    double sys_time_ms = 0.0; // 系统接收时间 (辅助调试)
};

class BaslerCamera {
public:
    explicit BaslerCamera(const std::string& serialNumber);
    ~BaslerCamera();

    bool open(TriggerMode mode = TriggerMode::Software);
    bool start();
    void close();

    // 相机参数设置
    void setFrameRate(double fps);
    void setGain(double gain);
    void setGamma(double gamma);
    void setExposureTime(double microseconds);

    GrabResult grabFrame(cv::Mat& out_frame, FrameMeta& out_meta);

    // 录像相关
    void startRecording(const std::string& filename, double fps = 30.0);
    void writeFrame(const cv::Mat& frame);
    void stopRecording();

    std::string getSerialNumber() const { return serialNumber_; }

private:
    Pylon::PylonAutoInitTerm autoInit_;
    Pylon::CInstantCamera camera_;
    std::string serialNumber_;
    bool isOpen_ = false;
    bool isMono_ = false;
    TriggerMode currentMode_ = TriggerMode::Software;

    Pylon::CImageFormatConverter converter_;
    
    // 录像控制变量 (为延迟初始化添加)
    bool recording_;          // 整体录像状态
    bool writerInitialized_;  // VideoWriter 是否已经成功初始化
    std::string videoFilename_; // 暂存文件名
    double videoFps_;           // 暂存帧率
    cv::VideoWriter writer_;
};

static inline double ms(
    const std::chrono::steady_clock::time_point& a,
    const std::chrono::steady_clock::time_point& b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace gazeestimation