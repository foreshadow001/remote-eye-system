#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <functional> // 新增

using namespace Pylon;

namespace gazeestimation {

// 定义触发模式枚举
enum class TriggerMode {
    Software,   
    Hardware    
};

enum class GrabResult {
    OK,             
    TIMEOUT,        
    ERROR_          
};

// 定义帧元数据结构
struct FrameMeta {
    int64_t blockID = 0;      
    int64_t timestamp = 0;    
    double sys_time_ms = 0.0; 
};

// 定义回调函数类型 (向外传递图像和元数据)
using FrameCallback = std::function<void(const cv::Mat&, FrameMeta)>;

// 让 BaslerCamera 继承 Pylon::CImageEventHandler  CBaslerUniversalImageEventHandler
class BaslerCamera : public Pylon::CBaslerUniversalImageEventHandler,public Pylon::CConfigurationEventHandler{
public:
    explicit BaslerCamera(const std::string& serialNumber);
    virtual ~BaslerCamera();

    bool open(TriggerMode mode = TriggerMode::Software);
    bool start();
    void close();

    // 绑定外部回调函数
    void setFrameCallback(FrameCallback cb) { callback_ = cb; }

    // 重写 Pylon 的回调函数
    virtual void OnImageGrabbed(CBaslerUniversalInstantCamera& camera, const CBaslerUniversalGrabResultPtr& ptrGrabResult) override;

    // 相机参数设置
    void setFrameRate(double fps);
    void setGain(double gain);
    void setGamma(double gamma);
    void setExposureTime(double microseconds);

    // 录像相关
    void startRecording(const std::string& filename, double fps = 30.0);
    void writeFrame(const cv::Mat& frame);
    void stopRecording();

    std::string getSerialNumber() const { return serialNumber_; }

private:
    Pylon::PylonAutoInitTerm autoInit_;
    Pylon::CBaslerUniversalInstantCamera camera_;
    std::string serialNumber_;
    bool isOpen_ = false;
    bool isMono_ = false;
    TriggerMode currentMode_ = TriggerMode::Software;

    Pylon::CImageFormatConverter converter_;
    
    // 录像控制变量
    bool recording_;          
    bool writerInitialized_;  
    std::string videoFilename_; 
    double videoFps_;           
    cv::VideoWriter writer_;

    // 保存外部传入的回调逻辑
    FrameCallback callback_;
};

static inline double ms(
    const std::chrono::steady_clock::time_point& a,
    const std::chrono::steady_clock::time_point& b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace gazeestimation