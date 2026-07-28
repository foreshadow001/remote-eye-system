#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <functional> 

using namespace Pylon;

namespace gazeestimation {

enum class TriggerMode { Software, Hardware };

enum class GrabResult { OK, TIMEOUT, ERROR_ };

struct FrameMeta {
    int64_t blockID = 0;      
    int64_t timestamp = 0;    
    double sys_time_ms = 0.0; 
};

// 【修改点】签名改为传递底层的 CBaslerUniversalGrabResultPtr，彻底避免提前拷贝
using FrameCallback = std::function<void(const Pylon::CBaslerUniversalGrabResultPtr&, FrameMeta)>;

class BaslerCamera : public Pylon::CBaslerUniversalImageEventHandler, public Pylon::CConfigurationEventHandler {
public:
    explicit BaslerCamera(const std::string& serialNumber);
    virtual ~BaslerCamera();

    bool open(TriggerMode mode = TriggerMode::Software);
    bool start();
    void close();

    void setFrameCallback(FrameCallback cb) { callback_ = cb; }

    virtual void OnImageGrabbed(CBaslerUniversalInstantCamera& camera, const CBaslerUniversalGrabResultPtr& ptrGrabResult) override;

    void setFrameRate(double fps);
    void setGain(double gain);
    void setGamma(double gamma);
    void setExposureTime(double microseconds);
    void setMaxNumBuffer(int n) { maxNumBuffer_ = n; }

    std::string getSerialNumber() const { return serialNumber_; }
    bool isMono() const { return isMono_; }

private:
    Pylon::PylonAutoInitTerm autoInit_;
    Pylon::CBaslerUniversalInstantCamera camera_;
    std::string serialNumber_;
    bool isOpen_ = false;
    bool isMono_ = false;
    TriggerMode currentMode_ = TriggerMode::Software;

    Pylon::CImageFormatConverter converter_;
    int maxNumBuffer_ = 150;

    bool recording_;    
    bool writerInitialized_;  
    std::string videoFilename_; 
    double videoFps_;           
    cv::VideoWriter writer_;

    FrameCallback callback_;
};

static inline double ms(const std::chrono::steady_clock::time_point& a, const std::chrono::steady_clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace gazeestimation