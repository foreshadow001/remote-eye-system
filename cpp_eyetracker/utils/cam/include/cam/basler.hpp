#pragma once

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace gazeestimation {

class BaslerCamera {
public:
    explicit BaslerCamera(const std::string& serialNumber);
    ~BaslerCamera();

    bool open();
    void close();

    // 相机参数设置
    void setFrameRate(double fps);
    void setGain(double gain);
    void setGamma(double gamma);
    void setExposureTime(double microseconds);

    cv::Mat grabFrame();

    // 录像相关
    void startRecording(const std::string& filename, double fps = 30.0);
    void writeFrame(const cv::Mat& frame);
    void stopRecording();

    std::string getSerialNumber() const { return serialNumber_; }

private:
    Pylon::PylonAutoInitTerm autoInit_;
    Pylon::CInstantCamera camera_;
    std::string serialNumber_;
    bool isOpen_;
    
    // 录像控制变量 (为延迟初始化添加)
    bool recording_;          // 整体录像状态
    bool writerInitialized_;  // VideoWriter 是否已经成功初始化
    std::string videoFilename_; // 暂存文件名
    double videoFps_;           // 暂存帧率
    cv::VideoWriter writer_;
};

} // namespace gazeestimation