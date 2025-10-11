#pragma once

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace gazeestimation {

class BaslerCamera {
public:
    BaslerCamera();
    ~BaslerCamera();

    bool open();                           // 打开相机
    void close();                          // 关闭相机

    void setFrameRate(double fps);         // 设置帧率
    void setGain(double gain);             // 设置增益
    void setGamma(double gamma);           // 设置Gamma
    void setExposureTime(double microseconds); // 设置曝光时间

    cv::Mat grabFrame();                   // 获取单帧

    void startRecording(const std::string& filename, int fourcc = cv::VideoWriter::fourcc('m','p','4','v'), double fps = 100.0);
    void writeFrame(const cv::Mat& frame);
    void stopRecording();

private:
    Pylon::PylonAutoInitTerm autoInit_;    // Pylon 初始化
    Pylon::CInstantCamera camera_;
    bool isOpen_;
    bool recording_;
    cv::VideoWriter writer_;
};

} // namespace gazeestimation
