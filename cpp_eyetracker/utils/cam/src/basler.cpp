#include "cam/basler.hpp"
#include <iostream>

using namespace std;
using namespace Pylon;
using namespace cv;
using namespace GenApi;

namespace gazeestimation {

BaslerCamera::BaslerCamera()
    : camera_(CTlFactory::GetInstance().CreateFirstDevice()),
      isOpen_(false),
      recording_(false) {}

BaslerCamera::~BaslerCamera() {
    stopRecording();
    close();
}

bool BaslerCamera::open() {
    try {
        camera_.Open();
        isOpen_ = true;
        std::cout << "[BaslerCamera] Camera opened: "
                  << camera_.GetDeviceInfo().GetModelName() << std::endl;
        return true;
    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] Failed to open camera: "
                  << e.GetDescription() << std::endl;
        isOpen_ = false;
        return false;
    }
}

void BaslerCamera::close() {
    if (camera_.IsGrabbing()) {
        camera_.StopGrabbing();
    }
    if (isOpen_) {
        camera_.Close();
        isOpen_ = false;
    }
}

void BaslerCamera::setFrameRate(double fps) {
    if (!isOpen_) return;
    try {
        CBooleanPtr enableNode(camera_.GetNodeMap().GetNode("AcquisitionFrameRateEnable"));
        CFloatPtr fpsNode(camera_.GetNodeMap().GetNode("AcquisitionFrameRate"));
        if (enableNode && fpsNode) {
            enableNode->SetValue(true);
            fpsNode->SetValue(fps);
            cout << "[BaslerCamera] Frame rate set to " << fps << " fps" << endl;
        }
    } catch (const GenericException& e) {
        cerr << "[BaslerCamera] setFrameRate error: " << e.GetDescription() << endl;
    }
}

void BaslerCamera::setGain(double gain) {
    if (!isOpen_) return;
    try {
        CFloatPtr gainNode(camera_.GetNodeMap().GetNode("Gain"));
        if (gainNode) gainNode->SetValue(gain);
    } catch (const GenericException& e) {
        cerr << "[BaslerCamera] setGain error: " << e.GetDescription() << endl;
    }
}

void BaslerCamera::setGamma(double gamma) {
    if (!isOpen_) return;
    try {
        CFloatPtr gammaNode(camera_.GetNodeMap().GetNode("Gamma"));
        if (gammaNode) gammaNode->SetValue(gamma);
    } catch (const GenericException& e) {
        cerr << "[BaslerCamera] setGamma error: " << e.GetDescription() << endl;
    }
}

void BaslerCamera::setExposureTime(double microseconds) {
    if (!isOpen_) return;
    try {
        CFloatPtr exposure(camera_.GetNodeMap().GetNode("ExposureTime"));
        if (exposure) exposure->SetValue(microseconds);
        cout << "[BaslerCamera] Exposure set to " << microseconds << " µs" << endl;
    } catch (const GenericException& e) {
        cerr << "[BaslerCamera] setExposureTime error: " << e.GetDescription() << endl;
    }
}

cv::Mat BaslerCamera::grabFrame() {
    cv::Mat image;
    if (!isOpen_) return image;

    try {
        if (!camera_.IsGrabbing())
            camera_.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        CGrabResultPtr ptrGrabResult;
        camera_.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded()) {
            static CImageFormatConverter converter;
            converter.OutputPixelFormat = PixelType_BGR8packed;
            CPylonImage pylonImage;
            converter.Convert(pylonImage, ptrGrabResult);
            image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                            CV_8UC3, (uint8_t*)pylonImage.GetBuffer()).clone();
        }
    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] grabFrame error: "
                  << e.GetDescription() << std::endl;
    }

    return image;
}

void BaslerCamera::startRecording(const std::string& filename, int fourcc, double fps)
{
    if (!isOpen_ || recording_) return;

    try {
        cv::Mat testFrame = grabFrame();
        if (testFrame.empty()) {
            std::cerr << "[BaslerCamera] ⚠️ No frame captured, cannot start recording.\n";
            return;
        }

        int width = testFrame.cols;
        int height = testFrame.rows;

        // 确保用 AVI 格式更兼容
        std::string fixed_filename = filename;
        if (fixed_filename.substr(fixed_filename.find_last_of('.') + 1) != "avi")
            fixed_filename += ".avi";

        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        writer_.open(fixed_filename, codec, fps, cv::Size(width, height), true);

        if (!writer_.isOpened()) {
            std::cerr << "[BaslerCamera] ❌ Failed to open VideoWriter for file: " 
                      << fixed_filename << std::endl;
            return;
        }

        recording_ = true;
        std::cout << "[BaslerCamera] ✅ Start recording: " << fixed_filename
                  << " (" << width << "x" << height << "@" << fps << "fps)" << std::endl;

    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] startRecording error: " << e.GetDescription() << std::endl;
        recording_ = false;
    }
}


void BaslerCamera::writeFrame(const cv::Mat& frame) {
    if (recording_ && writer_.isOpened() && !frame.empty()) {
        writer_.write(frame);
    }
}

void BaslerCamera::stopRecording() {
    if (recording_) {
        writer_.release();
        recording_ = false;
        cout << "[BaslerCamera] Recording stopped" << endl;
    }
}

} // namespace gazeestimation
