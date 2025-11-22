#include "cam/basler.hpp" // 请确保路径与你的项目结构一致
#include <iostream>

// 推荐在 cpp 中使用命名空间，不要在 hpp 中污染全局命名空间
using namespace std;
using namespace Pylon;
using namespace cv;
using namespace GenApi;

namespace gazeestimation {

// 构造函数：初始化新成员变量
BaslerCamera::BaslerCamera(const std::string& serialNumber)
    : serialNumber_(serialNumber),
      isOpen_(false),
      recording_(false),
      writerInitialized_(false) // ⚠️ 初始化新增的控制变量
{
    // 注意：camera_ 成员变量首先调用默认构造函数（创建一个空的 InstantCamera）
    // 我们在函数体内通过 Attach 将其绑定到具体的物理设备上

    try {
        // 1. 创建设备信息对象，设置目标序列号
        CDeviceInfo info;
        info.SetSerialNumber(Pylon::String_t(serialNumber.c_str()));

        // 2. 获取传输层工厂
        CTlFactory& tlFactory = CTlFactory::GetInstance();

        // 3. 尝试根据 info 查找并创建设备
        IPylonDevice* device = tlFactory.CreateFirstDevice(info);

        // 4. 将物理设备绑定到 camera_ 对象
        camera_.Attach(device);

        std::cout << "[BaslerCamera] ✅ Initialized object for SN: " << serialNumber 
                  << " (Model: " << camera_.GetDeviceInfo().GetModelName() << ")" << std::endl;

    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] ❌ Constructor Error (SN " << serialNumber << "): " 
                  << e.GetDescription() << std::endl;
    }
}

BaslerCamera::~BaslerCamera() {
    stopRecording();
    close();
    // Pylon::CInstantCamera 会在析构时自动清理资源
}

bool BaslerCamera::open() {
    // 检查是否有设备绑定
    if (!camera_.IsPylonDeviceAttached()) {
        std::cerr << "[BaslerCamera] Cannot open: No device attached. (Check if SN " 
                  << serialNumber_ << " is correct/connected)" << std::endl;
        return false;
    }

    try {
        camera_.Open();
        isOpen_ = true;
        std::cout << "[BaslerCamera] Camera opened: "
                  << camera_.GetDeviceInfo().GetModelName() 
                  << " [SN: " << serialNumber_ << "]" << std::endl;
        return true;
    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] Failed to open camera (SN " << serialNumber_ << "): "
                  << e.GetDescription() << std::endl;
        isOpen_ = false;
        return false;
    }
}

void BaslerCamera::close() {
    try {
        if (camera_.IsGrabbing()) {
            camera_.StopGrabbing();
        }
        if (isOpen_) {
            camera_.Close();
            isOpen_ = false;
            std::cout << "[BaslerCamera] Camera closed (SN " << serialNumber_ << ")" << std::endl;
        }
    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] Error closing camera: " << e.GetDescription() << std::endl;
    }
}

// --- 参数设置函数 (与原先版本保持一致) ---
void BaslerCamera::setFrameRate(double fps) {
    if (!isOpen_) return;
    try {
        CBooleanPtr enableNode(camera_.GetNodeMap().GetNode("AcquisitionFrameRateEnable"));
        CFloatPtr fpsNode(camera_.GetNodeMap().GetNode("AcquisitionFrameRate"));
        
        if (enableNode && fpsNode) {
            enableNode->SetValue(true);
            fpsNode->SetValue(fps);
            cout << "[BaslerCamera] Frame rate set to " << fps << " fps (SN " << serialNumber_ << ")" << endl;
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
        cout << "[BaslerCamera] Exposure set to " << microseconds << " us (SN " << serialNumber_ << ")" << endl;
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
        // 超时时间设为 5000ms
        camera_.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

        if (ptrGrabResult->GrabSucceeded()) {
            static CImageFormatConverter converter;
            converter.OutputPixelFormat = PixelType_BGR8packed;
            CPylonImage pylonImage;
            converter.Convert(pylonImage, ptrGrabResult);
            
            // 深度拷贝
            image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                            CV_8UC3, (uint8_t*)pylonImage.GetBuffer()).clone();
        } else {
            cerr << "[BaslerCamera] Grab Failed. Error: " 
                 << ptrGrabResult->GetErrorCode() << " " 
                 << ptrGrabResult->GetErrorDescription() << endl;
        }
    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] grabFrame error (SN " << serialNumber_ << "): "
                  << e.GetDescription() << std::endl;
    }

    return image;
}
// --- 录像功能使用延迟初始化 ---

void BaslerCamera::startRecording(const std::string& filename, double fps)
{
    if (!isOpen_ || recording_) return;

    // 1. 存储参数和文件名，不立即打开 writer
    videoFilename_ = filename;
    
    // 确保后缀是 .avi
    if (videoFilename_.length() < 4 || videoFilename_.substr(videoFilename_.length() - 4) != ".avi") {
        videoFilename_ += ".avi";
    }

    videoFps_ = fps;
    writerInitialized_ = false; // 标记为尚未初始化 Writer
    recording_ = true;          // 开启录制总开关

    std::cout << "[BaslerCamera] 🎬 Prepare to record: " << videoFilename_ 
              << " (SN " << serialNumber_ << "). Waiting for first frame..." << std::endl;
}

void BaslerCamera::writeFrame(const cv::Mat& frame) {
    // 必须在录制状态且帧非空时才能写入
    if (!recording_ || frame.empty()) return;

    // 2. 延迟初始化：如果 writer 尚未打开，则使用当前帧的尺寸打开
    if (!writerInitialized_) {
        try {
            int width = frame.cols;
            int height = frame.rows;
            int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // 强制使用 MJPG

            writer_.open(videoFilename_, codec, videoFps_, cv::Size(width, height), true);

            if (writer_.isOpened()) {
                writerInitialized_ = true;
                std::cout << "[BaslerCamera] ✅ VideoWriter initialized. Size: " 
                          << width << "x" << height << " @ " << videoFps_ << "fps" << std::endl;
            } else {
                std::cerr << "[BaslerCamera] ❌ Failed to open VideoWriter for file: " 
                          << videoFilename_ << std::endl;
                recording_ = false; // 初始化失败，关闭录制
                return;
            }
        } catch (const std::exception& e) {
            std::cerr << "[BaslerCamera] VideoWriter init error: " << e.what() << std::endl;
            recording_ = false;
            return;
        }
    }

    // 3. 写入帧
    if (writerInitialized_) {
        writer_.write(frame);
    }
}

void BaslerCamera::stopRecording() {
    if (recording_) {
        
        // 必须释放 VideoWriter 资源，确保文件头写入
        if (writerInitialized_ && writer_.isOpened()) {
            writer_.release();
        }
        
        recording_ = false;
        writerInitialized_ = false;
        cout << "[BaslerCamera] Recording stopped (SN " << serialNumber_ << ")" << endl;
    }
}

} // namespace gazeestimation