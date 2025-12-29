#include "cam/basler.hpp" // 请确保路径与你的项目结构一致
#include <iostream>
#include <chrono>

// 推荐在 cpp 中使用命名空间，不要在 hpp 中污染全局命名空间
using namespace std;
using namespace Pylon;
using namespace cv;
using namespace GenApi;

bool local_debug = false;

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
        GenApi::INodeMap& nodemap = camera_.GetNodeMap();

        // ---- PixelFormat 枚举节点 ----
        GenApi::CEnumerationPtr pixelFormat =
            nodemap.GetNode("PixelFormat");

        if (!pixelFormat || !GenApi::IsWritable(pixelFormat)) {
            throw std::runtime_error("PixelFormat not writable");
        }

        // ---- 判断是否支持 BayerRG8 ----
        auto bayerRG8Entry = pixelFormat->GetEntryByName("BayerRG8");

        if (GenApi::IsAvailable(bayerRG8Entry)) {
            // ========= 彩色相机 =========
            pixelFormat->FromString("BayerRG8");
            converter_.OutputPixelFormat = PixelType_BGR8packed;
            isMono_ = false;

            std::cout << "[BaslerCamera] Color camera detected, "
                      << "PixelFormat set to BayerRG8" << std::endl;
        }
        else {
            // ========= 黑白相机 =========
            auto mono8Entry = pixelFormat->GetEntryByName("Mono8");
            if (!GenApi::IsAvailable(mono8Entry)) {
                throw std::runtime_error(
                    "Camera supports neither BayerRG8 nor Mono8");
            }

            pixelFormat->FromString("Mono8");
            converter_.OutputPixelFormat = PixelType_Mono8;
            isMono_ = true;

            std::cout << "[BaslerCamera] Mono camera detected, "
                      << "PixelFormat set to Mono8" << std::endl;
        }

        // 初始化转换器，强制不进行对齐（OpenCV 默认也是连续内存）
        converter_.OutputBitAlignment = OutputBitAlignment_MsbAligned;

        // ---- 开始采集 ----
        camera_.StartGrabbing(GrabStrategy_LatestImageOnly);
        isOpen_ = true;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[BaslerCamera] open() failed: "
                  << e.what() << std::endl;
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

bool BaslerCamera::grabFrame(cv::Mat& out_frame) {
    if (!camera_.IsGrabbing()) return false;

    CGrabResultPtr ptrGrabResult;
    // 超时时间不要太长，防止阻塞 UI
    auto tg0 = chrono::steady_clock::now();
    camera_.RetrieveResult(20, ptrGrabResult, TimeoutHandling_ThrowException);
    auto tg1 = chrono::steady_clock::now();
    if (local_debug) {
        std::cout << "RetrieveResult time: " << ms(tg0, tg1) << " ms" << std::endl;
    }

    if (ptrGrabResult->GrabSucceeded()) {
        int width = ptrGrabResult->GetWidth();
        int height = ptrGrabResult->GetHeight();
        
        // 获取抓取到的原始 buffer 指针
        const uint8_t* pBuffer = (uint8_t*)ptrGrabResult->GetBuffer();

        if (isMono_) {
            out_frame.create(height, width, CV_8UC1);
            if (local_debug) {
                std::cout << "is_mono = true" << std::endl;
            }
            EPixelType type = ptrGrabResult->GetPixelType();
            auto t0 = chrono::steady_clock::now();
            if (type == PixelType_Mono8) {
                memcpy(out_frame.data, pBuffer, width * height);
            } else {
                converter_.Convert(out_frame.ptr(), width * height, ptrGrabResult);
            }
            auto t1 = chrono::steady_clock::now();
            if (local_debug) {
                std::cout << "Copy time: " << ms(t0, t1) << " ms" << std::endl;
            }
        } 
        else {
            // === 常规模式 (Color) ===
            if (local_debug) {
                std::cout << "is_color = true" << std::endl;
            }
            out_frame.create(height, width, CV_8UC3);

            // 使用 converter 直接写入 Mat 的数据区
            auto tc2 = chrono::steady_clock::now();
            converter_.Convert(out_frame.ptr(), width * height * 3, ptrGrabResult);
            auto tc3 = chrono::steady_clock::now();
            if (local_debug) {
                std::cout << "Converter time: " << ms(tc2, tc3) << " ms" << std::endl;
            }
        }
        return true;
    }
    return false;
}

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
        if (frame.channels() == 1) {
            // 仅在写入这一刻转换，不影响采集
            cv::Mat temp_color;
            cv::cvtColor(frame, temp_color, cv::COLOR_GRAY2BGR);
            writer_.write(temp_color);
        } else {
            writer_.write(frame);
        }
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