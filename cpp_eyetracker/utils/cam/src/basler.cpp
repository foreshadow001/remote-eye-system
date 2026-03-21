#include "cam/basler.hpp" // 请确保路径与你的项目结构一致
#include <filesystem>

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

// === 修改后的 open 函数 ===
bool BaslerCamera::open(TriggerMode mode) {
    if (!camera_.IsPylonDeviceAttached()) return false;

    try {
        camera_.Open();
        currentMode_ = mode;
        INodeMap& nodemap = camera_.GetNodeMap();

        // 1. 重置为默认配置 (可选，视情况而定)
        // CCommandPtr(nodemap.GetNode("UserSetSelector"))->SetValue("Default");
        // CCommandPtr(nodemap.GetNode("UserSetLoad"))->Execute();

        /*
        try {
            // 将底层循环缓冲区增加到 100 甚至 200（只要你的系统内存够用）
            camera_.MaxNumBuffer.SetValue(2000); 
        } catch (const GenericException& e) {
            std::cerr << "Failed to set MaxNumBuffer: " << e.GetDescription() << std::endl;
        }
        */

        // 2. 配置触发模式
        if (mode == TriggerMode::Hardware) {
            // 设置 Line1 为输入 (部分相机需要显式设置 LineMode)
            CEnumerationPtr(nodemap.GetNode("LineSelector"))->FromString("Line1");
            CEnumerationPtr(nodemap.GetNode("LineMode"))->FromString("Input");

            // 启用触发
            CEnumerationPtr triggerSelector(nodemap.GetNode("TriggerSelector"));
            triggerSelector->FromString("FrameStart");

            CEnumerationPtr triggerMode(nodemap.GetNode("TriggerMode"));
            triggerMode->FromString("On");

            CEnumerationPtr triggerSource(nodemap.GetNode("TriggerSource"));
            triggerSource->FromString("Line1"); // 硬件触发源

            CEnumerationPtr triggerActivation(nodemap.GetNode("TriggerActivation"));
            triggerActivation->FromString("RisingEdge"); // 上升沿

            // 关闭自动帧率 (由外部信号决定)
            // 注意：Basler 相机在 TriggerMode=On 时，AcquisitionFrameRateEnable 通常被忽略，但为了保险起见设为 false
            CBooleanPtr frameRateEnable(nodemap.GetNode("AcquisitionFrameRateEnable"));
            if (IsWritable(frameRateEnable)) {
                frameRateEnable->SetValue(false);
            }

            cout << "[Basler] " << serialNumber_ << " set to HARDWARE Trigger (Line1, RisingEdge)." << endl;
        } else {
            // 软件/连续模式
            CEnumerationPtr triggerMode(nodemap.GetNode("TriggerMode"));
            triggerMode->FromString("Off");
            
            cout << "[Basler] " << serialNumber_ << " set to CONTINUOUS/SOFTWARE mode." << endl;
        }

        // 3. 配置图像格式 (保持原有逻辑)
        CEnumerationPtr pixelFormat = nodemap.GetNode("PixelFormat");
        pixelFormat->FromString("Mono8");
        converter_.OutputPixelFormat = PixelType_Mono8;
        isMono_ = true;
        /*
        if (IsAvailable(pixelFormat->GetEntryByName("BayerRG8"))) {
            pixelFormat->FromString("BayerRG8");
            converter_.OutputPixelFormat = PixelType_BGR8packed;
            isMono_ = false;
        } else {
            pixelFormat->FromString("Mono8");
            converter_.OutputPixelFormat = PixelType_Mono8;
            isMono_ = true;
        }
        */
        // converter_.OutputBitAlignment = OutputBitAlignment_MsbAligned;

        isOpen_ = true;
        return true;
    } catch (const std::exception& e) {
        cerr << "[Basler] open() failed: " << e.what() << endl;
        return false;
    }
}

/*
bool BaslerCamera::start() {
    try {
        if (!camera_.IsOpen()) return false;
        
        // 这里的策略必须是 GrabLoop_ProvidedByUser，配合你自己的线程循环
        camera_.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByUser);
        // camera_.StartGrabbing(1000);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Basler Start Error] " << e.what() << std::endl;
        return false;
    }
}
*/

// === 关键修改点 1：start 函数 ===
bool BaslerCamera::start() {
    try {
        if (!camera_.IsOpen()) return false;
        // 显式设置底层 Buffer 数量 (与官方代码保持一致或略大)
        camera_.MaxNumBuffer.SetValue(10); 

        camera_.RegisterImageEventHandler(this, RegistrationMode_ReplaceAll, Cleanup_None);
        camera_.StartGrabbing(GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Basler Start Error] " << e.what() << std::endl;
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

// === 关键修改点 2：实现回调函数 (替代原先的 grabFrame) ===
void BaslerCamera::OnImageGrabbed(CBaslerUniversalInstantCamera& /*camera*/, const CBaslerUniversalGrabResultPtr& ptrGrabResult) {
    try {
        if (!ptrGrabResult->GrabSucceeded()) {
            // 如果是超时（硬件触发等待中），在回调模式下不会报错，只会静默等待。
            // 发生错误才是真错误
            cerr << "[Basler] Grab failed (SN " << serialNumber_ << "): " << ptrGrabResult->GetErrorDescription() << endl;
            return;
        }
        
        // cv::Mat out_frame;
        FrameMeta out_meta;

        out_meta.blockID   = ptrGrabResult->GetBlockID();
        out_meta.timestamp = ptrGrabResult->GetTimeStamp();
        out_meta.sys_time_ms = (double)cv::getTickCount() / cv::getTickFrequency() * 1000.0;

        int width  = ptrGrabResult->GetWidth();
        int height = ptrGrabResult->GetHeight();
        
        
        /*
        // 区分 Mono8 与 Color/Bayer 格式
        if (ptrGrabResult->GetPixelType() == PixelType_Mono8) {
            const uint8_t* pBuffer = reinterpret_cast<const uint8_t*>(ptrGrabResult->GetBuffer());
            out_frame.create(height, width, CV_8UC1);
            memcpy(out_frame.data, pBuffer, width * height);
        } 
        else {
            // 彩色相机进行 RGB 转换 (Bayer -> BGR)
            CPylonImage pylonImage;
            converter_.OutputPixelFormat = PixelType_BGR8packed;
            converter_.Convert(pylonImage, ptrGrabResult);
            out_frame.create(height, width, CV_8UC3);
            memcpy(out_frame.data, pylonImage.GetBuffer(), width * height * 3);
        }
        */

        /*
        std::string save_folder = "D:/test_cam/" + serialNumber_;
        std::string save_path = save_folder + "/" + std::to_string(ptrGrabResult->GetBlockID()) + ".png";

        std::filesystem::path save_folder_path(save_folder);
        if (!std::filesystem::exists(save_folder_path)) {
            std::filesystem::create_directories(save_folder_path);
        }

        CImagePersistence::Save(ImageFileFormat_Png, save_path.c_str(), ptrGrabResult);
        */

        /*
        // 我们直接按 CV_8UC1 内存硬拷贝，绝不在此处做任何像素格式转换！
        cv::Mat out_frame(height, width, CV_8UC1);
        const uint8_t* pBuffer = reinterpret_cast<const uint8_t*>(ptrGrabResult->GetBuffer());
        
        // memcpy 是极快的内存操作，耗时通常在零点几毫秒
        memcpy(out_frame.data, pBuffer, width * height);
        */

        
        // 【核心修改】：不申请内存！仅仅创建一个指向 Pylon 底层 Buffer 的 Mat 头 (O(1) 耗时)
        void* pBuffer = ptrGrabResult->GetBuffer();
        cv::Mat wrapped_frame(height, width, CV_8UC1, pBuffer);

        // 调用通过 setFrameCallback 传进来的外部逻辑
        if (callback_) {
            callback_(wrapped_frame, out_meta);
        }
        
        
    } catch (GenICam::GenericException &e) {
        cerr << "[Basler ERROR] (SN " << serialNumber_ << ") " << e.GetDescription() << endl;
    }
}

/*
GrabResult BaslerCamera::grabFrame(cv::Mat& out_frame, FrameMeta& out_meta) {
    if (!camera_.IsGrabbing()) {
        return GrabResult::ERROR_;
    }

    CGrabResultPtr ptrGrabResult;

    // ⚠️ 硬触发下：短 timeout + 轮询
    int timeout_ms = (currentMode_ == TriggerMode::Hardware) ? 50 : 100;

    try {
        // ❗ 不要 ThrowException
        camera_.RetrieveResult(timeout_ms, ptrGrabResult, TimeoutHandling_Return);
    }
    catch (const GenericException& e) {
        cerr << "[Basler ERROR] (SN " << serialNumber_ << ")" << e.GetDescription() << endl;
        return GrabResult::ERROR_;
    }

    // === 关键：超时但没异常 ===
    if (!ptrGrabResult) {
        return GrabResult::TIMEOUT;  // 等待触发
    }

    if (!ptrGrabResult->GrabSucceeded()) {
        cerr << "[Basler] Grab failed (SN " << serialNumber_ << ")" << endl;
        return GrabResult::ERROR_;
    }

    // === 成功帧 ===
    out_meta.blockID   = ptrGrabResult->GetBlockID();
    out_meta.timestamp = ptrGrabResult->GetTimeStamp();
    out_meta.sys_time_ms =
        (double)cv::getTickCount() / cv::getTickFrequency() * 1000.0;

    int width  = ptrGrabResult->GetWidth();
    int height = ptrGrabResult->GetHeight();

    if (isMono_) {
        const uint8_t* pBuffer =
            reinterpret_cast<const uint8_t*>(ptrGrabResult->GetBuffer());
        out_frame.create(height, width, CV_8UC1);
        memcpy(out_frame.data, pBuffer, width * height);
    } else {
        CPylonImage pylonImage;
        converter_.Convert(pylonImage, ptrGrabResult);
        out_frame.create(height, width, CV_8UC3);
        memcpy(out_frame.data, pylonImage.GetBuffer(), width * height * 3);
        // out_frame.create(height, width, CV_8UC3);
        // converter_.Convert(out_frame.ptr(), width * height * 3, ptrGrabResult);
    }

    std::string save_path = "D:/test_cam/" + serialNumber_ + "/" + std::to_string(ptrGrabResult->GetBlockID()) + ".png";
    std::string save_folder = "D:/test_cam/" + serialNumber_;
    std::filesystem::path save_folder_path(save_folder);
    if (!std::filesystem::exists(save_folder_path)) {
        std::filesystem::create_directories(save_folder_path);
    }
    if (!ptrGrabResult->GrabSucceeded()) {
        cerr << "[Basler] Grab failed (SN " << serialNumber_ << ")" << endl;
        return GrabResult::ERROR_;
    }
    // CImagePersistence::Save( ImageFileFormat_Png, save_path.c_str(), ptrGrabResult );
    cv::imwrite(save_path, out_frame);

    return GrabResult::OK;
}
*/

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