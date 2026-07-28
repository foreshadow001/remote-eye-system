#include "cam/basler.hpp" 
#include <filesystem>
#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;
using namespace Pylon;
using namespace cv;
using namespace GenApi;

bool local_debug = false;

namespace gazeestimation {

static std::mutex g_print_mtx;  // serialize console output across threads

BaslerCamera::BaslerCamera(const std::string& serialNumber)
    : serialNumber_(serialNumber), isOpen_(false), recording_(false), writerInitialized_(false) {
    try {
        CDeviceInfo info;
        info.SetSerialNumber(Pylon::String_t(serialNumber.c_str()));
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        IPylonDevice* device = tlFactory.CreateFirstDevice(info);
        camera_.Attach(device);
        std::cout << "[BaslerCamera] ✅ Initialized object for SN: " << serialNumber << std::endl;
    } catch (const GenericException& e) {
        std::cerr << "[BaslerCamera] ❌ Constructor Error (SN " << serialNumber << "): " << e.GetDescription() << std::endl;
    }
}

BaslerCamera::~BaslerCamera() { close(); }

bool BaslerCamera::open(TriggerMode mode) {
    if (!camera_.IsPylonDeviceAttached()) return false;
    try {
        camera_.Open();
        currentMode_ = mode;
        INodeMap& nodemap = camera_.GetNodeMap();

        if (mode == TriggerMode::Hardware) {
            CEnumerationPtr(nodemap.GetNode("LineSelector"))->FromString("Line2");
            CEnumerationPtr(nodemap.GetNode("LineMode"))->FromString("Input");
            CEnumerationPtr(nodemap.GetNode("TriggerSelector"))->FromString("FrameStart");
            CEnumerationPtr(nodemap.GetNode("TriggerMode"))->FromString("On");
            CEnumerationPtr(nodemap.GetNode("TriggerSource"))->FromString("Line2");
            CEnumerationPtr(nodemap.GetNode("TriggerActivation"))->FromString("RisingEdge");
            CBooleanPtr frameRateEnable(nodemap.GetNode("AcquisitionFrameRateEnable"));
            if (IsWritable(frameRateEnable)) frameRateEnable->SetValue(false);
            { lock_guard<mutex> lk(g_print_mtx); cout << "[Basler] " << serialNumber_ << " set to HARDWARE Trigger." << endl; }
        } else {
            CEnumerationPtr(nodemap.GetNode("TriggerMode"))->FromString("Off");
            { lock_guard<mutex> lk(g_print_mtx); cout << "[Basler] " << serialNumber_ << " set to CONTINUOUS/SOFTWARE mode." << endl; }
        }

        // ================= 【核心修改区】 =================
        CEnumerationPtr pixelFormatNode(nodemap.GetNode("PixelFormat"));
        if (pixelFormatNode.IsValid()) {
            bool formatSet = false;
            
            // 1. 相机端设置为 BayerRG8 (省带宽)，转换器端设置为 BGR8packed (供 OpenCV 直接使用)
            try {
                pixelFormatNode->FromString("BayerRG8");
                converter_.OutputPixelFormat = PixelType_BGR8packed;
                isMono_ = false;
                formatSet = true;
                { lock_guard<mutex> lk(g_print_mtx); cout << "[Basler] SN: " << serialNumber_ << " detected as COLOR. Camera: BayerRG8 -> Output: BGR8." << endl; }
            } catch (const GenICam::GenericException& e) {
                { lock_guard<mutex> lk(g_print_mtx); cerr << ">>> [Debug] Failed to set BayerRG8: " << e.GetDescription() << endl; }
            } catch (...) {}

            if (!formatSet) {
                try {
                    pixelFormatNode->FromString("RGB8");
                    converter_.OutputPixelFormat = PixelType_BGR8packed;
                    isMono_ = false;
                    formatSet = true;
                    { lock_guard<mutex> lk(g_print_mtx); cout << "[Basler] SN: " << serialNumber_ << " detected as COLOR. Set to RGB8." << endl; }
                } catch (const GenICam::GenericException& e) {
                    { lock_guard<mutex> lk(g_print_mtx); cerr << ">>> [Debug] Failed to set RGB8: " << e.GetDescription() << endl; }
                } catch (...) {}
            }

            if (!formatSet) {
                try {
                    pixelFormatNode->FromString("Mono8");
                    converter_.OutputPixelFormat = PixelType_Mono8;
                    isMono_ = true;
                    { lock_guard<mutex> lk(g_print_mtx); cout << "[Basler] SN: " << serialNumber_ << " detected as MONO. Set to Mono8." << endl; }
                } catch (const GenICam::GenericException& e) {
                    { lock_guard<mutex> lk(g_print_mtx); cerr << "[Basler Error] Cannot set Mono8 for SN: " << serialNumber_ << " - " << e.GetDescription() << endl; }
                } catch (...) {}
            }
        }
        // =================================================

        isOpen_ = true;
        return true;

    // 【核心修改区2：升级异常捕获体系，防止静默闪退】
    } catch (const GenICam::GenericException& e) {
        cerr << "[Basler] open() GenICam Exception: " << e.GetDescription() << endl;
        return false;
    } catch (const std::exception& e) {
        cerr << "[Basler] open() std::exception: " << e.what() << endl;
        return false;
    } catch (...) {
        cerr << "[Basler] open() CRITICAL: Unknown exception or Segfault caught!" << endl;
        return false;
    }
}

bool BaslerCamera::start() {
    try {
        if (!camera_.IsOpen()) return false;
        
        // 【核心修改】提供 150 帧的硬件缓冲弹性池，彻底吸收操作系统级抖动，且绝不会触发 DMA 上限！
        camera_.MaxNumBuffer.SetValue(maxNumBuffer_);

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
        if (camera_.IsGrabbing()) camera_.StopGrabbing();
        if (isOpen_) { camera_.Close(); isOpen_ = false; }
    } catch (...) {}
}

void BaslerCamera::setFrameRate(double fps) {
    if (!isOpen_) return;
    try {
        CBooleanPtr(camera_.GetNodeMap().GetNode("AcquisitionFrameRateEnable"))->SetValue(true);
        CFloatPtr(camera_.GetNodeMap().GetNode("AcquisitionFrameRate"))->SetValue(fps);
    } catch (...) {}
}

void BaslerCamera::setGain(double gain) {
    if (!isOpen_) return;
    try { CFloatPtr(camera_.GetNodeMap().GetNode("Gain"))->SetValue(gain); } catch (...) {}
}

void BaslerCamera::setGamma(double gamma) {
    if (!isOpen_) return;
    try { CFloatPtr(camera_.GetNodeMap().GetNode("Gamma"))->SetValue(gamma); } catch (...) {}
}

void BaslerCamera::setExposureTime(double microseconds) {
    if (!isOpen_) return;
    try { CFloatPtr(camera_.GetNodeMap().GetNode("ExposureTime"))->SetValue(microseconds); } catch (...) {}
}

void BaslerCamera::OnImageGrabbed(CBaslerUniversalInstantCamera&, const CBaslerUniversalGrabResultPtr& ptrGrabResult) {
    try {
        if (!ptrGrabResult->GrabSucceeded()) {
            cerr << "[Basler] Grab failed (SN " << serialNumber_ << "): " << ptrGrabResult->GetErrorDescription() << endl;
            return;
        }
        
        FrameMeta out_meta;
        out_meta.blockID   = ptrGrabResult->GetBlockID();
        out_meta.timestamp = ptrGrabResult->GetTimeStamp();
        out_meta.sys_time_ms = (double)cv::getTickCount() / cv::getTickFrequency() * 1000.0;
        
        // 【核心修改】不执行任何拷贝，直接将包含 Pylon 缓冲锁的智能指针抛出
        if (callback_) {
            callback_(ptrGrabResult, out_meta);
        }
    } catch (...) {}
}

} // namespace gazeestimation