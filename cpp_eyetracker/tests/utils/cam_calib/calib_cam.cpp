#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <map>
#include <stdexcept>

#include "HalconCpp.h"
#include "cfg/config.hpp"

using namespace HalconCpp;
using namespace std;

// --- 辅助函数：关闭更新 ---
void dev_update_off() { return; }

// --- 辅助函数：生成相机参数 ---
void gen_cam_par_area_scan_division(HTuple hv_Focus, HTuple hv_Kappa, HTuple hv_Sx, 
    HTuple hv_Sy, HTuple hv_Cx, HTuple hv_Cy, HTuple hv_ImageWidth, HTuple hv_ImageHeight, 
    HTuple *hv_CameraParam)
{
    (*hv_CameraParam).Clear();
    (*hv_CameraParam)[0] = "area_scan_division";
    (*hv_CameraParam).Append(hv_Focus);
    (*hv_CameraParam).Append(hv_Kappa);
    (*hv_CameraParam).Append(hv_Sx);
    (*hv_CameraParam).Append(hv_Sy);
    (*hv_CameraParam).Append(hv_Cx);
    (*hv_CameraParam).Append(hv_Cy);
    (*hv_CameraParam).Append(hv_ImageWidth);
    (*hv_CameraParam).Append(hv_ImageHeight);
}

void scan_calib_image_folder(
    const HTuple& hv_FolderPath,
    HTuple* hv_NumCameras,
    HTuple* hv_NumImages)
{
    namespace fs = std::filesystem;

    std::string folderPath = hv_FolderPath.S().Text();

    if (!fs::exists(folderPath) || !fs::is_directory(folderPath)) {
        throw std::runtime_error("Invalid calibration image folder");
    }

    // camIdx -> max image index
    std::map<int, int> camMaxImg;

    for (const auto& entry : fs::directory_iterator(folderPath))
    {
        if (!entry.is_regular_file())
            continue;

        std::string name = entry.path().stem().string();

        // 必须以 calib_cam_ 开头
        if (name.rfind("calib_cam_", 0) != 0)
            continue;

        // calib_cam_<cam>_<img>
        size_t p = name.find('_', 10);
        if (p == std::string::npos)
            continue;

        try {
            int camIdx = std::stoi(name.substr(10, p - 10));
            int imgIdx = std::stoi(name.substr(p + 1));

            auto it = camMaxImg.find(camIdx);
            if (it == camMaxImg.end()) {
                camMaxImg[camIdx] = imgIdx;
            } else {
                if (imgIdx > it->second)
                    it->second = imgIdx;
            }
        }
        catch (...) {
            continue;
        }
    }

    if (camMaxImg.empty()) {
        throw std::runtime_error("No valid calibration images found");
    }

    *hv_NumCameras = static_cast<Hlong>(camMaxImg.size());

    int maxImgIdx = 0;
    for (const auto& kv : camMaxImg) {
        if (kv.second > maxImgIdx)
            maxImgIdx = kv.second;
    }

    *hv_NumImages = maxImgIdx + 1; // 假定从 00 开始

    std::cout << "Found " << (*hv_NumCameras).I() << " cameras and " << (*hv_NumImages).I() << " images." << std::endl;
}

void action()
{
    // ========================================================================
    // 1. 配置区域
    // ========================================================================
    Cfg cfg;

    HTuple hv_ImagePath = cfg["cam_calib"]["input_folder"].as<std::string>().c_str();
    HTuple hv_CalibObjDescr = cfg["cam_calib"]["calib_plane"].as<std::string>().c_str();
    HTuple hv_OutputBaseDir = cfg["cam_calib"]["output_folder"].as<std::string>().c_str();

    std::string output_folder= cfg["cam_calib"]["output_folder"].as<std::string>();
    std::filesystem::create_directories(output_folder);

    HTuple hv_NumCameras; 
    HTuple hv_NumCalibImages;

    // 读取文件夹中的图片数量
    scan_calib_image_folder(hv_ImagePath, &hv_NumCameras, &hv_NumCalibImages);

    double focus = cfg["cam_calib"]["focus"].as<double>();
    double pixel_size_x = cfg["cam_calib"]["pixel_size_x"].as<double>();
    double pixel_size_y = cfg["cam_calib"]["pixel_size_y"].as<double>();
    
    // 变量声明
    HObject ho_Image;
    HTuple hv_CalibDataID, hv_CalibDataID_Check; // 两个句柄：主句柄 和 检测句柄
    HTuple hv_StartCamParam;
    HTuple hv_Width, hv_Height;
    HTuple hv_ValidFrames, hv_InvalidFrames;
    HTuple hv_Errors;

    std::cout << "\n--- Starting Robust Multi-Camera Calibration ---\n" << std::endl;

    // ========================================================================
    // 2. 初始化模型
    // ========================================================================
    // 创建主标定模型 (用于最终计算)
    CreateCalibData("calibration_object", hv_NumCameras, 1, &hv_CalibDataID);
    SetCalibDataCalibObject(hv_CalibDataID, 0, hv_CalibObjDescr);

    // [关键] 创建辅助标定模型 (仅用于检测是否能找到标定板)
    CreateCalibData("calibration_object", hv_NumCameras, 1, &hv_CalibDataID_Check);
    SetCalibDataCalibObject(hv_CalibDataID_Check, 0, hv_CalibObjDescr);

    std::cout << "--- Initializing Cameras ---" << std::endl;
    
    // 初始化相机参数（同时应用到两个模型）
    for (int camIdx = 0; camIdx < (Hlong)hv_NumCameras; ++camIdx)
    {
        HTuple hv_CurrentFile = hv_ImagePath + "/calib_cam_" + HTuple(camIdx) + "_01";
        try {
            ReadImage(&ho_Image, hv_CurrentFile);
            GetImageSize(ho_Image, &hv_Width, &hv_Height);
            
            gen_cam_par_area_scan_division(focus, 0, pixel_size_x, pixel_size_y, 
                                           hv_Width/2, hv_Height/2, hv_Width, hv_Height, 
                                           &hv_StartCamParam);
            
            // 设置主模型参数
            SetCalibDataCamParam(hv_CalibDataID, camIdx, HTuple(), hv_StartCamParam);
            // 设置检测模型参数
            SetCalibDataCamParam(hv_CalibDataID_Check, camIdx, HTuple(), hv_StartCamParam);
            
            std::cout << "Camera " << camIdx << " initialized (" << hv_Width.L() << "x" << hv_Height.L() << ")" << std::endl;
        }
        catch (HException &ex) {
            std::cerr << "Fatal Error: Cannot read init image: " << hv_CurrentFile.S() << std::endl;
            throw ex;
        }
    }

    // ========================================================================
    // 3. 循环处理每一帧 (预检 -> 提交 模式)
    // ========================================================================
    std::cout << "\n--- Processing Frames ---" << std::endl;
    
    hv_ValidFrames = HTuple();
    hv_InvalidFrames = HTuple();

    for (int imgIdx = 0; imgIdx < (Hlong)hv_NumCalibImages; ++imgIdx)
    {
        // 1. 先把当前时刻所有相机的图片读入内存，避免重复IO
        std::vector<HObject> currentFrameImages((Hlong)hv_NumCameras);
        bool readSuccess = true;

        for (int camIdx = 0; camIdx < (Hlong)hv_NumCameras; ++camIdx)
        {
            HTuple imgIdxStr = HTuple(imgIdx).TupleString("02d");
            HTuple currentFileName = hv_ImagePath + "/calib_cam_" + HTuple(camIdx) + "_" + imgIdxStr;
            try {
                ReadImage(&currentFrameImages[camIdx], currentFileName);
            } catch (HException &) {
                readSuccess = false; // 如果连图都读不到，直接标记失败
                std::cerr << "Error reading file: " << currentFileName.S() << std::endl;
            }
        }

        if (!readSuccess) {
            hv_InvalidFrames.Append(imgIdx);
            std::cout << "Frame " << imgIdx << ": Skipped (File IO Error)" << std::endl;
            continue;
        }

        // 2. [预检阶段] 检查所有相机是否能找到标定板
        // 我们使用 hv_CalibDataID_Check 进行测试，即使添加了脏数据也无所谓，因为我们不用它做最终计算
        bool allCamerasFound = true;
        
        for (int camIdx = 0; camIdx < (Hlong)hv_NumCameras; ++camIdx)
        {
            try {
                // 在 Check 模型上尝试寻找
                FindCalibObject(currentFrameImages[camIdx], hv_CalibDataID_Check, camIdx, 0, imgIdx, 
                                (HTuple("alpha").Append("sigma")), 
                                (HTuple(0.5).Append(1.0)));
            } catch (HException &) {
                // 只要有一个相机找不到，这一帧就废了
                allCamerasFound = false;
                // 为了效率，其实可以 break，但为了 debug 也可以跑完
            }
        }

        // 3. [提交阶段] 如果所有相机都合格，才加入主模型
        if (allCamerasFound)
        {
            std::cout << "Frame " << imgIdx << ": Valid (Sync Success) -> Adding to Main Model" << std::endl;
            hv_ValidFrames.Append(imgIdx);

            for (int camIdx = 0; camIdx < (Hlong)hv_NumCameras; ++camIdx)
            {
                // 在 Main 模型上真正执行添加
                // 因为图片已经在内存中且确认可检测，这里极大概率成功，不需要额外的 try-catch 逻辑处理
                FindCalibObject(currentFrameImages[camIdx], hv_CalibDataID, camIdx, 0, imgIdx, 
                                (HTuple("alpha").Append("sigma")), 
                                (HTuple(0.5).Append(1.0)));
            }
        }
        else
        {
            std::cout << "Frame " << imgIdx << ": Invalid (Partial Detection) -> Discarded" << std::endl;
            hv_InvalidFrames.Append(imgIdx);
            // 此时 hv_CalibDataID 主模型完全没有被触碰，保持了纯净，不需要 Remove 操作
        }
    }

    // ========================================================================
    // 4. 执行标定
    // ========================================================================
    std::cout << "\n--- Statistics ---" << std::endl;
    std::cout << "Valid Frames: " << hv_ValidFrames.TupleLength().I() << std::endl;
    std::cout << "Invalid Frames: " << hv_InvalidFrames.TupleLength().I() << std::endl;

    if (hv_ValidFrames.TupleLength().I() < 3) {
        std::cerr << "Error: Too Few Valid Frames (" << hv_ValidFrames.TupleLength().I() << "). Need at least 3." << std::endl;
        // 清理内存
        ClearCalibData(hv_CalibDataID);
        ClearCalibData(hv_CalibDataID_Check);
        return;
    }

    std::cout << "Calculating Calibration Parameters..." << std::endl;
    try {
        CalibrateCameras(hv_CalibDataID, &hv_Errors);
        std::cout << "Calibration Complete. Average Error: " << hv_Errors.D() << std::endl;
    } catch (HException &ex) {
        std::cerr << "Calibration Calculation Failed: " << ex.ErrorMessage().TextA() << std::endl;
        ClearCalibData(hv_CalibDataID);
        ClearCalibData(hv_CalibDataID_Check);
        return;
    }

    // ========================================================================
    // 5. 导出参数 (XML)
    // ========================================================================
    std::cout << "\n--- Exporting Parameters ---" << std::endl;
    
    for (int camIdx = 0; camIdx < (Hlong)hv_NumCameras; ++camIdx)
    {
        HTuple hv_CurrentCamParam, hv_CurrentPose, hv_StringPose;
        HTuple hv_FileHandle, hv_FullParamString;
        
        GetCalibData(hv_CalibDataID, "camera", camIdx, "params", &hv_CurrentCamParam);
        
        if (camIdx == 0) {
            CreatePose(0, 0, 0, 0, 0, 0, "Rp+T", "gba", "point", &hv_CurrentPose);
        } else {
            GetCalibData(hv_CalibDataID, "camera", camIdx, "pose", &hv_CurrentPose);
        }

        HTuple outFileName = hv_OutputBaseDir + "Camera" + HTuple(camIdx) + "_Data.xml";
        OpenFile(outFileName, "output", &hv_FileHandle);
        
        // Write XML Header
        FwriteString(hv_FileHandle, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        FwriteString(hv_FileHandle, "<CameraData>\n");
        
        // Internal Params
        FwriteString(hv_FileHandle, "  <InternalParameters>\n");
        HTuple type = ((const HTuple&)hv_CurrentCamParam)[0];
        HTuple vals = hv_CurrentCamParam.TupleSelectRange(1, hv_CurrentCamParam.TupleLength()-1);
        HTuple valStr;
        TupleString(vals, ".12g", &valStr);
        hv_FullParamString = type + " " + ((valStr + " ").TupleSum());
        FwriteString(hv_FileHandle, ("    <RawData>" + hv_FullParamString) + "</RawData>\n");
        FwriteString(hv_FileHandle, "  </InternalParameters>\n");

        // External Params
        FwriteString(hv_FileHandle, "  <ExternalParameters>\n");
        TupleString(hv_CurrentPose, ".12g", &hv_StringPose);
        
        FwriteString(hv_FileHandle, "    <Translation>\n");
        FwriteString(hv_FileHandle, ("      <X>" + HTuple(hv_StringPose[0])) + "</X>\n");
        FwriteString(hv_FileHandle, ("      <Y>" + HTuple(hv_StringPose[1])) + "</Y>\n");
        FwriteString(hv_FileHandle, ("      <Z>" + HTuple(hv_StringPose[2])) + "</Z>\n");
        FwriteString(hv_FileHandle, "    </Translation>\n");
        
        FwriteString(hv_FileHandle, "    <Rotation>\n");
        FwriteString(hv_FileHandle, ("      <Alpha>" + HTuple(hv_StringPose[3])) + "</Alpha>\n");
        FwriteString(hv_FileHandle, ("      <Beta>" + HTuple(hv_StringPose[4])) + "</Beta>\n");
        FwriteString(hv_FileHandle, ("      <Gamma>" + HTuple(hv_StringPose[5])) + "</Gamma>\n");
        FwriteString(hv_FileHandle, "    </Rotation>\n");
        
        // Meta
        FwriteString(hv_FileHandle, "    <Meta>\n");
        FwriteString(hv_FileHandle, ("      <PoseTypeCode>" + HTuple(hv_StringPose[6])) + "</PoseTypeCode>\n");
        FwriteString(hv_FileHandle, "      <OrderOfTransform>Rp+T</OrderOfTransform>\n");
        FwriteString(hv_FileHandle, "      <OrderOfRotation>gba</OrderOfRotation>\n");
        FwriteString(hv_FileHandle, "      <ViewOfTransform>point</ViewOfTransform>\n");
        FwriteString(hv_FileHandle, "    </Meta>\n");
        
        FwriteString(hv_FileHandle, ("    <RawPose>" + ((hv_StringPose + " ").TupleSum())) + "</RawPose>\n");
        FwriteString(hv_FileHandle, "  </ExternalParameters>\n");
        FwriteString(hv_FileHandle, "</CameraData>\n");
        
        CloseFile(hv_FileHandle);
        std::cout << "Saved: " << outFileName.S() << std::endl;
    }

    // 清理所有句柄
    ClearCalibData(hv_CalibDataID);
    ClearCalibData(hv_CalibDataID_Check);
}

// --- Main Entry Point ---
#ifndef NO_EXPORT_APP_MAIN
int main(int argc, char *argv[])
{
    int ret = 0;
    try
    {
#if defined(_WIN32)
        SetSystem("use_window_thread", "true");
#endif
        SetSystem("width", 512);
        SetSystem("height", 512);
        action();
    }
    catch (HException &exception)
    {
        fprintf(stderr, "Error #%u in %s: %s\n", exception.ErrorCode(),
            exception.ProcName().TextA(),
            exception.ErrorMessage().TextA());
        ret = 1;
    }
    return ret;
}
#endif