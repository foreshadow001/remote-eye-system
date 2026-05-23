#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <queue> // 引入队列，用于BFS图论校验

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
    HTuple* hv_NumImages,
    std::vector<std::string>* out_sn_list = nullptr)
{
    namespace fs = std::filesystem;

    std::string folderPath = hv_FolderPath.S().Text();

    if (!fs::exists(folderPath) || !fs::is_directory(folderPath)) {
        throw std::runtime_error("Invalid calibration image folder");
    }

    // SN -> max image index
    std::map<std::string, int> camMaxImg;

    for (const auto& entry : fs::directory_iterator(folderPath))
    {
        if (!entry.is_regular_file())
            continue;

        std::string name = entry.path().stem().string();

        // 必须以 calib_cam_ 开头
        if (name.rfind("calib_cam_", 0) != 0)
            continue;

        // calib_cam_<SN>_<imgIdx>
        // 从末尾找最后一个 '_'，之前的部分是 "calib_cam_<SN>"
        size_t p_last = name.find_last_of('_');
        if (p_last == std::string::npos || p_last <= 10)
            continue;

        std::string sn = name.substr(10, p_last - 10);
        int imgIdx = std::stoi(name.substr(p_last + 1));

        auto it = camMaxImg.find(sn);
        if (it == camMaxImg.end()) {
            camMaxImg[sn] = imgIdx;
        } else {
            if (imgIdx > it->second)
                it->second = imgIdx;
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

    // 输出排序后的 SN 列表（按加入顺序，保证确定性）
    if (out_sn_list) {
        out_sn_list->clear();
        for (const auto& kv : camMaxImg) {
            out_sn_list->push_back(kv.first);
        }
    }

    std::cout << "Found " << (*hv_NumCameras).I() << " cameras and " << (*hv_NumImages).I() << " images." << std::endl;
    if (out_sn_list) {
        for (size_t i = 0; i < out_sn_list->size(); ++i) {
            std::cout << "  Camera " << i << ": SN=" << (*out_sn_list)[i] << std::endl;
        }
    }
}

void action()
{
    // ========================================================================
    // 1. 配置区域与初始化
    // ========================================================================
    Cfg cfg;
    HTuple hv_ImagePath = cfg["cam_calib"]["input_folder"].as<std::string>().c_str();
    HTuple hv_CalibObjDescr = cfg["cam_calib"]["calib_plane"].as<std::string>().c_str();
    HTuple hv_OutputBaseDir = cfg["cam_calib"]["output_folder"].as<std::string>().c_str();
    std::string center_cam_sn;
    try { center_cam_sn = cfg["cam_calib"]["center_cam"].as<std::string>(); }
    catch (...) { center_cam_sn = ""; }

    std::string output_folder= cfg["cam_calib"]["output_folder"].as<std::string>();
    std::filesystem::create_directories(output_folder);

    HTuple hv_NumCameras;
    HTuple hv_NumCalibImages;
    std::vector<std::string> cam_sn_list;

    scan_calib_image_folder(hv_ImagePath, &hv_NumCameras, &hv_NumCalibImages, &cam_sn_list);

    double focus = cfg["cam_calib"]["focus"].as<double>();
    double pixel_size_x = cfg["cam_calib"]["pixel_size_x"].as<double>();
    double pixel_size_y = cfg["cam_calib"]["pixel_size_y"].as<double>();
    
    HObject ho_Image;
    HTuple hv_CalibDataID; // 只需要一个主句柄
    HTuple hv_StartCamParam;
    HTuple hv_Width, hv_Height;
    HTuple hv_Errors;

    int num_cams = hv_NumCameras.I();
    int num_images = hv_NumCalibImages.I();

    std::cout << "\n--- Starting Robust Multi-Camera Calibration ---\n" << std::endl;

    CreateCalibData("calibration_object", hv_NumCameras, 1, &hv_CalibDataID);
    SetCalibDataCalibObject(hv_CalibDataID, 0, hv_CalibObjDescr);

    std::cout << "--- Initializing Cameras ---" << std::endl;
    for (int camIdx = 0; camIdx < num_cams; ++camIdx)
    {
        HTuple hv_CurrentFile = hv_ImagePath + "/calib_cam_" + HTuple(cam_sn_list[camIdx].c_str()) + "_01";
        try {
            ReadImage(&ho_Image, hv_CurrentFile);
            HTuple hv_Channels;
            CountChannels(ho_Image, &hv_Channels);
            if (hv_Channels.I() == 3) {
                Rgb1ToGray(ho_Image, &ho_Image);
            }
            GetImageSize(ho_Image, &hv_Width, &hv_Height);

            // 检查该相机是否有特定的焦距覆盖
            double cam_focus = focus;
            try {
                auto& overrides = cfg["cam_calib"]["focus_overrides"];
                cam_focus = overrides[cam_sn_list[camIdx]].as<double>();
                std::cout << "Camera " << camIdx << " (" << cam_sn_list[camIdx]
                          << ") focus override: " << cam_focus * 1000.0 << "mm" << std::endl;
            } catch (...) {}

            gen_cam_par_area_scan_division(cam_focus, 0, pixel_size_x, pixel_size_y,
                                           hv_Width/2, hv_Height/2, hv_Width, hv_Height,
                                           &hv_StartCamParam);
            
            SetCalibDataCamParam(hv_CalibDataID, camIdx, HTuple(), hv_StartCamParam);
            std::cout << "Camera " << camIdx << " initialized (" << hv_Width.L() << "x" << hv_Height.L() << ")" << std::endl;
        }
        catch (HException &ex) {
            std::cerr << "Fatal Error: Cannot read init image: " << hv_CurrentFile.S() << std::endl;
            throw ex;
        }
    }

    // ========================================================================
    // 2. 阶段一：完全解耦的特征提取 (Independent Observation)
    // ========================================================================
    std::cout << "\n--- Processing Frames (Decoupled Observation) ---" << std::endl;
    
    // 数据结构：记录每一帧有哪些相机成功检测到了标定板
    std::vector<std::vector<int>> frame_observations(num_images);

    for (int imgIdx = 0; imgIdx < num_images; ++imgIdx)
    {
        HTuple imgIdxStr = HTuple(imgIdx).TupleString("02d");
        
        for (int camIdx = 0; camIdx < num_cams; ++camIdx)
        {
            HTuple currentFileName = hv_ImagePath + "/calib_cam_" + HTuple(cam_sn_list[camIdx].c_str()) + "_" + imgIdxStr;
            try {
                ReadImage(&ho_Image, currentFileName);

                // 彩色图转为灰度，保证标定一致性
                HTuple hv_Channels;
                CountChannels(ho_Image, &hv_Channels);
                if (hv_Channels.I() == 3) {
                    Rgb1ToGray(ho_Image, &ho_Image);
                }

                // 尝试提取特征并直接添加到模型
                FindCalibObject(ho_Image, hv_CalibDataID, camIdx, 0, imgIdx,
                                (HTuple("alpha").Append("sigma")), 
                                (HTuple(0.5).Append(1.0)));
                
                // 走到这里说明提取成功，记录该相机观测到了当前帧
                frame_observations[imgIdx].push_back(camIdx);
                
            } catch (HException &) {
                // 静默跳过：该相机在这帧没拍到或读图失败，不影响其他相机
                continue; 
            }
        }
        
        if (!frame_observations[imgIdx].empty()) {
            std::cout << "Frame " << imgIdx << " processed. Cams found: " << frame_observations[imgIdx].size() << std::endl;
        }
    }

    // ========================================================================
    // 3. 阶段二：连通图校验 (Graph Connectivity Validation)
    // ========================================================================
    std::cout << "\n--- Validating Camera Co-visibility Graph ---" << std::endl;
    
    // 参数：两个相机至少需要共同看到多少帧，才认为它们之间的位姿边是可靠的
    const int MIN_COVISIBILITY = 3; 
    
    // 构建邻接矩阵记录共视次数
    std::vector<std::vector<int>> adj_matrix(num_cams, std::vector<int>(num_cams, 0));
    
    for (const auto& obs_list : frame_observations) {
        // 如果一帧中有多个相机看到标定板，它们两两之间增加一次共视记录
        for (size_t i = 0; i < obs_list.size(); ++i) {
            for (size_t j = i + 1; j < obs_list.size(); ++j) {
                int c1 = obs_list[i];
                int c2 = obs_list[j];
                adj_matrix[c1][c2]++;
                adj_matrix[c2][c1]++;
            }
        }
    }

    // 使用 BFS (广度优先搜索) 检查连通性，以 Cam 0 为参考基准
    std::vector<bool> visited(num_cams, false);
    std::queue<int> q;
    
    q.push(0);
    visited[0] = true;
    int connected_count = 1;

    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        
        for (int next_cam = 0; next_cam < num_cams; ++next_cam) {
            // 如果未访问过，且共视次数达到阈值，则认为连通
            if (!visited[next_cam] && adj_matrix[curr][next_cam] >= MIN_COVISIBILITY) {
                visited[next_cam] = true;
                q.push(next_cam);
                connected_count++;
            }
        }
    }

    // 判断连通结果
    if (connected_count < num_cams) {
        std::cerr << "[ERROR] Camera Graph is Disconnected! Calibration will fail." << std::endl;
        std::cerr << "The following cameras lack sufficient co-visibility (minimum " << MIN_COVISIBILITY << " required):" << std::endl;
        for (int i = 0; i < num_cams; ++i) {
            if (!visited[i]) {
                std::cerr << " -> Camera " << i << " is isolated." << std::endl;
            }
        }
        ClearCalibData(hv_CalibDataID);
        return; // 提前安全终止，防止内部崩溃
    } else {
        std::cout << "Graph Check Passed: All cameras are safely connected to the reference frame." << std::endl;
    }

    // ========================================================================
    // 4. 阶段三：执行标定 (Execution)
    // ========================================================================
    std::cout << "\n--- Calculating Calibration Parameters ---" << std::endl;
    try {
        CalibrateCameras(hv_CalibDataID, &hv_Errors);
        std::cout << "Calibration Complete. Average Error: " << hv_Errors.D() << " pixels" << std::endl;
    } catch (HException &ex) {
        std::cerr << "Calibration Calculation Failed: " << ex.ErrorMessage().TextA() << std::endl;
        ClearCalibData(hv_CalibDataID);
        return;
    }

    // ========================================================================
    // 5. 重基准到中心相机
    // ========================================================================
    std::cout << "\n--- Re-basing to Center Camera: " << center_cam_sn << " ---" << std::endl;

    // 获取各相机在 cam0 坐标系下的位姿
    std::vector<HTuple> poses_in_cam0(num_cams);
    for (int camIdx = 0; camIdx < num_cams; ++camIdx) {
        if (camIdx == 0)
            CreatePose(0, 0, 0, 0, 0, 0, "Rp+T", "gba", "point", &poses_in_cam0[camIdx]);
        else
            GetCalibData(hv_CalibDataID, "camera", camIdx, "pose", &poses_in_cam0[camIdx]);
    }

    // 找到中心相机索引
    int center_idx = -1;
    for (int i = 0; i < num_cams; ++i) {
        if (cam_sn_list[i] == center_cam_sn) { center_idx = i; break; }
    }
    if (center_idx < 0) {
        std::cerr << "[Error] Center camera " << center_cam_sn << " not in camera list!" << std::endl;
    }

    // T_center_to_cam0 = pose of center camera in cam0 frame
    // T_cam0_to_center = inv(T_center_to_cam0)
    // T_cam_i_to_center = T_cam0_to_center * T_cam_i_to_cam0
    HTuple T_center_to_cam0, T_cam0_to_center;
    if (center_idx >= 0) {
        T_center_to_cam0 = poses_in_cam0[center_idx];
        PoseInvert(T_center_to_cam0, &T_cam0_to_center);
    }

    // 计算各相机在中心相机坐标系下的位姿
    std::vector<HTuple> poses_in_center(num_cams);
    for (int camIdx = 0; camIdx < num_cams; ++camIdx) {
        if (center_idx >= 0) {
            PoseCompose(T_cam0_to_center, poses_in_cam0[camIdx], &poses_in_center[camIdx]);
        } else {
            poses_in_center[camIdx] = poses_in_cam0[camIdx];  // fallback
        }
    }

    // ========================================================================
    // 6. 导出参数
    // ========================================================================
    std::cout << "\n--- Exporting Parameters ---" << std::endl;

    for (int camIdx = 0; camIdx < num_cams; ++camIdx)
    {
        HTuple hv_CurrentCamParam, hv_CurrentPose, hv_StringPose;
        HTuple hv_FileHandle, hv_FullParamString;

        GetCalibData(hv_CalibDataID, "camera", camIdx, "params", &hv_CurrentCamParam);
        hv_CurrentPose = poses_in_center[camIdx];

        HTuple outFileName = hv_OutputBaseDir + HTuple(cam_sn_list[camIdx].c_str()) + "_Data.xml";
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