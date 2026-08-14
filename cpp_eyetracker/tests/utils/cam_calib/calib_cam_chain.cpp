#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <queue>
#include <thread>
#include <atomic>

#include "HalconCpp.h"
#include "cfg/config.hpp"

using namespace HalconCpp;
using namespace std;
using namespace std::chrono;

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
    auto t_total_0 = steady_clock::now();

    // ========================================================================
    // 0. 配置读取 (cam_calib.yaml)
    // ========================================================================
    namespace fs = std::filesystem;
    fs::path cfg_dir = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg";
    Cfg cfg((cfg_dir / "cam_calib.yaml").string()); auto& calib = cfg["calib"];
    string base_pid = calib["participant_id"].as<string>();   // cam_calib.yaml 自带 (P001 / D001 系列)

    // input_participant_id: 优先使用 cam_calib.yaml 配置，空则 fallback 到 participant_id
    string pid;
    try { pid = calib["input_participant_id"].as<string>(); } catch(...) { pid = ""; }
    if (pid.empty()) pid = base_pid;

    string base_dir = calib["calib_save_dir"].as<string>();

    // input_dir: 优先使用配置值，空则自动计算
    string input_dir, output_dir;
    try { input_dir = calib["input_dir"].as<string>(); } catch(...) { input_dir = ""; }
    if (input_dir.empty()) input_dir = base_dir + "/" + pid + "/pictures";

    // output_dir: 优先使用配置值，空则自动计算
    try { output_dir = calib["output_dir"].as<string>(); } catch(...) { output_dir = ""; }
    if (output_dir.empty()) output_dir = base_dir + "/" + pid + "/output";

    filesystem::create_directories(output_dir);

    HTuple hv_ImagePath      = input_dir.c_str();
    HTuple hv_CalibObjDescr  = calib["calib_plane"].as<string>().c_str();
    HTuple hv_OutputBaseDir   = output_dir.c_str();

    double focus         = calib["focus"].as<double>();
    double pixel_size_x  = calib["pixel_size_x"].as<double>();
    double pixel_size_y  = calib["pixel_size_y"].as<double>();
    string center_cam_sn;
    try { center_cam_sn = calib["center_cam"].as<string>(); }
    catch (...) { center_cam_sn = ""; }

    cout << "\n=== Multi-Camera Calibration Chain ===\n" << endl;
    cout << "Input  : " << input_dir << endl;
    cout << "Output : " << output_dir << endl;
    cout << "Plate  : " << hv_CalibObjDescr.S() << endl;
    cout << "Focus  : " << focus * 1000.0 << " mm" << endl;
    cout << "Pixel  : " << pixel_size_x * 1e6 << " x " << pixel_size_y * 1e6 << " um" << endl;
    if (!center_cam_sn.empty()) cout << "Center : " << center_cam_sn << endl;

    // ========================================================================
    // Stage 0 — 目录扫描
    // ========================================================================
    HTuple hv_NumCameras, hv_NumCalibImages;
    vector<string> cam_sn_list;

    auto t0 = steady_clock::now();
    scan_calib_image_folder(hv_ImagePath, &hv_NumCameras, &hv_NumCalibImages, &cam_sn_list);
    double dt_scan = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 0 - Scan: " << fixed << setprecision(2) << dt_scan << "s" << endl;

    int num_cams   = hv_NumCameras.I();
    int num_images = hv_NumCalibImages.I();

    HObject ho_Image;
    HTuple hv_CalibDataID;
    HTuple hv_StartCamParam;
    HTuple hv_Width, hv_Height;
    HTuple hv_Errors;

    CreateCalibData("calibration_object", hv_NumCameras, 1, &hv_CalibDataID);
    SetCalibDataCalibObject(hv_CalibDataID, 0, hv_CalibObjDescr);

    // ========================================================================
    // Stage 1 — 相机初始化
    // ========================================================================
    t0 = steady_clock::now();
    for (int camIdx = 0; camIdx < num_cams; ++camIdx)
    {
        HTuple hv_CurrentFile = hv_ImagePath + "/calib_cam_" + HTuple(cam_sn_list[camIdx].c_str()) + "_01";
        try {
            ReadImage(&ho_Image, hv_CurrentFile);
            HTuple hv_Channels;
            CountChannels(ho_Image, &hv_Channels);
            if (hv_Channels.I() == 3) { Rgb1ToGray(ho_Image, &ho_Image); }
            GetImageSize(ho_Image, &hv_Width, &hv_Height);

            double cam_focus = focus;
            try {
                auto& overrides = calib["focus_overrides"];
                cam_focus = overrides[cam_sn_list[camIdx]].as<double>();
                cout << "  Camera " << camIdx << " (" << cam_sn_list[camIdx]
                     << ") focus override: " << cam_focus * 1000.0 << " mm" << endl;
            } catch (const runtime_error&) {}

            gen_cam_par_area_scan_division(cam_focus, 0, pixel_size_x, pixel_size_y,
                                           hv_Width/2, hv_Height/2, hv_Width, hv_Height,
                                           &hv_StartCamParam);
            SetCalibDataCamParam(hv_CalibDataID, camIdx, HTuple(), hv_StartCamParam);
        }
        catch (HException &ex) {
            cerr << "[Error] Cannot read init image: " << hv_CurrentFile.S() << endl;
            throw ex;
        }
    }
    double dt_init = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 1 - Init: " << fixed << setprecision(2) << dt_init << "s ("
         << num_cams << " cameras, " << (dt_init/num_cams) << "s avg each)" << endl;

    // ========================================================================
    // Stage 2 — 特征提取 (核心瓶颈)
    // ========================================================================
    vector<vector<int>> frame_observations(num_images);
    int feat_total = num_images * num_cams;
    int feat_success = 0, feat_failed = 0;

    // Build flat task list for parallel preload
    struct Task { int camIdx, imgIdx; string fn; };
    vector<Task> tasks; tasks.reserve(feat_total);
    for (int imgIdx = 0; imgIdx < num_images; ++imgIdx) {
        HTuple imgIdxStr = HTuple(imgIdx).TupleString("02d");
        for (int camIdx = 0; camIdx < num_cams; ++camIdx) {
            string fn = (hv_ImagePath + "/calib_cam_" + HTuple(cam_sn_list[camIdx].c_str()) + "_" + imgIdxStr).S();
            tasks.push_back({camIdx, imgIdx, fn});
        }
    }

    // Pre-allocate image buffer + per-task success flag
    vector<HObject> preloaded(feat_total);
    vector<bool> load_ok(feat_total, false);

    // ---- Stage 2a: Parallel ReadImage ----
    t0 = steady_clock::now();
    unsigned int hw_threads = thread::hardware_concurrency();
    int n_workers = max(1u, min(hw_threads, (unsigned int)feat_total));
    atomic<int> task_idx{0};
    atomic<int> loaded_cnt{0};

    auto worker = [&]() {
        while (true) {
            int i = task_idx.fetch_add(1);
            if (i >= feat_total) break;
            auto& t = tasks[i];
            try {
                ReadImage(&preloaded[i], HTuple(t.fn.c_str()));
                HTuple hv_Channels;
                CountChannels(preloaded[i], &hv_Channels);
                if (hv_Channels.I() == 3) { Rgb1ToGray(preloaded[i], &preloaded[i]); }
                load_ok[i] = true;
                loaded_cnt.fetch_add(1);
            } catch (HException &) {
                // image stays as default HObject, load_ok stays false
            }
        }
    };

    cout << "[Stage 2a] Parallel preload (" << feat_total << " images, " << n_workers << " threads)..." << endl;
    vector<thread> workers;
    for (int w = 0; w < n_workers; ++w) workers.emplace_back(worker);
    for (auto& w : workers) w.join();

    double dt_load = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 2a - Preload: " << fixed << setprecision(2) << dt_load << "s ("
         << loaded_cnt.load() << " loaded, " << (feat_total - loaded_cnt.load()) << " failed)" << endl;

    // ---- Stage 2b: Serial FindCalibObject ----
    t0 = steady_clock::now();
    cout << "[Stage 2b] Serial FindCalibObject..." << endl;
    for (int i = 0; i < feat_total; ++i) {
        if (!load_ok[i]) { feat_failed++; continue; }
        auto& t = tasks[i];
        try {
            FindCalibObject(preloaded[i], hv_CalibDataID, t.camIdx, 0, t.imgIdx,
                            (HTuple("alpha").Append("sigma")),
                            (HTuple(0.5).Append(1.0)));
            frame_observations[t.imgIdx].push_back(t.camIdx);
            feat_success++;
        } catch (HException &) {
            feat_failed++;
            continue;
        }
        // Progress bar per task
        if ((i + 1) % num_cams == 0 || i == feat_total - 1) {
            int img_done = (i + 1) / num_cams;
            double dt_sofar = duration<double>(steady_clock::now() - t0).count();
            double img_per_s = img_done / dt_sofar;
            double eta = dt_sofar / img_done * (num_images - img_done);
            int pct = img_done * 100 / num_images;
            int bar_w = 30, filled = img_done * bar_w / num_images;
            cout << "\r  [";
            for (int k = 0; k < bar_w; ++k) cout << (k < filled ? '=' : (k == filled && filled < bar_w ? '>' : ' '));
            cout << "] " << img_done << "/" << num_images
                 << " | " << fixed << setprecision(1) << dt_sofar << "s"
                 << " | " << setprecision(2) << img_per_s << " img/s"
                 << " | ETA " << (int)(eta + 0.5) << "s" << flush;
        }
    }
    cout << endl;
    double dt_find = duration<double>(steady_clock::now() - t0).count();
    double dt_feat = dt_load + dt_find;
    cout << "[Timer] Stage 2 - Feature: " << fixed << setprecision(2) << dt_feat << "s ("
         << feat_total << " calls, " << feat_success << " success, " << feat_failed << " failed)" << endl;
    cout << "  Preload (parallel):   " << dt_load << "s (" << (dt_load/dt_feat*100) << "%)" << endl;
    cout << "  FindCalibObject (ser): " << dt_find << "s (" << (dt_find/dt_feat*100) << "%)" << endl;
    preloaded.clear(); vector<HObject>().swap(preloaded);  // free ~2.4GB

    // ========================================================================
    // Stage 3 — 共视图连通性校验
    // ========================================================================
    t0 = steady_clock::now();
    const int MIN_COVISIBILITY = 3;
    vector<vector<int>> adj_matrix(num_cams, vector<int>(num_cams, 0));

    for (const auto& obs_list : frame_observations) {
        for (size_t i = 0; i < obs_list.size(); ++i) {
            for (size_t j = i + 1; j < obs_list.size(); ++j) {
                int c1 = obs_list[i], c2 = obs_list[j];
                adj_matrix[c1][c2]++; adj_matrix[c2][c1]++;
            }
        }
    }

    vector<bool> visited(num_cams, false);
    queue<int> q;
    q.push(0); visited[0] = true;
    int connected_count = 1;

    while (!q.empty()) {
        int curr = q.front(); q.pop();
        for (int next_cam = 0; next_cam < num_cams; ++next_cam) {
            if (!visited[next_cam] && adj_matrix[curr][next_cam] >= MIN_COVISIBILITY) {
                visited[next_cam] = true;
                q.push(next_cam);
                connected_count++;
            }
        }
    }

    if (connected_count < num_cams) {
        cerr << "[Error] Camera Graph is Disconnected! (min " << MIN_COVISIBILITY << " co-visibility required)" << endl;
        for (int i = 0; i < num_cams; ++i) {
            if (!visited[i]) cerr << "  -> Camera " << i << " is isolated." << endl;
        }
        ClearCalibData(hv_CalibDataID);
        return;
    }
    double dt_graph = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 3 - Graph: " << fixed << setprecision(2) << dt_graph << "s (connected)" << endl;

    // ========================================================================
    // Stage 4 — 执行标定
    // ========================================================================
    t0 = steady_clock::now();
    try {
        CalibrateCameras(hv_CalibDataID, &hv_Errors);
    } catch (HException &ex) {
        cerr << "[Error] Calibration failed: " << ex.ErrorMessage().TextA() << endl;
        ClearCalibData(hv_CalibDataID);
        return;
    }
    double dt_calib = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 4 - Calibrate: " << fixed << setprecision(2) << dt_calib
         << "s (RMS error=" << hv_Errors.D() << " px)" << endl;

    // ========================================================================
    // Stage 5 — 重基准到中心相机
    // ========================================================================
    t0 = steady_clock::now();
    vector<HTuple> poses_in_cam0(num_cams);
    for (int camIdx = 0; camIdx < num_cams; ++camIdx) {
        if (camIdx == 0)
            CreatePose(0, 0, 0, 0, 0, 0, "Rp+T", "gba", "point", &poses_in_cam0[camIdx]);
        else
            GetCalibData(hv_CalibDataID, "camera", camIdx, "pose", &poses_in_cam0[camIdx]);
    }

    int center_idx = -1;
    if (!center_cam_sn.empty()) {
        for (int i = 0; i < num_cams; ++i) {
            if (cam_sn_list[i] == center_cam_sn) { center_idx = i; break; }
        }
        if (center_idx < 0) cerr << "[Warn] Center camera " << center_cam_sn << " not found, using cam 0" << endl;
    }

    HTuple T_center_to_cam0, T_cam0_to_center;
    if (center_idx >= 0) {
        T_center_to_cam0 = poses_in_cam0[center_idx];
        PoseInvert(T_center_to_cam0, &T_cam0_to_center);
    }

    vector<HTuple> poses_in_center(num_cams);
    for (int camIdx = 0; camIdx < num_cams; ++camIdx) {
        if (center_idx >= 0)
            PoseCompose(T_cam0_to_center, poses_in_cam0[camIdx], &poses_in_center[camIdx]);
        else
            poses_in_center[camIdx] = poses_in_cam0[camIdx];
    }
    double dt_rebase = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 5 - Rebase: " << fixed << setprecision(2) << dt_rebase << "s" << endl;

    // ========================================================================
    // Stage 6 — 导出 XML
    // ========================================================================
    t0 = steady_clock::now();
    for (int camIdx = 0; camIdx < num_cams; ++camIdx)
    {
        HTuple hv_CurrentCamParam, hv_CurrentPose, hv_StringPose;
        HTuple hv_FileHandle, hv_FullParamString;

        GetCalibData(hv_CalibDataID, "camera", camIdx, "params", &hv_CurrentCamParam);
        hv_CurrentPose = poses_in_center[camIdx];

        HTuple outFileName = hv_OutputBaseDir + "/" + HTuple(cam_sn_list[camIdx].c_str()) + "_Data.xml";
        OpenFile(outFileName, "output", &hv_FileHandle);

        FwriteString(hv_FileHandle, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        FwriteString(hv_FileHandle, "<CameraData>\n");

        FwriteString(hv_FileHandle, "  <InternalParameters>\n");
        HTuple type = ((const HTuple&)hv_CurrentCamParam)[0];
        HTuple vals = hv_CurrentCamParam.TupleSelectRange(1, hv_CurrentCamParam.TupleLength()-1);
        HTuple valStr;
        TupleString(vals, ".12g", &valStr);
        hv_FullParamString = type + " " + ((valStr + " ").TupleSum());
        FwriteString(hv_FileHandle, ("    <RawData>" + hv_FullParamString) + "</RawData>\n");
        FwriteString(hv_FileHandle, "  </InternalParameters>\n");

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
    }
    double dt_export = duration<double>(steady_clock::now() - t0).count();
    cout << "[Timer] Stage 6 - Export: " << fixed << setprecision(2) << dt_export
         << "s (" << num_cams << " XML files)" << endl;

    ClearCalibData(hv_CalibDataID);

    // ========================================================================
    // Total
    // ========================================================================
    double dt_total = duration<double>(steady_clock::now() - t_total_0).count();
    cout << "========================================" << endl;
    cout << "[Timer] Total: " << fixed << setprecision(2) << dt_total << "s" << endl;
    cout << "  Scan:    " << setw(8) << dt_scan   << "s (" << (dt_scan/dt_total*100)  << "%)" << endl;
    cout << "  Init:    " << setw(8) << dt_init   << "s (" << (dt_init/dt_total*100)  << "%)" << endl;
    cout << "  Feature: " << setw(8) << dt_feat   << "s (" << (dt_feat/dt_total*100)  << "%)" << endl;
    cout << "  Graph:   " << setw(8) << dt_graph  << "s (" << (dt_graph/dt_total*100) << "%)" << endl;
    cout << "  Calibrate:" << setw(7) << dt_calib << "s (" << (dt_calib/dt_total*100) << "%)" << endl;
    cout << "  Rebase:  " << setw(8) << dt_rebase << "s (" << (dt_rebase/dt_total*100)<< "%)" << endl;
    cout << "  Export:  " << setw(8) << dt_export << "s (" << (dt_export/dt_total*100)<< "%)" << endl;
    cout << endl;
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
    catch (const std::exception& e)
    {
        fprintf(stderr, "[Fatal] %s\n", e.what());
        ret = 1;
    }
    return ret;
}
#endif