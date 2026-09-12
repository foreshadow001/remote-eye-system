// ================== hdf5_multi_process_child.cpp ==================
// Standalone child process for single-camera HDF5 writing.
// Reads raw image data from shared memory (zero-copy) and writes all three
// datasets (raw_image, gaze_target, valid) to a pre-created .h5 file.
//
// Exit codes: 0 = success, 1 = HDF5 error, 2 = shared memory error

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#else
    #error "This child process is Windows-only (uses Win32 shared memory APIs)"
#endif

#include <H5Cpp.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdlib>
#include <limits>
#include <filesystem>

namespace fs = std::filesystem;
using namespace std;

// ---- precreate 模式: 串行创建 <dir> 下全部 chunk 文件 ----
// 用法: hdf5_multi_process_child.exe --precreate <hdf5_dir> <n_chunks> <capacity> <cam_h> <cam_w>
// 每相机一个进程由主程序拉起 (10 路并行); 数据集布局/顺序必须与主程序
// precreateParallel/dump Step 0 逐字一致 (布局在 raw_image 前的小数据集决定 NTFS 行为)
static int runPrecreate(int argc, char* argv[]) {
    string dir       = argv[2];
    int    n_chunks  = atoi(argv[3]);
    int    capacity  = atoi(argv[4]);
    int    cam_h     = atoi(argv[5]);
    int    cam_w     = atoi(argv[6]);
    int    created   = 0;
    for (int ci = 0; ci < n_chunks; ++ci) {
        stringstream pss; pss << dir << "/" << setw(4) << setfill('0') << ci << ".h5";
        if (fs::exists(pss.str())) continue;              // 补创建: 已存在 (含已录数据) 跳过
        try {
            H5::H5File f(pss.str(), H5F_ACC_TRUNC);
            H5::DSetCreatPropList pl;
            pl.setAllocTime(H5D_ALLOC_TIME_EARLY);        // 创建即分配 (文件扩展到满尺寸)
            pl.setFillTime(H5D_FILL_TIME_NEVER);          // 不逐字节清零 ~10GB
            hsize_t gd[2] = {(hsize_t)capacity, 3};
            f.createDataSet("gaze_target", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, gd), pl);
            // 判定元数据+关节 合一 [42]: arm,status,err[2],flange[14],arm_pose[12],joints[12]
            // (总 dataset 数须 ≤8: 恰好 9 个时 HDF5 close 触发 NTFS 对 9.3GB raw 区零填充 ~4s/文件)
            hsize_t md[2] = {(hsize_t)capacity, 42};
            f.createDataSet("occ_meta", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, md), pl);
            hsize_t vd[1] = {(hsize_t)capacity};
            f.createDataSet("valid", H5::PredType::NATIVE_UINT8, H5::DataSpace(1, vd), pl);
            hsize_t rd[3] = {(hsize_t)capacity, (hsize_t)cam_h, (hsize_t)cam_w};
            f.createDataSet("raw_image", H5::PredType::NATIVE_UINT8, H5::DataSpace(3, rd), pl);
            ++created;
        } catch (const H5::Exception& e) {
            cerr << "[Precreate " << fs::path(dir).filename().string() << "] chunk " << ci
                 << " FAILED: " << e.getCDetailMsg() << endl;
            return 1;
        }
    }
    cout << "[Precreate " << fs::path(dir).filename().string() << "] " << created << " files created" << endl;
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc == 7 && string(argv[1]) == "--precreate") return runPrecreate(argc, argv);

    // Expected arguments (10 values after the program name):
    // argv[1]  = camera_index    argv[2]  = hdf5_dir
    // argv[3]  = chunk_idx       argv[4]  = frame_offset
    // argv[5]  = core_frames     argv[6]  = cam_h
    // argv[7]  = cam_w           argv[8]  = margin_frames
    // argv[9]  = shm_name        argv[10] = gaze_x
    // argv[11] = gaze_y          argv[12] = gaze_z
    // argv[13] = occluded (0/1, 可选; 1 = 本相机此目标被遮挡, valid 整段写 0)
    // argv[14] = joints (可选; "qU0,...,qU5,qL0,...,qL5" 逗号表 或 "-"; 写 occ_joints, "-" → NaN)
    // argv[15] = occ_status (可选; 判定状态, 写 occ_status: 0正常 2查询失败 3下发失败 4对账失配)
    // argv[16] = err_upper  / argv[17] = err_lower (可选; 对账误差 mm, "nan"→NaN; 写 occ_check_err)
    // argv[18] = occ_arm (可选; 录制臂 0=upper 1=lower; 写 occ_arm)
    // argv[19] = flange 14 逗号表 (可选; SDK 法兰位姿 U7+L7, "nan"→NaN; 写 occ_flange)
    // argv[20] = arm_pose 12 逗号表 (可选; 运行时手眼值 Ut3r3+Lt3r3; 写 occ_arm_pose)
    if (argc < 13) {
        cerr << "Usage: " << argv[0]
             << " <camera_index> <hdf5_dir> <chunk_idx> <frame_offset>"
             << " <core_frames> <cam_h> <cam_w> <margin_frames> <shm_name>"
             << " <gaze_x> <gaze_y> <gaze_z> [occluded] [joints] [status] [errU] [errL]"
             << endl;
        return 2;
    }

    int    cam_idx       = atoi(argv[1]);
    string hdf5_dir      = argv[2];
    int    chunk_idx     = atoi(argv[3]);
    int    frame_offset  = atoi(argv[4]);
    int    N             = atoi(argv[5]);   // core_frames
    int    cam_h         = atoi(argv[6]);
    int    cam_w         = atoi(argv[7]);
    int    margin_frames = atoi(argv[8]);
    string shm_name      = argv[9];
    double gaze_x        = atof(argv[10]);
    double gaze_y        = atof(argv[11]);
    double gaze_z        = atof(argv[12]);
    int    occluded      = (argc > 13) ? atoi(argv[13]) : 0;
    int    occ_status    = (argc > 15) ? atoi(argv[15]) : 0;
    float  occ_err[2];
    occ_err[0] = (argc > 16) ? (float)atof(argv[16]) : numeric_limits<float>::quiet_NaN();
    occ_err[1] = (argc > 17) ? (float)atof(argv[17]) : numeric_limits<float>::quiet_NaN();
    int    occ_arm       = (argc > 18) ? atoi(argv[18]) : 0;
    double occ_flange[14], occ_arm_pose[12];
    {   const double NAN_VAL = numeric_limits<double>::quiet_NaN();
        auto parse_list = [&](const char* s, double* out, int n) {
            for (int k = 0; k < n; ++k) out[k] = NAN_VAL;
            if (!s || !*s) return;
            stringstream js(s); string tok; int k = 0;
            while (k < n && getline(js, tok, ',')) {
                try { out[k++] = stod(tok); } catch (...) { return; }
            } };
        parse_list(argc > 19 ? argv[19] : nullptr, occ_flange, 14);
        parse_list(argc > 20 ? argv[20] : nullptr, occ_arm_pose, 12); }

    // 判定所用关节 (qU6+qL6; 无效/缺失 → NaN, 与图像/valid 同时序可辨识)
    // 无效标记判别必须整串比对 "-": 负关节角表 "-0.182,...,-" 首字符也是 '-',
    // 曾被 argv[14][0] != '-' 误判 → 整表丢弃写 NaN (NaN 三采根因)
    double joints[12];
    {
        const double NAN_VAL = numeric_limits<double>::quiet_NaN();
        bool ok = false;
        if (argc > 14 && strlen(argv[14]) > 1) {
            ok = true;
            stringstream js(argv[14]); string tok;
            int k = 0;
            while (k < 12 && getline(js, tok, ',')) {
                try { joints[k++] = stod(tok); }
                catch (...) { ok = false; break; }
            }
            ok = ok && (k == 12);
        }
        if (!ok) for (int k = 0; k < 12; ++k) joints[k] = NAN_VAL;
    }

    // ---- Open shared memory ----
    // Must map margin_frames + N frames to reach the core data region.
    // The margin frames are pre-trigger redundancy that we skip over.
    size_t map_size = (size_t)(margin_frames + N) * (size_t)cam_h * (size_t)cam_w;
    HANDLE hMap = OpenFileMappingA(FILE_MAP_READ, FALSE, shm_name.c_str());
    if (!hMap) {
        cerr << "[Child cam " << cam_idx << "] OpenFileMapping failed for \""
             << shm_name << "\" (error " << GetLastError() << ")" << endl;
        return 2;
    }

    uint8_t* base_data = (uint8_t*)MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, map_size);
    if (!base_data) {
        cerr << "[Child cam " << cam_idx << "] MapViewOfFile failed (error "
             << GetLastError() << ")" << endl;
        CloseHandle(hMap);
        return 2;
    }

    // Skip the margin frames to align with the core recording frames
    uint8_t* core_data = base_data + (size_t)margin_frames * (size_t)cam_h * (size_t)cam_w;

    // ---- Write HDF5 ----
    // File and all three datasets (raw_image, gaze_target, valid) are
    // pre-created by the parent process.  We open in RDWR mode and write.
    try {
        stringstream ss;
        ss << hdf5_dir << "/" << setw(4) << setfill('0') << chunk_idx << ".h5";
        H5::H5File f(ss.str(), H5F_ACC_RDWR);

        H5::DataSet raw_ds   = f.openDataSet("raw_image");
        H5::DataSet gaze_ds  = f.openDataSet("gaze_target");
        H5::DataSet valid_ds = f.openDataSet("valid");

        // ---- raw_image: hyperslab write, one frame at a time ----
        hsize_t f_start[3] = {0, 0, 0};
        hsize_t f_count[3] = {1, (hsize_t)cam_h, (hsize_t)cam_w};
        H5::DataSpace f_mem(3, f_count);

        for (int i = 0; i < N; ++i) {
            f_start[0] = (hsize_t)(frame_offset + i);
            H5::DataSpace f_file = raw_ds.getSpace();
            f_file.selectHyperslab(H5S_SELECT_SET, f_count, f_start);
            raw_ds.write(core_data + (size_t)i * (size_t)cam_h * (size_t)cam_w,
                         H5::PredType::NATIVE_UINT8, f_mem, f_file);
        }

        // ---- gaze_target: write actual gaze (x,y,z) for all N frames ----
        {
            hsize_t gz_start[2] = {(hsize_t)frame_offset, 0};
            hsize_t gz_count[2] = {(hsize_t)N, 3};
            H5::DataSpace gz_mem(2, gz_count);
            H5::DataSpace gz_file = gaze_ds.getSpace();
            gz_file.selectHyperslab(H5S_SELECT_SET, gz_count, gz_start);
            vector<double> gz_buf((size_t)N * 3);
            for (int i = 0; i < N; ++i) {
                gz_buf[i * 3]     = gaze_x;
                gz_buf[i * 3 + 1] = gaze_y;
                gz_buf[i * 3 + 2] = gaze_z;
            }
            gaze_ds.write(gz_buf.data(), H5::PredType::NATIVE_DOUBLE, gz_mem, gz_file);
        }

        // ---- occ_meta: 判定元数据+关节 合一 [42] (每录恒定) ----
        // 列: [0]arm [1]status [2:4]check_err [4:18]flange [18:30]arm_pose [30:42]joints
        // 旧文件可能缺该数据集 (预创建早于本格式) → 跳过, 不影响其他写入
        try {
            H5::DataSet md_ds = f.openDataSet("occ_meta");
            hsize_t md_start[2] = {(hsize_t)frame_offset, 0};
            hsize_t md_count[2] = {(hsize_t)N, 42};
            H5::DataSpace md_mem(2, md_count);
            H5::DataSpace md_file = md_ds.getSpace();
            md_file.selectHyperslab(H5S_SELECT_SET, md_count, md_start);
            vector<double> md_buf((size_t)N * 42);
            for (int i = 0; i < N; ++i) {
                double* row = &md_buf[i * 42];
                row[0] = occ_arm; row[1] = occ_status;
                row[2] = occ_err[0]; row[3] = occ_err[1];
                for (int k = 0; k < 14; ++k) row[4 + k]  = occ_flange[k];
                for (int k = 0; k < 12; ++k) row[18 + k] = occ_arm_pose[k];
                for (int k = 0; k < 12; ++k) row[30 + k] = joints[k];
            }
            md_ds.write(md_buf.data(), H5::PredType::NATIVE_DOUBLE, md_mem, md_file);
        } catch (const H5::Exception&) {}

        // ---- valid: 被遮挡相机整段写 0, 其余全 1 (tiny, ~0.001s) ----
        {
            hsize_t v_start[1] = {(hsize_t)frame_offset};
            hsize_t v_count[1] = {(hsize_t)N};
            H5::DataSpace v_mem(1, v_count);
            H5::DataSpace v_file = valid_ds.getSpace();
            v_file.selectHyperslab(H5S_SELECT_SET, v_count, v_start);
            vector<uint8_t> v_buf((size_t)N, occluded ? 0 : 1);
            valid_ds.write(v_buf.data(), H5::PredType::NATIVE_UINT8, v_mem, v_file);
        }

        // H5File destructor closes the file
    } catch (const H5::Exception& e) {
        cerr << "[Child cam " << cam_idx << "] HDF5 error: " << e.getCDetailMsg() << endl;
        UnmapViewOfFile(base_data);
        CloseHandle(hMap);
        return 1;
    }

    UnmapViewOfFile(base_data);
    CloseHandle(hMap);
    return 0;
}
