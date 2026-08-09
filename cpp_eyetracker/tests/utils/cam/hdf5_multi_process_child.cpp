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

using namespace std;

int main(int argc, char* argv[]) {
    // Expected arguments (10 values after the program name):
    // argv[1]  = camera_index    argv[2]  = hdf5_dir
    // argv[3]  = chunk_idx       argv[4]  = frame_offset
    // argv[5]  = core_frames     argv[6]  = cam_h
    // argv[7]  = cam_w           argv[8]  = margin_frames
    // argv[9]  = shm_name        argv[10] = gaze_x
    // argv[11] = gaze_y          argv[12] = gaze_z
    if (argc < 13) {
        cerr << "Usage: " << argv[0]
             << " <camera_index> <hdf5_dir> <chunk_idx> <frame_offset>"
             << " <core_frames> <cam_h> <cam_w> <margin_frames> <shm_name>"
             << " <gaze_x> <gaze_y> <gaze_z>"
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

        // ---- valid: all ones (tiny, ~0.001s) ----
        {
            hsize_t v_start[1] = {(hsize_t)frame_offset};
            hsize_t v_count[1] = {(hsize_t)N};
            H5::DataSpace v_mem(1, v_count);
            H5::DataSpace v_file = valid_ds.getSpace();
            v_file.selectHyperslab(H5S_SELECT_SET, v_count, v_start);
            vector<uint8_t> v_buf((size_t)N, 1);
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
