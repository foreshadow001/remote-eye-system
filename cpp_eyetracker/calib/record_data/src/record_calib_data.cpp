#include "record_calib_data.hpp"
#include <filesystem>
#include <iostream>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;
using namespace gazeestimation;

bool recordCalibrationSequence(
    BaslerCamera& cam,
    const std::string& image_dir,
    const std::string& video_dir
)
{
    vector<fs::path> image_files;
    for (auto& p : fs::directory_iterator(image_dir)) {
        if (p.path().extension() == ".jpg") image_files.push_back(p.path());
    }
    sort(image_files.begin(), image_files.end());
    if (image_files.empty()) {
        cerr << "[record] No images found in " << image_dir << endl;
        return false;
    }

    fs::create_directories(video_dir);

    cout << "[record] Found " << image_files.size() << " calibration points.\n";
    cout << "Press 'n' for next image, 'r' to start recording, SPACE to stop, ESC to exit.\n";

    for (size_t i = 0; i < image_files.size(); ++i) {
        Mat img = imread(image_files[i].string());
        if (img.empty()) continue;

        string filename = image_files[i].filename().string();
        string base_name = filename.substr(0, filename.find_last_of('.'));
        string video_path = (fs::path(video_dir) / (base_name + ".mp4")).string();

        cout << "[record] Showing: " << filename << endl;
        imshow("Calibration", img);
        cv::setWindowProperty("Calibration", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        bool recording = false;
        while (true) {
            int key = waitKey(10);
            if (key == 'n') {
                break;
            } else if (key == 'r' && !recording) {
                cam.startRecording(video_path);
                recording = true;
            } else if (key == 32 && recording) { // space
                cam.stopRecording();
                recording = false;
                break;
            } else if (key == 27) { // esc
                cam.stopRecording();
                return true;
            }
        }
    }

    destroyAllWindows();
    return true;
}
