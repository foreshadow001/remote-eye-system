#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"
#include "logger/logger.hpp"

using namespace glintdetection;
using namespace visualization;

int main() {
    Cfg cfg;
    GlintDetector glint_detector("inference");
    if (cfg["test_glint"]["debug_time"].as<bool>())
    {
        Logger::setLevel(Logger::Level::TIME);
    }

    if (cfg["test_glint"]["local_debug"].as<bool>())
    {
        Logger::setLevel(Logger::Level::DEBUG);
        glint_detector.local_debug_ = true;
    }

    std::string input_folder = cfg["test_glint"]["input_folder"].as<std::string>();
    std::string threshold_output_folder = input_folder + "\\threshold_output";
    std::string debug_img_output_folder = input_folder + "\\debug_img";

    std::filesystem::path threshold_folder_path(threshold_output_folder);
    if (!std::filesystem::exists(threshold_folder_path)) {
        std::filesystem::create_directories(threshold_folder_path);
    }

    std::filesystem::path debug_img_folder_path(debug_img_output_folder);
    if (!std::filesystem::exists(debug_img_folder_path)) {
        std::filesystem::create_directories(debug_img_folder_path);
    }

    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = FindFirstFile(search_path.c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) {
        Logger::error() << "Input folder does not exist or is not a folder: " << input_folder;
        return -1;
    }

    int idx = 0;
    int max_images = cfg["test_glint"]["num_images"].as<int>();

    do {
        std::string filename = fd.cFileName;

        // 跳过 "." 和 ".."
        if (filename == "." || filename == "..")
            continue;

        // 只处理文件，不处理目录
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            std::string filepath = input_folder + "\\" + filename;
            // 去掉后缀名
            std::string filename_no_ext = filename.substr(0, filename.find_last_of('.'));
            glint_detector.img_name_ = filename_no_ext;

            cv::Mat img = cv::imread(filepath, cv::IMREAD_GRAYSCALE);
            if (img.empty()) {
                Logger::error() << "Failed to read image: " << filepath;
                continue;
            }

            // 调用 glint 检测
            auto [leftEyeGlintsList, rightEyeGlintsList] = glint_detector.detectFullImage(img);

            std::vector<std::vector<cv::Point2d>> left_glints_list, right_glints_list;
            for (const auto& leftGlints : leftEyeGlintsList)
            {
                std::vector<cv::Point2d> glints;
                for (const auto& g : leftGlints)
                    glints.emplace_back(g.x, g.y);
                left_glints_list.emplace_back(glints);
            }
            for (const auto& rightGlints : rightEyeGlintsList)
            {
                std::vector<cv::Point2d> glints;
                for (const auto& g : rightGlints)
                    glints.emplace_back(g.x, g.y);
                right_glints_list.emplace_back(glints);
            }

            // 保存输出
            cv::Mat threshold_output = glint_detector.threshold_output_.clone();
            if (!glint_detector.debug_imgs_.empty())
            {
                for (int i = 0; i < glint_detector.debug_imgs_.size(); i++)
                {
                    cv::imwrite(
                        debug_img_output_folder + "\\" + "viz_" + std::to_string(i) + "_" + filename, 
                        glint_detector.debug_imgs_[i]
                    );
                }
            }
            cv::imwrite(threshold_output_folder + "\\" + filename, threshold_output);

            Logger::info() << "Saved to: " << threshold_output_folder + "\\" + filename
                            << " | num of glints: " << left_glints_list.size() << " | " << right_glints_list.size();

            idx++;
            if (idx >= max_images)
                break;  // 达到处理上限就退出
        }
    } while (FindNextFile(hFind, &fd) != 0);

    FindClose(hFind);
    Logger::info() << "Processed " << idx << " images.";
    return 0;
}
