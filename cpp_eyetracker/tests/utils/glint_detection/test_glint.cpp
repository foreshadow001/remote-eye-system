#include <windows.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"

using namespace glintdetection;
using namespace visualization;

int main() {
    Cfg cfg;

    std::string input_folder = cfg["test_glint"]["input_folder"].as<std::string>();
    std::string output_folder = cfg["test_glint"]["output_folder"].as<std::string>();

    // 创建输出文件夹（如果不存在）
    CreateDirectory(output_folder.c_str(), NULL);

    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = FindFirstFile(search_path.c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "Input folder does not exist or is not a folder: " << input_folder << std::endl;
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

            cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);
            if (img.empty()) {
                std::cerr << "Failed to read image: " << filepath << std::endl;
                continue;
            }

            // 调用 glint 检测
            auto [leftEyeGlintsList, rightEyeGlintsList, debug_img] = searchForGlints(img, cfg);

            std::vector<cv::Point2f> leftEyeGlints, rightEyeGlints;

            if (!leftEyeGlintsList.empty())
                leftEyeGlints = leftEyeGlintsList[0];
            if (!rightEyeGlintsList.empty())
                rightEyeGlints = rightEyeGlintsList[0];

            std::vector<cv::Point2d> left_glints, right_glints;
            for (const auto& g : leftEyeGlints)
                left_glints.emplace_back(g.x, g.y);
            for (const auto& g : rightEyeGlints)
                right_glints.emplace_back(g.x, g.y);

            // 可视化 glints
            cv::Mat left_viz = visualizeGlints(img, left_glints);
            cv::Mat viz = visualizeGlints(left_viz, right_glints);

            // 保存输出
            std::string output_path = output_folder + "\\" + filename;
            cv::imwrite(output_path, viz);

            std::cout << "Saved to: " << output_path
                      << " | num of glints: " << leftEyeGlints.size() + rightEyeGlints.size() << std::endl;

            idx++;
            if (idx >= max_images)
                break;  // 达到处理上限就退出
        }
    } while (FindNextFile(hFind, &fd) != 0);

    FindClose(hFind);
    std::cout << "Processed " << idx << " images." << std::endl;
    return 0;
}
