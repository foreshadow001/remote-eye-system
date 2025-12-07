#include <windows.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "glint_detection/detect_glint.h"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"

using namespace glintdetection;
using namespace visualization;

int main() {
    Cfg cfg;
    std::string input_folder = cfg["test_glint"]["input_folder"].as<std::string>();
    std::string output_folder = cfg["test_glint"]["output_folder"].as<std::string>();
    // create output folder if not exist
    CreateDirectory(output_folder.c_str(), NULL);

    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "the input folder does not exist or is not a folder: " << input_folder << std::endl;
        return -1;
    }

    int idx = 0;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        {
            std::string filename = fd.cFileName;
            std::string filepath = input_folder + "\\" + filename;

            cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);
            if (img.empty())
            {
                std::cerr << "failed to read image: " << filepath << std::endl;
                continue;
            }

            // call searchForGlints
            double threshold = cfg["test_glint"]["threshold"].as<double>();
            auto [leftEyeGlints, rightEyeGlints, debug_img] = searchForGlints(img, threshold);

			std::vector<cv::Point2d> left_glints, right_glints;

			for (const auto& g : leftEyeGlints) {
				left_glints.emplace_back(g.x, g.y);
			}

            for (const auto& g : rightEyeGlints) {
                right_glints.emplace_back(g.x, g.y);
			}

            // draw the glints
            cv::Mat left_viz = visualizeGlints(img, left_glints);
            cv::Mat viz = visualizeGlints(left_viz, right_glints);

            // save the image
            std::string index = std::to_string(idx);
            std::string num_glints = std::to_string(leftEyeGlints.size() + rightEyeGlints.size());
            std::string output_path = output_folder + "\\" + filename;
            cv::imwrite(output_path, viz);

            std::cout << "saved to: " << output_path << " | num of glints: " << num_glints << std::endl;
            idx++;
        }
    } while ((::FindNextFile(hFind, &fd) != 0) && (idx < cfg["test_glint"]["num_images"].as<int>()));
    ::FindClose(hFind);

    std::cout << "processed " << idx << " images." << std::endl;
    return 0;
}
