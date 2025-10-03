#include <windows.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "glint_detection/detect_glint.h"

using namespace glintdetection;

const std::string input_folder  = "D:/users/projects/new_dataset/data_collection/PCCR/test_dataset/images/src";
const std::string output_folder = "D:/users/projects/new_dataset/data_collection/PCCR/test_dataset/images/gaussed";

int main() {
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
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            std::string filename = fd.cFileName;
            std::string filepath = input_folder + "\\" + filename;

            cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);
            if (img.empty()) {
                std::cerr << "failed to read image: " << filepath << std::endl;
                continue;
            }

            // call searchForGlints
            auto result = searchForGlints(img, 50.0);
            const auto& glints  = result.first;
            const auto& gaussed = result.second;

            // save the gaussed image
            std::string outpath = output_folder + "\\" + filename;
            cv::imwrite(outpath, gaussed);

            std::cout << "saved to " << outpath << " | num of glints: " << glints.size() << std::endl;
            idx++;
        }
    } while (::FindNextFile(hFind, &fd));
    ::FindClose(hFind);

    std::cout << "processed " << idx << " images." << std::endl;
    return 0;
}
