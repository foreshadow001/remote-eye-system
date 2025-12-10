#include <windows.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"

using namespace glintdetection;
using namespace visualization;

Cfg cfg;

bool local_debug = cfg["collect_glint"]["local_debug"].as<bool>();

int main() {
    std::string input_folder = cfg["collect_glint"]["input_folder"].as<std::string>();
    std::string output_folder = cfg["collect_glint"]["output_folder"].as<std::string>();
    std::string threshold_folder = cfg["collect_glint"]["output_folder"].as<std::string>() + "\\threshold_output";

    // 创建输出文件夹
    CreateDirectory(output_folder.c_str(), NULL);
    CreateDirectory(threshold_folder.c_str(), NULL);

    // 1. 准备 CSV 文件
    std::string csv_path = output_folder + "\\" + "glint_data.csv";
    std::ofstream csv_file(csv_path);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open CSV file for writing: " << csv_path << std::endl;
        return -1;
    }
    // 写入表头
    csv_file << "img_name,side,lx,ly,rx,ry,mx,my\n";

    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "Input folder invalid: " << input_folder << std::endl;
        return -1;
    }

    int idx = 0;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        {
            std::string filename = fd.cFileName;
            std::string filepath = input_folder + "\\" + filename;

			// 去掉文件拓展名
			size_t pos = filename.find_last_of('.');
			if (pos != std::string::npos) {
				filename.erase(pos);
			}

            cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);
            if (img.empty()) {
                std::cerr << "Failed to read image: " << filepath << std::endl;
                continue;
            }

            double threshold = cfg["test_glint"]["threshold"].as<double>();

            // 2. 调用宽松搜索，获取所有候选组合和Debug图
            auto [leftTriList, rightTriList, debug_img] = relaxedSearchForGlints(img, threshold);

            // 3. 写入 CSV 数据
            // Left Eyes
			int counter = 0;

            for (const auto& tri : leftTriList) {
                const auto& l = tri[0];
                const auto& r = tri[1];
                const auto& m = tri[2];

				std::vector<cv::Point2d> left_glints;

				left_glints.emplace_back(l.x, l.y);
				left_glints.emplace_back(r.x, r.y);
				left_glints.emplace_back(m.x, m.y);

				cv::Mat src_copy_left = img.clone();
				cv::Mat left_viz = visualizeGlints(src_copy_left, left_glints);
				std::string output_filename = filename + "_" + std::to_string(counter) + "_left.png";
				std::string output_path = threshold_folder + "\\" + output_filename;
				cv::imwrite(output_path, left_viz);

                csv_file << output_filename << ",left," 
                         << l.x << "," << l.y << "," 
                         << r.x << "," << r.y << "," 
                         << m.x << "," << m.y << "\n";

				counter++;
            }

            // Right Eyes
            for (const auto& tri : rightTriList) {
                const auto& l = tri[0];
                const auto& r = tri[1];
                const auto& m = tri[2];

				std::vector<cv::Point2d> right_glints;

				right_glints.emplace_back(l.x, l.y);
				right_glints.emplace_back(r.x, r.y);
				right_glints.emplace_back(m.x, m.y);

				cv::Mat src_copy_right = img.clone();
				cv::Mat right_viz = visualizeGlints(src_copy_right, right_glints);
				std::string output_filename = filename + "_" + std::to_string(counter) + "_right.png";
				std::string output_path = threshold_folder + "\\" + output_filename;
				cv::imwrite(output_path, right_viz);

                csv_file << output_filename << ",right," 
                         << l.x << "," << l.y << "," 
                         << r.x << "," << r.y << "," 
                         << m.x << "," << m.y << "\n";

				counter++;
            }

            std::cout << "Processed: " << filename 
                      << " | L-Cands: " << leftTriList.size() 
                      << " | R-Cands: " << rightTriList.size() << std::endl;
            idx++;
        }
    } while ((::FindNextFile(hFind, &fd) != 0) && (idx < cfg["collect_glint"]["num_images"].as<int>()));
    ::FindClose(hFind);
    
    csv_file.close();
    std::cout << "Done. Processed " << idx << " images. Data saved to " << csv_path << std::endl;
    return 0;
}
