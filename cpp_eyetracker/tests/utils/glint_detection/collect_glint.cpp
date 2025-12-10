#include <windows.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"
#include "utils/visualize.hpp"

using namespace glintdetection;
using namespace visualization;

bool relaxedSide2Side(const cv::Point2f& l_pt, const cv::Point2f& r_pt);

bool relaxedSide2Mid(
	const cv::Point2f& m_pt,
	const cv::Point2f& l_pt,
	const cv::Point2f& r_pt
);

std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, cv::Mat>
relaxedSearchForGlints(cv::Mat src, double firstEyeThresh);

std::vector<std::vector<cv::Point2f>>
relaxedFindGeometry(const std::vector<cv::Point2f>& glintCandidates);

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
            for (const auto& tri : leftTriList) {
                // tri format: [l_pt, m_pt, r_pt] based on push_back order in relaxedFindGeometry
                // 注意：你在 relaxedFindGeometry 里 push 的顺序是 l, m, r
                const auto& l = tri[0];
                const auto& m = tri[1];
                const auto& r = tri[2];
                csv_file << filename << ",left," 
                         << l.x << "," << l.y << "," 
                         << r.x << "," << r.y << "," 
                         << m.x << "," << m.y << "\n";
            }

            // Right Eyes
            for (const auto& tri : rightTriList) {
                const auto& l = tri[0];
                const auto& m = tri[1];
                const auto& r = tri[2];
                csv_file << filename << ",right," 
                         << l.x << "," << l.y << "," 
                         << r.x << "," << r.y << "," 
                         << m.x << "," << m.y << "\n";
            }

            // 4. 保存 Debug 图像 (threshold_output)
            std::string output_path = threshold_folder + "\\" + filename;
            if (!debug_img.empty()) {
                cv::imwrite(output_path, debug_img);
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

bool relaxedSide2Side(const cv::Point2f& l_pt, const cv::Point2f& r_pt)
{
	double lr_x = std::abs(l_pt.x - r_pt.x);
	double lr_y = std::abs(l_pt.y - r_pt.y);

	return (lr_x >= 5 && lr_x <= 20 && lr_y >= 0 && lr_y <= 7); // original 5 15 0 5
}

bool relaxedSide2Mid(
	const cv::Point2f& m_pt,
	const cv::Point2f& l_pt,
	const cv::Point2f& r_pt
)
{
	double lr_x = std::abs(l_pt.x - r_pt.x);
	double lr_y = std::abs(l_pt.y - r_pt.y);
	double lm_x = m_pt.x - l_pt.x;
	double lm_y = m_pt.y - l_pt.y;
	double rm_x = r_pt.x - m_pt.x;
	double rm_y = m_pt.y - r_pt.y;

	// 0
	// for simplicity, we only consider the case where left is lower than right
	if (l_pt.y < r_pt.y)
	{
		double temp_lm_x = lm_x;
		double temp_lm_y = lm_y;
		lm_x = rm_x;
		lm_y = rm_y;
		rm_x = temp_lm_x;
		rm_y = temp_lm_y;
	}

	// 1
	// left right is horizontal
	if (lr_y < 2)
	{
		// left to mid
		bool lm_x_condition = lm_x >= lr_x * 0.20 && lm_x <= lr_x * 0.80;
		bool lm_y_condition = lm_y >= lr_x * 0.05 && lm_y <= lr_x * 0.55;
		if (local_debug)
		{
			std::cout << "left to mid: (lr_y < 2)" << std::endl;
			std::cout << "lm_x: " << lm_x << " | lr_x: " << lr_x << " | lm_x/lr_x: " << lm_x / lr_x << " | qualified: " << lm_x_condition << std::endl;
			std::cout << "lm_y: " << lm_y << " | lr_x: " << lr_x << " | lm_y/lr_x: " << lm_y / lr_x << " | qualified: " << lm_y_condition <<std::endl;
		}
		bool lm_condition = lm_x_condition && lm_y_condition;

		bool rm_x_condition = rm_x >= lr_x * 0.20 && rm_x <= lr_x * 0.80;
		bool rm_y_condition = rm_y >= lr_x * 0.05 && rm_y <= lr_x * 0.55;
		if (local_debug)
		{
			std::cout << "right to mid: (lr_y < 2)" << std::endl;
			std::cout << "rm_x: " << rm_x << " | lr_x: " << lr_x << " | rm_x/lr_x: " << rm_x / lr_x << " | qualified: " << rm_x_condition << std::endl;
			std::cout << "rm_y: " << rm_y << " | lr_x: " << lr_x << " | rm_y/lr_x: " << rm_y / lr_x << " | qualified: " << rm_y_condition <<std::endl;
		}
		bool rm_condition = rm_x_condition && rm_y_condition;

		return lm_condition && rm_condition;

	}
	else if (lr_y >= 2 && lr_y < 3)
	{
		// left to mid
		bool lm_x_condition = lm_x >= lr_x * 0.25 && lm_x <= lr_x * 0.90;
		bool lm_y_condition = lm_y >= lr_x * 0.00 && lm_y <= lr_x * 0.45;
		if (local_debug)
		{
			std::cout << "left to mid: (2 <= lr_y < 3)" << std::endl;
			std::cout << "lm_x: " << lm_x << " | lr_x: " << lr_x
					  << " | lm_x/lr_x: " << lm_x / lr_x << " | qualified: " << lm_x_condition << std::endl;
			std::cout << "lm_y: " << lm_y << " | lr_x: " << lr_x
					  << " | lm_y/lr_x: " << lm_y / lr_x << " | qualified: " << lm_y_condition << std::endl;
		}
		bool lm_condition = lm_x_condition && lm_y_condition;

		// right to mid
		bool rm_x_condition = rm_x >= lr_x * 0.10 && rm_x <= lr_x * 0.75;
		bool rm_y_condition = rm_y >= lr_x * 0.05 && rm_y <= lr_x * 0.70;
		if (local_debug)
		{
			std::cout << "right to mid: (2 <= lr_y < 3)" << std::endl;
			std::cout << "rm_x: " << rm_x << " | lr_x: " << lr_x
					  << " | rm_x/lr_x: " << rm_x / lr_x << " | qualified: " << rm_x_condition << std::endl;
			std::cout << "rm_y: " << rm_y << " | lr_x: " << lr_x
					  << " | rm_y/lr_x: " << rm_y / lr_x << " | qualified: " << rm_y_condition << std::endl;
		}
		bool rm_condition = rm_x_condition && rm_y_condition;

		return lm_condition && rm_condition;
	}
	else if (lr_y >= 3 && lr_y <= 5)
	{
		// left to mid
		bool lm_x_condition = lm_x >= lr_x * 0.25 && lm_x <= lr_x * 0.95;
		bool lm_y_condition = lm_y >= lr_x * 0.00 && lm_y <= lr_x * 0.45;
		if (local_debug)
		{
			std::cout << "left to mid: (3 <= lr_y <= 5)" << std::endl;
			std::cout << "lm_x: " << lm_x << " | lr_x: " << lr_x
					  << " | lm_x/lr_x: " << lm_x / lr_x << " | qualified: " << lm_x_condition << std::endl;
			std::cout << "lm_y: " << lm_y << " | lr_y: " << lr_x
					  << " | lm_y/lr_x: " << lm_y / lr_x << " | qualified: " << lm_y_condition << std::endl;
		}
		bool lm_condition = lm_x_condition && lm_y_condition;

		// right to mid
		bool rm_x_condition = rm_x >= lr_x * 0.05 && rm_x <= lr_x * 0.75;
		bool rm_y_condition = rm_y >= lr_x * 0.10 && rm_y <= lr_x * 0.85;
		if (local_debug)
		{
			std::cout << "right to mid: (3 <= lr_y < 5)" << std::endl;
			std::cout << "rm_x: " << rm_x << " | lr_x: " << lr_x
					  << " | rm_x/lr_x: " << rm_x / lr_x << " | qualified: " << rm_x_condition << std::endl;
			std::cout << "rm_y: " << rm_y << " | lr_y: " << lr_x
					  << " | rm_y/lr_x: " << rm_y / lr_x << " | qualified: " << rm_y_condition << std::endl;
		}
		bool rm_condition = rm_x_condition && rm_y_condition;

		return lm_condition && rm_condition;
	}
	else
	{
		return false;
	}

	return false;
}


std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, cv::Mat>
relaxedSearchForGlints(cv::Mat src, double firstEyeThresh)
{
	cv::Mat gray;

	if (src.channels() == 3)
	{
		cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	}

	int kernel_size = cfg["test_glint"]["laplician_kernel_size"].as<int>(); // 3 better results in extreme cases
	double scale = 1;
	double delta = 0;
	int ddepth = CV_16S;

	// 1 Gauss
	// Remove noise by blurring with a Gaussian filter
	// Blurring also generates wider range for finding contours
	cv::Mat gaussed;
	cv::GaussianBlur(gray, gaussed,
					 cv::Size(cfg["test_glint"]["gaussion_kernel_size"].as<int>(),
					 		  cfg["test_glint"]["gaussion_kernel_size"].as<int>()),
					 0, 0, cv::BORDER_DEFAULT); // better results in extreme cases

	// 2 Laplace
	// Apply Laplace function
	// Laplace Functino generates edges
	cv::Mat laplaced;
	// time the laplacian function
	// cv::TickMeter tm;
	// tm.start();
	cv::Laplacian(gaussed, laplaced, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
	// tm.stop();
	// std::cout << "Laplace function took " << tm.getTimeMilli() << " ms" << std::endl;
	
	// CHANGED
	cv::Mat abs_dst;
	cv::convertScaleAbs(laplaced, abs_dst);
	// convertScaleAbs(laplaced, abs_dst, (sigma + 1)*0.25);

	// 3 Find Contours
	// Detect edges using Threshold
	cv::Mat threshold_output;
	cv::threshold(abs_dst, threshold_output, firstEyeThresh, 255, cv::THRESH_BINARY);

	// FOR SPEED UP
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(threshold_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// 4 Approximate contours to polygons + get bounding rects and circles
	std::vector<cv::RotatedRect> minRect(contours.size());
	std::vector<cv::Point2f> contourCenter(contours.size());

	if (local_debug)
	{
		std::cout << "\nTotal contours found: " << contours.size() << std::endl;
	}

	for (int i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(cv::Mat(contours[i]));
		contourCenter[i] = cv::Point2f(minRect[i].center.x, minRect[i].center.y);
		if (local_debug)
		{
			std::cout << "Contour " << i << ": Center = (" << contourCenter[i].x << ", " << contourCenter[i].y << ")" << std::endl;
		}
	}

	// Detect glints on sclera and remove them from list ======================================================================
	// auto [glintCandidates, img_rm] = removeFalseGlints(contourCenter, src);

	auto [leftEyeGlintsCandidates, rightEyeGlintsCandidates] = splitGlintsByEye(contourCenter);

	if (local_debug)
	{
		std::cout << "Left eye glint candidates: " << leftEyeGlintsCandidates.size() << std::endl;
		std::cout << "Right eye glint candidates: " << rightEyeGlintsCandidates.size() << std::endl;
		std::cout << "\nStart left eye glint geometry finding..." << std::endl;
	}

	auto leftEyeGlints  = relaxedFindGeometry(leftEyeGlintsCandidates);
	if (local_debug)
	{
		std::cout << "Left eye glints found: " << leftEyeGlints.size() << std::endl;
		std::cout << "\nStart right eye glint geometry finding..." << std::endl;
	}
	auto rightEyeGlints = relaxedFindGeometry(rightEyeGlintsCandidates);
	if (local_debug)
	{
		std::cout << "Right eye glints found: " << rightEyeGlints.size() << std::endl;
	}
	
	return {leftEyeGlints, rightEyeGlints, threshold_output};

} // searchForGlints()


std::vector<std::vector<cv::Point2f>>
relaxedFindGeometry(const std::vector<cv::Point2f>& glintCandidates)
{
	std::vector<std::vector<cv::Point2f>> glintGeometryCandidates;

	// Go throuth each Point2f in list
	for (int i = 0; i < glintCandidates.size(); i++)
	{
		cv::Point2f temp_pt_1 = glintCandidates[i];
		for (int j = i + 1; j < glintCandidates.size(); j++)
		{
			cv::Point2f temp_pt_2 = glintCandidates[j];
			cv::Point2f l_pt = temp_pt_1.x < temp_pt_2.x ? temp_pt_1 : temp_pt_2;
			cv::Point2f r_pt = temp_pt_1.x > temp_pt_2.x ? temp_pt_1 : temp_pt_2;

			// 1
			// find horizontal pair
			if (relaxedSide2Side(l_pt, r_pt)) // original 15 5 5 0
			{
				if (local_debug)
				{
					std::cout << "1 found horizontal pair" << std::endl;
					std::cout << "left: (" << l_pt.x << ", " << l_pt.y << ")" << std::endl;
					std::cout << "right: (" << r_pt.x << ", " << r_pt.y << ")" << std::endl;
				}
				for (int k = 0; k < glintCandidates.size(); k++)
				{
					if (k == i || k == j) continue;

					cv::Point2f m_pt = glintCandidates[k];
					if (local_debug) std::cout << "Checking potential mid down at: ("
											   << m_pt.x << ", " << m_pt.y << ")" << std::endl;

					// 2
					// find mid point
					if (relaxedSide2Mid(m_pt, l_pt, r_pt))
					{
						if (local_debug)
						{
							std::cout << "2 found mid point" << std::endl;
							std::cout << "mid: (" << m_pt.x << ", " << m_pt.y << ")" << std::endl;
						}

						std::vector<cv::Point2f> glintGeometryCandidate;
						glintGeometryCandidate.push_back(l_pt);
						glintGeometryCandidate.push_back(m_pt);
						glintGeometryCandidate.push_back(r_pt);

						glintGeometryCandidates.push_back(glintGeometryCandidate);
					}
				}
			}
		}
	}

	return glintGeometryCandidates;
}