#include <future>
#include <list>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <limits>
#include <cmath>

#include "glint_detection/detect_glint.hpp"
#include "cfg/config.hpp"


namespace glintdetection {

Cfg cfg;

bool local_debug = cfg["test_glint"]["local_debug"].as<bool>();

// TODO:
// 1. optimaize side2mid function (add delta y of left and right point as a parameter, and adjust the range accordingly).
// 2. collect all the possible glint combinations, and choose the best one according to some criteria. ，  

bool side2side(const cv::Point2f& l_pt, const cv::Point2f& r_pt)
{
	double lr_x = std::abs(l_pt.x - r_pt.x);
	double lr_y = std::abs(l_pt.y - r_pt.y);

	return (lr_x >= 7 && lr_x <= 16 && lr_y >= 0 && lr_y <= 5); // original 5 15 0 5
}

bool side2mid(
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
		bool lm_x_condition = lm_x >= lr_x * 0.40 && lm_x <= lr_x * 0.60;
		bool lm_y_condition = lm_y >= lr_x * 0.15 && lm_y <= lr_x * 0.35;
		if (local_debug)
		{
			std::cout << "left to mid: (lr_y < 2)" << std::endl;
			std::cout << "lm_x: " << lm_x << " | lr_x: " << lr_x << " | lm_x/lr_x: " << lm_x / lr_x << " | qualified: " << lm_x_condition << std::endl;
			std::cout << "lm_y: " << lm_y << " | lr_x: " << lr_x << " | lm_y/lr_x: " << lm_y / lr_x << " | qualified: " << lm_y_condition <<std::endl;
		}
		bool lm_condition = lm_x_condition && lm_y_condition;

		bool rm_x_condition = rm_x >= lr_x * 0.40 && rm_x <= lr_x * 0.60;
		bool rm_y_condition = rm_y >= lr_x * 0.15 && rm_y <= lr_x * 0.35;
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
		bool lm_x_condition = lm_x >= lr_x * 0.45 && lm_x <= lr_x * 0.70;
		bool lm_y_condition = lm_y >= lr_x * 0.05 && lm_y <= lr_x * 0.25;
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
		bool rm_x_condition = rm_x >= lr_x * 0.30 && rm_x <= lr_x * 0.55;
		bool rm_y_condition = rm_y >= lr_x * 0.25 && rm_y <= lr_x * 0.50;
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
		bool lm_x_condition = lm_x >= lr_x * 0.45 && lm_x <= lr_x * 0.85;
		bool lm_y_condition = lm_y >= lr_x * 0.04 && lm_y <= lr_x * 0.25;
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
		bool rm_x_condition = rm_x >= lr_x * 0.15 && rm_x <= lr_x * 0.55;
		bool rm_y_condition = rm_y >= lr_x * 0.30 && rm_y <= lr_x * 0.65;
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

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
splitGlintsByEye(const std::vector<cv::Point2f>& glints, float distanceThrX, float outlierThr)
{
    std::vector<cv::Point2f> leftEye, rightEye;
    if (glints.empty()) return {leftEye, rightEye};

    std::vector<cv::Point2f> sorted = glints;
    std::sort(sorted.begin(), sorted.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });

    const float refX = sorted.front().x;

    // 按照左右分割
    for (const auto& p : sorted) {
        if (std::abs(p.x - refX) < distanceThrX)
            rightEye.push_back(p);
        else
            leftEye.push_back(p);
    }

    removeOutliersByMedian(leftEye,  outlierThr);
    removeOutliersByMedian(rightEye, outlierThr);

    // 最终排序：y 小在前
    auto byY = [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; };
    std::sort(leftEye.begin(), leftEye.end(), byY);
    std::sort(rightEye.begin(), rightEye.end(), byY);

    return {leftEye, rightEye};
}

static void
removeOutliersByMedian(std::vector<cv::Point2f>& pts, float thr)
{
    if (pts.size() <= 1) return;

    // 计算中位点（使用 x 和 y 的中位数形成中位点）
    std::vector<float> xs, ys;
    xs.reserve(pts.size());
    ys.reserve(pts.size());
    for (auto& p : pts) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }

    auto mid = pts.size() / 2;
    std::nth_element(xs.begin(), xs.begin() + mid, xs.end());
    std::nth_element(ys.begin(), ys.begin() + mid, ys.end());
    cv::Point2f medianPoint(xs[mid], ys[mid]);

    // 根据与中位点的距离过滤
    std::vector<cv::Point2f> filtered;
    filtered.reserve(pts.size());
    for (auto& p : pts) {
        float d = cv::norm(p - medianPoint);
        if (d < thr) filtered.push_back(p);
    }

    pts.swap(filtered);
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, cv::Mat>
searchForGlints(cv::Mat src, double firstEyeThresh)
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

	auto leftEyeGlints  = findGeometry(leftEyeGlintsCandidates);
	if (local_debug)
	{
		std::cout << "Left eye glints found: " << leftEyeGlints.size() << std::endl;
		std::cout << "\nStart right eye glint geometry finding..." << std::endl;
	}
	auto rightEyeGlints = findGeometry(rightEyeGlintsCandidates);
	if (local_debug)
	{
		std::cout << "Right eye glints found: " << rightEyeGlints.size() << std::endl;
	}
	
	return {leftEyeGlints, rightEyeGlints, threshold_output};

} // searchForGlints()

std::tuple<std::vector<cv::Point2f>, cv::Mat>
removeFalseGlints(std::vector<cv::Point2f> contourCenters, cv::Mat src)
{
    const int a = 3;  // radius of circle around glint
    const int maxMeanAroundGlint = 100; // max mean value around glint
    std::vector<cv::Point2f> glintCandidates;

    cv::Rect boundaries(0, 0, src.cols, src.rows);

    for (const auto& center : contourCenters)
    {
        cv::Point2i c(cvRound(center.x), cvRound(center.y)); // round to nearest integer
        cv::Rect region(c.x - a, c.y - a, 2 * a + 1, 2 * a + 1);

        // Check if region is within image boundaries
        if ((region & boundaries) != region)
            continue;

        double sum = 0.0;
        int count = 0;

        // Calculate mean value around glint
        for (int dy = -a; dy <= a; ++dy)
        {
            for (int dx = -a; dx <= a; ++dx)
            {
                if (dx == 0 && dy == 0) continue;
                sum += src.at<uchar>(c.y + dy, c.x + dx);
                ++count;
            }
        }

        double mean = sum / count;

		if (mean < maxMeanAroundGlint)
		{
			glintCandidates.emplace_back(center);
			cv::putText(src, std::to_string(glintCandidates.size() - 1),
						cv::Point2f(center.x, center.y + 10),
						cv::FONT_HERSHEY_SIMPLEX, 0.3,	cv::Scalar(255, 255, 0), 1);
		}
	}

	return {glintCandidates, src};
} // removeFalseGlints()

/*
std::vector<cv::Point2f>
oldfindGeometry(const std::vector<cv::Point2f>& glintCandidates)
{
	std::vector<cv::Point2f> glintCombi;
	std::list<cv::Point2f> glintList(glintCandidates.begin(), glintCandidates.end());

	// Go throuth each Point2f in list
	for (auto it = glintList.begin(); it != glintList.end(); ++it)
	{
		for (auto it1 = glintList.begin(); it1 != glintList.end(); ++it1)
		{
			int x = abs((*it).x - (*it1).x);
			int y = abs((*it).y - (*it1).y);

			// found horizontal Point2f pair
			if (side2side(x, y)) // original 15 5 5 0
			{
				// Point2f i is right  => search for third Point2f to the left
				if ((*it).x > (*it1).x)
				{
					if (local_debug)
					{
						std::cout << "1 found right to left" << std::endl;
						std::cout << "right: (" << (*it).x << ", " << (*it).y << ")" << std::endl;
						std::cout << "left: (" << (*it1).x << ", " << (*it1).y << ")" << std::endl;
					}
					for (std::list<cv::Point2f>::iterator it2 = glintList.begin(); it2 != glintList.end(); ++it2)
					{
						int x = (*it).x - (*it2).x;
						int y = (*it2).y - (*it).y;

						if (local_debug) std::cout << "Checking potential mid down at: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
						// right to mid down
						if (side2mid(*it2, *it1, *it, "right_to_mid"))//side2mid(x, y)
						{
							if (local_debug)
							{
								std::cout << "2 found right to mid down" << std::endl;
								std::cout << "right: (" << (*it).x << ", " << (*it).y << ")" << std::endl;
								std::cout << "mid down: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
							}

							int x = (*it2).x - (*it1).x;
							int y = (*it2).y - (*it1).y;

							// mid down to left
							if (side2mid(*it2, *it1, *it, "left_to_mid"))//(side2mid(x, y))
							{
								if (local_debug)
								{
									std::cout << "3 found mid down to left" << std::endl;
									std::cout << "mid down: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
									std::cout << "left: (" << (*it1).x << ", " << (*it1).y << ")" << std::endl;
								}
								std::vector<cv::Point2f> tmp;
								tmp.emplace_back((*it));
								tmp.emplace_back((*it1));
								tmp.emplace_back((*it2));

								glintCombi = tmp;
								return glintCombi;
							} else {
								if (local_debug)
								{
									std::cout << "3 not found mid down to left" << std::endl;
									std::cout << "mid down: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
									std::cout << "left: (" << (*it1).x << ", " << (*it1).y << ")" << std::endl;
								}
							}
						}
					}
				}
				else // Point2f i is left => search for third Point2f to the right
				{
					if (local_debug)
					{
						std::cout << "1 found left to right" << std::endl;
						std::cout << "left: (" << (*it).x << ", " << (*it).y << ")" << std::endl;
						std::cout << "right: (" << (*it1).x << ", " << (*it1).y << ")" << std::endl;
					}
					for (std::list<cv::Point2f>::iterator it2 = glintList.begin(); it2 != glintList.end(); ++it2)
					{
						int x = (*it2).x - (*it).x;
						int y = (*it2).y - (*it).y;

						if (local_debug) std::cout << "Checking potential mid down at: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
						// left to mid down
						if (side2mid(*it2, *it, *it1, "left_to_mid"))//(side2mid(x, y))
						{
							if (local_debug)
							{
								std::cout << "2 found left to mid down" << std::endl;
								std::cout << "left: (" << (*it).x << ", " << (*it).y << ")" << std::endl;
								std::cout << "mid down: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
							}

							int x = (*it1).x - (*it2).x;
							int y = (*it2).y - (*it1).y;

							// mid down to right
							if (side2mid(*it2, *it, *it1, "right_to_mid"))//(side2mid(x, y))
							{
								if (local_debug)
								{
									std::cout << "3 found mid down to right" << std::endl;
									std::cout << "mid down: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
									std::cout << "right: (" << (*it1).x << ", " << (*it1).y << ")" << std::endl;
								}
								std::vector<cv::Point2f> tmp;
								tmp.emplace_back((*it));
								tmp.emplace_back((*it1));
								tmp.emplace_back((*it2));

								glintCombi = tmp;
								return glintCombi;
							} else {
								if (local_debug)
								{
									std::cout << "3 not found mid down to right" << std::endl;
									std::cout << "mid down: (" << (*it2).x << ", " << (*it2).y << ")" << std::endl;
									std::cout << "right: (" << (*it1).x << ", " << (*it1).y << ")" << std::endl;
								}
							}
						}
					}
				}
			}
		}
	}

	return glintCombi;
}
*/

std::vector<cv::Point2f>
findGeometry(const std::vector<cv::Point2f>& glintCandidates)
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
			if (side2side(l_pt, r_pt)) // original 15 5 5 0
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
					if (side2mid(m_pt, l_pt, r_pt))
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

	std::vector<cv::Point2f> glintGeometry = findBestGeometry(glintGeometryCandidates);

	return glintGeometry;
}

std::vector<cv::Point2f>
findBestGeometry(const std::vector<std::vector<cv::Point2f>>& glintGeometryCandidates)
{
	std::vector<cv::Point2f> glintGeometry;

	if (glintGeometryCandidates.empty())
	{
		return glintGeometry;
	}

	glintGeometry = glintGeometryCandidates[0];

	return glintGeometry;
}

} // namespace glintdetection