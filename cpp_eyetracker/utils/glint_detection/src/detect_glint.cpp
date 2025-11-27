#include <future>
#include <list>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <limits>
#include <cmath>

#include "glint_detection/detect_glint.h"


namespace glintdetection {

bool local_debug = false;

bool side2mid(int x, int y)
{
	return (x <= 20 && x >= 5 && y <= 10 && y >= 3); // 15 3 5 0
}

std::vector<cv::Point2f>
chooseBestMid(const std::list<cv::Point2f>& glintList,
              const cv::Point2f& p1,
              const cv::Point2f& p2)
{
    // 0
	// confirm left and right
    cv::Point2f left  = (p1.x < p2.x) ? p1 : p2;
    cv::Point2f right = (p1.x < p2.x) ? p2 : p1;

    std::vector<cv::Point2f> midCandidates;

    // 0
	// calculate mid point
    float midX = (left.x + right.x) / 2.0f;
    float maxY = std::max(left.y, right.y);

    // 1
	// filter candidates
    for (const auto& p : glintList)
    {
        if (p == left || p == right)
            continue;

        int x1 = p.x - left.x;
        int y1 = p.y - left.y;
        int x2 = right.x - p.x;
        int y2 = p.y - right.y;

        if (side2mid(x1, y1) && side2mid(x2, y2))
        {
            midCandidates.push_back(p);
        }
    }

    // 2
	// if no candidates, return empty vector
    if (midCandidates.empty())
        return {};

    // 3
	// find best mid point
    double bestScore = std::numeric_limits<double>::max();
    cv::Point2f bestMid;

    for (const auto& m : midCandidates)
    {
        double dy = std::max(0.0f, m.y - maxY);
        double dx = std::abs(m.x - midX);
        double score = 2.0 * dy + 1.0 * dx;

        if (score < bestScore)
        {
            bestScore = score;
            bestMid = m;
        }
    }

    return {left, right, bestMid};
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
splitGlintsByEye(const std::vector<cv::Point2f>& glints, float distanceThr)
{
    std::vector<cv::Point2f> leftEye, rightEye;
    if (glints.empty()) return {leftEye, rightEye};

    std::vector<cv::Point2f> sorted = glints;
    std::sort(sorted.begin(), sorted.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });

    const float refX = sorted.front().x;

    for (const auto& p : sorted)
        (std::abs(p.x - refX) < distanceThr ? rightEye : leftEye).push_back(p);

    auto byY = [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; };
    std::sort(leftEye.begin(), leftEye.end(), byY);
    std::sort(rightEye.begin(), rightEye.end(), byY);

    return {leftEye, rightEye};
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, cv::Mat>
searchForGlints(cv::Mat src, double firstEyeThresh)
{
	cv::Mat gray;

	if (src.channels() == 3)
	{
		cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	}

	int kernel_size = 3; // better results in extreme cases
	double scale = 1;
	double delta = 0;
	int ddepth = CV_16S;

	// 1 Gauss
	// Remove noise by blurring with a Gaussian filter
	// Blurring also generates wider range for finding contours
	cv::Mat gaussed;
	cv::GaussianBlur(gray, gaussed, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT); // better results in extreme cases

	// 2 Laplace
	// Apply Laplace function
	// Laplace Functino generates edges
	cv::Mat laplaced;
	cv::Laplacian(gaussed, laplaced, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
	
	// CHANGED
	cv::Mat abs_dst;
	cv::convertScaleAbs(laplaced, abs_dst);
	//convertScaleAbs(laplaced, abs_dst, (sigma + 1)*0.25);

	// 3 Find Contours
	/// Detect edges using Threshold
	cv::Mat threshold_output;
	cv::threshold(abs_dst, threshold_output, firstEyeThresh, 255, cv::THRESH_BINARY);

	std::vector<cv::Point2f> leftEyeGlints_, rightEyeGlints_;
	// return {leftEyeGlints_, rightEyeGlints_, threshold_output};

	// FOR SPEED UP
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(threshold_output, contours, hierarchy, cv:: RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// 4 Approximate contours to polygons + get bounding rects and circles
	std::vector<cv::RotatedRect> minRect(contours.size());
	std::vector<cv::Point2f> contourCenter(contours.size());

	cv::Mat src_copy, src_copy_left, src_copy_right;
	src.copyTo(src_copy);

	for (int i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(cv::Mat(contours[i]));
		contourCenter.emplace_back(minRect[i].center.x, minRect[i].center.y);
	}

	// return {contourCenter, rightEyeGlints_, threshold_output};

	// Detect glints on sclera and remove them from list ======================================================================
	auto [glintCandidates, img_rm] = removeFalseGlints(contourCenter, src_copy);
	auto [leftEyeGlintsCandidates, rightEyeGlintsCandidates] = splitGlintsByEye(glintCandidates);

	return {leftEyeGlintsCandidates, rightEyeGlintsCandidates, threshold_output};

	auto leftEyeGlints  = myfindGeometry(leftEyeGlintsCandidates);
	if (local_debug) std::cout << "num leftEyeGlints: " << leftEyeGlints.size() << std::endl;
	auto rightEyeGlints = myfindGeometry(rightEyeGlintsCandidates);
	
	return {leftEyeGlints, rightEyeGlints, src};

} // searchForGlints()

std::tuple<std::vector<cv::Point2f>, cv::Mat>
removeFalseGlints(std::vector<cv::Point2f> contourCenters, cv::Mat src)
{
    const int a = 3;  // radius of circle around glint
    const int maxMeanAroundGlint = 80; // max mean value around glint
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

std::vector<cv::Point2f>
findGeometry(const std::vector<cv::Point2f>& glintCandidates)
{
	std::vector<cv::Point2f> glintCombi;
	std::list<cv::Point2f> glintList(glintCandidates.begin(), glintCandidates.end());

	bool local_debug = true;

	// Go throuth each Point2f in list
	for (auto it = glintList.begin(); it != glintList.end(); ++it)
	{
		for (auto it1 = glintList.begin(); it1 != glintList.end(); ++it1)
		{
			int x = abs((*it).x - (*it1).x);
			int y = abs((*it).y - (*it1).y);

			// found horizontal Point2f pair
			if (x <= 15 && x >= 5 && y <= 5 && y >= 0) // original
			{
				int yTolerance = 5; // tolerance for y-coordinate

				// Point2f i is right  => search for third Point2f to the left
				if ((*it).x > (*it1).x && abs((*it).y - (*it1).y) <= yTolerance)
				{
					if (local_debug)
					{
						std::cout << "1 found right to left" << std::endl;
					}
					for (std::list<cv::Point2f>::iterator it2 = glintList.begin(); it2 != glintList.end(); ++it2)
					{
						int x = (*it).x - (*it2).x;
						int y = (*it2).y - (*it).y;

						// right to mid down
						if (side2mid(x, y))
						{
							std::cout << "2 found right to mid down" << std::endl;

							int x = (*it2).x - (*it1).x;
							int y = (*it2).y - (*it1).y;

							// mid down to left
							if (side2mid(x, y))
							{
								std::vector<cv::Point2f> tmp;
								tmp.emplace_back((*it));
								tmp.emplace_back((*it1));
								tmp.emplace_back((*it2));

								glintCombi = tmp;
								return glintCombi;
							}
						}
					}
				}
				else // Point2f i is left => search for third Point2f to the right
				{
					if (local_debug)
					{
						std::cout << "1 found left to right" << std::endl;
					}
					for (std::list<cv::Point2f>::iterator it2 = glintList.begin(); it2 != glintList.end(); ++it2)
					{
						int x = (*it2).x - (*it).x;
						int y = (*it2).y - (*it).y;

						// left to mid down
						if (side2mid(x, y))
						{
							if (local_debug)
							{
								std::cout << "2 found left to mid down" << std::endl;
							}

							int x = (*it1).x - (*it2).x;
							int y = (*it2).y - (*it1).y;

							// mid down to right
							if (side2mid(x, y))
							{
								std::vector<cv::Point2f> tmp;
								tmp.emplace_back((*it));
								tmp.emplace_back((*it1));
								tmp.emplace_back((*it2));

								glintCombi = tmp;
								return glintCombi;
							}
						}
					}
				}
			}
		}
	}

	return glintCombi;
}

std::vector<cv::Point2f>
myfindGeometry(const std::vector<cv::Point2f>& glintCandidates)
{
	std::vector<cv::Point2f> glintCombi;
	std::list<cv::Point2f> glintList(glintCandidates.begin(), glintCandidates.end());

    if (glintList.size() < 3)
        return glintCombi;

    for (auto it = glintList.begin(); it != glintList.end(); ++it)
    {
        for (auto it1 = std::next(it); it1 != glintList.end(); ++it1)
        {
            auto result = chooseBestMid(glintList, *it, *it1);
            if (!result.empty())
            {
                auto left  = result[0];
                auto right = result[1];
                auto mid   = result[2];
                glintCombi = {left, right, mid};

                return glintCombi;
            }
        }
    }

    return glintCombi;
} // myfindGeometry()

} // namespace glintdetection