#pragma once

#include <opencv2/opencv.hpp>

namespace visualization
{

cv::Mat
visualizeGlintsAndPupil(
    const cv::Mat& frame,
    const std::vector<cv::Point2d>& glints,
    const cv::Point2d& pupil_center,
	const float radius
);

cv::Mat
visualizeGlints(
    const cv::Mat& frame,
    const std::vector<cv::Point2d>& glints
);

}