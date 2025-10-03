#include "opencv2/core.hpp"

namespace glintdetection {
//void GoalkeeperAnalysis::searchForGlints(Mat src, std::promise<vector<Point2f>> & p)
std::pair<std::vector<cv::Point2f>, cv::Mat> searchForGlints(cv::Mat src, double firstEyeThresh);
} // namespace glintdetection