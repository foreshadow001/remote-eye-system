#include "opencv2/core.hpp"

namespace glintdetection {

bool side2side(const cv::Point2f& l_pt, const cv::Point2f& r_pt);

bool side2mid(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const cv::Point2f& mid_pt
);

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
splitGlintsByEye(const std::vector<cv::Point2f>& glints, float distanceThrX = 100.f, float outlierThr = 200.f);

static void
removeOutliersByMedian(std::vector<cv::Point2f>& pts, float thr);

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, cv::Mat>
searchForGlints(cv::Mat src, double firstEyeThresh);

std::tuple<std::vector<cv::Point2f>, cv::Mat>
removeFalseGlints(std::vector<cv::Point2f> contourCenters, cv::Mat src);

std::vector<cv::Point2f>
findGeometry(const std::vector<cv::Point2f>& glintCandidates);

std::vector<cv::Point2f>
findBestGeometry(const std::vector<std::vector<cv::Point2f>>& glintGeometryCandidates);

} // namespace glintdetection