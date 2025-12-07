#include "opencv2/core.hpp"

namespace glintdetection {

bool side2mid(int x, int y);

std::vector<cv::Point2f>
chooseBestMid(const std::list<cv::Point2f>& glintList,
              const cv::Point2f& p1,
              const cv::Point2f& p2);

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
myfindGeometry(const std::vector<cv::Point2f>& glintCandidates);

} // namespace glintdetection