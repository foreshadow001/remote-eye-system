#include "opencv2/core.hpp"
#include "cfg/config.hpp"

namespace glintdetection {

bool side2side(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const CfgNode& horizontal_pair_cfg
);

bool side2mid(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const cv::Point2f& m_pt,
    const CfgNode& middle_point_cfg
);

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
splitGlintsByEye(const std::vector<cv::Point2f>& glints, float distanceThrX = 100.f, float outlierThr = 200.f);

static void
removeOutliersByMedian(std::vector<cv::Point2f>& pts, float thr);

std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, cv::Mat>
searchForGlints(cv::Mat src, const Cfg& cfg);

std::tuple<std::vector<cv::Point2f>, cv::Mat>
removeFalseGlints(std::vector<cv::Point2f> contourCenters, cv::Mat src);

std::vector<std::vector<cv::Point2f>>
findGeometry(const std::vector<cv::Point2f>& glintCandidates, const Cfg& cfg);

std::vector<std::vector<cv::Point2f>>
findBestGeometry(const std::vector<std::vector<cv::Point2f>>& glintGeometryCandidates);

} // namespace glintdetection