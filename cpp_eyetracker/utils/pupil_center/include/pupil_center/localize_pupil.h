#include <opencv2/opencv.hpp>


namespace pupilcenter {

std::tuple<cv::Point2f, cv::Mat, float>
localizePupilCenter(const cv::Mat& img, const std::vector<cv::Point2f>& glints);

std::tuple<cv::Point2f, cv::Mat>
cropEyeFromGlints(const std::vector<cv::Point2f>& glints, const cv::Mat& img,
                  int marginTop    = 40, int marginBottom = 20,
                  int marginLeft   = 30, int marginRight  = 30);

cv::Mat
removeGlints(const cv::Mat& gray,
             const std::vector<cv::Point2f>& glints,
             int glintRadius = 4);

cv::Point2f
roughPupilCenter(const cv::Mat& eyeROI);

double
meanIntensityInCircle(const cv::Mat& gray, const cv::Point2f& center, float radius);

float estimateRadius(const cv::Mat& gray, const cv::Point2f& center, float minR, float maxR);

std::tuple<cv::Point2f, cv::Mat, float>
refinePupilCenter(const cv::Mat& gray, cv::Point2f initCenter, 
                  float initRadius = 16.0f, float minRadius = 10.0f, float maxRadius = 20.0f);

std::tuple<cv::Point2f, cv::Mat>
refinePupilCenterEllipse(const cv::Mat& gray, cv::Point2f initCenter,
                         float initA = 10.0f, float initB = 10.0f,
                         float minA = 8.0f, float maxA = 12.0f,
                         float minB = 8.0f, float maxB = 12.0f);

} // namespace pupilcenter