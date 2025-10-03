#include <future>
#include <list>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>

namespace glintdetection {

// std::vector<cv::Point2f>
std::pair<std::vector<cv::Point2f>, cv::Mat> searchForGlints(cv::Mat src, double firstEyeThresh) {

	std::vector<cv::Point2f> glints;

	if (src.channels() == 3) {
		cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
	}

	int kernel_size = 3; // better results in extreme cases
	double scale = 1;
	double delta = 0;
	int ddepth = CV_16S;

	// 1 Gauss
	// Remove noise by blurring with a Gaussian filter
	// Blurring also generates wider range for finding contours

	// CHANGED
	// 1
	cv::Mat gaussed;
	cv::GaussianBlur(src, gaussed, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT); // better results in extreme cases

																				/// Convert the image to grayscale
	cv::Mat gaussed_gray;
	if (gaussed.channels() == 3) {
		cv::cvtColor(gaussed, gaussed_gray, cv::COLOR_BGR2GRAY);
	} else {
		gaussed.copyTo(gaussed_gray);
	}

	return {glints, gaussed_gray};

    /*
	// 2 Laplace
	// Apply Laplace function
	// Laplace Functino generates edges
	cv::Mat laplaced;
	cv::Laplacian(gaussed_gray, laplaced, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
	
	// CHANGED
	cv::Mat abs_dst;
	cv::convertScaleAbs(laplaced, abs_dst);
	//convertScaleAbs(laplaced, abs_dst, (sigma + 1)*0.25);
	

	
	

	// 3 Find Contours
	/// Detect edges using Threshold
	cv::Mat threshold_output, threshold_output2, threshold_output3;
	cv::threshold(abs_dst, threshold_output, firstEyeThresh, 255, cv::THRESH_BINARY);

	cv::Mat contoursMat;

	src.copyTo(contoursMat);
	cvtColor(contoursMat, contoursMat, cv::COLOR_GRAY2BGR);

	// FOR SPEED UP
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(threshold_output, contours, hierarchy, cv:: RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// 4 Approximate contours to polygons + get bounding rects and circles
	std::vector<cv::RotatedRect> minRect(contours.size());
	std::vector<cv::Point2f> contourCenter(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(cv::Mat(contours[i]));
		contourCenter.emplace_back(minRect[i].center.x, minRect[i].center.y);

		// DEBUG
		//drawContours(contoursMat, contours, i, cv::Scalar(255, 255, 0), 1, 8, hierarchy, 0, cv::Point());
		//circle(contoursMat, cv::Point2f(minRect[i].center.x , minRect[i].center.y), 2, cv::Scalar(0, 0, 255), 1, 8);			//CURRENT
	}

	// DEBUG
	//imshow("contoursMat", contoursMat);
	//cv::waitKey(1);

	// Detect glints on sclera and remove them from list ======================================================================
	std::vector<cv::Point2f> glintCandidates = removeFalseGlints(contourCenter, threshold_output, src);
	std::vector<cv::Point2f> glintCombi = findGeometry(glintCandidates, src);

	// no eye found
	if (glintCombi.empty()) {
		glints = { cv::Point2f(0, 0), cv::Point2f(0, 0), cv::Point2f(0, 0) };
	} else {
		glints = glintCombi;
		// sort with respect to position in between
		std::sort(glints.begin(), glints.end(), SortCvPoint2fByX());
	}
	
	return glints;
    */
} // searchForGlints()

struct SortCvPoint2fByX
{
	bool operator()(const cv::Point2f& lhs, const cv::Point2f& rhs)
	{
		return lhs.x < rhs.x;
	}
};

} // namespace glintdetection