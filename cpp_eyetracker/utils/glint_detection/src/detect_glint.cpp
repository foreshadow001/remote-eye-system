#include <future>
#include <list>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <limits>
#include <cmath>

#include "glint_detection/detect_glint.hpp"

namespace glintdetection {

Cfg cfg;

bool local_debug = cfg["test_glint"]["local_debug"].as<bool>() || cfg["collect_glint"]["local_debug"].as<bool>();\
bool debug_time = cfg["test_glint"]["debug_time"].as<bool>();

// TODO:
// ✅ 1. optimaize side2mid function (add delta y of left and right point as a parameter, and adjust the range accordingly).
// 2. collect all the possible glint combinations, and choose the best one according to some criteria. ，  

bool side2side(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const CfgNode& horizontal_pair_cfg
)
{
    double lr_x = std::abs(l_pt.x - r_pt.x);
    double lr_y = std::abs(l_pt.y - r_pt.y);

    double lr_x_min = horizontal_pair_cfg["lr_x_min"].as<double>();
    double lr_x_max = horizontal_pair_cfg["lr_x_max"].as<double>();
    double lr_y_min = horizontal_pair_cfg["lr_y_min"].as<double>();
    double lr_y_max = horizontal_pair_cfg["lr_y_max"].as<double>();

    bool cond_y = (lr_y >= lr_y_min && lr_y < lr_y_max);
    bool cond_x = (lr_x >= lr_x_min && lr_x <= lr_x_max);

    if (local_debug)
    {
        std::cout << "[side2side] lr_x=" << lr_x
                  << " lr_y=" << lr_y << std::endl;

        std::cout << "  X cond: [" << lr_x_min << ", " << lr_x_max
                  << "] -> " << cond_x << std::endl;

        std::cout << "  Y cond: [" << lr_y_min << ", " << lr_y_max
                  << ") -> " << cond_y << std::endl;

        std::cout << "  result=" << (cond_x && cond_y) << std::endl;
    }

    return cond_x && cond_y;
}


bool side2mid(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const cv::Point2f& m_pt,
    const CfgNode& middle_point_cfg
)
{
    double lr_x = std::abs(l_pt.x - r_pt.x);
    double lr_y = std::abs(l_pt.y - r_pt.y);

    double lm_x = m_pt.x - l_pt.x;
    double lm_y = m_pt.y - l_pt.y;
    double rm_x = r_pt.x - m_pt.x;
    double rm_y = m_pt.y - r_pt.y;

    // 若左 glint 更低，则交换中点相对量
    if (l_pt.y < r_pt.y)
    {
        std::swap(lm_x, rm_x);
        std::swap(lm_y, rm_y);
    }

    const auto& conditions = middle_point_cfg["conditions"].as<std::vector<std::vector<double>>>();

    for (const auto& cond : conditions)
    {
        double y_min = cond[0];
        double y_max = cond[1];

        // 判断此 condition 是否匹配本 lr_y
        if (!(lr_y >= y_min && lr_y < y_max)) continue;

        // 构造区间说明字符串（与原版一致风格）
        std::string range_desc;
        if (std::abs(y_min) < 1e-9)
            range_desc = "(lr_y < " + std::to_string((int)y_max) + ")";
        else
            range_desc = "(" + std::to_string((int)y_min)
                         + " <= lr_y < " + std::to_string((int)y_max) + ")";

        double lm_x_ratio = lm_x / lr_x;
        double lm_y_ratio = lm_y / lr_x;
        double rm_x_ratio = rm_x / lr_x;
        double rm_y_ratio = rm_y / lr_x;

        bool lm_condition =
               lm_x_ratio >= cond[2] && lm_x_ratio <= cond[3]
            && lm_y_ratio >= cond[4] && lm_y_ratio <= cond[5];

        bool rm_condition =
               rm_x_ratio >= cond[6] && rm_x_ratio <= cond[7]
            && rm_y_ratio >= cond[8] && rm_y_ratio <= cond[9];

        if (local_debug)
        {
            std::cout << "left to mid: " << range_desc << std::endl;
            std::cout << "lm_x: " << lm_x
                      << " | lr_x: " << lr_x
                      << " | lm_x/lr_x: " << lm_x_ratio
                      << " | qualified: "
                      << (lm_x_ratio >= cond[2] && lm_x_ratio <= cond[3])
                      << std::endl;

            std::cout << "lm_y: " << lm_y
                      << " | lr_x: " << lr_x
                      << " | lm_y/lr_x: " << lm_y_ratio
                      << " | qualified: "
                      << (lm_y_ratio >= cond[4] && lm_y_ratio <= cond[5])
                      << std::endl;

            std::cout << "right to mid: " << range_desc << std::endl;
            std::cout << "rm_x: " << rm_x
                      << " | lr_x: " << lr_x
                      << " | rm_x/lr_x: " << rm_x_ratio
                      << " | qualified: "
                      << (rm_x_ratio >= cond[6] && rm_x_ratio <= cond[7])
                      << std::endl;

            std::cout << "rm_y: " << rm_y
                      << " | lr_x: " << lr_x
                      << " | rm_y/lr_x: " << rm_y_ratio
                      << " | qualified: "
                      << (rm_y_ratio >= cond[8] && rm_y_ratio <= cond[9])
                      << std::endl;

            std::cout << "lm_condition=" << lm_condition
                      << " rm_condition=" << rm_condition << std::endl;
        }

        if (lm_condition && rm_condition)
            return true;
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

std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, cv::Mat>
searchForGlints(cv::Mat src, const Cfg& cfg)
{
	cv::Mat gray;
	double firstEyeThresh = cfg["test_glint"]["threshold"].as<double>();

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
    double t1 = cv::getTickCount();
	cv::GaussianBlur(gray, gaussed,
					 cv::Size(cfg["test_glint"]["gaussion_kernel_size"].as<int>(),
					 		  cfg["test_glint"]["gaussion_kernel_size"].as<int>()),
					 0, 0, cv::BORDER_DEFAULT); // better results in extreme cases
    double t2 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[1] Gaussion blur time: " << (t2 - t1) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }

	// 2 Laplace
	// Apply Laplace function
	// Laplace Functino generates edges
	cv::Mat laplaced;
	cv::Laplacian(gaussed, laplaced, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
    double t3 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[2] Laplace time: " << (t3 - t2) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }
	
	// CHANGED
	cv::Mat abs_dst;
	cv::convertScaleAbs(laplaced, abs_dst);
	// convertScaleAbs(laplaced, abs_dst, (sigma + 1)*0.25);
    double t4 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[3] convertScaleAbs time: " << (t4 - t3) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }

	// 3 Find Contours
	// Detect edges using Threshold
	cv::Mat threshold_output;
	cv::threshold(abs_dst, threshold_output, firstEyeThresh, 255, cv::THRESH_BINARY);
    double t5 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[4] Threshold time: " << (t5 - t4) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }

	// FOR SPEED UP
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(threshold_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    double t6 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[5] FindContours time: " << (t6 - t5) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }

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
    double t7 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[6] SplitGlintsByEye time: " << (t7 - t6) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }

	if (local_debug)
	{
		std::cout << "Left eye glint candidates: " << leftEyeGlintsCandidates.size() << std::endl;
		std::cout << "Right eye glint candidates: " << rightEyeGlintsCandidates.size() << std::endl;
		std::cout << "\nStart left eye glint geometry finding..." << std::endl;
	}

	auto leftEyeGlints  = findGeometry(leftEyeGlintsCandidates, cfg);
    double t8 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[7] FindGeometry time: " << (t8 - t7) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }
	if (local_debug)
	{
		std::cout << "Left eye glints found: " << leftEyeGlints.size() << std::endl;
		std::cout << "\nStart right eye glint geometry finding..." << std::endl;
	}
	auto rightEyeGlints = findGeometry(rightEyeGlintsCandidates, cfg);
    double t9 = cv::getTickCount();
    if (debug_time)
    {
        std::cout << "[8] FindGeometry time: " << (t9 - t8) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }
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

std::vector<std::vector<cv::Point2f>>
findGeometry(const std::vector<cv::Point2f>& glintCandidates, const Cfg& cfg)
{
	std::vector<std::vector<cv::Point2f>> glintGeometryCandidates;

	CfgNode horizontal_pair_cfg = cfg["collect_glint"]["is_collecting"].as<bool>()
								  ? cfg["relaxed_glint_hyperparameter"]["horizontal_pair"]
								  : cfg["glint_hyperparameter"]["horizontal_pair"];

	CfgNode middle_point_cfg = cfg["collect_glint"]["is_collecting"].as<bool>()
							   ? cfg["relaxed_glint_hyperparameter"]["middle_point"]
							   : cfg["glint_hyperparameter"]["middle_point"];

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
			if (side2side(l_pt, r_pt, horizontal_pair_cfg)) // original 15 5 5 0
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
					if (side2mid(l_pt, r_pt, m_pt, middle_point_cfg))
					{
						if (local_debug)
						{
							std::cout << "2 found mid point" << std::endl;
							std::cout << "mid: (" << m_pt.x << ", " << m_pt.y << ")" << std::endl;
						}

						std::vector<cv::Point2f> glintGeometryCandidate;
						glintGeometryCandidate.push_back(l_pt);
						glintGeometryCandidate.push_back(r_pt);
						glintGeometryCandidate.push_back(m_pt);

						glintGeometryCandidates.push_back(glintGeometryCandidate);
					}
				}
			}
		}
	}

	if (cfg["collect_glint"]["is_collecting"].as<bool>()) return glintGeometryCandidates;

	auto glintGeometry = findBestGeometry(glintGeometryCandidates);

	return glintGeometry;
}

std::vector<std::vector<cv::Point2f>>
findBestGeometry(const std::vector<std::vector<cv::Point2f>>& glintGeometryCandidates)
{
	std::vector<std::vector<cv::Point2f>> glintGeometryList;
    
	if (glintGeometryCandidates.empty())
	{
		return glintGeometryList;
	}

	glintGeometryList.push_back(glintGeometryCandidates[0]);

	return glintGeometryList;
}

} // namespace glintdetection