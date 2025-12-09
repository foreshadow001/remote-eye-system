#include <iostream>
#include <numeric>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "pupil_center/localize_pupil.hpp"
#include "glint_detection/detect_glint.hpp"

namespace pupilcenter {

bool local_debug = false;

std::tuple<cv::Point2f, cv::Mat, float>
localizePupilCenter(const cv::Mat& img, const std::vector<cv::Point2f>& glints)
{
    auto img_inpainted = removeGlints(img, glints);

    auto [offset, eyeROI] = cropEyeFromGlints(glints, img_inpainted);

    cv::Point2f rough = roughPupilCenter(eyeROI);
    auto [refined, pupilImage, radius] = refinePupilCenter(eyeROI, rough);

    cv::Point2f pupilCenter = refined + offset;

    return {pupilCenter, pupilImage, radius};
}

std::tuple<cv::Point2f, cv::Mat>
mylocalizePupilCenter(const cv::Mat& img, const std::vector<cv::Point2f>& glints)
{
    auto start = std::chrono::high_resolution_clock::now();

    // Step 1: Remove glints
    auto img_inpainted = removeGlints(img, glints);
    auto step1_end = std::chrono::high_resolution_clock::now();
    std::cout << "Step 1 (removeGlints) took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(step1_end - start).count()
              << " ms" << std::endl;

    // Step 2: Crop eye from glints
    auto [offset, eyeROI] = cropEyeFromGlints(glints, img_inpainted);
    auto step2_end = std::chrono::high_resolution_clock::now();
    std::cout << "Step 2 (cropEyeFromGlints) took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(step2_end - step1_end).count()
              << " ms" << std::endl;

    // Step 3: Rough pupil center
    cv::Point2f rough = roughPupilCenter(eyeROI);
    auto step3_end = std::chrono::high_resolution_clock::now();
    std::cout << "Step 3 (roughPupilCenter) took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(step3_end - step2_end).count()
              << " ms" << std::endl;

    // Step 4: Refine pupil center
    auto [refined, pupilImage, radius] = refinePupilCenter(eyeROI, rough);
    auto step4_end = std::chrono::high_resolution_clock::now();
    std::cout << "Step 4 (refinePupilCenter) took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(step4_end - step3_end).count()
              << " ms" << std::endl;

    // Final step: Compute pupil center
    cv::Point2f pupilCenter = refined + offset;
    auto total_end = std::chrono::high_resolution_clock::now();
    std::cout << "Final step (pupilCenter offset) took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(total_end - step4_end).count()
              << " ms" << std::endl;

    std::cout << "Total time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(total_end - start).count()
              << " ms" << std::endl;

    return {pupilCenter, pupilImage};
}

/**
 * @brief according to the glints, crop the eye image from the original image
 * @param glints  three glints detected in the eye image
 * @param img     the original image
 * @param marginTop    the distance from the top of the eye image to the top of the cropped image
 * @param marginBottom the distance from the bottom of the eye image to the bottom of the cropped image
 * @param marginLeft   the distance from the left of the eye image to the left of the cropped image
 * @param marginRight  the distance from the right of the eye image to the right of the cropped image
 * @return a tuple of the cropped image and the offset of the cropped image in the original image
 */
std::tuple<cv::Point2f, cv::Mat>
cropEyeFromGlints(const std::vector<cv::Point2f>& glints, const cv::Mat& img,
                  int marginTop, int marginBottom, int marginLeft, int marginRight)
{
    cv::Mat gray;

    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img.clone();

    if (glints.size()!= 3)
    {
        std::cerr << "Error: glints.size() must be 3" << std::endl;
        return {};
    }

    // 1. compute center of glints
    cv::Point2f center(0, 0);
    for (const auto& g : glints) center += g;
    center *= 1.0f / 3.0f;

    // 2. compute roi
    int x0 = static_cast<int>(std::round(center.x - marginLeft));
    int y0 = static_cast<int>(std::round(center.y - marginTop));
    int w  = marginLeft + marginRight;
    int h  = marginTop  + marginBottom;
    cv::Rect roi(x0, y0, w, h);

    cv::Point2f offset(x0, y0);

    // 3. confirm roi is within img
    cv::Rect imgRect(0, 0, img.cols, img.rows);
    cv::Rect valid = roi & imgRect;
    if (valid.empty())
    {
        // if roi is not within img, return white image of size (h, w)
        return {};
    }

    // 4. create white image of size (h, w)
    cv::Mat eyeImg(h, w, CV_8UC1, cv::Scalar(255));

    // 5. compute offset of valid roi in eyeImg
    int dx = valid.x - roi.x;
    int dy = valid.y - roi.y;

    // 6. copy valid roi from img to eyeImg
    cv::Mat dstROI = eyeImg(cv::Rect(dx, dy, valid.width, valid.height));
    img(valid).copyTo(dstROI);

    return {offset, eyeImg};
} // cropEyeFromGlints

cv::Mat
removeGlints(const cv::Mat& img, const std::vector<cv::Point2f>& glints, int glintRadius)
{
    cv::Mat gray;
    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img.clone();

    for (auto& g : glints)
    {
        cv::circle(gray, g, glintRadius, 255, -1); // 填白方便 inpaint
    }
    cv::inpaint(gray, gray == 255, gray, glintRadius, cv::INPAINT_TELEA);

    return gray;
}

cv::Point2f
roughPupilCenter(const cv::Mat& eyeROI)
{
    cv::Mat gray;
    if (eyeROI.channels() == 3)
        cv::cvtColor(eyeROI, gray, cv::COLOR_BGR2GRAY);
    else
        gray = eyeROI.clone();

    // 模糊去噪
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 2);

    // 阈值分割 (取暗区)
    cv::Mat bin;
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    // 形态学清理
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, cv::Mat::ones(5, 5, CV_8U));

    // 连通域
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return { gray.cols / 2.0f, gray.rows / 2.0f };

    // 选最大区域
    size_t maxIdx = 0;
    double maxArea = 0;
    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxIdx = i;
        }
    }

    cv::Moments m = cv::moments(contours[maxIdx]);
    return { static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00) };
}

double meanIntensityInCircle(const cv::Mat& gray, const cv::Point2f& center, float radius)
{
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8U);
    cv::circle(mask, center, static_cast<int>(radius), 255, -1);
    return cv::mean(gray, mask)[0];
}

float estimateRadius(const cv::Mat& gray, const cv::Point2f& center, float minR, float maxR)
{
    const int directions = 24;
    std::vector<float> radii;
    const int smoothKernel = 3;

    for (int i = 0; i < directions; ++i)
    {
        float angle = static_cast<float>(i) * 2.0f * CV_PI / directions;
        cv::Point2f dir(std::cos(angle), std::sin(angle));

        // 采样灰度曲线
        std::vector<float> samples;
        for (float r = minR; r <= maxR; r += 1.0f)
        {
            cv::Point2f p = center + dir * r;
            if (p.x < 0 || p.y < 0 || p.x >= gray.cols || p.y >= gray.rows)
                break;
            samples.push_back(static_cast<float>(gray.at<uchar>(cv::Point(cvRound(p.x), cvRound(p.y)))));
        }

        if (samples.size() < 5) continue;

        // 平滑
        std::vector<float> smooth(samples.size());
        for (size_t j = 0; j < samples.size(); ++j)
        {
            float sum = 0, count = 0;
            for (int k = -smoothKernel; k <= smoothKernel; ++k)
            {
                int idx = static_cast<int>(j) + k;
                if (idx >= 0 && idx < (int)samples.size())
                {
                    sum += samples[idx];
                    count += 1;
                }
            }
            smooth[j] = sum / count;
        }

        // 求一阶导（灰度梯度）
        std::vector<float> grad(samples.size() - 1);
        for (size_t j = 1; j < samples.size(); ++j)
            grad[j - 1] = smooth[j] - smooth[j - 1];

        // 寻找最大梯度位置（暗->亮）
        auto it = std::max_element(grad.begin(), grad.end());
        if (it != grad.end())
        {
            int idx = static_cast<int>(std::distance(grad.begin(), it));
            float r_est = minR + idx + 0.5f;
            if (r_est >= minR && r_est <= maxR)
                radii.push_back(r_est);
        }
    }

    if (radii.empty()) return (minR + maxR) * 0.5f;

    // 去掉异常值（用中值）
    std::nth_element(radii.begin(), radii.begin() + radii.size() / 2, radii.end());
    float medianR = radii[radii.size() / 2];
    return medianR;
}

std::tuple<cv::Point2f, cv::Mat, float>
refinePupilCenter(const cv::Mat& grayIn,
                  cv::Point2f initCenter,
                  float initRadius,
                  float minRadius,
                  float maxRadius)
{
    CV_Assert(grayIn.type() == CV_8UC1 || grayIn.type() == CV_8UC3);

    cv::Mat gray;
    if (grayIn.channels() == 3)
        cv::cvtColor(grayIn, gray, cv::COLOR_BGR2GRAY);
    else
        gray = grayIn.clone();

    cv::Point2f center = initCenter;
    float radius = initRadius;

    const int maxIter = 6;
    const int searchRange = 5;

    for (int iter = 0; iter < maxIter; ++iter)
    {
        double bestScore = 1e9;
        cv::Point2f bestCenter = center;

        // 搜索积分最小的区域（越暗越可能是瞳孔）
        for (int dx = -searchRange; dx <= searchRange; ++dx)
        {
            for (int dy = -searchRange; dy <= searchRange; ++dy)
            {
                cv::Point2f test = center + cv::Point2f(dx, dy);
                if (test.x < radius || test.y < radius ||
                    test.x >= gray.cols - radius || test.y >= gray.rows - radius)
                    continue;

                double score = meanIntensityInCircle(gray, test, radius);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestCenter = test;
                }
            }
        }

        if (cv::norm(bestCenter - center) < 0.3f)
            break;

        center = bestCenter;

        // 关键改动：使用梯度法动态更新半径
        float newRadius = estimateRadius(gray, center, minRadius, maxRadius);
        radius = 0.3f * radius + 0.7f * newRadius; // 加大更新权重

        if (local_debug)
            std::cout << "[iter " << iter << "] center=(" << center.x << ", " << center.y
                      << "), radius=" << radius << std::endl;
    }

    if (local_debug)
    {
        cv::Mat vis;
        cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
        cv::circle(vis, center, static_cast<int>(radius), {0, 255, 0}, 1);

        int crossLen = static_cast<int>(radius * 0.6);
        cv::line(vis,
                 cv::Point(cvRound(center.x - crossLen), cvRound(center.y)),
                 cv::Point(cvRound(center.x + crossLen), cvRound(center.y)),
                 {0, 255, 0}, 1);
        cv::line(vis,
                 cv::Point(cvRound(center.x), cvRound(center.y - crossLen)),
                 cv::Point(cvRound(center.x), cvRound(center.y + crossLen)),
                 {0, 255, 0}, 1);

        return {center, vis, radius};
    }

    return {center, gray, radius};
}

std::tuple<cv::Point2f, cv::Mat>
refinePupilCenterEllipse(const cv::Mat& gray, cv::Point2f initCenter,
                         float initA, float initB,
                         float minA, float maxA,
                         float minB, float maxB)
{
    cv::Point2f center = initCenter;
    float bestA = initA, bestB = initB;
    const int maxIter = 5;
    const int searchRange = 5;

    auto meanIntensityInEllipse = [&](const cv::Mat& img, const cv::Point2f& c, float a, float b) {
        cv::Mat mask = cv::Mat::zeros(img.size(), CV_8U);
        cv::ellipse(mask, c, cv::Size(cvRound(a), cvRound(b)), 0, 0, 360, 255, -1);
        return cv::mean(img, mask)[0];
    };

    for (int iter = 0; iter < maxIter; ++iter)
    {
        double bestScore = 1e9;
        cv::Point2f bestCenter = center;
        float bestLocalA = bestA, bestLocalB = bestB;

        // 遍历中心和椭圆长短轴
        for (int dx = -searchRange; dx <= searchRange; dx++)
        {
            for (int dy = -searchRange; dy <= searchRange; dy++)
            {
                cv::Point2f testC = center + cv::Point2f(dx, dy);
                if (testC.x < 0 || testC.y < 0 || testC.x >= gray.cols || testC.y >= gray.rows)
                    continue;

                for (float a = minA; a <= maxA; a += 2.0f)
                {
                    for (float b = minB; b <= maxB; b += 2.0f)
                    {
                        double score = meanIntensityInEllipse(gray, testC, a, b);
                        if (score < bestScore)
                        {
                            bestScore = score;
                            bestCenter = testC;
                            bestLocalA = a;
                            bestLocalB = b;
                        }
                    }
                }
            }
        }

        // 若中心变化小则停止
        if (cv::norm(bestCenter - center) < 0.3f)
            break;

        center = bestCenter;
        bestA = bestLocalA;
        bestB = bestLocalB;
    }

    // 🔍 输出可视化
    cv::Mat show;
    if (gray.channels() == 1)
        cv::cvtColor(gray, show, cv::COLOR_GRAY2BGR);
    else
        show = gray.clone();

    cv::ellipse(show, center, cv::Size(cvRound(bestA), cvRound(bestB)),
                0, 0, 360, cv::Scalar(255, 255, 255), 1);
    cv::line(show,
             cv::Point(cvRound(center.x - bestA * 0.6f), cvRound(center.y)),
             cv::Point(cvRound(center.x + bestA * 0.6f), cvRound(center.y)),
             cv::Scalar(255, 255, 255), 1);
    cv::line(show,
             cv::Point(cvRound(center.x), cvRound(center.y - bestB * 0.6f)),
             cv::Point(cvRound(center.x), cvRound(center.y + bestB * 0.6f)),
             cv::Scalar(255, 255, 255), 1);

    return {center, show};
}


} // namespace pupilcenter