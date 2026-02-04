#include "utils/visualize.hpp"

namespace visualization
{

cv::Mat visualizeGlintsAndPupil(
    const cv::Mat& frame,
    const std::vector<cv::Point2d>& glints,   // 期望 size == 3
    const cv::Point2d& pupil_center,
	const float radius)
{
    cv::Mat vis;
    if (frame.channels() == 1) cv::cvtColor(frame, vis, cv::COLOR_GRAY2BGR);
    else vis = frame.clone();

    // 如果 glints 数量为 3，就连成三角形并标出
    if (glints.size() == 3) {
        std::vector<cv::Point> pts;
        pts.reserve(3);
        for (const auto& g : glints)
            pts.emplace_back(cv::Point(cvRound(g.x), cvRound(g.y)));

        // 画三角线
        const cv::Scalar triColor(0, 255, 0); // 绿色
        cv::polylines(vis, pts, true, triColor, 1, cv::LINE_AA);

    } else {
        // 如果没有 3 个 glint，也尝试把已有的点画出来
        for (const auto& g : glints)
            cv::circle(vis, cv::Point(cvRound(g.x), cvRound(g.y)), 3, cv::Scalar(0,200,200), cv::FILLED, cv::LINE_AA);
    }

    // 在瞳孔中心画十字
    const int crossLen = 6;
    const cv::Point center(cvRound(pupil_center.x), cvRound(pupil_center.y));
    const cv::Scalar crossColor(0, 0, 255); // 红色
    cv::line(vis, center + cv::Point(-crossLen, 0), center + cv::Point(crossLen, 0), crossColor, 1, cv::LINE_AA);
    cv::line(vis, center + cv::Point(0, -crossLen), center + cv::Point(0, crossLen), crossColor, 1, cv::LINE_AA);
    // 可选：画一个半透明圆环表示不确定区域（这里用实线）
    cv::circle(vis, center, cvRound(radius), crossColor, 1, cv::LINE_AA);

    // 可选：输出坐标文本（你可以注释掉）
    std::ostringstream oss;
    oss << "(" << center.x << "," << center.y << ")" << " " << cvRound(radius);
    cv::putText(vis, oss.str(), center + cv::Point(8, -18),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1, cv::LINE_AA);

    return vis;
}

cv::Mat
visualizeGlints(
    const cv::Mat& frame,
    const std::vector<cv::Point2d>& glints
)
{
    cv::Mat vis;
    vis = frame.clone();

    // 如果 glints 数量为 3，就连成三角形并标出
    if (glints.size() == 3) {
        std::vector<cv::Point> pts;
        pts.reserve(3);
        for (const auto& g : glints)
            pts.emplace_back(cv::Point(cvRound(g.x), cvRound(g.y)));

        // 画三角线
        const cv::Scalar triColor(0, 255, 0); // 绿色
        cv::polylines(vis, pts, true, triColor, 1, cv::LINE_AA);

    } else {
        // 如果没有 3 个 glint，也尝试把已有的点画出来
        for (const auto& g : glints)
            cv::circle(vis, cv::Point(cvRound(g.x), cvRound(g.y)), 3, cv::Scalar(0,200,200), cv::FILLED, cv::LINE_AA);
    }

    return vis;
}

cv::Mat
visualizeGlintList(
    const cv::Mat& frame,
    const std::vector<std::vector<cv::Point2d>>& glintList
)
{
    cv::Mat vis;
    vis = frame.clone();

    for (const auto& glints : glintList)
    {
        // 如果 glints 数量为 3，就连成三角形并标出
        if (glints.size() == 3) {
            std::vector<cv::Point> pts;
            pts.reserve(3);
            for (const auto& g : glints)
                pts.emplace_back(cv::Point(cvRound(g.x), cvRound(g.y)));

            // 画三角线
            const cv::Scalar triColor(0, 255, 0); // 绿色
            cv::polylines(vis, pts, true, triColor, 1, cv::LINE_AA);

        } else {
            // 如果没有 3 个 glint，也尝试把已有的点画出来
            for (const auto& g : glints)
                cv::circle(vis, cv::Point(cvRound(g.x), cvRound(g.y)), 3, cv::Scalar(0,200,200), cv::FILLED, cv::LINE_AA);
        }
    }

    return vis;
}

}