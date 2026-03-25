#include <future>
#include <list>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <limits>
#include <cmath>
#include <filesystem>
#include <chrono>
#include <random>

#include "glint_detection/detect_glint.hpp"
#include "utils/shared_calculations.hpp"

/*
TODO:
✅ 1. optimaize side2mid function (add delta y of left and right point as a parameter, and adjust the range accordingly)
✅ 2. handle two glints case
        ✅ 2.1 side and mid
            2.2.1 find possible side and mid pair
                ⚠️ handle one glint case
            ✅ 2.2.2 search for the last side
    ✅ 2.2 side and side: serach for the mid
4. unify glass and non-glass logic
    ✅ 5.1 determine ROI
        ✅ 5.1.1 remove noisy points
        ✅ 5.1.2 expand the ROI
    5.2 find pupil center
    5.3 find best glint pair
    5.4 automate the calculation of hyperparameters
5. add temporal filter
*/

/*
================================================================================[Hyperparameters List for detect_glint.cpp]
维护说明：以下为在各个函数内部定义的局部硬编码超参数（Hardcoded Hyperparameters），
修改此类参数通常会影响该函数特定步骤的检测灵敏度与过滤严格度。
================================================================================

[searchGlassReflections] - 镜片反光检测
⚠️- kMinContourArea               = 50.0;  // 反光轮廓面积最小值
⚠️- kMaxContourArea               = 500.0; // 反光轮廓面积最大值
⚠️- kRoiPadding                   = 10;    // 提取霍夫圆检测 ROI 时的边界扩展量 (像素)
⚠️- kHoughDp                      = 1.0;   // 霍夫圆累加器分辨率
⚠️- kHoughMinDistDivisor          = 3.0;   // 霍夫圆最小圆心距的除数 (基于ROI高度)
- kHoughParam1                  = 100.0; // 霍夫圆 Canny边缘检测高阈值
- kHoughParam2                  = 5.0;   // 霍夫圆 累加器阈值(投票数)
⚠️- kHoughMinRadius               = 3;     // 霍夫圆 最小检测半径
⚠️- kHoughMaxRadius               = 25;    // 霍夫圆 最大检测半径
- kDistTransformMaskSize        = 5;     // 距离变换掩膜尺寸
- kMinDtRadius                  = 1.0f;  // 极细碎噪点过滤的最小距离变换峰值
- kDtRadiusCentroidThresh       = 3.0f;  // 使用质心替代峰值的DT半径阈值
- kFrameReflectionRatioThresh   = 3.0f;  // 镜框与镜片反光判定比值阈值 (霍夫半径 / DT峰值)
- kMagEpsilon                   = 1e-3f; // 向量模长极小值保护

[isInsideGlassExclusion] - 镜片反光排除判定
- kExclusionRadiusRatio         = 1.2f;  // 镜片反光排除区域半径的放大系数

[visualizePupilAndExclusion] - 瞳孔排除区域可视化
- kExclusionRadiusRatio         = 2.5f;  // 瞳孔排除区域的显示半径系数 (基于长轴)

[searchPupilInROI] - ROI内寻找瞳孔种子
⚠️- kAdaptiveThreshOffset         = 10.0;  // 自适应阈值计算偏移量
⚠️- kAdaptiveThreshMax            = 15.0;  // 自适应阈值最大限制
- kMorphKernelSize              = 5;     // 形态学操作核大小
- kMorphCloseIterations         = 2;     // 形态学闭操作迭代次数
⚠️- kMinPupilArea                 = 50.0;  // 瞳孔轮廓过滤的面积下限
⚠️- kMaxPupilArea                 = 800.0; // 瞳孔轮廓过滤的面积上限
⚠️- kMinPupilContourPoints        = 5;     // 瞳孔轮廓最小点数
⚠️- kMaxPupilAxis                 = 50.0f; // 瞳孔长轴最大限制
⚠️- kMaxAxisRatio                 = 2.0f;  // 瞳孔长宽比最大限制
⚠️- kMinSolidity                  = 0.60;  // 凸包面积比 (Solidity) 最小值
⚠️- kMinFitRatio                  = 0.65;  // 椭圆拟合面积比最小值
⚠️- kValidResidualRatio           = 0.60f; // 残差计算有效点比例
⚠️- kMaxAvgResidual               = 0.40f; // 平均残差最大值
⚠️- kMaxDarkness               = 15.0;  // 暗度上限

[isPupilNearby] - 瞳孔附近判定
⚠️- kExclusionRadiusRatio         = 2.5f;  // 距离阈值系数 (相对于瞳孔长轴)

[findNeighborInDirection] - 方向性寻找相邻反光点
- kDefaultSearchScaleLenRatio   = 3.0f;  // 默认搜索区域扩展系数 (相对当前节点的长度)
- kDefaultSearchScaleWidRatio   = 2.0f;  // 默认搜索区域扩展系数 (相对当前节点的宽度)
- kMinContourArea               = 2.0;   // 过滤噪点的最小面积
- kMinDistRatio                 = 0.5f;  // 节点间最小距离系数 (避免检测到自身)
- kMinReliableLenRatio          = 0.6f;  // 新节点可靠尺度继承的最小长度比例
- kAngleSwapMin                 = 45.0f; // 需要交换长宽的角度范围下限
- kAngleSwapMax                 = 135.0f;// 需要交换长宽的角度范围上限

[searchReflectionChains] - 搜索反光链
- kMinNormEpsilon               = 0.1f;  // 向量更新的最小模长，防跳变
- kMaxChainLength               = 20;    // 反光链的最大节点数

[searchFrameReflections] - 搜索镜框反光
⚠️- kMinArea                      = 10.0;  // 初始镜框反光种子的面积下限
⚠️- kMaxArea                      = 50.0;  // 初始镜框反光种子的面积上限
- kMinLengthWidthRatio          = 2.0f;  // 初始镜框反光种子的长宽比最小限制 (长条状)
- kSearchScaleLenRatio          = 3.0f;  // 搜索区域长向缩放系数
- kSearchScaleWidRatio          = 2.0f;  // 搜索区域宽向缩放系数
- kAngleSwapMin                 = 45.0f; // 需要交换搜索区域长宽的角度范围下限
- kAngleSwapMax                 = 135.0f;// 需要交换搜索区域长宽的角度范围上限
- kMinChainSizeToDraw           = 3;     // 绘制链条时要求的最短链长度

[buildExclusionMask] - 构建排除掩膜
- kGlassExclusionScale          = 1.2f;  // 镜片反光排除区域的放大倍数
- kFrameExclusionScale          = 1.5f;  // 单个镜框反光排除区域的放大倍数
- kChainExclusionScale          = 1.5f;  // 镜框反光链之间连线排除区域的放大倍数

[isInsideExclusionRegion] - 排除区域内部判定
- kMaskThreshold                = 128;   // 掩膜二值化的判定阈值

[shrinkRoiToValidGlints] - 缩小至有效光斑ROI
⚠️- kMaxNoiseArea                 = 5.0;   // 噪点最大像素数/面积
⚠️- kClusterDistThresh            = 80.0;  // 聚集距离阈值
⚠️- kMinClusterSize               = 2;     // 有效聚集的最小数量
⚠️- padding_y                     = 60;    // 扩展边界 (上下方向)
- kDefaultLowSensitivityThresh  = 25.0;  // 默认极低敏感度阈值
⚠️- kPaddingXRatio                = 0.2f;  // 缩小后 ROI 的宽向外扩展比例

[determineCornealReflectionROI] - 确定角膜反光ROI
⚠️- kConstraintRadiusRatio        = 2.5f;  // 瞳孔种子约束区域半径放大系数 (基于长轴)
- kRatioH                       = 3;     // 扩展步长循环控制 (水平)
- kRatioV                       = 1;     // 扩展步长循环控制 (垂直)
⚠️- kMinExpandedRoiArea           = 50;    // 候选区域最小有效面积
- kShrinkRatio                  = 0.05f; // 边界收缩比例 (去除极边缘部分)
- kMinRemainderArea             = 10;    // 差集操作后保留的最小碎片面积

[clusterROIs] - ROI 聚类
- kClusterMargin                = 1;     // 两个矩形扩充交集判定的像素大小

[splitGlintsGeometry] - 左右眼光斑分离
⚠️- kDistanceThresholdX           = 100.0; // 将两眼斑点分为左右眼的X轴距离阈值

[selectBestGlintsPerCluster] - 选择每组最优光斑
- kDistTolerance                = 5.0;   // 距离容差 (像素)
- kBrightTolerance              = 5.0;   // 亮度容差 (0-255)

[detectCluster] - 聚类检测
- kMinUniqueGlintDist           = 2.0;   // 判断新光斑是否属于已有光斑的最短距离阈值
- kMaxClusterResults            = 3;     // 提前退出搜索的目标数量

[findGeometry] - 寻找几何结构
- kMaxMissingPointsToFind       = 2;     // 寻找缺失中点或侧边点的最大数量限制

[isGlintRepeated] - 光斑重复判定
⚠️- kMinGlintDist                 = 1.0;   // 判定两个 glint 是否为同一个的最短欧氏距离阈值

[isGlintGeometryRepeated] - 几何体重复判定
- kPointMatchTolerance          = 0.5;   // 判定两个 Glint 几何点位置相同的容差距离

[checkAndPushGlintGeometry] - 校验并推入光斑结构
⚠️- brightness_threshold          = 25.0;  // 亮斑的背景亮度判定阈值
- kMinPadding                   = 5;     // ROI扩展的最小像素数
- kPaddingRatio                 = 0.20f; // 按比例扩展的系数
⚠️- kDangerMaskThresh             = 50.0;  // 寻找高亮斑(污染区)的二值化阈值
- kDilateKernelSize             = 3;     // 膨胀核尺寸(建立隔离带)
- kDilateIterations             = 1;     // 膨胀迭代次数
- kMinValidPixels               = 5;     // 进行截断平均所需的最少有效像素数
- kTrimRatio                    = 0.10f; // 截断平均的剪裁比例(上下各剔除的百分比)

================================================================================
*/

namespace glintdetection {

GlintDetector::GlintDetector(const std::string& mode)
    : mode_(mode) 
{
	horizontal_pair_cfg_ = (mode == "collect")
        ? cfg_["relaxed_glint_hyperparameter"]["horizontal_pair"]
	    : cfg_["glint_hyperparameter"]["horizontal_pair"];

	middle_point_cfg_ = (mode == "collect")
        ? cfg_["relaxed_glint_hyperparameter"]["middle_point"]
		: cfg_["glint_hyperparameter"]["middle_point"];

    gaussian_kernel_size_ = cfg_["test_glint"]["gaussian_kernel_size"].as<int>();
	laplacian_kernel_size_ = cfg_["test_glint"]["laplacian_kernel_size"].as<int>();

	init_threshold_value_ = cfg_["test_glint"]["init_threshold_value"].as<double>();
    threshold_step_ = cfg_["test_glint"]["threshold_step"].as<double>();
    viz_ = cfg_["test_glint"]["viz"].as<bool>();
}

bool GlintDetector::side2side(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt
)
{
    // 1
    // load hyperparameters
    double lr_x_min = horizontal_pair_cfg_["lr_x_min"].as<double>();
    double lr_x_max = horizontal_pair_cfg_["lr_x_max"].as<double>();
    double lr_y_min = horizontal_pair_cfg_["lr_y_min"].as<double>();
    double lr_y_max = horizontal_pair_cfg_["lr_y_max"].as<double>();

    // 2
    // check if the two points are horizontal pairs
    double lr_x = std::abs(l_pt.x - r_pt.x);
    double lr_y = std::abs(l_pt.y - r_pt.y);

    bool cond_y = (lr_y >= lr_y_min && lr_y < lr_y_max);
    bool cond_x = (lr_x >= lr_x_min && lr_x <= lr_x_max);

    Logger::debug() << "[5 Find Geometry] [side2side]";
    Logger::debug() << "\tleft: (" << l_pt.x << ", " << l_pt.y << ")";
    Logger::debug() << "\tright: (" << r_pt.x << ", " << r_pt.y << ")";
    Logger::debug() << "\tlr_x=" << lr_x
                    << " | qualified: " << cond_x;
    Logger::debug() << "\tlr_y=" << lr_y
                    << " | qualified: " << cond_y;
    Logger::debug() << "\tresult: " << (cond_x && cond_y);

    return cond_x && cond_y;
}

bool GlintDetector::side2mid(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const cv::Point2f& m_pt
)
{
    if (!side2side(l_pt, r_pt)) return false;

    // 1
    // load hyperparameters
    const auto& conditions = middle_point_cfg_["conditions"].as<std::vector<std::vector<double>>>();

    // 2
    // 2.1
    // calculate the relative position of the middle point to the two eyes
    double lr_x = std::abs(l_pt.x - r_pt.x);
    double lr_y = std::abs(l_pt.y - r_pt.y);

    double lm_x = m_pt.x - l_pt.x;
    double lm_y = m_pt.y - l_pt.y;
    double rm_x = r_pt.x - m_pt.x;
    double rm_y = m_pt.y - r_pt.y;

    // 2.2
    // swap left and right points if left point is higher
    if (l_pt.y < r_pt.y)
    {
        std::swap(lm_x, rm_x);
        std::swap(lm_y, rm_y);
    }

    // 3
    // check if the middle point is in the expected position
    for (const auto& cond : conditions)
    {
        // 3.1
        // choose the range of y values
        double y_min = cond[0];
        double y_max = cond[1];

        if (!(lr_y >= y_min && lr_y < y_max)) continue;

        // 3.2
        // calculate the relative position of the middle point to the two eyes
        double lm_x_ratio = lm_x / lr_x;
        double lm_y_ratio = lm_y / lr_x;
        double rm_x_ratio = rm_x / lr_x;
        double rm_y_ratio = rm_y / lr_x;

        // 3.3
        // check if the middle point is in the expected position
        bool lm_condition =
               lm_x_ratio >= cond[2] && lm_x_ratio <= cond[3]
            && lm_y_ratio >= cond[4] && lm_y_ratio <= cond[5];

        bool rm_condition =
               rm_x_ratio >= cond[6] && rm_x_ratio <= cond[7]
            && rm_y_ratio >= cond[8] && rm_y_ratio <= cond[9];

        std::string range_desc;
        if (std::abs(y_min) < 1e-9)
            range_desc = "(lr_y < " + std::to_string((int)y_max) + ")";
        else
            range_desc = "(" + std::to_string((int)y_min)
                        + " <= lr_y < " + std::to_string((int)y_max) + ")";

        Logger::debug() << "[5 Find Geometry] [side2mid]";
        Logger::debug() << "\tleft: (" << l_pt.x << ", " << l_pt.y << ")";
        Logger::debug() << "\tright: (" << r_pt.x << ", " << r_pt.y << ")";
        Logger::debug() << "\tmid: (" << m_pt.x << ", " << m_pt.y << ")";
        Logger::debug() << "\tleft to mid: " << range_desc;
        Logger::debug() << "\t\tlm_x: " << lm_x
                        << " | lr_x: " << lr_x
                        << " | lm_x/lr_x: " << lm_x_ratio
                        << " | qualified: "
                        << (lm_x_ratio >= cond[2] && lm_x_ratio <= cond[3]);

        Logger::debug() << "\t\tlm_y: " << lm_y
                        << " | lr_x: " << lr_x
                        << " | lm_y/lr_x: " << lm_y_ratio
                        << " | qualified: "
                        << (lm_y_ratio >= cond[4] && lm_y_ratio <= cond[5]);

        Logger::debug() << "\tright to mid: " << range_desc;
        Logger::debug() << "\t\trm_x: " << rm_x
                        << " | lr_x: " << lr_x
                        << " | rm_x/lr_x: " << rm_x_ratio
                        << " | qualified: "
                        << (rm_x_ratio >= cond[6] && rm_x_ratio <= cond[7]);

        Logger::debug() << "\t\trm_y: " << rm_y
                        << " | lr_x: " << lr_x
                        << " | rm_y/lr_x: " << rm_y_ratio
                        << " | qualified: "
                        << (rm_y_ratio >= cond[8] && rm_y_ratio <= cond[9]);

        Logger::debug() << "\tresult: " << (lm_condition && rm_condition);

        if (lm_condition && rm_condition)
            return true;
    }

    return false;
}

bool GlintDetector::side2mid(
    const cv::Point2f& s_pt,
    const cv::Point2f& m_pt
)
{
    // 1
    // load hyperparameters
    double lr_x_max = horizontal_pair_cfg_["lr_x_max"].as<double>();
    const auto& conditions = middle_point_cfg_["conditions"].as<std::vector<std::vector<double>>>();

    if (s_pt.x < m_pt.x)
    {
        double lm_x_max = lr_x_max * conditions.back()[3];
        double lm_y_max = lr_x_max * conditions[0][5];

        // 2
        // check
        double lm_x = m_pt.x - s_pt.x;
        double lm_y = m_pt.y - s_pt.y;

        bool lm_condition = lm_x >= 0 && lm_x <= lm_x_max && lm_y >= 0 && lm_y <= lm_y_max;

        Logger::debug() << "[5 Find Geometry] [side2mid]";
        Logger::debug() << "\tlm_x: " << lm_x
                        << " | lm_x_max: " << lm_x_max
                        << " | qualified: " << (lm_x >= 0 && lm_x <= lm_x_max);
        Logger::debug() << "\tlm_y: " << lm_y
                        << " | lm_y_max: " << lm_y_max
                        << " | qualified: " << (lm_y >= 0 && lm_y <= lm_y_max);
        Logger::debug() << "\tresult: " << lm_condition;

        return lm_condition;
    }
    else if (s_pt.x > m_pt.x)
    {
        double rm_x_max = lr_x_max * conditions[0][7];
        double rm_y_max = lr_x_max * conditions.back()[9];

        // 2
        // check
        double rm_x = s_pt.x - m_pt.x;
        double rm_y = m_pt.y - s_pt.y;

        bool rm_condition = rm_x >= 0 && rm_x <= rm_x_max && rm_y >= 0 && rm_y <= rm_y_max;

        Logger::debug() << "[5 Find Geometry] [side2mid]";
        Logger::debug() << "\trm_x: " << rm_x
                        << " | rm_x_max: " << rm_x_max
                        << " | qualified: " << (rm_x >= 0 && rm_x <= rm_x_max);
        Logger::debug() << "\trm_y: " << rm_y
                        << " | rm_y_max: " << rm_y_max
                        << " | qualified: " << (rm_y >= 0 && rm_y <= rm_y_max);
        Logger::debug() << "\tresult: " << rm_condition;

        return rm_condition;
    }
    else
    {
        return false;
    }

    return false;
}

std::tuple<cv::Mat, cv::Point2f>
GlintDetector::getSearchRegionSideAndMid(
    const cv::Point2f& s_pt,
    const cv::Point2f& m_pt
)
{
    int x_min, x_max, y_min, y_max;
    int offset_x = cfg_["test_glint"]["search_region_side_and_mid_offset_x"].as<int>();
    int offset_y = cfg_["test_glint"]["search_region_side_and_mid_offset_y"].as<int>();

    // 1
    // load hyperparameters
    double lr_x_min = horizontal_pair_cfg_["lr_x_min"].as<double>();
    double lr_x_max = horizontal_pair_cfg_["lr_x_max"].as<double>();
    double lr_y_max = horizontal_pair_cfg_["lr_y_max"].as<double>();

    // 2
    // calculate the search ROI
    if (s_pt.x < m_pt.x)
    {
        // left and mid
        x_min = cvCeil (m_pt.x + 1 + offset_x);
        x_max = cvCeil (s_pt.x + lr_x_max);
        y_min = cvFloor(s_pt.y - lr_y_max);
        y_max = cvFloor(m_pt.y + offset_y);
    }
    else
    {
        // right and mid
        x_min = cvFloor(s_pt.x - lr_x_max);
        x_max = cvFloor(m_pt.x - 1 - offset_x);
        y_min = cvFloor(s_pt.y - lr_y_max);
        y_max = cvFloor(m_pt.y + offset_y);
    }

    cv::Rect roi(x_min, y_min, x_max - x_min, y_max - y_min);
    roi &= cv::Rect(0, 0, abs_dst_.cols, abs_dst_.rows);
    cv::Mat search_region = abs_dst_(roi);

    return {search_region, cv::Point2f(x_min, y_min)};
}

std::tuple<cv::Mat, cv::Point2f>
GlintDetector::getSearchRegionSideAndSide(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt
)
{
    int additional_offset_y = cfg_["test_glint"]["search_region_side_and_side_offset_y"].as<int>();

    double lr_x = std::abs(l_pt.x - r_pt.x);
    double offset_y = lr_x * middle_point_cfg_["conditions"]
                      .as<std::vector<std::vector<double>>>().front()[5];
    double side_y_max = std::max(l_pt.y, r_pt.y);

    int x_min = cvCeil(l_pt.x);
    int x_max = cvFloor(r_pt.x);
    int y_min = cvCeil(side_y_max + additional_offset_y);
    int y_max = cvFloor(side_y_max + offset_y + additional_offset_y);

    cv::Rect roi(x_min, y_min, x_max - x_min, y_max - y_min);
    roi &= cv::Rect(0, 0, abs_dst_.cols, abs_dst_.rows);
    cv::Mat search_region = abs_dst_(roi);

    return {search_region, cv::Point2f(x_min, y_min)};
}

void GlintDetector::searchGlassReflections()
{
    // --- 超参数声明 (Hyperparameters) ---
    // 反光轮廓面积的最小和最大值，过滤过小噪点或过大反光
    const double kMinContourArea = 50.0;
    const double kMaxContourArea = 500.0;
    // 提取霍夫圆检测 ROI 时的边界扩展量 (像素)
    const int kRoiPadding = 10;
    // 霍夫圆变换参数
    const double kHoughDp = 1.0;                  // 累加器分辨率
    const double kHoughMinDistDivisor = 3.0;      // 最小圆心距的除数 (基于ROI高度)
    const double kHoughParam1 = 100.0;            // Canny边缘检测高阈值
    const double kHoughParam2 = 5.0;              // 累加器阈值(投票数)，决定检测灵敏度
    const int kHoughMinRadius = 3;                // 最小检测半径
    const int kHoughMaxRadius = 25;               // 最大检测半径
    // 距离变换参数
    const int kDistTransformMaskSize = 5;         // 距离变换掩膜尺寸
    const float kMinDtRadius = 1.0f;              // 极细碎噪点过滤的最小距离变换峰值
    const float kDtRadiusCentroidThresh = 3.0f;   // 使用质心替代峰值的DT半径阈值
    // 镜框与镜片反光判定比值阈值 (霍夫半径 / DT峰值)
    const float kFrameReflectionRatioThresh = 3.0f;
    const float kMagEpsilon = 1e-3f;              // 向量模长极小值保护

    glass_reflections_.clear();
    frame_reflections_.clear();

    // 1. Threshold & Basic Contours
    cv::Mat gray_thr_img;
    // 适当提高阈值，尽量让边缘清晰一些
    cv::threshold(gray_, gray_thr_img, glass_reflection_threshold_, 255, cv::THRESH_BINARY);

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray_thr_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point2f> final_centers;

    if (viz_)
    {
        cv::Mat viz;
        cv::cvtColor(gray_, viz, cv::COLOR_GRAY2BGR);
        debug_imgs_.push_back(viz); // glass reflection viz
    }

    // 2. Iterate through candidate contours
    for (const auto& contour : contours)
    {
        // 2.1 Basic Area Filter (根据你的描述调整)
        double area = cv::contourArea(contour);
        // 面积范围放宽，因为粘连可能会导致面积变大
        if (area < kMinContourArea || area > kMaxContourArea) continue;

        // 2.2 Define ROI (关键优化)
        // 计算包围盒并稍微外扩，确保包含完整的潜在圆形
        cv::Rect boundingRect = cv::boundingRect(contour);
        cv::Rect roiRect = boundingRect + cv::Size(kRoiPadding * 2, kRoiPadding * 2);
        roiRect -= cv::Point(kRoiPadding, kRoiPadding);
        // 确保 ROI 不超出图像边界
        roiRect &= cv::Rect(0, 0, gray_.cols, gray_.rows);

        if (roiRect.empty()) continue;

        // 2.3 Prepare ROI Image for Hough
        // 提取 ROI 的灰度图。霍夫圆变换通常在灰度图上效果更好，因为它内部会做边缘检测。
        cv::Mat roiGray = gray_(roiRect).clone();
        
        // 可选：为了减少ROI内背景噪声的干扰，可以只保留轮廓内的区域
        // (如果背景很干净，这一步可以省略，但加上更稳健)
        cv::Mat mask = cv::Mat::zeros(roiRect.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> roi_contours;
        std::vector<cv::Point> shifted_contour;
        for (const auto& p : contour) shifted_contour.push_back(p - roiRect.tl());
        roi_contours.push_back(shifted_contour);
        cv::drawContours(mask, roi_contours, -1, cv::Scalar(255), cv::FILLED);
        cv::Mat roiGrayMasked;
        roiGray.copyTo(roiGrayMasked, mask);

        // 2.4 Hough Circle Transform (核心步骤)
        std::vector<cv::Vec3f> circles;
        // 参数调优至关重要：
        // dp=1: 累加器分辨率与图像分辨率相同（最精确但慢，ROI很小所以没关系）
        // minDist: 圆心之间的最小距离。在一个小的ROI里我们只期望一个圆，设大点防止检测到同心圆。
        // param1: Canny边缘检测的高阈值。决定了什么样的梯度算边缘。
        // param2: 累加器阈值（投票数）。越小越容易检测到圆（也包括假圆）。需要根据实际图像噪点调整。
        // minRadius/maxRadius: 根据面积经验推算半径范围。sqrt(50/pi)~=4, sqrt(1200/pi)~=20. 适当放宽。
        cv::HoughCircles(roiGrayMasked, circles, cv::HOUGH_GRADIENT, kHoughDp, 
                         roiGrayMasked.rows / kHoughMinDistDivisor, // minDist
                         kHoughParam1, // param1 (Canny high threshold)
                         kHoughParam2, // param2 (Voting threshold - 关键！如果漏检就调小，误检就调大)
                         kHoughMinRadius,   // minRadius
                         kHoughMaxRadius    // maxRadius
        );

        if (circles.empty()) continue;

        // 通常霍夫变换返回的第一个圆是投票数最高的，即可信度最高的
        cv::Vec3f hc_circle = circles[0];
        cv::Point2f local_hc_center(hc_circle[0], hc_circle[1]);
        float hc_radius = hc_circle[2];

        // 2.5 Map back to global coordinates
        cv::Point2f global_hc_center = local_hc_center + cv::Point2f(roiRect.tl());

        // 3.4 Distance Transform
        // DIST_L2: 欧几里得距离, MASK_SIZE 5: 更平滑
        cv::Mat dist_img;
        cv::distanceTransform(roiGrayMasked, dist_img, cv::DIST_L2, kDistTransformMaskSize);

        // 3.5 Find the Peak (The true center of the "blob")
        double maxVal;
        cv::Point maxLoc;
        cv::minMaxLoc(dist_img, nullptr, &maxVal, nullptr, &maxLoc);

        // 3.6 初始获取距离变换中心
        cv::Point2f dt_center = cv::Point2f(maxLoc) + cv::Point2f(roiRect.tl());
        float dt_radius = static_cast<float>(maxVal);

        if (dt_radius < kMinDtRadius) continue; // 极细碎噪点过滤

        // --- ★ 新增：中心点修正逻辑 ★ ---
        cv::Moments mu = cv::moments(contour);
        cv::Point2f centroid(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));

        // 如果 dt_radius 较小，或者 ratio 较大（长条状），质心比 DT 峰值更稳定
        if (dt_radius < kDtRadiusCentroidThresh) {
            dt_center = centroid; // 修正为轮廓重心
        }

        float ratio = hc_radius / dt_radius;

        // 情况 A: 认为是正常的 Glass Reflection
        // 使用最小外接圆半径作为最终半径
        cv::Point2f min_circle_center;
        float min_circle_radius;
        cv::minEnclosingCircle(contour, min_circle_center, min_circle_radius);

        GlintDetector::GlassReflection gr;
        gr.center = global_hc_center; // 依然使用霍夫圆心，通常更准
        gr.radius = min_circle_radius;
        glass_reflections_.push_back(gr);

        if (ratio >= kFrameReflectionRatioThresh)
        {
            // 情况 B: 认为是 Frame Reflection (镜框反光)
            GlintDetector::FrameReflection fr;
            fr.center = dt_center;
            fr.contour = contour;

            // 4.1 计算长边方向 (垂直于 dt->hc 的连线)
            cv::Point2f dir = global_hc_center - dt_center;
            float mag = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            // 垂直向量 (v_x, v_y) -> (-v_y, v_x)
            cv::Point2f long_axis_dir = (mag > kMagEpsilon) ? cv::Point2f(-dir.y / mag, dir.x / mag) : cv::Point2f(1, 0);
            cv::Point2f short_axis_dir = cv::Point2f(-long_axis_dir.y, long_axis_dir.x);

            // 4.2 计算在两个轴向上的投影跨度以确定长宽
            float min_l = FLT_MAX, max_l = -FLT_MAX;
            float min_s = FLT_MAX, max_s = -FLT_MAX;

            for (const auto& pt : contour) {
                cv::Point2f p = cv::Point2f(pt) - dt_center;
                float proj_l = p.x * long_axis_dir.x + p.y * long_axis_dir.y;
                float proj_s = p.x * short_axis_dir.x + p.y * short_axis_dir.y;
                min_l = std::min(min_l, proj_l); max_l = std::max(max_l, proj_l);
                min_s = std::min(min_s, proj_s); max_s = std::max(max_s, proj_s);
            }

            fr.length = max_l - min_l;
            fr.width = max_s - min_s;
            fr.angle_deg = std::atan2(long_axis_dir.y, long_axis_dir.x) * 180.0f / CV_PI;
            if (fr.angle_deg < 0) fr.angle_deg += 180.0f;

            // 4.3 确定 4 个角点
            cv::Point2f half_l = long_axis_dir * (fr.length * 0.5f);
            cv::Point2f half_w = short_axis_dir * (fr.width * 0.5f);
            fr.points[0] = fr.center + half_l + half_w;
            fr.points[1] = fr.center - half_l + half_w;
            fr.points[2] = fr.center - half_l - half_w;
            fr.points[3] = fr.center + half_l - half_w;

            // 这里可以存储到类成员 vector 中
            frame_reflections_.push_back(fr); 

            if (viz_)
            {
                // 可视化：紫色矩形
                for (int i = 0; i < 4; i++)
                {
                    cv::line(debug_imgs_[0], fr.points[i], fr.points[(i+1)%4], cv::Scalar(255, 0, 255), 2);
                }
            }
        }

        if (viz_)
        {
            // --- Visualization Logic ---
            // 青色细线：原始的、混乱的轮廓
            cv::drawContours(debug_imgs_[0], std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 255, 0), 1);
            
            // 红色细圆：霍夫变换“投票”出的最可能的完整圆
            cv::circle(debug_imgs_[0], global_hc_center, static_cast<int>(hc_radius), cv::Scalar(0, 0, 255), 1);
            
            // 红色中心点
            cv::circle(debug_imgs_[0], global_hc_center, 2, cv::Scalar(0, 0, 255), -1);
            
            // 画出 ROI 框方便调试观察
            cv::rectangle(debug_imgs_[0], roiRect, cv::Scalar(255, 0, 0), 1);

            // 蓝色空心圆：算法检测到的镜片反光主体 (Inscribed Circle)
            // 这就是去除了镜框干扰后的结果
            cv::circle(debug_imgs_[0], dt_center, static_cast<int>(dt_radius), cv::Scalar(255, 0, 0), 1);
            
            // 红色中心点
            cv::circle(debug_imgs_[0], dt_center, 1, cv::Scalar(255, 0, 0), -1);

            // 标出比值，以便对比
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << ratio;
            cv::putText(
                debug_imgs_[0], 
                oss.str(), 
                dt_center + cv::Point2f(dt_radius * 3.0f, - dt_radius * 1.0f), 
                cv::FONT_HERSHEY_PLAIN, 
                1.0, 
                cv::Scalar(0, 100, 255)
            );
        }
    }
}

// 辅助函数：判断点是否在 Glass Reflection 的范围内
bool GlintDetector::isInsideGlassExclusion(const cv::Point2f& pt) const {
    // --- 超参数声明 (Hyperparameters) ---
    // 镜片反光排除区域半径的放大系数
    const float kExclusionRadiusRatio = 1.2f;

    for (const auto& gr : glass_reflections_) {
        float dist = cv::norm(pt - gr.center);
        if (dist < kExclusionRadiusRatio * gr.radius) {
            return true; 
        }
    }
    return false;
}

void GlintDetector::visualizePupilAndExclusion()
{
    // --- 超参数声明 (Hyperparameters) ---
    // 瞳孔排除区域的显示半径系数 (基于长轴)
    const float kExclusionRadiusRatio = 2.5f;

    if (debug_imgs_.empty()) return;

    // Visualize all found initial seeds
    for (const auto& pupil : init_pupil_seeds_)
    {
        // 1. Draw pupil ellipse (Green)
        cv::ellipse(debug_imgs_[1], pupil.rr, cv::Scalar(0, 255, 0), 1);
        cv::circle(debug_imgs_[1], pupil.rr.center, 2, cv::Scalar(0, 255, 0), -1);

        // 2. Draw exclusion radius (Cyan/Yellow dashed equivalent)
        float exclusion_radius = pupil.major_axis * kExclusionRadiusRatio;
        cv::circle(
            debug_imgs_[1],
            pupil.rr.center,
            static_cast<int>(exclusion_radius),
            cv::Scalar(0, 200, 255),
            1,
            cv::LINE_AA
        );
    }
}

std::vector<GlintDetector::Pupil> 
GlintDetector::searchPupilInROI(cv::Rect roi_rect)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 自适应阈值计算偏移量与最大限制
    const double kAdaptiveThreshOffset = 10.0;
    const double kAdaptiveThreshMax = 15.0;
    // 形态学操作核大小与闭操作迭代次数
    const int kMorphKernelSize = 5;
    const int kMorphCloseIterations = 2;
    // 瞳孔轮廓过滤的面积上下限
    const double kMinPupilArea = 50.0;
    const double kMaxPupilArea = 800.0;
    // 瞳孔轮廓最小点数
    const size_t kMinPupilContourPoints = 5;
    // 瞳孔长轴与长宽比限制
    const float kMaxPupilAxis = 50.0f;
    const float kMaxAxisRatio = 2.0f;
    // 凸包面积比 (Solidity) 最小值
    const double kMinSolidity = 0.60;
    // 椭圆拟合面积比最小值
    const double kMinFitRatio = 0.65;
    // 残差计算有效点比例与平均残差最大值
    const float kValidResidualRatio = 0.60f;
    const float kMaxAvgResidual = 0.40f;
    // 暗度上限
    const double kMaxDarkness = 15.0;

    std::vector<Pupil> pupils; 

    // 1. 准备 ROI 图像
    cv::Mat roi_img_raw = gray_(roi_rect); 
    cv::Mat roi_img = roi_img_raw.clone(); 

    // 2. 自适应阈值计算 (简化逻辑)
    double min_val;
    cv::minMaxLoc(roi_img, &min_val, nullptr, nullptr, nullptr);
    double adaptive_thresh = std::min(min_val + kAdaptiveThreshOffset, kAdaptiveThreshMax); 

    // 3. 二值化 & 形态学
    cv::Mat binary_pupil;
    cv::threshold(roi_img, binary_pupil, adaptive_thresh, 255, cv::THRESH_BINARY_INV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kMorphKernelSize, kMorphKernelSize));
    cv::morphologyEx(binary_pupil, binary_pupil, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary_pupil, binary_pupil, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), kMorphCloseIterations);

    // 4. 轮廓查找
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_pupil, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); ++i)
    {
        double contour_area = cv::contourArea(contours[i]);
        if (contour_area < kMinPupilArea || contour_area > kMaxPupilArea) continue;
        if (contours[i].size() < kMinPupilContourPoints) continue;

        cv::RotatedRect rr = cv::fitEllipse(contours[i]);
        float major = std::max(rr.size.width, rr.size.height);
        float minor = std::min(rr.size.width, rr.size.height);
        double ellipse_area = (CV_PI * major * minor) / 4.0;

        if (major > kMaxPupilAxis) continue;
        if (major / minor > kMaxAxisRatio) continue;
        
        // Solidity
        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);
        double hull_area = cv::contourArea(hull);
        double solidity = contour_area / hull_area;
        if (solidity < kMinSolidity) continue;

        // Fit Ratio
        double fit_ratio = contour_area / ellipse_area;
        if (fit_ratio < kMinFitRatio) continue;

        // Residual
        float a = major / 2.0f;
        float b = minor / 2.0f;
        float theta = rr.angle * (CV_PI / 180.0f);
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        std::vector<float> point_residuals;
        point_residuals.reserve(contours[i].size());

        for (const auto& pt : contours[i])
        {
            float dx = pt.x - rr.center.x;
            float dy = pt.y - rr.center.y;
            float x_local = dx * cos_t + dy * sin_t;
            float y_local = -dx * sin_t + dy * cos_t;

            float dist_ratio = std::sqrt((x_local * x_local) / (a * a) + (y_local * y_local) / (b * b));
            point_residuals.push_back(std::abs(dist_ratio - 1.0f));
        }

        std::sort(point_residuals.begin(), point_residuals.end());
        size_t valid_count = static_cast<size_t>(point_residuals.size() * kValidResidualRatio);
        if (valid_count == 0) valid_count = point_residuals.size();
        float sum_residual = 0.0f;
        for (size_t k = 0; k < valid_count; ++k) sum_residual += point_residuals[k];
        float avg_residual = sum_residual / valid_count;

        if (avg_residual > kMaxAvgResidual) continue;

        // Darkness (保留用于排序)
        cv::Mat mask = cv::Mat::zeros(binary_pupil.size(), CV_8UC1);
        cv::drawContours(mask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
        cv::Scalar mean_val = cv::mean(roi_img_raw, mask);
        double darkness = mean_val[0];

        if (darkness > kMaxDarkness) continue;

        // 构造 Pupil 对象
        Pupil p;
        p.rr = rr;
        p.rr.center += cv::Point2f(roi_rect.tl()); // 转为全局坐标
        p.major_axis = major;
        p.minor_axis = minor;
        p.darkness = darkness;
        
        pupils.push_back(p);
    }
    
    // 按暗度排序 (越黑越好)，供后续优先使用
    std::sort(pupils.begin(), pupils.end(),[](const Pupil& a, const Pupil& b){
        return a.darkness < b.darkness;
    });

    return pupils;
}

bool GlintDetector::isPupilNearby(const cv::Point2f& glint_pt)
{
    const float kExclusionRadiusRatio = 2.5f; // 距离阈值

    // 直接遍历缓存的 seeds
    for (const auto& pupil : init_pupil_seeds_)
    {
        // 只有纯几何距离判断，没有任何暗度/分数计算
        if ((cv::norm(glint_pt - pupil.rr.center) < (pupil.major_axis * kExclusionRadiusRatio))) {
            return true;
        }
    }
    return false;
}

// 辅助函数：计算两点角度
float calcAngle(const cv::Point2f& p1, const cv::Point2f& p2) {
    float angle = std::atan2(p2.y - p1.y, p2.x - p1.x) * 180.0f / CV_PI;
    if (angle < 0) angle += 180.0f; // 归一化到 [0, 180)
    return angle;
}

bool GlintDetector::findNeighborInDirection(
    const FrameReflection& current_fr,
    const cv::Point2f& search_dir, // 单位方向向量
    FrameReflection& out_new_fr    // 输出找到的新节点
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 默认搜索区域扩展系数 (相对当前节点的长度)
    const float kDefaultSearchScaleLenRatio = 3.0f;
    const float kDefaultSearchScaleWidRatio = 2.0f;
    // 过滤噪点的最小面积
    const double kMinContourArea = 2.0;
    // 节点间最小距离系数 (避免检测到自身)
    const float kMinDistRatio = 0.5f;
    // 新节点可靠尺度继承的最小长度比例
    const float kMinReliableLenRatio = 0.6f;
    // 需要交换长宽的角度范围
    const float kAngleSwapMin = 45.0f;
    const float kAngleSwapMax = 135.0f;

    // ---- ROI 尺度：优先使用 search_scale ----
    float roi_len, roi_wid;

    if (current_fr.search_scale.valid()) {
        roi_len = current_fr.search_scale.roi_len;
        roi_wid = current_fr.search_scale.roi_wid;
    } else {
        roi_len = kDefaultSearchScaleLenRatio * current_fr.length;
        roi_wid = kDefaultSearchScaleWidRatio * current_fr.length;
    }

    // ★ 关键逻辑：利用 search_dir 决定 ROI 的轴向布局 ★
    // 如果 search_dir 偏向垂直方向 (|y| > |x|)，则 Rect 的高应该是 len，宽应该是 wid
    float rect_w = roi_len;
    float rect_h = roi_wid;
    if (std::abs(search_dir.y) > std::abs(search_dir.x)) {
        std::swap(rect_w, rect_h);
    }

    float offset_dist = 0.5f * current_fr.length + 0.5f * roi_len;

    cv::Point2f roi_center = current_fr.center + search_dir * offset_dist;
    
    // 构建 ROI Rect
    cv::Rect roi(
        int(roi_center.x - roi_len * 0.5f),
        int(roi_center.y - roi_wid * 0.5f),
        int(roi_len),
        int(roi_wid)
    );
    // 边界保护
    roi &= cv::Rect(0, 0, abs_dst_.cols, abs_dst_.rows);
    if (roi.area() <= 0) return false;

    if (viz_) cv::rectangle(debug_imgs_[1], roi, cv::Scalar(255, 0, 0), 1);

    // 获取 ROI 图像
    cv::Mat roi_img = abs_dst_(roi);
    
    // 最佳候选
    bool found_candidate = false;
    float min_dist_sq = FLT_MAX; // 找离当前节点最近的，还是最显著的？通常找最近的比较稳

    // ---- 动态降低阈值循环 ----
    for (double thr = glass_reflection_threshold_; thr >= threshold_step_; thr -= threshold_step_)
    {
        cv::Mat thr_img;
        cv::threshold(roi_img, thr_img, thr, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thr_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& cnt : contours)
        {
            double area = cv::contourArea(cnt);
            if (area < kMinContourArea) continue; // 过滤噪点

            // 转换坐标到全图
            std::vector<cv::Point> global_cnt = cnt;
            for(auto& p : global_cnt) {
                p.x += roi.x;
                p.y += roi.y;
            }

            cv::RotatedRect rect = cv::minAreaRect(global_cnt);

            // ★ 新增：排除落在禁区内的候选节点
            if (isInsideGlassExclusion(rect.center)) continue;

            // 距离判定：不能太近 (避免检测到自己)，也不能太远
            float dist = cv::norm(rect.center - current_fr.center);
            if (dist < kMinDistRatio * current_fr.length) continue; // Too close

            if (isPupilNearby(rect.center)) continue;

            // ---- 构造新的 FrameReflection (后处理) ----
            // 关键：角度构造。
            // 规则：根据“连线方向”来决定新 Glint 的长边方向。
            
            // 1. 计算连线向量角度
            float link_angle = calcAngle(current_fr.center, rect.center);
            
            // 2. 获取原始拟合的长宽
            float w = rect.size.width;
            float h = rect.size.height;
            float raw_angle = rect.angle;
            
            // OpenCV MinAreaRect 角度比较混乱，这里简化处理：
            // 我们强制构造一个新的 FR，其 length 方向尽量平行于 link_angle
            
            FrameReflection candidate;
            candidate.center = rect.center;
            candidate.contour = global_cnt;
            rect.points(candidate.points);

            // 简单的逻辑：假设新的 Glint 是比较圆的或者小的，我们主要信赖连线方向
            // 如果一定要区分长宽，可以判断 rect 自身的长轴与 link_angle 的夹角
            // 这里为了稳健，直接将连线方向作为新 FR 的 angle_deg
            candidate.angle_deg = link_angle; 
            
            // 简单赋值长宽 (对于小光斑，长宽可能差不多)
            candidate.length = std::max(w, h);
            candidate.width = std::min(w, h);

            // ---- 搜索尺度继承逻辑 ----
            const float kMinReliableLen = kMinReliableLenRatio * current_fr.length;

            if (candidate.length >= kMinReliableLen) {
                // 新 glint 尺度可靠 → 使用自身
                candidate.search_scale.roi_len = kDefaultSearchScaleLenRatio * candidate.length;
                candidate.search_scale.roi_wid = kDefaultSearchScaleWidRatio * candidate.length;
                if (candidate.angle_deg > kAngleSwapMin && candidate.angle_deg < kAngleSwapMax)
                {
                    std::swap(candidate.search_scale.roi_len, candidate.search_scale.roi_wid);
                }
            } else {
                // 新 glint 过小 → 继承父节点
                candidate.search_scale = current_fr.search_scale;
            }
            
            // 如果在当前阈值找到了符合条件的，且是最优的（这里简化为找到第一个符合距离的即可，或者找面积最大的）
            // 为了策略：只要找到一个合法的，我们就认为在这个方向“接上”了，并且应该停止更低阈值的搜索以防噪点
            out_new_fr = candidate;
            found_candidate = true;
            
            // 可视化：画出新发现的这个 Glint
            if (viz_) {
                cv::circle(debug_imgs_[1], candidate.center, 2, cv::Scalar(255, 255, 0), -1); // 青色点
            }
        }
        
        // 停止条件：一旦在当前阈值层级找到了这一侧的 Glint，就停止下探阈值
        // 这样保证了我们在较高的信噪比下找到反光
        if (found_candidate) break; 
    }

    return found_candidate;
}

std::vector<GlintDetector::FrameReflectionChain>
GlintDetector::searchReflectionChains(
    std::vector<FrameReflection>& initial_seeds)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 向量更新的最小模长，防止除以零或由于距离太近导致方向剧烈跳变
    const float kMinNormEpsilon = 0.1f;
    // 反光链的最大节点数，防止无限循环或过度延伸
    const size_t kMaxChainLength = 20;

    std::vector<FrameReflectionChain> all_chains;

    // 标记已访问，避免重复搜索
    // 由于我们是在动态生成新节点，这里主要标记 initial_seeds
    // 实际工程中，可以通过空间索引判断新生成的点是否靠近已有链条，这里简化处理。

    for (auto& seed : initial_seeds) {
        if (seed.visited) continue;
        seed.visited = true;

        FrameReflectionChain current_chain;
        // 链的中心点是种子
        // 我们用 deque 方便前后插入，最后转 vector
        std::deque<FrameReflection> dq;
        dq.push_back(seed);

        // ---- 向“前”搜索 (Positive Direction) ----
        // 初始方向由 seed 的角度决定
        float theta = seed.angle_deg * CV_PI / 180.0f;
        cv::Point2f curr_dir(std::cos(theta), std::sin(theta)); 
        
        FrameReflection curr_node = seed;
        while (true) {
            FrameReflection next_node;
            // 尝试在 curr_dir 方向找
            if (findNeighborInDirection(curr_node, curr_dir, next_node)) {
                dq.push_back(next_node);
                
                // 更新方向：新方向 = 新节点中心 - 旧节点中心
                cv::Point2f vec = next_node.center - curr_node.center;
                float norm = cv::norm(vec);
                if (norm > kMinNormEpsilon) curr_dir = vec / norm; // 更新搜索方向
                
                curr_node = next_node; // 步进
            } else {
                break; // 这一侧断了
            }
            // 链太长保护
            if (dq.size() > kMaxChainLength) break; 
        }

        // ---- 向“后”搜索 (Negative Direction) ----
        // 重置方向为 seed 的反方向
        curr_dir = cv::Point2f(-std::cos(theta), -std::sin(theta));
        curr_node = seed; // 回到种子

        while (true) {
            FrameReflection prev_node;
            if (findNeighborInDirection(curr_node, curr_dir, prev_node)) {
                dq.push_front(prev_node);
                
                // 更新方向 (注意是 现节点 指向 新发现的后方节点)
                cv::Point2f vec = prev_node.center - curr_node.center;
                float norm = cv::norm(vec);
                if (norm > kMinNormEpsilon) curr_dir = vec / norm;

                curr_node = prev_node;
            } else {
                break; 
            }
            if (dq.size() > kMaxChainLength) break;
        }

        // 转存为 vector
        current_chain.assign(dq.begin(), dq.end());
        all_chains.push_back(current_chain);
    }
    
    std::sort(all_chains.begin(), all_chains.end(),[](const FrameReflectionChain& a, const FrameReflectionChain& b){
            return a.size() > b.size();
        });

    return all_chains;
}

void GlintDetector::searchFrameReflections()
{
    // --- 超参数声明 (Hyperparameters) ---
    // 初始镜框反光种子的面积限制
    const double kMinArea = 10.0;
    const double kMaxArea = 50.0;
    // 初始镜框反光种子的长宽比最小限制 (长条状)
    const float kMinLengthWidthRatio = 2.0f;
    // ROI 搜索区域的宽、高缩放系数 (相对于反光斑长边)
    const float kSearchScaleLenRatio = 3.0f;
    const float kSearchScaleWidRatio = 2.0f;
    // 需要交换搜索区域长宽的角度范围
    const float kAngleSwapMin = 45.0f;
    const float kAngleSwapMax = 135.0f;
    // 绘制链条时要求的最短链长度
    const size_t kMinChainSizeToDraw = 3;

    if (viz_)
    {
        // ---- 可视化底图 ----
        cv::Mat viz;
        cv::cvtColor(gray_, viz, cv::COLOR_GRAY2BGR);
        debug_imgs_.push_back(viz); // frame reflection viz
    }

    // 1. Threshold
    cv::Mat gray_thr_img;
    cv::threshold(gray_, gray_thr_img,
                  glass_reflection_threshold_, 255,
                  cv::THRESH_BINARY);

    // 2. Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray_thr_img, contours,
                     cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // 3. Process contours
    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area < kMinArea || area > kMaxArea)
            continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);

        // ★ 新增：排除落在 Glass Reflection 范围内的种子
        if (isInsideGlassExclusion(rect.center)) continue;

        float w = rect.size.width;
        float h = rect.size.height;
        if (w <= 0 || h <= 0)
            continue;

        float length = std::max(w, h);
        float width  = std::min(w, h);

        if (length / width < kMinLengthWidthRatio)
            continue;

        if (isPupilNearby(rect.center)) continue;

        // ---- 角度统一到[0, 180) ----
        float angle = rect.angle;
        if (w < h)
            angle += 90.0f;
        if (angle < 0.0f)
            angle += 180.0f;

        // ---- 保存结果 ----
        FrameReflection fr;
        fr.center    = rect.center;
        rect.points(fr.points);
        fr.length    = length;
        fr.width     = width;
        fr.angle_deg = angle;
        fr.search_scale.roi_len = kSearchScaleLenRatio * fr.length;
        fr.search_scale.roi_wid = kSearchScaleWidRatio * fr.length;
        if (angle > kAngleSwapMin && angle < kAngleSwapMax)
        {
            std::swap(fr.length, fr.width);
        }
        frame_reflections_.push_back(fr);
    }

    if (viz_)
    {
        for (size_t i = 0; i < frame_reflections_.size(); ++i)
        {
            GlintDetector::FrameReflection fr = frame_reflections_[i];

            for (int i = 0; i < 4; ++i)
            {
                cv::line(
                    debug_imgs_[1],
                    fr.points[i],
                    fr.points[(i + 1) % 4],
                    cv::Scalar(0, 0, 255), 
                    2
                );
            }

            // 标出角度
            cv::Point2f angle_pt = fr.center + cv::Point2f(0.75f * fr.length, - 1.5f * fr.width);
            cv::putText(debug_imgs_[1],
                        std::to_string(static_cast<int>(fr.angle_deg)) + "deg",
                        angle_pt,
                        cv::FONT_HERSHEY_PLAIN,
                        1.0f,
                        cv::Scalar(0, 0, 255),
                        1);
        }
    }

    // Step 3 & 4: 链式搜索
    auto chains = searchReflectionChains(frame_reflections_);
    frame_reflection_chains_ = chains;

    if (viz_)
    {
        // ---- 可视化 ----
        if (!chains.empty()) 
        {
            for (const auto& chain : chains)
            {
                for (size_t i = 0; i < chain.size(); ++i)
                {
                    const auto& fr = chain[i];

                    if (chain.size() < kMinChainSizeToDraw) continue; // 链太短不画

                    // 1. 画连接线 (连成一条龙)
                    if (i > 0) {
                        cv::line(debug_imgs_[1], chain[i-1].center, fr.center, 
                                cv::Scalar(255, 255, 0), 1); // 蓝色连线
                    }

                    // 2. 标序号
                    cv::putText(debug_imgs_[1], std::to_string(i), fr.center + cv::Point2f(0, -10), 
                                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 1);
                }
            }
        }

        // ---- 可视化 pupil 及其排除范围 ----
        visualizePupilAndExclusion();
    }
}

void GlintDetector::drawRotatedRectMask(cv::Mat& mask, const cv::RotatedRect& rr, const cv::Scalar& color)
{
    cv::Point2f vertices[4];
    rr.points(vertices);
    std::vector<cv::Point> pts;
    for (int i = 0; i < 4; i++) pts.push_back(vertices[i]);
    std::vector<std::vector<cv::Point>> contours = { pts };
    cv::drawContours(mask, contours, 0, color, cv::FILLED);
}

void GlintDetector::buildExclusionMask()
{
    // --- 超参数声明 (Hyperparameters) ---
    // 镜片反光排除区域的放大倍数
    const float kGlassExclusionScale = 1.2f;
    // 单个镜框反光排除区域的放大倍数
    const float kFrameExclusionScale = 1.5f;
    // 镜框反光链之间连线排除区域的放大倍数
    const float kChainExclusionScale = 1.5f;

    // 初始化全黑掩膜
    exclusion_mask_ = cv::Mat::zeros(gray_.size(), CV_8UC1);

    // --- 1. 处理镜片反光 (Glass Reflections) ---
    // 判据：1.2倍半径范围
    for (const auto& gr : glass_reflections_)
    {
        int ex_radius = static_cast<int>(gr.radius * kGlassExclusionScale);
        cv::circle(exclusion_mask_, gr.center, ex_radius, cv::Scalar(255), cv::FILLED);
    }

    // --- 2. 处理单个镜框反光 (Frame Reflections) ---
    // 判据：单个 frame reflection 长宽各扩大 1.5 倍
    for (const auto& fr : frame_reflections_)
    {
        // 构造旋转矩形
        cv::RotatedRect rr(fr.center, cv::Size2f(fr.length, fr.width), fr.angle_deg);
        
        // 扩大尺寸
        rr.size.width *= kFrameExclusionScale;
        rr.size.height *= kFrameExclusionScale;

        drawRotatedRectMask(exclusion_mask_, rr, cv::Scalar(255));
    }

    // --- 3. 处理镜框反光链 (Frame Reflection Chains) ---
    for (const auto& chain : frame_reflection_chains_)
    {
        if (chain.size() < 2) continue;

        // 3.1 找到该链条中最大的宽 (width)
        float max_width = 0.0f;
        for (const auto& fr : chain) {
            if (fr.width > max_width) max_width = fr.width;
        }

        // 3.2 遍历链条中的每一对相邻节点
        for (size_t i = 0; i < chain.size() - 1; ++i)
        {
            const auto& curr = chain[i];
            const auto& next = chain[i+1];

            // 计算两点间距和中心
            float dist = cv::norm(curr.center - next.center);
            cv::Point2f mid_pt = (curr.center + next.center) * 0.5f;
            
            // 计算连线角度
            float angle = std::atan2(next.center.y - curr.center.y, next.center.x - curr.center.x) * 180.0f / CV_PI;

            // 定义区域：长=连线距离，宽=链最大宽
            cv::RotatedRect chain_rect(mid_pt, cv::Size2f(dist, max_width), angle);

            // 判据：扩大尺寸
            chain_rect.size.width *= kChainExclusionScale;
            chain_rect.size.height *= kChainExclusionScale;

            drawRotatedRectMask(exclusion_mask_, chain_rect, cv::Scalar(255));
        }
    }
    
    if (viz_) debug_imgs_.push_back(exclusion_mask_); // exclusion mask viz
}

bool GlintDetector::isInsideExclusionRegion(const cv::Point2f& pt) const
{
    // --- 超参数声明 (Hyperparameters) ---
    // 掩膜二值化的判定阈值
    const uchar kMaskThreshold = 128;

    // 边界检查
    if (pt.x < 0 || pt.x >= exclusion_mask_.cols || 
        pt.y < 0 || pt.y >= exclusion_mask_.rows) {
        return true; // 越界视为无效区域
    }

    // 查表：如果掩膜值大于阈值，则在排除区域内
    return exclusion_mask_.at<uchar>(cv::Point(pt)) > kMaskThreshold;
}

// 辅助函数：尝试向某个方向扩张矩形，增加了 limit_rect 限制
bool GlintDetector::expandRect(cv::Rect& rect, const cv::Mat& mask, const cv::Rect& limit_rect, int direction)
{
    // direction: 0-Left, 1-Right, 2-Top, 3-Bottom
    cv::Rect test_rect = rect;

    // 1. 拟定扩张后的测试矩形
    switch (direction)
    {
    case 0: test_rect.x -= 1; test_rect.width = 1; break; // Left (取左边新增的一列)
    case 1: test_rect.x += rect.width; test_rect.width = 1; break; // Right (取右边新增的一列)
    case 2: test_rect.y -= 1; test_rect.height = 1; break; // Top (取上边新增的一行)
    case 3: test_rect.y += rect.height; test_rect.height = 1; break; // Bottom (取下边新增的一行)
    }

    // 2. 越界检查 (Image Boundary)
    if (test_rect.x < 0 || test_rect.y < 0 || 
        test_rect.x + test_rect.width > mask.cols || 
        test_rect.y + test_rect.height > mask.rows)
    {
        return false;
    }

    // 3. [新增] 限制区域检查 (Limit Rect Boundary)
    // 必须包含在 limit_rect 内部。注意 test_rect 只是新增的那一行/列
    // 简单的判断方法：判断 test_rect 是否完全在 limit_rect 内
    if ((test_rect & limit_rect) != test_rect)
    {
        return false; // 说明 test_rect 有一部分超出了 limit_rect
    }

    // 4. 障碍物检查 (Exclusion Mask 255 为障碍)
    // 只要有一点重叠就算撞墙
    cv::Mat region = mask(test_rect);
    if (cv::countNonZero(region) > 0)
    {
        return false;
    }

    // 5. 确认扩张
    switch (direction)
    {
    case 0: rect.x--; rect.width++; break;
    case 1: rect.width++; break;
    case 2: rect.y--; rect.height++; break;
    case 3: rect.height++; break;
    }

    return true;
}

cv::Rect GlintDetector::shrinkRoiToValidGlints(const cv::Rect& coarse_roi)
{
    // =========================================================================
    // 超参数声明 (Hyperparameters for Noise Filtering)
    // 适配不同分辨率时，请按图像比例放大/缩小以下参数。
    // 当前默认参数适用于常规摄像头分辨率（如 720p / 1080p 级别）
    // =========================================================================
    
    // 1. 噪点最大像素数/面积 (对应特征1：一般在5个pixel以下)
    // 如果一个亮斑的面积 <= 此值，它被视作“潜在噪点”，需进一步结合聚集情况判断。
    const double kMaxNoiseArea = 5.0;       
    
    // 2. 聚集距离阈值 (对应特征2：判定几个亮斑是否属于同一个“眼部聚集区”)
    // 两个连通域中心距离小于此值，则归为同一聚类 (Cluster)。
    const double kClusterDistThresh = 80.0; 
    
    // 3. 有效聚集的最小数量 (对应特征2：双眼区域glints聚集一般在3个及以上)
    // 如果一个聚类中全是微小斑点，且数量 < 此值，则整个聚类被视为孤立噪点剔除。
    const int kMinClusterSize = 2;          

    // 4. 扩展边界 (上下方向)
    const int padding_y = 60;
    
    // 5. 默认极低敏感度阈值 (当 threshold_step_ 无效时)
    const double kDefaultLowSensitivityThresh = 25.0;

    // 6. 缩小后 ROI 的宽向外扩展比例 (左右各占)
    const float kPaddingXRatio = 0.2f;
    // =========================================================================

    // 1. 边界保护与 ROI 提取
    cv::Rect valid_roi = coarse_roi & cv::Rect(0, 0, gray_.cols, gray_.rows);
    if (valid_roi.empty()) return cv::Rect();

    cv::Mat roi_img = abs_dst_(valid_roi);

    // 2. 阈值处理 (保留原有的极低敏感度阈值)
    double low_sensitivity_thresh = (threshold_step_ > 0.0) ? threshold_step_ : kDefaultLowSensitivityThresh;
    
    cv::Mat thr_img;
    cv::threshold(roi_img, thr_img, low_sensitivity_thresh, 255, cv::THRESH_BINARY);

    // 3. 寻找连通域 (替代原有的全局 boundingRect)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thr_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return cv::Rect();

    // 4. 记录每个轮廓的信息以便聚类分析
    struct BlobInfo {
        cv::Rect rect;
        cv::Point2f center;
        double area;
    };

    std::vector<BlobInfo> blobs;
    blobs.reserve(contours.size());
    for (const auto& cnt : contours) {
        BlobInfo blob;
        blob.rect = cv::boundingRect(cnt);
        
        // 对于极小的像素点，cv::contourArea 可能为 0。因此结合点集大小作为面积的保底估计
        blob.area = std::max(cv::contourArea(cnt), static_cast<double>(cnt.size())); 
        
        // 计算中心点
        blob.center = cv::Point2f(blob.rect.x + blob.rect.width * 0.5f, 
                                  blob.rect.y + blob.rect.height * 0.5f);
        blobs.push_back(blob);
    }

    // 5. 使用 OpenCV 内置的 cv::partition 根据距离阈值进行聚类
    std::vector<int> labels;
    int num_clusters = cv::partition(blobs, labels,[kClusterDistThresh](const BlobInfo& a, const BlobInfo& b) {
        return cv::norm(a.center - b.center) < kClusterDistThresh;
    });

    // 6. 统计每个聚类的信息：包含几个斑点？是否有任何面积足够大的斑点？
    std::vector<int> cluster_sizes(num_clusters, 0);
    std::vector<bool> cluster_has_large_blob(num_clusters, false);

    for (size_t i = 0; i < blobs.size(); ++i) {
        int label = labels[i];
        cluster_sizes[label]++;
        if (blobs[i].area > kMaxNoiseArea) {
            cluster_has_large_blob[label] = true;
        }
    }

    // 7. 过滤噪点并融合有效的 Bounding Rect
    cv::Rect final_local_bound;
    bool found_valid = false;

    for (size_t i = 0; i < blobs.size(); ++i) {
        int label = labels[i];
        
        // 保留条件：
        // (1) 该聚类中至少存在一个足够大的斑点 (证明有明确的结构特征，哪怕它孤立也保留防误杀)
        // (2) 或者该聚类内全是小斑点，但数量 >= kMinClusterSize (满足双眼区聚集特征)
        if (cluster_has_large_blob[label] || cluster_sizes[label] >= kMinClusterSize) {
            if (!found_valid) {
                final_local_bound = blobs[i].rect;
                found_valid = true;
            } else {
                final_local_bound |= blobs[i].rect; // 取并集扩展 ROI
            }
        }
    }

    // 如果所有轮廓都被判定为噪点
    if (!found_valid)
    {
        Logger::error() << "[shrinkRoiToValidGlints] No valid glints found to shrink ROI.";
        return cv::Rect();
    }

    // 8. 坐标转换 (局部 -> 全局)
    cv::Rect refined_rect = final_local_bound;
    refined_rect.x += valid_roi.x;
    refined_rect.y += valid_roi.y;

    // 9. 适当扩展 (根据瞳孔特征硬编码扩展量)
    // 宽：左右各自拓展比例
    int padding_x = static_cast<int>(final_local_bound.width * kPaddingXRatio);

    refined_rect.x -= padding_x;
    refined_rect.y -= padding_y;
    refined_rect.width += (padding_x * 2);
    refined_rect.height += (padding_y * 2);

    // 10. 最终边界限制
    refined_rect &= valid_roi;

    return refined_rect;
}

std::vector<cv::Rect> GlintDetector::determineCornealReflectionROI()
{
    // --- 超参数声明 (Hyperparameters) ---
    // 瞳孔种子约束区域半径放大系数 (基于长轴)
    const float kConstraintRadiusRatio = 2.5f;
    // 扩展步长循环控制
    const int kRatioH = 3;
    const int kRatioV = 1;
    // 候选区域最小有效面积
    const int kMinExpandedRoiArea = 50;
    // 边界收缩比例 (去除极边缘部分)
    const float kShrinkRatio = 0.05f;
    // 差集操作后保留的最小碎片面积
    const int kMinRemainderArea = 10;

    std::vector<cv::Rect> final_rois;
    std::vector<cv::Point> debug_pupil_centers;
    
    // Use the member variable glint_rect_ calculated in getROI
    if (glint_rect_.empty()) {
        Logger::debug() << "[determineROI] No glints found (glint_rect_ is empty).";
        return final_rois;
    }

    cv::Rect full_img_rect(0, 0, gray_.cols, gray_.rows);

    cv::Rect global_limit_rect = glint_rect_;
    global_limit_rect &= full_img_rect;

    std::vector<cv::Rect> expanded_candidates;

    // --- Optimized Loop using init_pupil_seeds_ ---
    Logger::debug() << "[determineROI] Using pre-calculated pupil seeds: " << init_pupil_seeds_.size();

    for (const auto& pupil : init_pupil_seeds_)
    {
        cv::Point seed_pt = pupil.rr.center;
        
        // Exclude seeds inside the exclusion mask
        if (isInsideExclusionRegion(seed_pt)) continue;
        
        debug_pupil_centers.push_back(seed_pt);

        // Dynamic constraint based on pupil size
        float constraint_radius = pupil.major_axis * kConstraintRadiusRatio;
        
        cv::Rect pupil_constraint_rect(
            static_cast<int>(seed_pt.x - constraint_radius),
            static_cast<int>(seed_pt.y - constraint_radius),
            static_cast<int>(constraint_radius * 2),
            static_cast<int>(constraint_radius * 2)
        );

        cv::Rect current_limit_rect = global_limit_rect & pupil_constraint_rect;
        current_limit_rect &= full_img_rect;

        // Expansion logic
        cv::Rect expanding_roi(seed_pt.x, seed_pt.y, 1, 1);
        
        bool can_grow_l = true, can_grow_r = true, can_grow_t = true, can_grow_b = true;
        bool global_any_grow = true; 

        while (global_any_grow)
        {
            global_any_grow = false;
            for (int i = 0; i < kRatioH; ++i) {
                if (can_grow_l) { if (expandRect(expanding_roi, exclusion_mask_, current_limit_rect, 0)) global_any_grow = true; else can_grow_l = false; }
                if (can_grow_r) { if (expandRect(expanding_roi, exclusion_mask_, current_limit_rect, 1)) global_any_grow = true; else can_grow_r = false; }
            }
            for (int i = 0; i < kRatioV; ++i) {
                if (can_grow_t) { if (expandRect(expanding_roi, exclusion_mask_, current_limit_rect, 2)) global_any_grow = true; else can_grow_t = false; }
                if (can_grow_b) { if (expandRect(expanding_roi, exclusion_mask_, current_limit_rect, 3)) global_any_grow = true; else can_grow_b = false; }
            }
        }

        if (expanding_roi.area() > kMinExpandedRoiArea) {
            int shrink_x = - static_cast<int>(expanding_roi.width  * kShrinkRatio);
            int shrink_y = - static_cast<int>(expanding_roi.height * kShrinkRatio);
            
            cv::Rect shrunk_roi = expanding_roi;
            shrunk_roi.x -= shrink_x; 
            shrunk_roi.y -= shrink_y;
            shrunk_roi.width += (shrink_x * 2);
            shrunk_roi.height += (shrink_y * 2);
            
            expanded_candidates.push_back(shrunk_roi);
        }
    }

    if (viz_)
    {
        // [Visualizer logic mostly same, just updating usage]
        cv::Mat viz_raw = gray_.clone();
        if (viz_raw.channels() == 1) cv::cvtColor(viz_raw, viz_raw, cv::COLOR_GRAY2BGR);
        
        cv::Mat mask_viz;
        cv::cvtColor(exclusion_mask_, mask_viz, cv::COLOR_GRAY2BGR);
        cv::addWeighted(viz_raw, 0.7, mask_viz, 0.3, 0, viz_raw);

        cv::rectangle(viz_raw, global_limit_rect, cv::Scalar(0, 255, 255), 1);
        for (const auto& r : expanded_candidates) {
            cv::rectangle(viz_raw, r, cv::Scalar(0, 255, 0), 1);
        }
        for (const auto& p : debug_pupil_centers) {
            cv::circle(viz_raw, p, 3, cv::Scalar(0, 0, 255), -1);
        }
        
        debug_imgs_.push_back(viz_raw);
    }

    // ...[Layered Difference Logic remains unchanged] ...
    if (!expanded_candidates.empty())
    {
        std::sort(expanded_candidates.begin(), expanded_candidates.end(),[](const cv::Rect& a, const cv::Rect& b) { return a.area() > b.area(); });

        auto subtractRect =[](const cv::Rect& subject, const cv::Rect& clipper) -> std::vector<cv::Rect> 
        {
            std::vector<cv::Rect> result;
            cv::Rect intersect = subject & clipper;
            if (intersect.empty()) { result.push_back(subject); return result; }
            if (intersect == subject) return result; 

            if (subject.y < intersect.y) 
                result.emplace_back(subject.x, subject.y, subject.width, intersect.y - subject.y);
            if (subject.y + subject.height > intersect.y + intersect.height) {
                int new_y = intersect.y + intersect.height;
                result.emplace_back(subject.x, new_y, subject.width, (subject.y + subject.height) - new_y);
            }
            if (subject.x < intersect.x) 
                result.emplace_back(subject.x, intersect.y, intersect.x - subject.x, intersect.height);
            if (subject.x + subject.width > intersect.x + intersect.width) {
                int new_x = intersect.x + intersect.width;
                result.emplace_back(new_x, intersect.y, (subject.x + subject.width) - new_x, intersect.height);
            }
            return result;
        };

        for (const auto& current_candidate : expanded_candidates)
        {
            std::vector<cv::Rect> current_pieces = { current_candidate };
            for (const auto& existing_roi : final_rois)
            {
                std::vector<cv::Rect> next_pieces;
                for (const auto& piece : current_pieces)
                {
                    auto remainders = subtractRect(piece, existing_roi);
                    for(const auto& r : remainders) { if (r.area() > kMinRemainderArea) next_pieces.push_back(r); }
                }
                current_pieces = next_pieces;
                if (current_pieces.empty()) break;
            }
            for (const auto& piece : current_pieces) final_rois.push_back(piece);
        }
    }

    return final_rois;
}

std::vector<cv::Rect> GlintDetector::getROI()
{
    // 1. Cleanup
    init_pupil_seeds_.clear();
    final_pupils_.clear();
    
    cv::Rect full_img_rect(0, 0, gray_.cols, gray_.rows);

    // 2. Determine Glint Area (Tight Bound)
    glint_rect_ = shrinkRoiToValidGlints(full_img_rect);

    // 3. One-time Pupil Search
    if (!glint_rect_.empty())
    { 
        // Populate init_pupil_seeds_
        init_pupil_seeds_ = searchPupilInROI(glint_rect_);
        
        Logger::debug() << "[getROI] Found " << init_pupil_seeds_.size() << " pupil candidates in glint region.";
    }
    else
    {
        Logger::debug() << "[getROI] No valid glint region found, skipping pupil search.";
    }

    // 4. Search Reflections
    // searchGlassReflections usually does NOT depend on pupils (purely circular check)
    searchGlassReflections(); 

    // searchFrameReflections depends on isPupilNearby, which now uses the cached init_pupil_seeds_
    searchFrameReflections();

    // 5. Build Mask based on reflections
    buildExclusionMask();

    // 6. Determine final ROIs (uses glint_rect_ and init_pupil_seeds_)
    auto rois = determineCornealReflectionROI();

    return rois;
}

std::vector<std::vector<cv::Rect>> 
GlintDetector::clusterROIs(const std::vector<cv::Rect>& rois)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 判定规则：两个矩形如果扩充 margin 个像素后有交集，则视为同一类（相邻）
    const int kClusterMargin = 1;

    if (rois.empty()) return {};

    int n = static_cast<int>(rois.size());
    std::vector<int> labels;
    
    int n_classes = cv::partition(rois, labels, [kClusterMargin](const cv::Rect& a, const cv::Rect& b) {
        cv::Rect ra = a; ra.x -= kClusterMargin; ra.y -= kClusterMargin; ra.width += kClusterMargin*2; ra.height += kClusterMargin*2;
        cv::Rect rb = b; rb.x -= kClusterMargin; rb.y -= kClusterMargin; rb.width += kClusterMargin*2; rb.height += kClusterMargin*2;
        return (ra & rb).area() > 0;
    });

    std::vector<std::vector<cv::Rect>> clusters(n_classes);
    for (int i = 0; i < n; ++i) {
        clusters[labels[i]].push_back(rois[i]);
    }
    return clusters;
}

std::vector<cv::Point2f>
GlintDetector::searchGlintsInROI(
    const cv::Mat& roi_img, 
    const cv::Point2f& roi_offset,
    const std::vector<cv::Point2f>& glints,
    const double threshold_value,
    const std::string& debug_tag
)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::Mat threshold_output;
    cv::threshold(
        roi_img, 
        threshold_output, 
        threshold_value, 
        255, 
        cv::THRESH_BINARY
    );

    if (viz_ && debug_tag.size() > 0)
    {
        std::string threshold_output_folder = 
            cfg_["test_glint"]["input_folder"].as<std::string>() + "\\threshold_output\\" + img_name_ + "\\"
            + debug_tag;

        std::string save_path = threshold_output_folder + "\\" +  std::to_string(threshold_value) + ".png";

        std::filesystem::path folder_path(threshold_output_folder);
        if (!std::filesystem::exists(folder_path)) std::filesystem::create_directories(folder_path);

        cv::imwrite(save_path, threshold_output);
    }

    cv::findContours(
        threshold_output, 
        contours, 
        hierarchy, 
        cv::RETR_EXTERNAL, 
        cv::CHAIN_APPROX_SIMPLE, 
        roi_offset
    );

    // get the center and rect area in the original image
    std::vector<cv::RotatedRect> min_rects;
    std::vector<cv::Point2f> contour_centers;

    if (contours.empty()) return contour_centers;

    for (size_t k = 0; k < contours.size(); ++k)
    {
        cv::RotatedRect rect = cv::minAreaRect(contours[k]);
        if (isGlintRepeated(glints, rect.center)) continue;

        min_rects.push_back(rect);
        contour_centers.push_back(rect.center);
    }

    return contour_centers;
}

std::tuple<
    std::vector<std::vector<cv::Point2f>>, 
    std::vector<std::vector<cv::Point2f>>
>
GlintDetector::splitGlintsGeometry(std::vector<std::vector<cv::Point2f>> glint_geometry_list)
{
    std::vector<std::vector<cv::Point2f>> left_eye_geometries, right_eye_geometries;
    const double kDistanceThresholdX = 100.0;  // 将两眼斑点分为左右眼的阈值

    if (glint_geometry_list.empty()) {
        return { left_eye_geometries, right_eye_geometries };
    }

    // 1. 计算每个 geometry 的平均 X 坐标并与其原始数据绑定
    struct IndexedGeometry {
        double avg_x;
        std::vector<cv::Point2f> points;
    };
    
    std::vector<IndexedGeometry> sorted_geometries;
    for (const auto& geo : glint_geometry_list) {
        if (geo.empty()) continue;
        double sum_x = 0;
        for (const auto& pt : geo) sum_x += pt.x;
        sorted_geometries.push_back({ sum_x / geo.size(), geo });
    }

    // 2. 按照平均 X 坐标从小到大排序 (右眼 x 小，排在前面)
    std::sort(sorted_geometries.begin(), sorted_geometries.end(),
              [](const IndexedGeometry& a, const IndexedGeometry& b) {
                  return a.avg_x < b.avg_x;
              });

    double right_x_mean = sorted_geometries.front().avg_x;
    right_eye_geometries.push_back(sorted_geometries.front().points);

    for (size_t i = 1; i < sorted_geometries.size(); ++i)
    {
        const auto& current_geo = sorted_geometries[i];

        // 判断当前几何体的平均 X 是否靠近当前“右眼组”的平均 X
        if (std::abs(current_geo.avg_x - right_x_mean) < kDistanceThresholdX)
        {
            right_eye_geometries.push_back(current_geo.points);

            // 增量更新右眼组的 X 平均值中心
            const size_t n = right_eye_geometries.size();
            right_x_mean += (current_geo.avg_x - right_x_mean) / static_cast<double>(n);
        }
        else
        {
            // 距离较远，归入左眼
            left_eye_geometries.push_back(current_geo.points);
        }
    }

    return { left_eye_geometries, right_eye_geometries };
}

GlintDetector::Pupil 
GlintDetector::findBestPupilForCluster(const std::vector<cv::Rect>& cluster_rois)
{
    Pupil best_pupil;
    // 初始化为一个无效/空的瞳孔，避免后续使用出错
    best_pupil.rr = cv::RotatedRect(); 
    best_pupil.major_axis = -1.0f;
    best_pupil.minor_axis = -1.0f;
    best_pupil.darkness = 255.0;

    if (init_pupil_seeds_.empty() || cluster_rois.empty()) {
        return best_pupil;
    }

    bool found = false;

    // 遍历所有初始种子瞳孔 (init_pupil_seeds_ 是在 getROI 中填充的成员变量)
    for (const auto& pupil : init_pupil_seeds_)
    {
        // 检查该瞳孔是否属于当前 Cluster
        // 判定标准：瞳孔中心点落在了 Cluster 包含的任意一个 ROI 内
        bool belongs_to_cluster = false;
        for (const auto& roi : cluster_rois) {
            if (roi.contains(pupil.rr.center)) {
                belongs_to_cluster = true;
                break;
            }
        }

        if (belongs_to_cluster) {
            // 择优标准：长轴最长 (major_axis)
            if (pupil.major_axis > best_pupil.major_axis) {
                best_pupil = pupil;
                found = true;
            }
        }
    }
    
    if (found) {
        Logger::debug() << "[ClusterPupil] Selected pupil with major axis: " << best_pupil.major_axis;
    } else {
        Logger::debug() << "[ClusterPupil] No matching pupil found for this cluster.";
    }

    return best_pupil;
}

std::vector<GlintDetector::GlintGeometry> 
GlintDetector::selectBestGlintsPerCluster(const std::vector<GlintGeometry>& all_candidates)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 距离容差 (像素)，用于判断两个 Glint 距离瞳孔是否同样优秀
    const double kDistTolerance = 5.0; 
    // 亮度容差 (0-255)，用于在距离相当时比较背景暗度
    const double kBrightTolerance = 5.0; 

    if (all_candidates.empty()) return {};

    // 1. 按 Cluster ID 分组
    std::map<int, std::vector<GlintGeometry>> clusters;
    for (const auto& geo : all_candidates) {
        if (geo.cluster_id >= 0) {
            clusters[geo.cluster_id].push_back(geo);
        }
    }

    std::vector<GlintGeometry> final_results;

    // 2. 对每个 Cluster 分别筛选
    for (auto& [id, candidates] : clusters)
    {
        if (candidates.empty()) continue;

        // 定义多级比较器 Lambda
        // 返回 true 表示 a 比 b "更好"
        auto comparator = [kDistTolerance, kBrightTolerance](const GlintGeometry& a, const GlintGeometry& b) -> bool {
            
            // --- [Tier 0]: 瞳孔有效性检查 ---
            bool has_pupil_a = (a.linked_pupil.major_axis > 0);
            bool has_pupil_b = (b.linked_pupil.major_axis > 0);
            
            if (has_pupil_a != has_pupil_b) return has_pupil_a; // 优先选有瞳孔关联的
            if (!has_pupil_a) {
                // 如果都没有瞳孔，回退到只比亮度
                return a.bg_brightness < b.bg_brightness;
            }

            // --- [Tier 1]: 距离判据 ---
            double dist_a = cv::norm(a.center() - a.linked_pupil.rr.center);
            double dist_b = cv::norm(b.center() - b.linked_pupil.rr.center);
            
            double dist_diff = std::abs(dist_a - dist_b);

            // 如果距离差异显著，距离短者胜出
            if (dist_diff > kDistTolerance) {
                return dist_a < dist_b;
            }

            // ---[Tier 2]: 暗度判据 (当距离相近时) ---
            // 距离都在容差范围内，视为"位置一样好"，此时比背景纯净度
            double bri_diff = std::abs(a.bg_brightness - b.bg_brightness);

            // 如果暗度差异显著，暗度低者胜出 (角膜背景 < 皮肤背景)
            if (bri_diff > kBrightTolerance) {
                return a.bg_brightness < b.bg_brightness;
            }

            // --- [Tier 3]: 阈值判据 (当暗度也相近时) ---
            // 距离和暗度都差不多，比谁的信号更强
            // 阈值越高，说明 Glint 越亮越显著，不易受噪点干扰
            return a.found_threshold > b.found_threshold; 
        };

        // 3. 取最优
        auto best_it = std::min_element(candidates.begin(), candidates.end(), comparator);
        
        if (best_it != candidates.end()) {
            final_results.push_back(*best_it);
            
            Logger::debug() << "[SelectBest] Cluster " << id << " selected:"
                            << " Dist=" << cv::norm(best_it->center() - best_it->linked_pupil.rr.center)
                            << " Bri=" << best_it->bg_brightness
                            << " Thr=" << best_it->found_threshold;
        }
    }

    return final_results;
}

std::vector<GlintDetector::GlintGeometry> 
GlintDetector::detectCluster(
    int cluster_id, 
    const std::vector<cv::Rect>& cluster_rois,
    const Pupil& best_pupil // <--- 接收参数
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 判定新光斑是否属于已存在的几何光斑的最短距离阈值
    const double kMinUniqueGlintDist = 2.0;
    // 提前退出搜索的目标数量
    const size_t kMaxClusterResults = 3;

    std::vector<GlintGeometry> cluster_results;
    double thr = init_threshold_value_;

    while (thr >= threshold_step_)
    {
        std::vector<cv::Point2f> current_pass_points;
        for (const auto& roi : cluster_rois)
        {
            cv::Mat roi_img = abs_dst_(roi);
            cv::Point2f roi_offset(roi.x, roi.y);
            std::vector<cv::Point2f> temp_buffer;
            auto pts = searchGlintsInROI(roi_img, roi_offset, temp_buffer, thr, "");
            current_pass_points.insert(current_pass_points.end(), pts.begin(), pts.end());
        }

        auto geometries_found = findGeometry(current_pass_points);

        bool found_new_unique = false;
        for (auto& geo : geometries_found)
        {
            bool exists = false;
            for (const auto& existing : cluster_results) {
                if (cv::norm(geo.center() - existing.center()) < kMinUniqueGlintDist) { 
                    exists = true; break; 
                }
            }

            if (!exists) {
                // 记录元数据
                geo.cluster_id = cluster_id;
                geo.found_threshold = thr;
                
                // --- [关键修改]：绑定瞳孔 ---
                geo.linked_pupil = best_pupil; 
                // --------------------------

                cluster_results.push_back(geo);
                found_new_unique = true;
            }
        }

        if (cluster_results.size() >= kMaxClusterResults) break;
        thr -= threshold_step_;
    }

    return cluster_results;
}

std::tuple<
    std::vector<std::vector<cv::Point2f>>, 
    std::vector<std::vector<cv::Point2f>>
>
GlintDetector::detect(cv::Mat gray)
{
    Logger::ScopedTimer timer("[GlintDetector::detectFullImage]");
    debug_imgs_.clear();

    gray_ = gray.clone();

    // 1-3 Preprocessing
    cv::GaussianBlur(gray, gaussed_, cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 0, 0, cv::BORDER_DEFAULT);
    cv::Laplacian(gaussed_, laplaced_, CV_16S, laplacian_kernel_size_, laplacian_scale_, laplacian_delta_, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(laplaced_, abs_dst_);
    cv::threshold(abs_dst_, threshold_output_, cfg_["test_glint"]["debug_threshold_value"].as<double>(), 255, cv::THRESH_BINARY);

    // 4 Get ROI (Populates init_pupil_seeds_)
    auto rois = getROI();
    timer.lap("[4] getROI()");

    // 5.1 ROI 聚类
    auto clusters = clusterROIs(rois);
    
    std::vector<GlintGeometry> all_geometries;

    // 5.2 对每个区域独立处理
    for (int i = 0; i < clusters.size(); ++i)
    {
        // A. 先为这个 Cluster 找到最合适的 Pupil
        Pupil best_pupil = findBestPupilForCluster(clusters[i]);

        // B. 带着 Pupil 信息去检测 Glints
        auto cluster_results = detectCluster(i, clusters[i], best_pupil);
        
        all_geometries.insert(all_geometries.end(), cluster_results.begin(), cluster_results.end());

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << clusters[i][0].x << "_" << clusters[i][0].y;
        std::string pt_str = oss.str();
        timer.lap("[5] Cluster " + oss.str());
    }

    // 6. Select Best Glints Per Cluster
    auto best_geometries = selectBestGlintsPerCluster(all_geometries);
    timer.lap("[6] Select Best Glints Per Cluster");

    // viz
    if (viz_)
    {
        std::vector<GlintGeometry> viz_geometries;
        if (cfg_["collect_glint"]["is_collecting"].as<bool>())
        {
            viz_geometries = all_geometries;
        } else {
            viz_geometries = best_geometries;
        }

        for (const auto& geo : viz_geometries)
        {
            // 6. Debug 可视化 (更新：绘制被排除的区域)
            cv::Scalar color;
            color = geo.on_cornea ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            cv::line(debug_imgs_[3], geo.l_pt, geo.r_pt, color, 1, cv::LINE_AA);
            cv::line(debug_imgs_[3], geo.l_pt, geo.m_pt, color, 1, cv::LINE_AA);
            cv::line(debug_imgs_[3], geo.r_pt, geo.m_pt, color, 1, cv::LINE_AA);
            
            cv::putText(debug_imgs_[3], std::to_string((int)geo.bg_brightness), 
                        geo.center() + cv::Point2f(10, -10), cv::FONT_HERSHEY_PLAIN, 0.8, color, 1);
        }
    }

    // 7. Split
    std::vector<std::vector<cv::Point2f>> glint_geometry_list;
    if (cfg_["collect_glint"]["is_collecting"].as<bool>())
    {
        glint_geometry_list = glintGeometryListToGlintVectors(all_geometries);
    } else {
        glint_geometry_list = glintGeometryListToGlintVectors(best_geometries);
    }
    auto [left_glint_geometries, right_glint_geometries] = splitGlintsGeometry(glint_geometry_list);
    timer.lap("[7] Split Glints");

    Logger::debug() << "[GlintDetector::detectFullImage] Total Glints: " << all_geometries.size();
    for (const auto& geo : all_geometries)
    {
        Logger::debug() << std::fixed << std::setprecision(2) << "\t" 
                        << "(" << geo.l_pt.x << ", " << geo.l_pt.y << ") "
                        << "(" << geo.r_pt.x << ", " << geo.r_pt.y << ") "
                        << "(" << geo.m_pt.x << ", " << geo.m_pt.y << ") ";
    }

    return { left_glint_geometries, right_glint_geometries };
}

std::vector<GlintDetector::GlintGeometry>
GlintDetector::findGeometry(std::vector<cv::Point2f> glint_candidates)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 寻找缺失中点或侧边点的最大数量限制
    const int kMaxMissingPointsToFind = 2;

    glint_geometry_list_.clear();

    Logger::debug() << "\n";

	// Go throuth each Point2f in list
	for (int i = 0; i < glint_candidates.size(); i++)
	{
		cv::Point2f temp_pt_1 = glint_candidates[i];
		for (int j = i + 1; j < glint_candidates.size(); j++)
		{
			cv::Point2f temp_pt_2 = glint_candidates[j];
			cv::Point2f l_pt = temp_pt_1.x < temp_pt_2.x ? temp_pt_1 : temp_pt_2;
			cv::Point2f r_pt = temp_pt_1.x > temp_pt_2.x ? temp_pt_1 : temp_pt_2;

			// 1 Find Horizontal Pair
			if (side2side(l_pt, r_pt)) // original 15 5 5 0
			{
                Logger::debug() << "[5 Find Geometry] 1 found horizontal pair" << "\n"
                                << "\tleft: (" << l_pt.x << ", " << l_pt.y << ")\n"
                                << "\tright: (" << r_pt.x << ", " << r_pt.y << ")";

                int init_num_glints = glint_geometry_list_.size();

				for (int k = 0; k < glint_candidates.size(); k++)
				{
					if (k == i || k == j) continue;

					cv::Point2f m_pt = glint_candidates[k];
                    Logger::debug() << "[5 Find Geometry] 1 Checking potential mid down at: \n\t("
                                    << m_pt.x << ", " << m_pt.y << ")";

					// 2
					// find mid point
					if (side2mid(l_pt, r_pt, m_pt))
					{
                        Logger::debug() << "[5 Find Geometry] 2 found mid point" << "\n"
                                        << "\tmid: (" << m_pt.x << ", " << m_pt.y << ")";
                        Logger::debug() << "[5 Find Geometry] Found glint geometry: \n\t" 
                                        << "(" << l_pt.x << ", " << l_pt.y << ") " 
                                        << "(" << r_pt.x << ", " << r_pt.y << ") "
                                        << "(" << m_pt.x << ", " << m_pt.y << ")";

						checkAndPushGlintGeometry(l_pt, r_pt, m_pt);
					}
				}

                if (glint_geometry_list_.size() == init_num_glints)
                {
                    Logger::debug() << "[5 Find Geometry] 1-1 not found mid point, try to find missing mid point for: ";
                    Logger::debug() << "\tleft: (" << l_pt.x << ", " << l_pt.y << ")\n"
                                    << "\tright: (" << r_pt.x << ", " << r_pt.y << ")";

                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(1) << l_pt.x << "_" << l_pt.y
                        << "_" << r_pt.x << "_" << r_pt.y;
                    std::string pt_str = oss.str();

                    auto [roi_img, roi_offset] = getSearchRegionSideAndSide(l_pt, r_pt);
                    double thr = init_threshold_value_;
                    std::vector<cv::Point2f> roi_glints;
                    int count = 0;

                    if (roi_img.empty()) continue;

                    do {
                        auto contourCenters = searchGlintsInROI(roi_img, roi_offset, roi_glints, thr, pt_str);

                        for (int i = 0; i < contourCenters.size(); i++)
                        {
                            roi_glints.push_back(contourCenters[i]);

                            Logger::debug() << "[5 Find Geometry] 1-2 Checking potential missing mid point at: \n\t("
                                        << contourCenters[i].x << ", " << contourCenters[i].y << ")";

                            if (side2mid(l_pt, r_pt, contourCenters[i]))
                            {
                                Logger::debug() << "[5 Find Geometry] 1-2 found missing mid point";
                                Logger::debug() << "[5 Find Geometry] Found glint geometry: \n\t" 
                                                << "(" << l_pt.x << ", " << l_pt.y << ") " 
                                                << "(" << r_pt.x << ", " << r_pt.y << ") "
                                                << "(" << contourCenters[i].x << ", " << contourCenters[i].y << ")";

                                checkAndPushGlintGeometry(l_pt, r_pt, contourCenters[i]);
                                count++;
                            }
                            else
                            {
                                Logger::debug() << "[5 Find Geometry] 1-2 not found missing mid point";
                            }
                        }

                        thr -= threshold_step_;
                    } while (thr >= threshold_step_ && count < kMaxMissingPointsToFind);
                }
			}
		}
	}

    Logger::debug() << "[5 Find Geometry] 2-1 Try to find side and mid pair";

    for (int i = 0; i < glint_candidates.size(); i++)
    {
        cv::Point2f temp_pt_1 = glint_candidates[i];
        for (int j = i + 1; j < glint_candidates.size(); j++)
        {
            cv::Point2f temp_pt_2 = glint_candidates[j];
            cv::Point2f s_pt = temp_pt_1.y < temp_pt_2.y ? temp_pt_1 : temp_pt_2;
            cv::Point2f m_pt = temp_pt_1.y > temp_pt_2.y ? temp_pt_1 : temp_pt_2;

            if (isGlintGeometryRepeated(s_pt, m_pt)) continue;

            // 2-1
            // find side and mid pair
            if (side2mid(s_pt, m_pt))
            {
                Logger::debug() << "[5 Find Geometry] 2-1 found side and mid pair" << "\n"
                                << "side: (" << s_pt.x << ", " << s_pt.y << ")\n"
                                << "mid: (" << m_pt.x << ", " << m_pt.y << ")";

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1) << s_pt.x << "_" << s_pt.y
                    << "_" << m_pt.x << "_" << m_pt.y;
                std::string pt_str = oss.str();

                auto [roi_img, roi_offset] = getSearchRegionSideAndMid(s_pt, m_pt);
                double thr = init_threshold_value_;
                std::vector<cv::Point2f> roi_glints;
                int count = 0;

                if (roi_img.empty()) continue;

                do {
                    auto contourCenters = searchGlintsInROI(roi_img, roi_offset, roi_glints, thr, pt_str);

                    for (int i = 0; i < contourCenters.size(); i++)
                    {
                        roi_glints.push_back(contourCenters[i]);

                        cv::Point2f l_pt = contourCenters[i].x < s_pt.x ? contourCenters[i] : s_pt;
                        cv::Point2f r_pt = contourCenters[i].x > s_pt.x ? contourCenters[i] : s_pt;

                        Logger::debug() << "[5 Find Geometry] 2-2 Checking potential missing side point at: \n\t("
                                    << contourCenters[i].x << ", " << contourCenters[i].y << ")";

                        if (side2mid(l_pt, r_pt, m_pt))
                        {
                            Logger::debug() << "[5 Find Geometry] 2-2 found missing side point";
                            Logger::debug() << "[5 Find Geometry] Found glint geometry: \n\t" 
                                            << "(" << l_pt.x << ", " << l_pt.y << ") " 
                                            << "(" << r_pt.x << ", " << r_pt.y << ") "
                                            << "(" << m_pt.x << ", " << m_pt.y << ")";

                            checkAndPushGlintGeometry(l_pt, r_pt, m_pt);
                            count++;
                        }
                        else
                        {
                            Logger::debug() << "[5 Find Geometry] 2-2 not found missing side point";
                        }
                    }

                    thr -= threshold_step_;
                } while (thr >= threshold_step_ && count < kMaxMissingPointsToFind);
            }
        }
    }

    Logger::debug() << "[5 Find Geometry] Result: ";
    auto glint_geometries = glintGeometryListToGlintVectors(glint_geometry_list_);
    for (const auto& glint_geometry : glint_geometries)
    {
        std::ostringstream oss;
        for (const auto& pt : glint_geometry)
        {
            oss << "(" << pt.x << ", " << pt.y << ") ";
        }
        Logger::debug() << "\t" << oss.str();
    }

	return glint_geometry_list_;
}

bool GlintDetector::isGlintRepeated(
    const std::vector<cv::Point2f>& roi_glints, 
    const cv::Point2f& glint
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 判定两个 glint 是否为同一个的最短欧氏距离阈值
    const double kMinGlintDist = 1.0;

    if (roi_glints.empty()) return false;

    for (const auto& roi_glint : roi_glints)
    {
        if (roi_glint.x == glint.x && roi_glint.y == glint.y)
        {
            return true;
        }
        // if distance between glints is less than kMinGlintDist, consider them as the same glint
        if (cv::norm(roi_glint - glint) < kMinGlintDist)
        {
            return true;
        }
    }

    return false;
}

bool GlintDetector::isGlintGeometryRepeated(
    const cv::Point2f& s_pt, 
    const cv::Point2f& m_pt
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 判定两个 Glint 几何点位置相同的容差距离
    const double kPointMatchTolerance = 0.5;

    if (glint_geometry_list_.empty()) return false;

    for (const auto& geo : glint_geometry_list_)
    {
        // 检查 m_pt 是否相同
        if (std::abs(geo.m_pt.x - m_pt.x) < kPointMatchTolerance && std::abs(geo.m_pt.y - m_pt.y) < kPointMatchTolerance) {
            // 检查 s_pt 是否是 l 或者 r
            if ((std::abs(geo.l_pt.x - s_pt.x) < kPointMatchTolerance && std::abs(geo.l_pt.y - s_pt.y) < kPointMatchTolerance) ||
                (std::abs(geo.r_pt.x - s_pt.x) < kPointMatchTolerance && std::abs(geo.r_pt.y - s_pt.y) < kPointMatchTolerance)) {
                return true;
            }
        }
    }
    return false;
}

void GlintDetector::checkAndPushGlintGeometry(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const cv::Point2f& m_pt
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 亮斑的背景亮度阈值
    const double brightness_threshold = 25.0;
    // ROI扩展的最小像素数和按比例扩展的系数
    const int kMinPadding = 5;
    const float kPaddingRatio = 0.20f;
    // 寻找高亮斑(污染区)的二值化阈值
    const double kDangerMaskThresh = 50.0;
    // 膨胀核尺寸与迭代次数(建立隔离带)
    const int kDilateKernelSize = 3;
    const int kDilateIterations = 1;
    // 进行截断平均所需的最少有效像素数
    const size_t kMinValidPixels = 5;
    // 截断平均的剪裁比例(上下各剔除的百分比)
    const float kTrimRatio = 0.10f;

    // 构造对象
    GlintGeometry geo;
    geo.l_pt = l_pt;
    geo.r_pt = r_pt;
    geo.m_pt = m_pt;
    
    std::vector<cv::Point2f> pts = {l_pt, r_pt, m_pt};

    // =========================================================
    // 第一步：获取 Bounding Rect 并按比例扩展 (保持不变)
    // =========================================================
    cv::Rect roi = cv::boundingRect(pts);
    int pad_x = std::max(kMinPadding, static_cast<int>(roi.width * kPaddingRatio));
    int pad_y = std::max(kMinPadding, static_cast<int>(roi.height * kPaddingRatio));

    roi.x -= pad_x;
    roi.y -= pad_y;
    roi.width += (pad_x * 2);
    roi.height += (pad_y * 2);

    roi &= cv::Rect(0, 0, gray_.cols, gray_.rows);
    if (roi.area() <= 0) return;

    // =========================================================
    // 第二步：构建“污染区”掩膜 (High Brightness Mask)
    // =========================================================
    cv::Mat roi_img = gray_(roi);
    cv::Mat danger_mask;

    // 1. 初始标记：找出所有大于阈值的亮像素
    // thresh_binary: src > thresh ? 255 : 0
    // 结果：亮斑区域为白色(255)，背景为黑色(0)
    cv::threshold(roi_img, danger_mask, kDangerMaskThresh, 255, cv::THRESH_BINARY);

    // =========================================================
    // 第三步：建立“隔离带” (Dilation)
    // =========================================================
    // 2. 膨胀：将亮斑区域向外扩展一圈
    // 指定核大小，膨胀迭代次数，意味着向四周扩充像素
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kDilateKernelSize, kDilateKernelSize));
    cv::dilate(danger_mask, danger_mask, kernel, cv::Point(-1, -1), kDilateIterations);

    // =========================================================
    // 第四步：安全采样
    // =========================================================
    std::vector<uchar> valid_pixels;
    valid_pixels.reserve(roi.area());

    for (int y = 0; y < roi_img.rows; ++y) {
        const uchar* img_ptr = roi_img.ptr<uchar>(y);
        const uchar* mask_ptr = danger_mask.ptr<uchar>(y);
        for (int x = 0; x < roi_img.cols; ++x) {
            // 只有 Mask 为 0 (黑色) 的地方才是绝对安全的背景
            // 既不是亮斑本身，也不是亮斑紧邻的一圈
            if (mask_ptr[x] == 0) {
                valid_pixels.push_back(img_ptr[x]);
            }
        }
    }

    // =========================================================
    // 第五步：统计 (截断平均值) (保持不变)
    // =========================================================
    if (valid_pixels.empty()) {
        geo.bg_brightness = 255.0;
        geo.on_cornea = false;
    } 
    else if (valid_pixels.size() < kMinValidPixels) {
        double sum = 0;
        for (auto v : valid_pixels) sum += v;
        geo.bg_brightness = sum / valid_pixels.size();
        geo.on_cornea = (geo.bg_brightness <= brightness_threshold);
    }
    else {
        std::sort(valid_pixels.begin(), valid_pixels.end());
        size_t n = valid_pixels.size();
        size_t trim_count = static_cast<size_t>(n * kTrimRatio);
        if (trim_count * 2 >= n) trim_count = 0; 

        double sum = 0.0;
        for (size_t k = trim_count; k < n - trim_count; ++k) {
            sum += valid_pixels[k];
        }
        geo.bg_brightness = sum / static_cast<double>(n - trim_count * 2);
        geo.on_cornea = (geo.bg_brightness <= brightness_threshold);
    }

    if (geo.on_cornea) glint_geometry_list_.push_back(geo);
}

std::vector<std::vector<cv::Point2f>>
GlintDetector::glintGeometryListToGlintVectors(const std::vector<GlintDetector::GlintGeometry>& glint_geometry)
{
    std::vector<std::vector<cv::Point2f>> glint_vectors;
    for (const auto& geo : glint_geometry) {
        std::vector<cv::Point2f> glint_vector = {geo.l_pt, geo.r_pt, geo.m_pt};
        glint_vectors.push_back(glint_vector);
    }
    return glint_vectors;
}

} // namespace glintdetection