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
            ✅ 2.2.1 find possible side and mid pair
            ✅ 2.2.2 search for the last side
    ✅ 2.2 side and side: serach for the mid
4. unify glass and non-glass logic
    ✅ 4.1 determine ROI
        ✅ 4.1.1 remove noisy points
        ✅ 4.1.2 expand the ROI
    4.2 find pupil center
        ⚠️ 4.2.1 remove upper eyelid
    4.3 find best glint pair
        4.3.1 within cluster
        4.3.2 between clusters
    ✅ 4.4 automate the calculation of hyperparameters
✅ 5. repair the cfg
    5.1 type transform
    5.2 error handling
6. update the repeat logic in findGeometry()
*/

/*
================================================================================[Hyperparameters List for detect_glint.cpp]
维护说明：以下为在各个函数内部定义的局部硬编码超参数（Hardcoded Hyperparameters），
修改此类参数通常会影响该函数特定步骤的检测灵敏度与过滤严格度。
================================================================================

❌[searchGlassReflections] - 镜片反光检测
- kMinContourArea               = 50.0;  // 反光轮廓面积最小值
- kMaxContourArea               = 500.0; // 反光轮廓面积最大值
- kRoiPadding                   = 10;    // 提取霍夫圆检测 ROI 时的边界扩展量 (像素)
- kHoughDp                      = 1.0;   // 霍夫圆累加器分辨率
- kHoughMinDistDivisor          = 3.0;   // 霍夫圆最小圆心距的除数 (基于ROI高度)
- kHoughParam1                  = 100.0; // 霍夫圆 Canny边缘检测高阈值
- kHoughParam2                  = 5.0;   // 霍夫圆 累加器阈值(投票数)
- kHoughMinRadius               = 3;     // 霍夫圆 最小检测半径
- kHoughMaxRadius               = 25;    // 霍夫圆 最大检测半径
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
⚠️- kAdaptiveThreshOffset         = 10.0;  // 自适应阈值计算偏移量 [计算方法：亮度统计；需要存储：瞳孔的亮度]
⚠️- kAdaptiveThreshMax            = 15.0;  // 自适应阈值最大限制   [计算方法：亮度统计；需要存储：瞳孔的亮度]
- kMorphKernelSize              = 5;     // 形态学操作核大小
- kMorphCloseIterations         = 2;     // 形态学闭操作迭代次数
⚠️- kMinPupilArea                 = 50.0;  // 瞳孔轮廓过滤的面积下限 [计算方法：面积统计；需要存储：瞳孔的面积]
⚠️- kMaxPupilArea                 = 800.0; // 瞳孔轮廓过滤的面积上限 [计算方法：面积统计；需要存储：瞳孔的面积]
⚠️- kMinPupilContourPoints        = 5;     // 瞳孔轮廓最小点数 [计算方法：轮廓点数统计；需要存储：瞳孔的轮廓点数]
⚠️- kMaxPupilAxis                 = 50.0f; // 瞳孔长轴最大限制 [计算方法：长轴统计；需要存储：瞳孔的长轴]
⚠️- kMaxAxisRatio                 = 2.0f;  // 瞳孔长宽比最大限制 [计算方法：长宽比统计；需要存储：瞳孔的长宽比]
⚠️- kMinSolidity                  = 0.60;  // 凸包面积比 (Solidity) 最小值 [计算方法：凸包面积比统计；需要存储：瞳孔的凸包面积比]
⚠️- kMinFitRatio                  = 0.65;  // 椭圆拟合面积比最小值 [计算方法：椭圆拟合面积比；需要存储：瞳孔的椭圆拟合面积比]
- kValidResidualRatio           = 0.60f; // 残差计算有效点比例
⚠️- kMaxAvgResidual               = 0.40f; // 平均残差最大值 [计算方法：残差计算；需要存储：瞳孔的残差]
⚠️- kMaxDarkness               = 15.0;  // 暗度上限 [计算方法：亮度统计；需要存储：瞳孔的亮度]

[isPupilNearby] - 瞳孔附近判定
⚠️- kExclusionRadiusRatio         = 2.5f;  // 距离阈值系数 (相对于瞳孔长轴) [计算方法：距离计算；需要存储：glints到瞳孔的距离]

❌[findNeighborInDirection] - 方向性寻找相邻反光点
- kDefaultSearchScaleLenRatio   = 3.0f;  // 默认搜索区域扩展系数 (相对当前节点的长度)
- kDefaultSearchScaleWidRatio   = 2.0f;  // 默认搜索区域扩展系数 (相对当前节点的宽度)
- kMinContourArea               = 2.0;   // 过滤噪点的最小面积
- kMinDistRatio                 = 0.5f;  // 节点间最小距离系数 (避免检测到自身)
- kMinReliableLenRatio          = 0.6f;  // 新节点可靠尺度继承的最小长度比例
- kAngleSwapMin                 = 45.0f; // 需要交换长宽的角度范围下限
- kAngleSwapMax                 = 135.0f;// 需要交换长宽的角度范围上限

❌[searchReflectionChains] - 搜索反光链
- kMinNormEpsilon               = 0.1f;  // 向量更新的最小模长，防跳变
- kMaxChainLength               = 20;    // 反光链的最大节点数

❌[searchFrameReflections] - 搜索镜框反光
- kMinArea                      = 10.0;  // 初始镜框反光种子的面积下限
- kMaxArea                      = 50.0;  // 初始镜框反光种子的面积上限
- kMinLengthWidthRatio          = 2.0f;  // 初始镜框反光种子的长宽比最小限制 (长条状)
- kSearchScaleLenRatio          = 3.0f;  // 搜索区域长向缩放系数
- kSearchScaleWidRatio          = 2.0f;  // 搜索区域宽向缩放系数
- kAngleSwapMin                 = 45.0f; // 需要交换搜索区域长宽的角度范围下限
- kAngleSwapMax                 = 135.0f;// 需要交换搜索区域长宽的角度范围上限
- kMinChainSizeToDraw           = 3;     // 绘制链条时要求的最短链长度

❌[buildExclusionMask] - 构建排除掩膜
- kGlassExclusionScale          = 1.2f;  // 镜片反光排除区域的放大倍数
- kFrameExclusionScale          = 1.5f;  // 单个镜框反光排除区域的放大倍数
- kChainExclusionScale          = 1.5f;  // 镜框反光链之间连线排除区域的放大倍数

❌[shrinkRoiToValidGlints] - 缩小至有效光斑ROI
- kMaxNoiseArea                 = 5.0;   // 噪点最大像素数/面积
- kClusterDistThresh            = 80.0;  // 聚集距离阈值
- kMinClusterSize               = 2;     // 有效聚集的最小数量
- padding_y                     = 60;    // 扩展边界 (上下方向)
- kDefaultLowSensitivityThresh  = 25.0;  // 默认极低敏感度阈值
- kPaddingXRatio                = 0.2f;  // 缩小后 ROI 的宽向外扩展比例

[excludeIrisReflectionsFromROIs] - 排除虹膜反射
- kIrisReflectionThreshold      = 100.0; // 虹膜反射二值化阈值
- kMinIrisReflectionArea        = 50.0;  // 判定为虹膜反射的最小连通域面积
- kPadding                      = 2;     // 虹膜反射区域扩展边界

[determineCornealReflectionROI] - 确定角膜反光ROI
⚠️- kConstraintRadiusRatio        = 2.5f;  // 瞳孔种子约束区域半径放大系数 (基于长轴)
- kMinExpandedRoiArea           = 50;    // 候选区域最小有效面积
- kShrinkRatio                  = 0.05f; // 边界收缩比例 (去除极边缘部分)
- kMinRemainderArea             = 10;    // 差集操作后保留的最小碎片面积

[splitGlintsGeometry] - 左右眼光斑分离
- kDistanceThresholdX           = 100.0; // 将两眼斑点分为左右眼的X轴距离阈值

[selectBestGlintsPerCluster] - 选择每组最优光斑
- kDistTolerance                = 5.0;   // 距离容差 (像素)
- kBrightTolerance              = 5.0;   // 亮度容差 (0-255)

[detectCluster] - 聚类检测
- kMinUniqueGlintDist           = 2.0;   // 判断新光斑是否属于已有光斑的最短距离阈值
- kMaxClusterResults            = 3;     // 提前退出搜索的目标数量

[findGeometry] - 寻找几何结构
- kMaxMissingPointsToFind       = 2;     // 寻找缺失中点或侧边点的最大数量限制

[isGlintRepeated] - 光斑重复判定
- kMinGlintDist                 = 1.0;   // 判定两个 glint 是否为同一个的最短欧氏距离阈值

[isGlintGeometryRepeated] - 几何体重复判定
- kPointMatchTolerance          = 0.5;   // 判定两个 Glint 几何点位置相同的容差距离

[checkAndPushGlintGeometry] - 校验并推入光斑结构
⚠️- kBrightnessThreshold          = 25.0;  // 亮斑的背景亮度判定阈值 [计算方法：亮度统计；需要存储：glint的背景亮度]
- kMinPadding                   = 5;     // ROI扩展的最小像素数
- kPaddingRatio                 = 0.20f; // 按比例扩展的系数
⚠️- kDangerMaskThresh             = 50.0;  // 寻找高亮斑(污染区)的二值化阈值
- kDilateKernelSize             = 3;     // 膨胀核尺寸(建立隔离带)
- kDilateIterations             = 1;     // 膨胀迭代次数
- kMinValidPixels               = 5;     // 进行截断平均所需的最少有效像素数
- kTrimRatio                    = 0.10f; // 截断平均的剪裁比例(上下各剔除的百分比)

[refinePupil] - 基于射线的精细化瞳孔拟合
- kRayCount                   = 72;     // 射线发射总数 (360度划分)
- kRayMinRadiusRatio          = 0.2f;   // 射线起始采样半径比例 (相对于粗长轴)
- kRayMaxRadiusRatio          = 1.2f;   // 射线最大采样半径比例 (相对于粗长轴)
- kGradientGap                = 3;      // 梯度计算的间隔跨度 (解决8vs11低对比度)
- kMinGradient                = 1.0f;   // 判定为边缘的最小微弱梯度阈值
- kGlintExclusionRadius       = 15.0f;  // 光斑排异的物理半径 (像素)
- kMaxDistanceDeviationRatio  = 0.2f;   // 统计排异：允许偏离中值半径的最大比例
- kRansacIterations           = 100;    // RANSAC 最大迭代次数
- kRansacTolerance            = 2.0f;   // RANSAC 内点判定的距离容差 (像素)
- kMinInlierRatio             = 0.4f;   // RANSAC 成功的最小内点比例

================================================================================
*/

namespace glintdetection {

GlintDetector::GlintDetector(const std::string& param_type)
    : param_type_(param_type) 
{
    std::string spec_key;
    if (param_type_ == "default") {
        spec_key = "recommended_specific_hyperparameter";
    } else if (param_type_ == "relaxed") {
        spec_key = "relaxed_specific_hyperparameter";
    }

    spec_pupil_cfg_ = cfg_[spec_key]["pupil"];
    spec_glint_cfg_ = cfg_[spec_key]["glint"];
    emp_cfg_ = cfg_["empirical_hyperparameter"];

    horizontal_pair_cfg_ = spec_glint_cfg_["horizontal_pair"];
    middle_point_cfg_ = spec_glint_cfg_["middle_point"];

    gaussian_kernel_size_ = cfg_["test_glint"]["gaussian_kernel_size"].as<int>();
	laplacian_kernel_size_ = cfg_["test_glint"]["laplacian_kernel_size"].as<int>();
	init_threshold_value_ = cfg_["test_glint"]["init_threshold_value"].as<double>();
    threshold_step_ = cfg_["test_glint"]["threshold_step"].as<double>();
    mini_threshold_ = cfg_["test_glint"]["mini_threshold"].as<double>();
    use_glint_sr_ = cfg_["test_glint"]["use_glint_sr"].as<bool>();
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

std::vector<cv::Rect> GlintDetector::getSearchRegionSideAndMid(
    const cv::Point2f& s_pt, 
    const cv::Point2f& m_pt,
    const RoiCluster& cluster // 传入统一数据结构，返回切分后的有效区域
)
{
    const int kOffsetX = emp_cfg_["getSearchRegionSideAndMid"]["kOffsetX"].as<int>();
    const int kOffsetY = emp_cfg_["getSearchRegionSideAndMid"]["kOffsetY"].as<int>();

    int x_min, x_max, y_min, y_max;

    // 1 load hyperparameters
    double lr_x_min = horizontal_pair_cfg_["lr_x_min"].as<double>();
    double lr_x_max = horizontal_pair_cfg_["lr_x_max"].as<double>();
    double lr_y_max = horizontal_pair_cfg_["lr_y_max"].as<double>();

    // 2 calculate the search ROI
    if (s_pt.x < m_pt.x) {
        // left and mid
        x_min = cvCeil (m_pt.x + 1 + kOffsetX);
        x_max = cvCeil (s_pt.x + lr_x_max);
        y_min = cvFloor(s_pt.y - lr_y_max);
        y_max = cvFloor(m_pt.y + kOffsetY);
    } else {
        // right and mid
        x_min = cvFloor(s_pt.x - lr_x_max);
        x_max = cvFloor(m_pt.x - 1 - kOffsetX);
        y_min = cvFloor(s_pt.y - lr_y_max);
        y_max = cvFloor(m_pt.y + kOffsetY);
    }

    cv::Rect roi(x_min, y_min, x_max - x_min, y_max - y_min);

    roi &= cv::Rect(0, 0, abs_dst_.cols, abs_dst_.rows);
    if (!cluster.limit_bound.empty()) {
        roi &= cluster.limit_bound; // 受限于本聚类的 limit_bound
    }

    if (roi.empty()) return {};

    Logger::debug() << "[5 Find Geometry] [getSearchRegionSideAndMid]";
    Logger::debug() << "\tsearch_region: (" << roi.x << ", " << roi.y << ") - (" 
                    << roi.x + roi.width << ", " << roi.y + roi.height << ")";

    // --- 扣除虹膜反光区域 ---
    const int kMinRemainderArea = emp_cfg_["getSearchRegionSideAndMid"]["kMinRemainderArea"].as<int>();
    std::vector<cv::Rect> current_pieces = { roi };

    for (const auto& bbox : cluster.iris_exclusions) {
        std::vector<cv::Rect> next_pieces;
        for (const auto& piece : current_pieces) {
            auto remainders = subtractRect(piece, bbox);
            for (const auto& r : remainders) {
                if (r.area() > kMinRemainderArea) next_pieces.push_back(r);
            }
        }
        current_pieces = next_pieces;
        if (current_pieces.empty()) break;
    }

    return current_pieces; // 返回完美避开虹膜和边界的干净搜索切片
}

std::vector<cv::Rect> GlintDetector::getSearchRegionSideAndSide(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const RoiCluster& cluster // 传入统一数据结构，返回切分后的有效区域
)
{
    int kOffsetY = emp_cfg_["getSearchRegionSideAndSide"]["kOffsetY"].as<int>();

    double lr_x = std::abs(l_pt.x - r_pt.x);
    double offset_y = lr_x * middle_point_cfg_["conditions"]
                      .as<std::vector<std::vector<double>>>().front()[5];
    double side_y_max = std::max(l_pt.y, r_pt.y);

    int x_min = cvCeil(l_pt.x);
    int x_max = cvFloor(r_pt.x);
    int y_min = cvCeil(side_y_max + kOffsetY);
    int y_max = cvFloor(side_y_max + offset_y + kOffsetY);

    cv::Rect roi(x_min, y_min, x_max - x_min, y_max - y_min);

    roi &= cv::Rect(0, 0, abs_dst_.cols, abs_dst_.rows);
    if (!cluster.limit_bound.empty()) {
        roi &= cluster.limit_bound; // 受限于本聚类的 limit_bound
    }

    if (roi.empty()) return {};

    Logger::debug() << "[5 Find Geometry] [getSearchRegionSideAndSide]";
    Logger::debug() << "\tsearch_region: (" << roi.x << ", " << roi.y << ") - (" 
                    << roi.x + roi.width << ", " << roi.y + roi.height << ")";

    // --- 扣除虹膜反光区域 ---
    const int kMinRemainderArea = emp_cfg_["getSearchRegionSideAndMid"]["kMinRemainderArea"].as<int>();
    std::vector<cv::Rect> current_pieces = { roi };

    for (const auto& bbox : cluster.iris_exclusions) {
        std::vector<cv::Rect> next_pieces;
        for (const auto& piece : current_pieces) {
            auto remainders = subtractRect(piece, bbox);
            for (const auto& r : remainders) {
                if (r.area() > kMinRemainderArea) next_pieces.push_back(r);
            }
        }
        current_pieces = next_pieces;
        if (current_pieces.empty()) break;
    }

    return current_pieces; // 返回完美避开虹膜和边界的干净搜索切片
}

void GlintDetector::searchGlassReflections()
{
    CfgNode emp = emp_cfg_["searchGlassReflections"];

    // --- 超参数声明 (Hyperparameters) ---
    // 反光轮廓面积的最小和最大值，过滤过小噪点或过大反光
    const double kMinContourArea = emp["kMinContourArea"].as<double>();
    const double kMaxContourArea = emp["kMaxContourArea"].as<double>();

    // 提取霍夫圆检测 ROI 时的边界扩展量 (像素)
    const int kRoiPadding = emp["kRoiPadding"].as<int>();
    // 霍夫圆变换参数
    const double kHoughDp = emp["kHoughDp"].as<double>();                           // 累加器分辨率
    const double kHoughMinDistDivisor = emp["kHoughMinDistDivisor"].as<double>();   // 最小圆心距的除数 (基于ROI高度)
    const double kHoughParam1 = emp["kHoughParam1"].as<double>();                   // Canny边缘检测高阈值
    const double kHoughParam2 = emp["kHoughParam2"].as<double>();                   // 累加器阈值(投票数)，决定检测灵敏度
    const int kHoughMinRadius = emp["kHoughMinRadius"].as<int>();                   // 最小检测半径
    const int kHoughMaxRadius = emp["kHoughMaxRadius"].as<int>();                   // 最大检测半径

    // 距离变换参数
    const int kDistTransformMaskSize = emp["kDistTransformMaskSize"].as<int>();         // 距离变换掩膜尺寸
    const float kMinDtRadius = emp["kMinDtRadius"].as<float>();                         // 极细碎噪点过滤的最小距离变换峰值
    const float kDtRadiusCentroidThresh = emp["kDtRadiusCentroidThresh"].as<float>();   // 使用质心替代峰值的DT半径阈值

    // 镜框与镜片反光判定比值阈值 (霍夫半径 / DT峰值)
    const float kFrameReflectionRatioThresh = emp["kFrameReflectionRatioThresh"].as<float>();
    const float kMagEpsilon = emp["kMagEpsilon"].as<float>();       // 向量模长极小值保护

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
        debug_imgs_[1] = viz; // glass reflection viz
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
                    cv::line(debug_imgs_[1], fr.points[i], fr.points[(i+1)%4], cv::Scalar(255, 0, 255), 2);
                }
            }
        }

        if (viz_)
        {
            // --- Visualization Logic ---
            // 青色细线：原始的、混乱的轮廓
            cv::drawContours(debug_imgs_[1], std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 255, 0), 1);
            
            // 红色细圆：霍夫变换“投票”出的最可能的完整圆
            cv::circle(debug_imgs_[1], global_hc_center, static_cast<int>(hc_radius), cv::Scalar(0, 0, 255), 1);
            
            // 红色中心点
            cv::circle(debug_imgs_[1], global_hc_center, 2, cv::Scalar(0, 0, 255), -1);
            
            // 画出 ROI 框方便调试观察
            cv::rectangle(debug_imgs_[1], roiRect, cv::Scalar(255, 0, 0), 1);

            // 蓝色空心圆：算法检测到的镜片反光主体 (Inscribed Circle)
            // 这就是去除了镜框干扰后的结果
            cv::circle(debug_imgs_[1], dt_center, static_cast<int>(dt_radius), cv::Scalar(255, 0, 0), 1);
            
            // 红色中心点
            cv::circle(debug_imgs_[1], dt_center, 1, cv::Scalar(255, 0, 0), -1);

            // 标出比值，以便对比
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << ratio;
            cv::putText(
                debug_imgs_[1], 
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
    const float kExclusionRadiusRatio = emp_cfg_["isInsideGlassExclusion"]["kExclusionRadiusRatio"].as<float>();

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
    const float kExclusionRadiusRatio = spec_glint_cfg_["isPupilNearby"]["kExclusionRadiusRatio"].as<float>();

    if (debug_imgs_.empty()) return;

    // Visualize all found initial seeds
    for (const auto& pupil : init_pupil_seeds_)
    {
        // 1. Draw pupil ellipse (Green)
        cv::ellipse(debug_imgs_[2], pupil.rr, cv::Scalar(0, 255, 0), 1);
        cv::circle(debug_imgs_[2], pupil.rr.center, 2, cv::Scalar(0, 255, 0), -1);

        // 2. Draw exclusion radius (Cyan/Yellow dashed equivalent)
        float exclusion_radius = pupil.major_axis * 0.5 * kExclusionRadiusRatio;
        cv::circle(
            debug_imgs_[2],
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
    // 从 Cfg 读取超参数
    auto spec = spec_pupil_cfg_["searchPupilInROI"];
    auto emp = emp_cfg_["searchPupilInROI"];

    const double kAdaptiveThreshOffset = spec["kAdaptiveThreshOffset"].as<double>();
    const double kAdaptiveThreshMax = spec["kAdaptiveThreshMax"].as<double>();
    const int kMinPupilArea = spec["kMinPupilArea"].as<int>();
    const int kMaxPupilArea = spec["kMaxPupilArea"].as<int>();
    const size_t kMinPupilContourPoints = spec["kMinPupilContourPoints"].as<int>();
    const float kMaxPupilAxis = spec["kMaxPupilAxis"].as<float>();
    const float kMaxAxisRatio = spec["kMaxAxisRatio"].as<float>();
    const double kMinSolidity = spec["kMinSolidity"].as<double>();
    const double kMinFitRatio = spec["kMinFitRatio"].as<double>();
    const float kMaxAvgResidual = spec["kMaxAvgResidual"].as<float>();
    const double kMaxDarkness = spec["kMaxDarkness"].as<double>();

    const int kMorphKernelSize = emp["kMorphKernelSize"].as<int>();
    const int kMorphCloseIterations = emp["kMorphCloseIterations"].as<int>();
    const float kValidResidualRatio = emp["kValidResidualRatio"].as<float>();
    const double kAxisRatioTorlerance = emp["kAxisRatioTolerance"].as<double>();
    const double kMaxAngleAbs = emp["kMaxAngleAbs"].as<double>();

    std::vector<Pupil> pupils; 

    cv::Mat roi_img_raw = gray_(roi_rect); 
    cv::Mat roi_img = roi_img_raw.clone(); 

    double min_val;
    cv::minMaxLoc(roi_img, &min_val, nullptr, nullptr, nullptr);
    double adaptive_thresh = std::min(min_val + kAdaptiveThreshOffset, kAdaptiveThreshMax); 
    Logger::debug() << "[searchPupilInROI] Adaptive threshold: " << adaptive_thresh;

    cv::Mat binary_pupil;
    cv::threshold(roi_img, binary_pupil, adaptive_thresh, 255, cv::THRESH_BINARY_INV);
    if (viz_) {
        cv::Mat viz = binary_pupil.clone();
        cv::cvtColor(viz, viz, cv::COLOR_GRAY2BGR);
        debug_imgs_[0] = viz;
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kMorphKernelSize, kMorphKernelSize));
    cv::morphologyEx(binary_pupil, binary_pupil, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary_pupil, binary_pupil, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), kMorphCloseIterations);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_pupil, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); ++i)
    {
        double contour_area = cv::contourArea(contours[i]);
        Logger::debug() << "[searchPupilInROI] Checking pupil: (" << contours[i][0].x << ", " << contours[i][0].y << ")";
        if (contour_area < kMinPupilArea || contour_area > kMaxPupilArea) {
            Logger::debug() << "[searchPupilInROI] Pupil contour area is out of range: " << contour_area << " | expected: [" << kMinPupilArea << ", " << kMaxPupilArea << "]";
            continue;
        }
        if (contours[i].size() < kMinPupilContourPoints) {
            Logger::debug() << "[searchPupilInROI] Pupil contour has too few points: " << contours[i].size() << " | expected: >" << kMinPupilContourPoints;
            continue;
        }

        cv::RotatedRect rr = cv::fitEllipse(contours[i]);
        float major = std::max(rr.size.width, rr.size.height);
        float minor = std::min(rr.size.width, rr.size.height);
        double ellipse_area = (CV_PI * major * minor) / 4.0;
        float axis_ratio = minor > 0 ? (major / minor) : 0.0f;
        float pupil_angle = rr.angle; 
        if (pupil_angle > 90.0f) pupil_angle -= 180.0f; 

        if (viz_) {
            // 可视化原始轮廓
            cv::ellipse(debug_imgs_[0], rr, cv::Scalar(0, 0, 255), 1);
        }

        if (axis_ratio > kMaxAxisRatio) {
            Logger::debug() << "[searchPupilInROI] Pupil axis ratio is too large: " << axis_ratio << " | expected: <" << kMaxAxisRatio;
            continue;       
        }
        if ((axis_ratio > kAxisRatioTorlerance) && (std::abs(pupil_angle) > kMaxAngleAbs)) {
            Logger::debug() << "[searchPupilInROI] Pupil angle is too horizontal: " << pupil_angle 
                            << " | expected: [-" <<  kMaxAngleAbs << ", " << kMaxAngleAbs << "], axis_ratio: " << axis_ratio;
            continue;
        } else {
            Logger::debug() << "[searchPupilInROI] Pupil angle is within range: " << pupil_angle 
                            << " | expected: [-" <<  kMaxAngleAbs << ", " << kMaxAngleAbs << "], axis_ratio: " << axis_ratio;
        }
        if (major > kMaxPupilAxis) {
            Logger::debug() << "[searchPupilInROI] Pupil major axis is too large: " << major << " | expected: <" << kMaxPupilAxis;
            continue;
        }

        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);
        double hull_area = cv::contourArea(hull);
        double solidity = contour_area / hull_area;
        if (solidity < kMinSolidity) {
            Logger::debug() << "[searchPupilInROI] Pupil solidity is too low: " << solidity << " | expected: >" << kMinSolidity;
            // continue;
        }

        double fit_ratio = contour_area / ellipse_area;
        if (fit_ratio < kMinFitRatio) {
            Logger::debug() << "[searchPupilInROI] Pupil fit ratio is too low: " << fit_ratio << " | expected: >" << kMinFitRatio;
            // continue;
        }

        // --- 残差计算 ---
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

        if (avg_residual > kMaxAvgResidual) {
            Logger::debug() << "[searchPupilInROI] Pupil average residual is too high: " << avg_residual << " | expected: <" << kMaxAvgResidual;
            // continue;
        }

        cv::Mat mask = cv::Mat::zeros(binary_pupil.size(), CV_8UC1);
        cv::drawContours(mask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
        cv::Scalar mean_val = cv::mean(roi_img_raw, mask);
        double darkness = mean_val[0];

        if (darkness > kMaxDarkness) {
            Logger::debug() << "[searchPupilInROI] Pupil darkness is too high: " << darkness << " | expected: <" << kMaxDarkness;
            continue;
        }

        Logger::debug() << "[searchPupilInROI] pupil accepted ===================================";

        if (viz_)
        {
            // 可视化pupil
            cv::ellipse(debug_imgs_[0], rr, cv::Scalar(0, 255, 0), 2);
        }

        // --- 构造 Pupil 对象并记录所有参数 ---
        Pupil p;
        p.rr = rr;
        p.rr.center += cv::Point2f(roi_rect.tl()); 
        p.major_axis = major;
        p.minor_axis = minor;
        
        // 保存特异型特征，用于统计计算
        p.roi_min_val = min_val;
        p.area = contour_area;
        p.contour_points = contours[i].size();
        p.axis_ratio = axis_ratio;
        p.solidity = solidity;
        p.fit_ratio = fit_ratio;
        p.avg_residual = avg_residual;
        p.darkness = darkness;
        
        pupils.push_back(p);
    }
    
    std::sort(pupils.begin(), pupils.end(),[](const Pupil& a, const Pupil& b){
        return a.darkness < b.darkness;
    });

    return pupils;
}

bool GlintDetector::isPupilNearby(const cv::Point2f& glint_pt)
{
    const float kExclusionRadiusRatio = spec_glint_cfg_["isPupilNearby"]["kExclusionRadiusRatio"].as<float>(); // 距离阈值

    // 直接遍历缓存的 seeds
    for (const auto& pupil : init_pupil_seeds_)
    {
        // 只有纯几何距离判断，没有任何暗度/分数计算
        if ((cv::norm(glint_pt - pupil.rr.center) < (pupil.major_axis * 0.5 * kExclusionRadiusRatio))) {
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
    CfgNode emp = emp_cfg_["findNeighborInDirection"];

    // --- 超参数声明 (Hyperparameters) ---
    // 默认搜索区域扩展系数 (相对当前节点的长度)
    const float kDefaultSearchScaleLenRatio = emp["kDefaultSearchScaleLenRatio"].as<float>();
    const float kDefaultSearchScaleWidRatio = emp["kDefaultSearchScaleWidRatio"].as<float>();
    // 过滤噪点的最小面积
    const double kMinContourArea = emp["kMinContourArea"].as<double>();
    // 节点间最小距离系数 (避免检测到自身)
    const float kMinDistRatio = emp["kMinDistRatio"].as<float>();
    // 新节点可靠尺度继承的最小长度比例
    const float kMinReliableLenRatio = emp["kMinReliableLenRatio"].as<float>();
    // 需要交换长宽的角度范围
    const float kAngleSwapMin = emp["kAngleSwapMin"].as<float>();
    const float kAngleSwapMax = emp["kAngleSwapMax"].as<float>();

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

    if (viz_) cv::rectangle(debug_imgs_[2], roi, cv::Scalar(255, 0, 0), 1);

    // 获取 ROI 图像
    cv::Mat roi_img = abs_dst_(roi);
    
    // 最佳候选
    bool found_candidate = false;
    float min_dist_sq = FLT_MAX; // 找离当前节点最近的，还是最显著的？通常找最近的比较稳

    // ---- 动态降低阈值循环 ----
    for (double thr = glass_reflection_threshold_; thr >= mini_threshold_; thr -= threshold_step_)
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
                cv::circle(debug_imgs_[2], candidate.center, 2, cv::Scalar(255, 255, 0), -1); // 青色点
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
    CfgNode emp = emp_cfg_["searchReflectionChains"];

    // --- 超参数声明 (Hyperparameters) ---
    // 向量更新的最小模长，防止除以零或由于距离太近导致方向剧烈跳变
    const float kMinNormEpsilon = emp["kMinNormEpsilon"].as<float>();
    // 反光链的最大节点数，防止无限循环或过度延伸
    const size_t kMaxChainLength = emp["kMaxChainLength"].as<size_t>();

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
    CfgNode emp = emp_cfg_["searchFrameReflections"];

    // --- 超参数声明 (Hyperparameters) ---
    // 初始镜框反光种子的面积限制
    const double kMinArea = emp["kMinArea"].as<double>();
    const double kMaxArea = emp["kMaxArea"].as<double>();
    // 初始镜框反光种子的长宽比最小限制 (长条状)
    const float kMinLengthWidthRatio = emp["kMinLengthWidthRatio"].as<float>();
    // ROI 搜索区域的宽、高缩放系数 (相对于反光斑长边)
    const float kSearchScaleLenRatio = emp["kSearchScaleLenRatio"].as<float>();
    const float kSearchScaleWidRatio = emp["kSearchScaleWidRatio"].as<float>();
    // 需要交换搜索区域长宽的角度范围
    const float kAngleSwapMin = emp["kAngleSwapMin"].as<float>();
    const float kAngleSwapMax = emp["kAngleSwapMax"].as<float>();
    // 绘制链条时要求的最短链长度
    const size_t kMinChainSizeToDraw = emp["kMinChainSizeToDraw"].as<size_t>();

    if (viz_)
    {
        // ---- 可视化底图 ----
        cv::Mat viz;
        cv::cvtColor(gray_, viz, cv::COLOR_GRAY2BGR);
        debug_imgs_[2] = viz; // frame reflection viz
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
                    debug_imgs_[2],
                    fr.points[i],
                    fr.points[(i + 1) % 4],
                    cv::Scalar(0, 0, 255), 
                    2
                );
            }

            // 标出角度
            cv::Point2f angle_pt = fr.center + cv::Point2f(0.75f * fr.length, - 1.5f * fr.width);
            cv::putText(debug_imgs_[2],
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
                        cv::line(debug_imgs_[2], chain[i-1].center, fr.center, 
                                cv::Scalar(255, 255, 0), 1); // 蓝色连线
                    }

                    // 2. 标序号
                    cv::putText(debug_imgs_[2], std::to_string(i), fr.center + cv::Point2f(0, -10), 
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
    CfgNode emp = emp_cfg_["buildExclusionMask"];

    // --- 超参数声明 (Hyperparameters) ---
    // 镜片反光排除区域的放大倍数
    const float kGlassExclusionScale = emp["kGlassExclusionScale"].as<float>();
    // 单个镜框反光排除区域的放大倍数
    const float kFrameExclusionScale = emp["kFrameExclusionScale"].as<float>();
    // 镜框反光链之间连线排除区域的放大倍数
    const float kChainExclusionScale = emp["kChainExclusionScale"].as<float>();

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
    
    if (viz_) debug_imgs_[3] = exclusion_mask_; // exclusion mask viz
}

bool GlintDetector::isInsideExclusionRegion(const cv::Point2f& pt) const
{
    // 边界检查
    if (pt.x < 0 || pt.x >= exclusion_mask_.cols || 
        pt.y < 0 || pt.y >= exclusion_mask_.rows) {
        return true; // 越界视为无效区域
    }

    // 查表：如果掩膜值大于阈值，则在排除区域内
    return exclusion_mask_.at<uchar>(cv::Point(pt)) > 128;
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
    CfgNode emp = emp_cfg_["shrinkRoiToValidGlints"];
    // =========================================================================
    // 超参数声明 (Hyperparameters for Noise Filtering)
    // 适配不同分辨率时，请按图像比例放大/缩小以下参数。
    // 当前默认参数适用于常规摄像头分辨率（如 720p / 1080p 级别）
    // =========================================================================
    
    // 1. 噪点最大像素数/面积 (对应特征1：一般在5个pixel以下)
    // 如果一个亮斑的面积 <= 此值，它被视作“潜在噪点”，需进一步结合聚集情况判断。
    const double kMaxNoiseArea = emp["kMaxNoiseArea"].as<double>();       
    
    // 2. 聚集距离阈值 (对应特征2：判定几个亮斑是否属于同一个“眼部聚集区”)
    // 两个连通域中心距离小于此值，则归为同一聚类 (Cluster)。
    const double kClusterDistThresh = emp["kClusterDistThresh"].as<double>();       
    
    // 3. 有效聚集的最小数量 (对应特征2：双眼区域glints聚集一般在3个及以上)
    // 如果一个聚类中全是微小斑点，且数量 < 此值，则整个聚类被视为孤立噪点剔除。
    const int kMinClusterSize = emp["kMinClusterSize"].as<int>();          

    // 4. 扩展边界 (上下方向)
    const int kPadding = emp["kPadding"].as<int>();       
    
    // 5. 默认极低敏感度阈值 (当 threshold_step_ 无效时)
    const double kDefaultLowSensitivityThresh = emp["kDefaultLowSensitivityThresh"].as<double>();       
 
    // =========================================================================

    // 1. 边界保护与 ROI 提取
    cv::Rect valid_roi = coarse_roi & cv::Rect(0, 0, gray_.cols, gray_.rows);
    if (valid_roi.empty()) return cv::Rect();

    cv::Mat roi_img = abs_dst_(valid_roi);

    // 2. 阈值处理 (保留原有的极低敏感度阈值)
    double low_sensitivity_thresh = kDefaultLowSensitivityThresh;
    
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
    refined_rect.x -= kPadding;
    refined_rect.y -= kPadding;
    refined_rect.width += (kPadding * 2);
    refined_rect.height += (kPadding * 2);

    // 10. 最终边界限制
    refined_rect &= valid_roi;

    return refined_rect;
}

std::vector<cv::Rect> GlintDetector::subtractRect(const cv::Rect& subject, const cv::Rect& clipper) const
{
    std::vector<cv::Rect> result;
    cv::Rect intersect = subject & clipper;
    
    // 1. 如果没有交集，原样返回
    if (intersect.empty()) { result.push_back(subject); return result; }
    // 2. 如果完全被遮挡，返回空
    if (intersect == subject) return result; 

    // 3. 拆分剩余部分 (上、下、左侧中间、右侧中间)
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
}

void GlintDetector::excludeIrisReflectionsFromROIs(std::vector<cv::Rect>& rois, std::vector<cv::Rect>& out_iris_bboxes)
{
    // --- 超参数声明 (Hyperparameters) ---
    CfgNode emp = emp_cfg_["excludeIrisReflectionsFromROIs"];
    const double kIrisReflectionThreshold = emp["kIrisReflectionThreshold"].as<double>();
    const double kMinIrisReflectionArea = emp["kMinIrisReflectionArea"].as<double>();
    const int kPadding = emp["kPadding"].as<int>();
    const int kMinRemainderArea = 10; 

    std::vector<cv::Rect> refined_rois;

    for (const auto& roi : rois)
    {
        cv::Rect safe_roi = roi & cv::Rect(0, 0, gray_.cols, gray_.rows);
        if (safe_roi.empty()) continue;

        if (viz_) {
            cv::rectangle(debug_imgs_[4], safe_roi, cv::Scalar(0, 255, 0), 1);
        }

        cv::Mat roi_gray = gray_(safe_roi);
        cv::Mat binary;
        cv::threshold(roi_gray, binary, kIrisReflectionThreshold, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Rect> iris_bboxes;
        for (const auto& cnt : contours)
        {
            double area = std::max(cv::contourArea(cnt), static_cast<double>(cnt.size()));
            if (area > kMinIrisReflectionArea)
            {
                cv::Rect local_bbox = cv::boundingRect(cnt);
                cv::Rect global_bbox = local_bbox;
                global_bbox.x += safe_roi.x;
                global_bbox.y += safe_roi.y;

                // 适当扩展
                global_bbox.x -= kPadding;
                global_bbox.y -= kPadding;
                global_bbox.width += (kPadding * 2);
                global_bbox.height += (kPadding * 2);

                Logger::debug() << "[4 getROI] Iris reflection found: (" << global_bbox.x << ", " << global_bbox.y 
                                << ", " << global_bbox.width << ", " << global_bbox.height << ")";

                iris_bboxes.push_back(global_bbox);
                out_iris_bboxes.push_back(global_bbox); // 收集到外部容器中提供给 Cluster

                if (viz_) {
                    cv::rectangle(debug_imgs_[4], global_bbox, cv::Scalar(255, 0, 0), 1);
                }
            }
        }

        // 使用类成员方法 this->subtractRect 进行碎片化切割
        std::vector<cv::Rect> current_pieces = { safe_roi };
        for (const auto& bbox : iris_bboxes)
        {
            std::vector<cv::Rect> next_pieces;
            for (const auto& piece : current_pieces)
            {
                auto remainders = subtractRect(piece, bbox);
                for (const auto& r : remainders) {
                    if (r.area() > kMinRemainderArea) {
                        next_pieces.push_back(r);
                    }
                }
            }
            current_pieces = next_pieces;
            if (current_pieces.empty()) break;
        }

        for (const auto& piece : current_pieces) {
            refined_rois.push_back(piece);
        }
    }

    rois = refined_rois;
}

std::vector<cv::Rect> GlintDetector::determineCornealReflectionROI()
{
    CfgNode emp = emp_cfg_["determineCornealReflectionROI"];

    // --- 超参数声明 (Hyperparameters) ---
    // 瞳孔种子约束区域半径放大系数 (基于长轴)
    const float kConstraintRadiusRatio = spec_glint_cfg_["isPupilNearby"]["kExclusionRadiusRatio"].as<float>();
    // 扩展步长循环控制
    const int kRatioH = 3;
    const int kRatioV = 1;
    // 候选区域最小有效面积
    const int kMinExpandedRoiArea = emp["kMinExpandedRoiArea"].as<int>();
    // 边界收缩比例 (去除极边缘部分)
    const float kShrinkRatio = emp["kShrinkRatio"].as<float>();
    // 差集操作后保留的最小碎片面积
    const int kMinRemainderArea = emp["kMinRemainderArea"].as<int>();

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
        float constraint_radius = pupil.major_axis * 0.5 * kConstraintRadiusRatio;
        
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
            cv::rectangle(viz_raw, r, cv::Scalar(255, 205, 135), 1);
        }
        for (const auto& p : debug_pupil_centers) {
            cv::circle(viz_raw, p, 3, cv::Scalar(0, 0, 255), -1);
        }
        
        debug_imgs_[4] = viz_raw;
    }

    // ...[Layered Difference Logic remains unchanged] ...
    if (!expanded_candidates.empty())
    {
        std::sort(expanded_candidates.begin(), expanded_candidates.end(),[](const cv::Rect& a, const cv::Rect& b) { return a.area() > b.area(); });

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

std::vector<GlintDetector::RoiCluster> GlintDetector::getROI()
{
    std::vector<RoiCluster> final_roi_clusters;
    // 1-5 步保持不变...
    init_pupil_seeds_.clear();
    final_pupils_.clear();
    cv::Rect full_img_rect(0, 0, gray_.cols, gray_.rows);
    glint_rect_ = shrinkRoiToValidGlints(full_img_rect);

    if (!glint_rect_.empty()) { 
        init_pupil_seeds_ = searchPupilInROI(glint_rect_);
    } else {
        return final_roi_clusters;
    }

    searchGlassReflections(); 
    searchFrameReflections();
    buildExclusionMask();

    // 6. 确定角膜反光 ROI
    auto rois = determineCornealReflectionROI();

    // 7. --- 新增：排除虹膜反射，并搜集全局所有的虹膜障碍物 ---
    std::vector<cv::Rect> global_iris_exclusions;
    excludeIrisReflectionsFromROIs(rois, global_iris_exclusions);

    // 8. --- 聚类与数据组装 ---
    auto clusters = clusterROIs(rois);
    for (int i = 0; i < clusters.size(); ++i)
    {
        RoiCluster rc;
        rc.cluster_id = i;
        rc.rois = clusters[i];

        // 8.1 计算极限边界 (Bounding Box)
        if (!rc.rois.empty()) {
            rc.limit_bound = rc.rois[0];
            for (size_t k = 1; k < rc.rois.size(); ++k) {
                rc.limit_bound |= rc.rois[k];
            }
        }

        // 8.2 获取最佳 Pupil
        rc.best_pupil = findBestPupilForCluster(rc.rois);

        // 8.3 筛选出与本聚类重合或内部的虹膜反射区域
        for (const auto& ex : global_iris_exclusions) {
            if ((ex & rc.limit_bound).area() > 0) {
                rc.iris_exclusions.push_back(ex);
            }
        }

        final_roi_clusters.push_back(rc);
    }

    return final_roi_clusters;
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

    if (viz_threshold_ && debug_tag.size() > 0)
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
        if (threshold_value == mini_threshold_) {
            Logger::debug() << std::fixed << std::setprecision(2) << "[SearchGlintsInROI] Mini threshold point found: (" << rect.center.x << ", " << rect.center.y << ")";
        }
        if (isGlintRepeated(glints, rect.center)) continue;

        min_rects.push_back(rect);
        contour_centers.push_back(rect.center);
    }

    return contour_centers;
}

/*
std::vector<cv::Point2f>
GlintDetector::searchGlintsInROI(
    const cv::Mat& roi_img, 
    const cv::Point2f& roi_offset,
    const std::vector<cv::Point2f>& glints,
    const double threshold_value,
    const std::string& debug_tag,
    bool use_super_resolution // 新增参数
)
{
    cv::Mat process_img;
    cv::Mat sr_raw_roi; // ★ 新增：用于存储超分后的原始灰度图（非特征提取图）
    
    int sr_scale = emp_cfg_["searchGlintsInROI"]["sr_scale"].as<int>();
    int sr_padding = emp_cfg_["searchGlintsInROI"]["sr_padding"].as<int>();

    if (use_super_resolution) {
        int x = static_cast<int>(roi_offset.x);
        int y = static_cast<int>(roi_offset.y);
        int w = roi_img.cols;
        int h = roi_img.rows;

        int pad_l = std::min(x, sr_padding);
        int pad_t = std::min(y, sr_padding);
        int pad_r = std::min(gray_.cols - (x + w), sr_padding);
        int pad_b = std::min(gray_.rows - (y + h), sr_padding);

        cv::Rect padded_rect(x - pad_l, y - pad_t, w + pad_l + pad_r, h + pad_t + pad_b);
        cv::Mat padded_img = gray_(padded_rect); 

        // 1. 使用 Lanczos4 插值生成高清原图
        cv::Mat upscaled_padded;
        cv::resize(padded_img, upscaled_padded, cv::Size(), sr_scale, sr_scale, cv::INTER_LANCZOS4);

        // 2. 裁剪出 ROI 对应的超分灰度图 (用于可视化)
        cv::Rect crop_rect(pad_l * sr_scale, pad_t * sr_scale, w * sr_scale, h * sr_scale);
        sr_raw_roi = upscaled_padded(crop_rect).clone();

        // 3. 执行特征提取管线 (用于实际检测)
        cv::Mat sr_gaussed, sr_laplaced, sr_abs;
        cv::GaussianBlur(sr_raw_roi, sr_gaussed, cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 0, 0);
        cv::Laplacian(sr_gaussed, sr_laplaced, CV_16S, laplacian_kernel_size_, laplacian_scale_, laplacian_delta_);
        cv::convertScaleAbs(sr_laplaced, sr_abs);
        process_img = sr_abs;
    } else {
        process_img = roi_img.clone();
    }

    // 执行阈值处理和轮廓搜索
    cv::Mat threshold_output;
    cv::threshold(process_img, threshold_output, threshold_value, 255, cv::THRESH_BINARY);

    // ★ 改进后的可视化逻辑：直接拷贝 SR 后的 gray 原图到底图
    if (viz_ && use_super_resolution && !debug_imgs_[7].empty()) {
        cv::Rect dst_rect(
            static_cast<int>(roi_offset.x) * sr_scale,
            static_cast<int>(roi_offset.y) * sr_scale,
            sr_raw_roi.cols,
            sr_raw_roi.rows
        );
        
        dst_rect &= cv::Rect(0, 0, debug_imgs_[7].cols, debug_imgs_[7].rows);
        
        if (dst_rect.width > 0 && dst_rect.height > 0) {
            for (int r = 0; r < dst_rect.height; ++r) {
                const uchar* src_ptr = sr_raw_roi.ptr<uchar>(r);
                cv::Vec3b* dst_ptr = debug_imgs_[7].ptr<cv::Vec3b>(dst_rect.y + r);
                for (int c = 0; c < dst_rect.width; ++c) {
                    uchar val = src_ptr[c];
                    // 拷贝原始灰度值，无颜色标记，方便对比 Lanczos4 的平滑度
                    dst_ptr[dst_rect.x + c] = cv::Vec3b(val, val, val);
                }
            }
        }
    }

    if (viz_threshold_ && debug_tag.size() > 0) {
        std::string threshold_output_folder = 
            cfg_["test_glint"]["input_folder"].as<std::string>() + "\\threshold_output\\" + img_name_ + "\\"
            + debug_tag;

        std::string save_path = threshold_output_folder + "\\" +  std::to_string(threshold_value) + ".png";

        std::filesystem::path folder_path(threshold_output_folder);
        if (!std::filesystem::exists(folder_path)) std::filesystem::create_directories(folder_path);

        cv::imwrite(save_path, threshold_output);
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // ★ 注意：此时不能传入 roi_offset，因为超分后坐标系尺度已经发生了变化，需在局部找完后再放缩
    cv::findContours(
        threshold_output, 
        contours, 
        hierarchy, 
        cv::RETR_EXTERNAL, 
        cv::CHAIN_APPROX_SIMPLE
    );

    std::vector<cv::RotatedRect> min_rects;
    std::vector<cv::Point2f> contour_centers;

    if (contours.empty()) return contour_centers;

    for (size_t k = 0; k < contours.size(); ++k)
    {
        cv::RotatedRect rect = cv::minAreaRect(contours[k]);
        cv::Point2f global_center;

        // 5. 将尺度缩放回原始原图坐标系
        if (use_super_resolution) {
            global_center.x = (rect.center.x / sr_scale) + roi_offset.x;
            global_center.y = (rect.center.y / sr_scale) + roi_offset.y;
        } else {
            global_center.x = rect.center.x + roi_offset.x;
            global_center.y = rect.center.y + roi_offset.y;
        }

        if (threshold_value == mini_threshold_) {
            Logger::debug() << std::fixed << std::setprecision(2) << "[SearchGlintsInROI] Mini threshold point found: (" << global_center.x << ", " << global_center.y << ")";
        }
        if (isGlintRepeated(glints, global_center)) continue;

        min_rects.push_back(rect);
        contour_centers.push_back(global_center);
    }

    return contour_centers;
}
*/

std::tuple<
    std::vector<GlintDetector::GlintGeometry>, 
    std::vector<GlintDetector::GlintGeometry>
>
GlintDetector::splitGlintsGeometry(std::vector<GlintDetector::GlintGeometry> glint_geometries)
{
    std::vector<GlintDetector::GlintGeometry> left_eye_geometries, right_eye_geometries;
    const double kDistanceThresholdX = emp_cfg_["splitGlintsGeometry"]["kDistanceThresholdX"].as<double>();  // 将两眼斑点分为左右眼的阈值

    if (glint_geometries.empty()) {
        return { left_eye_geometries, right_eye_geometries };
    }

    // 1. 计算每个 geometry 的平均 X 坐标并与其原始数据绑定
    struct IndexedGeometry {
        double avg_x;
        GlintDetector::GlintGeometry geometry;
    };
    
    std::vector<IndexedGeometry> sorted_geometries;
    for (const auto& geo : glint_geometries) {
        sorted_geometries.push_back({ geo.center().x, geo });
    }

    // 2. 按照平均 X 坐标从小到大排序 (右眼 x 小，排在前面)
    std::sort(sorted_geometries.begin(), sorted_geometries.end(),
              [](const IndexedGeometry& a, const IndexedGeometry& b) {
                  return a.avg_x < b.avg_x;
              });

    double right_x_mean = sorted_geometries.front().avg_x;
    right_eye_geometries.push_back(sorted_geometries.front().geometry);

    for (size_t i = 1; i < sorted_geometries.size(); ++i)
    {
        const auto& current_geo = sorted_geometries[i];

        // 判断当前几何体的平均 X 是否靠近当前“右眼组”的平均 X
        if (std::abs(current_geo.avg_x - right_x_mean) < kDistanceThresholdX)
        {
            right_eye_geometries.push_back(current_geo.geometry);

            // 增量更新右眼组的 X 平均值中心
            const size_t n = right_eye_geometries.size();
            right_x_mean += (current_geo.avg_x - right_x_mean) / static_cast<double>(n);
        }
        else
        {
            // 距离较远，归入左眼
            left_eye_geometries.push_back(current_geo.geometry);
        }
    }

    return { left_eye_geometries, right_eye_geometries };
}

GlintDetector::Pupil 
GlintDetector::findBestPupilForCluster(const std::vector<cv::Rect>& cluster_rois)
{
    Pupil best_pupil;
    // 初始化为一个无效/空的瞳孔
    best_pupil.area = -1.0f;

    if (init_pupil_seeds_.empty() || cluster_rois.empty()) {
        return best_pupil;
    }

    // --- 超参数声明 (Hyperparameters) ---
    // 从经验配置中读取暗度容差。请确保在配置文件中添加了此参数。
    CfgNode emp = emp_cfg_["findBestPupilForCluster"];
    const double kDarknessTolerance = emp["kDarknessTolerance"].as<double>(); 

    // 1. 收集所有属于当前 Cluster 的候选瞳孔
    std::vector<Pupil> cluster_candidates;
    for (const auto& pupil : init_pupil_seeds_)
    {
        bool belongs_to_cluster = false;
        for (const auto& roi : cluster_rois) {
            if (roi.contains(pupil.rr.center)) {
                belongs_to_cluster = true;
                break;
            }
        }

        if (belongs_to_cluster) {
            cluster_candidates.push_back(pupil);
        }
    }

    // 2. 如果没有候选瞳孔，直接返回空结果
    if (cluster_candidates.empty()) {
        Logger::debug() << "[ClusterPupil] No matching pupil found for this cluster.";
        return best_pupil;
    }

    // 3. 采用分级比较逻辑 (Tiered Selection) 寻找最优瞳孔
    best_pupil = cluster_candidates[0];

    for (size_t i = 1; i < cluster_candidates.size(); ++i) {
        const auto& a = cluster_candidates[i];
        const auto& b = best_pupil; // b 始终代表当前选出的"最优解"

        bool a_is_better = false;
        double dark_diff = std::abs(a.darkness - b.darkness);

        // --- [Tier 1]: 暗度判据 ---
        // 瞳孔越暗越好 (darkness 越小越好)。如果差异超过了容差，直接选择更暗的。
        if (dark_diff > kDarknessTolerance) {
            a_is_better = (a.darkness < b.darkness);
        } 
        // --- [Tier 2]: 面积判据 (当暗度相近时) ---
        // 如果两者的暗度差异在容差范围内，则认为暗度表现一致，转而选择面积更大的。
        else {
            a_is_better = (a.area > b.area);
        }

        // 更新最优解
        if (a_is_better) {
            best_pupil = a;
        }
    }
    
    Logger::debug() << "[ClusterPupil] Selected pupil for cluster. Center: (" 
                    << static_cast<int>(best_pupil.rr.center.x) << ", " 
                    << static_cast<int>(best_pupil.rr.center.y) 
                    << "), Area: " << best_pupil.area
                    << ", Darkness: " << best_pupil.darkness;

    return best_pupil;
}

std::vector<GlintDetector::GlintGeometry> 
GlintDetector::selectBestGlintsPerCluster(const std::vector<GlintGeometry>& all_candidates)
{
    CfgNode emp = emp_cfg_["selectBestGlintsPerCluster"];
    // --- 超参数声明 (Hyperparameters) ---
    const double kDistTolerance = emp["kDistTolerance"].as<double>(); 
    const double kBrightTolerance = emp["kBrightTolerance"].as<double>(); 

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

        // 使用手写循环替代 std::min_element，避免容差比较打破 C++ STL 的严格弱序原则导致崩溃
        GlintGeometry best_geo = candidates[0];

        for (size_t i = 1; i < candidates.size(); ++i) {
            const auto& a = candidates[i];
            const auto& b = best_geo; // b 始终代表当前选出的"最优解"

            bool has_pupil_a = (a.linked_pupil.major_axis > 0);
            bool has_pupil_b = (b.linked_pupil.major_axis > 0);
            
            bool a_is_better = false;

            // --- [Tier 0]: 瞳孔有效性检查 ---
            if (has_pupil_a != has_pupil_b) {
                a_is_better = has_pupil_a; // 谁有瞳孔谁就更好
            } 
            else if (!has_pupil_a) {
                // 如果都没有瞳孔，回退到只比亮度
                a_is_better = (a.bg_brightness < b.bg_brightness);
            } 
            else {
                // --- [Tier 1]: 距离判据 ---
                double dist_a = cv::norm(a.center() - a.linked_pupil.rr.center);
                double dist_b = cv::norm(b.center() - b.linked_pupil.rr.center);
                double dist_diff = std::abs(dist_a - dist_b);

                if (dist_diff > kDistTolerance) {
                    a_is_better = (dist_a < dist_b);
                } 
                else {
                    // ---[Tier 2]: 暗度判据 (当距离相近时) ---
                    double bri_diff = std::abs(a.bg_brightness - b.bg_brightness);
                    if (bri_diff > kBrightTolerance) {
                        a_is_better = (a.bg_brightness < b.bg_brightness);
                    } 
                    else {
                        // --- [Tier 3]: 阈值判据 (当暗度也相近时) ---
                        a_is_better = (a.found_threshold > b.found_threshold);
                    }
                }
            }

            // 更新最优解
            if (a_is_better) {
                best_geo = a;
            }
        }
        
        // 记录并输出日志
        final_results.push_back(best_geo);
        
        // 【安全打印】避免 best_geo 没有 pupil 时，用 (0,0) 算出离谱的距离误导 Debug
        double print_dist = (best_geo.linked_pupil.major_axis > 0) 
            ? cv::norm(best_geo.center() - best_geo.linked_pupil.rr.center) 
            : -1.0;

        Logger::debug() << "[SelectBest] Cluster " << id << " selected:"
                        << " Dist=" << print_dist
                        << " Bri=" << best_geo.bg_brightness
                        << " Thr=" << best_geo.found_threshold;
    }

    return final_results;
}

std::vector<GlintDetector::GlintGeometry> 
GlintDetector::detectCluster(const RoiCluster& cluster)
{
    CfgNode emp = emp_cfg_["detectCluster"];
    const size_t kMaxClusterResults = emp["kMaxClusterResults"].as<size_t>();

    std::vector<GlintGeometry> cluster_results;
    double thr = init_threshold_value_;

    // =========================================================================
    // 高阈值真实位置累加器，用于在后续降低阈值时屏蔽分裂的小噪点
    // =========================================================================
    std::vector<cv::Point2f> accumulated_glints; 

    while (thr >= mini_threshold_)
    {
        std::vector<cv::Point2f> newly_found_glints;
        // 遍历该 Cluster 内部被安全切割后的所有小碎片 ROI
        for (const auto& roi : cluster.rois)
        {
            cv::Mat roi_img = abs_dst_(roi);
            cv::Point2f roi_offset(roi.x, roi.y);

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << "cluster_" << roi.x << "_" << roi.y;
            
            // 传入 accumulated_glints 进行判重屏蔽
            auto pts = searchGlintsInROI(roi_img, roi_offset, accumulated_glints, thr, oss.str());
            newly_found_glints.insert(newly_found_glints.end(), pts.begin(), pts.end());
        }

        // 将当前阈值下新发现的点也加入全局累加器中
        accumulated_glints.insert(accumulated_glints.end(), newly_found_glints.begin(), newly_found_glints.end());

        // 供 findGeometry 搜索的点集，包含历史上（更高阈值）和当前阈值发现的所有点
        std::vector<cv::Point2f> current_pass_points = accumulated_glints;

        // 对候选点进行排序，保证匹配的稳定性和确定性
        sortGlintCandidates(current_pass_points);

        // 传入包含安全边界和障碍物信息的 cluster
        auto geometries_found = findGeometry(current_pass_points, thr, cluster_results, cluster);

        bool found_new_unique = false;
        for (auto& geo : geometries_found)
        {
            if (cluster_results.size() >= kMaxClusterResults) break;

            if (!isGlintGeometryRepeated(geo, cluster_results)) {
                // 从统一数据结构中读取信息
                geo.cluster_id = cluster.cluster_id;
                geo.found_threshold = thr;
                geo.linked_pupil = cluster.best_pupil; 

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
    std::vector<std::vector<cv::Point2f>>,
    cv::Point2f,
    cv::Point2f
>
GlintDetector::detect(cv::Mat gray)
{
    if (local_debug_) Logger::setLevel(Logger::Level::DEBUG);
    if (!local_debug_ && debug_time_) Logger::setLevel(Logger::Level::TIME);
    if (!local_debug_ && !debug_time_) Logger::setLevel(Logger::Level::INFO);
    Logger::ScopedTimer timer("[GlintDetector::detectFullImage]");
    
    debug_imgs_.clear();
    debug_imgs_.resize(8);
    final_geometries_.clear();

    gray_ = gray.clone();

    // 1-3 Preprocessing
    cv::GaussianBlur(gray, gaussed_, cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 0, 0, cv::BORDER_DEFAULT);
    cv::Laplacian(gaussed_, laplaced_, CV_16S, laplacian_kernel_size_, laplacian_scale_, laplacian_delta_, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(laplaced_, abs_dst_);
    cv::threshold(
        abs_dst_, threshold_output_, 
        cfg_["empirical_hyperparameter"]["shrinkRoiToValidGlints"]["kDefaultLowSensitivityThresh"].as<double>(), 
        255, cv::THRESH_BINARY);

    // 4 Get ROI (Populates init_pupil_seeds_ and returns integrated RoiClusters)
    auto roi_clusters = getROI();
    timer.lap("[4] getROI()");

    if (roi_clusters.empty()) {
        std::vector<std::vector<cv::Point2f>> left_glint_geometries, right_glint_geometries;
        cv::Point2f left_pupil_center(0.0f, 0.0f), right_pupil_center(0.0f, 0.0f);
        return { left_glint_geometries, right_glint_geometries, left_pupil_center, right_pupil_center };
    }

    std::vector<GlintGeometry> all_geometries;

    if (!cfg_["test_glint"]["debug_geometry"].as<bool>()) Logger::setLevel(Logger::Level::INFO);

    for (const auto& cluster : roi_clusters)
    {
        // 带着 Cluster 数据集（包含边界框，防虹膜越界约束等）去检测 Glints
        auto cluster_results = detectCluster(cluster);
        
        all_geometries.insert(all_geometries.end(), cluster_results.begin(), cluster_results.end());

        std::ostringstream oss;
        if (!cluster.rois.empty()) {
            oss << std::fixed << std::setprecision(2) << cluster.rois[0].x << "_" << cluster.rois[0].y;
        } else {
            oss << "empty";
        }
    }

    if (local_debug_) Logger::setLevel(Logger::Level::DEBUG);
    if (!local_debug_ && debug_time_) Logger::setLevel(Logger::Level::TIME);
    if (!local_debug_ && !debug_time_) Logger::setLevel(Logger::Level::INFO);
    timer.lap("[5] detectCluster()");

    // 6. Select Best Glints Per Cluster
    auto best_geometries = selectBestGlintsPerCluster(all_geometries);
    timer.lap("[6] Select Best Glints Per Cluster");

    // viz
    if (viz_) {
        std::vector<GlintGeometry> viz_geometries;
        if (is_collecting_) {
            viz_geometries = all_geometries;
        } else {
            viz_geometries = best_geometries;
        }

        for (const auto& geo : viz_geometries) {
            cv::Scalar color;
            color = geo.on_cornea ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            cv::line(debug_imgs_[4], geo.l_pt, geo.r_pt, color, 1, cv::LINE_AA);
            cv::line(debug_imgs_[4], geo.l_pt, geo.m_pt, color, 1, cv::LINE_AA);
            cv::line(debug_imgs_[4], geo.r_pt, geo.m_pt, color, 1, cv::LINE_AA);
            
            cv::putText(debug_imgs_[4], std::to_string((int)geo.bg_brightness), 
                        geo.center() + cv::Point2f(10, -10), cv::FONT_HERSHEY_PLAIN, 0.8, color, 1);
        }
    }

    Logger::debug() << "[GlintDetector::detect] Total Glints: " << best_geometries.size();
    for (const auto& geo : all_geometries)
    {
        Logger::debug() << std::fixed << std::setprecision(2) << "\t" 
                        << "(" << geo.l_pt.x << ", " << geo.l_pt.y << ") "
                        << "(" << geo.r_pt.x << ", " << geo.r_pt.y << ") "
                        << "(" << geo.m_pt.x << ", " << geo.m_pt.y << ") ";
    }

    // 7. Split
    std::vector<GlintDetector::GlintGeometry> glint_geometry_list;
    if (is_collecting_)
    {
        glint_geometry_list = all_geometries;
        final_geometries_ = all_geometries;
    } else {
        glint_geometry_list = best_geometries;
    }
    auto [left_glint_geometries, right_glint_geometries] = splitGlintsGeometry(glint_geometry_list);
    timer.lap("[7] Split Glints");

    // 8. 数量校验与提取瞳孔坐标
    cv::Point2f left_pupil_center(0.0f, 0.0f);
    cv::Point2f right_pupil_center(0.0f, 0.0f);

    if (!is_collecting_)
    {
        if (left_glint_geometries.size() == 1 && right_glint_geometries.size() == 1)
        {
            left_pupil_center = left_glint_geometries[0].linked_pupil.rr.center;
            right_pupil_center = right_glint_geometries[0].linked_pupil.rr.center;

            if (viz_) {
                cv::Mat viz_pupil = gray_.clone();
                cv::cvtColor(viz_pupil, viz_pupil, cv::COLOR_GRAY2BGR);
                debug_imgs_[5] = viz_pupil;
            }

            // === 针对左眼 (渲染至 debug_imgs_[7]) ===
            GlintGeometry left_geo = left_glint_geometries[0]; // 拷贝取值
            if (use_glint_sr_) refineGlintGeometry(gray_, left_geo, 7);
            Pupil refined_left = refinePupil(gaussed_, left_geo.linked_pupil, left_geo);
            left_geo.linked_pupil = refined_left;
            left_glint_geometries[0] = left_geo; // ✨ 显式写回容器

            // === 针对右眼 (渲染至 debug_imgs_[8]) ===
            GlintGeometry right_geo = right_glint_geometries[0]; // 拷贝取值
            if (use_glint_sr_) refineGlintGeometry(gray_, right_geo, 8);
            Pupil refined_right = refinePupil(gaussed_, right_geo.linked_pupil, right_geo);
            right_geo.linked_pupil = refined_right;
            right_glint_geometries[0] = right_geo; // ✨ 显式写回容器
        }
        else
        {
            left_glint_geometries.clear();
            right_glint_geometries.clear();
        }
    }
    timer.lap("[8] Glint Pupil refinement");

    Logger::debug() << "[GlintDetector::detect] Glints after refinement: ";
    if (!left_glint_geometries.empty()) {
        Logger::debug() << std::fixed << std::setprecision(2) << "\t" 
                        << "(" << left_glint_geometries[0].l_pt.x << ", " << left_glint_geometries[0].l_pt.y << ") "
                        << "(" << left_glint_geometries[0].r_pt.x << ", " << left_glint_geometries[0].r_pt.y << ") "
                        << "(" << left_glint_geometries[0].m_pt.x << ", " << left_glint_geometries[0].m_pt.y << ") ";
    }
    if (!right_glint_geometries.empty()) {
        Logger::debug() << std::fixed << std::setprecision(2) << "\t" 
                        << "(" << right_glint_geometries[0].l_pt.x << ", " << right_glint_geometries[0].l_pt.y << ") "
                        << "(" << right_glint_geometries[0].r_pt.x << ", " << right_glint_geometries[0].r_pt.y << ") "
                        << "(" << right_glint_geometries[0].m_pt.x << ", " << right_glint_geometries[0].m_pt.y << ") ";
    }

    // viz glint jitter
    if (viz_) {
        const int viz_mult = 5; // 放大倍数，可根据需要调整 (如 10 代表 1 个像素被放大为 10x10 的区域)
        cv::Mat viz_geo = cv::Mat::zeros(gray_.rows * viz_mult, gray_.cols * viz_mult, CV_8UC3);

        // 1. 图像多倍扩展：将 1 个灰度像素的值赋给 N*N 个 BGR 像素区块，与 refineGlintGeometry 保持一致
        for (int y = 0; y < gray_.rows; ++y) {
            const uchar* src_row = gray_.ptr<uchar>(y);
            int base_y = y * viz_mult;
            for (int x = 0; x < gray_.cols; ++x) {
                int base_x = x * viz_mult;
                cv::Vec3b color(src_row[x], src_row[x], src_row[x]); // 灰度转 BGR
                for (int dy = 0; dy < viz_mult; ++dy) {
                    cv::Vec3b* dst_row = viz_geo.ptr<cv::Vec3b>(base_y + dy);
                    for (int dx = 0; dx < viz_mult; ++dx) {
                        dst_row[base_x + dx] = color;
                    }
                }
            }
        }
        debug_imgs_[6] = viz_geo;

        // 2. 精细化子像素绘图函数
        auto setVizPixel = [&](float x, float y, cv::Vec3b color) {
            int px = static_cast<int>(std::round(x * viz_mult));
            int py = static_cast<int>(std::round(y * viz_mult));
            if (px >= 0 && px < debug_imgs_[6].cols && py >= 0 && py < debug_imgs_[6].rows) {
                debug_imgs_[6].at<cv::Vec3b>(py, px) = color;
            }
        };

        // 3. 十字准星绘制逻辑
        auto drawCrosshair = [&](const cv::Point2f& pt, cv::Vec3b color) {
            // 十字准星覆盖的单侧跨度：相当于在原图上延伸 2 个原图像素距离
            int cross_len = 2 * viz_mult; 
            for (int d = -cross_len; d <= cross_len; ++d) {
                setVizPixel(pt.x + d / static_cast<float>(viz_mult), pt.y, color); // 横线
                setVizPixel(pt.x, pt.y + d / static_cast<float>(viz_mult), color); // 竖线
            }
        };

        // 4. 绘制所有 Glints 的精细坐标
        for (const auto& geo : left_glint_geometries) {
            cv::Vec3b color = geo.on_cornea ? cv::Vec3b(0, 255, 0) : cv::Vec3b(0, 0, 255); // BGR: 绿 / 红
            drawCrosshair(geo.l_pt, color);
            drawCrosshair(geo.r_pt, color);
            drawCrosshair(geo.m_pt, color);
        }
        for (const auto& geo : right_glint_geometries) {
            cv::Vec3b color = geo.on_cornea ? cv::Vec3b(0, 255, 0) : cv::Vec3b(0, 0, 255);
            drawCrosshair(geo.l_pt, color);
            drawCrosshair(geo.r_pt, color);
            drawCrosshair(geo.m_pt, color);
        }
    }

    auto left_glint_geometry_list = glintGeometryListToGlintVector(left_glint_geometries);
    auto right_glint_geometry_list = glintGeometryListToGlintVector(right_glint_geometries);

    cv::Point2f offset(0.5f, 0.5f);
    left_pupil_center += offset;
    right_pupil_center += offset;

    return { left_glint_geometry_list, right_glint_geometry_list, left_pupil_center, right_pupil_center };
}

void GlintDetector::sortGlintCandidates(std::vector<cv::Point2f>& glints)
{
    std::sort(glints.begin(), glints.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        const float EPSILON = 1e-4f;
        // x 小的一定保证排在前面
        if (a.x < b.x - EPSILON) return true;
        if (a.x > b.x + EPSILON) return false;
        // 当 x 极其相近时，y 小的排在前面
        return a.y < b.y;
    });
}

std::vector<GlintDetector::GlintGeometry>
GlintDetector::findGeometry(
    std::vector<cv::Point2f> glint_candidates,
    double current_thr,
    const std::vector<GlintGeometry>& existing_geometries,
    const RoiCluster& cluster // --- 新增参数：使用统一数据结构 ---
)
{
    Logger::debug() << "\n[5 Find Geometry] Start finding geometry at threshold: " << current_thr;

    std::vector<GlintGeometry> found_geometries;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> candidate_pairs;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> bad_pairs;

    Logger::debug() << "[5 Find Geometry] Input candidates: ";
    for (const auto& can: glint_candidates) {
        Logger::debug() << "\t(" << can.x << ", " << can.y << ")";
    }

    // ====================================================================================
    // 第一轮：优先对所有水平对寻找三个点亮度相同的geo（这一轮不降低阈值）
    // ====================================================================================
    Logger::debug() << "[5 Find Geometry] Phase 1 Start. Finding horizontal pairs with same brightness.";
    for (size_t i = 0; i < glint_candidates.size(); i++)
    {
        cv::Point2f temp_pt_1 = glint_candidates[i];
        for (size_t j = i + 1; j < glint_candidates.size(); j++)
        {
            cv::Point2f temp_pt_2 = glint_candidates[j];
            cv::Point2f l_pt = temp_pt_1.x < temp_pt_2.x ? temp_pt_1 : temp_pt_2;
            cv::Point2f r_pt = temp_pt_1.x > temp_pt_2.x ? temp_pt_1 : temp_pt_2;

            if (side2side(l_pt, r_pt))
            {
                Logger::debug() << "[5 Find Geometry] Phase 1 found horizontal pair" << "\n"
                                << "\tleft: (" << l_pt.x << ", " << l_pt.y << ")\n"
                                << "\tright: (" << r_pt.x << ", " << r_pt.y << ")";

                bool pair_found_mid = false;
                bool pair_failed_check = false;

                for (size_t k = 0; k < glint_candidates.size(); k++)
                {
                    if (k == i || k == j) continue;

                    cv::Point2f m_pt = glint_candidates[k];

                    if (side2mid(l_pt, r_pt, m_pt))
                    {
                        double bg_brightness;
                        bool is_valid = checkGlintGeometry(l_pt, r_pt, m_pt, bg_brightness);

                        if (is_valid)
                        {
                            Logger::debug() << "[5 Find Geometry] Phase 1 found mid point" << "\n"
                                            << "\tmid: (" << m_pt.x << ", " << m_pt.y << ")";
                            
                            GlintGeometry geo;
                            geo.l_pt = l_pt;
                            geo.r_pt = r_pt;
                            geo.m_pt = m_pt;
                            geo.bg_brightness = bg_brightness;
                            geo.on_cornea = true;
                            geo.found_threshold = current_thr;

                            found_geometries.push_back(geo);
                            pair_found_mid = true;
                        }
                        else
                        {
                            pair_failed_check = true;
                        }
                    }
                }

                if (!pair_found_mid)
                {
                    if (pair_failed_check) {
                        bad_pairs.push_back({l_pt, r_pt});
                    } else {
                        candidate_pairs.push_back({l_pt, r_pt});
                    }
                }
            }
        }
    }

    if (!found_geometries.empty())
    {
        Logger::debug() << "[5 Find Geometry] Phase 1 Success. Found " 
                        << found_geometries.size() << " geometries. Returning all.";
        return found_geometries;
    }

    // ====================================================================================
    // 第二轮：如果没有找到 geo，开始对候选水平对降低阈值进行搜索。
    // ====================================================================================
    Logger::debug() << "\n[5 Find Geometry] Phase 2 Start. Try to find missing mid point for candidate pairs.";
    GlintGeometry best_geo;
    best_geo.found_threshold = -1.0;

    for (const auto& pair : candidate_pairs)
    {
        cv::Point2f l_pt = pair.first;
        cv::Point2f r_pt = pair.second;

        Logger::debug() << "[5 Find Geometry] Phase 2 try to find missing mid point for: ";
        Logger::debug() << "\tleft: (" << l_pt.x << ", " << l_pt.y << ")\n"
                        << "\tright: (" << r_pt.x << ", " << r_pt.y << ")";

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << "side_and_side_" << l_pt.x << "_" << l_pt.y
            << "_" << r_pt.x << "_" << r_pt.y;
        std::string pt_str = oss.str();

        // --- 核心修改：接收碎片阵列 ---
        auto search_rects = getSearchRegionSideAndSide(l_pt, r_pt, cluster);
        if (search_rects.empty()) continue;

        double search_thr = current_thr - threshold_step_;
        bool found_for_this_pair = false;
        std::vector<cv::Point2f> roi_glints = {l_pt, r_pt};

        while (search_thr >= mini_threshold_ && !found_for_this_pair)
        {
            std::vector<cv::Point2f> all_contourCenters;
            
            // --- 核心修改：遍历无虹膜反光的碎片切片，汇总找出的候选点 ---
            for (const auto& rect : search_rects) {
                cv::Mat roi_img = abs_dst_(rect);
                cv::Point2f roi_offset(rect.x, rect.y);
                auto pts = searchGlintsInROI(roi_img, roi_offset, roi_glints, search_thr, pt_str);
                all_contourCenters.insert(all_contourCenters.end(), pts.begin(), pts.end());
            }

            for (const auto& m_pt : all_contourCenters)
            {
                roi_glints.push_back(m_pt);

                Logger::debug() << "[5 Find Geometry] Phase 2 Checking potential missing mid point at: \n\t("
                                << m_pt.x << ", " << m_pt.y << ")";

                if (side2mid(l_pt, r_pt, m_pt))
                {
                    double bg_brightness;
                    bool is_valid = checkGlintGeometry(l_pt, r_pt, m_pt, bg_brightness);

                    if (is_valid)
                    {
                        Logger::debug() << "[5 Find Geometry] Phase 2 found missing mid point";
                        GlintGeometry geo;
                        geo.l_pt = l_pt;
                        geo.r_pt = r_pt;
                        geo.m_pt = m_pt;
                        geo.bg_brightness = bg_brightness;
                        geo.on_cornea = true;
                        geo.found_threshold = search_thr;

                        if (search_thr > best_geo.found_threshold)
                        {
                            best_geo = geo;
                        }
                        found_for_this_pair = true;
                        break; 
                    }
                    else
                    {
                        Logger::debug() << "[5 Find Geometry] Phase 2 missing mid failed check";
                    }
                }
                else
                {
                    Logger::debug() << "[5 Find Geometry] Phase 2 not found missing mid point";
                }
            }

            search_thr -= threshold_step_;
        }
    }

    if (best_geo.found_threshold > 0)
    {
        Logger::debug() << "[5 Find Geometry] Phase 2 Success. Best threshold=" << best_geo.found_threshold;
        found_geometries.push_back(best_geo);
        return found_geometries;
    }

    // ====================================================================================
    // 第三轮：如果没有找到geo，则开始寻找side-mid pair
    // ====================================================================================
    GlintGeometry best_geo_phase3;
    best_geo_phase3.found_threshold = -1.0;

    auto is_bad_point = [&](const cv::Point2f& pt) {
        const float EPSILON = 1e-4f;
        for (const auto& bp : bad_pairs) {
            if (cv::norm(bp.first - pt) < EPSILON || cv::norm(bp.second - pt) < EPSILON) {
                return true;
            }
        }
        return false;
    };

    Logger::debug() << "\n[5 Find Geometry] Phase 3 Start. Try to find side and mid pair";

    for (size_t i = 0; i < glint_candidates.size(); i++)
    {
        cv::Point2f temp_pt_1 = glint_candidates[i];
        for (size_t j = i + 1; j < glint_candidates.size(); j++)
        {
            cv::Point2f temp_pt_2 = glint_candidates[j];
            cv::Point2f s_pt = temp_pt_1.y < temp_pt_2.y ? temp_pt_1 : temp_pt_2;
            cv::Point2f m_pt = temp_pt_1.y > temp_pt_2.y ? temp_pt_1 : temp_pt_2;

            if (isGlintGeometryRepeated(s_pt, m_pt, existing_geometries)) continue;

            if (is_bad_point(s_pt) || is_bad_point(m_pt)) {
                Logger::debug() << "[5 Find Geometry] Phase 3: Skipping point rejected in Phase 1.";
                continue;
            }

            if (side2mid(s_pt, m_pt))
            {
                Logger::debug() << "[5 Find Geometry] Phase 3 found side and mid pair" << "\n"
                                << "side: (" << s_pt.x << ", " << s_pt.y << ")\n"
                                << "mid: (" << m_pt.x << ", " << m_pt.y << ")";

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1) << "side_and_mid_" << s_pt.x << "_" << s_pt.y
                    << "_" << m_pt.x << "_" << m_pt.y;
                std::string pt_str = oss.str();

                // --- 核心修改：接收碎片阵列 ---
                auto search_rects = getSearchRegionSideAndMid(s_pt, m_pt, cluster);
                if (search_rects.empty()) continue;

                double search_thr = current_thr - threshold_step_;
                bool found_for_this_pair = false;
                std::vector<cv::Point2f> roi_glints = {s_pt, m_pt}; 

                while (search_thr >= mini_threshold_ && !found_for_this_pair)
                {
                    std::vector<cv::Point2f> all_contourCenters;
                    
                    // --- 核心修改：遍历无虹膜反光的碎片切片，汇总找出的候选点 ---
                    for (const auto& rect : search_rects) {
                        cv::Mat roi_img = abs_dst_(rect);
                        cv::Point2f roi_offset(rect.x, rect.y);
                        auto pts = searchGlintsInROI(roi_img, roi_offset, roi_glints, search_thr, pt_str);
                        all_contourCenters.insert(all_contourCenters.end(), pts.begin(), pts.end());
                    }

                    for (const auto& new_pt : all_contourCenters)
                    {
                        roi_glints.push_back(new_pt);

                        cv::Point2f l_pt = new_pt.x < s_pt.x ? new_pt : s_pt;
                        cv::Point2f r_pt = new_pt.x > s_pt.x ? new_pt : s_pt;

                        Logger::debug() << "[5 Find Geometry] Phase 3 Checking potential missing side point at: \n\t("
                                        << new_pt.x << ", " << new_pt.y << ")";

                        if (side2mid(l_pt, r_pt, m_pt))
                        {
                            double bg_brightness;
                            bool is_valid = checkGlintGeometry(l_pt, r_pt, m_pt, bg_brightness);

                            if (is_valid)
                            {
                                Logger::debug() << "[5 Find Geometry] Phase 3 found missing side point";
                                GlintGeometry geo;
                                geo.l_pt = l_pt;
                                geo.r_pt = r_pt;
                                geo.m_pt = m_pt;
                                geo.bg_brightness = bg_brightness;
                                geo.on_cornea = true;
                                geo.found_threshold = search_thr;

                                if (search_thr > best_geo_phase3.found_threshold) {
                                    best_geo_phase3 = geo;
                                }
                                found_for_this_pair = true; 
                                break;
                            }
                            else
                            {
                                Logger::debug() << "[5 Find Geometry] Phase 3 missing side failed check";
                            }
                        }
                        else
                        {
                            Logger::debug() << "[5 Find Geometry] Phase 3 not found missing side point";
                        }
                    }
                    search_thr -= threshold_step_;
                }
            }
        }
    }

    if (best_geo_phase3.found_threshold > 0)
    {
        Logger::debug() << "[5 Find Geometry] Phase 3 Success. Best threshold=" << best_geo_phase3.found_threshold;
        found_geometries.push_back(best_geo_phase3);
        return found_geometries;
    }

    // ====================================================================================
    // 如果三轮都没有结果，返回空
    // ====================================================================================
    Logger::debug() << "[5 Find Geometry] No geometry found in all phases.";
    return found_geometries;
}

bool GlintDetector::isGlintRepeated(
    const std::vector<cv::Point2f>& roi_glints, 
    const cv::Point2f& glint
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 判定两个 glint 是否为同一个的最短欧氏距离阈值
    const double kMinGlintDist = emp_cfg_["isGlintRepeated"]["kMinGlintDist"].as<double>();

    if (roi_glints.empty()) return false;

    for (const auto& roi_glint : roi_glints)
    {
        // if distance between glints is less than kMinGlintDist, consider them as the same glint
        if (cv::norm(roi_glint - glint) < kMinGlintDist)
        {
            return true;
        }
    }

    return false;
}

bool GlintDetector::isGlintGeometryRepeated(
    const GlintGeometry& geo,
    const std::vector<GlintGeometry>& existing
)
{
    // --- 超参数声明 (Hyperparameters) ---
    // 判定两个 Glint 几何点位置相同的容差距离
    const double kPointMatchTolerance = emp_cfg_["isGlintGeometryRepeated"]["kPointMatchTolerance"].as<double>();

    if (existing.empty()) return false;

    for (const auto& existing_geo : existing)
    {
        if   (((cv::norm(geo.l_pt - existing_geo.l_pt) <= kPointMatchTolerance)
             && (cv::norm(geo.r_pt - existing_geo.r_pt) <= kPointMatchTolerance)
             && (cv::norm(geo.m_pt - existing_geo.m_pt) <= kPointMatchTolerance))

             || ((cv::norm(geo.center() - existing_geo.center()) <= kPointMatchTolerance))) {
                return true;
            }
    }
    return false;
}

bool GlintDetector::isGlintGeometryRepeated(
    const cv::Point2f& s_pt, 
    const cv::Point2f& m_pt,
    const std::vector<GlintGeometry>& existing
)
{
    const double kPointMatchTolerance = emp_cfg_["isGlintGeometryRepeated"]["kPointMatchTolerance"].as<double>();

    if (existing.empty()) return false;

    for (const auto& geo : existing)
    {
        if (cv::norm(geo.m_pt - m_pt) <= kPointMatchTolerance) {
            if ((cv::norm(geo.l_pt - s_pt) <= kPointMatchTolerance) ||
                (cv::norm(geo.r_pt - s_pt) <= kPointMatchTolerance)) {
                return true;
            }
        }
    }
    return false;
}

bool GlintDetector::checkGlintGeometry(
    const cv::Point2f& l_pt,
    const cv::Point2f& r_pt,
    const cv::Point2f& m_pt,
    double& out_bg_brightness
)
{
    auto spec = spec_glint_cfg_["checkGlintGeometry"];
    auto emp = emp_cfg_["checkGlintGeometry"];

    const double kBrightnessThreshold = spec["kBrightnessThreshold"].as<double>();
    const int kMinPadding = emp["kMinPadding"].as<int>();
    const float kPaddingRatio = emp["kPaddingRatio"].as<float>();
    const double kDangerMaskThresh = emp["kDangerMaskThresh"].as<double>(); 
    const int kDilateKernelSize = emp["kDilateKernelSize"].as<int>();
    const int kDilateIterations = emp["kDilateIterations"].as<int>();
    const size_t kMinValidPixels = emp["kMinValidPixels"].as<int>();
    const float kTrimRatio = emp["kTrimRatio"].as<float>();

    std::vector<cv::Point2f> pts = {l_pt, r_pt, m_pt};
    cv::Rect roi = cv::boundingRect(pts);
    int pad_x = std::max(kMinPadding, static_cast<int>(roi.width * kPaddingRatio));
    int pad_y = std::max(kMinPadding, static_cast<int>(roi.height * kPaddingRatio));

    roi.x -= pad_x;
    roi.y -= pad_y;
    roi.width += (pad_x * 2);
    roi.height += (pad_y * 2);

    roi &= cv::Rect(0, 0, gray_.cols, gray_.rows);
    if (roi.area() <= 0) return false;

    cv::Mat roi_img = gray_(roi);
    cv::Mat danger_mask;

    cv::threshold(roi_img, danger_mask, kDangerMaskThresh, 255, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kDilateKernelSize, kDilateKernelSize));
    cv::dilate(danger_mask, danger_mask, kernel, cv::Point(-1, -1), kDilateIterations);

    std::vector<uchar> valid_pixels;
    valid_pixels.reserve(roi.area());

    for (int y = 0; y < roi_img.rows; ++y) {
        const uchar* img_ptr = roi_img.ptr<uchar>(y);
        const uchar* mask_ptr = danger_mask.ptr<uchar>(y);
        for (int x = 0; x < roi_img.cols; ++x) {
            if (mask_ptr[x] == 0) {
                valid_pixels.push_back(img_ptr[x]);
            }
        }
    }

    if (valid_pixels.empty()) {
        out_bg_brightness = 255.0;
        return false;
    } 
    else if (valid_pixels.size() < kMinValidPixels) {
        double sum = 0;
        for (auto v : valid_pixels) sum += v;
        out_bg_brightness = sum / valid_pixels.size();
        return (out_bg_brightness <= kBrightnessThreshold);
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
        out_bg_brightness = sum / static_cast<double>(n - trim_count * 2);
        return (out_bg_brightness <= kBrightnessThreshold);
    }
}

std::vector<std::vector<cv::Point2f>>
GlintDetector::glintGeometryListToGlintVector(const std::vector<GlintDetector::GlintGeometry>& glint_geometry)
{
    std::vector<std::vector<cv::Point2f>> glint_vectors;
    for (const auto& geo : glint_geometry) {
        cv::Point2f offset(0.5f, 0.5f);
        std::vector<cv::Point2f> glint_vector = {
            geo.l_pt + offset, 
            geo.r_pt + offset, 
            geo.m_pt + offset
        };
        glint_vectors.push_back(glint_vector);
    }
    return glint_vectors;
}

float GlintDetector::getBilinearSubpixel(const cv::Mat& img, const cv::Point2f& pt) const
{
    int x = static_cast<int>(std::floor(pt.x));
    int y = static_cast<int>(std::floor(pt.y));

    // 边界保护
    if (x < 0 || x >= img.cols - 1 || y < 0 || y >= img.rows - 1) {
        return 0.0f; 
    }

    float dx = pt.x - x;
    float dy = pt.y - y;

    float p1 = img.at<uchar>(y, x);
    float p2 = img.at<uchar>(y, x + 1);
    float p3 = img.at<uchar>(y + 1, x);
    float p4 = img.at<uchar>(y + 1, x + 1);

    return p1 * (1.0f - dx) * (1.0f - dy) + 
           p2 * dx * (1.0f - dy) + 
           p3 * (1.0f - dx) * dy + 
           p4 * dx * dy;
}

// 辅助函数：角度归一化到 [0, 360)
inline float normalizeAngle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

std::vector<cv::Point2f> GlintDetector::samplePupilEdgesByRayCasting(
    const cv::Mat& gray_img, 
    const cv::Point2f& rough_center, 
    float rough_major_axis,
    const GlintGeometry& glint_geo)
{
    // =========================================================================
    // [Dynamic Hyperparameters Calculation]
    // =========================================================================
    CfgNode emp = emp_cfg_["samplePupilEdgesByRayCasting"];
    const int   kRayCount          = emp["kRayCount"].as<int>();
    const int   kGradientGap       = emp["kGradientGap"].as<int>();
    const float kMinGradient       = emp["kMinGradient"].as<float>();
    const double kMaxPupilBrightness = spec_pupil_cfg_["searchPupilInROI"]["kMaxDarkness"].as<double>();
    const double kMaxGlintBrightness = emp["kMaxGlintBrightness"].as<double>();
    const double kRayMaxRadiusRatio  = emp["kRayMaxRadiusRatio"].as<double>();

    // 估算光斑的物理半径 (基于瞳孔大小的 10% 作为一个稳健先验，外加 2 像素冗余)
    const float estimated_glint_radius = (rough_major_axis * 0.20f) + 2.0f;
    const float angle_offset_deg = 5.0f;  // 额外的角度安全偏移量
    const float radial_offset_px = 5.0f; // 额外的径向安全偏移量

    // =========================================================================

    std::vector<cv::Point2f> raw_edges;
    float r_max = rough_major_axis * 0.5 * kRayMaxRadiusRatio;

    // A. 预计算每个光斑的危险扇区范围和判定深度
    struct DangerZone {
        float center_angle;
        float half_width;
        float exclusion_dist;
    };
    std::vector<DangerZone> danger_zones;
    std::vector<cv::Point2f> glints = {glint_geo.l_pt, glint_geo.r_pt, glint_geo.m_pt};

    for (const auto& gp : glints) {
        float dist = cv::norm(gp - rough_center);
        if (dist < 1e-3f) continue;

        float angle = std::atan2(gp.y - rough_center.y, gp.x - rough_center.x) * 180.0f / CV_PI;
        
        DangerZone dz;
        dz.center_angle = normalizeAngle(angle);
        
        // ★ 动态角度半宽：atan2(半径, 距离)
        dz.half_width = std::atan2(estimated_glint_radius, dist) * 180.0f / CV_PI + angle_offset_deg;
        
        // ★ 动态判定深度：光斑到中心的距离 - 瞳孔预期半径 + 光斑直径跨度
        // 确保扫描能覆盖到光斑核心之后
        dz.exclusion_dist = (dist - (rough_major_axis * 0.5f)) + (estimated_glint_radius * 2.0f) + radial_offset_px;
        
        danger_zones.push_back(dz);
    }

    for (int i = 0; i < kRayCount; ++i)
    {
        float current_angle_deg = i * 360.0f / kRayCount;
        float angle_rad = current_angle_deg * CV_PI / 180.0f;
        float cos_a = std::cos(angle_rad);
        float sin_a = std::sin(angle_rad);

        // B. 动态匹配当前射线所属的危险扇区
        float active_exclusion_dist = 0.0f;
        bool in_danger_sector = false;
        for (const auto& dz : danger_zones) {
            float diff = std::abs(current_angle_deg - dz.center_angle);
            if (diff > 180.0f) diff = 360.0f - diff; 
            if (diff < dz.half_width) {
                in_danger_sector = true;
                active_exclusion_dist = dz.exclusion_dist;
                break;
            }
        }

        std::vector<float> intensities;
        std::vector<cv::Point2f> points;
        for (float r = 0.0f; r <= r_max; r += 1.0f) {
            cv::Point2f pt(rough_center.x + r * cos_a, rough_center.y + r * sin_a);
            points.push_back(pt);
            intensities.push_back(getBilinearSubpixel(gray_img, pt));
        }

        if (intensities.size() <= kGradientGap * 2) continue;

        int last_pupil_idx = -1;
        for (int j = 0; j < (int)intensities.size(); ++j) {
            if (intensities[j] < kMaxPupilBrightness) last_pupil_idx = j;
        }
        if (last_pupil_idx == -1) continue;

        // C. 执行动态判别
        if (in_danger_sector) {
            bool glint_nearby = false;
            // 使用该扇区特有的动态判定深度
            int check_limit = std::min(last_pupil_idx + (int)active_exclusion_dist, (int)intensities.size());
            for (int k = last_pupil_idx + 1; k < check_limit; ++k) {
                if (intensities[k] >= kMaxGlintBrightness) {
                    glint_nearby = true;
                    break;
                }
            }
            if (glint_nearby) continue; 
        }

        if (last_pupil_idx < (int)intensities.size() - kGradientGap) {
            float val_back = intensities[last_pupil_idx];
            float val_front = intensities[last_pupil_idx + kGradientGap];
            if (val_front - val_back > kMinGradient) {
                raw_edges.push_back(points[last_pupil_idx]);
            }
        }
    }
    return raw_edges;
}

std::vector<cv::Point2f> GlintDetector::filterPupilEdgePoints(
    const std::vector<cv::Point2f>& raw_edges, 
    const cv::Point2f& rough_center,
    const GlintGeometry& glint_geo)
{
    if (raw_edges.empty()) return {};

    // =========================================================================
    // [Hyperparameters for Edge Filtering]
    // =========================================================================
    CfgNode emp = emp_cfg_["refinePupil"];
    const float kInwardToleranceRatio = 0.88f; // 危险扇区内允许的最小半径比例 (0.88-0.92 较稳健)
    const float kDangerHalfWidth      = 30.0f; // 过滤时的危险扇区稍微设宽一点，确保覆盖完全
    // =========================================================================

    // 1. 分类点：安全区点 vs 危险区点
    std::vector<float> danger_angles;
    std::vector<cv::Point2f> glint_pts = {glint_geo.l_pt, glint_geo.r_pt, glint_geo.m_pt};
    for (const auto& gp : glint_pts) {
        float angle = std::atan2(gp.y - rough_center.y, gp.x - rough_center.x) * 180.0f / CV_PI;
        danger_angles.push_back(normalizeAngle(angle));
    }

    std::vector<float> safe_distances;
    struct EdgeInfo {
        cv::Point2f pt;
        float dist;
        bool in_danger;
    };
    std::vector<EdgeInfo> processed_edges;

    for (const auto& pt : raw_edges) {
        float angle = std::atan2(pt.y - rough_center.y, pt.x - rough_center.x) * 180.0f / CV_PI;
        float norm_angle = normalizeAngle(angle);

        bool in_danger = false;
        for (float da : danger_angles) {
            float diff = std::abs(norm_angle - da);
            if (diff > 180.0f) diff = 360.0f - diff;
            if (diff < kDangerHalfWidth) {
                in_danger = true;
                break;
            }
        }

        float d = cv::norm(pt - rough_center);
        processed_edges.push_back({pt, d, in_danger});
        if (!in_danger) {
            safe_distances.push_back(d);
        }
    }

    // 2. 计算安全区的稳健中值半径 (作为全局参考)
    if (safe_distances.size() < 5) return raw_edges; // 安全点太少，放弃过滤防止误杀

    std::sort(safe_distances.begin(), safe_distances.end());
    float median_radius = safe_distances[safe_distances.size() / 2];

    // 3. 执行定向剔除
    std::vector<cv::Point2f> clean_edges;
    for (const auto& edge : processed_edges) {
        if (edge.in_danger) {
            // ★ 核心逻辑：如果在危险区且半径明显偏小（向内凹陷），则剔除
            if (edge.dist > median_radius * kInwardToleranceRatio) {
                clean_edges.push_back(edge.pt);
            } else {
                // 这个点被判定为光斑引起的“假边缘”，跳过不加入 clean_edges
                if (local_debug_) {
                    Logger::debug() << "[FilterEdges] Point at dist " << edge.dist 
                                    << " (ref: " << median_radius << ") rejected.";
                }
            }
        } else {
            // 安全区的点无条件保留（或可增加一个基础的统计过滤）
            clean_edges.push_back(edge.pt);
        }
    }

    // 可视化过滤后的点 (青色)
    if (viz_) {
        for (const auto& pt : clean_edges) {
            cv::circle(debug_imgs_[5], pt, 1, cv::Scalar(255, 255, 0), -1);
        }
    }

    return clean_edges;
}

cv::RotatedRect GlintDetector::fitEllipseRANSAC(const std::vector<cv::Point2f>& clean_edges) 
{
    // RANSAC 参数
    const int   kRansacIterations = 250; 
    const float kInlierTolerance   = 1.5f; // 内点判定的距离容差（像素）

    if (clean_edges.size() < 6) return cv::RotatedRect();

    cv::RNG rng(cv::getTickCount());
    int best_inlier_count = 0;
    std::vector<cv::Point2f> best_inlier_set;

    for (int iter = 0; iter < kRansacIterations; ++iter) 
    {
        // 1. 随机抽取 5 个点（拟合椭圆的最小必要点数）
        std::vector<cv::Point2f> sample(5);
        for (int i = 0; i < 5; ++i) {
            sample[i] = clean_edges[rng.uniform(0, (int)clean_edges.size())];
        }

        // 2. 拟合候选椭圆（不加任何限制）
        cv::RotatedRect test_ellipse = cv::fitEllipse(sample);
        
        // 基础合法性检查（防止数学奇异）
        if (test_ellipse.size.width <= 0 || test_ellipse.size.height <= 0) continue;

        // 3. 计算内点集
        std::vector<cv::Point2f> current_inliers;
        float a = test_ellipse.size.width / 2.0f;
        float b = test_ellipse.size.height / 2.0f;
        float angle_rad = test_ellipse.angle * CV_PI / 180.0f;
        float cos_a = std::cos(-angle_rad);
        float sin_a = std::sin(-angle_rad);

        for (const auto& pt : clean_edges) {
            float dx = pt.x - test_ellipse.center.x;
            float dy = pt.y - test_ellipse.center.y;
            float lx = dx * cos_a - dy * sin_a;
            float ly = dx * sin_a + dy * cos_a;

            float pt_dist = std::sqrt(lx * lx + ly * ly);
            float phi = std::atan2(ly, lx);
            float r_theory = (a * b) / std::sqrt(std::pow(b * std::cos(phi), 2) + std::pow(a * std::sin(phi), 2));

            // 如果观测点到椭圆边缘的距离小于容差，计为内点
            if (std::abs(pt_dist - r_theory) < kInlierTolerance) {
                current_inliers.push_back(pt);
            }
        }

        // 4. 保存拥有最大内点集的模型
        if ((int)current_inliers.size() > best_inlier_count) {
            best_inlier_count = (int)current_inliers.size();
            best_inlier_set = current_inliers;
        }
    }

    // 5. 【核心改进】使用找到的所有内点进行最终拟合
    // 这样能保证椭圆不是只根据随机的5个点生成的，而是根据那 270 度所有的青色点生成的
    if (best_inlier_count >= 6) {
        return cv::fitEllipse(best_inlier_set);
    }

    return cv::RotatedRect(); 
}

GlintDetector::Pupil GlintDetector::refinePupil(
    const cv::Mat& gray_img, 
    const Pupil& rough_pupil, 
    const GlintGeometry& glint_geo)
{
    Logger::debug() << "[RefinePupil] Start refining pupil for center: (" 
                    << rough_pupil.rr.center.x << ", " << rough_pupil.rr.center.y << ")";

    // 如果原先长轴无效，保护退出
    if (rough_pupil.major_axis <= 0) return rough_pupil;

    // 1. 发散射线，采样边缘
    std::vector<cv::Point2f> raw_edges = samplePupilEdgesByRayCasting(
        gray_img, rough_pupil.rr.center, rough_pupil.major_axis, glint_geo);

    Logger::debug() << "Raw edges count: " << raw_edges.size();

    // 2. 根据 Glint 和统计学过滤坏点
    std::vector<cv::Point2f> clean_edges = filterPupilEdgePoints(
        raw_edges, rough_pupil.rr.center, glint_geo);

    Logger::debug() << "Clean edges count: " << clean_edges.size();

    // 3. RANSAC 鲁棒拟合
    cv::RotatedRect refined_rect = fitEllipseRANSAC(clean_edges);

    // 4. 判断拟合是否成功，并组装结果
    if (refined_rect.size.width > 0 && refined_rect.size.height > 0) 
    {
        Pupil refined = rough_pupil; // 继承原本的 darkness 等统计信息
        refined.rr = refined_rect;
        refined.major_axis = std::max(refined_rect.size.width, refined_rect.size.height);
        refined.minor_axis = std::min(refined_rect.size.width, refined_rect.size.height);
        
        Logger::debug() << "[RefinePupil] Refined center: (" 
                        << refined.rr.center.x << ", " << refined.rr.center.y << ")";

        if (viz_) {
            // 可视化：用粗黄线画出 Refined 后的完美瞳孔
            cv::ellipse(debug_imgs_[5], rough_pupil.rr, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
            cv::ellipse(debug_imgs_[5], refined.rr, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            cv::Point2f x_offset(3.0, 0.0);
            cv::Point2f y_offset(0.0, 3.0);
            cv::line(debug_imgs_[5], refined.rr.center - x_offset, refined.rr.center + x_offset, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            cv::line(debug_imgs_[5], refined.rr.center - y_offset, refined.rr.center + y_offset, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
        return refined;
    }

    Logger::debug() << "[RefinePupil] Refinement failed or rejected, returning rough pupil.";
    return rough_pupil; // 退回原粗略结果
}

cv::Point3f GlintDetector::fitCircleRANSAC(const std::vector<cv::Point2f>& points, int max_iters, float tolerance, float max_radius) 
{
    if (points.size() < 3) return cv::Point3f(0, 0, 0);

    cv::RNG rng(cv::getTickCount());
    int best_inlier_count = 0;
    cv::Point3f best_circle(0, 0, 0);
    std::vector<cv::Point2f> best_inliers; // ✨ 记录最优内点集

    for (int i = 0; i < max_iters; ++i) {
        cv::Point2f p1 = points[rng.uniform(0, (int)points.size())];
        cv::Point2f p2 = points[rng.uniform(0, (int)points.size())];
        cv::Point2f p3 = points[rng.uniform(0, (int)points.size())];

        // 求解外接圆
        float D = 2.0f * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
        if (std::abs(D) < 1e-5f) continue;

        float cx = ((p1.x*p1.x + p1.y*p1.y) * (p2.y - p3.y) + (p2.x*p2.x + p2.y*p2.y) * (p3.y - p1.y) + (p3.x*p3.x + p3.y*p3.y) * (p1.y - p2.y)) / D;
        float cy = ((p1.x*p1.x + p1.y*p1.y) * (p3.x - p2.x) + (p2.x*p2.x + p2.y*p2.y) * (p1.x - p3.x) + (p3.x*p3.x + p3.y*p3.y) * (p2.x - p1.x)) / D;
        float r = std::sqrt((p1.x - cx)*(p1.x - cx) + (p1.y - cy)*(p1.y - cy));

        // 约束限制
        if (r < 1.0f || r > max_radius) continue;

        // ✨ 收集当前模型的内点
        std::vector<cv::Point2f> current_inliers;
        for (const auto& pt : points) {
            float dist = std::sqrt((pt.x - cx)*(pt.x - cx) + (pt.y - cy)*(pt.y - cy));
            if (std::abs(dist - r) < tolerance) {
                current_inliers.push_back(pt);
            }
        }

        // 择优
        if ((int)current_inliers.size() > best_inlier_count) {
            best_inlier_count = (int)current_inliers.size();
            best_inliers = current_inliers;
            best_circle = cv::Point3f(cx, cy, r);
        }
    }

    // ====================================================================================
    // 🌟 核心改进：和 refinePupil 一样，使用收集到的*所有*最优内点进行全局最小二乘重拟合
    // 引入 Mean-Centering (坐标去均值化) + double 精度，绝对防止高次项运算导致浮点溢出/发散
    // ====================================================================================
    if (best_inlier_count >= 3) {
        double sum_x = 0, sum_y = 0;
        int n = best_inliers.size();
        for (const auto& pt : best_inliers) {
            sum_x += pt.x;
            sum_y += pt.y;
        }
        double mean_x = sum_x / n;
        double mean_y = sum_y / n;

        // 累加中心化后的高次矩
        double su2 = 0, sv2 = 0, suv = 0, su3 = 0, sv3 = 0, su2v = 0, suv2 = 0;
        for (const auto& pt : best_inliers) {
            double u = pt.x - mean_x;
            double v = pt.y - mean_y;
            double u2 = u * u;
            double v2 = v * v;
            
            su2 += u2;
            sv2 += v2;
            suv += u * v;
            su3 += u2 * u;
            sv3 += v2 * v;
            su2v += u2 * v;
            suv2 += u * v2;
        }

        // 构造线性方程组
        double c_x = su2;
        double c_y = sv2;
        double c_xy = suv;
        double d_x = 0.5 * (su3 + suv2);
        double d_y = 0.5 * (sv3 + su2v);

        // 求解
        double det = c_x * c_y - c_xy * c_xy;
        if (std::abs(det) > 1e-7) {
            double uc = (d_x * c_y - d_y * c_xy) / det;
            double vc = (c_x * d_y - c_xy * d_x) / det;
            
            // 还原到真实坐标系
            double cx = uc + mean_x;
            double cy = vc + mean_y;
            
            // 计算基于所有点的平均拟合半径
            double r2 = 0;
            for (const auto& pt : best_inliers) {
                r2 += (pt.x - cx) * (pt.x - cx) + (pt.y - cy) * (pt.y - cy);
            }
            double r = std::sqrt(r2 / n);
            
            // 最终约束，确保不会因为奇异点阵列拟合出无穷大直线
            if (r >= 1.0f && r <= max_radius) {
                best_circle = cv::Point3f(static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(r));
            }
        }
    }

    return best_circle;
}

void GlintDetector::refineGlintGeometry(const cv::Mat& gray_img, GlintGeometry& geo, int debug_img_idx)
{
    // --- 1. 超参数与配置读取 ---
    CfgNode emp = emp_cfg_["refineGlintPoint"]; 
    const int sr_scale         = emp["sr_scale"].as<int>();
    const int kRayCount        = emp["kRayCount"].as<int>();
    const int kGradientGap     = emp["kGradientGap"].as<int>();
    const float kMinGradient   = emp["kMinGradient"].as<float>();
    const int kRansacIters     = emp["kRansacIterations"].as<int>();
    const float kInlierTol     = emp["kInlierTolerance"].as<float>();
    
    // ✨ 新增亮度和半径约束的超参数
    const float kMinBrightness = emp["kMinBrightness"].as<float>(); 
    const float kRadiusMargin  = emp["kRadiusMargin"].as<float>(); 

    const int viz_mult         = emp["viz_mult"].as<int>();
    const int total_scale      = sr_scale * viz_mult;

    // --- 2. 计算统一的 ROI 包围盒 ---
    float min_x = std::min({geo.l_pt.x, geo.r_pt.x, geo.m_pt.x});
    float max_x = std::max({geo.l_pt.x, geo.r_pt.x, geo.m_pt.x});
    float min_y = std::min({geo.l_pt.y, geo.r_pt.y, geo.m_pt.y});
    float max_y = std::max({geo.l_pt.y, geo.r_pt.y, geo.m_pt.y});

    int pad = 15; // 针对三个光斑整体外扩 15 原图像素
    cv::Rect roi(
        std::max(0, static_cast<int>(min_x) - pad),
        std::max(0, static_cast<int>(min_y) - pad),
        static_cast<int>(max_x - min_x) + pad * 2,
        static_cast<int>(max_y - min_y) + pad * 2
    );
    roi &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);
    if (roi.empty()) return;

    // --- 3. 执行单次超分辨率计算 ---
    cv::Mat sr_patch;
    cv::resize(gray_img(roi), sr_patch, cv::Size(), sr_scale, sr_scale, cv::INTER_LANCZOS4);

// --- 4. 初始化 2x2 组合四象限局部高清画布 ---
    int W = roi.width * total_scale;
    int H = roi.height * total_scale;

    if (viz_) {
        if (debug_imgs_.size() <= debug_img_idx) debug_imgs_.resize(debug_img_idx + 1);
        
        // 创建 2W x 2H 的画布
        debug_imgs_[debug_img_idx] = cv::Mat::zeros(H * 2, W * 2, CV_8UC3);

        cv::Mat raw_roi = gray_img(roi);

        // [TL 左上角] 填充超分之前的原始 ROI
        for (int y = 0; y < raw_roi.rows; ++y) {
            const uchar* raw_ptr = raw_roi.ptr<uchar>(y);
            for (int x = 0; x < raw_roi.cols; ++x) {
                cv::Vec3b color(raw_ptr[x], raw_ptr[x], raw_ptr[x]);
                cv::Rect tl_rect(x * total_scale, y * total_scale, total_scale, total_scale);
                cv::rectangle(debug_imgs_[debug_img_idx], tl_rect, color, cv::FILLED);
            }
        }

        // [TR 右上角]、[BL 左下角]、[BR 右下角] 填充超分之后的 sr_patch
        for (int y = 0; y < sr_patch.rows; ++y) {
            const uchar* sr_row_ptr = sr_patch.ptr<uchar>(y);
            int base_y = y * viz_mult;
            for (int x = 0; x < sr_patch.cols; ++x) {
                int base_x = x * viz_mult;
                cv::Vec3b color(sr_row_ptr[x], sr_row_ptr[x], sr_row_ptr[x]);
                for (int dy = 0; dy < viz_mult; ++dy) {
                    cv::Vec3b* dst_row_TR = debug_imgs_[debug_img_idx].ptr<cv::Vec3b>(base_y + dy);
                    cv::Vec3b* dst_row_BL = debug_imgs_[debug_img_idx].ptr<cv::Vec3b>(base_y + dy + H);
                    cv::Vec3b* dst_row_BR = debug_imgs_[debug_img_idx].ptr<cv::Vec3b>(base_y + dy + H);
                    for (int dx = 0; dx < viz_mult; ++dx) {
                        dst_row_TR[base_x + dx + W] = color;
                        dst_row_BL[base_x + dx] = color;
                        dst_row_BR[base_x + dx + W] = color; // BR 初始化为纯净图
                    }
                }
            }
        }
        
        // 绘制象限标签
        cv::putText(debug_imgs_[debug_img_idx], "Raw ROI", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::putText(debug_imgs_[debug_img_idx], "SR Patch", cv::Point(W + 10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::putText(debug_imgs_[debug_img_idx], "RANSAC Viz", cv::Point(10, H + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::putText(debug_imgs_[debug_img_idx], "Global Grid & Final Coord", cv::Point(W + 10, H + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }

    // 限定渲染到 BL (左下角) 的像素绘制器（用于边缘点和拟合圆）
    auto setVizPixelBL = [&](float sr_x, float sr_y, cv::Vec3b color) {
        if (!viz_ || debug_imgs_[debug_img_idx].empty()) return;
        // ✨ 核心修正：视觉平移半个原图像素，使浮点坐标严格对齐到视觉像素块的中心
        sr_x += sr_scale / 2.0f;
        sr_y += sr_scale / 2.0f;
        int px = static_cast<int>(std::round(sr_x * viz_mult));
        int py = static_cast<int>(std::round(sr_y * viz_mult));
        if (px >= 0 && px < W && py >= 0 && py < H) {
            debug_imgs_[debug_img_idx].at<cv::Vec3b>(py + H, px) = color;
        }
    };

    // 同步渲染到 BL 或 BR 的十字准星绘制器
    auto drawCrosshair = [&](float sr_x, float sr_y, cv::Vec3b color, bool in_BL = true, bool in_BR = true) {
        if (!viz_ || debug_imgs_[debug_img_idx].empty()) return;
        // ✨ 核心修正：视觉平移半个原图像素，对齐像素中心
        sr_x += sr_scale / 2.0f;
        sr_y += sr_scale / 2.0f;
        int center_px = static_cast<int>(std::round(sr_x * viz_mult));
        int center_py = static_cast<int>(std::round(sr_y * viz_mult));
        
        for (int d = -3; d <= 3; ++d) {
            int px_h = center_px + d, py_h = center_py;
            int px_v = center_px, py_v = center_py + d;
            
            // 绘制 BL
            if (in_BL) {
                if (px_h >= 0 && px_h < W && py_h >= 0 && py_h < H) debug_imgs_[debug_img_idx].at<cv::Vec3b>(py_h + H, px_h) = color;
                if (px_v >= 0 && px_v < W && py_v >= 0 && py_v < H) debug_imgs_[debug_img_idx].at<cv::Vec3b>(py_v + H, px_v) = color;
            }
            // 绘制 BR
            if (in_BR) {
                if (px_h >= 0 && px_h < W && py_h >= 0 && py_h < H) debug_imgs_[debug_img_idx].at<cv::Vec3b>(py_h + H, px_h + W) = color;
                if (px_v >= 0 && px_v < W && py_v >= 0 && py_v < H) debug_imgs_[debug_img_idx].at<cv::Vec3b>(py_v + H, px_v + W) = color;
            }
        }
    };

    // 结构体：收集优化点信息，实现 Deferred 延迟渲染，确保光斑画在 Grid 之上
    struct BR_Viz_Info {
        cv::Point2f final_pt;
        bool fit_success;
    };
    std::vector<BR_Viz_Info> deferred_br_viz;

    // --- 5. 定义内部单点处理逻辑 ---
    auto processPoint = [&](const cv::Point2f& rough_pt, const cv::Point2f& other1, const cv::Point2f& other2, const std::string& pt_loc) -> cv::Point2f {
        
        float dist1 = cv::norm(rough_pt - other1);
        float dist2 = cv::norm(rough_pt - other2);
        float max_search_radius_orig = std::min(dist1, dist2) / 2.0f + kRadiusMargin;
        float max_search_radius_sr = max_search_radius_orig * sr_scale;

        float center_sr_x = (rough_pt.x - roi.x) * sr_scale;
        float center_sr_y = (rough_pt.y - roi.y) * sr_scale;
        cv::Point2f center_sr(center_sr_x, center_sr_y);
        
        // ✨ [Viz] 绘制 Rough 初始位置 (品红十字，仅显示在 BL)
        if (viz_) {
            drawCrosshair(center_sr_x, center_sr_y, cv::Vec3b(255, 0, 255), true, false);
        }

        std::vector<cv::Point2f> edge_pts_sr;

        for (int i = 0; i < kRayCount; ++i) {
            float angle = i * 2.0f * CV_PI / kRayCount;
            float cos_a = std::cos(angle);
            float sin_a = std::sin(angle);
            
            cv::Point2f best_edge_pt;
            bool found_edge = false;
            bool is_dropping = false; 

            for (float r = 0; r < max_search_radius_sr; r += 1.0f) {
                cv::Point2f pt1(center_sr.x + r * cos_a, center_sr.y + r * sin_a);
                cv::Point2f pt2(center_sr.x + (r + kGradientGap) * cos_a, center_sr.y + (r + kGradientGap) * sin_a);

                if (pt2.x < 0 || pt2.x >= sr_patch.cols - 1 || pt2.y < 0 || pt2.y >= sr_patch.rows - 1) break;

                float val1 = getBilinearSubpixel(sr_patch, pt1);
                float val2 = getBilinearSubpixel(sr_patch, pt2);
                float grad = val1 - val2; 
                
                if (grad > kMinGradient && val1 > kMinBrightness) {
                    if (viz_) setVizPixelBL(pt1.x, pt1.y, cv::Vec3b(255, 0, 0)); 
                }

                if (!is_dropping) {
                    if (grad > kMinGradient && val1 > kMinBrightness) {
                        is_dropping = true;
                    }
                } else {
                    if (grad <= 0.0f) {
                        if (val1 > kMinBrightness) {
                            best_edge_pt = cv::Point2f(center_sr.x + (r + kGradientGap/2.0f) * cos_a, center_sr.y + (r + kGradientGap/2.0f) * sin_a);
                            found_edge = true;
                        }
                        break; 
                    }
                    if (val1 <= kMinBrightness) {
                        best_edge_pt = cv::Point2f(center_sr.x + (r - 1.0f + kGradientGap/2.0f) * cos_a, center_sr.y + (r - 1.0f + kGradientGap/2.0f) * sin_a);
                        found_edge = true;
                        break; 
                    }
                }
            }
            if (found_edge) edge_pts_sr.push_back(best_edge_pt);
        }

        cv::Point2f final_pt = rough_pt;
        bool fit_success = false;
        cv::Point3f best_circle(0, 0, 0);

        if (edge_pts_sr.size() >= 3) {
            std::vector<float> dists;
            for (auto& p : edge_pts_sr) dists.push_back(cv::norm(p - center_sr));
            std::sort(dists.begin(), dists.end());
            float median_d = dists[dists.size() / 2];

            std::vector<cv::Point2f> clean_edges;
            for (auto& p : edge_pts_sr) {
                if (std::abs(cv::norm(p - center_sr) - median_d) < median_d * 0.5f) { 
                    clean_edges.push_back(p);
                    setVizPixelBL(p.x, p.y, cv::Vec3b(255, 255, 0)); // 黄色边缘点 (仅 BL)
                }
            }

            float max_r = max_search_radius_sr; 
            best_circle = fitCircleRANSAC(clean_edges, kRansacIters, kInlierTol, max_r);

            if (best_circle.z > 0) {
                final_pt = cv::Point2f((best_circle.x / sr_scale) + roi.x, (best_circle.y / sr_scale) + roi.y);
                fit_success = true;

                if (viz_) {
                    // 拟合红圈仅画在 BL 
                    int num_points = static_cast<int>(best_circle.z * viz_mult * 2 * CV_PI * 1.5); 
                    for (int i = 0; i < num_points; ++i) {
                        float theta = i * 2.0f * CV_PI / num_points;
                        float cx = best_circle.x + best_circle.z * std::cos(theta);
                        float cy = best_circle.y + best_circle.z * std::sin(theta);
                        setVizPixelBL(cx, cy, cv::Vec3b(0, 0, 255)); 
                    }
                    
                    // ✨ 绿十字仅先画在 BL，BR 的渲染延后处理
                    drawCrosshair(best_circle.x, best_circle.y, cv::Vec3b(0, 255, 0), true, false); 

                    // --- 在 BR 中绘制全局坐标系网格 (此时处于最底层) ---
                    int cx_global = static_cast<int>(std::round(final_pt.x));
                    int cy_global = static_cast<int>(std::round(final_pt.y));
                    int grid_radius = 3; // 优化点周围 +-3 像素的局部网格

                    // 绘制水平横线 (Y 坐标)
                    for (int y_g = cy_global - grid_radius; y_g <= cy_global + grid_radius; ++y_g) {
                        if (y_g >= roi.y && y_g < roi.y + roi.height) {
                            // 映射到 SR 的像素中心坐标
                            float sr_y = (y_g - roi.y) * sr_scale + sr_scale / 2.0f;
                            int line_y = H + static_cast<int>(sr_y * viz_mult);
                            
                            int start_x_g = std::max(roi.x, cx_global - grid_radius);
                            int end_x_g = std::min(roi.x + roi.width - 1, cx_global + grid_radius);
                            int px_start = W + static_cast<int>(((start_x_g - roi.x) * sr_scale + sr_scale / 2.0f) * viz_mult);
                            int px_end = W + static_cast<int>(((end_x_g - roi.x) * sr_scale + sr_scale / 2.0f) * viz_mult);
                            
                            cv::line(debug_imgs_[debug_img_idx], cv::Point(px_start, line_y), cv::Point(px_end, line_y), cv::Scalar(80, 80, 80), 1);
                            
                            // 交错排布避免遮挡
                            int stagger_x = (y_g % 2 == 0) ? 0 : 25;
                            cv::putText(debug_imgs_[debug_img_idx], std::to_string(y_g), cv::Point(px_end + 3 + stagger_x, line_y + 4), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
                        }
                    }

                    // 绘制垂直竖线 (X 坐标)
                    for (int x_g = cx_global - grid_radius; x_g <= cx_global + grid_radius; ++x_g) {
                        if (x_g >= roi.x && x_g < roi.x + roi.width) {
                            float sr_x = (x_g - roi.x) * sr_scale + sr_scale / 2.0f;
                            int line_x = W + static_cast<int>(sr_x * viz_mult);
                            
                            int start_y_g = std::max(roi.y, cy_global - grid_radius);
                            int end_y_g = std::min(roi.y + roi.height - 1, cy_global + grid_radius);
                            int py_start = H + static_cast<int>(((start_y_g - roi.y) * sr_scale + sr_scale / 2.0f) * viz_mult);
                            int py_end = H + static_cast<int>(((end_y_g - roi.y) * sr_scale + sr_scale / 2.0f) * viz_mult);
                            
                            cv::line(debug_imgs_[debug_img_idx], cv::Point(line_x, py_start), cv::Point(line_x, py_end), cv::Scalar(80, 80, 80), 1);
                            
                            // 交错排布避免遮挡
                            int stagger_y = (x_g % 2 == 0) ? 0 : 15;
                            cv::putText(debug_imgs_[debug_img_idx], std::to_string(x_g), cv::Point(line_x - 10, py_end + 12 + stagger_y), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
                        }
                    }
                }
            }
        }

        // 收集待渲染信息供 Deferred 阶段使用
        deferred_br_viz.push_back({final_pt, fit_success});

        // 绘制相关说明文本（固定限制在 BL 象限内展示）
        if (viz_) {
            std::ostringstream rough_ss, refined_ss;
            rough_ss << std::fixed << std::setprecision(2) << "Rough:   (" << rough_pt.x << ", " << rough_pt.y << ")";
            if (fit_success) {
                refined_ss << std::fixed << std::setprecision(2) << "Refined: (" << final_pt.x << ", " << final_pt.y << ")";
            } else {
                refined_ss << "Refined: Failed (fallback)";
            }

            int base_x = static_cast<int>((center_sr_x + sr_scale / 2.0f) * viz_mult);
            int base_y = static_cast<int>((center_sr_y + sr_scale / 2.0f) * viz_mult);
            int anchor_x, anchor_y_line1, anchor_y_line2;
            int y_offset = 20;

            if (pt_loc == "L") {
                anchor_x = base_x - 60; anchor_y_line1 = base_y - 35 - y_offset; anchor_y_line2 = base_y - 20 - y_offset;
            } else if (pt_loc == "R") {
                anchor_x = base_x + 10; anchor_y_line1 = base_y - 35 - y_offset; anchor_y_line2 = base_y - 20 - y_offset;
            } else {
                anchor_x = base_x - 30; anchor_y_line1 = base_y + 35 + y_offset; anchor_y_line2 = base_y + 50 + y_offset;
            }

            // 文本 Y 轴偏移到 BL 象限
            anchor_y_line1 += H;
            anchor_y_line2 += H;

            cv::putText(debug_imgs_[debug_img_idx], rough_ss.str(), cv::Point(anchor_x, anchor_y_line1), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
            cv::putText(debug_imgs_[debug_img_idx], refined_ss.str(), cv::Point(anchor_x, anchor_y_line2), cv::FONT_HERSHEY_SIMPLEX, 0.4, fit_success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }

        return final_pt;
    };

    // --- 6. 执行处理 ---
    geo.l_pt = processPoint(geo.l_pt, geo.r_pt, geo.m_pt, "L");
    geo.r_pt = processPoint(geo.r_pt, geo.l_pt, geo.m_pt, "R");
    geo.m_pt = processPoint(geo.m_pt, geo.l_pt, geo.r_pt, "M");

    // --- 7. Deferred Rendering 延迟绘制：保证光斑准星和坐标显示在网格最上层 ---
    if (viz_) {
        for (const auto& info : deferred_br_viz) {
            if (info.fit_success) {
                float sr_x = (info.final_pt.x - roi.x) * sr_scale;
                float sr_y = (info.final_pt.y - roi.y) * sr_scale;
                
                // 画在 BR 的绿十字
                drawCrosshair(sr_x, sr_y, cv::Vec3b(0, 255, 0), false, true);
                
                // 绘制带微小黑色背景的坐标文本，防网格干扰
                std::ostringstream br_ss;
                br_ss << std::fixed << std::setprecision(2) << "(" << info.final_pt.x << ", " << info.final_pt.y << ")";
                int text_x = W + static_cast<int>((sr_x + sr_scale / 2.0f) * viz_mult) - 30;
                int text_y = H + static_cast<int>((sr_y + sr_scale / 2.0f) * viz_mult) - 15;
                
                cv::Size text_size = cv::getTextSize(br_ss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, nullptr);
                cv::rectangle(debug_imgs_[debug_img_idx], cv::Rect(text_x - 2, text_y - text_size.height - 2, text_size.width + 4, text_size.height + 4), cv::Scalar(0, 0, 0), cv::FILLED);
                cv::putText(debug_imgs_[debug_img_idx], br_ss.str(), cv::Point(text_x, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
    }
}

} // namespace glintdetection