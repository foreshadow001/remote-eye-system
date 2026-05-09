#include <deque>
#include "opencv2/core.hpp"

#include "cfg/config.hpp"
#include "logger/logger.hpp"

/*
================================================================================
[Hyperparameters List for detect_glint.hpp]
维护说明：以下为类成员变量中定义的全局默认超参数。
================================================================================[Global Thresholds & Flags]
- glass_reflection_threshold_ = 200.0; // 镜片反光二值化基础阈值
- frame_reflection_threshold_ = 200.0; // 镜框反光二值化基础阈值
- threshold_step_             = 25.0;  // 阈值递减/搜索步长
- laplacian_scale_            = 1;     // 拉普拉斯边缘检测的缩放因子
- laplacian_delta_            = 0;     // 拉普拉斯边缘检测的偏移量
- local_debug_                = false; // 局部调试标志
- viz_                        = false; // 可视化标志

================================================================================
*/

namespace glintdetection {

class GlintDetector {
public:
    struct Pupil;
    struct GlintGeometry;
    explicit GlintDetector(const std::string& param_type = "default");
    ~GlintDetector() = default;

    std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, cv::Point2f, cv::Point2f>
    detect(cv::Mat gray);
    
    const std::vector<Pupil>& getPupils() const { return init_pupil_seeds_; }
    const std::vector<GlintGeometry>& getGlintGeometries() const { return final_geometries_; }

    void setIsCollecting(bool is_collecting) { is_collecting_ = is_collecting; }
    void setViz(bool viz) { viz_ = viz; }
    void setVizThreshold(bool viz_threshold) { viz_threshold_ = viz_threshold; }
    void setLocalDebug(bool local_debug) { local_debug_ = local_debug; }
    void setDebugTime(bool debug_time) { debug_time_ = debug_time; }
    void setImageName(const std::string& img_name) { img_name_ = img_name; }
    std::vector<cv::Mat> getDebugImgs() const { return debug_imgs_; }
    cv::Mat getThresholdOutput() const { return threshold_output_; }

private:
    enum class EyeType { Left, Right };

    struct Pupil {
        cv::RotatedRect rr;
        float major_axis;
        float minor_axis;

        double roi_min_val;    // ROI 中的最低亮度 (对应 kAdaptiveThreshOffset / kAdaptiveThreshMax)
        double area;           // 面积
        int contour_points;    // 轮廓点数
        float axis_ratio;      // 长宽比
        double solidity;       // 凸包面积比
        double fit_ratio;      // 椭圆拟合面积比
        float avg_residual;    // 平均残差
        double darkness;       // 瞳孔的亮度
    };

    struct GlassReflection
    {
        cv::Point2f center;
        float radius;
    };

    struct SearchScale
    {
        float roi_len = -1.0f;   // < 0 表示未初始化
        float roi_wid = -1.0f;

        bool valid() const {
            return roi_len > 0.0f && roi_wid > 0.0f;
        }
    };

    struct FrameReflection
    {
        // 基础信息
        cv::Point2f center;
        cv::Point2f points[4];
        float length;
        float width;
        float angle_deg; // [0, 180)

        // 原始轮廓 (用于后续精细化计算)
        std::vector<cv::Point> contour;

        // 链式搜索支持
        // 使用指针时需注意 vector 扩容可能导致失效，建议在链式搜索阶段使用 std::list 
        // 或者在搜索完所有节点后再建立链接。这里为了简单，我们用 vector 存储最终链条。
        bool visited = false; 

        // ★ 新增：搜索尺度（可继承）
        SearchScale search_scale;

        // ---- 工具函数 ----
        float aspect() const {
            return (width > 1e-3f) ? length / width : 0.0f;
        }

        bool isSmallGlint(float min_len) const {
            return length < min_len;
        }
    };

    // 定义一个链条类型
    using FrameReflectionChain = std::vector<GlintDetector::FrameReflection>;
    std::vector<GlintDetector::GlassReflection> glass_reflections_;
    std::vector<GlintDetector::FrameReflection> frame_reflections_;
    std::vector<GlintDetector::FrameReflectionChain> frame_reflection_chains_;

    struct GlintGeometry {
        cv::Point2f l_pt, r_pt, m_pt;
        double bg_brightness = 255.0;
        bool on_cornea = false;
        
        // 新增元数据
        int cluster_id = -1;          // 从属的区域 ID
        double found_threshold = 0.0; // 找到该几何体时的阈值

        Pupil linked_pupil; 

        cv::Point2f center() const { return (l_pt + r_pt + m_pt) / 3.0f; }
    };

    struct RoiCluster {
        int cluster_id = -1;
        std::vector<cv::Rect> rois;              // 属于该聚类的细分 ROI 碎片
        std::vector<cv::Rect> iris_exclusions;   // 该聚类内部的虹膜反光排除区域
        Pupil best_pupil;                        // 该聚类对应的最佳 Pupil
        cv::Rect limit_bound;                    // 整个聚类的外接限制框 (Bounding Box)
    };

    std::string img_name_;
    cv::Mat threshold_output_;
    std::vector<cv::Mat> debug_imgs_;
    bool local_debug_ = false;
    bool debug_time_ = false;
    bool viz_ = false;
    bool viz_threshold_ = false;
    bool is_collecting_ = false;
    bool use_glint_sr_ = false;
    std::string param_type_ = "default";

    Cfg cfg_;
    CfgNode spec_pupil_cfg_;
    CfgNode spec_glint_cfg_;
    CfgNode emp_cfg_;
    CfgNode horizontal_pair_cfg_;
    CfgNode middle_point_cfg_;

    cv::Mat gray_;

    int gaussian_kernel_size_;
    cv::Mat gaussed_;

    int laplacian_scale_ = 1;
    int laplacian_delta_ = 0;
    int laplacian_kernel_size_;
    cv::Mat laplaced_;

    cv::Mat abs_dst_;

    std::vector<Pupil> init_pupil_seeds_; // getROI开始时搜到的所有潜在瞳孔
    std::vector<Pupil> final_pupils_;     // 最终确定的瞳孔（预留）
    cv::Rect glint_rect_;                 // 全局 Glints 区域

    std::vector<Pupil> 
    searchPupilInROI(cv::Rect roi_rect);

    double glass_reflection_threshold_ = 200.0;
    double frame_reflection_threshold_ = 200.0;

    cv::Mat exclusion_mask_; 

    double init_threshold_value_;
    double threshold_step_;
    double mini_threshold_;

    std::vector<GlintGeometry> final_geometries_;

    bool side2side(const cv::Point2f& l_pt, const cv::Point2f& r_pt);
    bool side2mid(const cv::Point2f& l_pt, const cv::Point2f& r_pt, const cv::Point2f& m_pt);
    bool side2mid(const cv::Point2f& s_pt, const cv::Point2f& m_pt);

    std::vector<cv::Rect> getSearchRegionSideAndMid(
        const cv::Point2f& s_pt, 
        const cv::Point2f& m_pt,
        const RoiCluster& cluster // 传入统一数据结构，返回切分后的有效区域
    );

    std::vector<cv::Rect> getSearchRegionSideAndSide(
        const cv::Point2f& l_pt,
        const cv::Point2f& r_pt,
        const RoiCluster& cluster // 传入统一数据结构，返回切分后的有效区域
    );

    void searchGlassReflections();

    bool isInsideGlassExclusion(const cv::Point2f& pt) const;

    void visualizePupilAndExclusion();

    bool isPupilNearby(const cv::Point2f& glint_pt);

    bool findNeighborInDirection(
        const FrameReflection& current_fr,
        const cv::Point2f& search_dir, // 单位方向向量
        FrameReflection& out_new_fr    // 输出找到的新节点
    );

    std::vector<FrameReflectionChain>
    searchReflectionChains(
        std::vector<FrameReflection>& initial_seeds
    );

    void searchFrameReflections();

    void drawRotatedRectMask(
        cv::Mat& mask, 
        const cv::RotatedRect& rr, 
        const cv::Scalar& color
    );

    void buildExclusionMask();

    bool isInsideExclusionRegion(
        const cv::Point2f& pt
    ) const;

    bool expandRect(
        cv::Rect& rect, 
        const cv::Mat& mask, 
        const cv::Rect& limit_rect, 
        int direction
    );

    cv::Rect shrinkRoiToValidGlints(
        const cv::Rect& coarse_roi
    );

    std::vector<cv::Rect> subtractRect(const cv::Rect& subject, const cv::Rect& clipper) const;

    void excludeIrisReflectionsFromROIs(
        std::vector<cv::Rect>& rois, 
        std::vector<cv::Rect>& out_iris_bboxes // 新增输出参数，向外传递找到的反光区域
    );
    std::vector<cv::Rect>
    determineCornealReflectionROI();

    std::vector<RoiCluster> getROI();

    std::vector<std::vector<cv::Rect>> 
    clusterROIs(const std::vector<cv::Rect>& rois);

    Pupil findBestPupilForCluster(const std::vector<cv::Rect>& cluster_rois);

    std::vector<GlintGeometry> 
    selectBestGlintsPerCluster(const std::vector<GlintGeometry>& all_candidates);

    std::vector<GlintDetector::GlintGeometry> 
    GlintDetector::detectCluster(const RoiCluster& cluster);

    std::vector<cv::Point2f>
    searchGlintsInROI(
        const cv::Mat& roi_img,
        const cv::Point2f& roi_offset,
        const std::vector<cv::Point2f>& glints,
        const double threshold_value,
        const std::string& debug_tag
    );

    std::tuple<
        std::vector<GlintDetector::GlintGeometry>, 
        std::vector<GlintDetector::GlintGeometry>
    >
    GlintDetector::splitGlintsGeometry(std::vector<GlintDetector::GlintGeometry> glint_geometries);

    void sortGlintCandidates(std::vector<cv::Point2f>& glints);

    std::vector<GlintGeometry> findGeometry(
        std::vector<cv::Point2f> glint_candidates, 
        double current_thr, 
        const std::vector<GlintGeometry>& existing_geometries,
        const RoiCluster& cluster // 传入统一数据结构
    );

    bool isGlintRepeated(
        const std::vector<cv::Point2f>& roi_glints, 
        const cv::Point2f& glint
    );

    bool isGlintGeometryRepeated(
        const cv::Point2f& s_pt, 
        const cv::Point2f& m_pt,
        const std::vector<GlintGeometry>& existing
    );

    bool GlintDetector::isGlintGeometryRepeated(
        const GlintGeometry& geo,
        const std::vector<GlintGeometry>& existing
    );

    bool checkGlintGeometry(
        const cv::Point2f& l_pt,
        const cv::Point2f& r_pt,
        const cv::Point2f& m_pt,
        double& out_bg_brightness
    );

    std::vector<std::vector<cv::Point2f>>
    glintGeometryListToGlintVector(const std::vector<GlintGeometry>& glint_geometry);
    
    // --- 射线与精细化瞳孔拟合相关函数 ---
    Pupil refinePupil(
        const cv::Mat& gray_img, 
        const Pupil& rough_pupil, 
        const GlintGeometry& glint_geo
    );

    std::vector<cv::Point2f> samplePupilEdgesByRayCasting(
        const cv::Mat& gray_img, 
        const cv::Point2f& rough_center, 
        float rough_major_axis,
        const GlintGeometry& glint_geo // 新增参数
    );

    std::vector<cv::Point2f> filterPupilEdgePoints(
        const std::vector<cv::Point2f>& raw_edges, 
        const cv::Point2f& rough_center,
        const GlintGeometry& glint_geo
    );

    cv::RotatedRect fitEllipseRANSAC(
        const std::vector<cv::Point2f>& clean_edges
    );

    // 亚像素双线性插值辅助函数
    float getBilinearSubpixel(const cv::Mat& img, const cv::Point2f& pt) const;

    // (保留 fitCircleRANSAC)
    cv::Point3f fitCircleRANSAC(
        const std::vector<cv::Point2f>& points, 
        int max_iters, 
        float tolerance, 
        float max_radius
    );

    // ✨ 删除 refineGlintPoint 声明，将签名简化为：
    void refineGlintGeometry(
        const cv::Mat& gray_img, 
        GlintGeometry& geo,
        int debug_img_idx
    );
};

} // namespace glintdetection