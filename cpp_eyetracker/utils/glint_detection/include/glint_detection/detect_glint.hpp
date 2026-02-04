#include <deque>
#include "opencv2/core.hpp"

#include "cfg/config.hpp"
#include "logger/logger.hpp"

namespace glintdetection {

class GlintDetector {
public:
    explicit GlintDetector(const std::string& mode = "inference");
    ~GlintDetector() = default;

    std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>>
    detect(cv::Mat gray);

    std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>>
    detectFullImage(cv::Mat gray);

    std::string img_name_;
    cv::Mat threshold_output_;
    std::vector<cv::Mat> debug_imgs_;
    bool local_debug_ = false;
    bool viz_ = false;

private:
    enum class EyeType { Left, Right };

    struct Pupil {
        cv::RotatedRect rr;
        float major_axis;
        float minor_axis;
        double darkness; // 保留 darkness 用于在 searchPupilInROI 中排序
        
        // 判定点是否在瞳孔附近 (2.5倍长轴)
        bool isNearby(const cv::Point2f& pt) const {
            float dist = cv::norm(pt - rr.center);
            return dist < (major_axis * 2.5f);
        }
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

    std::string mode_;
    Cfg cfg_;
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
    double threshold_step_ = 25.0;

    double init_left_glints_x_min_, init_left_glints_x_max_;
    double init_right_glints_x_min_, init_right_glints_x_max_;

    std::vector<GlintGeometry> glint_geometry_list_;

    bool side2side(const cv::Point2f& l_pt, const cv::Point2f& r_pt);
    bool side2mid(const cv::Point2f& l_pt, const cv::Point2f& r_pt, const cv::Point2f& m_pt);
    bool side2mid(const cv::Point2f& s_pt, const cv::Point2f& m_pt);

    std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
    splitGlintsByEye(
        std::vector<cv::Point2f> contour_centers, 
        double distance_threshold_x = 100.0, 
        double outlier_distance_threshold = 200.0
    );

    std::tuple<cv::Mat, cv::Point2f>
    getSearchRegionSideAndMid(
        const cv::Point2f& s_pt, 
        const cv::Point2f& m_pt
    );

    std::tuple<cv::Mat, cv::Point2f>
    getSearchRegionSideAndSide(
        const cv::Point2f& l_pt,
        const cv::Point2f& r_pt
    );

    std::tuple<cv::Mat, cv::Point2f>
    makeEyeROI(EyeType eye) const;

    bool isGlintValid(
        const cv::Point2f& glint_center,
        bool reverse = false    
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

    std::vector<cv::Rect>
    determineCornealReflectionROI();

    std::vector<cv::Rect>
    getROI();

    std::vector<std::vector<cv::Rect>> 
    clusterROIs(const std::vector<cv::Rect>& rois);

    Pupil findBestPupilForCluster(const std::vector<cv::Rect>& cluster_rois);

    std::vector<GlintGeometry> 
    selectBestGlintsPerCluster(const std::vector<GlintGeometry>& all_candidates);

    std::vector<GlintGeometry> 
    detectCluster(
        int cluster_id, 
        const std::vector<cv::Rect>& cluster_rois,
        const Pupil& best_pupil
    );

    std::vector<cv::Point2f>
    searchGlintsInROI(
        const cv::Mat& roi_img,
        const cv::Point2f& roi_offset,
        const std::vector<cv::Point2f>& glints,
        const double threshold_value,
        const std::string& debug_tag
    );
    
    std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
    findGlintsSecond(
        std::vector<cv::Point2f> left_glints_candidates,
        std::vector<cv::Point2f> right_glints_candidates
    );

    std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
    findGlints();

    std::tuple<std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>>
    splitGlintsGeometry(
        std::vector<std::vector<cv::Point2f>> glint_geometry_list, 
        double distance_threshold_x = 100.0
    );

    void removeOutliersByMedian(std::vector<cv::Point2f>& pts, double outlier_distance_threshold = 200.0);

    std::vector<GlintGeometry>
    findGeometry(std::vector<cv::Point2f> glint_candidates);

    bool isGlintRepeated(
        const std::vector<cv::Point2f>& roi_glints, 
        const cv::Point2f& glint
    );

    bool isGlintGeometryRepeated(
        const cv::Point2f& s_pt, 
        const cv::Point2f& m_pt
    );

    void checkAndPushGlintGeometry(
        const cv::Point2f& l_pt,
        const cv::Point2f& r_pt,
        const cv::Point2f& m_pt,
        double brightness_threshold = 25.0
    );

    std::vector<std::vector<cv::Point2f>>
    glintGeometryListToGlintVectors(const std::vector<GlintGeometry>& glint_geometry);
    
};

} // namespace glintdetection