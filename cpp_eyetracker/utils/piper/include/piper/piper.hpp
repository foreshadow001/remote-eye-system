#pragma once

#include <string>
#include <map>
#include <vector>

namespace gazeestimation {

struct Quat { double x = 0, y = 0, z = 0, w = 1; };
struct Pt3  { double x = 0, y = 0, z = 0; };
struct Pose { Pt3 pos; Quat quat; };

// ===================== 纯数学函数 =====================

Quat zxzToQuat(double alpha_deg, double beta_deg, double gamma_deg);
Quat quatMultiply(const Quat& q1, const Quat& q0);
Pt3  quatRotate(const Quat& q, const Pt3& v);
Pose computeToolPose(const Pose& flange, const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg);
Pose composePoses(const Pose& base, const Pose& offset);
Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg);

// ===================== 封装类 =====================

class PiperToCam {
public:
    // 从 piper.yaml 加载 arms.<name>.tool + arms.<name>.arm_in_ccs
    explicit PiperToCam(const std::string& yaml_path);

    // 将 flange 位姿 (arm 基座坐标系) 转换为 tool 在相机坐标系 (CCS) 中的位姿
    // arm: "upper" / "lower"
    Pose convert(const std::string& arm, const Pose& flange) const;

    // convenience: 直接传入数值
    Pose convert(const std::string& arm,
                 double fx, double fy, double fz,
                 double qx, double qy, double qz, double qw) const;

    // 列出已加载的 arm 名称
    std::vector<std::string> arms() const;

private:
    struct ArmCfg {
        Pt3 tool_t, tool_r;   // tool offset: translation (m), rotation Z-X-Z'' (deg)
        Pt3 ccs_t, ccs_r;     // arm_in_ccs:   translation (m), rotation Z-X-Z'' (deg)
    };
    std::map<std::string, ArmCfg> cfg_;
};

} // namespace gazeestimation
