#pragma once

#include <string>
#include <map>
#include <vector>

namespace gazeestimation {

struct Quat { double x = 0, y = 0, z = 0, w = 1; };
struct Pt3  { double x = 0, y = 0, z = 0; };
struct Pose { Pt3 pos; Quat quat; };
struct PoseZxz { Pt3 pos; double alpha = 0, beta = 0, gamma = 0; }; // Z-X-Z'' degrees

// ===================== 纯数学函数 =====================

Quat zxzToQuat(double alpha_deg, double beta_deg, double gamma_deg);
PoseZxz quatToZxz(const Quat& q);  // quaternion → Z-X-Z'' Euler (degrees)
Quat quatMultiply(const Quat& q1, const Quat& q0);
Pt3  quatRotate(const Quat& q, const Pt3& v);
Pose computeToolPose(const Pose& flange, const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg);
Pose computeToolPoseTransFirst(const Pose& flange, const Pt3& trans, const Pt3& rot_zxz_deg);
Pose composePoses(const Pose& base, const Pose& offset);
Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg);

// ===================== 封装类 =====================

class PiperToCam {
public:
    explicit PiperToCam(const std::string& yaml_path);

    // --- 四元数接口 ---
    Pose convert(const std::string& arm, const Pose& flange) const;
    Pose convert(const std::string& arm,
                 double fx, double fy, double fz,
                 double qx, double qy, double qz, double qw) const;

    // --- Z-X-Z'' Euler 接口 (degrees) ---
    PoseZxz convertZxz(const std::string& arm, const Pose& flange) const;
    PoseZxz convertZxz(const std::string& arm,
                       double fx, double fy, double fz,
                       double qx, double qy, double qz, double qw) const;

    std::vector<std::string> arms() const;

private:
    struct ArmCfg {
        Pt3 tool_t, tool_r;   // m, deg (Z-X-Z'')
        Pt3 ccs_t, ccs_r;     // m, deg (Z-X-Z'')
    };
    std::map<std::string, ArmCfg> cfg_;
};

} // namespace gazeestimation
