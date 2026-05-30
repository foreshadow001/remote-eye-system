#pragma once

#include <string>
#include <map>
#include <vector>
#include <utility>

namespace gazeestimation {

struct Quat { double x = 0, y = 0, z = 0, w = 1; };
struct Pt3  { double x = 0, y = 0, z = 0; };
struct Pose { Pt3 pos; Quat quat; };
struct PoseZxz { Pt3 pos; double alpha = 0, beta = 0, gamma = 0; };

// ===================== 纯数学函数 =====================

Quat zxzToQuat(double alpha_deg, double beta_deg, double gamma_deg);
PoseZxz quatToZxz(const Quat& q);
Quat quatMultiply(const Quat& q1, const Quat& q0);
Quat quatConjugate(const Quat& q);
Pt3  quatRotate(const Quat& q, const Pt3& v);
Pose computeToolPose(const Pose& flange, const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg);
Pose computeToolPoseTransFirst(const Pose& flange, const Pt3& trans, const Pt3& rot_zxz_deg);
Pose composePoses(const Pose& base, const Pose& offset);
Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg);

// ===================== 正向变换类 =====================

class PiperToCam {
public:
    explicit PiperToCam(const std::string& yaml_path);
    Pose convert(const std::string& arm, const Pose& flange) const;
    Pose convert(const std::string& arm,
                 double fx, double fy, double fz,
                 double qx, double qy, double qz, double qw) const;
    PoseZxz convertZxz(const std::string& arm, const Pose& flange) const;
    PoseZxz convertZxz(const std::string& arm,
                       double fx, double fy, double fz,
                       double qx, double qy, double qz, double qw) const;
    std::vector<std::string> arms() const;

private:
    struct ArmCfg { Pt3 tool_t, tool_r, ccs_t, ccs_r; };
    std::map<std::string, ArmCfg> cfg_;
};

// ===================== 手眼标定 Ceres 求解类 =====================

struct HandEyeDataPoint {
    Pose flange;         // flange pose in arm base frame
    Pose board_gt_ccs;   // ground truth board pose in CCS
};

class PiperHandEyeCalib {
public:
    // 从 piper.yaml 加载 init_value + bounds (arm_in_ccs + calib_board_in_flange)
    explicit PiperHandEyeCalib(const std::string& yaml_path);

    // 标定: 返回 {tx,ty,tz, rx,ry,rz} — arm_in_ccs (m, deg Z-X-Z'')
    // calib_board 也参与优化但仅作为辅助, 不返回
    std::vector<double> calibrate(const std::string& arm,
                                   const std::vector<HandEyeDataPoint>& data) const;

    // 将标定结果写回 piper.yaml (calib_arm_in_ccs.translation.init_value + rotation_zxz.init_value)
    void saveArmInCcs(const std::string& arm, const std::vector<double>& result) const;

private:
    std::string yaml_path_;
    struct Bounds { std::vector<std::pair<double,double>> t, r; };  // translation + rotation bounds
    struct ArmInit { std::vector<double> ct, cr;  // ccs init: tx,ty,tz, rx,ry,rz (m, deg)
                     std::vector<double> bt, br; }; // board init: tx,ty,tz, rx,ry,rz (m, deg)
    std::map<std::string, std::pair<ArmInit, std::pair<Bounds,Bounds>>> configs_;
};

} // namespace gazeestimation
