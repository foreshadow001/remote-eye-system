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
PoseZxz quatToZxz(const Quat& q);
Quat quatMultiply(const Quat& q1, const Quat& q0);
Pt3  quatRotate(const Quat& q, const Pt3& v);
Pose computeToolPose(const Pose& flange, const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg);
Pose composePoses(const Pose& base, const Pose& offset);

// 完整变换链: T_board_in_ccs = T_arm_in_ccs * T_flange * T_tool_offset * T_board_in_tool
// 所有旋转参数均为 Z-X-Z'' 欧拉角 (度)
Pose computeFullChain(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg,
                      const Pt3& board_trans, const Pt3& board_rot_zxz_deg);

// ===================== 封装类 =====================

struct ArmCfg {
    Pt3 tool_t, tool_r;     // tool_in_flange:  m, deg Z-X-Z''
    Pt3 ccs_t,  ccs_r;      // arm_in_ccs:      m, deg Z-X-Z''
};

class PiperToCam {
public:
    explicit PiperToCam(const std::string& yaml_path);

    // --- 读取配置的 arm_in_ccs / tool_in_flange (供 Ceres 获取初始值) ---
    const ArmCfg& cfg(const std::string& arm) const;

    // --- 四元数接口 (使用配置中的 arm_in_ccs + tool_in_flange) ---
    Pose convert(const std::string& arm, const Pose& flange) const;
    Pose convert(const std::string& arm,
                 double fx, double fy, double fz,
                 double qx, double qy, double qz, double qw) const;

    // --- 四元数接口 (显式传入所有变换参数, 供 Ceres 调用) ---
    Pose convertWithParams(const Pose& flange,
                           const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                           const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg) const;
    Pose convertWithParams(const Pose& flange,
                           const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                           const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg,
                           const Pt3& board_trans, const Pt3& board_rot_zxz_deg) const;

    // --- Z-X-Z'' Euler 接口 (degrees) ---
    PoseZxz convertZxz(const std::string& arm, const Pose& flange) const;
    PoseZxz convertZxz(const std::string& arm,
                       double fx, double fy, double fz,
                       double qx, double qy, double qz, double qw) const;

    std::vector<std::string> arms() const;

    // 写回 arm_in_ccs 到 yaml
    void writeBackArmInCcs(const std::string& arm, const Pt3& trans, const Pt3& rot_zxz) const;

private:
    std::string path_;
    std::map<std::string, ArmCfg> cfg_;
};

// ===================== Ceres 标定求解类 =====================

// 单个观测: flange 位姿 + 标定板在 CCS 中的观测位姿
struct PiperCalibObs {
    Pose flange;
    PoseZxz board_in_ccs;  // 观测值 (来自 resolve_calib_board_pose 输出)
};

class PiperArmCalibrator {
public:
    // 从 piper.yaml 加载初始值和 bounds
    PiperArmCalibrator(const std::string& yaml_path, const std::string& arm);

    void addObservation(const Pose& flange, const PoseZxz& board_in_ccs);
    size_t numObservations() const { return obs_.size(); }

    // 求解, 返回 true 表示成功
    bool solve();

    // 获取优化后的参数
    Pt3 armTrans()   const { return arm_t_; }
    Pt3 armRotZxz()  const { return arm_r_; }

    // 写回 arm_in_ccs 到 yaml
    void writeBack() const;

private:
    std::string yaml_path_, arm_;
    Pt3 tool_t_, tool_r_;       // tool_in_flange (从配置)
    Pt3 arm_t_, arm_r_;         // arm_in_ccs 结果
    Pt3 board_t_, board_r_;     // board_in_flange 结果 (辅助, 不保存)

    // bounds: [x_lower, y_lower, z_lower], [x_upper, y_upper, z_upper]
    Pt3 arm_t_lo_, arm_t_hi_, arm_r_lo_, arm_r_hi_;
    Pt3 board_t_lo_, board_t_hi_, board_r_lo_, board_r_hi_;

    std::vector<PiperCalibObs> obs_;
};

} // namespace gazeestimation
