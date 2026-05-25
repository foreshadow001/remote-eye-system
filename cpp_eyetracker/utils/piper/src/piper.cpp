#include "piper/piper.hpp"
#include "cfg/config.hpp"
#include <cmath>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gazeestimation {

static double deg2rad(double d) { return d * M_PI / 180.0; }

// ============= Z-X-Z'' Euler → quaternion =============

Quat zxzToQuat(double alpha_deg, double beta_deg, double gamma_deg) {
    double a = deg2rad(alpha_deg), b = deg2rad(beta_deg), g = deg2rad(gamma_deg);
    double c1 = cos(a/2), s1 = sin(a/2);
    double c2 = cos(b/2), s2 = sin(b/2);
    double c3 = cos(g/2), s3 = sin(g/2);
    Quat q;
    q.w = c1*c2*c3 - s1*c2*s3;
    q.x = c1*s2*c3 + s1*s2*s3;
    q.y = s1*s2*c3 - c1*s2*s3;
    q.z = s1*c2*c3 + c1*c2*s3;
    return q;
}

// ============= Quaternion → Z-X-Z'' Euler (degrees) =============

static double clamp1(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }

PoseZxz quatToZxz(const Quat& q) {
    double r00 = 1 - 2*q.y*q.y - 2*q.z*q.z, r01 = 2*q.x*q.y - 2*q.z*q.w, r02 = 2*q.x*q.z + 2*q.y*q.w;
    double r12 = 2*q.y*q.z - 2*q.x*q.w, r20 = 2*q.x*q.z - 2*q.y*q.w, r21 = 2*q.y*q.z + 2*q.x*q.w;
    double r22 = 1 - 2*q.x*q.x - 2*q.y*q.y;
    double beta = acos(clamp1(r22)), sb = sin(beta);
    double alpha, gamma;
    if (fabs(sb) > 1e-6) { alpha = atan2(r02, -r12); gamma = atan2(r20, r21); }
    else { gamma = 0.0; alpha = (beta < M_PI/2.0) ? atan2(-r01, r00) : atan2(r01, r00); }
    return {{0,0,0}, alpha * 180.0 / M_PI, beta * 180.0 / M_PI, gamma * 180.0 / M_PI};
}

Quat quatMultiply(const Quat& q1, const Quat& q0) {
    Quat r;
    r.x = q1.w*q0.x + q1.x*q0.w + q1.y*q0.z - q1.z*q0.y;
    r.y = q1.w*q0.y - q1.x*q0.z + q1.y*q0.w + q1.z*q0.x;
    r.z = q1.w*q0.z + q1.x*q0.y - q1.y*q0.x + q1.z*q0.w;
    r.w = q1.w*q0.w - q1.x*q0.x - q1.y*q0.y - q1.z*q0.z;
    return r;
}

Pt3 quatRotate(const Quat& q, const Pt3& v) {
    Quat qv{v.x, v.y, v.z, 0.0};
    Quat qc{-q.x, -q.y, -q.z, q.w};
    Quat t = quatMultiply(q, qv);
    Quat r = quatMultiply(t, qc);
    return {r.x, r.y, r.z};
}

Pose computeToolPose(const Pose& flange, const Pt3& tool_trans,
                     const Pt3& tool_rot_zxz_deg) {
    Quat q_off = zxzToQuat(tool_rot_zxz_deg.x, tool_rot_zxz_deg.y, tool_rot_zxz_deg.z);
    Pt3 t_rot  = quatRotate(q_off, tool_trans);
    Pt3 t_w    = quatRotate(flange.quat, t_rot);
    Pose tool;
    tool.pos.x = flange.pos.x + t_w.x;
    tool.pos.y = flange.pos.y + t_w.y;
    tool.pos.z = flange.pos.z + t_w.z;
    tool.quat  = quatMultiply(flange.quat, q_off);
    return tool;
}

Pose composePoses(const Pose& base, const Pose& offset) {
    Pt3 p_rot = quatRotate(base.quat, offset.pos);
    Pose r;
    r.pos.x = base.pos.x + p_rot.x;
    r.pos.y = base.pos.y + p_rot.y;
    r.pos.z = base.pos.z + p_rot.z;
    r.quat  = quatMultiply(base.quat, offset.quat);
    return r;
}

// ============= 完整变换链 =============

Pose computeFullChain(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg,
                      const Pt3& board_trans, const Pt3& board_rot_zxz_deg) {
    // T_tool_in_arm = T_flange * T_tool_offset
    Pose tool_in_arm = computeToolPose(flange, tool_trans, tool_rot_zxz_deg);
    // T_board_in_arm = T_tool_in_arm * T_board_offset
    Pose board_in_arm = computeToolPose(tool_in_arm, board_trans, board_rot_zxz_deg);
    // T_arm_in_ccs
    Quat arm_q = zxzToQuat(arm_rot_zxz_deg.x, arm_rot_zxz_deg.y, arm_rot_zxz_deg.z);
    Pose arm_in_ccs{arm_trans, arm_q};
    // T_board_in_ccs = T_arm_in_ccs * T_board_in_arm
    return composePoses(arm_in_ccs, board_in_arm);
}

// ============= PiperToCam =============

static Pt3 readPt3(const CfgNode& n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

PiperToCam::PiperToCam(const std::string& yaml_path) : path_(yaml_path) {
    Cfg cfg(yaml_path);
    for (auto& arm_name : {"upper", "lower"}) {
        try {
            auto& a  = cfg["arms"][arm_name];
            auto& tl = a["tool"];
            auto& cc = a["arm_in_ccs"];
            ArmCfg c;
            c.tool_t = readPt3(tl["translation"]);
            c.tool_r = readPt3(tl["rotation_zxz"]);
            c.ccs_t  = readPt3(cc["translation"]);
            c.ccs_r  = readPt3(cc["rotation_zxz"]);
            cfg_[arm_name] = c;
        } catch (const std::exception&) {}
    }
    if (cfg_.empty())
        throw std::runtime_error("[PiperToCam] No arm configs found in " + yaml_path);
}

const ArmCfg& PiperToCam::cfg(const std::string& arm) const {
    auto it = cfg_.find(arm);
    if (it == cfg_.end()) throw std::runtime_error("[PiperToCam] Unknown arm: " + arm);
    return it->second;
}

// --- 四元数接口 (使用配置) ---

Pose PiperToCam::convert(const std::string& arm, const Pose& flange) const {
    const auto& c = cfg(arm);
    return computeFullChain(flange, c.tool_t, c.tool_r, c.ccs_t, c.ccs_r, {0,0,0}, {0,0,0});
}

Pose PiperToCam::convert(const std::string& arm,
                         double fx, double fy, double fz,
                         double qx, double qy, double qz, double qw) const {
    return convert(arm, {{fx,fy,fz},{qx,qy,qz,qw}});
}

// --- 四元数接口 (显式参数, 供 Ceres) ---

Pose PiperToCam::convertWithParams(const Pose& flange,
                                   const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                                   const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg) const {
    return computeFullChain(flange, tool_trans, tool_rot_zxz_deg,
                            arm_trans, arm_rot_zxz_deg, {0,0,0}, {0,0,0});
}

Pose PiperToCam::convertWithParams(const Pose& flange,
                                   const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                                   const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg,
                                   const Pt3& board_trans, const Pt3& board_rot_zxz_deg) const {
    return computeFullChain(flange, tool_trans, tool_rot_zxz_deg,
                            arm_trans, arm_rot_zxz_deg,
                            board_trans, board_rot_zxz_deg);
}

// --- Z-X-Z'' Euler 接口 ---

PoseZxz PiperToCam::convertZxz(const std::string& arm, const Pose& flange) const {
    Pose p = convert(arm, flange);
    PoseZxz r = quatToZxz(p.quat);
    r.pos = p.pos;
    return r;
}

PoseZxz PiperToCam::convertZxz(const std::string& arm,
                               double fx, double fy, double fz,
                               double qx, double qy, double qz, double qw) const {
    return convertZxz(arm, {{fx,fy,fz},{qx,qy,qz,qw}});
}

std::vector<std::string> PiperToCam::arms() const {
    std::vector<std::string> r;
    for (auto& kv : cfg_) r.push_back(kv.first);
    return r;
}

// --- 写回 ---

void PiperToCam::writeBackArmInCcs(const std::string& arm,
                                   const Pt3& trans, const Pt3& rot_zxz) const {
    ifstream in(path_);
    if (!in) throw std::runtime_error("[PiperToCam] Cannot read " + path_ + " for writeback");
    string yaml((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());
    in.close();

    // 在 arms.<arm>.arm_in_ccs 下替换 translation 和 rotation_zxz 行
    string key = "  " + arm + ":\n";
    size_t pos = yaml.find(key);
    if (pos == string::npos) throw std::runtime_error("[PiperToCam] Arm section not found");

    size_t ccs_pos = yaml.find("    arm_in_ccs:", pos);
    if (ccs_pos == string::npos) throw std::runtime_error("[PiperToCam] arm_in_ccs not found");

    size_t trans_pos = yaml.find("      translation:", ccs_pos);
    size_t rot_pos   = yaml.find("      rotation_zxz:", ccs_pos);
    size_t nl = trans_pos;
    while (nl < yaml.length() && yaml[nl] != '\n') nl++;
    size_t nl2 = rot_pos;
    while (nl2 < yaml.length() && yaml[nl2] != '\n') nl2++;

    stringstream ts, rs;
    ts << fixed << setprecision(4) << "      translation: ["
       << trans.x << ", " << trans.y << ", " << trans.z << "]";
    rs << fixed << setprecision(1) << "      rotation_zxz: ["
       << rot_zxz.x << ", " << rot_zxz.y << ", " << rot_zxz.z << "]";

    yaml.replace(trans_pos, nl - trans_pos, ts.str());
    yaml.replace(rot_pos, nl2 - rot_pos, rs.str());

    ofstream out(path_);
    if (!out) throw std::runtime_error("[PiperToCam] Cannot write " + path_);
    out << yaml;
}

} // namespace gazeestimation
