#include "piper/piper.hpp"
#include "cfg/config.hpp"
#include <cmath>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ceres/ceres.h>

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

// ============= PiperArmCalibrator =============

PiperArmCalibrator::PiperArmCalibrator(const std::string& yaml_path, const std::string& arm)
    : yaml_path_(yaml_path), arm_(arm)
{
    Cfg cfg(yaml_path);
    auto& a    = cfg["arms"][arm];
    auto& tool = a["tool"];
    tool_t_ = {tool["translation"][0].as<double>(), tool["translation"][1].as<double>(),
               tool["translation"][2].as<double>()};
    tool_r_ = {tool["rotation_zxz"][0].as<double>(), tool["rotation_zxz"][1].as<double>(),
               tool["rotation_zxz"][2].as<double>()};

    auto read3 = [](const CfgNode& n) -> Pt3 {
        return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
    };
    auto readBounds23 = [](const CfgNode& bnd) -> pair<Pt3,Pt3> {
        // bnd: [[lo_x,hi_x], [lo_y,hi_y], [lo_z,hi_z]]
        return {
            {bnd[0][0].as<double>(), bnd[1][0].as<double>(), bnd[2][0].as<double>()},
            {bnd[0][1].as<double>(), bnd[1][1].as<double>(), bnd[2][1].as<double>()},
        };
    };

    auto& aic  = a["calib_arm_in_ccs"];
    arm_t_     = read3(aic["translation"]["init_value"]);
    arm_r_     = read3(aic["rotation_zxz"]["init_value"]);
    tie(arm_t_lo_, arm_t_hi_) = readBounds23(aic["translation"]["bounds"]);
    tie(arm_r_lo_, arm_r_hi_) = readBounds23(aic["rotation_zxz"]["bounds"]);

    auto& bic   = a["calib_board_in_flange"];
    board_t_    = read3(bic["translation"]["init_value"]);
    board_r_    = read3(bic["rotation_zxz"]["init_value"]);
    tie(board_t_lo_, board_t_hi_) = readBounds23(bic["translation"]["bounds"]);
    tie(board_r_lo_, board_r_hi_) = readBounds23(bic["rotation_zxz"]["bounds"]);
}

void PiperArmCalibrator::addObservation(const Pose& flange, const PoseZxz& board_in_ccs) {
    obs_.push_back({flange, board_in_ccs});
}

// --- Ceres functor ---
class PiperCalibResidual {
public:
    PiperCalibResidual(const Pose& flange, const PoseZxz& board_obs,
                       const Pt3& tool_t, const Pt3& tool_r)
        : fl_(flange), bo_(board_obs), tt_(tool_t), tr_(tool_r) {}

    // params: 0=arm_t(3), 1=arm_r(3), 2=board_t(3), 3=board_r(3)
    bool operator()(double const* const* params, double* residual) const {
        Pt3 at{params[0][0], params[0][1], params[0][2]};
        Pt3 ar{params[1][0], params[1][1], params[1][2]};
        Pt3 bt{params[2][0], params[2][1], params[2][2]};
        Pt3 br{params[3][0], params[3][1], params[3][2]};

        Pose pred = computeFullChain(fl_, tt_, tr_, at, ar, bt, br);

        // Translation residual (m)
        residual[0] = pred.pos.x - bo_.pos.x;
        residual[1] = pred.pos.y - bo_.pos.y;
        residual[2] = pred.pos.z - bo_.pos.z;

        // Rotation residual: axis-angle of q_obs⁻¹ * q_pred
        Quat qo = zxzToQuat(bo_.alpha, bo_.beta, bo_.gamma);
        Quat qp_inv{-pred.quat.x, -pred.quat.y, -pred.quat.z, pred.quat.w};
        Quat qd = quatMultiply(qo, qp_inv);
        double ang = 2.0 * acos(max(-1.0, min(1.0, qd.w)));
        double s = sin(ang / 2.0);
        if (fabs(s) > 1e-10) {
            residual[3] = ang * qd.x / s;
            residual[4] = ang * qd.y / s;
            residual[5] = ang * qd.z / s;
        } else {
            residual[3] = residual[4] = residual[5] = 0.0;
        }
        return true;
    }
private:
    Pose fl_; PoseZxz bo_; Pt3 tt_, tr_;
};

bool PiperArmCalibrator::solve() {
    if (obs_.empty()) return false;

    vector<vector<double>> init = {
        {arm_t_.x,   arm_t_.y,   arm_t_.z},
        {arm_r_.x,   arm_r_.y,   arm_r_.z},
        {board_t_.x, board_t_.y, board_t_.z},
        {board_r_.x, board_r_.y, board_r_.z},
    };

    ceres::Problem problem;
    auto* cost = new ceres::DynamicNumericDiffCostFunction<PiperCalibResidual, ceres::CENTRAL>(
        new PiperCalibResidual(obs_[0].flange, obs_[0].board_in_ccs, tool_t_, tool_r_));
    for (auto& iv : init) cost->AddParameterBlock(iv.size());
    cost->SetNumResiduals(6);

    vector<double*> vars;
    for (size_t i = 0; i < init.size(); i++) {
        double* v = new double[init[i].size()];
        for (size_t j = 0; j < init[i].size(); j++) v[j] = init[i][j];
        vars.push_back(v);
    }
    problem.AddResidualBlock(cost, nullptr, vars);

    vector<Pt3> lo = {arm_t_lo_, arm_r_lo_, board_t_lo_, board_r_lo_};
    vector<Pt3> hi = {arm_t_hi_, arm_r_hi_, board_t_hi_, board_r_hi_};
    for (size_t i = 0; i < vars.size(); i++) {
        double* v = vars[i];
        problem.SetParameterLowerBound(v, 0, lo[i].x); problem.SetParameterUpperBound(v, 0, hi[i].x);
        problem.SetParameterLowerBound(v, 1, lo[i].y); problem.SetParameterUpperBound(v, 1, hi[i].y);
        problem.SetParameterLowerBound(v, 2, lo[i].z); problem.SetParameterUpperBound(v, 2, hi[i].z);
    }

    // Add remaining observations
    for (size_t k = 1; k < obs_.size(); k++) {
        auto* c2 = new ceres::DynamicNumericDiffCostFunction<PiperCalibResidual, ceres::CENTRAL>(
            new PiperCalibResidual(obs_[k].flange, obs_[k].board_in_ccs, tool_t_, tool_r_));
        for (auto& iv : init) c2->AddParameterBlock(iv.size());
        c2->SetNumResiduals(6);
        problem.AddResidualBlock(c2, nullptr, vars);
    }

    // --- 调试: 打印初始值和 bounds ---
    cout << fixed << setprecision(4);
    cout << "\n[Ceres] " << obs_.size() << " observations, " << (obs_.size() * 6) << " residuals" << endl;
    cout << "[Ceres] Initial values:" << endl;
    cout << "  arm_t:     " << arm_t_.x   << " " << arm_t_.y   << " " << arm_t_.z   << endl;
    cout << "  arm_r:     " << arm_r_.x   << " " << arm_r_.y   << " " << arm_r_.z   << endl;
    cout << "  board_t:   " << board_t_.x << " " << board_t_.y << " " << board_t_.z << endl;
    cout << "  board_r:   " << board_r_.x << " " << board_r_.y << " " << board_r_.z << endl;

    // 打印第一帧的前向计算结果
    {
        double* p0[4] = {vars[0], vars[1], vars[2], vars[3]};
        vector<double> r0(6);
        PiperCalibResidual res0(obs_[0].flange, obs_[0].board_in_ccs, tool_t_, tool_r_);
        res0(p0, r0.data());
        cout << "[Ceres] Frame 0 residual: t_err=[" << r0[0] << " " << r0[1] << " " << r0[2]
             << "] r_err=[" << r0[3] << " " << r0[4] << " " << r0[5] << "]" << endl;
        // 打印观测值和预测值
        PoseZxz ob = obs_[0].board_in_ccs;
        Pose pred = computeFullChain(obs_[0].flange, tool_t_, tool_r_, arm_t_, arm_r_, board_t_, board_r_);
        PoseZxz prz = quatToZxz(pred.quat); prz.pos = pred.pos;
        cout << "[Ceres] Frame 0 obs:   t=[" << ob.pos.x << " " << ob.pos.y << " " << ob.pos.z
             << "] r=[" << ob.alpha << " " << ob.beta << " " << ob.gamma << "]" << endl;
        cout << "[Ceres] Frame 0 pred:  t=[" << prz.pos.x << " " << prz.pos.y << " " << prz.pos.z
             << "] r=[" << prz.alpha << " " << prz.beta << " " << prz.gamma << "]" << endl;
    }

    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = true;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = 1000;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    cout << summary.FullReport() << endl;

    cout << "[Ceres] Final values:" << endl;
    cout << "  arm_t:     " << vars[0][0] << " " << vars[0][1] << " " << vars[0][2] << endl;
    cout << "  arm_r:     " << vars[1][0] << " " << vars[1][1] << " " << vars[1][2] << endl;
    cout << "  board_t:   " << vars[2][0] << " " << vars[2][1] << " " << vars[2][2] << endl;
    cout << "  board_r:   " << vars[3][0] << " " << vars[3][1] << " " << vars[3][2] << endl;
    cout << "[Ceres] Termination: " << summary.BriefReport() << endl;

    arm_t_   = {vars[0][0], vars[0][1], vars[0][2]};
    arm_r_   = {vars[1][0], vars[1][1], vars[1][2]};
    board_t_ = {vars[2][0], vars[2][1], vars[2][2]};
    board_r_ = {vars[3][0], vars[3][1], vars[3][2]};

    for (auto* v : vars) delete[] v;
    return summary.IsSolutionUsable();
}

void PiperArmCalibrator::writeBack() const {
    PiperToCam p2c(yaml_path_);
    p2c.writeBackArmInCcs(arm_, arm_t_, arm_r_);
}

} // namespace gazeestimation
