#include "piper/piper.hpp"
#include "cfg/config.hpp"
#include <cmath>
#include <stdexcept>
#include <cassert>
#include <ceres/ceres.h>

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

Quat quatConjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

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

// 旋转正常叠加, 平移只在 flange 坐标系的轴上 (不被 R_offset 预旋转)
Pose computeToolPoseTransFirst(const Pose& flange, const Pt3& trans, const Pt3& rot_zxz_deg) {
    Quat q_off = zxzToQuat(rot_zxz_deg.x, rot_zxz_deg.y, rot_zxz_deg.z);
    // p_tool = p_flange + R_flange * trans   (trans directly in flange frame)
    Pt3 t_w = quatRotate(flange.quat, trans);
    Pose tool;
    tool.pos.x = flange.pos.x + t_w.x;
    tool.pos.y = flange.pos.y + t_w.y;
    tool.pos.z = flange.pos.z + t_w.z;
    tool.quat = quatMultiply(flange.quat, q_off);   // R_tool = R_flange * R_offset
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

Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg) {
    Pose tool_in_arm = computeToolPose(flange, tool_trans, tool_rot_zxz_deg);
    Quat arm_q = zxzToQuat(arm_rot_zxz_deg.x, arm_rot_zxz_deg.y, arm_rot_zxz_deg.z);
    Pose arm_in_ccs{arm_trans, arm_q};
    return composePoses(arm_in_ccs, tool_in_arm);
}

// ============= PiperToCam =============

static Pt3 readPt3(const CfgNode& n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

PiperToCam::PiperToCam(const std::string& yaml_path) {
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
        } catch (const std::exception& e) {
            // arm not configured — skip
        }
    }
    if (cfg_.empty())
        throw std::runtime_error("[PiperToCam] No arm configs found in " + yaml_path);
}

Pose PiperToCam::convert(const std::string& arm, const Pose& flange) const {
    auto it = cfg_.find(arm);
    if (it == cfg_.end())
        throw std::runtime_error("[PiperToCam] Unknown arm: " + arm);
    const auto& c = it->second;
    return armToolToCamPose(flange, c.tool_t, c.tool_r, c.ccs_t, c.ccs_r);
}

Pose PiperToCam::convert(const std::string& arm,
                         double fx, double fy, double fz,
                         double qx, double qy, double qz, double qw) const {
    Pose flange{{fx, fy, fz}, {qx, qy, qz, qw}};
    return convert(arm, flange);
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
    Pose flange{{fx, fy, fz}, {qx, qy, qz, qw}};
    return convertZxz(arm, flange);
}

std::vector<std::string> PiperToCam::arms() const {
    std::vector<std::string> r;
    for (auto& kv : cfg_) r.push_back(kv.first);
    return r;
}

// ============= PiperHandEyeCalib =============

static std::vector<double> readVec(const CfgNode& n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

namespace {

class HandEyeErrorFunctor {
public:
    HandEyeErrorFunctor(const std::vector<HandEyeDataPoint>* data)
        : data_(data) {}

    bool operator()(double const* const* vars, double* residual) const {
        const double* ccs = vars[0];   // [tx,ty,tz, rx,ry,rz] (m, deg ZXZ)
        const double* cb  = vars[1];   // [tx,ty,tz, rx,ry,rz] (m, deg ZXZ)

        Pt3 ccs_t{ccs[0], ccs[1], ccs[2]}, ccs_r{ccs[3], ccs[4], ccs[5]};
        Pt3 cb_t{cb[0], cb[1], cb[2]}, cb_r{cb[3], cb[4], cb[5]};

        Quat arm_q = zxzToQuat(ccs_r.x, ccs_r.y, ccs_r.z);
        Pose arm_in_ccs{ccs_t, arm_q};

        int idx = 0;
        for (auto& dp : *data_) {
            Pose calib_in_flange = computeToolPoseTransFirst(dp.flange, cb_t, cb_r);
            Pose pred = composePoses(arm_in_ccs, calib_in_flange);

            residual[idx++] = pred.pos.x - dp.board_gt_ccs.pos.x;
            residual[idx++] = pred.pos.y - dp.board_gt_ccs.pos.y;
            residual[idx++] = pred.pos.z - dp.board_gt_ccs.pos.z;

            Quat q_err = quatMultiply(quatConjugate(pred.quat), dp.board_gt_ccs.quat);
            residual[idx++] = q_err.x;
            residual[idx++] = q_err.y;
            residual[idx++] = q_err.z;
        }
        return true;
    }

private:
    const std::vector<HandEyeDataPoint>* data_;
};

} // anonymous namespace

PiperHandEyeCalib::PiperHandEyeCalib(const std::string& yaml_path)
    : yaml_path_(yaml_path)
{
    Cfg cfg(yaml_path);
    for (auto& arm_name : {"upper", "lower"}) {
        try {
            auto& a = cfg["arms"][arm_name];
            auto& ca = a["calib_arm_in_ccs"];
            auto& cb = a["calib_board_in_flange"];

            ArmInit ai;
            ai.ct = readVec(ca["translation"]["init_value"]);
            ai.cr = readVec(ca["rotation_zxz"]["init_value"]);
            ai.bt = readVec(cb["translation"]["init_value"]);
            ai.br = readVec(cb["rotation_zxz"]["init_value"]);

            auto readBnd = [](const CfgNode& n) -> std::vector<std::pair<double,double>> {
                std::vector<std::pair<double,double>> r;
                for (size_t i = 0; i < 3; ++i) {
                    auto& b = n[i];
                    r.emplace_back(b[0].as<double>(), b[1].as<double>());
                }
                return r;
            };
            Bounds cbnd, abnd;
            abnd.t = readBnd(ca["translation"]["bounds"]);
            abnd.r = readBnd(ca["rotation_zxz"]["bounds"]);
            cbnd.t = readBnd(cb["translation"]["bounds"]);
            cbnd.r = readBnd(cb["rotation_zxz"]["bounds"]);

            configs_[arm_name] = {ai, {abnd, cbnd}};
        } catch (...) {}
    }
    if (configs_.empty())
        throw std::runtime_error("[PiperHandEyeCalib] No arm configs in " + yaml_path);
}

std::vector<double> PiperHandEyeCalib::calibrate(
    const std::string& arm, const std::vector<HandEyeDataPoint>& data) const
{
    auto it = configs_.find(arm);
    if (it == configs_.end())
        throw std::runtime_error("[PiperHandEyeCalib] Unknown arm: " + arm);

    const auto& ai = it->second.first;
    const auto& [abnd, cbnd] = it->second.second;

    std::vector<double> ccs_init = ai.ct; ccs_init.insert(ccs_init.end(), ai.cr.begin(), ai.cr.end());
    std::vector<double> cb_init  = ai.bt; cb_init.insert(cb_init.end(), ai.br.begin(), ai.br.end());
    assert(ccs_init.size() == 6 && cb_init.size() == 6);

    ceres::Problem problem;

    auto* cost_func = new ceres::DynamicNumericDiffCostFunction<HandEyeErrorFunctor, ceres::CENTRAL>(
        new HandEyeErrorFunctor(&data));
    cost_func->AddParameterBlock(6);
    cost_func->AddParameterBlock(6);
    cost_func->SetNumResiduals(static_cast<int>(data.size()) * 6);

    auto* ccs_var = new double[6]; memcpy(ccs_var, ccs_init.data(), 6 * sizeof(double));
    auto* cb_var  = new double[6]; memcpy(cb_var,  cb_init.data(),  6 * sizeof(double));

    std::vector<double*> vars{ccs_var, cb_var};
    problem.AddResidualBlock(cost_func, nullptr, vars);

    for (int j = 0; j < 3; ++j) {
        problem.SetParameterLowerBound(ccs_var, j, abnd.t[j].first);
        problem.SetParameterUpperBound(ccs_var, j, abnd.t[j].second);
        problem.SetParameterLowerBound(cb_var, j, cbnd.t[j].first);
        problem.SetParameterUpperBound(cb_var, j, cbnd.t[j].second);
    }
    for (int j = 3; j < 6; ++j) {
        int rj = j - 3;
        problem.SetParameterLowerBound(ccs_var, j, abnd.r[rj].first);
        problem.SetParameterUpperBound(ccs_var, j, abnd.r[rj].second);
        problem.SetParameterLowerBound(cb_var, j, cbnd.r[rj].first);
        problem.SetParameterUpperBound(cb_var, j, cbnd.r[rj].second);
    }

    ceres::Solver::Options opts;
    opts.minimizer_progress_to_stdout = true;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = 1000;

    ceres::Solver::Summary sum;
    ceres::Solve(opts, &problem, &sum);
    std::cout << sum.BriefReport() << "\n";

    std::vector<double> result(ccs_var, ccs_var + 6);
    delete[] ccs_var; delete[] cb_var;
    return result;
}

void PiperHandEyeCalib::saveArmInCcs(const std::string& arm,
                                      const std::vector<double>& result) const {
    Cfg cfg(yaml_path_);
    std::string base = "arms." + arm + ".calib_arm_in_ccs.";
    cfg.setVector<double>(base + "translation.init_value",
                          {result[0], result[1], result[2]}, 4);
    cfg.setVector<double>(base + "rotation_zxz.init_value",
                          {result[3], result[4], result[5]}, 4);
    cfg.save();
    std::cout << "[PiperHandEyeCalib] " << arm << " arm_in_ccs saved to "
              << yaml_path_ << std::endl;
}

} // namespace gazeestimation
