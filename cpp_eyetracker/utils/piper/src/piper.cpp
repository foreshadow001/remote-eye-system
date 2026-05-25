#include "piper/piper.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gazeestimation {

static double deg2rad(double d) { return d * M_PI / 180.0; }

// ============= Z-X-Z'' Euler → quaternion (matches end_pose_monitor.py) =============

Quat zxzToQuat(double alpha_deg, double beta_deg, double gamma_deg) {
    double a = deg2rad(alpha_deg), b = deg2rad(beta_deg), g = deg2rad(gamma_deg);
    double c1 = cos(a/2), s1 = sin(a/2);
    double c2 = cos(b/2), s2 = sin(b/2);
    double c3 = cos(g/2), s3 = sin(g/2);
    // q = q_z(α) * q_x(β) * q_z(γ)
    Quat q;
    q.w = c1*c2*c3 - s1*c2*s3;
    q.x = c1*s2*c3 + s1*s2*s3;
    q.y = s1*s2*c3 - c1*s2*s3;
    q.z = s1*c2*c3 + c1*c2*s3;
    return q;
}

// ============= Hamilton product q1 * q0 =============

Quat quatMultiply(const Quat& q1, const Quat& q0) {
    Quat r;
    r.x = q1.w*q0.x + q1.x*q0.w + q1.y*q0.z - q1.z*q0.y;
    r.y = q1.w*q0.y - q1.x*q0.z + q1.y*q0.w + q1.z*q0.x;
    r.z = q1.w*q0.z + q1.x*q0.y - q1.y*q0.x + q1.z*q0.w;
    r.w = q1.w*q0.w - q1.x*q0.x - q1.y*q0.y - q1.z*q0.z;
    return r;
}

// ============= Rotate vector by quaternion =============

Pt3 quatRotate(const Quat& q, const Pt3& v) {
    Quat qv{v.x, v.y, v.z, 0.0};
    Quat qc{-q.x, -q.y, -q.z, q.w}; // conjugate
    Quat t = quatMultiply(q, qv);
    Quat r = quatMultiply(t, qc);
    return {r.x, r.y, r.z};
}

// ============= computeToolPose (matches end_pose_monitor.py) =============
// T_tool = T_flange * T_offset
// T_offset = translate(tool_trans) * rotate(tool_rot_zxz)

Pose computeToolPose(const Pose& flange, const Pt3& tool_trans,
                     const Pt3& tool_rot_zxz_deg) {
    Quat q_offset = zxzToQuat(tool_rot_zxz_deg.x, tool_rot_zxz_deg.y, tool_rot_zxz_deg.z);

    // p_tool = p_flange + R_flange * (R_offset * t)
    Pt3 t_rotated = quatRotate(q_offset, tool_trans);
    Pt3 t_world   = quatRotate(flange.quat, t_rotated);

    Pose tool;
    tool.pos.x = flange.pos.x + t_world.x;
    tool.pos.y = flange.pos.y + t_world.y;
    tool.pos.z = flange.pos.z + t_world.z;
    tool.quat  = quatMultiply(flange.quat, q_offset);
    return tool;
}

// ============= composePoses (matches end_pose_monitor.py) =============
// T_result = T_base * T_offset

Pose composePoses(const Pose& base, const Pose& offset) {
    Pt3 p_rot = quatRotate(base.quat, offset.pos);
    Pose r;
    r.pos.x = base.pos.x + p_rot.x;
    r.pos.y = base.pos.y + p_rot.y;
    r.pos.z = base.pos.z + p_rot.z;
    r.quat  = quatMultiply(base.quat, offset.quat);
    return r;
}

// ============= Full chain =============

Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg) {
    Pose tool_in_arm = computeToolPose(flange, tool_trans, tool_rot_zxz_deg);

    Quat arm_quat = zxzToQuat(arm_rot_zxz_deg.x, arm_rot_zxz_deg.y, arm_rot_zxz_deg.z);
    Pose arm_in_ccs;
    arm_in_ccs.pos  = arm_trans;
    arm_in_ccs.quat = arm_quat;

    return composePoses(arm_in_ccs, tool_in_arm);
}

} // namespace gazeestimation
