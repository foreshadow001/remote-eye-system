#pragma once

namespace gazeestimation {

struct Quat { double x = 0, y = 0, z = 0, w = 1; };   // (x,y,z)=vector, w=scalar
struct Pt3  { double x = 0, y = 0, z = 0; };
struct Pose { Pt3 pos; Quat quat; };

// Z-X-Z'' intrinsic Euler (degrees) → quaternion [x,y,z,w]
Quat zxzToQuat(double alpha_deg, double beta_deg, double gamma_deg);

// Hamilton product q1*q0 (apply q0 first, then q1)
Quat quatMultiply(const Quat& q1, const Quat& q0);

// Rotate vector v by quaternion q: R(q)*v
Pt3 quatRotate(const Quat& q, const Pt3& v);

// T_tool_in_arm = T_flange * T_tool_offset
// tool_trans: flange-frame translation (m)
// tool_rot_zxz_deg: flange-frame Z-X-Z'' rotation (deg)
Pose computeToolPose(const Pose& flange, const Pt3& tool_trans,
                     const Pt3& tool_rot_zxz_deg);

// T_result = T_base * T_offset
Pose composePoses(const Pose& base, const Pose& offset);

// Full chain: flange → tool_in_arm → tool_in_ccs
Pose armToolToCamPose(const Pose& flange,
                      const Pt3& tool_trans, const Pt3& tool_rot_zxz_deg,
                      const Pt3& arm_trans,   const Pt3& arm_rot_zxz_deg);

} // namespace gazeestimation
