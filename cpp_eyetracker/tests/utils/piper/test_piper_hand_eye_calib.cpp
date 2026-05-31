// 手眼标定: 求解 arm_in_ccs (arm base in CCS), 写回 piper.yaml
// upper / lower 独立标定
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <map>
#include <utility>
#include <cmath>
#include "piper/piper.hpp"
#include "cfg/config.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace gazeestimation;
namespace fs = std::filesystem;

static double clamp(double v, double lo, double hi) { return max(lo, min(hi, v)); }

static map<int, Pose> loadFlange(const string& path) {
    map<int, Pose> m;
    if (!fs::exists(path)) return m;
    ifstream in(path); string line;
    while (getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line); string is, an; ss >> is >> an;
        double x,y,z,qx,qy,qz,qw,a,b,g;
        ss >> x>>y>>z>>qx>>qy>>qz>>qw>>a>>b>>g;
        m[stoi(is)] = {{x,y,z},{qx,qy,qz,qw}};
    }
    return m;
}

static map<int, Pose> loadBoardGT(const string& path) {
    map<int, Pose> m;
    if (!fs::exists(path)) return m;
    ifstream in(path); string line;
    while (getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line); string is; ss >> is;
        double x,y,z,qw,qx,qy,qz; int nc;
        ss >> x>>y>>z>>qw>>qx>>qy>>qz>>nc;
        m[stoi(is)] = {{x,y,z},{qx,qy,qz,qw}};
    }
    return m;
}

int main() {
    auto yp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
               / "cfg" / "piper.yaml").string();
    Cfg cfg(yp);
    string data_dir  = cfg["test_record_arm_data"]["calib_save_dir"].as<string>();
    string board_dir = data_dir;

    PiperHandEyeCalib calib(yp);

    for (auto& arm : {"upper", "lower"}) {
        auto fl = loadFlange(data_dir + "/flange_pose_mapping_" + arm + ".txt");
        auto gt = loadBoardGT(board_dir + "/board_poses_" + arm + ".txt");
        if (fl.empty() || gt.empty()) {
            cout << arm << ": no data, skipping." << endl;
            continue;
        }

        // Build data pairs (matching frame indices)
        vector<HandEyeDataPoint> data;
        for (auto& kv : fl) {
            int fi = kv.first;
            auto git = gt.find(fi);
            if (git == gt.end()) continue;
            data.push_back({kv.second, git->second});
        }
        cout << "\n===== " << arm << ": " << data.size() << " pairs =====" << endl;
        if (data.size() < 3) { cout << "Too few pairs, skipping." << endl; continue; }

        auto [ccs_result, cb_result] = calib.calibrate(arm, data);
        cout << fixed << setprecision(4);
        cout << "arm_in_ccs:       t=[" << ccs_result[0] << "," << ccs_result[1] << "," << ccs_result[2]
             << "]m  r_zxz=[" << ccs_result[3] << "," << ccs_result[4] << "," << ccs_result[5] << "]deg" << endl;
        cout << "calib_board_flange: t=[" << cb_result[0] << "," << cb_result[1] << "," << cb_result[2]
             << "]m  r_zxz=[" << cb_result[3] << "," << cb_result[4] << "," << cb_result[5] << "]deg" << endl;

        // 检查结果是否在 bounds 内
        auto chkBnd = [&](const string& label, const vector<double>& v,
                          const vector<pair<double,double>>& tb,
                          const vector<pair<double,double>>& rb) {
            for (int j = 0; j < 3; ++j) {
                double lo = tb[j].first, hi = tb[j].second;
                if (lo > hi) swap(lo, hi);
                bool ok = (v[j] >= lo - 1e-6 && v[j] <= hi + 1e-6);
                cerr << "[CHK] " << arm << " " << label << "_t[" << j << "]=" << v[j]
                     << " in [" << lo << "," << hi << "] " << (ok ? "OK" : "VIOLATED") << endl;
            }
            for (int j = 3; j < 6; ++j) {
                int rj = j - 3;
                double lo = rb[rj].first, hi = rb[rj].second;
                if (lo > hi) swap(lo, hi);
                bool ok = (v[j] >= lo - 1e-6 && v[j] <= hi + 1e-6);
                cerr << "[CHK] " << arm << " " << label << "_r[" << rj << "]=" << v[j]
                     << " in [" << lo << "," << hi << "] " << (ok ? "OK" : "VIOLATED") << endl;
            }
        };
        auto readBnd = [](const CfgNode& n) -> vector<pair<double,double>> {
            vector<pair<double,double>> r;
            for (size_t i = 0; i < 3; ++i) r.emplace_back(n[i][0].as<double>(), n[i][1].as<double>());
            return r;
        };
        bool ca_ok = false, cb_ok = false;
        try {
            auto& ca = cfg["arms"][arm]["calib_arm_in_ccs"];
            chkBnd("arm_ccs", ccs_result, readBnd(ca["translation"]["bounds"]),
                   readBnd(ca["rotation_zxz"]["bounds"]));
            ca_ok = true;
        } catch (...) { cerr << "[CHK] " << arm << " calib_arm_in_ccs NOT FOUND in yaml" << endl; }
        try {
            auto& cb = cfg["arms"][arm]["calib_board_in_flange"];
            chkBnd("board", cb_result, readBnd(cb["translation"]["bounds"]),
                   readBnd(cb["rotation_zxz"]["bounds"]));
            cb_ok = true;
        } catch (...) { cerr << "[CHK] " << arm << " calib_board_in_flange NOT FOUND in yaml" << endl; }

        // 计算平均误差（用优化后的参数）
        double sum_xyz = 0, sum_rot_deg = 0;
        Pt3 ccs_t{ccs_result[0], ccs_result[1], ccs_result[2]}, ccs_r{ccs_result[3], ccs_result[4], ccs_result[5]};
        Quat arm_q = zxzToQuat(ccs_r.x, ccs_r.y, ccs_r.z);
        Pose arm_in_ccs{ccs_t, arm_q};
        Pt3 cb_t{cb_result[0], cb_result[1], cb_result[2]}, cb_r{cb_result[3], cb_result[4], cb_result[5]};
        for (auto& dp : data) {
            Pose calib_in_flange = computeToolPoseTransFirst(dp.flange, cb_t, cb_r);
            Pose pred = composePoses(arm_in_ccs, calib_in_flange);
            double dx = pred.pos.x - dp.board_gt_ccs.pos.x;
            double dy = pred.pos.y - dp.board_gt_ccs.pos.y;
            double dz = pred.pos.z - dp.board_gt_ccs.pos.z;
            sum_xyz += sqrt(dx*dx + dy*dy + dz*dz) * 1000.0;  // mm
            Quat qe = quatMultiply(quatConjugate(pred.quat), dp.board_gt_ccs.quat);
            // Rotation angle from error quaternion: R_err trace = 4*w² - 1, angle = acos((tr-1)/2)
            double tr = 4.0 * qe.w * qe.w - 1.0;
            double angle = acos(clamp((tr - 1.0) / 2.0, -1.0, 1.0));
            if (!isnan(angle)) sum_rot_deg += angle * 180.0 / M_PI;
        }
        cout << fixed << setprecision(2);
        cout << "Avg pos error: " << (sum_xyz / data.size()) << " mm  |  "
             << "Avg rot error: " << (sum_rot_deg / data.size()) << " deg" << endl;

        calib.saveArmInCcs(arm, ccs_result);
        calib.saveCalibBoard(arm, cb_result);
    }
    return 0;
}
