// 手眼标定: 求解 arm_in_ccs (arm base in CCS), 写回 piper.yaml
// upper / lower 独立标定
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <map>
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

        auto result = calib.calibrate(arm, data);
        cout << fixed << setprecision(4);
        cout << "arm_in_ccs: t=[" << result[0] << "," << result[1] << "," << result[2]
             << "]m  r_zxz=[" << result[3] << "," << result[4] << "," << result[5] << "]deg" << endl;

        // 计算平均误差
        double sum_xyz = 0, sum_rot_deg = 0;
        Pt3 ccs_t{result[0], result[1], result[2]}, ccs_r{result[3], result[4], result[5]};
        Quat arm_q = zxzToQuat(ccs_r.x, ccs_r.y, ccs_r.z);
        Pose arm_in_ccs{ccs_t, arm_q};
        // calib_board params from config (fixed per arm)
        Pt3 cb_t{0,0,0}, cb_r{0,0,0};
        try {
            auto& a = cfg["arms"][arm]["calib_board_in_flange"];
            auto& tt = a["translation"]["init_value"];
            auto& tr = a["rotation_zxz"]["init_value"];
            cb_t = {tt[0].as<double>(), tt[1].as<double>(), tt[2].as<double>()};
            cb_r = {tr[0].as<double>(), tr[1].as<double>(), tr[2].as<double>()};
        } catch (...) {}

        for (auto& dp : data) {
            Pose calib_in_flange = computeToolPoseTransFirst(dp.flange, cb_t, cb_r);
            Pose pred = composePoses(arm_in_ccs, calib_in_flange);
            double dx = pred.pos.x - dp.board_gt_ccs.pos.x;
            double dy = pred.pos.y - dp.board_gt_ccs.pos.y;
            double dz = pred.pos.z - dp.board_gt_ccs.pos.z;
            sum_xyz += sqrt(dx*dx + dy*dy + dz*dz) * 1000.0;  // mm
            Quat qe = quatMultiply(quatConjugate(pred.quat), dp.board_gt_ccs.quat);
            double angle = 2.0 * acos(clamp(qe.w, -1.0, 1.0));
            if (!isnan(angle)) sum_rot_deg += angle * 180.0 / M_PI;
        }
        cout << fixed << setprecision(2);
        cout << "Avg pos error: " << (sum_xyz / data.size()) << " mm  |  "
             << "Avg rot error: " << (sum_rot_deg / data.size()) << " deg" << endl;

        calib.saveArmInCcs(arm, result);
    }
    return 0;
}
