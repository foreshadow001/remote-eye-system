// 保存坐标系转换链中间量，供 viz_piper_chain.py 可视化
// 使用 piper.yaml 中 calib_board_in_flange 和 calib_arm_in_ccs 的 init_value
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <filesystem>
#include <cmath>
#include "piper/piper.hpp"
#include "cfg/config.hpp"

using namespace std;
using namespace gazeestimation;
namespace fs = std::filesystem;

static Pt3 readPt3(const CfgNode& n) { return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()}; }

static map<int, Pose> loadFlange(const string& path) {
    map<int, Pose> m;
    if (!fs::exists(path)) return m;
    ifstream in(path); string line;
    while (getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line); string is, an; ss >> is >> an;
        int idx = stoi(is);
        double x, y, z, qx, qy, qz, qw, a, b, g;
        ss >> x >> y >> z >> qx >> qy >> qz >> qw >> a >> b >> g;
        m[idx] = {{x, y, z}, {qx, qy, qz, qw}};
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
        int idx = stoi(is);
        double x, y, z, a, b, g; int nc;
        ss >> x >> y >> z >> a >> b >> g >> nc;
        m[idx] = {{x, y, z}, zxzToQuat(a, b, g)};
    }
    return m;
}

int main() {
    auto yp = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
               / "cfg" / "piper.yaml").string();
    Cfg cfg(yp);
    string data_dir  = cfg["test_record_arm_data"]["calib_save_dir"].as<string>();
    string board_dir = cfg["test_record_arm_data"]["calib_save_dir"].as<string>();

    for (auto& arm_name : {"upper", "lower"}) {
        cout << "\n===== Arm: " << arm_name << " =====" << endl;

        // 读取 calib_board_in_flange + calib_arm_in_ccs (init_value)
        Pt3 cb_t{0,0,0}, cb_r{0,0,0}, ccs_t{0,0,0}, ccs_r{0,0,0};
        try {
            auto& a = cfg["arms"][arm_name];
            auto& cb = a["calib_board_in_flange"];
            auto& ca = a["calib_arm_in_ccs"];
            cb_t = readPt3(cb["translation"]["init_value"]);
            cb_r = readPt3(cb["rotation_zxz"]["init_value"]);
            ccs_t = readPt3(ca["translation"]["init_value"]);
            ccs_r = readPt3(ca["rotation_zxz"]["init_value"]);
        } catch (...) { cout << "Not configured, skipping." << endl; continue; }

        Pose arm_in_ccs = {{ccs_t.x, ccs_t.y, ccs_t.z},
                            zxzToQuat(ccs_r.x, ccs_r.y, ccs_r.z)};

        auto flanges = loadFlange(data_dir + "/flange_pose_mapping_" + arm_name + ".txt");
        auto board_gt = loadBoardGT(board_dir + "/board_poses_" + arm_name + ".txt");
        if (flanges.empty()) { cout << "No flange data." << endl; continue; }

        string out_path = board_dir + "/chain_viz_" + arm_name + ".txt";
        ofstream out(out_path); out << fixed << setprecision(6);

        // header: arm_in_ccs
        out << "# arm_in_ccs: ";
        out << ccs_t.x << " " << ccs_t.y << " " << ccs_t.z << " ";
        Quat aq = arm_in_ccs.quat;
        out << aq.x << " " << aq.y << " " << aq.z << " " << aq.w << "\n";
        out << "# frame flange_ccs(7) calib_ccs(7) board_gt_ccs(7)\n";
        out << "# each group: x y z qx qy qz qw\n";

        int count = 0;
        for (auto& kv : flanges) {
            int fi = kv.first;
            Pose flange_raw = kv.second;

            // 1) Flange in CCS = T_arm_in_ccs * T_flange
            Pose flange_ccs = composePoses(arm_in_ccs, flange_raw);

            // 2) Calib board in CCS = T_arm_in_ccs * T_flange * T_calib_in_flange
            //    平移在 flange 坐标系下解释 (不受 calib 旋转影响)
            Pose calib_in_flange = computeToolPoseTransFirst(flange_raw, cb_t, cb_r);
            Pose calib_ccs = composePoses(arm_in_ccs, calib_in_flange);

            // 3) Board ground truth (CCS)
            Pose gt{NAN, NAN, NAN, NAN, NAN, NAN, NAN};
            auto it = board_gt.find(fi);
            if (it != board_gt.end()) gt = it->second;

            out << setw(2) << setfill('0') << fi << " ";
            auto wp = [&](const Pose& p) {
                out << p.pos.x << " " << p.pos.y << " " << p.pos.z << " "
                    << p.quat.x << " " << p.quat.y << " " << p.quat.z << " " << p.quat.w << " ";
            };
            wp(flange_ccs); wp(calib_ccs); wp(gt);
            out << "\n";
            count++;
        }
        cout << arm_name << ": " << count << " frames -> " << out_path << endl;
    }
    return 0;
}
