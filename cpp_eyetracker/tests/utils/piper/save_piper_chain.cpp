// 保存坐标系转换链中间量，供 viz_piper_chain.py 可视化
// 使用 cfg/arm_pose/{day_id}.yaml (手眼标定结果) 的 arm_in_ccs + board_in_flange
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
        double x, y, z, qw, qx, qy, qz; int nc;
        ss >> x >> y >> z >> qw >> qx >> qy >> qz >> nc;
        m[idx] = {{x, y, z}, {qx, qy, qz, qw}};
    }
    return m;
}

int main() {
    auto cfg_dir = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg";
    Cfg arm_cfg((cfg_dir / "calib_arm.yaml").string());
    // 数据在 calib_save_dir/{day_id}/ 下 (day_id 与 test_record_arm_data/resolve 一致)
    string day_id = arm_cfg["record"]["day_id"].as<string>();
    string data_dir  = arm_cfg["record"]["calib_save_dir"].as<string>() + "/" + day_id;
    string board_dir = data_dir;   // board_poses_*.txt 与 chain_viz_*.txt 均在此目录
    cout << "Data dir: " << data_dir << endl;

    // 手眼标定结果 (test_piper_hand_eye_calib 的输出, 不是 piper.yaml)
    fs::path arm_pose_path = cfg_dir / "arm_pose" / (day_id + ".yaml");
    if (!fs::exists(arm_pose_path)) {
        cerr << "[Fatal] Not found: " << arm_pose_path.string()
             << " — run test_piper_hand_eye_calib first." << endl;
        return 1;
    }
    Cfg arm_pose(arm_pose_path.string());
    cout << "Arm pose : " << arm_pose_path.string() << endl;

    for (auto& arm_name : {"upper", "lower"}) {
        cout << "\n===== Arm: " << arm_name << " =====" << endl;

        // arm_in_ccs + board_in_flange: 手眼标定结果
        Pt3 cb_t{0,0,0}, cb_r{0,0,0}, ccs_t{0,0,0}, ccs_r{0,0,0};
        try {
            auto& a = arm_pose["arms"][arm_name];
            auto& cb = a["board_in_flange"];
            cb_t = readPt3(cb["translation"]);
            cb_r = readPt3(cb["rotation_zxz"]);
            auto& ccs = a["arm_in_ccs"];
            ccs_t = readPt3(ccs["translation"]);
            ccs_r = readPt3(ccs["rotation_zxz"]);
            cout << "  arm_in_ccs + board_in_flange: calibrated" << endl;
        } catch (...) { cout << "Not calibrated, skipping." << endl; continue; }

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
