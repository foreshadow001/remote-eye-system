// 手眼标定: 求解 arm_in_ccs (arm base in CCS) + calib_board_in_flange
// upper / lower 独立标定
//
// 标定结果写入 cfg/arm_pose/{day_id}.yaml (节点结构对齐 piper.yaml,
// 仅含 capture_with_LED.cpp 正向计算工具末端位姿所需参数), 不再写回 piper.yaml。
// 标定完成后立即运行 save_piper_chain 功能 (坐标系链可视化输出)。
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <map>
#include <set>
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

static Pt3 readPt3(const CfgNode& n) { return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()}; }

// 标定结果结构 (用于写 arm_pose yaml + 链可视化)
struct ArmCalibResult {
    string arm;
    vector<double> ccs;   // arm_in_ccs: t(3) + r_zxz(3)
    vector<double> cb;    // calib_board_in_flange: t(3) + r_zxz(3)
    bool valid = false;
};

// ================== 写 cfg/arm_pose/{day_id}.yaml ==================
// 节点结构对齐 piper.yaml, 仅含正向计算工具末端位姿所需参数: tool + arm_in_ccs
void writeArmPoseYaml(const fs::path& path, const Cfg& piper_cfg,
                      const vector<ArmCalibResult>& results) {
    fs::create_directories(path.parent_path());
    ofstream out(path);
    out << "# 手眼标定结果 (节点结构对齐 piper.yaml, 仅含正向计算所需参数)\n";
    out << "# 由 test_piper_hand_eye_calib.cpp 生成\n\n";
    out << "arms:\n";
    for (auto& r : results) {
        if (!r.valid) continue;
        out << "  " << r.arm << ":\n";
        // tool: 从 piper.yaml 原样复制 (固定机械参数)
        Pt3 tl_t{0,0,0}, tl_r{0,0,0};
        try {
            auto& a = piper_cfg["arms"][r.arm]["tool"];
            tl_t = readPt3(a["translation"]);
            tl_r = readPt3(a["rotation_zxz"]);
        } catch (...) {}
        out << fixed << setprecision(4);
        out << "    tool:\n";
        out << "      translation: [" << tl_t.x << ", " << tl_t.y << ", " << tl_t.z << "]\n";
        out << "      rotation_zxz: [" << tl_r.x << ", " << tl_r.y << ", " << tl_r.z << "]\n";
        // arm_in_ccs: 标定结果
        out << "    arm_in_ccs:\n";
        out << "      translation: [" << r.ccs[0] << ", " << r.ccs[1] << ", " << r.ccs[2] << "]\n";
        out << "      rotation_zxz: [" << r.ccs[3] << ", " << r.ccs[4] << ", " << r.ccs[5] << "]\n";
        // board_in_flange: 标定结果 (save_piper_chain 计算 calib board 位姿所需)
        out << "    board_in_flange:\n";
        out << "      translation: [" << r.cb[0] << ", " << r.cb[1] << ", " << r.cb[2] << "]\n";
        out << "      rotation_zxz: [" << r.cb[3] << ", " << r.cb[4] << ", " << r.cb[5] << "]\n";
    }
    out.close();
    cout << "[ArmPose] Written: " << path.string() << endl;
}

// ================== save_piper_chain 功能 (合并自 save_piper_chain.cpp) ==================
void saveChainViz(const string& arm_name, const string& data_dir,
                  Pose arm_in_ccs, Pt3 cb_t, Pt3 cb_r,
                  const map<int, Pose>& flanges, const map<int, Pose>& board_gt) {
    string out_path = data_dir + "/chain_viz_" + arm_name + ".txt";
    ofstream out(out_path); out << fixed << setprecision(6);

    // header: arm_in_ccs
    out << "# arm_in_ccs: ";
    out << arm_in_ccs.pos.x << " " << arm_in_ccs.pos.y << " " << arm_in_ccs.pos.z << " ";
    out << arm_in_ccs.quat.x << " " << arm_in_ccs.quat.y << " " << arm_in_ccs.quat.z << " "
        << arm_in_ccs.quat.w << "\n";
    out << "# frame flange_ccs(7) calib_ccs(7) board_gt_ccs(7)\n";
    out << "# each group: x y z qx qy qz qw\n";

    int count = 0;
    for (auto& kv : flanges) {
        int fi = kv.first;
        Pose flange_raw = kv.second;

        // 1) Flange in CCS = T_arm_in_ccs * T_flange
        Pose flange_ccs = composePoses(arm_in_ccs, flange_raw);

        // 2) Calib board in CCS = T_arm_in_ccs * T_flange * T_calib_in_flange
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
    out.close();
    cout << "[ChainViz] " << arm_name << ": " << count << " frames -> " << out_path << endl;
}

int main() {
    auto cfg_root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                    / "cfg";
    Cfg cfg((cfg_root / "piper.yaml").string());
    Cfg arm_cfg((cfg_root / "calib_arm.yaml").string());

    // 数据目录 = calib_arm.yaml: record.calib_save_dir / day_id
    string day_id   = arm_cfg["record"]["day_id"].as<string>();
    string data_dir = arm_cfg["record"]["calib_save_dir"].as<string>() + "/" + day_id;
    string board_dir = data_dir;

    // 标定结果输出 = cfg/arm_pose/{day_id}.yaml
    fs::path arm_pose_path = cfg_root / "arm_pose" / (day_id + ".yaml");

    // save_piper_chain 配置 (默认 false)
    bool use_calib = false;
    try { use_calib = cfg["save_piper_chain"]["use_calibrated_arm_in_ccs"].as<bool>(); }
    catch (...) {}

    cout << "=== Hand-Eye Calibration (day: " << day_id << ") ===" << endl;
    cout << "Data dir:   " << data_dir << endl;
    cout << "Result yaml:" << arm_pose_path.string() << endl;
    cout << "Chain viz use_calibrated: " << (use_calib ? "true" : "false") << endl;
    cout << "=========================================\n" << endl;

    PiperHandEyeCalib calib((cfg_root / "piper.yaml").string());
    // 旋转权重从 calib_arm.yaml 读取后注入
    try { calib.setRotationWeight(arm_cfg["hand_eye_calib"]["rotation_weight"].as<double>()); }
    catch (...) {}

    vector<ArmCalibResult> results;

    for (auto& arm : {"upper", "lower"}) {
        auto fl = loadFlange(data_dir + "/flange_pose_mapping_" + arm + ".txt");
        auto gt = loadBoardGT(board_dir + "/board_poses_" + arm + ".txt");
        if (fl.empty() || gt.empty()) {
            cout << arm << ": no data, skipping." << endl;
            continue;
        }

        // 读取该 arm 允许的相机 SN 列表 (calib_arm.yaml)
        vector<string> allowed_sns;
        try { allowed_sns = arm_cfg["hand_eye_calib"][arm].as<vector<string>>(); }
        catch (...) {}
        // 构建允许帧集合: 任一 allowed SN 有照片的帧索引
        set<int> allowed_frames;
        if (!allowed_sns.empty()) {
            for (auto& kv : fl) {
                int fi = kv.first;
                stringstream ssf; ssf << setw(2) << setfill('0') << fi;
                for (auto& sn : allowed_sns) {
                    string ip = data_dir + "/calib_cam_" + sn + "_" + ssf.str() + ".jpg";
                    if (fs::exists(ip)) { allowed_frames.insert(fi); break; }
                }
            }
            cout << "  Allowed cameras: " << allowed_sns.size()
                 << " → " << allowed_frames.size() << " valid frames" << endl;
        }

        vector<HandEyeDataPoint> data;
        for (auto& kv : fl) {
            int fi = kv.first;
            if (!allowed_sns.empty() && !allowed_frames.count(fi)) continue;
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
            double tr = 4.0 * qe.w * qe.w - 1.0;
            double angle = acos(clamp((tr - 1.0) / 2.0, -1.0, 1.0));
            if (!isnan(angle)) sum_rot_deg += angle * 180.0 / M_PI;
        }
        cout << fixed << setprecision(2);
        cout << "Avg pos error: " << (sum_xyz / data.size()) << " mm  |  "
             << "Avg rot error: " << (sum_rot_deg / data.size()) << " deg" << endl;

        // 记录结果 (不再写回 piper.yaml)
        results.push_back({string(arm), ccs_result, cb_result, true});

        // ---- 标定完成后立即运行 save_piper_chain 功能 ----
        Pt3 viz_ccs_t, viz_ccs_r, viz_cb_t, viz_cb_r;
        if (use_calib) {
            // 使用刚标定的结果
            viz_ccs_t = ccs_t; viz_ccs_r = ccs_r;
            viz_cb_t  = cb_t;  viz_cb_r  = cb_r;
            cout << "  chain_viz: using calibrated arm_in_ccs + calib_board_in_flange" << endl;
        } else {
            // 使用 piper.yaml 的 init_value
            try {
                auto& a = cfg["arms"][arm];
                auto& cb = a["calib_board_in_flange"];
                viz_cb_t = readPt3(cb["translation"]["init_value"]);
                viz_cb_r = readPt3(cb["rotation_zxz"]["init_value"]);
                auto& ca = a["calib_arm_in_ccs"];
                viz_ccs_t = readPt3(ca["translation"]["init_value"]);
                viz_ccs_r = readPt3(ca["rotation_zxz"]["init_value"]);
                cout << "  chain_viz: using init_value" << endl;
            } catch (...) {
                cerr << "  chain_viz: init_value not configured, skipping." << endl;
                continue;
            }
        }
        Pose viz_arm_in_ccs = {viz_ccs_t, zxzToQuat(viz_ccs_r.x, viz_ccs_r.y, viz_ccs_r.z)};
        saveChainViz(string(arm), data_dir, viz_arm_in_ccs, viz_cb_t, viz_cb_r, fl, gt);
    }

    // ---- 写标定结果 yaml ----
    if (!results.empty()) {
        writeArmPoseYaml(arm_pose_path, cfg, results);
    }
    return 0;
}
