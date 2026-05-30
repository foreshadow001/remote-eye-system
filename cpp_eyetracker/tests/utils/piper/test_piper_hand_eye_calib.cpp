// 手眼标定: 求解 arm_in_ccs (arm base in CCS), 写回 piper.yaml
// upper / lower 独立标定
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <map>
#include "piper/piper.hpp"
#include "cfg/config.hpp"

using namespace std;
using namespace gazeestimation;
namespace fs = std::filesystem;

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

        calib.saveArmInCcs(arm, result);
    }
    return 0;
}
