// 标定 arm_in_ccs: Ceres 优化求解, 结果写回 piper.yaml
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <map>
#include <cmath>

#include "piper/piper.hpp"
#include "cfg/config.hpp"

using namespace std;
using namespace gazeestimation;
namespace fs = std::filesystem;

int main() {
    auto yaml_path = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                      / "cfg" / "piper.yaml").string();
    Cfg cfg(yaml_path);
    string base_dir = cfg["test_record_arm_data"]["calib_save_dir"].as<string>();

    cout << "=== Piper Arm Calibration ===" << endl;
    cout << "Config:   " << yaml_path << endl;
    cout << "Data dir: " << base_dir << endl;

    for (auto& arm : {"upper", "lower"}) {
        // 读取 flange 位姿
        map<int, Pose> flange_map;
        {
            string fp = base_dir + "/flange_pose_mapping_" + arm + ".txt";
            ifstream in(fp);
            if (!in) { cout << "[Arm " << arm << "] " << fp << " not found, skip." << endl; continue; }
            string line;
            while (getline(in, line)) {
                if (line.empty() || line[0] == '#') continue;
                stringstream ss(line); string idx_str; ss >> idx_str;
                int idx = stoi(idx_str);
                // format: idx arm x y z qx qy qz qw alpha beta gamma
                string an; double f[10];
                ss >> an;
                for (int i = 0; i < 10; i++) ss >> f[i];
                flange_map[idx] = {{f[0], f[1], f[2]}, {f[3], f[4], f[5], f[6]}};
            }
        }

        // 读取 board-in-CCS 观测
        map<int, PoseZxz> board_map;
        {
            string fp = base_dir + "/board_poses_" + arm + ".txt";
            ifstream in(fp);
            if (!in) { cout << "[Arm " << arm << "] " << fp << " not found, skip." << endl; continue; }
            string line;
            while (getline(in, line)) {
                if (line.empty() || line[0] == '#') continue;
                stringstream ss(line);
                string idx_str; ss >> idx_str;
                int idx = stoi(idx_str);
                double f[7]; // x y z alpha beta gamma n_cams
                for (int i = 0; i < 7; i++) ss >> f[i];
                board_map[idx] = {{f[0], f[1], f[2]}, f[3], f[4], f[5]};
            }
        }

        // 匹配观测
        PiperArmCalibrator calib(yaml_path, arm);
        int matched = 0;
        for (auto& kv : flange_map) {
            int idx = kv.first;
            auto bit = board_map.find(idx);
            if (bit == board_map.end()) continue;
            calib.addObservation(kv.second, bit->second);
            matched++;
        }
        cout << "\n[Arm " << arm << "] " << matched << " matched observations." << endl;

        if (!calib.solve()) {
            cerr << "[Arm " << arm << "] Solve failed!" << endl;
            continue;
        }

        Pt3 at = calib.armTrans(), ar = calib.armRotZxz();
        cout << fixed << setprecision(4);
        cout << "[Arm " << arm << "] arm_in_ccs solved:" << endl;
        cout << "  translation:     [" << at.x << ", " << at.y << ", " << at.z << "] m" << endl;
        cout << setprecision(1);
        cout << "  rotation Z-X-Z'': [" << ar.x << ", " << ar.y << ", " << ar.z << "] deg" << endl;

        calib.writeBack();
        cout << "[Arm " << arm << "] Written back to piper.yaml" << endl;
    }

    cout << "\n=== Done ===" << endl;
    return 0;
}
