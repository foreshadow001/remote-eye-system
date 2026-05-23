// ================== resolve_calib_board_pose ==================
// 计算标定板在参考相机 (40772280) 坐标系下的位姿 (Z-X-Z'' 欧拉角 + 平移)。
// 多相机结果经变换后平均融合，无法确定的相机自动舍弃。
// 依赖: HALCON
// =================================================================

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "HalconCpp.h"
#include "cfg/config.hpp"

using namespace HalconCpp;
using namespace std;
namespace fs = std::filesystem;

// ================== 数据结构 ==================

struct CamCalibData {
    string sn;
    HTuple cam_param;       // internal params
    HTuple pose_in_ref;     // pose relative to center camera (from XML)
    bool valid = false;
};

struct BoardPose {
    double x, y, z;
    double alpha, beta, gamma;  // Z-X-Z'' Euler, degrees
    bool valid = false;
};

// ================== 辅助函数 ==================

double clamp(double v, double lo, double hi) { return max(lo, min(hi, v)); }

// Halcon pose → Z-X-Z'' Euler (degrees)
void poseToZxz(const HTuple& pose, double& alpha, double& beta, double& gamma) {
    HTuple hom;
    PoseToHomMat3d(pose, &hom);
    double r00 = hom[0].D(),  r01 = hom[1].D(),  r02 = hom[2].D();
    double r10 = hom[4].D(),  r11 = hom[5].D(),  r12 = hom[6].D();
    double r20 = hom[8].D(),  r21 = hom[9].D(),  r22 = hom[10].D();

    beta = acos(clamp(r22, -1.0, 1.0));
    double sb = sin(beta);
    if (fabs(sb) > 1e-6) {
        alpha = atan2(r02, -r12);
        gamma = atan2(r20, r21);
    } else {
        gamma = 0.0;
        if (beta < M_PI / 2.0)
            alpha = atan2(-r01, r00);
        else
            alpha = atan2(r01, r00);
    }
    alpha = alpha * 180.0 / M_PI;
    beta  = beta  * 180.0 / M_PI;
    gamma = gamma * 180.0 / M_PI;
}

void poseToQt(const HTuple& pose, double& qx, double& qy, double& qz, double& qw) {
    HTuple hom;
    PoseToHomMat3d(pose, &hom);
    double r00 = hom[0].D(), r01 = hom[1].D(), r02 = hom[2].D();
    double r10 = hom[4].D(), r11 = hom[5].D(), r12 = hom[6].D();
    double r20 = hom[8].D(), r21 = hom[9].D(), r22 = hom[10].D();
    double tr = r00 + r11 + r22;
    if (tr > 0) {
        double s = sqrt(tr + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (r21 - r12) / s;
        qy = (r02 - r20) / s;
        qz = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        double s = sqrt(1.0 + r00 - r11 - r22) * 2.0;
        qw = (r21 - r12) / s;
        qx = 0.25 * s;
        qy = (r01 + r10) / s;
        qz = (r02 + r20) / s;
    } else if (r11 > r22) {
        double s = sqrt(1.0 + r11 - r00 - r22) * 2.0;
        qw = (r02 - r20) / s;
        qx = (r01 + r10) / s;
        qy = 0.25 * s;
        qz = (r12 + r21) / s;
    } else {
        double s = sqrt(1.0 + r22 - r00 - r11) * 2.0;
        qw = (r10 - r01) / s;
        qx = (r02 + r20) / s;
        qy = (r12 + r21) / s;
        qz = 0.25 * s;
    }
}

HTuple qtToPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
    // quaternion → rotation matrix → Halcon pose
    double n = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx /= n; qy /= n; qz /= n; qw /= n;
    double r00 = 1 - 2*qy*qy - 2*qz*qz, r01 = 2*qx*qy - 2*qz*qw,     r02 = 2*qx*qz + 2*qy*qw;
    double r10 = 2*qx*qy + 2*qz*qw,     r11 = 1 - 2*qx*qx - 2*qz*qz, r12 = 2*qy*qz - 2*qx*qw;
    double r20 = 2*qx*qz - 2*qy*qw,     r21 = 2*qy*qz + 2*qx*qw,     r22 = 1 - 2*qx*qx - 2*qy*qy;

    HTuple hom = HTuple(r00).TupleConcat(r01).TupleConcat(r02).TupleConcat(0)
        .TupleConcat(r10).TupleConcat(r11).TupleConcat(r12).TupleConcat(0)
        .TupleConcat(r20).TupleConcat(r21).TupleConcat(r22).TupleConcat(0)
        .TupleConcat(x).TupleConcat(y).TupleConcat(z).TupleConcat(1);

    HTuple pose;
    HomMat3dToPose(hom, &pose);
    return pose;
}

// 简单 XML 解析 (不依赖 pugixml)
string xmlText(const string& xml, const string& tag) {
    string open = "<" + tag + ">";
    string close = "</" + tag + ">";
    size_t s = xml.find(open);
    if (s == string::npos) return "";
    s += open.length();
    size_t e = xml.find(close, s);
    if (e == string::npos) return "";
    return xml.substr(s, e - s);
}

bool loadCamCalibXml(const string& filepath, HTuple& cam_param, HTuple& pose) {
    ifstream in(filepath);
    if (!in) return false;
    string xml((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());

    // Internal params: <RawData>area_scan_division focus kappa sx sy cx cy w h</RawData>
    string raw = xmlText(xml, "RawData");
    if (raw.empty()) return false;

    // Split raw into tokens
    vector<string> tokens;
    stringstream ss(raw);
    string tok;
    while (ss >> tok) tokens.push_back(tok);
    if (tokens.size() < 9) return false;

    // Build Halcon camera param tuple
    cam_param.Clear();
    cam_param[0] = HTuple(tokens[0].c_str());  // type
    for (size_t i = 1; i < tokens.size(); ++i)
        cam_param = cam_param.TupleConcat(stod(tokens[i]));

    // External params
    double tx = stod(xmlText(xml, "X"));
    double ty = stod(xmlText(xml, "Y"));
    double tz = stod(xmlText(xml, "Z"));
    double alpha = stod(xmlText(xml, "Alpha"));
    double beta  = stod(xmlText(xml, "Beta"));
    double gamma = stod(xmlText(xml, "Gamma"));

    // XML 中的值是 Halcon 原生格式: 平移(m), 旋转(rad), gba 顺序
    CreatePose(tx, ty, tz, alpha, beta, gamma,
               "Rp+T", "gba", "point", &pose);
    return true;
}

// ================== 主程序 ==================

#ifndef NO_EXPORT_APP_MAIN
int main(int argc, char* argv[]) {
    int ret = 0;
    try {
#if defined(_WIN32)
        SetSystem("use_window_thread", "true");
#endif

        // --- 读取配置 ---
        namespace fs = std::filesystem;
        auto piper_path = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                           / "cfg" / "piper.yaml").string();
        Cfg cfg(piper_path);

        auto& rcfg = cfg["resolve_calib_board_pose"];
        string xml_dir = rcfg["cam_xml_dir"].as<string>();
        string img_dir = rcfg["input_image_dir"].as<string>();
        string out_file = rcfg["output_file"].as<string>();
        HTuple hv_calib_plane = rcfg["calib_plane"].as<string>().c_str();

        cout << "=== Resolve Calibration Board Pose ===" << endl;
        cout << "Cam XML dir:      " << xml_dir << endl;
        cout << "Image dir:        " << img_dir << endl;
        cout << "Output file:      " << out_file << endl;
        cout << "Calib plane:      " << hv_calib_plane.S() << endl;
        cout << "======================================\n" << endl;

        // --- 自动扫描 XML 文件, 加载相机标定数据 ---
        // XML 中的外参已经由 calib_cam_chain 重基准到中心相机坐标系
        vector<CamCalibData> cam_data;

        for (auto& entry : fs::directory_iterator(xml_dir)) {
            string fn = entry.path().filename().string();
            if (fn.find("_Data.xml") == string::npos) continue;
            string sn = fn.substr(0, fn.find("_Data.xml"));

            CamCalibData cd;
            cd.sn = sn;
            if (!loadCamCalibXml(entry.path().string(), cd.cam_param, cd.pose_in_ref)) {
                cout << "[Warn] Failed to load " << fn << ", skipping." << endl;
                continue;
            }
            cd.valid = true;
            cam_data.push_back(cd);
        }

        if (cam_data.empty()) {
            cerr << "[Error] No valid camera calibration XMLs found in " << xml_dir << endl;
            return 1;
        }

        // 按 SN 排序保证确定性
        sort(cam_data.begin(), cam_data.end(),
             [](const CamCalibData& a, const CamCalibData& b) { return a.sn < b.sn; });

        cout << "Loaded " << cam_data.size() << " cameras:" << endl;
        for (size_t i = 0; i < cam_data.size(); ++i)
            cout << "  " << i << ": SN=" << cam_data[i].sn
                 << "  focus=" << ((const HTuple&)cam_data[i].cam_param)[1].D() << "m" << endl;
        cout << "(Poses already relative to center camera from calib_cam_chain)\n" << endl;

        // --- 逐帧处理 ---
        ofstream fout(out_file);
        fout << fixed << setprecision(6);
        fout << "# frame x y z alpha beta gamma n_cams\n";

        int frame_idx = 0;
        int valid_frames = 0;
        int total_tried = 0;

        // 探测最大帧数：扫描任一相机图像确定范围
        int max_frames = 999;
        {
            string test_dir = img_dir;
            int found_max = -1;
            for (auto& entry : fs::directory_iterator(img_dir)) {
                string stem = entry.path().stem().string();
                if (stem.rfind("calib_cam_", 0) == 0) {
                    size_t last_us = stem.find_last_of('_');
                    if (last_us != string::npos) {
                        try { found_max = max(found_max, stoi(stem.substr(last_us + 1))); }
                        catch (...) {}
                    }
                }
            }
            if (found_max >= 0) max_frames = found_max;
        }
        cout << "\nProcessing frames 00-" << setfill('0') << setw(2) << max_frames << "..." << endl;

        for (int fi = 0; fi <= max_frames; ++fi) {
            stringstream ss_idx;
            ss_idx << setw(2) << setfill('0') << fi;
            string idx_str = ss_idx.str();

            vector<HTuple> poses_in_ref;  // board poses in ref frame (from each valid camera)

            for (size_t ci = 0; ci < cam_data.size(); ++ci) {
                if (!cam_data[ci].valid) continue;

                string img_path = img_dir + "/calib_cam_" + cam_data[ci].sn + "_" + idx_str + ".jpg";
                if (!fs::exists(img_path)) continue;
                total_tried++;

                try {
                    HObject ho_img;
                    ReadImage(&ho_img, HTuple(img_path.c_str()));

                    // 彩色图转灰度
                    HTuple hv_ch;
                    CountChannels(ho_img, &hv_ch);
                    if (hv_ch.I() == 3) Rgb1ToGray(ho_img, &ho_img);

                    // 创建单相机标定模型, 提取标定板位姿
                    HTuple hv_calib_id;
                    CreateCalibData("calibration_object", 1, 1, &hv_calib_id);
                    SetCalibDataCalibObject(hv_calib_id, 0, hv_calib_plane);
                    SetCalibDataCamParam(hv_calib_id, 0, HTuple(), cam_data[ci].cam_param);

                    HObject ho_contours;
                    HTuple hv_err;
                    try {
                        FindCalibObject(ho_img, hv_calib_id, 0, 0, 0,
                                        (HTuple("alpha").Append("sigma")),
                                        (HTuple(0.5).Append(1.0)));
                        GetCalibDataObservPose(hv_calib_id, 0, 0, 0, &hv_err);
                    } catch (HException&) {
                        ClearCalibData(hv_calib_id);
                        continue;  // 该相机未检测到标定板
                    }

                    // board pose in this camera's local frame
                    HTuple board_in_cam_i;
                    GetCalibData(hv_calib_id, "calib_obj", 0, "pose", &board_in_cam_i);

                    // board pose in ref frame: T_cam_i_to_ref * board_in_cam_i
                    HTuple board_in_ref;
                    PoseCompose(cam_data[ci].pose_in_ref, board_in_cam_i, &board_in_ref);
                    poses_in_ref.push_back(board_in_ref);

                    ClearCalibData(hv_calib_id);
                } catch (HException&) {
                    continue;
                }
            }

            if (!poses_in_ref.empty()) {
                // --- 多相机平均融合 ---
                double avg_qx = 0, avg_qy = 0, avg_qz = 0, avg_qw = 0;
                double avg_x = 0, avg_y = 0, avg_z = 0;

                for (auto& p : poses_in_ref) {
                    HTuple hom;
                    PoseToHomMat3d(p, &hom);
                    double px = hom[3].D(), py = hom[7].D(), pz = hom[11].D();
                    avg_x += px; avg_y += py; avg_z += pz;

                    double qx, qy, qz, qw;
                    poseToQt(p, qx, qy, qz, qw);
                    avg_qx += qx; avg_qy += qy; avg_qz += qz; avg_qw += qw;
                }

                double n_cams = (double)poses_in_ref.size();
                avg_x /= n_cams; avg_y /= n_cams; avg_z /= n_cams;

                double nq = sqrt(avg_qx*avg_qx + avg_qy*avg_qy + avg_qz*avg_qz + avg_qw*avg_qw);
                avg_qx /= nq; avg_qy /= nq; avg_qz /= nq; avg_qw /= nq;

                // Quaternion → Z-X-Z'' Euler
                // Rotation matrix from average quaternion
                double r00 = 1 - 2*avg_qy*avg_qy - 2*avg_qz*avg_qz;
                double r01 = 2*avg_qx*avg_qy - 2*avg_qz*avg_qw;
                double r02 = 2*avg_qx*avg_qz + 2*avg_qy*avg_qw;
                double r12 = 2*avg_qy*avg_qz - 2*avg_qx*avg_qw;
                double r20 = 2*avg_qx*avg_qz - 2*avg_qy*avg_qw;
                double r21 = 2*avg_qy*avg_qz + 2*avg_qx*avg_qw;
                double r22 = 1 - 2*avg_qx*avg_qx - 2*avg_qy*avg_qy;

                double beta = acos(clamp(r22, -1.0, 1.0));
                double sb = sin(beta);
                double alpha, gamma;
                if (fabs(sb) > 1e-6) {
                    alpha = atan2(r02, -r12);
                    gamma = atan2(r20, r21);
                } else {
                    gamma = 0.0;
                    if (beta < M_PI / 2.0)
                        alpha = atan2(-r01, r00);
                    else
                        alpha = atan2(r01, r00);
                }
                alpha *= 180.0 / M_PI;
                beta  *= 180.0 / M_PI;
                gamma *= 180.0 / M_PI;

                // 写到输出文件
                fout << idx_str << " "
                     << avg_x << " " << avg_y << " " << avg_z << " "
                     << alpha << " " << beta << " " << gamma << " "
                     << (int)poses_in_ref.size() << endl;

                valid_frames++;
                cout << "[Frame " << idx_str << "] " << poses_in_ref.size()
                     << "/" << cam_data.size() << " cams, "
                     << "pos=(" << fixed << setprecision(3)
                     << avg_x << "," << avg_y << "," << avg_z << ")m, "
                     << "euler=(" << setprecision(1) << alpha << "," << beta << "," << gamma << ")deg"
                     << endl;
            } else {
                if (total_tried > 0) {
                    cout << "[Frame " << idx_str << "] No valid poses ("
                         << total_tried << " images tried)" << endl;
                }
            }

            frame_idx++;
        }

        fout.close();

        cout << "\n=== Done ===" << endl;
        cout << "Valid frames: " << valid_frames << " / " << (frame_idx + 1) << endl;
        cout << "Output: " << out_file << endl;

    } catch (HException& ex) {
        cerr << "Halcon Error #" << ex.ErrorCode() << " in " << ex.ProcName().TextA()
             << ": " << ex.ErrorMessage().TextA() << endl;
        ret = 1;
    }
    return ret;
}
#endif
