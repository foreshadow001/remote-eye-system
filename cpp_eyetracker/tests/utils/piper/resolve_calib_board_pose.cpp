// ================== resolve_calib_board_pose ==================
// 流程 (对 upper / lower 分别执行):
//   1. 扫描 arm 子目录 → 获取相机 SN 和帧数
//   2. 加载 XML 外参 (calib_cam_chain 输出) → 重基准到中心相机
//   3. 每台相机独立单相机标定 → 精准内参 + 标定板位姿
//   4. 变换到中心相机坐标系 → 平移 + Z-X-Z'' 欧拉角输出
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

// ================== 辅助 ==================

double clamp(double v, double lo, double hi) { return max(lo, min(hi, v)); }

void genCamPar(double focus, double sx, double sy, double w, double h, HTuple* p) {
    p->Clear();
    (*p)[0] = "area_scan_division";
    *p = p->TupleConcat(focus).TupleConcat(0.0).TupleConcat(sx).TupleConcat(sy)
               .TupleConcat(w/2.0).TupleConcat(h/2.0).TupleConcat(w).TupleConcat(h);
}

void poseToQt(const HTuple& pose, double& qx, double& qy, double& qz, double& qw) {
    HTuple hom; PoseToHomMat3d(pose, &hom);
    double r00=hom[0].D(), r01=hom[1].D(), r02=hom[2].D();
    double r10=hom[4].D(), r11=hom[5].D(), r12=hom[6].D();
    double r20=hom[8].D(), r21=hom[9].D(), r22=hom[10].D();
    double tr=r00+r11+r22;
    if(tr>0){double s=sqrt(tr+1.0)*2.0;qw=0.25*s;qx=(r21-r12)/s;qy=(r02-r20)/s;qz=(r10-r01)/s;}
    else if(r00>r11&&r00>r22){double s=sqrt(1.0+r00-r11-r22)*2.0;qw=(r21-r12)/s;qx=0.25*s;qy=(r01+r10)/s;qz=(r02+r20)/s;}
    else if(r11>r22){double s=sqrt(1.0+r11-r00-r22)*2.0;qw=(r02-r20)/s;qx=(r01+r10)/s;qy=0.25*s;qz=(r12+r21)/s;}
    else{double s=sqrt(1.0+r22-r00-r11)*2.0;qw=(r10-r01)/s;qx=(r02+r20)/s;qy=(r12+r21)/s;qz=0.25*s;}
}

bool loadExtrinsicFromXml(const string& filepath, HTuple& pose) {
    ifstream in(filepath);
    if (!in) return false;
    string xml((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());
    auto tag = [&](const string& t) {
        string o="<"+t+">", c="</"+t+">";
        size_t s=xml.find(o); if(s==string::npos) return string();
        s+=o.length(); size_t e=xml.find(c,s); if(e==string::npos) return string();
        return xml.substr(s,e-s);
    };
    double tx=stod(tag("X")), ty=stod(tag("Y")), tz=stod(tag("Z"));
    double a=stod(tag("Alpha")), b=stod(tag("Beta")), g=stod(tag("Gamma"));
    CreatePose(tx, ty, tz, a, b, g, "Rp+T", "gba", "point", &pose);
    return true;
}

// 读取 flange_pose_mapping_<arm>.txt 获取该 arm 的帧索引集合
set<int> readMappingFrames(const string& path) {
    set<int> frames;
    if (!fs::exists(path)) return frames;
    ifstream in(path);
    string line;
    while (getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line);
        string idx_str; ss >> idx_str;
        try { frames.insert(stoi(idx_str)); } catch (...) {}
    }
    return frames;
}

// ================== 单 arm 处理 ==================

int processArm(const string& arm, const string& img_dir, const string& out_dir,
               const string& xml_dir, const string& center_sn, const HTuple& hv_cp,
               double focus, double px_sx, double px_sy,
               const set<int>& arm_frames, bool only_center) {
    if (arm_frames.empty()) {
        cout << "[Arm " << arm << "] No frames in mapping file, skipping." << endl;
        return 0;
    }

    // --- 扫描图片, 确定相机 SN (只统计 arm_frames 中的帧) ---
    map<string, int> sn_max;
    for (auto& e : fs::directory_iterator(img_dir)) {
        string stem = e.path().stem().string();
        if (stem.rfind("calib_cam_", 0) != 0) continue;
        size_t p = stem.find_last_of('_');
        if (p == string::npos || p <= 10) continue;
        int idx = stoi(stem.substr(p + 1));
        if (!arm_frames.count(idx)) continue;  // 只统计属于当前 arm 的帧
        string sn = stem.substr(10, p - 10);
        auto it = sn_max.find(sn);
        if (it == sn_max.end() || idx > it->second) sn_max[sn] = idx;
    }
    if (sn_max.empty()) {
        cout << "[Arm " << arm << "] No images found for this arm's frames." << endl;
        return 0;
    }

    vector<string> sns;
    int max_frames = *arm_frames.rbegin();  // arm_frames 非空时已验证
    for (auto& kv : sn_max) sns.push_back(kv.first);
    sort(sns.begin(), sns.end());

    // 仅用中心相机
    if (only_center) {
        sns.erase(remove_if(sns.begin(), sns.end(),
                            [&](const string& s) { return s != center_sn; }),
                  sns.end());
        if (sns.empty()) {
            cout << "[Arm " << arm << "] Center cam " << center_sn
                 << " not found, skipping." << endl;
            return 0;
        }
    }

    int n_cams = (int)sns.size();

    cout << "\n===== Arm: " << arm
         << (only_center ? " (center cam only)" : "") << " =====" << endl;
    cout << "Cameras: " << n_cams << ", frames: " << (max_frames + 1) << endl;
    for (int i = 0; i < n_cams; ++i) cout << "  " << i << ": SN=" << sns[i] << endl;

    // --- 加载 XML 外参, 重基准到中心相机 ---
    vector<HTuple> ext_pose(n_cams);
    int center_idx = -1;
    cout << "Extrinsics:" << endl;
    for (int i = 0; i < n_cams; ++i) {
        if (sns[i] == center_sn) center_idx = i;
        string xp = xml_dir + "/" + sns[i] + "_Data.xml";
        if (fs::exists(xp) && loadExtrinsicFromXml(xp, ext_pose[i])) {
            cout << "  " << sns[i] << " → OK" << endl;
        } else {
            cout << "  " << sns[i] << " → missing, identity" << endl;
            CreatePose(0,0,0,0,0,0,"Rp+T","gba","point",&ext_pose[i]);
        }
    }

    HTuple T_cam0_to_ref;
    if (center_idx >= 0) {
        HTuple T_ref_to_cam0 = ext_pose[center_idx];
        PoseInvert(T_ref_to_cam0, &T_cam0_to_ref);
        cout << "Center cam: " << center_sn << " (idx " << center_idx << ")" << endl;
    } else {
        CreatePose(0,0,0,0,0,0,"Rp+T","gba","point",&T_cam0_to_ref);
        cout << "Center cam not in list, using cam0 as reference." << endl;
    }

    vector<HTuple> T_cam_to_ref(n_cams);
    for (int i = 0; i < n_cams; ++i)
        PoseCompose(T_cam0_to_ref, ext_pose[i], &T_cam_to_ref[i]);

    // --- 每台相机独立标定 + 提取位姿 ---
    vector<vector<HTuple>> frame_poses(max_frames + 1);

    for (int ci = 0; ci < n_cams; ++ci) {
        cout << "\n--- Cam " << ci << " (" << sns[ci] << ") calibrating ---" << endl;

        // 检查是否有专用标定图片: <img_dir>/<arm>/<SN>/calib_*.jpg
        string dc_dir = img_dir + "/" + arm + "/" + sns[ci];
        bool has_dedicated = fs::exists(dc_dir);
        if (has_dedicated) {
            // 统计专用图片数量
            int dc_count = 0;
            for (auto& e : fs::directory_iterator(dc_dir))
                if (e.path().extension() == ".jpg") dc_count++;
            if (dc_count == 0) has_dedicated = false;
        }

        HTuple cam_params;  // calibrated intrinsics

        // --- 内参标定 + 位姿提取 ---
        if (has_dedicated) {
            cout << "  Using dedicated calib images from " << dc_dir << endl;
            // Phase 1: 用专用图片标定内参
            HTuple hv_w, hv_h;
            HObject ho_tmp;
            string first_img;
            for (auto& e : fs::directory_iterator(dc_dir))
                if (e.path().extension() == ".jpg") { first_img = e.path().string(); break; }
            ReadImage(&ho_tmp, HTuple(first_img.c_str()));
            GetImageSize(ho_tmp, &hv_w, &hv_h);

            HTuple dc_calib;
            CreateCalibData("calibration_object", 1, 1, &dc_calib);
            SetCalibDataCalibObject(dc_calib, 0, hv_cp);
            HTuple dc_param;
            genCamPar(focus, px_sx, px_sy, hv_w.D(), hv_h.D(), &dc_param);
            SetCalibDataCamParam(dc_calib, 0, HTuple(), dc_param);

            int dc_obs = 0;
            for (auto& e : fs::directory_iterator(dc_dir)) {
                if (e.path().extension() != ".jpg") continue;
                HObject ho_img;
                ReadImage(&ho_img, HTuple(e.path().string().c_str()));
                HTuple ch; CountChannels(ho_img, &ch);
                if (ch.I() == 3) Rgb1ToGray(ho_img, &ho_img);
                try {
                    FindCalibObject(ho_img, dc_calib, 0, 0, dc_obs,
                                    (HTuple("alpha").Append("sigma")),
                                    (HTuple(0.5).Append(1.0)));
                    dc_obs++;
                } catch (HException&) { continue; }
            }
            if (dc_obs < 3) {
                cout << "  Too few dedicated marks, skipping." << endl;
                ClearCalibData(dc_calib);
                continue;
            }
            HTuple dc_err;
            CalibrateCameras(dc_calib, &dc_err);
            cout << "  Dedicated calib: " << dc_obs << " obs, error=" << dc_err.D() << "px" << endl;
            GetCalibData(dc_calib, "camera", 0, "params", &cam_params);
            ClearCalibData(dc_calib);

            // Phase 2: 用精准内参作为起点, 帧图片 + CalibData 批量提取位姿 (与 fallback 路径一致)
            HTuple frm_calib;
            CreateCalibData("calibration_object", 1, 1, &frm_calib);
            SetCalibDataCalibObject(frm_calib, 0, hv_cp);
            SetCalibDataCamParam(frm_calib, 0, HTuple(), cam_params);  // 精准内参作为起点

            map<int, int> f2o;
            int obs_idx = 0;
            for (int fi = 0; fi <= max_frames; ++fi) {
                if (!arm_frames.count(fi)) continue;
                stringstream ssf; ssf << setw(2) << setfill('0') << fi;
                string ip = img_dir + "/calib_cam_" + sns[ci] + "_" + ssf.str() + ".jpg";
                if (!fs::exists(ip)) continue;
                HObject ho_img;
                ReadImage(&ho_img, HTuple(ip.c_str()));
                HTuple ch; CountChannels(ho_img, &ch);
                if (ch.I() == 3) Rgb1ToGray(ho_img, &ho_img);
                try {
                    FindCalibObject(ho_img, frm_calib, 0, 0, obs_idx,
                                    (HTuple("alpha").Append("sigma")),
                                    (HTuple(0.5).Append(1.0)));
                    f2o[fi] = obs_idx;
                    obs_idx++;
                } catch (HException&) { continue; }
            }
            if (obs_idx == 0) {
                cout << "  No frame marks found, skipping." << endl;
                ClearCalibData(frm_calib);
                continue;
            }
            HTuple frm_err;
            CalibrateCameras(frm_calib, &frm_err);
            cout << "  Frame calib: " << obs_idx << " obs, error=" << frm_err.D() << "px" << endl;

            for (auto& kv : f2o) {
                int fi = kv.first, oi = kv.second;
                HTuple board_in_cam;
                GetCalibDataObservPose(frm_calib, 0, 0, oi, &board_in_cam);
                HTuple board_in_ref;
                PoseCompose(T_cam_to_ref[ci], board_in_cam, &board_in_ref);
                frame_poses[fi].push_back(board_in_ref);
            }
            ClearCalibData(frm_calib);

        } else {
            // --- 原始路径: 用 arm 帧图像同时标定和提取 ---
            HTuple hv_w, hv_h;
            HObject ho_tmp;
            string first_img;
            for (int fi = 0; fi <= max_frames; ++fi) {
                stringstream ssf; ssf << setw(2) << setfill('0') << fi;
                if (!arm_frames.count(fi)) continue;
                first_img = img_dir + "/calib_cam_" + sns[ci] + "_" + ssf.str() + ".jpg";
                if (fs::exists(first_img)) break;
            }
            if (first_img.empty()) {
                cout << "  No frame images for this camera, skipping." << endl;
                continue;
            }
            ReadImage(&ho_tmp, HTuple(first_img.c_str()));
            GetImageSize(ho_tmp, &hv_w, &hv_h);

            HTuple calib_id, start_param;
            CreateCalibData("calibration_object", 1, 1, &calib_id);
            SetCalibDataCalibObject(calib_id, 0, hv_cp);
            genCamPar(focus, px_sx, px_sy, hv_w.D(), hv_h.D(), &start_param);
            SetCalibDataCamParam(calib_id, 0, HTuple(), start_param);

            map<int, int> f2o;
            int obs_idx = 0;
            for (int fi = 0; fi <= max_frames; ++fi) {
                if (!arm_frames.count(fi)) continue;
                stringstream ssf; ssf << setw(2) << setfill('0') << fi;
                string ip = img_dir + "/calib_cam_" + sns[ci] + "_" + ssf.str() + ".jpg";
                if (!fs::exists(ip)) continue;
                HObject ho_img;
                ReadImage(&ho_img, HTuple(ip.c_str()));
                HTuple ch; CountChannels(ho_img, &ch);
                if (ch.I() == 3) Rgb1ToGray(ho_img, &ho_img);
                try {
                    FindCalibObject(ho_img, calib_id, 0, 0, obs_idx,
                                    (HTuple("alpha").Append("sigma")),
                                    (HTuple(0.5).Append(1.0)));
                    f2o[fi] = obs_idx;
                    obs_idx++;
                } catch (HException&) { continue; }
            }

            if (obs_idx == 0) {
                cout << "  No marks found, skipping." << endl;
                ClearCalibData(calib_id);
                continue;
            }

            HTuple errors;
            try {
                CalibrateCameras(calib_id, &errors);
                cout << "  Calibrated: " << obs_idx << " obs, error=" << errors.D() << "px" << endl;
            } catch (HException& ex) {
                cerr << "  Calib failed: " << ex.ErrorMessage().TextA() << endl;
                ClearCalibData(calib_id);
                continue;
            }

            for (auto& kv : f2o) {
                int fi = kv.first, oi = kv.second;
                HTuple board_in_cam;
                GetCalibDataObservPose(calib_id, 0, 0, oi, &board_in_cam);
                HTuple board_in_ref;
                PoseCompose(T_cam_to_ref[ci], board_in_cam, &board_in_ref);
                frame_poses[fi].push_back(board_in_ref);
            }
            ClearCalibData(calib_id);
        }
    }

    // --- 输出 ---
    string out_path = out_dir + "/board_poses_" + arm + ".txt";
    ofstream fout(out_path);
    fout << fixed << setprecision(6);
    fout << "# frame x y z qw qx qy qz n_cams\n";

    int valid = 0;
    for (int fi = 0; fi <= max_frames; ++fi) {
        auto& poses = frame_poses[fi];
        if (poses.empty()) continue;
        valid++;

        double ax=0, ay=0, az=0, aqx=0, aqy=0, aqz=0, aqw=0;
        int np = (int)poses.size();
        for (auto& p : poses) {
            HTuple hom; PoseToHomMat3d(p, &hom);
            ax+=hom[3].D(); ay+=hom[7].D(); az+=hom[11].D();
            double qx,qy,qz,qw; poseToQt(p,qx,qy,qz,qw);
            aqx+=qx; aqy+=qy; aqz+=qz; aqw+=qw;
        }
        ax/=np; ay/=np; az/=np;
        double nq=sqrt(aqx*aqx+aqy*aqy+aqz*aqz+aqw*aqw);
        aqx/=nq; aqy/=nq; aqz/=nq; aqw/=nq;

        stringstream ssf; ssf << setw(2) << setfill('0') << fi;
        fout << ssf.str() << " " << ax << " " << ay << " " << az << " "
             << aqw << " " << aqx << " " << aqy << " " << aqz << " " << np << endl;

        cout << "[Arm " << arm << " frame " << ssf.str() << "] " << np << "/" << n_cams
             << " cams  pos=(" << setprecision(3) << ax << "," << ay << "," << az
             << ")m  quat(wxyz)=(" << fixed << setprecision(4)
             << aqw << "," << aqx << "," << aqy << "," << aqz << ")" << endl;
    }
    fout.close();
    cout << "[Arm " << arm << "] " << valid << " valid frames → " << out_path << endl;
    return valid;
}

// ================== main ==================

#ifndef NO_EXPORT_APP_MAIN
int main(int argc, char* argv[]) {
    int ret = 0;
    try {
#if defined(_WIN32)
        SetSystem("use_window_thread", "true");
#endif

        auto cfg_root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
                        / "cfg";
        Cfg arm_cfg((cfg_root / "calib_arm.yaml").string());
        Cfg cam_cfg((cfg_root / "cam_calib.yaml").string());
        auto& rcfg = arm_cfg["resolve"];

        // 输入目录 = calib_arm.yaml: record.calib_save_dir / record.day_id (无需单独配置)
        string img_base   = arm_cfg["record"]["calib_save_dir"].as<string>()
                          + "/" + arm_cfg["record"]["day_id"].as<string>();
        string out_dir    = img_base;   // 输出目录与输入目录一致
        // 相机标定 XML = cam_calib.yaml: calib.calib_save_dir / calib_arm.yaml: record.day_id / output
        string xml_dir    = cam_cfg["calib"]["calib_save_dir"].as<string>()
                          + "/" + arm_cfg["record"]["day_id"].as<string>() + "/output";
        HTuple hv_cp      = cam_cfg["calib"]["calib_plane"].as<string>().c_str();
        double focus      = rcfg["focus"].as<double>();
        double px_sx      = rcfg["pixel_size_x"].as<double>();
        double px_sy      = rcfg["pixel_size_y"].as<double>();
        string center_sn  = rcfg["center_cam"].as<string>();

        cout << "=== Resolve Calibration Board Pose ===" << endl;
        cout << "Image base:  " << img_base << endl;
        cout << "XML dir:     " << xml_dir << endl;
        cout << "Output dir:  " << out_dir << endl;
        cout << "Calib plane: " << hv_cp.S() << endl;
        cout << "Center cam:  " << center_sn << endl;
        cout << "Focus:       " << focus * 1000 << "mm" << endl;
        cout << "======================================\n" << endl;

        fs::create_directories(out_dir);

        // 从 mapping txt 读取各 arm 的帧索引
        auto upper_frames = readMappingFrames(img_base + "/flange_pose_mapping_upper.txt");
        auto lower_frames = readMappingFrames(img_base + "/flange_pose_mapping_lower.txt");
        cout << "Upper frames: " << upper_frames.size() << ", lower frames: "
             << lower_frames.size() << endl;

        int total = 0;
        bool only_center = false;
        try { only_center = rcfg["only_use_center_cam"].as<bool>(); } catch (...) {}

        total += processArm("upper", img_base, out_dir, xml_dir, center_sn,
                            hv_cp, focus, px_sx, px_sy, upper_frames, only_center);
        total += processArm("lower", img_base, out_dir, xml_dir, center_sn,
                            hv_cp, focus, px_sx, px_sy, lower_frames, only_center);

        cout << "\n=== Done: " << total << " total valid frames ===" << endl;

    } catch (HException& ex) {
        cerr << "Halcon Error #" << ex.ErrorCode() << ": " << ex.ErrorMessage().TextA() << endl;
        ret = 1;
    } catch (const std::exception& e) {
        cerr << "[Fatal] " << e.what() << endl;
        ret = 1;
    }
    return ret;
}
#endif
