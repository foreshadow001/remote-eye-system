// ================== calib_cam_intrinsics ==================
// 相机内参重标定 (仅焦距改变的相机, 无需重标外参)
// 对 {participant_id}/pictures/{SN}/ 下有足够标定图片的相机:
//   1. 从 {day_id}/output/{SN}_Data.xml 加载原始内参 (标定起点) + 外参 (保持不变)
//   2. 单相机 HALCON 标定 → 更新内参
//   3. 写出 {participant_id}/output/{SN}_Data.xml
// 其余没有标定图片的相机: 原始 XML 直接复制, 保证 {participant_id}/output 完整
// day_id 由 cfg/day_participant_map.json 从 participant_id 映射 (不读取 yaml 中的 day_id)
// 仅在 master 上运行
// =================================================================

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <iterator>

#include <yaml-cpp/yaml.h>

#include "HalconCpp.h"
#include "cfg/config.hpp"

using namespace HalconCpp;
using namespace std;
namespace fs = std::filesystem;

// 内参标定最少图片数 (HALCON 建议 >= 10, 少于 5 直接跳过)
const int MIN_IMAGES = 5;

// ================== XML 解析 (calib_cam_chain 输出格式) ==================

// 读取 <InternalParameters><RawData> → 相机参数 HTuple
bool parseIntrinsicsXml(const string& filepath, HTuple& params) {
    ifstream in(filepath);
    if (!in) return false;
    string xml((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());
    string o = "<RawData>", c = "</RawData>";
    size_t s = xml.find(o);
    if (s == string::npos) return false;
    s += o.length();
    size_t e = xml.find(c, s);
    if (e == string::npos) return false;
    string raw = xml.substr(s, e - s);

    vector<string> toks;
    stringstream ss(raw);
    string t;
    while (ss >> t) toks.push_back(t);
    if (toks.size() < 9) return false;  // type + focus kappa sx sy cx cy w h
    params.Clear();
    params[0] = HTuple(toks[0].c_str());
    for (size_t i = 1; i < toks.size(); ++i)
        params = params.TupleConcat(stod(toks[i]));
    return true;
}

// 读取 <ExternalParameters> → 位姿 HTuple
bool loadExtrinsicFromXml(const string& filepath, HTuple& pose) {
    ifstream in(filepath);
    if (!in) return false;
    string xml((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());
    auto tag = [&](const string& t) {
        string o = "<" + t + ">", c = "</" + t + ">";
        size_t s = xml.find(o);
        if (s == string::npos) return string();
        s += o.length();
        size_t e = xml.find(c, s);
        if (e == string::npos) return string();
        return xml.substr(s, e - s);
    };
    double tx = stod(tag("X")), ty = stod(tag("Y")), tz = stod(tag("Z"));
    double a = stod(tag("Alpha")), b = stod(tag("Beta")), g = stod(tag("Gamma"));
    CreatePose(tx, ty, tz, a, b, g, "Rp+T", "gba", "point", &pose);
    return true;
}

// 写 CameraData XML (格式与 calib_cam_chain Stage 6 完全一致)
void writeCameraXml(const string& path, const HTuple& params, const HTuple& pose) {
    HTuple hv_StringPose, hv_FileHandle, hv_FullParamString;
    OpenFile(HTuple(path.c_str()), "output", &hv_FileHandle);

    FwriteString(hv_FileHandle, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    FwriteString(hv_FileHandle, "<CameraData>\n");

    FwriteString(hv_FileHandle, "  <InternalParameters>\n");
    HTuple type = ((const HTuple&)params)[0];
    HTuple vals = params.TupleSelectRange(1, params.TupleLength() - 1);
    HTuple valStr;
    TupleString(vals, ".12g", &valStr);
    hv_FullParamString = type + " " + ((valStr + " ").TupleSum());
    FwriteString(hv_FileHandle, ("    <RawData>" + hv_FullParamString) + "</RawData>\n");
    FwriteString(hv_FileHandle, "  </InternalParameters>\n");

    FwriteString(hv_FileHandle, "  <ExternalParameters>\n");
    TupleString(pose, ".12g", &hv_StringPose);
    FwriteString(hv_FileHandle, "    <Translation>\n");
    FwriteString(hv_FileHandle, ("      <X>" + HTuple(hv_StringPose[0])) + "</X>\n");
    FwriteString(hv_FileHandle, ("      <Y>" + HTuple(hv_StringPose[1])) + "</Y>\n");
    FwriteString(hv_FileHandle, ("      <Z>" + HTuple(hv_StringPose[2])) + "</Z>\n");
    FwriteString(hv_FileHandle, "    </Translation>\n");

    FwriteString(hv_FileHandle, "    <Rotation>\n");
    FwriteString(hv_FileHandle, ("      <Alpha>" + HTuple(hv_StringPose[3])) + "</Alpha>\n");
    FwriteString(hv_FileHandle, ("      <Beta>" + HTuple(hv_StringPose[4])) + "</Beta>\n");
    FwriteString(hv_FileHandle, ("      <Gamma>" + HTuple(hv_StringPose[5])) + "</Gamma>\n");
    FwriteString(hv_FileHandle, "    </Rotation>\n");

    FwriteString(hv_FileHandle, "    <Meta>\n");
    FwriteString(hv_FileHandle, ("      <PoseTypeCode>" + HTuple(hv_StringPose[6])) + "</PoseTypeCode>\n");
    FwriteString(hv_FileHandle, "      <OrderOfTransform>Rp+T</OrderOfTransform>\n");
    FwriteString(hv_FileHandle, "      <OrderOfRotation>gba</OrderOfRotation>\n");
    FwriteString(hv_FileHandle, "      <ViewOfTransform>point</ViewOfTransform>\n");
    FwriteString(hv_FileHandle, "    </Meta>\n");

    FwriteString(hv_FileHandle, ("    <RawPose>" + ((hv_StringPose + " ").TupleSum())) + "</RawPose>\n");
    FwriteString(hv_FileHandle, "  </ExternalParameters>\n");
    FwriteString(hv_FileHandle, "</CameraData>\n");

    CloseFile(hv_FileHandle);
}

// ================== 内参标定 ==================

// 单相机内参标定. 成功返回 true 并把新参数写入 out_params
bool calibIntrinsics(const string& img_dir, const HTuple& plane,
                     const HTuple& start_params, HTuple& out_params) {
    vector<string> imgs;
    for (auto& e : fs::directory_iterator(img_dir))
        if (e.path().extension() == ".jpg") imgs.push_back(e.path().string());
    sort(imgs.begin(), imgs.end());
    if ((int)imgs.size() < MIN_IMAGES) {
        cout << "  Too few images (" << imgs.size() << " < " << MIN_IMAGES << "), skipping." << endl;
        return false;
    }

    HTuple calib_id;
    CreateCalibData("calibration_object", 1, 1, &calib_id);
    SetCalibDataCalibObject(calib_id, 0, plane);
    SetCalibDataCamParam(calib_id, 0, HTuple(), start_params);  // 原始内参作为起点

    int obs = 0;
    for (auto& ip : imgs) {
        HObject img;
        ReadImage(&img, HTuple(ip.c_str()));
        HTuple ch;
        CountChannels(img, &ch);
        if (ch.I() == 3) Rgb1ToGray(img, &img);
        try {
            FindCalibObject(img, calib_id, 0, 0, obs,
                            (HTuple("alpha").Append("sigma")),
                            (HTuple(0.5).Append(1.0)));
            obs++;
        } catch (HException&) { continue; }
    }
    if (obs < MIN_IMAGES) {
        cout << "  Only " << obs << " marks found, skipping." << endl;
        ClearCalibData(calib_id);
        return false;
    }
    HTuple errors;
    CalibrateCameras(calib_id, &errors);
    cout << "  Calibrated: " << obs << " obs, RMS=" << errors.D() << "px" << endl;
    GetCalibData(calib_id, "camera", 0, "params", &out_params);
    ClearCalibData(calib_id);
    return true;
}

// participant_id → day_id (cfg/day_participant_map.json, JSON 是 YAML 子集)
string findDayForParticipant(const string& json_path, const string& participant) {
    try {
        YAML::Node root = YAML::LoadFile(json_path);
        for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
            string day = it->first.as<string>();
            for (size_t i = 0; i < it->second.size(); ++i)
                if (it->second[i].as<string>() == participant) return day;
        }
    } catch (const std::exception& e) {
        cerr << "[Error] Failed to parse " << json_path << ": " << e.what() << endl;
    }
    return "";
}

// ================== main ==================

int main(int argc, char* argv[]) {
    int ret = 0;
    try {
#if defined(_WIN32)
        SetSystem("use_window_thread", "true");
#endif

        auto cfg_root = fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path() / "cfg";
        Cfg cam_cfg((cfg_root / "cam_calib.yaml").string());
        auto& calib = cam_cfg["calib"];

        string base_dir = calib["calib_save_dir"].as<string>();
        string participant = calib["participant_id"].as<string>();
        HTuple plane = calib["calib_plane"].as<string>().c_str();

        // participant → day (不读取 yaml 中的 day_id)
        string day = findDayForParticipant((cfg_root / "day_participant_map.json").string(), participant);
        if (day.empty()) {
            cerr << "[Fatal] Participant " << participant << " not found in cfg/day_participant_map.json" << endl;
            return 1;
        }

        string pic_root = base_dir + "/" + participant + "/pictures";   // 输入: per-SN 标定图片
        string xml_in = base_dir + "/" + day + "/output";               // 原始内参 + 外参
        string xml_out = base_dir + "/" + participant + "/output";      // 更新后的 XML

        cout << "=== Camera Intrinsics Re-Calibration ===" << endl;
        cout << "Participant: " << participant << "  (day: " << day << ")" << endl;
        cout << "Images:      " << pic_root << endl;
        cout << "XML in:      " << xml_in << endl;
        cout << "XML out:     " << xml_out << endl;
        cout << "Calib plane: " << plane.S() << endl;
        cout << "=========================================\n" << endl;

        // 扫描有标定图片的相机
        vector<string> sns;
        if (fs::exists(pic_root)) {
            for (auto& e : fs::directory_iterator(pic_root)) {
                if (!e.is_directory()) continue;
                int n = 0;
                for (auto& f : fs::directory_iterator(e.path()))
                    if (f.path().extension() == ".jpg") n++;
                if (n > 0) sns.push_back(e.path().filename().string());
            }
        }
        sort(sns.begin(), sns.end());
        if (sns.empty()) {
            cerr << "[Fatal] No intrinsics images under " << pic_root << endl;
            return 1;
        }
        cout << "Cameras with intrinsics images: " << sns.size() << endl;

        fs::create_directories(xml_out);
        int ok = 0;
        set<string> recalibrated;
        for (auto& sn : sns) {
            cout << "\n--- Cam " << sn << " ---" << endl;
            string orig_xml = xml_in + "/" + sn + "_Data.xml";
            if (!fs::exists(orig_xml)) {
                cerr << "  Original XML not found: " << orig_xml
                     << " — run full extrinsics calibration first." << endl;
                continue;
            }
            HTuple start_params, pose;
            if (!parseIntrinsicsXml(orig_xml, start_params)) {
                cerr << "  Failed to parse intrinsics from " << orig_xml << endl;
                continue;
            }
            if (!loadExtrinsicFromXml(orig_xml, pose)) {
                cerr << "  Failed to parse extrinsics from " << orig_xml << endl;
                continue;
            }
            double f0 = start_params[1].D();
            HTuple new_params;
            if (!calibIntrinsics(pic_root + "/" + sn, plane, start_params, new_params)) continue;
            double f1 = new_params[1].D();
            string out_xml = xml_out + "/" + sn + "_Data.xml";
            writeCameraXml(out_xml, new_params, pose);  // 外参保持不变
            cout << "  Focus: " << f0 * 1000.0 << "mm -> " << f1 * 1000.0 << "mm" << endl;
            cout << "  Written: " << out_xml << endl;
            recalibrated.insert(sn);
            ok++;
        }

        // 未进行内参标定的相机: 直接复制原始 XML (内参+外参均不变), 保证 {participant_id}/output 完整
        int copied = 0;
        if (fs::exists(xml_in)) {
            for (auto& e : fs::directory_iterator(xml_in)) {
                if (e.path().extension() != ".xml") continue;
                string stem = e.path().stem().string();          // {SN}_Data
                if (stem.length() <= 5 || stem.substr(stem.length() - 5) != "_Data") continue;
                string sn = stem.substr(0, stem.length() - 5);
                if (recalibrated.count(sn)) continue;            // 已重标定 → 跳过
                string dst = xml_out + "/" + stem + ".xml";
                fs::copy_file(e.path(), dst, fs::copy_options::overwrite_existing);
                cout << "[Copy] " << sn << " -> " << dst << endl;
                copied++;
            }
        }

        cout << "\n=== Done: " << ok << " re-calibrated, " << copied << " copied ===" << endl;

    } catch (HException& ex) {
        cerr << "Halcon Error #" << ex.ErrorCode() << " in " << ex.ProcName().TextA()
             << ": " << ex.ErrorMessage().TextA() << endl;
        ret = 1;
    } catch (const std::exception& e) {
        cerr << "[Fatal] " << e.what() << endl;
        ret = 1;
    }
    return ret;
}
