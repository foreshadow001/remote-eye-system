#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <pugixml.hpp> // 引入 pugixml 头文件

namespace gazeestimation
{

struct CameraParams {
    double cx;              // 主点 X
    double cy;              // 主点 Y
    double sy;              // 像元高度
    double sx;              // 像元宽度
    double focus;           // 焦距
    std::vector<double> T;  // 平移向量 [tx, ty, tz]
    std::vector<double> R;  // 旋转向量 [rx, ry, rz] (角度制)
};

inline CameraParams
LoadCameraParams(const std::string& xmlPath) {
    CameraParams params;
    params.T.resize(3, 0.0);
    params.R.resize(3, 0.0);

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(xmlPath.c_str());

    if (!result) {
        std::cerr << "[Error] XML load failed: " << result.description() << " [" << xmlPath << "]" << std::endl;
        return params; 
    }

    pugi::xml_node root = doc.child("CameraData");

    // 1. 解析内参
    pugi::xml_node internalNode = root.child("InternalParameters");
    std::string rawDataStr = internalNode.child("RawData").text().as_string();

    if (!rawDataStr.empty()) {
        std::stringstream ss(rawDataStr);
        std::string type;
        double kappa; 
        // HALCON 'area_scan_division' 顺序: Type, Focus, Kappa, Sx, Sy, Cx, Cy, ...
        ss >> type >> params.focus >> kappa >> params.sx >> params.sy >> params.cx >> params.cy;
    }

    // 2. 解析外参
    pugi::xml_node externalNode = root.child("ExternalParameters");
    
    pugi::xml_node transNode = externalNode.child("Translation");
    params.T[0] = transNode.child("X").text().as_double();
    params.T[1] = transNode.child("Y").text().as_double();
    params.T[2] = transNode.child("Z").text().as_double();

    pugi::xml_node rotNode = externalNode.child("Rotation");
    params.R[0] = rotNode.child("Alpha").text().as_double();
    params.R[1] = rotNode.child("Beta").text().as_double();
    params.R[2] = rotNode.child("Gamma").text().as_double();

    return params;
}

} // namespace gazeestimation