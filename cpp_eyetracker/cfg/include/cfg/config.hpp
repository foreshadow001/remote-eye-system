#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include "core/math_types.hpp"   // make_vec3, Vec3, Vec2

using namespace gazeestimation;

class CfgNode {
public:
    CfgNode();
    CfgNode(const YAML::Node& node, const std::string& path = "");

    // 访问子节点
    CfgNode operator[](const std::string& key) const;
    CfgNode operator[](size_t idx) const;

    // 迭代支持
    YAML::Node::const_iterator begin() const;
    YAML::Node::const_iterator end() const;

    // 模板接口（必须在头文件）
    template<typename T>
    T as() const {
        if (!node_.IsDefined()) {
            printWarning();
            return T{};
        }
        return node_.as<T>();
    }

    template<typename T>
    std::vector<T> asVector() const {
        if (!node_.IsDefined() || !node_.IsSequence()) {
            printWarning();
            return {};
        }

        std::vector<T> out;
        out.reserve(node_.size());

        for (const auto& it : node_) {
            CfgNode child(it, path_);
            out.push_back(child.as<T>());
        }
        return out;
    }

    bool isDefined() const { return node_.IsDefined(); }

private:
    YAML::Node node_;
    std::string path_;

    void printWarning() const;
};


class Cfg {
public:
    explicit Cfg(const std::string& filepath = "");

    CfgNode operator[](const std::string& key) const;

    std::string path() const { return filepath_; }

private:
    YAML::Node root_;
    std::string filepath_;

    static bool fileExists(const std::string& p);

    static constexpr const char* default_cfg_path_ =
        "D:/users/projects/new_dataset/data_collection/PCCR/cpp-remote-eye/"
        "cpp_eyetracker/cfg/default.yaml";
};

template<>
inline Vec3 CfgNode::as<Vec3>() const {
    if (!node_.IsSequence() || node_.size() != 3) {
        std::cerr << "[Cfg ERROR] Expect Vec3 at " << path_ << std::endl;
        return make_vec3(0, 0, 0);
    }

    return make_vec3(
        node_[0].as<double>(),
        node_[1].as<double>(),
        node_[2].as<double>()
    );
}

template<>
inline std::vector<Vec3> CfgNode::as<std::vector<Vec3>>() const {
    if (!node_.IsSequence()) {
        std::cerr << "[Cfg ERROR] Expect [Vec3...] at " << path_ << std::endl;
        return {};
    }

    std::vector<Vec3> out;
    out.reserve(node_.size());

    for (const auto& item : node_) {
        CfgNode tmp(item, path_);
        out.push_back(tmp.as<Vec3>());
    }
    return out;
}

template<>
inline Vec2 CfgNode::as<Vec2>() const {
    if (!node_.IsSequence() || node_.size() != 2) {
        std::cerr << "[Cfg ERROR] Expect Vec2 at " << path_ << std::endl;
        return make_vec2(0, 0);
    }

    return make_vec2(
        node_[0].as<double>(),
        node_[1].as<double>()
    );
}

template<>
inline std::vector<Vec2> CfgNode::as<std::vector<Vec2>>() const {
    if (!node_.IsSequence()) {
        std::cerr << "[Cfg ERROR] Expect [Vec2...] at " << path_ << std::endl;
        return {};
    }

    std::vector<Vec2> out;
    out.reserve(node_.size());

    for (const auto& item : node_) {
        CfgNode tmp(item, path_);
        out.push_back(tmp.as<Vec2>());
    }
    return out;
}