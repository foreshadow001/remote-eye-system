#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <typeinfo> // for typeid
#include <iomanip>     // for std::setprecision
#include <type_traits> // for std::is_floating_point

#include "core/math_types.hpp"
#include "logger/logger.hpp"

using namespace gazeestimation;

// 告诉 yaml-cpp 如何解析自定义类型 Vec2 和 Vec3
namespace YAML {
    template<>
    struct convert<Vec3> {
        static bool decode(const Node& node, Vec3& rhs) {
            if (!node.IsSequence() || node.size() != 3) {
                return false; // 返回 false 会让 yaml-cpp 抛出 BadConversion 异常
            }
            rhs = make_vec3(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
            return true;
        }
    };

    template<>
    struct convert<Vec2> {
        static bool decode(const Node& node, Vec2& rhs) {
            if (!node.IsSequence() || node.size() != 2) {
                return false;
            }
            rhs = make_vec2(node[0].as<double>(), node[1].as<double>());
            return true;
        }
    };
}

class CfgNode {
public:
    CfgNode();
    // 专门用于传递已失败状态的内部构造函数
    CfgNode(bool valid, const std::string& path);
    // 常规构造函数
    CfgNode(const YAML::Node& node, const std::string& path = "");

    // 访问子节点
    CfgNode operator[](const std::string& key) const;
    CfgNode operator[](size_t idx) const;

    // 迭代支持
    YAML::Node::const_iterator begin() const;
    YAML::Node::const_iterator end() const;

    template<typename T>
    T as() const {
        if (!is_valid_) {
            throw std::runtime_error(
                "[Cfg ERROR] Cannot read value at path: '" + path_
                + "'. Key does not exist in config file.");
        }
        try {
            return node_.as<T>();
        } catch (const YAML::Exception& e) {
            throw std::runtime_error(
                "[Cfg ERROR] Type mismatch at path: '" + path_
                + "'. Expected: " + typeid(T).name()
                + ". Details: " + std::string(e.what()));
        }
    }

    bool isDefined() const { return is_valid_; }

private:
    YAML::Node node_;
    std::string path_;
    bool is_valid_; // 核心标志：记录当前节点是否确实有效
};

// 帮助函数：将任意类型按指定精度转为字符串
template <typename T>
inline std::string formatValue(T val, int precision) {
    std::ostringstream oss;
    // 如果是浮点数且指定了精度，才设置 fixed 和 precision
    if (std::is_floating_point<T>::value && precision >= 0) {
        oss << std::fixed << std::setprecision(precision);
    }
    oss << val;
    return oss.str();
}

class Cfg {
public:
    explicit Cfg(const std::string& filepath = "");

    CfgNode operator[](const std::string& key) const;

    std::string path() const { return filepath_; }

    // 模板化：支持传入任意类型及精度设置 (precision = -1 表示使用默认格式)
    template<typename T>
    void setScalar(const std::string& path, T v, int precision = -1) {
        internalSetScalar(path, formatValue(v, precision));
    }

    // 新增：支持一维数组写入 (写入单行，如 [1, 2, 3])
    template<typename T>
    void setVector(const std::string& path, const std::vector<T>& v, int precision = -1) {
        std::string str = "[";
        for (size_t i = 0; i < v.size(); i++) {
            str += formatValue(v[i], precision);
            if (i + 1 < v.size()) str += ", ";
        }
        str += "]";
        internalSetScalar(path, str);
    }

    // 模板化：支持二维数组写入 (写入多行 block)
    template<typename T>
    void setVector2D(const std::string& path, const std::vector<std::vector<T>>& v, int precision = -1) {
        std::vector<std::string> lines;
        for (const auto& row : v) {
            std::string line = "- [";
            for (size_t i = 0; i < row.size(); i++) {
                line += formatValue(row[i], precision);
                if (i + 1 < row.size()) line += ", ";
            }
            line += "]";
            lines.push_back(line);
        }
        internalSetBlock(path, lines);
    }

    void save() const;

private:
    YAML::Node root_;
    std::string filepath_;
    std::string original_text_;

    static bool fileExists(const std::string& p);
    std::string getConfigPath() const;

    // 核心字符串替换逻辑
    void internalSetScalar(const std::string& path, const std::string& val_str);
    void internalSetBlock(const std::string& path, const std::vector<std::string>& block_lines);
};