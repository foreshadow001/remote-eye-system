#include "cfg/config.hpp"

// 彩色警告输出
static inline void yellow(const std::string& msg) {
    std::cerr << "\033[33m" << msg << "\033[0m" << std::endl;
}

// 彩色错误输出
static inline void red(const std::string& msg) {
    std::cerr << "\033[31m" << msg << "\033[0m" << std::endl;
}

// ===================== CfgNode ======================

CfgNode::CfgNode() {}

CfgNode::CfgNode(const YAML::Node& node, const std::string& path)
    : node_(node), path_(path) {}

CfgNode CfgNode::operator[](const std::string& key) const {
    std::string new_path = path_.empty() ? key : path_ + "." + key;
    return CfgNode(node_[key], new_path);
}

CfgNode CfgNode::operator[](size_t idx) const {
    return CfgNode(node_[idx], path_ + "[" + std::to_string(idx) + "]");
}

YAML::Node::const_iterator CfgNode::begin() const { return node_.begin(); }
YAML::Node::const_iterator CfgNode::end() const { return node_.end(); }

void CfgNode::printWarning() const {
    yellow("[Cfg WARNING] Missing key: " + path_);
}

// ========================= Cfg =========================

bool Cfg::fileExists(const std::string& p) {
    std::ifstream f(p);
    return f.good();
}

Cfg::Cfg(const std::string& filepath) {
    filepath_ = filepath.empty() ? getConfigPath() : filepath;

    if (!fileExists(filepath_)) {
        red("[Cfg ERROR] Cannot find config file: " + filepath_);
        return;
    }

    root_ = YAML::LoadFile(filepath_);
    std::cout << "[Cfg] Loaded config: " << filepath_ << std::endl;
}

CfgNode Cfg::operator[](const std::string& key) const {
    return CfgNode(root_[key], key);
}

std::string Cfg::getConfigPath() const {
    // 获取当前源文件目录
    std::string current_file_path = __FILE__;

    // 将当前路径和相对路径拼接
    return (std::filesystem::path(current_file_path).parent_path().parent_path().parent_path() / "cfg" / "default.yaml").string();
}