#include "cfg/config.hpp"
#include <sstream> // 必须包含，否则 split 和 istringstream 会报错
#include <algorithm> // for trim

// 简单的 trim 函数，去除首尾空白（包括 \r）
static std::string trim(const std::string& str) {
    const std::string whitespace = " \t\r\n";
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos) return ""; // 全是空格
    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;
    return str.substr(strBegin, strRange);
}

static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> res;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, '.')) res.push_back(item);
    return res;
}


static int findLineByFullPath(
    const std::vector<std::string>& lines,
    const std::vector<std::string>& keys
) {
    int level = 0;
    const int total = keys.size();

    for (int i = 0; i < (int)lines.size(); ++i) {
        const std::string& line = lines[i];
        if (trim(line).empty()) continue;

        // 计算缩进（空格数量）
        int indent = 0;
        while (indent < (int)line.size() && line[indent] == ' ') indent++;

        // 本层应有缩进
        int expected_indent = level * 2;

        // 缩进不符，肯定不是该层 key
        if (indent != expected_indent) continue;

        // 去掉缩进后的内容
        std::string content = trim(line);

        // 期待格式： "<key>:"
        std::string target = keys[level] + ":";

        if (content.rfind(target, 0) == 0) {  // 前缀匹配
            level++;
            if (level == total) {
                return i;  // 找到最终 key
            }
        }
    }

    return -1;
}


// --- CfgNode 实现 ---

CfgNode::CfgNode() : is_valid_(false) {}

CfgNode::CfgNode(bool valid, const std::string& path) 
    : is_valid_(valid), path_(path) {}

CfgNode::CfgNode(const YAML::Node& node, const std::string& path) 
    : node_(node), path_(path), is_valid_(true) 
{
    // 【核心拦截 1】：构造时立刻检查节点是否存在或为空！
    if (!node_.IsDefined() || node_.IsNull()) {
        Logger::error() << "[Cfg ERROR] Missing key at path: '" << path_ << "'";
        is_valid_ = false;
    }
}

CfgNode CfgNode::operator[](const std::string& key) const {
    std::string new_path = path_.empty() ? key : path_ + "." + key;
    
    if (!is_valid_) return CfgNode(false, new_path); // 父节点已错，安静地传递错误
    
    // 【核心拦截 2】：如果当前不是字典却用字符串访问，当场报错
    if (!node_.IsMap()) {
        Logger::error() << "[Cfg ERROR] Node '" << path_ << "' is not a dictionary. Cannot access key: '" << key << "'";
        return CfgNode(false, new_path);
    }
    
    return CfgNode(node_[key], new_path);
}

CfgNode CfgNode::operator[](size_t idx) const {
    std::string new_path = path_ + "[" + std::to_string(idx) + "]";
    
    if (!is_valid_) return CfgNode(false, new_path);
    
    // 【核心拦截 3】：检查数组访问权限及越界
    if (!node_.IsSequence()) {
        Logger::error() << "[Cfg ERROR] Node '" << path_ << "' is not a sequence. Cannot access index: " << idx;
        return CfgNode(false, new_path);
    }
    if (idx >= node_.size()) {
        Logger::error() << "[Cfg ERROR] Index out of bounds at '" << path_ << "'. Size is " << node_.size() << ", requested: " << idx;
        return CfgNode(false, new_path);
    }
    
    return CfgNode(node_[idx], new_path);
}

YAML::Node::const_iterator CfgNode::begin() const { 
    if (!is_valid_) {
        static const YAML::Node empty;
        return empty.begin();
    }
    // 【核心拦截 4】：不可迭代对象报错
    if (!node_.IsMap() && !node_.IsSequence()) {
        Logger::error() << "[Cfg ERROR] Cannot iterate over node '" << path_ << "' (not a dict or list).";
        static const YAML::Node empty;
        return empty.begin();
    }
    return node_.begin(); 
}

YAML::Node::const_iterator CfgNode::end() const { 
    if (!is_valid_ || (!node_.IsMap() && !node_.IsSequence())) {
        static const YAML::Node empty;
        return empty.end();
    }
    return node_.end(); 
}

// ... Cfg 的部分实现 ...

bool Cfg::fileExists(const std::string& p) {
    std::ifstream f(p);
    return f.good();
}

Cfg::Cfg(const std::string& filepath) {
    filepath_ = filepath.empty() ? getConfigPath() : filepath;

    if (!fileExists(filepath_)) {
        Logger::error() << "[Cfg] Cannot find config file: " << filepath_;
        return;
    }

    // 读取原始文本
    std::ifstream fin(filepath_);
    original_text_ = std::string(
        (std::istreambuf_iterator<char>(fin)),
        std::istreambuf_iterator<char>()
    );

    // 加载 YAML 对象
    try {
        root_ = YAML::LoadFile(filepath_);
        Logger::info() << "[Cfg] Loaded config: " << filepath_;
    } catch (const std::exception& e) {
        Logger::error() << "[Cfg] YAML Load Failed: " << std::string(e.what());
    }
}

CfgNode Cfg::operator[](const std::string& key) const {
    if (!root_.IsDefined() || !root_.IsMap()) {
        Logger::error() << "[Cfg ERROR] Root config is missing or not a dictionary.";
        return CfgNode(false, key);
    }
    return CfgNode(root_[key], key);
}

std::string Cfg::getConfigPath() const {
    std::string current_file_path = __FILE__;
    return (std::filesystem::path(current_file_path).parent_path().parent_path().parent_path() / "cfg" / "default.yaml").string();
}

void Cfg::internalSetScalar(const std::string& path, const std::string& val_str)
{
    auto keys = split(path);
    if (keys.empty()) return;

    std::vector<std::string> lines;
    std::istringstream iss(original_text_);
    std::string line;
    while (std::getline(iss, line)) lines.push_back(line);

    int line_no = findLineByFullPath(lines, keys);

    if (line_no == -1) {
        Logger::error() << "[Cfg ERROR] set failed! Path not found: " << path;
        return;
    }

    std::string& target_line = lines[line_no];
    size_t colon_pos = target_line.find(':');

    // 替换冒号后的内容为生成的字符串
    target_line = target_line.substr(0, colon_pos + 1) + " " + val_str;

    original_text_.clear();
    for (auto& l : lines) original_text_ += l + "\n";
}

void Cfg::internalSetBlock(const std::string& path, const std::vector<std::string>& block_lines)
{
    auto keys = split(path);
    if (keys.empty()) return;

    std::vector<std::string> lines;
    std::istringstream iss(original_text_);
    std::string line;
    while (std::getline(iss, line)) lines.push_back(line);

    int line_no = findLineByFullPath(lines, keys);

    if (line_no == -1) {
        Logger::error() << "[Cfg ERROR] set failed! Path not found: " << path;
        return;
    }

    // 获取当前层缩进
    int indent = 0;
    while (indent < (int)lines[line_no].size() && lines[line_no][indent] == ' ') indent++;
    int child_indent = indent + 2;

    // 删除旧 block
    int remove_start = line_no + 1;
    int remove_end = remove_start;

    while (remove_end < (int)lines.size()) {
        std::string tmp = trim(lines[remove_end]);
        if (tmp.empty()) { remove_end++; continue; }

        int curr_indent = 0;
        while (curr_indent < (int)lines[remove_end].size() && lines[remove_end][curr_indent] == ' ')
            curr_indent++;

        if (curr_indent <= indent) break;

        remove_end++;
    }

    if (remove_end > remove_start)
        lines.erase(lines.begin() + remove_start, lines.begin() + remove_end);

    // 插入新 block 字符串
    int insert_pos = remove_start;
    std::string child_indent_str(child_indent, ' ');

    for (const auto& row_str : block_lines) {
        lines.insert(lines.begin() + insert_pos, child_indent_str + row_str);
        insert_pos++;
    }

    original_text_.clear();
    for (auto& l : lines) original_text_ += l + "\n";
}

void Cfg::save() const
{
    std::ofstream fout(filepath_);
    if (!fout.is_open()) {
        Logger::error() << "[Cfg ERROR] Could not open file for writing: " << filepath_;
        return;
    }
    fout << original_text_;
    fout.close();
    Logger::info() << "[Cfg] Saved successfully to " << filepath_;
}