#include "cfg/config.hpp"
#include <sstream> // 必须包含，否则 split 和 istringstream 会报错
#include <algorithm> // for trim

// --- 辅助工具 ---

static inline void yellow(const std::string& msg) {
    std::cerr << "\033[33m" << msg << "\033[0m" << std::endl;
}

static inline void red(const std::string& msg) {
    std::cerr << "\033[31m" << msg << "\033[0m" << std::endl;
}

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


// ... CfgNode 的实现保持不变 ...
CfgNode::CfgNode() {}
CfgNode::CfgNode(const YAML::Node& node, const std::string& path) : node_(node), path_(path) {}
CfgNode CfgNode::operator[](const std::string& key) const {
    std::string new_path = path_.empty() ? key : path_ + "." + key;
    return CfgNode(node_[key], new_path);
}
CfgNode CfgNode::operator[](size_t idx) const {
    return CfgNode(node_[idx], path_ + "[" + std::to_string(idx) + "]");
}
YAML::Node::const_iterator CfgNode::begin() const { return node_.begin(); }
YAML::Node::const_iterator CfgNode::end() const { return node_.end(); }
void CfgNode::printWarning() const { yellow("[Cfg WARNING] Missing key: " + path_); }


// ... Cfg 的部分实现 ...

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

    // 读取原始文本
    std::ifstream fin(filepath_);
    original_text_ = std::string(
        (std::istreambuf_iterator<char>(fin)),
        std::istreambuf_iterator<char>()
    );

    // 加载 YAML 对象
    try {
        root_ = YAML::LoadFile(filepath_);
        std::cout << "[Cfg] Loaded config: " << filepath_ << std::endl;
    } catch (const std::exception& e) {
        red("[Cfg ERROR] YAML Load Failed: " + std::string(e.what()));
    }
}

CfgNode Cfg::operator[](const std::string& key) const {
    return CfgNode(root_[key], key);
}

std::string Cfg::getConfigPath() const {
    std::string current_file_path = __FILE__;
    return (std::filesystem::path(current_file_path).parent_path().parent_path().parent_path() / "cfg" / "default.yaml").string();
}

void Cfg::setScalar(const std::string& path, double v)
{
    auto keys = split(path);
    if (keys.empty()) return;

    // 1. 拆路径
    // keys = ["test_glint_hyperparameter", "horizontal_pair", "lr_y_min"]

    // 2. 按行读原文
    std::vector<std::string> lines;
    std::istringstream iss(original_text_);
    std::string line;
    while (std::getline(iss, line)) lines.push_back(line);

    // 3. 用完整路径查找
    int line_no = findLineByFullPath(lines, keys);

    if (line_no == -1) {
        red("[Cfg ERROR] setScalar failed! Full path not found: " + path);
        return;
    }

    // 4. 替换该行
    std::string& target_line = lines[line_no];
    size_t colon_pos = target_line.find(':');

    target_line = target_line.substr(0, colon_pos + 1) + " " + std::to_string(v);

    // 5. 重建原文缓存
    original_text_.clear();
    for (auto& l : lines) original_text_ += l + "\n";
}


void Cfg::setVector2D(const std::string& path, const std::vector<std::vector<double>>& vv)
{
    auto keys = split(path);
    if (keys.empty()) return;

    // 1. 按行读取
    std::vector<std::string> lines;
    std::istringstream iss(original_text_);
    std::string line;
    while (std::getline(iss, line)) lines.push_back(line);

    // 2. 找 conditions 行
    int line_no = findLineByFullPath(lines, keys);

    if (line_no == -1) {
        red("[Cfg ERROR] setVector2D failed! Full path not found: " + path);
        return;
    }

    // 3. 获取当前 indent
    int indent = 0;
    while (indent < (int)lines[line_no].size() && lines[line_no][indent] == ' ') indent++;

    int child_indent = indent + 2;

    // 4. 删除旧 block
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

    // 5. 插入新 block
    int insert_pos = remove_start;
    std::string child_indent_str(child_indent, ' ');

    for (const auto& row : vv) {
        std::ostringstream oss;
        oss << child_indent_str << "- [";
        for (size_t i = 0; i < row.size(); i++) {
            oss << row[i];
            if (i + 1 < row.size()) oss << ", ";
        }
        oss << "]";
        lines.insert(lines.begin() + insert_pos, oss.str());
        insert_pos++;
    }

    // 6. 重建原文缓存
    original_text_.clear();
    for (auto& l : lines) original_text_ += l + "\n";
}


void Cfg::save() const
{
    std::ofstream fout(filepath_);
    if (!fout.is_open()) {
        red("[Cfg ERROR] Could not open file for writing: " + filepath_);
        return;
    }
    fout << original_text_;
    fout.close();
    std::cout << "[Cfg] Saved to " << filepath_ << std::endl;
}