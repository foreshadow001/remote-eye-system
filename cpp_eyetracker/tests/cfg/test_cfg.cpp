#include "cfg/config.hpp"
#include "logger/logger.hpp" // 假设 logger 已正确配置

int main() {
    Cfg cfg; // 默认读取 default.yaml

    Logger::info() << "========== [ TEST WRITING PRECISION & TYPES ] ==========";

    // 1. 写入整型（不带小数点）
    // 预期写入: "screen_width_px: 1920" (没有 .000000 的恶心尾巴了)
    cfg.setScalar<int>("screen_width_px", 1920);

    // 2. 写入浮点型并控制精度（保留 4 位小数）
    // 预期写入: "focus: 0.0123"
    cfg.setScalar<double>("cam_calib.focus", 0.01234567, 4);

    // 3. 写入一维数组（保留 1 位小数）
    // 预期写入: "cam_pos: [30.1, -30.5, 18.0]"
    std::vector<double> new_cam_pos = {30.123, -30.54, 18.0};
    cfg.setVector<double>("cam_pos", new_cam_pos, 1);

    // 4. 写入二维数组（保留 2 位小数）
    auto conditions = cfg["original_specific_hyperparameter"]["glint"]["middle_point"]["conditions"].as<std::vector<std::vector<double>>>();
    if (!conditions.empty()) {
        // 第一行改为全是 0.12 的数值
        conditions[0] = {0, 2, 0.2, 0.8, 0.05, 0.55, 0.2, 0.8, 0.05, 0.55};
        // 预期写入的数组每个数字都会四舍五入保留 2 位
        cfg.setVector2D<double>("original_specific_hyperparameter.glint.middle_point.conditions", conditions, 2);
    }

    cfg.save();

    Logger::info() << "Please check the saved yaml file formatting!";
    return 0;
}