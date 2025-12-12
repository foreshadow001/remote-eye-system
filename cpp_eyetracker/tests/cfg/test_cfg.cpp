#include "cfg/config.hpp"

int main() {
    Cfg cfg;

    // 修改标量
    cfg.setScalar("test_glint_hyperparameter.horizontal_pair.lr_y_min", 1.0);

    // 修改 vector<double>
    std::vector<double> v = {-1, 2, 0.20, 0.80, 0.05, 0.55, 0.20, 0.80, 0.05, 0.55};

    // 修改 vector<vector<double>>
    auto conditions = cfg["test_glint_hyperparameter"]["middle_point"]["conditions"]
                        .as<std::vector<std::vector<double>>>();

    conditions[0] = v;

    cfg.setVector2D("test_glint_hyperparameter.middle_point.conditions", conditions);

    // 写回文件，不改变注释
    cfg.save();

    return 0;
}