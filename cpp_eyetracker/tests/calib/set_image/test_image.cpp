#include "set_image/create_image.hpp"
using namespace gazeestimation;

int main() {
    // 传 0,0 自动检测屏幕分辨率（Windows 下已尝试 DPI aware）
    bool ok = generateCalibrationImages(3, 5, 200, 150, "D:/users/projects/new_dataset/calibration_images", 0, 0, 3);
    if (!ok) return -1;

    cv::Mat img;
    cv::Point2f p;
    // 加载并全屏显示第 2 张 dark 背景
    if (loadCalibrationImage(2, "dark", "D:/users/projects/new_dataset/calibration_images", img, p, true)) {
        // p 即 PoG 像素坐标
        std::cout << "PoG: " << p.x << ", " << p.y << std::endl;
    }
}
