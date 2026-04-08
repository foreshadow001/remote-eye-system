#include <filesystem>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// 限制 Windows.h 包含的内容，加快编译
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#ifdef _MSC_VER
#pragma comment(lib, "user32.lib")
#endif

#include <opencv2/opencv.hpp>

// 假设这些是你项目中的自有头文件，保持包含即可
#include "cfg/config.hpp"
#include "logger/logger.hpp"

namespace fs = std::filesystem;
using namespace cv;

namespace gazeestimation {

// 获取 Windows 屏幕分辨率 (带 DPI 感知)
std::pair<int, int> getScreenResolution() {
    HMODULE user32 = LoadLibraryA("user32.dll");
    if (user32) {
        typedef BOOL(WINAPI *SetProcDPIAware_t)();
        auto p = (SetProcDPIAware_t)GetProcAddress(user32, "SetProcessDPIAware");
        if (p) p();
        FreeLibrary(user32);
    }
    int w = GetSystemMetrics(SM_CXSCREEN);
    int h = GetSystemMetrics(SM_CYSCREEN);
    return {w > 0 ? w : 1920, h > 0 ? h : 1080};
}

// 生成单个标记图案 (严格保持原版 Target 风格)
void drawMarkerPattern(Mat& img, const Point& center, const std::string& bg, int scale) {
    int cross_half = 10 * scale;     // 十字半长
    int circle_r   = 8 * scale;      // 小圆半径
    int dot_r      = 4 * scale;      // 红点半径
    int thick      = std::max(1, scale);

    Scalar cross_color = (bg == "dark") ? Scalar(0, 255, 0) : Scalar(0, 128, 0); // 绿色
    Scalar circle_color = (bg == "dark") ? Scalar(255, 255, 0) : Scalar(128, 128, 0); // 青色
    Scalar dot_color(0, 0, 255); // 红色中心点

    // 绘制十字、圆、中心红点
    // line(img, Point(center.x - cross_half, center.y), Point(center.x + cross_half, center.y), cross_color, thick, LINE_AA);
    // line(img, Point(center.x, center.y - cross_half), Point(center.x, center.y + cross_half), cross_color, thick, LINE_AA);
    // circle(img, center, circle_r, circle_color, thick, LINE_AA);
    circle(img, center, dot_r, dot_color, FILLED, LINE_AA);
}

// 生成标定图序列并保存
bool generateCalibrationImages(int rows, int cols, int margin_x, int margin_y,
                               const std::string& save_dir, int width, int height, int marker_scale) {
    if (rows <= 0 || cols <= 0) return false;

    // 自动获取屏幕分辨率作为图片大小
    if (width <= 0 || height <= 0) {
        auto [w, h] = getScreenResolution();
        width = w; height = h;
    }

    fs::create_directories(save_dir);

    std::vector<std::string> bgs = {"dark", "light"};
    for (const auto& bg : bgs) {
        int index = 0;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                // 计算坐标
                double x = (cols == 1) ? width / 2.0 : margin_x + c * (double)(width - 2 * margin_x) / (cols - 1);
                double y = (rows == 1) ? height / 2.0 : margin_y + r * (double)(height - 2 * margin_y) / (rows - 1);

                // 绘制
                Mat img(height, width, CV_8UC3, bg == "dark" ? Scalar(0, 0, 0) : Scalar(255, 255, 255));
                drawMarkerPattern(img, Point(cvRound(x), cvRound(y)), bg, marker_scale);

                // 文件命名并保存
                char filename[256];
                snprintf(filename, sizeof(filename), "%s_%02d_%d_%d.jpg", bg.c_str(), index++, cvRound(x), cvRound(y));
                imwrite((fs::path(save_dir) / filename).string(), img);
            }
        }
    }
    return true;
}

// 加载指定图片并在 Windows 下全屏显示
bool loadCalibrationImage(int target_index, const std::string& bg, const std::string& load_dir,
                          Mat& image, Point2f& point, bool show_fullscreen) {
    int count = 0;
    for (const auto& entry : fs::directory_iterator(load_dir)) {
        if (!entry.is_regular_file()) continue;
        std::string filename = entry.path().filename().string();

        // 匹配前缀 (dark_ / light_)
        if (filename.rfind(bg + "_", 0) != 0) continue;

        if (count == target_index) {
            // 解析坐标: 文件名格式如 dark_02_123_456.jpg
            int x = 0, y = 0;
            if (sscanf(filename.c_str(), "%*[^_]_%*d_%d_%d.jpg", &x, &y) >= 2) {
                point = Point2f((float)x, (float)y);
            }

            image = imread(entry.path().string());
            if (image.empty()) return false;

            if (show_fullscreen) {
                // 简化全屏逻辑：直接全屏化窗口，无需手动 Resize 和 Letterbox
                const std::string win = "CalibrationFullScreen";
                namedWindow(win, WINDOW_NORMAL);
                setWindowProperty(win, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
                imshow(win, image);
                waitKey(0);
                destroyWindow(win);
            }
            return true;
        }
        ++count;
    }
    return false;
}

} // namespace gazeestimation

int main() {
    Cfg cfg;
    Logger::info() << "Loading config file";
    int rows = cfg["calib_points"]["rows"].as<int>();
    int cols = cfg["calib_points"]["cols"].as<int>();
    int margin_x = cfg["calib_points"]["margin_x"].as<int>();
    int margin_y = cfg["calib_points"]["margin_y"].as<int>();
    std::string save_dir = cfg["calib_points"]["save_dir"].as<std::string>();
    int marker_scale = cfg["calib_points"]["marker_scale"].as<int>();
    std::string theme = cfg["calib_points"]["theme"].as<std::string>();

    // 生成图片 (width/height 传 0 将自动抓取当前屏幕分辨率)
    bool ok = gazeestimation::generateCalibrationImages(rows, cols, margin_x, margin_y, save_dir, 0, 0, marker_scale);
    if (!ok) {
        Logger::error() << "Failed to generate calibration images.";
        return -1;
    }

    cv::Mat img;
    cv::Point2f p;
    // 加载并全屏显示第 2 张图片
    if (gazeestimation::loadCalibrationImage(2, theme, save_dir, img, p, true)) {
        // p 即 PoG 像素坐标
        std::cout << "PoG: " << p.x << ", " << p.y << std::endl;
    } else {
        Logger::error() << "Image load or display failed.";
    }

    return 0;
}