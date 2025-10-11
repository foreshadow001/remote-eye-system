#include <filesystem>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <vector>

namespace fs = std::filesystem;
using namespace std;

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <windows.h>
    // No need to link explicitly here if using CMake, but for MSVC direct compile the pragma can help:
    #ifdef _MSC_VER
    #pragma comment(lib, "user32.lib")
    #endif
#endif

#if defined(__APPLE__)
    #include <ApplicationServices/ApplicationServices.h>
#endif

#if !defined(_WIN32) && !defined(__APPLE__)
    // try X11 on Linux
    #include <X11/Xlib.h>
#endif

#include "set_image/create_image.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

namespace gazeestimation {

std::pair<int,int> getScreenResolution()
{
#if defined(_WIN32)
    // Try to make process DPI aware so we get real pixel size on scaled displays:
    HMODULE user32 = LoadLibraryA("user32.dll");
    if (user32) {
        typedef BOOL (WINAPI *SetProcDPIAware_t)();
        SetProcDPIAware_t p = (SetProcDPIAware_t)GetProcAddress(user32, "SetProcessDPIAware");
        if (p) p();
        FreeLibrary(user32);
    }
    int w = GetSystemMetrics(SM_CXSCREEN);
    int h = GetSystemMetrics(SM_CYSCREEN);
    if (w > 0 && h > 0) return {w, h};
#elif defined(__APPLE__)
    CGDirectDisplayID d = CGMainDisplayID();
    int w = static_cast<int>(CGDisplayPixelsWide(d));
    int h = static_cast<int>(CGDisplayPixelsHigh(d));
    if (w > 0 && h > 0) return {w, h};
#else
    Display* dpy = XOpenDisplay(nullptr);
    if (dpy) {
        Screen* scr = DefaultScreenOfDisplay(dpy);
        if (scr) {
            int w = scr->width;
            int h = scr->height;
            XCloseDisplay(dpy);
            if (w > 0 && h > 0) return {w, h};
        } else {
            XCloseDisplay(dpy);
        }
    }
#endif
    // fallback
    return {1920, 1080};
}

// 生成单个标记图案
static void drawMarkerPattern(cv::Mat& img, const cv::Point& center, const std::string& bg, int scale) {
    int cross_half = 10 * scale;     // 十字半长
    int circle_r   = 8 * scale;      // 小圆半径
    int dot_r      = 2 * scale;      // 红点半径
    int thick      = std::max(1, scale);

    Scalar cross_color = (bg == "dark") ? Scalar(0, 255, 0) : Scalar(0, 128, 0); // 绿色
    Scalar circle_color = (bg == "dark") ? Scalar(255, 255, 0) : Scalar(128, 128, 0); // 青色
    Scalar dot_color(0, 0, 255); // 红色中心点

    // 十字
    line(img, Point(center.x - cross_half, center.y), Point(center.x + cross_half, center.y), cross_color, thick, LINE_AA);
    line(img, Point(center.x, center.y - cross_half), Point(center.x, center.y + cross_half), cross_color, thick, LINE_AA);

    // 圆
    circle(img, center, circle_r, circle_color, thick, LINE_AA);

    // 红点
    circle(img, center, dot_r, dot_color, FILLED, LINE_AA);
}

bool generateCalibrationImages(
    int rows,
    int cols,
    int margin_x,
    int margin_y,
    const std::string& save_dir,
    int width,
    int height,
    int marker_scale // 新增参数，控制标志大小
)
{
    if (rows <= 0 || cols <= 0) {
        cerr << "[set_image] invalid rows/cols\n";
        return false;
    }

    // 如果用户没传屏幕分辨率，就用默认
    if (width <= 0 || height <= 0) {
        width = 2560;
        height = 1440;
        cout << "[set_image] auto screen size: " << width << "x" << height << "\n";
    }

    // 确保输出目录存在
    try {
        fs::create_directories(save_dir);
    } catch (const std::exception& e) {
        cerr << "[set_image] cannot create directory '" << save_dir << "': " << e.what() << "\n";
        return false;
    }

    // 均匀分布的标定点坐标
    vector<Point2f> points;
    points.reserve(rows * cols);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double x = (cols == 1)
                           ? width / 2.0
                           : margin_x + c * (double)(width - 2 * margin_x) / (cols - 1);
            double y = (rows == 1)
                           ? height / 2.0
                           : margin_y + r * (double)(height - 2 * margin_y) / (rows - 1);
            points.emplace_back((float)x, (float)y);
        }
    }

    // 背景类型
    vector<string> bgs = {"dark", "light"};
    for (const auto& bg : bgs) {
        for (size_t i = 0; i < points.size(); ++i) {
            Mat img(height, width, CV_8UC3, bg == "dark" ? Scalar(0, 0, 0) : Scalar(255, 255, 255));
            Point2f p = points[i];

            // 绘制新标记
            drawMarkerPattern(img, Point(cvRound(p.x), cvRound(p.y)), bg, marker_scale);

            // 文件命名
            int ix = cvRound(p.x);
            int iy = cvRound(p.y);
            char filename[256];
            snprintf(filename, sizeof(filename), "%s_%02zu_%d_%d.jpg", bg.c_str(), i, ix, iy);
            fs::path filepath = fs::path(save_dir) / filename;

            if (!imwrite(filepath.string(), img)) {
                cerr << "[set_image] failed to save " << filepath.string() << "\n";
                return false;
            }
        }
    }

    cout << "[set_image] saved " << (rows * cols * 2) << " images to " << save_dir << "\n";
    return true;
}

void showImageFullscreen(const std::string& window_name, const cv::Mat& img, int screen_w, int screen_h)
{
    if (img.empty()) return;

    if (screen_w <= 0 || screen_h <= 0) {
        auto wh = getScreenResolution();
        screen_w = wh.first;
        screen_h = wh.second;
    }

    // compute scaled size preserving aspect ratio
    double scale = std::min((double)screen_w / img.cols, (double)screen_h / img.rows);
    int newW = std::max(1, (int)std::round(img.cols * scale));
    int newH = std::max(1, (int)std::round(img.rows * scale));

    Mat resized;
    if (newW != img.cols || newH != img.rows) {
        resize(img, resized, Size(newW, newH), 0, 0, (scale < 1.0) ? INTER_AREA : INTER_LINEAR);
    } else {
        resized = img;
    }

    // letterbox on black canvas
    Mat canvas(screen_h, screen_w, img.type(), Scalar::all(0));
    int x = (screen_w - newW) / 2;
    int y = (screen_h - newH) / 2;
    resized.copyTo(canvas(Rect(x, y, newW, newH)));

    const string win = window_name.empty() ? "CalibrationFullScreen" : window_name;

    namedWindow(win, WINDOW_NORMAL);
    // Try native fullscreen first:
    setWindowProperty(win, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
    imshow(win, canvas);

    // fallback: if fullscreen not applied, try resizing window and moving
    // (some backends may not honor WND_PROP_FULLSCREEN)
    int applied = (int)getWindowProperty(win, WND_PROP_FULLSCREEN);
    if (applied != WINDOW_FULLSCREEN) {
        // move & resize (border/title may still exist)
        resizeWindow(win, screen_w, screen_h);
        moveWindow(win, 0, 0);
    }

    // wait for a key
    waitKey(0);
    destroyWindow(win);
}

// --- 全屏显示函数（非阻塞） ---
void showImageFullscreenCapture(const std::string& window_name, const cv::Mat& img, int screen_w, int screen_h)
{
    if (img.empty()) return;

    if (screen_w <= 0 || screen_h <= 0) {
        auto wh = getScreenResolution();
        screen_w = wh.first;
        screen_h = wh.second;
    }

    double scale = std::min((double)screen_w / img.cols, (double)screen_h / img.rows);
    int newW = std::max(1, (int)std::round(img.cols * scale));
    int newH = std::max(1, (int)std::round(img.rows * scale));

    Mat resized;
    resize(img, resized, Size(newW, newH), 0, 0, (scale < 1.0) ? INTER_AREA : INTER_LINEAR);

    Mat canvas(screen_h, screen_w, img.type(), Scalar::all(0));
    int x = (screen_w - newW) / 2;
    int y = (screen_h - newH) / 2;
    resized.copyTo(canvas(Rect(x, y, newW, newH)));

    const string win = window_name.empty() ? "CalibrationFullScreen" : window_name;
    namedWindow(win, WINDOW_NORMAL);
    setWindowProperty(win, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
    imshow(win, canvas);
}

bool loadCalibrationImage(
    int index,
    const std::string& bg,
    const std::string& load_dir,
    cv::Mat& image,
    cv::Point2f& point,
    bool show_fullscreen
)
{
    if (bg != "dark" && bg != "light") {
        cerr << "[set_image] bg must be 'dark' or 'light'\n";
        return false;
    }

    int count = 0;
    try {
        for (const auto& entry : fs::directory_iterator(load_dir)) {
            if (!entry.is_regular_file()) continue;
            string filename = entry.path().filename().string();

            // match prefix
            if (filename.rfind(bg + "_", 0) != 0) continue;

            if (count == index) {
                // parse filename like: dark_02_123_456.jpg
                int x = 0, y = 0;
                // tolerate extra suffixes by scanning last two ints
                // try sscanf on the filename
                if (sscanf(filename.c_str(), "%*[^_]_%*d_%d_%d.jpg", &x, &y) >= 2) {
                    point = Point2f((float)x, (float)y);
                } else {
                    // fallback: set to center
                    point = Point2f(-1, -1);
                }

                image = imread(entry.path().string());
                if (image.empty()) {
                    cerr << "[set_image] cannot load image: " << entry.path().string() << "\n";
                    return false;
                }

                if (show_fullscreen) {
                    auto wh = getScreenResolution();
                    showImageFullscreen("Calibration", image, wh.first, wh.second);
                }

                return true;
            }
            ++count;
        }
    } catch (const std::exception& e) {
        cerr << "[set_image] load error: " << e.what() << "\n";
        return false;
    }

    cerr << "[set_image] image index not found\n";
    return false;
}

} // namespace gazeestimation
