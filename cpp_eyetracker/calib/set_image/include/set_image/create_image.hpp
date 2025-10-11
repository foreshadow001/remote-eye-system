#ifndef SET_IMAGE_CREATE_IMAGE_HPP_INCLUDED
#define SET_IMAGE_CREATE_IMAGE_HPP_INCLUDED

#include <opencv2/opencv.hpp>
#include <string>
#include <utility>

namespace gazeestimation {

static void drawMarkerPattern(cv::Mat& img, const cv::Point& center, const std::string& bg, int scale);

/**
 * Try to get primary screen resolution (pixels).
 * Returns {width, height}. If detection fails returns {1920,1080}.
 */
std::pair<int,int> getScreenResolution();

/**
 * Generate calibration images (black & white sets).
 * If width==0 or height==0, function will try to detect the screen resolution.
 *
 * @param rows, cols        grid of calibration points
 * @param margin_x,y        margins in pixels (left/right, top/bottom)
 * @param save_dir          output directory
 * @param width,height      output image resolution; if 0 => auto-detect screen resolution
 */
bool generateCalibrationImages(
    int rows,
    int cols,
    int margin_x,
    int margin_y,
    const std::string& save_dir,
    int width = 0,
    int height = 0,
    int marker_scale = 1
);

/**
 * Load a calibration image by index and bg ("dark"/"light").
 * If show_fullscreen==true the image will be shown fullscreen (blocking until key pressed).
 * Outputs loaded image and PoG pixel coordinates.
 */
bool loadCalibrationImage(
    int index,
    const std::string& bg,
    const std::string& load_dir,
    cv::Mat& image,
    cv::Point2f& point,
    bool show_fullscreen = false
);

/**
 * Show an image fullscreen (scaled to fit, preserving aspect ratio and centered).
 * If screen_w/h == 0 will auto-detect screen resolution.
 */
void showImageFullscreen(const std::string& window_name, const cv::Mat& img, int screen_w = 0, int screen_h = 0);

void showImageFullscreenCapture(const std::string& window_name, const cv::Mat& img, int screen_w = 0, int screen_h = 0);

} // namespace gazeestimation

#endif // SET_IMAGE_CREATE_IMAGE_HPP_INCLUDED
