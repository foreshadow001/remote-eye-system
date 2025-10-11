#ifndef RECORD_CALIB_DATA_HPP_INCLUDED
#define RECORD_CALIB_DATA_HPP_INCLUDED

#include "basler.hpp"
#include "set_image/create_image.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace gazeestimation {

bool recordCalibrationSequence(
    BaslerCamera& cam,
    const std::string& image_dir,
    const std::string& video_dir
);

}

#endif
