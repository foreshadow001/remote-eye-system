#include <windows.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "utils/gaze_estimation_types.hpp"
#include "core/math_types.hpp"
#include "glint_detection/detect_glint.hpp"
#include "pupil_center/localize_pupil.hpp"
#include "inference/one_camera_spherical.hpp"

using namespace gazeestimation;

const std::string input_folder  = "D:/users/projects/new_dataset/data_collection/PCCR/test_dataset/images/src";

int main() {
    std::string search_path = input_folder + "\\*.*";
    WIN32_FIND_DATA fd;
    HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "the input folder does not exist or is not a folder: " << input_folder << std::endl;
        return -1;
    }

    GazeTracker gazetracker = gazeestimation::GazeTracker();

	EyeAndCameraParameters parameters;
	parameters.alpha = deg_to_rad(-5);
	parameters.beta = deg_to_rad(1.5);
	parameters.R = 0.78;
	parameters.K = 0.42;
	parameters.n1 = 1.3375;
	parameters.n2 = 1;
	parameters.D = 0.53;

	const Vec3 actual_camera_position = make_vec3(24.5, -35, 10);
	const Vec3 wcs_offset = -make_vec3(24.5, -35, 10);

	PinholeCameraModel camera;
	camera.principal_point_x = 299.5;
	camera.principal_point_y = 399.5;
	camera.pixel_size_cm_x = 2.4 * 1e-6;
	camera.pixel_size_cm_y = 2.4 * 1e-6;
	camera.effective_focal_length_cm = 0.0119144;
	camera.position = actual_camera_position + wcs_offset;
	camera.set_camera_angles(deg_to_rad(8), 0, 0);
	parameters.cameras.push_back(camera);

	std::vector<Vec3> lights;
	lights.push_back(actual_camera_position + make_vec3(25, 0, 3) + wcs_offset);
	lights.push_back(actual_camera_position + make_vec3(-25, 0, 3) + wcs_offset);
    lights.push_back(actual_camera_position + make_vec3(0, -11, 8) + wcs_offset);
	parameters.light_positions = lights;

    parameters.eye_cam_dist_init = 70;

    int idx = 0;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        {
            std::string filename = fd.cFileName;
            std::string filepath = input_folder + "\\" + filename;

            cv::Mat img = cv::imread(filepath, cv::IMREAD_COLOR);

            if (img.empty())
            {
                std::cerr << "failed to read image: " << filepath << std::endl;
                continue;
            }

            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

            // call searchForGlints
            auto [leftEyeGlints, rightEyeGlints, processed_img_left, processed_img_right] = glintdetection::searchForGlints(img, 50.0);

            // convert glints to pupil centers
            auto [leftPupilCenter, leftEyeImage] = pupilcenter::localizePupilCenter(gray, leftEyeGlints);
            auto [rightPupilCenter, rightEyeImage] = pupilcenter::localizePupilCenter(gray, rightEyeGlints);

            PupilCenterGlintInputs inputs;
            PupilCenterGlintInput input;

            for (const auto& g : leftEyeGlints)
            {
                input.glints.push_back(make_vec2(g.x, g.y));
            }

            input.pupil_center = make_vec2(leftPupilCenter.x, leftPupilCenter.y);
            inputs.data.push_back(input);

            std::cout << "left pupil center: " << leftPupilCenter << std::endl;
            std::cout << "right pupil center: " << rightPupilCenter << std::endl;

            try {
                auto gaze_result = gazetracker.estimate(inputs, parameters);
            } catch (const std::exception& e) {
                std::cerr << "Exception caught: " << e.what() << std::endl;
            }

            idx++;
        }
    } while (::FindNextFile(hFind, &fd));
    ::FindClose(hFind);

    std::cout << "processed " << idx << " images." << std::endl;
    return 0;
}
