#pragma once

#include <opencv2/core.hpp>

#include <core/math_types.hpp>
#include <utils/gaze_estimation_types.hpp>

namespace gazeestimation
{

Vec3
calculatePointOfInterest(
    const Vec3& cornea_center,
    const Vec3& visual_axis_unit,
    double z_shift
);

Vec3
calculateReflex(
    double reflex_cam_dist,
    const Vec3& cam_pos,
    const Vec3& glint_wcs
);

Vec3
reflexToCorneaCenter(
    const Vec3& reflex,
    const Vec3& light,
    const Vec3& cam_pos,
    const double R
);

Vec3
calculateOpticalAxisUnit(
    const Vec3& pupil_image_wcs,
    const Vec3& cam_pos,
    const Vec3& cornea_center, 
    double R, double K,
    double n1, double n2,
    bool use_chen_noise_reduction
);

Vec3
calculateRefractionPoint(
    const Vec3& cam_pos,
    const Vec3& pupil_image_wcs,
    const Vec3& cornea_center_wcs,
    double R
);

double
calculateRefractionPointCamDistCoef(
    const Vec3& cam_pos,
    const Vec3& pupil_image_wcs,
    const Vec3& cornea_center,
    double R
);

Vec3
calculatePupilCenterWCS(
    const Vec3& cam_pos,
    const Vec3& refraction_point,
    const Vec3& cornea_center,
    double R, double K,
    double n1, double n2
);

Vec3
calculateRefractionRayUnit(
    const Vec3& cam_pos,
    const Vec3& refraction_point,
    const Vec3& cornea_center,
    double R, double n1, double n2
);

Vec3
calculateVisualAxisUnit(
    const Vec3& optical_axis_unit,
    double alpha, double beta
);

Vec3
calculateKappaVector(double alpha, double beta);

Vec3
calculateOpticalAxisVector(const Vec3& optic_axis_unit);

Mat3x3
calculateOpticalAxisRotationMatrix(double theta, double phi, double kappa);

bool isGlintValid(const Vec2& glint);

double
calculateDiamondAverage(const cv::Mat& img, double cx, double cy, int level, int k = 2);

}