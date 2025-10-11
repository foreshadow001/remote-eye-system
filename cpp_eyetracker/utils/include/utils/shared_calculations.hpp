#pragma once

#include <utils/math_types.hpp>
#include <utils/gaze_estimation_types.hpp>

namespace gazeestimation
{

inline Vec3
calculatePointOfInterest(
    const Vec3& cornea_center,
    const Vec3& visual_axis_unit,
    double z_shift
);

inline Vec3
calculateReflex(
    double reflex_cam_dist,
    const Vec3& cam_pos,
    const Vec3& glint_wcs
);

inline Vec3
reflexToCorneaCenter(
    const Vec3& reflex,
    const Vec3& light,
    const Vec3& cam_pos,
    const double R
);

inline Vec3
calculateOpticalAxisUnit(
    const Vec3& pupil_image_wcs,
    const Vec3& cam_pos,
    const Vec3& cornea_center, 
    double R, double K,
    double n1, double n2,
    bool use_chen_noise_reduction
);

inline Vec3
calculateRefractionPoint(
    const Vec3& cam_pos,
    const Vec3& pupil_image_wcs,
    const Vec3& cornea_center_wcs,
    double R
);

inline double
calculateRefractionPointCamDistCoef(
    const Vec3& cam_pos,
    const Vec3& pupil_image_wcs,
    const Vec3& cornea_center,
    double R
);

inline Vec3
calculatePupilCenterWCS(
    const Vec3& cam_pos,
    const Vec3& refraction_point,
    const Vec3& cornea_center,
    double R, double K,
    double n1, double n2
);

inline Vec3
calculateRefractionRayUnit(
    const Vec3& cam_pos,
    const Vec3& refraction_point,
    const Vec3& cornea_center,
    double R, double n1, double n2
);

inline Vec3
calculateVisualAxisUnit(
    const Vec3& optical_axis_unit,
    double alpha, double beta
);

inline Vec3
calculateKappaVector(double alpha, double beta);

inline Vec3
calculateOpticalAxisVector(const Vec3& optic_axis_unit);

inline Mat3x3
calculateOpticalAxisRotationMatrix(double theta, double phi, double kappa);

inline Vec3
calculatePointOfInterest(
    const Vec3& cornea_center,
    const Vec3& visual_axis_unit,
    double z_shift
)
/*
Calculates the point of interest (POI) based on the cornea center, the visual axis unit vector, and the z shift.
The POI is the point on the visual axis that is closest to the cornea center.

Args:
    cornea_center: The center of the cornea in the camera coordinate system.
    visual_axis_unit: The unit vector of the visual axis in the camera coordinate system.
    z_shift: The z shift of the cornea in the camera coordinate system.

Returns:
    The point of interest in the camera coordinate system.
*/
{
	const double cornea_screen_dist = (z_shift - cornea_center[2]) / visual_axis_unit[2];
	return cornea_center + cornea_screen_dist * visual_axis_unit;
}


inline Vec3
calculateReflex(
    double reflex_cam_dist,
    const Vec3& cam_pos,
    const Vec3& glint_wcs
)
/*
Calculate the reflection point of light source in WCS.

Args:
    reflex_cam_dist: distance between the camera and the reflection point
    cam_pos: camera position in WCS
    glint_wcs: light source position in WCS

Returns:
    The reflection point in WCS.
*/
{
    return cam_pos + reflex_cam_dist * normalized(cam_pos - glint_wcs);
} // calculateReflex


inline Vec3
reflexToCorneaCenter(
    const Vec3& reflex,
    const Vec3& light,
    const Vec3& cam_pos,
    const double R
)
/*
Calculate the center of cornea in WCS.

Args:
    reflex: the reflection point of light source in WCS
    light: the light source position in WCS
    cam_pos: the camera position in WCS
    R: the radius of the cornea

Returns:
    The center of cornea in WCS.
*/
{
    const Vec3 light_reflex_unit = normalized(light - reflex);
    const Vec3 cam_reflex_unit = normalized(cam_pos - reflex);
    return reflex - R * normalized(light_reflex_unit + cam_reflex_unit);
} // calculateCorneaCenter

inline Vec3
calculateOpticalAxisUnit(
    const Vec3& pupil_image_wcs,
    const Vec3& cam_pos,
    const Vec3& cornea_center, 
    double R, double K,
    double n1, double n2,
    bool use_chen_noise_reduction
)
/*
Calculate the unit vector of the optical axis.

Args:
    pupil_image_wcs: the pupil image position in WCS
    cam_pos: the camera position in WCS
    cornea_center: the center of cornea in WCS
    R: the radius of the cornea
    K: the focal length of the camera
    n1: the refractive index of the air
    n2: the refractive index of the cornea and anterior chamber

Returns:
    The unit vector of the optical axis.
*/
{
    const Vec3 refraction_point = calculateRefractionPoint(
        cam_pos, pupil_image_wcs, cornea_center, R
    );

    Vec3 pupil_center_wcs = calculatePupilCenterWCS(
        cam_pos, refraction_point, cornea_center, R, K, n1, n2
    );

    // force the pupil center to be on the cornea surface
    if(use_chen_noise_reduction)
    {
        double cxpx = cornea_center[0] - pupil_center_wcs[0];
        double cypy = cornea_center[1] - pupil_center_wcs[1];
        pupil_center_wcs[2] = cornea_center[2] - sqrt(K * K - cxpx * cxpx - cypy * cypy);
    }

    return normalized(pupil_center_wcs - cornea_center);
}

inline Vec3
calculateRefractionPoint(
    const Vec3& cam_pos,
    const Vec3& pupil_image_wcs,
    const Vec3& cornea_center_wcs,
    double R
)
/*
Calculate the refraction point of the pupil image.

Args:
    cam_pos: the camera position in WCS
    pupil_image_wcs: the pupil image position in WCS
    cornea_center_wcs: the center of cornea in WCS
    R: the radius of the cornea

Returns:
    The refraction point in WCS.
*/
{
    const double refraction_point_cam_dist_coef = calculateRefractionPointCamDistCoef(cam_pos, pupil_image_wcs, cornea_center_wcs, R);
    return cam_pos + refraction_point_cam_dist_coef * (cam_pos - pupil_image_wcs);
}

inline double
calculateRefractionPointCamDistCoef(
    const Vec3& cam_pos,
    const Vec3& pupil_image_wcs,
    const Vec3& cornea_center,
    double R
)
/*
Calculate the distance between the camera and the refraction point.

Args:
    cam_pos: the camera position in WCS
    pupil_image_wcs: the pupil image position in WCS
    cornea_center: the center of cornea in WCS
    R: the radius of the cornea

Returns:
    The coefficient between the refraction_point_cam_dist and the pupil_image_cam_dist.
*/
{
    const double a = squared_length(cam_pos - pupil_image_wcs);
    const double b = dot(pupil_image_wcs - cam_pos, cam_pos - cornea_center);
    const double c = squared_length(cam_pos - cornea_center) - R * R;

    return ( b - sqrt(b * b - a * c)) / a;
}

inline Vec3
calculatePupilCenterWCS(
    const Vec3& cam_pos,
    const Vec3& refraction_point,
    const Vec3& cornea_center,
    double R, double K,
    double n1, double n2
)
/*
Calculate the pupil center in WCS.

Args:
    cam_pos: the camera position in WCS
    refraction_point: the refraction point in WCS
    cornea_center: the center of cornea in WCS
    R: the radius of the cornea
    K: the focal length of the camera
    n1: the refractive index of the air
    n2: the refractive index of the cornea and anterior chamber

Returns:
    The pupil center in WCS.
*/
{
    /*
    Vec3 refraction_ray_unit = calculateRefractionRayUnit(
        cam_pos, refraction_point, cornea_center, R, n1, n2
    );

    double R_cos_theta_2 = dot((refraction_point - cornea_center), refraction_ray_unit);

    double refraction_point_cornea_center_dist = R_cos_theta_2 - sqrt(
        R_cos_theta_2 * R_cos_theta_2 - (R * R - K * K)
    );

    return refraction_point + refraction_point_cornea_center_dist * refraction_ray_unit;
    */

    Vec3 zeta = normalized(cam_pos - refraction_point);
    Vec3 eta = (refraction_point - cornea_center) / R;
    double eta_dot_zeta = dot(eta, zeta);

    double a = eta_dot_zeta - sqrt((n1 / n2)*(n1 / n2) - 1 + eta_dot_zeta * eta_dot_zeta);
    Vec3 iota =  (n2 / n1) * (a * eta - zeta);

    double rc_dot_iota = dot((refraction_point - cornea_center), iota);
    double kp = - 1 * rc_dot_iota - sqrt(rc_dot_iota*rc_dot_iota - (R * R - K * K));
    return refraction_point + kp * iota;
}

inline Vec3
calculateRefractionRayUnit(
    const Vec3& cam_pos,
    const Vec3& refraction_point,
    const Vec3& cornea_center,
    double R, double n1, double n2
)
/*
Calculate the unit vector of the refraction ray.

Args:
    cam_pos: the camera position in WCS
    refraction_point: the refraction point in WCS
    cornea_center: the center of cornea in WCS
    R: the radius of the cornea
    n1: the refractive index of the air
    n2: the refractive index of the cornea and anterior chamber

Returns:
    The unit vector of the refraction ray.
*/
{
    Vec3 incident_ray_unit = normalized(refraction_point - cam_pos);
    Vec3 normal = (refraction_point - cornea_center) / R;
    double eta = n2 / n1;
    double cos_theta_1 = - dot(incident_ray_unit, normal);
    double k = 1 - (eta * eta) * (1 - cos_theta_1 * cos_theta_1);

    if (k < 0) return make_vec3(0, 0, 0);

    double cos_theta_2 = sqrt(k);

    return eta * incident_ray_unit + (eta * cos_theta_1 - cos_theta_2) * normal;
}

inline Vec3
calculateVisualAxisUnit(
    const Vec3& optical_axis_unit,
    double alpha, double beta
)
/*
Calculate the unit vector of the visual axis.

Args:
    optical_axis_unit: the unit vector of the optical axis
    alpha: delta pitch between the optic axis and the visual axis
    beta: delta yaw between the optic axis and the visual axis

Returns:
    The unit vector of the visual axis.
*/
{
    const Vec3 kappa_vector = calculateKappaVector(alpha, beta);
    Vec3 optical_axis_vector = calculateOpticalAxisVector(optical_axis_unit);
    const Mat3x3 optical_axis_rotation_matrix = calculateOpticalAxisRotationMatrix(
        optical_axis_vector[0],
        optical_axis_vector[1],
        optical_axis_vector[2]
    );

    return normalized(mat3vec3_prod(optical_axis_rotation_matrix, kappa_vector));
}

inline Vec3
calculateKappaVector(double alpha, double beta)
/*
Calculate the kappa vector.

Args:
    alpha: delta pitch between the optic axis and the visual axis
    beta: delta yaw between the optic axis and the visual axis

Returns:
    The kappa vector.
*/
{
    return make_vec3(
        - std::sin(alpha) * std::cos(beta),
        std::sin(beta),
        std::cos(alpha) * std::cos(beta)
    );
}

inline Vec3
calculateOpticalAxisVector(const Vec3& optic_axis_unit)
/*
Calculate the optic axis vector.

Args:
    optic_axis_unit: the unit vector of the optic axis

Returns:
    The optic axis vector.
*/
{
    return make_vec3(
        - 1 * atan(optic_axis_unit[0] / optic_axis_unit[2]),
        asin(optic_axis_unit[1]),
        0
    );
}

inline Mat3x3
calculateOpticalAxisRotationMatrix(double theta, double phi, double kappa)
/*
Calculate the rotation matrix of the optical axis.

Args:
    theta: the x coordinate of optical axis vector
    phi: the y coordinate of optical axis vector
    kappa: the z coordinate of optical axis vector

Returns:
    The rotation matrix of the optical axis.
*/
{
    Mat3x3 Rflip, Rtheta, Rphi, Rkappa;

    Rflip << -1, 0, 0,
        0, 1, 0,
        0, 0, -1;

    Rtheta << std::cos(theta), 0, -std::sin(theta),
        0, 1, 0,
        std::sin(theta), 0, std::cos(theta);

    Rphi << 1, 0, 0,
        0, std::cos(phi), std::sin(phi),
        0, -std::sin(phi), std::cos(phi);

    Rkappa << std::cos(kappa), -std::sin(kappa), 0,
        std::sin(kappa), std::cos(kappa), 0,
        0, 0, 1;

    // return mat_prod(Rtheta, mat_prod(Rphi, Rkappa));
    return mat_prod(Rflip, mat_prod(Rtheta, mat_prod(Rphi, Rkappa)));
}

inline double
deg_to_rad(double a)
{
	return a * 3.141592653589793 / 180.;
}

inline double
rad_to_deg(double a)
{
	return (a * 180.) / 3.141592653589793;
}

inline bool
isGlintValid(const Vec2& glint)
{
    return glint[0] >= 0 && glint[1] >= 0;
}


}