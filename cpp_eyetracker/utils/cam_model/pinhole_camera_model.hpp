// cmake -B build -S . -G "Visual Studio 17 2022"
// cmake --build build --config Release

#include "math_types.hpp"

namespace gazeestimation {

    class PinholeCameraModel {
    /*
    This class can transfer a point in ICS to WCS.
    
    Attributes:
        intrinsic parameters:
            principal_point_x: (double) the x coordinate of principal point in pixel.
            principal_point_y: (double) the y coordinate of principal point in pixel.
            pixel_size_cm_x: (double) the x pixel size in cm.
            pixel_size_cm_y: (double) the y pixel size in cm.
            effective_focal_length_cm: (double) the effective focal length of the camera.
        extrinsic parameters:
            camera_angles: (Vec3) the rotation vector of the camera.
            position: (Vec3) the translation vector of the camera.

    Fuctions:
        ics_to_wcs: transfer a vector in ICS to WCS.
    */
        private:
        Vec3 camera_angles;
        Mat3x3 actual_rotation_matrix;

        public:
        // camera intrinsic parameters
        double principal_point_x;
        double principal_point_y;
        double pixel_size_cm_x;
        double pixel_size_cm_y;
        double effective_focal_length_cm;

        // position in WCS
        Vec3 position;

        PinholeCameraModel():
        camera_angles(make_vec3(0, 0, 0)),
        actual_rotation_matrix(identity_matrix3x3()),
        principal_point_x(0),
        principal_point_y(0),
        pixel_size_cm_x(0),
        pixel_size_cm_y(0),
        effective_focal_length_cm(0) { }

        void set_camera_angles(double x, double y, double z) {
            camera_angles = make_vec3(x, y, z);
            actual_rotation_matrix = calculate_extrinsic_rotation_matrix(
                camera_angles[0],
                camera_angles[1],
                camera_angles[2]
            );
        }

        // returns the rotation matrix of the camera
        Mat3x3 rotation_matrix() const {
            return actual_rotation_matrix;
        }

        // transfer the given vector in ICS to CCS
        Vec3 ics_to_ccs(const Vec2& position_ics) const {
            return make_vec3(
                (position_ics[0] - principal_point_x) * pixel_size_cm_x,
                (position_ics[1] - principal_point_y) * pixel_size_cm_y,
                - effective_focal_length_cm
            );
        }

        // transfer the given vector in CCS to WCS
        Vec3 ccs_to_wcs(const Vec3& position_ccs) const {
            return rotation_matrix() * position_ccs + position;
        }

        // transfer the given vector in ICS to WCS
        Vec3 ics_to_wcs(const Vec2& position_ics) const {
            return ccs_to_wcs(ics_to_ccs(position_ics));
        }

        double camera_angle_x() const {
            return camera_angles[0];
        }

        double camera_angle_y() const {
            return camera_angles[1];
        }

        double camera_angle_z() const {
            return camera_angles[2];
        }

        void set_camera_angle_x(double x) {
            set_camera_angles(x, camera_angles[1], camera_angles[2]);
        }

        void set_camera_angle_y(double y) {
            set_camera_angles(camera_angles[0], y, camera_angles[2]);
        }

        void set_camera_angle_z(double z) {
            set_camera_angles(camera_angles[0], camera_angles[1], z);
        }
    }; // class PinholeCameraModel

} // namespace gazeestimation

