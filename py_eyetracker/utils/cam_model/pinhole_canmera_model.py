import numpy as np
import numpy.typing as npt

class PinholeCameraModel:
    def __init__(
            self,
            principal_point_x: np.float32=0,
            principal_point_y: np.float32=0,
            pixel_size_x_cm: np.float32=0,
            pixel_size_y_cm: np.float32=0,
            effective_focal_length_cm: np.float32=0,
            camera_angles: npt.NDArray[np.float32]=np.array([0, 0, 0]),
            position: npt.NDArray[np.float32]=np.array([0, 0, 0])
    ):
        self.principal_point_x = principal_point_x
        self.principal_point_y = principal_point_y
        self.pixel_size_x_cm = pixel_size_x_cm
        self.pixel_size_y_cm = pixel_size_y_cm
        self.effective_focal_length_cm = effective_focal_length_cm
        self.camera_angles = camera_angles
        self.position = position
        self.rotation_matrix = self.calculate_extrinsic_rotation_matrix()

    def calculate_extrinsic_rotation_matrix(self) -> npt.NDArray[np.float32]:
        """
        Calculates the extrinsic rotation matrix of the camera.
        :return: The extrinsic rotation matrix of the camera.
        """
        # Calculate the rotation matrix of the camera
        rotation_matrix = np.array([
            [np.cos(self.camera_angles[2]), -np.sin(self.camera_angles[2]), 0],
            [np.sin(self.camera_angles[2]), np.cos(self.camera_angles[2]), 0],
            [0, 0, 1]
        ])
        return rotation_matrix
    
    def ics_to_ccs(self, ics: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        """
        Converts the image coordinates system (ICS) to the camera coordinates system (CCS).
        :param ics: The image coordinates system (ICS) coordinates.
        :return: The camera coordinates system (CCS) coordinates.
        """
        return np.array([
            ics[0] - self.principal_point_x,
            ics[1] - self.principal_point_y,
            - self.effective_focal_length_cm
        ])
    
    def ccs_to_wcs(self, ccs: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        """
        Converts the camera coordinates system (CCS) to the world coordinates system (WCS).
        :param ccs: The camera coordinates system (CCS) coordinates.
        :return: The world coordinates system (WCS) coordinates.
        """
        return np.dot(self.rotation_matrix, ccs) + self.position
    
    def ics_to_wcs(self, ics: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        """
        Converts the image coordinates system (ICS) to the world coordinates system (WCS).
        :param ics: The image coordinates system (ICS) coordinates.
        :return: The world coordinates system (WCS) coordinates.
        """
        ccs = self.ics_to_ccs(ics)
        wcs = self.ccs_to_wcs(ccs)
        return wcs
    