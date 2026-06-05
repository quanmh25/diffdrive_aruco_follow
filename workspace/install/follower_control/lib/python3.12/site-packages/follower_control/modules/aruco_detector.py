import math

import cv2
import numpy as np


class ArucoMarkerDetector:
    def __init__(self, camera_matrix, dist_coeffs, dictionary_id=cv2.aruco.DICT_4X4_250):
        self.camera_matrix = np.array(camera_matrix, dtype=np.float32)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float32)
        if hasattr(cv2.aruco, 'Dictionary_get'):
            self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        else:
            self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.polygonalApproxAccuracyRate = 0.02
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxErroneousBitsInBorderRate = 0.08
        self.detector = None

    def detect_markers(self, image):
        corners, ids, _ = cv2.aruco.detectMarkers(
            image, self.dictionary, parameters=self.parameters
        )
        if ids is not None and len(ids) > 0:
            return corners, ids.flatten()
        return None, None

    def estimate_pose(self, corners, marker_size=0.1):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, self.camera_matrix, self.dist_coeffs
        )
        return rvecs, tvecs

    def get_marker_pose_2d(self, corners, marker_size=0.1):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, self.camera_matrix, self.dist_coeffs
        )
        if rvecs is None:
            return None

        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        return self.pose_2d_from_vectors(rvec, tvec)

    @staticmethod
    def pose_2d_from_vectors(rvec, tvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        yaw = math.atan2(rotation_matrix[0, 2], rotation_matrix[2, 2])
        return [float(tvec[0]), float(tvec[2]), yaw]
