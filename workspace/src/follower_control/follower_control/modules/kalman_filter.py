import numpy as np
from follower_control.modules.utils import radian_normalization


class ArucoKalmanFilter:
    def __init__(self, initial_pose, initial_cov, predict_noise, measure_noise):
        self.X_ = np.array(initial_pose, dtype=float).reshape(3, 1)  # State [x, y, yaw]

        # Chuyển các mảng 1D (9 phần tử) thành ma trận 3x3
        self.P_ = np.array(initial_cov, dtype=float).reshape(3, 3)
        self.Q_ = np.array(predict_noise, dtype=float).reshape(3, 3)
        self.R_ = np.array(measure_noise, dtype=float).reshape(3, 3)

        self.F_ = np.eye(3)
        self.B_ = np.eye(3) * -1.0
        self.H_ = np.eye(3)

    def predict(self, u):
        u_vec = np.array(u, dtype=float).reshape(3, 1)
        self.X_ = self.F_ @ self.X_ + self.B_ @ u_vec
        self.P_ = self.F_ @ self.P_ @ self.F_.T + self.Q_
        self.X_[2, 0] = radian_normalization(self.X_[2, 0])

    def update(self, z):
        z_vec = np.array(z, dtype=float).reshape(3, 1)
        y = z_vec - self.H_ @ self.X_
        y[2, 0] = radian_normalization(y[2, 0])

        S = self.H_ @ self.P_ @ self.H_.T + self.R_
        K = self.P_ @ self.H_.T @ np.linalg.inv(S)

        self.X_ = self.X_ + K @ y
        self.X_[2, 0] = radian_normalization(self.X_[2, 0])

        I = np.eye(3)
        self.P_ = (I - K @ self.H_) @ self.P_

    def get_state(self):
        return self.X_.flatten()        # return array 1D [x, y, yaw]

    @classmethod
    def with_default_noise(cls, initial_pose):
        return cls(
            initial_pose,
            initial_cov=[0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.05],
            predict_noise=[0.002, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.005],
            measure_noise=[0.03, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.08],
        )
