import numpy as np

class PDController:
    def __init__(self, kp, kd):
        self.kp = np.array(kp, dtype=float)
        self.kd = np.array(kd, dtype=float)
        self.target = np.zeros(3)
        self.previous_error = np.zeros(3)

    def set_target(self, target):
        self.target = np.array(target, dtype=float)

    def update(self, measurement, dt):
        error = self.target - np.array(measurement, dtype=float)
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        
        # Tương đương kp_.cwiseProduct(error) + kd_.cwiseProduct(derivative) trong Eigen
        return self.kp * error + self.kd * derivative

    def set_kp(self, kp):
        self.kp = np.array(kp, dtype=float)

    def set_kd(self, kd):
        self.kd = np.array(kd, dtype=float)