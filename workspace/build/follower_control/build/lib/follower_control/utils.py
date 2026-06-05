import math
import numpy as np

def radian_normalization(radian):
    """Chuẩn hóa góc về khoảng [-PI, PI]"""
    while radian > math.pi:
        radian -= 2 * math.pi
    while radian < -math.pi:
        radian += 2 * math.pi
    return radian

def xy_to_polar_coordinates(x, y):
    """Chuyển đổi tọa độ Descartes sang tọa độ Cực (r, theta)"""
    length = math.hypot(x, y)
    angle = radian_normalization(math.atan2(y, x))
    return np.array([length, angle])