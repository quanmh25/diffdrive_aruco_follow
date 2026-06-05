import math
import numpy as np
from geometry_msgs.msg import TransformStamped


def radian_normalization(radian):
    """Chuẩn hóa góc về khoảng [-PI, PI]"""
    while radian > math.pi:
        radian -= 2 * math.pi
    while radian < -math.pi:
        radian += 2 * math.pi
    return radian


def normalize_angle(angle):
    return radian_normalization(angle)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def xy_to_polar_coordinates(x, y):
    """Chuyển đổi tọa độ Descartes sang tọa độ Cực (r, theta)"""
    length = math.hypot(x, y)
    angle = radian_normalization(math.atan2(y, x))
    return np.array([length, angle])


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def interpolate(start, end, ratio):
    ratio = clamp(ratio, 0.0, 1.0)
    return start + (end - start) * ratio


def make_transform_2d(stamp, parent_frame, child_frame, x, y, yaw):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(x)
    transform.transform.translation.y = float(y)
    transform.transform.translation.z = 0.0
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform
