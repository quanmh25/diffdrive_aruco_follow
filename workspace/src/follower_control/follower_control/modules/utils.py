import math


def radian_normalization(radian):
    while radian > math.pi:
        radian -= 2 * math.pi
    while radian < -math.pi:
        radian += 2 * math.pi
    return radian


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

