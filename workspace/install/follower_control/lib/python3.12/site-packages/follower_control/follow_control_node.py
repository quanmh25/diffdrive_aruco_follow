import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from follower_control.aruco_detector import ArucoMarkerDetector


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FollowControl(Node):
    def __init__(self):
        super().__init__('follow_control')

        self.declare_parameter(
            'cmd_vel_topic',
            '/follower/diff_drive_controller/cmd_vel',
        )
        self.declare_parameter('image_topic', '/follower/camera/image_raw')
        self.declare_parameter('debug_image_topic', '/follower/camera/image_debug')
        self.declare_parameter('marker_id', 42)
        self.declare_parameter('marker_size', 0.1)
        self.declare_parameter('desired_distance', 0.9)
        self.declare_parameter('linear_kp', 0.45)
        self.declare_parameter('angular_kp', 1.4)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('lost_timeout', 0.6)
        self.declare_parameter('use_path_memory', True)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('leader_frame', 'leader/base_link')
        self.declare_parameter('follower_frame', 'follower/base_link')
        self.declare_parameter('path_spacing', 0.03)
        self.declare_parameter('pure_pursuit_lookahead', 0.35)
        self.declare_parameter('max_path_points', 3000)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.desired_distance = float(
            self.get_parameter('desired_distance').value
        )
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.use_path_memory = bool(self.get_parameter('use_path_memory').value)
        self.world_frame = self.get_parameter('world_frame').value
        self.leader_frame = self.get_parameter('leader_frame').value
        self.follower_frame = self.get_parameter('follower_frame').value
        self.path_spacing = float(self.get_parameter('path_spacing').value)
        self.pure_pursuit_lookahead = float(
            self.get_parameter('pure_pursuit_lookahead').value
        )
        self.max_path_points = int(self.get_parameter('max_path_points').value)

        # Camera in follower_robot.xacro: 720x640, horizontal FOV 1.089 rad.
        fx = 720.0 / (2.0 * math.tan(1.089 / 2.0))
        camera_matrix = [
            [fx, 0.0, 360.0],
            [0.0, fx, 320.0],
            [0.0, 0.0, 1.0],
        ]
        dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.bridge = CvBridge()
        self.aruco_detector = ArucoMarkerDetector(camera_matrix, dist_coeffs)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.pub_debug_img = self.create_publisher(
            Image, self.debug_image_topic, 10
        )
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_cb, 10
        )
        self.timer = self.create_timer(0.05, self.control_loop)

        self.last_detection_time = None
        self.marker_x = 0.0
        self.marker_z = 0.0
        self.marker_yaw = 0.0
        self.has_marker = False
        self.leader_path = []
        self.path_total_distance = 0.0
        self.last_control_status = 'waiting for leader path'
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            'Follower control started: '
            f'marker_id={self.marker_id}, desired_distance={self.desired_distance:.2f}m'
        )
        self.get_logger().info(
            f'Subscribing {self.image_topic}, publishing {self.cmd_vel_topic}'
        )
        if self.use_path_memory:
            self.get_logger().info(
                'Path memory mode: '
                f'{self.world_frame} -> {self.leader_frame}, '
                f'{self.follower_frame}, spacing={self.path_spacing:.2f}m'
            )

    def image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Cannot convert camera image: {exc}')
            return

        corners, ids = self.aruco_detector.detect_markers(image)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids.reshape(-1, 1))
            matches = np.where(ids == self.marker_id)[0]
            if len(matches) > 0:
                idx = int(matches[0])
                rvecs, tvecs = self.aruco_detector.estimate_pose(
                    [corners[idx]], self.marker_size
                )
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                self.marker_x = float(tvec[0])
                self.marker_z = float(tvec[2])
                self.marker_yaw = self.get_marker_yaw(rvec)
                self.last_detection_time = self.get_clock().now()
                self.has_marker = True

                self.publish_marker_tf()
                cv2.drawFrameAxes(
                    image,
                    self.aruco_detector.camera_matrix,
                    self.aruco_detector.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size,
                )

        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug_msg.header = msg.header
        self.pub_debug_img.publish(debug_msg)

    def get_marker_yaw(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return math.atan2(rotation_matrix[0, 2], rotation_matrix[2, 2])

    def publish_marker_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'follower/camera_link'
        transform.child_frame_id = f'detected_aruco_{self.marker_id}'

        # OpenCV camera: x right, y down, z forward.
        # ROS base view here uses x forward, y left, z up.
        transform.transform.translation.x = self.marker_z
        transform.transform.translation.y = -self.marker_x
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.marker_yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.marker_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def marker_is_fresh(self):
        if self.last_detection_time is None:
            return False
        age = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds * 1e-9
        return age <= self.lost_timeout

    def control_loop(self):
        cmd = self.make_cmd()

        if self.use_path_memory and self.path_memory_control(cmd):
            pass
        elif self.marker_is_fresh():
            distance_error = self.marker_z - self.desired_distance
            cmd.twist.linear.x = clamp(
                self.linear_kp * distance_error,
                -self.max_linear_speed,
                self.max_linear_speed,
            )

            # Marker x is positive on image/camera right; ROS positive angular.z
            # turns left, so the sign is negative to steer toward the marker.
            cmd.twist.angular.z = clamp(
                -self.angular_kp * self.marker_x,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.last_control_status = (
                f'aruco z={self.marker_z:.2f}m x={self.marker_x:.2f}m'
            )
        else:
            self.has_marker = False
            self.last_control_status = 'waiting for fresh ArUco or TF path'

        self.pub_cmd.publish(cmd)
        self.log_status(cmd)

    def path_memory_control(self, cmd):
        leader_pose = self.lookup_world_pose(self.leader_frame)
        follower_pose = self.lookup_world_pose(self.follower_frame)
        if leader_pose is None or follower_pose is None:
            self.last_control_status = 'waiting for TF world poses'
            return False

        self.record_leader_pose(leader_pose)
        if not self.leader_path:
            self.last_control_status = 'waiting for first leader waypoint'
            return False

        nearest_s = self.find_nearest_path_s(follower_pose)
        allowed_s = max(0.0, self.path_total_distance - self.desired_distance)
        target_s = min(nearest_s + self.pure_pursuit_lookahead, allowed_s)
        target = self.interpolate_path(target_s)
        if target is None:
            self.last_control_status = 'waiting for enough leader path'
            return False

        dx = target[0] - follower_pose[0]
        dy = target[1] - follower_pose[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - follower_pose[2])

        if abs(heading_error) < math.radians(80.0):
            linear = self.linear_kp * distance * max(0.0, math.cos(heading_error))
        else:
            linear = 0.0

        cmd.twist.linear.x = clamp(linear, 0.0, self.max_linear_speed)
        cmd.twist.angular.z = clamp(
            self.angular_kp * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        self.publish_path_target_tf(target)
        self.last_control_status = (
            f'path={self.path_total_distance:.2f}m '
            f'target_s={target_s:.2f}m dist={distance:.2f}m '
            f'heading={math.degrees(heading_error):.1f}deg'
        )
        return True

    def lookup_world_pose(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                frame_id,
                Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation),
        )

    def record_leader_pose(self, pose):
        x, y, yaw = pose
        if not self.leader_path:
            self.leader_path.append((x, y, yaw, 0.0))
            return

        last_x, last_y, _, _ = self.leader_path[-1]
        segment = math.hypot(x - last_x, y - last_y)
        if segment < self.path_spacing:
            return

        self.path_total_distance += segment
        self.leader_path.append((x, y, yaw, self.path_total_distance))
        if len(self.leader_path) > self.max_path_points:
            self.leader_path.pop(0)

    def find_nearest_path_s(self, pose):
        x, y, _ = pose
        nearest = min(
            self.leader_path,
            key=lambda point: math.hypot(point[0] - x, point[1] - y),
        )
        return nearest[3]

    def interpolate_path(self, target_s):
        if not self.leader_path:
            return None
        if target_s <= self.leader_path[0][3]:
            return self.leader_path[0]

        for previous, current in zip(self.leader_path, self.leader_path[1:]):
            if current[3] < target_s:
                continue

            span = current[3] - previous[3]
            if span <= 1e-6:
                return current

            ratio = (target_s - previous[3]) / span
            x = previous[0] + ratio * (current[0] - previous[0])
            y = previous[1] + ratio * (current[1] - previous[1])
            yaw = previous[2] + ratio * normalize_angle(current[2] - previous[2])
            return (x, y, normalize_angle(yaw), target_s)

        return self.leader_path[-1]

    def publish_path_target_tf(self, target):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = 'path_memory_target'
        transform.transform.translation.x = float(target[0])
        transform.transform.translation.y = float(target[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(target[2] / 2.0)
        transform.transform.rotation.w = math.cos(target[2] / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def log_status(self, cmd):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 < 1.0:
            return
        self.last_log_time = now

        if self.has_marker and self.marker_is_fresh():
            self.get_logger().info(
                f'ArUco {self.marker_id}: z={self.marker_z:.2f}m, '
                f'x={self.marker_x:.2f}m -> v={cmd.twist.linear.x:.2f}, '
                f'w={cmd.twist.angular.z:.2f}; {self.last_control_status}'
            )
        else:
            self.get_logger().info(
                f'Follower cmd v={cmd.twist.linear.x:.2f}, '
                f'w={cmd.twist.angular.z:.2f}; {self.last_control_status}'
            )

    def stop(self):
        self.pub_cmd.publish(self.make_cmd())

    def make_cmd(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'follower/base_link'
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = FollowControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
