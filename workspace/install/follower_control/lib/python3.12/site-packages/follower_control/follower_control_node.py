import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from follower_control.modules.aruco_detector import ArucoMarkerDetector
from follower_control.modules.kalman_filter import ArucoKalmanFilter
from follower_control.modules.pd_controller import PDController
from follower_control.modules.utils import normalize_angle, yaw_from_quaternion, clamp

import math
import cv2
import numpy as np



class FollowControl(Node):
    def __init__(self):
        super().__init__('follow_control')
        self.pub_cmd = self.create_publisher(TwistStamped, '/follower/diff_drive_controller/cmd_vel', 10)
        self.pub_debug_img = self.create_publisher(Image, '/camera/image_debug', 10)
        self.sub_image = self.create_subscription(Image, '/follower/camera/image_raw', self.image_callback, 10)


        self.marker_id = 42
        self.marker_size = 0.1
        self.use_kalman_filter = False
        self.desired_distance = 0.3
        self.path_goal_tolerance = 0.15

        self.max_linear_speed = 1.2
        self.max_angular_speed = 2.0

        self.use_path_memory = True
        self.world_frame = 'world'
        self.marker_path_frame = ''
        if not self.marker_path_frame:
            self.marker_path_frame = f'detected_aruco_{self.marker_id}'

        self.follower_frame = 'follower/camera_link'
        self.path_spacing = 0.03
        self.pure_pursuit_lookahead = 0.35
        self.slow_distance = 1.5
        self.catchup_distance = 2.0
        self.slow_linear_speed = 0.35
        self.cruise_linear_speed = 0.75
        self.recovery_linear_speed = 0.45
        self.marker_recovery_x = 0.18
        self.marker_recovery_yaw = 1.2
        self.min_tracking_speed = 0.04
        self.max_path_points = 3000

        fx = 1920.0 / (2.0 * math.tan(2.094 / 2.0))
        camera_matrix = [[fx, 0.0, 960], [0.0, fx, 540], [0.0, 0.0, 1.0]]
        dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.bridge = CvBridge()
        self.aruco_detector = ArucoMarkerDetector(camera_matrix, dist_coeffs)
        self.aruco_pd_controller = PDController(kp=[0.8, 1.8, 0.0], kd=[0.01, 0.01, 0.0])
        self.path_pd_controller = PDController(kp=[0.8, 1.8, 0.0], kd=[0.01, 0.01, 0.0])


        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.timer = self.create_timer(0.05, self.control_loop)

        self.last_detection_time = None
        self.marker_x = 0.0
        self.marker_z = 0.0
        self.marker_yaw = 0.0
        self.aruco_kalman_filter = None
        self.has_marker = False
        self.leader_path = []
        self.path_total_distance = 0.0
        self.last_control_time = self.get_clock().now()
  
        self.last_log_time = self.get_clock().now()


    def image_callback(self, msg):
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

                marker_pose = self.aruco_detector.pose_2d_from_vectors(rvec, tvec)

                if not self.use_kalman_filter:
                    filtered_pose = marker_pose
                else:
                    if self.aruco_kalman_filter is None:
                        self.aruco_kalman_filter = ArucoKalmanFilter.with_default_noise(marker_pose)
                        filtered_pose = marker_pose
                    else:
                        self.aruco_kalman_filter.predict([0.0, 0.0, 0.0])
                        self.aruco_kalman_filter.update(marker_pose)
                        filtered_pose = self.aruco_kalman_filter.get_state().tolist()

                self.marker_x, self.marker_z, self.marker_yaw = filtered_pose


                self.last_detection_time = self.get_clock().now()
                self.has_marker = True

                self.publish_marker_tf()

                # draw coordinate 3D (x, y, z) on image_debug
                cv2.drawFrameAxes(
                    image,
                    self.aruco_detector.camera_matrix,
                    self.aruco_detector.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size,
                )

        # we can see it on rqt_view_image
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug_msg.header = msg.header
        self.pub_debug_img.publish(debug_msg)


    def publish_marker_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'follower/camera_link'
        transform.child_frame_id = f'detected_aruco_{self.marker_id}'

        transform.transform.translation.x = self.marker_z
        transform.transform.translation.y = -self.marker_x
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.marker_yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.marker_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)


    def marker_is_fresh(self):
        if self.last_detection_time is None:
            return False
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds * 1e-9
        return age <= 0.6


    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'follower/base_link'
        
        now = self.get_clock().now()
        dt = max((now - self.last_control_time).nanoseconds * 1e-9, 1e-3)
        self.last_control_time = now

        marker_fresh = self.marker_is_fresh() #True or False

        if marker_fresh:
            if self.marker_z <= self.desired_distance:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.get_logger().info(f"Aruco stop distance reached: z = {self.marker_z:.2f}m", throttle_duration_sec=2.0)
                self.pub_cmd.publish(cmd)
                return

            if self.use_path_memory:
                marker_pose = self.lookup_world_pose(self.marker_path_frame)
                if marker_pose is None:
                    self.get_logger().info(f'waiting for marker world pose')
            
                else:
                    x, y, yaw = marker_pose
                    if not self.leader_path:
                        self.leader_path.append((x, y, yaw, 0.0))
                    else:
                        last_x, last_y, _, _ = self.leader_path[-1]
                        segment = math.hypot(x - last_x, y - last_y)

                        if segment >= self.path_spacing:
                            self.path_total_distance += segment
                            self.leader_path.append((x, y, yaw, self.path_total_distance))

                            if len(self.leader_path) > self.max_path_points:
                                self.leader_path.pop(0)

            # use aruco_control when camera still see arruco
            self.aruco_control(cmd, dt)

        elif self.use_path_memory and self.path_memory_control(cmd, dt):
            pass
            
        else:
            self.has_marker = False

        self.pub_cmd.publish(cmd)


    def aruco_control(self, cmd, dt):
        distance_error = self.marker_z - self.desired_distance
        linear_x = 0.8 * distance_error
        angular_z = -1.8 * self.marker_x

        cmd.twist.linear.x = clamp(linear_x, 0.0, self.max_linear_speed)
        cmd.twist.angular.z = clamp(angular_z, -self.max_angular_speed, self.max_angular_speed,)


    def path_memory_control(self, cmd, dt):
        follower_pose = self.lookup_world_pose(self.follower_frame)
        if follower_pose is None:
            self.get_logger().info('waiting for follower world pose')
            return False

        if not self.leader_path:
            self.get_logger().info('waiting for first ArUco waypoint')
            return False

        x, y, _ = marker_pose
        nearest = min(
            self.leader_path,
            key=lambda point: math.hypot(point[0] - x, point[1] - y),
        )

        allowed_s = max(0.0, self.path_total_distance - self.desired_distance)
        target_s = min(nearest_s + self.pure_pursuit_lookahead, allowed_s)
        target = self.interpolate_path(target_s)

        if target is None:
            self.get_logger().info('waiting for enough ArUco path')
            return False

        dx = target[0] - follower_pose[0]
        dy = target[1] - follower_pose[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - follower_pose[2])

        if ((target_s >= allowed_s - 1e-6) and distance <= self.path_goal_tolerance):
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().info(f'path stop reached: dist={distance:.2f}m, target_s={target_s:.2f}m')
            self.publish_path_target_tf(target)
            return True

        if abs(heading_error) < math.radians(80.0):
            linear_measurement = -distance * max(0.0, math.cos(heading_error))
        else:
            linear_measurement = 0.0

        control = self.path_pd_controller.update([linear_measurement, -heading_error, 0.0], dt)

# Gọi ĐÚNG 1 HÀM để lấy cả 2 giới hạn tốc độ
        speed_limit, angular_limit = self.calculate_speed_limits(
            marker_fresh=self.marker_is_fresh(),
            nearest_s=nearest_s,
            allowed_s=allowed_s
        )
        cmd.twist.linear.x = clamp(control[0], 0.0, speed_limit)
        cmd.twist.angular.z = clamp(control[1], -angular_limit, angular_limit)
        self.publish_path_target_tf(target)

        return True


    def lookup_world_pose(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, frame_id, Time())
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation),
        )


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


    def stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'follower/base_link'
        self.pub_cmd.publish(cmd)


    def calculate_speed_limits(self, marker_fresh, nearest_s=0.0, allowed_s=0.0):
        """
        Tính toán giới hạn tốc độ dựa trên trạng thái (thấy ArUco hay đang đi mù)
        """
        # 1. Đánh giá xem xe có bị lệch quá mức và cần ưu tiên "phục hồi" (recovery) không
        needs_recovery = False
        if marker_fresh:
            needs_recovery = (abs(self.marker_x) >= self.marker_recovery_x or 
                              abs(self.marker_yaw) >= self.marker_recovery_yaw)

        # 2. TÍNH GIỚI HẠN TỐC ĐỘ XOAY (Angular Speed)
        angular_limit = self.max_angular_speed
        if marker_fresh and not needs_recovery:
            if self.marker_z < self.slow_distance:
                angular_limit = 0.9
            elif self.marker_z < self.catchup_distance:
                # Nội suy mượt mà từ 0.9 đến 1.5
                ratio = (self.marker_z - self.slow_distance) / (self.catchup_distance - self.slow_distance)
                ratio = clamp(ratio, 0.0, 1.0)
                angular_limit = 0.9 + (1.5 - 0.9) * ratio

        # 3. TÍNH GIỚI HẠN TỐC ĐỘ TIẾN (Linear Speed)
        linear_limit = 0.0
        if marker_fresh:
            # --- Nhánh A: Nhìn thấy mục tiêu ---
            if self.marker_z <= self.desired_distance:
                base_speed = 0.0
            elif self.marker_z < self.slow_distance:
                ratio = (self.marker_z - self.desired_distance) / (self.slow_distance - self.desired_distance)
                ratio = clamp(ratio, 0.0, 1.0)
                base_speed = self.min_tracking_speed + (self.slow_linear_speed - self.min_tracking_speed) * ratio
            elif self.marker_z < self.catchup_distance:
                ratio = (self.marker_z - self.slow_distance) / (self.catchup_distance - self.slow_distance)
                ratio = clamp(ratio, 0.0, 1.0)
                base_speed = self.slow_linear_speed + (self.cruise_linear_speed - self.slow_linear_speed) * ratio
            else:
                base_speed = self.max_linear_speed

            # Áp dụng kìm ga nếu cần phục hồi góc lệch
            if needs_recovery and self.marker_z > self.desired_distance:
                if self.marker_z < 1.0:
                    linear_limit = max(base_speed, self.slow_linear_speed)
                else:
                    linear_limit = max(base_speed, self.recovery_linear_speed)
            else:
                linear_limit = base_speed

        else:
            # --- Nhánh B: Đi theo quỹ đạo mù ---
            remaining = max(0.0, allowed_s - nearest_s)
            if remaining <= 0.0:
                linear_limit = 0.0
            else:
                ratio = clamp(remaining / self.slow_distance, 0.0, 1.0)
                linear_limit = clamp(self.max_linear_speed * ratio, self.min_tracking_speed, self.max_linear_speed)

        return linear_limit, angular_limit

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
