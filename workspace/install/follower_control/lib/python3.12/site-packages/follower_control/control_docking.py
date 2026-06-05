import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math
import numpy as np
import cv2

from robot_controller.utils import xy_to_polar_coordinates, radian_normalization
from robot_controller.pd_controller import PDController
from robot_controller.kalman_filter import ArucoKalmanFilter
from robot_controller.aruco_detector import ArucoMarkerDetector



class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.pub_cmd = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.lidar = self.create_subscription(LaserScan, '/diff_drive/scan', self.lidar_cb, 10)
        self.camera = self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.pub_debug_img = self.create_publisher(Image, '/camera/image_debug', 10) # show rqt-view-image
        self.timer = self.create_timer(0.03, self.control_loop) #show TF of aruco
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel = TwistStamped()
        self.bridge = CvBridge()
        self.robot_velocity_input = np.zeros(2) # [v, w]

        # for lidar
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        #state machine
        self.state = "GO"
        self.avoid_dir = None
        self.mission = "FOLLOW_TRACK"
        self.docking_step = "DETECTION" 
        self.has_detected_marker = False # để hệ thống biết camera đã thực sự nhìn thấy ArUco hay chưa

        self.detected_count = 0
        self.max_linear_vel = 0.3
        self.max_angular_vel = 0.25
        
        self.stop_dist = 1.0  
        self.clear_dist = 1.4    
        self.marker_area = 0.0

        # Nhận diện đường line trắng
        self.white_hole = False
        self.hole_position = "CENTER"
        self.min_hole = 300 
        self.roi_height_ratio = 0.3 

        camera_matrix = [[623.5, 0.0, 360.0], [0.0, 623.5, 320.0], [0.0, 0.0, 1.0]]
        dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.aruco_detector = ArucoMarkerDetector(camera_matrix, dist_coeffs)
        self.pd_controller = PDController(kp=[0.25, 0.25, 0.25], kd=[0.01, 0.01, 0.01])
        # self.pd_controller = PDController(kp=[1, 1, 1], kd=[0.05, 0.05, 0.05])
        

        init_cov = [0.01, 0.0, 0.0,  0.0, 0.01, 0.0,  0.0, 0.0, 0.005]
        pred_cov = [0.005, 0.0, 0.0,  0.0, 0.005, 0.0,  0.0, 0.0, 0.001]
        meas_cov = [0.005, 0.0, 0.0,  0.0, 0.005, 0.0,  0.0, 0.0, 0.18]
        self.kalman_filter = ArucoKalmanFilter([0.0, 0.0, 0.0], init_cov, pred_cov, meas_cov)


    def lidar_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        for i in range(len(ranges)):
            if not np.isfinite(ranges[i]) or ranges[i] < 0.02:
                ranges[i] = np.nan

        def sector(angle_start, angle_end):
            a_start = math.radians(angle_start)
            a_end = math.radians(angle_end)
            i_start = int((a_start - msg.angle_min) / msg.angle_increment) 
            i_end = int((a_end - msg.angle_min) / msg.angle_increment)
            i_start = max(0, min(len(ranges) - 1, i_start))
            i_end = max(0, min(len(ranges) - 1, i_end))
            if i_start > i_end:
                i_start, i_end = i_end, i_start 
            sector_slice = ranges[i_start:i_end+1]
            if np.all(np.isnan(sector_slice)):
                return float('inf')
            return float(np.nanmin(sector_slice))
           
        self.front_min = sector(-20, 20)        
        self.left_min = sector(45, 90)
        self.right_min = sector(-90, -45)


    def camera_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
        self.process_white_path(cv_image)
        self.process_aruco_markers(cv_image)


    def process_white_path(self, cv_image):
        height, width = cv_image.shape[:2]
        roi_y_start = int(height * (1 - self.roi_height_ratio))
        roi = cv_image[roi_y_start:height, 0:width]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        adaptive_mask = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2  
        )

        largest_hole = None
        largest_area = 0
        contours, _ = cv2.findContours(adaptive_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max(self.min_hole, largest_area):
                largest_area = area
                largest_hole = contour

        if largest_hole is not None:
            self.white_hole = True
            M = cv2.moments(largest_hole)
            if M['m00'] > 0:
                hole_cx = int(M['m10'] / M['m00'])
                if hole_cx < width/3:
                    self.hole_position = "LEFT"
                elif hole_cx > 2/3*width:
                    self.hole_position = "RIGHT"
                else:
                    self.hole_position = "CENTER"
        else:
            self.white_hole = False
            self.hole_position = "CENTER"


    def process_aruco_markers(self, cv_image):
        corners, ids = self.aruco_detector.detect_markers(cv_image)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids.reshape(-1, 1)) # Draw a green square frame
            
            if 3 in ids:
                idx = np.where(ids == 3)[0][0]
                marker_corners = corners[idx][0]
                self.marker_area = cv2.contourArea(marker_corners)                
                self.mission = "PREPARE_TO_TURN" if self.marker_area < 8500 else "EXECUTE_TURN"

            if 23 in ids:
                if self.mission in ["FOLLOW_TRACK", "EXECUTE_TURN"]:
                    self.mission = "DOCKING_23"
                    self.docking_step = "DETECTION"
                    self.detected_count = 0
                    self.get_logger().info("Chuyển sang chế độ DOCKING_23")

                idx = np.where(ids == 23)[0][0]
                rvecs, tvecs = self.aruco_detector.estimate_pose([corners[idx]])
                tvec = tvecs[0][0]
                rvec = rvecs[0][0]

                self.detected_count += 1
                self.has_detected_marker = True 
                
                r_mat, _ = cv2.Rodrigues(rvec)
                yaw = math.atan2(r_mat[0, 2], r_mat[2, 2])
                yaw = radian_normalization(yaw * -1) 

                update_state = [tvec[2], -tvec[0], yaw] 
                self.kalman_filter.update(update_state) #Optimal state estimate

        # to show wwindow with image after processing
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub_debug_img.publish(debug_msg)


    def send_aruco_transform(self, x, y, yaw, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'  # Trục mốc gốc của camera
        t.child_frame_id = child_frame_id
        
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        
        # Công thức toán học chuyển đổi góc Yaw sang Quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)


    def get_predict_vector(self, dt):
        state_p = self.kalman_filter.get_state()
        r, delta = xy_to_polar_coordinates(state_p[0], state_p[1])
        
        linear = self.robot_velocity_input[0]
        angular = self.robot_velocity_input[1]
        yaw = state_p[2]

        if yaw < math.pi / 2: yaw += math.pi
        if yaw < 0 and yaw > -math.pi / 2: yaw -= math.pi

        new_delta = delta + angular * dt
        pred_x = (r * math.cos(new_delta) - state_p[0]) + (linear * dt)
        pred_y = (r * math.sin(new_delta) - state_p[1])
        pred_yaw = angular * dt

        return [pred_x, pred_y, pred_yaw]


    def control_loop(self):
        if not np.isfinite(self.front_min):
            return
        
        # field in TwistStamped
        self.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel.header.frame_id = 'base_link'

        # Autonomous wall-avoidance mode when no ArUco marker is detected
        if self.mission not in ["DOCKING_23", "DOCKED"]:
            if self.state == "GO":
                if self.front_min < self.stop_dist:  
                    self.state = "AVOID"
                    self.avoid_dir = "L" if self.left_min > 1.2 else "R" 
            elif self.state == "AVOID":
                if self.front_min > self.clear_dist:  
                    self.state = "GO"
                    self.avoid_dir = None

        # when robot saw marker id 23
        if self.mission == "DOCKING_23":
            if self.has_detected_marker:
                # Kalman filter
                u_predict = self.get_predict_vector(0.03) # dt=0.03
                self.kalman_filter.predict(u_predict)
                state = self.kalman_filter.get_state()
                tx, ty, tyaw = state[0], state[1], state[2]

                # create waypoint
                wp_x = tx + 0.6 * math.cos(tyaw)
                wp_y = ty + 0.6 * math.sin(tyaw)
                wp_yaw = tyaw

                # show tf of marker on Rviz
                self.send_aruco_transform(tx, ty, tyaw, "filter_aruco_link")
                self.send_aruco_transform(wp_x, wp_y, wp_yaw, "aruco_waypoint1")

                target_x, target_y, target_yaw = wp_x, wp_y, wp_yaw

                if self.docking_step in ["STATION", "COME_TO_STATION"]:
                    target_x, target_y, target_yaw = tx, ty, tyaw

                pd_input = [-target_x, -target_y, target_yaw]
                
                # SỬA LỖI 2: Ép xung dt = 30.0 theo đúng C++
                output = self.pd_controller.update(pd_input, 30.0)
                
                # SỬA LỖI 3: Dùng hệ tọa độ Polar cho Linear & Angular
                vel_r, vel_theta = xy_to_polar_coordinates(output[0], output[1])
                
                vel_linear = np.clip(vel_r, -self.max_linear_vel, self.max_linear_vel)
                vel_angular = np.clip(vel_theta, -self.max_angular_vel, self.max_angular_vel)

                if self.docking_step == "DETECTION":
                    if self.detected_count >= 120: 
                        self.docking_step = "WAYPOINT"
                        self.get_logger().info("Chuyển: DETECTION -> WAYPOINT")

                elif self.docking_step == "WAYPOINT":
                    self.robot_velocity_input[0] = 0.0
                    self.robot_velocity_input[1] = vel_angular
                    if abs(target_y) <= 0.01:
                        self.docking_step = "COME_TO_WAYPOINT"
                        self.get_logger().info("Chuyển: WAYPOINT -> COME_TO_WAYPOINT")

                elif self.docking_step == "COME_TO_WAYPOINT":
                    self.robot_velocity_input[0] = vel_linear
                    self.robot_velocity_input[1] = 0.0
                    if abs(target_x) <= 0.02:
                        self.docking_step = "STATION"
                        self.get_logger().info("Chuyển: COME_TO_WAYPOINT -> STATION")

                elif self.docking_step == "STATION":
                    self.robot_velocity_input[0] = 0.0
                    self.robot_velocity_input[1] = vel_angular
                    if abs(target_y) <= 0.02:
                        self.docking_step = "COME_TO_STATION"
                        self.get_logger().info("Chuyển: STATION -> COME_TO_STATION")

                elif self.docking_step == "COME_TO_STATION":
                    linear_finish = False
                    angular_finish = False
                    
                    self.robot_velocity_input[0] = vel_linear
                    self.robot_velocity_input[1] = vel_angular

                    if abs(target_x) <= 0.1:
                        linear_finish = True
                        self.robot_velocity_input[0] = 0.0

                    if abs(target_y) <= 0.02:
                        angular_finish = True
                        self.robot_velocity_input[1] = 0.0

                    if linear_finish and angular_finish:
                        self.docking_step = "END"
                        self.mission = "DOCKED"
                        self.get_logger().info("HOÀN THÀNH DOCKING!")
                else:
                    self.robot_velocity_input[0] = 0.0
                    self.robot_velocity_input[1] = 0.0

            self.cmd_vel.twist.linear.x = float(self.robot_velocity_input[0])
            self.cmd_vel.twist.angular.z = float(self.robot_velocity_input[1])

        elif self.mission == "DOCKED":
            self.cmd_vel.twist.linear.x = 0.0
            self.cmd_vel.twist.angular.z = 0.0

        elif self.state == "GO":
            if self.front_min > self.clear_dist and self.left_min > self.clear_dist:
                if self.mission == "PREPARE_TO_TURN":
                    self.cmd_vel.twist.linear.x = 0.5
                    self.cmd_vel.twist.angular.z = 0.0
                elif self.mission == "EXECUTE_TURN":
                    self.cmd_vel.twist.linear.x = 0.4
                    self.cmd_vel.twist.angular.z = 0.8
                else: 
                    self.cmd_vel.twist.linear.x = 0.0
                    self.cmd_vel.twist.angular.z = 1.54 
            else:
                self.cmd_vel.twist.linear.x = 0.5
                self.cmd_vel.twist.angular.z = 0.0
                
                if self.mission == "EXECUTE_TURN":
                    self.mission = "FOLLOW_TRACK"
                    self.get_logger().info('Đã rẽ xong, khôi phục bám đường!')

            if self.white_hole and self.mission == "FOLLOW_TRACK":
                if self.hole_position in ["LEFT", "CENTER"]:
                    self.cmd_vel.twist.linear.x = 0.0
                    self.cmd_vel.twist.angular.z = -0.923
                elif self.hole_position == "RIGHT":
                    self.cmd_vel.twist.linear.x = 0.0
                    self.cmd_vel.twist.angular.z = 0.923

        elif self.state == "AVOID" and self.mission not in ["DOCKING_23", "DOCKED"]:
            if self.avoid_dir == "L":
                self.cmd_vel.twist.linear.x = 0.0
                self.cmd_vel.twist.angular.z = 1.54  
            else: 
                self.cmd_vel.twist.linear.x = 0.0
                self.cmd_vel.twist.angular.z = -1.54
        
        self.pub_cmd.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()