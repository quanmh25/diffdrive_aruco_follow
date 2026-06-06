import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node



class SimpleControl(Node):
    def __init__(self):
        super().__init__('simple_control')
        self.pub_cmd = self.create_publisher(TwistStamped, '/leader/diff_drive_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.start_time = self.get_clock().now()
        self.step_index = 0

        self.steps = [
            ('STOP_START', 5.0, 0.0, 0.0),
            ('PULL_AWAY', 6.0, 0.5, 0.0),
            ('SLOW_LEFT', 7.0, 0.25, 0.18),
            ('STRAIGHT_AFTER_LEFT', 5.0, 0.35, 0.0),
            ('TURN_RIGHT', 6.0, 0.3, -0.15),
            ('STRAIGHT_FINAL', 6.0, 0.35, 0.0),
            ('STOP_END', 5.0, 0.0, 0.0),
        ]


    def timer_cb(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        while ((self.step_index < len(self.steps) - 1) and elapsed >= self.steps[self.step_index][1]):
            elapsed -= self.steps[self.step_index][1]
            self.step_index += 1
            self.start_time = now - Duration(seconds=elapsed)

        _, _, linear_x, angular_z = self.steps[self.step_index]
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'leader/base_link'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.pub_cmd.publish(cmd)


    def stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'leader/base_link'
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.get_logger().info('Stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
