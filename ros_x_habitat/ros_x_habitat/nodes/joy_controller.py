#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyToHabitat(Node):
    def __init__(self):
        super().__init__('joy_to_habitat')
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription_ = self.create_subscription(
            Joy, 'joy', self.listener_callback, 10)

    def listener_callback(self, data: Joy) -> None:
        # to speed up
        vel_linear_scale_factor = 2.0
        vel_angular_scale_factor = 2.0

        # negative sign in vel_z because agent eyes look at negative z axis
        vel_max = 0.3  # m/s
        vel_z = 4 * data.axes[1] * vel_max

        # negative sign because pushing right produces
        # negative number on joystick
        vel_x = -4 * data.axes[0] * vel_max
        yaw = data.axes[3] * 30 / 180 * 3.1415926
        pitch = data.axes[4] * 30

        # h = std_msgs.msg.Header()
        # h.stamp = rospy.Time.now()
        vel_msg = Twist()
        vel_msg.linear.x = float(vel_linear_scale_factor) * vel_z
        vel_msg.linear.y = float(vel_linear_scale_factor) * vel_x
        vel_msg.linear.z = float(0)
        vel_msg.angular.x = float(0)
        vel_msg.angular.y = float(vel_angular_scale_factor) * pitch
        vel_msg.angular.z = float(vel_angular_scale_factor) * yaw
        # vel_msg.header = h

        self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    joy_to_habitat = JoyToHabitat()

    rclpy.spin(joy_to_habitat)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_to_habitat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
