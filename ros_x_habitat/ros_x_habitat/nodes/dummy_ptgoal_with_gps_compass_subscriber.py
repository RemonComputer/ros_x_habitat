#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from ros_x_habitat_interfaces.msg import PointGoalWithGPSCompass


# def callback(data):
#     # rospy.loginfo(f"dist_to_goal: {data.distance_to_goal}, angle_to_goal: {data.angle_to_goal}")
#     pass


# def listener():
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node("ptgoal_with_gps_compass_dummy_subscriber", anonymous=True)

#     rospy.Subscriber("pointgoal_with_gps_compass", PointGoalWithGPSCompass, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()


class PTGoalWithGPSCompassDummySubscriber(Node):
    def __init__(self) -> None:
        super().__init__('ptgoal_with_gps_compass_dummy_subscriber')
        self.subscription_ = self.create_subscription(
            PointGoalWithGPSCompass, 'pointgoal_with_gps_compass',
            self.listener_callback, 10)

    def listener_callback(self, data: PointGoalWithGPSCompass) -> None:
        self.get_logger().info(
            f"dist_to_goal: {data.distance_to_goal}, "
            f"angle_to_goal: {data.angle_to_goal}")
        pass


def main(args=None):
    rclpy.init(args=args)

    ptgoal_with_gps_compass_dummy_subscriber = \
        PTGoalWithGPSCompassDummySubscriber()

    rclpy.spin(ptgoal_with_gps_compass_dummy_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ptgoal_with_gps_compass_dummy_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # listener()
    main()
