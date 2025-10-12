#!/usr/bin/env python3
# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from std_srvs.srv import Empty


class CarControlNode:
    def __init__(self):
        rospy.init_node("vehicle_node")

        rospy.wait_for_service("/gazebo/reset_world")
        reset_sim = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        reset_sim()

        # Parameters
        self.freq = rospy.get_param("~control_frequency", 50)

        # Subscribers and Publishers
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.rate = rospy.Rate(self.freq)

    def odom_callback(self, msg):
        # Convert odometry to "localization"
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        rospy.loginfo(
            "Robot pose: x=%.2f y=%.2f theta=%.2f", position.x, position.y, yaw
        )

    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()
            # Example car-like forward command
            cmd.linear.x = 0.5  # throttle
            cmd.angular.z = 0.1  # steering angle
            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    node = CarControlNode()
    node.run()
