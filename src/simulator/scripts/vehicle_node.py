#!/usr/bin/env python3
# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import numpy as np
from simulator.msg import Path2D, PathPoint
from simulator.srv import MPCService, MPCServiceRequest

import os
import sys

# TODO remove this hack
sys.path.append(os.path.join(os.path.dirname(__file__), "../../trajectory/scripts"))
from trajectory_handler import TrajectoryHandler, Trajectory
from gazebo_utils import gazebo_reset_robot, RvizPathVisualizer


class CarControlNode:
    def __init__(self):
        rospy.init_node("vehicle_node")
        self.freq = rospy.get_param("~control_frequency", 10)
        self.rate = rospy.Rate(self.freq)

        self.velocity = 0.05
        self.path_handler = TrajectoryHandler(
            "/home/smummaneni/hobby/smooth-operator/trajectory_data/waypoints/b.csv"
        )

        # Restart Gazebo and place robot at start of trajectory
        start = self.path_handler.get_start_point()
        gazebo_reset_robot(start)

        self.client = rospy.ServiceProxy("mpc_control", MPCService)
        self.client.wait_for_service()

        # Visualize full trajectory in RViz
        self.viz = RvizPathVisualizer()
        full_path = self.path_handler.get_full_trajectory(ds=0.5)
        self.viz.visualize_global_path(full_path.x, full_path.y)

        # Subscribers and Publishers
        self.state = None  # [x, y, yaw, velocity, yaw_rate]
        self.odom = None
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    @staticmethod
    def trajectory_to_ros_msg(trajectory: Trajectory, velocity: float) -> Path2D:
        """
        Converts a Trajectory object to a ROS Trajectory2D message.
        Args:
            trajectory (Trajectory): Trajectory object to convert.
            velocity (float): Velocity to assign to each trajectory point.
        Returns:
            Path2D: ROS Trajectory2D message.
        """
        path_msg = Path2D()
        path_msg.points = []
        path_msg.header.stamp = rospy.Time.now()
        for x, y, theta in trajectory:
            point_msg = PathPoint()
            point_msg.x = x
            point_msg.y = y
            point_msg.theta = np.arctan2(theta[1], theta[0])
            point_msg.velocity = velocity
            path_msg.points.append(point_msg)
        return path_msg

    def odom_callback(self, msg):
        # Convert odometry to "localization"
        self.odom = msg
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        velocity = msg.twist.twist.linear.x
        yaw_rate = msg.twist.twist.angular.z

        self.state = [position.x, position.y, yaw, velocity, yaw_rate]
        self.viz.visualize_robot_location(position.x, position.y)

    def run(self):
        while self.state is None:
            rospy.loginfo("Waiting for initial odometry...")
            self.rate.sleep()

        while not rospy.is_shutdown():
            local_trajectory = self.path_handler.get_local_trajectory(
                np.array(self.state[:2]), horizon=5.0, ds=0.25
            )
            local_trajectory.x = local_trajectory.x[1:]
            local_trajectory.y = local_trajectory.y[1:]
            local_trajectory.theta = local_trajectory.theta[1:]
            path = self.trajectory_to_ros_msg(local_trajectory, velocity=0.2)

            self.viz.visualize_local_path(local_trajectory.x, local_trajectory.y)

            # Call MPC service
            request = MPCServiceRequest()
            request.path = path
            request.odom = self.odom
            response = self.client(request)

            cmd = Twist()
            # Example car-like forward command
            cmd.linear.x = response.command.linear_velocity
            cmd.angular.z = response.command.angular_velocity

            error = (
                local_trajectory.x[0] - self.state[0],
                local_trajectory.y[0] - self.state[1],
            )
            error = np.linalg.norm(error)
            print("Error: ", error)

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    node = CarControlNode()
    node.run()
