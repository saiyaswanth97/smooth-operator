#!/usr/bin/env python3
# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import numpy as np
from simulator.msg import TrajectoryPoint2D, Trajectory2D

import os
import sys

# TODO remove this hack
sys.path.append(os.path.join(os.path.dirname(__file__), "../../../scripts/trajectory"))
from trajectory.trajectory_handler import TrajectoryHandler, Trajectory
from gazebo_utils import gazebo_reset_robot, RvizPathVisualizer


class CarControlNode:
    def __init__(self):
        rospy.init_node("vehicle_node")
        self.freq = rospy.get_param("~control_frequency", 5)
        self.rate = rospy.Rate(self.freq)

        self.velocity = 0.05
        self.path_handler = TrajectoryHandler(
            "/home/smummaneni/hobby/smooth-operator/trajectory_data/waypoints/b.csv"
        )

        # Restart Gazebo and place robot at start of trajectory
        start = self.path_handler.get_start_point()
        gazebo_reset_robot(start)

        # Visualize full trajectory in RViz
        self.viz = RvizPathVisualizer()
        full_path = self.path_handler.get_full_trajectory(ds=0.5)
        x = full_path.x
        y = full_path.y
        print("Full trajectory points: ", len(x), type(x), x.shape)
        self.viz.visualize_global_path(x, y)

        # Subscribers and Publishers
        self.state = None
        self.odom = None
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    @staticmethod
    def trajectory_to_ros_msg(trajectory: Trajectory, velocity: float) -> Trajectory2D:
        """
        Converts a Trajectory object to a ROS Trajectory2D message.
        Args:
            trajectory (Trajectory): Trajectory object to convert.
            velocity (float): Velocity to assign to each trajectory point.
        Returns:
            Trajectory2D: ROS Trajectory2D message.
        """
        traj_msg = Trajectory2D()
        traj_msg.points = []
        traj_msg.header.stamp = rospy.Time.now()
        for x, y, theta in trajectory:
            point_msg = TrajectoryPoint2D()
            point_msg.x = x
            point_msg.y = y
            point_msg.theta = np.arctan2(theta[1], theta[0])
            point_msg.velocity = velocity
            traj_msg.points.append(point_msg)
        return traj_msg

    def get_local_trajectory_ros(
        self, current_position: Odometry, horizon: float = 5.0, ds: float = 0.1
    ) -> Trajectory2D:
        """
        Returns a local trajectory segment as a ROS Trajectory2D message based on the current position.
        Args:
            current_position (Odometry): Current position as a ROS Odometry message.
            horizon (float): Length of the local trajectory segment.
            ds (float): Distance between consecutive points in the trajectory.
        Returns:
            Trajectory2D: Local trajectory segment as a ROS Trajectory2D message.
        """
        position = current_position.pose.pose.position
        orientation_q = current_position.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        current_pos_np = np.array([position.x, position.y])
        local_trajectory = self.path_handler.get_local_trajectory(
            current_pos_np, horizon=horizon, ds=ds
        )
        traj_msg = self.trajectory_to_ros_msg(local_trajectory, velocity=0.5)
        return traj_msg

    def odom_callback(self, msg):
        # Convert odometry to "localization"
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.state = [position.x, position.y, yaw]
        self.odom = msg
        self.viz.visualize_robot_location(position.x, position.y)

    def run(self):
        while self.state is None:
            rospy.loginfo("Waiting for initial odometry...")
            self.rate.sleep()

        while not rospy.is_shutdown():
            path = self.get_local_trajectory_ros(self.odom, horizon=5.0, ds=0.1)
            x_viz = [point.x for point in path.points]
            y_viz = [point.y for point in path.points]
            self.viz.visualize_local_path(x_viz, y_viz)

            cmd = Twist()
            # Example car-like forward command
            cmd.linear.x = path.points[0].velocity  # constant speed
            cmd.angular.z = (path.points[0].theta - self.state[2]) * 2.5

            error = np.linalg.norm(
                np.array([path.points[0].x, path.points[0].y])
                - np.array(self.state[:2])
            )
            print("Error: ", error)

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    node = CarControlNode()
    node.run()
