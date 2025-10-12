# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import rospy
import tf
import numpy as np
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def gazebo_reset_robot(start: tuple = (0.0, 0.0, 0.0)) -> None:
    """
    Resets the Gazebo world and places the robot at the start of the trajectory.
    Args:
        start (tuple): A tuple containing the (x, y, theta) coordinates to place the robot at.
    """
    # Worls reset
    rospy.wait_for_service("/gazebo/reset_world")
    try:
        reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        reset_world()
        rospy.loginfo("Gazebo world reset.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset Gazebo world: {e}")

    # Move robot to start of trajectory
    state_msg = ModelState()
    # TODO: remove hard coded model name
    state_msg.model_name = "turtlebot3_burger"  # Change as per your robot model name
    state_msg.pose.position.x = start[0]
    state_msg.pose.position.y = start[1]

    quat = tf.transformations.quaternion_from_euler(0, 0, start[2])
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]

    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        set_state(state_msg)
        rospy.loginfo("Robot reset to trajectory start.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to set robot state: {e}")


class RvizPathVisualizer:
    """
    Class to visualize a path in RViz using markers.
    """

    def __init__(self, frame_id: str = "odom") -> None:
        """
        Initializes the RVizPathVisualizer.
        Args:
            frame_id (str): The frame in which to visualize the path.
        """
        self.global_path_pub = rospy.Publisher(
            "/viz/path/global_path", Marker, latch=True
        )
        self.local_path_pub = rospy.Publisher(
            "/viz/path/local_path", Marker, queue_size=10
        )
        self.robot_location_pub = rospy.Publisher(
            "/viz/path/robot_location", Marker, queue_size=10
        )
        self.frame_id = frame_id

    def visualize_local_path(self, x: np.array, y: np.array) -> None:
        """
        Visualizes a path in RViz using markers.
        Args:
            x (np.array): Array of x coordinates of the path.
            y (np.array): Array of y coordinates of the path.
        """

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.MODIFY
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.points = []
        for xi, yi in zip(x, y):
            point = Point()
            point.x = xi
            point.y = yi
            point.z = 0.0
            marker.points.append(point)
        self.local_path_pub.publish(marker)

    def visualize_global_path(self, x: np.array, y: np.array) -> None:
        """
        Visualizes a path in RViz using markers.
        Args:
            x (np.array): Array of x coordinates of the path.
            y (np.array): Array of y coordinates of the path.
        """

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.points = []
        for xi, yi in zip(x, y):
            point = Point()
            point.x = xi
            point.y = yi
            point.z = 0.0
            marker.points.append(point)
        self.global_path_pub.publish(marker)

    def visualize_robot_location(self, x: float, y: float) -> None:
        """
        Visualizes the robot location in RViz using a Point marker.
        Args:
            x (float): x coordinate of the robot.
            y (float): y coordinate of the robot.
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        self.robot_location_pub.publish(marker)
