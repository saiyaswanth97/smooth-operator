# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import rospy
import tf
import numpy as np
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


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


def gazebo_visualize_path(
    x: np.array, y: np.array, frame_id: str = "map", color: tuple = (0.0, 1.0, 0.0)
) -> None:
    """
    Visualizes a path in Gazebo using markers.
    Args:
        x (np.array): Array of x coordinates of the path.
        y (np.array): Array of y coordinates of the path.
        frame_id (str): The frame in which to visualize the path.
        color (tuple): RGB color tuple for the path.
    """
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.05  # Line width
    marker.color.a = 1.0  # Alpha
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    for xi, yi in zip(x, y):
        point = Point()
        point.x = xi
        point.y = yi
        point.z = 0.0
        marker.points.append(point)

    marker_pub.publish(marker)
