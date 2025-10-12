# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import os
from utils import read_waypoint_file, WayPoints, Trajectory, Spline, SplineLenght
import numpy as np
import matplotlib.pyplot as plt


class TrajectoryHandler:
    """
    Base class that handles trajectory
    """

    def __init__(self, trajectory_file: str):
        """
        Initializes the TrajectoryHandler with a trajectory file.
        Args:
            trajectory_file (str): Path to the trajectory file.
        Raises:
            ValueError: If the trajectory file cannot be loaded or if the spline cannot be created.
        """
        waypoints, success = read_waypoint_file(file_path)
        if not success:
            raise ValueError(f"Failed to load trajectory from {trajectory_file}")
        try:
            self.spline = Spline(self.waypoints_to_numpy(waypoints))
        except Exception as e:
            raise ValueError(f"Failed to create spline from waypoints: {e}")
        self.spline_s = SplineLenght(self.spline)
        self.t = None
        self.waypoints = waypoints
        print(f"Loaded trajectory with {len(waypoints.x)} waypoints")

    @staticmethod
    def waypoints_to_numpy(waypoints: WayPoints) -> np.ndarray:
        """
        Converts WayPoints object to a numpy array.
        Args:
            waypoints (WayPoints): WayPoints object containing x and y arrays.
        Returns:
            np.ndarray: Numpy array of shape (N, 2) where N is the number of waypoints.
        """
        return np.vstack((waypoints.x, waypoints.y)).T

    def reset_trajectory(self):
        """
        Resets the trajectory to the initial state.
        """
        pass

    def get_full_trajectory(self, ds: float = 0.1) -> Trajectory:
        """
        Returns the full trajectory.
        Args:
            ds (float): Distance between consecutive points in the trajectory.
        Returns:
            Trajectory: Full trajectory object.
        """
        path_lenght = self.spline_s.get_total_length()
        number_of_points = int(path_lenght / 0.1) + 1
        s_values = np.linspace(0, path_lenght, number_of_points)
        t_values = [self.spline_s.get_t_from_s(s) for s in s_values]
        points = self.spline(t_values)
        theta = self.spline.get_heading(t_values)
        return Trajectory(points[:, 0], points[:, 1], theta)

    def get_closest_point(
        self, current_position: np.ndarray, max_distance: float
    ) -> float:
        """
        Returns the closest point on the trajectory to the current position.
        Retuns None if the closest point is further away than max_distance.
        Args:
            current_position (np.ndarray): Current position as a numpy array [x, y].
            max_distance (float): Maximum allowable distance to consider a point as "closest".
        Returns:
            float: Closest point on the trajectory as a parameter t.
        """
        if self.t is None:
            t_max = self.spline.n
            t, euclidian_dist, normal_dist = self.spline.get_closest_point(
                current_position, 0.0, t_max
            )
            if euclidian_dist > max_distance:
                print(
                    f"Warning: Closest point is {euclidian_dist:.2f}m away, which is greater than max_distance {max_distance}m"
                )
                self.t = None
            else:
                self.t = t
        else:
            # TODO: make this config
            # TODO remove this
            t, euclidian_dist, normal_dist = self.spline.get_closest_point(
                current_position, self.t - 1.0, self.t + 1.0
            )
            if euclidian_dist > max_distance:
                print(
                    f"Warning: Closest point is {euclidian_dist:.2f}m away, which is greater than max_distance {max_distance}m"
                )
                self.t = None
            else:
                self.t = t
        return self.t

    def get_local_trajectory(
        self, current_position: np.ndarray, horizon: float = 5.0, ds: float = 0.1
    ) -> Trajectory:
        """
        Returns a local trajectory segment based on the current position.
        Args:
            current_position (np.ndarray): Current position as a numpy array [x, y].
            horizon (float): Length of the local trajectory segment.
            ds (float): Distance between consecutive points in the trajectory.
        Returns:
            Trajectory: Local trajectory segment object.
        """
        closest_t = self.get_closest_point(current_position, max_distance=1.0)
        if closest_t is None:
            print("Warning: No closest point found, returning empty trajectory")
            return Trajectory()

        closest_s = self.spline_s.get_curve_length(0.0, closest_t)
        number_of_points = int(horizon / ds) + 1
        s_sample = np.linspace(closest_s, closest_s + horizon, number_of_points)
        t_values = [self.spline_s.get_t_from_s(s) for s in s_sample]

        points = self.spline(t_values)
        theta = self.spline.get_heading(t_values)
        return Trajectory(points[:, 0], points[:, 1], theta)

    def demo_plot(self):
        """
        Demo plot of the trajectory.
        """
        trajectory = self.get_full_trajectory()
        _ = plt.figure(figsize=(10, 6))
        for i in range(20):
            for wx, wy in self.waypoints:
                plt.plot(trajectory.x, trajectory.y, "b-")

                x = wx + np.random.uniform(-0.5, 0.5)
                y = wy + np.random.uniform(-0.5, 0.5)
                plt.plot(x, y, "r*")

                closest_trajectory = self.get_local_trajectory(
                    np.array([x, y]), horizon=2.5
                )
                for tx, ty, theta in closest_trajectory:
                    plt.plot(tx, ty, "go")

                plt.axis("equal")
                plt.title("Full Trajectory with Headings")
                plt.xlabel("X Position")
                plt.ylabel("Y Position")
                plt.pause(1)
                plt.clf()
        plt.show()


if __name__ == "__main__":
    file_name = "../../trajectory_data/waypoints/b.csv"
    file_path = os.path.join(os.path.dirname(__file__), file_name)

    trajectory_handler = TrajectoryHandler(file_path)

    trajectory = trajectory_handler.get_full_trajectory()
    _ = plt.figure(figsize=(10, 6))
    for x, y, theta in trajectory:
        plt.plot(x, y, "b.")
        # plt.quiver(x, y, np.cos(theta), np.sin(theta), angles='xy', scale_units='xy', scale=5, color='r')
    plt.axis("equal")
    plt.title("Full Trajectory")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.show()

    trajectory_handler.demo_plot()
