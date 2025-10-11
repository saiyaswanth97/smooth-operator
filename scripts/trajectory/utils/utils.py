# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import os
import csv
from typing import Tuple
import numpy
import matplotlib.pyplot as plt


class WayPoints:
    """
    Class to represent waypoints with x and y coordinates.
    Attributes:
        x (numpy.ndarray): Array of x coordinates.
        y (numpy.ndarray): Array of y coordinates.
    """

    def __init__(self, x: numpy.ndarray = None, y: numpy.ndarray = None) -> None:
        """
        Initializes the WayPoints object with x and y coordinates.
        Args:
            x (numpy.ndarray): Array of x coordinates.
            y (numpy.ndarray): Array of y coordinates.
        """
        self.x = x if x is not None else numpy.array([])
        self.y = y if y is not None else numpy.array([])

    def __str__(self) -> str:
        """
        Returns a string representation of the WayPoints object.
        Returns:
            str: String representation of the WayPoints object.
        """
        max_len = 10
        x_display = self.x[:max_len].tolist() + (
            ["..."] if len(self.x) > max_len else []
        )
        y_display = self.y[:max_len].tolist() + (
            ["..."] if len(self.y) > max_len else []
        )
        return f"WayPoints(\n  x={x_display},\n  y={y_display}\n)"


class Trajectory:
    """
    Class to represent a trajectory with x and y coordinates.
    Attributes:
        x (numpy.ndarray): Array of x coordinates.
        y (numpy.ndarray): Array of y coordinates.
        theta (numpy.ndarray): Array of orientation angles (optional).
    """

    def __init__(
        self,
        x: numpy.ndarray = None,
        y: numpy.ndarray = None,
        theta: numpy.ndarray = None,
    ) -> None:
        """
        Initializes the Trajectory object with x, y coordinates and optional theta.
        Args:
            x (numpy.ndarray): Array of x coordinates.
            y (numpy.ndarray): Array of y coordinates.
            theta (numpy.ndarray): Array of orientation angles (optional).
        """
        self.x = x if x is not None else numpy.array([])
        self.y = y if y is not None else numpy.array([])
        self.theta = theta if theta is not None else numpy.array([])

    def __str__(self) -> str:
        """
        Returns a string representation of the Trajectory object.
        Returns:
            str: String representation of the Trajectory object.
        """
        max_len = 10
        x_display = self.x[:max_len].tolist() + (
            ["..."] if len(self.x) > max_len else []
        )
        y_display = self.y[:max_len].tolist() + (
            ["..."] if len(self.y) > max_len else []
        )
        theta_display = self.theta[:max_len].tolist() + (
            ["..."] if len(self.theta) > max_len else []
        )
        return (
            f"Trajectory(\n"
            f"  x={x_display},\n"
            f"  y={y_display},\n"
            f"  theta={theta_display}\n"
            f")"
        )


def read_waypoint_file(file_name: str) -> Tuple[numpy.ndarray, bool]:
    """
    Plots the trajectory data from a CSV file.
    Args:
        file_name (str): Path to the CSV file containing trajectory data.
    Returns:
        Tuple[numpy.ndarray, bool]:
            - WayPoints: object containing x and y arrays
            - Bool: True if successful, False otherwise
    """
    if not os.path.exists(file_name):
        print(f"File {file_name} does not exist.")
        return WayPoints(), False

    x = []
    y = []

    try:
        with open(file_name, "r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                x.append(float(row["x"]))
                y.append(float(row["y"]))
    except Exception as e:
        print(f"Error reading file {file_name}: {e}")
        return WayPoints(), False

    x = numpy.array(x)
    y = numpy.array(y)
    return WayPoints(x, y), True


def plot_waypoints(waypoints: WayPoints, plot: bool = True) -> numpy.ndarray:
    """
    Plots the given waypoints.
    Args:
        waypoints (WayPoints): Object containing x and y arrays.
        plot (bool): If True, displays the plot.
    Returns:
        numpy.ndarray: Image array of the plotted waypoints.
    """

    plt.figure(figsize=(10, 6))
    plt.plot(waypoints.x, waypoints.y, marker="*", linestyle="-", color="r")
    plt.title("Waypoints")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.axis("equal")

    img = plt.gcf()
    img.canvas.draw()
    img_array = numpy.frombuffer(img.canvas.tostring_rgb(), dtype=numpy.uint8)
    img_array = img_array.reshape(img.canvas.get_width_height()[::-1] + (3,))

    if plot:
        plt.show()
    plt.close()

    return img_array


if __name__ == "__main__":
    file_name = "../../../trajectory_data/waypoints/a.csv"
    file_path = os.path.join(os.path.dirname(__file__), file_name)
    waypoints, success = read_waypoint_file(file_path)
    if success:
        print(waypoints)
        plot_waypoints(waypoints, plot=True)
    else:
        print("Failed to read waypoints.")
