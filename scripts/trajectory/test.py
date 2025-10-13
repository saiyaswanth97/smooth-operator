# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import numpy as np
import matplotlib.pyplot as plt
from utils import Spline

# Example 1: Simple curve (your original example)
points1 = np.array([[0, 0], [1, 2], [2, 0], [3, 3]])

# Example 2: S-curve
points2 = np.array([[0, 0], [1, 1], [2, 1], [3, 0]])

# Example 3: Zigzag path
points3 = np.array([[0, 0], [1, 2], [2, -1], [3, 2], [4, 0]])

# Example 4: Circular-like path
points4 = np.array(
    [[0, 0], [1, 1], [2, 1.5], [3, 1], [4, 0], [3, -1], [2, -1.5], [1, -1], [0, 0]]
)

# Example 5: Sharp turn
points5 = np.array([[0, 0], [2, 0], [2, 2], [4, 2]])

# Example 6: Gentle wave
points6 = np.array([[0, 0], [1, 0.5], [2, 0], [3, -0.5], [4, 0], [5, 0.5]])

# Example 7: Complex path with multiple turns
points7 = np.array([[0, 0], [1, 3], [3, 4], [5, 2], [6, 5], [8, 3], [10, 4]])

example_points = [points1, points2, points3, points4, points5, points6, points7]


def save_spline_plot(spline: np.ndarray, waypoints: np.ndarray, file_name: str) -> None:
    """
    Plots the spline and waypoints.
    Args:
        spline (np.ndarray): The spline object.
        waypoints (np.ndarray): The original waypoints.
        file_name (str): The name of the file to save the plot.
    """
    plt.figure(figsize=(8, 6))
    plt.plot(waypoints[:, 0], waypoints[:, 1], "ro--", label="Waypoints")
    plt.plot(spline[:, 0], spline[:, 1], "b-", label="Spline")
    plt.plot(spline[::4, 0], spline[::4, 1], "b*", label="Spline")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.savefig(f"{file_name}")


if __name__ == "__main__":
    for i, points in enumerate(example_points, start=1):
        spline = Spline(points)
        s_sample = np.linspace(0, spline.n, 100)
        spline_points = spline(s_sample)
        file_name = f"plots/spline_beizer/spline_example_{i}.png"
        save_spline_plot(spline_points, points, file_name)
