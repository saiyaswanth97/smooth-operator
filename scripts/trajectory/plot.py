# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import os
import csv
import numpy
import matplotlib.pyplot as plt


FILE_NAME = "waypoints/a.csv"

"""
Plots the trajectory data from a CSV file.
"""


def plot_trajectory(file_name):
    if not os.path.exists(file_name):
        raise FileNotFoundError(f"File {file_name} does not exist.")

    x = []
    y = []

    with open(file_name, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x.append(float(row["x"]))
            y.append(float(row["y"]))

    x = numpy.array(x)
    y = numpy.array(y)

    plt.figure(figsize=(10, 6))
    plt.plot(x, y, marker="*", linestyle="-", color="r")
    plt.title("Trajectory Data")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    # plt.grid(True)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    file_name = FILE_NAME
    file_path = os.path.join(os.path.dirname(__file__), file_name)
    plot_trajectory(file_path)
