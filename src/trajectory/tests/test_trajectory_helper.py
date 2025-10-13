# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import numpy as np
import pytest
import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../scripts")))
from utils.spline import Spline


@pytest.fixture
def traj_helper() -> Spline:
    """Return a trajectory helper built on the simple spline.
    Returns:
        Spline: A Spline object representing a straight line.
    """
    points = np.array(
        [
            [0.0, 0.0],
            [1.0, 1.0],
            [2.0, 2.0],
            [3.0, 3.0],
        ]
    )
    spline = Spline(points)
    return spline
    # return simple_spline
    # return TrajectoryHelper(simple_spline)


def test_distance_on_line(traj_helper: Spline):
    """
    For a line y=x, a point on the line should have zero distances.
    Args:
        traj_helper (Spline): The trajectory helper object.
    """
    t_query = np.linspace(0, traj_helper.n, 5)
    point_on_line = np.array([1.5, 1.5])

    point_dist, normal_dist = traj_helper.get_distance(point_on_line, t_query)
    print("Point Distances:", point_dist, "Normal Distances:", normal_dist)

    points = traj_helper(t_query)
    point_dist_gt = np.linalg.norm(points - point_on_line, axis=1)
    assert np.allclose(point_dist, point_dist_gt, atol=1e-6)
    # assert np.allclose(point_dist, 0.0, atol=1e-6)
    assert np.allclose(normal_dist, 0.0, atol=1e-6)


def test_distance_off_line(traj_helper: Spline):
    """
    For a point off the line y=x, normal distance should match |y-x|/√2.
    Args:
        traj_helper (Spline): The trajectory helper object.
    """
    t_query = np.linspace(0, traj_helper.n, 5)
    point_off_line = np.array([1.0, 2.0])

    point_dist, normal_dist = traj_helper.get_distance(point_off_line, t_query)

    # For y=x line, perpendicular distance = |y-x| / √2
    expected = abs(point_off_line[1] - point_off_line[0]) / np.sqrt(2)
    assert np.allclose(normal_dist, expected, atol=1e-3)


def test_closest_point_center(traj_helper: Spline):
    """
    Closest point to (1.5, 1.5) should be near t=1.5.
    Args:
        traj_helper (Spline): The trajectory helper object.
    """
    query_point = np.array([1.5, 1.5])
    t_closest, dist, normal = traj_helper.get_closest_point(
        query_point, 0, traj_helper.n
    )
    print(f"t_closest: {t_closest}, dist: {dist}, normal: {normal}")

    assert np.isclose(t_closest, 1.5, atol=1e-2)
    assert np.isclose(dist, 0.0, atol=1e-4)
    assert np.isclose(normal, 0.0, atol=1e-6)


def test_closest_point_off_line(traj_helper: Spline):
    """
    For a point (1, 2), closest spline point should be (1.5, 1.5).
    Args:
        traj_helper (Spline): The trajectory helper object.
    """
    query_point = np.array([1.0, 2.0])
    t_closest, dist, normal = traj_helper.get_closest_point(
        query_point, 0, traj_helper.n
    )

    # Expect a midpoint near (1.5, 1.5)
    assert 1.0 < t_closest < 2.0
    assert dist > 0
    assert np.isclose(dist, normal, atol=1e-3)


# def test_invalid_t_range_raises(traj_helper):
#     """Should raise ValueError for too small t range."""
#     query_point = np.array([0.0, 0.0])
#     with pytest.raises(ValueError):
#         traj_helper.get_closest_point(query_point, 1.0, 1.001)


def test_vectorized_output_shapes(traj_helper: Spline):
    """
    Ensure distance computation preserves array shape.
    Args:
        traj_helper (Spline): The trajectory helper object.
    """
    t_query = np.linspace(0, traj_helper.n, 50)
    point = np.array([0.0, 0.0])

    point_dist, normal_dist = traj_helper.get_distance(point, t_query)
    assert point_dist.shape == (t_query.size,)
    assert normal_dist.shape == (t_query.size,)


# def test_curve_lenghth(traj_helper: Spline):
#     """
#     Ensure the computed curve length is as expected.
#     Args:
#         traj_helper (Spline): The trajectory helper object.
#     """
#     length = traj_helper.get_curve_length(0.0, traj_helper.n)
#     # For the line from (0,0) to (3,3), length should be 3*sqrt(2)
#     expected_length = 3 * np.sqrt(2)
#     assert np.isclose(length, expected_length, atol=1e-6)
