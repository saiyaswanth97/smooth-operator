# Copyright (c) 2025 Sai Yaswanth. All rights reserved.

import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt


class CubicSpline:
    """
    This class computes the coefficients for a cubic spline interpolation.
    """

    def __init__(self, points: np.ndarray):
        """
        Initialize the CubicSpline with given points.
        Args:
            points (np.ndarray): Array of shape (n+1, 2) representing the points.
        # Raises:
        #     AssertionError: If there are fewer than 4 points or if points are not in 2D.
        """
        assert points.shape[0] >= 4, (
            "At least two points are required to create a spline."
        )
        assert points.shape[1] == 2, "Points should be in 2D space (x, y)."

        [self.a, self.b, self.c, self.d] = self.compute_coefficients(
            points.shape[0] - 1, points
        )

    @staticmethod
    def solve_tridiagonal(B: np.ndarray) -> np.ndarray:
        """
        Solve the tridiagonal system Ax = B where A is a tridiagonal matrix
        with 4s on the diagonal and 1s on the off-diagonals.
        Args:
            B (np.ndarray): 1D array of size n.
        Returns:
            np.ndarray: Solution array x of size n.
        """
        n = B.size

        a = np.ones(n)
        b = 4 * np.ones(n)
        c = np.ones(n)
        d = B.copy()
        a[0] = 0
        b[0] = 2
        b[-1] = 2
        c[-1] = 0

        w = a[1:] / b[:-1]
        b[1:] -= w * c[:-1]
        d[1:] -= w * d[:-1]

        x = np.zeros(n)  # This is in reversed order
        x[-1] = d[-1] / b[-1]
        for i in reversed(range(n - 1)):
            x[i] = (d[i] - c[i] * x[i + 1]) / b[i]

        return x

    @staticmethod
    def compute_coefficients(
        n_seg: int, points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute cubic spline coefficients for x and y.
        Args:
            n_seg (int): Number of segments (n).
            points (np.ndarray): Array of shape (n+1, 2) containing the points.
        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: Coefficients a, b, c, d each of shape (n, 2).
        """
        coeff_size = n_seg
        tri_size = coeff_size + 1

        b_mat = np.zeros((tri_size, 2))
        b_mat[0, :] = points[1, :] - points[0, :]
        b_mat[-1, :] = points[-1, :] - points[-2, :]
        b_mat[1:-1, :] = points[2:, :] - points[:-2, :]

        d_points = points[1:, :] - points[:-1, :]

        a = points[:-1, :].copy()
        b = np.zeros((coeff_size, 2))
        c = np.zeros((coeff_size, 2))
        d = np.zeros((coeff_size, 2))

        derivative = np.zeros((tri_size, 2))
        for axis in range(2):
            derivative[:, axis] = __class__.solve_tridiagonal(3 * b_mat[:, axis])

        b[:, :] = derivative[:-1]
        c[:, :] = 3 * d_points[:, :] - 2 * derivative[:-1, :] - derivative[1:, :]
        d[:, :] = -2 * d_points[:, :] + derivative[:-1, :] + derivative[1:, :]
        return a, b, c, d


class BezierSpline:
    """
    This class computes the coefficients for a beizer spline interpolation.
    """

    def __init__(self, points: np.ndarray):
        """
        Initialize the BezierSpline with given control points.
        Args:
            points (np.ndarray): Array of shape (n+1, 2) representing control points.
        """
        self.points = points
        self.n = points.shape[0] - 1
        self.a = None
        self.b = None
        self.c = None
        self.d = None

    def build_b_matrix(self, coordinates: np.ndarray) -> np.ndarray:
        """
        Build the B-matrix used to compute Bezier spline coefficients.

        Args:
            coordinates (np.ndarray): 1D array of x or y coordinates.
        Returns:
            np.ndarray: (n, 4) matrix.
        """
        b_mat = np.zeros((self.n, 4))
        for i in range(self.n):
            if i == 0:
                b_mat[i, 0] = coordinates[i]
                b_mat[i, 1] = coordinates[i]
                b_mat[i, 2] = coordinates[i + 1]
                b_mat[i, 3] = coordinates[i + 2]
            elif i == self.n - 1:
                b_mat[i, 0] = coordinates[i - 1]
                b_mat[i, 1] = coordinates[i]
                b_mat[i, 2] = coordinates[i + 1]
                b_mat[i, 3] = coordinates[i + 1]
            else:
                b_mat[i, 0] = coordinates[i - 1]
                b_mat[i, 1] = coordinates[i]
                b_mat[i, 2] = coordinates[i + 1]
                b_mat[i, 3] = coordinates[i + 2]
        return b_mat

    def compute_coefficients(self):
        """
        Compute cubic Bezier spline coefficients for x and y.
        """
        # Bezier basis matrix
        basis_fn = (
            np.array(
                [[-1, 3, -3, 1], [3, -6, 0, 4], [-3, 3, 3, 1], [1, 0, 0, 0]],
                dtype=float,
            )
            / 6.0
        )

        self.a = np.zeros((self.n, 2))
        self.b = np.zeros((self.n, 2))
        self.c = np.zeros((self.n, 2))
        self.d = np.zeros((self.n, 2))

        for axis in range(2):
            d_points = self.points[:, axis]
            coefficients = self.build_b_matrix(d_points) @ basis_fn
            coefficients = coefficients[:, ::-1]  # reverse columns

            self.a[:, axis] = coefficients[:, 0]
            self.b[:, axis] = coefficients[:, 1]
            self.c[:, axis] = coefficients[:, 2]
            self.d[:, axis] = coefficients[:, 3]


class Spline:
    """
    Base class for spline interpolation.
    This class provides a framework for creating and querying splines.
    """

    def __init__(self, points: np.ndarray):
        """
        Initialize the Spline with given points.
        Args:
            points (np.ndarray): Array of shape (n+1, 2) representing control points.
        """
        assert points.shape[0] >= 4, (
            "At least two points are required to create a spline."
        )
        assert points.shape[1] == 2, "Points should be in 2D space (x, y)."

        self.n = points.shape[0] - 1
        self.spline_coefficients = CubicSpline(points)

    def t_query_point(self, t: float) -> np.ndarray:
        """
        Get a point on the spline at parameter t.
        TODO: Deprecate this method in favor of __call__.
        Args:
            t (float): Parameter value, where 0 <= t <= n (number of segments).
        Returns:
            np.ndarray: Point on the spline at parameter t.
        """
        if t < 0 or t > self.n:
            print(
                f"Warning: t={t} is out of bounds [0, {self.n}]. Clamping to valid range."
            )
            t = max(0, min(t, self.n))

        segment = min(int(t), self.n - 1)
        dt = t - segment

        point = (
            self.spline_coefficients.a[segment]
            + self.spline_coefficients.b[segment] * dt
            + self.spline_coefficients.c[segment] * dt**2
            + self.spline_coefficients.d[segment] * dt**3
        )

        return point

    def __call__(self, t: np.ndarray) -> np.ndarray:
        """
        Evaluate the spline at multiple parameter values.
        Args:
            t (np.ndarray): Array of parameter values.
        Returns:
            np.ndarray: Array of points on the spline corresponding to the parameter values.
        """
        t = np.clip(t, 0, self.n)

        seg_idx = np.floor(t).astype(int)
        seg_idx = np.clip(seg_idx, 0, self.n - 1)
        dt = t - seg_idx

        points = (
            self.spline_coefficients.a[seg_idx]
            + self.spline_coefficients.b[seg_idx] * dt[:, None]
            + self.spline_coefficients.c[seg_idx] * dt[:, None] ** 2
            + self.spline_coefficients.d[seg_idx] * dt[:, None] ** 3
        )
        return points

    def get_jacobian(self, t: np.ndarray) -> np.ndarray:
        """
        Compute the Jacobian (derivative) of the spline at multiple parameter values.
        Args:
            t (np.ndarray): Array of parameter values.
        Returns:
            np.ndarray: Array of derivatives (dx/dt, dy/dt) at the parameter values.
        """
        # TODO: remove hot fix
        t = np.array(np.clip(t, 0, self.n))

        seg_idx = np.floor(t).astype(int)
        seg_idx = np.array(np.clip(seg_idx, 0, self.n - 1))
        dt = t - seg_idx

        derivatives = (
            self.spline_coefficients.b[seg_idx]
            + 2 * self.spline_coefficients.c[seg_idx] * dt[:, None]
            + 3 * self.spline_coefficients.d[seg_idx] * dt[:, None] ** 2
        )
        return derivatives

    def get_jacobian_norm(self, t: float) -> float:
        """
        Compute the norm of the Jacobian (magnitude of the derivative) at multiple parameter values.
        Args:
            t (float): Parameter values.
        Returns:
            float: Norm of the derivative at the parameter values.
        """
        t = np.array([t])
        jacobian = self.get_jacobian(np.array([t]))[0]
        norms = np.linalg.norm(jacobian, axis=1)[0]
        return norms

    def get_curvature(self, t: np.ndarray) -> np.ndarray:
        """
        Compute the radius of curvature at multiple parameter values.
        Args:
            t (np.ndarray): Array of parameter values.
        Returns:
            np.ndarray: Array of radii of curvature at the parameter values.
        """
        t = np.array(np.clip(t, 0, self.n))

        seg_idx = np.floor(t).astype(int)
        seg_idx = np.array(np.clip(seg_idx, 0, self.n - 1))
        dt = t - seg_idx

        first_derivative = (
            self.spline_coefficients.b[seg_idx]
            + 2 * self.spline_coefficients.c[seg_idx] * dt[:, None]
            + 3 * self.spline_coefficients.d[seg_idx] * dt[:, None] ** 2
        )

        second_derivative = (
            2 * self.spline_coefficients.c[seg_idx]
            + 6 * self.spline_coefficients.d[seg_idx] * dt[:, None]
        )

        dx, dy = first_derivative[:, 0], first_derivative[:, 1]
        ddx, ddy = second_derivative[:, 0], second_derivative[:, 1]

        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
        radius_of_curvature = np.where(curvature != 0, 1 / curvature, np.inf)
        return radius_of_curvature

    def animate_spline(self, num_points: int = 100) -> None:
        """
        Generate points along the spline for animation or plotting.
        Args:
            num_points (int): Number of points to generate along the spline.=
        """
        t_values = np.linspace(0, self.n, num_points)
        # points = np.array([self.t_query_point(t) for t in t_values])
        points = self(t_values)

        _ = plt.figure()
        plt.plot(points[:, 0], points[:, 1], "r-", label="Cubic Spline")
        plt.title("Cubic Spline Animation")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis("equal")
        plt.plot(points[:, 0], points[:, 1], "r*", label="Waypoints")

        # for i, t in enumerate(t_values):
        #     plt.plot(points[i, 0], points[i, 1], 'g*')
        #     plt.pause(0.025)
        plt.show()


class SplineLenght:
    """
    Class to compute the length of a spline using numerical integration.
    """

    def __init__(self, spline: Spline, minimum_dt: int = 1):
        """
        Initialize the SplineLength with a given spline.
        Args:
            spline (Spline): The cubic spline object.
            minimum_dt (int): Minimum number of discrete points per segment for length computation.
        """
        self.spline = spline
        self.minimum_dt = minimum_dt
        self.cumulative_lengths = self.compute_cumulative_curve_length()

    @staticmethod
    def gauss_legendre_parameters(n: int) -> Tuple[np.ndarray, np.ndarray]:
        """Compute Gauss-Legendre weights and points.
        Args:
            n (int): Number of points.
        Returns:
            Tuple[np.ndarray, np.ndarray]: Points and weights for Gauss-Legendre quadrature.
        """
        points, weights = np.polynomial.legendre.leggauss(n)
        return points, weights

    def section_length(
        self, t_start: float, t_end: float, quantization_steps: int = 6
    ) -> float:
        """
        Compute arc length between t_start and t_end.
        Args:
            t_start (float): Start parameter value.
            t_end (float): End parameter value.
            quantization_steps (int): Number of steps for numerical integration.
        Returns:
            float: Length of the spline between t_start and t_end.
        """
        if t_start >= t_end:
            print("Warning: t_start should be less than t_end.")
            return 0.0

        points, weights = self.gauss_legendre_parameters(quantization_steps)
        mean = (t_start + t_end) / 2
        # TODO: Optimize this by directly evaluating the spline at the points
        delta = (t_end - t_start) / 2
        points_section = points * delta + mean
        dxdy = self.spline.get_jacobian(points_section)
        length = np.linalg.norm(dxdy, axis=1).dot(weights) * delta
        return length

    def compute_cumulative_curve_length(self) -> np.ndarray:
        """
        Compute cumulative curve length at discrete points along the spline.
        Returns:
            np.ndarray: Cumulative lengths at each discrete point.
        """
        t_values = np.linspace(0, self.spline.n, self.spline.n * self.minimum_dt + 1)
        cumulative_lengths = np.zeros(self.spline.n * self.minimum_dt + 1)
        for i in range(1, len(t_values)):
            cumulative_lengths[i] = cumulative_lengths[i - 1] + self.section_length(
                t_values[i - 1], t_values[i]
            )
        return cumulative_lengths

    def get_curve_length(self, t_start: float, t_end: float) -> float:
        """
        Get the length of the spline between t_start and t_end.
        Args:
            t_start (float): Start parameter value.
            t_end (float): End parameter value.
        Returns:
            float: Length of the spline between t_start and t_end.
        """
        if t_start >= t_end:
            print("Warning: t_start should be less than t_end.")
            return 0.0

        if t_start < 0 or t_end > self.spline.n:
            print(
                f"Warning: t_start={t_start} or t_end={t_end} is out of bounds [0, {self.spline.n}]. Clamping to valid range."
            )
            t_start = max(0, min(t_start, self.spline.n))
            t_end = max(0, min(t_end, self.spline.n))

        curve_length = (
            self.compute_section_length(np.floor(t_start), t_start)
            + self.compute_section_length(np.floor(t_end), t_end)
            + self.cumulative_lengths[int(np.floor(t_end))]
            - self.cumulative_lengths[int(np.floor(t_start))]
        )

        return curve_length

    def get_total_length(self) -> float:
        """
        Get the total length of the spline.
        Returns:
            float: Total length of the spline.
        """
        return self.cumulative_lengths[-1]

    def get_t_from_s(self, s: float) -> float:
        """
        Get the parameter t corresponding to a given arc length s.
        TODO: Make this faster, use vectorization.
        Args:
            s (float): Arc length along the spline.
        Returns:
            float: Parameter t corresponding to the arc length s.
        """
        if s < 0 or s > self.cumulative_lengths[-1]:
            print(
                f"Warning: s={s} is out of bounds [0, {self.cumulative_lengths[-1]}]. Clamping to valid range."
            )
            s = max(0, min(s, self.cumulative_lengths[-1]))

        t = 0.0
        for i in range(1, len(self.cumulative_lengths)):
            if self.cumulative_lengths[i] > s:
                s -= self.cumulative_lengths[i - 1]
                t = i - 1
                break

        num_steps = 10
        step_size = s / num_steps

        for _ in range(num_steps):
            k1 = step_size / self.spline.get_jacobian_norm(t)
            k2 = step_size / self.spline.get_jacobian_norm(t + k1 / 2)
            k3 = step_size / self.spline.get_jacobian_norm(t + k2 / 2)
            k4 = step_size / self.spline.get_jacobian_norm(t + k3)
            t += (k1 + 2 * k2 + 2 * k3 + k4) / 6

        return t

    def plot(self, num_points: int = 100) -> None:
        """
        Plot the cumulative curve length.
        Args:
            num_points (int): Number of points to plot.
        """
        s_values = np.linspace(0, self.cumulative_lengths[-1], num_points)
        t_values = [self.get_t_from_s(s) for s in s_values]

        # plt.figure(figsize=(10, 6))
        # plt.plot(t_values, s_values, 'b-', label='Cumulative Length')
        # plt.title('Cumulative Curve Length')
        # plt.xlabel('Parameter t')
        # plt.ylabel('Arc Length s')
        # plt.legend()
        # plt.show()

        points = self.spline(t_values)
        plt.figure(figsize=(10, 6))
        plt.plot(points[:, 0], points[:, 1], "r-", label="Cubic Spline")
        plt.title("Cubic Spline with Cumulative Length")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis("equal")
        plt.plot(points[:, 0], points[:, 1], "g*", label="Waypoints")
        plt.legend()
        plt.show()

    def demonstrate_length_mapping(self, horizon: float) -> None:
        """
        Demonstrate the mapping between parameter t and arc length s.
        Args:
            horizon (float): Maximum arc length to demonstrate.
        """
        t = np.linspace(0, self.spline.n, 100)
        points_full = self.spline(t)

        _ = plt.figure(figsize=(10, 6))

        for s in np.linspace(0, self.cumulative_lengths[-1], 10):
            s_sample = np.linspace(s, s + horizon, 15)
            t = [self.get_t_from_s(s) for s in s_sample]

            points = self.spline(t)
            plt.plot(points_full[:, 0], points_full[:, 1], "r-", label="Cubic Spline")
            plt.plot(points[0, 0], points[0, 1], "r*")
            plt.plot(points[1:, 0], points[1:, 1], "g*")
            plt.title("Constant lenght sampling")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.axis("equal")
            plt.pause(0.5)
            plt.clf()
        plt.show()


def interpolate_sample(points: np.ndarray, max_dist: int = 2) -> np.ndarray:
    """
    Interpolate points to ensure that the distance between consecutive points
    does not exceed max_dist.
    Args:
        points (np.ndarray): Array of shape (n, 2) representing the points.
        max_dist (int): Maximum allowed distance between consecutive points.
    Returns:
        np.ndarray: Interpolated points.
    """
    interpolated_points = [points[0]]
    for i in range(1, points.shape[0]):
        dist = np.linalg.norm(points[i] - points[i - 1])
        if dist > max_dist:
            num_new_points = int(np.ceil(dist / max_dist))
            new_points = np.linspace(points[i - 1], points[i], num_new_points + 1)[1:]
            interpolated_points.extend(new_points)
        else:
            interpolated_points.append(points[i])
    return np.array(interpolated_points)


if __name__ == "__main__":
    points = np.array([[0, 0], [1, 2], [2, 0], [3, 3]])
    # Animation for the constant parameter
    spline = Spline(points)
    spline.animate_spline(num_points=50)
    # Demo for Spline Length
    spline_length = SplineLenght(spline)
    spline_length.plot(num_points=50)
    # Animation for the constant lenght
    spline_length.demonstrate_length_mapping(horizon=1.0)

    # Non uniform way points
    points = np.array([[0, 0], [0, 20], [2, 22], [4, 20], [4, 0]])
    points = interpolate_sample(points, max_dist=3)
    # Animation for the constant parameter
    spline = Spline(points)
    s_sample = np.linspace(0, spline.n, 100)
    spline_points = spline(s_sample)
    fig = plt.figure()
    plt.plot(spline_points[:, 0], spline_points[:, 1], "b-", label="Spline Points")
    plt.plot(spline_points[:, 0], spline_points[:, 1], "b*")
    plt.plot(points[:, 0], points[:, 1], "r*", label="Waypoints")
    plt.title("Waypoints")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.show()
    # Demo for Spline Length
    spline_length = SplineLenght(spline)
    spline_length.plot(num_points=50)
