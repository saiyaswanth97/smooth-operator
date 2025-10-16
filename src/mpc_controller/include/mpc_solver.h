// Copyright (c) 2025 Sai Yaswanth. All rights reserved.

#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include <nav_msgs/Odometry.h>
#include <simulator/Command.h>
#include <simulator/Path2D.h>

#include <Eigen/Dense>

#include "OsqpEigen/OsqpEigen.h"

class MPCSolver {
public:
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
    struct Config {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        int prediction_horizon = 15;  // N - number of prediction steps
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double dt = 0.1;  // time step (seconds)

        // State weights [x, y, theta]
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_x = 10.0;  // Increase these significantly
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_y = 10.0;
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_theta = 4.0;

        // Control weights [v, omega]
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_v = 1.0;  // Keep these small
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_omega = 2.0;

        // Control rate weights (acceleration penalty in cost)
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_delta_v = 5.0;  // Reduce for more aggressive
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double weight_delta_omega = 10.0;

        // Simple velocity constraints only
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double max_velocity = 2.0;  // m/s
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double min_velocity = -0.1;  // m/s
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double max_angular_velocity = 1.4;  // rad/s
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
        double min_angular_velocity = -1.4;  // rad/s
    };
    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

    explicit MPCSolver(const Config &config);

    simulator::Command solve(const nav_msgs::Odometry &odom, const simulator::Path2D &path);

    void reset();

private:
    Config config_;
    OsqpEigen::Solver solver_;
    bool solver_initialized_;

    // Previous control for smoothness
    double prev_v_;
    double prev_omega_;

    Eigen::Vector3d getStateFromOdom(const nav_msgs::Odometry &odom);
    std::vector<Eigen::Vector3d> extractReferencePath(const simulator::Path2D &path);
};

#endif  // MPC_SOLVER_H