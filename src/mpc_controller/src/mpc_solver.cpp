// Copyright (c) 2025 Sai Yaswanth. All rights reserved.

#include "mpc_solver.h"

#include <cmath>
#include <iostream>

MPCSolver::MPCSolver(const Config &config)
    : config_(config), solver_initialized_(false), prev_v_(0.0), prev_omega_(0.0) {
    solver_.settings()->setVerbosity(true);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setAbsoluteTolerance(1e-3);
    solver_.settings()->setRelativeTolerance(1e-3);

    // solver_.settings()->setAbsoluteTolerance(1e-5);
    // solver_.settings()->setRelativeTolerance(1e-5);
    // solver_.settings()->setRho(1.0);
    // solver_.settings()->setAdaptiveRho(true);
    // solver_.settings()->setAdaptiveRhoInterval(10);
    // solver_.settings()->setAdaptiveRhoTolerance(2.0);
    // solver_.settings()->setMaxIteraction(1000);
}

void MPCSolver::reset() {
    prev_v_ = 0.0;
    prev_omega_ = 0.0;
    solver_initialized_ = false;
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    solver_.clearSolver();
}

Eigen::Vector3d MPCSolver::getStateFromOdom(const nav_msgs::Odometry &odom) {
    Eigen::Vector3d state;
    state(0) = odom.pose.pose.position.x;
    state(1) = odom.pose.pose.position.y;

    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;
    state(2) = atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));

    return state;
}

std::vector<Eigen::Vector3d> MPCSolver::extractReferencePath(const simulator::Path2D &path) {
    std::vector<Eigen::Vector3d> ref_path;

    for (size_t i = 0;
         i < std::min(path.points.size(), static_cast<size_t>(config_.prediction_horizon)); ++i) {
        Eigen::Vector3d point;
        point(0) = path.points[i + 1].x;
        point(1) = path.points[i + 1].y;
        point(2) = path.points[i + 1].theta;
        ref_path.push_back(point);
    }

    while (ref_path.size() < static_cast<size_t>(config_.prediction_horizon)) {
        ref_path.push_back(ref_path.back());
    }

    return ref_path;
}

simulator::Command MPCSolver::solve(const nav_msgs::Odometry &odom, const simulator::Path2D &path) {
    Eigen::Vector3d current_state = getStateFromOdom(odom);
    std::vector<Eigen::Vector3d> reference_path = extractReferencePath(path);

    // double vehcile_vx = odom.twist.twist.linear.x;
    // prev_v_ = vehcile_vx;

    const int N = config_.prediction_horizon;

    // Decision variables: [v0, omega0, v1, omega1, ..., vN-1, omegaN-1]
    // We directly optimize controls, predict states on the fly
    const int n_vars = N * 2;

    // Build cost function: min 0.5 * x^T * H * x + g^T * x
    Eigen::SparseMatrix<double> hessian(n_vars, n_vars);
    Eigen::VectorXd gradient(n_vars);
    gradient.setZero();

    std::vector<Eigen::Triplet<double>> hessian_triplets;

    // Predict states for the reference trajectory to compute cost
    Eigen::Vector3d state = current_state;

    for (int k = 0; k < N; ++k) {
        int v_idx = k * 2;
        int omega_idx = k * 2 + 1;

        // Control effort cost
        hessian_triplets.push_back(Eigen::Triplet<double>(v_idx, v_idx, 2.0 * config_.weight_v));
        hessian_triplets.push_back(
          Eigen::Triplet<double>(omega_idx, omega_idx, 2.0 * config_.weight_omega));

        // Control rate cost (smoothness)
        if (k > 0) {
            int v_prev = (k - 1) * 2;
            int omega_prev = (k - 1) * 2 + 1;

            // (v_k - v_{k-1})^2 = v_k^2 - 2*v_k*v_{k-1} + v_{k-1}^2
            hessian_triplets.push_back(
              Eigen::Triplet<double>(v_idx, v_idx, 2.0 * config_.weight_delta_v));
            hessian_triplets.push_back(
              Eigen::Triplet<double>(v_idx, v_prev, -2.0 * config_.weight_delta_v));
            hessian_triplets.push_back(
              Eigen::Triplet<double>(v_prev, v_prev, 2.0 * config_.weight_delta_v));

            hessian_triplets.push_back(
              Eigen::Triplet<double>(omega_idx, omega_idx, 2.0 * config_.weight_delta_omega));
            hessian_triplets.push_back(
              Eigen::Triplet<double>(omega_idx, omega_prev, -2.0 * config_.weight_delta_omega));
            hessian_triplets.push_back(
              Eigen::Triplet<double>(omega_prev, omega_prev, 2.0 * config_.weight_delta_omega));
        } else {
            // First control: penalize change from previous solve
            gradient(v_idx) += -2.0 * config_.weight_delta_v * prev_v_ * prev_v_;
            // gradient(v_idx) += -2.0 * config_.weight_delta_v * prev_v_;
            hessian_triplets.push_back(
              Eigen::Triplet<double>(v_idx, v_idx, 2.0 * config_.weight_delta_v));

            gradient(omega_idx) += -2.0 * config_.weight_delta_omega * prev_omega_ * prev_omega_;
            // gradient(omega_idx) += -2.0 * config_.weight_delta_omega * prev_omega_;
            hessian_triplets.push_back(
              Eigen::Triplet<double>(omega_idx, omega_idx, 2.0 * config_.weight_delta_omega));
        }

        // Tracking cost: we want the predicted state to reach the reference
        double dx = reference_path[k](0) - state(0);
        double dy = reference_path[k](1) - state(1);
        double dtheta = reference_path[k](2) - state(2);

        if (k == 0) {
            double diff = std::sqrt(dx * dx + dy * dy);
            std::cout << "diff: " << diff << std::endl;
        }

        // Normalize angle error
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

        // The key insight: v and omega directly affect how we move
        // Distance to reference in robot frame
        double cos_theta = cos(state(2));
        double sin_theta = sin(state(2));

        // Position error in robot frame
        double dx_body = dx * cos_theta + dy * sin_theta;   // forward error
        double dy_body = -dx * sin_theta + dy * cos_theta;  // lateral error

        // Cost gradient: moving forward (v) reduces forward error
        // We want: state + v*[cos,sin]*dt ≈ reference
        // So: v should be proportional to forward distance
        double forward_gain = config_.weight_x + config_.weight_y;
        gradient(v_idx) += -forward_gain * dx_body;

        // Angular velocity should reduce heading error AND help with lateral tracking
        gradient(omega_idx) += -config_.weight_theta * dtheta * 10.0;  // Strong heading correction
        if (std::abs(dx_body) > 0.01) {                                // If we have forward motion
            gradient(omega_idx) += -forward_gain * dy_body / std::max(0.05, std::abs(dx_body));
        }

        // Hessian terms for tracking (make it a proper quadratic cost)
        double tracking_hessian = (config_.weight_x + config_.weight_y) * config_.dt * config_.dt;
        hessian_triplets.push_back(Eigen::Triplet<double>(v_idx, v_idx, tracking_hessian));
        hessian_triplets.push_back(Eigen::Triplet<double>(
          omega_idx, omega_idx, config_.weight_theta * config_.dt * config_.dt));

        // Simulate forward with current control estimate (use small nominal values)
        double v_nom = (dx_body > 0) ? 1.0 : -0.05;  // Nominal velocity in right direction
        double omega_nom = dtheta * 2.0;             // Proportional heading correction
        omega_nom = std::max(-1.0, std::min(1.0, omega_nom));  // Clip

        state(0) += v_nom * cos(state(2)) * config_.dt;
        state(1) += v_nom * sin(state(2)) * config_.dt;
        state(2) += omega_nom * config_.dt;
    }

    hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

    // Simple box constraints on controls
    Eigen::SparseMatrix<double> constraint_matrix(n_vars, n_vars);
    constraint_matrix.setIdentity();

    Eigen::VectorXd lower_bound(n_vars);
    Eigen::VectorXd upper_bound(n_vars);

    for (int k = 0; k < N; ++k) {
        lower_bound(k * 2) = config_.min_velocity;
        upper_bound(k * 2) = config_.max_velocity;
        lower_bound(k * 2 + 1) = config_.min_angular_velocity;
        upper_bound(k * 2 + 1) = config_.max_angular_velocity;
    }
    lower_bound(0) = prev_v_;
    upper_bound(0) = prev_v_;

    // Initialize or update solver
    if (!solver_initialized_) {
        solver_.data()->setNumberOfVariables(n_vars);
        solver_.data()->setNumberOfConstraints(n_vars);

        if (!solver_.data()->setHessianMatrix(hessian)) {
            std::cerr << "Failed to set Hessian matrix" << std::endl;
        }
        if (!solver_.data()->setGradient(gradient)) {
            std::cerr << "Failed to set gradient" << std::endl;
        }
        if (!solver_.data()->setLinearConstraintsMatrix(constraint_matrix)) {
            std::cerr << "Failed to set constraint matrix" << std::endl;
        }
        if (!solver_.data()->setLowerBound(lower_bound)) {
            std::cerr << "Failed to set lower bound" << std::endl;
        }
        if (!solver_.data()->setUpperBound(upper_bound)) {
            std::cerr << "Failed to set upper bound" << std::endl;
        }

        if (!solver_.initSolver()) {
            std::cerr << "Failed to initialize solver" << std::endl;
        }

        solver_initialized_ = true;
    } else {
        solver_.updateHessianMatrix(hessian);
        solver_.updateGradient(gradient);
        solver_.updateBounds(lower_bound, upper_bound);
    }

    simulator::Command command;

    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "MPC solve failed!" << std::endl;
        command.linear_velocity = 0.0;
        command.angular_velocity = 0.0;
        return command;
    }

    Eigen::VectorXd solution = solver_.getSolution();

    command.linear_velocity = solution(2);
    command.angular_velocity = solution(3);

    // Debug output
    std::cout << "=== MPC Debug ===" << std::endl;
    std::cout << "Linear vel: " << command.linear_velocity
              << " Angular vel: " << command.angular_velocity << std::endl;
    std::cout << "Error to target: dx=" << (reference_path[0](0) - current_state(0))
              << " dy=" << (reference_path[0](1) - current_state(1)) << std::endl;

    prev_v_ = command.linear_velocity;
    prev_omega_ = command.angular_velocity;

    return command;
}