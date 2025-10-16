// Copyright (c) 2025 Sai Yaswanth. All rights reserved.

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <simulator/Command.h>
#include <simulator/MPCService.h>
#include <simulator/Path2D.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>

#include "OsqpEigen/OsqpEigen.h"
#include "mpc_solver.h"

// Constants for quaternion to yaw conversion
constexpr double QUAT_TO_YAW_FACTOR = 2.0;

// Default control parameters
constexpr double DEFAULT_K_P_YAW_DUMB = 2.5;
constexpr double DEFAULT_VELOCITY_DUMB = 0.5;
constexpr double DEFAULT_K_P_YAW_STANLEY = 1.0;
constexpr double DEFAULT_VELOCITY_STANLEY = 1.0;

// Stanley controller lookahead index
constexpr int STANLEY_LOOKAHEAD_INDEX = 10;

std::tuple<double, double> dumb_control(simulator::Path2D path, const nav_msgs::Odometry &odom,
                                        double k_p_yaw = DEFAULT_K_P_YAW_DUMB,
                                        double velocity = DEFAULT_VELOCITY_DUMB) {
    // Extract current yaw from odometry
    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;
    double yaw = atan2(QUAT_TO_YAW_FACTOR * (qw * qz), 1.0 - QUAT_TO_YAW_FACTOR * (qz * qz));

    // Target yaw from the first point in the path
    if (path.points.empty()) {
        return std::make_tuple(0.0, 0.0);
    }
    double target_yaw = path.points[0].theta;

    // Compute yaw error
    double yaw_error = target_yaw - yaw;
    if (yaw_error > M_PI) {
        yaw_error -= 2 * M_PI;
    } else if (yaw_error < -M_PI) {
        yaw_error += 2 * M_PI;
    }

    // Proportional control for angular velocity
    double angular_velocity = k_p_yaw * yaw_error;

    // Constant linear velocity
    double linear_velocity = velocity;

    return std::make_tuple(linear_velocity, angular_velocity);
}

std::tuple<double, double> stanley_control(simulator::Path2D path, const nav_msgs::Odometry &odom,
                                           double k_p_yaw = DEFAULT_K_P_YAW_STANLEY,
                                           double velocity = DEFAULT_VELOCITY_STANLEY) {
    // Extract current yaw from odometry
    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;
    double yaw = atan2(QUAT_TO_YAW_FACTOR * (qw * qz), 1.0 - QUAT_TO_YAW_FACTOR * (qz * qz));

    // Target yaw from the first point in the path
    if (path.points.empty()) {
        return std::make_tuple(0.0, 0.0);
    }

    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double target_x = path.points[STANLEY_LOOKAHEAD_INDEX].x;
    double target_y = path.points[STANLEY_LOOKAHEAD_INDEX].y;
    double target_yaw = std::atan2(target_y - y, target_x - x);

    // Compute yaw error
    double yaw_error = target_yaw - yaw;
    if (yaw_error > M_PI) {
        yaw_error -= 2 * M_PI;
    } else if (yaw_error < -M_PI) {
        yaw_error += 2 * M_PI;
    }

    // Proportional control for angular velocity
    double angular_velocity = k_p_yaw * yaw_error;

    // Constant linear velocity
    double linear_velocity = velocity;

    return std::make_tuple(linear_velocity, angular_velocity);
}

class MPCController {
public:
    MPCController(ros::NodeHandle &nh, std::string controller_type, double k_p_yaw, double velocity)
        : k_p_yaw_(k_p_yaw),
          velocity_(velocity),
          controller_type_(std::move(controller_type)),
          mpc_solver_(MPCSolver::Config()) {
        service_ = nh.advertiseService("mpc_control", &MPCController::serviceCallback, this);
    }

    // NOLINTNEXTLINE(google-runtime-references, readability-make-member-function-const)
    bool serviceCallback(simulator::MPCService::Request &req,
                         simulator::MPCService::Response &res) {
        if (controller_type_ == "mpc") {
            // Use MPC controller
            mpc_solver_.reset();
            simulator::Command command = mpc_solver_.solve(req.odom, req.path);
            res.command = command;
            std::cout << "Linear Velocity: " << command.linear_velocity
                      << ", Angular Velocity: " << command.angular_velocity << std::endl;
            return true;
        } else if (controller_type_ == "stanley") {
            // Use stanley controller
            simulator::Command command;
            std::tie(command.linear_velocity, command.angular_velocity) =
              stanley_control(req.path, req.odom, k_p_yaw_, velocity_);
            res.command = command;
            std::cout << "Linear Velocity: " << command.linear_velocity
                      << ", Angular Velocity: " << command.angular_velocity << std::endl;
            return true;
        } else if (controller_type_ == "dumb") {
            // Use dumb controller
            simulator::Command command;
            std::tie(command.linear_velocity, command.angular_velocity) =
              dumb_control(req.path, req.odom, k_p_yaw_, velocity_);
            res.command = command;
            std::cout << "Linear Velocity: " << command.linear_velocity
                      << ", Angular Velocity: " << command.angular_velocity << std::endl;
            return true;
        } else {
            std::cerr << "Unknown controller type: " << controller_type_ << std::endl;
            return false;
        }
    }

private:
    double k_p_yaw_;
    double velocity_;
    std::string controller_type_;
    MPCSolver mpc_solver_;
    ros::ServiceServer service_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string controller_type = pnh.param<std::string>("controller_type", "mpc");

    // Initialize controller with gains
    // NOLINTNEXTLINE(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
    MPCController controller(nh, controller_type, 2.5, 0.5);

    // ROS_INFO("MPC Controller Service Ready.");
    ros::spin();
    return 0;
}
