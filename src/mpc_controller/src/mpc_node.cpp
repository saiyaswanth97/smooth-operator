// Copyright (c) 2025 Sai Yaswanth. All rights reserved.

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <simulator/Command.h>
#include <simulator/MPCService.h>
#include <simulator/Path2D.h>
#include <std_msgs/Float64.h>

const struct DummyControllerConfig {
    const double k_p_yaw;
    const double velocity;
} config = {
  2.5,  // k_p_yaw
  0.5   // velocity
};

// NOLINTNEXTLINE(google-runtime-references)
bool mpc_service_callback(simulator::MPCService::Request &req,
                          // NOLINTNEXTLINE(google-runtime-references)
                          simulator::MPCService::Response &res) {
    // double qx = req.odom.pose.pose.orientation.x;
    // double qy = req.odom.pose.pose.orientation.y;
    double qz = req.odom.pose.pose.orientation.z;
    double qw = req.odom.pose.pose.orientation.w;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    double yaw = atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));
    double target_yaw = req.path.points[0].theta;
    // double velocity = req.odom.twist.twist.linear.x;
    double yaw_error = target_yaw - yaw;
    // if (yaw_error > M_PI) {
    //     yaw_error -= 2 * M_PI;
    // } else if (yaw_error < -M_PI) {
    //     yaw_error += 2 * M_PI;
    // }

    res.command.angular_velocity = yaw_error * config.k_p_yaw;
    res.command.linear_velocity = config.velocity;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("mpc_control", mpc_service_callback);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
    ROS_INFO("MPC Controller Service Ready.");

    ros::spin();
    return 0;
}
