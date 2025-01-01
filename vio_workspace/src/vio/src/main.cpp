#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "GroundTruthPublisher.h"
#include "VisualOdometryPublisher.h"
#include "VisualOdometry.h"


int main(int argc, char** argv) {
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    // // Keep node active and create publisher
    // rclcpp::spin(std::make_shared<VisualOdometryPublisher>());

    // Create seperate nodes for publishing visual odometry and ground truth
    auto visual_odom_publisher = std::make_shared<VisualOdometryPublisher>();
    auto ground_truth_publisher = std::make_shared<GroundTruthPublisher>();
    auto visual_odom = std::make_shared<VisualOdometry>();

    // Create multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add nodes to the executor
    executor.add_node(visual_odom_publisher);
    executor.add_node(ground_truth_publisher);
    executor.add_node(visual_odom);

    // Spin the executor
    executor.spin();

    // Shutdown ROS2 nodde
    rclcpp::shutdown();
    return 0;
}
