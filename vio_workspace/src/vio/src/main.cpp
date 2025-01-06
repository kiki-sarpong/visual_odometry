#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "OdometryPublisher.h"
#include "VisualOdometry.h"


int main(int argc, char** argv) {
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    // // Keep node active and create publisher
    // rclcpp::spin(std::make_shared<VisualOdometryPublisher>());

    // Create seperate nodes
    auto odometry_publisher = std::make_shared<OdometryPublisher>();
    /*
    I pass the odometry publisher node as an argument in constructing the visual odom node.
    I wanted to have two seperate notes, and also be able to call the publisher method.
    */
    auto visual_odom = std::make_shared<VisualOdometry>(odometry_publisher);
    
    // Create multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add nodes to the executor
    executor.add_node(odometry_publisher);
    executor.add_node(visual_odom);

    // Spin the executor
    executor.spin();
    // Shutdown ROS2 node
    rclcpp::shutdown();

    return 0;
}
