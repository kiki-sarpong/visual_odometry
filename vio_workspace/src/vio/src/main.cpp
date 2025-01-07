#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "OdometryPublisher.h"
#include "VisualOdometry.h"


int main(int argc, char** argv) {
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    // // Keep node active and create publisher
    // rclcpp::spin(std::make_shared<VisualOdometry>());

    // Create seperate node
    auto visual_odom = std::make_shared<VisualOdometry>();
    
    // Create multithreaded executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Add nodes to the executor
    executor.add_node(visual_odom);
    // Spin the executor
    executor.spin();
    // Shutdown ROS2 node
    rclcpp::shutdown();    

    return 0;
}
