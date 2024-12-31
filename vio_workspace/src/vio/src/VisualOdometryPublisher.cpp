#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "VisualOdometryPublisher.h"

std::string vo_node_name = "visual_odometry_publisher";
std::string& vo_topic_name = vo_node_name;

// Constructor
VisualOdometryPublisher::VisualOdometryPublisher() : Node(vo_node_name){

    // Set QOS profile for services and topics
    rclcpp::QoS qos_profile(qos_depth);
    qos_profile.reliable();  // Reliable communication
    qos_profile.durability_volatile();  // Retain last message

    vo_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>(vo_topic_name, qos_profile);
    RCLCPP_INFO(this->get_logger(), (vo_node_name + " has started.").c_str());
}


void VisualOdometryPublisher::call_publisher(){
    std::cout << "Publisher has been called!\n";
}