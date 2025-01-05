// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "GroundTruthPublisher.h"

std::string gt_node_name = "ground_truth_publisher";
std::string& gt_topic_name = gt_node_name;

// Constructor
GroundTruthPublisher::GroundTruthPublisher() : Node(gt_node_name){

    // Set QOS profile for services and topics
    rclcpp::QoS qos_profile(qos_depth);
    qos_profile.reliable();  // Reliable communication
    qos_profile.durability_volatile();  // Retain last message

    gt_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>(gt_topic_name, qos_profile);
    RCLCPP_INFO(this->get_logger(), (gt_node_name + " has started.").c_str());
}


void GroundTruthPublisher::call_publisher(cv::Mat& rotation_n_translation){

}
