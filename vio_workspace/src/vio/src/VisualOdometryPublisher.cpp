#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
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


/*
This method calls the visual odometry publisher.
It publishes a transform stamped msg, and gives every new message a unique
frame id based on a counter.
*/
 void VisualOdometryPublisher::call_publisher(cv::Mat& Rotation, cv::Mat& Translation){
    auto msg = geometry_msgs::msg::TransformStamped(); // Create transform stamped msg
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = map;
    // Make every child frame id unique 
    msg.child_frame_id = id_name + std::to_string(count);
    msg.transform.translation.x = Translation.at<double>(0);
    msg.transform.translation.y = Translation.at<double>(1);
    msg.transform.translation.z = Translation.at<double>(2);

    tf2::Quaternion quaternion = VisualOdometryPublisher::rot_to_quaternion(Rotation);
    msg.transform.rotation.x = quaternion.x();
    msg.transform.rotation.y = quaternion.y();
    msg.transform.rotation.z = quaternion.z();
    msg.transform.rotation.w = quaternion.w();

    // Publish message
    vo_publisher->publish(msg);

    count += 0.01;  // Increment count
   
}


/*
This method converts a cv::Mat to a quaternion
input: rotation matrix
returns: a quaternion
*/
tf2::Quaternion VisualOdometryPublisher::rot_to_quaternion(const cv::Mat& rotation_matrix) {
    tf2::Matrix3x3 tf2_rot(rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

    tf2::Quaternion quaternion;
    tf2_rot.getRotation(quaternion); // Extract quartenion from rotation

    return quaternion;
}